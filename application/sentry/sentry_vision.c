#include "sentry_vision.h"
#include <math.h>
#include <string.h>
_Static_assert(sizeof(float) == 4, "SP requires IEEE754 binary32");
static uint16_t ReadU16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
void SentryVisionWriteU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
void SentryVisionWriteFloat(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(u >> (8 * i));
}
float SentryVisionReadFloat(const uint8_t *p)
{
    uint32_t u = 0;
    float f;
    for (unsigned i = 0; i < 4; i++)
        u |= (uint32_t)p[i] << (8 * i);
    memcpy(&f, &u, 4);
    return f;
}
uint16_t SentryVisionCrc(const uint8_t *p, size_t n)
{
    uint16_t c = 0xffff;
    while (n--)
    {
        c ^= *p++;
        for (unsigned i = 0; i < 8; i++)
            c = (c & 1) ? (uint16_t)((c >> 1) ^ 0x8408) : (uint16_t)(c >> 1);
    }
    return c;
}
uint16_t SentryNavCrc(const uint8_t *p, size_t n)
{
    uint16_t c = 0xffff;
    while (n--)
    {
        c ^= (uint16_t)*p++ << 8;
        for (unsigned i = 0; i < 8; i++)
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
    }
    return c;
}
void SentryVisionStreamPush(SentryVisionStream_s *s, const uint8_t *data, size_t length, SentryVisionFrameFn cb,
                            void *ctx)
{
    while (length--)
    {
        if (s->used == sizeof(s->data))
        {
            memmove(s->data, s->data + 1, --s->used);
        }
        s->data[s->used++] = *data++;
        while (s->used >= 2)
        {
            size_t n = 0;
            uint8_t sp = s->data[0] == 'S' && s->data[1] == 'P';
            if (sp)
                n = 29;
            else if (s->data[0] == 0xa5 && s->data[1] == 0x5a)
                n = 15;
            if (!n)
            {
                memmove(s->data, s->data + 1, --s->used);
                continue;
            }
            if (s->used < n)
                break;
            uint16_t crc = sp ? SentryVisionCrc(s->data, n - 2) : SentryNavCrc(s->data, n - 2);
            if (crc == ReadU16(s->data + n - 2))
            {
                cb(ctx, s->data, n);
                s->used -= n;
                memmove(s->data, s->data + n, s->used);
            }
            else
            {
                if (sp)
                    ++s->invalid_sp_count;
                memmove(s->data, s->data + 1, --s->used);
            }
        }
    }
}
static uint8_t Bounded(float v, float max)
{
    return isfinite(v) && fabsf(v) <= max;
}
static void Quaternion(const SentryVisionFeedback_s *f, float q[4])
{
    /* Legacy Euler helper: Z(yaw), Y(roll), X(pitch), standard ZYX. */
    float cz = cosf(f->yaw * .5f), sz = sinf(f->yaw * .5f), cy = cosf(f->roll * .5f), sy = sinf(f->roll * .5f),
          cx = cosf(f->pitch * .5f), sx = sinf(f->pitch * .5f);
    q[0] = cz * cy * cx + sz * sy * sx;
    q[1] = cz * cy * sx - sz * sy * cx;
    q[2] = cz * sy * cx + sz * cy * sx;
    q[3] = sz * cy * cx - cz * sy * sx;
}
uint8_t SentryVisionFeedbackValid(const SentryVisionFeedback_s *f, uint32_t now)
{
    if (!f || f->work_mode > 3 || (uint32_t)(now - f->timestamp_ms) > SENTRY_VISION_FEEDBACK_TIMEOUT_MS ||
        !(f->capabilities & SENTRY_VISION_ATTITUDE))
        return 0;
    if (!Bounded(f->yaw, 3.15f) || !Bounded(f->pitch, 3.15f) || !Bounded(f->roll, 3.15f) ||
        !Bounded(f->yaw_rate, 100) || !Bounded(f->pitch_rate, 100))
        return 0;
    float q[4], dot = 0, norm = 0;
    Quaternion(f, q);
    for (unsigned i = 0; i < 4; i++)
    {
        if (!isfinite(f->quaternion[i]))
            return 0;
        dot += q[i] * f->quaternion[i];
        norm += f->quaternion[i] * f->quaternion[i];
    }
    if (norm < .95f || norm > 1.05f || fabsf(dot) / sqrtf(norm) < .999687516f)
        return 0;
    if ((f->capabilities & SENTRY_VISION_BULLET_SPEED) &&
        (!isfinite(f->bullet_speed) || f->bullet_speed < 0 || f->bullet_speed > 100))
        return 0;
    return 1;
}
static uint8_t Supported(const SentryVisionCommand_s *c, const SentryVisionFeedback_s *f)
{
    if (!f || f->work_mode == 0 || !(f->capabilities & SENTRY_VISION_ATTITUDE) ||
        !(f->capabilities & (SENTRY_VISION_YAW | SENTRY_VISION_PITCH)))
        return 0;
    if (!(f->capabilities & SENTRY_VISION_YAW) && (c->yaw != 0 || c->yaw_rate != 0 || c->yaw_accel != 0))
        return 0;
    if (!(f->capabilities & SENTRY_VISION_PITCH) && (c->pitch != 0 || c->pitch_rate != 0 || c->pitch_accel != 0))
        return 0;
    return c->mode != 2 || (f->capabilities & SENTRY_VISION_FIRE);
}
uint8_t SentryVisionAccept(SentryVisionState_s *s, const uint8_t *p, size_t n, uint32_t now,
                           const SentryVisionFeedback_s *f)
{
    SentryVisionCommand_s c = {0};
    if (!p || n != 29 || p[0] != 'S' || p[1] != 'P' || SentryVisionCrc(p, 27) != ReadU16(p + 27))
        goto reject;
    c.mode = p[2];
    c.yaw = SentryVisionReadFloat(p + 3);
    c.yaw_rate = SentryVisionReadFloat(p + 7);
    c.yaw_accel = SentryVisionReadFloat(p + 11);
    c.pitch = SentryVisionReadFloat(p + 15);
    c.pitch_rate = SentryVisionReadFloat(p + 19);
    c.pitch_accel = SentryVisionReadFloat(p + 23);
    if (c.mode > 2 || !Bounded(c.yaw, 3.15f) || !Bounded(c.pitch, 3.15f) || !Bounded(c.yaw_rate, 100) ||
        !Bounded(c.pitch_rate, 100) || !Bounded(c.yaw_accel, 1000) || !Bounded(c.pitch_accel, 1000))
        goto reject;
    if (c.mode && (!SentryVisionFeedbackValid(f, now) || !f->armed || !Supported(&c, f)))
        goto reject;
    s->command = c;
    s->command_ms = now;
    s->seen = 1;
    s->active = 0;
    s->flags &= (uint8_t) ~(SENTRY_VISION_REJECTED | SENTRY_VISION_TIMEOUT);
    s->accepted_count++;
    return 1;
reject:
    s->active = 0;
    s->seen = 0;
    s->flags |= SENTRY_VISION_REJECTED;
    s->rejected_count++;
    return 0;
}
void SentryVisionUpdate(SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint32_t now, uint8_t manual)
{
    s->flags &= (SENTRY_VISION_REJECTED | SENTRY_VISION_TIMEOUT);
    s->active = 0;
    uint8_t fresh = SentryVisionFeedbackValid(f, now);
    if (fresh)
        s->flags |= SENTRY_VISION_FEEDBACK_FRESH;
    if (fresh && f->armed)
        s->flags |= SENTRY_VISION_ARMED;
    if (manual)
        s->flags |= SENTRY_VISION_MANUAL_OVERRIDE;
    if (s->seen && s->command.mode && (uint32_t)(now - s->command_ms) > SENTRY_VISION_TIMEOUT_MS)
        s->flags |= SENTRY_VISION_TIMEOUT;
    if (s->seen && s->command.mode && fresh && f->armed && !manual && !(s->flags & SENTRY_VISION_TIMEOUT) &&
        Supported(&s->command, f))
        s->active = 1;
    /* Lost feedback/authority revokes the snapshot, requiring a new command. */
    if (s->seen && s->command.mode && (!fresh || !f->armed || !Supported(&s->command, f)))
        s->seen = 0;
    if (s->active)
        s->flags |= SENTRY_VISION_ACTIVE;
}
void SentryVisionEncodeStatus(const SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint8_t out[12])
{
    memset(out, 0, 12);
    out[0] = 'S';
    out[1] = 'C';
    out[2] = 1;
    out[3] = f ? f->capabilities : 0;
    out[4] = s->flags;
    out[5] = s->command.mode;
    SentryVisionWriteU16(out + 6, s->accepted_count);
    SentryVisionWriteU16(out + 8, s->rejected_count);
    SentryVisionWriteU16(out + 10, SentryVisionCrc(out, 10));
}
void SentryVisionEncodeTelemetry(const SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint8_t out[43])
{
    memset(out, 0, 43);
    out[0] = 'S';
    out[1] = 'P';
    /* Work mode selects the detector; command mode selects aim/fire authority. */
    const uint8_t available = (s->flags & (SENTRY_VISION_ARMED | SENTRY_VISION_FEEDBACK_FRESH)) ==
                              (SENTRY_VISION_ARMED | SENTRY_VISION_FEEDBACK_FRESH);
    const uint8_t inhibited = s->flags & SENTRY_VISION_MANUAL_OVERRIDE;
    out[2] = f && available && !inhibited && (f->capabilities & SENTRY_VISION_YAW) ? f->work_mode : 0;
    if (f && (s->flags & SENTRY_VISION_FEEDBACK_FRESH))
    {
        float q[4];
        Quaternion(f, q);
        for (unsigned i = 0; i < 4; i++)
            SentryVisionWriteFloat(out + 3 + 4 * i, q[i]);
        SentryVisionWriteFloat(out + 19, f->yaw);
        SentryVisionWriteFloat(out + 23, f->yaw_rate);
        SentryVisionWriteFloat(out + 27, f->pitch);
        SentryVisionWriteFloat(out + 31, f->pitch_rate);
        if (f->capabilities & SENTRY_VISION_BULLET_SPEED)
            SentryVisionWriteFloat(out + 35, f->bullet_speed);
        if (f->capabilities & SENTRY_VISION_BULLET_COUNT)
            SentryVisionWriteU16(out + 39, f->bullet_count);
    }
    SentryVisionWriteU16(out + 41, SentryVisionCrc(out, 41));
}
uint8_t SentryVisionYawRate(const SentryVisionState_s *s, const SentryVisionFeedback_s *f, float relative, float min,
                            float max, int sign, float max_rate, uint32_t now, float *rate)
{
    if (!s || !f || !rate || !s->active || (sign != -1 && sign != 1) || !isfinite(relative) || !isfinite(min) ||
        !isfinite(max) || min >= max || !isfinite(max_rate) || max_rate <= 0 ||
        (uint32_t)(now - s->command_ms) > SENTRY_VISION_TIMEOUT_MS)
        return 0;
    const float age = (float)(uint32_t)(now - s->command_ms) * .001f;
    const float target = s->command.yaw + s->command.yaw_rate * age + .5f * s->command.yaw_accel * age * age;
    const float error = remainderf(target - f->yaw, 6.283185307f);
    const float joint_target = relative + error * 57.29577951f * (float)sign;
    if (!isfinite(joint_target) || joint_target < min || joint_target > max)
        return 0;
    float requested = (4.0f * error + s->command.yaw_rate + s->command.yaw_accel * age) * (float)sign;
    *rate = fmaxf(-max_rate, fminf(max_rate, requested));
    return 1;
}
__attribute__((weak)) uint8_t SentryVisionReadFeedback(SentryVisionFeedback_s *f)
{
    memset(f, 0, sizeof(*f));
    return 0;
}

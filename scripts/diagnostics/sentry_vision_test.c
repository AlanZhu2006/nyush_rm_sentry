#include "sentry_vision.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned nav_count, sp_count;
static void Frame(void *ctx, const uint8_t *p, size_t n)
{
    (void)ctx;
    if (n == 15)
        nav_count++;
    else
    {
        assert(n == 29);
        sp_count++;
    }
    (void)p;
}
static void Command(uint8_t *p, uint8_t mode, float yaw, float pitch)
{
    memset(p, 0, 29);
    p[0] = 'S';
    p[1] = 'P';
    p[2] = mode;
    SentryVisionWriteFloat(p + 3, yaw);
    SentryVisionWriteFloat(p + 15, pitch);
    SentryVisionWriteU16(p + 27, SentryVisionCrc(p, 27));
}
int main(void)
{
    assert(SentryVisionCrc((const uint8_t *)"123456789", 9) == 0x6f91);
    assert(SentryNavCrc((const uint8_t *)"123456789", 9) == 0x29b1);
    SentryVisionState_s s = {0};
    SentryVisionFeedback_s f = {0};
    uint8_t p[29], sc[12], sp[43];
    Command(p, 1, 0.1f, 0);
    static const uint8_t golden[29] = {0x53, 0x50, 0x01, 0xcd, 0xcc, 0xcc, 0x3d, 0, 0, 0, 0, 0, 0,    0,   0,
                                       0,    0,    0,    0,    0,    0,    0,    0, 0, 0, 0, 0, 0xfe, 0xeb};
    assert(memcmp(p, golden, 29) == 0);
    assert(!SentryVisionAccept(&s, p, 29, 100, &f));
    assert(s.rejected_count == 1);
    f.capabilities = SENTRY_VISION_YAW | SENTRY_VISION_ATTITUDE;
    f.armed = 1;
    f.work_mode = 1;
    f.timestamp_ms = 100;
    f.quaternion[0] = 1;
    SentryVisionUpdate(&s, &f, 100, 0);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 1); /* Starts detector before first command. */
    assert(SentryVisionAccept(&s, p, 29, 100, &f));
    SentryVisionUpdate(&s, &f, 100, 0);
    assert(s.active);
    SentryVisionEncodeStatus(&s, &f, sc);
    assert(sc[0] == 'S' && sc[1] == 'C' && sc[2] == 1 && sc[3] == 9);
    assert(sc[4] & SENTRY_VISION_ACTIVE);
    static const uint8_t golden_status[12] = {0x53,0x43,1,9,35,1,1,0,1,0,0x1f,0x1c};
    assert(memcmp(sc,golden_status,12)==0);
    assert(SentryVisionCrc(sc,10)==((uint16_t)sc[10]|((uint16_t)sc[11]<<8)));
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 1);
    assert(SentryVisionReadFloat(sp + 3) == 1);
    SentryVisionUpdate(&s, &f, 101, 1);
    assert(!s.active && (s.flags & SENTRY_VISION_MANUAL_OVERRIDE));
    SentryVisionUpdate(&s, &f, 102, 0);
    assert(s.active);
    SentryVisionUpdate(&s, &f, 500, 0);
    assert(!s.active && (s.flags & SENTRY_VISION_TIMEOUT));
    f.timestamp_ms = 501;
    Command(p, 2, 0, 0);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 1, 0, .1f);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 1, NAN, 0);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 1, INFINITY, 0);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 1, 3.16f, 0);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 3, 0, 0);
    assert(!SentryVisionAccept(&s, p, 29, 501, &f));
    Command(p, 0, 0, 0);
    assert(SentryVisionAccept(&s, p, 29, 501, &f));
    SentryVisionUpdate(&s, &f, 501, 0);
    assert(!s.active);
    SentryVisionStream_s stream = {0};
    uint8_t nav[15] = {0xa5, 0x5a, 1, 1};
    SentryVisionWriteU16(nav + 13, SentryNavCrc(nav, 13));
    Command(p, 1, 0, 0);
    uint8_t mixed[100] = {0x53, 0xa5, 0x55};
    memcpy(mixed + 3, nav, 15);
    memcpy(mixed + 18, p, 29);
    memcpy(mixed + 47, nav, 15);
    for (size_t i = 0; i < 62; i++)
        SentryVisionStreamPush(&stream, mixed + i, 1, Frame, 0);
    assert(nav_count == 2 && sp_count == 1);
    p[10] ^= 1;
    SentryVisionStreamPush(&stream, p, 29, Frame, 0);
    SentryVisionStreamPush(&stream, nav, 15, Frame, 0);
    assert(nav_count == 3 && sp_count == 1);
    assert(stream.invalid_sp_count == 1);
    /* All split boundaries, both frame families, and a CRC-corrupt frame resync. */
    Command(p, 1, 0, 0);
    for (size_t split = 0; split <= 29; split++)
    {
        SentryVisionStream_s split_stream = {0};
        unsigned before = sp_count;
        SentryVisionStreamPush(&split_stream, p, split, Frame, 0);
        SentryVisionStreamPush(&split_stream, p + split, 29 - split, Frame, 0);
        assert(sp_count == before + 1);
    }
    f.timestamp_ms = 600;
    f.capabilities |= SENTRY_VISION_FIRE;
    Command(p, 2, .1f, 0);
    assert(SentryVisionAccept(&s, p, 29, 600, &f));
    SentryVisionUpdate(&s, &f, 600, 0);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 1); /* FIRE command must not select SMALL_BUFF. */
    float rate = 0;
    assert(SentryVisionYawRate(&s, &f, 30, 10, 60, 1, 1, 600, &rate));
    assert(fabsf(rate - .4f) < 1e-5f);
    assert(!SentryVisionYawRate(&s, &f, 59, 10, 60, 1, 1, 600, &rate));
    assert(SentryVisionYawRate(&s, &f, 30, 10, 60, -1, 1, 600, &rate));
    assert(fabsf(rate + .4f) < 1e-5f);
    assert(!SentryVisionYawRate(&s, &f, 30, 10, 60, 0, 1, 600, &rate));
    assert(!SentryVisionYawRate(&s, &f, 30, 10, 60, 1, 1, 900, &rate));
    /* Explicit detector idle revokes cached authority despite fresh real sensing. */
    SentryVisionFeedback_s idle_feedback = f;
    SentryVisionState_s idle_state = s;
    idle_feedback.work_mode = 0;
    assert(SentryVisionFeedbackValid(&idle_feedback, 600));
    SentryVisionUpdate(&idle_state, &idle_feedback, 600, 0);
    assert(!idle_state.active && !idle_state.seen);
    Command(p, 1, 0, 0);
    assert(!SentryVisionAccept(&idle_state, p, 29, 600, &idle_feedback));
    Command(p, 2, 0, 0);
    assert(!SentryVisionAccept(&idle_state, p, 29, 600, &idle_feedback));
    Command(p, 0, 0, 0);
    assert(SentryVisionAccept(&idle_state, p, 29, 600, &idle_feedback));
    f.armed = 0;
    SentryVisionUpdate(&s, &f, 601, 0);
    assert(!s.active && !s.seen);
    f.armed = 1;
    SentryVisionUpdate(&s, &f, 602, 0);
    assert(!s.active);
    /* Timeout revokes actuation but leaves requested detector mode available. */
    Command(p, 1, 0, 0);
    assert(SentryVisionAccept(&s, p, 29, 602, &f));
    f.timestamp_ms = 850;
    SentryVisionUpdate(&s, &f, 850, 0);
    assert(!s.active);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 1 && (s.flags & SENTRY_VISION_TIMEOUT));
    Command(p, 1, NAN, 0);
    assert(!SentryVisionAccept(&s, p, 29, 850, &f));
    SentryVisionUpdate(&s, &f, 850, 0);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 1);
    SentryVisionUpdate(&s, &f, 850, 1);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 0);
    f.timestamp_ms = 600;
    /* Mixed-axis quaternion consistency, including q/-q equivalence. */
    f.yaw = .4f;
    f.pitch = -.3f;
    f.roll = .2f;
    float cz = cosf(.2f), sz = sinf(.2f), cy = cosf(.1f), sy = sinf(.1f), cx = cosf(-.15f), sx = sinf(-.15f);
    f.quaternion[0] = cz * cy * cx + sz * sy * sx;
    f.quaternion[1] = cz * cy * sx - sz * sy * cx;
    f.quaternion[2] = cz * sy * cx + sz * cy * sx;
    f.quaternion[3] = sz * cy * cx - cz * sy * sx;
    assert(SentryVisionFeedbackValid(&f, 600));
    SentryVisionUpdate(&s, &f, 600, 0);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    for (size_t i = 0; i < 4; i++)
        assert(fabsf(SentryVisionReadFloat(sp + 3 + 4 * i) - f.quaternion[i]) < 1e-6f);
    assert(fabsf(SentryVisionReadFloat(sp + 19) - .4f) < 1e-6f);
    assert(fabsf(SentryVisionReadFloat(sp + 27) + .3f) < 1e-6f);
    for (size_t i = 0; i < 4; i++)
    {
        f.quaternion[i] = -f.quaternion[i];
    }
    assert(SentryVisionFeedbackValid(&f, 600));
    f.quaternion[1] += .5f;
    assert(!SentryVisionFeedbackValid(&f, 600));
    memset(&f, 0, sizeof(f));
    assert(!SentryVisionReadFeedback(&f));
    SentryVisionUpdate(&s, &f, 600, 0);
    SentryVisionEncodeTelemetry(&s, &f, sp);
    assert(sp[2] == 0 && SentryVisionReadFloat(sp + 3) == 0);
    puts("firmware vision protocol/state: PASS");
    return 0;
}

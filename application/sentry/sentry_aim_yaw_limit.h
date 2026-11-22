#ifndef SENTRY_AIM_YAW_LIMIT_H
#define SENTRY_AIM_YAW_LIMIT_H

#include <math.h>
#include <stdint.h>

typedef struct
{
    float target_deg;
    uint8_t enabled;
    uint8_t at_right_limit;
    uint8_t at_left_limit;
} SentryYawLimitResult_s;

typedef struct
{
    uint8_t at_left_limit;
    uint8_t at_right_limit;
} SentryYawLimitFlags_s;

/** Enter near either soft boundary; clear only after a deliberate inward move. */
static inline SentryYawLimitFlags_s SentryYawUpdateLimitFlags(
    float relative_deg, float left_travel_deg, float right_guard_deg,
    float engage_tolerance_deg, float release_margin_deg, SentryYawLimitFlags_s previous)
{
    SentryYawLimitFlags_s flags = { .at_left_limit = 1u, .at_right_limit = 1u };
    if (!isfinite(relative_deg) || !isfinite(left_travel_deg) ||
        !isfinite(right_guard_deg) || !isfinite(engage_tolerance_deg) ||
        !isfinite(release_margin_deg) || left_travel_deg >= 180.0f ||
        right_guard_deg < 0.0f || engage_tolerance_deg < 0.0f ||
        release_margin_deg <= engage_tolerance_deg ||
        left_travel_deg - right_guard_deg <= 2.0f * release_margin_deg)
        return flags;

    const float lower = -left_travel_deg;
    const float upper = -right_guard_deg;
    flags.at_left_limit = previous.at_left_limit
        ? relative_deg < lower + release_margin_deg
        : relative_deg <= lower + engage_tolerance_deg;
    flags.at_right_limit = previous.at_right_limit
        ? relative_deg > upper - release_margin_deg
        : relative_deg >= upper - engage_tolerance_deg;
    return flags;
}

/** Motor angle relative to the fixed right edge; positive is past the edge. */
static inline float SentryYawRelativeDeg(uint16_t ecd, uint16_t right_ecd)
{
    if (ecd >= 8192u || right_ecd >= 8192u)
        return NAN;
    int32_t delta = (int32_t)ecd - (int32_t)right_ecd;
    if (delta > 4096)
        delta -= 8192;
    else if (delta < -4096)
        delta += 8192;
    return (float)delta * (360.0f / 8192.0f);
}

/** Limit a logical rate. left_travel_deg already excludes the inward left guard. */
static inline SentryYawLimitResult_s SentryYawLimitStep(
    uint16_t ecd, uint16_t right_ecd, float current_total_deg,
    float previous_target_deg, float logical_rate_mrad_s, int8_t command_sign,
    float left_travel_deg, float right_guard_deg, float dt_s, uint8_t active)
{
    SentryYawLimitResult_s result = {
        .target_deg = current_total_deg,
        .enabled = 0u,
        .at_right_limit = 1u,
        .at_left_limit = 1u,
    };
    const float relative = SentryYawRelativeDeg(ecd, right_ecd);
    if (!isfinite(relative) || !isfinite(current_total_deg) ||
        !isfinite(previous_target_deg) || !isfinite(logical_rate_mrad_s) ||
        !isfinite(left_travel_deg) || !isfinite(right_guard_deg) ||
        !isfinite(dt_s) || dt_s < 0.0f || dt_s > 0.05f ||
        (command_sign != -1 && command_sign != 1) ||
        right_guard_deg < 0.0f || left_travel_deg <= right_guard_deg ||
        left_travel_deg >= 180.0f)
        return result;

    const float lower = -left_travel_deg;
    const float upper = -right_guard_deg;
    const float rate = fmaxf(-600.0f, fminf(600.0f, logical_rate_mrad_s)) *
                       (float)command_sign;
    result.at_right_limit = relative >= upper;
    result.at_left_limit = relative <= lower;

    /* A zero request at either edge must not leave an integral pushing into
       it. An inward request can still recover from an out-of-range start. */
    if ((relative >= upper && rate >= 0.0f) ||
        (relative <= lower && rate <= 0.0f))
        return result;

    const float right_total = current_total_deg - relative;
    float target = active ? previous_target_deg - right_total : relative;
    const float step = rate * 0.001f * (180.0f / 3.14159265358979323846f) * dt_s;
    if (relative > upper)
    {
        /* Escape left gradually; do not jump to the inward guard position. */
        target = fmaxf(lower, fminf(target, relative) + step);
    }
    else if (relative < lower)
    {
        target = fminf(upper, fmaxf(target, relative) + step);
    }
    else
    {
        target = fmaxf(lower, fminf(upper, target));
        target = fmaxf(lower, fminf(upper, target + step));
    }
    result.target_deg = right_total + target;
    result.enabled = 1u;
    return result;
}

/** Clip actual output toward an edge, including residual speed-PI output. */
static inline float SentryYawClampOutput(float output, float relative_deg,
                                        float left_travel_deg, float right_guard_deg)
{
    if (!isfinite(output) || !isfinite(relative_deg) ||
        !isfinite(left_travel_deg) || !isfinite(right_guard_deg) ||
        right_guard_deg < 0.0f || left_travel_deg <= right_guard_deg ||
        left_travel_deg >= 180.0f)
        return 0.0f;
    if ((relative_deg >= -right_guard_deg && output > 0.0f) ||
        (relative_deg <= -left_travel_deg && output < 0.0f))
        return 0.0f;
    return output;
}

#endif /* SENTRY_AIM_YAW_LIMIT_H */

#ifndef SENTRY_AIM_YAW_PROFILE_H
#define SENTRY_AIM_YAW_PROFILE_H

#include <math.h>
#include <stdint.h>

typedef struct
{
    float rate_deg_s;
    uint8_t valid;
} SentryYawProfileResult_s;

/** Shape a motor-frame rate; angle, speed and acceleration units are degrees. */
static inline SentryYawProfileResult_s SentryYawProfileStep(
    float requested_rate_deg_s, float previous_rate_deg_s,
    float relative_deg, float target_relative_deg,
    float left_travel_deg, float right_guard_deg, float max_rate_deg_s,
    float accel_deg_s2, float brake_deg_s2, float dt_s,
    uint8_t at_left, uint8_t at_right)
{
    SentryYawProfileResult_s result = { .rate_deg_s = 0.0f, .valid = 0u };
    if (!isfinite(requested_rate_deg_s) || !isfinite(previous_rate_deg_s) ||
        !isfinite(relative_deg) || !isfinite(target_relative_deg) ||
        !isfinite(left_travel_deg) || !isfinite(right_guard_deg) ||
        !isfinite(max_rate_deg_s) || !isfinite(accel_deg_s2) ||
        !isfinite(brake_deg_s2) || !isfinite(dt_s) ||
        right_guard_deg < 0.0f || left_travel_deg <= right_guard_deg ||
        left_travel_deg >= 180.0f || max_rate_deg_s <= 0.0f ||
        accel_deg_s2 <= 0.0f || brake_deg_s2 <= 0.0f ||
        dt_s <= 0.0f || dt_s > 0.05f)
        return result;

    result.valid = 1u;
    /* Releasing the key retains the existing immediate target-hold behavior. */
    if (requested_rate_deg_s == 0.0f || (at_left && at_right) ||
        (at_left && requested_rate_deg_s < 0.0f) ||
        (at_right && requested_rate_deg_s > 0.0f))
        return result;

    float previous = fmaxf(-max_rate_deg_s, fminf(max_rate_deg_s, previous_rate_deg_s));
    if ((at_left && previous < 0.0f) || (at_right && previous > 0.0f))
        previous = 0.0f;

    /* Use the closer of actual and target positions to each original soft
       target edge. An out-of-range position still permits inward recovery. */
    const float positive_distance = fmaxf(0.0f,
        -right_guard_deg - fmaxf(relative_deg, target_relative_deg));
    const float negative_distance = fmaxf(0.0f,
        fminf(relative_deg, target_relative_deg) + left_travel_deg);
    const float positive_cap = positive_distance > 0.0f ? fminf(max_rate_deg_s,
        sqrtf(2.0f * brake_deg_s2 * positive_distance)) : 0.0f;
    const float negative_cap = negative_distance > 0.0f ? fminf(max_rate_deg_s,
        sqrtf(2.0f * brake_deg_s2 * negative_distance)) : 0.0f;
    const float desired = fmaxf(-negative_cap,
        fminf(positive_cap, requested_rate_deg_s));
    const uint8_t reversing = (previous > 0.0f && desired < 0.0f) ||
                              (previous < 0.0f && desired > 0.0f);
    const float acceleration = reversing || fabsf(desired) < fabsf(previous)
        ? brake_deg_s2 : accel_deg_s2;
    const float step = acceleration * dt_s;
    const float next = previous + fmaxf(-step, fminf(step, desired - previous));

    /* Safety envelopes take precedence over the acceleration ramp. */
    result.rate_deg_s = fmaxf(-negative_cap, fminf(positive_cap, next));
    return result;
}

#endif /* SENTRY_AIM_YAW_PROFILE_H */

#ifndef SENTRY_VISION_H
#define SENTRY_VISION_H
#include "sentry_vision_protocol.h"
typedef struct
{
    uint8_t mode;
    float yaw, yaw_rate, yaw_accel, pitch, pitch_rate, pitch_accel;
} SentryVisionCommand_s;
/* All attitude fields describe the calibrated camera/gimbal frame, SP signs,
   radians, same sample timestamp. Quaternion is wxyz: yaw Z, pitch X, roll Y.
   capabilities include only available actuators/services; armed is real
   authority. */
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t capabilities, armed, work_mode;
    float quaternion[4], yaw, pitch, roll, yaw_rate, pitch_rate, bullet_speed;
    uint16_t bullet_count;
} SentryVisionFeedback_s;
typedef struct
{
    SentryVisionCommand_s command;
    uint32_t command_ms;
    uint16_t accepted_count, rejected_count;
    uint8_t seen, active, flags;
} SentryVisionState_s;
uint8_t SentryVisionFeedbackValid(const SentryVisionFeedback_s *f, uint32_t now_ms);
uint8_t SentryVisionAccept(SentryVisionState_s *s, const uint8_t *data, size_t length, uint32_t now_ms,
                           const SentryVisionFeedback_s *f);
void SentryVisionUpdate(SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint32_t now_ms, uint8_t manual);
void SentryVisionEncodeStatus(const SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint8_t out[12]);
void SentryVisionEncodeTelemetry(const SentryVisionState_s *s, const SentryVisionFeedback_s *f, uint8_t out[43]);
/* Position-error servo in the calibrated frame. Outputs motor rate in rad/s;
   absolute targets outside the configured joint travel reject, never become rates. */
uint8_t SentryVisionYawRate(const SentryVisionState_s *state, const SentryVisionFeedback_s *feedback,
                            float relative_deg, float min_deg, float max_deg, int sign, float max_rate_rad_s,
                            uint32_t now_ms, float *rate_rad_s);
/* Application hooks. Defaults supply no feedback or authority. A board-specific
   real provider must be configured before any aiming capability is advertised.
 */
uint8_t SentryVisionReadFeedback(SentryVisionFeedback_s *feedback);
#endif

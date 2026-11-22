#ifndef SENTRY_VISION_PROTOCOL_H
#define SENTRY_VISION_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define SENTRY_VISION_COMMAND_SIZE 29u
#define SENTRY_VISION_TELEMETRY_SIZE 43u
#define SENTRY_VISION_STATUS_SIZE 12u
#define SENTRY_VISION_TIMEOUT_MS 200u
#define SENTRY_VISION_FEEDBACK_TIMEOUT_MS 100u
enum
{
    SENTRY_VISION_YAW = 1,
    SENTRY_VISION_PITCH = 2,
    SENTRY_VISION_FIRE = 4,
    SENTRY_VISION_ATTITUDE = 8,
    SENTRY_VISION_BULLET_SPEED = 16,
    SENTRY_VISION_BULLET_COUNT = 32
};
enum
{
    SENTRY_VISION_ARMED = 1,
    SENTRY_VISION_ACTIVE = 2,
    SENTRY_VISION_REJECTED = 4,
    SENTRY_VISION_TIMEOUT = 8,
    SENTRY_VISION_MANUAL_OVERRIDE = 16,
    SENTRY_VISION_FEEDBACK_FRESH = 32
};
uint16_t SentryVisionCrc(const uint8_t *data, size_t length);
uint16_t SentryNavCrc(const uint8_t *data, size_t length);
void SentryVisionWriteU16(uint8_t *p, uint16_t v);
void SentryVisionWriteFloat(uint8_t *p, float v);
float SentryVisionReadFloat(const uint8_t *p);
typedef struct
{
    uint8_t data[64];
    size_t used;
    uint16_t invalid_sp_count;
} SentryVisionStream_s;
typedef void (*SentryVisionFrameFn)(void *context, const uint8_t *data, size_t length);
/* RX-only mixed command stream, CRC checked independently for SP29 and nav15.
 */
void SentryVisionStreamPush(SentryVisionStream_s *s, const uint8_t *data, size_t length, SentryVisionFrameFn callback,
                            void *context);
#endif

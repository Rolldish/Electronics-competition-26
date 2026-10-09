#ifndef APPLICATION_VISION_LINK_H
#define APPLICATION_VISION_LINK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t pi_session_id;
    uint32_t capture_timestamp_ms;
    uint32_t local_receive_ms;
    uint32_t measurement_age_ms;
    uint32_t measurement_update_count;
    int16_t position_0p1mm;
    int16_t velocity_mmps;
    int16_t target_position_0p1mm;
    uint16_t run_id;
    uint8_t confidence;
    uint8_t flags;
    uint8_t task_id;
    uint8_t control_flags;
    uint8_t sequence;
    uint8_t valid;
    uint8_t velocity_valid;
} VisionLinkSnapshot;

void VisionLink_GetSnapshot(VisionLinkSnapshot *output);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_VISION_LINK_H */

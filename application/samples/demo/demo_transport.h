/**
 * @file demo_transport.h
 * @brief Reliable SLE transport state for the demo bridge.
 */

#ifndef DEMO_TRANSPORT_H
#define DEMO_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include "demo_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEMO_TRANSPORT_TX_IDLE = 0,
    DEMO_TRANSPORT_TX_SEND_FRAGMENT,
    DEMO_TRANSPORT_TX_WAIT_IND_CFM,
    DEMO_TRANSPORT_TX_WAIT_FRAME_ACK,
} demo_transport_tx_phase_t;

typedef struct {
    bool active;
    const demo_frame_slot_t *frame;
    uint16_t frame_id;
    uint16_t offset;
    uint16_t last_payload_len;
    uint8_t frag_idx;
    uint8_t frag_count;
    uint8_t retry_count;
    uint8_t in_flight_type;
    demo_transport_tx_phase_t phase;
    uint32_t deadline_us;
} demo_transport_tx_state_t;

typedef struct {
    bool active;
    uint16_t frame_id;
    uint16_t total_len;
    uint16_t received_len;
    uint8_t frag_count;
    uint8_t next_frag_idx;
    uint16_t last_completed_frame_id;
    bool last_completed_valid;
    uint16_t duplicate_frame_id;
    uint8_t duplicate_frag_count;
    uint8_t duplicate_next_frag_idx;
    bool duplicate_active;
    uint8_t buffer[DEMO_LOGICAL_FRAME_MAX_SIZE];
    uint32_t first_fragment_us;
    uint32_t last_fragment_us;
    uint32_t max_gap_us;
} demo_transport_rx_state_t;

typedef struct {
    bool pending;
    bool waiting_cfm;
    uint8_t retry_count;
    uint8_t packet_type;
    uint16_t frame_id;
    uint32_t deadline_us;
} demo_transport_ack_state_t;

void demo_transport_reset_tx(demo_transport_tx_state_t *state);
void demo_transport_reset_rx(demo_transport_rx_state_t *state);
void demo_transport_reset_ack(demo_transport_ack_state_t *state);
uint32_t demo_transport_deadline_from_ms(uint32_t now_us, uint32_t timeout_ms);
bool demo_transport_deadline_expired(uint32_t now_us, uint32_t deadline_us);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_TRANSPORT_H */

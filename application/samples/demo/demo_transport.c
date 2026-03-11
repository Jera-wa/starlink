/**
 * @file demo_transport.c
 * @brief Reliable SLE transport state helpers for the demo bridge.
 */

#include "demo_transport.h"
#include "securec.h"

void demo_transport_reset_tx(demo_transport_tx_state_t *state)
{
    if (state == NULL) {
        return;
    }
    (void)memset_s(state, sizeof(*state), 0, sizeof(*state));
}

void demo_transport_reset_rx(demo_transport_rx_state_t *state)
{
    if (state == NULL) {
        return;
    }
    (void)memset_s(state, sizeof(*state), 0, sizeof(*state));
}

void demo_transport_reset_ack(demo_transport_ack_state_t *state)
{
    if (state == NULL) {
        return;
    }
    (void)memset_s(state, sizeof(*state), 0, sizeof(*state));
}

uint32_t demo_transport_deadline_from_ms(uint32_t now_us, uint32_t timeout_ms)
{
    return now_us + (timeout_ms * 1000U);
}

bool demo_transport_deadline_expired(uint32_t now_us, uint32_t deadline_us)
{
    if (deadline_us == 0U) {
        return false;
    }
    return now_us >= deadline_us;
}

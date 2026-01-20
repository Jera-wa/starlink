/**
 * @file demo_sle.h
 * @brief SLE Module Interface (Server/Client)
 */

#ifndef DEMO_SLE_H
#define DEMO_SLE_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize SLE (Server or Client based on config)
 * @return 0 on success
 */
int demo_sle_init(void);

/**
 * @brief Check if SLE is connected
 */
bool demo_sle_is_connected(void);

/**
 * @brief Get SLE RX ring buffer (data received from peer)
 */
ring_buffer_t* demo_sle_get_rx_ring(void);

/**
 * @brief Get SLE TX ring buffer (data to send to peer)
 */
ring_buffer_t* demo_sle_get_tx_ring(void);

/**
 * @brief Process SLE TX - send pending data
 * Call this from main loop when connected
 * @return Bytes sent this call
 */
uint32_t demo_sle_tx_process(void);

/**
 * @brief Get current MTU size
 */
uint16_t demo_sle_get_mtu(void);

/**
 * @brief Get connection ID
 */
uint16_t demo_sle_get_conn_id(void);

/**
 * @brief Process deferred SLE operations (Client only)
 * Call this from main loop to handle operations that can't run in callbacks
 */
void demo_sle_process_deferred(void);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_SLE_H */

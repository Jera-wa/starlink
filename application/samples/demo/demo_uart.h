/**
 * @file demo_uart.h
 * @brief UART DMA+Interrupt Module Interface
 */

#ifndef DEMO_UART_H
#define DEMO_UART_H

#include <stdbool.h>
#include <stdint.h>
#include "demo_frame.h"

#ifdef __cplusplus
extern "C" {
#endif

int demo_uart_init(void);
void demo_uart_deinit(void);
void demo_uart_reset_queues(void);

const demo_frame_slot_t *demo_uart_rx_frame_peek(void);
void demo_uart_rx_frame_consume(void);
bool demo_uart_queue_tx_frame(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us, uint32_t stage_duration_us);
bool demo_uart_tx_can_fast_path(void);
bool demo_uart_tx_direct_frame(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us, uint32_t stage_duration_us);

uint32_t demo_uart_tx_process(void);
bool demo_uart_tx_is_busy(void);
int32_t demo_uart_tx_direct(const uint8_t *data, uint16_t len);
uint32_t demo_uart_tx_send(const uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* DEMO_UART_H */

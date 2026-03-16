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

typedef struct {
    uint32_t idle_isr_count;
    uint32_t raw_callback_count;
    uint32_t raw_callback_bytes;
    uint32_t raw_callback_last_len;
    uint32_t publish_count;
    uint32_t publish_bytes;
    uint32_t publish_from_idle_cb;
    uint32_t publish_from_soft_flush;
    uint32_t publish_from_idle_fallback;
    uint32_t forced_idle_request_count;
    uint16_t last_transfer_num;
    uint16_t last_remaining;
    uint16_t last_received_blocks;
    uint16_t last_received_len;
    uint16_t last_idle_tail_len;
    uint16_t last_combined_len;
    uint8_t last_publish_reason;
    uint8_t last_rx_fifo_empty;
    uint16_t last_fifo_drain_len;
    uint32_t overrun_error_count;
    uint32_t frame_error_count;
    uint32_t parity_error_count;
} demo_uart_rx_diag_t;

int demo_uart_init(void);
void demo_uart_deinit(void);
void demo_uart_reset_queues(void);
void demo_uart_rx_poll(void);
uint32_t demo_uart_get_idle_isr_count(void);
void demo_uart_get_rx_diag(demo_uart_rx_diag_t *diag);

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

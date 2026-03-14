/**
 * @file demo_config.h
 * @brief SLE UART Bridge configuration and runtime statistics.
 */

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include "event/osal_event.h"
#include <stdbool.h>
#include <stdint.h>

/*============================================================================
 * Event-Driven Architecture Configuration
 *============================================================================*/
#define DEMO_EVENT_UART_RX (1U << 0)
#define DEMO_EVENT_SLE_RX (1U << 1)
#define DEMO_EVENT_SLE_TX_DONE (1U << 2)
#define DEMO_EVENT_ALL                                                         \
  (DEMO_EVENT_UART_RX | DEMO_EVENT_SLE_RX | DEMO_EVENT_SLE_TX_DONE)

#define DEMO_EVENT_TIMEOUT_MS 2

extern osal_event g_bridge_event;

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * UART Configuration
 *============================================================================*/
#define DEMO_UART_BUS CONFIG_SLE_UART_BUS
#define DEMO_UART_BAUDRATE 4000000
#define DEMO_UART_TX_PIN CONFIG_UART_TXD_PIN
#define DEMO_UART_RX_PIN CONFIG_UART_RXD_PIN

/*============================================================================
 * Logical Frame / Queue Configuration
 *============================================================================*/
#define DEMO_UART_RX_BUFFER_SIZE DEMO_LOGICAL_FRAME_MAX_SIZE
#define DEMO_UART_RX_THRESHOLD 16
#define DEMO_UART_TX_BUFFER_SIZE 4096
#define DEMO_UART_FRAME_GAP_TIMEOUT_US 20000U

#define DEMO_LOGICAL_FRAME_MAX_SIZE 2304
#define DEMO_FRAME_QUEUE_DEPTH 6
#define DEMO_DMA_CHUNK_SIZE DEMO_LOGICAL_FRAME_MAX_SIZE
#define DEMO_TX_BUFFER_COUNT 6

/*============================================================================
 * SLE Transport Configuration
 *============================================================================*/
/* Connected link scheduling interval, unit 125 us. */
#define DEMO_SLE_LINK_INTERVAL_MIN_LEGAL 0x0010
#define DEMO_SLE_LINK_INTERVAL_MAX_LEGAL 0x7D00

#define DEMO_SLE_CONN_INTV_MIN 0x0010
#define DEMO_SLE_CONN_INTV_MAX 0x0010

#define DEMO_SLE_ADV_INTERVAL_MIN 0x00C8
#define DEMO_SLE_ADV_INTERVAL_MAX 0x00C8
#define DEMO_SLE_ADV_CONN_INTV_MIN 0x001E
#define DEMO_SLE_ADV_CONN_INTV_MAX 0x001E
#define DEMO_SLE_SUPERVISION_TIMEOUT 0x01F4

#define DEMO_SLE_MTU_SIZE 1100
#define DEMO_SLE_DATA_LEN_TARGET DEMO_SLE_MTU_SIZE
#define DEMO_SLE_FRAG_HEADER_SIZE 12
/* Set to 0 for low-latency experiments, 1 for peer-reassembly ACK semantics. */
#define DEMO_SLE_FRAME_ACK_ENABLE 0
#define DEMO_SLE_INDICATE_TIMEOUT_MS 20
#define DEMO_SLE_FRAME_ACK_TIMEOUT_MS 30
#define DEMO_SLE_ACK_TIMEOUT_MS 20
#define DEMO_SLE_MAX_RETRIES 3
#define DEMO_SLE_LOW_LATENCY_ENABLE 1
#define DEMO_SLE_LOW_LATENCY_RATE 1 /* 500 Hz target, about 2 ms */

#define DEMO_SLE_ADV_HANDLE 1
#define DEMO_SLE_SERVER_NAME "sle_demo_server"
#define DEMO_SLE_SEEK_INTERVAL 100
#define DEMO_SLE_SEEK_WINDOW 100

/*============================================================================
 * Task Configuration
 *============================================================================*/
#define DEMO_TASK_PRIORITY 20
#define DEMO_TASK_STACK_SIZE 0x1200
#define DEMO_BRIDGE_POLL_MS 1

/*============================================================================
 * Debug Configuration
 *============================================================================*/
#define DEMO_DEBUG_LOG 0
#define DEMO_STATS_INTERVAL_MS 10000

#if DEMO_DEBUG_LOG
#define DEMO_LOG(fmt, ...) osal_printk("[DEMO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEMO_LOG(fmt, ...)
#endif

#define DEMO_INFO(fmt, ...) osal_printk("[DEMO] " fmt "\r\n", ##__VA_ARGS__)
#define DEMO_ERR(fmt, ...) osal_printk("[DEMO ERR] " fmt "\r\n", ##__VA_ARGS__)

/*============================================================================
 * Statistics Structure
 *============================================================================*/
typedef struct {
  volatile uint32_t uart_rx_bytes;
  volatile uint32_t uart_tx_bytes;
  volatile uint32_t sle_rx_bytes;
  volatile uint32_t sle_tx_bytes;

  volatile uint32_t uart_rx_frames;
  volatile uint32_t uart_tx_frames;
  volatile uint32_t sle_tx_frames;
  volatile uint32_t sle_rx_frames;
  volatile uint32_t sle_tx_fragments;
  volatile uint32_t sle_rx_fragments;

  volatile uint32_t uart_rx_drop_bytes;
  volatile uint32_t sle_rx_drop_bytes;
  volatile uint32_t ring_overflow;
  volatile uint32_t uart_rx_drop_frames;
  volatile uint32_t uart_tx_drop_frames;
  volatile uint32_t uart_tx_fast_path_hits;
  volatile uint32_t uart_tx_fast_path_misses;
  volatile uint32_t uart_rx_oversize_frames;
  volatile uint32_t sle_reassembly_drop_frames;
  volatile uint32_t sle_reassembly_reset_cnt;
  volatile uint32_t uart_rx_invalid_bytes;

  volatile uint32_t sle_tx_fail;
  volatile uint32_t sle_tx_busy_cnt;
  volatile uint32_t sle_ack_sent;
  volatile uint32_t sle_ack_received;
  volatile uint32_t sle_tx_retry_frames;
  volatile uint32_t sle_tx_hard_fail;
  volatile uint32_t sle_rx_duplicate_frames;
  volatile uint32_t sle_indicate_timeout_cnt;
  volatile uint32_t sle_frame_ack_timeout_cnt;

  volatile uint16_t uart_rx_ring_hwm;
  volatile uint16_t uart_tx_ring_hwm;
  volatile uint16_t sle_tx_ring_hwm;
  volatile uint16_t sle_rx_ring_hwm;

  volatile uint16_t sle_requested_conn_interval_min;
  volatile uint16_t sle_requested_conn_interval_max;
  volatile uint16_t sle_negotiated_conn_interval;
  volatile uint16_t sle_requested_data_len;
  volatile uint16_t sle_effective_payload;
  volatile uint16_t sle_fragment_payload;
  volatile uint8_t sle_low_latency_status;
  volatile uint8_t sle_low_latency_rate;

  volatile uint32_t loop_stall_ms_max;
  volatile uint32_t loop_count;

  volatile uint32_t time_uart_to_sle_us;
  volatile uint32_t time_sle_tx_us;
  volatile uint32_t time_sle_to_uart_us;
  volatile uint32_t time_uart_tx_us;
  volatile uint32_t timing_sample_count;

  volatile uint32_t sle_rtt_us_min;
  volatile uint32_t sle_rtt_us_max;
  volatile uint32_t sle_rtt_us_sum;
  volatile uint32_t sle_rtt_count;

  volatile uint32_t uart_rx_timestamp_us;
  volatile uint32_t sle_rx_timestamp_us;
  volatile uint32_t sle_tx_start_us;

  volatile uint32_t frame_tx_delay_us_min;
  volatile uint32_t frame_tx_delay_us_max;
  volatile uint32_t frame_tx_delay_us_sum;
  volatile uint32_t frame_tx_delay_count;

  volatile uint32_t frame_rx_delay_us_min;
  volatile uint32_t frame_rx_delay_us_max;
  volatile uint32_t frame_rx_delay_us_sum;
  volatile uint32_t frame_rx_delay_count;

  volatile uint32_t sle_reassembly_us_min;
  volatile uint32_t sle_reassembly_us_max;
  volatile uint32_t sle_reassembly_us_sum;
  volatile uint32_t sle_reassembly_count;

  volatile uint32_t uart_queue_wait_us_min;
  volatile uint32_t uart_queue_wait_us_max;
  volatile uint32_t uart_queue_wait_us_sum;
  volatile uint32_t uart_queue_wait_count;

  volatile uint32_t uart_submit_us_min;
  volatile uint32_t uart_submit_us_max;
  volatile uint32_t uart_submit_us_sum;
  volatile uint32_t uart_submit_count;

  volatile uint32_t intra_frame_gap_us_min;
  volatile uint32_t intra_frame_gap_us_max;
  volatile uint32_t intra_frame_gap_us_sum;
  volatile uint32_t intra_frame_gap_count;

  volatile uint32_t start_tick;
} demo_stats_t;

extern demo_stats_t g_demo_stats;

#define STATS_INC(field) (g_demo_stats.field++)
#define STATS_ADD(field, n) (g_demo_stats.field += (n))
#define STATS_SET_HWM(field, val)                                              \
  do {                                                                         \
    if ((val) > g_demo_stats.field) {                                          \
      g_demo_stats.field = (val);                                              \
    }                                                                          \
  } while (0)
#define STATS_UPDATE_RANGE(min_field, max_field, sum_field, count_field,       \
                           value)                                              \
  do {                                                                         \
    uint32_t _v = (uint32_t)(value);                                           \
    if (g_demo_stats.min_field == 0 || _v < g_demo_stats.min_field) {          \
      g_demo_stats.min_field = _v;                                             \
    }                                                                          \
    if (_v > g_demo_stats.max_field) {                                         \
      g_demo_stats.max_field = _v;                                             \
    }                                                                          \
    g_demo_stats.sum_field += _v;                                              \
    g_demo_stats.count_field++;                                                \
  } while (0)

/*============================================================================
 * Role Check Macros
 *============================================================================*/
#if defined(CONFIG_DEMO_SLE_SERVER)
#define IS_SLE_SERVER   1
#define IS_SLE_CLIENT   0
#elif defined(CONFIG_DEMO_SLE_CLIENT)
#define IS_SLE_SERVER   0
#define IS_SLE_CLIENT   1
#else
#define IS_SLE_SERVER   1
#define IS_SLE_CLIENT   0
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEMO_CONFIG_H */

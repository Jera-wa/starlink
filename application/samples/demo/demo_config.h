/**
 * @file demo_config.h
 * @brief SLE UART Bridge - Configuration and Constants
 * 
 * Configurable parameters for UART-SLE-UART bridge.
 * Target: ≥5 KB/s stable lossless transmission
 */

#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * Role Configuration (Select ONE via build config)
 *============================================================================*/
// Defined via Kconfig:
// CONFIG_DEMO_SLE_SERVER - Server role (broadcast, wait for connection)
// CONFIG_DEMO_SLE_CLIENT - Client role (scan, connect)

/*============================================================================
 * UART Configuration
 *============================================================================*/
#define DEMO_UART_BUS               CONFIG_SLE_UART_BUS  // Default: 1
#define DEMO_UART_BAUDRATE          2000000              
#define DEMO_UART_TX_PIN            CONFIG_UART_TXD_PIN  // GPIO15
#define DEMO_UART_RX_PIN            CONFIG_UART_RXD_PIN  // GPIO16

/*============================================================================
 * Buffer Configuration (Critical for no-loss)
 *============================================================================*/
#define DEMO_UART_RX_BUFFER_SIZE    4096     // UART RX ring buffer
#define DEMO_UART_RX_THRESHOLD      16       // RX回调触发阈值(小值=低延迟)
#define DEMO_UART_TX_BUFFER_SIZE    4096     // UART TX ring buffer
#define DEMO_SLE_TX_RING_SIZE       (16*1024) // UART->SLE ring buffer (reduced to save memory)
#define DEMO_SLE_RX_RING_SIZE       (16*1024) // SLE->UART ring buffer (reduced to save memory)
#define DEMO_DMA_CHUNK_SIZE         2048     // Max DMA transfer per batch

// Ping-pong buffer count for DMA
#define DEMO_TX_BUFFER_COUNT        6

/*============================================================================
 * Flow Control Watermarks
 *============================================================================*/
#define DEMO_HIGH_WATERMARK_PERCENT 75  // Stop reading at 75% full
#define DEMO_LOW_WATERMARK_PERCENT  25  // Resume reading at 25% full

#define RING_HIGH_WATERMARK(size)   ((size) * DEMO_HIGH_WATERMARK_PERCENT / 100)
#define RING_LOW_WATERMARK(size)    ((size) * DEMO_LOW_WATERMARK_PERCENT / 100)

/*============================================================================
 * SLE Connection Parameters (Optimized for throughput)
 *============================================================================*/
// Connection interval: 2.5ms = 0x14 (stable value, 1.25ms caused memory issues)
// unit: 125us, so 0x14 = 20 * 125us = 2.5ms
#define DEMO_SLE_CONN_INTV_MIN      0x14
#define DEMO_SLE_CONN_INTV_MAX      0x14

// Advertising interval: 25ms = 0xC8 (unit: 125us)
#define DEMO_SLE_ADV_INTERVAL_MIN   0xC8
#define DEMO_SLE_ADV_INTERVAL_MAX   0xC8

// Supervision timeout: 5000ms = 0x1F4 (unit: 10ms)
#define DEMO_SLE_SUPERVISION_TIMEOUT 0x1F4

// MTU size for SLE transfer
#define DEMO_SLE_MTU_SIZE           520

// Advertising handle
#define DEMO_SLE_ADV_HANDLE         1

// Server name for discovery
#define DEMO_SLE_SERVER_NAME        "sle_demo_server"

// Scan parameters
#define DEMO_SLE_SEEK_INTERVAL      100
#define DEMO_SLE_SEEK_WINDOW        100

/*============================================================================
 * Protocol Frame Configuration
 *============================================================================*/
// Simple transparent mode: no application-layer framing
// SLE link layer provides reliability

// Optional: Enable sequence number for packet loss detection
#define DEMO_ENABLE_SEQ_CHECK       1

// Optional: Enable CRC16 for extra verification
#define DEMO_ENABLE_CRC16           0

/*============================================================================
 * Task Configuration
 *============================================================================*/
#define DEMO_TASK_PRIORITY          20
#define DEMO_TASK_STACK_SIZE        0x1200
#define DEMO_BRIDGE_POLL_MS         1  // Poll interval when buffers empty

/*============================================================================
 * Low Power Configuration (Reserved - currently disabled)
 *============================================================================*/
// Set to 1 to enable low power sleep when idle
#define DEMO_ENABLE_LOW_POWER       0

// Idle timeout before entering low power mode (ms)
#define DEMO_IDLE_TIMEOUT_MS        100

// Low power mode: 0=light sleep (fast wake), 1=deep sleep (slow wake)
#define DEMO_LOW_POWER_MODE         0

/*============================================================================
 * Debug Configuration
 *============================================================================*/
#define DEMO_DEBUG_LOG              1  // Set to 1 to enable verbose logging
#define DEMO_STATS_INTERVAL_MS      10000  // Print stats every 10s

#if DEMO_DEBUG_LOG
#define DEMO_LOG(fmt, ...) osal_printk("[DEMO] " fmt "\r\n", ##__VA_ARGS__)
#else
#define DEMO_LOG(fmt, ...)
#endif

#define DEMO_INFO(fmt, ...) osal_printk("[DEMO] " fmt "\r\n", ##__VA_ARGS__)
#define DEMO_ERR(fmt, ...)  osal_printk("[DEMO ERR] " fmt "\r\n", ##__VA_ARGS__)

/*============================================================================
 * Statistics Structure
 *============================================================================*/
typedef struct {
    // Byte counters
    volatile uint32_t uart_rx_bytes;
    volatile uint32_t uart_tx_bytes;
    volatile uint32_t sle_rx_bytes;
    volatile uint32_t sle_tx_bytes;
    
    // Frame counters
    volatile uint32_t sle_tx_frames;
    volatile uint32_t sle_rx_frames;
    
    // [P1 FIX] Drop counters (critical for debugging)
    volatile uint32_t uart_rx_drop_bytes;   // UART RX ring overflow
    volatile uint32_t sle_rx_drop_bytes;    // SLE RX ring overflow
    volatile uint32_t ring_overflow;        // Legacy: total overflow events
    
    // Error counters
    volatile uint32_t sle_tx_fail;          // SLE send failures
    volatile uint32_t sle_tx_busy_cnt;      // SLE busy (congestion indicator)
    
    // [P1 FIX] Per-direction ring high water marks
    volatile uint16_t uart_rx_ring_hwm;     // UART RX -> SLE TX direction
    volatile uint16_t uart_tx_ring_hwm;     // SLE RX -> UART TX direction
    volatile uint16_t sle_tx_ring_hwm;      // Alias for uart_rx_ring_hwm
    volatile uint16_t sle_rx_ring_hwm;      // Alias for uart_tx_ring_hwm
    
    // [P2 FIX] Loop performance monitoring
    volatile uint32_t loop_stall_ms_max;    // Max single loop iteration time
    volatile uint32_t loop_count;           // Total loop iterations

    // [TIMING] Per-stage timing statistics (microseconds)
    volatile uint32_t time_uart_to_sle_us;      // bridge_uart_to_sle() cumulative time
    volatile uint32_t time_sle_tx_us;           // demo_sle_tx_process() cumulative time
    volatile uint32_t time_sle_to_uart_us;      // bridge_sle_to_uart() cumulative time
    volatile uint32_t time_uart_tx_us;          // demo_uart_tx_process() cumulative time
    volatile uint32_t timing_sample_count;      // Number of timing samples

    // [LATENCY] SLE round-trip latency (write_req -> write_cfm)
    volatile uint32_t sle_rtt_us_min;           // Minimum RTT in microseconds
    volatile uint32_t sle_rtt_us_max;           // Maximum RTT in microseconds
    volatile uint32_t sle_rtt_us_sum;           // Sum for average calculation
    volatile uint32_t sle_rtt_count;            // Number of RTT samples

    // [LATENCY] Detailed timing breakdown
    volatile uint32_t uart_rx_timestamp_us;     // When UART RX received data
    volatile uint32_t sle_tx_start_us;          // When SLE TX started sending

    // Timing
    volatile uint32_t start_tick;
} demo_stats_t;

extern demo_stats_t g_demo_stats;

// Stats macros
#define STATS_INC(field)        (g_demo_stats.field++)
#define STATS_ADD(field, n)     (g_demo_stats.field += (n))
#define STATS_SET_HWM(field, val) do { \
    if ((val) > g_demo_stats.field) g_demo_stats.field = (val); \
} while(0)

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
// Default to server if not specified
#define IS_SLE_SERVER   1
#define IS_SLE_CLIENT   0
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEMO_CONFIG_H */

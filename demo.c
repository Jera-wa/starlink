/**
 * @file demo.c
 * @brief SLE UART Bridge Main Application
 * 
 * Entry point and main bridge task for UART <-> SLE data transfer.
 * Target: ≥5 KB/s stable lossless transmission
 */

#include "demo_config.h"
#include "demo_uart.h"
#include "demo_sle.h"
#include "ring_buffer.h"
#include "soc_osal.h"
#include "app_init.h"
#include "driver/systick.h"

/*============================================================================
 * Global Statistics
 *============================================================================*/

demo_stats_t g_demo_stats = {0};

/*============================================================================
 * Bridge Logic
 *============================================================================*/

/**
 * @brief Transfer data from UART RX to SLE TX
 * 
 * [P0 FIX] Continue buffering even when disconnected to extend
 * the grace period before drops. This doubles effective buffer
 * capacity during disconnect events.
 */
static uint32_t bridge_uart_to_sle(void)
{
    ring_buffer_t *uart_rx = demo_uart_get_rx_ring();
    ring_buffer_t *sle_tx = demo_sle_get_tx_ring();
    
    // [P0 FIX] Don't check connection here - buffer data regardless of connection state
    // This allows us to accumulate data during brief disconnects (up to 2x buffer capacity)
    // The data will be sent once reconnected via demo_sle_tx_process()
    
    // Flow control: check SLE TX buffer level
    uint32_t sle_tx_level = ring_data_len(sle_tx);
    if (sle_tx_level > RING_HIGH_WATERMARK(DEMO_SLE_TX_RING_SIZE)) {
        // SLE TX buffer almost full, pause reading from UART RX ring
        // Note: UART ISR will still write to uart_rx ring, but we stop consuming it
        return 0;
    }
    
    uint32_t total = 0;
    
    // Transfer available data
    while (!ring_is_empty(uart_rx) && total < 4096) {
        uint8_t *ptr;
        uint32_t len;
        
        ring_peek_contiguous(uart_rx, &ptr, &len);
        if (len == 0) break;
        
        // Check destination space
        uint32_t free = ring_free_len(sle_tx);
        if (free == 0) break;
        
        if (len > free) len = free;
        if (len > 1024) len = 1024;  // Chunk size
        
        uint32_t written = ring_write(sle_tx, ptr, len);
        ring_consume(uart_rx, written);
        total += written;
    }
    
    // Update HWM for SLE TX ring
    STATS_SET_HWM(sle_tx_ring_hwm, (uint16_t)ring_data_len(sle_tx));
    
    return total;
}

/**
 * @brief Transfer data from SLE RX to UART TX
 */
static uint32_t bridge_sle_to_uart(void)
{
    ring_buffer_t *sle_rx = demo_sle_get_rx_ring();
    ring_buffer_t *uart_tx = demo_uart_get_tx_ring();
    
    // Flow control: check UART TX buffer level
    uint32_t uart_tx_level = ring_data_len(uart_tx);
    if (uart_tx_level > RING_HIGH_WATERMARK(DEMO_SLE_RX_RING_SIZE)) {
        return 0;
    }
    
    uint32_t total = 0;
    
    while (!ring_is_empty(sle_rx) && total < 4096) {
        uint8_t *ptr;
        uint32_t len;
        
        ring_peek_contiguous(sle_rx, &ptr, &len);
        if (len == 0) break;
        
        uint32_t free = ring_free_len(uart_tx);
        if (free == 0) break;
        
        if (len > free) len = free;
        if (len > 1024) len = 1024;
        
        uint32_t written = ring_write(uart_tx, ptr, len);
        ring_consume(sle_rx, written);
        total += written;
    }
    
    return total;
}

/**
 * @brief Print statistics periodically
 */
static void print_stats(void)
{
    static uint32_t last_print = 0;
    uint32_t now = (uint32_t)uapi_systick_get_ms();
    
    if (now - last_print < DEMO_STATS_INTERVAL_MS) {
        return;
    }
    last_print = now;
    
    uint32_t elapsed_s = (now - g_demo_stats.start_tick) / 1000;
    if (elapsed_s == 0) elapsed_s = 1;
    
    uint32_t uart_rx_kbps = g_demo_stats.uart_rx_bytes / elapsed_s / 1024;
    uint32_t sle_tx_kbps = g_demo_stats.sle_tx_bytes / elapsed_s / 1024;
    
    // [P2 FIX] Enhanced stats output with drop counters and per-direction HWM
    DEMO_INFO("[STATS %ds] UART: RX=%dKB TX=%dKB | SLE: RX=%dKB TX=%dKB",
              elapsed_s,
              g_demo_stats.uart_rx_bytes ,
              g_demo_stats.uart_tx_bytes ,
              g_demo_stats.sle_rx_bytes ,
              g_demo_stats.sle_tx_bytes );
    
    DEMO_INFO("[STATS] Frames: TX=%d RX=%d | Fail=%d | HWM: UART_RX=%d SLE_RX=%d",
              g_demo_stats.sle_tx_frames,
              g_demo_stats.sle_rx_frames,
              g_demo_stats.sle_tx_fail,
              g_demo_stats.uart_rx_ring_hwm,
              g_demo_stats.sle_rx_ring_hwm);
    
    // [P1 FIX] Critical: show drop counters
    if (g_demo_stats.uart_rx_drop_bytes > 0 || g_demo_stats.sle_rx_drop_bytes > 0) {
        DEMO_ERR("[DROPS] UART_RX_drop=%d SLE_RX_drop=%d (DATA LOSS!)",
                 g_demo_stats.uart_rx_drop_bytes,
                 g_demo_stats.sle_rx_drop_bytes);
    }
    
    // [P2 FIX] Show loop stall for performance debugging
    if (g_demo_stats.loop_stall_ms_max > 10) {
        DEMO_INFO("[PERF] Loop stall max: %dms (loops=%d)",
                  g_demo_stats.loop_stall_ms_max,
                  g_demo_stats.loop_count);
    }
    
    DEMO_INFO("[RATE] UART RX: %d KB/s | SLE TX: %d KB/s", uart_rx_kbps, sle_tx_kbps);
}

/*============================================================================
 * Main Task
 *============================================================================*/

static void *demo_bridge_task(const char *arg)
{
    (void)arg;
    
    DEMO_INFO("========================================");
    DEMO_INFO("SLE UART Bridge Starting...");
#if IS_SLE_SERVER
    DEMO_INFO("Role: SERVER");
#else
    DEMO_INFO("Role: CLIENT");
#endif
    DEMO_INFO("UART: %d @ %d baud", DEMO_UART_BUS, DEMO_UART_BAUDRATE);
    DEMO_INFO("Buffer: TX=%dKB RX=%dKB", 
              DEMO_SLE_TX_RING_SIZE/1024, DEMO_SLE_RX_RING_SIZE/1024);
    DEMO_INFO("========================================");
    
    // Initialize UART
    if (demo_uart_init() != 0) {
        DEMO_ERR("UART init failed!");
        return NULL;
    }
    
    // Initialize SLE
    if (demo_sle_init() != 0) {
        DEMO_ERR("SLE init failed!");
        return NULL;
    }
    
    // Record start time
    g_demo_stats.start_tick = (uint32_t)uapi_systick_get_ms();
    
    DEMO_INFO("Bridge running, waiting for connection...");
    
    // Main loop
    while (1) {
        uint32_t loop_start = (uint32_t)uapi_systick_get_ms();
        uint32_t work_done = 0;
        
        // 1. Bridge UART RX -> SLE TX
        work_done += bridge_uart_to_sle();
        
        // 2. Process SLE TX (send pending data)
        work_done += demo_sle_tx_process();
        
        // 3. Bridge SLE RX -> UART TX
        work_done += bridge_sle_to_uart();
        
        // 4. Process UART TX (DMA send) - [P1 FIX] Now throttled internally
        work_done += demo_uart_tx_process();
        
        // 5. [P2 FIX] Track loop timing for stall detection
        uint32_t loop_elapsed = (uint32_t)uapi_systick_get_ms() - loop_start;
        if (loop_elapsed > g_demo_stats.loop_stall_ms_max) {
            g_demo_stats.loop_stall_ms_max = loop_elapsed;
        }
        g_demo_stats.loop_count++;
        
        // 6. Print statistics
        print_stats();
        
        // 7. Yield CPU or enter low power mode
        if (work_done == 0) {
            // No data transferred, sleep briefly
            osal_msleep(DEMO_BRIDGE_POLL_MS);
            
#if DEMO_ENABLE_LOW_POWER
            /*
             * [LOW POWER MODE - Reserved Feature]
             * When enabled, enters low power sleep after idle timeout.
             * Uncomment and configure in demo_config.h:
             *   - DEMO_ENABLE_LOW_POWER = 1
             *   - DEMO_IDLE_TIMEOUT_MS = timeout before sleep
             *   - DEMO_LOW_POWER_MODE = 0 (light) or 1 (deep)
             */
            static uint32_t idle_start = 0;
            uint32_t now = (uint32_t)uapi_systick_get_ms();
            
            if (idle_start == 0) {
                idle_start = now;
            } else if (now - idle_start > DEMO_IDLE_TIMEOUT_MS) {
                // Enter low power mode
                DEMO_INFO("Entering low power mode...");
                
                // TODO: Call SDK low power API based on DEMO_LOW_POWER_MODE
                // #include "lpm.h"
                // if (DEMO_LOW_POWER_MODE == 0) {
                //     uapi_lpm_set_sleep_mode(LPM_SLEEP_LIGHT);
                // } else {
                //     uapi_lpm_set_sleep_mode(LPM_SLEEP_DEEP);
                // }
                // uapi_lpm_enter_sleep();
                
                idle_start = 0;  // Reset after wake
            }
#endif
        } else {
            // Work done, reset idle timer
#if DEMO_ENABLE_LOW_POWER
            // idle_start = 0;  // Reset idle counter when active
#endif
        }
        // If work was done, continue immediately (tight loop for throughput)
    }
    
    return NULL;
}

/*============================================================================
 * Entry Point
 *============================================================================*/

static void demo_entry(void)
{
    osal_task *task_handle = NULL;
    
    osal_kthread_lock();
    
    task_handle = osal_kthread_create(
        (osal_kthread_handler)demo_bridge_task,
        0,
        "SLEBridgeTask",
        DEMO_TASK_STACK_SIZE
    );
    
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, DEMO_TASK_PRIORITY);
        DEMO_INFO("Bridge task created");
    } else {
        DEMO_ERR("Failed to create bridge task!");
    }
    
    osal_kthread_unlock();
}

/* Auto-run entry */
app_run(demo_entry);

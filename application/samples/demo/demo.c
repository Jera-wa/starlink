/**
 * @file demo.c
 * @brief SLE UART bridge main application.
 */

#include "demo_config.h"
#include "demo_uart.h"
#include "demo_sle.h"
#include "soc_osal.h"
#include "app_init.h"
#include "driver/systick.h"

demo_stats_t g_demo_stats = {0};
osal_event g_bridge_event;

static void print_stats(void)
{
    static uint32_t last_print = 0;
    uint32_t now = (uint32_t)uapi_systick_get_ms();
    uint32_t elapsed_s;

    if (now - last_print < DEMO_STATS_INTERVAL_MS) {
        return;
    }
    last_print = now;

    elapsed_s = (now - g_demo_stats.start_tick) / 1000U;
    if (elapsed_s == 0U) {
        elapsed_s = 1U;
    }

    DEMO_INFO("[STATS %us] UART RX=%uB TX=%uB | SLE RX=%uB TX=%uB",
        elapsed_s, g_demo_stats.uart_rx_bytes, g_demo_stats.uart_tx_bytes,
        g_demo_stats.sle_rx_bytes, g_demo_stats.sle_tx_bytes);
    DEMO_INFO("[FRAMES] in=%u out=%u drop=%u/%u | SLE tx=%u rx=%u",
        g_demo_stats.uart_rx_frames, g_demo_stats.uart_tx_frames,
        g_demo_stats.uart_rx_drop_frames, g_demo_stats.uart_tx_drop_frames,
        g_demo_stats.sle_tx_frames, g_demo_stats.sle_rx_frames);
    DEMO_INFO("[TRANSPORT] nego=0x%x ll=%u rate=%u | hwm=%u/%u invalid=%u",
        g_demo_stats.sle_negotiated_conn_interval,
        g_demo_stats.sle_low_latency_status,
        g_demo_stats.sle_low_latency_rate,
        g_demo_stats.uart_rx_ring_hwm,
        g_demo_stats.uart_tx_ring_hwm,
        g_demo_stats.uart_rx_invalid_bytes);
}

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
    DEMO_INFO("Frame queue depth=%d max_frame=%d", DEMO_FRAME_QUEUE_DEPTH, DEMO_LOGICAL_FRAME_MAX_SIZE);
    DEMO_INFO("========================================");

    if (osal_event_init(&g_bridge_event) != OSAL_SUCCESS) {
        DEMO_ERR("Event init failed!");
        return NULL;
    }

    DEMO_INFO("Bridge step: UART init begin");
    if (demo_uart_init() != 0) {
        DEMO_ERR("UART init failed!");
        return NULL;
    }

    DEMO_INFO("Bridge step: SLE init begin");
    if (demo_sle_init() != 0) {
        DEMO_ERR("SLE init failed!");
        return NULL;
    }
    DEMO_INFO("Bridge step: UART RX start begin");
    if (demo_uart_start_client_rx() != 0) {
        DEMO_ERR("UART RX start failed after SLE init!");
        demo_uart_deinit();
        return NULL;
    }

    g_demo_stats.start_tick = (uint32_t)uapi_systick_get_ms();
    DEMO_INFO("Frame-aware bridge running, waiting for connection...");

    uint32_t work_done = 0;
    while (1) {
        (void)osal_event_read(&g_bridge_event, DEMO_EVENT_ALL,
            work_done > 0U ? 0U : DEMO_EVENT_TIMEOUT_MS,
            OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);

        work_done = 0;
        work_done += demo_uart_rx_poll();
        work_done += demo_uart_rx_process_raw_chunks();
        work_done += demo_sle_tx_process();
        work_done += demo_uart_rx_poll();
        work_done += demo_uart_tx_process();
        g_demo_stats.loop_count++;

        print_stats();

        if (work_done > 0U) {
            continue;
        }
    }
}

static void demo_entry(void)
{
    osal_task *task_handle = NULL;

    osal_kthread_lock();
    task_handle = osal_kthread_create((osal_kthread_handler)demo_bridge_task, 0,
        "SLEBridgeTask", DEMO_TASK_STACK_SIZE);
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, DEMO_TASK_PRIORITY);
        DEMO_INFO("Bridge task created");
    } else {
        DEMO_ERR("Failed to create bridge task!");
    }
    osal_kthread_unlock();
}

app_run(demo_entry);

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
    uint32_t avg_tx_delay = 0;
    uint32_t avg_rx_delay = 0;
    uint32_t avg_reassembly = 0;
    uint32_t avg_uart_wait = 0;
    uint32_t avg_uart_submit = 0;
    uint32_t avg_gap = 0;

    if (now - last_print < DEMO_STATS_INTERVAL_MS) {
        return;
    }
    last_print = now;

    elapsed_s = (now - g_demo_stats.start_tick) / 1000U;
    if (elapsed_s == 0U) {
        elapsed_s = 1U;
    }

    if (g_demo_stats.frame_tx_delay_count > 0U) {
        avg_tx_delay = g_demo_stats.frame_tx_delay_us_sum / g_demo_stats.frame_tx_delay_count;
    }
    if (g_demo_stats.frame_rx_delay_count > 0U) {
        avg_rx_delay = g_demo_stats.frame_rx_delay_us_sum / g_demo_stats.frame_rx_delay_count;
    }
    if (g_demo_stats.sle_reassembly_count > 0U) {
        avg_reassembly = g_demo_stats.sle_reassembly_us_sum / g_demo_stats.sle_reassembly_count;
    }
    if (g_demo_stats.uart_queue_wait_count > 0U) {
        avg_uart_wait = g_demo_stats.uart_queue_wait_us_sum / g_demo_stats.uart_queue_wait_count;
    }
    if (g_demo_stats.uart_submit_count > 0U) {
        avg_uart_submit = g_demo_stats.uart_submit_us_sum / g_demo_stats.uart_submit_count;
    }
    if (g_demo_stats.intra_frame_gap_count > 0U) {
        avg_gap = g_demo_stats.intra_frame_gap_us_sum / g_demo_stats.intra_frame_gap_count;
    }

    DEMO_INFO("[STATS %us] UART RX=%uB TX=%uB | SLE RX=%uB TX=%uB",
        elapsed_s, g_demo_stats.uart_rx_bytes, g_demo_stats.uart_tx_bytes,
        g_demo_stats.sle_rx_bytes, g_demo_stats.sle_tx_bytes);
    DEMO_INFO("[FRAMES] UART in=%u out=%u drop_in=%u drop_out=%u | SLE tx=%u rx=%u frags tx=%u rx=%u",
        g_demo_stats.uart_rx_frames, g_demo_stats.uart_tx_frames,
        g_demo_stats.uart_rx_drop_frames, g_demo_stats.uart_tx_drop_frames,
        g_demo_stats.sle_tx_frames, g_demo_stats.sle_rx_frames,
        g_demo_stats.sle_tx_fragments, g_demo_stats.sle_rx_fragments);
    DEMO_INFO("[FAST PATH] uart_tx hit=%u miss=%u",
        g_demo_stats.uart_tx_fast_path_hits, g_demo_stats.uart_tx_fast_path_misses);
    DEMO_INFO("[RELIABLE] ack_mode=%u ack_tx=%u ack_rx=%u retry=%u hard_fail=%u dup=%u ind_to=%u frame_ack_to=%u",
        DEMO_SLE_FRAME_ACK_ENABLE,
        g_demo_stats.sle_ack_sent, g_demo_stats.sle_ack_received,
        g_demo_stats.sle_tx_retry_frames, g_demo_stats.sle_tx_hard_fail,
        g_demo_stats.sle_rx_duplicate_frames, g_demo_stats.sle_indicate_timeout_cnt,
        g_demo_stats.sle_frame_ack_timeout_cnt);
    DEMO_INFO("[TRANSPORT] req_int=0x%x-0x%x nego=0x%x mtu=%u eff=%u frag=%u data_len=%u ll=%u rate=%u",
        g_demo_stats.sle_requested_conn_interval_min,
        g_demo_stats.sle_requested_conn_interval_max,
        g_demo_stats.sle_negotiated_conn_interval,
        demo_sle_get_mtu(),
        g_demo_stats.sle_effective_payload,
        g_demo_stats.sle_fragment_payload,
        g_demo_stats.sle_requested_data_len,
        g_demo_stats.sle_low_latency_status,
        g_demo_stats.sle_low_latency_rate);
    DEMO_INFO("[QUEUES] uart_rx_hwm=%u uart_tx_hwm=%u oversize=%u invalid=%u reassembly_reset=%u reassembly_drop=%u",
        g_demo_stats.uart_rx_ring_hwm,
        g_demo_stats.uart_tx_ring_hwm,
        g_demo_stats.uart_rx_oversize_frames,
        g_demo_stats.uart_rx_invalid_bytes,
        g_demo_stats.sle_reassembly_reset_cnt,
        g_demo_stats.sle_reassembly_drop_frames);
    DEMO_INFO("[DELAY] tx_queue min=%uus avg=%uus max=%uus | sle_to_uart min=%uus avg=%uus max=%uus | gap min=%uus avg=%uus max=%uus",
        g_demo_stats.frame_tx_delay_us_min, avg_tx_delay, g_demo_stats.frame_tx_delay_us_max,
        g_demo_stats.frame_rx_delay_us_min, avg_rx_delay, g_demo_stats.frame_rx_delay_us_max,
        g_demo_stats.intra_frame_gap_us_min, avg_gap, g_demo_stats.intra_frame_gap_us_max);
    DEMO_INFO("[RX PATH] reassembly min=%uus avg=%uus max=%uus | uart_wait min=%uus avg=%uus max=%uus | uart_submit min=%uus avg=%uus max=%uus",
        g_demo_stats.sle_reassembly_us_min, avg_reassembly, g_demo_stats.sle_reassembly_us_max,
        g_demo_stats.uart_queue_wait_us_min, avg_uart_wait, g_demo_stats.uart_queue_wait_us_max,
        g_demo_stats.uart_submit_us_min, avg_uart_submit, g_demo_stats.uart_submit_us_max);
#if IS_SLE_CLIENT
    {
        demo_uart_rx_diag_t uart_diag = {0};

        demo_uart_get_rx_diag(&uart_diag);
        DEMO_INFO("[UART DMA IDLE] idle_isr=%u raw_cb=%u bytes=%u last=%u | pub=%u bytes=%u idle_cb=%u soft=%u idle_fb=%u forced=%u | last(reason=%u xfer=%u rem=%u blk=%u rx=%u tail=%u combined=%u fifo_empty=%u)",
            uart_diag.idle_isr_count, uart_diag.raw_callback_count, uart_diag.raw_callback_bytes,
            uart_diag.raw_callback_last_len, uart_diag.publish_count, uart_diag.publish_bytes,
            uart_diag.publish_from_idle_cb, uart_diag.publish_from_soft_flush,
            uart_diag.publish_from_idle_fallback, uart_diag.forced_idle_request_count,
            uart_diag.last_publish_reason, uart_diag.last_transfer_num, uart_diag.last_remaining,
            uart_diag.last_received_blocks, uart_diag.last_received_len, uart_diag.last_idle_tail_len,
            uart_diag.last_combined_len, uart_diag.last_rx_fifo_empty);
    }
#endif
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

    g_demo_stats.start_tick = (uint32_t)uapi_systick_get_ms();
    DEMO_INFO("Frame-aware bridge running, waiting for connection...");

    while (1) {
        uint32_t loop_start;
        uint32_t work_done = 0;
        uint32_t t0;
        uint32_t t1;

        (void)osal_event_read(&g_bridge_event, DEMO_EVENT_ALL,
            DEMO_EVENT_TIMEOUT_MS, OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);

        demo_uart_rx_poll();
        loop_start = (uint32_t)uapi_systick_get_ms();

        t0 = (uint32_t)uapi_systick_get_us();
        work_done += demo_sle_tx_process();
        t1 = (uint32_t)uapi_systick_get_us();
        g_demo_stats.time_sle_tx_us += (t1 - t0);

        demo_uart_rx_poll();

        t0 = (uint32_t)uapi_systick_get_us();
        demo_sle_process_deferred();
        t1 = (uint32_t)uapi_systick_get_us();
        g_demo_stats.time_sle_to_uart_us += (t1 - t0);

        demo_uart_rx_poll();

        t0 = (uint32_t)uapi_systick_get_us();
        work_done += demo_uart_tx_process();
        t1 = (uint32_t)uapi_systick_get_us();
        g_demo_stats.time_uart_tx_us += (t1 - t0);

        g_demo_stats.timing_sample_count++;
        if ((uint32_t)uapi_systick_get_ms() - loop_start > g_demo_stats.loop_stall_ms_max) {
            g_demo_stats.loop_stall_ms_max = (uint32_t)uapi_systick_get_ms() - loop_start;
        }
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

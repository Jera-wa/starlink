/**
 * @file demo_uart.c
 * @brief UART DMA TX and logical-frame RX implementation.
 */

#include "demo_uart.h"
#include "demo_config.h"
#include "securec.h"
#include "soc_osal.h"
#include "pinctrl.h"
#include "uart.h"
#include "dma.h"
#include "hal_dma.h"
#include "driver/systick.h"

typedef struct {
    uint8_t buffer[DEMO_LOGICAL_FRAME_MAX_SIZE];
    uint16_t used;
    uint16_t expected_len;
    uint32_t first_byte_timestamp_us;
    uint32_t last_byte_timestamp_us;
    uint32_t raw_input_bytes;
    uint32_t dropped_bytes;
} demo_uart_frame_parser_t;

static demo_frame_queue_t s_uart_rx_frames;
static demo_frame_queue_t s_uart_tx_frames;
static demo_uart_frame_parser_t s_uart_rx_parser;

static uint8_t s_dma_tx_buf[DEMO_TX_BUFFER_COUNT][DEMO_DMA_CHUNK_SIZE];
static volatile uint8_t s_dma_tx_idx = 0;
static volatile bool s_dma_tx_busy = false;
static uint8_t s_uart_rx_buf[DEMO_UART_RX_BUFFER_SIZE];
static uint16_t s_next_uart_frame_id = 1;
static osal_mutex s_uart_tx_lock;
static volatile uint32_t s_uart_rx_raw_cb_count = 0;
static volatile uint32_t s_uart_rx_raw_cb_bytes = 0;
static volatile uint32_t s_uart_rx_raw_cb_last_len = 0;
static volatile uint32_t s_uart_rx_overrun_err_count = 0;
static volatile uint32_t s_uart_rx_frame_err_count = 0;
static volatile uint32_t s_uart_rx_parity_err_count = 0;

static uart_write_dma_config_t s_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_4,
    .priority = HAL_DMA_CH_PRIORITY_0
};

static uart_write_dma_config_t s_rx_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_1,
    .priority = HAL_DMA_CH_PRIORITY_0
};

static uart_buffer_config_t s_uart_buf_cfg = {
    .rx_buffer = s_uart_rx_buf,
    .rx_buffer_size = DEMO_UART_RX_BUFFER_SIZE
};

static uint16_t demo_uart_next_frame_id(void)
{
    uint16_t frame_id = s_next_uart_frame_id++;
    if (frame_id == 0) {
        frame_id = s_next_uart_frame_id++;
    }
    return frame_id;
}

static void demo_uart_queue_rx_frame(const uint8_t *data, uint16_t len,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us)
{
    uint16_t frame_id;
    uint8_t queued;
    uint8_t queue_count;
    uint32_t irq_sts;

    if (data == NULL || len == 0U || len > DEMO_LOGICAL_FRAME_MAX_SIZE) {
        return;
    }

    frame_id = demo_uart_next_frame_id();
    irq_sts = osal_irq_lock();
    queued = demo_frame_queue_push(&s_uart_rx_frames, data, len, frame_id,
        enqueue_timestamp_us, ready_timestamp_us, 0U);
    queue_count = demo_frame_queue_count(&s_uart_rx_frames);
    osal_irq_restore(irq_sts);

    if (!queued) {
        STATS_INC(uart_rx_drop_frames);
        STATS_ADD(uart_rx_drop_bytes, len);
        DEMO_ERR("UART RX frame queue full: id=%u len=%u", frame_id, len);
        return;
    }

    STATS_INC(uart_rx_frames);
    STATS_SET_HWM(uart_rx_ring_hwm, queue_count);
    DEMO_LOG("UART RX frame queued: id=%u len=%u q=%u", frame_id, len, queue_count);
    osal_event_write(&g_bridge_event, DEMO_EVENT_UART_RX);
}

static void demo_uart_parser_reset(demo_uart_frame_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->used = 0;
    parser->expected_len = 0;
    parser->first_byte_timestamp_us = 0;
    parser->last_byte_timestamp_us = 0;
    parser->raw_input_bytes = 0;
    parser->dropped_bytes = 0;
}

static void demo_uart_parser_drop_prefix(demo_uart_frame_parser_t *parser, uint16_t count, bool count_as_drop)
{
    if (parser == NULL || count == 0) {
        return;
    }

    if (count_as_drop) {
        parser->dropped_bytes += count;
    }

    if (count >= parser->used) {
        demo_uart_parser_reset(parser);
        return;
    }

    if (memmove_s(parser->buffer, sizeof(parser->buffer), parser->buffer + count,
        parser->used - count) != EOK) {
        demo_uart_parser_reset(parser);
        return;
    }

    parser->used = (uint16_t)(parser->used - count);
    if (parser->used < DEMO_LOGICAL_FRAME_HEADER_SIZE) {
        parser->expected_len = 0;
    }
}

static void demo_uart_parser_drop_stale_frame(demo_uart_frame_parser_t *parser, uint32_t timestamp_us)
{
    uint32_t gap_us;

    if (parser == NULL || parser->used == 0 || parser->last_byte_timestamp_us == 0U ||
        timestamp_us < parser->last_byte_timestamp_us) {
        return;
    }

    gap_us = timestamp_us - parser->last_byte_timestamp_us;
    if (gap_us <= DEMO_UART_FRAME_GAP_TIMEOUT_US) {
        return;
    }

    STATS_INC(uart_rx_drop_frames);
    STATS_ADD(uart_rx_drop_bytes, parser->used);

    /* Skip error logging for tiny residuals (≤2 bytes) — likely sender-side alignment */
    if (parser->used <= 2U) {
        demo_uart_parser_reset(parser);
        return;
    }

#if IS_SLE_CLIENT
    {
        uart_dma_idle_diag_t diag = {0};
        uint16_t deficit = 0;

        if (parser->expected_len > parser->used) {
            deficit = (uint16_t)(parser->expected_len - parser->used);
        }
        if (uapi_uart_dma_idle_get_diag(DEMO_UART_BUS, &diag) == ERRCODE_SUCC) {
            DEMO_ERR("UART RX timeout diag: deficit=%u raw_cb=%u raw_cb_bytes=%u last_raw=%u "
                "pub=%u bytes=%u idle_cb=%u soft=%u idle_fb=%u forced=%u "
                "last_reason=%u xfer=%u rem=%u blocks=%u rx=%u tail=%u combined=%u fifo_empty=%u "
                "overrun=%u frame_err=%u parity_err=%u fifo_drain=%u",
                deficit, s_uart_rx_raw_cb_count, s_uart_rx_raw_cb_bytes, s_uart_rx_raw_cb_last_len,
                diag.publish_count, diag.publish_bytes, diag.publish_from_idle_cb,
                diag.publish_from_soft_flush, diag.publish_from_idle_fallback,
                diag.forced_idle_request_count, diag.last_publish_reason, diag.last_transfer_num,
                diag.last_remaining, diag.last_received_blocks, diag.last_received_len,
                diag.last_idle_tail_len, diag.last_combined_len, diag.last_rx_fifo_empty,
                s_uart_rx_overrun_err_count, s_uart_rx_frame_err_count, s_uart_rx_parity_err_count,
                diag.last_fifo_drain_len);
        }
    }
#endif
    DEMO_ERR("UART RX partial frame timeout: used=%u expected=%u raw_in=%u dropped=%u gap=%u us",
        parser->used, parser->expected_len, parser->raw_input_bytes, parser->dropped_bytes, gap_us);
    demo_uart_parser_reset(parser);
}

static void demo_uart_queue_ingress_frame(demo_uart_frame_parser_t *parser, uint32_t ready_timestamp_us)
{
    if (parser == NULL || parser->expected_len == 0 || parser->used < parser->expected_len) {
        return;
    }

    demo_uart_queue_rx_frame(parser->buffer, parser->expected_len,
        parser->first_byte_timestamp_us, ready_timestamp_us);

    demo_uart_parser_drop_prefix(parser, parser->expected_len, false);
    parser->expected_len = 0;
}

static void demo_uart_parser_process(demo_uart_frame_parser_t *parser, uint32_t timestamp_us)
{
    uint16_t expected_len;

    if (parser == NULL) {
        return;
    }

    while (parser->used > 0) {
        if (parser->used >= 2 && !demo_logical_frame_header_valid(parser->buffer)) {
            STATS_INC(uart_rx_invalid_bytes);
            demo_uart_parser_drop_prefix(parser, 1, true);
            continue;
        }

        if (parser->used < DEMO_LOGICAL_FRAME_HEADER_SIZE) {
            break;
        }

        expected_len = demo_logical_frame_total_len(parser->buffer);
        if (expected_len < DEMO_LOGICAL_FRAME_HEADER_SIZE ||
            expected_len > DEMO_LOGICAL_FRAME_MAX_SIZE) {
            STATS_INC(uart_rx_oversize_frames);
            STATS_INC(uart_rx_invalid_bytes);
            DEMO_ERR("UART RX frame length invalid: total=%u", expected_len);
            demo_uart_parser_drop_prefix(parser, 1, true);
            continue;
        }

        parser->expected_len = expected_len;
        if (parser->used < expected_len) {
            break;
        }

        demo_uart_queue_ingress_frame(parser, timestamp_us);
        if (parser->used > 0) {
            parser->first_byte_timestamp_us = timestamp_us;
        }
    }
}

static void demo_uart_parser_feed(const uint8_t *buffer, uint16_t length, uint32_t timestamp_us)
{
    uint16_t copy_len;
    uint16_t space_left;

    demo_uart_parser_drop_stale_frame(&s_uart_rx_parser, timestamp_us);

    while (length > 0) {
        if (s_uart_rx_parser.used == 0) {
            s_uart_rx_parser.first_byte_timestamp_us = timestamp_us;
            s_uart_rx_parser.raw_input_bytes = 0;
            s_uart_rx_parser.dropped_bytes = 0;
        }

        space_left = (uint16_t)(DEMO_LOGICAL_FRAME_MAX_SIZE - s_uart_rx_parser.used);
        if (space_left == 0) {
            STATS_INC(uart_rx_oversize_frames);
            STATS_ADD(uart_rx_drop_bytes, length);
            DEMO_ERR("UART RX parser overflow, resetting state");
            demo_uart_parser_reset(&s_uart_rx_parser);
            return;
        }

        copy_len = (length < space_left) ? length : space_left;
        if (memcpy_s(s_uart_rx_parser.buffer + s_uart_rx_parser.used, space_left, buffer,
            copy_len) != EOK) {
            return;
        }

        s_uart_rx_parser.used = (uint16_t)(s_uart_rx_parser.used + copy_len);
        s_uart_rx_parser.last_byte_timestamp_us = timestamp_us;
        s_uart_rx_parser.raw_input_bytes += copy_len;
        buffer += copy_len;
        length = (uint16_t)(length - copy_len);

        demo_uart_parser_process(&s_uart_rx_parser, timestamp_us);
        if (s_uart_rx_parser.used > 0 && s_uart_rx_parser.first_byte_timestamp_us == 0) {
            s_uart_rx_parser.first_byte_timestamp_us = timestamp_us;
        }
        if (s_uart_rx_parser.used > 0 && s_uart_rx_parser.last_byte_timestamp_us == 0) {
            s_uart_rx_parser.last_byte_timestamp_us = timestamp_us;
        }
    }
}

static void uart_rx_callback(const void *buffer, uint16_t length, bool error)
{
    uint32_t rx_time_us;

    if (error || length == 0 || buffer == NULL) {
        return;
    }

    rx_time_us = (uint32_t)uapi_systick_get_us();
    g_demo_stats.uart_rx_timestamp_us = rx_time_us;
    STATS_ADD(uart_rx_bytes, length);
    demo_uart_parser_feed((const uint8_t *)buffer, length, rx_time_us);
}

#if IS_SLE_CLIENT
static bool uart_rx_dma_raw_callback(uint8_t *receive_buff, uint32_t receive_length)
{
    uint32_t rx_time_us;

    if (receive_buff == NULL || receive_length == 0U) {
        return true;
    }

    rx_time_us = (uint32_t)uapi_systick_get_us();
    g_demo_stats.uart_rx_timestamp_us = rx_time_us;
    STATS_ADD(uart_rx_bytes, receive_length);
    s_uart_rx_raw_cb_count++;
    s_uart_rx_raw_cb_bytes += receive_length;
    s_uart_rx_raw_cb_last_len = receive_length;
    demo_uart_parser_feed(receive_buff, (uint16_t)receive_length, rx_time_us);
    return true;
}
#endif

#if IS_SLE_CLIENT
static void uart_overrun_error_callback(uint32_t *err_info, uint32_t len)
{
    unused(err_info);
    unused(len);
    s_uart_rx_overrun_err_count++;
}

static void uart_frame_error_callback(uint32_t *err_info, uint32_t len)
{
    unused(err_info);
    unused(len);
    s_uart_rx_frame_err_count++;
}

static void uart_parity_error_callback(uint32_t *err_info, uint32_t len)
{
    unused(err_info);
    unused(len);
    s_uart_rx_parity_err_count++;
}
#endif

static void uart_init_pin(void)
{
    if (DEMO_UART_BUS == 0) {
        DEMO_INFO("Skip Pinmux for UART0 (USB Log)");
        return;
    }
    uapi_pin_set_mode(DEMO_UART_TX_PIN, PIN_MODE_1);
    uapi_pin_set_mode(DEMO_UART_RX_PIN, PIN_MODE_1);
}

int demo_uart_init(void)
{
    errcode_t ret;
    uart_attr_t attr = {
        .baud_rate = DEMO_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };
    uart_pin_config_t pin_cfg = {
        .tx_pin = DEMO_UART_TX_PIN,
        .rx_pin = DEMO_UART_RX_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };
    uart_extra_attr_t ext_cfg = {
        .tx_dma_enable = IS_SLE_CLIENT ? false : true,
        .tx_int_threshold = UART_FIFO_INT_TX_LEVEL_EQ_0_CHARACTER,
        .rx_dma_enable = IS_SLE_CLIENT ? true : false,
        .rx_int_threshold = IS_SLE_CLIENT ? UART_FIFO_INT_RX_LEVEL_1_4 :
            UART_FIFO_INT_RX_LEVEL_1_CHARACTER
    };

    demo_frame_queue_init(&s_uart_rx_frames);
    demo_frame_queue_init(&s_uart_tx_frames);
    (void)memset_s(&s_uart_rx_parser, sizeof(s_uart_rx_parser), 0, sizeof(s_uart_rx_parser));
    if (osal_mutex_init(&s_uart_tx_lock) != OSAL_SUCCESS) {
        DEMO_ERR("UART TX lock init failed");
        return -1;
    }

    uart_init_pin();

    DEMO_INFO("Initializing UART%d @ %d baud", DEMO_UART_BUS, DEMO_UART_BAUDRATE);
    ret = uapi_dma_init();
    if (ret != ERRCODE_SUCC) {
        DEMO_INFO("DMA init: 0x%x (may be already initialized)", ret);
    }

    ret = uapi_dma_open();
    if (ret != ERRCODE_SUCC) {
        DEMO_INFO("DMA open: 0x%x (may be already open)", ret);
    }

    (void)uapi_uart_deinit(DEMO_UART_BUS);
    ret = uapi_uart_init(DEMO_UART_BUS, &pin_cfg, &attr, &ext_cfg, &s_uart_buf_cfg);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART init failed: 0x%x", ret);
        osal_mutex_destroy(&s_uart_tx_lock);
        return -1;
    }

#if IS_SLE_CLIENT
    ret = uapi_uart_dma_recv_raw_data(DEMO_UART_BUS, &s_rx_dma_cfg, uart_rx_dma_raw_callback);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART RX DMA+IDLE register failed: 0x%x", ret);
        (void)uapi_uart_deinit(DEMO_UART_BUS);
        osal_mutex_destroy(&s_uart_tx_lock);
        return -1;
    }
    DEMO_INFO("UART init OK (UART TX, frame-aware RX, RX DMA+IDLE)");
    (void)uapi_uart_register_overrun_error_callback(DEMO_UART_BUS, uart_overrun_error_callback);
    (void)uapi_uart_register_frame_error_callback(DEMO_UART_BUS, uart_frame_error_callback);
    (void)uapi_uart_register_parity_error_callback(DEMO_UART_BUS, uart_parity_error_callback);
    DEMO_INFO("UART error callbacks registered (overrun/frame/parity)");
#else
    ret = uapi_uart_register_rx_callback(DEMO_UART_BUS,
        UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE,
        DEMO_UART_RX_THRESHOLD, uart_rx_callback);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART RX callback register failed: 0x%x", ret);
        (void)uapi_uart_deinit(DEMO_UART_BUS);
        osal_mutex_destroy(&s_uart_tx_lock);
        return -1;
    }
    DEMO_INFO("UART init OK (DMA TX, frame-aware RX)");
#endif
    return 0;
}

void demo_uart_deinit(void)
{
#if IS_SLE_CLIENT
    uapi_uart_unregister_read_by_dma_callback(DEMO_UART_BUS);
#endif
    uapi_uart_unregister_rx_callback(DEMO_UART_BUS);
    uapi_uart_deinit(DEMO_UART_BUS);
    osal_mutex_destroy(&s_uart_tx_lock);
}

void demo_uart_rx_poll(void)
{
#if IS_SLE_CLIENT
    static uint32_t s_last_poll_ms = 0;
    uint32_t now = (uint32_t)uapi_systick_get_ms();

    if ((now - s_last_poll_ms) < DEMO_UART_RX_SOFT_FLUSH_POLL_MS) {
        return;
    }
    s_last_poll_ms = now;
    (void)uapi_uart_dma_idle_flush_pending(DEMO_UART_BUS);
#endif
}

uint32_t demo_uart_get_idle_isr_count(void)
{
#if IS_SLE_CLIENT
    return uapi_uart_dma_idle_get_idle_isr_count(DEMO_UART_BUS);
#else
    return 0U;
#endif
}

void demo_uart_get_rx_diag(demo_uart_rx_diag_t *diag)
{
    if (diag == NULL) {
        return;
    }

    (void)memset_s(diag, sizeof(*diag), 0, sizeof(*diag));
#if IS_SLE_CLIENT
    {
        uart_dma_idle_diag_t uart_diag = {0};

        diag->idle_isr_count = uapi_uart_dma_idle_get_idle_isr_count(DEMO_UART_BUS);
        diag->raw_callback_count = s_uart_rx_raw_cb_count;
        diag->raw_callback_bytes = s_uart_rx_raw_cb_bytes;
        diag->raw_callback_last_len = s_uart_rx_raw_cb_last_len;
        if (uapi_uart_dma_idle_get_diag(DEMO_UART_BUS, &uart_diag) == ERRCODE_SUCC) {
            diag->publish_count = uart_diag.publish_count;
            diag->publish_bytes = uart_diag.publish_bytes;
            diag->publish_from_idle_cb = uart_diag.publish_from_idle_cb;
            diag->publish_from_soft_flush = uart_diag.publish_from_soft_flush;
            diag->publish_from_idle_fallback = uart_diag.publish_from_idle_fallback;
            diag->forced_idle_request_count = uart_diag.forced_idle_request_count;
            diag->last_transfer_num = uart_diag.last_transfer_num;
            diag->last_remaining = uart_diag.last_remaining;
            diag->last_received_blocks = uart_diag.last_received_blocks;
            diag->last_received_len = uart_diag.last_received_len;
            diag->last_idle_tail_len = uart_diag.last_idle_tail_len;
            diag->last_combined_len = uart_diag.last_combined_len;
            diag->last_publish_reason = uart_diag.last_publish_reason;
            diag->last_rx_fifo_empty = uart_diag.last_rx_fifo_empty;
            diag->last_fifo_drain_len = uart_diag.last_fifo_drain_len;
        }
        diag->overrun_error_count = s_uart_rx_overrun_err_count;
        diag->frame_error_count = s_uart_rx_frame_err_count;
        diag->parity_error_count = s_uart_rx_parity_err_count;
    }
#endif
}

void demo_uart_reset_queues(void)
{
    demo_frame_queue_init(&s_uart_rx_frames);
    demo_frame_queue_init(&s_uart_tx_frames);
    (void)memset_s(&s_uart_rx_parser, sizeof(s_uart_rx_parser), 0, sizeof(s_uart_rx_parser));
    s_dma_tx_busy = false;
    /* Reset RX diagnostics on connection reset */
#if IS_SLE_CLIENT
    s_uart_rx_raw_cb_count = 0;
    s_uart_rx_raw_cb_bytes = 0;
    s_uart_rx_raw_cb_last_len = 0;
#endif
}

const demo_frame_slot_t *demo_uart_rx_frame_peek(void)
{
    const demo_frame_slot_t *slot;
    uint32_t irq_sts = osal_irq_lock();
    slot = demo_frame_queue_peek(&s_uart_rx_frames);
    osal_irq_restore(irq_sts);
    return slot;
}

void demo_uart_rx_frame_consume(void)
{
    uint32_t irq_sts = osal_irq_lock();
    demo_frame_queue_consume(&s_uart_rx_frames);
    osal_irq_restore(irq_sts);
}

bool demo_uart_queue_tx_frame(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us, uint32_t stage_duration_us)
{
    bool queued = demo_frame_queue_push(&s_uart_tx_frames, data, len, frame_id,
        enqueue_timestamp_us, ready_timestamp_us, stage_duration_us);
    if (!queued) {
        STATS_INC(uart_tx_drop_frames);
        DEMO_ERR("UART TX frame queue full: id=%u len=%u", frame_id, len);
        return false;
    }

    STATS_SET_HWM(uart_tx_ring_hwm, demo_frame_queue_count(&s_uart_tx_frames));
    return true;
}

bool demo_uart_tx_is_busy(void)
{
    return s_dma_tx_busy;
}

bool demo_uart_tx_can_fast_path(void)
{
    return !s_dma_tx_busy && demo_frame_queue_is_empty(&s_uart_tx_frames);
}

static void demo_uart_record_tx_complete(uint16_t frame_id, uint16_t len,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us, uint32_t stage_duration_us,
    uint32_t tx_start_us, uint32_t now_us, bool fast_path)
{
    uint32_t delivery_us = 0;
    uint32_t reassembly_us = 0;
    uint32_t uart_queue_wait_us = 0;
    uint32_t uart_submit_us = 0;

    if (enqueue_timestamp_us > 0 && now_us >= enqueue_timestamp_us) {
        delivery_us = now_us - enqueue_timestamp_us;
        STATS_UPDATE_RANGE(frame_rx_delay_us_min, frame_rx_delay_us_max,
            frame_rx_delay_us_sum, frame_rx_delay_count, delivery_us);
    }
    if (stage_duration_us > 0U) {
        reassembly_us = stage_duration_us;
        STATS_UPDATE_RANGE(sle_reassembly_us_min, sle_reassembly_us_max,
            sle_reassembly_us_sum, sle_reassembly_count, reassembly_us);
    }
    if (ready_timestamp_us > 0 && tx_start_us >= ready_timestamp_us) {
        uart_queue_wait_us = tx_start_us - ready_timestamp_us;
        STATS_UPDATE_RANGE(uart_queue_wait_us_min, uart_queue_wait_us_max,
            uart_queue_wait_us_sum, uart_queue_wait_count, uart_queue_wait_us);
    }
    if (now_us >= tx_start_us) {
        uart_submit_us = now_us - tx_start_us;
        STATS_UPDATE_RANGE(uart_submit_us_min, uart_submit_us_max,
            uart_submit_us_sum, uart_submit_count, uart_submit_us);
    }

    STATS_INC(uart_tx_frames);
    DEMO_LOG("UART TX frame complete: id=%u len=%u reassembly=%u us queue_wait=%u us submit=%u us delivery=%u us%s",
        frame_id, len, reassembly_us, uart_queue_wait_us, uart_submit_us, delivery_us,
        fast_path ? " fast=1" : "");
}

uint32_t demo_uart_tx_send(const uint8_t *data, uint32_t len)
{
    int32_t ret;

#if !IS_SLE_CLIENT
    uint8_t *dma_buf;
#endif

    if (data == NULL || len == 0 || len > DEMO_DMA_CHUNK_SIZE) {
        return 0;
    }

#if IS_SLE_CLIENT
    ret = uapi_uart_write(DEMO_UART_BUS, data, len, 0);
    if (ret < 0) {
        DEMO_ERR("UART write failed: %d", ret);
        return 0;
    }
    STATS_ADD(uart_tx_bytes, len);
    return (uint32_t)ret;
#else
    dma_buf = s_dma_tx_buf[s_dma_tx_idx];
    if (memcpy_s(dma_buf, DEMO_DMA_CHUNK_SIZE, data, len) != EOK) {
        return 0;
    }

#if defined(osal_dcache_region_wb)
    osal_dcache_region_wb(dma_buf, len);
#endif

    s_dma_tx_busy = true;
    ret = uapi_uart_write_by_dma(DEMO_UART_BUS, dma_buf, len, &s_dma_cfg);
    s_dma_tx_busy = false;
    if (ret < 0) {
        DEMO_ERR("DMA write failed: %d", ret);
        return 0;
    }

    STATS_ADD(uart_tx_bytes, len);
    s_dma_tx_idx = (uint8_t)((s_dma_tx_idx + 1U) % DEMO_TX_BUFFER_COUNT);
    return len;
#endif
}

int32_t demo_uart_tx_direct(const uint8_t *data, uint16_t len)
{
    return (int32_t)demo_uart_tx_send(data, len);
}

bool demo_uart_tx_direct_frame(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us, uint32_t stage_duration_us)
{
    uint32_t tx_start_us;
    uint32_t now_us;
    uint32_t sent;

    if (!demo_uart_tx_can_fast_path()) {
        return false;
    }
    if (!osal_mutex_trylock(&s_uart_tx_lock)) {
        return false;
    }
    if (!demo_uart_tx_can_fast_path()) {
        osal_mutex_unlock(&s_uart_tx_lock);
        return false;
    }

    tx_start_us = (uint32_t)uapi_systick_get_us();
    sent = demo_uart_tx_send(data, len);
    osal_mutex_unlock(&s_uart_tx_lock);
    if (sent != len) {
        return false;
    }

    now_us = (uint32_t)uapi_systick_get_us();
    demo_uart_record_tx_complete(frame_id, len, enqueue_timestamp_us, ready_timestamp_us,
        stage_duration_us, tx_start_us, now_us, true);
    return true;
}

#define DEMO_MAX_DMA_PER_LOOP  4

uint32_t demo_uart_tx_process(void)
{
    const demo_frame_slot_t *slot;
    uint32_t total_sent = 0;
    uint32_t batch = 0;

    if (!osal_mutex_trylock(&s_uart_tx_lock)) {
        return 0;
    }

    while (batch < DEMO_MAX_DMA_PER_LOOP) {
        uint32_t tx_start_us;
        uint32_t now_us;
        uint32_t sent;

        slot = demo_frame_queue_peek(&s_uart_tx_frames);
        if (slot == NULL) {
            break;
        }

        tx_start_us = (uint32_t)uapi_systick_get_us();
        sent = demo_uart_tx_send(slot->data, slot->len);
        if (sent != slot->len) {
            STATS_INC(uart_tx_drop_frames);
            break;
        }

        now_us = (uint32_t)uapi_systick_get_us();
        demo_uart_record_tx_complete(slot->frame_id, slot->len, slot->enqueue_timestamp_us,
            slot->ready_timestamp_us, slot->stage_duration_us, tx_start_us, now_us, false);

        demo_frame_queue_consume(&s_uart_tx_frames);
        total_sent += sent;
        batch++;
    }

    osal_mutex_unlock(&s_uart_tx_lock);
    return total_sent;
}

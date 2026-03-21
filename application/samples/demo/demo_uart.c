/**
 * @file demo_uart.c
 * @brief UART DMA TX and logical-frame RX implementation.
 */

#include "demo_uart.h"
#include "demo_config.h"
#include "demo_sle.h"
#include "hal_uart.h"
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
    uint16_t timeout_pending_used;
    uint32_t first_byte_timestamp_us;
    uint32_t last_byte_timestamp_us;
    uint32_t raw_input_bytes;
    uint32_t dropped_bytes;
    bool timeout_pending;
} demo_uart_frame_parser_t;

#if IS_SLE_CLIENT
#define DEMO_UART_RAW_CHUNK_SLOT_COUNT (DEMO_UART_RAW_CHUNK_QUEUE_DEPTH + 1U)

typedef struct {
    uint16_t length;
    uint32_t timestamp_us;
    uint8_t data[DEMO_UART_RAW_CHUNK_SLOT_SIZE];
} demo_uart_raw_chunk_slot_t;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    demo_uart_raw_chunk_slot_t slots[DEMO_UART_RAW_CHUNK_SLOT_COUNT];
} demo_uart_raw_chunk_queue_t;
#endif

#define DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX 32U

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
static volatile bool s_uart_rx_dma_started = false;
static volatile uint32_t s_uart_rx_last_enqueue_us = 0;
static volatile uint32_t s_uart_rx_gap_us_min = 0xFFFFFFFFU;
static volatile uint32_t s_uart_rx_gap_us_max = 0;
#if IS_SLE_CLIENT
static demo_uart_raw_chunk_queue_t s_uart_rx_raw_chunks;
static volatile uint32_t s_uart_rx_raw_chunk_drop_count = 0;
static volatile uint32_t s_uart_rx_raw_chunk_processed_count = 0;
static volatile uint32_t s_uart_rx_raw_chunk_processed_bytes = 0;
static volatile uint16_t s_uart_rx_raw_chunk_hwm = 0;
#endif
static volatile uint32_t s_uart_rx_overrun_err_count = 0;
static volatile uint32_t s_uart_rx_frame_err_count = 0;
static volatile uint32_t s_uart_rx_parity_err_count = 0;
static volatile uint32_t s_uart_rx_timeout_count = 0;
static volatile uint32_t s_uart_rx_timeout_last_gap_us = 0;
static volatile uint32_t s_uart_rx_timeout_last_age_us = 0;
static volatile uint16_t s_uart_rx_timeout_last_used = 0;
static volatile uint16_t s_uart_rx_timeout_last_expected = 0;
static volatile uint32_t s_uart_rx_timeout_last_raw_input_bytes = 0;
static volatile uint32_t s_uart_rx_timeout_last_publish_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_dma_full_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_def_set_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_def_drain_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_overrun_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_frame_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_parity_delta = 0;
static volatile uint32_t s_uart_rx_timeout_last_publish_seq = 0;
static volatile uint32_t s_uart_rx_timeout_consecutive = 0;
static volatile uint32_t s_uart_rx_timeout_consecutive_max = 0;
static volatile uint32_t s_uart_rx_timeout_resync_discarded = 0;
static volatile uint32_t s_uart_rx_timeout_resync_discarded_last = 0;
static volatile bool s_uart_rx_timeout_wait_resync = false;
static volatile uint32_t s_uart_rx_abandoned_partial_frame_count = 0;
static volatile uint32_t s_uart_rx_abandoned_partial_frame_bytes = 0;
static volatile uint8_t s_uart_rx_timeout_last_header[4] = {0};
static uint8_t s_uart_rx_timeout_resync_prefix[DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX] = {0};
static uint8_t s_uart_rx_timeout_resync_prefix_last[DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX] = {0};
static volatile uint8_t s_uart_rx_timeout_resync_prefix_len = 0;
static volatile uint8_t s_uart_rx_timeout_resync_prefix_last_len = 0;
static uint32_t s_uart_rx_timeout_prev_publish_count = 0;
static uint32_t s_uart_rx_timeout_prev_dma_full_count = 0;
static uint32_t s_uart_rx_timeout_prev_def_set_count = 0;
static uint32_t s_uart_rx_timeout_prev_def_drain_count = 0;
static uint32_t s_uart_rx_timeout_prev_overrun_count = 0;
static uint32_t s_uart_rx_timeout_prev_frame_error_count = 0;
static uint32_t s_uart_rx_timeout_prev_parity_error_count = 0;

static uart_write_dma_config_t s_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_4,
    .priority = HAL_DMA_CH_PRIORITY_0
};

static uart_write_dma_config_t s_rx_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    /* LLI soft-flush publishes active-block partials without draining UART FIFO.
       Use single-byte RX DMA bursts so small frames do not leave 1-3 tail bytes
       stranded in FIFO until the next frame arrives. */
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_1,
    .priority = HAL_DMA_CH_PRIORITY_3
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

#if IS_SLE_CLIENT
    if (!queued && demo_sle_is_connected()) {
        uint8_t retry = 0U;

        while (retry < DEMO_FRAME_QUEUE_DEPTH) {
            if (demo_sle_tx_process() == 0U) {
                break;
            }
            irq_sts = osal_irq_lock();
            queued = demo_frame_queue_push(&s_uart_rx_frames, data, len, frame_id,
                enqueue_timestamp_us, ready_timestamp_us, 0U);
            queue_count = demo_frame_queue_count(&s_uart_rx_frames);
            osal_irq_restore(irq_sts);
            if (queued) {
                break;
            }
            retry++;
        }
    }
#endif

    if (!queued) {
        STATS_INC(uart_rx_drop_frames);
        STATS_ADD(uart_rx_drop_bytes, len);
        DEMO_ERR("UART RX frame queue full: id=%u len=%u q=%u gap_min=%u gap_max=%u us",
            frame_id, len, queue_count, s_uart_rx_gap_us_min, s_uart_rx_gap_us_max);
        return;
    }

    STATS_INC(uart_rx_frames);
    STATS_SET_HWM(uart_rx_ring_hwm, queue_count);
    DEMO_LOG("UART RX frame queued: id=%u len=%u q=%u", frame_id, len, queue_count);
    osal_event_write(&g_bridge_event, DEMO_EVENT_UART_RX);

#if IS_SLE_CLIENT
    /* In LLI mode a single raw chunk can contain many tiny logical frames.
       Drain toward SLE before the fixed-depth frame queue overflows. */
    while (queue_count >= (DEMO_FRAME_QUEUE_DEPTH - 1U) && demo_sle_is_connected()) {
        if (demo_sle_tx_process() == 0U) {
            break;
        }
        irq_sts = osal_irq_lock();
        queue_count = demo_frame_queue_count(&s_uart_rx_frames);
        osal_irq_restore(irq_sts);
    }
#endif
}

#if IS_SLE_CLIENT
static void demo_uart_raw_chunk_queue_init(void)
{
    s_uart_rx_raw_chunks.head = 0;
    s_uart_rx_raw_chunks.tail = 0;
}

static uint8_t demo_uart_raw_chunk_queue_count(void)
{
    return (uint8_t)((s_uart_rx_raw_chunks.tail + DEMO_UART_RAW_CHUNK_SLOT_COUNT -
        s_uart_rx_raw_chunks.head) % DEMO_UART_RAW_CHUNK_SLOT_COUNT);
}

static bool demo_uart_enqueue_raw_chunk(const uint8_t *data, uint16_t length, uint32_t timestamp_us)
{
    demo_uart_raw_chunk_slot_t *slot;
    uint8_t tail;
    uint8_t next_tail;
    uint8_t queue_count;
    uint32_t irq_sts;

    if (data == NULL || length == 0U || length > DEMO_UART_RAW_CHUNK_SLOT_SIZE) {
        return false;
    }

    tail = s_uart_rx_raw_chunks.tail;
    next_tail = (uint8_t)((tail + 1U) % DEMO_UART_RAW_CHUNK_SLOT_COUNT);
    if (next_tail == s_uart_rx_raw_chunks.head) {
        s_uart_rx_raw_chunk_drop_count++;
        STATS_INC(ring_overflow);
        STATS_ADD(uart_rx_drop_bytes, length);
        return false;
    }

    slot = &s_uart_rx_raw_chunks.slots[tail];
    if (memcpy_s(slot->data, sizeof(slot->data), data, length) != EOK) {
        s_uart_rx_raw_chunk_drop_count++;
        STATS_INC(ring_overflow);
        STATS_ADD(uart_rx_drop_bytes, length);
        return false;
    }

    slot->length = length;
    slot->timestamp_us = timestamp_us;

    irq_sts = osal_irq_lock();
    s_uart_rx_raw_chunks.tail = next_tail;
    queue_count = demo_uart_raw_chunk_queue_count();
    if (queue_count > s_uart_rx_raw_chunk_hwm) {
        s_uart_rx_raw_chunk_hwm = queue_count;
    }
    osal_irq_restore(irq_sts);
    return true;
}
#endif

static void demo_uart_reset_timeout_diag(void)
{
    uint8_t i;

    s_uart_rx_timeout_count = 0;
    s_uart_rx_timeout_last_gap_us = 0;
    s_uart_rx_timeout_last_age_us = 0;
    s_uart_rx_timeout_last_used = 0;
    s_uart_rx_timeout_last_expected = 0;
    s_uart_rx_timeout_last_raw_input_bytes = 0;
    s_uart_rx_timeout_last_publish_delta = 0;
    s_uart_rx_timeout_last_dma_full_delta = 0;
    s_uart_rx_timeout_last_def_set_delta = 0;
    s_uart_rx_timeout_last_def_drain_delta = 0;
    s_uart_rx_timeout_last_overrun_delta = 0;
    s_uart_rx_timeout_last_frame_delta = 0;
    s_uart_rx_timeout_last_parity_delta = 0;
    s_uart_rx_timeout_last_publish_seq = 0;
    s_uart_rx_timeout_consecutive = 0;
    s_uart_rx_timeout_consecutive_max = 0;
    s_uart_rx_timeout_resync_discarded = 0;
    s_uart_rx_timeout_resync_discarded_last = 0;
    s_uart_rx_timeout_wait_resync = false;
    s_uart_rx_timeout_resync_prefix_len = 0;
    s_uart_rx_timeout_resync_prefix_last_len = 0;
    s_uart_rx_abandoned_partial_frame_count = 0;
    s_uart_rx_abandoned_partial_frame_bytes = 0;
    s_uart_rx_timeout_prev_publish_count = 0;
    s_uart_rx_timeout_prev_dma_full_count = 0;
    s_uart_rx_timeout_prev_def_set_count = 0;
    s_uart_rx_timeout_prev_def_drain_count = 0;
    s_uart_rx_timeout_prev_overrun_count = 0;
    s_uart_rx_timeout_prev_frame_error_count = 0;
    s_uart_rx_timeout_prev_parity_error_count = 0;
    for (i = 0; i < 4U; i++) {
        s_uart_rx_timeout_last_header[i] = 0;
    }
}

static uint32_t demo_uart_counter_delta(uint32_t current, uint32_t *previous)
{
    uint32_t delta = current - *previous;
    *previous = current;
    return delta;
}

static void demo_uart_snapshot_timeout_header(const demo_uart_frame_parser_t *parser)
{
    uint8_t i;

    if (parser == NULL) {
        return;
    }

    for (i = 0; i < 4U; i++) {
        if (parser->used > (uint16_t)i) {
            s_uart_rx_timeout_last_header[i] = parser->buffer[i];
        } else {
            s_uart_rx_timeout_last_header[i] = 0;
        }
    }
}

static void demo_uart_capture_timeout_resync_prefix(const uint8_t *data, uint16_t count)
{
    uint16_t copy_len;

    if (data == NULL || count == 0U || !s_uart_rx_timeout_wait_resync) {
        return;
    }
    if (s_uart_rx_timeout_resync_prefix_len >= DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX) {
        return;
    }

    copy_len = (uint16_t)(DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX - s_uart_rx_timeout_resync_prefix_len);
    if (count < copy_len) {
        copy_len = count;
    }
    if (memcpy_s(&s_uart_rx_timeout_resync_prefix[s_uart_rx_timeout_resync_prefix_len],
        DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX - s_uart_rx_timeout_resync_prefix_len, data, copy_len) != EOK) {
        return;
    }
    s_uart_rx_timeout_resync_prefix_len = (uint8_t)(s_uart_rx_timeout_resync_prefix_len + copy_len);
}

static void demo_uart_format_bytes_hex(const uint8_t *data, uint8_t length, char *out, uint32_t out_len)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t i;
    uint32_t pos = 0;

    if (out == NULL || out_len == 0U) {
        return;
    }
    out[0] = '\0';
    if (data == NULL || length == 0U) {
        return;
    }

    for (i = 0; i < length; i++) {
        if ((pos + 3U) >= out_len) {
            break;
        }
        out[pos++] = hex[(data[i] >> 4) & 0x0FU];
        out[pos++] = hex[data[i] & 0x0FU];
        if (i + 1U < length) {
            out[pos++] = ' ';
        }
    }
    out[pos] = '\0';
}

static uint32_t demo_uart_frame_gap_timeout_us(void)
{
    uint32_t jitter_margin_us =
        (DEMO_UART_RX_SOFT_FLUSH_POLL_MS + DEMO_EVENT_TIMEOUT_MS) * 1000U;
    return DEMO_UART_FRAME_GAP_TIMEOUT_US + jitter_margin_us;
}

static uint32_t demo_uart_frame_age_timeout_us(const demo_uart_frame_parser_t *parser)
{
    uint16_t frame_len;
    uint64_t wire_time_us;
    uint32_t gap_timeout_us = demo_uart_frame_gap_timeout_us();

    if (parser == NULL) {
        return gap_timeout_us;
    }

    frame_len = parser->expected_len;
    if (frame_len == 0U || frame_len < parser->used) {
        frame_len = parser->used;
    }
    if (frame_len == 0U) {
        return gap_timeout_us;
    }

    /* 8N1 on-wire time plus one gap window for scheduler / poll jitter. */
    wire_time_us = ((uint64_t)frame_len * 10ULL * 1000000ULL + (uint64_t)DEMO_UART_BAUDRATE - 1ULL) /
        (uint64_t)DEMO_UART_BAUDRATE;
    return gap_timeout_us + (uint32_t)wire_time_us;
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
    parser->timeout_pending = false;
    parser->timeout_pending_used = 0;
}

static void demo_uart_parser_drop_prefix(demo_uart_frame_parser_t *parser, uint16_t count, bool count_as_drop)
{
    if (parser == NULL || count == 0) {
        return;
    }

    if (count_as_drop) {
        demo_uart_capture_timeout_resync_prefix(parser->buffer, count);
        parser->dropped_bytes += count;
        if (s_uart_rx_timeout_wait_resync) {
            s_uart_rx_timeout_resync_discarded += count;
        }
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

static bool demo_uart_parser_find_timeout_resync_offset(const demo_uart_frame_parser_t *parser, uint16_t *offset_out)
{
    uint16_t scan;
    uint16_t total_len;

    if (parser == NULL || offset_out == NULL || !parser->timeout_pending ||
        parser->expected_len <= parser->timeout_pending_used ||
        parser->used < DEMO_LOGICAL_FRAME_HEADER_SIZE ||
        parser->timeout_pending_used >= parser->used) {
        return false;
    }

    for (scan = parser->timeout_pending_used;
         (scan < parser->expected_len) &&
         ((uint32_t)scan + DEMO_LOGICAL_FRAME_HEADER_SIZE <= parser->used); scan++) {
        if (!demo_logical_frame_header_valid(&parser->buffer[scan])) {
            continue;
        }
        total_len = demo_logical_frame_total_len(&parser->buffer[scan]);
        if (total_len >= DEMO_LOGICAL_FRAME_HEADER_SIZE && total_len <= DEMO_LOGICAL_FRAME_MAX_SIZE) {
            *offset_out = scan;
            return true;
        }
    }

    return false;
}

static void demo_uart_parser_abandon_timeout_frame(demo_uart_frame_parser_t *parser,
    uint16_t resync_offset, uint32_t timestamp_us)
{
    uint16_t discard_prefix = 0;

    if (parser == NULL || !parser->timeout_pending) {
        return;
    }

    STATS_INC(uart_rx_drop_frames);
    STATS_ADD(uart_rx_drop_bytes, parser->timeout_pending_used);
    s_uart_rx_abandoned_partial_frame_count++;
    s_uart_rx_abandoned_partial_frame_bytes += parser->timeout_pending_used;

    s_uart_rx_timeout_wait_resync = true;
    s_uart_rx_timeout_resync_discarded = 0;
    s_uart_rx_timeout_resync_prefix_len = 0;

    if (resync_offset > parser->timeout_pending_used) {
        discard_prefix = (uint16_t)(resync_offset - parser->timeout_pending_used);
        demo_uart_capture_timeout_resync_prefix(parser->buffer + parser->timeout_pending_used, discard_prefix);
        s_uart_rx_timeout_resync_discarded = discard_prefix;
        parser->dropped_bytes += discard_prefix;
        STATS_ADD(uart_rx_invalid_bytes, discard_prefix);
    }

    demo_uart_parser_drop_prefix(parser, resync_offset, false);
    parser->timeout_pending = false;
    parser->timeout_pending_used = 0;
    if (parser->used > 0U) {
        parser->expected_len = 0;
        parser->first_byte_timestamp_us = timestamp_us;
        parser->last_byte_timestamp_us = timestamp_us;
        parser->raw_input_bytes = parser->used;
        parser->dropped_bytes = discard_prefix;
    }
}

static void demo_uart_parser_resolve_timeout_pending(demo_uart_frame_parser_t *parser, uint32_t timestamp_us)
{
    uint16_t resync_offset;
    uint32_t pending_age_us;
    uint32_t hard_limit_us;

    if (parser == NULL || !parser->timeout_pending || parser->expected_len == 0U) {
        return;
    }

    if (parser->used >= parser->expected_len) {
        parser->timeout_pending = false;
        parser->timeout_pending_used = 0;
        return;
    }

    pending_age_us = (timestamp_us >= parser->first_byte_timestamp_us) ?
        (timestamp_us - parser->first_byte_timestamp_us) : 0U;
    hard_limit_us = 2U * demo_uart_frame_age_timeout_us(parser);

    if (pending_age_us < hard_limit_us) {
        return;
    }

    if (demo_uart_parser_find_timeout_resync_offset(parser, &resync_offset)) {
        demo_uart_parser_abandon_timeout_frame(parser, resync_offset, timestamp_us);
    }
}

static void demo_uart_parser_drop_stale_frame(demo_uart_frame_parser_t *parser, uint32_t timestamp_us)
{
    demo_uart_rx_diag_t uart_diag = {0};
    uint32_t gap_us;
    uint32_t gap_timeout_us;
    uint32_t age_us = 0;
    uint32_t age_timeout_us;
    uint32_t publish_delta;
    uint32_t dma_full_delta;
    uint32_t def_set_delta;
    uint32_t def_drain_delta;
    uint32_t overrun_delta;
    uint32_t frame_delta;
    uint32_t parity_delta;

    if (parser == NULL || parser->used == 0 || parser->last_byte_timestamp_us == 0U ||
        timestamp_us < parser->last_byte_timestamp_us) {
        return;
    }
    if (parser->timeout_pending) {
        return;
    }

    gap_us = timestamp_us - parser->last_byte_timestamp_us;
    if (parser->first_byte_timestamp_us != 0U && timestamp_us >= parser->first_byte_timestamp_us) {
        age_us = timestamp_us - parser->first_byte_timestamp_us;
    }
    gap_timeout_us = demo_uart_frame_gap_timeout_us();
    age_timeout_us = demo_uart_frame_age_timeout_us(parser);
    if (gap_us <= gap_timeout_us && age_us <= age_timeout_us) {
        return;
    }

    /* Skip error logging for tiny residuals (≤2 bytes) — likely sender-side alignment */
    if (parser->used <= 2U) {
        demo_uart_parser_reset(parser);
        return;
    }
    if (parser->expected_len == 0U && parser->used >= DEMO_LOGICAL_FRAME_HEADER_SIZE &&
        demo_logical_frame_header_valid(parser->buffer)) {
        parser->expected_len = demo_logical_frame_total_len(parser->buffer);
    }
    if (parser->expected_len < DEMO_LOGICAL_FRAME_HEADER_SIZE ||
        parser->expected_len > DEMO_LOGICAL_FRAME_MAX_SIZE) {
        demo_uart_parser_reset(parser);
        return;
    }

    s_uart_rx_timeout_count++;
    s_uart_rx_timeout_last_gap_us = gap_us;
    s_uart_rx_timeout_last_age_us = age_us;
    s_uart_rx_timeout_last_used = parser->used;
    s_uart_rx_timeout_last_expected = parser->expected_len;
    s_uart_rx_timeout_last_raw_input_bytes = parser->raw_input_bytes;
    s_uart_rx_timeout_consecutive++;
    if (s_uart_rx_timeout_consecutive > s_uart_rx_timeout_consecutive_max) {
        s_uart_rx_timeout_consecutive_max = s_uart_rx_timeout_consecutive;
    }
    s_uart_rx_timeout_wait_resync = false;
    s_uart_rx_timeout_resync_discarded = 0;
    s_uart_rx_timeout_resync_prefix_len = 0;
    demo_uart_snapshot_timeout_header(parser);
    parser->timeout_pending = true;
    parser->timeout_pending_used = parser->used;

    demo_uart_get_rx_diag(&uart_diag);
    publish_delta = demo_uart_counter_delta(uart_diag.publish_count, &s_uart_rx_timeout_prev_publish_count);
    dma_full_delta = demo_uart_counter_delta(uart_diag.publish_from_dma_complete, &s_uart_rx_timeout_prev_dma_full_count);
    def_set_delta = demo_uart_counter_delta(uart_diag.deferred_publish_set_count, &s_uart_rx_timeout_prev_def_set_count);
    def_drain_delta = demo_uart_counter_delta(uart_diag.deferred_publish_drained_count,
        &s_uart_rx_timeout_prev_def_drain_count);
    overrun_delta = demo_uart_counter_delta(uart_diag.overrun_error_count, &s_uart_rx_timeout_prev_overrun_count);
    frame_delta = demo_uart_counter_delta(uart_diag.frame_error_count, &s_uart_rx_timeout_prev_frame_error_count);
    parity_delta = demo_uart_counter_delta(uart_diag.parity_error_count, &s_uart_rx_timeout_prev_parity_error_count);

    s_uart_rx_timeout_last_publish_delta = publish_delta;
    s_uart_rx_timeout_last_dma_full_delta = dma_full_delta;
    s_uart_rx_timeout_last_def_set_delta = def_set_delta;
    s_uart_rx_timeout_last_def_drain_delta = def_drain_delta;
    s_uart_rx_timeout_last_overrun_delta = overrun_delta;
    s_uart_rx_timeout_last_frame_delta = frame_delta;
    s_uart_rx_timeout_last_parity_delta = parity_delta;
    s_uart_rx_timeout_last_publish_seq = uart_diag.last_publish_seq;

    DEMO_ERR("UART RX partial frame timeout: used=%u expected=%u raw_in=%u dropped=%u gap=%u age=%u us hdr=%02X%02X%02X%02X | dma(pub=%u seq=%u +%u dma_full=%u(+%u) def_set=%u(+%u) def_drain=%u(+%u) last_reason=%u last_combined=%u last_dma_full_seq=%u last_def_set_seq=%u last_def_drain_seq=%u) | err(overrun=%u(+%u) frame=%u(+%u) parity=%u(+%u))",
        parser->used, parser->expected_len, parser->raw_input_bytes, parser->dropped_bytes, gap_us, age_us,
        uart_diag.timeout_last_header[0], uart_diag.timeout_last_header[1],
        uart_diag.timeout_last_header[2], uart_diag.timeout_last_header[3], uart_diag.publish_count,
        uart_diag.last_publish_seq, publish_delta, uart_diag.publish_from_dma_complete, dma_full_delta,
        uart_diag.deferred_publish_set_count, def_set_delta, uart_diag.deferred_publish_drained_count,
        def_drain_delta, uart_diag.last_publish_reason, uart_diag.last_combined_len,
        uart_diag.last_dma_complete_seq, uart_diag.last_deferred_publish_set_seq,
        uart_diag.last_deferred_publish_drained_seq, uart_diag.overrun_error_count, overrun_delta,
        uart_diag.frame_error_count, frame_delta, uart_diag.parity_error_count, parity_delta);
}

static void demo_uart_queue_ingress_frame(demo_uart_frame_parser_t *parser, uint32_t ready_timestamp_us)
{
    if (parser == NULL || parser->expected_len == 0 || parser->used < parser->expected_len) {
        return;
    }

    demo_uart_queue_rx_frame(parser->buffer, parser->expected_len,
        parser->first_byte_timestamp_us, ready_timestamp_us);

    if (s_uart_rx_timeout_consecutive > 0U) {
        DEMO_INFO("UART RX timeout burst recovered: burst=%u", s_uart_rx_timeout_consecutive);
        s_uart_rx_timeout_consecutive = 0;
    }

    parser->timeout_pending = false;
    parser->timeout_pending_used = 0;
    demo_uart_parser_drop_prefix(parser, parser->expected_len, false);
    parser->expected_len = 0;
}

static void demo_uart_parser_process(demo_uart_frame_parser_t *parser, uint32_t timestamp_us)
{
    uint16_t expected_len;
    bool header_valid;

    if (parser == NULL) {
        return;
    }

    while (parser->used > 0) {
        if (parser->used >= 2U) {
            header_valid = demo_logical_frame_header_valid(parser->buffer);
            if (!header_valid) {
                STATS_INC(uart_rx_invalid_bytes);
                demo_uart_parser_drop_prefix(parser, 1, true);
                continue;
            }

            if (s_uart_rx_timeout_wait_resync) {
                char prefix_hex[DEMO_UART_TIMEOUT_RESYNC_PREFIX_MAX * 3U] = {0};

                s_uart_rx_timeout_resync_discarded_last = s_uart_rx_timeout_resync_discarded;
                s_uart_rx_timeout_resync_prefix_last_len = s_uart_rx_timeout_resync_prefix_len;
                if (s_uart_rx_timeout_resync_prefix_last_len > 0U) {
                    (void)memcpy_s(s_uart_rx_timeout_resync_prefix_last, sizeof(s_uart_rx_timeout_resync_prefix_last),
                        s_uart_rx_timeout_resync_prefix, s_uart_rx_timeout_resync_prefix_last_len);
                    demo_uart_format_bytes_hex(s_uart_rx_timeout_resync_prefix_last,
                        s_uart_rx_timeout_resync_prefix_last_len, prefix_hex, sizeof(prefix_hex));
                }
                DEMO_INFO("UART RX parser resync after timeout: discarded=%u hdr=%02X%02X",
                    s_uart_rx_timeout_resync_discarded_last, parser->buffer[0], parser->buffer[1]);
                if (s_uart_rx_timeout_resync_prefix_last_len > 0U) {
                    DEMO_INFO("UART RX timeout resync prefix: len=%u bytes=%s",
                        s_uart_rx_timeout_resync_prefix_last_len, prefix_hex);
                }
                s_uart_rx_timeout_resync_discarded = 0;
                s_uart_rx_timeout_resync_prefix_len = 0;
                s_uart_rx_timeout_wait_resync = false;
            }
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

        demo_uart_parser_resolve_timeout_pending(&s_uart_rx_parser, timestamp_us);
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
    uint32_t gap_us;

    if (receive_buff == NULL || receive_length == 0U) {
        return true;
    }
    /* Note: Removed oversize guard. demo_uart_parser_feed() handles
       arbitrary length chunks via internal streaming and queue copies. */

    rx_time_us = (uint32_t)uapi_systick_get_us();

    /* Track inter-arrival gap stats */
    if (s_uart_rx_last_enqueue_us != 0 && rx_time_us > s_uart_rx_last_enqueue_us) {
        gap_us = rx_time_us - s_uart_rx_last_enqueue_us;
        if (gap_us < s_uart_rx_gap_us_min) {
            s_uart_rx_gap_us_min = gap_us;
        }
        if (gap_us > s_uart_rx_gap_us_max) {
            s_uart_rx_gap_us_max = gap_us;
        }
    }
    s_uart_rx_last_enqueue_us = rx_time_us;
    g_demo_stats.uart_rx_timestamp_us = rx_time_us;
    STATS_ADD(uart_rx_bytes, receive_length);
    s_uart_rx_raw_cb_count++;
    s_uart_rx_raw_cb_bytes += receive_length;
    s_uart_rx_raw_cb_last_len = receive_length;
    if (demo_uart_enqueue_raw_chunk(receive_buff, (uint16_t)receive_length, rx_time_us)) {
        osal_event_write(&g_bridge_event, DEMO_EVENT_UART_RX);
    }
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
    s_uart_rx_dma_started = false;
#if IS_SLE_CLIENT
    demo_uart_raw_chunk_queue_init();
    s_uart_rx_raw_chunk_drop_count = 0;
    s_uart_rx_raw_chunk_processed_count = 0;
    s_uart_rx_raw_chunk_processed_bytes = 0;
    s_uart_rx_raw_chunk_hwm = 0;
#endif
    demo_uart_reset_timeout_diag();
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
    DEMO_INFO("UART init OK (UART TX, raw idle RX, RX DMA deferred)");
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

int demo_uart_start_client_rx(void)
{
#if IS_SLE_CLIENT
    errcode_t ret;

    if (s_uart_rx_dma_started) {
        return 0;
    }

    ret = uapi_uart_dma_recv_raw_data(DEMO_UART_BUS, &s_rx_dma_cfg, uart_rx_dma_raw_callback);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART RX DMA+IDLE start failed: 0x%x", ret);
        return -1;
    }

    s_uart_rx_dma_started = true;
    DEMO_INFO("UART client RX DMA+IDLE started");
#endif
    return 0;
}

void demo_uart_deinit(void)
{
#if IS_SLE_CLIENT
    if (s_uart_rx_dma_started) {
        uapi_uart_unregister_read_by_dma_callback(DEMO_UART_BUS);
        s_uart_rx_dma_started = false;
    }
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

uint32_t demo_uart_rx_process_raw_chunks(void)
{
#if IS_SLE_CLIENT
    uint32_t processed = 0;

    while (s_uart_rx_raw_chunks.head != s_uart_rx_raw_chunks.tail) {
        uint8_t head = s_uart_rx_raw_chunks.head;
        demo_uart_raw_chunk_slot_t *slot = &s_uart_rx_raw_chunks.slots[head];

        demo_uart_parser_feed(slot->data, slot->length, slot->timestamp_us);
        s_uart_rx_raw_chunk_processed_count++;
        s_uart_rx_raw_chunk_processed_bytes += slot->length;
        processed++;
        s_uart_rx_raw_chunks.head = (uint8_t)((head + 1U) % DEMO_UART_RAW_CHUNK_SLOT_COUNT);
    }

    return processed;
#else
    return 0U;
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
        diag->raw_chunk_queue_drop_count = s_uart_rx_raw_chunk_drop_count;
        diag->raw_chunk_processed_count = s_uart_rx_raw_chunk_processed_count;
        diag->raw_chunk_processed_bytes = s_uart_rx_raw_chunk_processed_bytes;
        diag->raw_chunk_queue_hwm = s_uart_rx_raw_chunk_hwm;
        if (uapi_uart_dma_idle_get_diag(DEMO_UART_BUS, &uart_diag) == ERRCODE_SUCC) {
            diag->publish_count = uart_diag.publish_count;
            diag->publish_bytes = uart_diag.publish_bytes;
            diag->publish_from_idle_cb = uart_diag.publish_from_idle_cb;
            diag->publish_from_soft_flush = uart_diag.publish_from_soft_flush;
            diag->publish_from_idle_fallback = uart_diag.publish_from_idle_fallback;
            diag->publish_from_dma_complete = uart_diag.publish_from_dma_complete;
            diag->forced_idle_request_count = uart_diag.forced_idle_request_count;
            diag->deferred_publish_set_count = uart_diag.deferred_publish_set_count;
            diag->deferred_publish_drained_count = uart_diag.deferred_publish_drained_count;
            diag->last_publish_seq = uart_diag.last_publish_seq;
            diag->last_dma_complete_seq = uart_diag.last_dma_complete_seq;
            diag->last_deferred_publish_set_seq = uart_diag.last_deferred_publish_set_seq;
            diag->last_deferred_publish_drained_seq = uart_diag.last_deferred_publish_drained_seq;
            diag->last_transfer_num = uart_diag.last_transfer_num;
            diag->last_remaining = uart_diag.last_remaining;
            diag->last_received_blocks = uart_diag.last_received_blocks;
            diag->last_received_len = uart_diag.last_received_len;
            diag->last_idle_tail_len = uart_diag.last_idle_tail_len;
            diag->last_combined_len = uart_diag.last_combined_len;
            diag->last_publish_reason = uart_diag.last_publish_reason;
            diag->last_rx_fifo_empty = uart_diag.last_rx_fifo_empty;
            diag->last_fifo_drain_len = uart_diag.last_fifo_drain_len;
            diag->lli_rollover_count = uart_diag.lli_rollover_count;
            diag->lli_rollover_wrap_count = uart_diag.lli_rollover_wrap_count;
            diag->lli_rollover_regress_count = uart_diag.lli_rollover_regress_count;
            diag->lli_last_rollover_publish_seq = uart_diag.lli_last_rollover_publish_seq;
            diag->lli_last_rollover_block = uart_diag.lli_last_rollover_block;
            diag->lli_last_rollover_prev_published = uart_diag.lli_last_rollover_prev_published;
            diag->lli_last_rollover_partial_len = uart_diag.lli_last_rollover_partial_len;
            diag->lli_last_rollover_prev_remaining = uart_diag.lli_last_rollover_prev_remaining;
            diag->lli_last_rollover_remaining = uart_diag.lli_last_rollover_remaining;
            diag->lli_last_rollover_queue_offset = uart_diag.lli_last_rollover_queue_offset;
            diag->lli_last_rollover_queue_len = uart_diag.lli_last_rollover_queue_len;
            diag->lli_last_rollover_flags = uart_diag.lli_last_rollover_flags;
            diag->lli_last_rollover_prefix_len = uart_diag.lli_last_rollover_prefix_len;
            (void)memcpy_s(diag->lli_last_rollover_prefix, sizeof(diag->lli_last_rollover_prefix),
                uart_diag.lli_last_rollover_prefix, sizeof(uart_diag.lli_last_rollover_prefix));
            diag->lli_last_segment_reason = uart_diag.lli_last_segment_reason;
            diag->lli_last_segment_offset = uart_diag.lli_last_segment_offset;
            diag->lli_last_segment_length = uart_diag.lli_last_segment_length;
            diag->lli_last_segment_prefix_len = uart_diag.lli_last_segment_prefix_len;
            (void)memcpy_s(diag->lli_last_segment_prefix, sizeof(diag->lli_last_segment_prefix),
                uart_diag.lli_last_segment_prefix, sizeof(uart_diag.lli_last_segment_prefix));
        }
        diag->overrun_error_count = s_uart_rx_overrun_err_count;
        diag->frame_error_count = s_uart_rx_frame_err_count;
        diag->parity_error_count = s_uart_rx_parity_err_count;
        diag->enqueue_gap_us_min = s_uart_rx_gap_us_min;
        diag->enqueue_gap_us_max = s_uart_rx_gap_us_max;
    }
#endif
    diag->timeout_count = s_uart_rx_timeout_count;
    diag->timeout_last_gap_us = s_uart_rx_timeout_last_gap_us;
    diag->timeout_last_age_us = s_uart_rx_timeout_last_age_us;
    diag->timeout_last_used = s_uart_rx_timeout_last_used;
    diag->timeout_last_expected = s_uart_rx_timeout_last_expected;
    diag->timeout_last_raw_input_bytes = s_uart_rx_timeout_last_raw_input_bytes;
    diag->timeout_last_publish_delta = s_uart_rx_timeout_last_publish_delta;
    diag->timeout_last_dma_full_delta = s_uart_rx_timeout_last_dma_full_delta;
    diag->timeout_last_deferred_set_delta = s_uart_rx_timeout_last_def_set_delta;
    diag->timeout_last_deferred_drain_delta = s_uart_rx_timeout_last_def_drain_delta;
    diag->timeout_last_overrun_delta = s_uart_rx_timeout_last_overrun_delta;
    diag->timeout_last_frame_error_delta = s_uart_rx_timeout_last_frame_delta;
    diag->timeout_last_parity_error_delta = s_uart_rx_timeout_last_parity_delta;
    diag->timeout_last_publish_seq = s_uart_rx_timeout_last_publish_seq;
    diag->timeout_consecutive_count = s_uart_rx_timeout_consecutive;
    diag->timeout_consecutive_max = s_uart_rx_timeout_consecutive_max;
    diag->timeout_resync_discarded_bytes_last = s_uart_rx_timeout_resync_discarded_last;
    diag->abandoned_partial_frame_count = s_uart_rx_abandoned_partial_frame_count;
    diag->abandoned_partial_frame_bytes = s_uart_rx_abandoned_partial_frame_bytes;
    diag->timeout_last_header[0] = s_uart_rx_timeout_last_header[0];
    diag->timeout_last_header[1] = s_uart_rx_timeout_last_header[1];
    diag->timeout_last_header[2] = s_uart_rx_timeout_last_header[2];
    diag->timeout_last_header[3] = s_uart_rx_timeout_last_header[3];
}

void demo_uart_reset_queues(void)
{
    uint32_t irq_sts = osal_irq_lock();

    demo_frame_queue_init(&s_uart_rx_frames);
    demo_frame_queue_init(&s_uart_tx_frames);
    (void)memset_s(&s_uart_rx_parser, sizeof(s_uart_rx_parser), 0, sizeof(s_uart_rx_parser));
    demo_uart_reset_timeout_diag();
    s_dma_tx_busy = false;
    /* Reset RX diagnostics on connection reset */
#if IS_SLE_CLIENT
    demo_uart_raw_chunk_queue_init();
    s_uart_rx_raw_cb_count = 0;
    s_uart_rx_raw_cb_bytes = 0;
    s_uart_rx_raw_cb_last_len = 0;
    s_uart_rx_raw_chunk_drop_count = 0;
    s_uart_rx_raw_chunk_processed_count = 0;
    s_uart_rx_raw_chunk_processed_bytes = 0;
    s_uart_rx_raw_chunk_hwm = 0;
    s_uart_rx_last_enqueue_us = 0;
    s_uart_rx_gap_us_min = 0xFFFFFFFFU;
    s_uart_rx_gap_us_max = 0;
#endif
    osal_irq_restore(irq_sts);
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
    bool queued;
    uint8_t queue_count;
    uint32_t irq_sts;

    irq_sts = osal_irq_lock();
    queued = demo_frame_queue_push(&s_uart_tx_frames, data, len, frame_id,
        enqueue_timestamp_us, ready_timestamp_us, stage_duration_us);
    queue_count = demo_frame_queue_count(&s_uart_tx_frames);
    osal_irq_restore(irq_sts);

    if (!queued) {
        STATS_INC(uart_tx_drop_frames);
        DEMO_ERR("UART TX frame queue full: id=%u len=%u", frame_id, len);
        return false;
    }

    STATS_SET_HWM(uart_tx_ring_hwm, queue_count);
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
    demo_frame_slot_t local_slot;
    uint32_t total_sent = 0;
    uint32_t batch = 0;
    uint32_t irq_sts;

    if (!osal_mutex_trylock(&s_uart_tx_lock)) {
        return 0;
    }

    while (batch < DEMO_MAX_DMA_PER_LOOP) {
        uint32_t tx_start_us;
        uint32_t now_us;
        uint32_t sent;

        /* Copy frame out under IRQ lock (matches push-side locking) */
        irq_sts = osal_irq_lock();
        slot = demo_frame_queue_peek(&s_uart_tx_frames);
        if (slot == NULL) {
            osal_irq_restore(irq_sts);
            break;
        }
        local_slot = *slot;
        demo_frame_queue_consume(&s_uart_tx_frames);
        osal_irq_restore(irq_sts);

        tx_start_us = (uint32_t)uapi_systick_get_us();
        sent = demo_uart_tx_send(local_slot.data, local_slot.len);
        if (sent != local_slot.len) {
            STATS_INC(uart_tx_drop_frames);
            break;
        }

        now_us = (uint32_t)uapi_systick_get_us();
        demo_uart_record_tx_complete(local_slot.frame_id, local_slot.len,
            local_slot.enqueue_timestamp_us, local_slot.ready_timestamp_us,
            local_slot.stage_duration_us, tx_start_us, now_us, false);

        total_sent += sent;
        batch++;
    }

    osal_mutex_unlock(&s_uart_tx_lock);
    return total_sent;
}

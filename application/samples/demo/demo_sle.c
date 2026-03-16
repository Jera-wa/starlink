/**
 * @file demo_sle.c
 * @brief Frame-aware SLE Server/Client implementation.
 */

#include "demo_sle.h"
#include "demo_uart.h"
#include "demo_frame.h"
#include "demo_transport.h"
#include "demo_config.h"
#include <string.h>
#include "securec.h"
#include "soc_osal.h"
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_server.h"
#include "sle_ssap_client.h"
#include "driver/systick.h"

#if defined(CONFIG_FEATURE_GLE_LOW_LATENCY)
#include "sle_low_latency.h"
#define DEMO_SLE_LOW_LATENCY_AVAILABLE 1
#else
#define DEMO_SLE_LOW_LATENCY_AVAILABLE 0
#endif

#ifndef SLE_ADV_CHANNEL_MAP_DEFAULT
#define SLE_ADV_CHANNEL_MAP_DEFAULT 0x07
#endif
#ifndef SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME
#define SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME 0x0B
#endif
#ifndef SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME
#define SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME 0x0A
#endif

#define SLE_UUID_LEN_2              2
#define SLE_UUID_INDEX              14
#define SLE_UUID_SERVER_SERVICE     0xABCD
#define SLE_UUID_SERVER_PROPERTY    0xCDEF
#define SLE_UUID_TEST_PROPERTIES    0x03
#define SLE_UUID_TEST_DESCRIPTOR    0x01

static uint8_t s_sle_uuid_base[] = {
    0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t s_sle_tx_buf[DEMO_SLE_MTU_SIZE];
static volatile uint16_t s_conn_id = 0;
static volatile uint16_t s_mtu_size = DEMO_SLE_MTU_SIZE;
static volatile bool s_connected = false;
static volatile bool s_paired = false;
static volatile bool s_client_ready = false;
static volatile uint16_t s_client_write_handle = 0;
static bool s_seek_connect_pending = false;
static bool s_local_addr_valid = false;
static uint16_t s_requested_conn_interval_min = DEMO_SLE_CONN_INTV_MIN;
static uint16_t s_requested_conn_interval_max = DEMO_SLE_CONN_INTV_MAX;
static uint16_t s_requested_data_len = DEMO_SLE_DATA_LEN_TARGET;
static uint16_t s_effective_payload = 0;
static uint16_t s_fragment_payload = 0;
static demo_transport_tx_state_t s_tx_state;
static demo_transport_rx_state_t s_rx_state;
static demo_transport_ack_state_t s_ack_state;
static uint8_t s_server_id = 0;
static uint16_t s_service_handle = 0;
static uint16_t s_property_handle = 0;
static uint16_t s_peer_service_start = 0;
static uint16_t s_peer_service_end = 0;
static uint8_t s_server_ntf_buf[4][DEMO_SLE_MTU_SIZE];
static volatile uint8_t s_server_ntf_idx = 0;
static ssapc_write_param_t s_client_write_param = {0};
static uint8_t s_client_tx_buf[4][DEMO_SLE_MTU_SIZE];
static volatile uint8_t s_client_tx_idx = 0;
static sle_addr_t s_remote_addr = {0};
static sle_addr_t s_local_addr = {0};

static errcode_t client_start_scan(void);
static void on_sle_enable(errcode_t status);
static void on_seek_enable(errcode_t status);
static void on_seek_disable(errcode_t status);
static void on_seek_result(sle_seek_result_info_t *result);
static void on_exchange_info(uint8_t client_id, uint16_t conn_id, ssap_exchange_info_t *info, errcode_t status);
static void on_find_structure(uint8_t client_id, uint16_t conn_id, ssapc_find_service_result_t *service, errcode_t status);
static void on_find_property(uint8_t client_id, uint16_t conn_id, ssapc_find_property_result_t *property, errcode_t status);
static void on_notification(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status);
static void demo_sle_retry_active_frame(const char *reason);

static void sle_uuid_set_base(sle_uuid_t *out)
{
    if (memcpy_s(out->uuid, SLE_UUID_LEN, s_sle_uuid_base, SLE_UUID_LEN) != EOK) {
        out->len = 0;
        return;
    }
    out->len = SLE_UUID_LEN_2;
}

static void sle_uuid_set_u2(uint16_t u2, sle_uuid_t *out)
{
    sle_uuid_set_base(out);
    out->uuid[SLE_UUID_INDEX] = (uint8_t)(u2 >> 8);
    out->uuid[SLE_UUID_INDEX + 1] = (uint8_t)u2;
}

static bool demo_sle_uuid_matches_u2(const sle_uuid_t *uuid, uint16_t u2)
{
    if (uuid == NULL || uuid->len < SLE_UUID_LEN_2) {
        return false;
    }
    return uuid->uuid[SLE_UUID_INDEX] == (uint8_t)(u2 >> 8) &&
        uuid->uuid[SLE_UUID_INDEX + 1] == (uint8_t)u2;
}

static uint16_t demo_sle_clamp_link_interval(uint16_t interval)
{
    if (interval < DEMO_SLE_LINK_INTERVAL_MIN_LEGAL) {
        return DEMO_SLE_LINK_INTERVAL_MIN_LEGAL;
    }
    if (interval > DEMO_SLE_LINK_INTERVAL_MAX_LEGAL) {
        return DEMO_SLE_LINK_INTERVAL_MAX_LEGAL;
    }
    return interval;
}

static uint32_t demo_sle_link_interval_to_us(uint16_t interval)
{
    return (uint32_t)interval * 125U;
}

static int demo_sle_addr_compare(const sle_addr_t *lhs, const sle_addr_t *rhs)
{
    if (lhs == NULL || rhs == NULL) {
        return 0;
    }
    return memcmp(lhs->addr, rhs->addr, SLE_ADDR_LEN);
}

static bool demo_sle_addr_equal(const sle_addr_t *lhs, const sle_addr_t *rhs)
{
    return demo_sle_addr_compare(lhs, rhs) == 0;
}

static bool demo_sle_should_initiate_connect(const sle_addr_t *remote)
{
    if (remote == NULL) {
        return false;
    }
    return !(s_local_addr_valid && demo_sle_addr_equal(&s_local_addr, remote));
}

static void demo_sle_refresh_local_addr(void)
{
    errcode_t ret = sle_get_local_addr(&s_local_addr);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Get local addr failed: 0x%x", ret);
        s_local_addr_valid = false;
        return;
    }
    s_local_addr_valid = true;
    DEMO_INFO("Local addr:%02x:**:**:**:%02x:%02x",
        s_local_addr.addr[0], s_local_addr.addr[4], s_local_addr.addr[5]);
}

static uint32_t demo_sle_rate_to_hz(uint8_t rate)
{
    static const uint16_t k_rates_hz[] = {125, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
    if (rate >= (sizeof(k_rates_hz) / sizeof(k_rates_hz[0]))) {
        return 0;
    }
    return k_rates_hz[rate];
}

static void demo_sle_refresh_payload_limits(void)
{
    uint16_t mtu_payload = (s_mtu_size > 10U) ? (uint16_t)(s_mtu_size - 10U) : 0U;
    uint16_t data_len_payload = (s_requested_data_len > 10U) ? (uint16_t)(s_requested_data_len - 10U) : mtu_payload;

    s_effective_payload = mtu_payload;
    if (data_len_payload != 0U && data_len_payload < s_effective_payload) {
        s_effective_payload = data_len_payload;
    }
    if (s_effective_payload > (DEMO_SLE_MTU_SIZE - 10U)) {
        s_effective_payload = DEMO_SLE_MTU_SIZE - 10U;
    }
    s_fragment_payload = (s_effective_payload > DEMO_SLE_FRAG_HEADER_SIZE) ?
        (uint16_t)(s_effective_payload - DEMO_SLE_FRAG_HEADER_SIZE) : 0U;

    g_demo_stats.sle_requested_conn_interval_min = s_requested_conn_interval_min;
    g_demo_stats.sle_requested_conn_interval_max = s_requested_conn_interval_max;
    g_demo_stats.sle_requested_data_len = s_requested_data_len;
    g_demo_stats.sle_effective_payload = s_effective_payload;
    g_demo_stats.sle_fragment_payload = s_fragment_payload;
}

static void demo_sle_request_conn_param_update(const char *reason)
{
    sle_connection_param_update_t conn_param;
    errcode_t ret;

    if (!s_connected) {
        return;
    }

    conn_param.conn_id = s_conn_id;
    conn_param.interval_min = s_requested_conn_interval_min;
    conn_param.interval_max = s_requested_conn_interval_max;
    conn_param.max_latency = 0;
    conn_param.supervision_timeout = DEMO_SLE_SUPERVISION_TIMEOUT;
    ret = sle_update_connect_param(&conn_param);
    DEMO_INFO("Update conn_param(%s)=0x%x-0x%x timeout=0x%x ret=0x%x",
        reason ? reason : "state",
        s_requested_conn_interval_min,
        s_requested_conn_interval_max,
        DEMO_SLE_SUPERVISION_TIMEOUT,
        ret);
}

static errcode_t demo_sle_apply_default_connect_param(void)
{
    sle_default_connect_param_t param = {0};
    errcode_t ret;

    param.enable_filter_policy = 0;
    param.initiate_phys = 1;
    param.gt_negotiate = 1;
    param.scan_interval = DEMO_SLE_SEEK_INTERVAL;
    param.scan_window = DEMO_SLE_SEEK_WINDOW;
    param.min_interval = s_requested_conn_interval_min;
    param.max_interval = s_requested_conn_interval_max;
    param.timeout = DEMO_SLE_SUPERVISION_TIMEOUT;

    ret = sle_default_connection_param_set(&param);
    DEMO_INFO("Default conn param: scan=%u/%u interval=0x%x-0x%x timeout=0x%x ret=0x%x",
        param.scan_interval, param.scan_window,
        param.min_interval, param.max_interval, param.timeout, ret);
    return ret;
}

static void demo_sle_request_low_latency(const char *reason)
{
#if DEMO_SLE_LOW_LATENCY_ENABLE && DEMO_SLE_LOW_LATENCY_AVAILABLE
    errcode_t ret;

    if (!s_connected) {
        return;
    }

    ret = sle_low_latency_set(s_conn_id, SLE_LOW_LATENCY_ENABLE, DEMO_SLE_LOW_LATENCY_RATE);
    DEMO_INFO("[LOW LATENCY] request(%s): enable=%u rate=%u (%u Hz) ret=0x%x",
        reason ? reason : "state",
        SLE_LOW_LATENCY_ENABLE,
        DEMO_SLE_LOW_LATENCY_RATE,
        demo_sle_rate_to_hz(DEMO_SLE_LOW_LATENCY_RATE),
        ret);
#elif DEMO_SLE_LOW_LATENCY_ENABLE
    DEMO_INFO("[LOW LATENCY] request(%s): feature not linked, keep normal ACB scheduling",
        reason ? reason : "state");
#else
    (void)reason;
#endif
}

static void demo_sle_log_transport(const char *reason)
{
    DEMO_INFO("[TRANSPORT] %s req_int=0x%x-0x%x nego_int=0x%x mtu=%u req_data=%u eff=%u frag=%u ack=%u ll=%u/%uHz",
        reason ? reason : "state",
        s_requested_conn_interval_min, s_requested_conn_interval_max,
        g_demo_stats.sle_negotiated_conn_interval,
        s_mtu_size, s_requested_data_len, s_effective_payload, s_fragment_payload,
        DEMO_SLE_FRAME_ACK_ENABLE,
        g_demo_stats.sle_low_latency_status, demo_sle_rate_to_hz(g_demo_stats.sle_low_latency_rate));
}

static bool demo_sle_server_can_send(void)
{
    if (!s_connected || !s_paired) {
        return false;
    }
#if IS_SLE_SERVER
    return s_property_handle != 0U;
#else
    return s_client_write_handle != 0U;
#endif
}

static void demo_sle_reset_tx_state(void)
{
    demo_transport_reset_tx(&s_tx_state);
    g_demo_stats.sle_tx_start_us = 0;
}

static void demo_sle_reset_ack_state(void)
{
    demo_transport_reset_ack(&s_ack_state);
}

static void demo_sle_reset_peer_discovery(void)
{
    s_client_ready = false;
    s_peer_service_start = 0;
    s_peer_service_end = 0;
}

static void demo_sle_clear_duplicate_state(void)
{
    s_rx_state.duplicate_active = false;
    s_rx_state.duplicate_frame_id = 0;
    s_rx_state.duplicate_frag_count = 0;
    s_rx_state.duplicate_next_frag_idx = 0;
}

static void demo_sle_reset_rx_state(const char *reason)
{
    uint16_t last_completed_frame_id = s_rx_state.last_completed_frame_id;
    bool last_completed_valid = s_rx_state.last_completed_valid;
    bool duplicate_active = s_rx_state.duplicate_active;
    uint16_t duplicate_frame_id = s_rx_state.duplicate_frame_id;
    uint8_t duplicate_frag_count = s_rx_state.duplicate_frag_count;
    uint8_t duplicate_next_frag_idx = s_rx_state.duplicate_next_frag_idx;

    if (s_rx_state.active && reason != NULL) {
        STATS_INC(sle_reassembly_reset_cnt);
        DEMO_ERR("Reassembly reset: %s (id=%u rx=%u total=%u)",
            reason, s_rx_state.frame_id, s_rx_state.received_len, s_rx_state.total_len);
    }

    demo_transport_reset_rx(&s_rx_state);
    s_rx_state.last_completed_frame_id = last_completed_frame_id;
    s_rx_state.last_completed_valid = last_completed_valid;
    s_rx_state.duplicate_active = duplicate_active;
    s_rx_state.duplicate_frame_id = duplicate_frame_id;
    s_rx_state.duplicate_frag_count = duplicate_frag_count;
    s_rx_state.duplicate_next_frag_idx = duplicate_next_frag_idx;
}

static void demo_sle_schedule_control(uint8_t packet_type, uint16_t frame_id, const char *reason)
{
    (void)packet_type;
    (void)frame_id;
    (void)reason;
}

static void demo_sle_record_sle_delivery(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint8_t frag_count, uint32_t first_rx_us, uint32_t reassembly_us, uint32_t max_gap_us)
{
    uint32_t ready_timestamp_us = (uint32_t)uapi_systick_get_us();
#if DEMO_DEBUG_LOG
    uint32_t rx_to_queue_us = 0;

    if (first_rx_us > 0U && ready_timestamp_us >= first_rx_us) {
        rx_to_queue_us = ready_timestamp_us - first_rx_us;
    }
#endif
    STATS_ADD(sle_rx_bytes, len);
    STATS_INC(sle_rx_frames);
    DEMO_LOG("Peer SLE frame complete: id=%u len=%u reassembly=%u us max_gap=%u us",
        frame_id, len, reassembly_us, max_gap_us);
    if (demo_uart_tx_direct_frame(data, len, frame_id, first_rx_us, ready_timestamp_us, reassembly_us)) {
        STATS_INC(uart_tx_fast_path_hits);
        DEMO_LOG("Peer UART fast-path: id=%u len=%u rx_to_queue=%u us", frame_id, len, rx_to_queue_us);
        return;
    }

    STATS_INC(uart_tx_fast_path_misses);
    if (!demo_uart_queue_tx_frame(data, len, frame_id, first_rx_us, ready_timestamp_us, reassembly_us)) {
        STATS_INC(sle_reassembly_drop_frames);
        STATS_ADD(sle_rx_drop_bytes, len);
        DEMO_ERR("UART queue drop after SLE complete: id=%u len=%u", frame_id, len);
        return;
    }
    DEMO_LOG("Peer UART enqueue: id=%u len=%u rx_to_queue=%u us", frame_id, len, rx_to_queue_us);
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_RX);
}

static void demo_sle_complete_active_frame(uint16_t frame_id, bool acked)
{
    const demo_frame_slot_t *frame = s_tx_state.frame;
    uint32_t now_us = (uint32_t)uapi_systick_get_us();

    if (!s_tx_state.active || frame == NULL || s_tx_state.frame_id != frame_id) {
        DEMO_LOG("ACK ignored: id=%u active=%u current=%u",
            frame_id, s_tx_state.active ? 1U : 0U, s_tx_state.frame_id);
        return;
    }

    if (acked) {
        STATS_INC(sle_ack_received);
    }
    STATS_ADD(sle_tx_bytes, frame->len);
    STATS_INC(sle_tx_frames);
    if (g_demo_stats.sle_tx_start_us > 0U && now_us >= g_demo_stats.sle_tx_start_us) {
        uint32_t rtt_us = now_us - g_demo_stats.sle_tx_start_us;
        STATS_UPDATE_RANGE(sle_rtt_us_min, sle_rtt_us_max, sle_rtt_us_sum, sle_rtt_count, rtt_us);
        DEMO_LOG("Peer SLE delivery %s: id=%u len=%u retries=%u rtt=%u us",
            acked ? "ACK" : "indication-complete",
            frame_id, frame->len, s_tx_state.retry_count, rtt_us);
    } else {
        DEMO_LOG("Peer SLE delivery %s: id=%u len=%u retries=%u",
            acked ? "ACK" : "indication-complete",
            frame_id, frame->len, s_tx_state.retry_count);
    }
    demo_uart_rx_frame_consume();
    demo_sle_reset_tx_state();
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
}

static bool demo_sle_handle_duplicate_frame(const demo_sle_frag_hdr_t *hdr)
{
    if (hdr == NULL || !s_rx_state.last_completed_valid || hdr->frame_id != s_rx_state.last_completed_frame_id) {
        return false;
    }

    if (hdr->frag_idx == 0U) {
        STATS_INC(sle_rx_duplicate_frames);
        s_rx_state.duplicate_active = true;
        s_rx_state.duplicate_frame_id = hdr->frame_id;
        s_rx_state.duplicate_frag_count = hdr->frag_count;
        s_rx_state.duplicate_next_frag_idx = 1U;
        DEMO_LOG("Duplicate frame seen: id=%u frag_count=%u", hdr->frame_id, hdr->frag_count);
        if (hdr->frag_count <= 1U) {
            demo_sle_clear_duplicate_state();
        }
        return true;
    }

    if (s_rx_state.duplicate_active &&
        s_rx_state.duplicate_frame_id == hdr->frame_id &&
        s_rx_state.duplicate_frag_count == hdr->frag_count) {
        if (hdr->frag_idx == s_rx_state.duplicate_next_frag_idx) {
            s_rx_state.duplicate_next_frag_idx = (uint8_t)(hdr->frag_idx + 1U);
        }
        if ((uint8_t)(hdr->frag_idx + 1U) >= hdr->frag_count) {
            demo_sle_clear_duplicate_state();
        }
        return true;
    }

    return false;
}

static void demo_sle_handle_peer_reset(uint16_t frame_id)
{
    DEMO_ERR("Peer reset requested: frame_id=%u", frame_id);
    if (s_rx_state.active && (frame_id == 0U || s_rx_state.frame_id == frame_id)) {
        demo_sle_reset_rx_state("peer reset");
    }
    if (s_tx_state.active && s_tx_state.frame_id == frame_id) {
        demo_sle_retry_active_frame("peer reset");
    }
}

static void demo_sle_handle_rx_payload(const uint8_t *data, uint16_t len)
{
    demo_sle_frag_hdr_t hdr;
    const uint8_t *payload;
    uint16_t payload_len;
    uint32_t rx_time_us = (uint32_t)uapi_systick_get_us();

    if (data == NULL || len == 0U) {
        return;
    }

    g_demo_stats.sle_rx_timestamp_us = rx_time_us;

    if (!demo_sle_frag_decode(data, len, &hdr)) {
        if (len <= DEMO_LOGICAL_FRAME_MAX_SIZE &&
            demo_logical_frame_header_valid(data) &&
            demo_logical_frame_total_len(data) == len) {
            demo_sle_record_sle_delivery(data, len, 0, 1U, rx_time_us, 0, 0);
            return;
        }
        STATS_ADD(sle_rx_drop_bytes, len);
        DEMO_ERR("Unexpected SLE payload dropped: len=%u", len);
        return;
    }

    payload = data + DEMO_SLE_FRAG_HEADER_SIZE;
    payload_len = (uint16_t)(len - DEMO_SLE_FRAG_HEADER_SIZE);

    switch (hdr.type) {
        case DEMO_SLE_PKT_TYPE_ACK:
            demo_sle_complete_active_frame(hdr.frame_id, true);
            return;
        case DEMO_SLE_PKT_TYPE_RESET:
            demo_sle_handle_peer_reset(hdr.frame_id);
            return;
        case DEMO_SLE_PKT_TYPE_DATA:
            break;
        default:
            STATS_ADD(sle_rx_drop_bytes, len);
            DEMO_ERR("Unknown transport packet dropped: type=%u len=%u", hdr.type, len);
            return;
    }

    STATS_INC(sle_rx_fragments);
    if (demo_sle_handle_duplicate_frame(&hdr)) {
        return;
    }

    if (hdr.total_len == 0U || hdr.total_len > DEMO_LOGICAL_FRAME_MAX_SIZE ||
        hdr.frag_count == 0U || hdr.frag_idx >= hdr.frag_count ||
        hdr.frag_offset > hdr.total_len ||
        (uint32_t)hdr.frag_offset + payload_len > hdr.total_len) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_schedule_control(DEMO_SLE_PKT_TYPE_RESET, hdr.frame_id, "invalid_header");
        demo_sle_reset_rx_state("invalid header");
        return;
    }

    if (hdr.frag_idx == 0U) {
        if (s_rx_state.active) {
            if (s_rx_state.frame_id != hdr.frame_id) {
                demo_sle_schedule_control(DEMO_SLE_PKT_TYPE_RESET, s_rx_state.frame_id, "new_frame_before_completion");
            }
            demo_sle_reset_rx_state((s_rx_state.frame_id == hdr.frame_id) ? NULL : "new frame before completion");
        }
        s_rx_state.active = true;
        s_rx_state.frame_id = hdr.frame_id;
        s_rx_state.total_len = hdr.total_len;
        s_rx_state.received_len = 0;
        s_rx_state.frag_count = hdr.frag_count;
        s_rx_state.next_frag_idx = 0;
        s_rx_state.first_fragment_us = rx_time_us;
        s_rx_state.last_fragment_us = rx_time_us;
        s_rx_state.max_gap_us = 0;
        demo_sle_clear_duplicate_state();
    }

    if (!s_rx_state.active ||
        s_rx_state.frame_id != hdr.frame_id ||
        s_rx_state.total_len != hdr.total_len ||
        s_rx_state.frag_count != hdr.frag_count ||
        s_rx_state.next_frag_idx != hdr.frag_idx ||
        s_rx_state.received_len != hdr.frag_offset) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_schedule_control(DEMO_SLE_PKT_TYPE_RESET, hdr.frame_id, "fragment_sequence_mismatch");
        demo_sle_reset_rx_state("fragment sequence mismatch");
        return;
    }

    if (hdr.frag_idx > 0U && rx_time_us >= s_rx_state.last_fragment_us) {
        uint32_t gap_us = rx_time_us - s_rx_state.last_fragment_us;
        STATS_UPDATE_RANGE(intra_frame_gap_us_min, intra_frame_gap_us_max,
            intra_frame_gap_us_sum, intra_frame_gap_count, gap_us);
        if (gap_us > s_rx_state.max_gap_us) {
            s_rx_state.max_gap_us = gap_us;
        }
    }

    if (memcpy_s(s_rx_state.buffer + hdr.frag_offset,
        sizeof(s_rx_state.buffer) - hdr.frag_offset, payload, payload_len) != EOK) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_schedule_control(DEMO_SLE_PKT_TYPE_RESET, hdr.frame_id, "payload_copy_failed");
        demo_sle_reset_rx_state("payload copy failed");
        return;
    }

    s_rx_state.received_len = (uint16_t)(hdr.frag_offset + payload_len);
    s_rx_state.next_frag_idx = (uint8_t)(hdr.frag_idx + 1U);
    s_rx_state.last_fragment_us = rx_time_us;

    if ((uint8_t)(hdr.frag_idx + 1U) == hdr.frag_count) {
        uint32_t reassembly_us = (rx_time_us >= s_rx_state.first_fragment_us) ?
            (rx_time_us - s_rx_state.first_fragment_us) : 0U;
        if (s_rx_state.received_len != s_rx_state.total_len) {
            STATS_ADD(sle_rx_drop_bytes, payload_len);
            demo_sle_schedule_control(DEMO_SLE_PKT_TYPE_RESET, hdr.frame_id, "length_mismatch");
            demo_sle_reset_rx_state("length mismatch");
            return;
        }

        s_rx_state.last_completed_frame_id = s_rx_state.frame_id;
        s_rx_state.last_completed_valid = true;
        demo_sle_record_sle_delivery(s_rx_state.buffer, s_rx_state.total_len, s_rx_state.frame_id,
            s_rx_state.frag_count, s_rx_state.first_fragment_us, reassembly_us, s_rx_state.max_gap_us);
        demo_sle_reset_rx_state(NULL);
    }
}

static void on_connect_state_changed(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    errcode_t ret;

    DEMO_INFO("Connect state: conn_id=%u state=%d pair_state=%d disc_reason=0x%x",
        conn_id, conn_state, pair_state, disc_reason);

    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        sle_set_phy_t phy_param = {
            .tx_format = SLE_RADIO_FRAME_2,
            .rx_format = SLE_RADIO_FRAME_2,
            .tx_phy = SLE_PHY_4M,
            .rx_phy = SLE_PHY_4M,
            .tx_pilot_density = SLE_PHY_PILOT_DENSITY_16_TO_1,
            .rx_pilot_density = SLE_PHY_PILOT_DENSITY_16_TO_1,
            .g_feedback = 0,
            .t_feedback = 0,
        };
#if IS_SLE_CLIENT
        ssap_exchange_info_t info = { .mtu_size = DEMO_SLE_MTU_SIZE, .version = 1 };
#endif

        s_conn_id = conn_id;
        s_connected = true;
        s_paired = (pair_state == SLE_PAIR_PAIRED);
        demo_sle_reset_peer_discovery();
        s_seek_connect_pending = false;
        demo_sle_reset_tx_state();
        demo_sle_reset_ack_state();
        demo_sle_reset_rx_state(NULL);
        demo_uart_reset_queues();  /* Clear accumulated UART RX data before new connection */
        s_requested_conn_interval_min = demo_sle_clamp_link_interval(DEMO_SLE_CONN_INTV_MIN);
        s_requested_conn_interval_max = demo_sle_clamp_link_interval(DEMO_SLE_CONN_INTV_MAX);
        if (s_requested_conn_interval_max < s_requested_conn_interval_min) {
            s_requested_conn_interval_max = s_requested_conn_interval_min;
        }
        s_requested_data_len = DEMO_SLE_DATA_LEN_TARGET;
        demo_sle_refresh_payload_limits();

        ret = sle_set_data_len(conn_id, s_requested_data_len);
        DEMO_INFO("Set data_len=%u: ret=0x%x", s_requested_data_len, ret);
        demo_sle_request_conn_param_update("connected");

        ret = sle_set_phy_param(conn_id, &phy_param);
        DEMO_INFO("Set PHY 4M: ret=0x%x", ret);
        ret = sle_set_mcs(conn_id, 10);
        DEMO_INFO("Set MCS=10: ret=0x%x", ret);
        demo_sle_request_low_latency("connected");
#if IS_SLE_CLIENT
        if (addr != NULL && pair_state == SLE_PAIR_NONE) {
            ret = sle_pair_remote_device(addr);
            DEMO_INFO("Pair request: conn_id=%u ret=0x%x", conn_id, ret);
        } else if (pair_state == SLE_PAIR_PAIRED) {
            (void)ssapc_exchange_info_req(0, conn_id, &info);
            DEMO_INFO("Peer already paired, start service discovery");
        }
        (void)sle_stop_seek();
#endif
        demo_sle_log_transport("connected");
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        if (conn_id == s_conn_id) {
            s_conn_id = 0;
            s_connected = false;
            s_paired = false;
            demo_sle_reset_peer_discovery();
            s_client_write_handle = 0;
            s_client_write_param.handle = 0;
            s_client_write_param.data = NULL;
            s_client_write_param.data_len = 0;
        }
        demo_sle_reset_tx_state();
        demo_sle_reset_ack_state();
        demo_sle_reset_rx_state(NULL);
        demo_uart_reset_queues();
        DEMO_INFO("SLE Disconnected");
#if IS_SLE_SERVER
        (void)sle_start_announce(DEMO_SLE_ADV_HANDLE);
#else
        osal_msleep(100);
        (void)client_start_scan();
#endif
    }
}

static void on_connect_param_update_req(uint16_t conn_id, errcode_t status,
    const sle_connection_param_update_req_t *param)
{
    if (param == NULL) {
        DEMO_ERR("Conn param req: null param status=0x%x", status);
        return;
    }

    DEMO_INFO("[CONN REQ] conn_id=%u interval=0x%x-0x%x latency=%u timeout=%u0ms status=0x%x",
        conn_id, param->interval_min, param->interval_max,
        param->max_latency, param->supervision_timeout, status);
}

static void on_connect_param_update(uint16_t conn_id, errcode_t status,
    const sle_connection_param_update_evt_t *param)
{
    uint32_t interval_us;

    if (param == NULL) {
        DEMO_ERR("Conn param update: null param status=0x%x", status);
        return;
    }

    g_demo_stats.sle_negotiated_conn_interval = param->interval;
    interval_us = demo_sle_link_interval_to_us(param->interval);
    DEMO_INFO("[CONN PARAM] conn_id=%u interval=0x%x (%u.%03u ms) latency=%u timeout=%u0ms status=0x%x",
        conn_id, param->interval, interval_us / 1000U, interval_us % 1000U,
        param->latency, param->supervision, status);
    demo_sle_log_transport("param_update");
}

static void on_low_latency_set(uint8_t status, sle_addr_t *addr, uint8_t rate)
{
    (void)addr;
    g_demo_stats.sle_low_latency_status = status;
    g_demo_stats.sle_low_latency_rate = rate;
    DEMO_INFO("[LOW LATENCY] status=%u rate=%u (%u Hz)",
        status, rate, demo_sle_rate_to_hz(rate));
}

static void on_pair_complete(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    ssap_exchange_info_t info = { .mtu_size = DEMO_SLE_MTU_SIZE, .version = 1 };

    (void)addr;
    DEMO_INFO("Pair complete: conn_id=%u status=0x%x", conn_id, status);
    if (status != ERRCODE_SLE_SUCCESS) {
        return;
    }

    s_paired = true;
#if IS_SLE_SERVER
    ssaps_set_info(s_server_id, &info);
    s_mtu_size = DEMO_SLE_MTU_SIZE;
    demo_sle_refresh_payload_limits();
    demo_sle_request_conn_param_update("pair");
    demo_sle_request_low_latency("pair");
#else
    demo_sle_request_conn_param_update("pair");
    demo_sle_request_low_latency("pair");
    (void)ssapc_exchange_info_req(0, conn_id, &info);
#endif
    demo_sle_log_transport("pair");
}

static void on_announce_enable(uint32_t announce_id, errcode_t status) { DEMO_INFO("Announce enabled: id=%u status=0x%x", announce_id, status); }
static void on_announce_disable(uint32_t announce_id, errcode_t status) { DEMO_INFO("Announce disabled: id=%u status=0x%x", announce_id, status); }

static void demo_sle_fail_active_frame(const char *reason)
{
    const demo_frame_slot_t *frame = s_tx_state.frame;

    if (!s_tx_state.active || frame == NULL) {
        return;
    }

    STATS_INC(sle_tx_hard_fail);
    DEMO_ERR("SLE TX hard fail: id=%u len=%u reason=%s retries=%u",
        s_tx_state.frame_id, frame->len, reason ? reason : "state", s_tx_state.retry_count);
    demo_uart_rx_frame_consume();
    demo_sle_reset_tx_state();
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
}

static void demo_sle_retry_active_frame(const char *reason)
{
    if (!s_tx_state.active) {
        return;
    }

    if (s_tx_state.retry_count >= DEMO_SLE_MAX_RETRIES) {
        demo_sle_fail_active_frame(reason);
        return;
    }

    s_tx_state.retry_count++;
    s_tx_state.offset = 0;
    s_tx_state.last_payload_len = 0;
    s_tx_state.frag_idx = 0;
    s_tx_state.phase = DEMO_TRANSPORT_TX_SEND_FRAGMENT;
    s_tx_state.in_flight_type = 0;
    s_tx_state.deadline_us = 0;
    STATS_INC(sle_tx_retry_frames);
    DEMO_ERR("SLE TX retry: id=%u reason=%s retry=%u/%u",
        s_tx_state.frame_id, reason ? reason : "state",
        s_tx_state.retry_count, DEMO_SLE_MAX_RETRIES);
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
}

static void on_indicate_cfm(uint8_t server_id, uint16_t conn_id, sle_indication_cfm_result_t cfm_result, errcode_t status)
{
    (void)server_id;
    (void)conn_id;
    (void)cfm_result;
    (void)status;
}
static void on_server_read_request(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb, errcode_t status)
{ (void)server_id; (void)conn_id; (void)read_cb; (void)status; }

static void on_mtu_changed(uint8_t server_id, uint16_t conn_id, ssap_exchange_info_t *mtu_info, errcode_t status)
{
    uint16_t new_mtu;
    (void)server_id;
    (void)conn_id;
    if (mtu_info == NULL) {
        DEMO_ERR("MTU changed: null info status=0x%x", status);
        return;
    }
    new_mtu = mtu_info->mtu_size;
    if (new_mtu < 64U) new_mtu = 64U;
    if (new_mtu > DEMO_SLE_MTU_SIZE) new_mtu = DEMO_SLE_MTU_SIZE;
    s_mtu_size = new_mtu;
    demo_sle_refresh_payload_limits();
    demo_sle_log_transport("server_mtu");
}

static void on_server_write_request(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb, errcode_t status)
{
    (void)server_id;
    (void)conn_id;
    (void)status;
    if (write_cb == NULL || write_cb->length == 0U || write_cb->value == NULL) {
        return;
    }
    DEMO_LOG("Server write RX: handle=%u len=%u need_rsp=%u",
        write_cb->handle, write_cb->length, write_cb->need_rsp);
    demo_sle_handle_rx_payload(write_cb->value, write_cb->length);
    if (write_cb->need_rsp) {
        ssaps_send_rsp_t rsp = {
            .request_id = write_cb->request_id,
            .status = ERRCODE_SLE_SUCCESS,
            .value_len = 0,
            .value = NULL
        };
        ssaps_send_response(server_id, conn_id, &rsp);
    }
}

static errcode_t demo_register_callbacks(void)
{
    errcode_t ret;
    sle_announce_seek_callbacks_t announce_cbks = {
        .sle_enable_cb = on_sle_enable,
#if IS_SLE_SERVER
        .announce_enable_cb = on_announce_enable,
        .announce_disable_cb = on_announce_disable,
#else
        .seek_enable_cb = on_seek_enable,
        .seek_disable_cb = on_seek_disable,
        .seek_result_cb = on_seek_result,
#endif
    };
    sle_connection_callbacks_t conn_cbks = {
        .connect_state_changed_cb = on_connect_state_changed,
        .connect_param_update_req_cb = on_connect_param_update_req,
        .connect_param_update_cb = on_connect_param_update,
        .pair_complete_cb = on_pair_complete,
        .low_latency_cb = on_low_latency_set,
    };
    ssaps_callbacks_t ssaps_cbks = {
        .mtu_changed_cb = on_mtu_changed,
        .read_request_cb = on_server_read_request,
        .write_request_cb = on_server_write_request,
        .indicate_cfm_cb = on_indicate_cfm,
    };
    ssapc_callbacks_t ssapc_cbks = {
        .exchange_info_cb = on_exchange_info,
        .find_structure_cb = on_find_structure,
        .ssapc_find_property_cbk = on_find_property,
        .notification_cb = on_notification,
        .indication_cb = on_notification,
    };

    ret = sle_announce_seek_register_callbacks(&announce_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
    ret = sle_connection_register_callbacks(&conn_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
#if IS_SLE_SERVER
    ret = ssaps_register_callbacks(&ssaps_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
    return ERRCODE_SLE_SUCCESS;
#else
    return ssapc_register_callbacks(&ssapc_cbks);
#endif
}

static errcode_t server_add_service(void)
{
    errcode_t ret;
    sle_uuid_t uuid;
    ssaps_property_info_t property = {0};
    ssaps_desc_info_t descriptor = {0};
    uint8_t cccd_value[] = {0x01, 0x00};
    sle_uuid_t app_uuid = { .len = 2, .uuid = {0x12, 0x34} };

    ret = ssaps_register_server(&app_uuid, &s_server_id);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
    sle_uuid_set_u2(SLE_UUID_SERVER_SERVICE, &uuid);
    ret = ssaps_add_service_sync(s_server_id, &uuid, 1, &s_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;

    property.permissions = SLE_UUID_TEST_PROPERTIES;
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ |
        SSAP_OPERATE_INDICATION_BIT_NOTIFY |
        SSAP_OPERATE_INDICATION_BIT_WRITE;
    sle_uuid_set_u2(SLE_UUID_SERVER_PROPERTY, &property.uuid);
    property.value = (uint8_t *)osal_vmalloc(8);
    if (property.value == NULL) return ERRCODE_SLE_FAIL;
    (void)memset_s(property.value, 8, 0, 8);
    ret = ssaps_add_property_sync(s_server_id, s_service_handle, &property, &s_property_handle);
    osal_vfree(property.value);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;

    descriptor.permissions = SLE_UUID_TEST_DESCRIPTOR;
    descriptor.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    descriptor.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    descriptor.value = cccd_value;
    descriptor.value_len = sizeof(cccd_value);
    (void)ssaps_add_descriptor_sync(s_server_id, s_service_handle, s_property_handle, &descriptor);
    ret = ssaps_start_service(s_server_id, s_service_handle);
    if (ret == ERRCODE_SLE_SUCCESS) {
        DEMO_INFO("Server service ready: server_id=%u service=%u property=%u", s_server_id, s_service_handle, s_property_handle);
    }
    return ret;
}

static errcode_t server_start_announce(void)
{
    errcode_t ret;
    uint8_t adv_data[32];
    uint8_t name[] = DEMO_SLE_SERVER_NAME;
    uint8_t name_len = (uint8_t)(sizeof(name) - 1U);
    sle_announce_param_t param = {
        .announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE,
        .announce_handle = DEMO_SLE_ADV_HANDLE,
        .announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO,
        .announce_level = SLE_ANNOUNCE_LEVEL_NORMAL,
        .announce_channel_map = SLE_ADV_CHANNEL_MAP_DEFAULT,
        .announce_interval_min = DEMO_SLE_ADV_INTERVAL_MIN,
        .announce_interval_max = DEMO_SLE_ADV_INTERVAL_MAX,
        .conn_interval_min = DEMO_SLE_ADV_CONN_INTV_MIN,
        .conn_interval_max = DEMO_SLE_ADV_CONN_INTV_MAX,
        .conn_max_latency = 0,
        .conn_supervision_timeout = DEMO_SLE_SUPERVISION_TIMEOUT,
        .announce_tx_power = 18,
    };
    sle_announce_data_t data;

    if (s_local_addr_valid) {
        param.own_addr.type = s_local_addr.type;
        (void)memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, s_local_addr.addr, SLE_ADDR_LEN);
    } else {
        param.own_addr.type = SLE_ADDRESS_TYPE_PUBLIC;
        (void)memset_s(param.own_addr.addr, SLE_ADDR_LEN, 0, SLE_ADDR_LEN);
        DEMO_ERR("Local addr invalid, announce will use zero address");
    }
    ret = sle_set_announce_param(DEMO_SLE_ADV_HANDLE, &param);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;

    adv_data[0] = name_len + 1U;
    adv_data[1] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    (void)memcpy_s(&adv_data[2], sizeof(adv_data) - 2U, name, name_len);
    data.announce_data = adv_data;
    data.announce_data_len = name_len + 2U;
    data.seek_rsp_data = adv_data;
    data.seek_rsp_data_len = name_len + 2U;
    ret = sle_set_announce_data(DEMO_SLE_ADV_HANDLE, &data);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
    return sle_start_announce(DEMO_SLE_ADV_HANDLE);
}

static errcode_t demo_sle_send_packet(const uint8_t *data, uint16_t len)
{
#if IS_SLE_SERVER
    ssaps_ntf_ind_t param = {0};
    uint8_t *buf = s_server_ntf_buf[s_server_ntf_idx];

    s_server_ntf_idx = (uint8_t)((s_server_ntf_idx + 1U) % 4U);
    if (memcpy_s(buf, DEMO_SLE_MTU_SIZE, data, len) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    param.handle = s_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = buf;
    param.value_len = len;
    return ssaps_notify_indicate(s_server_id, s_conn_id, &param);
#else
    uint8_t *buf = s_client_tx_buf[s_client_tx_idx];

    if (s_client_write_handle == 0U) {
        return ERRCODE_SLE_FAIL;
    }
    s_client_tx_idx = (uint8_t)((s_client_tx_idx + 1U) % 4U);
    if (memcpy_s(buf, DEMO_SLE_MTU_SIZE, data, len) != EOK) {
        return ERRCODE_SLE_FAIL;
    }
    s_client_write_param.handle = s_client_write_handle;
    s_client_write_param.type = SSAP_PROPERTY_TYPE_VALUE;
    s_client_write_param.data_len = len;
    s_client_write_param.data = buf;
    return ssapc_write_cmd(0, s_conn_id, &s_client_write_param);
#endif
}
static void on_sle_enable(errcode_t status) { DEMO_INFO("SLE enabled: status=0x%x", status); }
static void on_seek_enable(errcode_t status) { DEMO_INFO("Seek enabled: status=0x%x", status); }

static bool find_local_name_in_ad(const uint8_t *data, uint16_t data_len, const char *target_name, uint8_t target_len)
{
    uint16_t offset = 0;
    while (offset < data_len) {
        uint8_t field_len = data[offset];
        if (field_len == 0U || (uint32_t)offset + field_len >= data_len) break;
        if ((data[offset + 1U] == SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME || data[offset + 1U] == SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME) &&
            field_len > 1U && field_len - 1U >= target_len && memcmp(&data[offset + 2U], target_name, target_len) == 0) {
            return true;
        }
        offset = (uint16_t)(offset + field_len + 1U);
    }
    return false;
}

static void on_seek_result(sle_seek_result_info_t *result)
{
    static const char target_name[] = DEMO_SLE_SERVER_NAME;
    if (result == NULL || result->data == NULL || result->data_length == 0U || s_connected || s_seek_connect_pending) return;
    if (!find_local_name_in_ad(result->data, result->data_length, target_name, (uint8_t)(sizeof(target_name) - 1U))) return;
    DEMO_LOG("Seek result peer=%02x:**:**:**:%02x:%02x type=%u len=%u",
        result->addr.addr[0], result->addr.addr[4], result->addr.addr[5],
        result->addr.type, result->data_length);
    if (!demo_sle_should_initiate_connect(&result->addr)) {
        DEMO_LOG("Seek result ignored by address arbitration");
        return;
    }
    DEMO_INFO("Found target server, connecting...");
    (void)memcpy_s(&s_remote_addr, sizeof(sle_addr_t), &result->addr, sizeof(sle_addr_t));
    s_seek_connect_pending = true;
    sle_stop_seek();
}

static void on_seek_disable(errcode_t status)
{
    DEMO_INFO("Seek disabled: status=0x%x", status);
    if (status == ERRCODE_SLE_SUCCESS && s_seek_connect_pending) {
        s_seek_connect_pending = false;
        sle_remove_paired_remote_device(&s_remote_addr);
        sle_connect_remote_device(&s_remote_addr);
    }
}

static void on_exchange_info(uint8_t client_id, uint16_t conn_id, ssap_exchange_info_t *info, errcode_t status)
{
    ssapc_find_structure_param_t find = { .type = SSAP_FIND_TYPE_PRIMARY_SERVICE, .start_hdl = 1, .end_hdl = 0xFFFF };
    (void)client_id;
    if (info == NULL) {
        DEMO_ERR("Exchange info: null info status=0x%x", status);
        return;
    }
    s_mtu_size = info->mtu_size;
    if (s_mtu_size < 64U) s_mtu_size = 64U;
    if (s_mtu_size > DEMO_SLE_MTU_SIZE) s_mtu_size = DEMO_SLE_MTU_SIZE;
    demo_sle_refresh_payload_limits();
    demo_sle_log_transport("client_mtu");
    (void)ssapc_find_structure(0, conn_id, &find);
}

static void on_find_structure(uint8_t client_id, uint16_t conn_id, ssapc_find_service_result_t *service, errcode_t status)
{
    ssapc_find_structure_param_t find_property = {0};

    (void)client_id;
    if (status != ERRCODE_SLE_SUCCESS || service == NULL) {
        return;
    }
    if (!demo_sle_uuid_matches_u2(&service->uuid, SLE_UUID_SERVER_SERVICE)) {
        return;
    }

    s_peer_service_start = service->start_hdl;
    s_peer_service_end = service->end_hdl;
    DEMO_INFO("Peer service found: start=%u end=%u", s_peer_service_start, s_peer_service_end);

    find_property.type = SSAP_FIND_TYPE_PROPERTY;
    find_property.start_hdl = service->start_hdl;
    find_property.end_hdl = service->end_hdl;
    (void)ssapc_find_structure(0, conn_id, &find_property);
}

static void on_find_property(uint8_t client_id, uint16_t conn_id, ssapc_find_property_result_t *property, errcode_t status)
{
    (void)client_id;
    (void)conn_id;
    if (status != ERRCODE_SLE_SUCCESS || property == NULL || property->handle == 0U) {
        DEMO_ERR("Find property: invalid property");
        return;
    }
    if (!demo_sle_uuid_matches_u2(&property->uuid, SLE_UUID_SERVER_PROPERTY)) {
        return;
    }
    if (s_peer_service_start != 0U &&
        (property->handle < s_peer_service_start || property->handle > s_peer_service_end)) {
        return;
    }
    if ((property->operate_indication &
        (SSAP_OPERATE_INDICATION_BIT_WRITE | SSAP_OPERATE_INDICATION_BIT_WRITE_NO_RSP)) == 0U) {
        return;
    }
    s_client_write_handle = property->handle;
    s_client_write_param.handle = property->handle;
    s_client_write_param.type = SSAP_PROPERTY_TYPE_VALUE;
    s_client_ready = true;
    DEMO_INFO("Client TX ready: handle=%u mtu=%u", s_client_write_handle, s_mtu_size);
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
}

static void on_notification(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data, errcode_t status)
{
    (void)client_id;
    (void)conn_id;
    (void)status;
    if (data == NULL || data->data == NULL || data->data_len == 0U) return;
    demo_sle_handle_rx_payload(data->data, data->data_len);
}

static errcode_t client_start_scan(void)
{
    sle_seek_param_t param = { .own_addr_type = 0, .filter_duplicates = 0, .seek_filter_policy = 0, .seek_phys = 1 };
    param.seek_type[0] = 1;
    param.seek_interval[0] = DEMO_SLE_SEEK_INTERVAL;
    param.seek_window[0] = DEMO_SLE_SEEK_WINDOW;
    if (sle_set_seek_param(&param) != ERRCODE_SLE_SUCCESS) return ERRCODE_SLE_FAIL;
    return sle_start_seek();
}

int demo_sle_init(void)
{
    errcode_t ret;
    s_requested_conn_interval_min = demo_sle_clamp_link_interval(DEMO_SLE_CONN_INTV_MIN);
    s_requested_conn_interval_max = demo_sle_clamp_link_interval(DEMO_SLE_CONN_INTV_MAX);
    if (s_requested_conn_interval_max < s_requested_conn_interval_min) {
        s_requested_conn_interval_max = s_requested_conn_interval_min;
    }
    demo_sle_refresh_payload_limits();
    DEMO_INFO("SLE init step: enable_sle begin");
    ret = enable_sle();
    DEMO_INFO("SLE init step: enable_sle ret=0x%x", ret);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("SLE enable failed: 0x%x", ret);
        return -1;
    }
    demo_sle_refresh_local_addr();
    (void)demo_sle_apply_default_connect_param();
    DEMO_INFO("Initializing as SLE %s", IS_SLE_SERVER ? "Server" : "Client");
    DEMO_INFO("SLE init step: register callbacks");
    ret = demo_register_callbacks();
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SLE register callbacks failed: 0x%x", ret);
        return -1;
    }
#if IS_SLE_SERVER
    ret = server_add_service();
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SLE add service failed: 0x%x", ret);
        return -1;
    }
    ret = server_start_announce();
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SLE start announce failed: 0x%x", ret);
        return -1;
    }
#else
    osal_msleep(1000);
    ret = client_start_scan();
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SLE start scan failed: 0x%x", ret);
        return -1;
    }
#endif
    DEMO_INFO("SLE init OK");
    return 0;
}

bool demo_sle_is_connected(void)
{
#if IS_SLE_SERVER
    return s_connected && s_paired;
#else
    return s_connected && s_paired && (s_client_write_handle != 0U);
#endif
}

uint16_t demo_sle_get_mtu(void) { return s_mtu_size; }
uint16_t demo_sle_get_conn_id(void) { return s_conn_id; }
uint16_t demo_sle_get_effective_payload(void) { return s_effective_payload; }
uint16_t demo_sle_get_fragment_payload(void) { return s_fragment_payload; }

static bool demo_sle_prepare_tx_frame(void)
{
    const demo_frame_slot_t *frame = demo_uart_rx_frame_peek();

    if (frame == NULL) {
        return false;
    }

    s_tx_state.active = true;
    s_tx_state.frame = frame;
    s_tx_state.frame_id = (frame->frame_id != 0U) ? frame->frame_id : 1U;
    s_tx_state.offset = 0;
    s_tx_state.last_payload_len = 0;
    s_tx_state.frag_idx = 0;
    s_tx_state.frag_count = demo_sle_calc_frag_count(frame->len, s_fragment_payload);
    s_tx_state.retry_count = 0;
    s_tx_state.in_flight_type = 0;
    s_tx_state.phase = DEMO_TRANSPORT_TX_SEND_FRAGMENT;
    s_tx_state.deadline_us = 0;
    if (s_tx_state.frag_count == 0U) {
        DEMO_ERR("Frame too large for reliable transport: id=%u len=%u frag_payload=%u",
            s_tx_state.frame_id, frame->len, s_fragment_payload);
        demo_sle_fail_active_frame("invalid fragment count");
        return false;
    }
    return true;
}

static void demo_sle_complete_active_frame_local(void)
{
    const demo_frame_slot_t *frame = s_tx_state.frame;

    if (!s_tx_state.active || frame == NULL) {
        return;
    }

    STATS_ADD(sle_tx_bytes, frame->len);
    STATS_INC(sle_tx_frames);
    DEMO_LOG("SLE TX frame queued: id=%u len=%u frags=%u retries=%u",
        s_tx_state.frame_id, frame->len, s_tx_state.frag_count, s_tx_state.retry_count);
    demo_uart_rx_frame_consume();
    demo_sle_reset_tx_state();
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
}

static uint32_t demo_sle_send_control_packet(void)
{
    demo_sle_frag_hdr_t hdr = {0};
    errcode_t ret;

    if (!s_ack_state.pending || s_ack_state.waiting_cfm) {
        return 0;
    }

    hdr.type = s_ack_state.packet_type;
    hdr.frame_id = s_ack_state.frame_id;
    demo_sle_frag_encode(s_sle_tx_buf, &hdr);
    ret = demo_sle_send_packet(s_sle_tx_buf, DEMO_SLE_FRAG_HEADER_SIZE);
    if (ret != ERRCODE_SLE_SUCCESS) {
        STATS_INC(sle_tx_fail);
        STATS_INC(sle_tx_busy_cnt);
        DEMO_ERR("SLE control send failed: type=%u id=%u ret=0x%x",
            s_ack_state.packet_type, s_ack_state.frame_id, ret);
        return 0;
    }

    s_ack_state.pending = false;
    s_ack_state.waiting_cfm = true;
    s_ack_state.deadline_us = demo_transport_deadline_from_ms((uint32_t)uapi_systick_get_us(),
        DEMO_SLE_ACK_TIMEOUT_MS);
    DEMO_LOG("Control sent: type=%u id=%u", s_ack_state.packet_type, s_ack_state.frame_id);
    return DEMO_SLE_FRAG_HEADER_SIZE;
}

static uint32_t demo_sle_send_next_fragment(void)
{
    demo_sle_frag_hdr_t hdr = {0};
    const demo_frame_slot_t *frame = s_tx_state.frame;
    uint16_t remaining;
    uint16_t payload_len;
    errcode_t ret;
    uint32_t now_us;

    if (!s_tx_state.active || frame == NULL) {
        return 0;
    }

    remaining = (uint16_t)(frame->len - s_tx_state.offset);
    payload_len = (remaining > s_fragment_payload) ? s_fragment_payload : remaining;
    hdr.type = DEMO_SLE_PKT_TYPE_DATA;
    hdr.frame_id = s_tx_state.frame_id;
    hdr.total_len = frame->len;
    hdr.frag_offset = s_tx_state.offset;
    hdr.frag_idx = s_tx_state.frag_idx;
    hdr.frag_count = s_tx_state.frag_count;
    demo_sle_frag_encode(s_sle_tx_buf, &hdr);
    if (memcpy_s(s_sle_tx_buf + DEMO_SLE_FRAG_HEADER_SIZE,
        sizeof(s_sle_tx_buf) - DEMO_SLE_FRAG_HEADER_SIZE,
        frame->data + s_tx_state.offset, payload_len) != EOK) {
        demo_sle_fail_active_frame("fragment copy failed");
        return 0;
    }

    ret = demo_sle_send_packet(s_sle_tx_buf, (uint16_t)(payload_len + DEMO_SLE_FRAG_HEADER_SIZE));
    if (ret != ERRCODE_SLE_SUCCESS) {
        STATS_INC(sle_tx_fail);
        STATS_INC(sle_tx_busy_cnt);
        DEMO_ERR("SLE data send failed: id=%u frag=%u/%u len=%u ret=0x%x",
            s_tx_state.frame_id, (uint8_t)(hdr.frag_idx + 1U), hdr.frag_count, payload_len, ret);
        return 0;
    }

    now_us = (uint32_t)uapi_systick_get_us();
    if (s_tx_state.offset == 0U && frame->enqueue_timestamp_us > 0U && now_us >= frame->enqueue_timestamp_us) {
        uint32_t frame_delay_us = now_us - frame->enqueue_timestamp_us;
        g_demo_stats.sle_tx_start_us = now_us;
        STATS_UPDATE_RANGE(frame_tx_delay_us_min, frame_tx_delay_us_max,
            frame_tx_delay_us_sum, frame_tx_delay_count, frame_delay_us);
        DEMO_LOG("SLE TX frame start: id=%u len=%u frags=%u queue_delay=%u us eff=%u frag=%u",
            s_tx_state.frame_id, frame->len, s_tx_state.frag_count,
            frame_delay_us, s_effective_payload, s_fragment_payload);
    }

    STATS_INC(sle_tx_fragments);
    s_tx_state.offset = (uint16_t)(s_tx_state.offset + payload_len);
    s_tx_state.frag_idx = (uint8_t)(s_tx_state.frag_idx + 1U);
    s_tx_state.last_payload_len = 0;
    s_tx_state.in_flight_type = 0;
    s_tx_state.phase = DEMO_TRANSPORT_TX_SEND_FRAGMENT;
    s_tx_state.deadline_us = 0;
    if (s_tx_state.offset >= frame->len) {
        demo_sle_complete_active_frame_local();
    }
    return payload_len;
}

uint32_t demo_sle_tx_process(void)
{
    static uint8_t s_tx_backoff = 0;
    static uint16_t s_last_blocked_frame_id = 0;
    uint32_t total_sent = 0;
    uint8_t batch = 0;
    const demo_frame_slot_t *pending_frame = demo_uart_rx_frame_peek();

    if (!demo_sle_server_can_send()) {
        if (pending_frame != NULL && pending_frame->frame_id != s_last_blocked_frame_id) {
            DEMO_ERR("SLE TX blocked: id=%u len=%u connected=%u paired=%u ready=%u handle=%u",
                pending_frame->frame_id, pending_frame->len,
                s_connected ? 1U : 0U, s_paired ? 1U : 0U,
                s_client_ready ? 1U : 0U, s_client_write_handle);
            s_last_blocked_frame_id = pending_frame->frame_id;
        }
        s_tx_backoff = 0;
        demo_sle_reset_tx_state();
        demo_sle_reset_ack_state();
        return 0;
    }

    s_last_blocked_frame_id = 0;

    demo_sle_refresh_payload_limits();
    if (s_ack_state.pending && !s_ack_state.waiting_cfm) {
        return demo_sle_send_control_packet();
    }
    if (!demo_sle_is_connected() || s_fragment_payload == 0U) {
        return 0;
    }

    if (s_tx_backoff > 0U) {
        s_tx_backoff--;
        return 0;
    }

    while (batch < 4U) {
        uint32_t sent;

        demo_uart_rx_poll();
        if (!s_tx_state.active && !demo_sle_prepare_tx_frame()) {
            break;
        }

        sent = demo_sle_send_next_fragment();
        if (sent == 0U) {
            s_tx_backoff = 1U;
            break;
        }
        total_sent += sent;
        batch++;
    }

    return total_sent;
}

void demo_sle_process_deferred(void)
{
    return;
}

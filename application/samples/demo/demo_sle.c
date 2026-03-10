/**
 * @file demo_sle.c
 * @brief Frame-aware SLE Server/Client implementation.
 */

#include "demo_sle.h"
#include "demo_uart.h"
#include "demo_frame.h"
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
#define DEMO_SLE_TX_MAX_FRAG_PER_LOOP 8

typedef struct {
    bool active;
    const demo_frame_slot_t *frame;
    uint16_t frame_id;
    uint16_t offset;
    uint8_t frag_idx;
    uint8_t frag_count;
} demo_sle_tx_state_t;

typedef struct {
    bool active;
    uint16_t frame_id;
    uint16_t total_len;
    uint16_t received_len;
    uint8_t frag_count;
    uint8_t next_frag_idx;
    uint8_t buffer[DEMO_LOGICAL_FRAME_MAX_SIZE];
    uint32_t first_fragment_us;
    uint32_t last_fragment_us;
    uint32_t max_gap_us;
} demo_sle_reassembly_t;

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
static bool s_seek_connect_pending = false;
static bool s_local_addr_valid = false;
static uint16_t s_requested_conn_interval_min = DEMO_SLE_CONN_INTV_MIN;
static uint16_t s_requested_conn_interval_max = DEMO_SLE_CONN_INTV_MAX;
static uint16_t s_requested_data_len = DEMO_SLE_DATA_LEN_TARGET;
static uint16_t s_effective_payload = 0;
static uint16_t s_fragment_payload = 0;
static demo_sle_tx_state_t s_tx_state;
static demo_sle_reassembly_t s_rx_reassembly;
static uint8_t s_server_id = 0;
static uint16_t s_service_handle = 0;
static uint16_t s_property_handle = 0;
static uint16_t s_peer_service_start = 0;
static uint16_t s_peer_service_end = 0;
static uint8_t s_server_ntf_buf[4][DEMO_SLE_MTU_SIZE];
static volatile uint8_t s_server_ntf_idx = 0;
static uint16_t s_client_rx_handle = 0;
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
    if (s_local_addr_valid && demo_sle_addr_equal(&s_local_addr, remote)) {
        return false;
    }
    if (!s_local_addr_valid) {
        return true;
    }
    return demo_sle_addr_compare(&s_local_addr, remote) < 0;
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
    DEMO_INFO("[TRANSPORT] %s req_int=0x%x-0x%x nego_int=0x%x mtu=%u req_data=%u eff=%u frag=%u ll=%u/%uHz",
        reason ? reason : "state",
        s_requested_conn_interval_min, s_requested_conn_interval_max,
        g_demo_stats.sle_negotiated_conn_interval,
        s_mtu_size, s_requested_data_len, s_effective_payload, s_fragment_payload,
        g_demo_stats.sle_low_latency_status, demo_sle_rate_to_hz(g_demo_stats.sle_low_latency_rate));
}

static void demo_sle_reset_tx_state(void)
{
    (void)memset_s(&s_tx_state, sizeof(s_tx_state), 0, sizeof(s_tx_state));
}

static void demo_sle_reset_peer_discovery(void)
{
    s_client_ready = false;
    s_client_rx_handle = 0;
    s_peer_service_start = 0;
    s_peer_service_end = 0;
}

static void demo_sle_reset_reassembly(const char *reason)
{
    if (s_rx_reassembly.active && reason != NULL) {
        STATS_INC(sle_reassembly_reset_cnt);
        DEMO_ERR("Reassembly reset: %s (id=%u rx=%u total=%u)",
            reason, s_rx_reassembly.frame_id, s_rx_reassembly.received_len, s_rx_reassembly.total_len);
    }
    (void)memset_s(&s_rx_reassembly, sizeof(s_rx_reassembly), 0, sizeof(s_rx_reassembly));
}

static void demo_sle_queue_complete_frame(const uint8_t *data, uint16_t len, uint16_t frame_id,
    uint32_t first_rx_us, uint32_t reassembly_us, uint32_t max_gap_us)
{
    if (!demo_uart_queue_tx_frame(data, len, frame_id, first_rx_us)) {
        STATS_INC(sle_reassembly_drop_frames);
        STATS_ADD(sle_rx_drop_bytes, len);
        return;
    }

    STATS_ADD(sle_rx_bytes, len);
    STATS_INC(sle_rx_frames);
    DEMO_LOG("SLE RX frame complete: id=%u len=%u reassembly=%u us max_gap=%u us",
        frame_id, len, reassembly_us, max_gap_us);
    osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_RX);
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
            demo_sle_queue_complete_frame(data, len, 0, rx_time_us, 0, 0);
            return;
        }
        STATS_ADD(sle_rx_drop_bytes, len);
        DEMO_ERR("Unexpected SLE payload dropped: len=%u", len);
        return;
    }

    STATS_INC(sle_rx_fragments);
    payload = data + DEMO_SLE_FRAG_HEADER_SIZE;
    payload_len = (uint16_t)(len - DEMO_SLE_FRAG_HEADER_SIZE);

    if (hdr.total_len == 0U || hdr.total_len > DEMO_LOGICAL_FRAME_MAX_SIZE ||
        hdr.frag_count == 0U || hdr.frag_idx >= hdr.frag_count ||
        hdr.frag_offset > hdr.total_len ||
        (uint32_t)hdr.frag_offset + payload_len > hdr.total_len) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_reset_reassembly("invalid header");
        return;
    }

    if (hdr.frag_idx == 0U) {
        if (s_rx_reassembly.active) {
            demo_sle_reset_reassembly("new frame before completion");
        }
        s_rx_reassembly.active = true;
        s_rx_reassembly.frame_id = hdr.frame_id;
        s_rx_reassembly.total_len = hdr.total_len;
        s_rx_reassembly.received_len = 0;
        s_rx_reassembly.frag_count = hdr.frag_count;
        s_rx_reassembly.next_frag_idx = 0;
        s_rx_reassembly.first_fragment_us = rx_time_us;
        s_rx_reassembly.last_fragment_us = rx_time_us;
        s_rx_reassembly.max_gap_us = 0;
    }

    if (!s_rx_reassembly.active ||
        s_rx_reassembly.frame_id != hdr.frame_id ||
        s_rx_reassembly.total_len != hdr.total_len ||
        s_rx_reassembly.frag_count != hdr.frag_count ||
        s_rx_reassembly.next_frag_idx != hdr.frag_idx ||
        s_rx_reassembly.received_len != hdr.frag_offset) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_reset_reassembly("fragment sequence mismatch");
        return;
    }

    if (hdr.frag_idx > 0U && rx_time_us >= s_rx_reassembly.last_fragment_us) {
        uint32_t gap_us = rx_time_us - s_rx_reassembly.last_fragment_us;
        STATS_UPDATE_RANGE(intra_frame_gap_us_min, intra_frame_gap_us_max,
            intra_frame_gap_us_sum, intra_frame_gap_count, gap_us);
        if (gap_us > s_rx_reassembly.max_gap_us) {
            s_rx_reassembly.max_gap_us = gap_us;
        }
    }

    if (memcpy_s(s_rx_reassembly.buffer + hdr.frag_offset,
        sizeof(s_rx_reassembly.buffer) - hdr.frag_offset, payload, payload_len) != EOK) {
        STATS_ADD(sle_rx_drop_bytes, payload_len);
        demo_sle_reset_reassembly("payload copy failed");
        return;
    }

    s_rx_reassembly.received_len = (uint16_t)(hdr.frag_offset + payload_len);
    s_rx_reassembly.next_frag_idx = (uint8_t)(hdr.frag_idx + 1U);
    s_rx_reassembly.last_fragment_us = rx_time_us;

    if ((uint8_t)(hdr.frag_idx + 1U) == hdr.frag_count) {
        uint32_t reassembly_us = (rx_time_us >= s_rx_reassembly.first_fragment_us) ?
            (rx_time_us - s_rx_reassembly.first_fragment_us) : 0U;
        if (s_rx_reassembly.received_len != s_rx_reassembly.total_len) {
            STATS_ADD(sle_rx_drop_bytes, payload_len);
            demo_sle_reset_reassembly("length mismatch");
            return;
        }
        demo_sle_queue_complete_frame(s_rx_reassembly.buffer, s_rx_reassembly.total_len,
            s_rx_reassembly.frame_id, s_rx_reassembly.first_fragment_us,
            reassembly_us, s_rx_reassembly.max_gap_us);
        demo_sle_reset_reassembly(NULL);
    }
}

static void on_connect_state_changed(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    errcode_t ret;

    DEMO_INFO("Connect state: conn_id=%u state=%d disc_reason=0x%x", conn_id, conn_state, disc_reason);

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

        s_conn_id = conn_id;
        s_connected = true;
        demo_sle_reset_peer_discovery();
        s_seek_connect_pending = false;
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
        if (addr != NULL && pair_state == SLE_PAIR_NONE) {
            ret = sle_pair_remote_device(addr);
            DEMO_INFO("Pair request: conn_id=%u ret=0x%x", conn_id, ret);
        }
        (void)sle_stop_seek();
        demo_sle_log_transport("connected");
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        if (conn_id == s_conn_id) {
            s_conn_id = 0;
            s_connected = false;
            s_paired = false;
            demo_sle_reset_peer_discovery();
        }
        demo_sle_reset_tx_state();
        demo_sle_reset_reassembly(NULL);
        DEMO_INFO("SLE Disconnected");
        (void)sle_start_announce(DEMO_SLE_ADV_HANDLE);
        osal_msleep(100);
        (void)client_start_scan();
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
    ssaps_set_info(s_server_id, &info);
    s_mtu_size = DEMO_SLE_MTU_SIZE;
    demo_sle_refresh_payload_limits();
    demo_sle_request_conn_param_update("pair");
    demo_sle_request_low_latency("pair");
    (void)ssapc_exchange_info_req(0, conn_id, &info);
    demo_sle_log_transport("pair");
}

static void on_announce_enable(uint32_t announce_id, errcode_t status) { DEMO_INFO("Announce enabled: id=%u status=0x%x", announce_id, status); }
static void on_announce_disable(uint32_t announce_id, errcode_t status) { DEMO_INFO("Announce disabled: id=%u status=0x%x", announce_id, status); }
static void on_indicate_cfm(uint8_t server_id, uint16_t conn_id, sle_indication_cfm_result_t cfm_result, errcode_t status)
{ (void)server_id; (void)conn_id; (void)cfm_result; (void)status; }
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
    /* Keep the server write callback for CCCD/legacy compatibility; demo TX itself is notify-only. */
    if (write_cb == NULL || write_cb->length == 0U || write_cb->value == NULL) {
        return;
    }
    DEMO_LOG("Server write ignored: handle=%u len=%u need_rsp=%u",
        write_cb->handle, write_cb->length, write_cb->need_rsp);
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
        .announce_enable_cb = on_announce_enable,
        .announce_disable_cb = on_announce_disable,
        .seek_enable_cb = on_seek_enable,
        .seek_disable_cb = on_seek_disable,
        .seek_result_cb = on_seek_result,
    };
    sle_connection_callbacks_t conn_cbks = {
        .connect_state_changed_cb = on_connect_state_changed,
        .connect_param_update_req_cb = on_connect_param_update_req,
        .connect_param_update_cb = on_connect_param_update,
        .pair_complete_cb = on_pair_complete,
        .low_latency_cb = on_low_latency_set,
    };
    ssaps_callbacks_t ssaps_cbks = { .mtu_changed_cb = on_mtu_changed, .read_request_cb = on_server_read_request, .write_request_cb = on_server_write_request, .indicate_cfm_cb = on_indicate_cfm };
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
    ret = ssaps_register_callbacks(&ssaps_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) return ret;
    return ssapc_register_callbacks(&ssapc_cbks);
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
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
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
    uint8_t local_addr[SLE_ADDR_LEN] = {0};
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

    param.own_addr.type = 0;
    (void)memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN);
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

static errcode_t server_send_notify(const uint8_t *data, uint16_t len)
{
    ssaps_ntf_ind_t param = {0};
    uint8_t *buf = s_server_ntf_buf[s_server_ntf_idx];

    s_server_ntf_idx = (uint8_t)((s_server_ntf_idx + 1U) % 4U);
    if (memcpy_s(buf, DEMO_SLE_MTU_SIZE, data, len) != EOK) return ERRCODE_SLE_FAIL;
    param.handle = s_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = buf;
    param.value_len = len;
    return ssaps_notify_indicate(s_server_id, s_conn_id, &param);
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
    if (!demo_sle_should_initiate_connect(&result->addr)) {
        return;
    }
    DEMO_INFO("Found peer hybrid node, connecting...");
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
    if ((property->operate_indication & SSAP_OPERATE_INDICATION_BIT_NOTIFY) == 0U) {
        return;
    }
    s_client_rx_handle = property->handle;
    s_client_ready = true;
    DEMO_INFO("Hybrid client RX ready: peer_notify_handle=%u mtu=%u", s_client_rx_handle, s_mtu_size);
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
    ret = enable_sle();
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("SLE enable failed: 0x%x", ret);
        return -1;
    }
    demo_sle_refresh_local_addr();
    (void)demo_sle_apply_default_connect_param();
    DEMO_INFO("Initializing as SLE Hybrid");
    ret = demo_register_callbacks();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    ret = server_add_service();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    ret = server_start_announce();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    osal_msleep(1000);
    ret = client_start_scan();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    DEMO_INFO("SLE init OK");
    return 0;
}

bool demo_sle_is_connected(void)
{
    return s_connected && s_paired && s_client_ready;
}

uint16_t demo_sle_get_mtu(void) { return s_mtu_size; }
uint16_t demo_sle_get_conn_id(void) { return s_conn_id; }
uint16_t demo_sle_get_effective_payload(void) { return s_effective_payload; }
uint16_t demo_sle_get_fragment_payload(void) { return s_fragment_payload; }

uint32_t demo_sle_tx_process(void)
{
    const demo_frame_slot_t *frame;
    uint32_t total_sent = 0;
    uint32_t batch = 0;

    if (!demo_sle_is_connected()) {
        demo_sle_reset_tx_state();
        return 0;
    }
    demo_sle_refresh_payload_limits();
    if (s_fragment_payload == 0U) return 0;

    while (batch < DEMO_SLE_TX_MAX_FRAG_PER_LOOP) {
        demo_sle_frag_hdr_t hdr;
        uint16_t remaining;
        uint16_t payload_len;
        errcode_t ret;
        uint32_t now_us;

        if (!s_tx_state.active) {
            frame = demo_uart_rx_frame_peek();
            if (frame == NULL) break;
            s_tx_state.active = true;
            s_tx_state.frame = frame;
            s_tx_state.frame_id = (frame->frame_id != 0U) ? frame->frame_id : 1U;
            s_tx_state.offset = 0;
            s_tx_state.frag_idx = 0;
            s_tx_state.frag_count = demo_sle_calc_frag_count(frame->len, s_fragment_payload);
            if (s_tx_state.frag_count == 0U) {
                STATS_INC(sle_tx_fail);
                break;
            }
        }

        frame = s_tx_state.frame;
        remaining = (uint16_t)(frame->len - s_tx_state.offset);
        payload_len = (remaining > s_fragment_payload) ? s_fragment_payload : remaining;
        hdr.frame_id = s_tx_state.frame_id;
        hdr.total_len = frame->len;
        hdr.frag_offset = s_tx_state.offset;
        hdr.frag_idx = s_tx_state.frag_idx;
        hdr.frag_count = s_tx_state.frag_count;
        demo_sle_frag_encode(s_sle_tx_buf, &hdr);
        if (memcpy_s(s_sle_tx_buf + DEMO_SLE_FRAG_HEADER_SIZE,
            sizeof(s_sle_tx_buf) - DEMO_SLE_FRAG_HEADER_SIZE,
            frame->data + s_tx_state.offset, payload_len) != EOK) {
            STATS_INC(sle_tx_fail);
            break;
        }

        ret = server_send_notify(s_sle_tx_buf, (uint16_t)(payload_len + DEMO_SLE_FRAG_HEADER_SIZE));
        if (ret != ERRCODE_SLE_SUCCESS) {
            STATS_INC(sle_tx_fail);
            STATS_INC(sle_tx_busy_cnt);
            break;
        }

        now_us = (uint32_t)uapi_systick_get_us();
        if (s_tx_state.offset == 0U && frame->enqueue_timestamp_us > 0U && now_us >= frame->enqueue_timestamp_us) {
            uint32_t frame_delay_us = now_us - frame->enqueue_timestamp_us;
            STATS_UPDATE_RANGE(frame_tx_delay_us_min, frame_tx_delay_us_max,
                frame_tx_delay_us_sum, frame_tx_delay_count, frame_delay_us);
            DEMO_LOG("SLE TX frame start: id=%u len=%u frags=%u queue_delay=%u us eff=%u frag=%u",
                s_tx_state.frame_id, frame->len, s_tx_state.frag_count,
                frame_delay_us, s_effective_payload, s_fragment_payload);
        }

        STATS_INC(sle_tx_fragments);
        total_sent += payload_len;
        s_tx_state.offset = (uint16_t)(s_tx_state.offset + payload_len);
        s_tx_state.frag_idx = (uint8_t)(s_tx_state.frag_idx + 1U);
        batch++;

        if (s_tx_state.offset >= frame->len) {
            STATS_ADD(sle_tx_bytes, frame->len);
            STATS_INC(sle_tx_frames);
            DEMO_LOG("SLE TX frame complete: id=%u len=%u frags=%u",
                s_tx_state.frame_id, frame->len, s_tx_state.frag_count);
            demo_uart_rx_frame_consume();
            demo_sle_reset_tx_state();
            osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_TX_DONE);
        }
    }

    return total_sent;
}

void demo_sle_process_deferred(void)
{
}

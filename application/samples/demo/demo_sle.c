/**
 * @file demo_sle.c
 * @brief SLE Server/Client Implementation
 * 
 * Supports both Server (broadcast) and Client (scan/connect) roles.
 * Role determined by CONFIG_DEMO_SLE_SERVER/CONFIG_DEMO_SLE_CLIENT.
 */

#include "demo_sle.h"
#include "demo_config.h"
#include "securec.h"
#include "soc_osal.h"
#include "sle_common.h"
#include "sle_errcode.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_ssap_server.h"
#include "sle_ssap_client.h"
#include "driver/systick.h"

/*
 * The SLE advertising data type and channel-map constants are defined in
 * `application/samples/products/sle_uart/sle_uart_server/sle_uart_server_adv.h`,
 * but the demo sample shouldn't depend on the products sample headers.
 * Keep local fallbacks aligned with those definitions to avoid build breakage
 * when SDK headers don't export these constants.
 */
#ifndef SLE_ADV_CHANNEL_MAP_DEFAULT
#define SLE_ADV_CHANNEL_MAP_DEFAULT 0x07
#endif
#ifndef SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME
#define SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME 0x0B
#endif
#ifndef SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME
#define SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME 0x0A
#endif

/*============================================================================
 * Static Variables - Common
 *============================================================================*/

// SLE RX Ring Buffer: SLE Callback -> Main Loop
static uint8_t s_sle_rx_ring_buf[DEMO_SLE_RX_RING_SIZE];
static ring_buffer_t s_sle_rx_ring;

// SLE TX Ring Buffer: Main Loop -> SLE TX
static uint8_t s_sle_tx_ring_buf[DEMO_SLE_TX_RING_SIZE];
static ring_buffer_t s_sle_tx_ring;

// SLE TX staging buffer
static uint8_t s_sle_tx_buf[DEMO_SLE_MTU_SIZE];

// Connection state
static volatile uint16_t s_conn_id = 0;
static volatile uint16_t s_mtu_size = DEMO_SLE_MTU_SIZE;
static volatile bool s_connected = false;
static volatile bool s_paired = false;

// Server/Client specific handles
#if IS_SLE_SERVER
static uint8_t s_server_id = 0;
static uint16_t s_service_handle = 0;
static uint16_t s_property_handle = 0;
// Server TX buffer for notifications (reduced from 32 to 4 to save memory)
static uint8_t s_server_ntf_buf[4][DEMO_SLE_MTU_SIZE];
static volatile uint8_t s_server_ntf_idx = 0;
// [FLOW CONTROL] Track pending indications to prevent queue buildup
static volatile uint8_t s_indicate_pending = 0;
#define DEMO_MAX_PENDING_INDICATES  2  // Back to 2 for stability
#else
// Client send param
static ssapc_write_param_t s_client_write_param = {0};
static sle_addr_t s_remote_addr = {0};
// [FIX] Track if client is ready to send (handle discovered and MTU exchanged)
static volatile bool s_client_tx_ready = false;
// [FIX] SSAP Client ID - required for MTU exchange and service discovery
static uint8_t s_ssapc_client_id = 0;
// [LATENCY] Timestamp for RTT measurement (circular buffer for pending writes)
#define LATENCY_SAMPLE_COUNT 16
static volatile uint32_t s_tx_timestamp_us[LATENCY_SAMPLE_COUNT];
static volatile uint8_t s_tx_timestamp_wr_idx = 0;  // Write index (producer)
static volatile uint8_t s_tx_timestamp_rd_idx = 0;  // Read index (consumer)
#endif

/*============================================================================
 * SLE UUID Helpers
 *============================================================================*/

#define SLE_UUID_LEN_2              2
#define SLE_UUID_INDEX              14

static uint8_t s_sle_uuid_base[] = { 
    0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
};

#define SLE_UUID_SERVER_SERVICE     0xABCD
#define SLE_UUID_SERVER_PROPERTY    0xCDEF
#define SLE_UUID_TEST_PROPERTIES    0x03
#define SLE_UUID_TEST_DESCRIPTOR    0x01

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
    out->len = SLE_UUID_LEN_2;
    out->uuid[SLE_UUID_INDEX] = (uint8_t)(u2 >> 8);
    out->uuid[SLE_UUID_INDEX + 1] = (uint8_t)(u2);
}

/*============================================================================
 * Common Callbacks
 *============================================================================*/

static void on_connect_state_changed(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    (void)addr;
    (void)pair_state;

    DEMO_INFO("Connect state: conn_id=%d, state=%d, disc_reason=0x%x",
              conn_id, conn_state, disc_reason);

    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        s_conn_id = conn_id;
        s_connected = true;
        DEMO_INFO("SLE Connected!");

        // Set link layer data length to match MTU for better throughput and lower latency
        errcode_t ret = sle_set_data_len(conn_id, 251);
        DEMO_INFO("Set data_len=251: ret=0x%x", ret);

        // [FIX] Update connection parameters to ensure 2.5ms interval takes effect
        sle_connection_param_update_t conn_param = {
            .conn_id = conn_id,
            .interval_min = DEMO_SLE_CONN_INTV_MIN,  // 0x14 = 2.5ms
            .interval_max = DEMO_SLE_CONN_INTV_MAX,  // 0x14 = 2.5ms
            .max_latency = 0,
            .supervision_timeout = DEMO_SLE_SUPERVISION_TIMEOUT
        };
        ret = sle_update_connect_param(&conn_param);
        DEMO_INFO("Update conn_param (interval=0x%x): ret=0x%x", DEMO_SLE_CONN_INTV_MIN, ret);

        // [THROUGHPUT] Set PHY to 4M for higher speed (both ends must support)
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
        ret = sle_set_phy_param(conn_id, &phy_param);
        DEMO_INFO("Set PHY 4M: ret=0x%x", ret);

        // [THROUGHPUT] Set MCS for higher modulation (官方示例使用 MCS 10)
        ret = sle_set_mcs(conn_id, 10);
        DEMO_INFO("Set MCS=10: ret=0x%x", ret);

#if IS_SLE_SERVER
        // Server waits for pairing initiated by client
#else
        // Client initiates pairing
        sle_pair_remote_device(&s_remote_addr);
#endif
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        s_conn_id = 0;
        s_connected = false;
        s_paired = false;
        DEMO_INFO("SLE Disconnected");

#if IS_SLE_SERVER
        // Restart advertising
        sle_start_announce(DEMO_SLE_ADV_HANDLE);
#else
        // [FIX] Clear handle and TX ready flag on disconnect
        s_client_write_param.handle = 0;
        s_client_tx_ready = false;
        // Restart scanning
        osal_msleep(100);
        sle_start_seek();
#endif
    }
}

// [DEBUG] Connection parameter update callback - shows actual connection interval
static void on_connect_param_update(uint16_t conn_id, errcode_t status,
    const sle_connection_param_update_evt_t *param)
{
    if (param == NULL) {
        DEMO_ERR("Conn param update: null param, status=0x%x", status);
        return;
    }
    // interval 单位是 slot (125us)，转换为 ms: interval * 0.125
    uint32_t interval_us = param->interval * 125;
    DEMO_INFO("[CONN PARAM] conn_id=%d, interval=0x%x (%d.%03d ms), latency=%d, timeout=%d0 ms, status=0x%x",
              conn_id, param->interval, interval_us / 1000, interval_us % 1000,
              param->latency, param->supervision, status);
}

static void on_pair_complete(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    (void)addr;
    DEMO_INFO("Pair complete: conn_id=%d, status=0x%x", conn_id, status);

    if (status == ERRCODE_SLE_SUCCESS) {
        s_paired = true;

#if IS_SLE_SERVER
        // Server sets MTU info
        ssap_exchange_info_t info = { .mtu_size = DEMO_SLE_MTU_SIZE, .version = 1 };
        ssaps_set_info(s_server_id, &info);
        s_mtu_size = DEMO_SLE_MTU_SIZE;
#else
        // Client initiates MTU exchange
        ssap_exchange_info_t info = { .mtu_size = DEMO_SLE_MTU_SIZE, .version = 1 };
        DEMO_INFO("MTU exchange: client_id=%d, conn_id=%d, mtu=%d",
                  s_ssapc_client_id, conn_id, info.mtu_size);
        // NOTE: SDK samples use client_id=0, not the registered ID
        errcode_t ret = ssapc_exchange_info_req(0, conn_id, &info);
        DEMO_INFO("MTU exchange request: ret=0x%x", ret);
#endif
    }
}

/*============================================================================
 * Server Specific
 *============================================================================*/

#if IS_SLE_SERVER

static void on_announce_enable(uint32_t announce_id, errcode_t status)
{
    DEMO_INFO("Announce enabled: id=%d, status=0x%x", announce_id, status);
}

static void on_announce_disable(uint32_t announce_id, errcode_t status)
{
    DEMO_INFO("Announce disabled: id=%d, status=0x%x", announce_id, status);
}

static void on_mtu_changed(uint8_t server_id, uint16_t conn_id, 
    ssap_exchange_info_t *mtu_info, errcode_t status)
{
    (void)server_id;
    (void)conn_id;
    
    // [P0 FIX] Check null before dereference
    if (mtu_info == NULL) {
        DEMO_ERR("MTU changed: null info, status=0x%x", status);
        return;
    }
    
    DEMO_INFO("MTU changed: size=%d, status=0x%x", mtu_info->mtu_size, status);
    
    // Clamp MTU to valid range
    uint16_t new_mtu = mtu_info->mtu_size;
    if (new_mtu < 64) new_mtu = 64;
    if (new_mtu > DEMO_SLE_MTU_SIZE) new_mtu = DEMO_SLE_MTU_SIZE;
    s_mtu_size = new_mtu;
}

static void on_server_read_request(uint8_t server_id, uint16_t conn_id,
    ssaps_req_read_cb_t *read_cb, errcode_t status)
{
    (void)server_id;
    (void)conn_id;
    (void)read_cb;
    (void)status;
}

static void on_server_write_request(uint8_t server_id, uint16_t conn_id,
    ssaps_req_write_cb_t *write_cb, errcode_t status)
{
    (void)status;

    // [LATENCY] Record SLE RX timestamp - for measuring SLE RX to UART TX latency
    uint32_t rx_time_us = (uint32_t)uapi_systick_get_us();
    g_demo_stats.sle_rx_timestamp_us = rx_time_us;

    // [P0 FIX] Check null pointer
    if (write_cb == NULL) return;

    // Data received from Client
    if (write_cb->length > 0 && write_cb->value != NULL) {
        uint32_t written = ring_write(&s_sle_rx_ring, write_cb->value, write_cb->length);
        STATS_ADD(sle_rx_bytes, written);
        STATS_INC(sle_rx_frames);
        STATS_SET_HWM(sle_rx_ring_hwm, (uint16_t)ring_data_len(&s_sle_rx_ring));

        DEMO_INFO("[SRV RX] %d bytes @ %d us", write_cb->length, rx_time_us);

        if (written < write_cb->length) {
            STATS_ADD(sle_rx_drop_bytes, write_cb->length - written);
        }
        
        // [EVENT-DRIVEN] Signal main task
        osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_RX);
    }

    // [FIX] Send response if client used ssapc_write_req() (expects ACK)
    DEMO_LOG("[SRV] need_rsp=%d, request_id=%d", write_cb->need_rsp, write_cb->request_id);

    if (write_cb->need_rsp) {
        ssaps_send_rsp_t rsp = {
            .request_id = write_cb->request_id,
            .status = ERRCODE_SLE_SUCCESS,
            .value_len = 0,
            .value = NULL
        };

        uint32_t rsp_start_us = (uint32_t)uapi_systick_get_us();
        ssaps_send_response(server_id, conn_id, &rsp);
        uint32_t rsp_end_us = (uint32_t)uapi_systick_get_us();

        // [LATENCY] T3: Server processing time (RX callback → response sent)
        DEMO_INFO("[SRV RSP] T3=%d us (rsp_call=%d us)",
                  rsp_end_us - rx_time_us, rsp_end_us - rsp_start_us);
    }
}

// [NOTE] Indication confirmation callback
// This is called when Client ACKs an indication (not notify)
// Currently using notify which doesn't trigger this callback
static void on_indicate_cfm(uint8_t server_id, uint16_t conn_id,
    sle_indication_cfm_result_t cfm_result, errcode_t status)
{
    (void)server_id;
    (void)conn_id;
    (void)status;
    (void)cfm_result;

    // Log if this ever gets called (for debugging)
    DEMO_LOG("[SRV] indicate_cfm received");
}

static errcode_t server_register_callbacks(void)
{
    errcode_t ret;
    
    // Announce callbacks
    sle_announce_seek_callbacks_t announce_cbks = {
        .announce_enable_cb = on_announce_enable,
        .announce_disable_cb = on_announce_disable,
    };
    ret = sle_announce_seek_register_callbacks(&announce_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Announce callback register failed: 0x%x", ret);
        return ret;
    }

    // Connection callbacks
    sle_connection_callbacks_t conn_cbks = {
        .connect_state_changed_cb = on_connect_state_changed,
        .pair_complete_cb = on_pair_complete,
        .connect_param_update_cb = on_connect_param_update,  // [DEBUG] Monitor connection interval
    };
    ret = sle_connection_register_callbacks(&conn_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Connection callback register failed: 0x%x", ret);
        return ret;
    }

    // SSAP server callbacks
    ssaps_callbacks_t ssaps_cbks = {
        .mtu_changed_cb = on_mtu_changed,
        .read_request_cb = on_server_read_request,
        .write_request_cb = on_server_write_request,
        .indicate_cfm_cb = on_indicate_cfm,  // [FLOW CONTROL] Track indication ACKs
    };
    ret = ssaps_register_callbacks(&ssaps_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SSAPS callback register failed: 0x%x", ret);
        return ret;
    }
    
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t server_add_service(void)
{
    errcode_t ret;
    sle_uuid_t uuid;
    
    // Register server
    sle_uuid_t app_uuid = { .len = 2, .uuid = {0x12, 0x34} };
    ret = ssaps_register_server(&app_uuid, &s_server_id);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Register server failed: 0x%x", ret);
        return ret;
    }
    
    // Add service
    sle_uuid_set_u2(SLE_UUID_SERVER_SERVICE, &uuid);
    ret = ssaps_add_service_sync(s_server_id, &uuid, 1, &s_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Add service failed: 0x%x", ret);
        return ret;
    }
    
    // Add property
    ssaps_property_info_t property = {0};
    property.permissions = SLE_UUID_TEST_PROPERTIES;
    property.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ |
                                  SSAP_OPERATE_INDICATION_BIT_NOTIFY |
                                  SSAP_OPERATE_INDICATION_BIT_WRITE;
    sle_uuid_set_u2(SLE_UUID_SERVER_PROPERTY, &property.uuid);
    property.value = (uint8_t *)osal_vmalloc(8);
    if (property.value == NULL) {
        return ERRCODE_SLE_FAIL;
    }
    memset_s(property.value, 8, 0, 8);
    
    ret = ssaps_add_property_sync(s_server_id, s_service_handle, &property, &s_property_handle);
    osal_vfree(property.value);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Add property failed: 0x%x", ret);
        return ret;
    }
    
    // [LOW-LATENCY] Add Client Configuration Descriptor (CCCD)
    // 0x0001 = Notification (no ACK) - faster but may lose packets
    // 0x0002 = Indication (with ACK) - reliable but slower (~8ms per frame)
    // Changed to Notification for lower latency
    ssaps_desc_info_t descriptor = {0};
    uint8_t cccd_value[] = {0x01, 0x00};  // Enable notification (0x01) for low latency
    descriptor.permissions = SLE_UUID_TEST_DESCRIPTOR;
    descriptor.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    descriptor.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    descriptor.value = cccd_value;
    descriptor.value_len = sizeof(cccd_value);
    
    ret = ssaps_add_descriptor_sync(s_server_id, s_service_handle, s_property_handle, &descriptor);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Add CCCD descriptor failed: 0x%x", ret);
        // Continue anyway - some implementations may work without explicit CCCD
    } else {
        DEMO_INFO("CCCD descriptor added (notification enabled for low latency)");
    }
    
    // Start service
    ret = ssaps_start_service(s_server_id, s_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Start service failed: 0x%x", ret);
        return ret;
    }
    
    DEMO_INFO("Server service added: server_id=%d, service=%d, property=%d",
              s_server_id, s_service_handle, s_property_handle);
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t server_start_announce(void)
{
    errcode_t ret;
    uint8_t local_addr[SLE_ADDR_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
    
    // Announce parameters
    sle_announce_param_t param = {
        .announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE,
        .announce_handle = DEMO_SLE_ADV_HANDLE,
        .announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO,
        .announce_level = SLE_ANNOUNCE_LEVEL_NORMAL,
        .announce_channel_map = SLE_ADV_CHANNEL_MAP_DEFAULT,
        .announce_interval_min = DEMO_SLE_ADV_INTERVAL_MIN,
        .announce_interval_max = DEMO_SLE_ADV_INTERVAL_MAX,
        .conn_interval_min = DEMO_SLE_CONN_INTV_MIN,
        .conn_interval_max = DEMO_SLE_CONN_INTV_MAX,
        .conn_max_latency = 0,
        .conn_supervision_timeout = DEMO_SLE_SUPERVISION_TIMEOUT,
        .announce_tx_power = 18,
    };
    param.own_addr.type = 0;
    (void)memcpy_s(param.own_addr.addr, SLE_ADDR_LEN, local_addr, SLE_ADDR_LEN);
    
    ret = sle_set_announce_param(DEMO_SLE_ADV_HANDLE, &param);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Set announce param failed: 0x%x", ret);
        return ret;
    }
    
    // Announce data - device name
    uint8_t adv_data[32];
    uint8_t name[] = DEMO_SLE_SERVER_NAME;
    uint8_t name_len = sizeof(name) - 1;
    adv_data[0] = name_len + 1;
    adv_data[1] = SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME;
    (void)memcpy_s(&adv_data[2], sizeof(adv_data) - 2, name, name_len);
    
    sle_announce_data_t data = {
        .announce_data = adv_data,
        .announce_data_len = name_len + 2,
        .seek_rsp_data = adv_data,
        .seek_rsp_data_len = name_len + 2,
    };
    
    ret = sle_set_announce_data(DEMO_SLE_ADV_HANDLE, &data);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Set announce data failed: 0x%x", ret);
        return ret;
    }
    
    ret = sle_start_announce(DEMO_SLE_ADV_HANDLE);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Start announce failed: 0x%x", ret);
        return ret;
    }
    
    DEMO_INFO("Server advertising started");
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t server_send_notify(const uint8_t *data, uint16_t len)
{
    // [FIX] Removed pending check - ssaps_notify_indicate sends Notify (no ACK),
    // so indicate_cfm_cb is never called and pending counter never decrements.
    // The protocol stack has its own internal flow control.

    ssaps_ntf_ind_t param = {0};

    uint8_t *buf = s_server_ntf_buf[s_server_ntf_idx];
    s_server_ntf_idx = (s_server_ntf_idx + 1) % 4;

    if (memcpy_s(buf, DEMO_SLE_MTU_SIZE, data, len) != EOK) {
        return ERRCODE_SLE_FAIL;
    }

    param.handle = s_property_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = buf;
    param.value_len = len;

    uint32_t tx_time_us = (uint32_t)uapi_systick_get_us();
    errcode_t ret = ssaps_notify_indicate(s_server_id, s_conn_id, &param);

    if (ret == ERRCODE_SLE_SUCCESS) {
        DEMO_INFO("[SRV TX] %d bytes @ %d us", len, tx_time_us);
    } else {
        // Protocol stack returns error when busy - this is normal flow control
        DEMO_LOG("[SRV TX] busy: ret=0x%x, len=%d", ret, len);
    }
    return ret;
}

#endif /* IS_SLE_SERVER */

/*============================================================================
 * Client Specific
 *============================================================================*/

#if IS_SLE_CLIENT

static void on_sle_enable(errcode_t status)
{
    DEMO_INFO("SLE enabled: status=0x%x", status);
}

static void on_seek_enable(errcode_t status)
{
    DEMO_INFO("Seek enabled: status=0x%x", status);
}

/**
 * [P0 FIX] Parse AD structure to find Complete Local Name (type 0x09)
 * AD format: [len][type][data...] repeated
 */
static bool find_local_name_in_ad(const uint8_t *data, uint16_t data_len,
                                   const char *target_name, uint8_t target_len)
{
    uint16_t offset = 0;
    while (offset < data_len) {
        uint8_t field_len = data[offset];
        if (field_len == 0 || offset + field_len >= data_len) {
            break;
        }
        uint8_t field_type = data[offset + 1];
        
        // Check for Complete Local Name or Shortened Local Name
        if (field_type == SLE_ADV_DATA_TYPE_COMPLETE_LOCAL_NAME ||
            field_type == SLE_ADV_DATA_TYPE_SHORTENED_LOCAL_NAME) {
            uint8_t name_len = field_len - 1;  // Exclude type byte
            const uint8_t *name_ptr = &data[offset + 2];
            
            // Check if target name is within this field
            if (name_len >= target_len &&
                memcmp(name_ptr, target_name, target_len) == 0) {
                return true;
            }
        }
        
        offset += field_len + 1;
    }
    return false;
}

static void on_seek_result(sle_seek_result_info_t *result)
{
    if (result == NULL || result->data == NULL || result->data_length == 0) {
        return;
    }
    
    // [P0 FIX] Parse AD structure safely, don't use strstr on binary data
    static const char target_name[] = DEMO_SLE_SERVER_NAME;
    bool found = find_local_name_in_ad(result->data, result->data_length,
                                        target_name, sizeof(target_name) - 1);
    
    if (found) {
        DEMO_INFO("Found target server, connecting...");
        (void)memcpy_s(&s_remote_addr, sizeof(sle_addr_t), &result->addr, sizeof(sle_addr_t));
        sle_stop_seek();  // Will trigger on_seek_disable -> connect
    }
}

static void on_seek_disable(errcode_t status)
{
    DEMO_INFO("Seek disabled: status=0x%x", status);
    if (status == ERRCODE_SLE_SUCCESS) {
        sle_remove_paired_remote_device(&s_remote_addr);
        sle_connect_remote_device(&s_remote_addr);
    }
}

static void on_exchange_info(uint8_t client_id, uint16_t conn_id,
    ssap_exchange_info_t *info, errcode_t status)
{
    (void)client_id;
    
    // [P0 FIX] Check null before dereference
    if (info == NULL) {
        DEMO_ERR("Exchange info: null info, status=0x%x", status);
        return;
    }
    
    DEMO_INFO("Exchange info: mtu=%d, status=0x%x", info->mtu_size, status);
    
    // Clamp MTU to valid range
    uint16_t new_mtu = info->mtu_size;
    if (new_mtu < 64) new_mtu = 64;
    if (new_mtu > DEMO_SLE_MTU_SIZE) new_mtu = DEMO_SLE_MTU_SIZE;
    s_mtu_size = new_mtu;
    
    // Find service structure to get property handle
    ssapc_find_structure_param_t find = {
        .type = SSAP_FIND_TYPE_PROPERTY,
        .start_hdl = 1,
        .end_hdl = 0xFFFF,
    };
    errcode_t find_ret = ssapc_find_structure(0, conn_id, &find);
    DEMO_INFO("Find structure request: ret=0x%x", find_ret);
}

static void on_find_structure(uint8_t client_id, uint16_t conn_id,
    ssapc_find_service_result_t *service, errcode_t status)
{
    (void)client_id;
    (void)conn_id;
    if (service == NULL) {
        DEMO_ERR("Find structure: null service, status=0x%x", status);
        return;
    }
    DEMO_INFO("Find structure: start=%d, end=%d, status=0x%x",
              service->start_hdl, service->end_hdl, status);
}

static void on_find_property(uint8_t client_id, uint16_t conn_id,
    ssapc_find_property_result_t *property, errcode_t status)
{
    (void)client_id;
    (void)conn_id;
    (void)status;

    // [FIX] Check null pointer and valid handle
    if (property == NULL || property->handle == 0) {
        DEMO_ERR("Find property: invalid property or handle");
        return;
    }

    DEMO_INFO("Find property: handle=%d", property->handle);

    // Save property handle for data transmission
    s_client_write_param.handle = property->handle;
    s_client_write_param.type = SSAP_PROPERTY_TYPE_VALUE;

    // [FIX] Mark TX as ready - handle discovered
    s_client_tx_ready = true;
    DEMO_INFO("Client TX ready: handle=%d, mtu=%d", property->handle, s_mtu_size);
}

static void on_notification(uint8_t client_id, uint16_t conn_id,
    ssapc_handle_value_t *data, errcode_t status)
{
    (void)client_id;
    (void)conn_id;
    (void)status;

    // [LATENCY] Record RX timestamp - for measuring SLE RX to UART TX latency
    uint32_t rx_time_us = (uint32_t)uapi_systick_get_us();
    g_demo_stats.sle_rx_timestamp_us = rx_time_us;

    // [P0 FIX] Check null pointer
    if (data == NULL) return;

    // Data received from Server
    if (data->data_len > 0 && data->data != NULL) {
        uint32_t written = ring_write(&s_sle_rx_ring, data->data, data->data_len);
        STATS_ADD(sle_rx_bytes, written);
        STATS_INC(sle_rx_frames);
        STATS_SET_HWM(sle_rx_ring_hwm, (uint16_t)ring_data_len(&s_sle_rx_ring));

        DEMO_INFO("[CLI RX] %d bytes @ %d us", data->data_len, rx_time_us);

        if (written < data->data_len) {
            STATS_ADD(sle_rx_drop_bytes, data->data_len - written);
            DEMO_ERR("[CLI RX DROP] %d bytes dropped", data->data_len - written);
        }
        
        // [EVENT-DRIVEN] Signal main task
        osal_event_write(&g_bridge_event, DEMO_EVENT_SLE_RX);
    }
}

static void on_write_cfm(uint8_t client_id, uint16_t conn_id,
    ssapc_write_result_t *result, errcode_t status)
{
    (void)client_id;
    (void)conn_id;

    // [LATENCY] Calculate and print RTT for each frame (FIFO order)
    uint32_t now_us = (uint32_t)uapi_systick_get_us();
    uint8_t rd_idx = s_tx_timestamp_rd_idx;
    uint32_t send_time = s_tx_timestamp_us[rd_idx];

    if (send_time > 0 && now_us >= send_time) {
        uint32_t rtt_us = now_us - send_time;
        // Print each RTT
        DEMO_INFO("[RTT] %d us (%d ms)", rtt_us, rtt_us / 1000);

        // Clear and advance read index
        s_tx_timestamp_us[rd_idx] = 0;
        s_tx_timestamp_rd_idx = (rd_idx + 1) % LATENCY_SAMPLE_COUNT;

        // Update stats
        g_demo_stats.sle_rtt_us_sum += rtt_us;
        g_demo_stats.sle_rtt_count++;

        if (g_demo_stats.sle_rtt_us_min == 0 || rtt_us < g_demo_stats.sle_rtt_us_min) {
            g_demo_stats.sle_rtt_us_min = rtt_us;
        }
        if (rtt_us > g_demo_stats.sle_rtt_us_max) {
            g_demo_stats.sle_rtt_us_max = rtt_us;
        }
    }

    // [FIX] Log write confirmation for debugging
    if (status != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Write CFM failed: status=0x%x, handle=%d",
                 status, result ? result->handle : 0);
        STATS_INC(sle_tx_fail);
    }
}

static errcode_t client_register_callbacks(void)
{
    errcode_t ret;
    
    // Seek callbacks
    sle_announce_seek_callbacks_t seek_cbks = {
        .sle_enable_cb = on_sle_enable,
        .seek_enable_cb = on_seek_enable,
        .seek_result_cb = on_seek_result,
        .seek_disable_cb = on_seek_disable,
    };
    ret = sle_announce_seek_register_callbacks(&seek_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Seek callback register failed: 0x%x", ret);
        return ret;
    }

    // Connection callbacks
    sle_connection_callbacks_t conn_cbks = {
        .connect_state_changed_cb = on_connect_state_changed,
        .pair_complete_cb = on_pair_complete,
        .connect_param_update_cb = on_connect_param_update,  // [DEBUG] Monitor connection interval
    };
    ret = sle_connection_register_callbacks(&conn_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Connection callback register failed: 0x%x", ret);
        return ret;
    }

    // SSAP client callbacks
    ssapc_callbacks_t ssapc_cbks = {
        .exchange_info_cb = on_exchange_info,
        .find_structure_cb = on_find_structure,
        .ssapc_find_property_cbk = on_find_property,
        .notification_cb = on_notification,
        .indication_cb = on_notification,
        .write_cfm_cb = on_write_cfm,
    };
    ret = ssapc_register_callbacks(&ssapc_cbks);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("SSAPC callback register failed: 0x%x", ret);
        return ret;
    }
    DEMO_INFO("SSAPC callbacks registered");

    // NOTE: SDK samples do NOT call ssapc_register_client(), they use client_id=0 directly
    // Commenting out to match SDK behavior
    // sle_uuid_t app_uuid = { .len = 2, .uuid = {0x12, 0x34} };
    // ret = ssapc_register_client(&app_uuid, &s_ssapc_client_id);
    // DEMO_INFO("SSAPC register_client: ret=0x%x, id=%d", ret, s_ssapc_client_id);

    return ERRCODE_SLE_SUCCESS;
}

static errcode_t client_start_scan(void)
{
    sle_seek_param_t param = {
        .own_addr_type = 0,
        .filter_duplicates = 0,
        .seek_filter_policy = 0,
        .seek_phys = 1,
    };
    param.seek_type[0] = 1;
    param.seek_interval[0] = DEMO_SLE_SEEK_INTERVAL;
    param.seek_window[0] = DEMO_SLE_SEEK_WINDOW;
    
    errcode_t ret = sle_set_seek_param(&param);
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Set seek param failed: 0x%x", ret);
        return ret;
    }
    
    ret = sle_start_seek();
    if (ret != ERRCODE_SLE_SUCCESS) {
        DEMO_ERR("Start seek failed: 0x%x", ret);
        return ret;
    }
    
    DEMO_INFO("Client scanning started");
    return ERRCODE_SLE_SUCCESS;
}

static errcode_t client_send_write(const uint8_t *data, uint16_t len)
{
    s_client_write_param.data_len = len;
    s_client_write_param.data = (uint8_t *)data;

    // [LATENCY] Record send timestamp and calculate delay from UART RX
    uint32_t now_us = (uint32_t)uapi_systick_get_us();
    uint8_t wr_idx = s_tx_timestamp_wr_idx;
    s_tx_timestamp_us[wr_idx] = now_us;
    s_tx_timestamp_wr_idx = (wr_idx + 1) % LATENCY_SAMPLE_COUNT;

    // Print delay from UART RX to SLE TX
    if (g_demo_stats.uart_rx_timestamp_us > 0) {
        uint32_t delay_us = now_us - g_demo_stats.uart_rx_timestamp_us;
        DEMO_INFO("[DELAY] UART_RX->SLE_TX: %d us (%d ms)", delay_us, delay_us / 1000);
    }

    g_demo_stats.sle_tx_start_us = now_us;

    // [LATENCY FIX] Use write_cmd instead of write_req
    // write_req waits for server response (blocking ~500ms per packet)
    // write_cmd is fire-and-forget (non-blocking, much faster)
    return ssapc_write_cmd(0, s_conn_id, &s_client_write_param);
}

#endif /* IS_SLE_CLIENT */

/*============================================================================
 * Public API
 *============================================================================*/

int demo_sle_init(void)
{
    errcode_t ret;
    
    // Init ring buffers
    ring_init(&s_sle_rx_ring, s_sle_rx_ring_buf, sizeof(s_sle_rx_ring_buf));
    ring_init(&s_sle_tx_ring, s_sle_tx_ring_buf, sizeof(s_sle_tx_ring_buf));
    
    // Enable SLE
    ret = enable_sle();
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("SLE enable failed: 0x%x", ret);
        return -1;
    }
    
#if IS_SLE_SERVER
    DEMO_INFO("Initializing as SLE Server");
    
    ret = server_register_callbacks();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    
    ret = server_add_service();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    
    ret = server_start_announce();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    
#else
    DEMO_INFO("Initializing as SLE Client");
    
    ret = client_register_callbacks();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
    
    // Wait for SLE stack ready
    osal_msleep(1000);
    
    ret = client_start_scan();
    if (ret != ERRCODE_SLE_SUCCESS) return -1;
#endif
    
    DEMO_INFO("SLE init OK");
    return 0;
}

bool demo_sle_is_connected(void)
{
#if IS_SLE_SERVER
    return s_connected && s_paired;
#else
    // [FIX] Client must also have valid handle before TX
    // Use handle check instead of separate flag
    bool ready = s_connected && s_paired && (s_client_write_param.handle != 0);
    if (s_connected && s_paired && !ready) {
        // Debug: why not ready (log only once per second to avoid flooding)
        static uint32_t s_last_log = 0;
        uint32_t now = (uint32_t)osal_get_jiffies();
        if (now - s_last_log > 1000) {
            DEMO_INFO("TX blocked: handle=%d", s_client_write_param.handle);
            s_last_log = now;
        }
    }
    return ready;
#endif
}

ring_buffer_t* demo_sle_get_rx_ring(void)
{
    return &s_sle_rx_ring;
}

ring_buffer_t* demo_sle_get_tx_ring(void)
{
    return &s_sle_tx_ring;
}

uint16_t demo_sle_get_mtu(void)
{
    return s_mtu_size;
}

uint16_t demo_sle_get_conn_id(void)
{
    return s_conn_id;
}

/**
 * [P1 FIX] Throttled SLE TX processing.
 * Limits to max 8 frames per call for scheduling fairness.
 * Track busy count for congestion monitoring.
 */
#define DEMO_SLE_TX_MAX_BATCH  12  // Balance between speed and stability

uint32_t demo_sle_tx_process(void)
{
    // [FLOW CONTROL] Static backoff counter - when TX fails, wait before retrying
    static uint8_t s_tx_backoff = 0;
    
    if (!demo_sle_is_connected()) {
        s_tx_backoff = 0;  // Reset on disconnect
        return 0;
    }
    
    // [FLOW CONTROL] If in backoff state, decrement and skip this round
    if (s_tx_backoff > 0) {
        s_tx_backoff--;
        return 0;
    }
    
    uint32_t total_sent = 0;
    uint32_t batch = 0;
    // [P1 FIX] Ensure max_payload never exceeds buffer size
    uint16_t max_payload = (s_mtu_size > 10) ? (s_mtu_size - 10) : 200;
    if (max_payload > (DEMO_SLE_MTU_SIZE - 10)) {
        max_payload = DEMO_SLE_MTU_SIZE - 10;
    }
    
    // [P1 FIX] Reduced batch limit for scheduling fairness
    while (!ring_is_empty(&s_sle_tx_ring) && batch < DEMO_SLE_TX_MAX_BATCH) {
        uint8_t *ptr;
        uint32_t contiguous_len;
        
        ring_peek_contiguous(&s_sle_tx_ring, &ptr, &contiguous_len);
        if (contiguous_len == 0) break;
        
        if (contiguous_len > max_payload) {
            contiguous_len = max_payload;
        }
        
        // Copy to TX buffer
        if (memcpy_s(s_sle_tx_buf, sizeof(s_sle_tx_buf), ptr, contiguous_len) != EOK) {
            break;
        }
        
        errcode_t ret;
#if IS_SLE_SERVER
        ret = server_send_notify(s_sle_tx_buf, (uint16_t)contiguous_len);
#else
        ret = client_send_write(s_sle_tx_buf, (uint16_t)contiguous_len);
#endif

        if (ret == ERRCODE_SLE_SUCCESS) {
            ring_consume(&s_sle_tx_ring, contiguous_len);
            STATS_ADD(sle_tx_bytes, contiguous_len);
            STATS_INC(sle_tx_frames);
            total_sent += contiguous_len;
            batch++;
            // [DEBUG] Log each frame sent
            DEMO_LOG("SLE TX frame: %d bytes, batch=%d", contiguous_len, batch);
        } else {
            // [FLOW CONTROL] TX failed - minimal backoff to retry quickly
            s_tx_backoff = 1;  // Reduced from 2 for lower latency
            STATS_INC(sle_tx_fail);
            STATS_INC(sle_tx_busy_cnt);
            DEMO_LOG("SLE TX fail: ret=0x%x, len=%d, backoff=%d", ret, contiguous_len, s_tx_backoff);
            break;  // SLE busy, retry later
        }
    }
    
    return total_sent;
}

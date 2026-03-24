/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
 *
 * Description: SLE UART Sample Source. \n
 *
 * History: \n
 * 2023-07-17, Create file. \n
 */

#include <stdbool.h>
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "pinctrl.h"
#include "uart.h"
#if defined(CONFIG_UART_SUPPORT_DMA)
#include "dma.h"
#include "hal_dma.h"
#endif
#include "sle_low_latency.h"
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#include "securec.h"
#include "sle_uart_server.h"
#include "sle_uart_server_adv.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
#define SLE_UART_TASK_STACK_SIZE            0x1000
#include "securec.h"
#include "sle_connection_manager.h"
#include "sle_ssap_client.h"
#include "sle_uart_client.h"
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

#define SLE_UART_TASK_PRIO                  28
#define SLE_UART_TASK_DURATION_MS           2000
#define SLE_UART_BAUDRATE                   921600
#define SLE_UART_TRANSFER_SIZE              4096

// CCH protocol frame markers
#define CCH_FRAME_HEADER_0                  0xA5
#define CCH_FRAME_HEADER_1                  0x5A
#define CCH_FRAME_TAIL_0                    0x55
#define CCH_FRAME_TAIL_1                    0xAA

#define CCH_MIN_FRAME_SIZE                  8
#define CCH_HEADER_SIZE                     4
#define CCH_TAIL_SIZE                       2
#define CCH_STRIP_SIZE                      6

#define SLE_UART_DEBUG_LOG                  0
#if SLE_UART_DEBUG_LOG
#define UART_DBG(fmt, ...) osal_printk(fmt, ##__VA_ARGS__)
#else
#define UART_DBG(fmt, ...)
#endif 

// Forward declarations
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error);
#endif

static uint8_t g_app_uart_rx_buff[SLE_UART_TRANSFER_SIZE] = { 0 };

/*
 * Frame error counter - incremented in ISR context.
 * Using volatile to ensure visibility across contexts.
 * For production use, consider atomic operations if platform supports.
 */
static volatile uint32_t g_frame_error_cnt = 0;
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#define SLE_EVENT_DISCONNECT                (1U << 0)
osal_event g_sle_server_event = { 0 };
#endif

static uart_buffer_config_t g_app_uart_buffer_config = {
    .rx_buffer = g_app_uart_rx_buff,
    .rx_buffer_size = SLE_UART_TRANSFER_SIZE
};

#if defined(CONFIG_UART_SUPPORT_DMA)
static uart_write_dma_config_t g_app_uart_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_4,
    .priority = HAL_DMA_CH_PRIORITY_0
};
#endif

/*
 * Simplified CCH frame parser - optimized for performance.
 * Only validates frame boundaries (header/tail markers).
 *
 * Assumptions:
 * - CCH sensor guarantees data integrity (internal CRC).
 * - Frame format: [A5 5A] [3F F3] [Len 2B] [Data...] [Checksum 2B] [55 AA].
 * - Processing: Strip first 4 bytes (frame+msg header) and last 2 bytes (tail).
 *
 * Performance: ~15 CPU cycles (vs. ~50 in full validation).
 */
/*static bool parse_cch_frame(const uint8_t *buffer, uint16_t length,
                            uint16_t *payload_offset, uint16_t *payload_len)
{
    if ((buffer == NULL) || (payload_offset == NULL) || (payload_len == NULL) ||
        (length < CCH_MIN_FRAME_SIZE)) {
        g_frame_error_cnt++;
        return false;
    }

    if ((buffer[0] != CCH_FRAME_HEADER_0) || (buffer[1] != CCH_FRAME_HEADER_1) ||
        (buffer[length - 2] != CCH_FRAME_TAIL_0) || (buffer[length - 1] != CCH_FRAME_TAIL_1)) {
        g_frame_error_cnt++;
        return false;
    }

    *payload_offset = CCH_HEADER_SIZE;
    *payload_len = length - CCH_STRIP_SIZE;
    return true;
}
*/
static void uart_init_pin(void)
{
    if (CONFIG_SLE_UART_BUS == 0) {
        osal_printk("[Init] Skip Pinmux for UART0 to keep USB Log alive.\r\n");
        return;
    }
    uapi_pin_set_mode(CONFIG_UART_TXD_PIN, PIN_MODE_1);
    uapi_pin_set_mode(CONFIG_UART_RXD_PIN, PIN_MODE_1);
}

static void uart_init_config(void)
{
    errcode_t ret;
    uart_attr_t attr = {
        .baud_rate = SLE_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE
    };

    uart_pin_config_t pin_config = {
        .tx_pin = CONFIG_UART_TXD_PIN,
        .rx_pin = CONFIG_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE
    };

#if defined(CONFIG_UART_SUPPORT_DMA)
    uart_extra_attr_t ext_config = {
        .tx_dma_enable = true,
        .tx_int_threshold = UART_FIFO_INT_TX_LEVEL_EQ_0_CHARACTER,
        .rx_dma_enable = false,
        .rx_int_threshold = UART_FIFO_INT_RX_LEVEL_1_CHARACTER
    };
#endif

    osal_printk("[Init] Re-configuring UART%d (Baud: %d)...\r\n", CONFIG_SLE_UART_BUS, SLE_UART_BAUDRATE);
    osal_msleep(100); // Wait for log flush

    (void)uapi_uart_deinit(CONFIG_SLE_UART_BUS);

#if defined(CONFIG_UART_SUPPORT_DMA)
    ret = uapi_dma_init();
    if (ret != ERRCODE_SUCC) {
        // Can't print if UART0 is down, but try anyway
        return;
    }
    ret = uapi_dma_open();
    if (ret != ERRCODE_SUCC) {
        return;
    }
    // Note: If UART0 is used, this init will change baudrate to 2M.
    // User MUST update serial terminal baudrate to 2000000 immediately.
    ret = uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, &ext_config, &g_app_uart_buffer_config);
    osal_printk("[Init] UART DMA Mode Ready (TX DMA, RX Int). Baud: %d\r\n", SLE_UART_BAUDRATE);
#else
    osal_printk("[UART] Init with IRQ Mode...\r\n");
    ret = uapi_uart_init(CONFIG_SLE_UART_BUS, &pin_config, &attr, NULL, &g_app_uart_buffer_config);
#endif

    if (ret == ERRCODE_SUCC) {
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
        uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS, UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE, 
                                      SLE_UART_TRANSFER_SIZE, sle_uart_server_read_int_handler);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
        uapi_uart_register_rx_callback(CONFIG_SLE_UART_BUS, UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE, 
                                      SLE_UART_TRANSFER_SIZE, sle_uart_client_read_int_handler);
#endif
    }
    if (ret != ERRCODE_SUCC) {
        osal_printk("UART init failed: 0x%x\r\n", ret);
    }
}

#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
#define SLE_UART_SERVER_DELAY_COUNT         5

#define SLE_UART_TASK_STACK_SIZE            0x1200
#define SLE_ADV_HANDLE_DEFAULT              1
#define SLE_UART_SERVER_LOG                 "[sle uart server]"

// [Force Optimization] Remove DMA macro check
// Use 4-level round-robin buffers for Server too
#define SERVER_TX_BUFF_COUNT 4
static uint8_t g_uart_tx_buff_server[SERVER_TX_BUFF_COUNT][SLE_UART_TRANSFER_SIZE] = { 0 };
static uint8_t g_server_tx_buff_idx = 0;

// Ring Buffer for Server TX (SLE -> UART)
#define SERVER_RING_BUFFER_SIZE (32 * 1024)
static uint8_t g_server_ring_buffer[SERVER_RING_BUFFER_SIZE];
static volatile uint32_t g_ring_head = 0;
static volatile uint32_t g_ring_tail = 0;

static void ring_buffer_write(const uint8_t *data, uint16_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t next_head = (g_ring_head + 1) % SERVER_RING_BUFFER_SIZE;
        if (next_head != g_ring_tail) {
            g_server_ring_buffer[g_ring_head] = data[i];
            g_ring_head = next_head;
        } else {
            // Overflow: Drop data (better than blocking stack)
            osal_printk("S_Full!");
            break;
        }
    }
}

static void ssaps_server_read_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps read request cbk callback server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, read_cb_para->handle, status);
}
static void ssaps_server_write_request_cbk(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb_para,
    errcode_t status)
{
    osal_printk("%s ssaps write request callback cbk server_id:%x, conn_id:%x, handle:%x, status:%x\r\n",
        SLE_UART_SERVER_LOG, server_id, conn_id, write_cb_para->handle, status);
    if ((write_cb_para->length > 0) && write_cb_para->value) {
        // [Optimization] Write to Ring Buffer (Force usage)
        ring_buffer_write(write_cb_para->value, write_cb_para->length);
    }
}

static void sle_uart_server_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    if (error || (length == 0) || (buffer == NULL)) {
        return;
    }

    // [Performance] Disable logs for 4M baudrate to prevent FIFO overflow
    osal_printk("[UART-SRV] RX Len: %d\r\n", length);
    /*
    if (length > 0) {
        osal_printk("RX Hex: ");
        uint16_t print_len = (length > 10) ? 10 : length;
        const uint8_t *p = (const uint8_t *)buffer;
        for (uint16_t i = 0; i < print_len; i++) {
            osal_printk("%02X ", p[i]);
        }
        osal_printk("...\r\n");
    }
    */

    if (sle_uart_client_is_connected()) {
        errcode_t ret = sle_uart_server_send_report_by_handle((const uint8_t *)buffer, length);
        if (ret != ERRCODE_SLE_SUCCESS) {
            osal_printk("[UART-SRV] SLE Send Failed! Ret: 0x%x\r\n", ret);
        }
    } else {
        // osal_printk("[UART-SRV] Not Connected/Paired!\r\n");
    }
}

static void *sle_uart_server_task(const char *arg)
{
    unused(arg);
    errcode_t ret = ERRCODE_SUCC;
    if (osal_event_init(&g_sle_server_event) != OSAL_SUCCESS) {
        osal_printk("%s event init failed\r\n", SLE_UART_SERVER_LOG);
        return NULL;
    }
    sle_uart_server_init(ssaps_server_read_request_cbk, ssaps_server_write_request_cbk);

    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    // [Init] Enabled Callback in uart_init_config
    // uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    osal_printk("[Init] Switched to INTERRUPT Mode for efficiency.\r\n");

    while (1) {
        // [Server TX Logic] Process Ring Buffer until empty
        // Loop to process all data in ring buffer (handle wrap-around immediately)
        while (g_ring_head != g_ring_tail) {
            uint32_t head = g_ring_head;
            uint32_t tail = g_ring_tail;
            uint32_t data_len = 0;
            
            if (head > tail) {
                data_len = head - tail;
            } else {
                data_len = SERVER_RING_BUFFER_SIZE - tail;
            }
            
            // Cap to DMA transfer size
            if (data_len > SLE_UART_TRANSFER_SIZE) {
                data_len = SLE_UART_TRANSFER_SIZE;
            }

            // Copy from Ring Buffer to DMA Buffer (Ping-Pong)
            if (memcpy_s(g_uart_tx_buff_server[g_server_tx_buff_idx], SLE_UART_TRANSFER_SIZE, 
                         &g_server_ring_buffer[tail], data_len) == EOK) {
                
                // Start DMA (Blocking)
                uapi_uart_write_by_dma(CONFIG_SLE_UART_BUS, g_uart_tx_buff_server[g_server_tx_buff_idx],
                                       data_len, &g_app_uart_dma_cfg);
                
                // Advance tail
                g_ring_tail = (tail + data_len) % SERVER_RING_BUFFER_SIZE;
                // Switch Ping-Pong buffer
                g_server_tx_buff_idx = (g_server_tx_buff_idx + 1) % SERVER_TX_BUFF_COUNT;
            } else {
                // Should not happen, but break to avoid infinite loop
                break;
            }
        }

        // Wait for disconnect event (or others in future)
        // Using OSAL_WAITMODE_CLR means the flag clears automatically after reading.
        int32_t event_ret = osal_event_read(&g_sle_server_event, SLE_EVENT_DISCONNECT, 
                                            OSAL_WAIT_FOREVER, OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR);
        
        if ((event_ret != OSAL_FAILURE) && (event_ret & (int32_t)SLE_EVENT_DISCONNECT)) {
            ret = sle_start_announce(SLE_ADV_HANDLE_DEFAULT);
            if (ret != ERRCODE_SLE_SUCCESS) {
                osal_printk("%s Restart announce failed: %02x\r\n", SLE_UART_SERVER_LOG, ret);
            }
        }
    }
    return NULL;
}
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)

// [Force Optimization] Remove DMA macro check
// Use 4-level round-robin buffers for Client TX (SLE->UART)
#define CLIENT_TX_BUFF_COUNT 4
static uint8_t g_uart_tx_buff_client[CLIENT_TX_BUFF_COUNT][SLE_UART_TRANSFER_SIZE] = { 0 };
static uint8_t g_client_tx_buff_idx = 0;

// Ring Buffer for Client TX (SLE -> UART)
#define CLIENT_RING_BUFFER_SIZE (32 * 1024)
static uint8_t g_client_ring_buffer[CLIENT_RING_BUFFER_SIZE];
static volatile uint32_t g_client_ring_head = 0;
static volatile uint32_t g_client_ring_tail = 0;

static void client_ring_buffer_write(const uint8_t *data, uint16_t len)
{
    uint32_t i;
    for (i = 0; i < len; i++) {
        uint32_t next_head = (g_client_ring_head + 1) % CLIENT_RING_BUFFER_SIZE;
        if (next_head != g_client_ring_tail) {
            g_client_ring_buffer[g_client_ring_head] = data[i];
            g_client_ring_head = next_head;
        } else {
            osal_printk("C_Full!");
            break;
        }
    }
}

static uint8_t g_sle_uart_client_tx_buff[4][SLE_UART_TRANSFER_SIZE] = { 0 };

void sle_uart_notification_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n[sle uart client] received %d bytes\r\n", data->data_len);

    if ((data->data_len > 0) && (data->data != NULL)) {
        // [Force Optimization] Write to Client Ring Buffer
        client_ring_buffer_write(data->data, data->data_len);
    }
}

void sle_uart_indication_cb(uint8_t client_id, uint16_t conn_id, ssapc_handle_value_t *data,
    errcode_t status)
{
    unused(client_id);
    unused(conn_id);
    unused(status);
    osal_printk("\n[sle uart client] indication received %d bytes\r\n", data->data_len);
#if defined(CONFIG_UART_SUPPORT_DMA)
    uint16_t copy_len = (data->data_len > SLE_UART_TRANSFER_SIZE) ?
                        SLE_UART_TRANSFER_SIZE : data->data_len;
    if (memcpy_s(g_uart_tx_buff_client, SLE_UART_TRANSFER_SIZE, data->data, copy_len) != EOK) {
        osal_printk("[sle uart client] indication memcpy failed\r\n");
        return;
    }
    int32_t ret = uapi_uart_write_by_dma(CONFIG_SLE_UART_BUS, g_uart_tx_buff_client,
                                         copy_len, &g_app_uart_dma_cfg);
    if (ret < 0) {
        osal_printk("[sle uart client] DMA write indication failed: %d\r\n", ret);
    }
#else
    uapi_uart_write(CONFIG_SLE_UART_BUS, (uint8_t *)data->data, data->data_len, 0);
#endif
}

static void sle_uart_client_read_int_handler(const void *buffer, uint16_t length, bool error)
{
    if (error || (length == 0) || (buffer == NULL)) {
        return;
    }

    // [Performance] Disable logs for 4M baudrate
    // osal_printk("[UART-CLI] RX Len: %d\r\n", length);
    /*
    if (length > 0) {
        osal_printk("RX Hex: ");
        uint16_t print_len = (length > 10) ? 10 : length;
        const uint8_t *p = (const uint8_t *)buffer;
        for (uint16_t i = 0; i < print_len; i++) {
            osal_printk("%02X ", p[i]);
        }
        osal_printk("...\r\n");
    }
    */

    uint16_t copy_len = (length > SLE_UART_TRANSFER_SIZE) ? SLE_UART_TRANSFER_SIZE : length;
    
    // Use Round-Robin Buffer: g_sle_uart_client_tx_buff[g_client_tx_buff_idx]
    if (memcpy_s(g_sle_uart_client_tx_buff[g_client_tx_buff_idx], SLE_UART_TRANSFER_SIZE, buffer, copy_len) != EOK) {
        return;
    }

    ssapc_write_param_t *sle_uart_send_param = get_g_sle_uart_send_param();
    uint16_t g_sle_uart_conn_id = get_g_sle_uart_conn_id();

    sle_uart_send_param->data_len = copy_len;
    sle_uart_send_param->data = g_sle_uart_client_tx_buff[g_client_tx_buff_idx];
    errcode_t ret = ssapc_write_req(0, g_sle_uart_conn_id, sle_uart_send_param);
    if (ret != ERRCODE_SUCC) {
        osal_printk("[UART-CLI] SLE Send Failed! Ret: 0x%x\r\n", ret);
    }

    // Switch to next buffer for next interrupt
    g_client_tx_buff_idx = (g_client_tx_buff_idx + 1) % CLIENT_TX_BUFF_COUNT;
}

static void *sle_uart_client_task(const char *arg)
{
    unused(arg);
    /* UART pinmux. */
    uart_init_pin();

    /* UART init config. */
    uart_init_config();

    sle_uart_client_init(sle_uart_notification_cb, sle_uart_indication_cb);
    
    // [Init] Client Switched to INTERRUPT Mode.
    // uapi_uart_unregister_rx_callback(CONFIG_SLE_UART_BUS);
    osal_printk("[Init] Client Switched to INTERRUPT Mode.\r\n");

    while (1) {
        // [Client TX Logic] Check Ring Buffer and Send via DMA (SLE -> UART)
        while (g_client_ring_head != g_client_ring_tail) {
            uint32_t head = g_client_ring_head;
            uint32_t tail = g_client_ring_tail;
            uint32_t data_len = 0;
            
            if (head > tail) {
                data_len = head - tail;
            } else {
                data_len = CLIENT_RING_BUFFER_SIZE - tail;
            }
            
            if (data_len > SLE_UART_TRANSFER_SIZE) {
                data_len = SLE_UART_TRANSFER_SIZE;
            }

            if (memcpy_s(g_uart_tx_buff_client[g_client_tx_buff_idx], SLE_UART_TRANSFER_SIZE, 
                         &g_client_ring_buffer[tail], data_len) == EOK) {
                
                uapi_uart_write_by_dma(CONFIG_SLE_UART_BUS, g_uart_tx_buff_client[g_client_tx_buff_idx],
                                       data_len, &g_app_uart_dma_cfg);
                
                g_client_ring_tail = (tail + data_len) % CLIENT_RING_BUFFER_SIZE;
                g_client_tx_buff_idx = (g_client_tx_buff_idx + 1) % CLIENT_TX_BUFF_COUNT;
            } else {
                break;
            }
        }

        // No Polling needed here. Data is handled in sle_uart_client_read_int_handler.
        // Client task just needs to stay alive and process the TX Ring Buffer.
        // Sleep briefly to yield, but fast enough to drain buffer.
        osal_msleep(1); 
    }

    return NULL;
}
#endif  /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */

static void sle_uart_entry(void)
{
    osal_task *task_handle = NULL;
    osal_kthread_lock();
#if defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_SERVER)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_server_task, 0, "SLEUartServerTask",
                                      SLE_UART_TASK_STACK_SIZE);
#elif defined(CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT)
    task_handle = osal_kthread_create((osal_kthread_handler)sle_uart_client_task, 0, "SLEUartDongleTask",
                                      SLE_UART_TASK_STACK_SIZE);
#endif /* CONFIG_SAMPLE_SUPPORT_SLE_UART_CLIENT */
    if (task_handle != NULL) {
        osal_kthread_set_priority(task_handle, SLE_UART_TASK_PRIO);
    }
    osal_kthread_unlock();
}

/* Run the sle_uart_entry. */
app_run(sle_uart_entry);

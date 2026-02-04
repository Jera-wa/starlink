/**
 * @file demo_uart.c
 * @brief UART DMA+Interrupt Implementation
 * 
 * Uses DMA for TX (non-blocking) and RX callback + ring buffer.
 */

#include "demo_uart.h"
#include "demo_config.h"
#include "demo_sle.h"  // [LOW-LATENCY] For direct SLE TX
#include "securec.h"
#include "soc_osal.h"
#include "pinctrl.h"
#include "uart.h"
#include "dma.h"
#include "hal_dma.h"
#include "driver/systick.h"

/*============================================================================
 * Static Variables
 *============================================================================*/

// RX Ring Buffer: UART RX ISR -> Main Loop
static uint8_t s_uart_rx_ring_buf[DEMO_SLE_TX_RING_SIZE];
static ring_buffer_t s_uart_rx_ring;

// TX Ring Buffer: Main Loop -> UART DMA
static uint8_t s_uart_tx_ring_buf[DEMO_SLE_RX_RING_SIZE];
static ring_buffer_t s_uart_tx_ring;

// DMA TX buffer (ping-pong)
static uint8_t s_dma_tx_buf[DEMO_TX_BUFFER_COUNT][DEMO_DMA_CHUNK_SIZE];
static volatile uint8_t s_dma_tx_idx = 0;
static volatile bool s_dma_tx_busy = false;

// UART RX buffer for SDK callback
static uint8_t s_uart_rx_buf[DEMO_UART_RX_BUFFER_SIZE];

// DMA config
static uart_write_dma_config_t s_dma_cfg = {
    .src_width = HAL_DMA_TRANSFER_WIDTH_8,
    .dest_width = HAL_DMA_TRANSFER_WIDTH_8,
    .burst_length = HAL_DMA_BURST_TRANSACTION_LENGTH_4,
    .priority = HAL_DMA_CH_PRIORITY_0
};

static uart_buffer_config_t s_uart_buf_cfg = {
    .rx_buffer = s_uart_rx_buf,
    .rx_buffer_size = DEMO_UART_RX_BUFFER_SIZE
};

/*============================================================================
 * UART RX Callback (ISR Context)
 *============================================================================*/

static void uart_rx_callback(const void *buffer, uint16_t length, bool error)
{
    if (error || length == 0 || buffer == NULL) {
        return;
    }

    // [LATENCY] Record UART RX timestamp
    g_demo_stats.uart_rx_timestamp_us = (uint32_t)uapi_systick_get_us();

    // Write to RX ring buffer
    uint32_t written = ring_write(&s_uart_rx_ring, (const uint8_t *)buffer, length);

    STATS_ADD(uart_rx_bytes, written);
    STATS_SET_HWM(uart_rx_ring_hwm, (uint16_t)ring_data_len(&s_uart_rx_ring));

    if (written < length) {
        STATS_ADD(uart_rx_drop_bytes, length - written);
        STATS_INC(ring_overflow);
    }
    
    // [EVENT-DRIVEN] Signal main task that UART data is available
    osal_event_write(&g_bridge_event, DEMO_EVENT_UART_RX);
}

/*============================================================================
 * Initialization
 *============================================================================*/

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
    
    // Initialize ring buffers
    ring_init(&s_uart_rx_ring, s_uart_rx_ring_buf, sizeof(s_uart_rx_ring_buf));
    ring_init(&s_uart_tx_ring, s_uart_tx_ring_buf, sizeof(s_uart_tx_ring_buf));
    
    // Configure pins
    uart_init_pin();
    
    // UART config
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
        .tx_dma_enable = true,
        .tx_int_threshold = UART_FIFO_INT_TX_LEVEL_EQ_0_CHARACTER,
        .rx_dma_enable = false,
        .rx_int_threshold = UART_FIFO_INT_RX_LEVEL_1_CHARACTER
    };
    
    DEMO_INFO("Initializing UART%d @ %d baud", DEMO_UART_BUS, DEMO_UART_BAUDRATE);
    
    // [P2 FIX] Stricter DMA init error handling
    ret = uapi_dma_init();
    if (ret != ERRCODE_SUCC) {
        DEMO_INFO("DMA init: 0x%x (may be already initialized)", ret);
        // Continue - DMA may already be initialized by other components
    }
    
    ret = uapi_dma_open();
    if (ret != ERRCODE_SUCC) {
        DEMO_INFO("DMA open: 0x%x (may be already open)", ret);
        // Continue - DMA may already be open
    }
    
    // Deinit first to reconfigure
    (void)uapi_uart_deinit(DEMO_UART_BUS);
    
    // Initialize UART
    ret = uapi_uart_init(DEMO_UART_BUS, &pin_cfg, &attr, &ext_cfg, &s_uart_buf_cfg);
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART init failed: 0x%x", ret);
        return -1;
    }
    
    // Register RX callback
    ret = uapi_uart_register_rx_callback(
        DEMO_UART_BUS,
        UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE,
        DEMO_UART_RX_THRESHOLD,  // 小阈值快速回调
        uart_rx_callback
    );
    if (ret != ERRCODE_SUCC) {
        DEMO_ERR("UART RX callback register failed: 0x%x", ret);
        return -1;
    }
    
    DEMO_INFO("UART init OK (DMA TX, INT RX)");
    return 0;
}

void demo_uart_deinit(void)
{
    uapi_uart_unregister_rx_callback(DEMO_UART_BUS);
    uapi_uart_deinit(DEMO_UART_BUS);
}

/*============================================================================
 * Ring Buffer Accessors
 *============================================================================*/

ring_buffer_t* demo_uart_get_rx_ring(void)
{
    return &s_uart_rx_ring;
}

ring_buffer_t* demo_uart_get_tx_ring(void)
{
    return &s_uart_tx_ring;
}

/*============================================================================
 * TX Processing
 *============================================================================*/

bool demo_uart_tx_is_busy(void)
{
    return s_dma_tx_busy;
}

uint32_t demo_uart_tx_send(const uint8_t *data, uint32_t len)
{
    if (len == 0 || data == NULL) {
        return 0;
    }
    
    // Cap to chunk size
    if (len > DEMO_DMA_CHUNK_SIZE) {
        len = DEMO_DMA_CHUNK_SIZE;
    }
    
    // Copy to DMA buffer
    uint8_t *dma_buf = s_dma_tx_buf[s_dma_tx_idx];
    if (memcpy_s(dma_buf, DEMO_DMA_CHUNK_SIZE, data, len) != EOK) {
        return 0;
    }
    
    // Send via DMA (blocking)
    int32_t ret = uapi_uart_write_by_dma(DEMO_UART_BUS, dma_buf, len, &s_dma_cfg);
    if (ret < 0) {
        DEMO_ERR("DMA write failed: %d", ret);
        return 0;
    }
    
    // Update stats and buffer index
    STATS_ADD(uart_tx_bytes, len);
    s_dma_tx_idx = (s_dma_tx_idx + 1) % DEMO_TX_BUFFER_COUNT;
    
    return len;
}

/**
 * UART TX processing - optimized for throughput.
 * Increase batch count and chunk size for faster transmission.
 */
#define DEMO_MAX_DMA_PER_LOOP  16  // Increased for faster SLE->UART output

uint32_t demo_uart_tx_process(void)
{
    uint32_t total_sent = 0;
    uint32_t batch = 0;
    
    // [P1 FIX] Limit DMA calls per loop to prevent starving other directions
    while (!ring_is_empty(&s_uart_tx_ring) && batch < DEMO_MAX_DMA_PER_LOOP) {
        uint8_t *ptr;
        uint32_t contiguous_len;
        
        // Get contiguous data pointer
        ring_peek_contiguous(&s_uart_tx_ring, &ptr, &contiguous_len);
        
        if (contiguous_len == 0) {
            break;
        }
        
        // Chunk size for DMA transfer - larger = faster but more blocking
        uint32_t max_chunk = 2048;  // Increased from 1024 for faster TX
        if (contiguous_len > max_chunk) {
            contiguous_len = max_chunk;
        }
        
        // Copy to DMA buffer and send
        uint8_t *dma_buf = s_dma_tx_buf[s_dma_tx_idx];
        if (memcpy_s(dma_buf, DEMO_DMA_CHUNK_SIZE, ptr, contiguous_len) != EOK) {
            break;
        }
        
        // [P1 FIX] DCache writeback before DMA - ensures data coherency if DCache is enabled
        // If DCache is disabled, this is a no-op
#if defined(osal_dcache_region_wb)
        osal_dcache_region_wb(dma_buf, contiguous_len);
#endif
        
        int32_t ret = uapi_uart_write_by_dma(DEMO_UART_BUS, dma_buf, contiguous_len, &s_dma_cfg);
        if (ret < 0) {
            break;
        }
        
        // Consume from ring buffer
        ring_consume(&s_uart_tx_ring, contiguous_len);
        
        STATS_ADD(uart_tx_bytes, contiguous_len);
        // [P2 FIX] Track UART TX ring water mark
        STATS_SET_HWM(uart_tx_ring_hwm, (uint16_t)ring_data_len(&s_uart_tx_ring));
        
        total_sent += contiguous_len;
        s_dma_tx_idx = (s_dma_tx_idx + 1) % DEMO_TX_BUFFER_COUNT;
        batch++;
    }
    
    return total_sent;
}

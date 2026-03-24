/**
 * Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2022-2022. All
 * rights reserved.
 *
 * Description: Provides uart driver source \n
 *
 * History: \n
 * 2022-06-09, Create file. \n
 */
#include "uart.h"
#include "common_def.h"
#include "hal_uart_v151_regs_op.h"
#include "memory/osal_cache.h"
#include "securec.h"
#include "soc_osal.h"
#include <stdbool.h>
#if defined(CONFIG_UART_SUPPORT_DMA)
#include "dma.h"
#include "dma_porting.h"
#endif

/**
 * @brief  Maximum number of fragments per uart for transmission.
 * Configurable field that specifies the maximum numbers of transmissions the
 * driver can queue before it returns false on the write requests.
 */
#define UART_MAX_NUMBER_OF_FRAGMENTS 4

/**
 * @brief  Uart parity error bit mask
 */
#define UART_PARITY_ERROR_MASK BIT(8)

/**
 * @brief  Uart frame error bit mask.
 */
#define UART_FRAME_ERROR_MASK BIT(7)

#if defined(CONFIG_UART_SUPPORT_TX)
/**
 * @brief  A fragment of data that is to be transmitted.
 */
typedef struct {
  uint8_t *data;
  void *params;
  uart_tx_callback_t release_func;
  uint32_t data_length;
} uart_tx_fragment_t;

/**
 * @brief  The UART transmission configuration parameters.
 */
typedef struct {
  uart_tx_fragment_t
      *current_tx_fragment; /*!< Current TX fragment being transmitted. */
  uart_tx_fragment_t *free_tx_fragment; /*!< The unused TX fragment admin blocks
                                           available for re-use/freeing. */
  uint16_t fragments_to_process; /*!< Number of fragments to process including
                                    the current one. */
  uint16_t
      current_tx_fragment_pos; /*!< Index of the current position of the next
                                  byte to be transmitted in the current TX
                                  fragment current_tx_fragment_pos == 0 means
                                    the first byte is yet to be sent for
                                  transmission */
  uart_tx_fragment_t
      fragment_buffer[UART_MAX_NUMBER_OF_FRAGMENTS]; /*!< Fragments buffer
                                                        pointer. */
} uart_tx_state_t;

/**
 * @brief  Internal UART TX configuration.
 */
#if defined(CONFIG_UART_SUPPORT_TX_INT)
static uart_tx_state_t g_uart_tx_state_array[UART_BUS_MAX_NUM];
#endif

#if defined(CONFIG_SUPPORT_UART_POLL_TIMEOUT)
#define UART_READ_MAX_TIMEOUT 1000000
#endif

#if defined(CONFIG_UART_SUPPORT_DMA)
#define DMA_UART_TRANSFER_TIMEOUT_MS 1000
#define UART_DMA_TRANS_MEMORY_TO_PERIPHERAL_DMA 1
#define UART_DMA_TRANS_PERIPHERAL_TO_MEMORY_DMA 2
#define UART_DMA_TRANSFER_DIR_MEM_TO_PERIPHERAL 0
#define UART_DMA_TRANSFER_DIR_PERIPHERAL_TO_MEM 1
#define UART_DMA_ADDRESS_INC_INCREMENT 0
#define UART_DMA_ADDRESS_INC_NO_CHANGE 2
#define UART_DMA_PROTECTION_CONTROL_BUFFERABLE 1
#define UART_DMA_LLI_MAX_BLOCKS 16U
#define UART_DMA_LLI_TARGET_BLOCK_SIZE 256U
#define UART_DMA_LLI_PENDING_SEGMENTS (UART_DMA_LLI_MAX_BLOCKS + 1U)

typedef struct uart_dma_trans_inf {
  bool inited;
  bool trans_succ;
  uint8_t channel;
  uint8_t reserved;
  osal_semaphore dma_sem;
} uart_dma_trans_inf_t;

static uart_dma_trans_inf_t g_dma_trans[UART_BUS_MAX_NUM] = {0};

#if defined(CONFIG_UART_SUPPORT_RX)
#define UART_DMA_IDLE_PUBLISH_REASON_NONE 0U
#define UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB 1U
#define UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH 2U
#define UART_DMA_IDLE_PUBLISH_REASON_IDLE_FALLBACK 3U
#define UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE 4U
#define UART_DCACHE_LINE_SIZE 32U
#define UART_DMA_LLI_ROLLOVER_FLAG_WRAP BIT(0)
#define UART_DMA_LLI_ROLLOVER_FLAG_REGRESS BIT(1)

typedef struct uart_rx_dma_lli_segment {
  uint16_t offset;
  uint16_t length;
  uint8_t reason;
} uart_rx_dma_lli_segment_t;

typedef struct uart_rx_dma_idle_state {
  bool enabled;
  bool publishing;
  bool idle_publish_seen;
  bool pingpong_allocated;
  bool lli_mode;
  bool deferred_publish; /* true = publish skipped due to in-progress publish */
  uint8_t channel;
  uint16_t transfer_num;
  uint8_t *dma_buffer;
  uint8_t *dma_buffer_alt;
  uint16_t dma_buffer_size;
  uint16_t lli_block_size;
  uint16_t lli_block_count;
  uint16_t lli_active_block;
  uint16_t lli_active_block_published;
  uint16_t lli_last_remaining;
  bool lli_rollover_dma_complete_pending;
  uint8_t lli_skip_dma_complete_count;
  uint8_t lli_pending_head;
  uint8_t lli_pending_tail;
  uint32_t lli_queue_overrun_count;
  uart_rx_dma_lli_segment_t lli_pending[UART_DMA_LLI_PENDING_SEGMENTS];
  uart_write_dma_config_t dma_cfg;
  uart_idle_int_receive_cb_t raw_callback;
  uart_dma_idle_diag_t diag;
  bool idle_debounce_armed;         /* true = waiting for stable DMA position */
  uint16_t idle_debounce_remaining; /* DMA remaining snapshot at arm time */
  uint8_t idle_buffer[CONFIG_UART_FIFO_DEPTH];
} uart_rx_dma_idle_state_t;

static uart_rx_dma_idle_state_t g_uart_rx_dma_idle_state[UART_BUS_MAX_NUM] = {
    0};
static volatile uint32_t g_uart_rx_dma_idle_isr_cnt[UART_BUS_MAX_NUM] = {0};
#endif /* CONFIG_UART_SUPPORT_RX */

static void uart_dma_set_config(uart_bus_t bus,
                                const uart_extra_attr_t *extra_attr);
#endif /* CONFIG_UART_SUPPORT_DMA */

#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX)
/**
 * @brief  The UART reception configuration parameters.
 */
typedef struct {
  uart_rx_callback_t
      rx_callback; /*!< The RX callback to make when the condition is met. */
  uart_error_callback_t
      parity_error_callback;                  /*!< The parity error callback. */
  uart_error_callback_t frame_error_callback; /*!< The frame error callback. */
  uart_error_callback_t
      overrun_error_callback; /*!< The overrun error callback. */
  uint8_t *rx_buffer;         /*!< The RX data buffer. */
  uint16_t rx_buffer_size;    /*!< The size of the receive buffer. */
  uint16_t rx_condition_size; /*!< The size relating the condition. */
  uint16_t new_rx_pos; /*!< Index to the position in the RX buffer that is where
                          new data should be put if (new_rx_pos == 0) the buffer
                          is empty. */
  uart_rx_condition_t
      rx_condition; /*!< The condition under which an RX callback is made. */
} uart_rx_state_t;

/**
 * @brief  Internal UART RX configuration.
 */
static uart_rx_state_t g_uart_rx_state_array[UART_BUS_MAX_NUM];

#endif /* CONFIG_UART_SUPPORT_RX */

static bool g_uart_inited[UART_BUS_MAX_NUM] = {false};

#if defined(CONFIG_UART_SUPPORT_RX)
static bool
uart_config_rx_state(uart_bus_t bus,
                     const uart_buffer_config_t *uart_buffer_config);
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
static void uart_config_tx_state(uart_bus_t bus);
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX) || defined(CONFIG_UART_SUPPORT_TX)
static void uart_deconfig_state(uart_bus_t bus);
#endif /* defined(CONFIG_UART_SUPPORT_RX) || defined(CONFIG_UART_SUPPORT_TX)   \
        */

#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
static bool
uart_helper_add_fragment(uart_bus_t bus, const uint8_t *buffer, uint32_t length,
                         void *params,
                         uart_tx_callback_t finished_with_buffer_func);
static inline bool
uart_helper_is_the_current_fragment_the_last_to_process(uart_bus_t bus);

static inline bool uart_helper_are_there_fragments_to_process(uart_bus_t bus);

static inline bool uart_helper_send_next_char(uart_bus_t bus);

static inline void uart_helper_invoke_current_fragment_callback(uart_bus_t bus);

static inline void uart_helper_move_to_next_fragment(uart_bus_t bus);
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX)
static inline void uart_rx_buffer_release(uart_bus_t bus);

static inline bool uart_rx_buffer_has_free_space(uart_bus_t bus);

static inline uint16_t uart_rx_buffer_data_available(uart_bus_t bus);
#endif /* CONFIG_UART_SUPPORT_RX */

static int32_t uart_check_params_attr(const uart_attr_t *attr);

static int32_t uart_init_check_params(uart_bus_t bus,
                                      const uart_pin_config_t *pins,
                                      const uart_attr_t *attr);

static void uart_claim_pins(uart_bus_t bus, const uart_pin_config_t *pins);

static void uart_release_pins(uart_bus_t bus);

#if defined(CONFIG_UART_SUPPORT_RX)
static void uart_idle_isr(uart_bus_t bus);

static void uart_rx_isr(uart_bus_t bus);

static void uart_error_isr(uart_bus_t bus);
#if defined(CONFIG_UART_SUPPORT_DMA)
static int32_t
uart_read_by_dma_config(uart_bus_t bus, const void *buffer, uint32_t length,
                        uart_write_dma_config_t *dma_cfg,
                        dma_ch_user_peripheral_config_t *user_cfg);
static bool uart_rx_dma_idle_enabled(uart_bus_t bus);
static errcode_t uart_rx_dma_idle_start(uart_bus_t bus);
static void uart_rx_dma_idle_stop(uart_bus_t bus);
static void uart_rx_dma_idle_publish(uart_bus_t bus, uint16_t idle_tail_len,
                                     uint8_t reason);
static bool uart_rx_dma_idle_queue_lli_segment(uart_rx_dma_idle_state_t *state,
                                               uint16_t offset, uint16_t length,
                                               uint8_t reason);
static bool uart_rx_dma_idle_queue_lli_active_partial(uart_bus_t bus,
                                                      uint8_t reason,
                                                      uint16_t *remaining_out,
                                                      uint16_t *partial_out,
                                                      bool *rollover_out);
static void uart_rx_dma_idle_drain_lli(uart_bus_t bus);
#endif
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_TX_INT)
static void uart_tx_isr(uart_bus_t bus);
#endif /* CONFIG_UART_SUPPORT_TX_INIT */

#if defined(CONFIG_UART_SUPPORT_LPM)
static bool g_uart_suspend_flag[UART_BUS_MAX_NUM] = {false};
static uart_pin_config_t g_uart_pins[UART_BUS_MAX_NUM] = {0};
static uart_attr_t g_uart_attr[UART_BUS_MAX_NUM] = {0};
static uart_buffer_config_t g_uart_buffer_config[UART_BUS_MAX_NUM] = {0};
static uart_rx_condition_t g_uart_condition[UART_BUS_MAX_NUM] = {0};
static uint32_t g_uart_size[UART_BUS_MAX_NUM] = {0};
#endif /* CONFIG_UART_SUPPORT_LPM */

#if defined(CONFIG_UART_SUPPORT_LPM) || defined(CONFIG_UART_SUPPORT_RX_THREAD)
STATIC uart_extra_attr_t g_uart_extra_attr[UART_BUS_MAX_NUM] = {0};
STATIC uart_rx_callback_t g_uart_callback[UART_BUS_MAX_NUM] = {0};
#endif

#if defined(CONFIG_UART_SUPPORT_RX_THREAD)
STATIC uart_rx_callback_t g_uart_thread_callback[UART_BUS_MAX_NUM] = {0};
STATIC osal_task *g_uart_rx_thread = NULL;

typedef struct {
  uint32_t cur_heap_size;
} uart_rx_thread_ctrl;

typedef struct {
  void *buf;
  uint16_t buf_len;
  bool error;
  struct osal_list_head node;
} uart_rx_thread_node;

static uart_rx_thread_ctrl g_uart_rx_thread_ctrl = {0};
static struct osal_list_head g_uart_rx_list[UART_BUS_MAX_NUMBER];
static osal_semaphore g_rx_thread_sem;

#if defined(CONFIG_UART_SUPPORT_RX_THREAD_DEBUG)
typedef struct {
  uint32_t cur_heap_size;
  uint32_t max_heap_size;
  uint32_t cur_node_cnt;
  uint32_t max_node_cnt;
  uint32_t max_node_size;
  uint32_t drop_node_size;
} uart_rx_thread_debug;

static uart_rx_thread_debug g_uart_rx_thread_debug;

void uart_rx_thread_debug_print(void) {
  print_str("uart_rx_thread_debug_print :\r\n");
  print_str("buffer size:[%u, %u]\r\n", g_uart_rx_thread_debug.cur_heap_size,
            g_uart_rx_thread_debug.max_heap_size);
  print_str("queue size:[%u, %u, %u, %u]", g_uart_rx_thread_debug.cur_node_cnt,
            g_uart_rx_thread_debug.max_node_cnt,
            g_uart_rx_thread_debug.max_node_size,
            g_uart_rx_thread_debug.drop_node_size);
}

STATIC void uart_rx_thread_debug_increase(uint16_t length) {
  g_uart_rx_thread_debug.cur_heap_size += length;
  g_uart_rx_thread_debug.cur_heap_size += sizeof(uart_rx_thread_node);
  g_uart_rx_thread_debug.cur_node_cnt++;

  if (g_uart_rx_thread_debug.cur_heap_size >
      g_uart_rx_thread_debug.max_heap_size) {
    g_uart_rx_thread_debug.max_heap_size = g_uart_rx_thread_debug.cur_heap_size;
  }

  if (g_uart_rx_thread_debug.cur_node_cnt >
      g_uart_rx_thread_debug.max_node_cnt) {
    g_uart_rx_thread_debug.max_node_cnt = g_uart_rx_thread_debug.cur_node_cnt;
  }

  if (g_uart_rx_thread_debug.max_node_size < length) {
    g_uart_rx_thread_debug.max_node_size = length;
  }
}

STATIC void uart_rx_thread_debug_decrease(uint16_t length) {
  g_uart_rx_thread_debug.cur_heap_size -= length;
  g_uart_rx_thread_debug.cur_heap_size -= sizeof(uart_rx_thread_node);
  g_uart_rx_thread_debug.cur_node_cnt--;
}
#endif

STATIC int uart_rx_thread(void *unused) {
  UNUSED(unused);
  int i;
  uint32_t irq_sts;
  uart_rx_thread_node *rx_list_node = NULL;
  struct osal_list_head *rx_list_entry;
  struct osal_list_head *rx_list_entry_tmp;
  struct osal_list_head rx_list_excute;

  OSAL_INIT_LIST_HEAD(&rx_list_excute);

  while (1) {
    if (osal_sem_down(&g_rx_thread_sem) != OSAL_SUCCESS) {
      continue;
    }
    for (i = 0; i < UART_BUS_MAX_NUMBER; i++) {
      if ((g_uart_extra_attr[i].rx_thread_enable != true) ||
          (osal_list_empty(&(g_uart_rx_list[i])) != 0)) {
        continue;
      }

      irq_sts = osal_irq_lock();
      osal_list_for_each_safe(rx_list_entry, rx_list_entry_tmp,
                              &(g_uart_rx_list[i])) {
        rx_list_node =
            osal_list_entry(rx_list_entry, uart_rx_thread_node, node);
        osal_list_del(rx_list_entry);
        osal_list_add_tail(&(rx_list_node->node), &rx_list_excute);
      }
      osal_irq_restore(irq_sts);

      osal_list_for_each_safe(rx_list_entry, rx_list_entry_tmp,
                              &rx_list_excute) {
        rx_list_node =
            osal_list_entry(rx_list_entry, uart_rx_thread_node, node);
        g_uart_thread_callback[i](rx_list_node->buf, rx_list_node->buf_len,
                                  rx_list_node->error);
        osal_list_del(rx_list_entry);

        irq_sts = osal_irq_lock();
        g_uart_rx_thread_ctrl.cur_heap_size -=
            ((size_t)(sizeof(uart_rx_thread_node)) + (rx_list_node->buf_len));
#if defined(CONFIG_UART_SUPPORT_RX_THREAD_DEBUG)
        uart_rx_thread_debug_decrease(rx_list_node->buf_len);
#endif
        osal_irq_restore(irq_sts);

        osal_kfree(rx_list_node->buf);
        osal_kfree(rx_list_node);
        rx_list_node = NULL;
      }
    }
  }
  return 0;
}

STATIC void uart_rx_thread_trigger(void) { osal_sem_up(&g_rx_thread_sem); }

STATIC void uart_rx_thread_entry(uart_bus_t bus, const void *buffer,
                                 uint16_t length, bool error) {
  if (g_uart_rx_thread_ctrl.cur_heap_size + sizeof(uart_rx_thread_node) +
          length >
      CONFIG_UART_SUPPORT_RX_THREAD_BUFFER_SIZE) {
#if defined(CONFIG_UART_SUPPORT_RX_THREAD_DEBUG)
    g_uart_rx_thread_debug.drop_node_size += length;
#endif
    return;
  }

  uart_rx_thread_node *node =
      osal_kmalloc(sizeof(uart_rx_thread_node), OSAL_GFP_KERNEL);
  if (node == NULL) {
    return;
  }

  node->buf = osal_kmalloc(length, OSAL_GFP_KERNEL);
  if (node->buf == NULL) {
    osal_kfree(node);
    return;
  }

  node->buf_len = length;
  node->error = error;
  errno_t ret = memcpy_s(node->buf, node->buf_len, buffer, length);
  if (ret != EOK) {
    osal_kfree(node->buf);
    osal_kfree(node);
    return;
  }

  uint32_t irq_sts = osal_irq_lock();
  g_uart_rx_thread_ctrl.cur_heap_size +=
      ((size_t)(sizeof(uart_rx_thread_node)) + length);
  osal_list_add_tail(&(node->node), &(g_uart_rx_list[bus]));
#if defined(CONFIG_UART_SUPPORT_RX_THREAD_DEBUG)
  uart_rx_thread_debug_increase(length);
#endif
  osal_irq_restore(irq_sts);

  uart_rx_thread_trigger();
}

#if UART_BUS_MAX_NUMBER > 0
STATIC void uart_rx_thread_callback_0(const void *buffer, uint16_t length,
                                      bool error) {
  if (g_uart_thread_callback[UART_BUS_0] == NULL) {
    return;
  }
  uart_rx_thread_entry(UART_BUS_0, buffer, length, error);
}
#endif

#if UART_BUS_MAX_NUMBER > 1
STATIC void uart_rx_thread_callback_1(const void *buffer, uint16_t length,
                                      bool error) {
  if (g_uart_thread_callback[UART_BUS_1] == NULL) {
    return;
  }
  uart_rx_thread_entry(UART_BUS_1, buffer, length, error);
}
#endif

#if UART_BUS_MAX_NUMBER > 2
STATIC void uart_rx_thread_callback_2(const void *buffer, uint16_t length,
                                      bool error) {
  if (g_uart_thread_callback[UART_BUS_2] == NULL) {
    return;
  }
  uart_rx_thread_entry(UART_BUS_2, buffer, length, error);
}
#endif

#if UART_BUS_MAX_NUMBER > 3
STATIC void uart_rx_thread_callback_3(const void *buffer, uint16_t length,
                                      bool error) {
  if (g_uart_thread_callback[UART_BUS_3] == NULL) {
    return;
  }
  uart_rx_thread_entry(UART_BUS_3, buffer, length, error);
}
#endif

STATIC void uart_rx_thread_hook_callback(uart_bus_t bus,
                                         uart_rx_callback_t callback) {
#if UART_BUS_MAX_NUMBER > 0
  if (bus == UART_BUS_0) {
    g_uart_callback[bus] = uart_rx_thread_callback_0;
  }
#endif

#if UART_BUS_MAX_NUMBER > 1
  if (bus == UART_BUS_1) {
    g_uart_callback[bus] = uart_rx_thread_callback_1;
  }
#endif

#if UART_BUS_MAX_NUMBER > 2
  if (bus == UART_BUS_2) {
    g_uart_callback[bus] = uart_rx_thread_callback_2;
  }
#endif

#if UART_BUS_MAX_NUMBER > 3
  if (bus == UART_BUS_3) {
    g_uart_callback[bus] = uart_rx_thread_callback_3;
  }
#endif
#if defined(CONFIG_UART_SUPPORT_LPM)
  if (g_uart_suspend_flag[bus] == true) {
    return;
  }
#endif
  g_uart_thread_callback[bus] = callback;
}

STATIC errcode_t uart_rx_thread_init(uart_bus_t bus,
                                     uart_rx_callback_t callback) {
  if (!(g_uart_extra_attr[bus].rx_thread_enable)) {
    g_uart_callback[bus] = callback;
    return ERRCODE_SUCC;
  }

  if (g_uart_rx_thread != NULL) {
    goto setup_callback;
  }

  int i;

  osal_kthread_lock();
  g_uart_rx_thread =
      osal_kthread_create(uart_rx_thread, NULL, "uart_rx",
                          CONFIG_UART_SUPPORT_RX_THREAD_STACK_SIZE);
  if (g_uart_rx_thread == NULL) {
    osal_kthread_unlock();
    return ERRCODE_MALLOC;
  }
  osal_kthread_set_priority(g_uart_rx_thread,
                            CONFIG_UART_SUPPORT_RX_THREAD_PRIORITY);
  osal_kthread_unlock();

  for (i = 0; i < UART_BUS_MAX_NUMBER; i++) {
    OSAL_INIT_LIST_HEAD(&(g_uart_rx_list[i]));
  }
  (void)osal_sem_init(&g_rx_thread_sem, 0);
#if defined(CONFIG_UART_SUPPORT_RX_THREAD_DEBUG)
  memset_s(&g_uart_rx_thread_debug, sizeof(uart_rx_thread_debug), 0,
           sizeof(uart_rx_thread_debug));
#endif

setup_callback:
  uart_rx_thread_hook_callback(bus, callback);
  return ERRCODE_SUCC;
}
#endif

static errcode_t uart_evt_callback(uart_bus_t bus, hal_uart_evt_id_t evt,
                                   uintptr_t param);

errcode_t uapi_uart_init(uart_bus_t bus, const uart_pin_config_t *pins,
                         const uart_attr_t *attr,
                         const uart_extra_attr_t *extra_attr,
                         uart_buffer_config_t *uart_buffer_config) {
  unused(uart_buffer_config);
  if (uart_init_check_params(bus, pins, attr) != 0) {
    return ERRCODE_INVALID_PARAM;
  }
  if (g_uart_inited[bus]) {
    return ERRCODE_SUCC;
  }
#if defined(CONFIG_UART_SUPPORT_LPM)
  if (g_uart_suspend_flag[bus] == false) {
    (void)memcpy_s(&g_uart_pins[bus], sizeof(uart_pin_config_t), pins,
                   sizeof(uart_pin_config_t));
    (void)memcpy_s(&g_uart_attr[bus], sizeof(uart_attr_t), attr,
                   sizeof(uart_attr_t));
    (void)memcpy_s(&g_uart_extra_attr[bus], sizeof(uart_extra_attr_t),
                   extra_attr, sizeof(uart_extra_attr_t));
    (void)memcpy_s(&g_uart_buffer_config[bus], sizeof(uart_buffer_config_t),
                   uart_buffer_config, sizeof(uart_buffer_config_t));
  }
#endif /* CONFIG_UART_SUPPORT_LPM */
#if defined(CONFIG_UART_SUPPORT_RX_THREAD)
  g_uart_extra_attr[bus].rx_thread_enable =
      (extra_attr != NULL) ? extra_attr->rx_thread_enable : 0;
#endif
#if defined(CONFIG_UART_SUPPORT_LPC)
  uart_port_clock_enable(bus, true);
#endif
  uart_claim_pins(bus, pins);

#if defined(CONFIG_UART_SUPPORT_RX)
  if (uart_config_rx_state(bus, uart_buffer_config) == false) {
    return ERRCODE_UART_INIT_TRX_STATE_FAIL;
  }
#endif /* CONFIG_UART_SUPPORT_RX */
#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
  uart_config_tx_state(bus);
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */
  uint8_t flow_ctrl = UART_FLOW_CTRL_SOFT;
#if defined(CONFIG_UART_SUPPORT_FLOW_CTRL)
  flow_ctrl = attr->flow_ctrl;
#endif /* CONFIG_UART_SUPPORT_FLOW_CTRL */
  errcode_t ret =
      hal_uart_init(bus, uart_evt_callback, (hal_uart_pin_config_t *)pins,
                    (hal_uart_attr_t *)attr, (hal_uart_flow_ctrl_t)flow_ctrl,
                    (hal_uart_extra_attr_t *)extra_attr);
  if (ret != ERRCODE_SUCC) {
    return ret;
  }

#if defined(CONFIG_UART_SUPPORT_DMA)
  if ((extra_attr != NULL) &&
      (extra_attr->tx_dma_enable || extra_attr->rx_dma_enable)) {
    uart_dma_set_config(bus, extra_attr);
  }
#else
  unused(extra_attr);
#endif /* CONFIG_UART_SUPPORT_DMA */
  g_uart_inited[bus] = true;
  uart_port_register_irq(bus);
  return ret;
}

errcode_t uapi_uart_deinit(uart_bus_t bus) {
  errcode_t ret = ERRCODE_FAIL;
  if (bus >= UART_BUS_MAX_NUM) {
    return ERRCODE_INVALID_PARAM;
  }
  if (!g_uart_inited[bus]) {
    return ERRCODE_SUCC;
  }
#if defined(CONFIG_UART_SUPPORT_DMA) && defined(CONFIG_UART_SUPPORT_RX)
  uapi_uart_unregister_read_by_dma_callback(bus);
#endif
  ret = hal_uart_deinit(bus);

  uart_port_unregister_irq(bus);

#if defined(CONFIG_UART_SUPPORT_RX) || defined(CONFIG_UART_SUPPORT_TX)
  uart_deconfig_state(bus);
#endif /* defined(CONFIG_UART_SUPPORT_RX) || defined(CONFIG_UART_SUPPORT_TX)   \
        */

  uart_release_pins(bus);

#if defined(CONFIG_UART_SUPPORT_DMA)
  if (g_dma_trans[bus].inited) {
    osal_sem_destroy(&(g_dma_trans[bus].dma_sem));
    g_dma_trans[bus].inited = false;
  }
#endif /* CONFIG_UART_SUPPORT_DMA */
#if defined(CONFIG_UART_SUPPORT_LPC)
  uart_port_clock_enable(bus, false);
#endif
  g_uart_inited[bus] = false;
  return ret;
}

errcode_t uapi_uart_set_attr(uart_bus_t bus, const uart_attr_t *attr) {
  if (bus >= UART_BUS_MAX_NUM) {
    return ERRCODE_INVALID_PARAM;
  }
  if ((uart_check_params_attr(attr)) != 0) {
    return ERRCODE_INVALID_PARAM;
  }
  uint32_t irq_sts = uart_porting_lock(bus);
  errcode_t ret = hal_uart_ctrl(bus, UART_CTRL_SET_ATTR, (uintptr_t)attr);
  uart_porting_unlock(bus, irq_sts);
  return ret;
}

errcode_t uapi_uart_get_attr(uart_bus_t bus, const uart_attr_t *attr) {
  if (bus >= UART_BUS_MAX_NUM || attr == NULL) {
    return ERRCODE_INVALID_PARAM;
  }
  uint32_t irq_sts = uart_porting_lock(bus);
  errcode_t ret = hal_uart_ctrl(bus, UART_CTRL_GET_ATTR, (uintptr_t)attr);
  uart_porting_unlock(bus, irq_sts);
  return ret;
}

#if defined(CONFIG_UART_SUPPORT_RX)
errcode_t uapi_uart_register_rx_callback(uart_bus_t bus,
                                         uart_rx_condition_t condition,
                                         uint32_t size,
                                         uart_rx_callback_t callback) {
  errcode_t ret = ERRCODE_FAIL;

  if (bus >= UART_BUS_MAX_NUM || callback == NULL) {
    return ERRCODE_INVALID_PARAM;
  }

#if defined(CONFIG_UART_SUPPORT_LPM)
  if (g_uart_suspend_flag[bus] == false) {
    memcpy_s(&g_uart_condition[bus], sizeof(uart_rx_condition_t), &condition,
             sizeof(uart_rx_condition_t));
    memcpy_s(&g_uart_size[bus], sizeof(uint32_t), &size, sizeof(uint32_t));
    memcpy_s(&g_uart_callback[bus], sizeof(uart_rx_callback_t), &callback,
             sizeof(uart_rx_callback_t));
  }
#endif /* CONFIG_UART_SUPPORT_LPM */
#if defined(CONFIG_UART_SUPPORT_RX_THREAD)
  (void)uart_rx_thread_init(bus, callback);
#endif

  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uint32_t irq_sts = uart_porting_lock(bus);
#if defined(CONFIG_UART_SUPPORT_RX_THREAD)
  rx_state->rx_callback = g_uart_callback[bus];
#else
  rx_state->rx_callback = callback;
#endif
  rx_state->rx_condition = condition;
#if !defined(CONFIG_UART_NOT_SUPPORT_RX_CONDITON_SIZE_OPTIMIZE)
  uint32_t uart_rx_fifo_thresh = 0;
  ret = hal_uart_ctrl(bus, UART_CTRL_GET_RX_FIFO_THRESHOLD,
                      (uintptr_t)&uart_rx_fifo_thresh);
  size = size > uart_rx_fifo_thresh ? uart_rx_fifo_thresh : size;
#endif
  rx_state->rx_condition_size = (uint16_t)size;
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_RX_INT, 1);
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_FRAME_ERR_INT, 1);
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_PARITY_ERR_INT, 1);
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_IDLE_INT, 1);
  uart_porting_unlock(bus, irq_sts);

  return ret;
}

errcode_t
uapi_uart_register_parity_error_callback(uart_bus_t bus,
                                         uart_error_callback_t callback) {
  errcode_t ret = ERRCODE_FAIL;

  if (bus >= UART_BUS_MAX_NUM || callback == NULL) {
    return ERRCODE_INVALID_PARAM;
  }
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uint32_t irq_sts = uart_porting_lock(bus);
  rx_state->parity_error_callback = callback;
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_PARITY_ERR_INT, 1);
  uart_porting_unlock(bus, irq_sts);

  return ret;
}

errcode_t
uapi_uart_register_frame_error_callback(uart_bus_t bus,
                                        uart_error_callback_t callback) {
  errcode_t ret = ERRCODE_FAIL;

  if (bus >= UART_BUS_MAX_NUM || callback == NULL) {
    return ERRCODE_INVALID_PARAM;
  }
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uint32_t irq_sts = uart_porting_lock(bus);
  rx_state->frame_error_callback = callback;
  ret = hal_uart_ctrl(bus, UART_CTRL_EN_FRAME_ERR_INT, 1);
  uart_porting_unlock(bus, irq_sts);

  return ret;
}

errcode_t
uapi_uart_register_overrun_error_callback(uart_bus_t bus,
                                          uart_error_callback_t callback) {
  if (bus >= UART_BUS_MAX_NUM || callback == NULL) {
    return ERRCODE_INVALID_PARAM;
  }
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uint32_t irq_sts = uart_porting_lock(bus);
  rx_state->overrun_error_callback = callback;
  uart_porting_unlock(bus, irq_sts);

  return ERRCODE_SUCC;
}
#endif /* CONFIG_UART_SUPPORT_RX */

static int32_t uapi_uart_param_check(uart_bus_t bus, const uint8_t *buffer,
                                     uint32_t length) {
  if (bus >= UART_BUS_MAX_NUM || buffer == NULL || length == 0) {
    return ERRCODE_INVALID_PARAM;
  }
  if (!g_uart_inited[bus]) {
    return ERRCODE_UART_NOT_INIT;
  }

  return ERRCODE_SUCC;
}

#if defined(CONFIG_UART_SUPPORT_TX)
int32_t uapi_uart_write(uart_bus_t bus, const uint8_t *buffer, uint32_t length,
                        uint32_t timeout) {
  unused(timeout);
  bool tx_fifo_full = false;
  uint8_t *data_buffer = (uint8_t *)buffer;
  int32_t write_count = 0;
  uint32_t len = length;

  int32_t ret = uapi_uart_param_check(bus, buffer, length);
  if (ret != ERRCODE_SUCC) {
    return ret;
  }

  uint32_t irq_sts = uart_porting_lock(bus);
  while (len > 0) {
    hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_FIFO_FULL, (uintptr_t)&tx_fifo_full);
    if (tx_fifo_full == false) {
      hal_uart_write(bus, data_buffer++, 1);
      len--;
      write_count++;
    }
  }
  uart_porting_unlock(bus, irq_sts);

  return write_count;
}

#if defined(CONFIG_UART_SUPPORT_TX_INT)
static void uapi_uart_data_send(uart_bus_t bus) {
  bool tx_fifo_full = false;
  hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_FIFO_FULL, (uintptr_t)&tx_fifo_full);

  /* Populate the UART TX FIFO if there is data to send */
  while (tx_fifo_full == false) {
    /* There is some data to transmit so provide another byte to the UART */
    bool end_of_fragment = uart_helper_send_next_char(bus);
    if (end_of_fragment) {
      /* If it is the end of the fragment invoke the callback and move to the
       * next one */
      uart_helper_invoke_current_fragment_callback(bus);
      uart_helper_move_to_next_fragment(bus);
      /* As it was the only fragment leave */
      break;
    }

    hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_FIFO_FULL, (uintptr_t)&tx_fifo_full);
  }
}

errcode_t uapi_uart_write_int(uart_bus_t bus, const uint8_t *buffer,
                              uint32_t length, void *params,
                              uart_tx_callback_t finished_with_buffer_func) {
  errcode_t ret = (errcode_t)uapi_uart_param_check(bus, buffer, length);
  if (ret != ERRCODE_SUCC) {
    return ret;
  }

  uint32_t irq_sts = uart_porting_lock(bus);
  if (uart_helper_are_there_fragments_to_process(bus) == true) {
    uapi_uart_data_send(bus);
  }

  bool fragment_added = uart_helper_add_fragment(bus, buffer, length, params,
                                                 finished_with_buffer_func);
  if (!fragment_added) {
    uart_porting_unlock(bus, irq_sts);
    return ERRCODE_UART_ADD_QUEUE_FAIL;
  }
  /* If it is the first on the list process it */
  if (uart_helper_is_the_current_fragment_the_last_to_process(bus) ==
      true) { /* No other fragments require transmission so start the
                 transmission */
    uapi_uart_data_send(bus);
    /* if we have not finished transmitting it enable the interrupts */
    if (uart_helper_are_there_fragments_to_process(bus) ==
        true) { /* if it is not finished transmitting it */
      hal_uart_ctrl(bus, UART_CTRL_EN_TX_INT, true);
    }
  }
  uart_porting_unlock(bus, irq_sts);
  return ERRCODE_SUCC;
}
#endif

#if defined(CONFIG_UART_SUPPORT_DMA)
static void uart_dma_set_config(uart_bus_t bus,
                                const uart_extra_attr_t *extra_attr) {
  hal_uart_set_dma_config(bus, (hal_uart_extra_attr_t *)extra_attr);
  (void)memset_s(&(g_dma_trans[bus].dma_sem), sizeof(g_dma_trans[bus].dma_sem),
                 0, sizeof(g_dma_trans[bus].dma_sem));
  (void)osal_sem_init(&(g_dma_trans[bus].dma_sem), 0);
  g_dma_trans[bus].inited = true;
}

#if defined(CONFIG_UART_SUPPORT_RX)
static bool uart_rx_dma_idle_enabled(uart_bus_t bus) {
  if (bus >= UART_BUS_MAX_NUM) {
    return false;
  }
  return g_uart_rx_dma_idle_state[bus].enabled;
}

static uint16_t
uart_rx_dma_idle_get_lli_block_size(const uart_rx_dma_idle_state_t *state) {
  uint16_t align;
  uint16_t block_size;
  uint16_t block_count;

  if ((state == NULL) || (state->dma_buffer_size < 2U)) {
    return 0U;
  }

  align = (uint16_t)bit(state->dma_cfg.src_width);
  if (align == 0U) {
    return 0U;
  }

  block_size = (state->dma_buffer_size < UART_DMA_LLI_TARGET_BLOCK_SIZE)
                   ? state->dma_buffer_size
                   : UART_DMA_LLI_TARGET_BLOCK_SIZE;
  block_size = (uint16_t)(block_size - (block_size % align));
  if (block_size < align) {
    block_size = align;
  }
  while (block_size <= state->dma_buffer_size) {
    if ((state->dma_buffer_size % block_size) == 0U) {
      block_count = (uint16_t)(state->dma_buffer_size / block_size);
      if ((block_count >= 2U) && (block_count <= UART_DMA_LLI_MAX_BLOCKS)) {
        return block_size;
      }
    }
    block_size = (uint16_t)(block_size + align);
  }

  return 0U;
}

static void
uart_rx_dma_idle_diag_capture_lli_segment(uart_rx_dma_idle_state_t *state,
                                          uint16_t offset, uint16_t length,
                                          uint8_t reason) {
  uint8_t copy_len;

  if (state == NULL) {
    return;
  }

  state->diag.lli_last_segment_reason = reason;
  state->diag.lli_last_segment_offset = offset;
  state->diag.lli_last_segment_length = length;
  state->diag.lli_last_segment_prefix_len = 0U;
  (void)memset_s(state->diag.lli_last_segment_prefix,
                 sizeof(state->diag.lli_last_segment_prefix), 0,
                 sizeof(state->diag.lli_last_segment_prefix));
  if ((state->dma_buffer == NULL) || (length == 0U) ||
      (offset >= state->dma_buffer_size) ||
      ((uint32_t)offset + length > state->dma_buffer_size)) {
    return;
  }

  copy_len = (uint8_t)((length < sizeof(state->diag.lli_last_segment_prefix))
                           ? length
                           : sizeof(state->diag.lli_last_segment_prefix));
  state->diag.lli_last_segment_prefix_len = copy_len;
  (void)memcpy_s(state->diag.lli_last_segment_prefix,
                 sizeof(state->diag.lli_last_segment_prefix),
                 state->dma_buffer + offset, copy_len);
}

static void uart_rx_dma_idle_diag_record_lli_rollover(
    uart_rx_dma_idle_state_t *state, uint16_t remaining, uint16_t partial_len,
    uint8_t flags, uint16_t start_offset, uint16_t remain_len) {
  uint8_t copy_len;

  if (state == NULL) {
    return;
  }

  state->diag.lli_rollover_count++;
  if ((flags & UART_DMA_LLI_ROLLOVER_FLAG_WRAP) != 0U) {
    state->diag.lli_rollover_wrap_count++;
  }
  if ((flags & UART_DMA_LLI_ROLLOVER_FLAG_REGRESS) != 0U) {
    state->diag.lli_rollover_regress_count++;
  }
  state->diag.lli_last_rollover_publish_seq = state->diag.last_publish_seq;
  state->diag.lli_last_rollover_block = state->lli_active_block;
  state->diag.lli_last_rollover_prev_published =
      state->lli_active_block_published;
  state->diag.lli_last_rollover_partial_len = partial_len;
  state->diag.lli_last_rollover_prev_remaining = state->lli_last_remaining;
  state->diag.lli_last_rollover_remaining = remaining;
  state->diag.lli_last_rollover_queue_offset = start_offset;
  state->diag.lli_last_rollover_queue_len = remain_len;
  state->diag.lli_last_rollover_flags = flags;
  state->diag.lli_last_rollover_prefix_len = 0U;
  (void)memset_s(state->diag.lli_last_rollover_prefix,
                 sizeof(state->diag.lli_last_rollover_prefix), 0,
                 sizeof(state->diag.lli_last_rollover_prefix));
  if ((state->dma_buffer == NULL) || (remain_len == 0U) ||
      (start_offset >= state->dma_buffer_size) ||
      ((uint32_t)start_offset + remain_len > state->dma_buffer_size)) {
    return;
  }
  copy_len =
      (uint8_t)((remain_len < sizeof(state->diag.lli_last_rollover_prefix))
                    ? remain_len
                    : sizeof(state->diag.lli_last_rollover_prefix));
  state->diag.lli_last_rollover_prefix_len = copy_len;
  (void)memcpy_s(state->diag.lli_last_rollover_prefix,
                 sizeof(state->diag.lli_last_rollover_prefix),
                 state->dma_buffer + start_offset, copy_len);
}

static bool uart_rx_dma_idle_queue_lli_segment(uart_rx_dma_idle_state_t *state,
                                               uint16_t offset, uint16_t length,
                                               uint8_t reason) {
  uint8_t next_tail;
  uart_rx_dma_lli_segment_t *segment;

  if ((state == NULL) || (length == 0U) || (length > state->dma_buffer_size) ||
      (offset >= state->dma_buffer_size) ||
      ((uint32_t)offset + length > state->dma_buffer_size)) {
    return false;
  }

  next_tail =
      (uint8_t)((state->lli_pending_tail + 1U) % UART_DMA_LLI_PENDING_SEGMENTS);
  if (next_tail == state->lli_pending_head) {
    state->lli_queue_overrun_count++;
    state->enabled = false;
    return false;
  }

  segment = &state->lli_pending[state->lli_pending_tail];
  segment->offset = offset;
  segment->length = length;
  segment->reason = reason;
  uart_rx_dma_idle_diag_capture_lli_segment(state, offset, length, reason);
  state->lli_pending_tail = next_tail;
  return true;
}

static bool uart_rx_dma_idle_sync_lli_rollover(uart_rx_dma_idle_state_t *state,
                                               uint16_t remaining) {
  uint16_t start_offset;
  uint16_t actual_end;
  uint16_t remain_len;
  uint16_t partial_len;
  bool remaining_wrapped;
  bool partial_regressed;
  uint8_t rollover_flags = 0U;

  if ((state == NULL) || !state->enabled || !state->lli_mode ||
      (state->lli_block_size == 0U)) {
    return false;
  }

  partial_len = (uint16_t)(state->lli_block_size -
                           (remaining << state->dma_cfg.src_width));
  if (partial_len > state->lli_block_size) {
    partial_len = state->lli_block_size;
  }
  remaining_wrapped = ((state->lli_last_remaining < state->transfer_num) &&
                       (remaining > state->lli_last_remaining));
  partial_regressed = (partial_len < state->lli_active_block_published);
  if (remaining_wrapped) {
    rollover_flags |= UART_DMA_LLI_ROLLOVER_FLAG_WRAP;
  }
  if (partial_regressed) {
    rollover_flags |= UART_DMA_LLI_ROLLOVER_FLAG_REGRESS;
  }
  if (!remaining_wrapped && !partial_regressed) {
    return false;
  }

  start_offset = (uint16_t)(state->lli_active_block * state->lli_block_size +
                            state->lli_active_block_published);
  actual_end = (partial_len > state->lli_active_block_published)
                   ? partial_len
                   : state->lli_block_size;
  remain_len = (uint16_t)(actual_end - state->lli_active_block_published);
  uart_rx_dma_idle_diag_record_lli_rollover(
      state, remaining, partial_len, rollover_flags, start_offset, remain_len);
  if (remain_len > 0U) {
    if (!uart_rx_dma_idle_queue_lli_segment(
            state, start_offset, remain_len,
            UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE)) {
      return false;
    }
  }

  state->lli_active_block =
      (uint16_t)((state->lli_active_block + 1U) % state->lli_block_count);
  state->lli_active_block_published = 0U;
  state->lli_last_remaining = state->transfer_num;
  state->lli_rollover_dma_complete_pending = true;
  if (state->lli_skip_dma_complete_count < 0xFFU) {
    state->lli_skip_dma_complete_count++;
  }
  return true;
}

static bool uart_rx_dma_idle_queue_lli_active_partial(uart_bus_t bus,
                                                      uint8_t reason,
                                                      uint16_t *remaining_out,
                                                      uint16_t *partial_out,
                                                      bool *rollover_out) {
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];
  uint16_t remaining;
  uint16_t partial_len;
  uint16_t start_offset;
  uint8_t channel;
  uint8_t pending_tail_before;

  if (!state->enabled || !state->lli_mode || (state->channel == 0U)) {
    return false;
  }
  if (rollover_out != NULL) {
    *rollover_out = false;
  }

  channel = (uint8_t)(state->channel - 1U);
  remaining = (uint16_t)uapi_dma_get_block_ts(channel);
  if (remaining > state->transfer_num) {
    remaining = state->transfer_num;
  }

  pending_tail_before = state->lli_pending_tail;
  if (uart_rx_dma_idle_sync_lli_rollover(state, remaining)) {
    if (rollover_out != NULL) {
      *rollover_out = true;
    }
    channel = (uint8_t)(state->channel - 1U);
    remaining = (uint16_t)uapi_dma_get_block_ts(channel);
    if (remaining > state->transfer_num) {
      remaining = state->transfer_num;
    }
    partial_len = (uint16_t)(state->lli_block_size -
                             (remaining << state->dma_cfg.src_width));
    if (partial_len > state->lli_block_size) {
      partial_len = state->lli_block_size;
    }
    state->lli_last_remaining = remaining;
    if (remaining_out != NULL) {
      *remaining_out = remaining;
    }
    if (partial_out != NULL) {
      *partial_out = partial_len;
    }
    return (state->lli_pending_tail != pending_tail_before);
  }

  partial_len = (uint16_t)(state->lli_block_size -
                           (remaining << state->dma_cfg.src_width));
#ifdef CONFIG_SUPPORT_DATA_CACHE
  /* Never publish the cache line DMA may still be updating. Leave the tail to
     the next soft-flush or the DMA-complete path. */
  if (partial_len >
      (uint16_t)(state->lli_active_block_published + UART_DCACHE_LINE_SIZE)) {
    partial_len =
        (uint16_t)(partial_len & ~(uint16_t)(UART_DCACHE_LINE_SIZE - 1U));
  }
#endif
  if ((partial_len > state->lli_block_size) ||
      (partial_len <= state->lli_active_block_published)) {
    state->lli_last_remaining = remaining;
    if (remaining_out != NULL) {
      *remaining_out = remaining;
    }
    if (partial_out != NULL) {
      *partial_out = partial_len;
    }
    return false;
  }

  if (state->lli_rollover_dma_complete_pending) {
    state->lli_last_remaining = remaining;
    if (remaining_out != NULL) {
      *remaining_out = remaining;
    }
    if (partial_out != NULL) {
      *partial_out = partial_len;
    }
    return false;
  }

  start_offset = (uint16_t)(state->lli_active_block * state->lli_block_size +
                            state->lli_active_block_published);
  if (!uart_rx_dma_idle_queue_lli_segment(
          state, start_offset,
          (uint16_t)(partial_len - state->lli_active_block_published),
          reason)) {
    if (remaining_out != NULL) {
      *remaining_out = remaining;
    }
    if (partial_out != NULL) {
      *partial_out = partial_len;
    }
    return false;
  }

  state->lli_active_block_published = partial_len;
  state->lli_last_remaining = remaining;
  if (remaining_out != NULL) {
    *remaining_out = remaining;
  }
  if (partial_out != NULL) {
    *partial_out = partial_len;
  }
  return true;
}

static void uart_rx_dma_idle_drain_lli(uart_bus_t bus) {
  uart_rx_dma_idle_state_t *state;
  uart_rx_dma_lli_segment_t segment;
  bool keep_receiving = true;

  if (bus >= UART_BUS_MAX_NUM) {
    return;
  }

  state = &g_uart_rx_dma_idle_state[bus];
  while (keep_receiving) {
    uint32_t irq_sts = osal_irq_lock();

    if (!state->enabled ||
        (state->lli_pending_head == state->lli_pending_tail) ||
        (state->raw_callback == NULL)) {
      state->publishing = false;
      osal_irq_restore(irq_sts);
      break;
    }

    segment = state->lli_pending[state->lli_pending_head];
    state->lli_pending_head = (uint8_t)((state->lli_pending_head + 1U) %
                                        UART_DMA_LLI_PENDING_SEGMENTS);
    if (state->lli_pending_head != state->lli_pending_tail) {
      state->diag.deferred_publish_drained_count++;
      state->diag.last_deferred_publish_drained_seq =
          state->diag.last_publish_seq;
    }
    osal_irq_restore(irq_sts);

    state->diag.publish_count++;
    state->diag.publish_bytes += segment.length;
    state->diag.last_publish_seq = state->diag.publish_count;
    state->diag.last_transfer_num = state->transfer_num;
    state->diag.last_remaining = 0U;
    state->diag.last_received_blocks = 1U;
    state->diag.last_received_len = segment.length;
    state->diag.last_idle_tail_len = 0U;
    state->diag.last_combined_len = segment.length;
    state->diag.last_publish_reason = segment.reason;
    state->diag.last_fifo_drain_len = 0U;
    if (segment.reason == UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB) {
      state->diag.publish_from_idle_cb++;
    } else if (segment.reason == UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH) {
      state->diag.publish_from_soft_flush++;
    } else if (segment.reason == UART_DMA_IDLE_PUBLISH_REASON_IDLE_FALLBACK) {
      state->diag.publish_from_idle_fallback++;
    } else if (segment.reason == UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE) {
      state->diag.publish_from_dma_complete++;
      state->diag.last_dma_complete_seq = state->diag.last_publish_seq;
    }

#ifdef CONFIG_SUPPORT_DATA_CACHE
    osal_dcache_region_inv(state->dma_buffer + segment.offset, segment.length);
#endif
    keep_receiving =
        state->raw_callback(state->dma_buffer + segment.offset, segment.length);
    if (!keep_receiving) {
      irq_sts = osal_irq_lock();
      state->enabled = false;
      state->publishing = false;
      osal_irq_restore(irq_sts);
      uart_rx_dma_idle_stop(bus);
      break;
    }
  }
}

static void uart_rx_dma_idle_dma_isr(uint8_t int_type, uint8_t ch,
                                     uintptr_t arg) {
  uart_bus_t bus = (uart_bus_t)arg;

  unused(ch);
  if ((int_type != 0) || (bus >= UART_BUS_MAX_NUM)) {
    return;
  }

  uart_rx_dma_idle_publish(bus, 0U, UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE);
}

static errcode_t uart_rx_dma_idle_start(uart_bus_t bus) {
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];
  dma_ch_user_peripheral_config_t user_cfg = {0};
  uint8_t dma_ch;
  int32_t ret;
#if defined(CONFIG_DMA_SUPPORT_LLI)
  uint16_t block_idx;
#endif

  if ((state->dma_buffer == NULL) || (state->dma_buffer_size == 0U)) {
    return ERRCODE_INVALID_PARAM;
  }

  ret = uart_read_by_dma_config(bus, state->dma_buffer,
                                state->lli_mode ? state->lli_block_size
                                                : state->dma_buffer_size,
                                &state->dma_cfg, &user_cfg);
  if (ret != ERRCODE_SUCC) {
    return (errcode_t)ret;
  }
  if (user_cfg.src_handshaking == HAL_DMA_HANDSHAKING_MAX_NUM) {
    return ERRCODE_FAIL;
  }

#if defined(CONFIG_DMA_SUPPORT_LLI)
  if (state->lli_mode) {
    dma_ch = uapi_dma_get_lli_channel((uint8_t)state->dma_cfg.burst_length,
                                      user_cfg.src_handshaking);
    if (dma_ch >= DMA_CHANNEL_MAX_NUM) {
      return ERRCODE_DMA_RET_NO_AVAIL_CH;
    }

    state->lli_pending_head = 0U;
    state->lli_pending_tail = 0U;
    state->lli_active_block = 0U;
    state->lli_active_block_published = 0U;
    user_cfg.transfer_num =
        (uint16_t)(state->lli_block_size >> state->dma_cfg.src_width);
    state->lli_last_remaining = user_cfg.transfer_num;
    state->lli_rollover_dma_complete_pending = false;
    state->lli_skip_dma_complete_count = 0U;
    for (block_idx = 0U; block_idx < state->lli_block_count; block_idx++) {
      user_cfg.dest = (uint32_t)(uintptr_t)(state->dma_buffer +
                                            block_idx * state->lli_block_size);
      ret = uapi_dma_configure_peripheral_transfer_lli(dma_ch, &user_cfg, NULL);
      if (ret != ERRCODE_SUCC) {
        (void)uapi_dma_end_transfer(dma_ch);
        return (errcode_t)ret;
      }
    }

    state->channel = (uint8_t)(dma_ch + 1U);
    state->transfer_num = user_cfg.transfer_num;
    ret = uapi_dma_enable_lli(dma_ch, uart_rx_dma_idle_dma_isr, (uintptr_t)bus);
    if (ret != ERRCODE_SUCC) {
      (void)uapi_dma_end_transfer(dma_ch);
      state->channel = 0U;
      return (errcode_t)ret;
    }
    return ERRCODE_SUCC;
  }
#endif

  ret = uapi_dma_configure_peripheral_transfer_single(
      &user_cfg, &dma_ch, uart_rx_dma_idle_dma_isr, (uintptr_t)bus);
  if (ret != ERRCODE_SUCC) {
    return (errcode_t)ret;
  }

  state->channel = (uint8_t)(dma_ch + 1U);
  state->transfer_num = user_cfg.transfer_num;
  ret = uapi_dma_start_transfer(dma_ch);
  if (ret != ERRCODE_SUCC) {
    state->channel = 0;
    return (errcode_t)ret;
  }

  return ERRCODE_SUCC;
}

static uart_bus_t uart_rx_dma_idle_find_bus_by_buffer(const void *buffer) {
  uart_bus_t bus;

  for (bus = UART_BUS_0; bus < UART_BUS_MAX_NUM; bus++) {
    uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];
    if (state->enabled && buffer == state->idle_buffer) {
      return bus;
    }
  }
  return UART_BUS_MAX_NUM;
}

static void uart_rx_dma_idle_restore_rx_buffer(uart_bus_t bus) {
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];

  rx_state->rx_buffer = state->dma_buffer;
  rx_state->rx_buffer_size = state->dma_buffer_size;
  rx_state->new_rx_pos = 0;
}

static void uart_rx_dma_idle_prepare_idle_buffer(uart_bus_t bus);

static void uart_rx_dma_idle_publish(uart_bus_t bus, uint16_t idle_tail_len,
                                     uint8_t reason) {
  uart_rx_dma_idle_state_t *state;
  uint16_t remaining;
  uint16_t received_blocks;
  uint16_t received_len;
  uint16_t combined_len;
  uint16_t fifo_drain = 0;
  uint8_t channel;
  uint8_t *completed_buf;
  bool keep_receiving = true;
  bool fifo_empty = false;

  if (bus >= UART_BUS_MAX_NUM) {
    return;
  }

  state = &g_uart_rx_dma_idle_state[bus];
  if (state->lli_mode) {
    bool queued = false;
    bool stop_dma = false;
    uint32_t irq_sts = osal_irq_lock();

    if (!state->enabled || (state->channel == 0U) ||
        (state->dma_buffer == NULL) || (state->dma_buffer_size == 0U)) {
      osal_irq_restore(irq_sts);
      return;
    }

    state->idle_publish_seen = true;
    state->idle_debounce_armed = false;
    if (reason == UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE) {
      if (state->lli_skip_dma_complete_count > 0U) {
        state->lli_skip_dma_complete_count--;
        state->lli_last_remaining = state->transfer_num;
        state->lli_rollover_dma_complete_pending = false;
        osal_irq_restore(irq_sts);
        return;
      }
      uint16_t start_offset =
          (uint16_t)(state->lli_active_block * state->lli_block_size +
                     state->lli_active_block_published);
      uint16_t remain_len =
          (uint16_t)(state->lli_block_size - state->lli_active_block_published);

      if (remain_len > 0U) {
        queued = uart_rx_dma_idle_queue_lli_segment(state, start_offset,
                                                    remain_len, reason);
      }
      state->lli_active_block =
          (uint16_t)((state->lli_active_block + 1U) % state->lli_block_count);
      state->lli_active_block_published = 0U;
      state->lli_last_remaining = state->transfer_num;
    } else {
      uint16_t remaining = 0U;
      uint16_t partial_len = 0U;

      queued = uart_rx_dma_idle_queue_lli_active_partial(
          bus, reason, &remaining, &partial_len, NULL);
      state->diag.last_transfer_num = state->transfer_num;
      state->diag.last_remaining = remaining;
      state->diag.last_received_blocks = 0U;
      state->diag.last_received_len = partial_len;
    }

    if (!queued) {
      stop_dma = !state->enabled;
      osal_irq_restore(irq_sts);
      if (stop_dma) {
        uart_rx_dma_idle_stop(bus);
      }
      return;
    }

    if (state->publishing) {
      state->diag.deferred_publish_set_count++;
      state->diag.last_deferred_publish_set_seq = state->diag.last_publish_seq;
      osal_irq_restore(irq_sts);
      return;
    }

    state->publishing = true;
    osal_irq_restore(irq_sts);
    uart_rx_dma_idle_drain_lli(bus);
    return;
  }

  if (state->publishing) {
    state->deferred_publish = true;
    state->diag.deferred_publish_set_count++;
    state->diag.last_deferred_publish_set_seq = state->diag.last_publish_seq;
    return;
  }
  if (!state->enabled || (state->channel == 0U) ||
      (state->dma_buffer == NULL) || (state->dma_buffer_size == 0U)) {
    return;
  }

  state->publishing = true;
  state->idle_debounce_armed = false;
  state->idle_publish_seen = true;
  channel = (uint8_t)(state->channel - 1U);
  (void)uapi_dma_end_transfer(channel);
  state->channel = 0;
  remaining = (uint16_t)uapi_dma_get_block_ts(channel);
  if (remaining > state->transfer_num) {
    remaining = state->transfer_num;
  }

  received_blocks = (uint16_t)(state->transfer_num - remaining);
  received_len = (uint16_t)(received_blocks << state->dma_cfg.src_width);
  if (received_len > state->dma_buffer_size) {
    received_len = state->dma_buffer_size;
  }

  /* Save pointer to completed buffer before swapping */
  completed_buf = state->dma_buffer;

#ifdef CONFIG_SUPPORT_DATA_CACHE
  if (received_len > 0U) {
    osal_dcache_region_inv(completed_buf, received_len);
  }
#endif

  /* Drain FIFO residual bytes into completed buffer tail – minimizes overrun
   * window */
  (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                      (uintptr_t)&fifo_empty);
  while (!fifo_empty &&
         ((uint32_t)received_len + fifo_drain) < state->dma_buffer_size) {
    hal_uart_read(bus, &completed_buf[received_len + fifo_drain], 1);
    fifo_drain++;
    (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                        (uintptr_t)&fifo_empty);
  }

  /* Ping-pong: swap to alternate buffer and restart DMA immediately */
  if (state->dma_buffer_alt != NULL) {
    state->dma_buffer = state->dma_buffer_alt;
    state->dma_buffer_alt = completed_buf;
    uart_rx_dma_idle_prepare_idle_buffer(bus);
    if (uart_rx_dma_idle_start(bus) != ERRCODE_SUCC) {
      state->enabled = false;
    }
  }

  /* Append idle tail (from ISR path) to completed buffer after FIFO drain */
  combined_len = (uint16_t)(received_len + fifo_drain);
  if ((idle_tail_len > 0U) && (combined_len < state->dma_buffer_size)) {
    uint16_t tail_len = idle_tail_len;
    if ((uint32_t)combined_len + tail_len > state->dma_buffer_size) {
      tail_len = (uint16_t)(state->dma_buffer_size - combined_len);
    }
    (void)memcpy_s(completed_buf + combined_len,
                   state->dma_buffer_size - combined_len, state->idle_buffer,
                   tail_len);
    combined_len = (uint16_t)(combined_len + tail_len);
  }

  state->diag.publish_count++;
  state->diag.publish_bytes += combined_len;
  state->diag.last_publish_seq = state->diag.publish_count;
  state->diag.last_transfer_num = state->transfer_num;
  state->diag.last_remaining = remaining;
  state->diag.last_received_blocks = received_blocks;
  state->diag.last_received_len = received_len;
  state->diag.last_idle_tail_len = idle_tail_len;
  state->diag.last_combined_len = combined_len;
  state->diag.last_publish_reason = reason;
  state->diag.last_fifo_drain_len = fifo_drain;
  if (reason == UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB) {
    state->diag.publish_from_idle_cb++;
  } else if (reason == UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH) {
    state->diag.publish_from_soft_flush++;
  } else if (reason == UART_DMA_IDLE_PUBLISH_REASON_IDLE_FALLBACK) {
    state->diag.publish_from_idle_fallback++;
  } else if (reason == UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE) {
    state->diag.publish_from_dma_complete++;
    state->diag.last_dma_complete_seq = state->diag.last_publish_seq;
  }

  if ((combined_len > 0U) && (state->raw_callback != NULL)) {
    keep_receiving = state->raw_callback(completed_buf, combined_len);
  }
  if (!keep_receiving) {
    state->enabled = false;
  }

  /* Fallback: if no alt buffer, restart DMA the old way (after callback) */
  if (state->dma_buffer_alt == NULL && state->enabled) {
    uart_rx_dma_idle_prepare_idle_buffer(bus);
    if (uart_rx_dma_idle_start(bus) != ERRCODE_SUCC) {
      state->enabled = false;
    }
  }

  /* Drain deferred publish queue BEFORE releasing publishing flag so
     concurrent interrupts keep deferring instead of racing the drain loop. */
  while (state->deferred_publish && state->enabled && (state->channel != 0U)) {
    state->deferred_publish = false;
    state->diag.deferred_publish_drained_count++;
    state->diag.last_deferred_publish_drained_seq =
        state->diag.last_publish_seq;
    uart_rx_dma_idle_publish(bus, 0U,
                             UART_DMA_IDLE_PUBLISH_REASON_DMA_COMPLETE);
  }

  state->publishing = false;
}

static void uart_rx_dma_idle_prepare_idle_buffer(uart_bus_t bus) {
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];

  rx_state->rx_buffer = state->idle_buffer;
  rx_state->rx_buffer_size = CONFIG_UART_FIFO_DEPTH;
  rx_state->new_rx_pos = 0;
}

static void uart_rx_dma_idle_rx_callback(const void *buffer, uint16_t length,
                                         bool error) {
  uart_bus_t bus = uart_rx_dma_idle_find_bus_by_buffer(buffer);

  unused(error);
  if (bus >= UART_BUS_MAX_NUM) {
    return;
  }
  uart_rx_dma_idle_publish(bus, length, UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB);
}

static void uart_rx_dma_idle_stop(uart_bus_t bus) {
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];

  if (state->channel != 0U) {
    (void)uapi_dma_end_transfer((uint8_t)(state->channel - 1U));
    state->channel = 0;
  }
}
#endif /* CONFIG_UART_SUPPORT_RX */

static void uart_dma_isr(uint8_t int_type, uint8_t ch, uintptr_t arg) {
  unused(arg);
  uint8_t bus = UART_BUS_MAX_NUM;
  for (uint8_t i = UART_BUS_0; i < UART_BUS_MAX_NUM; i++) {
    /* channel default value is 0, means not used. channel > 0 means used.
       So ch + 1 will not misjudgment with channel value 0. */
    if (g_dma_trans[i].channel == ch + 1) {
      bus = i;
      break;
    }
  }

  if (bus != UART_BUS_MAX_NUM) {
    if (int_type == 0) {
      g_dma_trans[bus].trans_succ = true;
    }
    osal_sem_up(&(g_dma_trans[bus].dma_sem));
  }
}

static int32_t
uart_write_by_dma_config(uart_bus_t bus, const void *buffer, uint32_t length,
                         uart_write_dma_config_t *dma_cfg,
                         dma_ch_user_peripheral_config_t *user_cfg) {
  uint32_t uart_data_addr = 0;
  errcode_t ret = hal_uart_ctrl(bus, UART_CTRL_GET_DMA_DATA_ADDR,
                                (uintptr_t)&uart_data_addr);
  if (ret != ERRCODE_SUCC) {
    return -1;
  }
  user_cfg->src = (uint32_t)(uintptr_t)buffer;
  user_cfg->dest = uart_data_addr;
  user_cfg->transfer_num = (uint16_t)(length >> dma_cfg->src_width);
  user_cfg->src_handshaking = 0;
  user_cfg->trans_type = UART_DMA_TRANS_MEMORY_TO_PERIPHERAL_DMA;
  user_cfg->trans_dir = UART_DMA_TRANSFER_DIR_MEM_TO_PERIPHERAL;
  user_cfg->priority = dma_cfg->priority;
  user_cfg->src_width = dma_cfg->src_width;
  user_cfg->dest_width = dma_cfg->dest_width;
  user_cfg->burst_length = dma_cfg->burst_length;
  user_cfg->src_increment = UART_DMA_ADDRESS_INC_INCREMENT;
  user_cfg->dest_increment = UART_DMA_ADDRESS_INC_NO_CHANGE;
  user_cfg->protection = UART_DMA_PROTECTION_CONTROL_BUFFERABLE;
  user_cfg->dest_handshaking = uart_port_get_dma_trans_dest_handshaking(bus);
  return ERRCODE_SUCC;
}

static int32_t uapi_uart_dma_check(uart_bus_t bus, const void *buffer,
                                   uint32_t length,
                                   const uart_write_dma_config_t *dma_cfg) {
  if ((bus >= UART_BUS_MAX_NUM) || (dma_cfg == NULL)) {
    return UART_DMA_CFG_PARAM_INVALID;
  }
  if ((buffer == NULL) || (length == 0)) {
    return UART_DMA_BUFF_NULL;
  }
  if (length % bit(dma_cfg->src_width) != 0) {
    return UART_DMA_CFG_PARAM_INVALID;
  }
  return 0;
}

int32_t uapi_uart_write_by_dma(uart_bus_t bus, const void *buffer,
                               uint32_t length,
                               uart_write_dma_config_t *dma_cfg) {
  int32_t ret = uapi_uart_dma_check(bus, buffer, length, dma_cfg);
  if (ret != 0) {
    return ret;
  }

  dma_ch_user_peripheral_config_t user_cfg = {0};

  ret = uart_write_by_dma_config(bus, buffer, length, dma_cfg, &user_cfg);
  if (ret != ERRCODE_SUCC ||
      user_cfg.dest_handshaking == HAL_DMA_HANDSHAKING_MAX_NUM) {
    return UART_DMA_SHAKING_INVALID_OR_UART_FUNCS_NULL;
  }

  uint8_t dma_ch;
  if (uapi_dma_configure_peripheral_transfer_single(
          &user_cfg, &dma_ch, uart_dma_isr, (uintptr_t)NULL) != ERRCODE_SUCC) {
    return UART_DMA_CONFIGURE_FAIL;
  }

  g_dma_trans[bus].channel = dma_ch + 1;
  g_dma_trans[bus].trans_succ = false;

  if (uapi_dma_start_transfer(dma_ch) != ERRCODE_SUCC) {
    g_dma_trans[bus].channel = 0;
    return UART_DMA_START_TRANSFER_FAIL;
  }

  if (osal_sem_down_timeout(&(g_dma_trans[bus].dma_sem),
                            DMA_UART_TRANSFER_TIMEOUT_MS) != OSAL_SUCCESS) {
    g_dma_trans[bus].channel = 0;
    return UART_DMA_TRANSFER_TIMEOUT;
  }

  g_dma_trans[bus].channel = 0;

  if (!g_dma_trans[bus].trans_succ) {
    return UART_DMA_TRANSFER_ERROR;
  }

  return (int32_t)uapi_dma_get_block_ts(dma_ch);
}

static int32_t
uart_read_by_dma_config(uart_bus_t bus, const void *buffer, uint32_t length,
                        uart_write_dma_config_t *dma_cfg,
                        dma_ch_user_peripheral_config_t *user_cfg) {
  uint32_t uart_data_addr = 0;
  errcode_t ret = hal_uart_ctrl(bus, UART_CTRL_GET_DMA_DATA_ADDR,
                                (uintptr_t)&uart_data_addr);
  if (ret != ERRCODE_SUCC) {
    return -1;
  }
  user_cfg->src = uart_data_addr;
  user_cfg->dest = (uint32_t)(uintptr_t)buffer;
  user_cfg->transfer_num = (uint16_t)(length >> dma_cfg->src_width);
  user_cfg->dest_handshaking = 0;
  user_cfg->trans_type = UART_DMA_TRANS_PERIPHERAL_TO_MEMORY_DMA;
  user_cfg->trans_dir = UART_DMA_TRANSFER_DIR_PERIPHERAL_TO_MEM;
  user_cfg->priority = dma_cfg->priority;
  user_cfg->src_width = dma_cfg->src_width;
  user_cfg->dest_width = dma_cfg->dest_width;
  user_cfg->burst_length = dma_cfg->burst_length;
  user_cfg->src_increment = UART_DMA_ADDRESS_INC_NO_CHANGE;
  user_cfg->dest_increment = UART_DMA_ADDRESS_INC_INCREMENT;
  user_cfg->protection = UART_DMA_PROTECTION_CONTROL_BUFFERABLE;
  user_cfg->src_handshaking = uart_port_get_dma_trans_src_handshaking(bus);
  return ERRCODE_SUCC;
}

int32_t uapi_uart_read_by_dma(uart_bus_t bus, const void *buffer,
                              uint32_t length,
                              uart_write_dma_config_t *dma_cfg) {
  int32_t ret = uapi_uart_dma_check(bus, buffer, length, dma_cfg);
  if (ret != 0) {
    return ret;
  }

  dma_ch_user_peripheral_config_t user_cfg = {0};
  uint8_t dma_ch;

  ret = uart_read_by_dma_config(bus, buffer, length, dma_cfg, &user_cfg);
  if (ret != ERRCODE_SUCC ||
      user_cfg.src_handshaking == HAL_DMA_HANDSHAKING_MAX_NUM) {
    return -1;
  }

  if (uapi_dma_configure_peripheral_transfer_single(
          &user_cfg, &dma_ch, uart_dma_isr, (uintptr_t)NULL) != ERRCODE_SUCC) {
    return -1;
  }

  g_dma_trans[bus].channel = dma_ch + 1;
  g_dma_trans[bus].trans_succ = false;

  if (uapi_dma_start_transfer(dma_ch) != ERRCODE_SUCC) {
    g_dma_trans[bus].channel = 0;
    return -1;
  }

  if (osal_sem_down(&(g_dma_trans[bus].dma_sem)) != OSAL_SUCCESS) {
    g_dma_trans[bus].channel = 0;
    return -1;
  }

  g_dma_trans[bus].channel = 0;

  if (!g_dma_trans[bus].trans_succ) {
    return -1;
  }

  return (int32_t)uapi_dma_get_block_ts(dma_ch);
}

errcode_t
uapi_uart_register_read_by_dma_callback(uart_bus_t bus,
                                        uart_write_dma_config_t *dma_cfg) {
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];
  uint32_t irq_sts;
  errcode_t ret;

  if ((bus >= UART_BUS_MAX_NUM) || (dma_cfg == NULL) || !g_uart_inited[bus]) {
    return ERRCODE_INVALID_PARAM;
  }

  irq_sts = uart_porting_lock(bus);
  state->enabled = false;
  state->publishing = false;
  state->deferred_publish = false;
  state->idle_publish_seen = false;
  state->pingpong_allocated = false;
  state->lli_mode = false;
  state->channel = 0;
  state->transfer_num = 0;
  state->dma_buffer_alt = NULL;
  state->lli_block_size = 0U;
  state->lli_block_count = 0U;
  state->lli_active_block = 0U;
  state->lli_active_block_published = 0U;
  state->lli_last_remaining = 0U;
  state->lli_rollover_dma_complete_pending = false;
  state->lli_skip_dma_complete_count = 0U;
  state->lli_pending_head = 0U;
  state->lli_pending_tail = 0U;
  state->lli_queue_overrun_count = 0U;
  state->idle_debounce_armed = false;
  state->idle_debounce_remaining = 0U;
  (void)memset_s(&state->diag, sizeof(state->diag), 0, sizeof(state->diag));
  state->dma_buffer = rx_state->rx_buffer;
  state->dma_buffer_size = rx_state->rx_buffer_size;
  state->raw_callback = NULL;
  if ((state->dma_buffer == NULL) || (state->dma_buffer_size == 0U)) {
    uart_porting_unlock(bus, irq_sts);
    return ERRCODE_INVALID_PARAM;
  }
  (void)memcpy_s(&state->dma_cfg, sizeof(state->dma_cfg), dma_cfg,
                 sizeof(*dma_cfg));
  ret = uart_rx_dma_idle_start(bus);
  if (ret != ERRCODE_SUCC) {
    uart_porting_unlock(bus, irq_sts);
    return ret;
  }
  state->enabled = true;
  uart_porting_unlock(bus, irq_sts);
  return ERRCODE_SUCC;
#else
  unused(bus);
  unused(dma_cfg);
  return ERRCODE_NOT_SUPPORT;
#endif
}

errcode_t uapi_uart_dma_recv_raw_data(uart_bus_t bus,
                                      uart_write_dma_config_t *dma_cfg,
                                      uart_idle_int_receive_cb_t callback) {
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  uart_rx_dma_idle_state_t *state = &g_uart_rx_dma_idle_state[bus];
  uint32_t irq_sts;
  errcode_t ret;

  if ((bus >= UART_BUS_MAX_NUM) || (dma_cfg == NULL) || (callback == NULL) ||
      !g_uart_inited[bus]) {
    return ERRCODE_INVALID_PARAM;
  }

  irq_sts = uart_porting_lock(bus);
  state->enabled = false;
  state->publishing = false;
  state->deferred_publish = false;
  state->idle_publish_seen = false;
  state->pingpong_allocated = false;
  state->lli_mode = false;
  state->channel = 0;
  state->transfer_num = 0;
  g_uart_rx_dma_idle_isr_cnt[bus] = 0;
  state->idle_debounce_armed = false;
  state->idle_debounce_remaining = 0;
  state->dma_buffer_alt = NULL;
  state->lli_block_size = 0U;
  state->lli_block_count = 0U;
  state->lli_active_block = 0U;
  state->lli_active_block_published = 0U;
  state->lli_last_remaining = 0U;
  state->lli_rollover_dma_complete_pending = false;
  state->lli_skip_dma_complete_count = 0U;
  state->lli_pending_head = 0U;
  state->lli_pending_tail = 0U;
  state->lli_queue_overrun_count = 0U;
  (void)memset_s(&state->diag, sizeof(state->diag), 0, sizeof(state->diag));
  state->dma_buffer = rx_state->rx_buffer;
  state->dma_buffer_size = rx_state->rx_buffer_size;
  if ((state->dma_buffer == NULL) || (state->dma_buffer_size == 0U)) {
    uart_porting_unlock(bus, irq_sts);
    return ERRCODE_INVALID_PARAM;
  }
  state->raw_callback = callback;
  (void)memcpy_s(&state->dma_cfg, sizeof(state->dma_cfg), dma_cfg,
                 sizeof(*dma_cfg));
#if defined(CONFIG_DMA_SUPPORT_LLI)
  state->lli_block_size = uart_rx_dma_idle_get_lli_block_size(state);
  if (state->lli_block_size != 0U) {
    state->lli_mode = true;
    state->lli_block_count =
        (uint16_t)(state->dma_buffer_size / state->lli_block_size);
  } else {
#endif
    state->dma_buffer_alt =
        (uint8_t *)osal_kmalloc(rx_state->rx_buffer_size, OSAL_GFP_KERNEL);
    if (state->dma_buffer_alt == NULL) {
      uart_porting_unlock(bus, irq_sts);
      return ERRCODE_MALLOC;
    }
    state->pingpong_allocated = true;
#if defined(CONFIG_DMA_SUPPORT_LLI)
  }
#endif
  uart_rx_dma_idle_prepare_idle_buffer(bus);
  ret = uart_rx_dma_idle_start(bus);
  if (ret != ERRCODE_SUCC) {
    uart_rx_dma_idle_restore_rx_buffer(bus);
    if (state->pingpong_allocated && (state->dma_buffer_alt != NULL)) {
      osal_kfree(state->dma_buffer_alt);
      state->dma_buffer_alt = NULL;
    }
    state->pingpong_allocated = false;
    state->lli_mode = false;
    state->lli_block_size = 0U;
    state->lli_block_count = 0U;
    uart_porting_unlock(bus, irq_sts);
    return ret;
  }
  state->enabled = true;
  uart_porting_unlock(bus, irq_sts);
#if defined(CONFIG_DMA_SUPPORT_LLI)
  if (state->lli_mode) {
    irq_sts = uart_porting_lock(bus);
    ret = hal_uart_ctrl(bus, UART_CTRL_EN_IDLE_INT, 1);
    uart_porting_unlock(bus, irq_sts);
    if (ret != ERRCODE_SUCC) {
      irq_sts = uart_porting_lock(bus);
      state->enabled = false;
      uart_rx_dma_idle_stop(bus);
      state->lli_mode = false;
      state->lli_block_size = 0U;
      state->lli_block_count = 0U;
      uart_porting_unlock(bus, irq_sts);
      return ret;
    }
    return ERRCODE_SUCC;
  }
#endif
  ret = uapi_uart_register_rx_callback(bus, UART_RX_CONDITION_MASK_IDLE,
                                       CONFIG_UART_FIFO_DEPTH,
                                       uart_rx_dma_idle_rx_callback);
  if (ret != ERRCODE_SUCC) {
    irq_sts = uart_porting_lock(bus);
    state->enabled = false;
    uart_rx_dma_idle_stop(bus);
    uart_rx_dma_idle_restore_rx_buffer(bus);
    if (state->pingpong_allocated && (state->dma_buffer_alt != NULL)) {
      osal_kfree(state->dma_buffer_alt);
      state->dma_buffer_alt = NULL;
    }
    state->pingpong_allocated = false;
    state->lli_mode = false;
    state->lli_block_size = 0U;
    state->lli_block_count = 0U;
    uart_porting_unlock(bus, irq_sts);
    return ret;
  }
  return ERRCODE_SUCC;
#else
  unused(bus);
  unused(dma_cfg);
  unused(callback);
  return ERRCODE_NOT_SUPPORT;
#endif
}

errcode_t uapi_uart_dma_idle_flush_pending(uart_bus_t bus) {
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_dma_idle_state_t *state;
  uint16_t remaining;
  uint16_t received_blocks;
  uint8_t channel;
  bool rx_fifo_empty = true;
  uint32_t irq_sts;

  if (bus >= UART_BUS_MAX_NUM) {
    return ERRCODE_INVALID_PARAM;
  }

  state = &g_uart_rx_dma_idle_state[bus];

  /* IRQ lock to protect shared state between ISR and task context */
  irq_sts = osal_irq_lock();
  if (!state->enabled || (state->channel == 0U) ||
      (state->raw_callback == NULL)) {
    osal_irq_restore(irq_sts);
    return ERRCODE_SUCC;
  }
  if (state->lli_mode) {
    uint16_t cur;

    if (state->publishing) {
      osal_irq_restore(irq_sts);
      return ERRCODE_SUCC;
    }
    channel = (uint8_t)(state->channel - 1U);
    cur = (uint16_t)uapi_dma_get_block_ts(channel);
    if (cur > state->transfer_num) {
      cur = state->transfer_num;
    }
    (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                        (uintptr_t)&rx_fifo_empty);
    state->diag.last_rx_fifo_empty = rx_fifo_empty ? 1U : 0U;
    if (rx_fifo_empty) {
      state->idle_debounce_armed = false;
      osal_irq_restore(irq_sts);
      uart_rx_dma_idle_publish(bus, 0U, UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH);
      return ERRCODE_SUCC;
    }
    if (!state->idle_debounce_armed) {
      state->idle_debounce_armed = true;
      state->idle_debounce_remaining = cur;
      osal_irq_restore(irq_sts);
      return ERRCODE_SUCC;
    }
    if (cur == state->idle_debounce_remaining) {
      state->idle_debounce_armed = false;
      osal_irq_restore(irq_sts);
      uart_rx_dma_idle_publish(bus, 0U, UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH);
      return ERRCODE_SUCC;
    }
    state->idle_debounce_remaining = cur;
    osal_irq_restore(irq_sts);
    return ERRCODE_SUCC;
  }
  if (state->publishing) {
    osal_irq_restore(irq_sts);
    return ERRCODE_SUCC;
  }
  channel = (uint8_t)(state->channel - 1U);

  /* Debounce path: IDLE ISR armed but last IDLE may have been the final one.
     Check if DMA position is stable — if so, publish from task context. */
  if (state->idle_debounce_armed) {
    uint16_t cur = (uint16_t)uapi_dma_get_block_ts(channel);
    (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                        (uintptr_t)&rx_fifo_empty);
    if ((cur == state->idle_debounce_remaining) || rx_fifo_empty) {
      /* Stable or FIFO empty → publish */
      state->idle_debounce_armed = false;
      osal_irq_restore(irq_sts);
      uart_rx_dma_idle_publish(bus, 0U,
                               UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH);
    } else {
      /* Still moving → update snapshot */
      state->idle_debounce_remaining = cur;
      osal_irq_restore(irq_sts);
    }
    return ERRCODE_SUCC;
  }

  /* No debounce pending — check if data arrived without any IDLE.
     If data exists, arm debounce and wait for next poll to confirm stability
     instead of publishing immediately (avoids mid-frame publish). */
  remaining = (uint16_t)uapi_dma_get_block_ts(channel);
  if (remaining > state->transfer_num) {
    remaining = state->transfer_num;
  }
  received_blocks = (uint16_t)(state->transfer_num - remaining);
  (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                      (uintptr_t)&rx_fifo_empty);
  state->diag.last_rx_fifo_empty = rx_fifo_empty ? 1U : 0U;
  if ((received_blocks == 0U) && rx_fifo_empty) {
    osal_irq_restore(irq_sts);
    return ERRCODE_SUCC;
  }

  /* Data present — if FIFO empty, publish immediately; else arm debounce */
  if (rx_fifo_empty) {
    osal_irq_restore(irq_sts);
    uart_rx_dma_idle_publish(bus, 0U, UART_DMA_IDLE_PUBLISH_REASON_SOFT_FLUSH);
  } else {
    state->idle_debounce_armed = true;
    state->idle_debounce_remaining = remaining;
    osal_irq_restore(irq_sts);
  }
  return ERRCODE_SUCC;
#else
  unused(bus);
  return ERRCODE_NOT_SUPPORT;
#endif
}

uint32_t uapi_uart_dma_idle_get_idle_isr_count(uart_bus_t bus) {
#if defined(CONFIG_UART_SUPPORT_RX)
  if (bus >= UART_BUS_MAX_NUM) {
    return 0U;
  }
  return g_uart_rx_dma_idle_isr_cnt[bus];
#else
  unused(bus);
  return 0U;
#endif
}

errcode_t uapi_uart_dma_idle_get_diag(uart_bus_t bus,
                                      uart_dma_idle_diag_t *diag) {
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_dma_idle_state_t *state;
  uint32_t irq_sts;

  if ((bus >= UART_BUS_MAX_NUM) || (diag == NULL)) {
    return ERRCODE_INVALID_PARAM;
  }

  state = &g_uart_rx_dma_idle_state[bus];
  irq_sts = uart_porting_lock(bus);
  (void)memcpy_s(diag, sizeof(*diag), &state->diag, sizeof(state->diag));
  uart_porting_unlock(bus, irq_sts);
  return ERRCODE_SUCC;
#else
  unused(bus);
  unused(diag);
  return ERRCODE_NOT_SUPPORT;
#endif
}

void uapi_uart_unregister_read_by_dma_callback(uart_bus_t bus) {
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_dma_idle_state_t *state;
  uart_rx_state_t *rx_state;
  uint32_t irq_sts;

  if (bus >= UART_BUS_MAX_NUM) {
    return;
  }

  state = &g_uart_rx_dma_idle_state[bus];
  rx_state = &g_uart_rx_state_array[bus];
  irq_sts = uart_porting_lock(bus);
  state->enabled = false;
  state->publishing = false;
  state->raw_callback = NULL;
  state->deferred_publish = false;
  state->lli_mode = false;
  state->idle_debounce_armed = false;
  state->idle_debounce_remaining = 0;
  state->lli_block_size = 0U;
  state->lli_block_count = 0U;
  state->lli_active_block = 0U;
  state->lli_active_block_published = 0U;
  state->lli_last_remaining = 0U;
  state->lli_rollover_dma_complete_pending = false;
  state->lli_skip_dma_complete_count = 0U;
  state->lli_pending_head = 0U;
  state->lli_pending_tail = 0U;
  state->lli_queue_overrun_count = 0U;
  uart_rx_dma_idle_stop(bus);
  if (state->pingpong_allocated && state->dma_buffer_alt != NULL) {
    osal_kfree(state->dma_buffer_alt);
    state->dma_buffer_alt = NULL;
    state->pingpong_allocated = false;
  }
  state->dma_buffer_alt = NULL;
  rx_state->rx_buffer = state->dma_buffer;
  rx_state->rx_buffer_size = state->dma_buffer_size;
  rx_state->new_rx_pos = 0;
  uart_porting_unlock(bus, irq_sts);
#else
  unused(bus);
#endif
}
#endif /* CONFIG_UART_SUPPORT_DMA */
#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX)
int32_t uapi_uart_read(uart_bus_t bus, const uint8_t *buffer, uint32_t length,
                       uint32_t timeout) {
  int32_t ret = uapi_uart_param_check(bus, buffer, length);
  if (ret != ERRCODE_SUCC) {
    return ret;
  }

  unused(timeout);
  bool rx_fifo_empty = false;
  uint8_t *data_buffer = (uint8_t *)buffer;
  int32_t read_count = 0;
  uint32_t len = length;

  uint32_t irq_sts = uart_porting_lock(bus);
  uint32_t cnt = 0;

  while (len > 0) {
    hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                  (uintptr_t)&rx_fifo_empty);
    if (rx_fifo_empty == false) {
      hal_uart_read(bus, data_buffer++, 1);
      len--;
      read_count++;
    } else {
#if defined(CONFIG_SUPPORT_UART_POLL_TIMEOUT)
      cnt++;
      if (cnt > UART_READ_MAX_TIMEOUT) {
        break;
      }
#else
      unused(cnt);
#endif
    }
  }
  uart_porting_unlock(bus, irq_sts);

  return read_count;
}
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_RX)
static bool
uart_config_rx_state(uart_bus_t bus,
                     const uart_buffer_config_t *uart_buffer_config) {
  if ((uart_buffer_config == NULL) || (uart_buffer_config->rx_buffer == NULL) ||
      (uart_buffer_config->rx_buffer_size == 0)) { /* No RX buffer specified */
    return false;
  }
  /* Configure RX state structure */
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  rx_state->rx_buffer = uart_buffer_config->rx_buffer;
  rx_state->rx_buffer_size = (uint16_t)uart_buffer_config->rx_buffer_size;

  return true;
}
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
static void uart_config_tx_state(uart_bus_t bus) {
  /* Configure TX state structure */
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  tx_state->current_tx_fragment =
      tx_state->fragment_buffer; /* the queue is empty */
  tx_state->free_tx_fragment =
      tx_state->fragment_buffer; /* the queue is empty */
}
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX) || (CONFIG_UART_SUPPORT_TX)
static void uart_deconfig_state(uart_bus_t bus) {
  unused(bus);
#if defined(CONFIG_UART_SUPPORT_RX)
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  (void)memset_s(rx_state, sizeof(uart_rx_state_t), 0, sizeof(uart_rx_state_t));
#endif /* CONFIG_UART_SUPPORT_RX */
#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  (void)memset_s(tx_state, sizeof(uart_tx_state_t), 0, sizeof(uart_tx_state_t));
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */
}
#endif /* defined(CONFIG_UART_SUPPORT_RX) || (CONFIG_UART_SUPPORT_TX) */

#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
static bool
uart_helper_add_fragment(uart_bus_t bus, const uint8_t *buffer, uint32_t length,
                         void *params,
                         uart_tx_callback_t finished_with_buffer_func) {
  uart_tx_fragment_t *fragment;

  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  /* If we have fragments left add it */
  if (tx_state->fragments_to_process >= UART_MAX_NUMBER_OF_FRAGMENTS) {
    return false;
  }

  /* Put it on the queue */
  fragment = tx_state->free_tx_fragment;
  /* Populate the fragment */
  fragment->data = (uint8_t *)buffer;
  fragment->params = params;
  fragment->data_length = length;
  fragment->release_func = finished_with_buffer_func;

  /* Update the counters */
  tx_state->free_tx_fragment++;
  if (tx_state->free_tx_fragment >=
      tx_state->fragment_buffer + UART_MAX_NUMBER_OF_FRAGMENTS) {
    tx_state->free_tx_fragment = tx_state->fragment_buffer; /* wrapping */
  }
  tx_state->fragments_to_process++;
  return true;
}

static inline bool
uart_helper_is_the_current_fragment_the_last_to_process(uart_bus_t bus) {
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  return (tx_state->fragments_to_process == 1);
}

static inline bool uart_helper_are_there_fragments_to_process(uart_bus_t bus) {
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  return (tx_state->fragments_to_process > 0);
}

static bool uart_helper_send_next_char(uart_bus_t bus) {
  uart_tx_fragment_t *current_fragment;
  uint16_t current_fragment_pos;

  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  current_fragment = tx_state->current_tx_fragment;
  current_fragment_pos = tx_state->current_tx_fragment_pos;
  hal_uart_write(bus, &current_fragment->data[current_fragment_pos], 1);
  /* update the counters */
  tx_state->current_tx_fragment_pos++;

  return (tx_state->current_tx_fragment_pos >= current_fragment->data_length);
}

static inline void
uart_helper_invoke_current_fragment_callback(uart_bus_t bus) {
  uart_tx_fragment_t *current_fragment;
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  current_fragment = tx_state->current_tx_fragment;
  /* Call any TX data release call-back */
  if (current_fragment->release_func != NULL) {
    current_fragment->release_func(current_fragment->data,
                                   current_fragment->data_length,
                                   current_fragment->params);
  }
}

static inline void uart_helper_move_to_next_fragment(uart_bus_t bus) {
  /* Move onto the next fragment and re-set the position to zero */
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  tx_state->current_tx_fragment++;
  if (tx_state->current_tx_fragment >=
      tx_state->fragment_buffer + UART_MAX_NUMBER_OF_FRAGMENTS) {
    tx_state->current_tx_fragment = tx_state->fragment_buffer; /* wrapping */
  }
  tx_state->current_tx_fragment_pos = 0; /* reset the current fragment */
  tx_state->fragments_to_process--;      /* one fragment less to process */
}
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#endif /* CONFIG_UART_SUPPORT_TX */

#if defined(CONFIG_UART_SUPPORT_RX)
static inline void uart_rx_buffer_release(uart_bus_t bus) {
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  rx_state->new_rx_pos = 0;
}

static inline bool uart_rx_buffer_has_free_space(uart_bus_t bus) {
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  return (rx_state->new_rx_pos < rx_state->rx_buffer_size);
}

static inline uint16_t uart_rx_buffer_data_available(uart_bus_t bus) {
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  return rx_state->new_rx_pos;
}
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_LPM)
errcode_t uapi_uart_suspend(uintptr_t arg) {
  errcode_t ret = ERRCODE_SUCC;
  unused(arg);
  for (uint32_t i = 0; i < UART_BUS_MAX_NUM; i++) {
    if (g_uart_inited[i] == false) {
      continue;
    }
    g_uart_suspend_flag[i] = true;
#if defined(CONFIG_UART_SUPPORT_DMA)
    uapi_dma_suspend(arg);
#endif
  }
  return ret;
}

errcode_t uapi_uart_resume(uintptr_t arg) {
  errcode_t ret = ERRCODE_SUCC;
  unused(arg);
  for (uint32_t i = 0; i < UART_BUS_MAX_NUM; i++) {
    if (g_uart_suspend_flag[i] == false) {
      continue;
    }
    ret |= uapi_uart_deinit(i);
    ret |= uapi_uart_init(i, &g_uart_pins[i], &g_uart_attr[i],
                          &g_uart_extra_attr[i], &g_uart_buffer_config[i]);
    if (g_uart_callback[i] != NULL) {
      ret |= uapi_uart_register_rx_callback(i, g_uart_condition[i],
                                            g_uart_size[i], g_uart_callback[i]);
    }
#if defined(CONFIG_UART_SUPPORT_DMA)
    uapi_dma_resume(arg);
#endif
    g_uart_suspend_flag[i] = false;
  }
  return ret;
}
#endif /* CONFIG_UART_SUPPORT_LPM */

static int32_t uart_check_params_attr(const uart_attr_t *attr) {
  if (attr == NULL || attr->data_bits > UART_DATA_BIT_8) {
    return -1;
  }

  if (attr->parity > UART_PARITY_EVEN) {
    return -1;
  }

  if (attr->stop_bits != UART_STOP_BIT_1 &&
      attr->stop_bits != UART_STOP_BIT_2) {
    return -1;
  }

  return 0;
}

static int32_t uart_init_check_params(uart_bus_t bus,
                                      const uart_pin_config_t *pins,
                                      const uart_attr_t *attr) {
  if (bus >= UART_BUS_MAX_NUM || pins == NULL) {
    return -1;
  }

#if defined(CONFIG_UART_SUPPORT_RX)
  if (pins->rx_pin >= PIN_NONE) {
    return -1;
  }
#endif
#if defined(CONFIG_UART_SUPPORT_TX)
  if (pins->tx_pin >= PIN_NONE) {
    return -1;
  }
#endif

  return uart_check_params_attr(attr);
}

static void uart_claim_pins(uart_bus_t bus, const uart_pin_config_t *pins) {
  unused(pins);
  uart_port_config_pinmux(bus);
}

static void uart_release_pins(uart_bus_t bus) { unused(bus); }

#if defined(CONFIG_UART_SUPPORT_RX)
static void uart_idle_isr(uart_bus_t bus) {
  uint16_t char_recv_cnt = 0;
  uint16_t uart_rx_isr_available = 0;
  uint8_t uart_rx_isr_data = 0;
  bool rx_fifo_empty = false;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY, (uintptr_t)&rx_fifo_empty);
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  while (rx_fifo_empty != true) {
    /* Read the data out of the UART to clear the interrupt */
    /* using a volatile variable to ensure the read always happens */
    hal_uart_read(bus, &uart_rx_isr_data, 1);
    char_recv_cnt++;
    /* Only bother to try and record UART data it there is an RX callback
     * registered */
    if (rx_state->rx_callback == NULL) {
      hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                    (uintptr_t)&rx_fifo_empty);
      continue;
    }
    /* There is some space in the RX buffer so put the data in and move the
     * pointers */
    rx_state->rx_buffer[rx_state->new_rx_pos] = uart_rx_isr_data;
    rx_state->new_rx_pos++;
    /* When the rx buffer is full, callback should be invoked */
    if (uart_rx_buffer_has_free_space(bus) == false) {
      if (rx_state->rx_callback != NULL) {
        uart_rx_isr_available = uart_rx_buffer_data_available(bus);
        rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available,
                              false);
      }
      uart_rx_buffer_release(bus);
    }

    hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                  (uintptr_t)&rx_fifo_empty);
  }
  /**
   * Is the RX callback an exact size condition or
   * it has already been determined that a callback must be made.
   */
  uart_rx_isr_available = uart_rx_buffer_data_available(bus);
  if (uart_rx_isr_available > 0 &&
      ((((uint8_t)rx_state->rx_condition & UART_RX_CONDITION_MASK_IDLE) != 0) ||
       (((uint8_t)rx_state->rx_condition &
         UART_RX_CONDITION_MASK_SUFFICIENT_DATA) != 0 &&
        char_recv_cnt >= rx_state->rx_condition_size))) {
    if (rx_state->rx_callback != NULL) {
      rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available, false);
    }
    uart_rx_buffer_release(bus);
  }
}

static void uart_rx_isr(uart_bus_t bus) {
  uint16_t uart_rx_isr_available;
  uint8_t uart_rx_isr_data = 0;
  bool rx_fifo_empty = false;
  uint16_t char_recv_cnt = 0;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY, (uintptr_t)&rx_fifo_empty);
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  /* Check that the UART is opened */
  while (rx_fifo_empty != true) {
    /* Read the data out of the UART to clear the interrupt */
    /* Using a volatile variable to ensure the read always happens */
    hal_uart_read(bus, &uart_rx_isr_data, 1);
    char_recv_cnt++;
    /* Only bother to try and record UART data it there is an RX callback
     * registered */
    if (rx_state->rx_callback == NULL) {
      hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                    (uintptr_t)&rx_fifo_empty);
      continue;
    }
    /* There is some space in the RX buffer so put the data in and move the
     * pointers */
    rx_state->rx_buffer[rx_state->new_rx_pos] = uart_rx_isr_data;
    rx_state->new_rx_pos++;
    /* When the rx buffer is full, callback should be invoked */
    if (uart_rx_buffer_has_free_space(bus) == false) {
      uart_rx_isr_available = uart_rx_buffer_data_available(bus);
      rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available, false);
      uart_rx_buffer_release(bus);
    }

    hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                  (uintptr_t)&rx_fifo_empty);
  }
  /* Check to see if the callback should be invoked */
  uart_rx_isr_available = uart_rx_buffer_data_available(bus);
  if (uart_rx_isr_available > 0 &&
      (((uint8_t)rx_state->rx_condition &
        UART_RX_CONDITION_MASK_SUFFICIENT_DATA) != 0 &&
       char_recv_cnt >= rx_state->rx_condition_size)) {
    if (rx_state->rx_callback != NULL) {
      rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available, false);
    }
    uart_rx_buffer_release(bus);
  }
}

static void uart_error_isr(uart_bus_t bus) {
  uint16_t uart_rx_isr_available;
  uint8_t uart_rx_isr_data = 0;
  bool rx_fifo_empty = false;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY, (uintptr_t)&rx_fifo_empty);
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  while (rx_fifo_empty != true) {
    /* Read the data out of the UART FIFO */
    hal_uart_read(bus, &uart_rx_isr_data, 1);
    /* There is some space in the RX buffer so put the data in and move the
     * pointers */
    rx_state->rx_buffer[rx_state->new_rx_pos] = (uint8_t)uart_rx_isr_data;
    rx_state->new_rx_pos++;
    /* Only bother to try and record UART data if there is an RX callback
     * registered */
    if (uart_rx_buffer_has_free_space(bus) == false) {
      if (rx_state->rx_callback != NULL) {
        /* Calculate the amount of available data */
        uart_rx_isr_available = uart_rx_buffer_data_available(bus);
        /* Callback should be invoked in any case */
        rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available, true);
      }
      uart_rx_buffer_release(bus);
    }
    hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                  (uintptr_t)&rx_fifo_empty);
  }
  uart_rx_isr_available = uart_rx_buffer_data_available(bus);
  if (uart_rx_isr_available > 0 && rx_state->rx_callback != NULL) {
    rx_state->rx_callback(rx_state->rx_buffer, uart_rx_isr_available, true);
    uart_rx_buffer_release(bus);
  }
}
#endif /* CONFIG_UART_SUPPORT_RX */

#if defined(CONFIG_UART_SUPPORT_TX_INT)
static void uart_tx_isr(uart_bus_t bus) {
  bool tx_fifo_full = false;

  /* if there are fragments to process do it */
  if (!uart_helper_are_there_fragments_to_process(bus)) {
    /* No data to transmit so disable the TX interrupt */
    hal_uart_ctrl(bus, UART_CTRL_EN_TX_INT, false);
    return;
  }

  hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_FIFO_FULL, (uintptr_t)&tx_fifo_full);
  /* Populate the UART TX FIFO if there is data to send */
  while (tx_fifo_full != true) {
    /* There is some data to transmit so provide another byte to the UART */
    bool end_of_fragment = uart_helper_send_next_char(bus);
    if (end_of_fragment) {
      /* If it is the end of the fragment invoke the callback and move to the
       * next one */
      uart_helper_invoke_current_fragment_callback(bus);
      uart_helper_move_to_next_fragment(bus);
      /* If it was the last fragment disable the TX interrupts and leave */
      if (uart_helper_are_there_fragments_to_process(bus) == false) {
        /* No data to transmit so disable the TX interrupt */
        hal_uart_ctrl(bus, UART_CTRL_EN_TX_INT, false);
        break;
      }
    }

    hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_FIFO_FULL, (uintptr_t)&tx_fifo_full);
  }
}
#endif /* CONFIG_UART_SUPPORT_TX_INIT */

static errcode_t uart_evt_callback(uart_bus_t bus, hal_uart_evt_id_t evt,
                                   uintptr_t param) {
  unused(param);
  unused(bus);
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
  switch (evt) {
#if defined(CONFIG_UART_SUPPORT_TX_INT)
  case UART_EVT_TX_ISR:
    uart_tx_isr(bus);
    break;
#endif /* CONFIG_UART_SUPPORT_TX_INIT */

#if defined(CONFIG_UART_SUPPORT_RX)
  case UART_EVT_RX_ISR:
#if defined(CONFIG_UART_SUPPORT_DMA)
    if (uart_rx_dma_idle_enabled(bus)) {
      break;
    }
#endif
    uart_rx_isr(bus);
    break;

  case UART_EVT_IDLE_ISR:
#if defined(CONFIG_UART_SUPPORT_DMA)
    if (uart_rx_dma_idle_enabled(bus)) {
      uart_rx_dma_idle_state_t *st = &g_uart_rx_dma_idle_state[bus];
      g_uart_rx_dma_idle_isr_cnt[bus]++;
      if (st->enabled && st->channel != 0U) {
        uint8_t ch = (uint8_t)(st->channel - 1U);
        uint16_t cur = (uint16_t)uapi_dma_get_block_ts(ch);
        bool fifo_empty = false;
        (void)hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                            (uintptr_t)&fifo_empty);
        st->diag.last_rx_fifo_empty = fifo_empty ? 1U : 0U;
        if (st->lli_mode && fifo_empty) {
          /* FIFO empty → truly idle, publish immediately */
          st->idle_debounce_armed = false;
          uart_rx_dma_idle_publish(bus, 0U,
                                   UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB);
        } else if (!st->publishing && fifo_empty) {
          st->idle_debounce_armed = false;
          uart_rx_dma_idle_publish(bus, 0U,
                                   UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB);
        } else if (!st->lli_mode && !st->publishing &&
                   !st->idle_debounce_armed) {
          /* First IDLE in burst: arm debounce, save snapshot */
          st->idle_debounce_armed = true;
          st->idle_debounce_remaining = cur;
        } else if (!st->lli_mode && !st->publishing &&
                   (cur == st->idle_debounce_remaining)) {
          /* Position stable since last check → publish */
          st->idle_debounce_armed = false;
          uart_rx_dma_idle_publish(bus, 0U,
                                   UART_DMA_IDLE_PUBLISH_REASON_IDLE_CB);
        } else if (!st->lli_mode && !st->publishing) {
          /* Position changed → data still arriving → update snapshot */
          st->idle_debounce_remaining = cur;
        }
      }
      break;
    }
#endif
    uart_idle_isr(bus);
    break;

  case UART_EVT_PARITY_ERR_ISR:
    if (rx_state->parity_error_callback != NULL) {
      rx_state->parity_error_callback(NULL, 0);
    }
    uart_error_isr(bus);
    break;

  case UART_EVT_FRAME_ERR_ISR:
    if (rx_state->frame_error_callback != NULL) {
      rx_state->frame_error_callback(NULL, 0);
    }
    uart_error_isr(bus);
    break;

  case UART_EVT_BREAK_ERR_ISR:
    uart_error_isr(bus);
    break;

  case UART_EVT_OVERRUN_ERR_ISR:
    if (rx_state->overrun_error_callback != NULL) {
      rx_state->overrun_error_callback(NULL, 0);
    }
    uart_error_isr(bus);
    break;

#endif /* CONFIG_UART_SUPPORT_RX */
  default:
#if defined(CONFIG_UART_SUPPORT_RX)
    uart_error_isr(bus);
#endif /* CONFIG_UART_SUPPORT_RX */
    break;
  }
  return ERRCODE_SUCC;
}

bool uapi_uart_has_pending_transmissions(uart_bus_t bus) {
  if (bus >= UART_BUS_MAX_NUM) {
    return false;
  }
  if (!g_uart_inited[bus]) {
    return false;
  }

  bool currentstate = false;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_UART_BUSY, (uintptr_t)&currentstate);

#if defined(CONFIG_UART_SUPPORT_TX)
#if defined(CONFIG_UART_SUPPORT_TX_INT)
  uart_tx_state_t *tx_state = &g_uart_tx_state_array[bus];
  return ((tx_state->fragments_to_process > 0) || currentstate);
#else
  return currentstate;
#endif /* CONFIG_UART_SUPPORT_TX_INIT */
#else
  return currentstate;
#endif /* CONFIG_UART_SUPPORT_TX */
}

bool uapi_uart_rx_fifo_is_empty(uart_bus_t bus) {
  if (bus >= UART_BUS_MAX_NUM) {
    return false;
  }
  if (!g_uart_inited[bus]) {
    return false;
  }

  bool currentstate = false;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY, (uintptr_t)&currentstate);

  return currentstate;
}

bool uapi_uart_tx_fifo_is_empty(uart_bus_t bus) {
  if (bus >= UART_BUS_MAX_NUM) {
    return false;
  }
  if (!g_uart_inited[bus]) {
    return false;
  }

  bool currentstate = false;

  hal_uart_ctrl(bus, UART_CTRL_CHECK_TX_BUSY, (uintptr_t)&currentstate);

  return currentstate;
}

void uapi_uart_unregister_rx_callback(uart_bus_t bus) {
  bool rx_fifo_empty = false;
  uint8_t uart_rx_isr_data;
  uint32_t fifo_depth = CONFIG_UART_FIFO_DEPTH;
  if (bus >= UART_BUS_MAX_NUM) {
    return;
  }
  uint32_t irq_sts = uart_porting_lock(bus);
  uart_rx_state_t *rx_state = &g_uart_rx_state_array[bus];
#if defined(CONFIG_UART_SUPPORT_DMA)
  uart_rx_dma_idle_state_t *dma_state = &g_uart_rx_dma_idle_state[bus];
  dma_state->enabled = false;
  uart_rx_dma_idle_stop(bus);
  uart_rx_dma_idle_restore_rx_buffer(bus);
#endif
  rx_state->rx_callback = NULL;
  hal_uart_ctrl(bus, UART_CTRL_EN_RX_INT, 0);
  hal_uart_ctrl(bus, UART_CTRL_EN_FRAME_ERR_INT, 0);
  hal_uart_ctrl(bus, UART_CTRL_EN_PARITY_ERR_INT, 0);
  hal_uart_ctrl(bus, UART_CTRL_EN_IDLE_INT, 0);
  /* Flush the data on the RX FIFO */
  while (fifo_depth > 0) {
    hal_uart_ctrl(bus, UART_CTRL_CHECK_RX_FIFO_EMPTY,
                  (uintptr_t)&rx_fifo_empty);
    if (rx_fifo_empty) {
      break;
    }
    hal_uart_read(bus, &uart_rx_isr_data, 1);
    fifo_depth--;
    unused(uart_rx_isr_data);
  }
  uart_porting_unlock(bus, irq_sts);
}

/**
 * @file demo_frame.h
 * @brief Logical frame and internal fragment helpers for the demo bridge.
 */

#ifndef DEMO_FRAME_H
#define DEMO_FRAME_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include "securec.h"
#include "demo_config.h"
#include "uapi_crc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DEMO_LOGICAL_FRAME_HEADER_SIZE 4
#define DEMO_FRAME_QUEUE_SLOT_COUNT    (DEMO_FRAME_QUEUE_DEPTH + 1U)

#define DEMO_FRAME_HDR_A0              0x3F
#define DEMO_FRAME_HDR_A1              0xF3
#define DEMO_FRAME_HDR_B0              0x5F
#define DEMO_FRAME_HDR_B1              0xF5

#define DEMO_SLE_FRAG_MAGIC0           0xD5
#define DEMO_SLE_FRAG_MAGIC1           0x5E
#define DEMO_SLE_PKT_TYPE_DATA         0x01
#define DEMO_SLE_PKT_TYPE_ACK          0x02
#define DEMO_SLE_PKT_TYPE_RESET        0x03

typedef struct {
    uint16_t len;
    uint16_t frame_id;
    uint32_t enqueue_timestamp_us;
    uint32_t ready_timestamp_us;
    uint32_t stage_duration_us;
    uint8_t data[DEMO_LOGICAL_FRAME_MAX_SIZE];
} demo_frame_slot_t;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    demo_frame_slot_t slots[DEMO_FRAME_QUEUE_SLOT_COUNT];
} demo_frame_queue_t;

typedef struct {
    uint8_t type;
    uint8_t flags;
    uint16_t frame_id;
    uint16_t total_len;
    uint16_t frag_offset;
    uint8_t frag_idx;
    uint8_t frag_count;
} demo_sle_frag_hdr_t;

static inline void demo_frame_queue_init(demo_frame_queue_t *queue)
{
    if (queue == NULL) {
        return;
    }
    queue->head = 0;
    queue->tail = 0;
}

static inline bool demo_frame_queue_is_empty(const demo_frame_queue_t *queue)
{
    return (queue == NULL) || (queue->head == queue->tail);
}

static inline uint8_t demo_frame_queue_count(const demo_frame_queue_t *queue)
{
    if (queue == NULL) {
        return 0;
    }
    return (uint8_t)((queue->tail + DEMO_FRAME_QUEUE_SLOT_COUNT - queue->head) %
                     DEMO_FRAME_QUEUE_SLOT_COUNT);
}

static inline bool demo_frame_queue_push(demo_frame_queue_t *queue, const uint8_t *data,
    uint16_t len, uint16_t frame_id, uint32_t enqueue_timestamp_us, uint32_t ready_timestamp_us,
    uint32_t stage_duration_us)
{
    uint8_t next_tail;
    demo_frame_slot_t *slot;

    if (queue == NULL || data == NULL || len == 0 || len > DEMO_LOGICAL_FRAME_MAX_SIZE) {
        return false;
    }

    next_tail = (uint8_t)((queue->tail + 1U) % DEMO_FRAME_QUEUE_SLOT_COUNT);
    if (next_tail == queue->head) {
        return false;
    }

    slot = &queue->slots[queue->tail];
    if (memcpy_s(slot->data, sizeof(slot->data), data, len) != EOK) {
        return false;
    }

    slot->len = len;
    slot->frame_id = frame_id;
    slot->enqueue_timestamp_us = enqueue_timestamp_us;
    slot->ready_timestamp_us = ready_timestamp_us;
    slot->stage_duration_us = stage_duration_us;
    queue->tail = next_tail;
    return true;
}

static inline const demo_frame_slot_t *demo_frame_queue_peek(const demo_frame_queue_t *queue)
{
    if (demo_frame_queue_is_empty(queue)) {
        return NULL;
    }
    return &queue->slots[queue->head];
}

static inline void demo_frame_queue_consume(demo_frame_queue_t *queue)
{
    if (demo_frame_queue_is_empty(queue)) {
        return;
    }
    queue->head = (uint8_t)((queue->head + 1U) % DEMO_FRAME_QUEUE_SLOT_COUNT);
}

static inline bool demo_logical_frame_header_valid(const uint8_t *data)
{
    if (data == NULL) {
        return false;
    }
    return ((data[0] == DEMO_FRAME_HDR_A0 && data[1] == DEMO_FRAME_HDR_A1) ||
            (data[0] == DEMO_FRAME_HDR_B0 && data[1] == DEMO_FRAME_HDR_B1));
}

static inline uint16_t demo_logical_frame_total_len(const uint8_t *data)
{
    uint16_t payload_len;

    if (data == NULL) {
        return 0;
    }

    payload_len = (uint16_t)(((uint16_t)data[2] << 8) | data[3]);
    return (uint16_t)(payload_len + DEMO_LOGICAL_FRAME_HEADER_SIZE);
}

static inline uint8_t demo_sle_calc_frag_count(uint16_t total_len, uint16_t frag_payload)
{
    uint16_t count;

    if (total_len == 0 || frag_payload == 0) {
        return 0;
    }

    count = (uint16_t)((total_len + frag_payload - 1U) / frag_payload);
    if (count > 0xFFU) {
        return 0xFFU;
    }
    return (uint8_t)count;
}

static inline void demo_sle_frag_encode(uint8_t *dst, const demo_sle_frag_hdr_t *hdr)
{
    if (dst == NULL || hdr == NULL) {
        return;
    }

    dst[0] = DEMO_SLE_FRAG_MAGIC0;
    dst[1] = DEMO_SLE_FRAG_MAGIC1;
    dst[2] = hdr->type;
    dst[3] = hdr->flags;
    dst[4] = (uint8_t)(hdr->frame_id >> 8);
    dst[5] = (uint8_t)(hdr->frame_id & 0xFFU);
    dst[6] = (uint8_t)(hdr->total_len >> 8);
    dst[7] = (uint8_t)(hdr->total_len & 0xFFU);
    dst[8] = (uint8_t)(hdr->frag_offset >> 8);
    dst[9] = (uint8_t)(hdr->frag_offset & 0xFFU);
    dst[10] = hdr->frag_idx;
    dst[11] = hdr->frag_count;
}

static inline bool demo_sle_frag_decode(const uint8_t *src, uint16_t len, demo_sle_frag_hdr_t *hdr)
{
    if (src == NULL || hdr == NULL || len < DEMO_SLE_FRAG_HEADER_SIZE) {
        return false;
    }

    if (src[0] != DEMO_SLE_FRAG_MAGIC0 || src[1] != DEMO_SLE_FRAG_MAGIC1) {
        return false;
    }

    hdr->type = src[2];
    hdr->flags = src[3];
    hdr->frame_id = (uint16_t)(((uint16_t)src[4] << 8) | src[5]);
    hdr->total_len = (uint16_t)(((uint16_t)src[6] << 8) | src[7]);
    hdr->frag_offset = (uint16_t)(((uint16_t)src[8] << 8) | src[9]);
    hdr->frag_idx = src[10];
    hdr->frag_count = src[11];
    return true;
}

#ifdef __cplusplus
}
#endif

#endif /* DEMO_FRAME_H */

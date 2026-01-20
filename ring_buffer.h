/**
 * @file ring_buffer.h
 * @brief Lock-free Ring Buffer for ISR-safe data transfer
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include "securec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    volatile uint32_t head;  // Write position (ISR updates)
    volatile uint32_t tail;  // Read position (Main loop updates)
} ring_buffer_t;

/**
 * @brief Initialize ring buffer
 */
static inline void ring_init(ring_buffer_t *rb, uint8_t *buf, uint32_t size)
{
    rb->buffer = buf;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

/**
 * @brief Get available data length
 */
static inline uint32_t ring_data_len(const ring_buffer_t *rb)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    if (head >= tail) {
        return head - tail;
    }
    return rb->size - tail + head;
}

/**
 * @brief Get free space length
 */
static inline uint32_t ring_free_len(const ring_buffer_t *rb)
{
    return rb->size - 1 - ring_data_len(rb);
}

/**
 * @brief Check if buffer is empty
 */
static inline bool ring_is_empty(const ring_buffer_t *rb)
{
    return rb->head == rb->tail;
}

/**
 * @brief Write data to ring buffer (ISR-safe for single producer)
 * @return Actual bytes written
 */
static inline uint32_t ring_write(ring_buffer_t *rb, const uint8_t *data, uint32_t len)
{
    uint32_t free = ring_free_len(rb);
    if (len > free) {
        len = free;
    }
    if (len == 0) {
        return 0;
    }
    
    uint32_t head = rb->head;
    uint32_t to_end = rb->size - head;
    
    if (len <= to_end) {
        (void)memcpy_s(&rb->buffer[head], to_end, data, len);
    } else {
        (void)memcpy_s(&rb->buffer[head], to_end, data, to_end);
        (void)memcpy_s(&rb->buffer[0], rb->size, data + to_end, len - to_end);
    }
    
    rb->head = (head + len) % rb->size;
    return len;
}

/**
 * @brief Read data from ring buffer
 * @return Actual bytes read
 */
static inline uint32_t ring_read(ring_buffer_t *rb, uint8_t *data, uint32_t max_len)
{
    uint32_t avail = ring_data_len(rb);
    if (max_len > avail) {
        max_len = avail;
    }
    if (max_len == 0) {
        return 0;
    }
    
    uint32_t tail = rb->tail;
    uint32_t to_end = rb->size - tail;
    
    if (max_len <= to_end) {
        (void)memcpy_s(data, max_len, &rb->buffer[tail], max_len);
    } else {
        (void)memcpy_s(data, max_len, &rb->buffer[tail], to_end);
        (void)memcpy_s(data + to_end, max_len - to_end, &rb->buffer[0], max_len - to_end);
    }
    
    rb->tail = (tail + max_len) % rb->size;
    return max_len;
}

/**
 * @brief Peek data without consuming (for contiguous read)
 * @param out_ptr Output pointer to data start
 * @param out_len Output contiguous length available
 */
static inline void ring_peek_contiguous(ring_buffer_t *rb, uint8_t **out_ptr, uint32_t *out_len)
{
    uint32_t head = rb->head;
    uint32_t tail = rb->tail;
    
    *out_ptr = &rb->buffer[tail];
    
    if (head >= tail) {
        *out_len = head - tail;
    } else {
        *out_len = rb->size - tail;  // Only contiguous part to end
    }
}

/**
 * @brief Advance tail after consuming data
 */
static inline void ring_consume(ring_buffer_t *rb, uint32_t len)
{
    rb->tail = (rb->tail + len) % rb->size;
}

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */

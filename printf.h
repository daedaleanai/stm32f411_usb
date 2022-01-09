#pragma once
/*
    Everything you need to implement a printf to serial or similar devices.
    The implementation is done by stb_printf.h

    Typically you make a puts_t callback to copy to ringbuffer and enable the irq or dma
    that empties it, and in the irq/dma handler you pull from the ringbuffer and switch it off when empty.
*/

#include <stdarg.h>  // for varargs
#include <stddef.h>  // for size_t
#include <stdint.h>  // for uintX_t

struct Ringbuffer {
    uint8_t  buf[1 << 8];  // size must be power of two
    uint16_t head;         // writes happen here
    uint16_t tail;         // reads happen here
};

// invariant:
// as long as you only put_head() if !full(), and get_tail() if !empty(), tail <= head and head-tail < sizeof buf.

inline uint16_t ringbuffer_avail(const struct Ringbuffer *rb) { return rb->head - rb->tail; }                          // 0..size -1
inline uint16_t ringbuffer_free(const struct Ringbuffer *rb)  { return sizeof(rb->buf) - (rb->head - rb->tail) - 1; }  // size-1 .. 0
inline int      ringbuffer_empty(const struct Ringbuffer *rb) { return rb->head == rb->tail; }
inline int      ringbuffer_full(const struct Ringbuffer *rb)  { return sizeof(rb->buf) - 1 < (uint16_t)(rb->head - rb->tail); }
inline void     ringbuffer_clear(struct Ringbuffer *rb)       { rb->head = rb->tail; } // may race with get_tail
inline void     ringbuffer_put_head(struct Ringbuffer *rb, uint8_t c)    { rb->buf[rb->head++ % sizeof rb->buf] = c; }
inline uint8_t  ringbuffer_get_tail(struct Ringbuffer *rb)               { return rb->buf[rb->tail++ % sizeof rb->buf]; }

// copies as much of buf[:len] to rb as will fit, returns the number of bytes copied.
size_t ringbuffer_puts(struct Ringbuffer *rb, const char *buf, size_t len);

// puts_t is the type of a callback called by cbprintf() repeatedly.
// a puts(..)-like function should try to output buf[0:len], and return
// the number of characters actually copied out. if the returned value is less
// than len, cbprintf() will return immediately without attempting to print more.
typedef size_t puts_t(const char *buf, size_t len);

// cbprintf() interprets fmt as a format string for the variable parameters and calls the callback to
// copy the characters out, up to 64 at a time.  When the callback returns less than len, printing
// is aborted.
int cbprintf(puts_t *callback, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

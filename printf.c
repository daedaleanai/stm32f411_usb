#include "printf.h"

// definitions are required in one compilation unit
inline uint16_t ringbuffer_avail(const struct Ringbuffer *rb);
inline uint16_t ringbuffer_free(const struct Ringbuffer *rb);
inline int      ringbuffer_empty(const struct Ringbuffer *rb);
inline int      ringbuffer_full(const struct Ringbuffer *rb);
inline void     ringbuffer_clear(struct Ringbuffer *rb);
inline void     ringbuffer_put_head(struct Ringbuffer *rb, uint8_t c);
inline uint8_t  ringbuffer_get_tail(struct Ringbuffer *rb);

size_t ringbuffer_puts(struct Ringbuffer *rb, const char *buf, size_t len) {
    const size_t l = ringbuffer_free(rb);
    if (l < len) {
        len = l;
    }
    for (size_t i = 0; i < len; ++i) {
        ringbuffer_put_head(rb, buf[i]);
    }
    return len;
}

#define STB_SPRINTF_STATIC
#define STB_SPRINTF_MIN 64
#define STB_SPRINTF_NOFLOAT
#define STB_SPRINTF_IMPLEMENTATION
#define STB_SPRINTF_NOUNALIGNED  // Required on PIC32 or printing will cause Adress Exception on load

#include "stb_sprintf.h"

// a little signature adapter
static char *rb_putcb(char *buf, void *user, int len) {
    puts_t *callback = (puts_t *)user;
    size_t  ln       = len;  // explicit cast
    if (callback(buf, len) < ln) {
        return NULL;
    }
    return buf;
}

int cbprintf(puts_t *callback, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    char b[STB_SPRINTF_MIN];
    stbsp_set_separators('\'', '.');
    int rv = stbsp_vsprintfcb(rb_putcb, callback, b, fmt, ap);
    va_end(ap);
    return rv;
}

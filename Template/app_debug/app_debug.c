#include "app_debug.h"
#include <stdarg.h>
#include "stdio.h"
#include <string.h>
#include <ctype.h>

#if DEBUG_ISR_ENABLE
#include "lwrb.h"
#endif

#define APP_DEBUG_NUMBER_OF_DEBUG_PORT 4
#define ZEROPAD (1 << 0)   /* Pad with zero */
#define SIGN (1 << 1)      /* Unsigned/signed long */
#define UPPERCASE (1 << 6) /* 'ABCDEF' */
#define PLUS (1 << 2)      /* Show plus */
#define HEX_PREP (1 << 5)  /* 0x */
#define SPACE (1 << 3)     /* Spacer */
#define LEFT (1 << 4)      /* Left justified */
#define is_digit(c) ((c) >= '0' && (c) <= '9')

static app_debug_output_cb_t m_write_cb[APP_DEBUG_NUMBER_OF_DEBUG_PORT];
static uint8_t number_of_callback = 0;
static app_debug_get_timestamp_ms_cb_t m_get_ms;
static app_debug_lock_cb_t m_lock_cb;
static char *lower_digits = "0123456789abcdefghijklmnopqrstuvwxyz";
static char *upper_digits = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

static uint32_t __strnlen(char *ptr, uint32_t max_size)
{
    uint32_t len = 0;
    if (ptr || max_size == 0)
    {
        return 0;
    }

    while (*ptr++ && max_size--)
    {
        len++;
    }
    return len;
}

#if DEBUG_ISR_ENABLE
static lwrb_t m_ringbuffer_debug_isr;
static uint8_t m_isr_buffer_size[DEBUG_ISR_RINGBUFFER_SIZE];
#endif

#ifdef HAS_FLOAT

char *ecvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
char *fcvtbuf(double arg, int ndigits, int *decpt, int *sign, char *buf);
static void ee_bufcpy(char *d, char *s, int count);

void ee_bufcpy(char *pd, char *ps, int count) {
  char *pe=ps+count;
  while (ps!=pe)
    *pd++=*ps++;
}

static void parse_float(double value, char *buffer, char fmt, int precision)
{
  int decpt, sign, exp, pos;
  char *fdigits = NULL;
  char cvtbuf[80];
  int capexp = 0;
  int magnitude;

  if (fmt == 'G' || fmt == 'E')
  {
    capexp = 1;
    fmt += 'a' - 'A';
  }

  if (fmt == 'g')
  {
    fdigits = ecvtbuf(value, precision, &decpt, &sign, cvtbuf);
    magnitude = decpt - 1;
    if (magnitude < -4  ||  magnitude > precision - 1)
    {
      fmt = 'e';
      precision -= 1;
    }
    else
    {
      fmt = 'f';
      precision -= decpt;
    }
  }

  if (fmt == 'e')
  {
    fdigits = ecvtbuf(value, precision + 1, &decpt, &sign, cvtbuf);

    if (sign) *buffer++ = '-';
    *buffer++ = *fdigits;
    if (precision > 0) *buffer++ = '.';
    ee_bufcpy(buffer, fdigits + 1, precision);
    buffer += precision;
    *buffer++ = capexp ? 'E' : 'e';

    if (decpt == 0)
    {
      if (value == 0.0)
        exp = 0;
      else
        exp = -1;
    }
    else
      exp = decpt - 1;

    if (exp < 0)
    {
      *buffer++ = '-';
      exp = -exp;
    }
    else
      *buffer++ = '+';

    buffer[2] = (exp % 10) + '0';
    exp = exp / 10;
    buffer[1] = (exp % 10) + '0';
    exp = exp / 10;
    buffer[0] = (exp % 10) + '0';
    buffer += 3;
  }
  else if (fmt == 'f')
  {
    fdigits = fcvtbuf(value, precision, &decpt, &sign, cvtbuf);
    if (sign) *buffer++ = '-';
    if (*fdigits)
    {
      if (decpt <= 0)
      {
        *buffer++ = '0';
        *buffer++ = '.';
        for (pos = 0; pos < -decpt; pos++) *buffer++ = '0';
        while (*fdigits) *buffer++ = *fdigits++;
      }
      else
      {
        pos = 0;
        while (*fdigits)
        {
          if (pos++ == decpt) *buffer++ = '.';
          *buffer++ = *fdigits++;
        }
      }
    }
    else
    {
      *buffer++ = '0';
      if (precision > 0)
      {
        *buffer++ = '.';
        for (pos = 0; pos < precision; pos++) *buffer++ = '0';
      }
    }
  }

  *buffer = '\0';
}

static void decimal_point(char *buffer)
{
  while (*buffer)
  {
    if (*buffer == '.') return;
    if (*buffer == 'e' || *buffer == 'E') break;
    buffer++;
  }

  if (*buffer)
  {
    int n = __strnlen(buffer,256);
    while (n > 0)
    {
      buffer[n + 1] = buffer[n];
      n--;
    }

    *buffer = '.';
  }
  else
  {
    *buffer++ = '.';
    *buffer = '\0';
  }
}

static void cropzeros(char *buffer)
{
  char *stop;

  while (*buffer && *buffer != '.') buffer++;
  if (*buffer++)
  {
    while (*buffer && *buffer != 'e' && *buffer != 'E') buffer++;
    stop = buffer--;
    while (*buffer == '0') buffer--;
    if (*buffer == '.') buffer--;
    while (buffer!=stop)
    *++buffer=0;
  }
}

static char *flt(char *str, double num, int size, int precision, char fmt, int flags)
{
  char tmp[80];
  char c, sign;
  int n, i;

  // Left align means no zero padding
#ifdef TINY_PRINTF
#else
  if (flags & LEFT) flags &= ~ZEROPAD;
#endif

  // Determine padding and sign char
  c = (flags & ZEROPAD) ? '0' : ' ';
  sign = 0;
  if (flags & SIGN)
  {
    if (num < 0.0)
    {
      sign = '-';
      num = -num;
      size--;
    }
#ifdef TINY_PRINTF
#else
    else if (flags & PLUS)
    {
      sign = '+';
      size--;
    }
    else if (flags & SPACE)
    {
      sign = ' ';
      size--;
    }
#endif
  }

  // Compute the precision value
  if (precision < 0)
    precision = 6; // Default precision: 6

  // Convert floating point number to text
  parse_float(num, tmp, fmt, precision);

#ifdef TINY_PRINTF
#else
  if ((flags & HEX_PREP) && precision == 0) decimal_point(tmp);
#endif
  if (fmt == 'g' && !(flags & HEX_PREP)) cropzeros(tmp);

  n = __strnlen(tmp,256);

  // Output number with alignment and padding
  size -= n;
  if (!(flags & (ZEROPAD | LEFT))) while (size-- > 0) *str++ = ' ';
  if (sign) *str++ = sign;
  if (!(flags & LEFT)) while (size-- > 0) *str++ = c;
  for (i = 0; i < n; i++) *str++ = tmp[i];
  while (size-- > 0) *str++ = ' ';

  return str;
}

#endif

void app_debug_init(app_debug_get_timestamp_ms_cb_t get_ms, app_debug_lock_cb_t lock_cb)
{
    m_get_ms = get_ms;
    m_lock_cb = lock_cb;
#if DEBUG_ISR_ENABLE
    lwrb_init(&m_ringbuffer_debug_isr, &m_isr_buffer_size, DEBUG_ISR_RINGBUFFER_SIZE);
#endif
}

uint32_t app_debug_get_ms(void)
{
    if (m_get_ms)
    {
        return m_get_ms();
    }
    return 0;
}

void app_debug_print_nothing(const char *fmt, ...)
{
}

static void put_byte(uint8_t data)
{
    for (uint8_t index = 0; index < APP_DEBUG_NUMBER_OF_DEBUG_PORT; index++)
    {
        if (m_write_cb[index])
        {
            m_write_cb[index](&data, 1);
        }
    }
}

void put_string(char *s)
{
    if (s)
    {
        while (*s != '\0')
        {
            put_byte((*s++));
        }
    }
    else
    {
        put_byte('N');
        put_byte('U');
        put_byte('L');
        put_byte('L');
    }
}

#if DEBUG_ISR_ENABLE
static inline void put_byte_in_isr(uint8_t data)
{
    lwrb_write(&m_ringbuffer_debug_isr, &data, 1);
}

static inline void put_string_in_isr(char *s)
{
    if (s)
    {
        while (*s != '\0')
        {
            put_byte_in_isr((*s++));
        }
    }
    else
    {
        char *null_ptr = "NULL";
        lwrb_write(&m_ringbuffer_debug_isr, null_ptr, 4);
    }
}

static void custom_itoa_isr(int32_t val, int32_t radix, int32_t len)
{
    uint8_t c, r, sgn = 0, pad = ' ';
    uint8_t s[20], i = 0;
    uint32_t v;

    if (radix == 0)
    {
        return;
    }
    if (radix < 0)
    {
        radix = -radix;
        if (val < 0)
        {

            val = -val;
            sgn = '-';
        }
    }
    v = val;
    r = radix;
    if (len < 0)
    {
        len = -len;
        pad = '0';
    }

    if (len > 20)
    {
        return;
    }

    do
    {
        c = (uint8_t)(v % r);
        if (c >= 10)
            c += 7;
        c += '0';
        s[i++] = c;
        v /= r;
    } while (v);

    if (sgn)
    {
        s[i++] = sgn;
    }

    while (i < len)
    {
        s[i++] = pad;
    }

    do
    {
        put_byte_in_isr(s[--i]);
    } while (i);
}

#endif


#if 0
static void custom_itoa(int32_t val, int32_t radix, int32_t len)
{
    uint8_t c, r, sgn = 0, pad = ' ';
    uint8_t s[20], i = 0;
    uint32_t v;

    if (radix == 0)
    {
        return;
    }
    if (radix < 0)
    {
        radix = -radix;
        if (val < 0)
        {

            val = -val;
            sgn = '-';
        }
    }
    v = val;
    r = radix;
    if (len < 0)
    {
        len = -len;
        pad = '0';
    }

    if (len > 20)
    {
        return;
    }

    do
    {
        c = (uint8_t)(v % r);
        if (c >= 10)
            c += 7;
        c += '0';
        s[i++] = c;
        v /= r;
    } while (v);

    if (sgn)
    {
        s[i++] = sgn;
    }

    while (i < len)
    {
        s[i++] = pad;
    }

    do
    {
        put_byte(s[--i]);
    } while (i);
}
#endif

// void app_debug_print_raw(const char *fmt, ...)
// {
//     if (m_lock_cb)
//     {
//         m_lock_cb(true, 0xFFFFFFFF);
//     }
// #if 0
//     int32_t n;
//     char *p = &m_debug_buffer[0];
//     int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
//     int time_stamp_size;

//     p += sprintf(m_debug_buffer, "<%u>: ", app_debug_get_ms());
//     time_stamp_size = (p-m_debug_buffer);
//     size -= time_stamp_size;
//     va_list args;

//     va_start (args, fmt);
//     n = vsnprintf(p, size, fmt, args);
//     if (n > (int)size)
//     {
//         for (uint8_t index = 0; index < APP_DEBUG_NUMBER_OF_DEBUG_PORT; index++)
//         {
//             if (m_write_cb[index])
//             {
//                 m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
//             }
//         }

//     }
//     else if (n > 0)
//     {
//         for (uint8_t index = 0; index < APP_DEBUG_NUMBER_OF_DEBUG_PORT; index++)
//         {
//             if (m_write_cb[index] != NULL)
//             {
//                 m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
//             }
//         }

//     }
//     va_end(args);
// #else

//     va_list arp;
//     int32_t d, r, w, s, l;
//     va_start(arp, fmt);

//     while ((d = *fmt++) != 0)
//     {
//         if (d != '%')
//         {
//             put_byte(d);
//             continue;
//         }
//         const char *next = fmt;
//         if (*next == '%')
//         {
//             fmt++;
//             put_byte('%');
//             continue;
//         }

//         d = *fmt++;
//         w = r = s = l = 0;

//         if (d == '0')
//         {
//             d = *fmt++; s = 1;
//         }

//         while ((d >= '0') && (d <= '9'))
//         {
//             w += w * 10 + (d - '0');
//             d = *fmt++;
//         }

//         if (s)
//         {
//             w = -w;
//         }

//         if (d == 'l')
//         {
//             l = 1;
//             d = *fmt++;
//         }

//         if (!d)
//         {
//             break;
//         }

//         if (d == 's')
//         {
//             put_string(va_arg(arp, char*));
//             continue;
//         }

//         if (d == 'c')
//         {
//             put_byte((char)va_arg(arp, int32_t));
//             continue;
//         }

//         if (d == 'u') r = 10;
//         if (d == 'd') r = -10;
//         if (d == 'X' || d == 'x') r = 16; // 'x' added by mthomas in increase compatibility
//         if (d == 'b') r = 2;

//         if (!r)
//         {
//             break;
//         }

//         if (l)
//         {
//             custom_itoa((int32_t)va_arg(arp, int32_t), r, w);
//         }
//         else
//         {
//             if (r > 0)
//             {
//                 custom_itoa((uint32_t)va_arg(arp, int32_t), r, w);
//             }
//             else
//             {
//                 custom_itoa((int32_t)va_arg(arp, int32_t), r, w);
//             }
//         }
//     }
//     va_end(arp);
// #endif
//     if (m_lock_cb)
//     {
//         m_lock_cb(false, 0);
//     }
// }

static int ee_skip_atoi(const char **s)
{
    int i = 0;
    while (is_digit(**s))
        i = i * 10 + *((*s)++) - '0';
    return i;
}

static int ee_number(long num, int base, int size, int precision, int type)
{
    char c;
    char sign, tmp[66];
    char *dig = lower_digits;
    int i;
    int number_of_byte = 0;

    if (type & UPPERCASE)
        dig = upper_digits;
    if (type & LEFT)
        type &= ~ZEROPAD;
    if (base < 2 || base > 36)
        return 0;

    c = (type & ZEROPAD) ? '0' : ' ';
    sign = 0;
    if (type & SIGN)
    {
        if (num < 0)
        {
            sign = '-';
            num = -num;
            size--;
        }
        else if (type & PLUS)
        {
            sign = '+';
            size--;
        }
        else if (type & SPACE)
        {
            sign = ' ';
            size--;
        }
    }

    if (type & HEX_PREP)
    {
        if (base == 16)
            size -= 2;
        else if (base == 8)
            size--;
    }

    i = 0;

    if (num == 0)
        tmp[i++] = '0';
    else
    {
        while (num != 0)
        {
            tmp[i++] = dig[((unsigned long)num) % (unsigned)base];
            num = ((unsigned long)num) / (unsigned)base;
        }
    }

    if (i > precision)
        precision = i;
    size -= precision;
    if (!(type & (ZEROPAD /* TINY option   | LEFT */)))
    {
        while (size-- > 0)
        {
            put_byte(' ');
            number_of_byte++;
        }
    }
    if (sign)
    {
        put_byte(sign);
        number_of_byte++;
    }

    if (type & HEX_PREP)
    {
        if (base == 8)
        {
            put_byte('0');
            number_of_byte++;
        }
        else if (base == 16)
        {
            put_byte('0');
            put_byte(lower_digits[33]);
            number_of_byte += 2;
        }
    }

    if (!(type & LEFT))
    {
        while (size-- > 0)
        {
            put_byte(c);
            number_of_byte++;
        }
    }
    while (i < precision--)
    {
        put_byte('0');
        number_of_byte++;
    }
    while (i-- > 0)
    {
        put_byte(tmp[i]);
        number_of_byte++;
    }
    while (size-- > 0)
    {
        put_byte(' ');
        number_of_byte++;
    }

    return number_of_byte;
}

void app_debug_print_raw(const char *fmt, ...)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    unsigned long num;
    int base;
#ifdef HAS_FLOAT
    char *str;
#endif
    int len;
    int i;
    char *s;

    int flags; // Flags to number()

    int field_width; // Width of output field
    int precision;   // Min. # of digits for integers; max number of chars for from string
    int qualifier;   // 'h', 'l', or 'L' for integer fields
    uint32_t nb_of_bytes = 0;
    va_list args;
    va_start(args, fmt);

    for (; *fmt; fmt++)
    {
        if (*fmt != '%')
        {
            nb_of_bytes++;
            put_byte(*fmt);
            continue;
        }

        // Process flags
        flags = 0;
    repeat:
        fmt++; // This also skips first '%'
        switch (*fmt)
        {
        case '-':
            flags |= LEFT;
            goto repeat;
        case '+':
            flags |= PLUS;
            goto repeat;
        case ' ':
            flags |= SPACE;
            goto repeat;
        case '#':
            flags |= HEX_PREP;
            goto repeat;
        case '0':
            flags |= ZEROPAD;
            goto repeat;
        }

        // Get field width
        field_width = -1;
        if (is_digit(*fmt))
            field_width = ee_skip_atoi(&fmt);
        else if (*fmt == '*')
        {
            fmt++;
            field_width = va_arg(args, int);
            if (field_width < 0)
            {
                field_width = -field_width;
                flags |= LEFT;
            }
        }

        // Get the precision
        precision = -1;
        if (*fmt == '.')
        {
            ++fmt;
            if (is_digit(*fmt))
                precision = ee_skip_atoi(&fmt);
            else if (*fmt == '*')
            {
                ++fmt;
                precision = va_arg(args, int);
            }
            if (precision < 0)
                precision = 0;
        }

        // Get the conversion qualifier
        qualifier = -1;
        if (*fmt == 'l' || *fmt == 'L')
        {
            qualifier = *fmt;
            fmt++;
        }

        // Default base
        base = 10;

        switch (*fmt)
        {
        case 'c':
            if (!(flags & LEFT))
            {
                while (--field_width > 0)
                {
                    put_byte(' ');
                    nb_of_bytes++;
                };
            }

            put_byte((unsigned char)va_arg(args, int));
            nb_of_bytes++;

            while (--field_width > 0)
            {
                put_byte(' ');
                nb_of_bytes++;
            };
            continue;

        case 's':
            s = va_arg(args, char *);
            if (!s)
            {
                s = "<NULL>";
            }
            if (precision > 0)
            {
            	len = __strnlen(s, precision);
            }
            else
            {
            	len = strlen(s);
            }
            if (!(flags & LEFT))
            {
                while (len < field_width--)
                {
                    put_byte(' ');
                    nb_of_bytes++;
                };
            }
            for (i = 0; i < len; ++i)
            {
                put_byte(*s++);
                nb_of_bytes++;
            };

            while (len < field_width--)
            {
                put_byte(' ');
                nb_of_bytes++;
            };
            continue;

        case 'p':
            if (field_width == -1)
            {
                field_width = 2 * sizeof(void *);
                flags |= ZEROPAD;
            }
            nb_of_bytes += ee_number((unsigned long)va_arg(args, void *), 16, field_width, precision, flags);
            continue;

        case 'A':
            flags |= UPPERCASE;

        // case 'a':
        //     if (qualifier == 'l')
        //         str = eaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
        //     else
        //         str = iaddr(str, va_arg(args, unsigned char *), field_width, precision, flags);
        //     continue;

        // Integer number formats - set up the flags and "break"
        case 'o':
            base = 8;
            break;

        case 'X':
            flags |= UPPERCASE;

        case 'x':
            base = 16;
            break;

        case 'd':
        case 'i':
            flags |= SIGN;

        case 'u':
            break;

#ifdef HAS_FLOAT

        case 'f':
            str = flt(str, va_arg(args, double), field_width, precision, *fmt, flags | SIGN);
            continue;

#endif

        default:
            if (*fmt != '%')
            {
                put_byte('%');
                nb_of_bytes++;
            }
            if (*fmt)
            {
                put_byte(*fmt);
                nb_of_bytes++;
            }
            else
            {
                --fmt;
            }
            continue;
        }

        if (qualifier == 'l')
            num = va_arg(args, unsigned long);
        else if (flags & SIGN)
            num = va_arg(args, int);
        else
            num = va_arg(args, unsigned int);

        nb_of_bytes += ee_number(num, base, field_width, precision, flags);
    }
    va_end(args);


    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
    // return nb_of_bytes;
}

static void simple_print_hex(uint8_t hex_value)
{
    const char *hex_str = "0123456789ABCDEF";
    put_byte(hex_str[(hex_value >> 4) & 0x0F]);
    put_byte(hex_str[(hex_value)&0x0F]);
}

#if DEBUG_ISR_ENABLE
void app_debug_print_isr(const char *fmt, ...)
{
#if 0
    int32_t n;
    char *p = &m_debug_buffer[0];
    int size = SEGGER_RTT_PRINTF_BUFFER_SIZE;
    int time_stamp_size;

    p += sprintf(m_debug_buffer, "<%u>: ", app_debug_get_ms());
    time_stamp_size = (p-m_debug_buffer);
    size -= time_stamp_size;
    va_list args;

    va_start (args, fmt);
    n = vsnprintf(p, size, fmt, args);
    if (n > (int)size) 
    {
        for (uint8_t index = 0; index < APP_DEBUG_NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index])
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }
        
    } 
    else if (n > 0) 
    {
        for (uint8_t index = 0; index < APP_DEBUG_NUMBER_OF_DEBUG_PORT; index++)
        {
            if (m_write_cb[index] != NULL)
            {
                m_write_cb[index](0, m_debug_buffer, n + time_stamp_size);
            }
        }

    }
    va_end(args);
#else

    va_list arp;
    int32_t d, r, w, s, l;
    va_start(arp, fmt);

    while ((d = *fmt++) != 0)
    {
        if (d != '%')
        {
            put_byte_in_isr(d);
            continue;
        }
        const char *next = fmt;
        if (*next == '%')
        {
            fmt++;
            put_byte_in_isr('%');
            continue;
        }

        d = *fmt++;
        w = r = s = l = 0;

        if (d == '0')
        {
            d = *fmt++;
            s = 1;
        }

        while ((d >= '0') && (d <= '9'))
        {
            w += w * 10 + (d - '0');
            d = *fmt++;
        }

        if (s)
        {
            w = -w;
        }

        if (d == 'l')
        {
            l = 1;
            d = *fmt++;
        }

        if (!d)
        {
            break;
        }

        if (d == 's')
        {
            put_string_in_isr(va_arg(arp, char *));
            continue;
        }

        if (d == 'c')
        {
            put_byte_in_isr((char)va_arg(arp, int32_t));
            continue;
        }

        if (d == 'u')
            r = 10;
        if (d == 'd')
            r = -10;
        if (d == 'X' || d == 'x')
            r = 16; // 'x' added by mthomas in increase compatibility
        if (d == 'b')
            r = 2;

        if (!r)
        {
            break;
        }

        if (l)
        {
            custom_itoa_isr((int32_t)va_arg(arp, int32_t), r, w);
        }
        else
        {
            if (r > 0)
            {
                custom_itoa_isr((uint32_t)va_arg(arp, int32_t), r, w);
            }
            else
            {
                custom_itoa_isr((int32_t)va_arg(arp, int32_t), r, w);
            }
        }
    }
    va_end(arp);
#endif
}

/**
 * @brief           Flush all data in ringbuffer of ISR
 */
void app_debug_isr_ringbuffer_flush(void)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }

    uint8_t tmp;
    while (lwrb_read(&m_ringbuffer_debug_isr, &tmp, 1))
    {
        put_byte(tmp);
    }

    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

#endif /* DEBUG_ISR_ENABLE */

void app_debug_dump(const void *data, int32_t len, const char *message)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }

    uint8_t *p = (uint8_t *)data;
    uint8_t buffer[16];
    int32_t i_len;
    int32_t i;
    
    char tmp[32];
    sprintf(tmp, "%ld", len);
    put_string((char*)message);
    put_string(" : ");
    put_string(tmp);
    put_string("bytes\r\n");
    
//    DEBUG_RAW("%s : %u bytes\r\n", message, len);

    while (len > 0)
    {
        i_len = (len > 16) ? 16 : len;
        memset(buffer, 0, 16);
        memcpy(buffer, p, i_len);
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
            {
                simple_print_hex(buffer[i]);
                put_byte(' ');
            }
            else
            {
                put_string("   ");
            }
        }
        put_string("\t");
        for (i = 0; i < 16; i++)
        {
            if (i < i_len)
            {
                if (isprint(buffer[i]))
                {
                    sprintf(tmp, "%c", (char)buffer[i]);
                    put_string(tmp);
                }
                else
                {
                    put_string(".");
                }
            }
            else
            {
                put_string(" ");
            }
        }
        put_string("\r\n");
        len -= i_len;
        p += i_len;
    }

    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

void app_debug_register_callback_print(app_debug_output_cb_t callback)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    uint8_t callback_exist = 0; // Check for existion function pointer in function pointer arry
    if (callback)
    {
        for (uint8_t func_count = 0; func_count < APP_DEBUG_NUMBER_OF_DEBUG_PORT; func_count++)
        {
            if (callback == m_write_cb[func_count])
            {
                // Callback already existed in array
                callback_exist = 1;
            }
        }

        if (!callback_exist)
        {
            m_write_cb[number_of_callback] = callback;
            number_of_callback++;
        }
    }
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

void app_debug_unregister_callback_print(app_debug_output_cb_t callback)
{
    if (m_lock_cb)
    {
        m_lock_cb(true, 0xFFFFFFFF);
    }
    for (uint8_t func_count = 0; func_count < APP_DEBUG_NUMBER_OF_DEBUG_PORT; func_count++)
    {
        if (callback == m_write_cb[func_count])
        {
            number_of_callback--;
        }
    }
    if (m_lock_cb)
    {
        m_lock_cb(false, 0);
    }
}

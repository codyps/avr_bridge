/* ex: set et sw=4 sts=4: */

#include <math.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "libarduino2.h"

#define BIT_SET(_out_, _bit_, _val_) ((_out_) = ((_out_) & ~(1 << (_bit_))) | ((_val_) << (_bit_)))
#define BIT_GET(_out_, _bit_) (((_out_) >> (_bit_)) & 1)
#define BIT_HI(_out_, _bit_) ((_out_) |= (1 << (_bit_)))
#define BIT_LO(_out_, _bit_) ((_out_) &= ~(1 << (_bit_)))

/*
 * Digital IO
 */
void digital_init(pin_t pin, pinmode_t mode) {
    if (pin < 8) {
        BIT_SET(DDRD, pin, mode);
    } else if (pin < 16) {
        BIT_SET(DDRB, pin - 8, mode);
    } else {
        ERROR("digital_init", "invalid pin index");
    }
}

bool digital_get(pin_t pin) {
    if (pin < 8) {
        return BIT_GET(PORTD, pin);
    } else if (pin < 16) {
        return BIT_GET(PORTB, pin - 8);
    } else {
        ERROR("digital_get", "invalid pin index");
        return 0;
    }
}

void digital_set(pin_t pin, bool value) {
    if (pin < 8) {
        BIT_SET(PORTD, pin, value);
    } else if (pin < 16) {
        BIT_SET(PORTB, pin - 8, value);
    } else {
        ERROR("digital_set", "invalid pin index");
    }
}

/*
 * Analog Input (ADCs)
 */
#define ADC_NUM           6
#define ADC_MAXCLOCK      200000L
#define ADC_PRESCALE_BITS ((uint8_t)ceil(log((float)F_CPU / ADC_MAXCLOCK) / log(2)))
#define ADC_PRESCALE      ((uint8_t)pow(2, ADC_PRESCALE_BITS))
#define ADC_CUR           (ADMUX & 7)
#define ADC_NEXT          ((ADC_CUR + 1) & 7)
#define ADC_PREV          ((ADC_CUR) ? ADC_CUR - 1 : ADC_NUM - 1)
#define ADC_SET(_pin_)    (ADMUX = (ADMUX & ~7) | ((_pin_) & 7))

static uint16_t volatile g_adc_val[6];
static bool volatile     g_adc_done = false;

void analog_init(void) {
    int i;

    /* Set all pins connected to ADCs as analog inputs. */
    for (i = 0; i < ADC_NUM; ++i) {
        BIT_SET(DDRC, i, PIN_INPUT);
    }

    BIT_LO(PRR,    PRADC); /* disable power reduction */
    BIT_HI(ADMUX,  REFS0); /* use AVCC as the reference voltage */
    BIT_HI(ADCSRA, ADATE); /* start conversion on interrupt */
    BIT_HI(ADCSRA, ADIE);  /* interrupt on completion */
    BIT_HI(ADCSRA, ADEN);  /* enable */
    ADCSRA |= ADC_PRESCALE_BITS; /* clock divisor */

    BIT_HI(ADCSRA, ADSC);        /* begin conversion */
    _delay_loop_2(ADC_PRESCALE); /* wait for ADMUX to be read */
    ADC_SET(ADC_NEXT);           /* prepare for the next conversion */
}

bool analog_available(void) {
    bool buf   = g_adc_done;
    g_adc_done = false;
    return buf;
}

uint16_t analog_get(pin_t pin) {
    uint16_t value;
    if (pin < ADC_NUM) {
        /* Reading 16-bit value from g_adc_cur must be atomic. */
        value = g_adc_val[pin];
        return value;
    } else {
        ERROR("analog_get", "invalid pin index");
        return 0;
    }
}

ISR(ADC_vect) {
    g_adc_val[ADC_PREV] = ADC; /* ADC_CUR is in progress */
    g_adc_done = g_adc_done || !ADC_CUR;
    ADC_SET(ADC_NEXT);
}

/*
 * Analog Output (PWM)
 */
void pwm_init(pin_t pin) {
    if (pin == 1 || pin == 2) {
        BIT_HI(TCCR1A, WGM10); /* 8-bit */
        BIT_HI(TCCR1B, CS12);  /* divide by 256 */
        TCNT1 = 0;
    } else if (pin == 3 || pin == 4) {
        BIT_HI(TCCR2A, WGM20);
        BIT_HI(TCCR2B, CS21);
        BIT_HI(TCCR2B, CS22);
        TCNT2 = 0;
    }

    switch (pin) {
    case 1:
        OCR1A = 0;              /* Initial value. */
        BIT_HI(DDRB, 1);        /* Set pin as output. */
        BIT_HI(TCCR1A, COM1A1); /* Enable */
        break;

    case 2:
        OCR1B = 0;
        BIT_HI(DDRB, 2);
        BIT_HI(TCCR1A, COM1B1);
        break;

    case 3:
        OCR2A = 0;
        BIT_HI(DDRB, 3);
        BIT_HI(TCCR2A, COM2A1);
        break;

    case 4:
        OCR2B = 0;
        BIT_HI(DDRD, 3);
        BIT_HI(TCCR2A, COM2B1);
        break;

    default:
        ERROR("pwm_init", "invalid pin index");
    }
}

void pwm_set(pin_t pin, uint8_t value) {
    switch (pin) {
    case 1:
        OCR1A = value;
        break;

    case 2:
        OCR1B = value;
        break;

    case 3:
        OCR2A = value;
        break;

    case 4:
        OCR2B = value;
        break;

    default:
        ERROR("pwm_set", "invalid pin index");
    }
}

/*
 * Serial Communication (UART)
 */
#define UART_BAUD (F_CPU / (SERIAL_BAUD * 16L) - 1)
#define UART_SIZE 64

static struct uart_rx {
    uint8_t buf[UART_SIZE];
    uint8_t head;
    uint8_t tail;
} g_uart_rx;

static struct uart_tx {
    uint8_t buf[UART_SIZE];
    uint8_t head;
    uint8_t tail;
} g_uart_tx;

static FILE g_uart;

FILE *serial_init(void) {
    /* Set the default baud rate. */
    UBRR0H = UART_BAUD >> 8;
    UBRR0L = UART_BAUD;

    BIT_HI(UCSR0B, TXEN0);  /* enable send */
    BIT_HI(UCSR0B, RXEN0);  /* enable receive */
    BIT_HI(UCSR0B, RXCIE0); /* enable receive interrupt */

    /* g_uart is initialized via FDEV_SETUP_STREAM initializer above */
    fdev_setup_stream(&g_uart, serial_putchar, serial_getchar, _FDEV_SETUP_RW);

    stdout = &g_uart;
    stderr = &g_uart;
    stdin  = &g_uart;
    return &g_uart;
}

inline static void barrier(void)
{
    asm volatile("":::"memory");
}

int serial_getchar_nonblock(FILE *fp) {
    char ch;

    /* Only read from the buffer if new data is available. */
    if (g_uart_rx.head != g_uart_rx.tail) {
        ch = g_uart_rx.buf[g_uart_rx.tail];
        g_uart_rx.tail = (g_uart_rx.tail + 1) & (sizeof(g_uart_rx.buf) - 1);
        return ch;
    } else {
        return EOF;
    }
}

int serial_getchar(FILE *fp) {
    char ch;

    /* Block until data is available. */

    while (g_uart_rx.head == g_uart_rx.tail) {
        /* force a re-read of uart_read, uart_write */
        barrier();
    }

    /* Read UART data from the ring buffer. */
    ch = g_uart_rx.buf[g_uart_rx.tail];
    g_uart_rx.tail = (g_uart_rx.tail + 1) & (sizeof(g_uart_rx.buf) - 1);
    return ch;
}

static void serial_txi_on(void) {
    UCSR0B |= (1 << UDRIE0);
}

static void serial_txi_off(void) {
    UCSR0B &= ~(1 << UDRIE0);
}

int serial_putchar(char ch, FILE *fp) {
    /* Block until space is available. */
    while (((g_uart_tx.head + 1) & (sizeof(g_uart_tx.buf) - 1))
            == g_uart_tx.tail) {
        barrier();
    }

    g_uart_tx.buf[g_uart_tx.head] = ch;
    g_uart_tx.head = (g_uart_tx.head + 1) & (sizeof(g_uart_tx.buf) - 1);

    serial_txi_on();
    return 0;
}

ISR(USART_UDRE_vect) {
    if (g_uart_tx.head == g_uart_tx.tail) {
        serial_txi_off();
        return;
    }

    UDR0 = g_uart_tx.buf[g_uart_tx.tail];

    g_uart_tx.tail = (g_uart_tx.tail + 1) & (sizeof(g_uart_tx.buf) - 1);
}

ISR(USART_RX_vect) {
    g_uart_rx.buf[g_uart_rx.head] = UDR0;
    g_uart_rx.head = (g_uart_rx.head + 1) & (sizeof(g_uart_rx.buf) - 1);
}

/*
 * Timer
 */
static bool     g_timer_done  = false;
static uint16_t g_timer_target_ms = 0;
static uint32_t g_timer_ms;
static uint32_t g_timer_fract;

static uint32_t g_timer0_ms;
static uint32_t g_timer0_fract;

void timer_init(void) {
    TCCR0B = 0;
    TIMSK0 |= (1 << TOIE0); /* enable interrupt on overflow */
    TCCR0B |= (1 << CS00) | (1 << CS01); /* 64 prescale factor */
}

void timer_start(uint16_t ms) {
    g_timer_target_ms = ms;
}

bool timer_done(void) {
    bool temp = g_timer_done;
    g_timer_done = false;
    return temp;
}

#define US_PER_TIM0_OVF ((64 * 256) / (F_CPU / 1000 / 1000))
#define MS_INC (US_PER_TIM0_OVF / 1000)

#define FRACT_INC ((US_PER_TIM0_OVF % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

#define UPDATE_TIM0(ms, fract)  \
    {                           \
        uint32_t m = (ms);      \
        uint32_t f = (fract);   \
        m += MS_INC;            \
        f += FRACT_INC;         \
        if (f >= FRACT_MAX) {   \
            f -= FRACT_MAX;     \
            m += 1;             \
        }                       \
                                \
        (ms) = m;               \
        (fract) = f;            \
    }

ISR(TIMER0_OVF_vect) {
    if (g_timer_ms >= g_timer_target_ms) {
        g_timer_ms = 0;
        g_timer_fract = 0;
        g_timer_done  = true;
    }

    UPDATE_TIM0(g_timer_ms, g_timer_fract);

    UPDATE_TIM0(g_timer0_ms, g_timer0_fract);
}

/*
 * Miscellaneous
 */
int readline(char *begin, char *end) {
    char   *cursor  = begin;
    uint8_t escape  = 0;

    for (;;) {
        char ch = getchar();


        /* Parse ANSI escape sequences, which have the following syntax:
         *   CLI ##;##; ... LTR
         * where
         *   CLI is the 27 followed by one in the range 64-95.
         *   ## is an arbitrarily long string of digits
         *   LTR is in the range 64-126
         */
        if (ch == 27) {
            escape = 1;
        }
        else if (escape == 1 && 64 <= ch && ch <= 95) {
            escape = 2;
        }
        else if (escape == 2 && (('0' <= ch && ch <= '9') || (ch == ';'))) {
            escape = 3;
        }
        else if ((escape == 2 || escape == 3) && 64 <= ch && ch <= 126) {
            escape  = 0;
        }
        else if (escape != 0) {
            ERROR("readline", "invalid ANSI escape sequence");
            escape = 0;
        }
        /* Printable characters. */
        else if (32 <= ch && ch <= 126 && cursor != end) {
            *cursor = ch;
            ++cursor;
            putchar(ch);
        }
        /* Backspace deletes the previous character in the buffer. */
        else if (ch == 127 && cursor != begin) {
            /* Move the cursor backwards. */
            putchar(0x1B);
            putchar(0x5B);
            putchar('1');
            putchar('D');

            /* Overwrite the current character with a space. */
            putchar(' ');

            /* Move the cursor backwards. */
            putchar(0x1B);
            putchar(0x5B);
            putchar('1');
            putchar('D');
            --cursor;
        }
        /* We're done when we receive a newline. */
        else if (ch == 13) {
            *cursor = '\0';
            break;
        }
    }
    putchar('\r');
    putchar('\n');
    return cursor - begin;
}

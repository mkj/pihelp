#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <util/crc16.h>

//#include "simple_ds18b20.h"
//#include "onewire.h"

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// TICK should be 8 or less (8 untested). all timers need
// to be a multiple.

#define TICK 1
// we have 1024 prescaler, 32768 crystal.
#define SLEEP_COMPARE (32*TICK-1)

#define KEYLEN 20

#define VALUE_NOSENSOR 0x07D0 // 125 degrees
#define VALUE_BROKEN 0x07D1 // 125.0625

#define OVERSHOOT_MAX_DIV 1800.0 // 30 mins
#define WORT_INVALID_TIME 900 // 15 mins
// fridge min/max are only used if the wort sensor is invalid
#define FRIDGE_AIR_MIN_RANGE 40 // 4º
#define FRIDGE_AIR_MAX_RANGE 40 // 4º

#define BAUD 19200
#define UBRR ((F_CPU)/8/(BAUD)-1)

#define PORT_LED PORTC
#define DDR_LED DDRC
#define PIN_LED PC4

#define PORT_SHDN PORTD
#define DDR_SHDN DDRD
#define PIN_SHDN PD7

#define PORT_FRIDGE PORTD
#define DDR_FRIDGE DDRD
#define PIN_FRIDGE PD6

// total amount of 16bit values available for measurements.
// adjust emperically, be sure to allow enough stack space too
#define TOTAL_MEASUREMENTS 800

// each sensor slot uses 8 bytes
#define MAX_SENSORS 6

// fixed at 8, have a shorter name
#define ID_LEN OW_ROMCODE_SIZE

// #define HAVE_UART_ECHO

// stores a value of clock_epoch combined with the remainder of TCNT2,
// for 1/32 second accuracy
struct epoch_ticks
{
    uint32_t ticks;
    // remainder
    uint8_t rem;
};

// eeprom-settable parameters, default values defined here. 
// all timeouts should be a multiple of TICK
static uint32_t watchdog_long_limit = 60*60*24;
static uint32_t watchdog_short_limit = 0;
static uint32_t newboot_limit = 60*10;

// avr proves itself
static uint8_t avr_keys[NKEYS][KEYLEN] = {0};


// ---- Atomic guards required accessing these variables
// clock_epoch in seconds
static uint32_t clock_epoch;
// watchdog counts up
static uint32_t watchdog_long_count;
static uint32_t watchdog_short_count;
// newboot counts down - it's a one-shot
static uint32_t newboot_count;
// ---- End atomic guards required

// boolean flags
static uint8_t watchdog_long_hit;
static uint8_t watchdog_short_hit;
static uint8_t newboot_hit;

static uint8_t readpos;
static char readbuf[50];
static uint8_t have_cmd;

int uart_putchar(char c, FILE *stream);
static void long_delay(int ms);
static void blink();
static uint16_t adc_vcc();

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
        _FDEV_SETUP_WRITE);

// thanks to http://projectgus.com/2010/07/eeprom-access-with-arduino/
#define eeprom_read_to(dst_p, eeprom_field, dst_size) eeprom_read_block((dst_p), (void *)offsetof(struct __eeprom_data, eeprom_field), (dst_size))
#define eeprom_read(dst, eeprom_field) eeprom_read_to((&dst), eeprom_field, sizeof(dst))
#define eeprom_write_from(src_p, eeprom_field, src_size) eeprom_write_block((src_p), (void *)offsetof(struct __eeprom_data, eeprom_field), (src_size))
#define eeprom_write(src, eeprom_field) { eeprom_write_from(&src, eeprom_field, sizeof(src)); }

#define EXPECT_MAGIC 0xdf83

struct __attribute__ ((__packed__)) __eeprom_data {
    uint32_t watchdog_long_limit;
    uint32_t watchdog_short_limit;
    uint32_t newboot_limit;

    uint8_t avr_key[NKEYS][KEYLEN];

    uint16_t magic;
};

static void deep_sleep();

// Very first setup
static void
setup_chip()
{
    cli();

    // stop watchdog timer (might have been used to cause a reset)
    wdt_reset();
    MCUSR &= ~_BV(WDRF);
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    WDTCSR = 0;

    // set to 8S, in case sha1 is slow etc.
    wdt_enable(WDTO_8S);

    // Set clock to 2mhz
    CLKPR = _BV(CLKPCE);
    // divide by 4
    CLKPR = _BV(CLKPS1);

    // enable pullups
    // XXX matt pihelp
    PORTB = 0xff; // XXX change when using SPI
    PORTD = 0xff;
    PORTC = 0xff;

    // 3.3v power for bluetooth and SD
    DDR_LED |= _BV(PIN_LED);

    // set pullup
    PORTD |= _BV(PD2);
    // INT0 setup
    EICRA = (1<<ISC01); // falling edge - data sheet says it won't work?
    EIMSK = _BV(INT0);

    // comparator disable
    ACSR = _BV(ACD);

    // disable adc pin input buffers
    DIDR0 = 0x3F; // acd0-adc5
    DIDR1 = (1<<AIN1D)|(1<<AIN0D); // ain0/ain1

    sei();
}

static void
get_epoch_ticks(struct epoch_ticks *t)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        t->ticks = clock_epoch;
        t->rem = TCNT2;
    }
}

static void
setup_tick_counter()
{
    // set up counter2. 
    // COM21 COM20 Set OC2 on Compare Match (p116)
    // WGM21 Clear counter on compare
    //TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(WGM21);
    // toggle on match
    TCCR2A = _BV(COM2A0);
    // CS22 CS21 CS20  clk/1024
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
    // set async mode
    ASSR |= _BV(AS2);
    TCNT2 = 0;
    OCR2A = SLEEP_COMPARE;
    // interrupt
    TIMSK2 = _BV(OCIE2A);
}

static void 
uart_on()
{
    // Power reduction register
    PRR &= ~_BV(PRUSART0);
 
    // All of this needs to be done each time after turning off the PRR
    // baud rate
    UBRR0H = (unsigned char)(UBRR >> 8);
    UBRR0L = (unsigned char)UBRR;
    // set 2x clock, improves accuracy of UBRR
    UCSR0A |= _BV(U2X0);
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
    //8N1
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
    uart_enabled = 1;
}

static void 
uart_off()
{
    // Turn off interrupts and disable tx/rx
    UCSR0B = 0;
    uart_enabled = 0;

    // Power reduction register
    PRR |= _BV(PRUSART0);
}

int 
uart_putchar(char c, FILE *stream)
{
    if (!uart_enabled)
    {
        return EOF;
    }
    // XXX could perhaps sleep in the loop for power.
    if (c == '\n')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\r';
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    if (stream == crc_stdout)
    {
        crc_out = _crc_ccitt_update(crc_out, c);
    }
    if (c == '\r')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\n';
        if (stream == crc_stdout)
        {
            crc_out = _crc_ccitt_update(crc_out, '\n');
        }
    }
    return (unsigned char)c;
}

static void
cmd_reset()
{
    printf_P(PSTR("reset\n"));
    _delay_ms(100);
    cli(); // disable interrupts 
    wdt_enable(WDTO_15MS); // enable watchdog 
    while(1); // wait for watchdog to reset processor 
}



static void
cmd_get_params()
{
    uint32_t cur_watchdog_long, cur_watchdog_short, cur_newboot;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        cur_watchdog_long = watchdot_long_count;
        cur_watchdog_short = watchdot_short_count;
        cur_newboot = newboot_limit_count;
    }

    printf_P(PSTR("limit (count) : watchdog_long %lu (%lu) watchdog_short %lu (%lu) newboot %lu (%lu)\n"),
        watchdog_long_limit,
        watchdog_long_count,
        watchdog_short_limit,
        watchdog_short_count,
        newboot_limit,
        newboot_count);
}

static void
cmd_set_params(const char *params)
{
    uint32_t new_watchdog_long_limit;
    uint32_t new_watchdog_short_limit;
    uint32_t new_newboot_limit;

    int ret = sscanf_P(params, PSTR("%lu %lu %lu"),
            &new_watchdog_long_limit,
            &new_watchdog_short_limit,
            &new_newboot_limit);


    if (ret != 3)
    {
        printf_P(PSTR("Bad values\n"));
    }
    else
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            eeprom_write(new_watchdog_long_limit, watchdog_long_limit);
            eeprom_write(new_watchdog_short_limit, watchdog_short_limit);
            eeprom_write(new_newboot_limit, newboot_limit);
            uint16_t magic = EXPECT_MAGIC;
            eeprom_write(magic, magic);
        }
        printf_P(PSTR("set_params for next boot\n"));
        printf_P(PSTR("watchdog_long %lu watchdog_short %lu newboot %lu\n"),
            new_watchdog_long_limit,
            new_watchdog_short_limit,
            new_newboot_limit);

    }
}

uint8_t from_hex(char c)
{
    if (c >= '0' && c <= '9') {
        return c-'0';
    }
    if (c >= 'a' && c <= 'f') {
        return c-'a' + 0xa;
    }
    if (c >= 'A' && c <= 'F') {
        return c-'A' + 0xa;
    }
    return 0;
}

static void
cmd_set_avr_key(const char *params)
{
    // "N HEXKEY"
    if (strlen(params)) != KEYLEN*2+2) {
        printf_P(PSTR("Wrong length key\n"));
        return;
    }

    uint8_t new_key[KEYLEN];
    for (int i = 0, p = 0; i < KEYLEN; i++, p += 2)
    {
        new_key[i] = (fromhex(params[p]) << 4) | fromhex(params[p+1]);
    }
}

static void
load_params()
{
    uint16_t magic;
    eeprom_read(magic, magic);
    if (magic == EXPECT_MAGIC)
    {
        eeprom_read(watchdog_long_limit, watchdog_long_limit);
        eeprom_read(watchdog_short_limit, watchdog_short_limit);
        eeprom_read(netboot_limit);
        eeprom_read(avr_key);
   }
}

// returns true if eeprom was written
static bool
set_initial_eeprom()
{
    uint16_t magic;
    eeprom_read(magic, magic);
    if (magic == EXPECT_MAGIC)
    {
        return false;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        eeprom_write(measure_wake, measure_wake);
        eeprom_write(comms_wake, comms_wake);
        eeprom_write(wake_secs, wake_secs);
        eeprom_write(fridge_setpoint, fridge_setpoint);
        eeprom_write(fridge_difference, fridge_difference);
        eeprom_write(fridge_delay, fridge_delay);
        eeprom_write(overshoot_delay, overshoot_delay);
        eeprom_write(overshoot_factor, overshoot_factor);
        magic = EXPECT_MAGIC;
        eeprom_write(magic, magic);
    }

    return true;
}

static void
read_handler()
{
    if (strcmp_P(readbuf, PSTR("fetch")) == 0)
    {
        cmd_fetch();
    }
    else if (strcmp_P(readbuf, PSTR("clear")) == 0)
    {
        cmd_clear();
    }
    else if (strcmp_P(readbuf, PSTR("btoff")) == 0)
    {
        cmd_btoff();
    }
    else if (strcmp_P(readbuf, PSTR("measure")) == 0)
    {
        cmd_measure();
    }
    else if (strcmp_P(readbuf, PSTR("sensors")) == 0)
    {
        cmd_sensors();
    }
    else if (strcmp_P(readbuf, PSTR("get_params")) == 0)
    {
        cmd_get_params();
    }
    else if (strncmp_P(readbuf, PSTR("set_params "), 11) == 0)
    {
        cmd_set_params(&readbuf[11]);
    }
    else if (strcmp_P(readbuf, PSTR("awake")) == 0)
    {
        cmd_awake();
    }
    else if (strncmp_P(readbuf, PSTR("fridge_setpoint "), 16) == 0)
    {
        cmd_set_fridge_setpoint(&readbuf[16]);
    }
    else if (strncmp_P(readbuf, PSTR("fridge_diff "), 12) == 0)
    {
        cmd_set_fridge_difference(&readbuf[12]);
    }
    else if (strncmp_P(readbuf, PSTR("fridge_delay "), 13) == 0)
    {
        cmd_set_fridge_delay(&readbuf[13]);
    }
    else if (strncmp_P(readbuf, PSTR("overshoot_delay "), 16) == 0)
    {
        cmd_set_overshoot_delay(&readbuf[16]);
    }
    else if (strncmp_P(readbuf, PSTR("overshoot_factor "), 17) == 0)
    {
        cmd_set_overshoot_factor(&readbuf[17]);
    }
    else if (strcmp_P(readbuf, PSTR("reset")) == 0)
    {
        cmd_reset();
    }
    else
    {
        printf_P(PSTR("Bad command '%s'\n"), readbuf);
    }
}

ISR(INT0_vect)
{
    button_pressed = 1;
    blink();
    _delay_ms(100);
    blink();
}


ISR(USART_RX_vect)
{
    char c = UDR0;
#ifdef HAVE_UART_ECHO
    uart_putchar(c, NULL);
#endif
    if (c == '\r' || c == '\n')
    {
        if (readpos > 0)
        {
            readbuf[readpos] = '\0';
            have_cmd = 1;
            readpos = 0;
        }
    }
    else
    {
        readbuf[readpos] = c;
        readpos++;
        if (readpos >= sizeof(readbuf))
        {
            readpos = 0;
        }
    }
}

ISR(TIMER2_COMPA_vect)
{
    TCNT2 = 0;

    clock_epoch += TICK;

    // watchdogs count up, continuous
    if (watchdog_long_limit > 0) {
        watchdog_count += TICK;
        if (watchdog_long_count >= watchdog_long_limit)
        {
            watchdog_long_count = 0;
            watchdog_long_hit = 1;
        }
    }

    if (watchdog_short_limit > 0) {
        watchdog_count += TICK;
        if (watchdog_short_count >= watchdog_short_limit)
        {
            watchdog_short_count = 0;
            watchdog_short_hit = 1;
        }
    }

    // newboot counts down, oneshot.
    if (newboot_count > 0)
    {
        newboot_count--;
        if (newboot_count == 0)
        {
            newboot_hit = 1;
        }
    }

}

static void
deep_sleep()
{
    // p119 of manual
    OCR2A = SLEEP_COMPARE;
    loop_until_bit_is_clear(ASSR, OCR2AUB);

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_mode();
}

static void
idle_sleep()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
}

static uint16_t
adc_vcc()
{
    PRR &= ~_BV(PRADC);
    
    // /16 prescaler
    ADCSRA = _BV(ADEN) | _BV(ADPS2);

    // set to measure 1.1 reference
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    // average a number of samples
    uint16_t sum = 0;
    uint8_t num = 0;
    for (uint8_t n = 0; n < 20; n++)
    {
        ADCSRA |= _BV(ADSC);
        loop_until_bit_is_clear(ADCSRA, ADSC);

        uint8_t low_11 = ADCL;
        uint8_t high_11 = ADCH;
        uint16_t val = low_11 + (high_11 << 8);

        if (n >= 4)
        {
            sum += val;
            num++;
        }
    }
    ADCSRA = 0;
    PRR |= _BV(PRADC);

    //float res_volts = 1.1 * 1024 * num / sum;
    //return 1000 * res_volts;
    return ((uint32_t)1100*1024*num) / sum;
}

static void
do_comms()
{
    // avoid receiving rubbish, perhaps
    uart_on();
    
    // write sd card here? same 3.3v regulator...
    
    while (1)
    {
        wdt_reset();
        if (have_cmd)
        {
            have_cmd = 0;
            read_handler();
            continue;
        }

        // wait for commands from the master
        idle_sleep();
    }
}

static void
blink()
{
    PORT_LED &= ~_BV(PIN_LED);
    _delay_ms(1);
    PORT_LED |= _BV(PIN_LED);
}

static void
long_delay(int ms)
{
    int iter = ms / 100;

    for (int i = 0; i < iter; i++)
    {
        _delay_ms(100);
    }
}

ISR(BADISR_vect)
{
    //uart_on();
    printf_P(PSTR("Bad interrupt\n"));
}

int main(void)
{
    setup_chip();
    blink();

    stdout = &mystdout;
    uart_on();

    printf(PSTR("Started.\n"));

    load_params();

    setup_tick_counter();

    sei();

    // doesn't return
    do_comms();

    return 0;   /* never reached */
}

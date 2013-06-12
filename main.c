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

#include "hmac-sha1.h"
#include "aes.h"

//#include "simple_ds18b20.h"
//#include "onewire.h"

LOCKBITS = (LB_MODE_3 & BLB0_MODE_4 & BLB1_MODE_4);

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// TICK should be 8 or less (8 untested). all timers need
// to be a multiple.

#define TICK 1
#define SLEEP_COMPARE (2000000/64)   // == 31250 exactly
#define NKEYS 10
#define HMACLEN 20
#define AESLEN 16
#define KEYLEN HMACLEN

#define BAUD 19200
#define UBRR ((F_CPU)/8/(BAUD)-1)

#define PORT_PI_BOOT PORTD
#define DDR_PI_BOOT DDRD
#define PIN_PI_BOOT PD5

#define PORT_PI_RESET PORTD
#define DDR_PI_RESET DDRD
#define PIN_PI_RESET PD6

#define PORT_LED PORTD
#define DDR_LED DDRD
#define PIN_LED PD7

// #define HAVE_UART_ECHO

// stores a value of clock_epoch combined with the remainder of TCNT1,
// for 1/32 second accuracy
struct epoch_ticks
{
    uint32_t ticks;
    // remainder
    uint8_t rem;
};

// eeprom-settable parameters, default values defined here. 
// all timeouts should be a multiple of TICK
static uint32_t watchdog_long_limit = (60L*60*24);
static uint32_t watchdog_short_limit = 0;
static uint32_t newboot_limit = 60*10;

// avr proves itself
static uint8_t avr_keys[NKEYS][KEYLEN] = {{0}};

// ---- Atomic guards required accessing these variables
// clock_epoch in seconds
static uint32_t clock_epoch;
// watchdog counts up
static uint32_t watchdog_long_count;
static uint32_t watchdog_short_count;
// newboot counts down
static uint32_t newboot_count;
// oneshot counts down
static uint32_t oneshot_count;

// ---- End atomic guards required

// boolean flags
static uint8_t watchdog_long_hit;
static uint8_t watchdog_short_hit;
static uint8_t newboot_hit;
static uint8_t oneshot_hit;

static uint8_t readpos;
static char readbuf[50];
static uint8_t have_cmd;

int uart_putchar(char c, FILE *stream);
static void long_delay(int ms);
static void blink();
static uint16_t adc_vcc();
static void set_pi_boot_normal(uint8_t normal);

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

    uint8_t avr_keys[NKEYS][KEYLEN];

    uint16_t magic;
};

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
    CLKPR = _BV(CLKPS1);

    // enable pullups
    // XXX matt pihelp
    //PORTB = 0xff; // XXX change when using SPI
    //PORTD = 0xff;
    //PORTC = 0xff;

    // 3.3v power for bluetooth and SD
    DDR_LED |= _BV(PIN_LED);

#if 0
    // set pullup
    PORTD |= _BV(PD2);
    // INT0 setup
    EICRA = (1<<ISC01); // falling edge - data sheet says it won't work?
    EIMSK = _BV(INT0);
#endif

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
        t->rem = TCNT1;
    }
}

static void
setup_tick_counter()
{
    // set up counter1

    // set up counter2. 
    // COM21 COM20 Set OC2 on Compare Match (p116)
    // WGM21 Clear counter on compare
    //TCCR2A = _BV(COM2A1) | _BV(COM2A0) | _BV(WGM21);
    // toggle on match
    TCCR1A = _BV(COM1A0);
#ifdef SIM_DEBUG
    // systemclock/8
    TCCR1B = _BV(CS11);
#else
    // systemclock/64
    TCCR1B = _BV(CS11) | _BV(CS10);
#endif
    TCNT1 = 0;
    OCR1A = SLEEP_COMPARE;
    // interrupt
    TIMSK1 = _BV(OCIE1A);
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
}

static void 
uart_off()
{
    // Turn off interrupts and disable tx/rx
    UCSR0B = 0;

    // Power reduction register
    PRR |= _BV(PRUSART0);
}

int 
uart_putchar(char c, FILE *stream)
{
    // XXX could perhaps sleep in the loop for power.
    if (c == '\n')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\r';
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    if (c == '\r')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\n';
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
cmd_newboot()
{
    set_pi_boot_normal(1);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        newboot_count = newboot_limit;
    }
    printf_P(PSTR("newboot for %d secs"), newboot_limit);
}



static void
cmd_get_params()
{
    uint32_t cur_watchdog_long, cur_watchdog_short, cur_newboot, cur_oneshot;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        cur_watchdog_long = watchdog_long_count;
        cur_watchdog_short = watchdog_short_count;
        cur_newboot = newboot_count;
        cur_oneshot = oneshot_count;
    }

    printf_P(PSTR("limit (count) : watchdog_long %lu (%lu) watchdog_short %lu (%lu) newboot %lu (%lu) oneshot (%lu)\n"),
        watchdog_long_limit,
        cur_watchdog_long,
        watchdog_short_limit,
        cur_watchdog_short,
        newboot_limit,
        cur_newboot,
        cur_oneshot);
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
printhex_nibble(const unsigned char b, FILE *stream)
{
    unsigned char  c = b & 0x0f;
    if ( c > 9 ) { 
        c += 'A'-10; 
    }
    else {
        c += '0';
    }
    fputc(c, stream);
}

void 
printhex_byte(const unsigned char b, FILE *stream)
{
    printhex_nibble( b >> 4, stream);
    printhex_nibble( b, stream);
}

void
printhex(uint8_t *id, uint8_t n, FILE *stream)
{
    for (uint8_t i = 0; i < n; i++)
    {
        if (i > 0)
        {
            fputc(' ', stream);
        }
        printhex_byte(id[i], stream);
    }
}

static int8_t
parse_key(const char *params, uint8_t *key_index, uint8_t *bytes,
    uint8_t bytes_len)
{
    // "N HEXKEY"
    if (strlen(params) != bytes_len*2 + 2) {
        printf_P(PSTR("Wrong length key\n"));
        return -1;
    }

    if (params[1] != ' ')
    {
        printf_P(PSTR("Missing space\n"));
        return -1;
    }

    *key_index = from_hex(params[0]);
    if (*key_index >= NKEYS)
    {
        printf_P(PSTR("Bad key index %d, max %d\n"), *key_index, NKEYS);
        return -1;
    }

    for (int i = 0, p = 0; i < bytes_len; i++, p += 2)
    {
        bytes[i] = (from_hex(params[p+2]) << 4) | from_hex(params[p+3]);
    }
    return 0;
}

static void
cmd_set_avr_key(const char *params)
{
    uint8_t new_key[KEYLEN];
    uint8_t key_index;
    if (parse_key(params, &key_index, new_key, sizeof(new_key)) != 0)
    {
        return;
    }
    memcpy(avr_keys[key_index], new_key, sizeof(new_key));
#ifndef SIM_DEBUG
    eeprom_write(avr_keys, avr_keys);
#endif
}

static void
cmd_hmac(const char *params)
{
    uint8_t indata[2+HMACLEN] = {'H', ':'};
    uint8_t outdata[HMACLEN];
    uint8_t key_index;
    if (parse_key(params, &key_index, &indata[2], HMACLEN) != 0)
    {
        printf_P(PSTR("FAIL: Bad input\n"));
        return;
    }

#ifndef SIM_DEBUG
    long_delay(200);
#endif

    hmac_sha1(outdata, avr_keys[key_index], KEYLEN*8, indata, sizeof(indata)*8);
    printf_P(PSTR("HMAC: "));
    printhex(outdata, HMACLEN, stdout);
    fputc('\n', stdout);
}

static void
cmd_decrypt(const char *params)
{
    uint8_t indata[HMACLEN+AESLEN]; // XXX
    // a temporary buffer
    uint8_t output[HMACLEN] = {'D', ':'};
    _Static_assert(AESLEN+2 <= sizeof(output), "sufficient output buffer");
    uint8_t key_index;
    if (parse_key(params, &key_index, indata, sizeof(indata)) != 0)
    {
        printf_P(PSTR("FAIL: Bad input\n"));
        return;
    }

#ifndef SIM_DEBUG
    long_delay(200);
#endif

    // check the signature
    memcpy(&output[2], &indata[HMACLEN], AESLEN);
    hmac_sha1(output, avr_keys[key_index+1], KEYLEN*8, output, (2+AESLEN)*8);

    if (memcmp(output, indata, HMACLEN) != 0) {
        printf_P(PSTR("FAIL: hmac mismatch\n"));
    }

    uint8_t tmpbuf[256];
    aesInit(avr_keys[key_index], tmpbuf);
    aesDecrypt(&indata[HMACLEN], NULL);

    printf_P(PSTR("DECRYPTED: "));
    printhex(output, AESLEN, stdout);
    fputc('\n', stdout);
}

static void
cmd_oneshot_reboot(const char *params)
{
    uint32_t new_delay = strtoul(params, NULL, 10);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        oneshot_count = new_delay;
    }
    printf_P(PSTR("oneshot delay %lu\n"), new_delay);
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
        eeprom_read(newboot_limit, newboot_limit);
   }

   eeprom_read(avr_keys, avr_keys);
}

static void
cmd_vcc()
{
    uint16_t vcc = adc_vcc();
    printf_P("vcc: %u mV\n", vcc);
}

static void
read_handler()
{
    if (strcmp_P(readbuf, PSTR("get_params")) == 0)
    {
        cmd_get_params();
    }
    else if (strncmp_P(readbuf, PSTR("set_params "), 11) == 0)
    {
        cmd_set_params(&readbuf[11]);
    }
    else if (strncmp_P(readbuf, PSTR("set_key "), 8) == 0)
    {
        cmd_set_avr_key(&readbuf[8]);
    }
    else if (strncmp_P(readbuf, PSTR("oneshot "), 8) == 0)
    {
        cmd_oneshot_reboot(&readbuf[8]);
    }
    else if (strncmp_P(readbuf, PSTR("hmac "), 5) == 0)
    {
        cmd_hmac(&readbuf[5]);
    }
    else if (strncmp_P(readbuf, PSTR("decrypt "), 8) == 0)
    {
        cmd_hmac(&readbuf[8]);
    }
    else if (strcmp_P(readbuf, PSTR("vcc")) == 0)
    {
        cmd_vcc();
    }
    else if (strcmp_P(readbuf, PSTR("reset")) == 0)
    {
        cmd_reset();
    }
    else if (strcmp_P(readbuf, PSTR("newboot")) == 0)
    {
        cmd_newboot();
    }
    else
    {
        printf_P(PSTR("Bad command '%s'\n"), readbuf);
    }
}

ISR(INT0_vect)
{
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

ISR(TIMER1_COMPA_vect)
{
    TCNT1 = 0;

    clock_epoch += TICK;

    // watchdogs count up, continuous
    if (watchdog_long_limit > 0) {
        watchdog_long_count += TICK;
        if (watchdog_long_count >= watchdog_long_limit)
        {
            watchdog_long_count = 0;
            watchdog_long_hit = 1;
        }
    }

    if (watchdog_short_limit > 0) {
        watchdog_short_count += TICK;
        if (watchdog_short_count >= watchdog_short_limit)
        {
            watchdog_short_count = 0;
            watchdog_short_hit = 1;
        }
    }

    // newboot counts down
    if (newboot_count > 0)
    {
        newboot_count-=TICK;
        if (newboot_count <= 0)
        {
            newboot_hit = 1;
            newboot_count = 0;
        }
    }

    if (oneshot_count > 0)
    {
        oneshot_count-=TICK;
        if (oneshot_count <= 0)
        {
            oneshot_hit = 1;
            oneshot_count = 0;
        }
    }
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
reboot_pi()
{
    // pull it low for 30ms
    PORT_PI_RESET &= ~_BV(PIN_PI_RESET);
    DDR_PI_RESET |= _BV(PIN_PI_RESET);
    _delay_ms(30);
    DDR_PI_RESET &= ~_BV(PIN_PI_RESET);
}

static void
set_pi_boot_normal(uint8_t normal) 
{
    PORT_PI_BOOT &= ~_BV(PIN_PI_BOOT);
    if (normal) 
    {
        // tristate
        DDR_PI_BOOT &= ~_BV(PIN_PI_BOOT);
    }
    else
    {
        // pull it low
        DDR_PI_RESET |= _BV(PIN_PI_BOOT);

    }
}

static void
check_flags()
{
    if (watchdog_long_hit 
        || watchdog_short_hit
        || oneshot_hit)
    {
        reboot_pi();
    }

    if (newboot_hit) {
        set_pi_boot_normal(0);
    }

    watchdog_long_hit = 0;
    watchdog_short_hit = 0;
    newboot_hit = 0;
    oneshot_hit = 0;
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

        check_flags();

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

    printf(PSTR("Pi Watchdog\nMatt Johnston matt@ucc.asn.au"));

    set_pi_boot_normal(0);

    load_params();

    setup_tick_counter();

    sei();

#if 0
    // encryption test
    cmd_set_avr_key("1 6161626263636464656566666767686800000000");
    cmd_set_avr_key("2 7979757569696f6f646465656666717164646969");
    cmd_decrypt("1 ecd858ee07a8e16575723513d2d072a7565865e40ba302059bfc650d4491268448102119");
#endif

    // doesn't return
    do_comms();

    return 0;   /* never reached */
}

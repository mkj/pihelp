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

#include "fat.h"
#include "fat_config.h"
#include "partition.h"
#include "sd_raw.h"
#include "sd_raw_config.h"

//#include "simple_ds18b20.h"
//#include "onewire.h"

// not set via bootloader
//LOCKBITS = (LB_MODE_3 & BLB0_MODE_4 & BLB1_MODE_4);

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// TICK should be 8 or less (8 untested). all timers need
// to be a multiple.

#define TICK 1
#define SLEEP_COMPARE (F_CPU/256)   // == 19200 for 4915200mhz
#define NKEYS 10
#define HMACLEN 20
#define AESLEN 16
#define KEYLEN HMACLEN

#define BAUD 115200
#define UBRR ((F_CPU)/(16*(BAUD))-1)

#define PORT_PI_BOOT PORTD
#define DDR_PI_BOOT DDRD
#define PIN_PI_BOOT PD5

#define PORT_PI_RESET PORTD
#define DDR_PI_RESET DDRD
#define PIN_PI_RESET PD6

#define PORT_PI_WARNING PORTD
#define DDR_PI_WARNING DDRD
#define PIN_PI_WARNING PD7

// #define HAVE_UART_ECHO

// stores a value of clock_epoch combined with the remainder of TCNT1,
// for 1/32 second accuracy
struct epoch_ticks
{
    uint32_t ticks;
    // remainder
    uint16_t rem;
};

#define WATCHDOG_LONG_MIN (60L*40) // 40 mins
#define WATCHDOG_LONG_MAX (60L*60*72) // 72 hours
#define WATCHDOG_LONG_DEFAULT (60L*60*6) // 6 hours

#define WATCHDOG_SHORT_MIN (60L*15) // 15 mins

#define NEWBOOT_DEFAULT (60*10) // 10 minutes
#define NEWBOOT_MIN (60*2) // 2 minutes
#define NEWBOOT_MAX (60*30) // 30 mins

#define WARNING_TIME 10

// eeprom-settable parameters, default values defined here. 
// all timeouts should be a multiple of TICK
static uint32_t watchdog_long_limit = WATCHDOG_LONG_DEFAULT;
static uint32_t watchdog_short_limit = 0;
static uint32_t newboot_limit = NEWBOOT_DEFAULT;

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
// countdown after the warning.
static uint8_t reboot_count;
// set by adc completion interrupt
static uint8_t adc_done;

// ---- End atomic guards required

// boolean flags
static uint8_t watchdog_long_hit;
static uint8_t watchdog_short_hit;
static uint8_t newboot_hit;
static uint8_t oneshot_hit;
static uint8_t reboot_hit;

// informational for status messages
static uint8_t boot_normal_status;

// flips between 0 and 1 each watchdog_long_hit, so eventually a
// working firmware should boot. set back to 0 for each 'alive'
// command
static uint8_t long_reboot_mode = 0;

static uint8_t readpos;
static char readbuf[150];
static uint8_t have_cmd;

int uart_putchar(char c, FILE *stream);
static void long_delay(uint16_t ms);
static void blink();
static uint16_t adc_vcc();
static uint16_t adc_5v(uint16_t vcc);
static uint16_t adc_temp();
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

    // set to 8 seconds, in case sha1 is slow etc.
    wdt_enable(WDTO_8S);

    // Set scaler to /1, -> clock to 8mhz
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;

    // enable pullups
    // XXX matt pihelp
    //PORTB = 0xff; // XXX change when using SPI
    //PORTD = 0xff;
    //PORTC = 0xff;

    DDR_PI_WARNING |= _BV(PIN_PI_WARNING);

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
    // systemclock/256
    TCCR1B = _BV(CS12);
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
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
    //8N1
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

#ifdef SIM_DEBUG
static char sim_out[140];
static uint8_t sim_idx = 0;
static uint8_t last_sim_idx = 0;
#endif

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
#ifdef SIM_DEBUG
    sim_out[sim_idx] = c;
    sim_idx++;
	sim_idx %= sizeof(sim_out);
#endif
    if (c == '\r')
    {
        loop_until_bit_is_set(UCSR0A, UDRE0);
        UDR0 = '\n';
    }
    return (unsigned char)c;
}

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
    {
        if(strcmp(dir_entry->long_name, name) == 0)
        {
            fat_reset_dir(dd);
            return 1;
        }
    }

    return 0;
}

struct fat_file_struct* 
open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}

static void
hmac_file(const char* fn)
{
    sd_raw_init();
    struct partition_struct* partition = partition_open(sd_raw_read, sd_raw_read_interval, 0, 0, 0);
    struct fat_fs_struct* fs = fat_open(partition);
    struct fat_dir_entry_struct directory;
    fat_get_dir_entry_of_path(fs, "/", &directory);

    struct sd_raw_info disk_info;
    sd_raw_get_info(&disk_info);
    printf("diskinfo size %d", disk_info.capacity);

    struct fat_dir_struct* dd = fat_open_dir(fs, &directory);
    struct fat_file_struct* fd = open_file_in_dir(fs, dd, "fn");

    char c = 0;
    char buf[512];
    for (int i = 0; i < 10; i++)
    {
        fat_read_file(fd, buf, sizeof(buf));
        c ^= buf[0];
    }
    printf("total %d\n", c);
}

static void cmd_reset() __attribute__ ((noreturn));
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
    printf_P(PSTR("newboot for %d secs\n"), newboot_limit);
}

static void
cmd_oldboot()
{
    set_pi_boot_normal(0);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        newboot_count = 0;
    }
    printf_P(PSTR("back to old boot\n"));
}


static void
cmd_status()
{
    uint32_t cur_watchdog_long, cur_watchdog_short, cur_newboot, cur_oneshot;
    struct epoch_ticks t;

    get_epoch_ticks(&t);

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        cur_watchdog_long = watchdog_long_count;
        cur_watchdog_short = watchdog_short_count;
        cur_newboot = newboot_count;
        cur_oneshot = oneshot_count;
    }

    printf_P(PSTR("limit (current)\n"
        "watchdog_long %lu (%lu) watchdog newboot mode %d\n"
        "watchdog_short %lu (%lu)\n"
        "newboot %lu (%lu)\n"
        "oneshot (%lu)\n"
        "uptime %lu rem %u\n"
        "boot normal %hhu\n"
        ),
        watchdog_long_limit, cur_watchdog_long, long_reboot_mode,
        watchdog_short_limit, cur_watchdog_short,
        newboot_limit, cur_newboot,
        cur_oneshot,
        t.ticks, t.rem,
        boot_normal_status);
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
        printhex_byte(id[i], stream);
    }
}

static int8_t
parse_key(const char *params, uint8_t *key_index, uint8_t *bytes,
    uint8_t bytes_len)
{
    // "N HEXKEY"
    if (strlen(params) != bytes_len*2 + 2) {
        printf_P(PSTR("Wrong length key. wanted %d, got %d, '%s'\n"),
            bytes_len*2+2, strlen(params), params);
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
    printf_P(PSTR("Set key %d: "), key_index);
    printhex(new_key, sizeof(new_key), stdout);
    putchar('\n');
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
    putchar('\n');
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
    hmac_sha1(output, avr_keys[key_index], KEYLEN*8, output, (2+AESLEN)*8);

    if (memcmp(output, indata, HMACLEN) != 0) {
        printf_P(PSTR("FAIL: hmac mismatch\n"));
    }

    uint8_t tmpbuf[256];
    aesInit(avr_keys[key_index], tmpbuf);
    aesDecrypt(&indata[HMACLEN], NULL);

    printf_P(PSTR("DECRYPTED: "));
    printhex(output, AESLEN, stdout);
    putchar('\n');
}

static void
cmd_oneshot_reboot(const char *params)
{
    uint32_t new_delay = strtoul(params, NULL, 10);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        oneshot_count = new_delay;
    }
    printf_P(PSTR("oneshot new delay %lu\n"), new_delay);
}

static void
clamp_params()
{
    if (watchdog_long_limit < WATCHDOG_LONG_MIN 
        || watchdog_long_limit > WATCHDOG_LONG_MAX)
    {
        watchdog_long_limit = WATCHDOG_LONG_DEFAULT;
    }

    if (watchdog_short_limit != 0
        && watchdog_short_limit < WATCHDOG_SHORT_MIN)
    {
        watchdog_short_limit = 0;
    }

    if (newboot_limit < NEWBOOT_MIN || newboot_limit > NEWBOOT_MAX)
    {
        newboot_limit = NEWBOOT_DEFAULT;
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
        eeprom_read(newboot_limit, newboot_limit);
   }

   clamp_params();

   eeprom_read(avr_keys, avr_keys);
}

static void
cmd_alive()
{
    printf_P(PSTR("Ah, good.\n"));
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        watchdog_long_count = 0;
        watchdog_short_count = 0;
    }
    long_reboot_mode = 0;
}

static void
cmd_vcc()
{
    uint16_t vcc = adc_vcc();
    uint16_t v5 = adc_5v(vcc);
    uint16_t temp = adc_temp();
    // roughly?
    uint16_t temp_deg = temp - 290;
    printf_P(PSTR("vcc: %u mV\n"
                    "5v: %u mV\n"
                    "temp: %u mV (%dºC)\n"
                    ),
        vcc, v5, temp, temp_deg);
}


void(*bootloader)() __attribute__ ((noreturn)) = (void*)0x7800;

#ifndef PROG_PASSWORD
#define PROG_PASSWORD "Y2vvjxO5"
#endif

static void
cmd_prog(const char* arg)
{
    if (strcmp(arg, PROG_PASSWORD) != 0)
    {
        printf_P(PSTR("Bad prog password\n"));
        return;
    }

    // disable wdt
    wdt_disable();

    // disable interrupts
    TIMSK0 = 0;
    TIMSK1 = 0;
    TIMSK2 = 0;
    EIMSK = 0;
    PCMSK0 = 0;
    PCMSK1 = 0;
    PCMSK2 = 0;
    ACSR &= ~_BV(ACIE);
    ADCSRA &= ~_BV(ADIE);
    UCSR0B &= ~_BV(RXCIE0);
    UCSR0B &= _BV(TXCIE0);
    // doesn't do TWI, other uart, probably others

    _delay_ms(20);

    bootloader();
}


static void
adc_sleep()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
}

#define BITSET(v, n) (((v) >> (n)) & 1)

static inline uint8_t
popcnt(uint8_t v)
{
    return BITSET(v, 0)
        + BITSET(v, 1)
        + BITSET(v, 2)
        + BITSET(v, 3)
        + BITSET(v, 4)
        + BITSET(v, 5)
        + BITSET(v, 6)
        + BITSET(v, 7);
}

static uint8_t
adc_bit()
{
    ADCSRA |= _BV(ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    uint8_t low = ADCL;
    uint8_t high = ADCH;
    return (popcnt(low)&1) ^ (popcnt(high)&1);
}

static void
adc_random(uint8_t admux, 
    uint8_t *out, uint16_t num, uint32_t *tries)
{
    uint8_t ret = 0;
    uint8_t count = 0;

    PRR &= ~_BV(PRADC);
    // /16 prescaler for 691mhz, no interrupt
    ADCSRA = _BV(ADEN) | _BV(ADPS2);

    *tries = 0;
    for (int i = 0; i < num; i++)
    {
        while (count <= 7)
        {
            (*tries)++;

            // Von Neumann extractor
            uint8_t one = adc_bit();
            uint8_t two = adc_bit();
            if (one == two)
            {
                continue;
            }
            ret |= one << count;
            count++;
        }
        out[i] = ret;
    }
    ADCSRA = 0;
    PRR |= _BV(PRADC);
}

ISR(ADC_vect)
{
    adc_done = 1;
}

static void
adc_generic(uint8_t admux, uint8_t *ret_num, uint16_t *ret_sum)
{
    PRR &= ~_BV(PRADC);
    
    // /64 prescaler, interrupt
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADIE);

    // set to measure 1.1 reference
    ADMUX = admux;

    // delay after setting reference etc, allow settling
    long_delay(300);
    // average a number of samples
    uint16_t sum = 0;
    uint8_t num = 0;
    for (uint8_t n = 0; n < 20; n++)
    {
        while (1)
        {
            adc_done = 0;
            ADCSRA |= _BV(ADSC);
            adc_sleep();

            uint8_t done;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                done = adc_done;
            }
            if (done)
            {
                break;
            }
        }

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

    *ret_num = num;
    *ret_sum = sum;
}

static uint16_t
adc_vcc()
{
    const uint8_t mux = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    uint16_t sum;
    uint8_t num;

    adc_generic(mux, &num, &sum);

    //float res_volts = 1.1 * 1024 * num / sum;
    //return 1000 * res_volts;
    return ((uint32_t)1100*1024*num) / sum;
}

#define SCALER_5V 2

static uint16_t
adc_5v(uint16_t vcc)
{
    // set to measure ADC4 against AVCC
    const uint8_t mux = _BV(REFS0) | _BV(MUX2);
    uint16_t sum;
    uint8_t num;
    
    adc_generic(mux, &num, &sum);

    return ((uint32_t)vcc*sum*SCALER_5V/(num*1024));
}

static uint16_t
adc_temp()
{
    // set to measure temperature against 1.1v reference.
    const uint8_t mux = _BV(REFS0) | _BV(REFS1) | _BV(MUX3);
    uint16_t sum;
    uint8_t num;
    
    adc_generic(mux, &num, &sum);

    // return the voltage

    return ((uint32_t)1100*sum) / (num*1024);
}

static void
cmd_random(const char* params)
{
    uint8_t admux;
    uint16_t num;
    uint8_t buf[100];

    int ret = sscanf_P(params, PSTR("%hhu %u"),
        &admux, &num);
    if (!ret)
    {
        printf_P(PSTR("Bad arguments\n"));
        return;
    }
    uint32_t tries = 0;
    printf_P(PSTR("output: "));
    for (int i = 0; i < num; i+= sizeof(buf))
    {
        uint32_t t;
        uint16_t nr = MIN(num-i, sizeof(buf));
        adc_random(admux, buf, nr, &t);
        printhex(buf, nr, stdout);
        tries += t;
    }
    putchar('\n');
    printf_P(PSTR("%ld tries\n"), tries);
}



static void
read_handler()
{
#define LOCAL_PSTR(x) const static char x ## _str[] PROGMEM = #x;
#define LOCAL_HELP(x, d) const static char x ## _help[] PROGMEM = d;

    LOCAL_PSTR(set_params);
    LOCAL_PSTR(set_key);
    LOCAL_PSTR(oneshot);
    LOCAL_PSTR(hmac);
    LOCAL_PSTR(decrypt);
    LOCAL_PSTR(alive);
    LOCAL_PSTR(vcc);
    LOCAL_PSTR(reset);
    LOCAL_PSTR(newboot);
    LOCAL_PSTR(oldboot);
    LOCAL_PSTR(status);
    LOCAL_PSTR(random);
    LOCAL_PSTR(prog);
    LOCAL_HELP(set_params, "<long_limit> <short_limit> <newboot_limit>");
    LOCAL_HELP(set_key, "20_byte_hex>");
    LOCAL_HELP(oneshot, "<timeout>");
    LOCAL_HELP(prog, "<password>");
    LOCAL_HELP(random, "<admux> <nbytes>");
    LOCAL_HELP(hmac, "<key_index> <20_byte_hex_data>");
    LOCAL_HELP(decrypt, "<key_index> <20_byte_hmac|16_byte_aes_block>");

    static const struct handler {
        PGM_P name;
        void(*cmd)(const char *param);
        // existence of arg_help indicates if the cmd takes a parameter.
        PGM_P arg_help;
    } handlers[] PROGMEM = 
    {
        {alive_str, cmd_alive, NULL},
        {newboot_str, cmd_newboot, NULL},
        {oldboot_str, cmd_oldboot, NULL},
        {oneshot_str, cmd_oneshot_reboot, oneshot_help},
        {status_str, cmd_status, NULL},
        {hmac_str, cmd_hmac, hmac_help},
        {decrypt_str, cmd_decrypt, decrypt_help},
        {set_params_str, cmd_set_params, set_params_help},
        {set_key_str, cmd_set_avr_key, set_key_help},
        {random_str, cmd_random, random_help},
        {vcc_str, cmd_vcc, NULL},
        {reset_str, cmd_reset, NULL},
        {prog_str, cmd_prog, prog_help},
    };

    if (readbuf[0] == '\0')
    {
        return;
    }

    if (strcmp_P(readbuf, PSTR("help")) == 0)
    {
        printf_P(PSTR("Commands:---\n"));
        for (int i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++)
        {
            struct handler h;
            memcpy_P(&h, &handlers[i], sizeof(h));
            printf_P(h.name);
            if (h.arg_help)
            {
                putchar(' ');
                printf_P(h.arg_help);
            }
            putchar('\n');
        };
        printf_P(PSTR("---\n"));
		return;
    }

    for (int i = 0; i < sizeof(handlers) / sizeof(handlers[0]); i++)
    {
        struct handler h;
        memcpy_P(&h, &handlers[i], sizeof(h));

        const int h_len = strlen_P(h.name);
        if (strncmp_P(readbuf, h.name, h_len) == 0)
        {
            if (h.arg_help)
            {
                if (readbuf[h_len] == ' ')
                {
                    h.cmd(&readbuf[h_len+1]);
                    return;
                }
            }
            else 
            {
                if (readbuf[h_len] == '\0')
                {
                    void(*void_cmd)() = h.cmd;
                    void_cmd();
                    return;
                }
            }
        }
    }

    printf_P(PSTR("Bad command '%s'\n"), readbuf);
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

    if (reboot_count > 0)
    {
        reboot_count -= TICK;
        if (reboot_count <= 0)
        {
            reboot_hit = 1;
            reboot_count = 0;
        }
    }
}

static void
idle_sleep()
{
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
}

static void
reboot_pi()
{
    printf_P(PSTR("Real reboot now\n"));
    // pull it low for 200ms
    PORT_PI_RESET &= ~_BV(PIN_PI_RESET);
    DDR_PI_RESET |= _BV(PIN_PI_RESET);
    _delay_ms(200);
	
	PORT_PI_WARNING &= ~_BV(PIN_PI_WARNING);
    DDR_PI_RESET &= ~_BV(PIN_PI_RESET);	
}

static void
wait_reboot_pi()
{
	PORT_PI_WARNING |= _BV(PIN_PI_WARNING);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        reboot_count = WARNING_TIME;
    }
    printf_P(PSTR("Rebooting in %hhu seconds\n"), reboot_count);
}

static void
set_pi_boot_normal(uint8_t normal) 
{
    boot_normal_status = normal;
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
    if (watchdog_long_hit)
    {
        // alternate between booting normal and emergency
        if (long_reboot_mode)
        {
            cmd_newboot();
        }
        long_reboot_mode ^= 1;
    }

    if (watchdog_long_hit 
        || watchdog_short_hit
        || oneshot_hit)
    {
		printf_P(PSTR("Rebooting! long %d, short %d, oneshot %d\n"),
			watchdog_long_hit, watchdog_short_hit, oneshot_hit);
        wait_reboot_pi();
    }

    if (newboot_hit) {
        set_pi_boot_normal(0);
    }

    if (reboot_hit) {
        reboot_pi();
    }

    watchdog_long_hit = 0;
    watchdog_short_hit = 0;
    newboot_hit = 0;
    oneshot_hit = 0;
    reboot_hit = 0;
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

#ifdef SIM_DEBUG
        if (sim_idx != last_sim_idx)
        {
            last_sim_idx = sim_idx;
        }
#endif

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
    #if 0
    PORT_ &= ~_BV(PIN_LED);
    _delay_ms(1);
    PORT_LED |= _BV(PIN_LED);
    #endif
}

static void
long_delay(uint16_t ms)
{
    uint16_t iter = ms / 10;

    for (uint16_t i = 0; i < iter; i++)
    {
        _delay_ms(10);
    }
}

ISR(BADISR_vect)
{
    //uart_on();
    printf_P(PSTR("Bad interrupt\n"));
}

// disable watchdog on boot
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
    MCUSR = 0;
    wdt_disable();
}

int main(void)
{
    _Static_assert(F_CPU % 256 == 0, "clock prescaler remainder 0");
    _Static_assert(NEWBOOT_MAX < WATCHDOG_LONG_MIN, "newboot max shorter than watchdog min");
    _Static_assert((F_CPU)%(16*(BAUD)) == 0, "baud rate good multiple");

    setup_chip();
    blink();

    stdout = &mystdout;
    uart_on();

    long_delay(500);
    printf_P(PSTR("Pi Watchdog\nMatt Johnston matt@ucc.asn.au\n"));

    set_pi_boot_normal(0);

    load_params();

    setup_tick_counter();

    sei();

#if 0
    // encryption test
    cmd_set_avr_key("1 6161626263636464656566666767686800000000");
    cmd_set_avr_key("2 7979757569696f6f646465656666717164646969");
    //cmd_decrypt("1 ecd858ee07a8e16575723513d2d072a7565865e40ba302059bfc650d4491268448102119");
	cmd_decrypt("1 5a587b50fd48688bbda1b510cf9a3fab6fd4737b" "0ba302059bfc650d4491268448102119");
	cmd_hmac("2 7979757569696f6f646465656666717164646969");
#endif

    // doesn't return
    do_comms();

    return 0;   /* never reached */
}

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

#include "simple_ds18b20.h"
#include "onewire.h"

// configuration params
// - measurement interval
// - transmit interval
// - bluetooth params
// - number of sensors (and range?)

#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

// TICK should be 8 or less (8 untested). all timers need
// to be a multiple.

#define TICK 6
// we have 1024 prescaler, 32768 crystal.
#define SLEEP_COMPARE (32*TICK-1)

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

// eeprom-settable parameters. all timeouts should
// be a multiple of TICK (6 seconds probably)
static uint16_t measure_wake = 61; // not a divisor of comms_wake
static uint16_t comms_wake = 600;
static uint8_t wake_secs = 30;
// decidegrees
static int16_t fridge_setpoint = 180; // 18.0ºC
static uint16_t fridge_difference = 3; // 0.3ºC
static uint16_t fridge_delay = 600; // seconds

static uint16_t overshoot_delay = 720; // 12 mins
static uint8_t overshoot_factor = 10; // 1.0ºC

// ---- Atomic guards required accessing these variables
// clock_epoch in seconds
static uint32_t clock_epoch;
static uint16_t comms_count;
static uint16_t measure_count;
// ---- End atomic guards required

static uint16_t n_measurements;

// calculated at startup as TOTAL_MEASUREMENTS/n_sensors
static uint16_t max_measurements;

static uint16_t measurements[TOTAL_MEASUREMENTS];

static struct epoch_ticks first_measurement_clock;
// last_measurement_clock is redundant but checks that we're not missing
// samples
static struct epoch_ticks last_measurement_clock;
static struct epoch_ticks last_comms_clock;

// boolean flags
static uint8_t need_measurement;
static uint8_t need_comms;
static uint8_t uart_enabled;
static uint8_t stay_awake;
static uint8_t button_pressed;

// counts down from WAKE_SECS to 0, goes to deep sleep when hits 0
static uint8_t comms_timeout;

static uint8_t readpos;
static char readbuf[30];
static uint8_t have_cmd;

static uint8_t n_sensors;
static uint8_t sensor_id[MAX_SENSORS][ID_LEN];

static int16_t last_fridge = DS18X20_INVALID_DECICELSIUS;
static int16_t last_wort = DS18X20_INVALID_DECICELSIUS;
static struct epoch_ticks fridge_off_clock = {0};
static struct epoch_ticks fridge_on_clock = {0};
static struct epoch_ticks wort_valid_clock = {0};

int uart_putchar(char c, FILE *stream);
static void long_delay(int ms);
static void blink();
static uint16_t adc_vcc();

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
        _FDEV_SETUP_WRITE);

static uint16_t crc_out;
static FILE _crc_stdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
        _FDEV_SETUP_WRITE);
// convenience
static FILE *crc_stdout = &_crc_stdout;


// thanks to http://projectgus.com/2010/07/eeprom-access-with-arduino/
#define eeprom_read_to(dst_p, eeprom_field, dst_size) eeprom_read_block((dst_p), (void *)offsetof(struct __eeprom_data, eeprom_field), (dst_size))
#define eeprom_read(dst, eeprom_field) eeprom_read_to((&dst), eeprom_field, sizeof(dst))
#define eeprom_write_from(src_p, eeprom_field, src_size) eeprom_write_block((src_p), (void *)offsetof(struct __eeprom_data, eeprom_field), (src_size))
#define eeprom_write(src, eeprom_field) { eeprom_write_from(&src, eeprom_field, sizeof(src)); }

#define EXPECT_MAGIC 0x67c9

struct __attribute__ ((__packed__)) __eeprom_data {
    uint16_t measure_wake;
    uint16_t comms_wake;
    uint8_t wake_secs;

    int16_t fridge_setpoint; // decidegrees
    uint16_t fridge_difference; // decidegrees
    uint16_t fridge_delay;

    uint16_t overshoot_delay;
    uint8_t overshoot_factor; // decidegrees

#if 0
    static uint8_t wort_id[ID_LEN];
    static uint8_t fridge_id[ID_LEN];
#endif

    uint16_t magic;
};

static const uint8_t fridge_id[ID_LEN] = 
    {0x28,0xCE,0xB2,0x1A,0x03,0x00,0x00,0x99};
static const uint8_t wort_id[ID_LEN] = 
    {0x28,0x49,0xBC,0x1A,0x03,0x00,0x00,0x54};

static void deep_sleep();

// 0 or 1
static uint8_t
is_fridge_on()
{
    if (PORT_FRIDGE & _BV(PIN_FRIDGE))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

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

    // Set clock to 2mhz
    CLKPR = _BV(CLKPCE);
    // divide by 4
    CLKPR = _BV(CLKPS1);

    // enable pullups
    PORTB = 0xff; // XXX change when using SPI
    PORTD = 0xff;
    PORTC = 0xff;

    // 3.3v power for bluetooth and SD
    DDR_LED |= _BV(PIN_LED);
    DDR_SHDN |= _BV(PIN_SHDN);

    PORT_FRIDGE &= ~_BV(PIN_FRIDGE);
    DDR_FRIDGE |= _BV(PIN_FRIDGE);

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
set_aux_power(uint8_t on)
{
    if (on)
    {
        PORT_SHDN &= ~_BV(PIN_SHDN);
    }
    else
    {
        PORT_SHDN |= _BV(PIN_SHDN);
    }
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
set_measurement(uint8_t sensor, uint16_t measurement, uint16_t reading)
{
    measurements[sensor*max_measurements + measurement] = reading;
}

static uint16_t 
get_measurement(uint8_t sensor, uint16_t measurement)
{
    return measurements[sensor*max_measurements + measurement];
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
cmd_fetch()
{
    crc_out = 0;

    fprintf_P(crc_stdout, PSTR("START\n"));
    {
        struct epoch_ticks now;
        get_epoch_ticks(&now);
        fprintf_P(crc_stdout, PSTR("now=%lu\n"), now.ticks);
        fprintf_P(crc_stdout, PSTR("now_rem=%hhu\n"), now.rem);
    }
    fprintf_P(crc_stdout, PSTR("time_step=%hu\n"), measure_wake);
    fprintf_P(crc_stdout, PSTR("first_time=%lu\n"), first_measurement_clock.ticks);
    fprintf_P(crc_stdout, PSTR("first_time_rem=%hhu\n"), first_measurement_clock.rem);
    fprintf_P(crc_stdout, PSTR("last_time=%lu\n"),  last_measurement_clock.ticks);
    fprintf_P(crc_stdout, PSTR("last_time_rem=%hhu\n"),  last_measurement_clock.rem);
    fprintf_P(crc_stdout, PSTR("comms_time=%lu\n"), last_comms_clock.ticks);
    fprintf_P(crc_stdout, PSTR("comms_time_rem=%hhu\n"), last_comms_clock.rem);
    fprintf_P(crc_stdout, PSTR("voltage=%hu\n"), adc_vcc());
    fprintf_P(crc_stdout, PSTR("measure=%hu\n"), measure_wake);
    fprintf_P(crc_stdout, PSTR("comms=%hu\n"), comms_wake);
    fprintf_P(crc_stdout, PSTR("wake=%hhu\n"), wake_secs);
    fprintf_P(crc_stdout, PSTR("fridge=%.1f\n"), fridge_setpoint/10.0);
    fprintf_P(crc_stdout, PSTR("fridge_diff=%.1f\n"), fridge_difference/10.0);
    fprintf_P(crc_stdout, PSTR("fridge_delay=%hu\n"), fridge_delay);
    fprintf_P(crc_stdout, PSTR("overshoot_factor=%.1f\n"), overshoot_factor/10.0);
    fprintf_P(crc_stdout, PSTR("overshoot_delay=%hu\n"), overshoot_delay);
    fprintf_P(crc_stdout, PSTR("fridge_status=%hhu\n"), is_fridge_on());
    fprintf_P(crc_stdout, PSTR("fridge_last_on=%lu\n"), fridge_on_clock.ticks);
    fprintf_P(crc_stdout, PSTR("fridge_last_off=%lu\n"), fridge_off_clock.ticks);
    fprintf_P(crc_stdout, PSTR("last_fridge=%hu\n"), last_fridge);
    fprintf_P(crc_stdout, PSTR("last_wort=%hu\n"), last_wort);
    fprintf_P(crc_stdout, PSTR("tick_secs=%d\n"), TICK);
    fprintf_P(crc_stdout, PSTR("tick_wake=%d\n"), SLEEP_COMPARE);
    fprintf_P(crc_stdout, PSTR("maxsens=%hhu\n"), MAX_SENSORS);
    fprintf_P(crc_stdout, PSTR("totalmeas=%hu\n"), TOTAL_MEASUREMENTS);
    fprintf_P(crc_stdout, PSTR("sensors=%hhu\n"), n_sensors);
    for (uint8_t s = 0; s < n_sensors; s++)
    {
        fprintf_P(crc_stdout, PSTR("sensor_id%hhu="), s);
        printhex(sensor_id[s], ID_LEN, crc_stdout);
        fputc('\n', crc_stdout);
    }
    fprintf_P(crc_stdout, PSTR("measurements=%hu\n"), n_measurements);
    for (uint16_t n = 0; n < n_measurements; n++)
    {
        fprintf_P(crc_stdout, PSTR("meas%hu="), n);
        for (uint8_t s = 0; s < n_sensors; s++)
        {
            fprintf_P(crc_stdout, PSTR(" %04hx"), get_measurement(s, n));
        }
        fputc('\n', crc_stdout);
    }
    fprintf_P(crc_stdout, PSTR("END\n"));
    fprintf_P(stdout, PSTR("CRC=%hu\n"), crc_out);
}

static void
cmd_clear()
{
    n_measurements = 0;
    printf_P(PSTR("cleared\n"));
}

static void
cmd_btoff()
{
    uint8_t rem;
    uint16_t count_copy;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        count_copy = comms_count;
        rem = TCNT2;
    }
    printf_P(PSTR("next_wake=%hu,"), comms_wake-count_copy);
    printf_P(PSTR("rem=%hhu,"), rem);
    printf_P(PSTR("tick_secs=%hhu,"), TICK);
    printf_P(PSTR("tick_wake=%hhu\n"), SLEEP_COMPARE);
    _delay_ms(100);
    comms_timeout = 0;
    stay_awake = 0;
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
cmd_measure()
{
    printf_P(PSTR("measuring\n"));
    need_measurement = 1;
}

static void
cmd_sensors()
{
    uint8_t ret = simple_ds18b20_start_meas(NULL);
    printf_P(PSTR("All sensors, ret %hhu, waiting...\n"), ret);
    long_delay(DS18B20_TCONV_12BIT);
    simple_ds18b20_read_all();
}

static void
init_sensors()
{
    uint8_t id[OW_ROMCODE_SIZE];
    printf_P(PSTR("init sensors\n"));
    ow_reset();
    for( uint8_t diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE; )
    {
        diff = ow_rom_search( diff, &id[0] );
        if( diff == OW_PRESENCE_ERR ) {
            printf_P( PSTR("No Sensor found\r") );
            return;
        }
        
        if( diff == OW_DATA_ERR ) {
            printf_P( PSTR("Bus Error\r") );
            return;
        }

        if (n_sensors < MAX_SENSORS)
        {
            memcpy(sensor_id[n_sensors], id, ID_LEN);
            printf_P(PSTR("Added sensor %hhu : "), n_sensors);
            printhex(id, ID_LEN, stdout);
            putchar('\n');
            n_sensors++;
        }
        else
        {
            printf_P(PSTR("Too many sensors\n"));
        }
    }

    max_measurements = TOTAL_MEASUREMENTS / n_sensors;
}

static void
load_params()
{
    uint16_t magic;
    eeprom_read(magic, magic);
    if (magic == EXPECT_MAGIC)
    {
        eeprom_read(measure_wake, measure_wake);
        eeprom_read(comms_wake, comms_wake);
        eeprom_read(wake_secs, wake_secs);
        eeprom_read(fridge_setpoint, fridge_setpoint);
        eeprom_read(fridge_difference, fridge_difference);
        eeprom_read(fridge_delay, fridge_delay);
        eeprom_read(overshoot_delay, overshoot_delay);
        eeprom_read(overshoot_factor, overshoot_factor);
    }
}

static void
cmd_get_params()
{
    printf_P(PSTR("measure %hu\n"), measure_wake);
    printf_P(PSTR("comms %hu\n"), comms_wake);
    printf_P(PSTR("wake %hhu\n"), wake_secs);
    printf_P(PSTR("tick %d\n"), TICK);
    printf_P(PSTR("fridge %.1fº\n"), fridge_setpoint / 10.0f);
    printf_P(PSTR("fridge difference %.1fº\n"), fridge_difference / 10.0f);
    printf_P(PSTR("fridge_delay %hu\n"), fridge_delay);
    printf_P(PSTR("overshoot factor %.1fº\n"), overshoot_factor / 10.0f);
    printf_P(PSTR("overshoot delay %hu\n"), overshoot_delay);
    printf_P(PSTR("sensors %hhu (%hhu)\n"), 
            n_sensors, MAX_SENSORS);
    printf_P(PSTR("meas %hu (%hu)\n"),
            max_measurements, TOTAL_MEASUREMENTS);
}

static void
cmd_set_params(const char *params)
{
    uint16_t new_measure_wake;
    uint16_t new_comms_wake;
    uint8_t new_wake_secs;
    int ret = sscanf_P(params, PSTR("%hu %hu %hhu"),
            &new_measure_wake, &new_comms_wake, &new_wake_secs);

    if (ret != 3)
    {
        printf_P(PSTR("Bad values\n"));
    }
    else
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            eeprom_write(new_measure_wake, measure_wake);
            eeprom_write(new_comms_wake, comms_wake);
            eeprom_write(new_wake_secs, wake_secs);
            uint16_t magic = EXPECT_MAGIC;
            eeprom_write(magic, magic);
        }
        printf_P(PSTR("set_params for next boot\n"));
        printf_P(PSTR("measure %hu comms %hu wake %hhu\n"),
                new_measure_wake, new_comms_wake, new_wake_secs);
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
cmd_set_fridge_setpoint(char *params)
{
    float new_f = atof(params);
    if (new_f < 2 || new_f > 30)
    {
        printf_P(PSTR("Bad fridge value %f\n"), new_f);
        return;
    }

    int16_t old_setpoint = fridge_setpoint;

    fridge_setpoint = new_f * 10;
    bool written = set_initial_eeprom();
    if (!written)
    {
        if (old_setpoint != fridge_setpoint)
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                eeprom_write(fridge_setpoint, fridge_setpoint);
            }
        }
    }
    printf_P(PSTR("old fridge %.1fº new fridge %.1fº\n"), 
            old_setpoint / 10.0f, fridge_setpoint / 10.0f);
}

static void
cmd_set_fridge_difference(char *params)
{
    float new_f = atof(params);
    if (new_f < 0 || new_f > 30)
    {
        printf_P(PSTR("Bad fridge value %f\n"), new_f);
        return;
    }

    fridge_difference = new_f * 10;
    bool written = set_initial_eeprom();
    if (!written)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            eeprom_write(fridge_difference, fridge_difference);
        }
    }
    printf_P(PSTR("new fridge difference %.1fº\n"), fridge_difference / 10.0f);
}

static void
cmd_set_fridge_delay(char *params)
{
    uint16_t new_delay = atoi(params);
    if (new_delay < 5)
    {
        printf_P(PSTR("Bad fridge delay %d\n"), new_delay);
        return;
    }

    fridge_delay = new_delay;
    bool written = set_initial_eeprom();
    if (!written)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            eeprom_write(fridge_delay, fridge_delay);
        }
    }
    printf_P(PSTR("new fridge delay %hu\n"), fridge_delay);
}

static void
cmd_set_overshoot_factor(char *params)
{
    float new_f = atof(params);
    if (new_f <= 0 || new_f > 20)
    {
        printf_P(PSTR("Bad overshoot factor %f\n"), new_f);
        return;
    }

    uint8_t old = overshoot_factor;

    overshoot_factor = new_f * 10;
    bool written = set_initial_eeprom();
    if (!written)
    {
        if (old != overshoot_factor)
        {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
            {
                eeprom_write(overshoot_factor, overshoot_factor);
            }
        }
    }
    printf_P(PSTR("old factor %.1fº new factor %.1fº\n"), 
            old / 10.0f, overshoot_factor / 10.0f);
}
    
static void
cmd_set_overshoot_delay(char *params)
{
    uint16_t new_delay = atoi(params);
    if (new_delay < 5)
    {
        printf_P(PSTR("Bad overshoot delay %d\n"), new_delay);
        return;
    }

    overshoot_delay = new_delay;
    bool written = set_initial_eeprom();
    if (!written)
    {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            eeprom_write(overshoot_delay, overshoot_delay);
        }
    }
    printf_P(PSTR("new overshoot delay %hu\n"), overshoot_delay);
}

static void
cmd_awake()
{
    stay_awake = 1;
    printf_P(PSTR("awake\n"));
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
    measure_count += TICK;
    comms_count += TICK;

    clock_epoch += TICK;

    if (comms_timeout != 0)
    {
        comms_timeout -= TICK;
    }

    if (measure_count >= measure_wake)
    {
        measure_count = 0;
        need_measurement = 1;
    }

    if (comms_count >= comms_wake)
    {
        comms_count = 0;
        need_comms = 1;
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
do_fridge()
{
    struct epoch_ticks now;
    get_epoch_ticks(&now);
    uint32_t off_time = now.ticks - fridge_off_clock.ticks;
    bool wort_valid = last_wort != DS18X20_INVALID_DECICELSIUS;
    bool fridge_valid = last_fridge != DS18X20_INVALID_DECICELSIUS;

    int16_t wort_max = fridge_setpoint + fridge_difference;
    int16_t wort_min = fridge_setpoint;

    // the fridge min/max only apply if the wort sensor is broken
    int16_t fridge_min = fridge_setpoint - FRIDGE_AIR_MIN_RANGE;
    int16_t fridge_max = fridge_setpoint + FRIDGE_AIR_MAX_RANGE;

    uint8_t fridge_on = PORT_FRIDGE & _BV(PIN_FRIDGE);
    printf_P(PSTR("last_wort %hd (%hd, %hd), last_fridge %hd (%hd, %hd), setpoint %hd, diff %hd, fridge_on %hhu\n"), 
            last_wort, wort_min, wort_max, 
            last_fridge, fridge_min, fridge_max, 
            fridge_setpoint, fridge_difference, fridge_on);

    if (off_time < fridge_delay)
    {
        printf_P(PSTR("waiting for fridge delay current %hu, wait %hu\n"),
                off_time, fridge_delay);
        return;
    }

    // handle failure of the wort sensor. if it is a short (intermittent?)
    // failure we wait until it has been broken for a period of time
    // (WORT_INVALID_TIME) before doing anything.
    if (wort_valid)
    {
        wort_valid_clock = now;
    }
    else
    {
        printf_P(PSTR("wort sensor is invalid\n"));
        uint32_t invalid_time = now.ticks - wort_valid_clock.ticks;
        if (invalid_time < WORT_INVALID_TIME)
        {
            printf("only been invalid for %ld, waiting\n", invalid_time);
            return;
        }
    }

    if (!fridge_valid)
    {
        printf_P(PSTR("fridge sensor is invalid\n"));
    }

    if (fridge_on)
    {
        bool turn_off = false;
        uint16_t on_time = now.ticks - fridge_on_clock.ticks;

        uint16_t overshoot = 0;
        if (on_time > overshoot_delay)
        {
            overshoot = overshoot_factor * MIN(OVERSHOOT_MAX_DIV, on_time) / OVERSHOOT_MAX_DIV;
        }

        printf_P(PSTR("on_time %hu, overshoot %hu\n"), on_time, overshoot);

        // wort has cooled enough. will probably cool a bit more by itself
        if (wort_valid)
        {
            if ((last_wort - overshoot) < fridge_setpoint)
            {
                printf_P(PSTR("wort has cooled enough, overshoot %hu on_time %hu\n"), overshoot, on_time);
                turn_off = true;
            }
        }
        else
        {
            if (fridge_valid && last_fridge < fridge_min)
            {
                printf_P(PSTR("fridge off fallback\n"));
                turn_off = true;
            }
        }

        if (turn_off)
        {
            // too cold, turn off
            printf_P(PSTR("Turning fridge off\n"));
            PORT_FRIDGE &= ~_BV(PIN_FRIDGE);
            fridge_off_clock = now;
        }
    }
    else
    {
        bool turn_on = false;

        if (wort_valid)
        {
            if (last_wort >= wort_max)
            {
                printf_P(PSTR("wort is too hot\n"));
                turn_on = true;
            }
        }
        else
        {
            if (fridge_valid && last_fridge >= fridge_max)
            {
                printf_P(PSTR("fridge on fallback\n"));
                turn_on = true;
            }
        }

        if (turn_on)
        {
            // too hot, turn on
            printf_P(PSTR("Turning fridge on\n"));
            PORT_FRIDGE |= _BV(PIN_FRIDGE);
            fridge_on_clock = now;
        }
    }
}

static void
do_measurement()
{
    blink();

    /* Take the timer here since deep_sleep() below could take 6 seconds */
    get_epoch_ticks(&last_measurement_clock);
    if (n_measurements == 0)
    {
        first_measurement_clock = last_measurement_clock;
    }

    simple_ds18b20_start_meas(NULL);
    _delay_ms(DS18B20_TCONV_12BIT);

    if (n_measurements == max_measurements)
    {
        n_measurements = 0;
    }

    for (uint8_t s = 0; s < n_sensors; s++)
    {
        uint16_t reading;
        uint8_t ret = simple_ds18b20_read_raw(sensor_id[s], &reading);
        if (ret != DS18X20_OK)
        {
            reading = VALUE_BROKEN;
        }
        set_measurement(s, n_measurements, reading);

        if (memcmp(sensor_id[s], fridge_id, sizeof(fridge_id)) == 0)
        {
            last_fridge = ds18b20_raw16_to_decicelsius(reading);
        }
        if (memcmp(sensor_id[s], wort_id, sizeof(wort_id)) == 0)
        {
            last_wort = ds18b20_raw16_to_decicelsius(reading);
        }
    }

    n_measurements++;
}

static void
do_comms()
{
    get_epoch_ticks(&last_comms_clock);

    // turn on bluetooth
    set_aux_power(1);
    // avoid receiving rubbish, perhaps
    _delay_ms(50);
    uart_on();
    
    // write sd card here? same 3.3v regulator...
    
    for (comms_timeout = wake_secs; 
        comms_timeout > 0 || stay_awake;  
        )
    {
        if (need_measurement)
        {
            need_measurement = 0;
            do_measurement();
            do_fridge();
            continue;
        }

        if (have_cmd)
        {
            have_cmd = 0;
            read_handler();
            continue;
        }

        // wait for commands from the master
        idle_sleep();
    }

    uart_off();
    // in case bluetooth takes time to flush
    _delay_ms(100);
    set_aux_power(0);
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

    set_aux_power(0);

    stdout = &mystdout;
    uart_on();

    printf(PSTR("Started.\n"));

    load_params();

    init_sensors();

    uart_off();

    // turn off everything except timer2
    PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC);

    setup_tick_counter();

    sei();

    need_comms = 1;
    need_measurement = 1;

    stay_awake = 1;

    for(;;)
    {
        if (button_pressed)
        {
            // debounce
            _delay_ms(200);
            need_comms = 1;
            comms_timeout = wake_secs;
            button_pressed = 0;
            continue;
        }

        if (need_comms)
        {
            need_comms = 0;
            do_comms();
            continue;
        }

        if (need_measurement)
        {
            need_measurement = 0;
            do_measurement();
            do_fridge();
            continue;
        }

        deep_sleep();
    }

    return 0;   /* never reached */
}

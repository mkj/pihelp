// Matt Johnston 2012
// Based on ds18x20.c by Martin Thomas, in turn based on code by 
// Peter // Dannegger and others.
//
#include <stdio.h>
#include <avr/pgmspace.h>

#include "ds18x20.h"
#include "onewire.h"
#include "crc8.h"

#include "simple_ds18b20.h"

uint8_t 
simple_ds18b20_start_meas(uint8_t id[])
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		ow_command_with_parasite_enable(DS18X20_CONVERT_T, id);
		ret = DS18X20_OK;
	} 
	else { 
		ret = DS18X20_START_FAIL;
	}

	return ret;
}

static uint8_t 
read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t i;
	uint8_t ret;

	ow_command( DS18X20_READ, id );
	for ( i = 0; i < n; i++ ) {
		sp[i] = ow_byte_rd();
	}
	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) {
		ret = DS18X20_ERROR_CRC;
	} else {
		ret = DS18X20_OK;
	}

	return ret;
}

int16_t 
ds18b20_raw16_to_decicelsius(uint16_t measure)
{
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;

	// check for negative 
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) {
		decicelsius = -decicelsius;
	}

	if ( decicelsius == 850 || decicelsius < -550 || decicelsius > 1250 ) {
		return DS18X20_INVALID_DECICELSIUS;
	} else {
		return decicelsius;
	}
}

uint8_t 
simple_ds18b20_read_decicelsius( uint8_t id[], int16_t *decicelsius )
{
    uint16_t reading;
	uint8_t ret;

    ret = simple_ds18b20_read_raw(id, &reading);
    if (ret == DS18X20_OK)
    {
        *decicelsius = ds18b20_raw16_to_decicelsius(reading);
    }
    return ret;
}

uint8_t 
simple_ds18b20_read_raw( uint8_t id[], uint16_t *reading )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	if (id)
	{
		ow_reset();
	}
	ret = read_scratchpad( id, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
        *reading = sp[0] | (sp[1] << 8);
	}
	return ret;
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


uint8_t 
simple_ds18b20_read_all()
{
	uint8_t id[OW_ROMCODE_SIZE];
	for( uint8_t diff = OW_SEARCH_FIRST; diff != OW_LAST_DEVICE; )
	{
		diff = ow_rom_search( diff, &id[0] );

		if( diff == OW_PRESENCE_ERR ) {
			printf_P( PSTR("No Sensor found\r") );
			return OW_PRESENCE_ERR; // <--- early exit!
		}
		
		if( diff == OW_DATA_ERR ) {
			printf_P( PSTR("Bus Error\r") );
			return OW_DATA_ERR;     // <--- early exit!
		}

		int16_t decicelsius;
		uint8_t ret = simple_ds18b20_read_decicelsius(NULL, &decicelsius);
		if (ret != DS18X20_OK)
		{
			printf_P(PSTR("Failed reading\r"));
			return OW_DATA_ERR;
		}

		printf_P(PSTR("DS18B20 %d: "), diff);
		if (crc8(id, OW_ROMCODE_SIZE))
		{
			printf_P(PSTR("CRC fail"));
		}
		printhex(id, OW_ROMCODE_SIZE, stdout);
		printf_P(PSTR(" %d.%d ºC\n"), decicelsius/10, decicelsius % 10);
	}
	printf_P(PSTR("Done sensors\n"));
	return DS18X20_OK;
}


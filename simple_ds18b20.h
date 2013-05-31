#ifndef SIMPLE_DS18B20_H_
#define SIMPLE_DS18B20_H_
#include <stdint.h>
#include <stdio.h>

#include "ds18x20.h"

uint8_t simple_ds18b20_start_meas(uint8_t id[]);
void printhex(uint8_t *id, uint8_t n, FILE *stream);
void printhex_byte( const unsigned char  b, FILE *stream );
uint8_t simple_ds18b20_read_decicelsius( uint8_t id[], int16_t *decicelsius );
int16_t ds18b20_raw16_to_decicelsius(uint16_t measure);
uint8_t simple_ds18b20_read_raw( uint8_t id[], uint16_t *reading );
uint8_t simple_ds18b20_read_all();

#endif // SIMPLE_DS18B20_H_

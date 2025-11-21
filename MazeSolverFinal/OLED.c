/*
 * OLED.c
 *
 * Created: 9/19/2024 9:50:58 PM
 *  Author: sternbal
 */ 
#include <avr/io.h>
#include "OLED.h"
#define F_CPU 16000000L
#include <util/delay.h>
#include <avr/pgmspace.h>

extern const PROGMEM uint8_t pololuOledFont[][5];


void OLED_init(void)
{
	// set reset pin to output
	DDRD |= 1<<OLEDRES;

	// set clock pin to output and make it low
	DDRD |= 1<<OLEDCLK;
	PORTD &= ~(1<<OLEDCLK);

	// set MOSI pin to output
	DDRD |= 1<<OLEDMOSI;


	// set data/command pin to output
	DDRB |= 1<<OLEDDC;
	
}

void OLED_clk_low(void)
{
	PORTD &= ~(1<<OLEDCLK);
	
}
void OLED_clk_high(void)
{
	PORTD |= (1<<OLEDCLK);
	
}

void OLED_mos_pin(uint8_t val)
{
	if(val==0) PORTD &= ~(1<<OLEDMOSI);
	else PORTD |= 1<<OLEDMOSI;
}

void OLED_reset(void)
{
	PORTD &= ~(1<<OLEDRES);
	_delay_us(10);
	PORTD |= (1<<OLEDRES);
	_delay_us(10);
}


void OLED_clearDisplayRam(void)
{
	OLED_clk_low();
	OLED_commandMode();
	OLED_write(SET_COL_ADDR_LOW | 2);
	for (uint8_t page = 0;page < 8; page++)
	{
		OLED_commandMode();
		OLED_write(SET_PAGE_ADDR | page);
		OLED_write(SET_COL_ADDR_HIGH | 0);
		OLED_dataMode();
		for (uint8_t i = 0 ; i < 128; i++)
		{
			OLED_write(0);
		}
	}
}

void OLED_configureDefault(void)
{
	OLED_clk_low();	
	OLED_commandMode();
	OLED_write(SET_SEG_REMAP | 1);
	OLED_write(SET_COM_SCAN_DIR | 8);
	OLED_write(SET_CONTRAST);
	OLED_write(0xFF);
	OLED_write(SET_DISPLAY_ON | 1);
}

void OLED_commandMode(void)
{
	PORTB &= ~(1<<OLEDDC);
}

void OLED_dataMode(void)
{
	PORTB |= (1<<OLEDDC);
}


void OLED_write(uint8_t d)
{
	OLED_clk_low();
	OLED_mos_pin(d >> 7 & 0x01);
	OLED_clk_high();
	
	OLED_clk_low();
	OLED_mos_pin(d >> 6 & 0x01);
	OLED_clk_high();

	OLED_clk_low();
	OLED_mos_pin(d >> 5 & 0x01);
	OLED_clk_high();
	
	OLED_clk_low();
	OLED_mos_pin(d >> 4 & 0x01);
	OLED_clk_high();
	
	OLED_clk_low();
	OLED_mos_pin(d >> 3 & 0x01);
	OLED_clk_high();

	OLED_clk_low();
	OLED_mos_pin(d >> 2 & 0x01);
	OLED_clk_high();
	
	OLED_clk_low();
	OLED_mos_pin(d >> 1 & 0x01);
	OLED_clk_high();

	OLED_clk_low();
	OLED_mos_pin(d >> 0 & 0x01);
	OLED_clk_high();
}

void OLED_writeText(char *text)
{
	OLED_commandMode();
	OLED_write(SET_PAGE_ADDR | 0);
	OLED_write(SET_COL_ADDR_HIGH | 1);
	OLED_write(SET_COL_ADDR_LOW | 0);
	OLED_dataMode();
	for (uint8_t i=0;i<6;i++)
	{
		char glyph= *text++;

		for (uint8_t pixelX=0;pixelX<5;pixelX++)
		{
			uint8_t column=getGlyphColumn(glyph,pixelX);
			OLED_write(column);
//			OLED_write(column);
		}
		OLED_write(0);
		OLED_write(0);
	}
}
uint8_t getGlyphColumn(uint8_t glyph, uint8_t pixelX)
{
	if(glyph>=0x20)
	{
		return pgm_read_byte(&pololuOledFont[glyph - 0x20][pixelX]);
	}
	return(0);
}

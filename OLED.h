/*
 * OLED.h
 *
 * Created: 9/19/2024 7:12:26 PM
 *  Author: sternbal
 */ 


#ifndef OLED_H_
#define OLED_H_

#define OLEDCLK PD3 // D1
#define OLEDRES PD2 // D0
#define OLEDDC PB0 // D17
#define OLEDMOSI PD5
void OLED_clk_low(void);
void OLED_clk_high(void);
void OLED_mos_pin(uint8_t);
void OLED_write(uint8_t);
void OLED_commandMode(void);
void OLED_dataMode(void);
void OLED_reset(void);
void OLED_clearDisplayRam(void);
void OLED_configureDefault(void);
void OLED_writeText(char*);
void OLED_init(void);
uint8_t getGlyphColumn(uint8_t, uint8_t);
#define SET_COL_ADDR_LOW 0x00
#define SET_COL_ADDR_HIGH 0x10
#define SET_CONTRAST 0x81
#define SET_SEG_REMAP 0xA0
#define SET_INV_DISP 0xA6
#define SET_DISPLAY_ON 0xAE
#define SET_PAGE_ADDR 0xB0
#define SET_COM_SCAN_DIR 0xC0

#endif /* OLED_H_ */
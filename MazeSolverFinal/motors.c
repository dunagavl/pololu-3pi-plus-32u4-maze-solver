#include <avr/io.h>
#include "motors.h"

#ifndef _BV
#define _BV(bit) (1U << (bit))
#endif

// Pin map (3pi+ 32U4):
// OC1A = PB5 (Right PWM), OC1B = PB6 (Left PWM)
// DIR_R = PB1, DIR_L = PB2
#define RIGHT_DIR PB1
#define LEFT_DIR  PB2
#define RIGHT_PWM PB5   // OC1A
#define LEFT_PWM  PB6   // OC1B

static uint8_t  flipL = 0, flipR = 0;
static uint16_t gTOP  = 400;

static inline uint16_t clampu(uint16_t x, uint16_t hi) { return x > hi ? hi : x; }

void motors_init(uint16_t top)
{
	gTOP = top ? top : 400;   // Pololu uses 400

	// GPIO setup
	DDRB |= _BV(RIGHT_DIR) | _BV(LEFT_DIR) | _BV(RIGHT_PWM) | _BV(LEFT_PWM);
	PORTB &= ~(_BV(RIGHT_DIR) | _BV(LEFT_DIR)); // default forward (DIR low)

	// Timer1: Phase & Freq Correct PWM (mode 8), TOP=ICR1, clk/1 → 20 kHz at TOP=400
	TCCR1A = (1<<COM1A1) | (1<<COM1B1);  // non-inverting on A/B
	TCCR1B = (1<<WGM13)  | (1<<CS10);    // mode 8, prescaler /1
	ICR1   = gTOP;
	OCR1A  = 0;
	OCR1B  = 0;
}

uint16_t motors_get_top(void) { return gTOP; }

void motors_flipLeft(bool flip)  { flipL = flip ? 1 : 0; }
void motors_flipRight(bool flip) { flipR = flip ? 1 : 0; }

static inline void set_one(int16_t spd, uint8_t dir_bit, volatile uint16_t *ocr, uint8_t flip)
{
	uint8_t reverse = 0;

	if (spd < 0) { spd = -spd; reverse = 1; }
	if (spd > (int16_t)gTOP) spd = (int16_t)gTOP;

	*ocr = clampu((uint16_t)spd, gTOP);

	// reverse ^ flip -> DIR high
	if (reverse ^ flip) PORTB |=  _BV(dir_bit);
	else                PORTB &= ~_BV(dir_bit);
}

void motors_set_speeds(int16_t left, int16_t right)
{
	// Left -> OCR1B, Right -> OCR1A (matches Pololu)
	set_one(left,  LEFT_DIR,  &OCR1B, flipL);
	set_one(right, RIGHT_DIR, &OCR1A, flipR);
}

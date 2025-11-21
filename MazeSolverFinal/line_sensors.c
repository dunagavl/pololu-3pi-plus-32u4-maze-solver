#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "line_sensors.h"

#ifndef _BV
#define _BV(bit) (1U << (bit))
#endif

// ---- 3pi+ 32U4 pin map (matches Pololu Arduino pins) ----
// emitterPin = 11 -> PB7
#define EMIT   PB7
// line0=12 -> PD6; line1=A0 -> PF7; line2=A2 -> PF5; line3=A3 -> PF4; line4=A4 -> PF1
#define DN0_PD PD6
#define DN1_PF PF7
#define DN2_PF PF5
#define DN3_PF PF4
#define DN4_PF PF1

// Timer3 prescale = 8 -> 0.5 µs per tick
#define T3_PRESCALE_8 0x02

static uint16_t timeout_us   = 4000;   // Pololu default
static uint16_t timeout_ticks;         // = timeout_us * 2 (0.5 µs/tick)

static uint16_t calMinOn[NUM_SENSORS],  calMaxOn[NUM_SENSORS];
static uint16_t calMinOff[NUM_SENSORS], calMaxOff[NUM_SENSORS];
static uint8_t  haveOn = 0, haveOff = 0;

static uint16_t lastPosition = 0;      // 0..4000 (sticky when line lost)

// ---- emitter control (Pololu: on = output HIGH, off = input) ----
static inline void emit_on_hw(void)  { DDRB |= _BV(EMIT); PORTB |= _BV(EMIT); }
static inline void emit_off_hw(void) { DDRB &= ~_BV(EMIT); /* tri-state */ }

void line_emitters_on(void)  { emit_on_hw(); }
void line_emitters_off(void) { emit_off_hw(); }

void line_sensors_set_timeout_us(uint16_t us)
{
	if (us == 0) us = 1;
	if (us > 32767) us = 32767;
	timeout_us    = us;
	timeout_ticks = (uint16_t)(us * 2u); // 0.5 µs / tick
}

uint16_t line_sensors_get_timeout_us(void) { return timeout_us; }

void line_calibrate_reset(void)
{
	// Min starts at maxValue (timeout), max at 0 (Pololu behavior)
	for (uint8_t i=0;i<NUM_SENSORS;i++) {
		calMinOn[i]  = timeout_us;
		calMaxOn[i]  = 0;
		calMinOff[i] = timeout_us;
		calMaxOff[i] = 0;
	}
	haveOn = haveOff = 0;
	lastPosition = 0;
}

void line_sensors_init(void)
{
	// Timer3 for timing
	TCCR3A = 0x00;
	TCCR3B = T3_PRESCALE_8;

	emit_off_hw(); // start with emitters off
	line_sensors_set_timeout_us(4000);
	line_calibrate_reset();
}

// ---- core raw read in timer ticks (honors mode) ----
static void read_raw_ticks(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode)
{
	// Set emitter state per mode
	if (mode == LS_MODE_On)       emit_on_hw();
	else if (mode == LS_MODE_Off) emit_off_hw();
	// Manual: leave as-is

	// Charge (outputs HIGH)
	DDRD  |= _BV(DN0_PD);
	PORTD |= _BV(DN0_PD);
	DDRF  |= _BV(DN1_PF)|_BV(DN2_PF)|_BV(DN3_PF)|_BV(DN4_PF);
	PORTF |= _BV(DN1_PF)|_BV(DN2_PF)|_BV(DN3_PF)|_BV(DN4_PF);

	_delay_us(10);

	// Discharge (inputs) and measure with Timer3
	TCNT3 = 0;
	DDRD  &= ~_BV(DN0_PD);
	DDRF  &= ~(_BV(DN1_PF)|_BV(DN2_PF)|_BV(DN3_PF)|_BV(DN4_PF));
	PORTD &= ~_BV(DN0_PD);
	PORTF &= ~(_BV(DN1_PF)|_BV(DN2_PF)|_BV(DN3_PF)|_BV(DN4_PF));

	out[0]=out[1]=out[2]=out[3]=out[4]=timeout_ticks;
	uint8_t active = 0x1F;

	while (active && (TCNT3 < timeout_ticks)) {
		uint8_t d = PIND;
		uint8_t f = PINF;
		uint16_t t = TCNT3;

		if ((active & _BV(0)) && ((d & _BV(DN0_PD)) == 0)) { out[0] = t; active &= ~_BV(0); }
		if ((active & _BV(1)) && ((f & _BV(DN1_PF)) == 0)) { out[1] = t; active &= ~_BV(1); }
		if ((active & _BV(2)) && ((f & _BV(DN2_PF)) == 0)) { out[2] = t; active &= ~_BV(2); }
		if ((active & _BV(3)) && ((f & _BV(DN3_PF)) == 0)) { out[3] = t; active &= ~_BV(3); }
		if ((active & _BV(4)) && ((f & _BV(DN4_PF)) == 0)) { out[4] = t; active &= ~_BV(4); }
	}

	// Restore emitters off if we forced a state
	if (mode == LS_MODE_On || mode == LS_MODE_Off) emit_off_hw();
}

void line_read(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode)
{
	uint16_t ticks[NUM_SENSORS];
	read_raw_ticks(ticks, mode);

	// Convert ticks → µs (0.5 µs / tick) and clamp
	for (uint8_t i=0;i<NUM_SENSORS;i++) {
		uint32_t us = (uint32_t)ticks[i] / 2u;
		if (us > timeout_us) us = timeout_us;
		out[i] = (uint16_t)us;
	}
}

void line_calibrate(LineSensorsReadMode mode)
{
	if (mode == LS_MODE_Manual) return;  // not supported

	uint16_t r[NUM_SENSORS];
	uint16_t max10[NUM_SENSORS] = {0};
	uint16_t min10[NUM_SENSORS];

	for (uint8_t i=0;i<NUM_SENSORS;i++) min10[i] = timeout_us;

	// 10 samples per Pololu spec
	for (uint8_t j=0;j<10;j++) {
		line_read(r, mode);
		for (uint8_t i=0;i<NUM_SENSORS;i++) {
			if (r[i] > max10[i]) max10[i] = r[i];
			if (r[i] < min10[i]) min10[i] = r[i];
		}
		_delay_ms(1);
	}

	if (mode == LS_MODE_On) {
		haveOn = 1;
		for (uint8_t i=0;i<NUM_SENSORS;i++) {
			if (min10[i] > calMaxOn[i]) calMaxOn[i] = min10[i];
			if (max10[i] < calMinOn[i]) calMinOn[i] = max10[i];
		}
		} else { // Off
		haveOff = 1;
		for (uint8_t i=0;i<NUM_SENSORS;i++) {
			if (min10[i] > calMaxOff[i]) calMaxOff[i] = min10[i];
			if (max10[i] < calMinOff[i]) calMinOff[i] = max10[i];
		}
	}
}

static void normalize_from(const uint16_t raw[NUM_SENSORS],
uint16_t out[NUM_SENSORS],
const uint16_t *minv,
const uint16_t *maxv)
{
	for (uint8_t i=0;i<NUM_SENSORS;i++) {
		uint16_t lo = minv[i], hi = maxv[i];
		uint16_t v  = raw[i];

		if (hi <= lo) { out[i] = 0; continue; }
		if (v <= lo)  { out[i] = 0; continue; }
		if (v >= hi)  { out[i] = 1000; continue; }

		out[i] = (uint16_t)(((uint32_t)(v - lo) * 1000u) / (uint32_t)(hi - lo));
	}
}

void line_read_calibrated(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode)
{
	if (mode == LS_MODE_Manual) { for (uint8_t i=0;i<NUM_SENSORS;i++) out[i]=0; return; }

	uint16_t raw[NUM_SENSORS];
	line_read(raw, mode);

	if (mode == LS_MODE_On && haveOn)        normalize_from(raw, out, calMinOn,  calMaxOn);
	else if (mode == LS_MODE_Off && haveOff) normalize_from(raw, out, calMinOff, calMaxOff);
	else                                     for (uint8_t i=0;i<NUM_SENSORS;i++) out[i]=0;
}

static uint16_t readLinePrivate(uint16_t out[NUM_SENSORS],
LineSensorsReadMode mode,
bool invert)
{
	line_read_calibrated(out, mode);

	if (invert) {
		for (uint8_t i=1;i<4;i++) out[i] = 1000 - out[i];
	}

	uint8_t  onLine = 0;
	uint32_t avg = 0;  // weighted sum
	uint32_t sum = 0;  // denominator

	for (uint8_t i=1;i<4;i++) {
		uint16_t v = out[i];
		if (v > 200) onLine = 1;                 // Pololu on-line threshold
		if (v > 50) {                            // ignore small noise
			avg += (uint32_t)v * (uint32_t)(i * 1000u);
			sum += v;
		}
	}

	if (!onLine) {
		// Stick to last side like Pololu
		return (lastPosition < 2000) ? 0 : 4000;
	}

	lastPosition = (uint16_t)(avg / sum);
	return lastPosition;
}

uint16_t line_readLineBlack(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode)
{
	return readLinePrivate(out, mode, false);
}

uint16_t line_readLineWhite(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode)
{
	return readLinePrivate(out, mode, true);
}

// Accessors
const uint16_t* line_get_min_on(void)   { return calMinOn;  }
const uint16_t* line_get_max_on(void)   { return calMaxOn;  }
const uint16_t* line_get_min_off(void)  { return calMinOff; }
const uint16_t* line_get_max_off(void)  { return calMaxOff; }

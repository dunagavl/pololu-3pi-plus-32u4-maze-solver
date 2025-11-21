#ifndef MOTORS_H
#define MOTORS_H

#include <stdint.h>
#include <stdbool.h>

// Initialize Timer1 PWM. Pass 0 to use default TOP=400 (→ 20 kHz at clk/1).
void motors_init(uint16_t top);

// Optional direction flips (match Pololu semantics).
void motors_flipLeft(bool flip);
void motors_flipRight(bool flip);

// Signed speeds in range -TOP..+TOP (Pololu uses TOP=400). Values are clamped.
void motors_set_speeds(int16_t left, int16_t right);

uint16_t motors_get_top(void);

#endif

#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

#define NUM_SENSORS 5

// Mirror Pololu read modes.
typedef enum {
	LS_MODE_Off = 0,   // emitters forced off during the read
	LS_MODE_On,        // emitters forced on during the read
	LS_MODE_Manual     // leave emitters as-is (not supported for calibration)
} LineSensorsReadMode;

// Init + timeout (microseconds). Default 4000 µs; max 32767 µs.
void     line_sensors_init(void);
void     line_sensors_set_timeout_us(uint16_t us);
uint16_t line_sensors_get_timeout_us(void);

// Manual emitter control (only has effect with LS_MODE_Manual).
void     line_emitters_on(void);
void     line_emitters_off(void);

// Calibration API (On/Off sets are independent). Each call samples 10 times.
void     line_calibrate_reset(void);
void     line_calibrate(LineSensorsReadMode mode);

// Reading API
void     line_read(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode);              // raw µs
void     line_read_calibrated(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode);   // 0..1000

uint16_t line_readLineBlack(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode);     // 0..4000
uint16_t line_readLineWhite(uint16_t out[NUM_SENSORS], LineSensorsReadMode mode);     // 0..4000

// Optional: inspect learned calibration (for saving/logging)
const uint16_t* line_get_min_on(void);
const uint16_t* line_get_max_on(void);
const uint16_t* line_get_min_off(void);
const uint16_t* line_get_max_off(void);

#endif

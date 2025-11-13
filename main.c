/*
 * main.c — 3pi+ 32U4 Line Maze Solver (learn + rerun on Button B) using your drivers
 * Deps: motors.h/.c, line_sensors.h/.c, OLED.h/.c, font.c (for getGlyphColumn) optional
 * No EEPROM / No Arduino core / No buzzer required
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "motors.h"
#include "line_sensors.h"
#include "OLED.h"   // your SSD1306-ish driver (with getGlyphColumn)

/* -------- Tiny delay wrapper for variable milliseconds -------- */
static inline void delay_ms_u16(uint16_t ms)
{
    while (ms--) { _delay_ms(1); }   // 1 ms is a compile-time constant
}

/* -------- Minimal Button B driver (PD5 / Arduino D5, active-low) -------- */
#define BTN_B_DDR   DDRD
#define BTN_B_PORT  PORTD
#define BTN_B_PINR  PIND
#define BTN_B_BIT   5

static inline void buttonB_init(void)
{
    BTN_B_DDR  &= ~(1 << BTN_B_BIT);   // input
    BTN_B_PORT |=  (1 << BTN_B_BIT);   // enable internal pull-up
}

static inline bool buttonB_is_pressed_raw(void)
{
    // Active-low: pressed == 0
    return (BTN_B_PINR & (1 << BTN_B_BIT)) == 0;
}

/* Simple debounced "press" edge detector (returns true once per clean press).
   Assumes you poll it roughly every 1 ms (we do, in the wait loop). */
static bool buttonB_getSingleDebouncedPress(void)
{
    static uint8_t  state = 0;  // 0=released,1=maybe-pressed,2=pressed,3=maybe-released
    static uint16_t timer = 0;

    bool v = buttonB_is_pressed_raw();
    switch (state)
    {
        case 0: if (v) { state = 1; timer = 0; } break;
        case 1: if (v) { if (++timer >= 15) { state = 2; return true; } } else { state = 0; } break;
        case 2: if (!v) { state = 3; timer = 0; } break;
        case 3: if (!v) { if (++timer >= 15) state = 0; } else { state = 2; } break;
    }
    return false;
}

/* ---------------- Parameters (tuned conservatively) ---------------- */
static uint16_t maxSpeed;
static int16_t  minSpeed;
static uint16_t baseSpeed;
static uint16_t calibrationSpeed;
static uint16_t proportional; // P * 256
static uint16_t derivative;   // D * 256
static uint16_t integral;     // I * 256

// integral accumulator (optional; safe to keep since your PID had it)
static int32_t i_acc = 0;
static const int16_t I_TERM_MAX = 150; // limit the I contribution in "speed units"

// Intersection probing speeds/intervals (ported from your C++ sketch)
static uint16_t angintSpeed      = 61;  // short creep after segment
static uint16_t angintDelay_ms   = 38;
static uint16_t interSpeed       = 40;  // center wheels in the intersection
static uint16_t interDelay_ms    = 90;
static uint16_t turnSpeed        = 100; // in-place turning speed
static uint16_t turnDelay_ms     = 200; // ~90° timing — re-tune on your floor

// Gentle braking on rerun before making each turn
static uint16_t llbrake1Speed    = 72, llbrake1Delay_ms = 38;
static uint16_t llbrake2Speed    = 50, llbrake2Delay_ms = 55;

/* ---------------- Thresholds ----------------
   Sensor readings from your driver are 0..1000 after calibration.
   Tweak if your tape/floor contrast differs.
*/
enum {
    TH_NO_AHEAD   = 100,  // consider “no line ahead” if all three mids < this
    TH_SIDE       = 200,  // consider side branch present if side > this
    TH_STRAIGHT   = 300,  // consider straight present if any mid > this
    TH_FINISH     = 600   // finish: all three mids > this (large dark blob)
};

/* ---------------- Preset ---------------- */
static void selectStandard(void)
{
    maxSpeed         = 120;
    minSpeed         = 0;
    baseSpeed        = maxSpeed;
    calibrationSpeed = 60;

    proportional     = 64;    // 1/4
    derivative       = 256;   // 1
    integral         = 16;    // light I (optional)
}

/* ---------------- Small utils ---------------- */
static inline int16_t clamp16(int16_t x, int16_t lo, int16_t hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* ---------------- OLED helpers (as in your previous code) ---------------- */
static void oled_set_cursor(uint8_t page, uint8_t col /*0..127*/)
{
    OLED_commandMode();
    OLED_write(SET_PAGE_ADDR | (page & 0x0F));
    OLED_write(SET_COL_ADDR_HIGH | ((col >> 4) & 0x0F));
    OLED_write(SET_COL_ADDR_LOW  | (col & 0x0F));
    OLED_dataMode();
}

static void oled_write_char(char c)
{
    extern uint8_t getGlyphColumn(uint8_t c, uint8_t col);
    for (uint8_t x = 0; x < 5; x++)
    {
        uint8_t col = getGlyphColumn((uint8_t)c, x);
        OLED_write(col);
    }
    OLED_write(0); // spacing
}

static void oled_write_string_at(uint8_t page, uint8_t col, const char *s)
{
    oled_set_cursor(page, col);
    while (*s) oled_write_char(*s++);
}

static void oled_clear_page(uint8_t page)
{
    OLED_commandMode();
    OLED_write(SET_PAGE_ADDR | (page & 0x0F));
    OLED_write(SET_COL_ADDR_HIGH | 0);
    OLED_write(SET_COL_ADDR_LOW  | 2);
    OLED_dataMode();
    for (uint8_t i = 0; i < 128; i++) { OLED_write(0); }
}

static void oled_begin(void)
{
    OLED_init();
    OLED_reset();
    OLED_clearDisplayRam();
    OLED_configureDefault();
    oled_write_string_at(0, 0, "Maze Solver");
    oled_write_string_at(1, 0, "Calibrating");
}

/* ---------------- Path store + simplify ---------------- */
static char path[128];
static uint8_t path_len = 0;

static void path_reset(void) { path_len = 0; path[0] = 0; }

static void path_push(char c)
{
    if (path_len < sizeof(path)-1) {
        path[path_len++] = c;
        path[path_len]   = 0;
    }
}

/* Replace x B x with a single net turn (classic simplification) */
static void simplify_path(void)
{
    if (path_len < 3) return;
    if (path[path_len-2] != 'B') return;

    int total = 0;
    for (uint8_t i = 1; i <= 3; i++) {
        char t = path[path_len - i];
        if (t == 'R') total += 90;
        else if (t == 'L') total += 270;
        else if (t == 'B') total += 180;
    }
    total %= 360;

    switch (total) {
        case 0:   path[path_len-3] = 'S'; break;
        case 90:  path[path_len-3] = 'R'; break;
        case 180: path[path_len-3] = 'B'; break;
        case 270: path[path_len-3] = 'L'; break;
    }
    path_len -= 2;
    path[path_len] = 0;
}

/* ---------------- Turning ---------------- */
static void turn(char dir)
{
    switch (dir)
    {
        case 'L':
            motors_set_speeds(-(int16_t)turnSpeed, (int16_t)turnSpeed);
            delay_ms_u16(turnDelay_ms);
            break;
        case 'R':
            motors_set_speeds((int16_t)turnSpeed, -(int16_t)turnSpeed);
            delay_ms_u16(turnDelay_ms);
            break;
        case 'B':
            motors_set_speeds((int16_t)turnSpeed, -(int16_t)turnSpeed);
            delay_ms_u16((uint16_t)(2*turnDelay_ms));
            break;
        case 'S':
        default:
            /* no-op */
            break;
    }
    motors_set_speeds(0, 0);
    _delay_ms(20);
}

/* Decide turn by left-hand rule */
static char select_turn(bool left, bool straight, bool right)
{
    if (left)     return 'L';
    if (straight) return 'S';
    if (right)    return 'R';
    return 'B';
}

/* ---------------- PID line follower for one segment ----------------
   Run until we detect: (a) dead-end ahead, or (b) intersection entry.
   We return at the moment of detection; caller will creep forward,
   probe branches, and decide the turn.
*/
static int16_t lastError = 0;

static void follow_segment(void)
{
    uint16_t values[NUM_SENSORS];
    lastError = 0;

    while (1)
    {
        // Position 0..4000 (black line), plus per-sensor values[] 0..1000
        uint16_t position = line_readLineBlack(values, LS_MODE_On);
        int16_t  error    = (int16_t)position - 2000;

        // P, D (fixed-point /256)
        int16_t p_term = (int16_t)(((int32_t)error               * proportional) >> 8);
        int16_t d_term = (int16_t)(((int32_t)error - lastError) * derivative   >> 8);

        // I term (optional, lightly damped; bleeds when line lost)
        uint32_t sumCal = (uint32_t)values[0]+values[1]+values[2]+values[3]+values[4];
        if (sumCal > 200) { i_acc += error; } else { i_acc -= (i_acc >> 3); }
        int32_t i_term32 = ((int32_t)integral * i_acc) >> 8;
        if (i_term32 >  I_TERM_MAX) { i_term32 =  I_TERM_MAX; if (integral) i_acc = ((int32_t)I_TERM_MAX << 8)/integral; }
        if (i_term32 < -I_TERM_MAX) { i_term32 = -I_TERM_MAX; if (integral) i_acc = -((int32_t)I_TERM_MAX << 8)/integral; }
        int16_t i_term = (int16_t)i_term32;

        int16_t speedDiff = p_term + d_term + i_term;
        lastError = error;

        int16_t left  = (int16_t)baseSpeed + speedDiff;
        int16_t right = (int16_t)baseSpeed - speedDiff;

        left  = clamp16(left,  minSpeed, (int16_t)maxSpeed);
        right = clamp16(right, minSpeed, (int16_t)maxSpeed);
        motors_set_speeds(left, right);

        // Stop criteria:
        // 1) Dead end: inner 3 sensors all below small threshold
        bool no_ahead = (values[1] < TH_NO_AHEAD) &&
                        (values[2] < TH_NO_AHEAD) &&
                        (values[3] < TH_NO_AHEAD);
        if (no_ahead) { return; }

        // 2) Intersection likely if a side sees line
        if (values[0] > TH_SIDE || values[4] > TH_SIDE) { return; }

        _delay_ms(2); // ~500 Hz control loop; adjust if needed
    }
}

/* ---------------- Learning + rerun (button-gated) ---------------- */
static void run_maze_solver(void)
{
    char buf[22];

    // --- LEARNING LAP ---
    path_reset();
    oled_clear_page(0); oled_clear_page(1);
    oled_write_string_at(0, 0, "Learning...");

    while (1)
    {
        // Follow until intersection/dead-end
        follow_segment();

        // small creep to stabilize if we entered at an angle
        motors_set_speeds(angintSpeed, angintSpeed);
        delay_ms_u16(angintDelay_ms);

        // Probe for exits
        bool found_left = false, found_right = false, found_straight = false;
        uint16_t s[NUM_SENSORS];

        line_readLineBlack(s, LS_MODE_On);
        if (s[0] > TH_SIDE) found_left  = true;
        if (s[4] > TH_SIDE) found_right = true;

        // move forward a touch to align wheels with intersection, then recheck straight
        motors_set_speeds(interSpeed, interSpeed);
        delay_ms_u16(interDelay_ms);

        line_readLineBlack(s, LS_MODE_On);
        if (s[1] > TH_STRAIGHT || s[2] > TH_STRAIGHT || s[3] > TH_STRAIGHT)
            found_straight = true;

        // FINISH? (all three mids very dark)
        if (s[1] > TH_FINISH && s[2] > TH_FINISH && s[3] > TH_FINISH) {
            motors_set_speeds(0, 0);
            break;
        }

        // Decide and turn
        char dir = select_turn(found_left, found_straight, found_right);
        turn(dir);

        // Record + simplify
        path_push(dir);
        simplify_path();

        // tiny HUD hint (optional)
        oled_clear_page(1);
        snprintf(buf, sizeof(buf), "Path: %s", path);
        oled_write_string_at(1, 0, buf);
    }

    // --------- Infinite button-gated re-runs ----------
    for (;;)
    {
        // Wait for Button B press to begin the rerun
        oled_clear_page(0); oled_clear_page(1);
        oled_write_string_at(0, 0, "Press B to");
        oled_write_string_at(1, 0, "re-run path");

        for (;;)
        {
            if (buttonB_getSingleDebouncedPress()) break;
            _delay_ms(1);
        }

        // Rerun the minimized path
        oled_clear_page(0); oled_clear_page(1);
        oled_write_string_at(0, 0, "Rerun...");
        oled_write_string_at(1, 0, path);

        for (uint8_t i = 0; i < path_len; i++)
        {
            follow_segment();

            // short braking profile before executing the turn
            motors_set_speeds(llbrake1Speed, llbrake1Speed); delay_ms_u16(llbrake1Delay_ms);
            motors_set_speeds(llbrake2Speed, llbrake2Speed); delay_ms_u16(llbrake2Delay_ms);

            turn(path[i]);
        }

        // Final segment to the finish
        follow_segment();
        motors_set_speeds(0, 0);

        oled_clear_page(0); oled_clear_page(1);
        oled_write_string_at(0, 0, "Done! Press");
        oled_write_string_at(1, 0, "B to rerun");
    }
}

/* ---------------- Main ---------------- */
int main(void)
{
    // Init subsystems
    line_sensors_init();        // Timer3 + emitter control
    motors_init(400);           // Timer1 20 kHz (ICR1=400), speeds +/-400
    buttonB_init();             // Button B (PD5)
    selectStandard();
    oled_begin();

    _delay_ms(300);

    // -------- Auto-calibration (emitters ON), no buttons --------
    line_calibrate_reset();
    for (uint16_t i = 0; i < 80; i++)
    {
        if ((i % 8) == 0) { oled_clear_page(1); oled_write_string_at(1, 0, "Calibrating"); }

        if (i > 20 && i <= 60)
            motors_set_speeds(-(int16_t)calibrationSpeed, (int16_t)calibrationSpeed);
        else
            motors_set_speeds((int16_t)calibrationSpeed, -(int16_t)calibrationSpeed);

        line_calibrate(LS_MODE_On);   // does 10 reads per call (per your driver)
        _delay_ms(10);
    }
    motors_set_speeds(0, 0);

    // Kick off the maze solver (learn once, then rerun on Button B forever)
    run_maze_solver();

    // Not reached
    while (1) { }
}

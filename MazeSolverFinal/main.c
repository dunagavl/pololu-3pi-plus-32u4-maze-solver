/*
 * MazeSolver.c
 *
 * 3pi+ 32U4 Line Maze Solver
 *  - First pass: explore and learn the maze
 *  - Simplify the path
 *  - Replay the simplified path
 *
 * Author: Camille Gowda
 * Uses:
 *  - motors.h        (motors_init, motors_set_speeds)
 *  - line_sensors.h  (line_sensors_init, line_calibrate, line_readLineBlack, etc.)
 *  - OLED.h          (SSD1306-style API with getGlyphColumn)
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
#include "OLED.h"




/* ---------- Simple time helper ---------- */
static inline void wait_ms(uint16_t ms)
{
    while (ms--)
    {
        _delay_ms(1);
    }
}


/* ---------- Controller & robot parameters ---------- */

typedef struct
{
    uint16_t maxSpeed;
    int16_t  minSpeed;
    uint16_t cruiseSpeed;
    uint16_t calibSpeed;

    uint16_t Kp;    // scaled by 256
    uint16_t Kd;    // scaled by 256
    uint16_t Ki;    // scaled by 256

    // movement to probe intersections
    uint16_t creepSpeed;
    uint16_t creepTime_ms;

    uint16_t centerSpeed;
    uint16_t centerTime_ms;

    uint16_t turnSpeed;
    uint16_t quarterTurn_ms;

    // braking profile for rerun
    uint16_t brake1Speed;
    uint16_t brake1Time_ms;
    uint16_t brake2Speed;
    uint16_t brake2Time_ms;
} RobotConfig;

static RobotConfig cfg;

static void load_default_config(void)
{
    cfg.maxSpeed       = 80;
    cfg.minSpeed       = 0;
    cfg.cruiseSpeed    = 80;
    cfg.calibSpeed     = 60;

    cfg.Kp             = 16;    // ~1/4
    cfg.Kd             = 64;   // ~1
    cfg.Ki             = 0;    // small I

    cfg.creepSpeed     = 61;
    cfg.creepTime_ms   = 38;

    cfg.centerSpeed    = 40;
    cfg.centerTime_ms  = 90;

    cfg.turnSpeed      = 100;
    cfg.quarterTurn_ms = 200;

    cfg.brake1Speed    = 72;
    cfg.brake1Time_ms  = 38;
    cfg.brake2Speed    = 50;
    cfg.brake2Time_ms  = 55;
}

/* ---------- PID state ---------- */

static int32_t i_sum = 0;
static const int16_t I_LIM = 150;
static int16_t lastError = 0;

static inline int16_t clamp16(int16_t val, int16_t min, int16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ---------- OLED helpers ---------- */

static void oled_setCursor(uint8_t page, uint8_t col)
{
    OLED_commandMode();
    OLED_write(SET_PAGE_ADDR | (page & 0x0F));
    OLED_write(SET_COL_ADDR_HIGH | ((col >> 4) & 0x0F));
    OLED_write(SET_COL_ADDR_LOW  | (col & 0x0F));
    OLED_dataMode();
}

static void oled_putChar(char c)
{
    extern uint8_t getGlyphColumn(uint8_t, uint8_t);
    for (uint8_t i = 0; i < 5; i++)
    {
        uint8_t col = getGlyphColumn((uint8_t)c, i);
        OLED_write(col);
    }
    OLED_write(0);
}

static void oled_putStringAt(uint8_t page, uint8_t col, const char *s)
{
    oled_setCursor(page, col);
    while (*s) { oled_putChar(*s++); }
}

static void oled_clearPage(uint8_t page)
{
    OLED_commandMode();
    OLED_write(SET_PAGE_ADDR | (page & 0x0F));
    OLED_write(SET_COL_ADDR_HIGH | 0);
    OLED_write(SET_COL_ADDR_LOW  | 2);
    OLED_dataMode();
    for (uint8_t i = 0; i < 128; i++) { OLED_write(0); }
}

static void oled_startup(void)
{
    OLED_init();
    OLED_reset();
    OLED_clearDisplayRam();
    OLED_configureDefault();
    oled_putStringAt(0, 0, "Maze Solver");
    oled_putStringAt(1, 0, "Calibrating");
}

/* ---------- Path recording and simplification ---------- */

#define PATH_MAX 64
static char   path[PATH_MAX];
static uint8_t pathLen = 0;

static void path_clear(void)
{
    pathLen = 0;
    path[0] = 0;
}

static void path_add(char step)
{
    if (pathLen < PATH_MAX - 1)
    {
        path[pathLen++] = step;
        path[pathLen]   = 0;
    }
}

/* Replace the pattern X ? U ? Y with a single equivalent turn. */
static void path_simplify(void)
{
    if (pathLen < 3) return;
    if (path[pathLen - 2] != 'U') return;  // only collapse when we just did a U-turn

    int angle = 0;
    for (uint8_t k = 1; k <= 3; k++)
    {
        char c = path[pathLen - k];
        if (c == 'R') angle += 90;
        else if (c == 'L') angle += 270;
        else if (c == 'U') angle += 180;
    }
    angle %= 360;

    switch (angle)
    {
        case 0:   path[pathLen - 3] = 'S'; break;
        case 90:  path[pathLen - 3] = 'R'; break;
        case 180: path[pathLen - 3] = 'U'; break;
        case 270: path[pathLen - 3] = 'L'; break;
        default:  /* should not happen */    break;
    }

    pathLen -= 2;
    path[pathLen] = 0;
}

/* ---------- Turning helpers ---------- */

static void rotate_robot(char code)
{
    switch (code)
    {
        case 'L': // 90 deg left
            motors_set_speeds(-(int16_t)cfg.turnSpeed, (int16_t)cfg.turnSpeed);
            wait_ms(cfg.quarterTurn_ms);
            break;

        case 'R': // 90 deg right
            motors_set_speeds((int16_t)cfg.turnSpeed, -(int16_t)cfg.turnSpeed);
            wait_ms(cfg.quarterTurn_ms);
            break;

        case 'U': // 180 deg
            motors_set_speeds((int16_t)cfg.turnSpeed, -(int16_t)cfg.turnSpeed);
            wait_ms((uint16_t)(2 * cfg.quarterTurn_ms));
            break;

        case 'S': // straight
        default:
            break;
    }
    motors_set_speeds(0, 0);
    _delay_ms(20);
}

static char choose_turn(bool left_ok, bool straight_ok, bool right_ok)
{
    // Left-hand rule
    if (left_ok)     return 'L';
    if (straight_ok) return 'S';
    if (right_ok)    return 'R';
    return 'U'; // dead end: U-turn
}

/* ---------- Thresholds for interpreting sensor states ---------- */

enum
{
    TH_NO_LINE     = 100,   // “nothing ahead” if inner sensors < this
    TH_SIDE_LINE   = 200,   // side branch present if side > this
    TH_STRAIGHT_OK = 300,   // forward line present if any mid > this
    TH_GOAL        = 300    // “finish blob” if all mid sensors > this
};

/* ---------- Line follower for a single segment ---------- */
/* Runs until:
 *  - straight line disappears, or
 *  - side branch is detected.
 */
static void follow_segment_until_break(void)
{
    uint16_t sense[NUM_SENSORS];
    lastError = 0;

    while (1)
    {
        uint16_t pos = line_readLineBlack(sense, LS_MODE_On);  // 0..4000
        int16_t  e   = (int16_t)pos - 2000;

        // P, D (fixed point scaled by 256)
        int16_t P = (int16_t)(((int32_t)e - 0) * cfg.Kp >> 8);
        int16_t D = (int16_t)(((int32_t)e - lastError) * cfg.Kd >> 8);

        // crude I with bleed when line is lost
        uint32_t sum = (uint32_t)sense[0] + sense[1] + sense[2] + sense[3] + sense[4];
        if (sum > 200) { i_sum += e; }
        else           { i_sum -= (i_sum >> 3); }

        int32_t I32 = ((int32_t)cfg.Ki * i_sum) >> 8;
        if (I32 >  I_LIM) { I32 =  I_LIM; if (cfg.Ki) i_sum = ((int32_t)I_LIM << 8) / cfg.Ki; }
        if (I32 < -I_LIM) { I32 = -I_LIM; if (cfg.Ki) i_sum = -((int32_t)I_LIM << 8) / cfg.Ki; }

        int16_t I = (int16_t)I32;

        int16_t diff = P + D + I;
        lastError = e;

        int16_t l = (int16_t)cfg.cruiseSpeed + diff;
        int16_t r = (int16_t)cfg.cruiseSpeed - diff;

        l = clamp16(l, cfg.minSpeed, (int16_t)cfg.maxSpeed);
        r = clamp16(r, cfg.minSpeed, (int16_t)cfg.maxSpeed);
        motors_set_speeds(l, r);
		


        // Condition A: no line ahead (approaching node / gap / dead end)
        bool center_empty = (sense[1] < TH_NO_LINE &&
                             sense[2] < TH_NO_LINE &&
                             sense[3] < TH_NO_LINE);
        if (center_empty) return;

        // Condition B: either side sees a branch
        if (sense[0] > TH_SIDE_LINE || sense[4] > TH_SIDE_LINE) return;

        _delay_ms(2);
    }
}

/* ---------- Single intersection handling (learn phase) ---------- */
/* 1. follow_segment_until_break()
 * 2. small creep forward
 * 3. read sensors to detect L/R branches
 * 4. move further into intersection to check straight
 * 5. detect goal or choose turn (left-hand rule)
 * 6. rotate and record path + simplify
 * Returns true if goal was reached.
 */
static bool handle_node_learn(void)
{
    char status[24];
    uint16_t s[NUM_SENSORS];


    follow_segment_until_break();


    // creep forward to settle at node
    motors_set_speeds(cfg.creepSpeed, cfg.creepSpeed);
    wait_ms(cfg.creepTime_ms);

    bool has_left  = false;
    bool has_right = false;
    bool has_straight = false;

    // Read sensors and check sides
    line_readLineBlack(s, LS_MODE_On);
    if (s[0] > TH_SIDE_LINE) has_left  = true;
    if (s[4] > TH_SIDE_LINE) has_right = true;

    // Inch into intersection to decide straight / goal
    motors_set_speeds(cfg.centerSpeed, cfg.centerSpeed);
    wait_ms(cfg.centerTime_ms);

    line_readLineBlack(s, LS_MODE_On);
	
	// Decide if straight available
    if (s[1] > TH_STRAIGHT_OK || s[2] > TH_STRAIGHT_OK || s[3] > TH_STRAIGHT_OK)
        has_straight = true;

    // Goal check: All sensors high
    if (s[0]>TH_STRAIGHT_OK && s[1]>TH_STRAIGHT_OK && s[2]>TH_STRAIGHT_OK && s[3]>TH_STRAIGHT_OK && s[4]>TH_STRAIGHT_OK)
    {

	    motors_set_speeds(0, 0);
	    oled_clearPage(0);
	    oled_clearPage(1);
	    oled_putStringAt(0, 0, "Goal Reached");
	    _delay_ms(4000);
	    return true;
	    
	    
    }

    // Decide which direction to turn next
    char t = choose_turn(has_left, has_straight, has_right);

    rotate_robot(t);

    path_add(t);
    path_simplify();

    oled_clearPage(1);
    snprintf(status, sizeof(status), "Path: %s", path);
    oled_putStringAt(1, 0, status);





    // Goal check: Hard coded version simply counting turns
    
	//if (pathLen>26)
    //{
	//    motors_set_speeds(0, 0);
	//    oled_clearPage(0);
	//    oled_clearPage(1);
	//    oled_putStringAt(0, 0, "Goal Reached");
	//    _delay_ms(4000);
	//    return true;
    //}


    return false;
}

// Learn maze by following left hand rule and simplifying path until goal is reached

static void learn_maze(void)
{
    oled_clearPage(0);
    oled_clearPage(1);
    oled_putStringAt(0, 0, "Learning...");
    path_clear();

    while (!handle_node_learn())
    {
        // Keep exploring until goal is found (true gets returned)
    }
}

static void rerun_maze(void)
{
    char msg[24];
	
	
	// Reset robot
	oled_clearPage(0);
	oled_clearPage(1);
	oled_putStringAt(0, 0, "Reset to start...");
	oled_putStringAt(1, 0, path);

	_delay_ms(7000);

        // Replay stored path
        oled_clearPage(0);
        oled_clearPage(1);
        oled_putStringAt(0, 0, "Replaying...");
        oled_putStringAt(1, 0, path);

        for (uint8_t i = 0; i < pathLen; i++)
        {
            follow_segment_until_break();

            // Soft braking before turning
            motors_set_speeds(cfg.brake1Speed, cfg.brake1Speed);
            wait_ms(cfg.brake1Time_ms);
            motors_set_speeds(cfg.brake2Speed, cfg.brake2Speed);
            wait_ms(cfg.brake2Time_ms);

            rotate_robot(path[i]);
        }

        // Final segment into goal
        follow_segment_until_break();
        motors_set_speeds(0, 0);

        oled_clearPage(0);
        oled_clearPage(1);
        snprintf(msg, sizeof(msg), "Done!");
        oled_putStringAt(0, 0, msg);
    }
//}

/* ---------- Main ---------- */

int main(void)
{
    // Initialize hardware, motors, OLED, etc.
    line_sensors_init();
    motors_init(400);       // Timer1, TOP=400
    load_default_config();
    oled_startup();
    _delay_ms(300);

    // Calibrate line sensors
    line_calibrate_reset();
    for (uint16_t i = 0; i < 80; i++)
    {
        if ((i % 8) == 0)
        {
            oled_clearPage(1);
            oled_putStringAt(1, 0, "Calibrating");
        }

        if (i > 20 && i <= 60)
            motors_set_speeds(-(int16_t)cfg.calibSpeed, (int16_t)cfg.calibSpeed);
        else
            motors_set_speeds((int16_t)cfg.calibSpeed, -(int16_t)cfg.calibSpeed);

        line_calibrate(LS_MODE_On);
        _delay_ms(10);
    }
	
	// End of calibration
    motors_set_speeds(0, 0);

    // Learn the maze and store the simplified path
    learn_maze();
	
	// Place robot back at beginning and rerun
    rerun_maze();

    while (1) { }
}

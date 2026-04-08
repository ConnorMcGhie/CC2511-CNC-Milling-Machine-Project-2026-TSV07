/**************************************************************
 * cnc_control.c
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#include "cnc_control.h"
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Spindle state
static uint spindle_slice;
static uint spindle_chan;

// Position tracking (steps)
static int32_t x_pos = 0;
static int32_t y_pos = 0;
static int32_t z_pos = 0;

/* ═══════════════════════════════════════════════════════════
 * INITIALISE HARDWARE
 * ═══════════════════════════════════════════════════════════ */

void init_axis(unsigned int step_pin, unsigned int dir_pin)
{
    gpio_init(step_pin);
    gpio_set_dir(step_pin, GPIO_OUT);
    gpio_put(step_pin, 0);

    gpio_init(dir_pin);
    gpio_set_dir(dir_pin, GPIO_OUT);
    gpio_put(dir_pin, 0);
}

void init_enable(void)
{
    gpio_init(EN_PIN);
    gpio_set_dir(EN_PIN, GPIO_OUT);
    gpio_put(EN_PIN, 1);
}

void init_spindle(void)
{
    gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    spindle_slice = pwm_gpio_to_slice_num(SPINDLE_PWM_PIN);
    spindle_chan = pwm_gpio_to_channel(SPINDLE_PWM_PIN);
    pwm_set_wrap(spindle_slice, SPINDLE_WRAP);
    pwm_set_chan_level(spindle_slice, spindle_chan, 0);
    pwm_set_enabled(spindle_slice, true);
}

/* ═══════════════════════════════════════════════════════════
 * INITIALISE MICROSTEPPING
 * ═══════════════════════════════════════════════════════════ */

// 200 steps/rev * 8 microsteps / 4mm lead = 400 steps/mm
void init_microstepping(void)
{
    // Initialise all MODE pins as outputs — start in 1/8 step mode
    gpio_init(X_MODE0_PIN);
    gpio_set_dir(X_MODE0_PIN, GPIO_OUT);
    gpio_init(X_MODE1_PIN);
    gpio_set_dir(X_MODE1_PIN, GPIO_OUT);
    gpio_init(X_MODE2_PIN);
    gpio_set_dir(X_MODE2_PIN, GPIO_OUT);

    gpio_init(Y_MODE0_PIN);
    gpio_set_dir(Y_MODE0_PIN, GPIO_OUT);
    gpio_init(Y_MODE1_PIN);
    gpio_set_dir(Y_MODE1_PIN, GPIO_OUT);
    gpio_init(Y_MODE2_PIN);
    gpio_set_dir(Y_MODE2_PIN, GPIO_OUT);

    // Default to full-step at startup; command mode will switch to 1/8
    set_xy_microstepping(false);
}

void set_xy_microstepping(bool micro)
{
    if (micro)
    {
        /* 1/8 step: MODE2=0, MODE1=1, MODE0=1 */
        gpio_put(X_MODE0_PIN, 1);
        gpio_put(X_MODE1_PIN, 1);
        gpio_put(X_MODE2_PIN, 0);

        gpio_put(Y_MODE0_PIN, 1);
        gpio_put(Y_MODE1_PIN, 1);
        gpio_put(Y_MODE2_PIN, 0);
    }
    else
    {
        /* Full step: MODE2=0, MODE1=0, MODE0=0 */
        gpio_put(X_MODE0_PIN, 0);
        gpio_put(X_MODE1_PIN, 0);
        gpio_put(X_MODE2_PIN, 0);

        gpio_put(Y_MODE0_PIN, 0);
        gpio_put(Y_MODE1_PIN, 0);
        gpio_put(Y_MODE2_PIN, 0);
    }
}

/* ═══════════════════════════════════════════════════════════
 * SINGLE-AXIS MOVE (with position tracking)
 * ═══════════════════════════════════════════════════════════ */

void move(unsigned int step_pin, unsigned int dir_pin, bool forward, int steps)
{
    /* EN_PIN is active-low on the DRV8825 — pulling it LOW enables the driver */
    gpio_put(EN_PIN, 0);
    sleep_us(DIR_SETUP_US);

    /* Set direction pin, then wait for the DRV8825's minimum DIR-to-STEP
     * setup time before the first pulse */
    gpio_put(dir_pin, forward);
    sleep_us(DIR_SETUP_US);

    for (int i = 0; i < steps; i++)
    {
        /* Each step is a high pulse followed by a low dwell.
         * STEP_HIGH_US satisfies the DRV8825's minimum pulse width (1.9 µs).
         * STEP_LOW_US sets the inter-step gap, which controls motor speed. */
        gpio_put(step_pin, 1);
        sleep_us(STEP_HIGH_US);
        gpio_put(step_pin, 0);
        sleep_us(STEP_LOW_US);
    }

    /* Disable drivers after the move to reduce heat and holding current */
    gpio_put(EN_PIN, 1);

    /* Update position tracking.
     * In manual mode X/Y are in full-step so each step = 8 microsteps.
     * Multiply the delta by 8 so the mm display stays consistent with
     * the STEPS_PER_MM (400) constant used everywhere else.            */
    int delta = forward ? steps : -steps;
    if (step_pin == X_STEP_PIN)
        x_pos += delta * 8;
    else if (step_pin == Y_STEP_PIN)
        y_pos += delta * 8;
    else if (step_pin == Z_STEP_PIN)
        z_pos += delta;
}

/* ═══════════════════════════════════════════════════════════
 * SPINDLE CONTROL
 * ═══════════════════════════════════════════════════════════ */

void spindle_set(int percent)
{
    if (percent < 0)
        percent = 0;
    if (percent > 100)
        percent = 100;
    int level = (percent * SPINDLE_WRAP) / 100;
    pwm_set_chan_level(spindle_slice, spindle_chan, level);
}

/* ═══════════════════════════════════════════════════════════
 * POSITION QUERIES
 * ═══════════════════════════════════════════════════════════ */

int32_t get_x_steps(void) { return x_pos; }
int32_t get_y_steps(void) { return y_pos; }
int32_t get_z_steps(void) { return z_pos; }

float get_x_mm(void) { return (float)x_pos / STEPS_PER_MM; }
float get_y_mm(void) { return (float)y_pos / STEPS_PER_MM; }
float get_z_mm(void) { return (float)z_pos / STEPS_PER_MM; }

/* ═══════════════════════════════════════════════════════════
 * SET ZERO (mark current position as origin)
 * ═══════════════════════════════════════════════════════════ */

void cnc_set_zero(void)
{
    x_pos = 0;
    y_pos = 0;
    z_pos = 0;
}

/* ═══════════════════════════════════════════════════════════
 * BRESENHAM INTERPOLATED MULTI-AXIS MOVE
 * ═══════════════════════════════════════════════════════════ */
void cnc_move_to(float x_mm, float y_mm, float z_mm, uint32_t step_delay_us)
{
    /* Convert the absolute mm target into a relative step count for each axis.
     * NAN is used as a sentinel meaning "don't move this axis". */
    int dx = 0, dy = 0, dz = 0;
    if (!isnan(x_mm))
        dx = (int)((x_mm - get_x_mm()) * STEPS_PER_MM);
    if (!isnan(y_mm))
        dy = (int)((y_mm - get_y_mm()) * STEPS_PER_MM);
    if (!isnan(z_mm))
        dz = (int)((z_mm - get_z_mm()) * STEPS_PER_MM);

    int abs_dx = abs(dx);
    int abs_dy = abs(dy);
    int abs_dz = abs(dz);

    /* The Bresenham algorithm iterates once per step of the dominant (longest)
     * axis. Shorter axes are only stepped when their accumulated error exceeds
     * the threshold, which keeps all axes proportional to the dominant one and
     * produces a straight line in 3D space. */
    int total = abs_dx;
    if (abs_dy > total)
        total = abs_dy;
    if (abs_dz > total)
        total = abs_dz;
    if (total == 0)
        return; /* Already at target — nothing to do */

    /* Set direction pins before enabling the drivers.
     * DIR_SETUP_US gives the DRV8825 time to latch each direction change. */
    if (abs_dx > 0)
    {
        gpio_put(X_DIR_PIN, dx > 0);
        sleep_us(DIR_SETUP_US);
    }
    if (abs_dy > 0)
    {
        gpio_put(Y_DIR_PIN, dy > 0);
        sleep_us(DIR_SETUP_US);
    }
    if (abs_dz > 0)
    {
        gpio_put(Z_DIR_PIN, dz > 0);
        sleep_us(DIR_SETUP_US);
    }

    /* Enable all three drivers together so the axes start simultaneously */
    gpio_put(EN_PIN, 0);
    sleep_us(DIR_SETUP_US);

    /* Initialise Bresenham error accumulators at total/2 to centre the
     * rounding error — this gives the most evenly distributed step pattern
     * rather than biasing all the "extra" steps toward the start or end. */
    int err_x = total / 2;
    int err_y = total / 2;
    int err_z = total / 2;

    for (int i = 0; i < total; i++)
    {
        /* For each axis, accumulate its step count.  When the running sum
         * reaches or exceeds 'total', it's time to take a step on that axis
         * and subtract 'total' to carry the remainder into the next iteration. */
        bool sx = false, sy = false, sz = false;

        err_x += abs_dx;
        if (err_x >= total)
        {
            err_x -= total;
            sx = true;
        }

        err_y += abs_dy;
        if (err_y >= total)
        {
            err_y -= total;
            sy = true;
        }

        err_z += abs_dz;
        if (err_z >= total)
        {
            err_z -= total;
            sz = true;
        }

        /* Assert STEP pins for all active axes at the same time so they
         * physically move in parallel rather than sequentially */
        if (sx)
            gpio_put(X_STEP_PIN, 1);
        if (sy)
            gpio_put(Y_STEP_PIN, 1);
        if (sz)
            gpio_put(Z_STEP_PIN, 1);

        sleep_us(STEP_HIGH_US); /* Hold high long enough for the DRV8825 to register the pulse */

        if (sx)
            gpio_put(X_STEP_PIN, 0);
        if (sy)
            gpio_put(Y_STEP_PIN, 0);
        if (sz)
            gpio_put(Z_STEP_PIN, 0);

        sleep_us(step_delay_us); /* Inter-step delay — controls feedrate */

        /* Track position one microstep at a time in the correct direction */
        if (sx)
            x_pos += (dx > 0) ? 1 : -1;
        if (sy)
            y_pos += (dy > 0) ? 1 : -1;
        if (sz)
            z_pos += (dz > 0) ? 1 : -1;
    }

    /* Disable drivers after move to reduce heat and holding current */
    gpio_put(EN_PIN, 1);
}

/* ═══════════════════════════════════════════════════════════
 * HOME (return to zero using interpolated move)
 * ═══════════════════════════════════════════════════════════ */

void cnc_home(void)
{
    cnc_move_to(0.0f, 0.0f, 0.0f, STEP_LOW_FAST_US);
}

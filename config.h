/**************************************************************
 * config.h
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#ifndef CONFIG_H
#define CONFIG_H

/* ── Pin Definitions ──────────────────────────────────────── */
#define SERIAL_TX_PIN 0
#define SERIAL_RX_PIN 1

#define X_MODE0_PIN 2
#define X_MODE1_PIN 3
#define X_MODE2_PIN 4
#define X_STEP_PIN 5
#define X_DIR_PIN 6
#define X_FLT_PIN 7

#define Y_MODE0_PIN 8
#define Y_MODE1_PIN 9
#define Y_MODE2_PIN 10
#define Y_STEP_PIN 11
#define Y_DIR_PIN 12
#define Y_FLT_PIN 13

#define Z_STEP_PIN 14
#define Z_DIR_PIN 15
#define Z_FLT_PIN 16

#define SPINDLE_PWM_PIN 17
#define EN_PIN 22

#define GPIO19_PIN 19
#define GPIO20_PIN 20
#define GPIO21_PIN 21
#define GPIO22_PIN 22
#define GPIO26_PIN 26
#define GPIO27_PIN 27
#define GPIO28_PIN 28

/* ── Step Timing (microseconds) ───────────────────────────── */
#define STEP_HIGH_US 20
#define STEP_LOW_US 2000
#define DIR_SETUP_US 5

/* ── Jog Size ─────────────────────────────────────────────── */
#define JOG_STEPS 50

/* ── Spindle PWM ──────────────────────────────────────────── */
#define SPINDLE_WRAP 1000

/* ── Machine Calibration (Vevor CNC 3018 Pro) ─────────────── */
/* T8 leadscrew, 4mm lead, 200 steps/rev, 1/8 microstepping   */
/* 200 * 8 / 4 = 400 steps per mm                             */
#define STEPS_PER_MM 400.0f

/* Working area limits (mm) */
#define X_MAX_MM 300.0f
#define Y_MAX_MM 180.0f
#define Z_MAX_MM 45.0f

/* ── Arc Interpolation ────────────────────────────────────── */
/* Angular step size for G2/G3 arc segmentation (degrees).    */
/* Smaller = smoother arcs, more segments, slower execution.  */
#define ARC_STEP_DEG 1.0f

#endif // CONFIG_H

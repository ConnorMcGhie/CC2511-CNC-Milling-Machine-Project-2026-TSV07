/**************************************************************
 * cnc_control.h
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#ifndef CNC_CONTROL_H
#define CNC_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* ── Hardware init ────────────────────────────────────────── */
void init_axis(unsigned int step_pin, unsigned int dir_pin);
void init_enable(void);
void init_spindle(void);
void init_microstepping(void);

/**
 * @brief Switch X/Y microstepping mode on the fly.
 *
 * @param micro  true  → 1/8 step  (use before all G-code moves)
 *               false → full step (use before manual jogs)
 *
 * Z axis has no MODE pins and is unaffected.
 */
void set_xy_microstepping(bool micro);

/* ── Single-axis move (updates position tracking) ─────────── */
void move(unsigned int step_pin, unsigned int dir_pin, bool forward, int steps);

/* ── Spindle control ──────────────────────────────────────── */
void spindle_set(int percent);

/* ── Position tracking ────────────────────────────────────── */
int32_t get_x_steps(void);
int32_t get_y_steps(void);
int32_t get_z_steps(void);
float get_x_mm(void);
float get_y_mm(void);
float get_z_mm(void);

/* ── Set current position as origin (zero all counters) ───── */
void cnc_set_zero(void);

/* ── Move all axes to zero simultaneously (Bresenham) ─────── */
void cnc_home(void);

/* ── Bresenham interpolated multi-axis move (absolute mm) ─── */
/* Pass NAN for any axis you don't want to move                */
void cnc_move_to(float x_mm, float y_mm, float z_mm, uint32_t step_delay_us);

#endif // CNC_CONTROL_H

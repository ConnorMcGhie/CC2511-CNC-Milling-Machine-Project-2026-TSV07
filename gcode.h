/**************************************************************
 * gcode.h
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#ifndef GCODE_H
#define GCODE_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file gcode.h
 * @brief Minimal G-code parser and executor.
 *
 * Supported codes:
 *   G0  Xn Yn Zn          – Rapid move to absolute position (mm)
 *   G1  Xn Yn Zn Fn       – Linear move at feedrate F (mm/min)
 *   G2  Xn Yn In Jn Fn    – Clockwise arc (XY plane), I/J offsets from start
 *   G3  Xn Yn In Jn Fn    – Counter-clockwise arc (XY plane), I/J offsets
 *   G4  Pn                – Dwell for P milliseconds
 *   G28                   – Return to home (origin)
 *   G28.1                 – Set current position as home (zero all axes)
 *   M3  Sn                – Spindle on at S percent (default 100%)
 *   M5                    – Spindle off
 *   M114                  – Report current position
 *
 * Words X, Y, Z are optional on G0/G1 — omit any axis to leave it stationary.
 * Comments after ';' or inside '(' ')' are stripped before parsing.
 *
 * Multiple codes can be chained on one line, e.g.:
 *   G28.1 G0 X10 Y20 G2 X10 Y20 I5 J0 F300 M5
 */

/**
 * @brief Result codes returned by gcode_execute() and gcode_execute_line().
 */
typedef enum
{
    GCODE_OK = 0,        /* Command executed successfully          */
    GCODE_EMPTY,         /* Line was blank or comment-only         */
    GCODE_UNKNOWN,       /* Unrecognised G/M code                  */
    GCODE_MISSING_COORD, /* G0/G1 with no X/Y/Z words              */
} gcode_result_t;

/**
 * @brief Parse and execute a single G-code command (one G or M code).
 *
 * @param line       Null-terminated string, e.g. "G1 X10.5 Y-3.2 F600"
 * @param status     Output buffer for a human-readable result message.
 * @param status_len Size of the status buffer.
 * @return           gcode_result_t indicating success or failure reason.
 */
gcode_result_t gcode_execute(const char *line, char *status, int status_len);

/**
 * @brief Split a line on G/M code boundaries and execute each chunk.
 *        Stops and returns on the first error.
 *        Status is set to the last executed command's message, or the
 *        error message of the failing command.
 *
 * @param line       Null-terminated string, e.g. "G28 G0 X10 Y20 M3 S50"
 * @param status     Output buffer for a human-readable result message.
 * @param status_len Size of the status buffer.
 * @return           GCODE_OK if all commands succeeded, otherwise the
 *                   result code of the first failing command.
 */
gcode_result_t gcode_execute_line(const char *line, char *status, int status_len);

#endif // GCODE_H

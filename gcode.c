/**************************************************************
 * gcode.c
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#include "gcode.h"
#include "cnc_control.h"
#include "config.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

/* ── Internal: strip comments and normalise whitespace ───────
 * Removes:  ; line comments
 *           ( block comments )
 * Result is written into dst (max dst_len bytes, null-terminated).
 * ─────────────────────────────────────────────────────────── */
static void strip_comments(const char *src, char *dst, int dst_len)
{
    int d = 0;
    bool in_block = false;

    for (int s = 0; src[s] != '\0' && d < dst_len - 1; s++)
    {
        char c = src[s];

        if (in_block)
        {
            if (c == ')')
                in_block = false;
            continue;
        }
        if (c == '(')
        {
            in_block = true;
            continue;
        }
        if (c == ';')
            break; /* rest of line is a comment */

        dst[d++] = c;
    }
    dst[d] = '\0';
}

/* ── Internal: find word value ───────────────────────────────
 * Searches the cleaned line for letter 'word' (e.g. 'X') and
 * returns its numeric value via *out_val.
 * Returns true if the word was found, false otherwise.
 * ─────────────────────────────────────────────────────────── */
static bool find_word(const char *line, char word, float *out_val)
{
    char upper = (char)toupper((unsigned char)word);
    for (int i = 0; line[i] != '\0'; i++)
    {
        if (toupper((unsigned char)line[i]) == upper)
        {
            /* Guard against matching letters that appear inside a number,
             * e.g. the 'E' in scientific notation like "1E5".  If the
             * character before this letter is a digit or '.', it's part of
             * a number and not a G-code word. */
            if (i > 0 && (isdigit((unsigned char)line[i - 1]) || line[i - 1] == '.'))
                continue;

            /* strtof will parse as far as it can and leave 'end' pointing at
             * the first character it couldn't consume.  If end == start, no
             * number was found at all (e.g. a bare 'X' with no value). */
            char *end;
            float val = strtof(&line[i + 1], &end);
            if (end != &line[i + 1])
            { /* strtof consumed at least one char */
                *out_val = val;
                return true;
            }
        }
    }
    return false;
}

/* ── Internal: feedrate → step delay ────────────────────────
 * Converts F word (mm/min) to a step_delay_us value for
 * cnc_move_to().  Clamps to a safe minimum.
 * ─────────────────────────────────────────────────────────── */
static uint32_t feedrate_to_delay(float f_mm_per_min)
{
    if (f_mm_per_min <= 0.0f)
        return STEP_LOW_US; /* Fall back to default jog speed for invalid feedrates */

    /* Derivation:
     *   steps/min  = mm/min × steps/mm
     *   period(µs) = 60,000,000 µs/min ÷ steps/min
     * This gives the time between step pulses needed to achieve the
     * requested feedrate. */
    float steps_per_min = f_mm_per_min * STEPS_PER_MM;
    float period_us = 60000000.0f / steps_per_min;

    /* Clamp to the minimum safe inter-step gap.
     * STEP_HIGH_US is the pulse width; 50 µs extra gives the driver
     * and motor sufficient low time before the next rising edge. */
    uint32_t min_delay = STEP_HIGH_US + 50;
    uint32_t delay = (uint32_t)period_us;
    return (delay < min_delay) ? min_delay : delay;
}

/* ═══════════════════════════════════════════════════════════
 * PUBLIC: gcode_execute
 * ═══════════════════════════════════════════════════════════ */

gcode_result_t gcode_execute(const char *line, char *status, int status_len)
{
    /* ── 1. Strip comments and check for empty line ─────── */
    char clean[128];
    strip_comments(line, clean, sizeof(clean));

    /* Trim leading whitespace */
    int start = 0;
    while (clean[start] == ' ' || clean[start] == '\t')
        start++;
    if (clean[start] == '\0')
    {
        snprintf(status, status_len, "Empty line.");
        return GCODE_EMPTY;
    }

    const char *tok = clean + start;

    /* ── 2. Extract G/M code number ─────────────────────── */
    char code_letter = (char)toupper((unsigned char)tok[0]);
    if (code_letter != 'G' && code_letter != 'M')
    {
        snprintf(status, status_len, "Unknown code: '%s'", tok);
        return GCODE_UNKNOWN;
    }

    /* Parse number after G/M — supports decimal (e.g. G28.1) */
    char *num_end;
    float code_num = strtof(tok + 1, &num_end);

    /* ── 3. Dispatch ─────────────────────────────────────── */

    /* ── G4 — Dwell ─────────────────────────────────────── */
    if (code_letter == 'G' && code_num == 4.0f)
    {
        float p;
        if (!find_word(tok, 'P', &p) || p < 0.0f)
        {
            snprintf(status, status_len, "G4: Missing or invalid P (ms).");
            return GCODE_MISSING_COORD;
        }
        sleep_ms((uint32_t)p);
        snprintf(status, status_len, "G4: Dwelled %.0f ms", p);
        return GCODE_OK;
    }

    /* ── G28.1 — Set current position as home ───────────── */
    if (code_letter == 'G' && code_num == 28.1f)
    {
        cnc_set_zero();
        snprintf(status, status_len,
                 "G28.1: Origin set — X=0.00  Y=0.00  Z=0.00");
        return GCODE_OK;
    }

    /* ── G28 — Return to home ───────────────────────────── */
    if (code_letter == 'G' && code_num == 28.0f)
    {
        snprintf(status, status_len, "G28: Homing...");
        cnc_home();
        snprintf(status, status_len,
                 "G28: Homed — X=%.2f  Y=%.2f  Z=%.2f",
                 get_x_mm(), get_y_mm(), get_z_mm());
        return GCODE_OK;
    }

    /* ── G0 — Rapid move (no feedrate) ─────────────────── */
    if (code_letter == 'G' && code_num == 0.0f)
    {
        float x, y, z;
        bool hx = find_word(tok, 'X', &x);
        bool hy = find_word(tok, 'Y', &y);
        bool hz = find_word(tok, 'Z', &z);

        if (!hx && !hy && !hz)
        {
            snprintf(status, status_len, "G0: No axis words (X/Y/Z) found.");
            return GCODE_MISSING_COORD;
        }

        cnc_move_to(hx ? x : NAN, hy ? y : NAN, hz ? z : NAN, STEP_LOW_FAST_US);
        snprintf(status, status_len,
                 "G0: Rapid -> X=%.2f  Y=%.2f  Z=%.2f",
                 get_x_mm(), get_y_mm(), get_z_mm());
        return GCODE_OK;
    }

    /* ── G1 — Linear move at feedrate ───────────────────── */
    if (code_letter == 'G' && code_num == 1.0f)
    {
        float x, y, z, f;
        bool hx = find_word(tok, 'X', &x);
        bool hy = find_word(tok, 'Y', &y);
        bool hz = find_word(tok, 'Z', &z);
        bool hf = find_word(tok, 'F', &f);

        if (!hx && !hy && !hz)
        {
            snprintf(status, status_len, "G1: No axis words (X/Y/Z) found.");
            return GCODE_MISSING_COORD;
        }

        uint32_t delay = hf ? feedrate_to_delay(f) : STEP_LOW_US;
        cnc_move_to(hx ? x : NAN, hy ? y : NAN, hz ? z : NAN, delay);
        snprintf(status, status_len,
                 "G1: Move -> X=%.2f  Y=%.2f  Z=%.2f  (F=%.0f mm/min)",
                 get_x_mm(), get_y_mm(), get_z_mm(), hf ? f : 0.0f);
        return GCODE_OK;
    }

    /* ── G2 / G3 — Circular interpolation (XY plane) ──────── */
    if (code_letter == 'G' && (code_num == 2.0f || code_num == 3.0f))
    {
        float x_end, y_end, i_off, j_off, f;
        bool hx = find_word(tok, 'X', &x_end);
        bool hy = find_word(tok, 'Y', &y_end);
        bool hi = find_word(tok, 'I', &i_off);
        bool hj = find_word(tok, 'J', &j_off);
        bool hf = find_word(tok, 'F', &f);

        if (!hi && !hj)
        {
            snprintf(status, status_len,
                     "G%d: Missing I/J centre offsets.", (int)code_num);
            return GCODE_MISSING_COORD;
        }

        /* Default missing endpoint to current position (full circle) */
        float x_start = get_x_mm();
        float y_start = get_y_mm();
        if (!hx)
            x_end = x_start;
        if (!hy)
            y_end = y_start;
        if (!hi)
            i_off = 0.0f;
        if (!hj)
            j_off = 0.0f;

        /* Centre of arc in absolute mm — I and J are offsets from the
         * current position (start point), not from the origin */
        float cx = x_start + i_off;
        float cy = y_start + j_off;

        /* Radius is the distance from the centre to the start point.
         * We use this same radius for every segment of the arc, which
         * means the end point must lie on the same circle.  A mismatch
         * here (e.g. due to floating-point rounding in the G-code file)
         * will cause the arc to snap slightly at the end — the final
         * cnc_move_to(x_end, y_end) corrects this. */
        float r = sqrtf(i_off * i_off + j_off * j_off);
        if (r < 0.001f)
        {
            snprintf(status, status_len, "G%d: Zero radius arc.", (int)code_num);
            return GCODE_MISSING_COORD;
        }

        /* atan2 returns angles in [-π, +π].  These are the angles from
         * the arc centre to the start and end points respectively. */
        float angle_start = atan2f(y_start - cy, x_start - cx);
        float angle_end = atan2f(y_end - cy, x_end - cx);

        /* Calculate the signed angular sweep from start to end.
         * G2 = clockwise = negative sweep (decreasing angle in standard maths).
         * G3 = counter-clockwise = positive sweep.
         * If the raw difference has the wrong sign we add or subtract a full
         * revolution (2π) to wrap it into the correct half of the number line. */
        float sweep;
        if (code_num == 2.0f)
        {
            /* CW: sweep must be negative */
            sweep = angle_end - angle_start;
            if (sweep > 0.0f)
                sweep -= 2.0f * (float)M_PI;
        }
        else
        {
            /* CCW: sweep must be positive */
            sweep = angle_end - angle_start;
            if (sweep < 0.0f)
                sweep += 2.0f * (float)M_PI;
        }

        /* If start == end the above gives sweep ≈ 0, but the intent is a
         * full circle — force the complete 360° sweep in the right direction */
        if (fabsf(sweep) < 0.0001f)
            sweep = (code_num == 2.0f) ? -2.0f * (float)M_PI : 2.0f * (float)M_PI;

        /* Divide the total sweep into fixed angular segments (ARC_STEP_DEG).
         * Each segment is a straight-line chord approximating the arc.
         * Smaller ARC_STEP_DEG = more segments = smoother curve but slower. */
        float step_rad = ARC_STEP_DEG * (float)M_PI / 180.0f;
        int segments = (int)(fabsf(sweep) / step_rad);
        if (segments < 1)
            segments = 1; /* Always produce at least one move */

        float angle_step = sweep / segments; /* Signed — preserves CW/CCW direction */
        uint32_t delay = hf ? feedrate_to_delay(f) : STEP_LOW_US;

        /* Walk around the arc, computing the XY coordinate of each chord
         * endpoint from the centre + radius + current angle */
        for (int seg = 1; seg <= segments; seg++)
        {
            float angle = angle_start + angle_step * seg;
            float nx = cx + r * cosf(angle);
            float ny = cy + r * sinf(angle);
            cnc_move_to(nx, ny, NAN, delay);
        }

        /* Final snap move corrects any accumulated floating-point error
         * and guarantees the tool lands exactly on the commanded endpoint */
        cnc_move_to(x_end, y_end, NAN, delay);

        snprintf(status, status_len,
                 "G%d: Arc done -> X=%.2f  Y=%.2f  R=%.2f  (%d segs)",
                 (int)code_num, get_x_mm(), get_y_mm(), r, segments);
        return GCODE_OK;
    }

    /* ── M114 — Report current position ─────────────────── */
    if (code_letter == 'M' && code_num == 114.0f)
    {
        snprintf(status, status_len,
                 "M114: X=%.2f  Y=%.2f  Z=%.2f mm",
                 get_x_mm(), get_y_mm(), get_z_mm());
        return GCODE_OK;
    }

    /* ── M3 — Spindle on ────────────────────────────────── */
    if (code_letter == 'M' && code_num == 3.0f)
    {
        float s;
        int percent = 100;
        if (find_word(tok, 'S', &s))
            percent = (int)s;
        spindle_set(percent);
        snprintf(status, status_len, "M3: Spindle on at %d%%", percent);
        return GCODE_OK;
    }

    /* ── M5 — Spindle off ───────────────────────────────── */
    if (code_letter == 'M' && code_num == 5.0f)
    {
        spindle_set(0);
        snprintf(status, status_len, "M5: Spindle off");
        return GCODE_OK;
    }

    /* ── Unrecognised ────────────────────────────────────── */
    snprintf(status, status_len,
             "Unknown: '%c%.4g'  (supported: G0 G1 G2 G3 G4 G28 G28.1 M3 M5 M114)", code_letter, code_num);
    return GCODE_UNKNOWN;
}

/* ═══════════════════════════════════════════════════════════
 * PUBLIC: gcode_execute_line
 * Splits a line into chunks on G/M boundaries, then executes
 * each chunk in sequence via gcode_execute().
 * Stops and returns on the first error.
 * ═══════════════════════════════════════════════════════════ */

gcode_result_t gcode_execute_line(const char *line, char *status, int status_len)
{
    /* Strip comments into a working buffer first */
    char clean[256];
    strip_comments(line, clean, sizeof(clean));

    int len = (int)strlen(clean);
    if (len == 0)
    {
        snprintf(status, status_len, "Empty line.");
        return GCODE_EMPTY;
    }

    gcode_result_t last_result = GCODE_EMPTY;

    /* Walk the cleaned line looking for G or M characters that mark the
     * start of a new command.  Each time we find one (or reach end-of-string)
     * we flush the previous chunk to gcode_execute().
     *
     * Example:  "G28.1 G0 X10 Y20 M3 S50"
     *   chunk 1 → "G28.1 "
     *   chunk 2 → "G0 X10 Y20 "
     *   chunk 3 → "M3 S50"
     */
    int chunk_start = -1; /* index of the current chunk's first char */

    for (int i = 0; i <= len; i++)
    {
        char c = (i < len) ? (char)toupper((unsigned char)clean[i]) : '\0';
        bool is_code_start = (c == 'G' || c == 'M');

        /* Also treat end-of-string as a boundary to flush the last chunk */
        bool is_boundary = is_code_start || (i == len);

        if (is_boundary && chunk_start >= 0)
        {
            /* Extract chunk [chunk_start, i) into a null-terminated buffer */
            int chunk_len = i - chunk_start;
            char chunk[128];
            if (chunk_len >= (int)sizeof(chunk))
                chunk_len = (int)sizeof(chunk) - 1;
            memcpy(chunk, clean + chunk_start, chunk_len);
            chunk[chunk_len] = '\0';

            /* Skip chunks that are only whitespace */
            bool has_content = false;
            for (int j = 0; j < chunk_len; j++)
            {
                if (chunk[j] != ' ' && chunk[j] != '\t')
                {
                    has_content = true;
                    break;
                }
            }

            if (has_content)
            {
                last_result = gcode_execute(chunk, status, status_len);
                if (last_result != GCODE_OK && last_result != GCODE_EMPTY)
                    return last_result; /* Stop on first error */
            }
        }

        if (is_code_start)
            chunk_start = i;
    }

    if (last_result == GCODE_EMPTY)
    {
        snprintf(status, status_len, "No valid G/M codes found.");
        return GCODE_EMPTY;
    }

    return GCODE_OK;
}
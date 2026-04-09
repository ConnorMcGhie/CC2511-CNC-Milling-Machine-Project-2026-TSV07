/**************************************************************
 * ui.c
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#include "ui.h"
#include "pico/stdlib.h"
#include "config.h"
#include "cnc_control.h"
#include "gcode.h"
#include <stdio.h>
#include <string.h>

/* ── ANSI Escape Helpers ──────────────────────────────────── */
#define ANSI_CLEAR "\033[2J"
#define ANSI_HOME "\033[H"
#define ANSI_MOVE(r, c) "\033[" #r ";" #c "H"
#define ANSI_CLEAR_LINE "\033[2K"
#define ANSI_BOLD "\033[1m"
#define ANSI_RESET "\033[0m"
#define ANSI_REVERSE "\033[7m"
#define ANSI_DIM "\033[2m"

/* ── Layout Row Constants ─────────────────────────────────── */
#define ROW_TOP 1
#define ROW_TITLE 2
#define ROW_DIV1 3
#define ROW_POS 4 /* Live position display */
#define ROW_DIV2 5
#define ROW_MODE_HDR 6
#define ROW_CTRL1 7
#define ROW_CTRL2 8
#define ROW_CTRL3 9
#define ROW_CTRL4 10
#define ROW_CTRL5 11 /* home/zero commands */
#define ROW_BOTTOM 12
#define ROW_BLANK 13
#define ROW_INPUT 14
#define ROW_BLANK2 15
#define ROW_STATUS 16

#define INPUT_COL 5 /* Column where user types after "  > " */

/* ── Command Buffer ───────────────────────────────────────── */
#define CMD_BUF_LEN 80

static cnc_mode_t current_mode = MODE_MANUAL;
static char cmd_buf[CMD_BUF_LEN];
static int cmd_len = 0;

/* Drain any pending characters from the input buffer to prevent overflow */
static void drain_input_buffer(void)
{
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
    {
        /* Discard character */
    }
}

/* ── Internal helpers ─────────────────────────────────────── */

/* Move cursor to a specific row/col (1-indexed, runtime values) */
static void cursor_move(int row, int col)
{
    printf("\033[%d;%dH", row, col);
}

/* ── Position Display ─────────────────────────────────────── */

/**
 * @brief Redraw the live position row without disturbing the cursor.
 *        Called after every movement to keep the display current.
 */
static void redraw_position(void)
{
    cursor_move(ROW_POS, 1);
    printf(ANSI_CLEAR_LINE);
    printf("  |" ANSI_BOLD "  X:" ANSI_RESET " %8.2f mm  " ANSI_BOLD "Y:" ANSI_RESET " %8.2f mm  " ANSI_BOLD "Z:" ANSI_RESET " %8.2f mm     |",
           get_x_mm(), get_y_mm(), get_z_mm());
    /* Return cursor to input line */
    cursor_move(ROW_INPUT, INPUT_COL + cmd_len);
    fflush(stdout);
}

/* ── Status Bar ───────────────────────────────────────────── */

void ui_set_status(const char *msg)
{
    cursor_move(ROW_STATUS, 1);
    printf(ANSI_CLEAR_LINE);
    printf(ANSI_REVERSE " STATUS " ANSI_RESET " %s", msg);
    cursor_move(ROW_INPUT, INPUT_COL + cmd_len);
    fflush(stdout);
}

/* ── Redraw Helpers ───────────────────────────────────────── */

static void redraw_mode_line(void)
{
    cursor_move(ROW_MODE_HDR, 1);
    printf(ANSI_CLEAR_LINE);
    if (current_mode == MODE_MANUAL)
        printf("  " ANSI_BOLD "[ MANUAL MODE ]" ANSI_RESET
               "  TAB = switch to Command Mode  (full-step, fast jog)");
    else
        printf("  " ANSI_BOLD "[ COMMAND MODE ]" ANSI_RESET
               "  TAB = switch to Manual Mode  (1/8-step, accurate)");
    fflush(stdout);
}

static void redraw_input_line(void)
{
    cursor_move(ROW_INPUT, 1);
    printf(ANSI_CLEAR_LINE);
    if (current_mode == MODE_MANUAL)
        printf("  > ");
    else
        printf("  > %.*s", cmd_len, cmd_buf);
    fflush(stdout);
}

/* ── Command Execution ────────────────────────────────────── */

static void execute_command(void)
{
    if (cmd_len == 0)
    {
        ui_set_status("No command entered.");
        return;
    }
    cmd_buf[cmd_len] = '\0';

    /* Switch to 1/8 microstepping for accurate G-code moves,
     * then restore full-step mode when done so the next manual
     * jog is still fast.  The mode is already COMMAND here, but
     * we call set_xy_microstepping(true) explicitly so that any
     * path that calls execute_command() is always correct.      */
    set_xy_microstepping(true);

    char status_msg[128];
    gcode_execute_line(cmd_buf, status_msg, sizeof(status_msg));

    /* Back to full-step so manual jogs are responsive again     */
    set_xy_microstepping(false);

    redraw_position();
    ui_set_status(status_msg);
}

/* ═══════════════════════════════════════════════════════════
 * PUBLIC API
 * ═══════════════════════════════════════════════════════════ */

void ui_init(void)
{
    /* Start in full-step (manual) mode */
    set_xy_microstepping(false);

    printf(ANSI_CLEAR ANSI_HOME);

    cursor_move(ROW_TOP, 1);
    printf("  +------------------------------------------------------+");
    cursor_move(ROW_TITLE, 1);
    printf("  |" ANSI_BOLD "            CNC MILL CONTROLLER v0.2                  " ANSI_RESET "|");
    cursor_move(ROW_DIV1, 1);
    printf("  +------------------------------------------------------+");

    /* Position display row */
    cursor_move(ROW_POS, 1);
    printf("  |" ANSI_BOLD "  X:" ANSI_RESET "     0.00 mm  " ANSI_BOLD "Y:" ANSI_RESET "     0.00 mm  " ANSI_BOLD "Z:" ANSI_RESET "     0.00 mm     |");

    cursor_move(ROW_DIV2, 1);
    printf("  +------------------------------------------------------+");
    cursor_move(ROW_CTRL1, 1);
    printf("  |  MANUAL CONTROLS:                                    |");
    cursor_move(ROW_CTRL2, 1);
    printf("  |  A/D = X Axis      W/S = Y Axis      Q/E = Z Axis   |");
    cursor_move(ROW_CTRL3, 1);
    printf("  |  0 = Spindle Off   1=25%%  2=50%%  3=75%%  4=100%%       |");
    cursor_move(ROW_CTRL4, 1);
    printf("  |  TAB = Toggle Command Mode                           |");
    cursor_move(ROW_CTRL5, 1);
    printf("  |  Codes: G0 G1 G2 G3 G4 G28 G28.1 M3 M5 M114        |");
    cursor_move(ROW_BOTTOM, 1);
    printf("  +------------------------------------------------------+");

    redraw_mode_line();
    redraw_input_line();
    ui_set_status("Ready. Jog to origin, then 'G28.1' to set home.");

    fflush(stdout);
}

cnc_mode_t ui_get_mode(void)
{
    return current_mode;
}

void ui_handle_input(int ch)
{
    /* TAB toggles mode */
    if (ch == '\t')
    {
        current_mode = (current_mode == MODE_MANUAL) ? MODE_COMMAND : MODE_MANUAL;
        cmd_len = 0;
        memset(cmd_buf, 0, sizeof(cmd_buf));

        if (current_mode == MODE_MANUAL)
        {
            /* Entering manual mode — switch to full-step for fast jogs */
            set_xy_microstepping(false);
            redraw_mode_line();
            redraw_input_line();
            ui_set_status("Switched to Manual Mode. (full-step)");
        }
        else
        {
            /* Entering command mode — switch to 1/8 step for accuracy.
             * set_xy_microstepping(true) is also called inside
             * execute_command() just before each move, so this is just
             * for consistency while the mode label shows on screen.    */
            set_xy_microstepping(true);
            redraw_mode_line();
            redraw_input_line();
            ui_set_status("Switched to Command Mode. (1/8-step) G0 G1 G2 G3 G4 G28 G28.1 M3 M5 M114");
        }
        return;
    }

    if (current_mode == MODE_MANUAL)
    {
        /* ── Manual mode: single keystroke actions (full-step) ── */
        switch (ch)
        {
        case 'd':
        case 'D':
            move(X_STEP_PIN, X_DIR_PIN, false, JOG_STEPS);
            drain_input_buffer();
            redraw_position();
            ui_set_status("X axis: decrement (X-)");
            break;
        case 'a':
        case 'A':
            move(X_STEP_PIN, X_DIR_PIN, true, JOG_STEPS);
            drain_input_buffer();
            redraw_position();
            ui_set_status("X axis: increment (X+)");
            break;
        case 's':
        case 'S':
            move(Y_STEP_PIN, Y_DIR_PIN, true, JOG_STEPS);
            drain_input_buffer();
            redraw_position();
            ui_set_status("Y axis: increment (Y+)");
            break;
        case 'w':
        case 'W':
            move(Y_STEP_PIN, Y_DIR_PIN, false, JOG_STEPS);
            drain_input_buffer();
            redraw_position();
            ui_set_status("Y axis: decrement (Y-)");
            break;
        case 'q':
        case 'Q':
            move(Z_STEP_PIN, Z_DIR_PIN, true, JOG_STEPS);
            drain_input_buffer();
            redraw_position();
            ui_set_status("Z axis: increment (Z+)");
            break;
        case 'e':
        case 'E':
            move(Z_STEP_PIN, Z_DIR_PIN, false, JOG_STEPS);

            redraw_position();
            ui_set_status("Z axis: decrement (Z-)");
            break;
        case '0':
            spindle_set(0);
            ui_set_status("Spindle: Off (0%)");
            break;
        case '1':
            spindle_set(25);
            ui_set_status("Spindle: 25%");
            break;
        case '2':
            spindle_set(50);
            ui_set_status("Spindle: 50%");
            break;
        case '3':
            spindle_set(75);
            ui_set_status("Spindle: 75%");
            break;
        case '4':
            spindle_set(100);
            ui_set_status("Spindle: 100%");
            break;
        default:
            break;
        }
    }
    else
    {
        /* ── Command mode: buffered line input (1/8-step) ── */
        if (ch == '\r' || ch == '\n')
        {
            execute_command();
            cmd_len = 0;
            memset(cmd_buf, 0, sizeof(cmd_buf));
            redraw_input_line();
        }
        else if ((ch == '\b' || ch == 127) && cmd_len > 0)
        {
            cmd_buf[--cmd_len] = '\0';
            redraw_input_line();
        }
        else if (ch >= 32 && ch < 127 && cmd_len < CMD_BUF_LEN - 1)
        {
            cmd_buf[cmd_len++] = (char)ch;
            cursor_move(ROW_INPUT, INPUT_COL + cmd_len);
            printf("%c", (char)ch);
            fflush(stdout);
        }
    }
}

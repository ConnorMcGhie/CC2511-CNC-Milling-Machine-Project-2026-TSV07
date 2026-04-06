/**************************************************************
 * ui.h
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#ifndef UI_H
#define UI_H

#include <stdbool.h>

/* Mode state */
typedef enum
{
    MODE_MANUAL,
    MODE_COMMAND
} cnc_mode_t;

/* Initialise and draw the static UI layout */
void ui_init(void);

/* Call each loop iteration with the latest character from getchar_timeout_us */
void ui_handle_input(int ch);

/* Update the status bar text at the bottom of the UI */
void ui_set_status(const char *msg);

/* Returns the current mode */
cnc_mode_t ui_get_mode(void);

#endif // UI_H

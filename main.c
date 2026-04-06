/**************************************************************
 * main.c
 * 2026 CNC Milling Machine Assignment (TSV07)
 * by Connor McGhie, Michael Rozis, Jed Fuller.
 * ***********************************************************/

// Include modules
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "config.h"
#include "cnc_control.h"
#include "ui.h"

int main(void)
{
    stdio_init_all();

    /* Hardware init can run immediately — no USB needed for this */
    init_enable();
    init_axis(X_STEP_PIN, X_DIR_PIN);
    init_axis(Y_STEP_PIN, Y_DIR_PIN);
    init_axis(Z_STEP_PIN, Z_DIR_PIN);
    init_spindle();
    init_microstepping();

    /* Block until PuTTY (or any USB CDC host) has fully enumerated.
     * Without this, early printf calls are lost before the connection
     * is established and the UI draws blank or corrupted. */
    while (!stdio_usb_connected())
    {
        sleep_ms(100);
    }

    /* Small settling delay — gives the terminal emulator time to
     * process the connection before we start sending ANSI escapes */
    sleep_ms(200);

    ui_init();

    while (true)
    {
        int ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT)
            continue;

        ui_handle_input(ch);
    }

    return 0;
}
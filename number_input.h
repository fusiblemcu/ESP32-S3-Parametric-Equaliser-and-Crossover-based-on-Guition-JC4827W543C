#ifndef NUMBER_INPUT_H
#define NUMBER_INPUT_H

#include <lvgl.h>

// Callback invoked when the user dismisses the popup.
//   value     : final numeric value (clamped to [min, max] if confirmed)
//   cancelled : true  if user tapped Cancel or backdrop
//               false if user tapped OK
//   user_data : opaque pointer passed to number_input_show()
//
// If cancelled is true, value is undefined — do not use it.
typedef void (*number_input_cb_t)(float value, bool cancelled, void *user_data);

// Show the popup keypad for numeric entry. Only one popup may be visible at
// a time — calling show while one is already up is a no-op (verify with
// number_input_is_visible() if you need to know).
//
//   title          : label shown above the value display (e.g. "HP Frequency")
//   initial        : starting value, will be clamped into [min, max]
//   min, max       : valid range, inclusive
//   decimals       : number of decimal places to display & accept (0–3)
//   allow_negative : show the +/- sign-toggle button
//   callback       : invoked once on dismiss
//   user_data      : passed through to callback
void number_input_show(const char *title, float initial, float min, float max,
                       int decimals, bool allow_negative,
                       number_input_cb_t callback, void *user_data);

// Force-hide the popup (e.g. on page change or app-level cancel).
// Invokes the callback with cancelled=true if currently visible.
void number_input_hide(void);

bool number_input_is_visible(void);

#endif

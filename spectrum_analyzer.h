#ifndef SPECTRUM_ANALYZER_H
#define SPECTRUM_ANALYZER_H

#include <lvgl.h>

#define NUM_SPECTRUM_BANDS 102

// Shared array populated by the DSP core (Core 1), read by UI core (Core 0)
// Magnitudes are expected in dB roughly from -90.0 to 0.0
extern float shared_spectrum_magnitudes[NUM_SPECTRUM_BANDS];

// Initialize the spectrum analyzer UI on a given parent (e.g., a tile view)
void spectrum_ui_create(lv_obj_t *parent);

// Pause or resume the spectrum animation timer
void spectrum_timer_set_paused(bool paused);

void spectrum_set_hidden(bool hidden);
void spectrum_set_mode(int mode);

#endif // SPECTRUM_ANALYZER_H

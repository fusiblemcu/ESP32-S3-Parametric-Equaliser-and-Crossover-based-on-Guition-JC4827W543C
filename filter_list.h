#ifndef FILTER_LIST_H
#define FILTER_LIST_H

// ============================================================
// Filter List View (Phase 12)
// ============================================================
// Modal overlay listing every band in an EQ stage, one row per band, with
// tap-to-edit for freq/gain/Q (via the existing number_input keypad) and
// inline controls for type-cycle / enable-toggle / delete. An "Add" button
// in the header appends a new band.
//
// Architecture:
//   - Single shared overlay built once at boot (filter_list_init).
//   - Persistent hidden; filter_list_open(stage) populates from `stage`
//     and shows, filter_list_close() hides. Contents are rebuilt on every
//     open — no need to refresh while hidden.
//   - Tapping a row's num cell selects that band (drives the same slider
//     state as the tab-bar). Tapping field buttons pops up number_input.
//   - All stage manipulation goes through the eq_ui_* stage-ops API
//     declared in eq_ui.h. This module owns NO band data and does NO DSP
//     pushes directly.
//
// All calls run on the LVGL UI thread (Core 1), so no locking is needed.
// ------------------------------------------------------------

#include <lvgl.h>
#include "eq_ui.h"

// Build the overlay + all row slots. Hidden until opened. Call once, after
// eq_ui_create has set up the EQ pages.
void filter_list_init(void);

// Populate rows from `stage` and show the overlay. The stage pointer is
// cached internally for the life of the open session and used by all row
// edit callbacks; do not free or re-init the stage while the list is open.
void filter_list_open(eq_stage_t *stage);

// Hide the overlay. Safe to call when already hidden. Does not rebuild.
void filter_list_close(void);

// True if the overlay is currently visible. Used by eq_ui to e.g. refresh
// a row when the number_input popup commits a new value.
bool filter_list_is_visible(void);

// Refresh one row's displayed values from its underlying band. Call after
// a field edit commits (typically from the number_input callback).
//  - No-op if the overlay is hidden or `idx` is not an active row.
void filter_list_refresh_row(int idx);

// Full rebuild: hide all rows, then repopulate from the currently open
// stage's bands. Call after add/delete/channel-mode-change that alters
// which bands are displayed.
//  - No-op if the overlay is hidden.
void filter_list_rebuild(void);

#endif

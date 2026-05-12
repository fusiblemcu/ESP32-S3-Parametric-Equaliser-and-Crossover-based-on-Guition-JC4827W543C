// number_input.cpp — popup numeric keypad for ParaEQ v2.
//
// Layout: half-screen panel docked to the LEFT of the screen (240×272), so the
// underlying page's right half (curve area) remains visible while typing.
// A semi-transparent backdrop covers ONLY the panel area; the right half of
// the screen stays interactive (intended for live-curve preview later).
// Tap on the backdrop = Cancel. Tap on the panel itself does nothing
// (clicks are consumed by panel's CLICKABLE flag, preventing bubble-up).
//
// Keypad: 4 rows × 4 cols.
//   Row 0:  7   8   9   <-
//   Row 1:  4   5   6   .
//   Row 2:  1   2   3   ±
//   Row 3:    Cancel    |    OK         (each spans 2 cols)
//
// Decimal key (col 3, row 1) and sign key (col 3, row 2) are always visible
// but become inert (greyed-out, non-clickable) when not enabled by config.
// This keeps button positions stable across calls — important for muscle
// memory across HP-freq, LP-freq, and delay popups.
//
// Out-of-range or unparseable values: display in red, OK button greyed and
// disabled. User must edit to a valid value before they can confirm.

#include "number_input.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#ifndef NAN
#define NAN __builtin_nanf("")
#endif

// Local theme — duplicated from eq_ui.cpp (only what's needed).
// If a third file ever needs these, factor into theme.h.
#define COL_BG          lv_color_hex(0x1A1A2E)
#define COL_PANEL       lv_color_hex(0x22223A)
#define COL_BOX         lv_color_hex(0x2A2A42)
#define COL_BOX_BORDER  lv_color_hex(0x3A3A5A)
#define COL_BOX_BDR_ACT lv_color_hex(0x7777CC)
#define COL_TEXT        lv_color_hex(0xCCCCDD)
#define COL_TEXT_DIM    lv_color_hex(0x888888)
#define COL_TEXT_RED    lv_color_hex(0xCC3333)
#define COL_OK_GREEN    lv_color_hex(0x33AA55)
#define COL_BLACK       lv_color_hex(0x000000)

// Panel geometry
#define PANEL_W         240
#define PANEL_H         272
#define PANEL_X         0
#define PANEL_Y         0

// Layout constants
#define KEY_PAD         4
#define KEY_ROWS        4    // 3 digit-rows + 1 row of OK/Cancel
#define KEY_COLS        4
#define VALUE_PAD_TOP   24   // Title + spacing
#define VALUE_H         34
#define KEYS_TOP        (VALUE_PAD_TOP + VALUE_H + 8)
#define KEYS_LEFT_PAD   6
#define KEYS_AVAIL_H    (PANEL_H - KEYS_TOP - 6)
#define KEY_H           ((KEYS_AVAIL_H - (KEY_ROWS - 1) * KEY_PAD) / KEY_ROWS)
#define KEY_W           ((PANEL_W - 2 * KEYS_LEFT_PAD - (KEY_COLS - 1) * KEY_PAD) / KEY_COLS)

#define VALUE_BUF_LEN   16

// Key codes for key_event_cb dispatch
#define KEY_DECIMAL     10
#define KEY_BACKSPACE   11
#define KEY_SIGN        12

// =========================================================
// Module state (one popup at a time)
// =========================================================
static lv_obj_t *backdrop      = NULL;
static lv_obj_t *panel         = NULL;
static lv_obj_t *title_label   = NULL;
static lv_obj_t *value_label   = NULL;
static lv_obj_t *ok_btn        = NULL;
static lv_obj_t *ok_label      = NULL;

static char     value_buf[VALUE_BUF_LEN];
static bool     is_negative      = false;
static bool     just_dismissed   = false;  // Suppress double-callback
static bool     is_showing_initial = false; // True between show() and first
                                            // digit/decimal keypress. While
                                            // true, the next digit/decimal
                                            // replaces the buffer instead of
                                            // appending — so the user can
                                            // type a new value without
                                            // backspacing first. Cleared by
                                            // any edit (digit/decimal clears
                                            // AND wipes buffer; backspace
                                            // and sign just clear the flag
                                            // and leave the buffer in place).
static int      cfg_decimals     = 0;
static float    cfg_min          = 0.0f;
static float    cfg_max          = 0.0f;
static bool     cfg_allow_negative = false;
static number_input_cb_t cfg_callback = NULL;
static void    *cfg_user_data    = NULL;

// Forward decls
static void refresh_display(void);
static void teardown(void);
static void dismiss_with(bool cancelled, float value);

// =========================================================
// Parsing & validation
// =========================================================

static float parse_current_value(void) {
    if (value_buf[0] == '\0' || strcmp(value_buf, ".") == 0) return NAN;
    float v = strtof(value_buf, NULL);
    if (is_negative) v = -v;
    return v;
}

static bool current_value_in_range(void) {
    float v = parse_current_value();
    if (isnan(v)) return false;
    return (v >= cfg_min && v <= cfg_max);
}

// =========================================================
// Entry editing
// =========================================================

static void append_digit(char d) {
    // First digit press after show() replaces the initial value instead of
    // appending to it. Sign is also reset so users typing a fresh positive
    // number after opening with a negative initial don't inherit the minus.
    if (is_showing_initial) {
        value_buf[0] = '\0';
        is_negative = false;
        is_showing_initial = false;
    }

    int len = strlen(value_buf);
    if (len >= VALUE_BUF_LEN - 2) return;

    // Decimal-places constraint
    char *dot = strchr(value_buf, '.');
    if (dot != NULL) {
        int after_dot = (int)((value_buf + len) - dot - 1);
        if (after_dot >= cfg_decimals) return;
    }
    value_buf[len]     = d;
    value_buf[len + 1] = '\0';
    refresh_display();
}

static void append_decimal(void) {
    if (cfg_decimals == 0) return;

    // First decimal press after show() replaces the initial value. E.g.
    // opening with "3" and tapping "." should give ".0" not "3.0".
    if (is_showing_initial) {
        value_buf[0] = '\0';
        is_negative = false;
        is_showing_initial = false;
    }

    if (strchr(value_buf, '.') != NULL) return;
    int len = strlen(value_buf);
    if (len == 0) {
        value_buf[0] = '0';
        value_buf[1] = '.';
        value_buf[2] = '\0';
    } else {
        if (len >= VALUE_BUF_LEN - 2) return;
        value_buf[len]     = '.';
        value_buf[len + 1] = '\0';
    }
    refresh_display();
}

static void backspace(void) {
    // Editing the existing value character-by-character — exit initial state
    // but don't wipe the buffer.
    is_showing_initial = false;
    int len = strlen(value_buf);
    if (len > 0) value_buf[len - 1] = '\0';
    refresh_display();
}

static void toggle_sign(void) {
    if (!cfg_allow_negative) return;
    if (is_showing_initial) {
        value_buf[0] = '\0';
        is_showing_initial = false;
    }
    is_negative = !is_negative;
    refresh_display();
}

// =========================================================
// Display refresh
// =========================================================

static void refresh_display(void) {
    static char display_buf[VALUE_BUF_LEN + 4];  // static per LVGL v9 trap
    if (value_buf[0] == '\0') {
        snprintf(display_buf, sizeof(display_buf), "%s_", is_negative ? "-" : "");
    } else {
        snprintf(display_buf, sizeof(display_buf), "%s%s", is_negative ? "-" : "", value_buf);
    }
    if (value_label) lv_label_set_text(value_label, display_buf);

    bool valid = current_value_in_range();
    if (value_label) {
        lv_obj_set_style_text_color(value_label, valid ? COL_TEXT : COL_TEXT_RED, 0);
    }
    if (ok_btn && ok_label) {
        if (valid) {
            lv_obj_set_style_bg_color(ok_btn, COL_OK_GREEN, 0);
            lv_obj_set_style_text_color(ok_label, COL_TEXT, 0);
            lv_obj_clear_state(ok_btn, LV_STATE_DISABLED);
        } else {
            lv_obj_set_style_bg_color(ok_btn, COL_BOX, 0);
            lv_obj_set_style_text_color(ok_label, COL_TEXT_DIM, 0);
            lv_obj_add_state(ok_btn, LV_STATE_DISABLED);
        }
    }
}

// =========================================================
// Dismissal
// =========================================================

static void teardown(void) {
    if (backdrop) {
        lv_obj_del(backdrop);  // Cascades to panel + all children
        backdrop    = NULL;
        panel       = NULL;
        title_label = NULL;
        value_label = NULL;
        ok_btn      = NULL;
        ok_label    = NULL;
    }
    cfg_callback   = NULL;
    cfg_user_data  = NULL;
    just_dismissed = false;
    is_showing_initial = false;
}

static void dismiss_with(bool cancelled, float value) {
    if (just_dismissed) return;  // Defensive
    just_dismissed = true;
    number_input_cb_t cb = cfg_callback;
    void *ud             = cfg_user_data;
    teardown();
    if (cb) cb(value, cancelled, ud);
}

// =========================================================
// Event handlers
// =========================================================

static void key_event_cb(lv_event_t *e) {
    int key = (int)(intptr_t)lv_event_get_user_data(e);
    switch (key) {
        case 0: case 1: case 2: case 3: case 4:
        case 5: case 6: case 7: case 8: case 9:
            append_digit('0' + key);
            break;
        case KEY_DECIMAL:   append_decimal(); break;
        case KEY_BACKSPACE: backspace();      break;
        case KEY_SIGN:      toggle_sign();    break;
    }
}

static void ok_event_cb(lv_event_t *e) {
    (void)e;
    if (!ok_btn || lv_obj_has_state(ok_btn, LV_STATE_DISABLED)) return;
    float v = parse_current_value();
    if (isnan(v)) return;
    if (v < cfg_min) v = cfg_min;
    if (v > cfg_max) v = cfg_max;
    dismiss_with(false, v);
}

static void cancel_event_cb(lv_event_t *e) {
    (void)e;
    dismiss_with(true, NAN);
}

static void backdrop_event_cb(lv_event_t *e) {
    (void)e;
    dismiss_with(true, NAN);
}

// =========================================================
// Construction
// =========================================================

// Create one keypad button at (col, row). When `active` is false the button
// is greyed out and not clickable, but still drawn — preserves grid layout
// across different (decimals, allow_negative) configurations.
static lv_obj_t *make_key(lv_obj_t *parent, const char *text, int code,
                          int col, int row, bool active) {
    lv_obj_t *btn = lv_obj_create(parent);
    lv_obj_remove_style_all(btn);
    lv_obj_set_size(btn, KEY_W, KEY_H);
    lv_obj_set_pos(btn,
                   KEYS_LEFT_PAD + col * (KEY_W + KEY_PAD),
                   KEYS_TOP      + row * (KEY_H + KEY_PAD));
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(btn, active ? COL_BOX : COL_PANEL, 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_border_color(btn, active ? COL_BOX_BORDER : COL_BOX, 0);
    lv_obj_set_style_radius(btn, 3, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    if (active) {
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(btn, key_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)code);
    } else {
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
    }

    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, text);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(lbl, active ? COL_TEXT : COL_TEXT_DIM, 0);
    lv_obj_center(lbl);
    return btn;
}

static void build_ui(const char *title) {
    lv_obj_t *scr = lv_screen_active();

    // --- Backdrop covers panel area only ---
    backdrop = lv_obj_create(scr);
    lv_obj_remove_style_all(backdrop);
    lv_obj_set_size(backdrop, PANEL_W, PANEL_H);
    lv_obj_set_pos(backdrop, PANEL_X, PANEL_Y);
    lv_obj_set_style_bg_color(backdrop, COL_BLACK, 0);
    lv_obj_set_style_bg_opa(backdrop, LV_OPA_70, 0);
    lv_obj_add_flag(backdrop, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(backdrop, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(backdrop, backdrop_event_cb, LV_EVENT_CLICKED, NULL);

    // --- Panel (keypad container) ---
    panel = lv_obj_create(backdrop);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, PANEL_W, PANEL_H);
    lv_obj_set_pos(panel, 0, 0);
    lv_obj_set_style_bg_color(panel, COL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, COL_BOX_BDR_ACT, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    // Consume clicks on panel so they don't reach backdrop (cancel)
    lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);

    // --- Title ---
    title_label = lv_label_create(panel);
    lv_label_set_text(title_label, title ? title : "");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title_label, COL_TEXT, 0);
    lv_obj_set_pos(title_label, 8, 4);

    // --- Value display ---
    value_label = lv_label_create(panel);
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(value_label, COL_TEXT, 0);
    lv_obj_set_style_text_align(value_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_size(value_label, PANEL_W - 16, VALUE_H);
    lv_obj_set_pos(value_label, 8, VALUE_PAD_TOP);
    lv_obj_set_style_pad_right(value_label, 4, 0);

    // --- Keypad layout (4 cols × 3 digit rows + 1 OK/Cancel row) ---
    //   Row 0:  7  8  9  <-
    //   Row 1:  4  5  6  .
    //   Row 2:  1  2  3  ±
    //   Row 3:  0  [Cancel/OK spans cols 1-3]
    make_key(panel, "7",                      7,             0, 0, true);
    make_key(panel, "8",                      8,             1, 0, true);
    make_key(panel, "9",                      9,             2, 0, true);
    make_key(panel, LV_SYMBOL_BACKSPACE,      KEY_BACKSPACE, 3, 0, true);

    make_key(panel, "4",                      4,             0, 1, true);
    make_key(panel, "5",                      5,             1, 1, true);
    make_key(panel, "6",                      6,             2, 1, true);
    make_key(panel, ".",                      KEY_DECIMAL,   3, 1, cfg_decimals > 0);

    make_key(panel, "1",                      1,             0, 2, true);
    make_key(panel, "2",                      2,             1, 2, true);
    make_key(panel, "3",                      3,             2, 2, true);
    make_key(panel, "+/-",                    KEY_SIGN,      3, 2, cfg_allow_negative);

    // Row 3 — col 0 is "0", cols 1-3 split into Cancel | OK
    make_key(panel, "0",                      0,             0, 3, true);

    int row3_y     = KEYS_TOP + 3 * (KEY_H + KEY_PAD);
    int btn_left_x = KEYS_LEFT_PAD + 1 * (KEY_W + KEY_PAD);
    int btn_w      = (3 * KEY_W + 2 * KEY_PAD - KEY_PAD) / 2;

    // Cancel button
    lv_obj_t *cancel_btn = lv_obj_create(panel);
    lv_obj_remove_style_all(cancel_btn);
    lv_obj_set_size(cancel_btn, btn_w, KEY_H);
    lv_obj_set_pos(cancel_btn, btn_left_x, row3_y);
    lv_obj_set_style_bg_opa(cancel_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(cancel_btn, COL_BOX, 0);
    lv_obj_set_style_border_width(cancel_btn, 1, 0);
    lv_obj_set_style_border_color(cancel_btn, COL_TEXT_RED, 0);
    lv_obj_set_style_radius(cancel_btn, 3, 0);
    lv_obj_set_style_pad_all(cancel_btn, 0, 0);
    lv_obj_clear_flag(cancel_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(cancel_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(cancel_btn, cancel_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cl = lv_label_create(cancel_btn);
    lv_label_set_text(cl, "Cancel");
    lv_obj_set_style_text_font(cl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(cl, COL_TEXT, 0);
    lv_obj_center(cl);

    // OK button
    ok_btn = lv_obj_create(panel);
    lv_obj_remove_style_all(ok_btn);
    lv_obj_set_size(ok_btn, btn_w, KEY_H);
    lv_obj_set_pos(ok_btn, btn_left_x + btn_w + KEY_PAD, row3_y);
    lv_obj_set_style_bg_opa(ok_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(ok_btn, COL_OK_GREEN, 0);
    lv_obj_set_style_border_width(ok_btn, 1, 0);
    lv_obj_set_style_border_color(ok_btn, COL_BOX_BDR_ACT, 0);
    lv_obj_set_style_radius(ok_btn, 3, 0);
    lv_obj_set_style_pad_all(ok_btn, 0, 0);
    lv_obj_clear_flag(ok_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ok_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ok_btn, ok_event_cb, LV_EVENT_CLICKED, NULL);
    ok_label = lv_label_create(ok_btn);
    lv_label_set_text(ok_label, "OK");
    lv_obj_set_style_text_font(ok_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(ok_label, COL_TEXT, 0);
    lv_obj_center(ok_label);
}

// =========================================================
// Public API
// =========================================================

void number_input_show(const char *title, float initial, float min, float max,
                       int decimals, bool allow_negative,
                       number_input_cb_t callback, void *user_data) {
    if (backdrop != NULL) return;  // Already showing — no-op
    if (decimals < 0) decimals = 0;
    if (decimals > 3) decimals = 3;
    if (min > max) { float t = min; min = max; max = t; }

    cfg_min            = min;
    cfg_max            = max;
    cfg_decimals       = decimals;
    cfg_allow_negative = allow_negative;
    cfg_callback       = callback;
    cfg_user_data      = user_data;
    just_dismissed     = false;
    is_showing_initial = true;   // Next digit/decimal press wipes the buffer.

    float clamped = initial;
    if (clamped < min) clamped = min;
    if (clamped > max) clamped = max;
    is_negative = (clamped < 0.0f);

    char fmt[8];
    snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
    snprintf(value_buf, VALUE_BUF_LEN, fmt, fabsf(clamped));

    build_ui(title);
    refresh_display();
}

void number_input_hide(void) {
    if (backdrop == NULL) return;
    dismiss_with(true, NAN);
}

bool number_input_is_visible(void) {
    return backdrop != NULL;
}
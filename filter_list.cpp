// ============================================================
// Filter List View (Phase 12) — implementation
// ============================================================
// Modal overlay listing every band in the active EQ stage, with tap-to-
// edit fields and inline controls. See filter_list.h for API + design
// notes. All data manipulation routes through eq_ui.h's public stage-ops
// API; we never touch bands[] or the DSP directly.
// ------------------------------------------------------------

#include "filter_list.h"
#include "number_input.h"
#include "eq_data.h"
#include <stdio.h>

// --- Colors (mirror of eq_ui.cpp palette; kept local to avoid a shared
// colors header — there are only a handful of colors in play) ---
#define COL_BG            lv_color_hex(0x1A1A2E)
#define COL_PANEL         lv_color_hex(0x22223A)
#define COL_BOX           lv_color_hex(0x2A2A42)
#define COL_BOX_ACT       lv_color_hex(0x3A3A6E)
#define COL_BOX_BORDER    lv_color_hex(0x3A3A5A)
#define COL_BOX_BDR_ACT   lv_color_hex(0x7777CC)
#define COL_TEXT          lv_color_hex(0xCCCCDD)
#define COL_TEXT_DIM      lv_color_hex(0x888888)
#define COL_SLIDER_IND    lv_color_hex(0x4444AA)
#define COL_BLACK         lv_color_hex(0x000000)
#define COL_WHITE         lv_color_hex(0xFFFFFF)
#define COL_RED_DIM       lv_color_hex(0x661818)

// --- Overlay geometry ---
#define SCR_W          480
#define SCR_H          272
#define PANEL_W        440
#define PANEL_H        240
#define PANEL_X        ((SCR_W - PANEL_W) / 2)
#define PANEL_Y        ((SCR_H - PANEL_H) / 2)
#define HEADER_H       30
#define LIST_Y         HEADER_H
#define LIST_H         (PANEL_H - HEADER_H)
#define ROW_H          30
#define ROW_GAP        2

// --- Row field widths + x-offsets (inside row container) ---
// Row container width = PANEL_W - 4 (2px pad each side inside list) = 436
// Total field width must fit within that.
#define ROW_PAD_L      4
#define F_NUM_W        22
#define F_TYPE_W       40
#define F_FREQ_W       82
#define F_GAIN_W       70
#define F_Q_W          56
#define F_EN_W         44
#define F_DEL_W        28
#define F_GAP          4

#define F_NUM_X        (ROW_PAD_L)
#define F_TYPE_X       (F_NUM_X  + F_NUM_W  + F_GAP)
#define F_FREQ_X       (F_TYPE_X + F_TYPE_W + F_GAP)
#define F_GAIN_X       (F_FREQ_X + F_FREQ_W + F_GAP)
#define F_Q_X          (F_GAIN_X + F_GAIN_W + F_GAP)
#define F_EN_X         (F_Q_X    + F_Q_W    + F_GAP)
#define F_DEL_X        (F_EN_X   + F_EN_W   + F_GAP)

// ============================================================
// Row struct
// ============================================================
typedef struct {
  lv_obj_t *container;
  lv_obj_t *num_btn;
  lv_obj_t *num_label;
  lv_obj_t *type_btn;
  lv_obj_t *type_label;
  lv_obj_t *freq_btn;
  lv_obj_t *freq_label;
  lv_obj_t *gain_btn;
  lv_obj_t *gain_label;
  lv_obj_t *q_btn;
  lv_obj_t *q_label;
  lv_obj_t *enable_btn;
  lv_obj_t *enable_label;
  lv_obj_t *delete_btn;
  int band_idx;  // -1 when row is unused
} filter_list_row_t;

// ============================================================
// Module state
// ============================================================
static eq_stage_t   *active_stage = nullptr;
static lv_obj_t     *overlay       = nullptr;
static lv_obj_t     *bg_dim        = nullptr;
static lv_obj_t     *panel         = nullptr;
static lv_obj_t     *header        = nullptr;
static lv_obj_t     *title_label   = nullptr;
static lv_obj_t     *add_btn       = nullptr;
static lv_obj_t     *close_btn     = nullptr;
static lv_obj_t     *list_container = nullptr;
static lv_obj_t     *clear_all_btn  = nullptr;
static lv_timer_t   *clear_hold_timer  = nullptr;
static lv_timer_t   *clear_flash_timer = nullptr;
static bool          clear_flash_state = false;
static filter_list_row_t rows[MAX_BANDS];

// number_input commit context (only one popup can be visible at a time,
// so a single static context is sufficient).
typedef enum { FIELD_FREQ, FIELD_GAIN, FIELD_Q } field_kind_t;
typedef struct {
  int          band_idx;
  field_kind_t field;
} num_ctx_t;
static num_ctx_t num_ctx;

// ============================================================
// Forward declarations
// ============================================================
static void populate_row(filter_list_row_t *r, int band_idx);
static void style_row_selected(filter_list_row_t *r, bool selected);
static void style_row_enabled(filter_list_row_t *r, bool enabled);
static inline void row_hide(filter_list_row_t *r);
static inline void row_show(filter_list_row_t *r);

static void num_btn_cb(lv_event_t *e);
static void type_btn_cb(lv_event_t *e);
static void freq_btn_cb(lv_event_t *e);
static void gain_btn_cb(lv_event_t *e);
static void q_btn_cb(lv_event_t *e);
static void enable_btn_cb(lv_event_t *e);
static void delete_btn_cb(lv_event_t *e);

static void add_btn_cb(lv_event_t *e);
static void close_btn_cb(lv_event_t *e);
static void clear_all_btn_cb(lv_event_t *e);
static void bg_dim_cb(lv_event_t *e);

static void number_input_commit_cb(float value, bool cancelled, void *user_data);

// ============================================================
// Field formatters
// ============================================================
static void fmt_freq_full(char *buf, size_t len, float f) {
  if (f >= 10000.0f)      snprintf(buf, len, "%.0f kHz", f / 1000.0f);
  else if (f >= 1000.0f)  snprintf(buf, len, "%.2f kHz", f / 1000.0f);
  else if (f >= 100.0f)   snprintf(buf, len, "%.0f Hz",  f);
  else                    snprintf(buf, len, "%.1f Hz",  f);
}

static void fmt_gain_full(char *buf, size_t len, float g) {
  snprintf(buf, len, "%+.1f dB", g);
}

static void fmt_q_full(char *buf, size_t len, float q) {
  snprintf(buf, len, "%.2f", q);
}

static const char *type_str_short(filter_type_t t) {
  switch (t) {
    case FTYPE_PEAK:       return "PK";
    case FTYPE_LOW_SHELF:  return "LS";
    case FTYPE_HIGH_SHELF: return "HS";
  }
  return "?";
}

// ============================================================
// Button factory — builds a styled field button with label
// ============================================================
static lv_obj_t *make_field_btn(lv_obj_t *parent, int x, int w, int h,
                                const char *initial_text, lv_obj_t **label_out,
                                void *user_data, lv_event_cb_t cb) {
  lv_obj_t *btn = lv_obj_create(parent);
  lv_obj_remove_style_all(btn);
  lv_obj_set_pos(btn, x, (ROW_H - h) / 2);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(btn, COL_BOX, 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_border_color(btn, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(btn, 3, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, initial_text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_center(lbl);

  if (label_out) *label_out = lbl;
  return btn;
}

// ============================================================
// Row builder — called once per slot in filter_list_init
// ============================================================
static void build_row(filter_list_row_t *r, int slot_idx, lv_obj_t *parent) {
  r->band_idx = -1;  // unused

  // Row container — acts as a horizontal track holding absolute-positioned
  // field buttons. Flex would also work, but absolute positions are
  // pixel-predictable and trivial to tune.
  r->container = lv_obj_create(parent);
  lv_obj_remove_style_all(r->container);
  lv_obj_set_size(r->container, PANEL_W - 4, ROW_H);
  lv_obj_set_style_bg_opa(r->container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(r->container, 0, 0);
  lv_obj_clear_flag(r->container, LV_OBJ_FLAG_SCROLLABLE);
  // Hidden by default — filter_list_open will show slots that are in use.
  row_hide(r);

  // Field buttons. user_data on each button is the row pointer, so the
  // callback can read r->band_idx and figure out which band to touch.
  r->num_btn = make_field_btn(r->container, F_NUM_X, F_NUM_W, ROW_H - 4,
                              "1", &r->num_label, r, num_btn_cb);
  r->type_btn = make_field_btn(r->container, F_TYPE_X, F_TYPE_W, ROW_H - 4,
                               "PK", &r->type_label, r, type_btn_cb);
  r->freq_btn = make_field_btn(r->container, F_FREQ_X, F_FREQ_W, ROW_H - 4,
                               "1.00 kHz", &r->freq_label, r, freq_btn_cb);
  r->gain_btn = make_field_btn(r->container, F_GAIN_X, F_GAIN_W, ROW_H - 4,
                               "+0.0 dB", &r->gain_label, r, gain_btn_cb);
  r->q_btn    = make_field_btn(r->container, F_Q_X, F_Q_W, ROW_H - 4,
                               "1.00", &r->q_label, r, q_btn_cb);
  r->enable_btn = make_field_btn(r->container, F_EN_X, F_EN_W, ROW_H - 4,
                                 "ON", &r->enable_label, r, enable_btn_cb);

  // Delete button — plain "X" (hyphen-minus/x choice: "X" reads as delete
  // without relying on symbol fonts).
  r->delete_btn = lv_obj_create(r->container);
  lv_obj_remove_style_all(r->delete_btn);
  lv_obj_set_pos(r->delete_btn, F_DEL_X, (ROW_H - (ROW_H - 4)) / 2);
  lv_obj_set_size(r->delete_btn, F_DEL_W, ROW_H - 4);
  lv_obj_set_style_bg_opa(r->delete_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(r->delete_btn, COL_RED_DIM, 0);
  lv_obj_set_style_border_width(r->delete_btn, 1, 0);
  lv_obj_set_style_border_color(r->delete_btn, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(r->delete_btn, 3, 0);
  lv_obj_set_style_pad_all(r->delete_btn, 0, 0);
  lv_obj_clear_flag(r->delete_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(r->delete_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(r->delete_btn, delete_btn_cb, LV_EVENT_CLICKED, r);

  lv_obj_t *del_lbl = lv_label_create(r->delete_btn);
  lv_label_set_text(del_lbl, "X");
  lv_obj_set_style_text_font(del_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(del_lbl, COL_TEXT, 0);
  lv_obj_center(del_lbl);

  (void)slot_idx;  // unused; kept for future per-slot behaviour if needed
}

// ============================================================
// Row visibility helpers (paired HIDDEN + FLOATING so the parent's flex
// layout reclaims the vertical space of hidden rows — belt-and-braces;
// some LVGL v9 builds skip hidden children in flex, some don't).
// ============================================================
static inline void row_hide(filter_list_row_t *r) {
  lv_obj_add_flag(r->container, (lv_obj_flag_t)(LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_FLOATING));
}
static inline void row_show(filter_list_row_t *r) {
  lv_obj_clear_flag(r->container, (lv_obj_flag_t)(LV_OBJ_FLAG_HIDDEN | LV_OBJ_FLAG_FLOATING));
}

// ============================================================
// Row visual styling
// ============================================================
static void style_row_selected(filter_list_row_t *r, bool selected) {
  // The num button doubles as the "selected band" indicator: it lights up
  // with the band's own palette color when this row is the stage's selected
  // band. Keeps the list clean without needing a separate highlight bar.
  if (selected) {
    lv_color_t col = band_color(r->band_idx);
    lv_obj_set_style_bg_color(r->num_btn, col, 0);
    lv_obj_set_style_border_color(r->num_btn, COL_BOX_BDR_ACT, 0);
    lv_obj_set_style_text_color(r->num_label, band_color_text(r->band_idx), 0);
  } else {
    lv_color_t col = band_color(r->band_idx);
    lv_obj_set_style_bg_color(r->num_btn, COL_BOX, 0);
    lv_obj_set_style_border_color(r->num_btn, COL_BOX_BORDER, 0);
    lv_obj_set_style_text_color(r->num_label, col, 0);
  }
}

static void style_row_enabled(filter_list_row_t *r, bool enabled) {
  // Dim the value fields of a disabled band so the user can see at a
  // glance that it isn't contributing. Enable button itself flips to an
  // "OFF" state with accent styling.
  lv_color_t field_text = enabled ? COL_TEXT : COL_TEXT_DIM;
  lv_obj_set_style_text_color(r->type_label, field_text, 0);
  lv_obj_set_style_text_color(r->freq_label, field_text, 0);
  lv_obj_set_style_text_color(r->gain_label, field_text, 0);
  lv_obj_set_style_text_color(r->q_label,    field_text, 0);

  if (enabled) {
    lv_obj_set_style_bg_color(r->enable_btn, COL_SLIDER_IND, 0);
    lv_obj_set_style_text_color(r->enable_label, COL_WHITE, 0);
    lv_label_set_text(r->enable_label, "ON");
  } else {
    lv_obj_set_style_bg_color(r->enable_btn, COL_BOX, 0);
    lv_obj_set_style_text_color(r->enable_label, COL_TEXT_DIM, 0);
    lv_label_set_text(r->enable_label, "OFF");
  }
}

// ============================================================
// Populate a row from a band in the active stage
// ============================================================
static void populate_row(filter_list_row_t *r, int band_idx) {
  r->band_idx = band_idx;

  eq_band_t *bands = eq_ui_get_bands(active_stage);
  eq_band_t *b = &bands[band_idx];

  char buf[24];

  snprintf(buf, sizeof(buf), "%d", band_idx + 1);
  lv_label_set_text(r->num_label, buf);

  lv_label_set_text(r->type_label, type_str_short(b->type));

  fmt_freq_full(buf, sizeof(buf), b->freq);
  lv_label_set_text(r->freq_label, buf);

  fmt_gain_full(buf, sizeof(buf), b->gain);
  lv_label_set_text(r->gain_label, buf);

  fmt_q_full(buf, sizeof(buf), b->q);
  lv_label_set_text(r->q_label, buf);

  style_row_enabled(r, b->enabled);
  style_row_selected(r, active_stage->selected_band == band_idx);

  row_show(r);
}

// ============================================================
// Row event callbacks
// ============================================================
static void num_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;
  eq_ui_select_band(active_stage, r->band_idx);

  // Refresh all rows' selected-state — only one row can be selected at a time.
  int nb = eq_ui_get_band_count(active_stage);
  for (int i = 0; i < nb; i++) {
    style_row_selected(&rows[i], active_stage->selected_band == rows[i].band_idx);
  }
}

static void type_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[r->band_idx];
  // Free 3-way cycle: PK -> LS -> HS -> PK (per user spec, list acts as
  // power-user override — unlike the tab-bar PK/Shelf buttons which
  // enforce LS-on-band-0 / HS-on-others policy).
  switch (b->type) {
    case FTYPE_PEAK:       b->type = FTYPE_LOW_SHELF;  break;
    case FTYPE_LOW_SHELF:  b->type = FTYPE_HIGH_SHELF; break;
    case FTYPE_HIGH_SHELF: b->type = FTYPE_PEAK;       break;
  }
  lv_label_set_text(r->type_label, type_str_short(b->type));
  eq_ui_commit_band(active_stage, r->band_idx);
}

static void freq_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[r->band_idx];
  num_ctx.band_idx = r->band_idx;
  num_ctx.field    = FIELD_FREQ;
  number_input_show("Frequency (Hz)", b->freq, FREQ_MIN, FREQ_MAX,
                    1, false, number_input_commit_cb, &num_ctx);
}

static void gain_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[r->band_idx];
  num_ctx.band_idx = r->band_idx;
  num_ctx.field    = FIELD_GAIN;
  number_input_show("Gain (dB)", b->gain, GAIN_MIN, GAIN_MAX,
                    1, true, number_input_commit_cb, &num_ctx);
}

static void q_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[r->band_idx];
  num_ctx.band_idx = r->band_idx;
  num_ctx.field    = FIELD_Q;
  number_input_show("Q", b->q, Q_MIN, Q_MAX,
                    2, false, number_input_commit_cb, &num_ctx);
}

static void enable_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[r->band_idx];
  b->enabled = !b->enabled;
  style_row_enabled(r, b->enabled);
  eq_ui_commit_band(active_stage, r->band_idx);
}

static void delete_btn_cb(lv_event_t *e) {
  filter_list_row_t *r = (filter_list_row_t *)lv_event_get_user_data(e);
  if (!active_stage || r->band_idx < 0) return;

  // Guard: stage must keep >=1 band. The op itself no-ops at count==1
  // but checking here lets us avoid the redundant full rebuild.
  if (eq_ui_get_band_count(active_stage) <= 1) return;

  eq_ui_remove_band_at(active_stage, r->band_idx);
  filter_list_rebuild();
}

// ============================================================
// number_input commit — one callback for all three numeric fields
// ============================================================
static void number_input_commit_cb(float value, bool cancelled, void *user_data) {
  if (cancelled || !active_stage) return;
  num_ctx_t *ctx = (num_ctx_t *)user_data;
  if (ctx->band_idx < 0 || ctx->band_idx >= eq_ui_get_band_count(active_stage)) return;

  eq_band_t *b = &eq_ui_get_bands(active_stage)[ctx->band_idx];
  switch (ctx->field) {
    case FIELD_FREQ: b->freq = value; break;
    case FIELD_GAIN: b->gain = value; break;
    case FIELD_Q:    b->q    = value; break;
  }
  eq_ui_commit_band(active_stage, ctx->band_idx);
  filter_list_refresh_row(ctx->band_idx);
}

// ============================================================
// Header event callbacks
// ============================================================
static void clear_flash_tick(lv_timer_t *t) {
  (void)t;
  if (!clear_all_btn) return;
  clear_flash_state = !clear_flash_state;
  lv_obj_set_style_bg_color(clear_all_btn,
      clear_flash_state ? lv_color_hex(0xFFFFFF) : COL_BOX, 0);
}

static void clear_flash_stop(void) {
  if (clear_flash_timer) {
    lv_timer_del(clear_flash_timer);
    clear_flash_timer = nullptr;
  }
  clear_flash_state = false;
  if (clear_all_btn)
    lv_obj_set_style_bg_color(clear_all_btn, COL_BOX, 0);
}

static void clear_all_execute(lv_timer_t *t) {
  (void)t;
  clear_hold_timer = nullptr;
  clear_flash_stop();
  if (!active_stage) return;

  // Remove all bands except the first (remove from top to avoid index shift)
  while (eq_ui_get_band_count(active_stage) > 1)
    eq_ui_remove_band_at(active_stage,
                         eq_ui_get_band_count(active_stage) - 1);

  // Reset band 0 gain to 0 dB
  eq_band_t *bands = eq_ui_get_bands(active_stage);
  bands[0].gain = 0.0f;
  eq_ui_commit_band(active_stage, 0);

  filter_list_rebuild();
}

static void clear_all_btn_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_PRESSED) {
    // Start 3-second hold timer and rapid flash
    if (clear_hold_timer) lv_timer_del(clear_hold_timer);
    clear_hold_timer = lv_timer_create(clear_all_execute, 3000, nullptr);
    lv_timer_set_repeat_count(clear_hold_timer, 1);

    if (clear_flash_timer) lv_timer_del(clear_flash_timer);
    clear_flash_state = false;
    clear_flash_timer = lv_timer_create(clear_flash_tick, 120, nullptr);
  } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    if (clear_hold_timer) {
      lv_timer_del(clear_hold_timer);
      clear_hold_timer = nullptr;
    }
    clear_flash_stop();
  }
}

static void add_btn_cb(lv_event_t *e) {
  if (!active_stage) return;
  if (eq_ui_get_band_count(active_stage) >= MAX_BANDS) return;

  eq_ui_add_band(active_stage);
  filter_list_rebuild();
}

static void close_btn_cb(lv_event_t *e) {
  filter_list_close();
}

static void bg_dim_cb(lv_event_t *e) {
  // Tapping the dim backdrop dismisses the overlay. Panel clicks don't
  // reach here because the panel is a clickable sibling that absorbs its
  // own area.
  filter_list_close();
}

// ============================================================
// Public API
// ============================================================
void filter_list_init(void) {
  if (overlay) return;  // already built; idempotent

  lv_obj_t *scr = lv_screen_active();

  // --- Overlay (hidden container holding both backdrop and panel) ---
  overlay = lv_obj_create(scr);
  lv_obj_remove_style_all(overlay);
  lv_obj_set_size(overlay, SCR_W, SCR_H);
  lv_obj_set_pos(overlay, 0, 0);
  lv_obj_set_style_bg_opa(overlay, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(overlay, 0, 0);
  lv_obj_clear_flag(overlay, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

  // --- Backdrop (dim + click absorber) ---
  bg_dim = lv_obj_create(overlay);
  lv_obj_remove_style_all(bg_dim);
  lv_obj_set_size(bg_dim, SCR_W, SCR_H);
  lv_obj_set_pos(bg_dim, 0, 0);
  lv_obj_set_style_bg_color(bg_dim, COL_BLACK, 0);
  // Backdrop opacity: ~5% (raw 13 of 255). LV_OPA_10 is the nearest named
  // constant at ~10% — chose raw value to match the exact target. Tunable
  // if a stronger tint is wanted.
  lv_obj_set_style_bg_opa(bg_dim, (lv_opa_t)13, 0);
  lv_obj_set_style_pad_all(bg_dim, 0, 0);
  lv_obj_clear_flag(bg_dim, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(bg_dim, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(bg_dim, bg_dim_cb, LV_EVENT_CLICKED, nullptr);

  // --- Panel ---
  panel = lv_obj_create(overlay);
  lv_obj_remove_style_all(panel);
  lv_obj_set_size(panel, PANEL_W, PANEL_H);
  lv_obj_set_pos(panel, PANEL_X, PANEL_Y);
  lv_obj_set_style_bg_color(panel, COL_PANEL, 0);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(panel, 1, 0);
  lv_obj_set_style_border_color(panel, COL_BOX_BDR_ACT, 0);
  lv_obj_set_style_radius(panel, 6, 0);
  lv_obj_set_style_pad_all(panel, 0, 0);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  // Panel is clickable so taps inside it DON'T reach bg_dim and dismiss
  // the overlay. Its own click handler does nothing.
  lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);

  // --- Header ---
  header = lv_obj_create(panel);
  lv_obj_remove_style_all(header);
  lv_obj_set_size(header, PANEL_W, HEADER_H);
  lv_obj_set_pos(header, 0, 0);
  lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(header, 0, 0);
  lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

  title_label = lv_label_create(header);
  lv_label_set_text(title_label, "FILTERS");
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(title_label, COL_TEXT, 0);
  lv_obj_align(title_label, LV_ALIGN_CENTER, 0, 0);

  // Add button (top right area). Size = 10% larger than initial spec
  // (60x20 -> 66x22) per hardware touch-accuracy feedback.
  add_btn = lv_obj_create(header);
  lv_obj_remove_style_all(add_btn);
  lv_obj_set_size(add_btn, 66, 22);
  lv_obj_align(add_btn, LV_ALIGN_LEFT_MID, 8, 0);
  lv_obj_set_style_bg_opa(add_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(add_btn, COL_BOX, 0);
  lv_obj_set_style_border_width(add_btn, 1, 0);
  lv_obj_set_style_border_color(add_btn, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(add_btn, 3, 0);
  lv_obj_set_style_pad_all(add_btn, 0, 0);
  lv_obj_clear_flag(add_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(add_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(add_btn, add_btn_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *add_lbl = lv_label_create(add_btn);
  lv_label_set_text(add_lbl, "+ ADD");
  lv_obj_set_style_text_font(add_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(add_lbl, COL_TEXT, 0);
  lv_obj_center(add_lbl);

  // Close button (far right). Size = 10% larger than initial spec
  // (28x20 -> 31x22) per hardware touch-accuracy feedback.
  close_btn = lv_obj_create(header);
  lv_obj_remove_style_all(close_btn);
  lv_obj_set_size(close_btn, 31, 22);
  lv_obj_align(close_btn, LV_ALIGN_RIGHT_MID, -4, 0);
  lv_obj_set_style_bg_opa(close_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(close_btn, COL_BOX, 0);
  lv_obj_set_style_border_width(close_btn, 1, 0);
  lv_obj_set_style_border_color(close_btn, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(close_btn, 3, 0);
  lv_obj_set_style_pad_all(close_btn, 0, 0);
  lv_obj_clear_flag(close_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(close_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(close_btn, close_btn_cb, LV_EVENT_CLICKED, nullptr);
  lv_obj_t *close_lbl = lv_label_create(close_btn);
  lv_label_set_text(close_lbl, "X");
  lv_obj_set_style_text_font(close_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(close_lbl, COL_TEXT, 0);
  lv_obj_center(close_lbl);

  // CLR ALL button — hold 3 seconds to clear all bands and reset to one at 0dB.
  // Sits between the title and the X button.
  // close_btn left edge = PANEL_W - 4 - 31 = 405px; place this 4px to its left.
  // RIGHT_MID offset: -(4 + 31 + 4 + 44) = -83 puts right edge at PANEL_W - 83 + 44 = 401px.
  clear_all_btn = lv_obj_create(header);
  lv_obj_remove_style_all(clear_all_btn);
  lv_obj_set_size(clear_all_btn, 44, 22);
  lv_obj_align(clear_all_btn, LV_ALIGN_RIGHT_MID, -39, 0);
  lv_obj_set_style_bg_opa(clear_all_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(clear_all_btn, COL_BOX, 0);
  lv_obj_set_style_border_width(clear_all_btn, 1, 0);
  lv_obj_set_style_border_color(clear_all_btn, lv_color_hex(0x884444), 0);
  lv_obj_set_style_radius(clear_all_btn, 3, 0);
  lv_obj_set_style_pad_all(clear_all_btn, 0, 0);
  lv_obj_clear_flag(clear_all_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(clear_all_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(clear_all_btn, clear_all_btn_cb, LV_EVENT_PRESSED,   nullptr);
  lv_obj_add_event_cb(clear_all_btn, clear_all_btn_cb, LV_EVENT_RELEASED,  nullptr);
  lv_obj_add_event_cb(clear_all_btn, clear_all_btn_cb, LV_EVENT_PRESS_LOST, nullptr);
  lv_obj_t *clr_lbl = lv_label_create(clear_all_btn);
  lv_label_set_text(clr_lbl, "CLR ALL");
  lv_obj_set_style_text_font(clr_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(clr_lbl, lv_color_hex(0xFF8888), 0);
  lv_obj_center(clr_lbl);

  // --- List container (scrollable) ---
  list_container = lv_obj_create(panel);
  lv_obj_remove_style_all(list_container);
  lv_obj_set_size(list_container, PANEL_W, LIST_H);
  lv_obj_set_pos(list_container, 0, LIST_Y);
  lv_obj_set_style_bg_opa(list_container, LV_OPA_TRANSP, 0);
  lv_obj_set_style_pad_all(list_container, 2, 0);
  lv_obj_set_style_pad_top(list_container, 7, 0);  // +5px buffer above first row
  lv_obj_set_style_pad_row(list_container, ROW_GAP, 0);
  lv_obj_set_flex_flow(list_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(list_container, LV_FLEX_ALIGN_START,
                        LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_scroll_dir(list_container, LV_DIR_VER);
  lv_obj_add_flag(list_container, LV_OBJ_FLAG_SCROLLABLE);

  // Scrollbar: shown automatically when content exceeds the visible window
  // (>6 rows). Styled as a slim accent-colored indicator on the right edge.
  lv_obj_set_scrollbar_mode(list_container, LV_SCROLLBAR_MODE_AUTO);
  lv_obj_set_style_bg_color(list_container, COL_SLIDER_IND, LV_PART_SCROLLBAR);
  lv_obj_set_style_bg_opa(list_container,   LV_OPA_80,      LV_PART_SCROLLBAR);
  lv_obj_set_style_width(list_container,    4,              LV_PART_SCROLLBAR);
  lv_obj_set_style_radius(list_container,   2,              LV_PART_SCROLLBAR);
  lv_obj_set_style_pad_right(list_container, 2,             LV_PART_SCROLLBAR);

  // Pre-build all MAX_BANDS row slots, hidden until populated.
  for (int i = 0; i < MAX_BANDS; i++) {
    build_row(&rows[i], i, list_container);
  }
}

void filter_list_open(eq_stage_t *stage) {
  if (!overlay || !stage) return;
  active_stage = stage;

  // Update title to reflect which stage is open. stage->name is "INPUT" /
  // "HIGH OUT" / "LOW OUT" per init_stage_data().
  char title[32];
  snprintf(title, sizeof(title), "%s FILTERS", stage->name ? stage->name : "");
  lv_label_set_text(title_label, title);

  filter_list_rebuild();

  // Bring overlay to the front so it renders above any number_input popup
  // that might have been left parented on the screen, and above the
  // tileview. number_input opens will then be created AFTER the overlay
  // and will stack above it — verify on hardware per handover note.
  lv_obj_move_foreground(overlay);
  lv_obj_clear_flag(overlay, LV_OBJ_FLAG_HIDDEN);
}

void filter_list_close(void) {
  if (!overlay) return;
  // Cancel any in-progress CLR ALL hold and flash
  if (clear_hold_timer) {
    lv_timer_del(clear_hold_timer);
    clear_hold_timer = nullptr;
  }
  clear_flash_stop();
  // If the number_input popup is up on top of us, close it first to avoid
  // a stranded popup once we hide.
  if (number_input_is_visible()) number_input_hide();
  lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);
  active_stage = nullptr;
}

bool filter_list_is_visible(void) {
  if (!overlay) return false;
  return !lv_obj_has_flag(overlay, LV_OBJ_FLAG_HIDDEN);
}

void filter_list_refresh_row(int idx) {
  if (!filter_list_is_visible() || !active_stage) return;
  if (idx < 0 || idx >= MAX_BANDS) return;
  if (rows[idx].band_idx != idx) return;  // slot not currently showing this band
  populate_row(&rows[idx], idx);
}

void filter_list_rebuild(void) {
  if (!overlay || !active_stage) return;

  int nb = eq_ui_get_band_count(active_stage);

  // Hide the list container during rebuild to avoid any visible flash as
  // rows flip hidden/unhidden.
  lv_obj_add_flag(list_container, LV_OBJ_FLAG_HIDDEN);

  for (int i = 0; i < MAX_BANDS; i++) {
    if (i < nb) {
      populate_row(&rows[i], i);
    } else {
      rows[i].band_idx = -1;
      row_hide(&rows[i]);
    }
  }

  lv_obj_clear_flag(list_container, LV_OBJ_FLAG_HIDDEN);
  lv_obj_scroll_to_y(list_container, 0, LV_ANIM_OFF);
}
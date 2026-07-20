#include "eq_ui.h"
#include "boot_config.h"
#include "dsp_engine.h"
#include "filter_list.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s_input.h"
#include "number_input.h"
#include "preset_store.h"
#include "spectrum_analyzer.h"
#include "storage_backend.h"
#include <Arduino.h>
#include <esp_dsp.h>
#include <lvgl.h>
#include <math.h>
#include <stdio.h>

// ============================================================
// Layout constants
// ============================================================
#define SCREEN_W 480
#define SCREEN_H 272

#define CURVE_X 28
#define CURVE_Y 6
#define CURVE_W 400
#define CURVE_H 164

// Level meter layout
#define METER_AREA_X 0
#define METER_AREA_Y CURVE_Y
#define METER_AREA_W CURVE_X
#define METER_AREA_H CURVE_H

#define METER_BAR_W 3
#define METER_GAP 1
#define METER_BLOCK_W (METER_BAR_W + METER_GAP + METER_BAR_W) // 7px
#define METER_X_START ((METER_AREA_W - METER_BLOCK_W) / 2)    // 10px
#define METER_X_L METER_X_START
#define METER_X_R (METER_X_START + METER_BAR_W + METER_GAP) // 14px

// Meter dB thresholds and pixel positions (pre-calculated for CURVE_H=164)
#define METER_FLOOR_DB (-48.0f)
#define METER_DB_20 (-20.0f)
#define METER_DB_16 (-16.0f)
// Pixel heights from bottom: linear mapping (db - floor) / (0 - floor) * height
// At -20dB: (28/48) * 164 = 95.67 -> 96px
// At -16dB: (32/48) * 164 = 109.33 -> 109px
#define METER_PX_20 96
#define METER_PX_16 109

// Slider area
#define SL_AREA_Y (CURVE_Y + CURVE_H + 8)
#define SL_ROW_H 22
#define SL_LBL_X 8
#define SL_LBL_W 34
#define SL_BAR_X 44
#define SL_BAR_W 362
#define SL_VAL_X 412
#define SL_VAL_W 60

// --- Redesigned EQ control section: FREQ/GAIN/Q rings (Stage 1) ---
// Ring area object spans below the curve. Rings are custom-drawn arcs with a
// gap at the bottom (open wedge straddling 0deg in LVGL's arc angle system:
// 0deg=bottom, 90deg=right, clockwise). Values map to fill via value_to_norm().
#define RING_AREA_X      0
#define RING_AREA_Y      (CURVE_Y + CURVE_H - 6)          // 164: 10px above prev
#define RING_AREA_W      SCREEN_W                          // full width; draw clips
#define RING_AREA_H      (SCREEN_H - RING_AREA_Y)          // to screen bottom
#define RING_R           26                                // arc radius (mockup)
#define RING_STROKE      4                                 // arc width (mockup)
#define RING_CY          (40)                              // arc centre Y within area (RING_AREA_Y+40 = 214 screen)
#define RING_GAP_DEG     64                                // bottom open wedge, degrees
// Filled sweep runs clockwise. NOTE: despite the lv_draw_arc_get_area header
// doc claiming "0 deg on the bottom", this build renders 0 deg at the RIGHT
// (verified on hardware: a gap centred on 0 deg appeared at 3 o'clock). Angles
// increase clockwise, so bottom = 90 deg. Gap is therefore centred on 90 deg.
#define RING_TRACK_DEG   (360 - RING_GAP_DEG)              // 296
#define RING_START_DEG   (90 + RING_GAP_DEG / 2)          // 122: just CW of bottom
#define RING_END_DEG     (90 - RING_GAP_DEG / 2 + 360)    // 418: bottom, long way CW
// Three ring centres, evenly spaced between the band-readout column right edge
// (~x56) and the FINE circle left edge (x325-13=312). Derived, not eyeballed.
#define RING_CX0         127
#define RING_CX1         196
#define RING_CX2         265

// Band readout column (left of the rings): "< N >" over a "BAND" label.
// N = selected_band+1 (matches curve marker). Arrows tap prev/next active band,
// split at the number's horizontal centre. Left arrow padded from screen edge.
#define BAND_RO_ARROW_X0   29                // '<' left ref; box left edge -> CURVE_X (28)
#define BAND_RO_ARROW_INSET (-1)             // arrow inner edge = number-box edge minus this (was 1; -1 pulls arrows 2px inward). Tune on HW.
#define BAND_RO_ROW_DY     10                // '< N >' row vertical drop (BAND label stays)
#define BAND_RO_TAP_VPAD   20                // tap zone half-height above/below split
// N-row baseline shares the ring value row (cy-relative); BAND label shares the
// FREQ/GAIN/Q param row. Both derived in the draw cb from RING_CY / RING_R.

// Stage 2: single retargeting parameter slider, in the freed space below the
// ring param labels (old slider strip + bottom row removed). Full width.
#define PSL_X            8
#define PSL_W            (SCREEN_W - 16)                   // 464
#define PSL_Y            257
#define PSL_H            6

// Slider centre tick (GAIN only, marks 0 dB). 2px wide, centred on the even
// track width so it straddles the true centre. Vertical span pokes ~2px past
// the knob top/bottom. Knob half-height is DERIVED (PSL_H/2 + KNOB pad), NOT
// measured against this build's render — tune PSL_TICK_HALF against hardware.
#define PSL_CX           (PSL_X + PSL_W / 2)               // 240 (track centre)
#define PSL_TICK_W       2
#define PSL_TICK_CY      (PSL_Y + PSL_H / 2)               // 260 (track centre Y)
#define PSL_TICK_HALF    11                                // knob ~= PSL_H+2*6=18 -> half 9, +2 poke = 11

// Slider lockout strip: full-width container over the bottom strip that wins
// the touch hit-test (CLICKABLE) so horizontal drags don't reach the tileview
// scroll (page swipe). Slider + centre tick are its children. Mirrors the proven
// gain-popup panel pattern (SCROLLABLE + GESTURE_BUBBLE cleared). Top = 2px below
// BELL bottom (RBTN_SHAPE_Y+RBTN_H+2); extends to screen bottom.
#define PSLK_X           0
#define PSLK_Y           (RBTN_SHAPE_Y + RBTN_H + 2)       // 243
#define PSLK_W           SCREEN_W                          // 480
#define PSLK_H           (SCREEN_H - PSLK_Y)               // 29
// Centre-tap snap zone in px around PSL_CX. Reuses the existing gain snap zone
// (±BAND_GAIN_SNAP_ZONE=30 slider units) converted through the GAIN track:
// PSL_W(464)/300 units = px/unit -> 30*464/300 = 46px. Defined at use to avoid
// a forward-ref to BAND_GAIN_SNAP_ZONE (declared later).

// Right-hand solid button column (DEL / ADD / BELL). Right edge pinned to the
// graph right border (CURVE_X+CURVE_W=428). W/H/pitch are first-pass values to
// tune against a render. Column ends before PSL_Y(257): BELL bottom = 230+19=249.
#define RBTN_W           76
#define RBTN_H           18
#define RBTN_RIGHT       (CURVE_X + CURVE_W)               // 428
#define RBTN_X           (RBTN_RIGHT - RBTN_W)             // 352
#define RBTN_PITCH       23                                // H(18) + 5 gap
#define RBTN_DEL_Y       177                               // top fixed
#define RBTN_ADD_Y       200                               // DEL_Y + PITCH
#define RBTN_SHAPE_Y     223                               // DEL_Y + 2*PITCH; BELL bottom 223+18=241
// Solid fills from the band palette; black text. No state-recolour.
#define RBTN_COL_DEL     lv_color_hex(0xCC3333)            // band palette Red   (idx0)
#define RBTN_COL_ADD     lv_color_hex(0x33BB33)            // band palette Green (idx3)
#define RBTN_COL_SHAPE   lv_color_hex(0xCCAA00)            // band palette Gold  (idx2)

// FINE circle: drawn ring in the gap between the Q ring (right edge 305) and the
// button column (left edge 332). Centre in that gap, on the ring value-row Y
// (RING_AREA_Y+RING_CY=214). R13 spans 306..332 — tight (~1px slack each side).
#define FINE_CX          321
#define FINE_CY          214                               // = RING_AREA_Y + RING_CY
#define FINE_R           13
#define FINE_STROKE      2
#define FINE_MARGIN      2                                 // pad area so stroke+AA isn't clipped by object bounds

// Band button row
#define BB_W 20
#define BB_H 18
#define BB_GAP 1
#define BB_ROW_Y (SCREEN_H - BB_H - 4)

// DEL / ADD button layout (LHS, side by side)
#define BB_BTN_W     27                              // half of original 54
#define BB_BTN_GAP   5                               // gap between DEL and ADD
#define BB_DEL_X     8                               // DEL left edge
#define BB_ADD_X     (BB_DEL_X + BB_BTN_W + BB_BTN_GAP) // ADD left edge
#define BB_BAND_START_X (BB_ADD_X + BB_BTN_W + 5)       // first band button

// LSH / HSH shelf buttons — fixed at RHS of max band position
#define BB_SHF_W     22                              // shelf buttons narrower than DEL/ADD
#define BB_MAX_RIGHT (BB_BAND_START_X + MAX_BANDS * (BB_W + BB_GAP) - BB_GAP)
#define BB_LSH_X     (BB_MAX_RIGHT + 5)
#define BB_HSH_X     (BB_LSH_X + BB_SHF_W + 5)

// Slider internal ranges (integer)
#define FREQ_SLIDER_MAX 1000
#define Q_SLIDER_MAX 1000
#define GAIN_SLIDER_MIN -150
#define GAIN_SLIDER_MAX 150

// Stage gain slider range (dB) — adjust these to taste
#define STAGE_GAIN_MIN_DB -60
#define STAGE_GAIN_MAX_DB +6
#define STAGE_GAIN_DEFAULT 0

// Stage gain slider internal range (integer, 0.1dB resolution)
#define STAGE_GAIN_SLIDER_MIN (STAGE_GAIN_MIN_DB * 10)
#define STAGE_GAIN_SLIDER_MAX (STAGE_GAIN_MAX_DB * 10)

// Frequency mapping: power < 1 gives more slider travel to high end
#define FREQ_CURVE 0.7f

// Colors
#define COL_BG lv_color_hex(0x000000)
#define COL_PANEL lv_color_hex(0x22223A)
#define COL_PANEL_ACT lv_color_hex(0x2A2A5E)
#define COL_BOX lv_color_hex(0x2A2A42)
#define COL_BOX_ACT lv_color_hex(0x3A3A6E)
#define COL_BOX_BORDER lv_color_hex(0x3A3A5A)
#define COL_BOX_BDR_ACT lv_color_hex(0x7777CC)
#define COL_BOX_LIST lv_color_hex(0xe17b3b)
#define COL_TEXT lv_color_hex(0xCCCCDD)
#define COL_TEXT_ACT lv_color_hex(0xAAAAFF)
#define COL_TEXT_DIM lv_color_hex(0x888888)
#define COL_GRID lv_color_hex(0x2A2A4A)
#define COL_GRID_ZERO lv_color_hex(0x3A3A5A)
#define COL_CURVE lv_color_hex(0x00FF88)
#define COL_SLIDER_IND lv_color_hex(0x4444AA)
#define COL_FINE_ON    lv_color_hex(0xFF8C00)   // flashing inner of the FINE circle when active
#define COL_BAND_RED lv_color_hex(0xCC3333)
#define COL_BLACK lv_color_hex(0x000000)
#define COL_BTN_FACE lv_color_hex(0x1A1A2E)
#define COL_WHITE lv_color_hex(0xFFFFFF)

// Preset UI colors
#define COL_PRESET_RED lv_color_hex(0xCC3333)

// Config page two-column layout
#define CFG_COL1_X 28    // Left column start (same as CURVE_X)
#define CFG_COL2_X 250   // Right column start
#define CFG_COL_W 200    // Column width
#define CFG_SLIDER_W 100 // Shortened slider width for left column

// Preset box layout
#define PRESET_BOX_W 36
#define PRESET_BOX_H 28
#define PRESET_BOX_GAP 8
#define PRESET_BTN_W 50
#define PRESET_BTN_H 24

// Preset timing
#define PRESET_SAVE_ARM_HOLD_MS 1000 // Hold time to arm save
#define PRESET_SAVE_TIMEOUT_MS 5000  // Timeout after armed
#define PRESET_FLASH_PERIOD_MS 150   // Flash on/off period
#define PRESET_FLASH_COUNT 6         // 3 flashes = 6 toggles

// Meter colors
#define COL_METER_GREEN lv_color_hex(0x00CC44)
#define COL_METER_YELLOW lv_color_hex(0xCCCC00)
#define COL_METER_ORANGE lv_color_hex(0xCC6600)
#define COL_METER_RED lv_color_hex(0xCC0000)

// ============================================================
// Band color palette (Phase 12) — 10 distinct hues, cycling for 11-20.
// ============================================================
// Chosen for maximum perceptual distance on a dark background at small
// label sizes (montserrat_10 on 480×272). Used by band buttons, curve
// markers, and filter list row numbers.
static const uint32_t band_palette_hex[10] = {
    0xCC3333, //  0  Red
    0xDD7700, //  1  Orange
    0xCCAA00, //  2  Gold
    0x33BB33, //  3  Green
    0x00AA88, //  4  Teal
    0x00BBDD, //  5  Cyan
    0x4488EE, //  6  Blue
    0x8855DD, //  7  Purple
    0xCC44AA, //  8  Magenta
    0xEE6688, //  9  Pink
};
// Text color for use ON a filled band-color background. Black for bright
// hues (orange, gold, green, cyan, pink), white for darker hues.
static const uint32_t band_text_hex[10] = {
    0xFFFFFF, //  0  Red       → white
    0x000000, //  1  Orange    → black
    0x000000, //  2  Gold      → black
    0x000000, //  3  Green     → black
    0x000000, //  4  Teal      → black
    0x000000, //  5  Cyan      → black
    0xFFFFFF, //  6  Blue      → white
    0xFFFFFF, //  7  Purple    → white
    0xFFFFFF, //  8  Magenta   → white
    0x000000, //  9  Pink      → black
};

lv_color_t band_color(int idx) {
  if (idx < 0)
    idx = 0;
  // Second cycle (bands 11-20) shifted by 5 (half the palette) so band N
  // and band N+10 sit on opposite sides of the color wheel — maximizes
  // perceptual distance when both cycles are active. Modulo-2 on the
  // cycle count is defensive in case MAX_BANDS ever grows past 20.
  int cycle_shift = ((idx / 10) % 2) * 5;
  return lv_color_hex(band_palette_hex[(idx + cycle_shift) % 10]);
}
lv_color_t band_color_text(int idx) {
  if (idx < 0)
    idx = 0;
  int cycle_shift = ((idx / 10) % 2) * 5;
  return lv_color_hex(band_text_hex[(idx + cycle_shift) % 10]);
}

// ============================================================
// Global state
// ============================================================
eq_stage_t stage_input;
eq_stage_t stage_output;
eq_stage_t stage_low; // Phase 6: sub output as a third EQ stage
sys_config_t sys_config;

// Crossover page, settings (zero-init; populated in eq_ui_create / Phase 9
// preset load)
crossover_page_t crossover_page = {};
crossover_settings_t xover_settings = {};

static lv_obj_t *main_tileview = NULL;
static lv_obj_t *tile_config_ref = NULL; // Reference for config page
static lv_obj_t *tile_spectrum_ref =
    NULL; // Reference for showing/hiding spectrum tile
static lv_timer_t *curve_timer = NULL;
static bool fine_flash_on = false;   // toggled ~4Hz by the flash block for the FINE circle

// Fine-adjust floating readout: shown while adjusting the param slider in fine
// mode, persisting FINE_READOUT_HOLD_MS after the last activity. The flash block
// (~33ms) hides it once elapsed. Which stage owns it is implicit: only the active
// tile's ring area draws, and selected_param picks the arc.
static uint32_t fine_readout_active_time = 0;   // lv_tick of last press/adjust; 0 = never
static bool     fine_readout_visible = false;
#define FINE_READOUT_HOLD_MS 1000

static void fine_readout_update(eq_stage_t *stage);   // fwd: defined after fmt helpers
static void fine_readout_hide(eq_stage_t *stage);     // fwd: defined after fmt helpers

// Arm/refresh the fine-adjust readout (only meaningful in fine mode).
static inline void fine_readout_touch(eq_stage_t *stage) {
  if (!stage->fine_mode) return;
  fine_readout_visible = true;
  fine_readout_active_time = lv_tick_get();
  fine_readout_update(stage);   // set text + position + show
}

// Track last active tile for meter activation
static int last_active_tile_idx = -1;

// Nav buttons (CFG/VIS on EQ/xover, RET/VIS on config). Tile indices:
// config=0, input=1, xover=2, output=3, sub=4, spectrum=5.
static int nav_return_tile = 1;   // where RET goes; set on CFG-jump, default INPUT
static int viz_return_tile = 1;   // where viz tap-return goes; set on VIS-jump, default INPUT

static void tone_panel_close(void);   // fwd: defined with the tone panel

static void nav_cfg_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !main_tileview) return;
  if (sys_config.tone_active) tone_panel_close();   // kill tone popup on leave
  nav_return_tile = sys_config.active_tile;   // remember origin EQ/xover tile
  if (nav_return_tile < 1 || nav_return_tile > 4) nav_return_tile = 1;  // guard
  lv_obj_set_tile_id(main_tileview, 0, 0, LV_ANIM_OFF);
}

static void nav_vis_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !main_tileview) return;
  if (sys_config.tone_active) tone_panel_close();   // kill tone popup on leave
  viz_return_tile = sys_config.active_tile;   // remember origin for tap-return
  if (viz_return_tile < 0 || viz_return_tile > 4) viz_return_tile = 1;  // guard (0=config allowed)
  lv_obj_set_tile_id(main_tileview, 5, 0, LV_ANIM_OFF);
}

// Public: tap on the viz page returns to the tile viz was entered from.
// Called from spectrum_analyzer.cpp's spectrum_area CLICKED handler.
void eq_ui_viz_return(void) {
  if (!main_tileview) return;
  lv_obj_set_tile_id(main_tileview, viz_return_tile, 0, LV_ANIM_OFF);
}

static void nav_ret_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !main_tileview) return;
  lv_obj_set_tile_id(main_tileview, nav_return_tile, 0, LV_ANIM_OFF);
}

// Robust scroll guard: set true for the FULL duration of scroll+snap animation.
// Using an event flag is more reliable than polling LV_STATE_SCROLLED, which
// clears before the snap animation completes.
volatile bool tileview_scrolling = false;

uint32_t last_scroll_end_time = 0;
#define SCROLL_SETTLE_MS 330

// Forward declaration for preset scroll guard
static void config_page_scroll_guard(void);

// Gain popup forward declarations
static void gain_popup_open(eq_stage_t *stage);
static void gain_popup_close(eq_stage_t *stage);

// Limiter popup forward declarations
static void limiter_popup_open(int limiter_idx, int source_y);
static void limiter_popup_close(void);

static void tileview_scroll_event_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_SCROLL_BEGIN) {
    tileview_scrolling = true;
    if (curve_timer)
      lv_timer_pause(curve_timer);
    spectrum_timer_set_paused(true);

    // Disarm preset load/save on swipe away
    config_page_scroll_guard();

    // Hide all custom draw objects from LVGL compositor
    if (stage_input.curve_area_obj)
      lv_obj_add_flag(stage_input.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (stage_output.curve_area_obj)
      lv_obj_add_flag(stage_output.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (stage_low.curve_area_obj)
      lv_obj_add_flag(stage_low.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (crossover_page.curve_area_obj)
      lv_obj_add_flag(crossover_page.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (stage_input.meter_area_obj)
      lv_obj_add_flag(stage_input.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (stage_output.meter_area_obj)
      lv_obj_add_flag(stage_output.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
    if (stage_low.meter_area_obj)
      lv_obj_add_flag(stage_low.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
    spectrum_set_hidden(true);

    stage_input.curve_dirty = true;
    stage_output.curve_dirty = true;
    stage_low.curve_dirty = true;
    crossover_page.curve_dirty = true;

    // Close any open gain popups so their timers are cleaned up
    gain_popup_close(&stage_input);
    gain_popup_close(&stage_output);
    gain_popup_close(&stage_low);
    limiter_popup_close();

  } else if (code == LV_EVENT_SCROLL_END) {
    last_scroll_end_time = lv_tick_get();
    tileview_scrolling = false;

    // Do NOT unhide here — curve_timer_cb will unhide after settle period

    if (curve_timer)
      lv_timer_resume(curve_timer);
    spectrum_timer_set_paused(false);
  }
}

// Forward declarations
static void format_gain_label(char *buf, int bufsize, float db);
static void rebuild_band_buttons(eq_stage_t *stage);
static void sync_band_display(eq_stage_t *stage);
static void param_slider_retarget(eq_stage_t *stage);
static void update_channel_button_style(eq_stage_t *stage);
static void sync_input_gain_to_config(void);
static void sync_input_gain_from_config(void);
static void preset_timer_check(void);
static void update_all_preset_boxes(void);

// Phase 5 forward decls — definitions live near the bottom of the file with
// the rest of the crossover-page code, but tileview_scroll_event_cb and
// curve_timer_cb (defined earlier) need to call them.
static void crossover_recompute_curve(void);
static void crossover_update_freq_labels(void);
static void create_crossover_page(lv_obj_t *parent);

// Phase 9 forward decls — eq_ui_update_preset_display (defined earlier) needs
// to sync crossover dropdowns and routing button visuals after a preset load.
static int type_to_dd_sel(xover_filter_type_t t);
static int slope_to_dd_sel(xover_slope_t s);
static void update_mono_button_style(void);
static void update_phase_button_style(void);
static void update_delay_label(void);

// Guard flag to prevent infinite callback loops when mirroring gain sliders
static bool updating_gain_from_mirror = false;

// Guard flag for popup slider mirroring in linked mode
static bool updating_popup_from_mirror = false;

// ============================================================
// Active channel accessors
// ============================================================
static inline eq_band_t *get_bands(eq_stage_t *s) {
  return (s->channel_mode == CH_MODE_RIGHT) ? s->bands_r : s->bands_l;
}

// Phase 6: dispatch eq_stage_t pointer → DSP stage index.
// 0 = input EQ, 1 = output EQ, 2 = sub EQ. Used by all DSP API calls
// (dsp_update_band, dsp_set_active_bands, dsp_set_meter_active, etc.).
static inline int stage_to_idx(eq_stage_t *s) {
  if (s == &stage_input)
    return 0;
  if (s == &stage_output)
    return 1;
  return 2; // stage_low
}
static inline int *get_nbp(eq_stage_t *s) { // num_bands pointer
  return (s->channel_mode == CH_MODE_RIGHT) ? &s->num_bands_r : &s->num_bands_l;
}

// Clamp a stage's band selection into the currently active range. Needed
// after any operation that replaces band data wholesale (preset load,
// session restore): selected_band survives the swap, and if the incoming
// preset has fewer active bands the selection points at a phantom slot —
// in-bounds memory-wise (arrays are MAX_BANDS) but not an active band, so
// the rings/readout show a band the filter list doesn't contain.
static void clamp_selected_band(eq_stage_t *s) {
  int n = *get_nbp(s);
  if (s->selected_band >= n)
    s->selected_band = (n > 0) ? (n - 1) : 0;
}

// Push band data to DSP for the active channel (or both if linked)
static void dsp_push_band(eq_stage_t *stage, int band_idx) {
  int si = stage_to_idx(stage);
  eq_band_t *b = &get_bands(stage)[band_idx];
  if (stage->channel_mode == CH_MODE_LINKED) {
    dsp_update_band(si, 0, band_idx, b);
    dsp_update_band(si, 1, band_idx, b);
  } else {
    int ch = (stage->channel_mode == CH_MODE_RIGHT) ? 1 : 0;
    dsp_update_band(si, ch, band_idx, b);
  }
}

static void dsp_push_active_count(eq_stage_t *stage) {
  int si = stage_to_idx(stage);
  int nb = *get_nbp(stage);
  if (stage->channel_mode == CH_MODE_LINKED) {
    dsp_set_active_bands(si, 0, nb);
    dsp_set_active_bands(si, 1, nb);
  } else {
    int ch = (stage->channel_mode == CH_MODE_RIGHT) ? 1 : 0;
    dsp_set_active_bands(si, ch, nb);
  }
}

// Mirror band data and count to the other channel when linked
static void mirror_band_if_linked(eq_stage_t *s, int idx) {
  if (s->channel_mode == CH_MODE_LINKED)
    s->bands_r[idx] = s->bands_l[idx];
}
static void mirror_count_if_linked(eq_stage_t *s) {
  if (s->channel_mode == CH_MODE_LINKED)
    s->num_bands_r = s->num_bands_l;
}

// ============================================================
// Slider styling helper
// ============================================================
static void style_slider_slim(lv_obj_t *slider) {
  lv_obj_set_style_bg_color(slider, COL_BOX, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(slider, 2, LV_PART_MAIN);
  lv_obj_set_style_pad_all(slider, 0, LV_PART_MAIN);

  lv_obj_set_style_bg_color(slider, COL_SLIDER_IND, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_INDICATOR);
  lv_obj_set_style_radius(slider, 2, LV_PART_INDICATOR);

  lv_obj_set_style_bg_color(slider, COL_TEXT, LV_PART_KNOB);
  lv_obj_set_style_bg_opa(slider, LV_OPA_COVER, LV_PART_KNOB);
  lv_obj_set_style_radius(slider, 5, LV_PART_KNOB);
  lv_obj_set_style_pad_all(slider, 4, LV_PART_KNOB);

  lv_obj_clear_flag(slider, LV_OBJ_FLAG_SCROLLABLE);
}

// ============================================================
// Slider <-> parameter conversion
// ============================================================
static int freq_to_slider(eq_stage_t *stage, float freq) {
  if (stage->fine_mode) {
    float norm = (logf(freq) - logf(stage->fine_freq_lo)) /
                 (logf(stage->fine_freq_hi) - logf(stage->fine_freq_lo));
    if (norm < 0.0f)
      norm = 0.0f;
    if (norm > 1.0f)
      norm = 1.0f;
    return (int)(norm * FREQ_SLIDER_MAX + 0.5f);
  }
  float log_norm = freq_to_log(freq);
  return (int)(powf(log_norm, 1.0f / FREQ_CURVE) * FREQ_SLIDER_MAX + 0.5f);
}

static float slider_to_freq(eq_stage_t *stage, int val) {
  float t = (float)val / FREQ_SLIDER_MAX;
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  if (stage->fine_mode) {
    float f = expf(logf(stage->fine_freq_lo) +
                   t * (logf(stage->fine_freq_hi) - logf(stage->fine_freq_lo)));
    if (f < FREQ_MIN)
      f = FREQ_MIN;
    if (f > FREQ_MAX)
      f = FREQ_MAX;
    return f;
  }
  float adjusted = powf(t, FREQ_CURVE);
  return log_to_freq(adjusted);
}

// Tone freq <-> slider (0..FREQ_SLIDER_MAX). Mirrors slider_to_freq/freq_to_slider
// but uses sys_config.tone_fine_* for the fine window. Coarse = full-range log.
static float tone_slider_to_freq(int val) {
  float t = (float)val / FREQ_SLIDER_MAX;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  if (sys_config.tone_fine_mode) {
    return expf(logf(sys_config.tone_fine_lo) +
                t * (logf(sys_config.tone_fine_hi) - logf(sys_config.tone_fine_lo)));
  }
  return log_to_freq(powf(t, FREQ_CURVE));
}

static int tone_freq_to_slider(float freq) {
  if (sys_config.tone_fine_mode) {
    float norm = (logf(freq) - logf(sys_config.tone_fine_lo)) /
                 (logf(sys_config.tone_fine_hi) - logf(sys_config.tone_fine_lo));
    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;
    return (int)(norm * FREQ_SLIDER_MAX + 0.5f);
  }
  return (int)(powf(freq_to_log(freq), 1.0f / FREQ_CURVE) * FREQ_SLIDER_MAX + 0.5f);
}

static int q_to_slider(eq_stage_t *stage, float q) {
  if (stage->fine_mode) {
    float norm = (logf(q) - logf(stage->fine_q_lo)) /
                 (logf(stage->fine_q_hi) - logf(stage->fine_q_lo));
    if (norm < 0.0f)
      norm = 0.0f;
    if (norm > 1.0f)
      norm = 1.0f;
    return (int)(norm * Q_SLIDER_MAX + 0.5f);
  }
  float norm = (logf(q) - logf(Q_MIN)) / (logf(Q_MAX) - logf(Q_MIN));
  return (int)(norm * Q_SLIDER_MAX + 0.5f);
}

static float slider_to_q(eq_stage_t *stage, int val) {
  float t = (float)val / Q_SLIDER_MAX;
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  if (stage->fine_mode) {
    float q = expf(logf(stage->fine_q_lo) +
                   t * (logf(stage->fine_q_hi) - logf(stage->fine_q_lo)));
    if (q < Q_MIN)
      q = Q_MIN;
    if (q > Q_MAX)
      q = Q_MAX;
    return q;
  }
  return expf(logf(Q_MIN) + t * (logf(Q_MAX) - logf(Q_MIN)));
}

static int gain_to_slider(eq_stage_t *stage, float gain) {
  if (stage->fine_mode) {
    float norm = (gain - stage->fine_gain_lo) /
                 (stage->fine_gain_hi - stage->fine_gain_lo);
    if (norm < 0.0f)
      norm = 0.0f;
    if (norm > 1.0f)
      norm = 1.0f;
    return GAIN_SLIDER_MIN +
           (int)(norm * (GAIN_SLIDER_MAX - GAIN_SLIDER_MIN) + 0.5f);
  }
  return (int)(gain * 10.0f + 0.5f * (gain >= 0 ? 1 : -1));
}

static float slider_to_gain(eq_stage_t *stage, int val) {
  if (stage->fine_mode) {
    float norm = (float)(val - GAIN_SLIDER_MIN) /
                 (float)(GAIN_SLIDER_MAX - GAIN_SLIDER_MIN);
    if (norm < 0.0f)
      norm = 0.0f;
    if (norm > 1.0f)
      norm = 1.0f;
    float g = stage->fine_gain_lo +
              norm * (stage->fine_gain_hi - stage->fine_gain_lo);
    if (g < GAIN_MIN)
      g = GAIN_MIN;
    if (g > GAIN_MAX)
      g = GAIN_MAX;
    return g;
  }
  return (float)val / 10.0f;
}

// ============================================================
// Update value labels from current band data
// ============================================================
// ============================================================
// Shelf button styles + callbacks (LSH / HSH)
// ============================================================
static void update_shelf_button_styles(eq_stage_t *stage) {
  if (!stage->lsh_btn || !stage->hsh_btn) return;
  filter_type_t t = get_bands(stage)[stage->selected_band].type;

  bool lsh_active = (t == FTYPE_LOW_SHELF);
  bool hsh_active = (t == FTYPE_HIGH_SHELF);

  lv_obj_set_style_bg_color(stage->lsh_btn, COL_BOX_LIST, 0);
  lv_obj_set_style_bg_opa(stage->lsh_btn, lsh_active ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(stage->lsh_btn, lsh_active ? 0 : 1, 0);

  lv_obj_set_style_bg_color(stage->hsh_btn, COL_BOX_LIST, 0);
  lv_obj_set_style_bg_opa(stage->hsh_btn, hsh_active ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(stage->hsh_btn, hsh_active ? 0 : 1, 0);

  lv_obj_invalidate(stage->lsh_btn);
  lv_obj_invalidate(stage->hsh_btn);
}

// Draw the staircase icons. Button is BB_SHF_W × 22px.
// High y ≈ 30% from top, Low y ≈ 70%, step at horizontal midpoint.
static void lsh_draw_cb(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t   *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  bool active = (get_bands(stage)[stage->selected_band].type == FTYPE_LOW_SHELF);
  lv_color_t col = active ? COL_WHITE : COL_TEXT_DIM;

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  int ox = coords.x1, oy = coords.y1;

  int yH = oy + 6,  yL = oy + 15;
  int xL = ox + 3,  xM = ox + (BB_SHF_W / 2), xR = ox + BB_SHF_W - 3;

  lv_draw_line_dsc_t dsc;
  lv_draw_line_dsc_init(&dsc);
  dsc.color = col;
  dsc.width = 1;
  dsc.opa   = LV_OPA_COVER;

  // High dash → step down → low dash
  dsc.p1.x = xL; dsc.p1.y = yH; dsc.p2.x = xM; dsc.p2.y = yH;
  lv_draw_line(layer, &dsc);
  dsc.p1.x = xM; dsc.p1.y = yH; dsc.p2.x = xM; dsc.p2.y = yL;
  lv_draw_line(layer, &dsc);
  dsc.p1.x = xM; dsc.p1.y = yL; dsc.p2.x = xR; dsc.p2.y = yL;
  lv_draw_line(layer, &dsc);
}

static void hsh_draw_cb(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t   *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  bool active = (get_bands(stage)[stage->selected_band].type == FTYPE_HIGH_SHELF);
  lv_color_t col = active ? COL_WHITE : COL_TEXT_DIM;

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  int ox = coords.x1, oy = coords.y1;

  int yH = oy + 6,  yL = oy + 15;
  int xL = ox + 3,  xM = ox + (BB_SHF_W / 2), xR = ox + BB_SHF_W - 3;

  lv_draw_line_dsc_t dsc;
  lv_draw_line_dsc_init(&dsc);
  dsc.color = col;
  dsc.width = 1;
  dsc.opa   = LV_OPA_COVER;

  // Low dash → step up → high dash
  dsc.p1.x = xL; dsc.p1.y = yL; dsc.p2.x = xM; dsc.p2.y = yL;
  lv_draw_line(layer, &dsc);
  dsc.p1.x = xM; dsc.p1.y = yL; dsc.p2.x = xM; dsc.p2.y = yH;
  lv_draw_line(layer, &dsc);
  dsc.p1.x = xM; dsc.p1.y = yH; dsc.p2.x = xR; dsc.p2.y = yH;
  lv_draw_line(layer, &dsc);
}

// Shape-cycle button: label reflects the selected band's filter type.
// Explicit switch (enum numeric order in eq_data.h is unverified here).
static void update_shape_button_label(eq_stage_t *stage) {
  if (!stage->shape_btn_label) return;
  filter_type_t t = get_bands(stage)[stage->selected_band].type;
  const char *txt = "";   // PEAK: no text, bell glyph drawn by shape_btn_draw_cb
  switch (t) {
    case FTYPE_LOW_SHELF:  txt = "LO SHLF"; break;
    case FTYPE_HIGH_SHELF: txt = "HI SHLF"; break;
    default:               txt = "";        break;  // FTYPE_PEAK -> bell glyph
  }
  lv_label_set_text(stage->shape_btn_label, txt);
  if (stage->shape_btn) lv_obj_invalidate(stage->shape_btn);  // redraw bell/clear
}

// Draw a small bell curve on the shape button when the selected band is PEAK.
// Black 1px polyline, ~20px wide, 10px tall, centred, with 2px flat flare tails
// at the base. Shelf states draw nothing (their text label shows instead).
static void shape_btn_draw_cb(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t   *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  if (get_bands(stage)[stage->selected_band].type != FTYPE_PEAK) return;

  lv_area_t co;
  lv_obj_get_coords(obj, &co);
  int cx = (co.x1 + co.x2) / 2;
  int cy = (co.y1 + co.y2) / 2;
  int by = cy + 5;    // baseline (curve is 10px tall: peak at cy-5)

  // Half-bell profile as (dx, dy-from-baseline) points, base->peak. Mirrored for
  // the right half. Tails: 2px flat flare at the base (dy=0 across dx 10->8).
  static const int prof[][2] = {
    {10, 0}, {8, 0},   // 2px flat flare
    {7, 1}, {6, 3}, {5, 5}, {4, 7}, {3, 8}, {2, 9}, {1, 10}, {0, 10}
  };
  const int N = (int)(sizeof(prof) / sizeof(prof[0]));

  lv_draw_line_dsc_t ld;
  lv_draw_line_dsc_init(&ld);
  ld.color = RBTN_COL_SHAPE;
  ld.width = 1;
  ld.opa   = LV_OPA_COVER;

  // Left half: from left tail up to peak.
  for (int i = 0; i < N - 1; i++) {
    ld.p1.x = (lv_coord_t)(cx - prof[i][0]);   ld.p1.y = (lv_coord_t)(by - prof[i][1]);
    ld.p2.x = (lv_coord_t)(cx - prof[i+1][0]); ld.p2.y = (lv_coord_t)(by - prof[i+1][1]);
    lv_draw_line(layer, &ld);
  }
  // Right half: mirror.
  for (int i = 0; i < N - 1; i++) {
    ld.p1.x = (lv_coord_t)(cx + prof[i][0]);   ld.p1.y = (lv_coord_t)(by - prof[i][1]);
    ld.p2.x = (lv_coord_t)(cx + prof[i+1][0]); ld.p2.y = (lv_coord_t)(by - prof[i+1][1]);
    lv_draw_line(layer, &ld);
  }
}

// Cycle BELL(PEAK) -> LO SHLF -> HI SHLF -> BELL. Order-independent switch.
static void shape_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  eq_band_t *b = &get_bands(stage)[stage->selected_band];
  switch (b->type) {
    case FTYPE_PEAK:       b->type = FTYPE_LOW_SHELF;  break;
    case FTYPE_LOW_SHELF:  b->type = FTYPE_HIGH_SHELF; break;
    case FTYPE_HIGH_SHELF: b->type = FTYPE_PEAK;       break;
    default:               b->type = FTYPE_PEAK;       break;
  }
  update_shape_button_label(stage);
  stage->curve_dirty = true;
  mirror_band_if_linked(stage, stage->selected_band);
  dsp_push_band(stage, stage->selected_band);
  lv_obj_invalidate(lv_screen_active());
  preset_mark_dirty();
}

// ============================================================
// Sync slider positions + labels to selected band
// ============================================================
static void sync_band_display(eq_stage_t *stage) {
  // Refreshes the ring display (FREQ/GAIN/Q) by invalidating the ring area, plus
  // shelf/shape styles. (Formerly sync_sliders_to_band — the band sliders it
  // drove were removed in the ring redesign.)
  if (stage->ring_area_obj)
    lv_obj_invalidate(stage->ring_area_obj);
  param_slider_retarget(stage);
  update_shelf_button_styles(stage);
  update_shape_button_label(stage);
}

// ============================================================
// Fine mode toggle
// ============================================================
// ============================================================
// Mute button (20×12, flashing red fill when active)
// ============================================================
static void mute_draw_cb(lv_event_t *e) {
  lv_layer_t  *layer = lv_event_get_layer(e);
  lv_obj_t    *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t  *stage = (eq_stage_t *)lv_event_get_user_data(e);

  int si = stage_to_idx(stage);
  bool muted = dsp_get_mute(si);

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);

  if (muted && stage->mute_flash_state) {
    lv_draw_rect_dsc_t rdsc;
    lv_draw_rect_dsc_init(&rdsc);
    rdsc.bg_color   = lv_color_hex(0xCC2222);
    rdsc.bg_opa     = LV_OPA_COVER;
    rdsc.radius     = 2;
    rdsc.border_width = 0;
    lv_draw_rect(layer, &rdsc, &coords);
  }

  // "MUTE" label centred horizontally and vertically
  static const char *lbl = "MUTE";
  lv_draw_label_dsc_t ldsc;
  lv_draw_label_dsc_init(&ldsc);
  ldsc.font  = &lv_font_montserrat_10;
  ldsc.color = COL_WHITE;
  ldsc.opa   = LV_OPA_COVER;
  ldsc.align = LV_TEXT_ALIGN_CENTER;
  ldsc.text  = lbl;
  // Centre vertically within the button area
  int fh = lv_font_get_line_height(&lv_font_montserrat_10);
  int btn_h = coords.y2 - coords.y1 + 1;
  lv_area_t label_area = coords;
  label_area.y1 = coords.y1 + (btn_h - fh) / 2;
  label_area.y2 = label_area.y1 + fh;
  lv_draw_label(layer, &ldsc, &label_area);
}

static void mute_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  int si = stage_to_idx(stage);
  bool now_muted = !dsp_get_mute(si);
  dsp_set_mute(si, now_muted);
  stage->mute_flash_state = now_muted;
  lv_obj_invalidate(stage->mute_btn);
}

static void update_fine_button_style(eq_stage_t *stage) {
  // FINE is now a custom-drawn circle (fine_circle_draw_cb owns appearance).
  // This just triggers a redraw; retained for its ~10 call sites.
  if (stage->fine_circle)
    lv_obj_invalidate(stage->fine_circle);
}

// FINE circle draw: grey ring always; flashing orange inner disc when active.
// Area is 2*FINE_R square, so centre = coords centre.
static void fine_circle_draw_cb(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t   *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  int cx = (coords.x1 + coords.x2) / 2;
  int cy = (coords.y1 + coords.y2) / 2;

  // Ring (full circle, grey wall)
  lv_draw_arc_dsc_t adsc;
  lv_draw_arc_dsc_init(&adsc);
  adsc.center.x = cx;
  adsc.center.y = cy;
  adsc.radius   = FINE_R;
  adsc.width    = FINE_STROKE;
  adsc.color    = COL_TEXT_DIM;
  adsc.opa      = LV_OPA_COVER;
  adsc.start_angle = 0;
  adsc.end_angle   = 360;
  lv_draw_arc(layer, &adsc);

  // Flashing orange inner disc when fine mode is on. Drawn as a full-circle arc
  // (same primitive + centre as the ring) so it can't offset against it the way
  // an inscribed rounded-rect did. width>=radius fills solid to the centre.
  if (stage->fine_mode && fine_flash_on) {
    int ir = FINE_R - FINE_STROKE / 2 - 1;   // sits just inside the ring inner edge
    lv_draw_arc_dsc_t ddsc;
    lv_draw_arc_dsc_init(&ddsc);
    ddsc.center.x    = cx;
    ddsc.center.y    = cy;
    ddsc.radius      = ir;
    ddsc.width       = ir + 1;               // >= radius -> filled disc, no centre pinhole
    ddsc.color       = COL_FINE_ON;
    ddsc.opa         = LV_OPA_COVER;
    ddsc.start_angle = 0;
    ddsc.end_angle   = 360;
    lv_draw_arc(layer, &ddsc);
  }
}

static void fine_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  stage->fine_mode = !stage->fine_mode;

  if (stage->fine_mode) {
    fine_flash_on = true;   // light immediately, don't wait for the flash timer
    eq_band_t *b = &get_bands(stage)[stage->selected_band];

    stage->fine_freq_lo = fmaxf(FREQ_MIN, b->freq / 2.0f);
    stage->fine_freq_hi = fminf(FREQ_MAX, b->freq * 2.0f);
    stage->fine_q_lo = fmaxf(Q_MIN, b->q / 2.0f);
    stage->fine_q_hi = fminf(Q_MAX, b->q * 4.0f);  // asymmetric: knob at ~33%, more range above for surgical narrowing
    stage->fine_gain_lo = fmaxf(GAIN_MIN, b->gain - 3.0f);
    stage->fine_gain_hi = fminf(GAIN_MAX, b->gain + 3.0f);
  } else {
    fine_readout_visible = false;
    fine_readout_hide(stage);
  }

  update_fine_button_style(stage);
  sync_band_display(stage);
}

// ============================================================
// Curve computation and update
// ============================================================
static void recompute_curve(eq_stage_t *stage) {
  int nb = *get_nbp(stage);
  eq_band_t *bands = get_bands(stage);

  ui_biquad_t ui_bands[MAX_BANDS];
  for (int i = 0; i < nb; i++) {
    ui_biquad_calc(&bands[i], current_fs, &ui_bands[i]);
  }

  ui_xover_t ui_xover;
  ui_xover.active = false;
  if (stage->is_low) {
    ui_xover_calc(&xover_settings.lp, current_fs, false, &ui_xover);
  } else if (stage->is_output_hp_overlay) {
    ui_xover_calc(&xover_settings.hp, current_fs, true, &ui_xover);
  }

  for (int i = 0; i <= CURVE_POINTS; i++) {
    float t = (float)i / CURVE_POINTS;
    float freq = log_to_freq(t);
    float db = ui_biquad_eval(ui_bands, nb, freq, current_fs);

    // Cascade the relevant crossover filter into the displayed curve.
    //   - Low stage:    cascade the LP filter going into the low path.
    //   - Output stage: cascade the HP filter going into the main path.
    // In both cases the overlay points store the xover-only response so it
    // can be drawn dimmed underneath the combined curve.
    float xover_db = 0.0f;
    if (stage->is_low || stage->is_output_hp_overlay) {
      xover_db = ui_xover_eval(&ui_xover, freq, current_fs);
      db += xover_db;
    }

    if (db > DB_RANGE)
      db = DB_RANGE;
    if (db < -DB_RANGE)
      db = -DB_RANGE;

    // CORRECTION: Bound the maximum X coordinate to CURVE_W - 1 (399) to
    // prevent clipping faults
    stage->curve_pts[i].x = (lv_value_precise_t)(t * (CURVE_W - 1));

    float y_val = CURVE_H / 2.0f - (db / DB_RANGE) * (CURVE_H / 2.0f);

    // Strict clamping to stay 1 pixel inside the object bounds
    if (y_val < 1.0f)
      y_val = 1.0f;
    if (y_val > CURVE_H - 1.0f)
      y_val = CURVE_H - 1.0f;

    stage->curve_pts[i].y = (lv_value_precise_t)y_val;

    // Xover-only reference overlay
    if (stage->is_low || stage->is_output_hp_overlay) {
      float ov = xover_db;
      if (ov > DB_RANGE)
        ov = DB_RANGE;
      if (ov < -DB_RANGE)
        ov = -DB_RANGE;
      float ov_y = CURVE_H / 2.0f - (ov / DB_RANGE) * (CURVE_H / 2.0f);
      if (ov_y < 1.0f)
        ov_y = 1.0f;
      if (ov_y > CURVE_H - 1.0f)
        ov_y = CURVE_H - 1.0f;
      stage->xover_overlay_pts[i].x = stage->curve_pts[i].x;
      stage->xover_overlay_pts[i].y = (lv_value_precise_t)ov_y;
    }
  }
}

static void update_curve_display(eq_stage_t *stage) {
  recompute_curve(stage);
  lv_obj_invalidate(stage->curve_area_obj);
}

static void curve_timer_cb(lv_timer_t *timer) {
  (void)timer;

  if (tileview_scrolling)
    return;
  if (last_scroll_end_time != 0 &&
      (lv_tick_get() - last_scroll_end_time) < SCROLL_SETTLE_MS)
    return;
  if (!main_tileview)
    return;

  // Unhide all custom draw objects after settle period
  if (stage_input.curve_area_obj)
    lv_obj_clear_flag(stage_input.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (stage_output.curve_area_obj)
    lv_obj_clear_flag(stage_output.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (stage_low.curve_area_obj)
    lv_obj_clear_flag(stage_low.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (crossover_page.curve_area_obj)
    lv_obj_clear_flag(crossover_page.curve_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (stage_input.meter_area_obj)
    lv_obj_clear_flag(stage_input.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (stage_output.meter_area_obj)
    lv_obj_clear_flag(stage_output.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
  if (stage_low.meter_area_obj)
    lv_obj_clear_flag(stage_low.meter_area_obj, LV_OBJ_FLAG_HIDDEN);
  spectrum_set_hidden(false);

  // --- Page tracking for meter activation ---
  // Tile indices: 0=Config, 1=Input EQ, 2=Crossover, 3=Output EQ, 4=Low EQ,
  // 5=Spectrum
  lv_obj_t *active_tile = lv_tileview_get_tile_active(main_tileview);
  int tile_idx = -1;
  if (active_tile == stage_input.parent_tile) {
    tile_idx = 1;
  } else if (active_tile == crossover_page.parent_tile) {
    tile_idx = 2;
  } else if (active_tile == stage_output.parent_tile) {
    tile_idx = 3;
  } else if (active_tile == stage_low.parent_tile) {
    tile_idx = 4;
  } else if (active_tile == tile_spectrum_ref) {
    tile_idx = 5;
  }
  // tile_idx remains -1 for config (0) page

  if (tile_idx != last_active_tile_idx) {
    // Page changed — update meter activation. At most one stage's meter is
    // ever active to save DSP cycles and prevent stale peak-hold accumulation.
    // Spectrum tile (5) also activates the output meter so the VU-meter
    // visualizer can read live peak values.
    bool input_on = (tile_idx == 1);
    bool output_on = (tile_idx == 3) || (tile_idx == 5);
    bool low_on = (tile_idx == 4);
    dsp_set_meter_active(0, input_on);
    dsp_set_meter_active(1, output_on);
    dsp_set_meter_active(2, low_on);

    // Phase 15: Track active tile for reboot state memory
    static bool first_tile_eval = true;
    int proper_idx = (tile_idx == -1) ? 0 : tile_idx;
    if (!first_tile_eval && sys_config.active_tile != proper_idx) {
      sys_config.active_tile = proper_idx;
      preset_mark_dirty();
    }
    first_tile_eval = false;
    last_active_tile_idx = tile_idx;
  }

  // --- Curve updates ---
  if (stage_input.curve_dirty && stage_input.parent_tile != NULL) {
    update_curve_display(&stage_input);
    stage_input.curve_dirty = false;
  }
  if (stage_output.curve_dirty && stage_output.parent_tile != NULL) {
    update_curve_display(&stage_output);
    stage_output.curve_dirty = false;
  }
  if (stage_low.curve_dirty && stage_low.parent_tile != NULL) {
    update_curve_display(&stage_low);
    stage_low.curve_dirty = false;
  }
  if (crossover_page.curve_dirty && crossover_page.parent_tile != NULL) {
    crossover_recompute_curve();
    lv_obj_invalidate(crossover_page.curve_area_obj);
    crossover_page.curve_dirty = false;
  }

  // --- Meter updates (invalidate to trigger redraw) ---
  if (tile_idx == 1 && stage_input.meter_area_obj) {
    lv_obj_invalidate(stage_input.meter_area_obj);
  } else if (tile_idx == 3 && stage_output.meter_area_obj) {
    lv_obj_invalidate(stage_output.meter_area_obj);
  } else if (tile_idx == 4 && stage_low.meter_area_obj) {
    lv_obj_invalidate(stage_low.meter_area_obj);
  }

  // --- Preset system checks ---
  preset_check_save();  // Debounced auto-save
  preset_timer_check(); // Save timeout + flash animation
}

// Open the filter-list overlay for the stage this button belongs to.
// user_data is the stage pointer (wired up in create_eq_page).
static void list_btn_cb(lv_event_t *e) {
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  gain_popup_close(stage);  // Dismiss any open gain popup first
  filter_list_open(stage);
}

static void add_band_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  if ((*get_nbp(stage)) >= MAX_BANDS)
    return;

  get_bands(stage)[*get_nbp(stage)].freq = 1000.0f;
  get_bands(stage)[*get_nbp(stage)].gain = 0.0f;
  get_bands(stage)[*get_nbp(stage)].q = 1.0f;
  get_bands(stage)[*get_nbp(stage)].type = FTYPE_PEAK;
  get_bands(stage)[*get_nbp(stage)].enabled = true;

  stage->selected_band = (*get_nbp(stage));
  (*get_nbp(stage))++;

  if (stage->fine_mode) {
    stage->fine_mode = false;
    update_fine_button_style(stage);
  }

  sync_band_display(stage);
  rebuild_band_buttons(stage);
  stage->curve_dirty = true;

  mirror_band_if_linked(stage, stage->selected_band);
  mirror_count_if_linked(stage);
  dsp_push_band(stage, stage->selected_band);
  dsp_push_active_count(stage);
  preset_mark_dirty();
}

static void remove_band_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  eq_ui_remove_band_at(stage, stage->selected_band);
}

// ============================================================
// Public stage-ops API (Phase 12: for filter_list.cpp)
// ============================================================
// Thin wrappers around the static helpers above. See eq_ui.h for intent.
// These all respect channel_mode: bands_l when LINKED or LEFT, bands_r
// when RIGHT. In LINKED mode, edits are mirrored to the other channel's
// array and pushed to both DSP channels.
// ------------------------------------------------------------

eq_band_t *eq_ui_get_bands(eq_stage_t *s) { return get_bands(s); }

int eq_ui_get_band_count(eq_stage_t *s) { return *get_nbp(s); }

void eq_ui_select_band(eq_stage_t *stage, int idx) {
  if (idx < 0 || idx >= *get_nbp(stage))
    return;
  if (stage->selected_band == idx)
    return;

  stage->selected_band = idx;

  if (stage->fine_mode) {
    stage->fine_mode = false;
    update_fine_button_style(stage);
  }

  sync_band_display(stage);
  rebuild_band_buttons(stage);
  // Selection change does not change the curve, so no curve_dirty flag.
}

void eq_ui_commit_band(eq_stage_t *stage, int idx) {
  if (idx < 0 || idx >= *get_nbp(stage))
    return;

  mirror_band_if_linked(stage, idx);
  dsp_push_band(stage, idx);
  stage->curve_dirty = true;

  // Rebuild band buttons so per-band color and disabled-dim state update
  // (cheap; ~20 style-set calls for the whole row).
  rebuild_band_buttons(stage);

  // If the committed band is currently selected, refresh its sliders
  // (e.g. after a type change the sliders' semantics don't move but
  // after a freq/gain/q change they're stale).
  if (stage->selected_band == idx) {
    sync_band_display(stage);
  }

  preset_mark_dirty();
}

void eq_ui_add_band(eq_stage_t *stage) {
  int *nbp = get_nbp(stage);
  if (*nbp >= MAX_BANDS)
    return;

  eq_band_t *bands = get_bands(stage);
  bands[*nbp].freq = 1000.0f;
  bands[*nbp].gain = 0.0f;
  bands[*nbp].q = 1.0f;
  bands[*nbp].type = FTYPE_PEAK;
  bands[*nbp].enabled = true;

  stage->selected_band = *nbp;
  (*nbp)++;

  if (stage->fine_mode) {
    stage->fine_mode = false;
    update_fine_button_style(stage);
  }

  sync_band_display(stage);
  rebuild_band_buttons(stage);
  stage->curve_dirty = true;

  mirror_band_if_linked(stage, stage->selected_band);
  mirror_count_if_linked(stage);
  dsp_push_band(stage, stage->selected_band);
  dsp_push_active_count(stage);
  preset_mark_dirty();
}

void eq_ui_remove_band_at(eq_stage_t *stage, int idx) {
  int *nbp = get_nbp(stage);
  if (*nbp <= 1)
    return;
  if (idx < 0 || idx >= *nbp)
    return;

  eq_band_t *bands = get_bands(stage);

  // Shift bands [idx+1 .. count-1] down by one slot.
  for (int i = idx; i < *nbp - 1; i++) {
    bands[i] = bands[i + 1];
  }
  // Zero the now-unused tail slot so the DSP sees a silent coefficient set.
  eq_band_t silent = {0};
  bands[*nbp - 1] = silent;

  (*nbp)--;

  // Fix up selection: if the removed band was selected, drop to the one
  // below (or 0). If a band above the removed one was selected, its index
  // just shifted down by one.
  if (stage->selected_band == idx) {
    if (stage->selected_band >= *nbp)
      stage->selected_band = *nbp - 1;
    // else stay put — a new band now occupies this slot (the old idx+1).
  } else if (stage->selected_band > idx) {
    stage->selected_band--;
  }

  if (stage->fine_mode) {
    stage->fine_mode = false;
    update_fine_button_style(stage);
  }

  // Push every shifted slot, plus the silenced tail, to the DSP, on the
  // appropriate channel(s). Mirror in linked mode.
  int si = stage_to_idx(stage);

  // Also mirror the full bands array + count to the other channel in
  // linked mode, so bands_r stays in sync with bands_l (covers both the
  // shift and the silenced tail in one pass).
  if (stage->channel_mode == CH_MODE_LINKED) {
    eq_band_t *other =
        (bands == stage->bands_l) ? stage->bands_r : stage->bands_l;
    for (int i = 0; i <= *nbp; i++) {
      if (i < MAX_BANDS)
        other[i] = bands[i];
    }
    stage->num_bands_l = stage->num_bands_r = *nbp;

    for (int i = idx; i < *nbp; i++) {
      dsp_update_band(si, 0, i, &bands[i]);
      dsp_update_band(si, 1, i, &bands[i]);
    }
    dsp_update_band(si, 0, *nbp, &silent);
    dsp_update_band(si, 1, *nbp, &silent);
    dsp_set_active_bands(si, 0, *nbp);
    dsp_set_active_bands(si, 1, *nbp);
  } else {
    int ch = (stage->channel_mode == CH_MODE_RIGHT) ? 1 : 0;
    for (int i = idx; i < *nbp; i++) {
      dsp_update_band(si, ch, i, &bands[i]);
    }
    dsp_update_band(si, ch, *nbp, &silent);
    dsp_set_active_bands(si, ch, *nbp);
  }

  sync_band_display(stage);
  rebuild_band_buttons(stage);
  stage->curve_dirty = true;
  preset_mark_dirty();
}

// ============================================================
// Rebuild band button positions, visibility, and styles
// ============================================================
static void rebuild_band_buttons(eq_stage_t *stage) {
  int n = *get_nbp(stage);
  int start_x = BB_BAND_START_X;

  // Old band-ID buttons removed (ring redesign). Skip their layout/style when
  // absent; DEL/ADD sections below already null-guard. Band selection now via
  // curve tap. Effectively a no-op post-removal but kept (with its 11 callers)
  // until the BAND-readout tap-cycle lands.
  if (stage->band_btns[0]) {
    for (int i = 0; i < MAX_BANDS; i++) {
      if (i < n) {
      int x = start_x + i * (BB_W + BB_GAP);
      lv_obj_set_pos(stage->band_btns[i], x, BB_ROW_Y);
      lv_obj_clear_flag(stage->band_btns[i], LV_OBJ_FLAG_HIDDEN);

      lv_color_t col = band_color(i);
      bool enabled = get_bands(stage)[i].enabled;

      if (i == stage->selected_band) {
        if (enabled) {
          // Selected + enabled: filled with band color
          lv_obj_set_style_bg_color(stage->band_btns[i], col, 0);
          lv_obj_set_style_bg_opa(stage->band_btns[i], LV_OPA_COVER, 0);
          lv_obj_set_style_border_color(stage->band_btns[i], col, 0);
          lv_obj_set_style_text_color(stage->band_btn_labels[i],
                                      band_color_text(i), 0);
        } else {
          // Selected + disabled: dimmed fill, border in band color
          lv_obj_set_style_bg_color(stage->band_btns[i], COL_BOX_ACT, 0);
          lv_obj_set_style_bg_opa(stage->band_btns[i], LV_OPA_COVER, 0);
          lv_obj_set_style_border_color(stage->band_btns[i], col, 0);
          lv_obj_set_style_text_color(stage->band_btn_labels[i], COL_TEXT_DIM,
                                      0);
        }
      } else {
        // Unselected: transparent bg, border+text in band color or dimmed
        lv_color_t dim = enabled ? col : lv_color_hex(0x555555);
        lv_obj_set_style_bg_opa(stage->band_btns[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_color(stage->band_btns[i], dim, 0);
        lv_obj_set_style_text_color(stage->band_btn_labels[i], dim, 0);
      }
    } else {
      lv_obj_add_flag(stage->band_btns[i], LV_OBJ_FLAG_HIDDEN);
    }
  }
  } // end if (band_btns[0])

  // DEL/ADD are solid, no state recolour. Handlers self-guard at limits
  // (ADD no-ops at MAX_BANDS, DEL at 1 band), so no grey-out/disable here.

  lv_obj_invalidate(lv_screen_active());
}

// ============================================================
// Level meter drawing
// ============================================================
static void meter_area_draw_cb(lv_event_t *e) {
  extern volatile bool tileview_scrolling;
  if (tileview_scrolling)
    return;

  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  // Determine stage index for DSP API
  int stage_idx = stage_to_idx(stage);

  // Get meter readings from DSP
  float peak_db_l = dsp_get_meter_peak(stage_idx, 0);
  float peak_db_r = dsp_get_meter_peak(stage_idx, 1);
  float hold_db_l = dsp_get_meter_peak_hold(stage_idx, 0);
  float hold_db_r = dsp_get_meter_peak_hold(stage_idx, 1);
  bool clip_l = dsp_get_meter_clipping(stage_idx, 0);
  bool clip_r = dsp_get_meter_clipping(stage_idx, 1);

  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);
  int ox = obj_coords.x1;
  int oy = obj_coords.y1;

  // Helper: convert dB to pixel height from bottom (0 = bottom, METER_AREA_H =
  // top)
  auto db_to_px = [](float db) -> int {
    if (db <= METER_FLOOR_DB)
      return 0;
    if (db >= 0.0f)
      return METER_AREA_H;
    return (int)(((db - METER_FLOOR_DB) / (0.0f - METER_FLOOR_DB)) *
                 METER_AREA_H);
  };

  // Helper: get color for a given dB level
  auto db_to_color = [](float db, bool clipping) -> lv_color_t {
    if (clipping)
      return COL_METER_RED;
    if (db >= METER_DB_16)
      return COL_METER_ORANGE;
    if (db >= METER_DB_20)
      return COL_METER_YELLOW;
    return COL_METER_GREEN;
  };

  // Helper: draw one meter bar
  auto draw_bar = [&](int bar_x, float peak_db, float hold_db, bool clipping) {
    int peak_px = db_to_px(peak_db);
    int hold_px = db_to_px(hold_db);

    lv_draw_rect_dsc_t rect_dsc;

    if (clipping) {
      // Entire bar is red when clipping
      if (peak_px > 0) {
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = COL_METER_RED;
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = 0;
        rect_dsc.border_width = 0;
        lv_area_t bar_area = {(lv_coord_t)(ox + bar_x),
                              (lv_coord_t)(oy + METER_AREA_H - peak_px),
                              (lv_coord_t)(ox + bar_x + METER_BAR_W - 1),
                              (lv_coord_t)(oy + METER_AREA_H - 1)};
        lv_draw_rect(layer, &rect_dsc, &bar_area);
      }
    } else {
      // Draw segmented bar from bottom to peak_px
      // Segment 1: Green (0 to METER_PX_20)
      if (peak_px > 0) {
        int seg_top = (peak_px < METER_PX_20) ? peak_px : METER_PX_20;
        if (seg_top > 0) {
          lv_draw_rect_dsc_init(&rect_dsc);
          rect_dsc.bg_color = COL_METER_GREEN;
          rect_dsc.bg_opa = LV_OPA_COVER;
          rect_dsc.radius = 0;
          rect_dsc.border_width = 0;
          lv_area_t seg = {(lv_coord_t)(ox + bar_x),
                           (lv_coord_t)(oy + METER_AREA_H - seg_top),
                           (lv_coord_t)(ox + bar_x + METER_BAR_W - 1),
                           (lv_coord_t)(oy + METER_AREA_H - 1)};
          lv_draw_rect(layer, &rect_dsc, &seg);
        }
      }

      // Segment 2: Yellow (METER_PX_20 to METER_PX_16)
      if (peak_px > METER_PX_20) {
        int seg_bottom = METER_PX_20;
        int seg_top = (peak_px < METER_PX_16) ? peak_px : METER_PX_16;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = COL_METER_YELLOW;
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = 0;
        rect_dsc.border_width = 0;
        lv_area_t seg = {(lv_coord_t)(ox + bar_x),
                         (lv_coord_t)(oy + METER_AREA_H - seg_top),
                         (lv_coord_t)(ox + bar_x + METER_BAR_W - 1),
                         (lv_coord_t)(oy + METER_AREA_H - seg_bottom - 1)};
        lv_draw_rect(layer, &rect_dsc, &seg);
      }

      // Segment 3: Orange (METER_PX_16 to peak_px)
      if (peak_px > METER_PX_16) {
        int seg_bottom = METER_PX_16;
        int seg_top = peak_px;
        lv_draw_rect_dsc_init(&rect_dsc);
        rect_dsc.bg_color = COL_METER_ORANGE;
        rect_dsc.bg_opa = LV_OPA_COVER;
        rect_dsc.radius = 0;
        rect_dsc.border_width = 0;
        lv_area_t seg = {(lv_coord_t)(ox + bar_x),
                         (lv_coord_t)(oy + METER_AREA_H - seg_top),
                         (lv_coord_t)(ox + bar_x + METER_BAR_W - 1),
                         (lv_coord_t)(oy + METER_AREA_H - seg_bottom - 1)};
        lv_draw_rect(layer, &rect_dsc, &seg);
      }
    }

    // Peak hold indicator (1px horizontal line)
    if (hold_px > 0 && hold_px <= METER_AREA_H) {
      lv_draw_rect_dsc_init(&rect_dsc);
      rect_dsc.bg_color = db_to_color(hold_db, clipping);
      rect_dsc.bg_opa = LV_OPA_COVER;
      rect_dsc.radius = 0;
      rect_dsc.border_width = 0;
      lv_area_t hold_line = {(lv_coord_t)(ox + bar_x),
                             (lv_coord_t)(oy + METER_AREA_H - hold_px),
                             (lv_coord_t)(ox + bar_x + METER_BAR_W - 1),
                             (lv_coord_t)(oy + METER_AREA_H - hold_px)};
      lv_draw_rect(layer, &rect_dsc, &hold_line);
    }
  };

  // Draw L and R bars
  draw_bar(METER_X_L, peak_db_l, hold_db_l, clip_l);
  draw_bar(METER_X_R, peak_db_r, hold_db_r, clip_r);
}

// ============================================================
// Curve area grid drawing
// ============================================================
// ============================================================
// Curve area click handler — tap markers to select bands
// ============================================================
// Hit-tests the tap coordinates against all enabled band markers. If a
// marker is within the touch radius, selects that band (same effect as
// tapping its button in the bottom row or its num cell in the filter list).
//
// Overlap handling: if multiple markers are within the touch radius, the
// first tap selects the lowest-index band. Tapping the same spot again
// (within 15px and within 1 second) cycles to the next overlapping band.
// This allows disambiguation without double-tap latency or hidden gestures.
static void curve_area_click_cb(lv_event_t *e) {
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);

  lv_point_t tap;
  lv_indev_get_point(lv_indev_get_act(), &tap);

  // Convert tap coords from screen-space to curve-area-relative
  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);
  int tap_x = tap.x - obj_coords.x1;
  int tap_y = tap.y - obj_coords.y1;

  // Touch radius for marker hit-test: 20px is finger-friendly.
  const int touch_r = 20;

  // Repeat-tap cycling state: if this tap lands within 15px of the last
  // tap and within 1 second, we cycle to the next marker in the hit set
  // instead of always picking the first one. Allows overlap disambiguation.
  static uint32_t last_tap_time = 0;
  static int last_tap_x = -100, last_tap_y = -100;
  static int last_selected_from_tap = -1;

  uint32_t now = millis();
  bool is_repeat =
      (abs(tap_x - last_tap_x) < 15 && abs(tap_y - last_tap_y) < 15 &&
       (now - last_tap_time) < 1000);

  last_tap_time = now;
  last_tap_x = tap_x;
  last_tap_y = tap_y;

  eq_band_t *bands = get_bands(stage);
  int nb = *get_nbp(stage);

  // Build list of all bands hit by this tap
  int hits[MAX_BANDS];
  int hit_count = 0;

  for (int i = 0; i < nb; i++) {
    if (!bands[i].enabled)
      continue;

    // Marker position: same logic as the draw callback
    float freq = bands[i].freq;
    float t = freq_to_log(freq);
    int mx = (int)(t * (CURVE_W - 1));

    float db = compute_total_response(bands, nb, freq, current_fs);
    if (stage->is_low) {
      db += compute_xover_response(&xover_settings.lp, freq, current_fs, false);
    } else if (stage->is_output_hp_overlay) {
      db += compute_xover_response(&xover_settings.hp, freq, current_fs, true);
    }
    if (db > DB_RANGE)
      db = DB_RANGE;
    if (db < -DB_RANGE)
      db = -DB_RANGE;

    int my = (int)(CURVE_H / 2.0f - (db / DB_RANGE) * (CURVE_H / 2.0f));
    if (my < 1)
      my = 1;
    if (my > CURVE_H - 2)
      my = CURVE_H - 2;

    // Distance check
    int dx = tap_x - mx;
    int dy = tap_y - my;
    if (dx * dx + dy * dy < touch_r * touch_r) {
      hits[hit_count++] = i;
    }
  }

  if (hit_count == 0) {
    // No hit — tap landed on curve background, no action.
    last_selected_from_tap = -1;
    return;
  }

  // Pick which band to select from the hit set
  int target_idx;
  if (hit_count == 1) {
    // Only one marker hit, select it
    target_idx = hits[0];
  } else if (is_repeat) {
    // Multiple hits + repeat tap: cycle to next in the list
    int current_pos = -1;
    for (int i = 0; i < hit_count; i++) {
      if (hits[i] == last_selected_from_tap) {
        current_pos = i;
        break;
      }
    }
    // Wrap around: if last selected isn't in this hit set or we're at
    // the end, start from the beginning.
    target_idx = hits[(current_pos + 1) % hit_count];
  } else {
    // Multiple hits + fresh tap: select the first (lowest index)
    target_idx = hits[0];
  }

  last_selected_from_tap = target_idx;
  eq_ui_select_band(stage, target_idx);
}

// ============================================================
// Curve area draw callback
// ============================================================
static void curve_area_draw_cb(lv_event_t *e) {
  extern volatile bool tileview_scrolling;
  if (tileview_scrolling)
    return;
  // No settle guard here: lv_draw_line primitives are clip-safe during snap
  // animation. The settle guard in curve_timer_cb is sufficient to avoid
  // redundant recomputes.

  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);
  int ox = obj_coords.x1;
  int oy = obj_coords.y1;

  lv_draw_line_dsc_t line_dsc;
  lv_draw_label_dsc_t label_dsc;

  // Graph boundary frame
  lv_draw_rect_dsc_t border_dsc;
  lv_draw_rect_dsc_init(&border_dsc);
  border_dsc.bg_opa = LV_OPA_TRANSP;
  border_dsc.border_color = COL_GRID_ZERO;
  border_dsc.border_width = 1;
  border_dsc.border_opa = LV_OPA_COVER;
  border_dsc.radius = 0;
  lv_area_t frame = {(lv_coord_t)ox, (lv_coord_t)oy,
                     (lv_coord_t)(ox + CURVE_W - 1),
                     (lv_coord_t)(oy + CURVE_H - 1)};
  lv_draw_rect(layer, &border_dsc, &frame);

  // Horizontal grid lines
  for (int db = -15; db <= 15; db += 5) {
    if (db == 0)
      continue;
    int y = oy + CURVE_H / 2 - (int)((float)db / DB_RANGE * CURVE_H / 2.0f);
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COL_GRID;
    line_dsc.width = 1;
    line_dsc.p1.x = ox;
    line_dsc.p1.y = y;
    line_dsc.p2.x = ox + CURVE_W;
    line_dsc.p2.y = y;
    lv_draw_line(layer, &line_dsc);
  }

  // Zero line
  lv_draw_line_dsc_init(&line_dsc);
  line_dsc.color = COL_GRID_ZERO;
  line_dsc.width = 1;
  line_dsc.p1.x = ox;
  line_dsc.p1.y = oy + CURVE_H / 2;
  line_dsc.p2.x = ox + CURVE_W;
  line_dsc.p2.y = oy + CURVE_H / 2;
  lv_draw_line(layer, &line_dsc);

  // Vertical grid lines + freq labels (OPTIMIZED FOR LVGL 9 STABILITY)
  static const float freq_marks[] = {20,   50,   100,  200,   500,
                                     1000, 2000, 5000, 10000, 20000};
  static const char *freq_labels[] = {"20", "50", "100", "200", "500",
                                      "1k", "2k", "5k",  "10k", "20k"};

  for (int i = 0; i < 10; i++) {
    int x = ox + (int)(freq_to_log(freq_marks[i]) * CURVE_W);
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COL_GRID;
    line_dsc.width = 1;
    line_dsc.p1.x = x;
    line_dsc.p1.y = oy;
    line_dsc.p2.x = x;
    line_dsc.p2.y = oy + CURVE_H;
    lv_draw_line(layer, &line_dsc);

    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = COL_TEXT_DIM;
    label_dsc.font = &lv_font_montserrat_10;

    // Safety: Constant pointers for thread/context safety during swipes
    label_dsc.text = freq_labels[i];

    lv_area_t la;
    if (i == 0) {
      label_dsc.align = LV_TEXT_ALIGN_LEFT;
      la = (lv_area_t){(lv_coord_t)(x + 2), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x + 30), (lv_coord_t)(oy + CURVE_H - 1)};
    } else if (i == 9) {
      label_dsc.align = LV_TEXT_ALIGN_RIGHT;
      la = (lv_area_t){(lv_coord_t)(x - 30), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x - 2), (lv_coord_t)(oy + CURVE_H - 1)};
    } else {
      label_dsc.align = LV_TEXT_ALIGN_CENTER;
      la = (lv_area_t){(lv_coord_t)(x - 15), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x + 15), (lv_coord_t)(oy + CURVE_H - 1)};
    }
    lv_draw_label(layer, &label_dsc, &la);
  }

  // dB labels inside graph (OPTIMIZED FOR LVGL 9 STABILITY)
  static const int db_marks[] = {20, 10, -10};
  static const char *db_labels[] = {"20", "10", "10"}; // abs(db) labels

  for (int i = 0; i < 3; i++) {
    int db_val = db_marks[i];
    int y = oy + CURVE_H / 2 - (int)((float)db_val / DB_RANGE * CURVE_H / 2.0f);

    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = COL_TEXT_DIM;
    label_dsc.font = &lv_font_montserrat_10;
    label_dsc.text = db_labels[i];
    label_dsc.align = LV_TEXT_ALIGN_LEFT;

    lv_area_t la;
    if (db_val > 0) {
      la = (lv_area_t){(lv_coord_t)(ox + 2), (lv_coord_t)(y + 1),
                       (lv_coord_t)(ox + 24), (lv_coord_t)(y + 13)};
    } else {
      la = (lv_area_t){(lv_coord_t)(ox + 2), (lv_coord_t)(y - 13),
                       (lv_coord_t)(ox + 24), (lv_coord_t)(y - 1)};
    }
    lv_draw_label(layer, &label_dsc, &la);
  }

  // Stage Name Label (INPUT / OUTPUT) inside the graph
  lv_draw_label_dsc_init(&label_dsc);
  label_dsc.color = COL_WHITE;
  label_dsc.font = &lv_font_montserrat_18;
  label_dsc.text = stage->name;
  label_dsc.align = LV_TEXT_ALIGN_RIGHT;
 lv_area_t name_la = {(lv_coord_t)(ox + CURVE_W - 160), (lv_coord_t)(oy + 8),
                     (lv_coord_t)(ox + CURVE_W - 8), (lv_coord_t)(oy + 28)};

  lv_draw_label(layer, &label_dsc, &name_la);

  // Xover reference overlay (drawn first so the combined curve sits on top).
  // Dimmed to match the color used on the crossover page:
  //   - Low stage:    orange (COL_XO_LP = 0xFF8800) for LP overlay
  //   - Output stage: cyan   (COL_XO_HP = 0x00AAFF) for HP overlay
  if (stage->is_low || stage->is_output_hp_overlay) {
    lv_draw_line_dsc_t ov_dsc;
    lv_draw_line_dsc_init(&ov_dsc);
    ov_dsc.color =
        stage->is_low ? lv_color_hex(0xFF8800) : lv_color_hex(0x00AAFF);
    ov_dsc.opa = LV_OPA_40;
    ov_dsc.width = 1;
    for (int i = 0; i < CURVE_POINTS; i++) {
      ov_dsc.p1.x = ox + (lv_coord_t)stage->xover_overlay_pts[i].x;
      ov_dsc.p1.y = oy + (lv_coord_t)stage->xover_overlay_pts[i].y;
      ov_dsc.p2.x = ox + (lv_coord_t)stage->xover_overlay_pts[i + 1].x;
      ov_dsc.p2.y = oy + (lv_coord_t)stage->xover_overlay_pts[i + 1].y;
      lv_draw_line(layer, &ov_dsc);
    }
  }

  // EQ curve — drawn as individual segments so the tileview_scrolling guard
  // above prevents rendering during swipe (no lv_line widget, no bounding-box
  // issues).
  lv_draw_line_dsc_t curve_dsc;
  lv_draw_line_dsc_init(&curve_dsc);
  curve_dsc.color = COL_CURVE;
  curve_dsc.width = 2;
  curve_dsc.round_start = 0;
  curve_dsc.round_end = 0;
  for (int i = 0; i < CURVE_POINTS; i++) {
    curve_dsc.p1.x = ox + (lv_coord_t)stage->curve_pts[i].x;
    curve_dsc.p1.y = oy + (lv_coord_t)stage->curve_pts[i].y;
    curve_dsc.p2.x = ox + (lv_coord_t)stage->curve_pts[i + 1].x;
    curve_dsc.p2.y = oy + (lv_coord_t)stage->curve_pts[i + 1].y;
    lv_draw_line(layer, &curve_dsc);
  }

  // ============================================================
  // Band markers on curve (Phase 12)
  // ============================================================
  // Numbered colored dots at each enabled band's frequency, positioned on
  // the composite curve (EQ + crossover where applicable). Disabled bands
  // are skipped. Selected band gets a larger dot + white outline ring.
  //
  // LVGL v9 draw callback rule: all label text must live in static buffers
  // because lv_draw_label reads the pointer after this stack frame returns.
  {
    static char marker_bufs[MAX_BANDS][4];
    eq_band_t *bands = get_bands(stage);
    int nb = *get_nbp(stage);

    ui_biquad_t ui_bands[MAX_BANDS];
    for (int i = 0; i < nb; i++) {
      ui_biquad_calc(&bands[i], current_fs, &ui_bands[i]);
    }

    ui_xover_t ui_xover;
    ui_xover.active = false;
    if (stage->is_low) {
      ui_xover_calc(&xover_settings.lp, current_fs, false, &ui_xover);
    } else if (stage->is_output_hp_overlay) {
      ui_xover_calc(&xover_settings.hp, current_fs, true, &ui_xover);
    }

    for (int i = 0; i < nb; i++) {
      if (!bands[i].enabled)
        continue;

      float freq = bands[i].freq;
      float t = freq_to_log(freq);
      int mx = ox + (int)(t * (CURVE_W - 1));

      // Y position: total EQ response + crossover cascade (matches
      // recompute_curve logic so markers sit exactly on the drawn line).
      float db = ui_biquad_eval(ui_bands, nb, freq, current_fs);
      if (stage->is_low || stage->is_output_hp_overlay) {
        db += ui_xover_eval(&ui_xover, freq, current_fs);
      }
      if (db > DB_RANGE)
        db = DB_RANGE;
      if (db < -DB_RANGE)
        db = -DB_RANGE;

      int my = oy + (int)(CURVE_H / 2.0f - (db / DB_RANGE) * (CURVE_H / 2.0f));
      if (my < oy + 1)
        my = oy + 1;
      if (my > oy + CURVE_H - 2)
        my = oy + CURVE_H - 2;

      lv_color_t col = band_color(i);
      bool sel = (i == stage->selected_band);
      int dot_r = sel ? 6 : 4;

      // Selected: white outline ring drawn first (behind the dot)
      if (sel) {
        lv_draw_rect_dsc_t ring_dsc;
        lv_draw_rect_dsc_init(&ring_dsc);
        ring_dsc.bg_opa = LV_OPA_TRANSP;
        ring_dsc.border_color = COL_WHITE;
        ring_dsc.border_width = 3;
        ring_dsc.border_opa = LV_OPA_COVER;
        ring_dsc.radius = dot_r + 3;
        lv_area_t ring_a = {
            (lv_coord_t)(mx - dot_r - 3), (lv_coord_t)(my - dot_r - 3),
            (lv_coord_t)(mx + dot_r + 2), (lv_coord_t)(my + dot_r + 2)};
        lv_draw_rect(layer, &ring_dsc, &ring_a);
      }

      // Filled dot
      lv_draw_rect_dsc_t dot_dsc;
      lv_draw_rect_dsc_init(&dot_dsc);
      dot_dsc.bg_color = col;
      dot_dsc.bg_opa = LV_OPA_COVER;
      dot_dsc.border_width = 0;
      dot_dsc.radius = dot_r;
      lv_area_t dot_a = {(lv_coord_t)(mx - dot_r), (lv_coord_t)(my - dot_r),
                         (lv_coord_t)(mx + dot_r - 1),
                         (lv_coord_t)(my + dot_r - 1)};
      lv_draw_rect(layer, &dot_dsc, &dot_a);

      // Number label above dot (flip below if too close to top edge)
      snprintf(marker_bufs[i], sizeof(marker_bufs[i]), "%d", i + 1);
      lv_draw_label_dsc_init(&label_dsc);
      label_dsc.color = col;
      label_dsc.font = &lv_font_montserrat_12;
      label_dsc.text = marker_bufs[i];
      label_dsc.align = LV_TEXT_ALIGN_CENTER;

      int lbl_h = lv_font_get_line_height(label_dsc.font);
      // Position label clear of the ring: bottom edge 1px above ring outer top
      // (ring outer radius = dot_r + 3). Self-correcting for font height.
      int lbl_y = my - dot_r - 4 - lbl_h;
      bool lbl_flipped = false;
      if (lbl_y < oy) {
        lbl_y = my + dot_r + 4; // flip below: top edge 1px under ring bottom
        lbl_flipped = true;
      }

      lv_area_t lbl_a = {(lv_coord_t)(mx - 12), (lv_coord_t)(lbl_y),
                         (lv_coord_t)(mx + 12), (lv_coord_t)(lbl_y + lbl_h)};
      lv_draw_label(layer, &label_dsc, &lbl_a);

      // Selected-band arrow: filled white triangle pointing at the dot.
      // Normally above the number pointing DOWN; flips below pointing UP when
      // its base would clip the curve-area top (oy) — decided on the ARROW's
      // own geometry, independent of the label flip, so it never disappears at
      // high positive gain. Base = ring outer diameter (2*(dot_r+3)=18),
      // height ~= base. Scanline-filled via lv_draw_line (lv_draw_triangle
      // signature unverified for this build; line fill is proven above).
      if (sel) {
        const int A_HALF = dot_r + 3;   // base half-width = ring outer radius (9)
        const int A_H    = 2 * A_HALF;  // full triangle height (18); base truncated in fill
        lv_draw_line_dsc_t ad;
        lv_draw_line_dsc_init(&ad);
        ad.color = COL_WHITE;
        ad.width = 1;
        ad.opa   = LV_OPA_COVER;

        // Preferred: above the number, pointing down, tip 1px above label top.
        int down_tip  = lbl_flipped ? (my - dot_r - 3 - 1) : (lbl_y - 1);
        int down_base = down_tip - A_H;
        int tip_y, base_y, dir;
        if (down_base >= oy) {
          // Fits above without clipping — point down.
          tip_y = down_tip; base_y = down_base; dir = +1;
        } else {
          // Would clip the top — flip below the number, point up.
          int ref_bot = lbl_flipped ? (lbl_y + lbl_h) : (my + dot_r + 3);
          tip_y = ref_bot + 1; base_y = tip_y + A_H; dir = -1;
        }
        // Scanline fill; skip the 4 widest base rows (truncated base). Tip and
        // taper slope unchanged; shape becomes a trapezoid, 4px shorter at base.
        for (int s = 4; s <= A_H; s++) {
          int y = base_y + dir * s;
          int hw = A_HALF - (A_HALF * s) / A_H;  // A_HALF at base -> 0 at tip
          if (hw < 0) hw = 0;
          ad.p1.x = (lv_coord_t)(mx - hw); ad.p1.y = (lv_coord_t)y;
          ad.p2.x = (lv_coord_t)(mx + hw); ad.p2.y = (lv_coord_t)y;
          lv_draw_line(layer, &ad);
        }
      }
    }
  }
}

// ============================================================
// Channel selection (L / R / Lk) buttons
// ============================================================
#define CH_BTN_W 25
#define CH_BTN_H 25
#define CH_BTN_GAP 3
#define CH_BTN_X (CURVE_X + 3)
#define CH_BTN_Y (CURVE_Y + CURVE_H - CH_BTN_H - 16)

#define COL_CH_ACTIVE lv_color_hex(0xe17b3b)
#define COL_CH_INACTIVE lv_color_hex(0x333355)

static void update_channel_button_style(eq_stage_t *stage) {
  lv_obj_t *btns[] = {stage->ch_l_btn, stage->ch_r_btn, stage->ch_lk_btn};
  lv_obj_t *lbls[] = {stage->ch_l_label, stage->ch_r_label, stage->ch_lk_label};
  int active = (stage->channel_mode == CH_MODE_LEFT)    ? 0
               : (stage->channel_mode == CH_MODE_RIGHT) ? 1
                                                        : 2;

  for (int i = 0; i < 3; i++) {
    if (!btns[i])
      continue;
    if (i == active) {
      lv_obj_set_style_bg_color(btns[i], COL_CH_ACTIVE, 0);
      lv_obj_set_style_bg_opa(btns[i], LV_OPA_60, 0);
      lv_obj_set_style_text_color(lbls[i], COL_BLACK, 0);
    } else {
      lv_obj_set_style_bg_color(btns[i], COL_CH_INACTIVE, 0);
      lv_obj_set_style_bg_opa(btns[i], LV_OPA_60, 0);
      lv_obj_set_style_text_color(lbls[i], COL_TEXT_DIM, 0);
    }
  }
}

static void channel_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);

  channel_mode_t old_mode = stage->channel_mode;
  channel_mode_t new_mode;

  if (btn == stage->ch_l_btn)
    new_mode = CH_MODE_LEFT;
  else if (btn == stage->ch_r_btn)
    new_mode = CH_MODE_RIGHT;
  else
    new_mode = CH_MODE_LINKED;

  if (new_mode == old_mode)
    return;

  // Transition: LINKED → L or R: copy current (L) to R so both start identical
  if (old_mode == CH_MODE_LINKED && new_mode != CH_MODE_LINKED) {
    memcpy(stage->bands_r, stage->bands_l, sizeof(stage->bands_l));
    stage->num_bands_r = stage->num_bands_l;
  }

  // Transition: L or R → LINKED: copy L to R (L becomes canonical)
  if (old_mode != CH_MODE_LINKED && new_mode == CH_MODE_LINKED) {
    memcpy(stage->bands_r, stage->bands_l, sizeof(stage->bands_l));
    stage->num_bands_r = stage->num_bands_l;
    // Push L data to both DSP channels
    int si = stage_to_idx(stage);
    for (int i = 0; i < stage->num_bands_l; i++) {
      dsp_update_band(si, 0, i, &stage->bands_l[i]);
      dsp_update_band(si, 1, i, &stage->bands_l[i]);
    }
    dsp_set_active_bands(si, 0, stage->num_bands_l);
    dsp_set_active_bands(si, 1, stage->num_bands_l);

    // Sync Gain: Highest fader snaps to match the lower fader
    float min_gain = (stage->stage_gain_l_db < stage->stage_gain_r_db) ? stage->stage_gain_l_db : stage->stage_gain_r_db;
    stage->stage_gain_l_db = min_gain;
    stage->stage_gain_r_db = min_gain;
    
    if (stage == &stage_input) {
      dsp_set_input_gain(0, min_gain);
      dsp_set_input_gain(1, min_gain);
    } else if (stage == &stage_output) {
      dsp_set_output_gain(0, min_gain);
      dsp_set_output_gain(1, min_gain);
    } else {
      low_set_output_gain(0, min_gain);
      low_set_output_gain(1, min_gain);
    }

    char buf[8];
    format_gain_label(buf, sizeof(buf), min_gain);

    if (stage->gain_popup_slider_l && stage->gain_popup_slider_r) {
        updating_popup_from_mirror = true;
        lv_slider_set_value(stage->gain_popup_slider_l, (int)(min_gain * 10), LV_ANIM_OFF);
        lv_slider_set_value(stage->gain_popup_slider_r, (int)(min_gain * 10), LV_ANIM_OFF);
        if (stage->gain_popup_val_l) lv_label_set_text(stage->gain_popup_val_l, buf);
        if (stage->gain_popup_val_r) lv_label_set_text(stage->gain_popup_val_r, buf);
        updating_popup_from_mirror = false;
    }

    if (stage->stage_gain_slider_l && stage->stage_gain_slider_r) {
        updating_gain_from_mirror = true;
        lv_slider_set_value(stage->stage_gain_slider_l, (int)(min_gain * 10), LV_ANIM_OFF);
        lv_slider_set_value(stage->stage_gain_slider_r, (int)(min_gain * 10), LV_ANIM_OFF);
        updating_gain_from_mirror = false;
        if (stage->stage_gain_readout_l) lv_label_set_text(stage->stage_gain_readout_l, buf);
        if (stage->stage_gain_readout_r) lv_label_set_text(stage->stage_gain_readout_r, buf);
    }
    
    if (stage == &stage_input) sync_input_gain_to_config();
  }

  stage->channel_mode = new_mode;

  // Clamp selected_band to new channel's band count
  int nb = *get_nbp(stage);
  if (stage->selected_band >= nb)
    stage->selected_band = nb - 1;
  if (stage->selected_band < 0)
    stage->selected_band = 0;

  if (stage->fine_mode) {
    stage->fine_mode = false;
    update_fine_button_style(stage);
  }

  update_channel_button_style(stage);
  sync_band_display(stage);
  rebuild_band_buttons(stage);
  stage->curve_dirty = true;
  preset_mark_dirty();
}

// ============================================================
// Stage Gain Slider Event Callback
// ============================================================
// Format gain dB for labels. At slider minimum (-60 dB), shows "-inf".
static void format_gain_label(char *buf, int bufsize, float db) {
  if (db <= (float)STAGE_GAIN_MIN_DB)
    snprintf(buf, bufsize, "-inf");
  else
    snprintf(buf, bufsize, "%+.1f", db);
}

// ============================================================
// Zero-snap: quick tap near 0 dB reference → snap to unity gain
// ============================================================
// If the user touches a stage gain slider within ±3 dB of 0 and releases
// within 300 ms, the slider snaps to exactly 0.0 dB. Handles linked mode.
#define ZERO_SNAP_ZONE 30 // ±3.0 dB in slider units (val / 10)
#define ZERO_SNAP_MS 300  // Max press duration to trigger snap

static uint32_t gain_press_time = 0;

static void stage_gain_snap_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_PRESSED) {
    gain_press_time = lv_tick_get();
    return;
  }

  // LV_EVENT_RELEASED
  if ((lv_tick_get() - gain_press_time) > ZERO_SNAP_MS)
    return;

  int val = lv_slider_get_value(slider);
  if (val < -ZERO_SNAP_ZONE || val > ZERO_SNAP_ZONE)
    return;

  // Quick tap near zero — snap to unity
  bool is_left = (slider == stage->stage_gain_slider_l);

  auto set_gain = [&](int channel, float gain_db) {
    if (stage == &stage_input)
      dsp_set_input_gain(channel, gain_db);
    else if (stage == &stage_output)
      dsp_set_output_gain(channel, gain_db);
    else
      low_set_output_gain(channel, gain_db);
  };

  if (stage->channel_mode == CH_MODE_LINKED) {
    stage->stage_gain_l_db = 0.0f;
    stage->stage_gain_r_db = 0.0f;
    lv_slider_set_value(stage->stage_gain_slider_l, 0, LV_ANIM_OFF);
    lv_slider_set_value(stage->stage_gain_slider_r, 0, LV_ANIM_OFF);
    set_gain(0, 0.0f);
    set_gain(1, 0.0f);
  } else {
    if (is_left) {
      stage->stage_gain_l_db = 0.0f;
      lv_slider_set_value(slider, 0, LV_ANIM_OFF);
      set_gain(0, 0.0f);
    } else {
      stage->stage_gain_r_db = 0.0f;
      lv_slider_set_value(slider, 0, LV_ANIM_OFF);
      set_gain(1, 0.0f);
    }
  }

  if (stage == &stage_input)
    sync_input_gain_to_config();
  preset_mark_dirty();
}

// ============================================================
// EQ band gain zero-snap: quick tap near 0 dB → snap to flat
// ============================================================
// Same UX as stage gain snap.  Disabled in fine mode (slider is
// remapped so value 0 ≠ 0 dB, and the user is doing precision work).
#define BAND_GAIN_SNAP_ZONE 30 // ±3.0 dB in slider units (val/10)
#define BAND_GAIN_SNAP_MS 300  // Max press duration
// (Constants above are still used by param_slider_snap_cb and lk_tile_lock_cb.)

static void stage_gain_slider_event_cb(lv_event_t *e) {
  if (updating_gain_from_mirror)
    return;

  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  int val = lv_slider_get_value(slider);
  float db = val / 10.0f; // Convert to dB (0.1dB resolution)

  // Determine which channel this slider controls
  bool is_left = (slider == stage->stage_gain_slider_l);
  bool is_right = (slider == stage->stage_gain_slider_r);

  // Per-stage gain setter dispatcher (Phase 6: input/output/sub).
  auto set_gain = [&](int channel, float gain_db) {
    if (stage == &stage_input)
      dsp_set_input_gain(channel, gain_db);
    else if (stage == &stage_output)
      dsp_set_output_gain(channel, gain_db);
    else
      low_set_output_gain(channel, gain_db);
  };

  if (stage->channel_mode == CH_MODE_LINKED) {
    // In linked mode, both sliders move together
    stage->stage_gain_l_db = db;
    stage->stage_gain_r_db = db;
    lv_slider_set_value(stage->stage_gain_slider_l, val, LV_ANIM_OFF);
    lv_slider_set_value(stage->stage_gain_slider_r, val, LV_ANIM_OFF);

    set_gain(0, db);
    set_gain(1, db);
  } else {
    // Independent L/R mode
    if (is_left) {
      stage->stage_gain_l_db = db;
      set_gain(0, db);
    } else if (is_right) {
      stage->stage_gain_r_db = db;
      set_gain(1, db);
    }
  }

  // Mirror input gain to config page
  if (stage == &stage_input) {
    sync_input_gain_to_config();
  }
  preset_mark_dirty();
}

// Config page input gain slider callback
static void config_input_gain_slider_event_cb(lv_event_t *e) {
  if (updating_gain_from_mirror)
    return;

  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);

  int val = lv_slider_get_value(slider);
  float db = val / 10.0f;

  bool is_left = (slider == sys_config.input_gain_slider_l);
  bool is_right = (slider == sys_config.input_gain_slider_r);

  if (stage_input.channel_mode == CH_MODE_LINKED) {
    // In linked mode, both sliders move together
    stage_input.stage_gain_l_db = db;
    stage_input.stage_gain_r_db = db;
    lv_slider_set_value(sys_config.input_gain_slider_l, val, LV_ANIM_OFF);
    lv_slider_set_value(sys_config.input_gain_slider_r, val, LV_ANIM_OFF);
    dsp_set_input_gain(0, db);
    dsp_set_input_gain(1, db);
  } else {
    if (is_left) {
      stage_input.stage_gain_l_db = db;
      dsp_set_input_gain(0, db);
    } else if (is_right) {
      stage_input.stage_gain_r_db = db;
      dsp_set_input_gain(1, db);
    }
  }

  // Mirror back to input EQ page
  sync_input_gain_from_config();

  // Update config page labels
  char buf[16];
  format_gain_label(buf, sizeof(buf), stage_input.stage_gain_l_db);
  lv_label_set_text(sys_config.input_gain_label_l, buf);
  format_gain_label(buf, sizeof(buf), stage_input.stage_gain_r_db);
  lv_label_set_text(sys_config.input_gain_label_r, buf);
}

// Sync input gain sliders from EQ page to config page
static void sync_input_gain_to_config(void) {
  if (!sys_config.input_gain_slider_l || !sys_config.input_gain_slider_r)
    return;

  updating_gain_from_mirror = true;
  lv_slider_set_value(sys_config.input_gain_slider_l,
                      (int)(stage_input.stage_gain_l_db * 10), LV_ANIM_OFF);
  lv_slider_set_value(sys_config.input_gain_slider_r,
                      (int)(stage_input.stage_gain_r_db * 10), LV_ANIM_OFF);
  updating_gain_from_mirror = false;

  // Update config page labels
  if (sys_config.input_gain_label_l && sys_config.input_gain_label_r) {
    char buf[16];
    format_gain_label(buf, sizeof(buf), stage_input.stage_gain_l_db);
    lv_label_set_text(sys_config.input_gain_label_l, buf);
    format_gain_label(buf, sizeof(buf), stage_input.stage_gain_r_db);
    lv_label_set_text(sys_config.input_gain_label_r, buf);
  }
}

// Sync input gain sliders from config page to EQ page
static void sync_input_gain_from_config(void) {
  if (!stage_input.stage_gain_slider_l || !stage_input.stage_gain_slider_r)
    return;

  updating_gain_from_mirror = true;
  lv_slider_set_value(stage_input.stage_gain_slider_l,
                      (int)(stage_input.stage_gain_l_db * 10), LV_ANIM_OFF);
  lv_slider_set_value(stage_input.stage_gain_slider_r,
                      (int)(stage_input.stage_gain_r_db * 10), LV_ANIM_OFF);
  updating_gain_from_mirror = false;
}

static lv_obj_t *create_ch_button(lv_obj_t *parent, eq_stage_t *stage, int pos,
                                  const char *text) {
  lv_obj_t *btn = lv_obj_create(parent);
  lv_obj_remove_style_all(btn);
  lv_obj_set_size(btn, CH_BTN_W, CH_BTN_H);
  lv_obj_set_pos(btn, CH_BTN_X + pos * (CH_BTN_W + CH_BTN_GAP), CH_BTN_Y);
  lv_obj_set_style_bg_color(btn, COL_CH_INACTIVE, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_60, 0);
  lv_obj_set_style_radius(btn, 2, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, channel_btn_cb, LV_EVENT_CLICKED, stage);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, COL_TEXT_DIM, 0);
  lv_obj_center(lbl);

  return btn;
}

// ============================================================
// Gain Popup Overlay
// ============================================================

static void gain_popup_slider_cb(lv_event_t *e) {
  if (updating_popup_from_mirror) return;

  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  
  // Reset timer on any slider move
  if (stage->gain_popup_timer) {
    lv_timer_reset(stage->gain_popup_timer);
  }

  int val = lv_slider_get_value(slider);
  float db = val / 10.0f;
  
  bool is_left = (slider == stage->gain_popup_slider_l);
  bool is_right = (slider == stage->gain_popup_slider_r);

  auto set_gain = [&](int channel, float gain_db) {
    if (stage == &stage_input)
      dsp_set_input_gain(channel, gain_db);
    else if (stage == &stage_output)
      dsp_set_output_gain(channel, gain_db);
    else
      low_set_output_gain(channel, gain_db);
  };

  if (stage->channel_mode == CH_MODE_LINKED) {
    stage->stage_gain_l_db = db;
    stage->stage_gain_r_db = db;
    
    updating_popup_from_mirror = true;
    if (is_left && stage->gain_popup_slider_r) {
      lv_slider_set_value(stage->gain_popup_slider_r, val, LV_ANIM_OFF);
    } else if (is_right && stage->gain_popup_slider_l) {
      lv_slider_set_value(stage->gain_popup_slider_l, val, LV_ANIM_OFF);
    }
    updating_popup_from_mirror = false;

    set_gain(0, db);
    set_gain(1, db);
  } else {
    if (is_left) {
      stage->stage_gain_l_db = db;
      set_gain(0, db);
    } else if (is_right) {
      stage->stage_gain_r_db = db;
      set_gain(1, db);
    }
  }

  // Update popup labels
  char buf[8];
  if (is_left || stage->channel_mode == CH_MODE_LINKED) {
    if (stage->gain_popup_val_l) {
      format_gain_label(buf, sizeof(buf), stage->stage_gain_l_db);
      lv_label_set_text(stage->gain_popup_val_l, buf);
    }
  }
  if (is_right || stage->channel_mode == CH_MODE_LINKED) {
    if (stage->gain_popup_val_r) {
      format_gain_label(buf, sizeof(buf), stage->stage_gain_r_db);
      lv_label_set_text(stage->gain_popup_val_r, buf);
    }
  }

  if (stage == &stage_input) sync_input_gain_to_config();
  preset_mark_dirty();
  update_all_preset_boxes();
}

static uint32_t popup_gain_press_time = 0;

static void gain_popup_snap_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_PRESSED) {
    popup_gain_press_time = lv_tick_get();
    if (stage->gain_popup_timer) lv_timer_reset(stage->gain_popup_timer);
    return;
  }

  if ((lv_tick_get() - popup_gain_press_time) > ZERO_SNAP_MS) return;

  int val = lv_slider_get_value(slider);
  if (val < -ZERO_SNAP_ZONE || val > ZERO_SNAP_ZONE) return;

  bool is_left = (slider == stage->gain_popup_slider_l);

  auto set_gain = [&](int channel, float gain_db) {
    if (stage == &stage_input)
      dsp_set_input_gain(channel, gain_db);
    else if (stage == &stage_output)
      dsp_set_output_gain(channel, gain_db);
    else
      low_set_output_gain(channel, gain_db);
  };

  updating_popup_from_mirror = true;
  if (stage->channel_mode == CH_MODE_LINKED) {
    stage->stage_gain_l_db = 0.0f;
    stage->stage_gain_r_db = 0.0f;
    lv_slider_set_value(stage->gain_popup_slider_l, 0, LV_ANIM_OFF);
    lv_slider_set_value(stage->gain_popup_slider_r, 0, LV_ANIM_OFF);
    set_gain(0, 0.0f);
    set_gain(1, 0.0f);
  } else {
    if (is_left) {
      stage->stage_gain_l_db = 0.0f;
      lv_slider_set_value(slider, 0, LV_ANIM_OFF);
      set_gain(0, 0.0f);
    } else {
      stage->stage_gain_r_db = 0.0f;
      lv_slider_set_value(slider, 0, LV_ANIM_OFF);
      set_gain(1, 0.0f);
    }
  }
  updating_popup_from_mirror = false;

  char buf[8];
  format_gain_label(buf, sizeof(buf), 0.0f);
  if (stage->channel_mode == CH_MODE_LINKED || is_left) {
    if (stage->gain_popup_val_l) lv_label_set_text(stage->gain_popup_val_l, buf);
  }
  if (stage->channel_mode == CH_MODE_LINKED || !is_left) {
    if (stage->gain_popup_val_r) lv_label_set_text(stage->gain_popup_val_r, buf);
  }

  if (stage == &stage_input) sync_input_gain_to_config();
  preset_mark_dirty();
  update_all_preset_boxes();
}

static void gain_popup_pressing_cb(lv_event_t *e) {
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  if (stage->gain_popup_timer) {
    lv_timer_reset(stage->gain_popup_timer);
  }
}

static void gain_popup_timer_cb(lv_timer_t *t) {
  eq_stage_t *stage = (eq_stage_t *)lv_timer_get_user_data(t);
  stage->gain_popup_timer = NULL; // NULL it before calling close to prevent double-delete
  gain_popup_close(stage);
}

static void gain_popup_close(eq_stage_t *stage) {
  if (stage->gain_popup_timer) {
    lv_timer_delete(stage->gain_popup_timer);
    stage->gain_popup_timer = NULL;
  }
  
  if (main_tileview) {
      lv_obj_add_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
  }

  if (stage->gain_popup) {
    lv_obj_add_flag(stage->gain_popup, LV_OBJ_FLAG_HIDDEN);
  }

  // Sync small sliders
  if (stage->stage_gain_slider_l) {
    lv_slider_set_value(stage->stage_gain_slider_l, (int)(stage->stage_gain_l_db * 10), LV_ANIM_OFF);
  }
  if (stage->stage_gain_slider_r) {
    lv_slider_set_value(stage->stage_gain_slider_r, (int)(stage->stage_gain_r_db * 10), LV_ANIM_OFF);
  }

  // Sync readout labels
  char buf[8];
  if (stage->stage_gain_readout_l) {
    format_gain_label(buf, sizeof(buf), stage->stage_gain_l_db);
    lv_label_set_text(stage->stage_gain_readout_l, buf);
  }
  if (stage->stage_gain_readout_r) {
    format_gain_label(buf, sizeof(buf), stage->stage_gain_r_db);
    lv_label_set_text(stage->stage_gain_readout_r, buf);
  }
}

static void gain_popup_open(eq_stage_t *stage) {
  if (!stage->gain_popup) return;

  if (main_tileview) {
      lv_obj_clear_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
  }

  updating_popup_from_mirror = true;
  if (stage->gain_popup_slider_l) {
    lv_slider_set_value(stage->gain_popup_slider_l, (int)(stage->stage_gain_l_db * 10), LV_ANIM_OFF);
  }
  if (stage->gain_popup_slider_r) {
    lv_slider_set_value(stage->gain_popup_slider_r, (int)(stage->stage_gain_r_db * 10), LV_ANIM_OFF);
  }
  updating_popup_from_mirror = false;

  char buf[8];
  if (stage->gain_popup_val_l) {
    format_gain_label(buf, sizeof(buf), stage->stage_gain_l_db);
    lv_label_set_text(stage->gain_popup_val_l, buf);
  }
  if (stage->gain_popup_val_r) {
    format_gain_label(buf, sizeof(buf), stage->stage_gain_r_db);
    lv_label_set_text(stage->gain_popup_val_r, buf);
  }

  lv_obj_move_foreground(stage->gain_popup);
  lv_obj_clear_flag(stage->gain_popup, LV_OBJ_FLAG_HIDDEN);

  if (stage->gain_popup_timer) {
    lv_timer_reset(stage->gain_popup_timer);
  } else {
    stage->gain_popup_timer = lv_timer_create(gain_popup_timer_cb, 1000, stage);
    lv_timer_set_repeat_count(stage->gain_popup_timer, 1);
  }
}

// ============================================================
// Limiter Popup Functions
// ============================================================

static float map_limiter_slider_to_threshold(int val) {
    if (val <= 700) {
        return -60.0f + (val / 700.0f) * 58.0f;
    } else {
        return -2.0f + ((val - 700) / 300.0f) * 2.0f;
    }
}

static int map_threshold_to_limiter_slider(float threshold) {
    if (threshold <= -2.0f) {
        return (int)(((threshold + 60.0f) / 58.0f) * 700.0f);
    } else {
        return 700 + (int)(((threshold + 2.0f) / 2.0f) * 300.0f);
    }
}

static void limiter_popup_close(void) {
  if (sys_config.limiter_popup_timer) {
    lv_timer_delete(sys_config.limiter_popup_timer);
    sys_config.limiter_popup_timer = NULL;
  }
  
  if (main_tileview) {
      lv_obj_add_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
  }

  if (sys_config.limiter_popup) {
    lv_obj_add_flag(sys_config.limiter_popup, LV_OBJ_FLAG_HIDDEN);
  }
  sys_config.active_limiter_popup = 0;

  // Sync small sliders
  if (sys_config.threshold_slider) {
    lv_slider_set_value(sys_config.threshold_slider, map_threshold_to_limiter_slider(sys_config.limiter_threshold), LV_ANIM_OFF);
  }
  if (sys_config.low_threshold_slider) {
    lv_slider_set_value(sys_config.low_threshold_slider, map_threshold_to_limiter_slider(sys_config.low_limiter_threshold), LV_ANIM_OFF);
  }
}

static void limiter_popup_timer_cb(lv_timer_t *t) {
  sys_config.limiter_popup_timer = NULL; // NULL it before calling close
  limiter_popup_close();
}

static bool updating_limiter_from_mirror = false;

static void limiter_popup_slider_cb(lv_event_t *e) {
  if (updating_limiter_from_mirror) return;

  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int raw_val = lv_slider_get_value(slider);
  float new_val = map_limiter_slider_to_threshold(raw_val);
  
  if (sys_config.active_limiter_popup == 1) {
      sys_config.limiter_threshold = new_val;
      dsp_update_limiter(sys_config.limiter_enabled, new_val);
      
      // Update label
      if (sys_config.limiter_popup_val) {
          char buf[16];
          snprintf(buf, sizeof(buf), "%.1fdB", new_val);
          lv_label_set_text(sys_config.limiter_popup_val, buf);
      }
      // Mirror to underlying config page slider/label
      if (sys_config.threshold_val_label) {
          char buf[16];
          snprintf(buf, sizeof(buf), "%.1fdB", new_val);
          lv_label_set_text(sys_config.threshold_val_label, buf);
      }
      if (sys_config.threshold_slider) {
          lv_slider_set_value(sys_config.threshold_slider, raw_val, LV_ANIM_OFF);
      }
  } else if (sys_config.active_limiter_popup == 2) {
      sys_config.low_limiter_threshold = new_val;
      low_update_limiter(sys_config.low_limiter_enabled, new_val);
      
      // Update label
      if (sys_config.limiter_popup_val) {
          char buf[16];
          snprintf(buf, sizeof(buf), "%.1fdB", new_val);
          lv_label_set_text(sys_config.limiter_popup_val, buf);
      }
      // Mirror to underlying config page slider/label
      if (sys_config.low_threshold_val_label) {
          char buf[16];
          snprintf(buf, sizeof(buf), "%.1fdB", new_val);
          lv_label_set_text(sys_config.low_threshold_val_label, buf);
      }
      if (sys_config.low_threshold_slider) {
          lv_slider_set_value(sys_config.low_threshold_slider, raw_val, LV_ANIM_OFF);
      }
  }

  // Reset auto-close timer
  if (sys_config.limiter_popup_timer) {
    lv_timer_reset(sys_config.limiter_popup_timer);
  } else {
    sys_config.limiter_popup_timer = lv_timer_create(limiter_popup_timer_cb, 1000, NULL);
    lv_timer_set_repeat_count(sys_config.limiter_popup_timer, 1);
  }

  preset_mark_dirty();
  update_all_preset_boxes();
}

static void limiter_popup_open(int limiter_idx, int source_y) {
  if (!tile_config_ref) return;

  sys_config.active_limiter_popup = limiter_idx;

  // Lock scrolling
  if (main_tileview) {
      lv_obj_clear_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
  }

  float current_val = (limiter_idx == 1) ? sys_config.limiter_threshold : sys_config.low_limiter_threshold;

  if (!sys_config.limiter_popup) {
    sys_config.limiter_popup = lv_obj_create(tile_config_ref);
    lv_obj_remove_style_all(sys_config.limiter_popup);
    // Extend popup to right edge (width 450)
    lv_obj_set_size(sys_config.limiter_popup, 450, 52);
    lv_obj_set_style_bg_color(sys_config.limiter_popup, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(sys_config.limiter_popup, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sys_config.limiter_popup, 1, 0);
    lv_obj_set_style_border_color(sys_config.limiter_popup, COL_TEXT_DIM, 0);
    lv_obj_set_style_radius(sys_config.limiter_popup, 4, 0);
    
    // Create readout label
    sys_config.limiter_popup_val = lv_label_create(sys_config.limiter_popup);
    lv_obj_set_style_text_font(sys_config.limiter_popup_val, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(sys_config.limiter_popup_val, COL_WHITE, 0);
    lv_obj_align(sys_config.limiter_popup_val, LV_ALIGN_RIGHT_MID, -16, 0);
    
    // Create large horizontal slider
    sys_config.limiter_popup_slider = lv_slider_create(sys_config.limiter_popup);
    lv_obj_set_size(sys_config.limiter_popup_slider, 360, 6);
    lv_obj_align(sys_config.limiter_popup_slider, LV_ALIGN_LEFT_MID, 16, 0);
    lv_slider_set_range(sys_config.limiter_popup_slider, 0, 1000);
    style_slider_slim(sys_config.limiter_popup_slider);
    
    lv_obj_add_event_cb(sys_config.limiter_popup_slider, limiter_popup_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // Close when tapping outside the slider but inside the popup
    lv_obj_add_flag(sys_config.limiter_popup, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(sys_config.limiter_popup, [](lv_event_t *e){
        limiter_popup_close();
    }, LV_EVENT_CLICKED, NULL);
  }

  // Position it vertically aligned with the row, starting near the left edge (centered leaves it at X=15)
  lv_obj_set_pos(sys_config.limiter_popup, (SCREEN_W - 450) / 2, source_y - 26);
  
  updating_limiter_from_mirror = true;
  lv_slider_set_value(sys_config.limiter_popup_slider, map_threshold_to_limiter_slider(current_val), LV_ANIM_OFF);
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1fdB", current_val);
  lv_label_set_text(sys_config.limiter_popup_val, buf);
  updating_limiter_from_mirror = false;

  lv_obj_clear_flag(sys_config.limiter_popup, LV_OBJ_FLAG_HIDDEN);

  if (sys_config.limiter_popup_timer) {
    lv_timer_reset(sys_config.limiter_popup_timer);
  } else {
    sys_config.limiter_popup_timer = lv_timer_create(limiter_popup_timer_cb, 1000, NULL);
    lv_timer_set_repeat_count(sys_config.limiter_popup_timer, 1);
  }
}

// Touch overlay handler for limiters
static void limiter_trigger_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING) return;
  
  int limiter_idx = (int)(intptr_t)lv_event_get_user_data(e);
  lv_obj_t *overlay = (lv_obj_t *)lv_event_get_target(e);
  
  if (sys_config.active_limiter_popup != limiter_idx) {
      // Calculate the visual center Y of the overlay to spawn the popup there
      lv_area_t coords;
      lv_obj_get_coords(overlay, &coords);
      int source_y = coords.y1 + (coords.y2 - coords.y1) / 2;
      
      limiter_popup_open(limiter_idx, source_y);
  }
  
  // Forward dragging coordinates exactly like gain popups
  lv_indev_t * indev = lv_indev_active();
  if (!indev || !sys_config.limiter_popup_slider) return;
  
  lv_point_t p;
  lv_indev_get_point(indev, &p);
  
  lv_area_t slider_coords;
  lv_obj_get_coords(sys_config.limiter_popup_slider, &slider_coords);
  
  int x_rel = p.x - slider_coords.x1;
  if (x_rel < 0) x_rel = 0;
  if (x_rel > 360) x_rel = 360;
  
  float ratio = (float)x_rel / 360.0f;
  int new_val = (int)(ratio * 1000.0f);
  
  lv_slider_set_value(sys_config.limiter_popup_slider, new_val, LV_ANIM_OFF);
  lv_obj_send_event(sys_config.limiter_popup_slider, LV_EVENT_VALUE_CHANGED, NULL);
}

static void create_gain_popup(lv_obj_t *parent, eq_stage_t *stage) {
  lv_obj_t *panel = lv_obj_create(parent);
  lv_obj_remove_style_all(panel);
  lv_obj_set_size(panel, 52, 272);
  lv_obj_set_pos(panel, 428, 0);
  lv_obj_set_style_bg_color(panel, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(panel, 1, 0);
  lv_obj_set_style_border_color(panel, lv_color_hex(0x666666), 0);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(panel, LV_OBJ_FLAG_GESTURE_BUBBLE);
  lv_obj_add_flag(panel, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
  stage->gain_popup = panel;

  // L/R Labels
  lv_obj_t *lbl_l = lv_label_create(panel);
  lv_label_set_text(lbl_l, "L");
  lv_obj_set_style_text_font(lbl_l, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(lbl_l, COL_TEXT_DIM, 0);
  lv_obj_set_pos(lbl_l, 11, 4);

  lv_obj_t *lbl_r = lv_label_create(panel);
  lv_label_set_text(lbl_r, "R");
  lv_obj_set_style_text_font(lbl_r, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(lbl_r, COL_TEXT_DIM, 0);
  lv_obj_set_pos(lbl_r, 33, 4);

  // Sliders
  stage->gain_popup_slider_l = lv_slider_create(panel);
  lv_obj_set_size(stage->gain_popup_slider_l, 6, 238);
  lv_obj_set_pos(stage->gain_popup_slider_l, 12, 16);
  lv_slider_set_range(stage->gain_popup_slider_l, STAGE_GAIN_SLIDER_MIN, STAGE_GAIN_SLIDER_MAX);
  lv_slider_set_value(stage->gain_popup_slider_l, STAGE_GAIN_DEFAULT * 10, LV_ANIM_OFF);
  style_slider_slim(stage->gain_popup_slider_l);
  lv_obj_set_style_pad_all(stage->gain_popup_slider_l, 6, LV_PART_KNOB);
  lv_obj_add_event_cb(stage->gain_popup_slider_l, gain_popup_slider_cb, LV_EVENT_VALUE_CHANGED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_l, gain_popup_snap_cb, LV_EVENT_PRESSED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_l, gain_popup_snap_cb, LV_EVENT_RELEASED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_l, gain_popup_pressing_cb, LV_EVENT_PRESSING, stage);

  stage->gain_popup_slider_r = lv_slider_create(panel);
  lv_obj_set_size(stage->gain_popup_slider_r, 6, 238);
  lv_obj_set_pos(stage->gain_popup_slider_r, 34, 16);
  lv_slider_set_range(stage->gain_popup_slider_r, STAGE_GAIN_SLIDER_MIN, STAGE_GAIN_SLIDER_MAX);
  lv_slider_set_value(stage->gain_popup_slider_r, STAGE_GAIN_DEFAULT * 10, LV_ANIM_OFF);
  style_slider_slim(stage->gain_popup_slider_r);
  lv_obj_set_style_pad_all(stage->gain_popup_slider_r, 6, LV_PART_KNOB);
  lv_obj_add_event_cb(stage->gain_popup_slider_r, gain_popup_slider_cb, LV_EVENT_VALUE_CHANGED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_r, gain_popup_snap_cb, LV_EVENT_PRESSED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_r, gain_popup_snap_cb, LV_EVENT_RELEASED, stage);
  lv_obj_add_event_cb(stage->gain_popup_slider_r, gain_popup_pressing_cb, LV_EVENT_PRESSING, stage);

  // 0dB line
  int zero_db_offset = (238 * (-STAGE_GAIN_SLIDER_MIN)) / (STAGE_GAIN_SLIDER_MAX - STAGE_GAIN_SLIDER_MIN);
  int zero_db_y = 16 + 238 - zero_db_offset;
  lv_obj_t *zero_line = lv_obj_create(panel);
  lv_obj_remove_style_all(zero_line);
  lv_obj_set_size(zero_line, 28, 1);
  lv_obj_set_pos(zero_line, 12, zero_db_y);
  lv_obj_set_style_bg_color(zero_line, COL_GRID_ZERO, 0);
  lv_obj_set_style_bg_opa(zero_line, LV_OPA_COVER, 0);
  lv_obj_clear_flag(zero_line, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

  // Value Labels
  stage->gain_popup_val_l = lv_label_create(panel);
  lv_label_set_text(stage->gain_popup_val_l, "0.0");
  lv_label_set_long_mode(stage->gain_popup_val_l, LV_LABEL_LONG_CLIP);
  lv_obj_set_size(stage->gain_popup_val_l, 26, 16);
  lv_obj_set_pos(stage->gain_popup_val_l, 0, 256);
  lv_obj_set_style_text_font(stage->gain_popup_val_l, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(stage->gain_popup_val_l, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(stage->gain_popup_val_l, LV_TEXT_ALIGN_CENTER, 0);

  stage->gain_popup_val_r = lv_label_create(panel);
  lv_label_set_text(stage->gain_popup_val_r, "0.0");
  lv_label_set_long_mode(stage->gain_popup_val_r, LV_LABEL_LONG_CLIP);
  lv_obj_set_size(stage->gain_popup_val_r, 26, 16);
  lv_obj_set_pos(stage->gain_popup_val_r, 26, 256);
  lv_obj_set_style_text_font(stage->gain_popup_val_r, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(stage->gain_popup_val_r, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_align(stage->gain_popup_val_r, LV_TEXT_ALIGN_CENTER, 0);
}

// ============================================================
// Redesigned control section — FREQ/GAIN/Q rings (Stage 1)
// ============================================================
// UI-local formatters for the ring centre text. These differ from the shared
// eq_data.h format_freq/format_q (which stay untouched for crossover + other
// callers): FREQ is split into a bare number + separate unit ("Hz"/"kHz"),
// and Q is shown to 1 decimal instead of 2.
static void ring_fmt_value(char *buf, size_t len, int param, float val) {
  switch (param) {
  case 0: // FREQ: number only; unit supplied separately
    if (val >= 1000.0f) snprintf(buf, len, "%.1f", val / 1000.0f);
    else                snprintf(buf, len, "%.0f", val);
    break;
  case 1: // GAIN: signed, 1 decimal
    snprintf(buf, len, "%+.1f", val);
    break;
  case 2: // Q: 1 decimal (ring UI diverges from format_q's %.2f)
    snprintf(buf, len, "%.1f", val);
    break;
  default:
    buf[0] = '\0';
  }
}

static const char *ring_unit_str(int param, float val) {
  switch (param) {
  case 0: return (val >= 1000.0f) ? "kHz" : "Hz";
  case 1: return "dB";
  case 2: return "";
  }
  return "";
}

// Ring centre X for each of the 3 params.
static inline int ring_center_x(int param) {
  return (param == 0) ? RING_CX0 : (param == 1) ? RING_CX1 : RING_CX2;
}

// Retarget the single parameter slider to the currently selected param: set its
// range for that param and its value from the selected band. FREQ/Q use the
// 0..*_SLIDER_MAX integer range; GAIN uses GAIN_SLIDER_MIN..MAX. The existing
// *_to_slider helpers handle per-param log/linear mapping and fine-mode scaling.
static void param_slider_retarget(eq_stage_t *stage) {
  if (!stage->param_slider_obj) return;
  eq_band_t *b = &get_bands(stage)[stage->selected_band];
  lv_obj_t *s = stage->param_slider_obj;
  // Indicator colour tracks the selected band (matches ring/marker accent).
  lv_obj_set_style_bg_color(s, band_color(stage->selected_band), LV_PART_INDICATOR);
  // Centre tick marks 0 dB — meaningful for GAIN only.
  if (stage->slider_center_mark) {
    if (stage->selected_param == 1)
      lv_obj_clear_flag(stage->slider_center_mark, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(stage->slider_center_mark, LV_OBJ_FLAG_HIDDEN);
  }
  switch (stage->selected_param) {
  case 0:
    lv_slider_set_range(s, 0, FREQ_SLIDER_MAX);
    lv_slider_set_value(s, freq_to_slider(stage, b->freq), LV_ANIM_OFF);
    break;
  case 1:
    lv_slider_set_range(s, GAIN_SLIDER_MIN, GAIN_SLIDER_MAX);
    lv_slider_set_value(s, gain_to_slider(stage, b->gain), LV_ANIM_OFF);
    break;
  case 2:
    lv_slider_set_range(s, 0, Q_SLIDER_MAX);
    lv_slider_set_value(s, q_to_slider(stage, b->q), LV_ANIM_OFF);
    break;
  }
}

// Value-changed handler for the parameter slider. Writes the selected param of
// the selected band, then does the standard commit chain (curve dirty, mirror,
// DSP push, preset dirty) and refreshes the rings. GAIN snaps to 0 near centre
// unless fine mode is active.
static void param_slider_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_obj_t *s = (lv_obj_t *)lv_event_get_target(e);
  int val = lv_slider_get_value(s);
  eq_band_t *b = &get_bands(stage)[stage->selected_band];

  fine_readout_touch(stage);   // arm/refresh fine-adjust readout (no-op if not fine)

  switch (stage->selected_param) {
  case 0:
    b->freq = slider_to_freq(stage, val);
    break;
  case 1:
    b->gain = slider_to_gain(stage, val);
    break;
  case 2:
    b->q = slider_to_q(stage, val);
    break;
  }

  stage->curve_dirty = true;
  mirror_band_if_linked(stage, stage->selected_band);
  dsp_push_band(stage, stage->selected_band);
  if (stage->ring_area_obj) lv_obj_invalidate(stage->ring_area_obj);
  preset_mark_dirty();
  update_all_preset_boxes();
}

// Quick-tap snap-to-0 for the parameter slider, active only when GAIN is the
// selected param and fine mode is off. A short press (< snap ms) released within
// the centre zone snaps gain to 0 dB, using BAND_GAIN_SNAP_ZONE / _MS.
static uint32_t param_slider_press_time = 0;
// Tile-swipe lockout: while the param slider is held, disable the tileview's
// own scroll so a horizontal drag adjusts the value instead of paging. Applies
// to all params (separate from the GAIN-only snap cb). Restore on RELEASED and
// PRESS_LOST (backstop against a stuck-disabled tileview if the press is lost).
// Lockout container press/release: clear the tileview's own SCROLLABLE while the
// strip is touched so a horizontal track drag doesn't page. The container gets
// PRESSED before the tileview SCROLL_BEGIN (verified), so this pre-empts the
// swipe — the same lever the gain popup uses on open. Restore on release/lost.
static void lk_tile_lock_cb(lv_event_t *e) {
  lv_event_code_t code = lv_event_get_code(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  if (!main_tileview) return;
  if (code == LV_EVENT_PRESSED) {
    lv_obj_clear_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
    param_slider_press_time = lv_tick_get();
    if (stage) fine_readout_touch(stage);   // arm fine readout on track press
    return;
  }
  // RELEASED or PRESS_LOST
  lv_obj_add_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);
  if (code != LV_EVENT_RELEASED) return;

  // Quick tap near the centre line -> snap GAIN to 0 dB. Handles taps on the
  // track (which fall through the slider's ADV_HITTEST and land here, not on
  // the slider), so param_slider_snap_cb never sees them.
  if (!stage || stage->selected_param != 1 || stage->fine_mode) return;
  if ((lv_tick_get() - param_slider_press_time) > BAND_GAIN_SNAP_MS) return;

  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;
  lv_point_t pt;
  lv_indev_get_point(indev, &pt);
  // Snap zone: ±BAND_GAIN_SNAP_ZONE slider units mapped to px across the GAIN
  // track (PSL_W over 300 units).
  int zone_px = (BAND_GAIN_SNAP_ZONE * PSL_W) / 300;   // 30*464/300 = 46
  if (pt.x < PSL_CX - zone_px || pt.x > PSL_CX + zone_px) return;

  eq_band_t *b = &get_bands(stage)[stage->selected_band];
  b->gain = 0.0f;
  if (stage->param_slider_obj) lv_slider_set_value(stage->param_slider_obj, 0, LV_ANIM_OFF);
  stage->curve_dirty = true;
  mirror_band_if_linked(stage, stage->selected_band);
  dsp_push_band(stage, stage->selected_band);
  if (stage->ring_area_obj) lv_obj_invalidate(stage->ring_area_obj);
  preset_mark_dirty();
  update_all_preset_boxes();
}

static void param_slider_snap_cb(lv_event_t *e) {
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_obj_t *s = (lv_obj_t *)lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_PRESSED)
    fine_readout_touch(stage);   // all params; no-op if not fine

  if (stage->selected_param != 1) return;   // GAIN only (snap logic below)
  if (code == LV_EVENT_PRESSED) {
    param_slider_press_time = lv_tick_get();
    return;
  }
  // LV_EVENT_RELEASED
  if (stage->fine_mode) return;
  if ((lv_tick_get() - param_slider_press_time) > BAND_GAIN_SNAP_MS) return;

  int val = lv_slider_get_value(s);
  if (val < -BAND_GAIN_SNAP_ZONE || val > BAND_GAIN_SNAP_ZONE) return;

  eq_band_t *b = &get_bands(stage)[stage->selected_band];
  b->gain = 0.0f;
  lv_slider_set_value(s, 0, LV_ANIM_OFF);
  stage->curve_dirty = true;
  mirror_band_if_linked(stage, stage->selected_band);
  dsp_push_band(stage, stage->selected_band);
  if (stage->ring_area_obj) lv_obj_invalidate(stage->ring_area_obj);
  preset_mark_dirty();
  update_all_preset_boxes();
}

// Custom draw: 3 gap-down arcs (track + accent fill), centre value+unit text,
// and the FREQ/GAIN/Q param label below each. Runs on ring_area_obj.
// LVGL arc angles on this build: 0=right, 90=bottom, clockwise (see the
// RING_*_DEG defines for the header-doc discrepancy). Track drawn
// RING_START_DEG->RING_END_DEG clockwise over the top. Fill shares the bottom
// end (RING_END_DEG), its start sweeping in as the value's fraction grows.
// Decimals for the fine readout, derived from the actual per-step change of the
// slider mapping at the current position (probe slider_to_X at val±1). Capped 2.
static int fine_readout_decimals(eq_stage_t *stage, int param) {
  int sv, smax;
  float here, next;
  switch (param) {
    case 0: smax = FREQ_SLIDER_MAX; sv = freq_to_slider(stage, get_bands(stage)[stage->selected_band].freq);
            break;
    case 1: smax = GAIN_SLIDER_MAX; sv = gain_to_slider(stage, get_bands(stage)[stage->selected_band].gain);
            break;
    default: smax = Q_SLIDER_MAX; sv = q_to_slider(stage, get_bands(stage)[stage->selected_band].q);
            break;
  }
  int a = sv, bstep = (sv < smax) ? sv + 1 : sv - 1;
  switch (param) {
    case 0: here = slider_to_freq(stage, a); next = slider_to_freq(stage, bstep); break;
    case 1: here = slider_to_gain(stage, a); next = slider_to_gain(stage, bstep); break;
    default: here = slider_to_q(stage, a);   next = slider_to_q(stage, bstep);   break;
  }
  float step = fabsf(next - here);
  if (step <= 0.0f) return 2;
  // Show one digit finer than the step: dp = ceil(-log10(step)) + 0, clamped 0..2.
  int dp = (int)ceilf(-log10f(step));
  if (dp < 0) dp = 0;
  if (dp > 2) dp = 2;
  return dp;
}

// Format the fine readout value + unit for a param, at the given decimals.
// FREQ shown in Hz (fine ranges are narrow; kHz split not needed here).
static void fine_readout_fmt(char *vbuf, int vsz, char *ubuf, int usz,
                             eq_stage_t *stage, int param, int dp) {
  eq_band_t *bd = &get_bands(stage)[stage->selected_band];
  switch (param) {
    case 0: snprintf(vbuf, vsz, "%.*f", dp, bd->freq); snprintf(ubuf, usz, "Hz"); break;
    case 1: snprintf(vbuf, vsz, "%+.*f", dp, bd->gain); snprintf(ubuf, usz, "dB"); break;
    default: snprintf(vbuf, vsz, "%.*f", dp, bd->q);   snprintf(ubuf, usz, "Q");  break;
  }
}

// Update the fine-readout overlay: set value+unit text, size to content, and
// centre the box on the selected arc. Shows the box. No-op if not built.
static void fine_readout_update(eq_stage_t *stage) {
  if (!stage->fine_readout_box || !stage->fine_readout_val ||
      !stage->fine_readout_unit)
    return;
  int sp = stage->selected_param;
  int dp = fine_readout_decimals(stage, sp);
  char vbuf[12], ubuf[4];
  fine_readout_fmt(vbuf, sizeof(vbuf), ubuf, sizeof(ubuf), stage, sp, dp);
  lv_label_set_text(stage->fine_readout_val, vbuf);
  lv_label_set_text(stage->fine_readout_unit, ubuf);

  lv_obj_clear_flag(stage->fine_readout_box, LV_OBJ_FLAG_HIDDEN);
  lv_obj_update_layout(stage->fine_readout_box);   // force content-size calc

  int w = lv_obj_get_width(stage->fine_readout_box);
  int h = lv_obj_get_height(stage->fine_readout_box);
  int cx = ring_center_x(sp);                       // tile-absolute = screen X
  int cy = RING_AREA_Y + RING_CY;                   // arc centre screen Y (204)
  lv_obj_set_pos(stage->fine_readout_box, cx - w / 2, cy - h / 2);
}

static void fine_readout_hide(eq_stage_t *stage) {
  if (stage->fine_readout_box)
    lv_obj_add_flag(stage->fine_readout_box, LV_OBJ_FLAG_HIDDEN);
}

static void ring_area_draw_cb(lv_event_t *e) {
  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t   *obj   = (lv_obj_t *)lv_event_get_target(e);
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);

  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  int ox = coords.x1, oy = coords.y1;

  eq_band_t *b = &get_bands(stage)[stage->selected_band];
  lv_color_t accent = band_color(stage->selected_band);

  // Static buffers: lv_draw_label reads the pointer after this frame returns
  // (LVGL v9 async-draw rule, per the marker block).
  static char ring_val_buf[3][8];
  static char ring_unit_buf[3][6];
  static const char *param_labels[3] = {"FREQ", "GAIN", "Q"};

  for (int p = 0; p < 3; p++) {
    float val  = get_band_param(b, p);
    float frac = value_to_norm(p, val);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    int cx = ox + ring_center_x(p);
    int cy = oy + RING_CY;

    // Track (dim)
    lv_draw_arc_dsc_t adsc;
    lv_draw_arc_dsc_init(&adsc);
    adsc.center.x = cx;
    adsc.center.y = cy;
    adsc.radius   = RING_R;
    adsc.width    = RING_STROKE;
    adsc.color    = COL_TEXT_DIM;
    adsc.opa      = LV_OPA_COVER;
    adsc.start_angle = RING_START_DEG;   // 122 (just CW of bottom)
    adsc.end_angle   = RING_END_DEG;     // 418 (=58, bottom), CW over the top
    lv_draw_arc(layer, &adsc);

    // Accent fill. FREQ/Q (unipolar): grow clockwise from the bottom-left end
    // (RING_START_DEG) by TRACK*frac. GAIN (bipolar): anchored at top (the
    // sweep midpoint); positive fills clockwise toward the right, negative
    // fills counter-clockwise toward the left; nothing at gain=0.
    if (p == 1) {
      // GAIN bipolar. Midpoint angle = START + TRACK/2 (top of the ring).
      int mid = RING_START_DEG + RING_TRACK_DEG / 2;   // 270
      int half = RING_TRACK_DEG / 2;                   // 148
      float dev = frac - 0.5f;                          // -0.5..+0.5
      if (dev > 0.001f) {
        adsc.color       = accent;
        adsc.start_angle = mid;
        adsc.end_angle   = mid + (int)(half * dev * 2.0f + 0.5f);
        lv_draw_arc(layer, &adsc);
      } else if (dev < -0.001f) {
        adsc.color       = accent;
        adsc.start_angle = mid - (int)(half * (-dev) * 2.0f + 0.5f);
        adsc.end_angle   = mid;
        lv_draw_arc(layer, &adsc);
      }
      // dev ~= 0: no fill (grey track only)
    } else {
      // FREQ / Q unipolar: fill grows clockwise from bottom-left.
      if (frac > 0.001f) {
        adsc.color       = accent;
        adsc.start_angle = RING_START_DEG;
        adsc.end_angle   = RING_START_DEG + (int)(RING_TRACK_DEG * frac + 0.5f);
        lv_draw_arc(layer, &adsc);
      }
    }

    // Centre value text (selected param brighter). Value upper-middle, unit
    // below — offsets mirror the mockup (value ~R above centre, unit under).
    ring_fmt_value(ring_val_buf[p], sizeof(ring_val_buf[p]), p, val);
    snprintf(ring_unit_buf[p], sizeof(ring_unit_buf[p]), "%s",
             ring_unit_str(p, val));

    lv_draw_label_dsc_t vdsc;
    lv_draw_label_dsc_init(&vdsc);
    vdsc.color = (p == stage->selected_param) ? COL_WHITE : COL_TEXT_DIM;
    vdsc.font  = &lv_font_montserrat_14;
    vdsc.text  = ring_val_buf[p];
    vdsc.align = LV_TEXT_ALIGN_CENTER;
    {
      int vh = lv_font_get_line_height(vdsc.font);
      lv_area_t va = {(lv_coord_t)(cx - RING_R), (lv_coord_t)(cy - vh + 2),
                      (lv_coord_t)(cx + RING_R), (lv_coord_t)(cy + 2)};
      lv_draw_label(layer, &vdsc, &va);
    }

    // Unit line, just below the value (white when selected, else dim)
    lv_draw_label_dsc_t udsc;
    lv_draw_label_dsc_init(&udsc);
    udsc.color = (p == stage->selected_param) ? COL_WHITE : COL_TEXT_DIM;
    udsc.font  = &lv_font_montserrat_10;
    udsc.text  = ring_unit_buf[p];
    udsc.align = LV_TEXT_ALIGN_CENTER;
    {
      lv_area_t ua = {(lv_coord_t)(cx - RING_R), (lv_coord_t)(cy + 4),
                      (lv_coord_t)(cx + RING_R), (lv_coord_t)(cy + 16)};
      lv_draw_label(layer, &udsc, &ua);
    }

    // Selected-param indicator: 10px white rect centred in the bottom gap, on
    // the arc's stroke centreline (cy + RING_R - RING_STROKE/2 = cy+24), 2px.
    if (p == stage->selected_param) {
      lv_draw_rect_dsc_t gdsc;
      lv_draw_rect_dsc_init(&gdsc);
      gdsc.bg_color = COL_WHITE;
      gdsc.bg_opa   = LV_OPA_COVER;
      int gy = cy + RING_R - RING_STROKE / 2 - 3;   // 21 (moved up 3px)
      lv_area_t gm = {(lv_coord_t)(cx - 6), (lv_coord_t)(gy - 1),
                      (lv_coord_t)(cx + 6), (lv_coord_t)(gy + 1)};
      lv_draw_rect(layer, &gdsc, &gm);
    }

    // Param label (FREQ/GAIN/Q) below the ring
    lv_draw_label_dsc_t pdsc;
    lv_draw_label_dsc_init(&pdsc);
    pdsc.color = (p == stage->selected_param) ? COL_WHITE : COL_TEXT_DIM;
    pdsc.font  = &lv_font_montserrat_10;
    pdsc.text  = param_labels[p];
    pdsc.align = LV_TEXT_ALIGN_CENTER;
    {
      lv_area_t pa = {(lv_coord_t)(cx - RING_R), (lv_coord_t)(cy + RING_R + 2),
                      (lv_coord_t)(cx + RING_R), (lv_coord_t)(cy + RING_R + 14)};
      lv_draw_label(layer, &pdsc, &pa);
    }
  }

  // --- Band readout: "< N >" over "BAND", left of the rings ---------------
  // N = selected_band+1 (matches curve marker, line ~1806). Arrows are drawn
  // static text; hit-testing lives in ring_area_click_cb. Vertical rows are
  // shared with the ring value row (montserrat_18 number) and the FREQ/GAIN/Q
  // param row (montserrat_10 "BAND" label), derived from RING_CY / RING_R.
  {
    int cy = oy + RING_CY;

    // Fixed column boxes (screen-local X + area origin ox). Number box is the
    // anchor: band_cx is pinned so shrinking the gap moves the arrows inward
    // rather than shifting the number. Arrow boxes 12px, number box 26px
    // (two digits at montserrat_22 — GUESS, eyeball "15" for clip), gap 1px.
    int aw   = 16;                             // arrow box width (fits montserrat_16)
    int nw   = 26;                             // number box width
    int band_cx = ox + BAND_RO_ARROW_X0 + 28;
    int nx0  = band_cx - nw / 2;               // number box left (derived)
    // Arrow inner edges sit BAND_RO_ARROW_INSET px outside the number box.
    int ax0  = (nx0 - BAND_RO_ARROW_INSET) - aw;   // '<' box left
    int rx0  = nx0 + nw + BAND_RO_ARROW_INSET;     // '>' box left
    // Box spans '<' outer to '>' outer, +1px pad each side.
    int box_x0 = ax0 - 1;
    int box_x1 = rx0 + aw + 1;

    // Number row: vertically centred on the ring value text row (cy-relative).
    // Ring value box is (cy - vh + 2 .. cy + 2); reuse that band for the number.
    int nvh = lv_font_get_line_height(&lv_font_montserrat_22);
    int nrow_y0 = cy - nvh + 2 + BAND_RO_ROW_DY;
    int nrow_y1 = cy + 2 + BAND_RO_ROW_DY;
    // Arrow row: vertically centred to the number using its own line height.
    int avh = lv_font_get_line_height(&lv_font_montserrat_16);
    int arow_y0 = (nrow_y0 + nrow_y1) / 2 - avh / 2;
    int arow_y1 = arow_y0 + avh;

    // Light-grey rounded box behind the assembly. Height = arc height (2*RING_R),
    // centred on the arc centre Y. Width spans the arrows +1px pad (box_x0/x1).
    {
      lv_draw_rect_dsc_t boxd;
      lv_draw_rect_dsc_init(&boxd);
      boxd.bg_color = lv_color_hex(0x141414);   // neutral, slightly lighter than black bg
      boxd.bg_opa   = LV_OPA_COVER;
      boxd.radius   = 3;
      lv_area_t boxa = {(lv_coord_t)box_x0, (lv_coord_t)(cy - RING_R),
                        (lv_coord_t)box_x1, (lv_coord_t)(cy + RING_R - 4)};
      lv_draw_rect(layer, &boxd, &boxa);
    }

    static char band_num_buf[4];
    snprintf(band_num_buf, sizeof(band_num_buf), "%d", stage->selected_band + 1);

    // Number (accent colour of the selected band, montserrat_22, centred)
    lv_draw_label_dsc_t ndsc;
    lv_draw_label_dsc_init(&ndsc);
    ndsc.color = accent;
    ndsc.font  = &lv_font_montserrat_22;
    ndsc.text  = band_num_buf;
    ndsc.align = LV_TEXT_ALIGN_CENTER;
    {
      lv_area_t na = {(lv_coord_t)nx0, (lv_coord_t)nrow_y0,
                      (lv_coord_t)(nx0 + nw), (lv_coord_t)nrow_y1};
      lv_draw_label(layer, &ndsc, &na);
    }

    // Arrows (white, montserrat_16, centred in their boxes)
    lv_draw_label_dsc_t adsc2;
    lv_draw_label_dsc_init(&adsc2);
    adsc2.color = COL_WHITE;
    adsc2.font  = &lv_font_montserrat_16;
    adsc2.align = LV_TEXT_ALIGN_CENTER;
    static const char *lt = "<";
    static const char *gt = ">";
    adsc2.text = (char *)lt;
    {
      lv_area_t la = {(lv_coord_t)ax0, (lv_coord_t)arow_y0,
                      (lv_coord_t)(ax0 + aw), (lv_coord_t)arow_y1};
      lv_draw_label(layer, &adsc2, &la);
    }
    adsc2.text = (char *)gt;
    {
      lv_area_t ga = {(lv_coord_t)rx0, (lv_coord_t)arow_y0,
                      (lv_coord_t)(rx0 + aw), (lv_coord_t)arow_y1};
      lv_draw_label(layer, &adsc2, &ga);
    }

    // "BAND" label on the param row (montserrat_10, dim, centred on the number)
    lv_draw_label_dsc_t bdsc;
    lv_draw_label_dsc_init(&bdsc);
    bdsc.color = COL_TEXT_DIM;
    bdsc.font  = &lv_font_montserrat_10;
    bdsc.text  = (char *)"BAND";
    bdsc.align = LV_TEXT_ALIGN_CENTER;
    {
      lv_area_t ba = {(lv_coord_t)(band_cx - RING_R), (lv_coord_t)(cy + RING_R + 2),
                      (lv_coord_t)(band_cx + RING_R), (lv_coord_t)(cy + RING_R + 14)};
      lv_draw_label(layer, &bdsc, &ba);
    }
  }
}

// Tap a ring to select that parameter (drives the slider in a later stage).
// Hit-test by X against each ring centre, mirroring curve_area_click_cb.
static void ring_area_click_cb(lv_event_t *e) {
  eq_stage_t *stage = (eq_stage_t *)lv_event_get_user_data(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
  lv_indev_t *indev = lv_indev_active();
  if (!indev) return;

  lv_point_t pt;
  lv_indev_get_point(indev, &pt);
  lv_area_t coords;
  lv_obj_get_coords(obj, &coords);
  int lx = pt.x - coords.x1;   // local X within ring area
  int ly = pt.y - coords.y1;   // local Y within ring area

  // Band readout "< N >" hit-test (mirrors the draw geometry above). Column
  // spans local X [BAND_RO_ARROW_X0 .. rx_end], split at the number centre:
  // left half -> previous active band, right half -> next, wrapping per the
  // active channel's band count. Vertical band = RING_CY +/- BAND_RO_TAP_VPAD.
  {
    const int aw = 16, nw = 26;
    const int band_cx = BAND_RO_ARROW_X0 + 28;            // pinned centre (matches draw)
    const int nx0     = band_cx - nw / 2;                 // number left
    const int ax0     = (nx0 - BAND_RO_ARROW_INSET) - aw; // '<' left
    const int rx_end  = (nx0 + nw + BAND_RO_ARROW_INSET) + aw;  // '>' right edge
    // Tap zone widened 10px each side beyond the drawn glyph column.
    if (lx >= ax0 - 10 && lx <= rx_end + 10 &&
        ly >= RING_CY + BAND_RO_ROW_DY - BAND_RO_TAP_VPAD &&
        ly <= RING_CY + BAND_RO_ROW_DY + BAND_RO_TAP_VPAD) {
      int n = eq_ui_get_band_count(stage);
      if (n > 0) {
        int cur = stage->selected_band;
        int nxt = (lx < band_cx) ? (cur - 1 + n) % n : (cur + 1) % n;
        eq_ui_select_band(stage, nxt);
        lv_obj_invalidate(obj);
      }
      return;
    }
  }

  // Radial hit-test: select a ring if the tap falls within its inner circle
  // (radius = arc inner edge, RING_R - RING_STROKE). Full circle, including
  // across the bottom gap. Centre Y is RING_CY within the area.
  const int inner_r = RING_R - RING_STROKE;   // 22
  const int inner_r2 = inner_r * inner_r;
  int best = -1;
  for (int p = 0; p < 3; p++) {
    int dx = lx - ring_center_x(p);
    int dy = ly - RING_CY;
    if (dx * dx + dy * dy <= inner_r2) { best = p; break; }
  }
  if (best >= 0) {
    stage->selected_param = best;
    lv_obj_invalidate(obj);
    param_slider_retarget(stage);
  }
}

// ============================================================
// Build a single EQ page inside a parent container
// ============================================================
// Nav button geometry (shared by EQ/config/xover). Column at x = SCREEN_W-40,
// three 34x17 slots 4px apart. Top slot (NAV_Y0) is MUTE on EQ, empty elsewhere.
// Middle (NAV_Y1) = CFG or RET; bottom (NAV_Y2) = VIS. VIS pinned across pages.
#define NAV_BTN_W  34
#define NAV_BTN_H  17
#define NAV_BTN_X  (SCREEN_W - 40)
#define NAV_Y0     (STAGE_GAIN_SL_Y + STAGE_GAIN_SL_H + 18)   // 182 (mute row)
#define NAV_Y1     (NAV_Y0 + NAV_BTN_H + 4)                   // 203
#define NAV_Y2     (NAV_Y1 + NAV_BTN_H + 4)                   // 224

// Create a solid nav button with a near-black label at (NAV_BTN_X, y).
static lv_obj_t *create_nav_button(lv_obj_t *parent, int y, const char *label,
                                   lv_event_cb_t cb) {
  lv_obj_t *b = lv_obj_create(parent);
  lv_obj_remove_style_all(b);
  lv_obj_set_size(b, NAV_BTN_W, NAV_BTN_H);
  lv_obj_set_pos(b, NAV_BTN_X, y);
  lv_obj_set_style_bg_color(b, COL_BOX_LIST, 0);
  lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(b, 2, 0);
  lv_obj_set_style_pad_all(b, 0, 0);
  lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(b);
  lv_label_set_text(lbl, label);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(0x1A1A1A), 0);
  lv_obj_center(lbl);
  return b;
}

static void create_eq_page(lv_obj_t *parent, eq_stage_t *stage) {
  // --- Level meter area (left of curve) ---
  lv_obj_t *meter_area_obj = lv_obj_create(parent);
  lv_obj_remove_style_all(meter_area_obj);
  lv_obj_set_size(meter_area_obj, METER_AREA_W, METER_AREA_H);
  lv_obj_set_pos(meter_area_obj, METER_AREA_X, METER_AREA_Y);
  lv_obj_set_style_bg_opa(meter_area_obj, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(meter_area_obj, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE |
                                                    LV_OBJ_FLAG_SCROLLABLE));
  lv_obj_add_event_cb(meter_area_obj, meter_area_draw_cb,
                      LV_EVENT_DRAW_MAIN_END, stage);
  stage->meter_area_obj = meter_area_obj;

  // --- Curve display area ---
  lv_obj_t *curve_area_obj = lv_obj_create(parent);
  lv_obj_remove_style_all(curve_area_obj);
  lv_obj_set_size(curve_area_obj, CURVE_W, CURVE_H);
  lv_obj_set_pos(curve_area_obj, CURVE_X, CURVE_Y);
  lv_obj_set_style_bg_opa(curve_area_obj, LV_OPA_TRANSP, 0);
  // Curve area is now clickable for marker tap-to-select (Phase 12).
  // SCROLLABLE remains cleared so tileview swipes work normally.
  lv_obj_clear_flag(curve_area_obj, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(curve_area_obj, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(curve_area_obj, curve_area_draw_cb,
                      LV_EVENT_DRAW_MAIN_END, stage);
  lv_obj_add_event_cb(curve_area_obj, curve_area_click_cb, LV_EVENT_CLICKED,
                      stage);
  stage->curve_area_obj = curve_area_obj;

  recompute_curve(stage);

  // --- Ring control area (Stage 1 redesign) ---
  // Transparent full-width object below the curve carrying the custom-drawn
  // FREQ/GAIN/Q rings. Clickable for ring tap-to-select; SCROLLABLE cleared so
  // tileview swipes still work (same pattern as curve_area_obj).
  stage->selected_param = 0;
  {
    lv_obj_t *ring_area = lv_obj_create(parent);
    lv_obj_remove_style_all(ring_area);
    lv_obj_set_size(ring_area, RING_AREA_W, RING_AREA_H);
    lv_obj_set_pos(ring_area, RING_AREA_X, RING_AREA_Y);
    lv_obj_set_style_bg_opa(ring_area, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(ring_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ring_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(ring_area, ring_area_draw_cb, LV_EVENT_DRAW_MAIN_END,
                        stage);
    lv_obj_add_event_cb(ring_area, ring_area_click_cb, LV_EVENT_CLICKED, stage);
    stage->ring_area_obj = ring_area;
  }

  // --- Parameter slider (Stage 2) ---
  // Single slider driving whichever ring param is selected. Range + value are
  // set by param_slider_retarget() on ring-tap and band-select. Snap-to-0 for
  // GAIN via param_slider_snap_cb (quick tap, fine mode off).
  {
    // Lockout container over the bottom strip. CLICKABLE so it wins the touch
    // hit-test across the whole strip (incl. the slider track, which ADV_HITTEST
    // would otherwise let fall through to the tileview scroll). SCROLLABLE +
    // GESTURE_BUBBLE cleared so it absorbs the horizontal drag instead of paging.
    // Transparent. Slider + tick are children; a track touch lands on the
    // container (no value jump), a knob touch still reaches the slider.
    lv_obj_t *lk = lv_obj_create(parent);
    lv_obj_remove_style_all(lk);
    lv_obj_set_size(lk, PSLK_W, PSLK_H);
    lv_obj_set_pos(lk, PSLK_X, PSLK_Y);
    lv_obj_set_style_bg_opa(lk, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(lk, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(lk, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_add_flag(lk, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(lk, lk_tile_lock_cb, LV_EVENT_PRESSED, stage);
    lv_obj_add_event_cb(lk, lk_tile_lock_cb, LV_EVENT_RELEASED, stage);
    lv_obj_add_event_cb(lk, lk_tile_lock_cb, LV_EVENT_PRESS_LOST, stage);

    // Centre tick first, so the knob renders over it. Vertical line pokes past
    // the knob top/bottom. Shown only for GAIN (toggled in param_slider_retarget).
    // Positions are container-local (container x=0 so local x = screen x; y -PSLK_Y).
    lv_obj_t *tick = lv_obj_create(lk);
    lv_obj_remove_style_all(tick);
    lv_obj_set_size(tick, PSL_TICK_W, 2 * PSL_TICK_HALF);
    lv_obj_set_pos(tick, PSL_CX - PSL_TICK_W / 2, (PSL_TICK_CY - PSL_TICK_HALF) - PSLK_Y);
    lv_obj_set_style_bg_color(tick, COL_TEXT_DIM, 0);
    lv_obj_set_style_bg_opa(tick, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(tick, 0, 0);
    lv_obj_clear_flag(tick, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    lv_obj_add_flag(tick, LV_OBJ_FLAG_HIDDEN);   // shown for GAIN only
    stage->slider_center_mark = tick;

    lv_obj_t *psl = lv_slider_create(lk);
    lv_obj_set_size(psl, PSL_W, PSL_H);
    lv_obj_set_pos(psl, PSL_X - PSLK_X, PSL_Y - PSLK_Y);   // container-local
    lv_slider_set_range(psl, 0, FREQ_SLIDER_MAX);   // retarget sets real range
    style_slider_slim(psl);
    lv_obj_set_style_pad_all(psl, 6, LV_PART_KNOB);
    lv_obj_add_flag(psl, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_add_event_cb(psl, param_slider_cb, LV_EVENT_VALUE_CHANGED, stage);
    lv_obj_add_event_cb(psl, param_slider_snap_cb, LV_EVENT_PRESSED, stage);
    lv_obj_add_event_cb(psl, param_slider_snap_cb, LV_EVENT_RELEASED, stage);
    stage->param_slider_obj = psl;
  }
  param_slider_retarget(stage);

  // --- Fine-adjust readout overlay: flex-column box (content-sized, translucent
  // black, 4px pad) with a 24pt value label + 18pt unit label. Parented to the
  // tile so it renders above the ring area; positioned/shown on fine adjust,
  // hidden otherwise. LV_USE_FLEX confirmed enabled in lv_conf.h.
  {
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(box, 194, 0);   // 76% opaque (was LV_OPA_70=70%; -20% rel transparency)
    lv_obj_set_style_radius(box, 3, 0);
    lv_obj_set_style_pad_all(box, 4, 0);
    lv_obj_set_size(box, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(box, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(box, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(box, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    lv_obj_add_flag(box, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *vlbl = lv_label_create(box);
    lv_obj_set_style_text_color(vlbl, COL_WHITE, 0);
    lv_obj_set_style_text_font(vlbl, &lv_font_montserrat_24, 0);
    lv_label_set_text(vlbl, "");

    lv_obj_t *ulbl = lv_label_create(box);
    lv_obj_set_style_text_color(ulbl, COL_WHITE, 0);
    lv_obj_set_style_text_font(ulbl, &lv_font_montserrat_18, 0);
    lv_label_set_text(ulbl, "");

    stage->fine_readout_box  = box;
    stage->fine_readout_val  = vlbl;
    stage->fine_readout_unit = ulbl;
  }

  // --- Filter-list "LIST" button (Phase 12) ---
  // Small button at bottom-right of curve area, just above the "20k" freq
  // label. Parented to the tile (same pattern as the sub-page routing
  // buttons) so coordinates are tile-absolute.
  //
  // Layout rationale:
  //   Right edge: CURVE_X + CURVE_W - 4  = 424  (4 px inset from curve right)
  //   Bottom:     CURVE_Y + CURVE_H - 14 = 156  (sits 1 px above "20k" label
  //                                              which spans y=157..169 on
  //                                              screen, per freq-axis draw)
  //   Size:       40 x 16, montserrat_10 — matches sub routing buttons.
  {
    lv_obj_t *list_btn = lv_obj_create(parent);
    lv_obj_remove_style_all(list_btn);
    lv_obj_set_size(list_btn, 38, 25);
    lv_obj_set_pos(list_btn, CURVE_X + CURVE_W - 4 - 38,
                   CURVE_Y + CURVE_H - 14 - 26);
    lv_obj_set_style_bg_opa(list_btn, LV_OPA_60, 0);
    lv_obj_set_style_bg_color(list_btn, COL_BOX_LIST, 0);
    //lv_obj_set_style_border_width(list_btn, 1, 0);
    //lv_obj_set_style_border_color(list_btn, COL_BOX_BORDER, 0);
    lv_obj_set_style_radius(list_btn, 3, 0);
    lv_obj_set_style_pad_all(list_btn, 0, 0);
    lv_obj_clear_flag(list_btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(list_btn, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(list_btn, list_btn_cb, LV_EVENT_CLICKED, stage);

    lv_obj_t *lbl = lv_label_create(list_btn);
    lv_label_set_text(lbl, "LIST");
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
    lv_obj_center(lbl);
  }

// --- Stage gain sliders (vertical, right side of graph) — display only ---
// These are now read-only position indicators. Tapping them opens the popup.
#define STAGE_GAIN_SL_X_L 441
#define STAGE_GAIN_SL_X_R 465
#define STAGE_GAIN_SL_W 6
#define STAGE_GAIN_SL_H (CURVE_H - 24)
#define STAGE_GAIN_LBL_Y (CURVE_Y + 2)
#define STAGE_GAIN_SL_Y (CURVE_Y + 18)

  // L label
  stage->stage_gain_label_l = lv_label_create(parent);
  lv_label_set_text(stage->stage_gain_label_l, "L");
  lv_obj_set_style_text_color(stage->stage_gain_label_l, COL_TEXT_DIM, 0);
  lv_obj_set_style_text_font(stage->stage_gain_label_l, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(stage->stage_gain_label_l, STAGE_GAIN_SL_X_L, STAGE_GAIN_LBL_Y);

  // R label
  stage->stage_gain_label_r = lv_label_create(parent);
  lv_label_set_text(stage->stage_gain_label_r, "R");
  lv_obj_set_style_text_color(stage->stage_gain_label_r, COL_TEXT_DIM, 0);
  lv_obj_set_style_text_font(stage->stage_gain_label_r, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(stage->stage_gain_label_r, STAGE_GAIN_SL_X_R, STAGE_GAIN_LBL_Y);

  // L slider — display only (not interactive; popup handles input)
  stage->stage_gain_slider_l = lv_slider_create(parent);
  lv_obj_set_size(stage->stage_gain_slider_l, STAGE_GAIN_SL_W, STAGE_GAIN_SL_H);
  lv_obj_set_pos(stage->stage_gain_slider_l, STAGE_GAIN_SL_X_L, STAGE_GAIN_SL_Y);
  lv_slider_set_range(stage->stage_gain_slider_l, STAGE_GAIN_SLIDER_MIN, STAGE_GAIN_SLIDER_MAX);
  lv_slider_set_value(stage->stage_gain_slider_l, STAGE_GAIN_DEFAULT * 10, LV_ANIM_OFF);
  style_slider_slim(stage->stage_gain_slider_l);
  lv_obj_set_style_pad_all(stage->stage_gain_slider_l, 4, LV_PART_KNOB);
  // Non-interactive — clicks handled by the transparent overlay below
  lv_obj_clear_flag(stage->stage_gain_slider_l, LV_OBJ_FLAG_CLICKABLE);

  // R slider — display only
  stage->stage_gain_slider_r = lv_slider_create(parent);
  lv_obj_set_size(stage->stage_gain_slider_r, STAGE_GAIN_SL_W, STAGE_GAIN_SL_H);
  lv_obj_set_pos(stage->stage_gain_slider_r, STAGE_GAIN_SL_X_R, STAGE_GAIN_SL_Y);
  lv_slider_set_range(stage->stage_gain_slider_r, STAGE_GAIN_SLIDER_MIN, STAGE_GAIN_SLIDER_MAX);
  lv_slider_set_value(stage->stage_gain_slider_r, STAGE_GAIN_DEFAULT * 10, LV_ANIM_OFF);
  style_slider_slim(stage->stage_gain_slider_r);
  lv_obj_set_style_pad_all(stage->stage_gain_slider_r, 4, LV_PART_KNOB);
  lv_obj_clear_flag(stage->stage_gain_slider_r, LV_OBJ_FLAG_CLICKABLE);

  // Initialize gain state
  stage->stage_gain_l_db = 0.0f;
  stage->stage_gain_r_db = 0.0f;

  // 0dB reference line between sliders
  int zero_db_offset = (STAGE_GAIN_SL_H * (-STAGE_GAIN_SLIDER_MIN)) /
                       (STAGE_GAIN_SLIDER_MAX - STAGE_GAIN_SLIDER_MIN);
  int zero_db_y = STAGE_GAIN_SL_Y + STAGE_GAIN_SL_H - zero_db_offset;
  {
    lv_obj_t *zero_line = lv_obj_create(parent);
    lv_obj_remove_style_all(zero_line);
    lv_obj_set_size(zero_line, 14, 2);
    lv_obj_set_pos(zero_line, STAGE_GAIN_SL_X_L + STAGE_GAIN_SL_W + 2, zero_db_y);
    lv_obj_set_style_bg_color(zero_line, COL_GRID_ZERO, 0);
    lv_obj_set_style_bg_opa(zero_line, LV_OPA_COVER, 0);
    lv_obj_clear_flag(zero_line, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
  }

  // Gain readout labels — small dB value below each slider
  // Positioned just below the slider bottom edge
  #define STAGE_GAIN_READOUT_Y (STAGE_GAIN_SL_Y + STAGE_GAIN_SL_H + 2)
  {
    char gbuf[8];
    stage->stage_gain_readout_l = lv_label_create(parent);
    format_gain_label(gbuf, sizeof(gbuf), 0.0f);
    lv_label_set_text(stage->stage_gain_readout_l, gbuf);
    lv_label_set_long_mode(stage->stage_gain_readout_l, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(stage->stage_gain_readout_l, 22, 12);
    lv_obj_set_pos(stage->stage_gain_readout_l, STAGE_GAIN_SL_X_L - 7, STAGE_GAIN_READOUT_Y);
    lv_obj_set_style_text_font(stage->stage_gain_readout_l, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(stage->stage_gain_readout_l, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_align(stage->stage_gain_readout_l, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(stage->stage_gain_readout_l, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    stage->stage_gain_readout_r = lv_label_create(parent);
    lv_label_set_text(stage->stage_gain_readout_r, gbuf);
    lv_label_set_long_mode(stage->stage_gain_readout_r, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(stage->stage_gain_readout_r, 22, 12);
    lv_obj_set_pos(stage->stage_gain_readout_r, STAGE_GAIN_SL_X_R - 7, STAGE_GAIN_READOUT_Y);
    lv_obj_set_style_text_font(stage->stage_gain_readout_r, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(stage->stage_gain_readout_r, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_align(stage->stage_gain_readout_r, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_clear_flag(stage->stage_gain_readout_r, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
  }

  // Transparent touch overlay covering both small sliders — opens the popup on tap
  {
    lv_obj_t *gain_touch = lv_obj_create(parent);
    lv_obj_remove_style_all(gain_touch);
    // Cover both sliders + labels: from LBL_Y to READOUT_Y bottom, L to R+width
    lv_obj_set_pos(gain_touch, STAGE_GAIN_SL_X_L - 5, STAGE_GAIN_LBL_Y);
    lv_obj_set_size(gain_touch,
                    (STAGE_GAIN_SL_X_R + STAGE_GAIN_SL_W + 4) - (STAGE_GAIN_SL_X_L - 5),
                    STAGE_GAIN_SL_H + 20);
    lv_obj_set_style_bg_opa(gain_touch, LV_OPA_TRANSP, 0);
    lv_obj_add_flag(gain_touch, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(gain_touch, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(gain_touch, [](lv_event_t *e) {
      lv_event_code_t code = lv_event_get_code(e);
      if (code != LV_EVENT_PRESSED && code != LV_EVENT_PRESSING && code != LV_EVENT_RELEASED) return;

      eq_stage_t *s = (eq_stage_t *)lv_event_get_user_data(e);
      lv_indev_t * indev = lv_indev_active();
      if (!indev) return;

      lv_point_t p;
      lv_indev_get_point(indev, &p);

      bool is_left = (p.x < 454);
      lv_obj_t *target_slider = NULL;
      if (s->channel_mode == CH_MODE_LINKED) {
          target_slider = s->gain_popup_slider_l;
      } else {
          target_slider = is_left ? s->gain_popup_slider_l : s->gain_popup_slider_r;
      }

      if (code == LV_EVENT_PRESSED) {
        gain_popup_open(s);
        if (target_slider) lv_obj_send_event(target_slider, LV_EVENT_PRESSED, NULL);
      } else if (code == LV_EVENT_PRESSING && target_slider) {
        int y_rel = p.y - 16;
        if (y_rel < 0) y_rel = 0;
        if (y_rel > 238) y_rel = 238;

        float ratio = (float)y_rel / 238.0f;
        int val_range = STAGE_GAIN_SLIDER_MAX - STAGE_GAIN_SLIDER_MIN;
        int new_val = STAGE_GAIN_SLIDER_MAX - (int)(ratio * val_range);

        lv_slider_set_value(target_slider, new_val, LV_ANIM_OFF);
        lv_obj_send_event(target_slider, LV_EVENT_VALUE_CHANGED, NULL);
      } else if (code == LV_EVENT_RELEASED && target_slider) {
        lv_obj_send_event(target_slider, LV_EVENT_RELEASED, NULL);
      }
    }, LV_EVENT_ALL, stage);
  }

  // --- Band parameter sliders removed (ring redesign) ---
  // FREQ/GAIN/Q are now shown and edited via the rings + static slider. Null
  // the old field pointers so sync_band_display's null-guarded paths stay safe.
  stage->freq_slider_obj = NULL;
  stage->q_slider_obj = NULL;
  stage->band_gain_slider_obj = NULL;
  stage->freq_val_label = NULL;
  stage->q_val_label = NULL;
  stage->band_gain_val_label = NULL;

  sync_band_display(stage);

  // --- FINE circle (custom-drawn ring, flashing orange inner when active) ---
  // Sits in the gap between the Q ring and the button column. Area is padded
  // (2*(FINE_R+FINE_MARGIN)) so the stroke+AA isn't clipped; draw cb centres on it.
  stage->fine_circle = lv_obj_create(parent);
  lv_obj_remove_style_all(stage->fine_circle);
  lv_obj_set_size(stage->fine_circle, 2 * (FINE_R + FINE_MARGIN), 2 * (FINE_R + FINE_MARGIN));
  lv_obj_set_pos(stage->fine_circle, FINE_CX - FINE_R - FINE_MARGIN, FINE_CY - FINE_R - FINE_MARGIN);
  lv_obj_set_style_bg_opa(stage->fine_circle, LV_OPA_TRANSP, 0);
  lv_obj_add_flag(stage->fine_circle, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(stage->fine_circle, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(stage->fine_circle, fine_btn_cb, LV_EVENT_CLICKED, stage);
  lv_obj_add_event_cb(stage->fine_circle, fine_circle_draw_cb, LV_EVENT_DRAW_POST, stage);

  // "FINE ADJ" label on the ring param-label row (same Y/font/colour as
  // FREQ/GAIN/Q: RING_AREA_Y+RING_CY+RING_R+2).
  stage->fine_label = lv_label_create(parent);
  lv_label_set_text(stage->fine_label, "FINE ADJ");
  lv_obj_set_style_text_font(stage->fine_label, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(stage->fine_label, COL_TEXT_DIM, 0);
  lv_obj_set_style_text_align(stage->fine_label, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_pos(stage->fine_label, FINE_CX - 24, RING_AREA_Y + RING_CY + RING_R + 2);
  lv_obj_set_width(stage->fine_label, 48);

  // --- MUTE button — 20×12, centred below the stage gain sliders ---
  #define MUTE_BTN_W 34
  #define MUTE_BTN_H 17
  #define MUTE_BTN_X (SCREEN_W - 40)
  #define MUTE_BTN_Y NAV_Y0   // mute occupies the top nav slot on EQ pages
  stage->mute_btn = lv_obj_create(parent);
  stage->mute_flash_state = false;
  lv_obj_remove_style_all(stage->mute_btn);
  lv_obj_set_size(stage->mute_btn, MUTE_BTN_W, MUTE_BTN_H);
  lv_obj_set_pos(stage->mute_btn, MUTE_BTN_X, MUTE_BTN_Y);
  lv_obj_set_style_bg_color(stage->mute_btn, COL_BOX_LIST, 0);
  lv_obj_set_style_bg_opa(stage->mute_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(stage->mute_btn, 2, 0);
  lv_obj_set_style_pad_all(stage->mute_btn, 0, 0);
  lv_obj_add_flag(stage->mute_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(stage->mute_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(stage->mute_btn, mute_btn_cb,  LV_EVENT_CLICKED,   stage);
  lv_obj_add_event_cb(stage->mute_btn, mute_draw_cb, LV_EVENT_DRAW_POST, stage);

  // Nav buttons: CFG (middle) jumps to config, VIS (bottom) to spectrum.
  create_nav_button(parent, NAV_Y1, "CFG", nav_cfg_cb);
  create_nav_button(parent, NAV_Y2, "VIS", nav_vis_cb);

  // --- Old bottom row removed (shelf LSH/HSH, band-ID buttons) ---
  // Band selection remains available via curve tap + BAND-readout arrows.
  // Null the pointers so the null-guarded style/update paths stay safe.
  stage->lsh_btn = NULL;
  stage->hsh_btn = NULL;

  // --- Right-hand solid button column: DEL / ADD / BELL ---
  // Right edge pinned to the graph right border. Solid fill, black text, no
  // state recolour (handlers self-guard: ADD no-ops at MAX_BANDS, DEL at 1).
  struct { lv_obj_t **btn; lv_obj_t **lbl; int y; lv_color_t col;
           const char *txt; lv_event_cb_t cb; } rbtns[] = {
    { &stage->del_band_btn, NULL,                   RBTN_DEL_Y,   RBTN_COL_DEL,   "DEL BAND", remove_band_cb },
    { &stage->add_band_btn, NULL,                   RBTN_ADD_Y,   RBTN_COL_ADD,   "ADD BAND", add_band_cb    },
    { &stage->shape_btn,    &stage->shape_btn_label, RBTN_SHAPE_Y, RBTN_COL_SHAPE, "BELL",    shape_btn_cb   },
  };
  for (unsigned i = 0; i < sizeof(rbtns) / sizeof(rbtns[0]); i++) {
    lv_obj_t *b = lv_obj_create(parent);
    lv_obj_remove_style_all(b);
    lv_obj_set_size(b, RBTN_W, RBTN_H);
    lv_obj_set_pos(b, RBTN_X, rbtns[i].y);
    lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(b, rbtns[i].col, 0);
    lv_obj_set_style_border_width(b, 1, 0);
    lv_obj_set_style_border_opa(b, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(b, 3, 0);
    lv_obj_set_style_pad_all(b, 0, 0);
    lv_obj_add_flag(b, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(b, rbtns[i].cb, LV_EVENT_CLICKED, stage);

    lv_obj_t *lbl = lv_label_create(b);
    lv_label_set_text(lbl, rbtns[i].txt);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(lbl, rbtns[i].col, 0);
    lv_obj_center(lbl);

    *rbtns[i].btn = b;
    if (rbtns[i].lbl) *rbtns[i].lbl = lbl;
  }
  update_shape_button_label(stage);  // set BELL/LO SHLF/HI SHLF to match band
  if (stage->shape_btn)
    lv_obj_add_event_cb(stage->shape_btn, shape_btn_draw_cb, LV_EVENT_DRAW_MAIN_END, stage);
  for (int i = 0; i < MAX_BANDS; i++) {
    stage->band_btns[i] = NULL;
    stage->band_btn_labels[i] = NULL;
  }

  // --- Channel selection buttons (L / R / Lk) in bottom-left of graph ---
  stage->ch_l_btn = create_ch_button(parent, stage, 0, "L");
  stage->ch_l_label = lv_obj_get_child(stage->ch_l_btn, 0);

  stage->ch_r_btn = create_ch_button(parent, stage, 1, "R");
  stage->ch_r_label = lv_obj_get_child(stage->ch_r_btn, 0);

  stage->ch_lk_btn = create_ch_button(parent, stage, 2, "LR");
  stage->ch_lk_label = lv_obj_get_child(stage->ch_lk_btn, 0);

  update_channel_button_style(stage);

  // Create gain popup (hidden by default)
  create_gain_popup(parent, stage);
}

// ============================================================
// Config Mode
// ============================================================

static void config_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int type = (intptr_t)lv_event_get_user_data(e);
  int val = lv_slider_get_value(slider);

  if (type == 1) { // Backlight
    sys_config.backlight_val = val;
    char buf[16];
    snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(sys_config.backlight_val_label, buf);
    hardware_set_backlight(val);
  }
  preset_mark_dirty();
  update_all_preset_boxes(); // This removes the red box immediately
}

static void config_switch_event_cb(lv_event_t *e) {
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  sys_config.limiter_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
  dsp_update_limiter(sys_config.limiter_enabled, sys_config.limiter_threshold);
  preset_mark_dirty();
  update_all_preset_boxes();
}

// Phase 8: sub limiter switch — separate from main limiter so the handler
// knows which DSP API to call. (config_switch_event_cb above always hits
// dsp_update_limiter for the main path.)
static void config_low_limiter_switch_cb(lv_event_t *e) {
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  sys_config.low_limiter_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
  low_update_limiter(sys_config.low_limiter_enabled, sys_config.low_limiter_threshold);
  preset_mark_dirty();
  update_all_preset_boxes();
}

// Phase 13: Test signal mode dropdown callback
// ============================================================
// Adjustable-sine tone panel
// ============================================================
// Screen-level translucent overlay shown on EQ/xover while the ADJ SINE test
// source is active. Carries freq display + wide freq slider, level slider,
// COARSE/FINE toggle, and an X to dismiss. Swipe is disabled while active.

static void tone_freq_update_label(void) {
  if (!sys_config.tone_freq_val_label) return;
  char buf[16];
  // Unit merged into the string, same font: integer Hz below 1k, else kHz 2dp.
  if (sys_config.tone_freq_hz < 1000.0f)
    snprintf(buf, sizeof(buf), "%dHz", (int)(sys_config.tone_freq_hz + 0.5f));
  else
    snprintf(buf, sizeof(buf), "%.2fkHz", sys_config.tone_freq_hz / 1000.0f);
  lv_label_set_text(sys_config.tone_freq_val_label, buf);
}

static void tone_freq_slider_cb(lv_event_t *e) {
  lv_obj_t *s = (lv_obj_t *)lv_event_get_target(e);
  int val = lv_slider_get_value(s);
  sys_config.tone_freq_hz = tone_slider_to_freq(val);
  dsp_set_tone_freq(sys_config.tone_freq_hz);
  tone_freq_update_label();
}

static void tone_level_slider_cb(lv_event_t *e) {
  lv_obj_t *s = (lv_obj_t *)lv_event_get_target(e);
  int val = lv_slider_get_value(s);
  sys_config.tone_level_db = (float)val / 10.0f;   // -600..0 -> -60..0 dB
  dsp_set_tone_level(sys_config.tone_level_db);
  if (sys_config.tone_level_val_label) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1fdB", sys_config.tone_level_db);
    lv_label_set_text(sys_config.tone_level_val_label, buf);
  }
}

// COARSE/FINE toggle: fine snapshots current freq into a +/- octave window
// (freq/2 .. freq*2), mirroring the EQ fine behaviour.
static void tone_cf_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  sys_config.tone_fine_mode = !sys_config.tone_fine_mode;
  if (sys_config.tone_fine_mode) {
    sys_config.tone_fine_lo = sys_config.tone_freq_hz * 0.5f;
    sys_config.tone_fine_hi = sys_config.tone_freq_hz * 2.0f;
  }
  if (sys_config.tone_cf_btn_label)
    lv_label_set_text(sys_config.tone_cf_btn_label,
                      sys_config.tone_fine_mode ? "FINE" : "COARSE");
  // Re-seat the slider knob for the new mapping at the current freq.
  if (sys_config.tone_freq_slider)
    lv_slider_set_value(sys_config.tone_freq_slider,
                        tone_freq_to_slider(sys_config.tone_freq_hz), LV_ANIM_OFF);
}

static void tone_panel_close(void) {
  sys_config.tone_active = false;
  if (sys_config.tone_panel)
    lv_obj_add_flag(sys_config.tone_panel, LV_OBJ_FLAG_HIDDEN);
  dsp_set_input_source(DSP_INPUT_I2S);   // restore normal input
  if (main_tileview)
    lv_obj_add_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);  // restore swipe
  // Reset the dropdown selection to OFF to reflect tone-off.
  if (sys_config.test_signal_dropdown) {
    lv_dropdown_set_selected(sys_config.test_signal_dropdown, 0);
    sys_config.test_signal_mode = 0;
  }
}

static void tone_x_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  tone_panel_close();   // stays on the current EQ/xover tile
}

static void tone_panel_open(void) {
  if (!sys_config.tone_panel) return;
  sys_config.tone_active = true;

  // Defaults on open: 1 kHz, -40 dB, coarse.
  sys_config.tone_freq_hz = 1000.0f;
  sys_config.tone_level_db = -40.0f;
  sys_config.tone_fine_mode = false;
  dsp_set_tone_freq(sys_config.tone_freq_hz);
  dsp_set_tone_level(sys_config.tone_level_db);

  if (sys_config.tone_cf_btn_label)
    lv_label_set_text(sys_config.tone_cf_btn_label, "COARSE");
  if (sys_config.tone_freq_slider)
    lv_slider_set_value(sys_config.tone_freq_slider,
                        tone_freq_to_slider(sys_config.tone_freq_hz), LV_ANIM_OFF);
  if (sys_config.tone_level_slider)
    lv_slider_set_value(sys_config.tone_level_slider,
                        (int)(sys_config.tone_level_db * 10), LV_ANIM_OFF);
  if (sys_config.tone_level_val_label) {
    char lbuf[16];
    snprintf(lbuf, sizeof(lbuf), "%.1fdB", sys_config.tone_level_db);
    lv_label_set_text(sys_config.tone_level_val_label, lbuf);
  }
  tone_freq_update_label();

  lv_obj_move_foreground(sys_config.tone_panel);
  lv_obj_clear_flag(sys_config.tone_panel, LV_OBJ_FLAG_HIDDEN);
  if (main_tileview)
    lv_obj_clear_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE);  // disable swipe
}

// Build the tone panel once, on the top layer so it floats above any tile.
// Positions/sizes are first-pass (flagged for tuning). Full width, ~graph
// height, translucent so the EQ page shows faintly through.
static void create_tone_panel(void) {
  lv_obj_t *scr = lv_layer_top();   // top layer floats above the tileview
  lv_obj_t *p = lv_obj_create(scr);
  lv_obj_remove_style_all(p);
  lv_obj_set_size(p, SCREEN_W, CURVE_H);            // full width, graph height
  lv_obj_set_pos(p, 0, CURVE_Y);                    // aligned to graph top
  lv_obj_set_style_bg_color(p, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(p, 198, 0);               // translucent (flag: tune)
  lv_obj_set_style_radius(p, 0, 0);
  lv_obj_set_style_pad_all(p, 0, 0);
  lv_obj_add_flag(p, LV_OBJ_FLAG_CLICKABLE);        // absorb touches to the page beneath
  lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
  sys_config.tone_panel = p;

  // Freq + unit, single montserrat_24 label (e.g. "2.82kHz").
  lv_obj_t *fv = lv_label_create(p);
  lv_obj_set_style_text_font(fv, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(fv, COL_WHITE, 0);
  lv_label_set_text(fv, "1000Hz");
  lv_obj_set_pos(fv, 70, 34);                       // flag: tune
  sys_config.tone_freq_val_label = fv;

  // Wide freq slider along the bottom of the panel.
  lv_obj_t *fs = lv_slider_create(p);
  lv_obj_set_size(fs, SCREEN_W - 40, 8);
  lv_obj_set_pos(fs, 20, CURVE_H - 30);             // flag: tune
  lv_slider_set_range(fs, 0, FREQ_SLIDER_MAX);
  style_slider_slim(fs);
  lv_obj_add_event_cb(fs, tone_freq_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
  sys_config.tone_freq_slider = fs;

  // Level slider (short, width = RBTN_W) to the right of the freq display.
  lv_obj_t *ls = lv_slider_create(p);
  lv_obj_set_size(ls, RBTN_W, 6);
  lv_obj_set_pos(ls, SCREEN_W - RBTN_W - 170, 50);  // flag: tune
  lv_slider_set_range(ls, -600, 0);
  style_slider_slim(ls);
  lv_obj_add_event_cb(ls, tone_level_slider_cb, LV_EVENT_VALUE_CHANGED, NULL);
  sys_config.tone_level_slider = ls;

  // "SIG LVL" label above the level slider (old coarse/fine btn spot).
  lv_obj_t *sll = lv_label_create(p);
  lv_label_set_text(sll, "TONE LEVEL");
  lv_obj_set_style_text_font(sll, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(sll, COL_WHITE, 0);
  lv_obj_set_pos(sll, SCREEN_W - RBTN_W - 170, 21);  // flag: tune

  // dB readout, 5px right of the slider's right edge.
  lv_obj_t *lvl = lv_label_create(p);
  lv_obj_set_style_text_font(lvl, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(lvl, COL_TEXT, 0);
  lv_label_set_text(lvl, "-40.0dB");
  lv_obj_set_pos(lvl, SCREEN_W - RBTN_W - 170 + RBTN_W + 5, 46);  // flag: tune
  sys_config.tone_level_val_label = lvl;

  // COARSE/FINE toggle (RBTN size) above the level slider.
  lv_obj_t *cf = lv_obj_create(p);
  lv_obj_remove_style_all(cf);
  lv_obj_set_size(cf, RBTN_W - 10, RBTN_H);
  lv_obj_set_pos(cf, SCREEN_W - (RBTN_W - 10) - 30, 41);  // flag: tune (right-anchored, clear of dB readout)
  lv_obj_set_style_bg_color(cf, COL_BOX_LIST, 0);
  lv_obj_set_style_bg_opa(cf, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(cf, 2, 0);
  lv_obj_add_flag(cf, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(cf, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(cf, tone_cf_btn_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *cfl = lv_label_create(cf);
  lv_label_set_text(cfl, "COARSE");
  lv_obj_set_style_text_font(cfl, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(cfl, lv_color_hex(0x1A1A1A), 0);
  lv_obj_center(cfl);
  sys_config.tone_cf_btn_label = cfl;

  // Large X, top-right corner.
  lv_obj_t *x = lv_obj_create(p);
  lv_obj_remove_style_all(x);
  lv_obj_set_size(x, 28, 28);
  lv_obj_set_pos(x, SCREEN_W - 34, 4);              // flag: tune
  lv_obj_add_flag(x, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(x, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(x, tone_x_btn_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *xl = lv_label_create(x);
  lv_label_set_text(xl, "X");
  lv_obj_set_style_text_font(xl, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(xl, COL_WHITE, 0);
  lv_obj_center(xl);
}

static void test_signal_dropdown_event_cb(lv_event_t *e) {
  lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
  uint16_t sel = lv_dropdown_get_selected(dd);

  sys_config.test_signal_mode = sel;

  // Map dropdown index to DSP input source enum
  static const dsp_input_source_t mode_map[] = {
      DSP_INPUT_I2S,              // 0: OFF
      DSP_INPUT_TONE,             // 1: ADJ SINE
      DSP_INPUT_NOISE,            // 2: Pink Noise
      DSP_INPUT_SWEEP_30_20K_30S, // 3: Sweep 30-20k (30s)
      DSP_INPUT_SWEEP_20_20K_35S, // 4: Sweep 20-20k (35s)
      DSP_INPUT_WARBLE_30_20K_30S // 5: Warble 30-20k (30s)
  };

  dsp_set_input_source(mode_map[sel]);

  if (sel == 1) {
    // ADJ SINE: jump to the last EQ/xover tile and open the tone panel there.
    int t = nav_return_tile;                 // last EQ/xover origin (guarded 1..4)
    if (t < 1 || t > 4) t = 1;
    if (main_tileview) lv_obj_set_tile_id(main_tileview, t, 0, LV_ANIM_OFF);
    tone_panel_open();
    return;                                  // tone has independent level
  }

  // Update DSP level (shared across all test signals)
  dsp_update_noise_gen(true, sys_config.test_signal_db);
}

// Phase 13: Test signal level slider callback (shared for all test modes)
static void test_signal_slider_event_cb(lv_event_t *e) {
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int val = lv_slider_get_value(slider);
  sys_config.test_signal_db = (float)val / 10.0f;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.1fdB", sys_config.test_signal_db);
  if (sys_config.test_signal_val_label) {
    lv_label_set_text(sys_config.test_signal_val_label, buf);
  }
  dsp_update_noise_gen(true, sys_config.test_signal_db);
}

static void visualizer_btn_event_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
  lv_obj_t *lbl = (lv_obj_t *)lv_event_get_user_data(e);
  bool is_on = lv_obj_has_state(btn, LV_STATE_CHECKED);

  sys_config.fft_enabled = is_on;
  dsp_set_fft_enabled(is_on);

  // Restore last non-zero mode when turning on; force mode 0 when off.
  int mode = is_on ? (sys_config.visualizer_mode > 0 ? sys_config.visualizer_mode : 1) : 0;
  sys_config.visualizer_mode = mode;
  spectrum_set_mode(mode);

  lv_label_set_text(lbl, is_on ? "ON" : "OFF");
  lv_obj_set_style_bg_color(btn, is_on ? lv_color_hex(0x007700) : lv_color_hex(0x333333), 0);

  // Show or hide the spectrum tile in the tileview
  if (tile_spectrum_ref) {
    if (is_on) {
      lv_obj_clear_flag(tile_spectrum_ref, LV_OBJ_FLAG_HIDDEN);
      spectrum_timer_set_paused(false);
    } else {
      lv_obj_add_flag(tile_spectrum_ref, LV_OBJ_FLAG_HIDDEN);
      spectrum_timer_set_paused(true);
    }
  }

  preset_mark_dirty();
}

// Input source dropdown callback
static void input_source_event_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
  int sel = lv_dropdown_get_selected(dd);
  sys_config.input_source_idx = sel;
  dsp_set_input_source(DSP_INPUT_I2S);
  preset_mark_dirty();
}

// Storage backend switch (NVS / SD). Off = NVS, On = SD. Persists the new
// selection to cfg_boot/storage and reboots — the active backend can only
// be safely swapped at boot (no in-flight write hazards, no stale handles).
//
// The reboot lands on whatever the new backend holds; if that is empty or
// absent, the device boots to defaults (all stages muted — see dsp_engine.cpp
// dsp_stage_muted). Because a stray tap on this switch therefore blows away
// the running state and reboots, it is gated behind a confirm dialog.
static lv_obj_t *storage_confirm_panel = NULL;   // top-layer modal, built lazily
static uint8_t   storage_confirm_target = 0;     // BOOT_STORAGE_* awaiting confirm

// Put the switch back where the active backend says it should be. Called on
// cancel — LVGL has already flipped the widget visually by the time the
// VALUE_CHANGED callback runs.
static void storage_switch_revert(void) {
  if (!sys_config.storage_switch)
    return;
  if (boot_config_get_storage() == BOOT_STORAGE_SD)
    lv_obj_add_state(sys_config.storage_switch, LV_STATE_CHECKED);
  else
    lv_obj_clear_state(sys_config.storage_switch, LV_STATE_CHECKED);
}

static void storage_confirm_hide(void) {
  if (storage_confirm_panel)
    lv_obj_add_flag(storage_confirm_panel, LV_OBJ_FLAG_HIDDEN);
  if (main_tileview)
    lv_obj_add_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE); // restore swipe
}

static void storage_confirm_cancel_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  storage_confirm_hide();
  storage_switch_revert();
}

static void storage_confirm_ok_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  storage_confirm_hide();
  if (!boot_config_set_storage(storage_confirm_target)) {
    Serial.println("[BOOT] Failed to write storage selector");
    storage_switch_revert();
    return;
  }
  Serial.printf("[BOOT] Storage selector = %s. Rebooting...\n",
                storage_confirm_target == BOOT_STORAGE_SD ? "SD" : "NVS");
  delay(300);
  ESP.restart();
}

// Build once on the top layer (same pattern as the tone panel) so it floats
// above the tileview and absorbs touches to the page beneath.
static void storage_confirm_create(void) {
  lv_obj_t *scr = lv_layer_top();
  lv_obj_t *p = lv_obj_create(scr);
  lv_obj_remove_style_all(p);
  lv_obj_set_size(p, SCREEN_W, SCREEN_H);
  lv_obj_set_pos(p, 0, 0);
  lv_obj_set_style_bg_color(p, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(p, 210, 0);
  lv_obj_set_style_radius(p, 0, 0);
  lv_obj_set_style_pad_all(p, 0, 0);
  lv_obj_add_flag(p, LV_OBJ_FLAG_CLICKABLE);   // absorb stray taps
  lv_obj_clear_flag(p, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
  storage_confirm_panel = p;

  lv_obj_t *l1 = lv_label_create(p);
  lv_obj_set_style_text_font(l1, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(l1, COL_WHITE, 0);
  lv_label_set_text(l1, "Switch storage backend?");
  lv_obj_set_pos(l1, 120, 78);

  lv_obj_t *l2 = lv_label_create(p);
  lv_obj_set_style_text_font(l2, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(l2, COL_TEXT, 0);
  lv_label_set_text(l2, "Device will reboot and load from the new backend.");
  lv_obj_set_pos(l2, 68, 106);

  lv_obj_t *cancel = lv_button_create(p);
  lv_obj_set_size(cancel, 96, 34);
  lv_obj_set_pos(cancel, 108, 152);
  lv_obj_set_style_bg_color(cancel, COL_BOX, 0);
  lv_obj_set_style_border_color(cancel, COL_BOX_BORDER, 0);
  lv_obj_set_style_border_width(cancel, 1, 0);
  lv_obj_add_event_cb(cancel, storage_confirm_cancel_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *cl = lv_label_create(cancel);
  lv_obj_set_style_text_font(cl, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(cl, COL_TEXT, 0);
  lv_label_set_text(cl, "CANCEL");
  lv_obj_center(cl);

  lv_obj_t *ok = lv_button_create(p);
  lv_obj_set_size(ok, 96, 34);
  lv_obj_set_pos(ok, 268, 152);
  lv_obj_set_style_bg_color(ok, COL_BOX_ACT, 0);
  lv_obj_set_style_border_color(ok, COL_BOX_BDR_ACT, 0);
  lv_obj_set_style_border_width(ok, 1, 0);
  lv_obj_add_event_cb(ok, storage_confirm_ok_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *okl = lv_label_create(ok);
  lv_obj_set_style_text_font(okl, &lv_font_montserrat_12, 0);
  lv_obj_set_style_text_color(okl, COL_WHITE, 0);
  lv_label_set_text(okl, "SWITCH");
  lv_obj_center(okl);
}

static void storage_switch_event_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
  uint8_t target = lv_obj_has_state(sw, LV_STATE_CHECKED) ? BOOT_STORAGE_SD
                                                          : BOOT_STORAGE_NVS;
  if (target == boot_config_get_storage())
    return; // no-op

  storage_confirm_target = target;
  if (!storage_confirm_panel)
    storage_confirm_create();
  lv_obj_clear_flag(storage_confirm_panel, LV_OBJ_FLAG_HIDDEN);
  lv_obj_move_foreground(storage_confirm_panel);
  if (main_tileview)
    lv_obj_clear_flag(main_tileview, LV_OBJ_FLAG_SCROLLABLE); // no swipe while modal
}

static void create_config_row_slider(lv_obj_t *parent, int y,
                                     const char *label_text, int min, int max,
                                     int val, int type, bool half_width,
                                     lv_obj_t **out_label) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, label_text);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(lbl, SL_LBL_X, y - 3);
  lv_obj_set_size(lbl, 60, 12);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_RIGHT, 0);

  int slider_width = half_width ? (SL_BAR_W - 30) / 2 : (SL_BAR_W - 30);

  lv_obj_t *slider = lv_slider_create(parent);
  lv_obj_set_size(slider, slider_width, 6);
  lv_obj_set_pos(slider, SL_BAR_X + 30, y);
  lv_slider_set_range(slider, min, max);
  lv_slider_set_value(slider, val, LV_ANIM_OFF);
  style_slider_slim(slider);
  lv_obj_add_event_cb(slider, config_slider_event_cb, LV_EVENT_VALUE_CHANGED,
                      (void *)(intptr_t)type);

  lv_obj_t *vlbl = lv_label_create(parent);
  lv_label_set_text(vlbl, "");
  lv_obj_set_style_text_color(vlbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(vlbl, &lv_font_montserrat_10, 0);

  if (half_width) {
    lv_obj_set_pos(vlbl, SL_BAR_X + 30 + slider_width + 10, y - 3);
  } else {
    lv_obj_set_pos(vlbl, SL_VAL_X, y - 3);
  }
  *out_label = vlbl;
}

static lv_obj_t *create_config_row_dropdown(lv_obj_t *parent, int y,
                                            const char *label_text,
                                            const char *options, int *val_ref) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, label_text);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(lbl, SL_LBL_X, y + 4);
  lv_obj_set_size(lbl, 60, 12);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_RIGHT, 0);

  lv_obj_t *dd = lv_dropdown_create(parent);
  lv_dropdown_set_options(dd, options);
  lv_obj_set_pos(dd, SL_BAR_X + 30, y - 4);
  lv_obj_set_size(dd, 120, 24);

  lv_obj_set_style_bg_color(dd, COL_BOX, 0);
  lv_obj_set_style_border_color(dd, COL_BOX_BORDER, 0);
  lv_obj_set_style_text_color(dd, COL_TEXT, 0);
  lv_obj_set_style_text_font(dd, &lv_font_montserrat_10, 0);

  return dd;
}

static void create_config_row_switch(lv_obj_t *parent, int y,
                                     const char *label_text) {
  lv_obj_t *lbl = lv_label_create(parent);
  lv_label_set_text(lbl, label_text);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(lbl, SL_LBL_X, y + 4);
  lv_obj_set_size(lbl, 60, 12);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_RIGHT, 0);

  lv_obj_t *sw = lv_switch_create(parent);
  lv_obj_set_pos(sw, SL_BAR_X + 30, y - 2);
  lv_obj_set_size(sw, 40, 20);
  lv_obj_set_style_bg_color(sw, COL_BOX, LV_PART_MAIN);
  // Explicit 0 for indicator state as LV_STATE_CHECKED modifier handles the
  // rest typically, but this is fine in slim styling
  lv_obj_add_event_cb(sw, config_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
}

// ============================================================
// Preset UI Functions
// ============================================================

static void update_preset_button_style(lv_obj_t *btn, lv_obj_t *lbl,
                                       bool armed) {
  if (armed) {
    // Armed: white fill, black text
    lv_obj_set_style_bg_color(btn, COL_WHITE, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(btn, COL_WHITE, 0);
    lv_obj_set_style_text_color(lbl, COL_BLACK, 0);
  } else {
    // Normal: dark fill, white border, white text
    lv_obj_set_style_bg_color(btn, COL_BTN_FACE, 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(btn, COL_WHITE, 0);
    lv_obj_set_style_text_color(lbl, COL_WHITE, 0);
  }
}

static void update_preset_box_style(int slot) {
  if (slot < 0 || slot >= 4)
    return;
  if (!sys_config.preset_boxes[slot])
    return;

  int active_slot = preset_get_active_slot();
  bool is_active = (slot == active_slot);

  lv_obj_t *box = sys_config.preset_boxes[slot];
  lv_obj_t *lbl = sys_config.preset_labels[slot];

  if (is_active) {
    // Active: red fill, black text
    lv_obj_set_style_bg_color(box, COL_PRESET_RED, 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(lbl, COL_BLACK, 0);
  } else {
    // Inactive: no fill, red outline, red text
    lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(box, COL_PRESET_RED, 0);
    lv_obj_set_style_border_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_text_color(lbl, COL_PRESET_RED, 0);
  }
}

static void update_all_preset_boxes(void) {
  for (int i = 0; i < 4; i++) {
    update_preset_box_style(i);
  }
}

static void preset_disarm(void) {
  sys_config.preset_mode = PRESET_MODE_IDLE;
  if (sys_config.load_btn && sys_config.load_btn_label) {
    update_preset_button_style(sys_config.load_btn, sys_config.load_btn_label,
                               false);
  }
  if (sys_config.save_btn && sys_config.save_btn_label) {
    update_preset_button_style(sys_config.save_btn, sys_config.save_btn_label,
                               false);
  }
}

static void preset_box_clicked_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;
  int slot = (intptr_t)lv_event_get_user_data(e);
  if (slot < 0 || slot >= 4)
    return;

  if (sys_config.preset_mode == PRESET_MODE_LOAD_ARMED) {
    if (preset_load_slot(slot)) {
      preset_disarm();
      update_all_preset_boxes();
      eq_ui_update_preset_display(); // CRITICAL: Refreshes sliders/switches to
                                     // match preset
    }
  } else if (sys_config.preset_mode == PRESET_MODE_SAVE_ARMED) {
    if (preset_save_slot(slot)) {
      preset_disarm();
      update_all_preset_boxes();
      sys_config.flash_slot = slot;
      sys_config.flash_count = PRESET_FLASH_COUNT;
      sys_config.flash_time = lv_tick_get();
      sys_config.flash_state = false;
    }
  }
}

static void load_btn_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED)
    return;

  if (sys_config.preset_mode == PRESET_MODE_LOAD_ARMED) {
    // Already armed - disarm
    preset_disarm();
  } else {
    // Arm load mode
    preset_disarm(); // Clear any other armed state
    sys_config.preset_mode = PRESET_MODE_LOAD_ARMED;
    update_preset_button_style(sys_config.load_btn, sys_config.load_btn_label,
                               true);
  }
}

static uint32_t save_press_start = 0;
static bool save_btn_is_pressed = false;

static void save_btn_pressed_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_PRESSED)
    return;
  save_press_start = lv_tick_get();
  save_btn_is_pressed = true; // Mark as pressed
}

static void save_btn_released_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_RELEASED)
    return;

  save_btn_is_pressed = false; // Mark as released
  uint32_t held = lv_tick_get() - save_press_start;

  if (held >= PRESET_SAVE_ARM_HOLD_MS) {
    // Long press - arm save mode
    preset_disarm();
    sys_config.preset_mode = PRESET_MODE_SAVE_ARMED;
    sys_config.save_arm_time = lv_tick_get();
    update_preset_button_style(sys_config.save_btn, sys_config.save_btn_label,
                               true);
    Serial.println("[PRESET] Save armed");
  } else {
    // Short press: If they released early, ensure the button isn't white
    if (sys_config.preset_mode != PRESET_MODE_SAVE_ARMED) {
      update_preset_button_style(sys_config.save_btn, sys_config.save_btn_label,
                                 false);
    }
  }
}

// Called from curve_timer_cb to handle save timeout and flash animation
static void preset_timer_check(void) {
  if (save_btn_is_pressed) {
    uint32_t held_so_far = lv_tick_get() - save_press_start;
    if (held_so_far >= PRESET_SAVE_ARM_HOLD_MS) {
      // Turn white immediately while still holding
      update_preset_button_style(sys_config.save_btn, sys_config.save_btn_label,
                                 true);
    }
  }

  // Save timeout
  if (sys_config.preset_mode == PRESET_MODE_SAVE_ARMED) {
    if ((lv_tick_get() - sys_config.save_arm_time) > PRESET_SAVE_TIMEOUT_MS) {
      preset_disarm();
      Serial.println("[PRESET] Save timeout");
    }
  }

  // Flash animation
  if (sys_config.flash_slot >= 0 && sys_config.flash_count > 0) {
    if ((lv_tick_get() - sys_config.flash_time) > PRESET_FLASH_PERIOD_MS) {
      sys_config.flash_time = lv_tick_get();
      sys_config.flash_state = !sys_config.flash_state;
      sys_config.flash_count--;

      lv_obj_t *box = sys_config.preset_boxes[sys_config.flash_slot];
      lv_obj_t *lbl = sys_config.preset_labels[sys_config.flash_slot];

      if (sys_config.flash_state) {
        // Flash ON - invert colors (white fill, black text)
        lv_obj_set_style_bg_color(box, COL_WHITE, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(lbl, COL_BLACK, 0);
      } else {
        // Flash OFF - back to active style (red fill)
        lv_obj_set_style_bg_color(box, COL_PRESET_RED, 0);
        lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
        lv_obj_set_style_text_color(lbl, COL_BLACK, 0);
      }

      if (sys_config.flash_count == 0) {
        // Done flashing - restore proper style
        sys_config.flash_slot = -1;
        update_preset_box_style(preset_get_active_slot());
      }
    }
  }

  // --- Fine-adjust readout timeout: hide FINE_READOUT_HOLD_MS after last
  // activity. Checked every tick (not gated by the 250ms flash divider) so the
  // hide is prompt. Invalidate the ring area of whichever stage is in fine mode.
  if (fine_readout_visible &&
      (lv_tick_get() - fine_readout_active_time) > FINE_READOUT_HOLD_MS) {
    fine_readout_visible = false;
    eq_stage_t *fr_stages[] = {&stage_input, &stage_output, &stage_low};
    for (int i = 0; i < 3; i++)
      fine_readout_hide(fr_stages[i]);
  }

  // --- Mute/Fine button flash (~4Hz, driven from 50ms timer) ---
  static uint8_t mute_tick = 0;
  static bool flash_state = false;
  if (++mute_tick >= 8) {  // 8 × 33ms = approx250ms
    mute_tick = 0;
    flash_state = !flash_state;
    fine_flash_on = flash_state;
    eq_stage_t *mute_stages[] = {&stage_input, &stage_output, &stage_low};
    for (int i = 0; i < 3; i++) {
      eq_stage_t *s = mute_stages[i];
      if (s->mute_btn && dsp_get_mute(i)) {
        s->mute_flash_state = flash_state;
        lv_obj_invalidate(s->mute_btn);
      }
      if (s->fine_circle && s->fine_mode) {
        lv_obj_invalidate(s->fine_circle);
      }
    }
  }
}

// Called when swiping away from config page - disarm presets
static void config_page_scroll_guard(void) {
  if (sys_config.preset_mode != PRESET_MODE_IDLE) {
    preset_disarm();
  }
}

static void create_config_page(lv_obj_t *parent) {
  tile_config_ref = parent;

  lv_obj_t *title = lv_label_create(parent);
  lv_label_set_text(title, "SYSTEM CONFIGURATION");
  lv_obj_set_style_text_color(title, COL_WHITE, 0);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
  lv_obj_set_pos(title, CURVE_X, CURVE_Y + 6);

  int start_y = CURVE_Y + 36;
  int row_h = 32;
  char buf[16];

  // ============================================================
  // LEFT COLUMN - System settings
  // ============================================================

  // Row 1: VISUALIZER dropdown (replaces FFT switch)
  lv_obj_t *vis_lbl = lv_label_create(parent);
  lv_label_set_text(vis_lbl, "VISUALIZER");
  lv_obj_set_style_text_color(vis_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(vis_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(vis_lbl, CFG_COL1_X, start_y + 4);

  sys_config.visualizer_dropdown = NULL; // Replaced by visualizer_btn below
  sys_config.visualizer_btn = lv_btn_create(parent);
  lv_obj_set_pos(sys_config.visualizer_btn, CFG_COL1_X + 80, start_y - 2);
  lv_obj_set_size(sys_config.visualizer_btn, 60, 24);
  lv_obj_add_flag(sys_config.visualizer_btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_style_bg_color(sys_config.visualizer_btn, lv_color_hex(0x333333), 0);
  lv_obj_set_style_bg_color(sys_config.visualizer_btn, lv_color_hex(0x007700), LV_STATE_CHECKED);
  lv_obj_set_style_radius(sys_config.visualizer_btn, 3, 0);
  lv_obj_t *vis_btn_lbl = lv_label_create(sys_config.visualizer_btn);
  lv_label_set_text(vis_btn_lbl, "OFF");
  lv_obj_set_style_text_color(vis_btn_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(vis_btn_lbl, &lv_font_montserrat_10, 0);
  lv_obj_center(vis_btn_lbl);
  lv_obj_add_event_cb(sys_config.visualizer_btn, visualizer_btn_event_cb,
                      LV_EVENT_VALUE_CHANGED, vis_btn_lbl);
  sys_config.visualizer_mode = 0;
  sys_config.fft_enabled = false;

  // Row 2: BRIGHT slider
  lv_obj_t *brt_lbl = lv_label_create(parent);
  lv_label_set_text(brt_lbl, "BRIGHT");
  lv_obj_set_style_text_color(brt_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(brt_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(brt_lbl, CFG_COL1_X, start_y + row_h + 4);

  sys_config.backlight_slider = lv_slider_create(parent);
  lv_obj_set_size(sys_config.backlight_slider, CFG_SLIDER_W, 6);
  lv_obj_set_pos(sys_config.backlight_slider, CFG_COL1_X + 50,
                 start_y + row_h + 8);
  lv_slider_set_range(sys_config.backlight_slider, 5, 100);
  sys_config.backlight_val = 80;
  lv_slider_set_value(sys_config.backlight_slider, sys_config.backlight_val,
                      LV_ANIM_OFF);
  style_slider_slim(sys_config.backlight_slider);
  lv_obj_add_event_cb(sys_config.backlight_slider, config_slider_event_cb,
                      LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)1);

  sys_config.backlight_val_label = lv_label_create(parent);
  lv_obj_set_style_text_color(sys_config.backlight_val_label, COL_TEXT, 0);
  lv_obj_set_style_text_font(sys_config.backlight_val_label,
                             &lv_font_montserrat_10, 0);
  lv_obj_set_pos(sys_config.backlight_val_label, CFG_COL1_X + 155,
                 start_y + row_h + 4);
  snprintf(buf, sizeof(buf), "%d%%", sys_config.backlight_val);
  lv_label_set_text(sys_config.backlight_val_label, buf);

  // Row 3: Test Signal controls (Phase 13: dropdown + level slider)
  // Layout: level slider above dropdown (vertical stack), left-aligned.

  // Test Signal Level label + slider + value
  lv_obj_t *test_lvl_lbl = lv_label_create(parent);
  lv_label_set_text(test_lvl_lbl, "TEST SIGNAL LEVEL");
  lv_obj_set_style_text_color(test_lvl_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(test_lvl_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(test_lvl_lbl, CFG_COL1_X, start_y + row_h * 2 - 6);

  sys_config.test_signal_slider = lv_slider_create(parent);
  lv_obj_set_size(sys_config.test_signal_slider, 140, 6);
  lv_obj_set_pos(sys_config.test_signal_slider, CFG_COL1_X,
                 start_y + row_h * 2 + 17);
  lv_slider_set_range(sys_config.test_signal_slider, -600,
                      0); // -60.0dB to 0.0dB
  lv_slider_set_value(sys_config.test_signal_slider, -400,
                      LV_ANIM_OFF); // -40dB default
  style_slider_slim(sys_config.test_signal_slider);
  lv_obj_add_event_cb(sys_config.test_signal_slider,
                      test_signal_slider_event_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);

  sys_config.test_signal_val_label = lv_label_create(parent);
  lv_obj_set_style_text_color(sys_config.test_signal_val_label, COL_TEXT, 0);
  lv_obj_set_style_text_font(sys_config.test_signal_val_label,
                             &lv_font_montserrat_10, 0);
  lv_obj_set_pos(sys_config.test_signal_val_label, CFG_COL1_X + 145,
                 start_y + row_h * 2 + 13);
  lv_label_set_text(sys_config.test_signal_val_label, "-40.0dB");

  // Test Signal dropdown
  lv_obj_t *test_mode_lbl = lv_label_create(parent);
  lv_label_set_text(test_mode_lbl, "TEST SIGNAL");
  lv_obj_set_style_text_color(test_mode_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(test_mode_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(test_mode_lbl, CFG_COL1_X, start_y + row_h * 2 + 35);

  sys_config.test_signal_dropdown = lv_dropdown_create(parent);
  lv_dropdown_set_options(sys_config.test_signal_dropdown,
                          "OFF\n"
                          "ADJ SINE\n"
                          "Pink Noise\n"
                          "Sweep 30-20k (30s)\n"
                          "Sweep 20-20k (35s)\n"
                          "Warble 30-20k (30s)");
  lv_dropdown_set_selected(sys_config.test_signal_dropdown, 0); // Default: OFF
  lv_obj_set_pos(sys_config.test_signal_dropdown, CFG_COL1_X,
                 start_y + row_h * 2 + 58);
  lv_obj_set_size(sys_config.test_signal_dropdown, 180, 24);
  lv_obj_set_style_bg_color(sys_config.test_signal_dropdown, COL_BOX,
                            LV_PART_MAIN);
  lv_obj_set_style_text_color(sys_config.test_signal_dropdown, COL_TEXT,
                              LV_PART_MAIN);
  lv_obj_set_style_text_font(sys_config.test_signal_dropdown,
                             &lv_font_montserrat_10, LV_PART_MAIN);
  lv_obj_add_event_cb(sys_config.test_signal_dropdown,
                      test_signal_dropdown_event_cb, LV_EVENT_VALUE_CHANGED,
                      NULL);

  // Initialize test signal state
  sys_config.test_signal_mode = 0; // OFF
  sys_config.test_signal_db = -40.0f;

  // Row 4: HI LIM switch + threshold slider (main output limiter; compact
  // one-row format matching SUB LIM so the two limiters sit visually paired.
  // Label will be revised later once a permanent layout is chosen.)
  lv_obj_t *lim_lbl = lv_label_create(parent);
  lv_label_set_text(lim_lbl, "HI LIM");
  lv_obj_set_style_text_color(lim_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(lim_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(lim_lbl, CFG_COL1_X, start_y + row_h * 4 + 32);

  sys_config.limiter_sw = lv_switch_create(parent);
  lv_obj_set_pos(sys_config.limiter_sw, CFG_COL1_X + 45,
                 start_y + row_h * 4 + 28);
  lv_obj_set_size(sys_config.limiter_sw, 36, 20);
  lv_obj_set_style_bg_color(sys_config.limiter_sw, COL_BOX, LV_PART_MAIN);
  lv_obj_add_event_cb(sys_config.limiter_sw, config_switch_event_cb,
                      LV_EVENT_VALUE_CHANGED, NULL);

  sys_config.threshold_slider = lv_slider_create(parent);
  lv_obj_set_size(sys_config.threshold_slider, 70, 6);
  lv_obj_set_pos(sys_config.threshold_slider, CFG_COL1_X + 90,
                 start_y + row_h * 4 + 36);
  lv_slider_set_range(sys_config.threshold_slider, 0, 1000);
  sys_config.limiter_threshold = -1.0f;
  lv_slider_set_value(sys_config.threshold_slider, map_threshold_to_limiter_slider(sys_config.limiter_threshold),
                      LV_ANIM_OFF);
  style_slider_slim(sys_config.threshold_slider);
  lv_obj_clear_flag(sys_config.threshold_slider, LV_OBJ_FLAG_CLICKABLE);

  sys_config.threshold_val_label = lv_label_create(parent);
  lv_obj_set_style_text_color(sys_config.threshold_val_label, COL_TEXT, 0);
  lv_obj_set_style_text_font(sys_config.threshold_val_label,
                             &lv_font_montserrat_10, 0);
  lv_obj_set_pos(sys_config.threshold_val_label, CFG_COL1_X + 165,
                 start_y + row_h * 4 + 32);
  snprintf(buf, sizeof(buf), "%.1fdB", sys_config.limiter_threshold);
  lv_label_set_text(sys_config.threshold_val_label, buf);

  // Trigger hitbox for main limiter
  lv_obj_t *lim_touch = lv_obj_create(parent);
  lv_obj_remove_style_all(lim_touch);
  lv_obj_set_pos(lim_touch, CFG_COL1_X + 85, start_y + row_h * 4 + 28);
  lv_obj_set_size(lim_touch, 120, 24); // Covers slider and label
  lv_obj_set_style_bg_opa(lim_touch, LV_OPA_TRANSP, 0);
  lv_obj_add_flag(lim_touch, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(lim_touch, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(lim_touch, limiter_trigger_cb, LV_EVENT_ALL, (void *)(intptr_t)1);

  // Row 5: SUB LIM switch + threshold slider (Phase 8)
  // Same compact pattern as the HIGH LIM row above: label, switch, slider,
  // value.
  lv_obj_t *low_lim_lbl = lv_label_create(parent);
  lv_label_set_text(low_lim_lbl, "LO LIM");
  lv_obj_set_style_text_color(low_lim_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(low_lim_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(low_lim_lbl, CFG_COL1_X, start_y + row_h * 5 + 32);

  sys_config.low_limiter_sw = lv_switch_create(parent);
  lv_obj_set_pos(sys_config.low_limiter_sw, CFG_COL1_X + 45,
                 start_y + row_h * 5 + 28);
  lv_obj_set_size(sys_config.low_limiter_sw, 36, 20);
  lv_obj_set_style_bg_color(sys_config.low_limiter_sw, COL_BOX, LV_PART_MAIN);
  lv_obj_add_event_cb(sys_config.low_limiter_sw, config_low_limiter_switch_cb,
                      LV_EVENT_VALUE_CHANGED, NULL);

  sys_config.low_threshold_slider = lv_slider_create(parent);
  lv_obj_set_size(sys_config.low_threshold_slider, 70, 6);
  lv_obj_set_pos(sys_config.low_threshold_slider, CFG_COL1_X + 90,
                 start_y + row_h * 5 + 36);
  lv_slider_set_range(sys_config.low_threshold_slider, 0, 1000);
  sys_config.low_limiter_threshold = -1.0f; // Same default as main limiter
  lv_slider_set_value(sys_config.low_threshold_slider,
                      map_threshold_to_limiter_slider(sys_config.low_limiter_threshold), LV_ANIM_OFF);
  style_slider_slim(sys_config.low_threshold_slider);
  lv_obj_clear_flag(sys_config.low_threshold_slider, LV_OBJ_FLAG_CLICKABLE);

  sys_config.low_threshold_val_label = lv_label_create(parent);
  lv_obj_set_style_text_color(sys_config.low_threshold_val_label, COL_TEXT, 0);
  lv_obj_set_style_text_font(sys_config.low_threshold_val_label,
                             &lv_font_montserrat_10, 0);
  lv_obj_set_pos(sys_config.low_threshold_val_label, CFG_COL1_X + 165,
                 start_y + row_h * 5 + 32);
  snprintf(buf, sizeof(buf), "%.1fdB", sys_config.low_limiter_threshold);
  lv_label_set_text(sys_config.low_threshold_val_label, buf);

  // Trigger hitbox for low limiter
  lv_obj_t *low_lim_touch = lv_obj_create(parent);
  lv_obj_remove_style_all(low_lim_touch);
  lv_obj_set_pos(low_lim_touch, CFG_COL1_X + 85, start_y + row_h * 5 + 28);
  lv_obj_set_size(low_lim_touch, 120, 24); // Covers slider and label
  lv_obj_set_style_bg_opa(low_lim_touch, LV_OPA_TRANSP, 0);
  lv_obj_add_flag(low_lim_touch, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(low_lim_touch, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(low_lim_touch, limiter_trigger_cb, LV_EVENT_ALL, (void *)(intptr_t)2);

  // Initialize sub limiter state (disabled by default, matches main limiter
  // default)
  sys_config.low_limiter_enabled = false;

  // ============================================================
  // RIGHT COLUMN
  //   Row 0: STORAGE label + NVS/switch/SD
  //   Row 1: PRESETS (small, inline) + LOAD + SAVE
  //   Row 2: 4 preset boxes
  // ============================================================

  // --- Row 0: STORAGE selector ---------------------------------
  // "STORAGE  NVS [switch] SD" — flips boot selector and reboots on change.
  lv_obj_t *stor_lbl = lv_label_create(parent);
  lv_label_set_text(stor_lbl, "STORAGE");
  lv_obj_set_style_text_color(stor_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(stor_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(stor_lbl, CFG_COL2_X, start_y + 4);

  lv_obj_t *nvs_lbl = lv_label_create(parent);
  lv_label_set_text(nvs_lbl, "NVS");
  lv_obj_set_style_text_color(nvs_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(nvs_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(nvs_lbl, CFG_COL2_X + 70, start_y + 4);

  sys_config.storage_switch = lv_switch_create(parent);
  lv_obj_set_pos(sys_config.storage_switch, CFG_COL2_X + 100, start_y);
  lv_obj_set_size(sys_config.storage_switch, 36, 20);
  lv_obj_set_style_bg_color(sys_config.storage_switch, COL_BOX, LV_PART_MAIN);
  if (boot_config_get_storage() == BOOT_STORAGE_SD) {
    lv_obj_add_state(sys_config.storage_switch, LV_STATE_CHECKED);
  }
  lv_obj_add_event_cb(sys_config.storage_switch, storage_switch_event_cb,
                      LV_EVENT_VALUE_CHANGED, NULL);

  lv_obj_t *sd_lbl = lv_label_create(parent);
  lv_label_set_text(sd_lbl, "SD");
  lv_obj_set_style_text_color(sd_lbl, COL_TEXT, 0);
  lv_obj_set_style_text_font(sd_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(sd_lbl, CFG_COL2_X + 142, start_y + 4);

  // --- Row 1: PRESETS (small, inline) + LOAD + SAVE -----------
  lv_obj_t *preset_title = lv_label_create(parent);
  lv_label_set_text(preset_title, "PRESETS");
  lv_obj_set_style_text_color(preset_title, COL_WHITE, 0);
  lv_obj_set_style_text_font(preset_title, &lv_font_montserrat_10, 0);
  lv_obj_set_pos(preset_title, CFG_COL2_X, start_y + row_h + 6);

  // LOAD button
  sys_config.load_btn = lv_obj_create(parent);
  lv_obj_remove_style_all(sys_config.load_btn);
  lv_obj_set_size(sys_config.load_btn, PRESET_BTN_W, PRESET_BTN_H);
  lv_obj_set_pos(sys_config.load_btn, CFG_COL2_X + 50, start_y + row_h);
  lv_obj_set_style_bg_color(sys_config.load_btn, COL_BTN_FACE, 0);
  lv_obj_set_style_bg_opa(sys_config.load_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(sys_config.load_btn, COL_WHITE, 0);
  lv_obj_set_style_border_width(sys_config.load_btn, 1, 0);
  lv_obj_set_style_border_opa(sys_config.load_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(sys_config.load_btn, 3, 0);
  lv_obj_add_flag(sys_config.load_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(sys_config.load_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(sys_config.load_btn, load_btn_cb, LV_EVENT_CLICKED, NULL);

  sys_config.load_btn_label = lv_label_create(sys_config.load_btn);
  lv_label_set_text(sys_config.load_btn_label, "LOAD");
  lv_obj_set_style_text_font(sys_config.load_btn_label, &lv_font_montserrat_10,
                             0);
  lv_obj_set_style_text_color(sys_config.load_btn_label, COL_WHITE, 0);
  lv_obj_center(sys_config.load_btn_label);

  // SAVE button
  sys_config.save_btn = lv_obj_create(parent);
  lv_obj_remove_style_all(sys_config.save_btn);
  lv_obj_set_size(sys_config.save_btn, PRESET_BTN_W, PRESET_BTN_H);
  lv_obj_set_pos(sys_config.save_btn, CFG_COL2_X + 50 + PRESET_BTN_W + 10,
                 start_y + row_h);
  lv_obj_set_style_bg_color(sys_config.save_btn, COL_BTN_FACE, 0);
  lv_obj_set_style_bg_opa(sys_config.save_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(sys_config.save_btn, COL_WHITE, 0);
  lv_obj_set_style_border_width(sys_config.save_btn, 1, 0);
  lv_obj_set_style_border_opa(sys_config.save_btn, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(sys_config.save_btn, 3, 0);
  lv_obj_add_flag(sys_config.save_btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(sys_config.save_btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(sys_config.save_btn, save_btn_pressed_cb,
                      LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(sys_config.save_btn, save_btn_released_cb,
                      LV_EVENT_RELEASED, NULL);

  sys_config.save_btn_label = lv_label_create(sys_config.save_btn);
  lv_label_set_text(sys_config.save_btn_label, "SAVE");
  lv_obj_set_style_text_font(sys_config.save_btn_label, &lv_font_montserrat_10,
                             0);
  lv_obj_set_style_text_color(sys_config.save_btn_label, COL_WHITE, 0);
  lv_obj_center(sys_config.save_btn_label);

  // 4 Preset boxes (STORAGE is Row 0, PRESETS is Row 1, boxes are Row 2)
  int box_y = start_y + row_h * 2 + 10;
  for (int i = 0; i < 4; i++) {
    int box_x = CFG_COL2_X + i * (PRESET_BOX_W + PRESET_BOX_GAP);

    sys_config.preset_boxes[i] = lv_obj_create(parent);
    lv_obj_remove_style_all(sys_config.preset_boxes[i]);
    lv_obj_set_size(sys_config.preset_boxes[i], PRESET_BOX_W, PRESET_BOX_H);
    lv_obj_set_pos(sys_config.preset_boxes[i], box_x, box_y);
    lv_obj_set_style_radius(sys_config.preset_boxes[i], 4, 0);
    lv_obj_add_flag(sys_config.preset_boxes[i], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(sys_config.preset_boxes[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(sys_config.preset_boxes[i], preset_box_clicked_cb,
                        LV_EVENT_CLICKED, (void *)(intptr_t)i);

    sys_config.preset_labels[i] = lv_label_create(sys_config.preset_boxes[i]);
    snprintf(buf, sizeof(buf), "%d", i + 1);
    lv_label_set_text(sys_config.preset_labels[i], buf);
    lv_obj_set_style_text_font(sys_config.preset_labels[i],
                               &lv_font_montserrat_14, 0);
    lv_obj_center(sys_config.preset_labels[i]);
  }

  // Initialize preset state
  sys_config.preset_mode = PRESET_MODE_IDLE;
  sys_config.flash_slot = -1;
  sys_config.flash_count = 0;

  // Style all preset boxes
  update_all_preset_boxes();

  // Input dropdown removed - not needed
  sys_config.input_dropdown = NULL;

  // Input gain sliders disabled
  sys_config.input_gain_slider_l = NULL;
  sys_config.input_gain_slider_r = NULL;
  sys_config.input_gain_label_l = NULL;
  sys_config.input_gain_label_r = NULL;

  // Nav buttons: RET (middle) returns to origin EQ tile, VIS (bottom) to spectrum.
  // Positions match the EQ page so they don't move when paging.
  create_nav_button(parent, NAV_Y1, "RET", nav_ret_cb);
  create_nav_button(parent, NAV_Y2, "VIS", nav_vis_cb);
}

// ============================================================
// Stage Initialization Helper
// ============================================================
static void init_stage_data(eq_stage_t *stage, const char *name) {
  stage->name = name;
  stage->channel_mode = CH_MODE_LINKED;
  stage->is_low = false; // Phase 6: sub stage flips this true after init
  stage->is_output_hp_overlay =
      false; // Phase 7: output stage flips this true after init

  // Init both channels identically
  eq_band_t default_band = {60.0f, 0.0f, 1.0f, FTYPE_LOW_SHELF, true};
  stage->bands_l[0] = default_band;
  stage->bands_r[0] = default_band;
  stage->num_bands_l = 1;
  stage->num_bands_r = 1;

  stage->selected_band = 0;
  stage->fine_mode = false;
  stage->curve_dirty = true;

  // Initialize gain state
  stage->stage_gain_l_db = 0.0f;
  stage->stage_gain_r_db = 0.0f;
  stage->stage_gain_slider_l = NULL;
  stage->stage_gain_slider_r = NULL;
  stage->stage_gain_label_l = NULL;
  stage->stage_gain_label_r = NULL;
  stage->stage_gain_readout_l = NULL;
  stage->stage_gain_readout_r = NULL;
  stage->gain_popup          = NULL;
  stage->gain_popup_slider_l = NULL;
  stage->gain_popup_slider_r = NULL;
  stage->gain_popup_val_l    = NULL;
  stage->gain_popup_val_r    = NULL;
  stage->gain_popup_timer    = NULL;

  // Initialize meter area object
  stage->meter_area_obj = NULL;
}

// ============================================================
// Update UI after preset load
// ============================================================
void eq_ui_update_preset_display(void) {
  // 0. Preset/session data just replaced the band arrays — clamp selection
  // into each stage's new active range BEFORE any redraw below reads it
  // (rebuild_band_buttons, sync_band_display, ring draws all derive from
  // selected_band).
  clamp_selected_band(&stage_input);
  clamp_selected_band(&stage_output);
  clamp_selected_band(&stage_low);

  // 1. Update Curves and Bands
  stage_input.curve_dirty = true;
  stage_output.curve_dirty = true;
  stage_low.curve_dirty = true;
  rebuild_band_buttons(&stage_input);
  rebuild_band_buttons(&stage_output);
  rebuild_band_buttons(&stage_low);
  sync_band_display(&stage_input);
  sync_band_display(&stage_output);
  sync_band_display(&stage_low);
  update_channel_button_style(&stage_input);
  update_channel_button_style(&stage_output);
  update_channel_button_style(&stage_low);

  // 2. Update Stage Gains
  if (stage_input.stage_gain_slider_l)
    lv_slider_set_value(stage_input.stage_gain_slider_l,
                        (int)(stage_input.stage_gain_l_db * 10), LV_ANIM_OFF);
  if (stage_input.stage_gain_slider_r)
    lv_slider_set_value(stage_input.stage_gain_slider_r,
                        (int)(stage_input.stage_gain_r_db * 10), LV_ANIM_OFF);
  if (stage_output.stage_gain_slider_l)
    lv_slider_set_value(stage_output.stage_gain_slider_l,
                        (int)(stage_output.stage_gain_l_db * 10), LV_ANIM_OFF);
  if (stage_output.stage_gain_slider_r)
    lv_slider_set_value(stage_output.stage_gain_slider_r,
                        (int)(stage_output.stage_gain_r_db * 10), LV_ANIM_OFF);
  if (stage_low.stage_gain_slider_l)
    lv_slider_set_value(stage_low.stage_gain_slider_l,
                        (int)(stage_low.stage_gain_l_db * 10), LV_ANIM_OFF);
  if (stage_low.stage_gain_slider_r)
    lv_slider_set_value(stage_low.stage_gain_slider_r,
                        (int)(stage_low.stage_gain_r_db * 10), LV_ANIM_OFF);

  // Sync readout labels
  char rbuf[8];
  if (stage_input.stage_gain_readout_l) {
    format_gain_label(rbuf, sizeof(rbuf), stage_input.stage_gain_l_db);
    lv_label_set_text(stage_input.stage_gain_readout_l, rbuf);
  }
  if (stage_input.stage_gain_readout_r) {
    format_gain_label(rbuf, sizeof(rbuf), stage_input.stage_gain_r_db);
    lv_label_set_text(stage_input.stage_gain_readout_r, rbuf);
  }
  if (stage_output.stage_gain_readout_l) {
    format_gain_label(rbuf, sizeof(rbuf), stage_output.stage_gain_l_db);
    lv_label_set_text(stage_output.stage_gain_readout_l, rbuf);
  }
  if (stage_output.stage_gain_readout_r) {
    format_gain_label(rbuf, sizeof(rbuf), stage_output.stage_gain_r_db);
    lv_label_set_text(stage_output.stage_gain_readout_r, rbuf);
  }
  if (stage_low.stage_gain_readout_l) {
    format_gain_label(rbuf, sizeof(rbuf), stage_low.stage_gain_l_db);
    lv_label_set_text(stage_low.stage_gain_readout_l, rbuf);
  }
  if (stage_low.stage_gain_readout_r) {
    format_gain_label(rbuf, sizeof(rbuf), stage_low.stage_gain_r_db);
    lv_label_set_text(stage_low.stage_gain_readout_r, rbuf);
  }

  // Update popup sliders if open
  if (stage_input.gain_popup && !lv_obj_has_flag(stage_input.gain_popup, LV_OBJ_FLAG_HIDDEN)) {
    gain_popup_open(&stage_input);
  }
  if (stage_output.gain_popup && !lv_obj_has_flag(stage_output.gain_popup, LV_OBJ_FLAG_HIDDEN)) {
    gain_popup_open(&stage_output);
  }
  if (stage_low.gain_popup && !lv_obj_has_flag(stage_low.gain_popup, LV_OBJ_FLAG_HIDDEN)) {
    gain_popup_open(&stage_low);
  }

  // 3. Update System Config Switches and Sliders
  char buf[16];
  if (sys_config.backlight_slider) {
    lv_slider_set_value(sys_config.backlight_slider, sys_config.backlight_val,
                        LV_ANIM_OFF);
    if (sys_config.backlight_val_label) {
      snprintf(buf, sizeof(buf), "%d%%", sys_config.backlight_val);
      lv_label_set_text(sys_config.backlight_val_label, buf);
    }
  }

  if (sys_config.limiter_sw) {
    if (sys_config.limiter_enabled)
      lv_obj_add_state(sys_config.limiter_sw, LV_STATE_CHECKED);
    else
      lv_obj_remove_state(sys_config.limiter_sw, LV_STATE_CHECKED);
  }
  if (sys_config.threshold_slider) {
    lv_slider_set_value(sys_config.threshold_slider,
                        map_threshold_to_limiter_slider(sys_config.limiter_threshold), LV_ANIM_OFF);
    if (sys_config.threshold_val_label) {
      snprintf(buf, sizeof(buf), "%.1fdB", sys_config.limiter_threshold);
      lv_label_set_text(sys_config.threshold_val_label, buf);
    }
  }

  // Low limiter (Phase 8)
  if (sys_config.low_limiter_sw) {
    if (sys_config.low_limiter_enabled)
      lv_obj_add_state(sys_config.low_limiter_sw, LV_STATE_CHECKED);
    else
      lv_obj_remove_state(sys_config.low_limiter_sw, LV_STATE_CHECKED);
  }
  if (sys_config.low_threshold_slider) {
    lv_slider_set_value(sys_config.low_threshold_slider,
                        map_threshold_to_limiter_slider(sys_config.low_limiter_threshold), LV_ANIM_OFF);
    if (sys_config.low_threshold_val_label) {
      snprintf(buf, sizeof(buf), "%.1fdB", sys_config.low_limiter_threshold);
      lv_label_set_text(sys_config.low_threshold_val_label, buf);
    }
  }

  // Phase 9: crossover UI refresh after preset load.
  // xover_settings has been replaced by the loaded values; sync the dropdowns,
  // freq labels, routing buttons, and curve so the UI matches the new state.
  if (crossover_page.hp_type_dropdown) {
    lv_dropdown_set_selected(crossover_page.hp_type_dropdown,
                             type_to_dd_sel(xover_settings.hp.type));
    lv_dropdown_set_selected(crossover_page.hp_slope_dropdown,
                             slope_to_dd_sel(xover_settings.hp.slope));
    lv_dropdown_set_selected(crossover_page.lp_type_dropdown,
                             type_to_dd_sel(xover_settings.lp.type));
    lv_dropdown_set_selected(crossover_page.lp_slope_dropdown,
                             slope_to_dd_sel(xover_settings.lp.slope));
    crossover_update_freq_labels();
    crossover_page.curve_dirty = true;
  }
  if (crossover_page.mono_btn)
    update_mono_button_style();
  if (crossover_page.phase_btn)
    update_phase_button_style();
  if (crossover_page.delay_box)
    update_delay_label();

  if (sys_config.visualizer_btn) {
    bool is_on = (sys_config.visualizer_mode > 0);
    if (is_on) {
      lv_obj_add_state(sys_config.visualizer_btn, LV_STATE_CHECKED);
      lv_obj_set_style_bg_color(sys_config.visualizer_btn, lv_color_hex(0x007700), 0);
    } else {
      lv_obj_clear_state(sys_config.visualizer_btn, LV_STATE_CHECKED);
      lv_obj_set_style_bg_color(sys_config.visualizer_btn, lv_color_hex(0x333333), 0);
    }
    // Update label
    lv_obj_t *lbl = lv_obj_get_child(sys_config.visualizer_btn, 0);
    if (lbl) lv_label_set_text(lbl, is_on ? "ON" : "OFF");
    // Apply mode and FFT state directly without firing the button callback
    dsp_set_fft_enabled(is_on);
    spectrum_set_mode(sys_config.visualizer_mode);
    if (tile_spectrum_ref) {
      if (is_on) {
        lv_obj_clear_flag(tile_spectrum_ref, LV_OBJ_FLAG_HIDDEN);
        spectrum_timer_set_paused(false);
      } else {
        lv_obj_add_flag(tile_spectrum_ref, LV_OBJ_FLAG_HIDDEN);
        spectrum_timer_set_paused(true);
      }
    }
  }

  // Phase 13: test signal resets to OFF on preset load — not persisted.
  if (sys_config.test_signal_dropdown) {
    lv_dropdown_set_selected(sys_config.test_signal_dropdown, 0); // OFF
  }
  if (sys_config.test_signal_slider) {
    lv_slider_set_value(sys_config.test_signal_slider,
                        (int)(sys_config.test_signal_db * 10), LV_ANIM_OFF);
    if (sys_config.test_signal_val_label) {
      snprintf(buf, sizeof(buf), "%.1fdB", sys_config.test_signal_db);
      lv_label_set_text(sys_config.test_signal_val_label, buf);
    }
  }

  // 4. Update the Red Boxes
  update_all_preset_boxes();

  // 5. Update Active Tile (Phase 15: boot memory)
  if (sys_config.active_tile >= 0 && sys_config.active_tile <= 5) {
    if (main_tileview) {
      lv_obj_set_tile_id(main_tileview, sys_config.active_tile, 0, LV_ANIM_OFF);
    }
  }
}

// ============================================================
// CROSSOVER PAGE (Phase 5)
// ============================================================
//
// Tile index 2. Layout:
//   Top: curve area (CURVE_X/Y/W/H) showing HP, LP, and sum curves.
//   Bottom: HP controls left, LP controls right, each with type+slope dropdowns
//           and a tappable frequency box that opens the number_input popup.
//
// Pattern parity with EQ pages:
//   * Custom-draw curve callback hidden during scroll (LVGL v9 trap)
//   * curve_dirty flag drives redraw via curve_timer_cb at ~30Hz
//   * Static buffers for all lv_draw_label text (LVGL v9 trap)
//   * Flat object hierarchy: dropdowns/labels parented directly to tile, not
//     to nested containers (Adam's established hierarchy rule)

// Curve colours — HP and LP get distinct hues, sum is brighter
#define COL_XO_HP lv_color_hex(0x00AAFF) // Cyan-blue
#define COL_XO_LP lv_color_hex(0xFF8800) // Orange
#define COL_XO_SUM                                                             \
  lv_color_hex(0xCCCCDD) // Off-white (subtle, doesn't dominate)

// Dropdown option strings — terse forms per Adam's preference (option B)
static const char *XOVER_TYPE_OPTIONS = "Off\nBW\nLR\nBessel";
static const char *XOVER_SLOPE_OPTIONS = "6dB\n12dB\n24dB\n48dB";

// Local forward decl — recompute/update_freq_labels are forward-declared at
// top of file (needed by curve_timer_cb / popup callback there).
static void crossover_curve_draw_cb(lv_event_t *e);

// --------- Curve computation ---------
//
// Sum curve is the coherent in-phase amplitude sum (honours phase invert):
//     sum_dB = 20·log10(|H_HP| ± |H_LP|)
// Both branches carry the same source signal, so they sum by amplitude, not
// by power. For LR filters at the crossover point this is flat (each is -6dB
// = 0.5 linear, 0.5 + 0.5 = 1.0 = 0dB). For Butterworth (-3dB at crossover)
// the sum has a +3dB bump at fc. Overlapped corners (HP well below LP) show
// the full +6dB of both branches passing the overlap band at unity.
// Delay is not modelled — this is the in-phase reference sum.
static void crossover_recompute_curve(void) {
  ui_xover_t ui_hp, ui_lp;
  ui_xover_calc(&xover_settings.hp, current_fs, true, &ui_hp);
  ui_xover_calc(&xover_settings.lp, current_fs, false, &ui_lp);

  for (int i = 0; i <= CURVE_POINTS; i++) {
    float t = (float)i / CURVE_POINTS;
    float freq = log_to_freq(t);

    float hp_db = ui_xover_eval(&ui_hp, freq, current_fs);
    float lp_db = ui_xover_eval(&ui_lp, freq, current_fs);

    // Coherent amplitude sum (see header comment): the branches carry the
    // same signal, so they add linearly. Phase invert flips the low branch.
    float hp_lin = powf(10.0f, hp_db / 20.0f);
    float lp_lin = powf(10.0f, lp_db / 20.0f);
    float sum_lin =
        fabsf(hp_lin + (xover_settings.phase_invert ? -lp_lin : lp_lin));
    float sum_db = (sum_lin > 1e-10f) ? 20.0f * log10f(sum_lin) : -100.0f;

    // Clamp into display range
    if (hp_db > DB_RANGE)
      hp_db = DB_RANGE;
    if (hp_db < -DB_RANGE)
      hp_db = -DB_RANGE;
    if (lp_db > DB_RANGE)
      lp_db = DB_RANGE;
    if (lp_db < -DB_RANGE)
      lp_db = -DB_RANGE;
    if (sum_db > DB_RANGE)
      sum_db = DB_RANGE;
    if (sum_db < -DB_RANGE)
      sum_db = -DB_RANGE;

    lv_value_precise_t x = (lv_value_precise_t)(t * (CURVE_W - 1));
    crossover_page.hp_curve_pts[i].x = x;
    crossover_page.lp_curve_pts[i].x = x;
    crossover_page.sum_curve_pts[i].x = x;

    float hp_y = CURVE_H / 2.0f - (hp_db / DB_RANGE) * (CURVE_H / 2.0f);
    float lp_y = CURVE_H / 2.0f - (lp_db / DB_RANGE) * (CURVE_H / 2.0f);
    float sum_y = CURVE_H / 2.0f - (sum_db / DB_RANGE) * (CURVE_H / 2.0f);
    if (hp_y < 1.0f)
      hp_y = 1.0f;
    if (hp_y > CURVE_H - 1)
      hp_y = CURVE_H - 1;
    if (lp_y < 1.0f)
      lp_y = 1.0f;
    if (lp_y > CURVE_H - 1)
      lp_y = CURVE_H - 1;
    if (sum_y < 1.0f)
      sum_y = 1.0f;
    if (sum_y > CURVE_H - 1)
      sum_y = CURVE_H - 1;

    crossover_page.hp_curve_pts[i].y = (lv_value_precise_t)hp_y;
    crossover_page.lp_curve_pts[i].y = (lv_value_precise_t)lp_y;
    crossover_page.sum_curve_pts[i].y = (lv_value_precise_t)sum_y;
  }
}

// --------- Curve draw callback ---------
static void crossover_curve_draw_cb(lv_event_t *e) {
  extern volatile bool tileview_scrolling;
  if (tileview_scrolling)
    return;

  lv_layer_t *layer = lv_event_get_layer(e);
  lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);

  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);
  int ox = obj_coords.x1;
  int oy = obj_coords.y1;

  lv_draw_line_dsc_t line_dsc;
  lv_draw_label_dsc_t label_dsc;

  // Graph border
  lv_draw_rect_dsc_t border_dsc;
  lv_draw_rect_dsc_init(&border_dsc);
  border_dsc.bg_opa = LV_OPA_TRANSP;
  border_dsc.border_color = COL_GRID_ZERO;
  border_dsc.border_width = 1;
  border_dsc.border_opa = LV_OPA_COVER;
  border_dsc.radius = 0;
  lv_area_t frame = {(lv_coord_t)ox, (lv_coord_t)oy,
                     (lv_coord_t)(ox + CURVE_W - 1),
                     (lv_coord_t)(oy + CURVE_H - 1)};
  lv_draw_rect(layer, &border_dsc, &frame);

  // Horizontal grid lines
  for (int db = -15; db <= 15; db += 5) {
    if (db == 0)
      continue;
    int y = oy + CURVE_H / 2 - (int)((float)db / DB_RANGE * CURVE_H / 2.0f);
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COL_GRID;
    line_dsc.width = 1;
    line_dsc.p1.x = ox;
    line_dsc.p1.y = y;
    line_dsc.p2.x = ox + CURVE_W;
    line_dsc.p2.y = y;
    lv_draw_line(layer, &line_dsc);
  }

  // Zero line
  lv_draw_line_dsc_init(&line_dsc);
  line_dsc.color = COL_GRID_ZERO;
  line_dsc.width = 1;
  line_dsc.p1.x = ox;
  line_dsc.p1.y = oy + CURVE_H / 2;
  line_dsc.p2.x = ox + CURVE_W;
  line_dsc.p2.y = oy + CURVE_H / 2;
  lv_draw_line(layer, &line_dsc);

  // Vertical grid + freq labels — copied verbatim from EQ page pattern
  static const float freq_marks[] = {20,   50,   100,  200,   500,
                                     1000, 2000, 5000, 10000, 20000};
  static const char *freq_labels[] = {"20", "50", "100", "200", "500",
                                      "1k", "2k", "5k",  "10k", "20k"};
  for (int i = 0; i < 10; i++) {
    int x = ox + (int)(freq_to_log(freq_marks[i]) * CURVE_W);
    lv_draw_line_dsc_init(&line_dsc);
    line_dsc.color = COL_GRID;
    line_dsc.width = 1;
    line_dsc.p1.x = x;
    line_dsc.p1.y = oy;
    line_dsc.p2.x = x;
    line_dsc.p2.y = oy + CURVE_H;
    lv_draw_line(layer, &line_dsc);

    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = COL_TEXT_DIM;
    label_dsc.font = &lv_font_montserrat_10;
    label_dsc.text = freq_labels[i];

    lv_area_t la;
    if (i == 0) {
      label_dsc.align = LV_TEXT_ALIGN_LEFT;
      la = (lv_area_t){(lv_coord_t)(x + 2), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x + 30), (lv_coord_t)(oy + CURVE_H - 1)};
    } else if (i == 9) {
      label_dsc.align = LV_TEXT_ALIGN_RIGHT;
      la = (lv_area_t){(lv_coord_t)(x - 30), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x - 2), (lv_coord_t)(oy + CURVE_H - 1)};
    } else {
      label_dsc.align = LV_TEXT_ALIGN_CENTER;
      la = (lv_area_t){(lv_coord_t)(x - 15), (lv_coord_t)(oy + CURVE_H - 13),
                       (lv_coord_t)(x + 15), (lv_coord_t)(oy + CURVE_H - 1)};
    }
    lv_draw_label(layer, &label_dsc, &la);
  }

  // dB labels inside graph
  static const int db_marks[] = {20, 10, -10};
  static const char *db_labels[] = {"20", "10", "10"};
  for (int i = 0; i < 3; i++) {
    int db_val = db_marks[i];
    int y = oy + CURVE_H / 2 - (int)((float)db_val / DB_RANGE * CURVE_H / 2.0f);
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = COL_TEXT_DIM;
    label_dsc.font = &lv_font_montserrat_10;
    label_dsc.text = db_labels[i];
    label_dsc.align = LV_TEXT_ALIGN_LEFT;
    lv_area_t la;
    if (db_val > 0) {
      la = (lv_area_t){(lv_coord_t)(ox + 2), (lv_coord_t)(y + 1),
                       (lv_coord_t)(ox + 24), (lv_coord_t)(y + 13)};
    } else {
      la = (lv_area_t){(lv_coord_t)(ox + 2), (lv_coord_t)(y - 13),
                       (lv_coord_t)(ox + 24), (lv_coord_t)(y - 1)};
    }
    lv_draw_label(layer, &label_dsc, &la);
  }

  // Page name label — area widened from the EQ-pages' 100px to 140px so
  // "CROSSOVER" (9 chars at montserrat_18) fits on one line.
  lv_draw_label_dsc_init(&label_dsc);
  label_dsc.color = COL_WHITE;
  label_dsc.font = &lv_font_montserrat_18;
  static const char xover_label_text[] =
      "CROSSOVER"; // static for LVGL v9 safety
  label_dsc.text = xover_label_text;
  label_dsc.align = LV_TEXT_ALIGN_RIGHT;
  lv_area_t name_la = {(lv_coord_t)(ox + CURVE_W - 140), (lv_coord_t)(oy + 8),
                       (lv_coord_t)(ox + CURVE_W - 8), (lv_coord_t)(oy + 28)};
  lv_draw_label(layer, &label_dsc, &name_la);

// Helper to draw one polyline from segment array.
// Note: parameter is named `w_` not `width` because some platform header
// (LVGL or Arduino) defines `width` as a numeric macro, which corrupted
// `cd.width` on expansion.
#define DRAW_POLY(pts, col, w_)                                                \
  do {                                                                         \
    lv_draw_line_dsc_t cd;                                                     \
    lv_draw_line_dsc_init(&cd);                                                \
    cd.color = (col);                                                          \
    cd.width = (w_);                                                           \
    cd.round_start = 0;                                                        \
    cd.round_end = 0;                                                          \
    for (int i = 0; i < CURVE_POINTS; i++) {                                   \
      cd.p1.x = ox + (lv_coord_t)(pts)[i].x;                                   \
      cd.p1.y = oy + (lv_coord_t)(pts)[i].y;                                   \
      cd.p2.x = ox + (lv_coord_t)(pts)[i + 1].x;                               \
      cd.p2.y = oy + (lv_coord_t)(pts)[i + 1].y;                               \
      lv_draw_line(layer, &cd);                                                \
    }                                                                          \
  } while (0)

  // Draw HP and LP first (thin), sum on top (thicker, brighter).
  // Sum is hidden when either filter is bypassed — the sum curve only has
  // physical meaning when both bands are actively crossing over. When one
  // or both are pass-through, the audio still goes to two separate DACs
  // (not electrically summed), so showing a +3dB sum line would be
  // misleading.
  DRAW_POLY(crossover_page.hp_curve_pts, COL_XO_HP, 1);
  DRAW_POLY(crossover_page.lp_curve_pts, COL_XO_LP, 1);
  if (xover_settings.hp.enabled && xover_settings.lp.enabled &&
      xover_settings.hp.type != XOVER_BYPASS &&
      xover_settings.lp.type != XOVER_BYPASS) {
    DRAW_POLY(crossover_page.sum_curve_pts, COL_XO_SUM, 2);
  }

#undef DRAW_POLY
}

// --------- Frequency label updates ---------
//
// Each freq box has a child label showing the current value (e.g. "120 Hz").
// Updated whenever HP or LP freq changes — from popup confirm or initial build.
static void crossover_update_freq_labels(void) {
  static char hp_buf[16]; // static per LVGL v9 trap
  static char lp_buf[16];
  char fhp[12];
  format_freq(fhp, sizeof(fhp), xover_settings.hp.freq);
  char flp[12];
  format_freq(flp, sizeof(flp), xover_settings.lp.freq);
  snprintf(hp_buf, sizeof(hp_buf), "%s Hz", fhp);
  snprintf(lp_buf, sizeof(lp_buf), "%s Hz", flp);
  if (crossover_page.hp_freq_label)
    lv_label_set_text(crossover_page.hp_freq_label, hp_buf);
  if (crossover_page.lp_freq_label)
    lv_label_set_text(crossover_page.lp_freq_label, lp_buf);
}

// --------- Event handlers ---------

// Map dropdown selection (0-3) to xover_filter_type_t. We intentionally use the
// same ordering as the enum so a direct cast works:
//   0=Off→XOVER_BYPASS, 1=BW→XOVER_BUTTERWORTH, 2=LR→XOVER_LINKWITZ_RILEY,
//   3=Bessel
// Slope dropdown maps directly to xover_slope_t (0=6dB, 1=12dB, 2=24dB,
// 3=48dB).

static void hp_type_dropdown_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  int sel = lv_dropdown_get_selected(crossover_page.hp_type_dropdown);
  xover_settings.hp.type = (xover_filter_type_t)sel;
  xover_settings.hp.enabled = (sel != XOVER_BYPASS);
  xover_update_hp(&xover_settings.hp);
  crossover_page.curve_dirty = true;
  stage_output.curve_dirty = true; // Output EQ page shows HP overlay (Phase 7)
  preset_mark_dirty();
}

static void hp_slope_dropdown_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  int sel = lv_dropdown_get_selected(crossover_page.hp_slope_dropdown);
  xover_settings.hp.slope = (xover_slope_t)sel;
  xover_update_hp(&xover_settings.hp);
  crossover_page.curve_dirty = true;
  stage_output.curve_dirty = true;
  preset_mark_dirty();
}

static void lp_type_dropdown_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  int sel = lv_dropdown_get_selected(crossover_page.lp_type_dropdown);
  xover_settings.lp.type = (xover_filter_type_t)sel;
  xover_settings.lp.enabled = (sel != XOVER_BYPASS);
  xover_update_lp(&xover_settings.lp);
  crossover_page.curve_dirty = true;
  stage_low.curve_dirty = true; // Low page curve includes LP overlay
  preset_mark_dirty();
}

static void lp_slope_dropdown_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED)
    return;
  int sel = lv_dropdown_get_selected(crossover_page.lp_slope_dropdown);
  xover_settings.lp.slope = (xover_slope_t)sel;
  xover_update_lp(&xover_settings.lp);
  crossover_page.curve_dirty = true;
  stage_low.curve_dirty = true;
  preset_mark_dirty();
}

// Popup callback dispatch — user_data carries 0=HP, 1=LP
static void freq_popup_cb(float value, bool cancelled, void *user_data) {
  if (cancelled)
    return;
  int which = (int)(intptr_t)user_data;
  if (which == 0) {
    xover_settings.hp.freq = value;
    xover_update_hp(&xover_settings.hp);
    stage_output.curve_dirty = true;
  } else {
    xover_settings.lp.freq = value;
    xover_update_lp(&xover_settings.lp);
    stage_low.curve_dirty = true; // Low page curve includes LP overlay
  }
  crossover_update_freq_labels();
  crossover_page.curve_dirty = true;
  preset_mark_dirty();
}

static void hp_freq_box_event_cb(lv_event_t *e) {
  (void)e;
  number_input_show("HP Freq (Hz)", xover_settings.hp.freq, 30.0f, 20000.0f, 0,
                    false, freq_popup_cb, (void *)(intptr_t)0);
}

static void lp_freq_box_event_cb(lv_event_t *e) {
  (void)e;
  number_input_show("LP Freq (Hz)", xover_settings.lp.freq, 30.0f, 20000.0f, 0,
                    false, freq_popup_cb, (void *)(intptr_t)1);
}

// --------- Page builder ---------

#define XO_CTRL_TOP (CURVE_Y + CURVE_H + 8) // y origin for control row
#define XO_DD_W 72
#define XO_DD_H 24
#define XO_FREQ_BOX_W 86
#define XO_FREQ_BOX_H 24
#define XO_LBL_W 30
#define XO_ROW_PAD 4

// Phase 6 routing row (row 3): mono, phase, delay
// NOTE: These buttons moved to the low page (under the "LOW" label).
// See LOW_RBTN_* defines near create_low_routing_buttons().

// Helper: make a styled box that visually matches the freq box / dropdown
// look. Used for the mono and phase toggles, and the delay tap target.
// Note: COL_BOX = inactive, COL_BOX_ACT = active/checked.
static lv_obj_t *make_xover_button(lv_obj_t *parent, int x, int y, int w, int h,
                                   const char *initial_text,
                                   lv_obj_t **label_out, lv_event_cb_t cb) {
  lv_obj_t *btn = lv_obj_create(parent);
  lv_obj_remove_style_all(btn);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(btn, COL_BOX, 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_border_color(btn, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(btn, 3, 0);
  lv_obj_set_style_pad_all(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(btn);
  lv_label_set_text(lbl, initial_text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_center(lbl);

  if (label_out)
    *label_out = lbl;
  return btn;
}

// Visual state refresh for the mono/phase toggles. Active = brighter bg.
static void update_mono_button_style(void) {
  if (!crossover_page.mono_btn)
    return;
  bool on = xover_settings.low_mono;
  lv_obj_set_style_bg_color(crossover_page.mono_btn, on ? COL_BOX_ACT : COL_BOX,
                            0);
  // Static buffer: LVGL v9 trap — see project rules.
  static const char *txt_on = "MONO";
  static const char *txt_off = "STEREO";
  lv_label_set_text(crossover_page.mono_label, on ? txt_on : txt_off);
}

static void update_phase_button_style(void) {
  if (!crossover_page.phase_btn)
    return;
  bool inv = xover_settings.phase_invert;
  lv_obj_set_style_bg_color(crossover_page.phase_btn,
                            inv ? COL_BOX_ACT : COL_BOX, 0);
  static const char *txt_0 = "PH 0";
  static const char *txt_180 = "PH 180";
  lv_label_set_text(crossover_page.phase_label, inv ? txt_180 : txt_0);
}

// Static label buffer for the delay box — LVGL v9 trap rule.
static char xo_delay_label_buf[24];

static void update_delay_label(void) {
  if (!crossover_page.delay_label)
    return;
  snprintf(xo_delay_label_buf, sizeof(xo_delay_label_buf), "DLY %+.3f ms",
           xover_settings.delay_ms);
  lv_label_set_text(crossover_page.delay_label, xo_delay_label_buf);
}

static void mono_btn_cb(lv_event_t *e) {
  (void)e;
  xover_settings.low_mono = !xover_settings.low_mono;
  xover_set_mono(xover_settings.low_mono);
  update_mono_button_style();
  preset_mark_dirty();
}

static void phase_btn_cb(lv_event_t *e) {
  (void)e;
  xover_settings.phase_invert = !xover_settings.phase_invert;
  xover_set_phase_invert(xover_settings.phase_invert);
  update_phase_button_style();
  preset_mark_dirty();
}

static void delay_popup_cb(float value, bool cancelled, void *user_data) {
  (void)user_data;
  if (cancelled)
    return;
  xover_settings.delay_ms = value;
  xover_set_delay(value);
  update_delay_label();
  preset_mark_dirty();
}

static void delay_box_event_cb(lv_event_t *e) {
  (void)e;
  number_input_show("Delay (ms)", xover_settings.delay_ms, -8.0f, 8.0f, 3, true,
                    delay_popup_cb, NULL);
}

// Helper: create a styled dropdown matching the codebase's panel/box look.
static lv_obj_t *make_xover_dropdown(lv_obj_t *parent, int x, int y, int w,
                                     const char *opts, int initial_sel,
                                     lv_event_cb_t cb) {
  lv_obj_t *dd = lv_dropdown_create(parent);
  lv_dropdown_set_options(dd, opts);
  lv_obj_set_pos(dd, x, y);
  lv_obj_set_size(dd, w, XO_DD_H);
  lv_obj_set_style_bg_color(dd, COL_BOX, 0);
  lv_obj_set_style_border_color(dd, COL_BOX_BORDER, 0);
  lv_obj_set_style_text_color(dd, COL_TEXT, 0);
  lv_obj_set_style_text_font(dd, &lv_font_montserrat_10, 0);
  lv_dropdown_set_selected(dd, initial_sel);
  lv_obj_add_event_cb(dd, cb, LV_EVENT_VALUE_CHANGED, NULL);
  return dd;
}

// Helper: create a tappable freq box (visually similar to a dropdown but
// triggers number_input on click).
static void make_freq_box(lv_obj_t *parent, int x, int y, lv_obj_t **box_out,
                          lv_obj_t **label_out, lv_event_cb_t cb) {
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_remove_style_all(box);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_size(box, XO_FREQ_BOX_W, XO_FREQ_BOX_H);
  lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(box, COL_BOX, 0);
  lv_obj_set_style_border_width(box, 1, 0);
  lv_obj_set_style_border_color(box, COL_BOX_BORDER, 0);
  lv_obj_set_style_radius(box, 3, 0);
  lv_obj_set_style_pad_all(box, 0, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_flag(box, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_add_event_cb(box, cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *lbl = lv_label_create(box);
  lv_label_set_text(lbl, "120 Hz");
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lbl, COL_TEXT, 0);
  lv_obj_center(lbl);

  *box_out = box;
  *label_out = lbl;
}

// Map xover_filter_type_t → dropdown selection (same numeric value).
static int type_to_dd_sel(xover_filter_type_t t) { return (int)t; }
static int slope_to_dd_sel(xover_slope_t s) { return (int)s; }

static void create_crossover_page(lv_obj_t *parent) {
  crossover_page.parent_tile = parent;

  // --- Curve area (custom-draw, parented flat to the tile per hierarchy rule)
  // ---
  lv_obj_t *curve_area = lv_obj_create(parent);
  lv_obj_remove_style_all(curve_area);
  lv_obj_set_size(curve_area, CURVE_W, CURVE_H);
  lv_obj_set_pos(curve_area, CURVE_X, CURVE_Y);
  lv_obj_set_style_bg_opa(curve_area, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(curve_area, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE |
                                                LV_OBJ_FLAG_SCROLLABLE));
  lv_obj_add_event_cb(curve_area, crossover_curve_draw_cb,
                      LV_EVENT_DRAW_MAIN_END, NULL);
  crossover_page.curve_area_obj = curve_area;

  // --- Section labels and controls ---
  // Layout: LP on the LEFT (low-frequency end of curve x-axis), HP on the
  // RIGHT (high-frequency end). Matches reading the controls left-to-right
  // along the same axis as the curves above.
  int lp_x = CURVE_X;
  int hp_x = CURVE_X + 232;
  int row1_y = XO_CTRL_TOP;
  int row2_y = XO_CTRL_TOP + XO_DD_H + 6;

  // HP section label
  lv_obj_t *hp_lbl = lv_label_create(parent);
  lv_label_set_text(hp_lbl, "HP");
  lv_obj_set_style_text_font(hp_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(hp_lbl, COL_XO_HP, 0);
  lv_obj_set_pos(hp_lbl, hp_x, row1_y + 4);

  // HP type + slope dropdowns (row 1)
  crossover_page.hp_type_dropdown = make_xover_dropdown(
      parent, hp_x + XO_LBL_W, row1_y, XO_DD_W, XOVER_TYPE_OPTIONS,
      type_to_dd_sel(xover_settings.hp.type), hp_type_dropdown_cb);
  crossover_page.hp_slope_dropdown = make_xover_dropdown(
      parent, hp_x + XO_LBL_W + XO_DD_W + XO_ROW_PAD, row1_y, XO_DD_W,
      XOVER_SLOPE_OPTIONS, slope_to_dd_sel(xover_settings.hp.slope),
      hp_slope_dropdown_cb);

  // HP freq box (row 2)
  lv_obj_t *hp_freq_lbl = lv_label_create(parent);
  lv_label_set_text(hp_freq_lbl, "Freq:");
  lv_obj_set_style_text_font(hp_freq_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(hp_freq_lbl, COL_TEXT, 0);
  lv_obj_set_pos(hp_freq_lbl, hp_x, row2_y + 6);

  make_freq_box(parent, hp_x + XO_LBL_W, row2_y, &crossover_page.hp_freq_box,
                &crossover_page.hp_freq_label, hp_freq_box_event_cb);

  // LP section label
  lv_obj_t *lp_lbl = lv_label_create(parent);
  lv_label_set_text(lp_lbl, "LP");
  lv_obj_set_style_text_font(lp_lbl, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(lp_lbl, COL_XO_LP, 0);
  lv_obj_set_pos(lp_lbl, lp_x, row1_y + 4);

  crossover_page.lp_type_dropdown = make_xover_dropdown(
      parent, lp_x + XO_LBL_W, row1_y, XO_DD_W, XOVER_TYPE_OPTIONS,
      type_to_dd_sel(xover_settings.lp.type), lp_type_dropdown_cb);
  crossover_page.lp_slope_dropdown = make_xover_dropdown(
      parent, lp_x + XO_LBL_W + XO_DD_W + XO_ROW_PAD, row1_y, XO_DD_W,
      XOVER_SLOPE_OPTIONS, slope_to_dd_sel(xover_settings.lp.slope),
      lp_slope_dropdown_cb);

  lv_obj_t *lp_freq_lbl = lv_label_create(parent);
  lv_label_set_text(lp_freq_lbl, "Freq:");
  lv_obj_set_style_text_font(lp_freq_lbl, &lv_font_montserrat_10, 0);
  lv_obj_set_style_text_color(lp_freq_lbl, COL_TEXT, 0);
  lv_obj_set_pos(lp_freq_lbl, lp_x, row2_y + 6);

  make_freq_box(parent, lp_x + XO_LBL_W, row2_y, &crossover_page.lp_freq_box,
                &crossover_page.lp_freq_label, lp_freq_box_event_cb);

  // Initial label population + curve compute
  crossover_update_freq_labels();
  crossover_page.curve_dirty = true;

  // Nav buttons: CFG (middle) to config, VIS (bottom) to spectrum. Positions
  // match the EQ page so they don't move when paging.
  create_nav_button(parent, NAV_Y1, "CFG", nav_cfg_cb);
  create_nav_button(parent, NAV_Y2, "VIS", nav_vis_cb);
}

// ============================================================
// Low-page routing buttons (MONO / PH / DLY)
// ============================================================
// Rendered inside the sub-page curve area, stacked vertically below the
// "LOW" label on the right edge. Right-aligned so they sit out of the way
// of the EQ curves (which are mostly low-frequency / left-side on the sub).
//
// Uses compact montserrat_10 labels; `make_xover_button` defaults to
// montserrat_14, so we override the font after creation.
#define LOW_RBTN_W 84
#define LOW_RBTN_H 16
#define LOW_RBTN_GAP 2
// Right-edge inset: 8px in from the curve-area right edge.
#define LOW_RBTN_X (CURVE_X + CURVE_W - 8 - LOW_RBTN_W)
// First row sits just below the "LOW" label (which ends at curve-relative
// y=28).
#define LOW_RBTN_Y0 (CURVE_Y + 32)
#define LOW_RBTN_Y1 (LOW_RBTN_Y0 + LOW_RBTN_H + LOW_RBTN_GAP)
#define LOW_RBTN_Y2 (LOW_RBTN_Y1 + LOW_RBTN_H + LOW_RBTN_GAP)

static void create_low_routing_buttons(lv_obj_t *parent) {
  crossover_page.mono_btn =
      make_xover_button(parent, LOW_RBTN_X, LOW_RBTN_Y0, LOW_RBTN_W, LOW_RBTN_H,
                        "MONO", &crossover_page.mono_label, mono_btn_cb);

  crossover_page.phase_btn =
      make_xover_button(parent, LOW_RBTN_X, LOW_RBTN_Y1, LOW_RBTN_W, LOW_RBTN_H,
                        "PH 0", &crossover_page.phase_label, phase_btn_cb);

  crossover_page.delay_box = make_xover_button(
      parent, LOW_RBTN_X, LOW_RBTN_Y2, LOW_RBTN_W, LOW_RBTN_H, "DLY +0.000 ms",
      &crossover_page.delay_label, delay_box_event_cb);

  // Override to compact font (make_xover_button defaults to montserrat_14)
  lv_obj_set_style_text_font(crossover_page.mono_label, &lv_font_montserrat_10,
                             0);
  lv_obj_set_style_text_font(crossover_page.phase_label, &lv_font_montserrat_10,
                             0);
  lv_obj_set_style_text_font(crossover_page.delay_label, &lv_font_montserrat_10,
                             0);

  // Seed visual state from xover_settings (populated in eq_ui_create before
  // this runs — same source-of-truth pattern as the crossover dropdowns).
  update_mono_button_style();
  update_phase_button_style();
  update_delay_label();
}

// ============================================================
// Build the complete UI
// ============================================================
void eq_ui_create(void) {
  init_stage_data(&stage_input, "INPUT");
  init_stage_data(&stage_output, "HIGH OUT");
  stage_output.is_output_hp_overlay =
      true; // Phase 7: show HP crossover overlay on output curve
  init_stage_data(&stage_low, "LOW OUT");
  stage_low.is_low = true;

  // --- Crossover defaults (Phase 5) ---
  // Single source of truth: this struct seeds both the UI dropdowns and the
  // DSP filter coefficients. Must be populated BEFORE create_crossover_page()
  // (which reads it to set initial dropdown selections) and BEFORE dsp_init()
  // (which reads it to set initial filter coefficients).
  // Phase 9 will replace these with values loaded from preset flash.
  xover_settings.hp.type = XOVER_LINKWITZ_RILEY;
  xover_settings.hp.slope = XOVER_SLOPE_24DB;
  xover_settings.hp.freq = 2200.0f;
  xover_settings.hp.enabled = true;
  xover_settings.lp.type = XOVER_LINKWITZ_RILEY;
  xover_settings.lp.slope = XOVER_SLOPE_24DB;
  xover_settings.lp.freq = 2200.0f;
  xover_settings.lp.enabled = true;
  xover_settings.phase_invert = false;
  xover_settings.delay_ms = 0.0f;
  xover_settings.low_mono = true;

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, COL_BG, 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  lv_obj_t *tileview = lv_tileview_create(scr);
  main_tileview = tileview;
  lv_obj_remove_style_all(tileview);
  lv_obj_set_size(tileview, SCREEN_W, SCREEN_H);
  lv_obj_set_scrollbar_mode(tileview, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scroll_snap_x(tileview, LV_SCROLL_SNAP_CENTER);

  // Adjust friction/momentum for a "snappy" landing without dangerous pointer
  // hacks
  lv_obj_remove_flag(tileview, LV_OBJ_FLAG_SCROLL_MOMENTUM);
  lv_obj_set_style_anim_duration(tileview, 50, 0); // Fast snap-back

  // Register scroll events to drive the robust scroll guard flag
  lv_obj_add_event_cb(tileview, tileview_scroll_event_cb, LV_EVENT_SCROLL_BEGIN,
                      NULL);
  lv_obj_add_event_cb(tileview, tileview_scroll_event_cb, LV_EVENT_SCROLL_END,
                      NULL);

  lv_obj_t *tile_config = lv_tileview_add_tile(tileview, 0, 0, LV_DIR_HOR);
  lv_obj_t *tile_input = lv_tileview_add_tile(tileview, 1, 0, LV_DIR_HOR);
  lv_obj_t *tile_crossover = lv_tileview_add_tile(tileview, 2, 0, LV_DIR_HOR);
  lv_obj_t *tile_output = lv_tileview_add_tile(tileview, 3, 0, LV_DIR_HOR);
  lv_obj_t *tile_sub = lv_tileview_add_tile(tileview, 4, 0, LV_DIR_HOR);
  lv_obj_t *tile_spectrum = lv_tileview_add_tile(tileview, 5, 0, LV_DIR_HOR);
  tile_spectrum_ref = tile_spectrum; // Store for FFT toggle

  stage_input.parent_tile = tile_input;
  stage_output.parent_tile = tile_output;
  stage_low.parent_tile = tile_sub;

  create_config_page(tile_config);
  create_eq_page(tile_input, &stage_input);
  create_crossover_page(tile_crossover);
  create_eq_page(tile_output, &stage_output);
  create_eq_page(tile_sub, &stage_low);
  create_low_routing_buttons(tile_sub);
  spectrum_ui_create(tile_spectrum);

  // Filter-list overlay: built once (hidden), shared across all three EQ
  // stages. Must be created AFTER the EQ pages (so their tiles exist) but
  // BEFORE any code path that might open it. It parents to lv_scr_act(),
  // not to any tile, so it floats above the tileview.
  filter_list_init();

  // Tone panel: built once on the top layer (hidden), shown on ADJ SINE.
  create_tone_panel();

  // FFT/Spectrum is disabled by default — hide tile and pause timer
  lv_obj_add_flag(tile_spectrum, LV_OBJ_FLAG_HIDDEN);
  spectrum_timer_set_paused(true);

  // Set the initial tile to the Input EQ
  lv_obj_set_tile_id(tileview, 1, 0, LV_ANIM_OFF);

  // Initialize DSP with active bands (both channels, all three stages)
  for (int i = 0; i < stage_input.num_bands_l; i++) {
    dsp_update_band(0, 0, i, &stage_input.bands_l[i]);
    dsp_update_band(0, 1, i, &stage_input.bands_r[i]);
  }
  dsp_set_active_bands(0, 0, stage_input.num_bands_l);
  dsp_set_active_bands(0, 1, stage_input.num_bands_r);
  for (int i = 0; i < stage_output.num_bands_l; i++) {
    dsp_update_band(1, 0, i, &stage_output.bands_l[i]);
    dsp_update_band(1, 1, i, &stage_output.bands_r[i]);
  }
  dsp_set_active_bands(1, 0, stage_output.num_bands_l);
  dsp_set_active_bands(1, 1, stage_output.num_bands_r);
  for (int i = 0; i < stage_low.num_bands_l; i++) {
    dsp_update_band(2, 0, i, &stage_low.bands_l[i]);
    dsp_update_band(2, 1, i, &stage_low.bands_r[i]);
  }
  dsp_set_active_bands(2, 0, stage_low.num_bands_l);
  dsp_set_active_bands(2, 1, stage_low.num_bands_r);

  // Phase 13: Initialize test signal (default: OFF)
  dsp_set_input_source(DSP_INPUT_I2S);
  dsp_update_noise_gen(true, sys_config.test_signal_db);

  // Defer DSP meter activation to the timer's first post-settle tick
  // to ensure the DSP task is fully initialized and accepting commands.
  last_active_tile_idx = -1;

  // Force initial meter display
  lv_obj_invalidate(stage_input.meter_area_obj);

  // --- Curve update timer (20Hz) ---
  curve_timer = lv_timer_create(curve_timer_cb, 33, NULL);
}
#ifndef EQ_UI_H
#define EQ_UI_H

#include <lvgl.h>
#include <stdio.h>
#include "eq_data.h"

void eq_ui_create(void);
void eq_ui_update_preset_display(void);
void hardware_set_backlight(int percent);

extern float current_fs;
extern volatile bool dsp_suspended;

typedef enum {
    CH_MODE_LINKED = 0,
    CH_MODE_LEFT,
    CH_MODE_RIGHT
} channel_mode_t;

typedef struct {
    const char * name;
    eq_band_t bands_l[MAX_BANDS];
    eq_band_t bands_r[MAX_BANDS];
    int num_bands_l;
    int num_bands_r;
    int selected_band;
    channel_mode_t channel_mode;
    bool fine_mode;
    float fine_freq_lo, fine_freq_hi;
    float fine_q_lo, fine_q_hi;
    float fine_gain_lo, fine_gain_hi;
    lv_obj_t * curve_area_obj;
    lv_obj_t * meter_area_obj;
    lv_obj_t * parent_tile;
    lv_obj_t * freq_slider_obj;
    lv_obj_t * q_slider_obj;
    lv_obj_t * band_gain_slider_obj;
    lv_obj_t * freq_val_label;
    lv_obj_t * q_val_label;
    lv_obj_t * band_gain_val_label;
    lv_obj_t * fine_btn;
    lv_obj_t * fine_btn_label;
    lv_obj_t * lsh_btn;           // Low Shelf button: lit when selected band is LSH, dim otherwise
    lv_obj_t * hsh_btn;           // High Shelf button: lit when selected band is HSH, dim otherwise
    lv_obj_t * ch_l_btn;
    lv_obj_t * ch_l_label;
    lv_obj_t * ch_r_btn;
    lv_obj_t * ch_r_label;
    lv_obj_t * ch_lk_btn;
    lv_obj_t * ch_lk_label;
    lv_obj_t * stage_gain_slider_l;
    lv_obj_t * stage_gain_slider_r;
    lv_obj_t * stage_gain_label_l;
    lv_obj_t * stage_gain_label_r;
    float stage_gain_l_db;
    float stage_gain_r_db;
    // Gain readout labels shown below the small sliders
    lv_obj_t * stage_gain_readout_l;
    lv_obj_t * stage_gain_readout_r;
    // Gain popup overlay (full-height, launched by tapping the small sliders)
    lv_obj_t * gain_popup;
    lv_obj_t * gain_popup_slider_l;
    lv_obj_t * gain_popup_slider_r;
    lv_obj_t * gain_popup_val_l;
    lv_obj_t * gain_popup_val_r;
    lv_timer_t * gain_popup_timer;
    lv_obj_t * band_btns[MAX_BANDS];
    lv_obj_t * band_btn_labels[MAX_BANDS];
    lv_obj_t * add_band_btn;   // fixed ADD BND button (bottom-right)
    lv_obj_t * del_band_btn;   // fixed DEL BND button (bottom-left)
    lv_obj_t * mute_btn;       // per-stage mute (below gain sliders)
    bool mute_flash_state;     // toggled by curve_timer for flash effect
    lv_point_precise_t curve_pts[CURVE_POINTS + 1];
    bool curve_dirty;
    // Crossover overlay (used on low and output stages, Phase 6/7).
    //   - Low stage:    curve_pts = combined LP+EQ response; overlay = LP-only reference.
    //   - Output stage: curve_pts = combined HP+EQ response; overlay = HP-only reference.
    // Only one of is_low / is_output_hp_overlay is ever true per stage.
    bool is_low;
    bool is_output_hp_overlay;
    lv_point_precise_t xover_overlay_pts[CURVE_POINTS + 1];
} eq_stage_t;

typedef enum {
    PRESET_MODE_IDLE = 0,
    PRESET_MODE_LOAD_ARMED,
    PRESET_MODE_SAVE_ARMED
} preset_mode_t;

typedef struct {
    int backlight_val;
    int input_source_idx;
    bool limiter_enabled;
    float limiter_threshold;
    bool low_limiter_enabled;    // Phase 8
    float  low_limiter_threshold;  // Phase 8
    int test_signal_mode;        // Phase 13: index into test signal dropdown
    float test_signal_db;        // Phase 13: shared level for all test signals
    bool fft_enabled; // Keep for backward compatibility with presets
    int visualizer_mode;
    lv_obj_t * visualizer_dropdown; // NULL — replaced by visualizer_btn
    lv_obj_t * visualizer_btn;      // ON/OFF toggle; double-tap cycles modes
    lv_obj_t * storage_switch;        // NVS / SD selector on the Config page
    lv_obj_t * backlight_slider;
    lv_obj_t * backlight_val_label;
    lv_obj_t * limiter_sw;
    lv_obj_t * threshold_slider;
    lv_obj_t * threshold_val_label;
    lv_obj_t * low_limiter_sw;        // Phase 8
    lv_obj_t * low_threshold_slider;  // Phase 8
    lv_obj_t * low_threshold_val_label; // Phase 8
    lv_obj_t * input_dropdown;
    lv_obj_t * test_signal_dropdown; // Phase 13: replaces noise_sw
    lv_obj_t * test_signal_slider;   // Phase 13: replaces noise_slider
    lv_obj_t * test_signal_val_label; // Phase 13: replaces noise_val_label
    lv_obj_t * input_gain_slider_l;
    lv_obj_t * input_gain_slider_r;
    lv_obj_t * input_gain_label_l;
    lv_obj_t * input_gain_label_r;
    lv_obj_t * load_btn;
    lv_obj_t * load_btn_label;
    lv_obj_t * save_btn;
    lv_obj_t * save_btn_label;
    lv_obj_t * preset_boxes[4];
    lv_obj_t * preset_labels[4];
    preset_mode_t preset_mode;
    uint32_t save_arm_time;
    int flash_slot;
    int flash_count;
    uint32_t flash_time;
    bool flash_state;
    int active_tile;
    
    // Phase 14: Limiter horizontal popups
    lv_obj_t * limiter_popup;
    lv_obj_t * limiter_popup_slider;
    lv_obj_t * limiter_popup_val;
    lv_timer_t * limiter_popup_timer;
    int active_limiter_popup; // 0=none, 1=main, 2=low
} sys_config_t;

extern eq_stage_t stage_input;
extern eq_stage_t stage_output;
extern sys_config_t sys_config;

// --- Crossover page UI structure (Phase 1 scaffolding) ---

typedef struct {
    lv_obj_t *parent_tile;
    lv_obj_t *curve_area_obj;

    // HP controls
    lv_obj_t *hp_type_dropdown;
    lv_obj_t *hp_slope_dropdown;
    lv_obj_t *hp_freq_box;
    lv_obj_t *hp_freq_label;

    // LP controls
    lv_obj_t *lp_type_dropdown;
    lv_obj_t *lp_slope_dropdown;
    lv_obj_t *lp_freq_box;
    lv_obj_t *lp_freq_label;

    // Routing controls — pre-EQ mono sum, low polarity, low time alignment.
    // Rendered on the LOW page (tile 4) under the "LOW" label, not on the
    // crossover page. The pointers live here because they map directly to
    // xover_settings and are logically routing/crossover state, not low-EQ.
    lv_obj_t *mono_btn;
    lv_obj_t *mono_label;
    lv_obj_t *phase_btn;
    lv_obj_t *phase_label;
    lv_obj_t *delay_box;
    lv_obj_t *delay_label;

    // Curve points for drawing
    lv_point_precise_t hp_curve_pts[CURVE_POINTS + 1];
    lv_point_precise_t lp_curve_pts[CURVE_POINTS + 1];
    lv_point_precise_t sum_curve_pts[CURVE_POINTS + 1];
    bool curve_dirty;
} crossover_page_t;

// --- Low output page (Phase 6) ---
//
// The low page is just another eq_stage_t (`stage_low`) with `is_low = true`.
// Its curve carries the COMBINED LP-crossover + low-EQ response, with the
// LP-only overlay drawn dimmed underneath as a reference. All the EQ controls
// (band buttons, sliders, channel selector, stage gain) come for free from
// the existing create_eq_page() builder. Routing controls (mono/phase/delay)
// live on the crossover page above.

extern eq_stage_t stage_low;

extern crossover_page_t crossover_page;
extern crossover_settings_t xover_settings;

// ============================================================
// Stage-ops API (Phase 12: for filter_list.cpp)
// ============================================================
// Public wrappers around eq_ui.cpp internals, so filter_list and any other
// future UI that manipulates bands doesn't need to know about the
// channel-mode / DSP-dispatch / mirroring machinery. All functions here
// respect the stage's channel_mode (operating on bands_l or bands_r as
// appropriate, and mirroring in linked mode).
// ------------------------------------------------------------

// Active-channel band array and count.
eq_band_t *eq_ui_get_bands(eq_stage_t *s);
int        eq_ui_get_band_count(eq_stage_t *s);

// Select a band (updates sliders + tab-bar highlight). Does not push to DSP.
void eq_ui_select_band(eq_stage_t *stage, int idx);

// After editing one band's fields in place (freq/gain/q/type/enabled),
// call this to push to DSP, mirror to the linked channel, refresh the
// curve, and mark the preset dirty.
void eq_ui_commit_band(eq_stage_t *stage, int idx);

// Append a new band with defaults {PK, 1000Hz, 0dB, Q=1.0, enabled}.
// Selects the new band. No-op at MAX_BANDS.
void eq_ui_add_band(eq_stage_t *stage);

// Remove the band at `idx`, shifting higher bands down by one. Unlike the
// old pop-last-band remove tab, this handles arbitrary indices. No-op when
// count is already 1 or idx is out of range.
void eq_ui_remove_band_at(eq_stage_t *stage, int idx);

// Band color palette — 10 distinct colors, cycling for bands 11-20.
// Used by band buttons, curve markers, and filter list rows.
lv_color_t band_color(int band_idx);
lv_color_t band_color_text(int band_idx);  // contrast text (black or white)

#endif
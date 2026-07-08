#include "spectrum_analyzer.h"
#include "dsp_engine.h"
#include "eq_ui.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>

// Forward declaration — defined in preset_store.cpp
extern void preset_mark_dirty(void);

// ============================================================
// Layout constants
// ============================================================
#define SCREEN_W 480
#define SCREEN_H 272

#define GRAPH_W 410
#define GRAPH_H 230
#define GRAPH_X 45
#define GRAPH_Y 15

#define DB_MIN -90.0f
#define DB_MAX 0.0f
#define DB_RANGE (DB_MAX - DB_MIN)         
#define DB_RANGE_INV (1.0f / DB_RANGE)     

// ============================================================
// Shared state
// ============================================================
float shared_spectrum_magnitudes[NUM_SPECTRUM_BANDS] = {0};

static float display_magnitudes[NUM_SPECTRUM_BANDS] = {0};
static float peak_magnitudes[NUM_SPECTRUM_BANDS] = {0};
static lv_obj_t *spectrum_area = NULL;
static lv_obj_t *spectrum_parent_tile = NULL;
static lv_timer_t *spectrum_timer_handle = NULL;

// Shared scroll guard flag — set/cleared by tileview events in eq_ui.cpp
extern volatile bool tileview_scrolling;
extern uint32_t last_scroll_end_time;
#define SCROLL_SETTLE_MS 150

static int current_visualizer_mode = 1;

void spectrum_set_mode(int mode) {
    current_visualizer_mode = mode;
    if (spectrum_timer_handle) {
        lv_timer_set_period(spectrum_timer_handle, 50);  // All modes at 20 fps
    }
    if (spectrum_area) {
        lv_obj_invalidate(spectrum_area);
    }
}

// Set when a gesture (swipe) is detected, so the CLICKED that LVGL fires on the
// same release isn't mistaken for a tap. Cleared on a fresh press.
static bool spectrum_was_gesture = false;
// Press point, to reject low-quality swipes that stay below LVGL's gesture
// threshold (so GESTURE never fires) yet still travel too far to be a tap.
static lv_point_t spectrum_press_pt = {0, 0};
#define SPEC_TAP_MAX_TRAVEL 12   // px; >this from press-point = swipe, not tap. Tune on HW.

static void spectrum_press_cb(lv_event_t *e) {
    (void)e;
    spectrum_was_gesture = false;   // new interaction
    lv_indev_get_point(lv_indev_active(), &spectrum_press_pt);
}

static void spectrum_tap_cb(lv_event_t *e) {
    (void)e;
    if (tileview_scrolling) return;
    if (spectrum_was_gesture) { spectrum_was_gesture = false; return; }
    lv_point_t rel;
    lv_indev_get_point(lv_indev_active(), &rel);
    int dx = rel.x - spectrum_press_pt.x;
    int dy = rel.y - spectrum_press_pt.y;
    if (dx * dx + dy * dy > SPEC_TAP_MAX_TRAVEL * SPEC_TAP_MAX_TRAVEL) return;  // moved too far = swipe
    eq_ui_viz_return();
}

static void spectrum_gesture_cb(lv_event_t *e) {
    if (tileview_scrolling) return;
    spectrum_was_gesture = true;   // mark so the following CLICKED is ignored
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
    if (dir == LV_DIR_TOP) {
        // Swipe up — next mode
        int next_mode = current_visualizer_mode + 1;
        if (next_mode > 6) next_mode = 1;
        spectrum_set_mode(next_mode);
        sys_config.visualizer_mode = next_mode;
        preset_mark_dirty();
    } else if (dir == LV_DIR_BOTTOM) {
        // Swipe down — previous mode
        int next_mode = current_visualizer_mode - 1;
        if (next_mode < 1) next_mode = 6;
        spectrum_set_mode(next_mode);
        sys_config.visualizer_mode = next_mode;
        preset_mark_dirty();
    }
}


// ============================================================
// VU Meter constants & state (mode 3)
// ============================================================
#define VU_PIVOT_X          240
#define VU_PIVOT_Y          340    
#define VU_NEEDLE_LEN       268
#define VU_SCALE_R_OUTER    280    
#define VU_SCALE_R_INNER    276    
#define VU_TICK_MAJOR_IN    268    
#define VU_TICK_MAJOR_OUT   288    
#define VU_TICK_MINOR_IN    272    
#define VU_TICK_MINOR_OUT   284
#define VU_LABEL_R          254    
#define VU_ARC_START_DEG   -50.0f
#define VU_ARC_END_DEG      50.0f
#define VU_DB_MIN          -40.0f  
#define VU_DB_MAX            3.0f
#define VU_RED_ZONE_DB       0.0f  
#define VU_PEAK_LED_DB      -0.5f  
#define VU_PEAK_HOLD_MS      200   

#define VU_COL_L            0xFFDD00  
#define VU_COL_R            0x44AAFF  
#define VU_COL_SCALE        0xDDDDDD  
#define VU_COL_RED          0xFF2222  
#define VU_COL_LED_ON       0xFF1111  
#define VU_COL_LED_OFF      0x2A0000  

static float l_db_display = -100.0f;
static float r_db_display = -100.0f;
static uint32_t vu_peak_led_l_until_ms = 0;
static uint32_t vu_peak_led_r_until_ms = 0;

static bool   vu_lut_ready = false;
static float  vu_vmin;          
static float  vu_range_inv;     
static float  vu_red_start_t;   

#define VU_ARC_SEGMENTS 60
static int16_t vu_arc_ox[VU_ARC_SEGMENTS + 1], vu_arc_oy[VU_ARC_SEGMENTS + 1];
static int16_t vu_arc_ix[VU_ARC_SEGMENTS + 1], vu_arc_iy[VU_ARC_SEGMENTS + 1];
static int16_t vu_arc_mx[VU_ARC_SEGMENTS + 1], vu_arc_my[VU_ARC_SEGMENTS + 1];

static int16_t vu_maj_in_x[10], vu_maj_in_y[10];
static int16_t vu_maj_out_x[10], vu_maj_out_y[10];
static int16_t vu_maj_lbl_x[10], vu_maj_lbl_y[10];

static int16_t vu_min_in_x[7], vu_min_in_y[7];
static int16_t vu_min_out_x[7], vu_min_out_y[7];

static void vu_precompute(void) {
    if (vu_lut_ready) return;
    float full = powf(10.0f, VU_DB_MAX / 20.0f);
    vu_vmin = powf(10.0f, VU_DB_MIN / 20.0f);
    vu_range_inv = 1.0f / (full - vu_vmin);

    auto db_norm = [&](float db) -> float {
        float v = powf(10.0f, db / 20.0f);
        float t = (v - vu_vmin) * vu_range_inv;
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        return t;
    };

    auto norm_to_angle_rad = [](float t) -> float {
        float deg = VU_ARC_START_DEG + t * (VU_ARC_END_DEG - VU_ARC_START_DEG);
        return deg * (float)M_PI / 180.0f;
    };

    vu_red_start_t = db_norm(VU_RED_ZONE_DB);

    for (int i = 0; i <= VU_ARC_SEGMENTS; i++) {
        float t = (float)i / (float)VU_ARC_SEGMENTS;
        float rad = norm_to_angle_rad(t);
        float sr = sinf(rad), cr = cosf(rad);
        vu_arc_ox[i] = (int16_t)(VU_SCALE_R_OUTER * sr);
        vu_arc_oy[i] = (int16_t)(-VU_SCALE_R_OUTER * cr);
        vu_arc_ix[i] = (int16_t)(VU_SCALE_R_INNER * sr);
        vu_arc_iy[i] = (int16_t)(-VU_SCALE_R_INNER * cr);
        float mid_r = (VU_SCALE_R_OUTER + VU_SCALE_R_INNER) * 0.5f;
        vu_arc_mx[i] = (int16_t)(mid_r * sr);
        vu_arc_my[i] = (int16_t)(-mid_r * cr);
    }

    static const float major_db[] = {-40.0f, -20.0f, -10.0f, -7.0f, -5.0f, -3.0f, 0.0f, 1.0f, 2.0f, 3.0f};
    for (int i = 0; i < 10; i++) {
        float t = db_norm(major_db[i]);
        float rad = norm_to_angle_rad(t);
        float sr = sinf(rad), cr = cosf(rad);
        vu_maj_in_x[i] = (int16_t)(VU_TICK_MAJOR_IN * sr);
        vu_maj_in_y[i] = (int16_t)(-VU_TICK_MAJOR_IN * cr);
        vu_maj_out_x[i] = (int16_t)(VU_TICK_MAJOR_OUT * sr);
        vu_maj_out_y[i] = (int16_t)(-VU_TICK_MAJOR_OUT * cr);
        vu_maj_lbl_x[i] = (int16_t)(VU_LABEL_R * sr);
        vu_maj_lbl_y[i] = (int16_t)(-VU_LABEL_R * cr);
    }

    static const float minor_db[] = {-30.0f, -15.0f, -8.0f, -6.0f, -4.0f, -2.0f, -1.0f};
    for (int i = 0; i < 7; i++) {
        float t = db_norm(minor_db[i]);
        float rad = norm_to_angle_rad(t);
        float sr = sinf(rad), cr = cosf(rad);
        vu_min_in_x[i] = (int16_t)(VU_TICK_MINOR_IN * sr);
        vu_min_in_y[i] = (int16_t)(-VU_TICK_MINOR_IN * cr);
        vu_min_out_x[i] = (int16_t)(VU_TICK_MINOR_OUT * sr);
        vu_min_out_y[i] = (int16_t)(-VU_TICK_MINOR_OUT * cr);
    }
    vu_lut_ready = true;
}

// ============================================================
// Starfield visualizer (mode 4)
// ============================================================
#define SF_MAX_STARS      250     
#define SF_BASE_STARS     100     
#define SF_FFT_STARS        9     
#define SF_Z_MAX          80.0f
#define SF_Z_MIN           0.3f
#define SF_SPEED_MIN       0.15f  
#define SF_SPEED_MAX       1.0f   
#define SF_BURST_MULT      5.0f   
#define SF_PROJ           400.0f  
#define SF_THRESHOLD      -40.0f  
#define SF_COL_STAR       0xCCCCCC

static struct {
    float x, y, z;
} sf_stars[SF_MAX_STARS];

static int sf_active_count = SF_BASE_STARS;
static float sf_global_speed = SF_SPEED_MIN;
static bool sf_band_hot[SF_FFT_STARS] = {};  

static const int sf_fft_band[] = {10, 23, 37, 50, 63, 73, 81, 87, 93};
static const uint32_t sf_fft_color[] = {
    0xCC3333, 0xDD7700, 0xCCAA00, 0x33BB33, 0x00AA88, 0x00BBDD, 0x4488EE, 0x8855DD, 0xCC44AA
};

static uint32_t sf_rng = 0xDEADBEEF;
static float sf_randf(void) {
    sf_rng ^= sf_rng << 13;
    sf_rng ^= sf_rng >> 17;
    sf_rng ^= sf_rng << 5;
    return (float)sf_rng / 4294967295.0f;
}

static void sf_respawn(int i) {
    sf_stars[i].x = (sf_randf() * 2.0f - 1.0f) * 20.0f;
    sf_stars[i].y = (sf_randf() * 2.0f - 1.0f) * 20.0f;
    sf_stars[i].z = SF_Z_MAX;
}

static bool sf_initialized = false;
static void sf_init(void) {
    for (int i = 0; i < SF_MAX_STARS; i++) {
        sf_stars[i].x = (sf_randf() * 2.0f - 1.0f) * 20.0f;
        sf_stars[i].y = (sf_randf() * 2.0f - 1.0f) * 20.0f;
        sf_stars[i].z = SF_Z_MIN + sf_randf() * (SF_Z_MAX - SF_Z_MIN);
    }
    sf_initialized = true;
}

// ============================================================
// Warp Field visualizer (mode 5)
// ============================================================
#define WF_MAX_STARS      125     
#define WF_BASE_STARS     100     // Smooth performance sweet spot
#define WF_Z_MAX          80.0f
#define WF_Z_MIN           0.3f
#define WF_SPEED_MIN       0.40f  
#define WF_SPEED_MAX       5.0f   // Doubled range for aggressive reactivity
#define WF_PROJ           400.0f
#define WF_TAIL_MAX_LEN   120.0f  // Increased 20% to 120
#define WF_COL_STAR       0xA020F0 
#define WF_COL_TAIL       0x300060 
#define WF_COL_TAIL_WARP  0xA051C6 // 20% whiter purple

static struct {
    float x, y, z;
    float speed;
    float nx, ny; 
} wf_stars[WF_MAX_STARS];

static int wf_active_group = 0;
static float wf_global_speed = WF_SPEED_MIN;
static bool  wf_initialized  = false;

static void wf_respawn(int i) {
    // Stars are spread randomly across the full 360-degree field
    float angle = sf_randf() * 2.0f * 3.14159f;
    float dist  = 2.0f + sf_randf() * 32.0f; 
    wf_stars[i].nx = cosf(angle);
    wf_stars[i].ny = sinf(angle);
    wf_stars[i].x = wf_stars[i].nx * dist;
    wf_stars[i].y = wf_stars[i].ny * dist;
    wf_stars[i].z = WF_Z_MAX - sf_randf() * 15.0f;
    wf_stars[i].speed = WF_SPEED_MIN;
}

static void wf_init(void) {
    for (int i = 0; i < WF_MAX_STARS; i++) {
        float angle = sf_randf() * 2.0f * 3.14159f;
        float dist  = 2.0f + sf_randf() * 30.0f; 
        wf_stars[i].nx = cosf(angle);
        wf_stars[i].ny = sinf(angle);
        wf_stars[i].x = wf_stars[i].nx * dist;
        wf_stars[i].y = wf_stars[i].ny * dist;
        wf_stars[i].z = SF_Z_MIN + sf_randf() * (SF_Z_MAX - SF_Z_MIN);
        wf_stars[i].speed = WF_SPEED_MIN;
    }
    wf_active_group = 0;
    wf_initialized = true;
}

static float log_20;           
static float log_range_inv;    
static int band_x_lut[NUM_SPECTRUM_BANDS];

#define NUM_FREQ_MARKS 10
static int grid_x_lut[NUM_FREQ_MARKS];
static float iso_freqs[NUM_SPECTRUM_BANDS];
static const int freq_marks[NUM_FREQ_MARKS] = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
static const char* freq_labels[NUM_FREQ_MARKS] = {"20", "50", "100", "200", "500", "1k", "2k", "5k", "10k", "20k"};

#define NUM_DB_MARKS 5
static const int db_marks[NUM_DB_MARKS] = {0, -20, -40, -60, -90};
static int grid_y_lut[NUM_DB_MARKS];

static float freq_to_norm_log(float f) {
    if(f < 20.0f) f = 20.0f;
    if(f > 20000.0f) f = 20000.0f;
    return (logf(f) - log_20) * log_range_inv;
}

static lv_color_t get_rainbow_color(float t) {
    float h = t * 6.0f;
    int i = (int)h;
    float f = h - (float)i;
    float q = 1.0f - f;
    float r = 0, g = 0, b = 0;
    switch(i % 6) {
        case 0: r = 1.0f; g = f; b = 0.0f; break;
        case 1: r = q; g = 1.0f; b = 0.0f; break;
        case 2: r = 0.0f; g = 1.0f; b = f; break;
        case 3: r = 0.0f; g = q; b = 1.0f; break;
        case 4: r = f; g = 0.0f; b = 1.0f; break;
        case 5: r = 1.0f; g = 0.0f; b = q; break;
    }
    return lv_color_hex((uint32_t)((uint8_t)(r*255.0f) << 16) | (uint32_t)((uint8_t)(g*255.0f) << 8) | (uint8_t)(b*255.0f));
}

static lv_color_t get_intensity_color(float norm_y) {
    uint8_t r, g, b;
    if (norm_y < 0.33f) {
        float t = norm_y * 3.030303f;
        r = 255; g = 255; b = (uint8_t)(255.0f * (1.0f - t));
    } else if (norm_y < 0.66f) {
        float t = (norm_y - 0.33f) * 3.030303f;
        r = 255; g = (uint8_t)(255.0f - (t * 85.0f)); b = 0;
    } else {
        float t = (norm_y - 0.66f) * 2.941176f;
        r = 255; g = (uint8_t)(170.0f - (t * 170.0f)); b = 0;
    }
    return lv_color_hex((uint32_t)(r << 16) | (uint32_t)(g << 8) | b);
}

// ============================================================
// Mode 6: Whirlpool (M51) — chaos-game IFS into persistent PSRAM buffer
// ============================================================
// Drive is spectral SHAPE: per-band dB relative to the spectrum mean, so
// response is level-independent above the noise gate. Spectral flux
// (self-normalised against its own running average) adds bounded transient
// punch. No resets: silence starves point inflow to a simmer and the
// rolling decay dims the field; new material inks over old as the
// transform parameters drift (quasi-periodic LFO stacks — never settles).
//
// Per tick (all in spectrum_timer_cb, mode 6):
//   band extract -> AGC/gate/flux -> morph params -> chaos points deposit
//   into wp_acc (RGB888, saturating) -> rolling row decay -> tone-map LUT
//   -> wp_img (RGB565) -> invalidate. Draw cb blits wp_img full-screen.

#define WP_PERF_SERIAL 0              // printf sim/tonemap timings
#define WP_PERF_EVERY  40             // every N frames (2 s at 20 fps)

#define WP_W SCREEN_W
#define WP_H SCREEN_H
#define WP_NPX (WP_W * WP_H)
#define WP_BANDS  8                   // spectral groups == transforms
#define WP_XFORMS 8

#define WP_PTS_SIMMER 400             // floor: embers stay alive in silence
#define WP_PTS_MAX    4500

#define WP_GATE_DB     (DB_MIN + 15.0f)  // mean env below this = silence
#define WP_AGC_SPAN_DB 12.0f             // gate -> full drive span
#define WP_AGC_ATTACK  0.30f
#define WP_AGC_RELEASE 0.008f            // ~6 s tau at 20 fps

#define WP_WEXP_K  0.09f              // weight = exp(K*(band_dB - mean_dB))
#define WP_W_FLOOR 0.02f              // no organ ever fully dies

#define WP_FLUX_AVG_COEF 0.02f
#define WP_PUNCH_MIN 0.70f
#define WP_PUNCH_MAX 1.50f

#define WP_INTEN_BASE 10.0f           // deposit per hit at punch = 1
#define WP_DECAY_ROWS 34              // 272/34: full sweep every 8 frames
#define WP_DECAY_MUL  0.90f

#define WP_SCALE_X 230.0f             // attractor unit space -> px (anisotropic:
#define WP_SCALE_Y 130.0f             //  ratio ~480/272 so the field fills the panel)
#define WP_ESCAPE_R2 16.0f

#define WP_SLUT_N 1024                // sine LUT size (power of two)

// --- zoom servo: regulates the off-screen reject fraction so the attractor's
// density boundary always lies past the panel edges ---
#define WP_REJ_LO    0.08f            // below this: boundary approaching, zoom in
#define WP_REJ_HI    0.25f            // above this: wasting points, zoom out
#define WP_REJ_EMA   0.05f
#define WP_ZOOM_MIN  1.0f
#define WP_ZOOM_MAX  2.5f
#define WP_ZOOM_SLEW 1.002f           // per-frame multiplicative step

// --- slow field rotation (drifting-past feel) ---
#define WP_ROT_RATE 0.0008f           // rad/frame, scaled by morph rate

// --- starfield ---
#define WP_NSTARS        140
#define WP_CLUSTER_FRAC  0.42f        // fraction seeded around cluster centres
#define WP_WARM_FRAC     0.25f        // orange-shifted stars; rest blue-white
#define WP_STAR_DEP      0.040f       // per-frame deposit per unit raw brightness.
                                      //  Steady state ~= 72 * deposit under the
                                      //  rolling decay: raw 82 -> ~236, 10 -> ~29
#define WP_STAR_BIG_B    52.0f        // raw brightness above which star spills 2px
#define WP_GLINT_P       0.0016f      // per star per frame scintillation chance
#define WP_GLINT_GAIN    4.0f         // deposit multiplier during a glint (tune)

// --- nebula hue dwell warp: hue advances faster through the cool sector so
// the palette lingers on warms. Cool sector spans 220 deg vs 140 deg warm;
// mult 1.57 = equal warm/cool dwell, higher skews warm (1.8 -> ~53% warm) ---
#define WP_HUE_COOL_LO   70.0f
#define WP_HUE_COOL_HI   290.0f
#define WP_HUE_COOL_MULT 1.8f

static uint8_t  *wp_mem = NULL;       // single lazy PSRAM block, carved below
static uint8_t  *wp_acc = NULL;       // RGB888 accumulation
static uint16_t *wp_img = NULL;       // RGB565 blit buffer
static bool wp_alloc_failed = false;
static lv_image_dsc_t wp_img_dsc;

// All mutable arrays live in the PSRAM block (internal BSS kept ~150 B —
// boot-time internal heap must stay clear for the 8 KB DSP task stack)
static float   *wp_slut      = NULL;  // (WP_SLUT_N + 1) floats
static uint8_t *wp_tone_lut  = NULL;  // 256
static uint8_t *wp_decay_lut = NULL;  // 256

static float   *wp_prev_mag  = NULL;  // NUM_SPECTRUM_BANDS floats

// Starfield: fixed positions, integer twinkle phase accumulators (immune to
// float-precision drift over long uptimes), pre-tinted per-frame deposits
typedef struct {
    uint16_t x, y;
    uint16_t tw_ph;                   // twinkle phase, wraps at 65536
    uint16_t tw_rate;                 // phase increment per frame
    float    dep_r, dep_g, dep_b;     // tinted deposit at nominal twinkle
    uint8_t  big;                     // spills into +x / +y neighbours
    uint8_t  glint;                   // frames of scintillation remaining
    uint8_t  pad[2];
} wp_star_t;                          // 24 B
static wp_star_t *wp_stars = NULL;    // WP_NSTARS entries, in the PSRAM block

static float wp_zoom    = 1.3f;       // servo state
static float wp_rej_ema = 0.15f;
static float wp_rot     = 0.0f;       // field rotation angle
static float    wp_env_db   = DB_MIN;
static float    wp_flux_avg = 5.0f;
static float    wp_mt       = 0.0f;   // morph time (flux-rate-modulated)
static uint32_t wp_frame    = 0;
static int      wp_decay_row = 0;

// Persistent chaos-game trajectory
static float wp_ptx = 0.1f, wp_pty = 0.1f;
static int   wp_pcr = 200, wp_pcg = 200, wp_pcb = 200;

static float       wp_hue[WP_XFORMS]      = {310.0f, 45.0f, 140.0f, 255.0f, 20.0f, 90.0f, 200.0f, 330.0f};
static const float wp_hue_rate[WP_XFORMS] = {0.11f, 0.07f, 0.16f, 0.05f, 0.09f, 0.13f, 0.06f, 0.10f};

// Band group edges over the 102 log-spaced bins (== NUM_SPECTRUM_BANDS)
static const uint8_t wp_edges[WP_BANDS + 1] = {0, 13, 26, 38, 51, 64, 76, 89, 102};

// Band k drives transform k. Archetypes: 0 spiral<->horseshoe,
// 1 sinusoidal<->polar, 2 spherical<->swirl, 3 linear anchor.
static const uint8_t wp_arch[WP_XFORMS]    = {0, 3, 1, 2, 0, 1, 2, 3};
static const uint8_t wp_variant[WP_XFORMS] = {0, 0, 0, 0, 1, 1, 1, 1};

// Per-transform LFO phase offsets (keeps the 8 organs incommensurate)
static const float wp_ph[WP_XFORMS][5] = {
    {0.0f, 2.0f, 1.0f, 4.0f, 0.7f},
    {2.2f, 1.8f, 3.6f, 0.3f, 5.1f},
    {3.0f, 5.0f, 1.1f, 2.9f, 0.2f},
    {0.4f, 5.5f, 4.2f, 1.6f, 2.4f},
    {3.1f, 0.9f, 5.3f, 2.2f, 4.4f},
    {1.4f, 4.1f, 0.6f, 5.8f, 2.7f},
    {5.2f, 2.6f, 3.9f, 1.2f, 0.8f},
    {2.8f, 0.5f, 1.9f, 3.3f, 5.6f}};

typedef struct { float a, b, c, d, e, m; } wp_xp_t;
static wp_xp_t *wp_xp = NULL;         // WP_XFORMS entries, in the PSRAM block

static uint32_t wp_rng = 0x1234ABCDu;
static inline float wp_randf(void) {
    wp_rng ^= wp_rng << 13;
    wp_rng ^= wp_rng >> 17;
    wp_rng ^= wp_rng << 5;
    return (float)wp_rng * (1.0f / 4294967296.0f);
}

// LUT sine with linear interp — per-point trig without libm
static inline float wp_sin(float x) {
    float u = x * ((float)WP_SLUT_N / 6.2831853f);
    float fl = floorf(u);
    int i = (int)fl;
    float f = u - fl;
    int i0 = i & (WP_SLUT_N - 1);
    return wp_slut[i0] + (wp_slut[i0 + 1] - wp_slut[i0]) * f;
}
static inline float wp_cos(float x) { return wp_sin(x + 1.5707963f); }

static void wp_hsv(float h, float s, float v, uint8_t *out) {
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    float c = v * s;
    float hp = h * (1.0f / 60.0f);
    float x = c * (1.0f - fabsf(fmodf(hp, 2.0f) - 1.0f));
    float r = 0.0f, g = 0.0f, b = 0.0f;
    switch ((int)hp) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    float m = v - c;
    out[0] = (uint8_t)((r + m) * 255.0f);
    out[1] = (uint8_t)((g + m) * 255.0f);
    out[2] = (uint8_t)((b + m) * 255.0f);
}

static bool wp_ensure_init(void) {
    if (wp_alloc_failed) return false;
    if (wp_mem) return true;
    // Single PSRAM block, carved. Ordering keeps float/struct segments at
    // 4-byte offsets: 4100 + 408 + 192 are all multiples of 4; the uint16
    // image lands on an even offset; byte arrays follow unconstrained.
    const size_t SZ_SLUT = (size_t)(WP_SLUT_N + 1) * sizeof(float);       // 4100
    const size_t SZ_PREV = (size_t)NUM_SPECTRUM_BANDS * sizeof(float);    // 408
    const size_t SZ_XP   = (size_t)WP_XFORMS * sizeof(wp_xp_t);           // 192
    const size_t SZ_STAR = (size_t)WP_NSTARS * sizeof(wp_star_t);         // 3360
    const size_t SZ_IMG  = (size_t)WP_NPX * 2u;                           // 261120
    const size_t SZ_ACC  = (size_t)WP_NPX * 3u;                           // 391680
    const size_t total   = SZ_SLUT + SZ_PREV + SZ_XP + SZ_STAR + SZ_IMG + SZ_ACC + 256 + 256;
    wp_mem = (uint8_t *)heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
    if (!wp_mem) {
        printf("WP: PSRAM alloc failed (%u bytes)\n", (unsigned)total);
        wp_alloc_failed = true;
        return false;
    }
    size_t off = 0;
    wp_slut      = (float *)(wp_mem + off);     off += SZ_SLUT;
    wp_prev_mag  = (float *)(wp_mem + off);     off += SZ_PREV;
    wp_xp        = (wp_xp_t *)(wp_mem + off);   off += SZ_XP;
    wp_stars     = (wp_star_t *)(wp_mem + off); off += SZ_STAR;
    wp_img       = (uint16_t *)(wp_mem + off);  off += SZ_IMG;
    wp_acc       = wp_mem + off;                off += SZ_ACC;
    wp_tone_lut  = wp_mem + off;                off += 256;
    wp_decay_lut = wp_mem + off;                off += 256;
    memset(wp_mem, 0, total);
    for (int i = 0; i < WP_SLUT_N; i++) wp_slut[i] = sinf((float)i * (6.2831853f / (float)WP_SLUT_N));
    wp_slut[WP_SLUT_N] = wp_slut[0];
    for (int i = 0; i < 256; i++) {
        wp_tone_lut[i]  = (uint8_t)(255.0f * (1.0f - expf(-(float)i / 64.0f)));
        wp_decay_lut[i] = (uint8_t)((float)i * WP_DECAY_MUL);   // truncation: 1 -> 0
    }
    for (int i = 0; i < NUM_SPECTRUM_BANDS; i++) wp_prev_mag[i] = DB_MIN;
    // --- seed starfield: 4-6 cluster centres, ~42% of stars gaussian-scattered
    // around them, the rest uniform; power-law brightness; warm/cool tints ---
    {
        float clx[7], cly[7], cls[7];
        int nc = 4 + (int)(wp_randf() * 3.0f);
        for (int i = 0; i < nc; i++) {
            clx[i] = wp_randf() * (float)WP_W;
            cly[i] = wp_randf() * (float)WP_H;
            cls[i] = 12.0f + wp_randf() * 18.0f;
        }
        for (int i = 0; i < WP_NSTARS; i++) {
            wp_star_t *s = &wp_stars[i];
            float x, y;
            if (wp_randf() < WP_CLUSTER_FRAC) {
                int c = (int)(wp_randf() * (float)nc);
                x = clx[c] + (wp_randf() + wp_randf() + wp_randf() - 1.5f) * 0.8f * cls[c];
                y = cly[c] + (wp_randf() + wp_randf() + wp_randf() - 1.5f) * 0.8f * cls[c];
            } else {
                x = wp_randf() * (float)WP_W;
                y = wp_randf() * (float)WP_H;
            }
            if (x < 0.0f) x = 0.0f;
            if (x > (float)(WP_W - 1)) x = (float)(WP_W - 1);
            if (y < 0.0f) y = 0.0f;
            if (y > (float)(WP_H - 1)) y = (float)(WP_H - 1);
            s->x = (uint16_t)x;
            s->y = (uint16_t)y;
            float raw = 10.0f + 72.0f * powf(wp_randf(), 2.6f);
            float dep = raw * WP_STAR_DEP;
            bool warm = (wp_randf() < WP_WARM_FRAC);
            s->dep_r = dep * (warm ? 1.10f : 0.95f);
            s->dep_g = dep * (warm ? 0.94f : 0.99f);
            s->dep_b = dep * (warm ? 0.78f : 1.10f);
            s->big   = (raw > WP_STAR_BIG_B) ? 1 : 0;
            s->glint = 0;
            s->tw_ph   = (uint16_t)(wp_randf() * 65535.0f);
            s->tw_rate = (uint16_t)(210.0f + wp_randf() * 520.0f);
        }
    }
    memset(&wp_img_dsc, 0, sizeof(wp_img_dsc));
    wp_img_dsc.header.magic  = LV_IMAGE_HEADER_MAGIC;  // if compile fails here on an older v9 minor, delete this line
    wp_img_dsc.header.cf     = LV_COLOR_FORMAT_RGB565;
    wp_img_dsc.header.w      = WP_W;
    wp_img_dsc.header.h      = WP_H;
    wp_img_dsc.header.stride = WP_W * 2;
    wp_img_dsc.data_size     = (uint32_t)WP_NPX * 2u;
    wp_img_dsc.data          = (const uint8_t *)wp_img;
    printf("WP: init OK, %u bytes PSRAM (single block)\n", (unsigned)total);
    return true;
}

// Recompute the 8 transform parameter sets from morph time + band drive
static void wp_update_params(const float *wshare) {
    for (int k = 0; k < WP_XFORMS; k++) {
        const float *P = wp_ph[k];
        float bmod = wshare[k] * (float)WP_BANDS;   // ~1.0 at flat spectrum
        if (bmod > 2.0f) bmod = 2.0f;
        wp_xp_t *x = &wp_xp[k];
        switch (wp_arch[k]) {
            case 0: {   // linear spiral <-> horseshoe
                float rot = 1.5f * sinf(wp_mt * 0.0021f + P[0])
                          + 0.4f * sinf(wp_mt * 0.013f + P[1]) + bmod * 0.3f;
                if (wp_variant[k]) rot = -rot;
                float s0 = 0.78f + 0.12f * sinf(wp_mt * 0.0034f + P[2]);
                x->a = cosf(rot) * s0;
                x->b = sinf(rot) * s0;
                x->c = 0.15f * sinf(wp_mt * 0.0043f + P[3]);
                if (wp_variant[k]) x->c = -x->c;
                x->m = 0.5f + 0.5f * sinf(wp_mt * 0.0017f + P[4]);
            } break;
            case 1: {   // sinusoidal <-> polar
                x->a = 2.0f + 0.9f * sinf(wp_mt * 0.0052f + P[0]);
                x->b = 2.4f + 0.9f * sinf(wp_mt * 0.0047f + P[1]);
                x->c = 0.85f + 0.10f * sinf(wp_mt * 0.008f + P[2]);
                x->d = wp_variant[k] ? 2.4f : 0.3f;    // sine phase anchors
                x->e = wp_variant[k] ? 1.1f : -0.2f;
                x->m = 0.5f + 0.5f * sinf(wp_mt * 0.0013f + P[3]);
            } break;
            case 2: {   // spherical <-> swirl
                x->a = 0.28f + 0.10f * sinf(wp_mt * 0.006f + P[0]);
                float sw = 1.2f * sinf(wp_mt * 0.0028f + P[1]) + bmod * 0.5f;
                x->b = wp_variant[k] ? -sw : sw;
                x->m = 0.5f + 0.5f * sinf(wp_mt * 0.0019f + P[2]);
            } break;
            default: {  // contracting linear anchor (structure seeds)
                x->a = 0.45f + 0.12f * sinf(wp_mt * 0.0061f + P[0]);
                x->c = (wp_variant[k] ? -0.30f : 0.34f) + 0.15f * sinf(wp_mt * 0.0038f + P[1]);
                x->d = (wp_variant[k] ? 0.22f : -0.26f) + 0.15f * sinf(wp_mt * 0.0044f + P[2]);
                x->m = 0.0f;
            } break;
        }
    }
}

static void wp_update(void) {
#if WP_PERF_SERIAL
    int64_t t0 = esp_timer_get_time();
#endif
    wp_frame++;

    // ---- band means (dB) over log-spaced groups ----
    float band_db[WP_BANDS];
    float mean_db = 0.0f;
    for (int b = 0; b < WP_BANDS; b++) {
        float s = 0.0f;
        for (int i = wp_edges[b]; i < wp_edges[b + 1]; i++) s += display_magnitudes[i];
        band_db[b] = s / (float)(wp_edges[b + 1] - wp_edges[b]);
        mean_db += band_db[b];
    }
    mean_db *= (1.0f / (float)WP_BANDS);

    // ---- AGC envelope + noise gate -> activity 0..1 ----
    if (mean_db > wp_env_db) wp_env_db += (mean_db - wp_env_db) * WP_AGC_ATTACK;
    else                     wp_env_db += (mean_db - wp_env_db) * WP_AGC_RELEASE;
    float activity = (wp_env_db - WP_GATE_DB) / WP_AGC_SPAN_DB;
    if (activity < 0.0f) activity = 0.0f;
    if (activity > 1.0f) activity = 1.0f;

    // ---- spectral flux, self-normalised (level-independent) ----
    float flux = 0.0f;
    for (int i = 0; i < NUM_SPECTRUM_BANDS; i++) {
        flux += fabsf(display_magnitudes[i] - wp_prev_mag[i]);
        wp_prev_mag[i] = display_magnitudes[i];
    }
    wp_flux_avg += (flux - wp_flux_avg) * WP_FLUX_AVG_COEF;
    float flux_norm = flux / (wp_flux_avg + 1.0f);
    float punch = WP_PUNCH_MIN + 0.3f * flux_norm;
    if (punch > WP_PUNCH_MAX) punch = WP_PUNCH_MAX;
    float morph_mul = 0.7f + 0.5f * (flux_norm > 2.0f ? 2.0f : flux_norm);
    wp_mt += morph_mul;
    wp_rot += WP_ROT_RATE * (0.5f + morph_mul);
    if (wp_rot > 6.2831853f) wp_rot -= 6.2831853f;
    const float wp_rc = cosf(wp_rot), wp_rs = sinf(wp_rot);

    // ---- transform weights from spectral shape ----
    float w[WP_XFORMS], wsum = 0.0f;
    for (int b = 0; b < WP_BANDS; b++) {
        float e = expf(WP_WEXP_K * (band_db[b] - mean_db)) + WP_W_FLOOR;
        w[b] = e;
        wsum += e;
    }
    float wshare[WP_XFORMS];
    for (int b = 0; b < WP_BANDS; b++) wshare[b] = w[b] / wsum;
    float cw[WP_XFORMS];
    float csum = 0.0f;
    for (int b = 0; b < WP_BANDS; b++) { csum += wshare[b]; cw[b] = csum; }
    cw[WP_XFORMS - 1] = 1.01f;   // guard: selection scan cannot run off the end

    // ---- rotating per-transform hues ----
    uint8_t tc[WP_XFORMS][3];
    for (int k = 0; k < WP_XFORMS; k++) {
        float mult = (wp_hue[k] > WP_HUE_COOL_LO && wp_hue[k] < WP_HUE_COOL_HI)
                     ? WP_HUE_COOL_MULT : 1.0f;
        wp_hue[k] += wp_hue_rate[k] * morph_mul * 2.0f * mult;
        if (wp_hue[k] >= 360.0f) wp_hue[k] -= 360.0f;
        wp_hsv(wp_hue[k], ((k == 1) || (k == 7)) ? 0.8f : 0.9f, 1.0f, tc[k]);
    }

    wp_update_params(wshare);

    // ---- chaos game ----
    int inten = (int)(WP_INTEN_BASE * punch * (0.6f + 0.4f * activity) + 0.5f);
    int npts = WP_PTS_SIMMER + (int)((float)(WP_PTS_MAX - WP_PTS_SIMMER) * activity);
    int rejects = 0;
    float px = wp_ptx, py = wp_pty;
    int cr = wp_pcr, cg = wp_pcg, cb = wp_pcb;
    for (int n = 0; n < npts; n++) {
        float r = wp_randf();
        int k = 0;
        while (r > cw[k]) k++;
        const wp_xp_t *x = &wp_xp[k];
        float nx, ny;
        switch (wp_arch[k]) {
            case 0: {
                float lx = x->a * px - x->b * py + x->c;
                float ly = x->b * px + x->a * py;
                float rr = sqrtf(px * px + py * py) + 0.02f;
                float hx = (px * px - py * py) / rr * 0.55f;
                float hy = 2.0f * px * py / rr * 0.55f;
                nx = lx + (hx - lx) * x->m;
                ny = ly + (hy - ly) * x->m;
            } break;
            case 1: {
                float ax = x->c * wp_sin(x->a * py + x->d);
                float ay = x->c * wp_sin(x->b * px + x->e);
                float m = x->m;
                if (m > 0.02f) {
                    float an = atan2f(py, px);
                    float rr = sqrtf(px * px + py * py);
                    float bx = an * (0.9f / 3.1415927f);
                    float by = (rr - 0.75f) * 0.9f;
                    nx = ax + (bx - ax) * m;
                    ny = ay + (by - ay) * m;
                } else {
                    nx = ax;
                    ny = ay;
                }
            } break;
            case 2: {
                float r2 = px * px + py * py + 0.005f;
                float ix = px / r2 * x->a;
                float iy = py / r2 * x->a;
                float an = x->b * sqrtf(r2);
                float ca = wp_cos(an), sa = wp_sin(an);
                float bx = ca * px - sa * py;
                float by = sa * px + ca * py;
                nx = ix + (bx - ix) * x->m;
                ny = iy + (by - iy) * x->m;
            } break;
            default:
                nx = x->a * px + x->c;
                ny = x->a * py + x->d;
                break;
        }
        px = nx;
        py = ny;
        if (!(px * px + py * py < WP_ESCAPE_R2)) {   // negated compare also catches NaN
            px = wp_randf() * 2.0f - 1.0f;
            py = wp_randf() * 2.0f - 1.0f;
            continue;
        }
        cr = (cr + tc[k][0]) >> 1;
        cg = (cg + tc[k][1]) >> 1;
        cb = (cb + tc[k][2]) >> 1;
        float qx = wp_rc * px - wp_rs * py;      // slow field rotation
        float qy = wp_rs * px + wp_rc * py;
        // Offset BEFORE the int cast: C truncation rounds toward zero, so
        // casting the signed product first collapsed (-1,0) and (0,+1) into
        // the same centre bin — 2x deposit density accumulating into thin
        // bright lines down both screen centres. In the positive domain
        // truncation == floor and every bin is one unit wide.
        float fx = qx * WP_SCALE_X * wp_zoom + (float)(WP_W / 2);
        float fy = qy * WP_SCALE_Y * wp_zoom + (float)(WP_H / 2);
        if (fx < 0.0f || fy < 0.0f) { rejects++; continue; }
        int sx = (int)fx;
        int sy = (int)fy;
        if (sx >= WP_W || sy >= WP_H) { rejects++; continue; }
        uint8_t *p = wp_acc + (((sy * WP_W) + sx) * 3);
        int v;
        v = p[0] + ((cr * inten) >> 8); p[0] = (v > 255) ? 255 : (uint8_t)v;
        v = p[1] + ((cg * inten) >> 8); p[1] = (v > 255) ? 255 : (uint8_t)v;
        v = p[2] + ((cb * inten) >> 8); p[2] = (v > 255) ? 255 : (uint8_t)v;
    }
    wp_ptx = px; wp_pty = py;
    wp_pcr = cr; wp_pcg = cg; wp_pcb = cb;

    // ---- zoom servo on the reject fraction ----
    wp_rej_ema += ((float)rejects / (float)npts - wp_rej_ema) * WP_REJ_EMA;
    if (wp_rej_ema < WP_REJ_LO)      wp_zoom *= WP_ZOOM_SLEW;
    else if (wp_rej_ema > WP_REJ_HI) wp_zoom /= WP_ZOOM_SLEW;
    if (wp_zoom < WP_ZOOM_MIN) wp_zoom = WP_ZOOM_MIN;
    if (wp_zoom > WP_ZOOM_MAX) wp_zoom = WP_ZOOM_MAX;

    // ---- starfield: twinkle + rare scintillation, stochastic rounding so
    // sub-unit deposits survive the uint8 accumulator ----
    for (int i = 0; i < WP_NSTARS; i++) {
        wp_star_t *s = &wp_stars[i];
        if (s->glint > 0) s->glint--;
        else if (wp_randf() < WP_GLINT_P) s->glint = 2 + (uint8_t)(wp_randf() * 3.0f);
        s->tw_ph = (uint16_t)(s->tw_ph + s->tw_rate);
        float tw = 0.86f + 0.14f * wp_slut[s->tw_ph >> 6];
        if (s->glint > 0) tw *= WP_GLINT_GAIN;
        float u = wp_randf();                     // shared stochastic-round dither
        uint8_t *p = wp_acc + ((((int)s->y * WP_W) + (int)s->x) * 3);
        int v;
        v = p[0] + (int)(s->dep_r * tw + u); p[0] = (v > 255) ? 255 : (uint8_t)v;
        v = p[1] + (int)(s->dep_g * tw + u); p[1] = (v > 255) ? 255 : (uint8_t)v;
        v = p[2] + (int)(s->dep_b * tw + u); p[2] = (v > 255) ? 255 : (uint8_t)v;
        if (s->big) {
            float sp = tw * 0.30f;
            if (s->x + 1 < WP_W) {
                uint8_t *q = p + 3;
                v = q[0] + (int)(s->dep_r * sp + u); q[0] = (v > 255) ? 255 : (uint8_t)v;
                v = q[1] + (int)(s->dep_g * sp + u); q[1] = (v > 255) ? 255 : (uint8_t)v;
                v = q[2] + (int)(s->dep_b * sp + u); q[2] = (v > 255) ? 255 : (uint8_t)v;
            }
            if (s->y + 1 < WP_H) {
                uint8_t *q = p + WP_W * 3;
                v = q[0] + (int)(s->dep_r * sp + u); q[0] = (v > 255) ? 255 : (uint8_t)v;
                v = q[1] + (int)(s->dep_g * sp + u); q[1] = (v > 255) ? 255 : (uint8_t)v;
                v = q[2] + (int)(s->dep_b * sp + u); q[2] = (v > 255) ? 255 : (uint8_t)v;
            }
        }
    }

    // ---- rolling row decay ----
    for (int r = 0; r < WP_DECAY_ROWS; r++) {
        uint8_t *p = wp_acc + ((size_t)wp_decay_row * WP_W * 3);
        for (int i = 0; i < WP_W * 3; i++) p[i] = wp_decay_lut[p[i]];
        wp_decay_row++;
        if (wp_decay_row >= WP_H) wp_decay_row = 0;
    }

#if WP_PERF_SERIAL
    int64_t t1 = esp_timer_get_time();
#endif

    // ---- tone-map RGB888 acc -> RGB565 blit buffer ----
    {
        const uint8_t *s = wp_acc;
        uint16_t *d = wp_img;
        for (int i = 0; i < WP_NPX; i++) {
            uint16_t rr = wp_tone_lut[s[0]];
            uint16_t gg = wp_tone_lut[s[1]];
            uint16_t bb = wp_tone_lut[s[2]];
            *d++ = (uint16_t)(((rr & 0xF8u) << 8) | ((gg & 0xFCu) << 3) | (bb >> 3));
            s += 3;
        }
    }

#if WP_PERF_SERIAL
    int64_t t2 = esp_timer_get_time();
    if ((wp_frame % WP_PERF_EVERY) == 0) {
        printf("WP sim=%dus tm=%dus pts=%d act=%.2f flux=%.2f zoom=%.2f rej=%.2f\n",
               (int)(t1 - t0), (int)(t2 - t1), npts, (double)activity, (double)flux_norm,
               (double)wp_zoom, (double)wp_rej_ema);
    }
#endif
}

static void spectrum_draw_cb(lv_event_t *e) {
    if (tileview_scrolling) return;
    lv_layer_t *layer = lv_event_get_layer(e);
    lv_obj_t *obj = (lv_obj_t *)lv_event_get_target(e);
    lv_area_t obj_coords;
    lv_obj_get_coords(obj, &obj_coords);
    const int sox = obj_coords.x1;
    const int soy = obj_coords.y1;

    if (current_visualizer_mode == 1) {
        // --- Mode 1: Spectrum ---
        const int ox = sox + GRAPH_X;
        const int oy = soy + GRAPH_Y;
        const int w = GRAPH_W, h = GRAPH_H;

        lv_draw_rect_dsc_t bar_dsc;
        lv_draw_rect_dsc_init(&bar_dsc);
        bar_dsc.bg_opa = LV_OPA_COVER;

        for(int i = 0; i < NUM_SPECTRUM_BANDS; i++) {
            float db = display_magnitudes[i];
            if(db < DB_MIN) db = DB_MIN;
            if(db > DB_MAX) db = DB_MAX;
            float norm_val = (db - DB_MIN) * DB_RANGE_INV;
            
            int y_top = oy + (int)((1.0f - norm_val) * h);
            int x_start = ox + band_x_lut[i];
            int x_end = (i < NUM_SPECTRUM_BANDS - 1) ? (ox + band_x_lut[i + 1]) : (ox + w);
            
            int bar_h = (oy + h) - y_top;
            if (bar_h <= 0) continue; 
            
            bar_dsc.bg_color = get_intensity_color(norm_val);
            lv_area_t bar_area = {
                (lv_coord_t)x_start, (lv_coord_t)y_top,
                (lv_coord_t)(x_end - 2), (lv_coord_t)(oy + h - 1)
            };
            lv_draw_rect(layer, &bar_dsc, &bar_area);
        }

        // Draw grid/frame
        lv_draw_rect_dsc_t border_dsc;
        lv_draw_rect_dsc_init(&border_dsc);
        border_dsc.bg_opa = LV_OPA_TRANSP;
        border_dsc.border_color = lv_color_hex(0x3A3A5A);
        border_dsc.border_width = 1;
        lv_area_t frame = {(lv_coord_t)ox, (lv_coord_t)oy, (lv_coord_t)(ox + w - 1), (lv_coord_t)(oy + h - 1)};
        lv_draw_rect(layer, &border_dsc, &frame);

        lv_draw_label_dsc_t label_dsc;
        lv_draw_label_dsc_init(&label_dsc);
        label_dsc.color = lv_color_hex(0x888888);
        extern const lv_font_t lv_font_montserrat_10;
        label_dsc.font = &lv_font_montserrat_10;

        lv_draw_line_dsc_t grid_dsc;
        lv_draw_line_dsc_init(&grid_dsc);
        grid_dsc.color = lv_color_hex(0x2A2A4A);
        grid_dsc.width = 1;

        static const char* db_labels[] = {"0", "-20", "-40", "-60", "-90"};
        for(int i = 0; i < NUM_DB_MARKS; i++) {
            int y = oy + grid_y_lut[i];
            grid_dsc.p1.x = ox; grid_dsc.p1.y = y;
            grid_dsc.p2.x = ox + w; grid_dsc.p2.y = y;
            lv_draw_line(layer, &grid_dsc);
            
            label_dsc.text = db_labels[i];
            label_dsc.align = LV_TEXT_ALIGN_RIGHT;
            lv_area_t la = {(lv_coord_t)(ox - 35), (lv_coord_t)(y - 6), (lv_coord_t)(ox - 5), (lv_coord_t)(y + 8)};
            lv_draw_label(layer, &label_dsc, &la);
        }

        for(int i = 0; i < NUM_FREQ_MARKS; i++) {
            int x = ox + grid_x_lut[i];
            grid_dsc.p1.x = x; grid_dsc.p1.y = oy;
            grid_dsc.p2.x = x; grid_dsc.p2.y = oy + h;
            lv_draw_line(layer, &grid_dsc);
            
            label_dsc.text = freq_labels[i];
            label_dsc.align = LV_TEXT_ALIGN_CENTER;
            lv_area_t la = {(lv_coord_t)(x - 20), (lv_coord_t)(oy + h + 4), (lv_coord_t)(x + 20), (lv_coord_t)(oy + h + 18)};
            lv_draw_label(layer, &label_dsc, &la);
        }

    } else if (current_visualizer_mode == 2) {
        // --- Mode 2: Neon Octaves ---
        int num_blocks = 17;
        int block_w = GRAPH_W / num_blocks;
        const int ox = sox + GRAPH_X;
        const int oy = soy + GRAPH_Y;

        lv_draw_rect_dsc_t block_dsc;
        lv_draw_rect_dsc_init(&block_dsc);
        block_dsc.bg_opa = LV_OPA_COVER;
        block_dsc.radius = 3; 

        lv_draw_rect_dsc_t peak_dsc;
        lv_draw_rect_dsc_init(&peak_dsc);
        peak_dsc.bg_opa = LV_OPA_COVER;
        peak_dsc.radius = 2;

        for (int b = 0; b < num_blocks; b++) {
            int start_idx = b * 6;
            int end_idx = start_idx + 5;
            if (end_idx > NUM_SPECTRUM_BANDS - 1) end_idx = NUM_SPECTRUM_BANDS - 1;
            
            float max_val = DB_MIN;
            float p_val = DB_MIN;
            for (int k = start_idx; k <= end_idx; k++) {
                if (display_magnitudes[k] > max_val) max_val = display_magnitudes[k];
                if (peak_magnitudes[k] > p_val) p_val = peak_magnitudes[k];
            }
            
            float n_val = (max_val - DB_MIN) * DB_RANGE_INV;
            float n_peak = (p_val - DB_MIN) * DB_RANGE_INV;
            
            int bar_h = (int)(n_val * GRAPH_H);
            if (bar_h < 4) bar_h = 4;
            
            int x_pos = ox + b * block_w + 2;
            int bar_w = block_w - 4; 
            
            lv_color_t rcol = get_rainbow_color((float)b / (num_blocks - 1));
            block_dsc.bg_color = rcol;
            
            lv_area_t bar_area = {
                (lv_coord_t)x_pos, (lv_coord_t)(oy + GRAPH_H - bar_h),
                (lv_coord_t)(x_pos + bar_w), (lv_coord_t)(oy + GRAPH_H)
            };
            lv_draw_rect(layer, &block_dsc, &bar_area);

            // Peak mark
            int peak_h = (int)(n_peak * GRAPH_H);
            int peak_y = oy + GRAPH_H - peak_h - 4; 
            lv_area_t peak_area = {
                (lv_coord_t)x_pos, (lv_coord_t)peak_y,
                (lv_coord_t)(x_pos + bar_w), (lv_coord_t)(peak_y + 4)
            };
            peak_dsc.bg_color = lv_color_mix(lv_color_hex(0xFFFFFF), rcol, 100);
            lv_draw_rect(layer, &peak_dsc, &peak_area);
        }

    } else if (current_visualizer_mode == 3) {
        // --- Mode 3: VU Meter ---
        const int cx = sox + VU_PIVOT_X;
        const int cy = soy + VU_PIVOT_Y;
        vu_precompute();

        auto db_to_norm = [](float db) -> float {
            if (db < VU_DB_MIN) db = VU_DB_MIN;
            float v = powf(10.0f, db / 20.0f);
            float t = (v - vu_vmin) * vu_range_inv;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            return t;
        };

        auto norm_to_angle_rad = [](float t) -> float {
            float deg = VU_ARC_START_DEG + t * (VU_ARC_END_DEG - VU_ARC_START_DEG);
            return deg * (float)M_PI / 180.0f;
        };

        lv_draw_line_dsc_t ln_dsc;
        lv_draw_line_dsc_init(&ln_dsc);
        ln_dsc.opa = LV_OPA_COVER;

        // Draw dual scale arcs
        for (int i = 1; i <= VU_ARC_SEGMENTS; i++) {
            float t = (float)i / (float)VU_ARC_SEGMENTS;
            bool in_red = (t >= vu_red_start_t);
            uint32_t col = in_red ? VU_COL_RED : VU_COL_SCALE;
            
            ln_dsc.width = 2;
            ln_dsc.color = lv_color_hex(col);
            
            ln_dsc.p1.x = cx + vu_arc_ox[i-1]; ln_dsc.p1.y = cy + vu_arc_oy[i-1];
            ln_dsc.p2.x = cx + vu_arc_ox[i];   ln_dsc.p2.y = cy + vu_arc_oy[i];
            lv_draw_line(layer, &ln_dsc);
            
            ln_dsc.p1.x = cx + vu_arc_ix[i-1]; ln_dsc.p1.y = cy + vu_arc_iy[i-1];
            ln_dsc.p2.x = cx + vu_arc_ix[i];   ln_dsc.p2.y = cy + vu_arc_iy[i];
            lv_draw_line(layer, &ln_dsc);

            if (in_red) {
                ln_dsc.width = VU_SCALE_R_OUTER - VU_SCALE_R_INNER;
                ln_dsc.p1.x = cx + vu_arc_mx[i-1]; ln_dsc.p1.y = cy + vu_arc_my[i-1];
                ln_dsc.p2.x = cx + vu_arc_mx[i];   ln_dsc.p2.y = cy + vu_arc_my[i];
                lv_draw_line(layer, &ln_dsc);
            }
        }

        // Ticks
        static const float minor_ticks[] = {-30.0f, -15.0f, -8.0f, -6.0f, -4.0f, -2.0f, -1.0f};
        for (int i = 0; i < 7; i++) {
            bool in_red = (minor_ticks[i] >= VU_RED_ZONE_DB);
            ln_dsc.width = 1;
            ln_dsc.color = lv_color_hex(in_red ? VU_COL_RED : VU_COL_SCALE);
            ln_dsc.p1.x = cx + vu_min_in_x[i];  ln_dsc.p1.y = cy + vu_min_in_y[i];
            ln_dsc.p2.x = cx + vu_min_out_x[i]; ln_dsc.p2.y = cy + vu_min_out_y[i];
            lv_draw_line(layer, &ln_dsc);
        }

        lv_draw_label_dsc_t lbl_dsc;
        lv_draw_label_dsc_init(&lbl_dsc);
        extern const lv_font_t lv_font_montserrat_10;
        lbl_dsc.font = &lv_font_montserrat_10;
        lbl_dsc.align = LV_TEXT_ALIGN_CENTER;

        static const float major_ticks[]  = {-40.0f, -20.0f, -10.0f, -7.0f, -5.0f, -3.0f, 0.0f, 1.0f, 2.0f, 3.0f};
        static const char* major_labels[] = {"-40","-20","-10","-7","-5","-3","0","+1","+2","+3"};
        for (int i = 0; i < 10; i++) {
            bool in_red = (major_ticks[i] >= VU_RED_ZONE_DB);
            uint32_t col = in_red ? VU_COL_RED : VU_COL_SCALE;
            
            ln_dsc.width = 2;
            ln_dsc.color = lv_color_hex(col);
            ln_dsc.p1.x = cx + vu_maj_in_x[i];  ln_dsc.p1.y = cy + vu_maj_in_y[i];
            ln_dsc.p2.x = cx + vu_maj_out_x[i]; ln_dsc.p2.y = cy + vu_maj_out_y[i];
            lv_draw_line(layer, &ln_dsc);
            
            lbl_dsc.color = lv_color_hex(col);
            lbl_dsc.text = major_labels[i];
            lv_area_t la = {
                (lv_coord_t)(cx + vu_maj_lbl_x[i] - 20), (lv_coord_t)(cy + vu_maj_lbl_y[i] - 6),
                (lv_coord_t)(cx + vu_maj_lbl_x[i] + 20), (lv_coord_t)(cy + vu_maj_lbl_y[i] + 8)
            };
            lv_draw_label(layer, &lbl_dsc, &la);
        }

        // Draw needles
        auto draw_needle = [&](float db, uint32_t col) {
            float rad = norm_to_angle_rad(db_to_norm(db));
            float sr = sinf(rad), cr = cosf(rad);
            ln_dsc.width = 2;
            ln_dsc.color = lv_color_hex(col);
            ln_dsc.p1.x = cx + 20 * sr;
            ln_dsc.p1.y = cy - 20 * cr;
            ln_dsc.p2.x = cx + VU_NEEDLE_LEN * sr;
            ln_dsc.p2.y = cy - VU_NEEDLE_LEN * cr;
            lv_draw_line(layer, &ln_dsc);
        };
        draw_needle(r_db_display, VU_COL_R);
        draw_needle(l_db_display, VU_COL_L);

        // Peak LEDs
        uint32_t now = lv_tick_get();
        auto draw_led = [&](int lcx, int lcy, uint32_t lcol, const char* txt, bool lit, bool left) {
            lv_draw_rect_dsc_t led_dsc;
            lv_draw_rect_dsc_init(&led_dsc);
            led_dsc.radius = 7;
            if (lit) {
                led_dsc.bg_color = lv_color_hex(VU_COL_LED_ON);
                led_dsc.bg_opa = LV_OPA_COVER;
            } else {
                led_dsc.bg_opa = LV_OPA_TRANSP;
                led_dsc.border_color = lv_color_hex(VU_COL_LED_OFF);
                led_dsc.border_width = 2;
            }
            lv_area_t la = {(lv_coord_t)(lcx - 7), (lv_coord_t)(lcy - 7), (lv_coord_t)(lcx + 7), (lv_coord_t)(lcy + 7)};
            lv_draw_rect(layer, &led_dsc, &la);
            
            extern const lv_font_t lv_font_montserrat_12;
            lbl_dsc.font = &lv_font_montserrat_12;
            lbl_dsc.color = lv_color_hex(lcol);
            lbl_dsc.text = txt;
            lbl_dsc.align = left ? LV_TEXT_ALIGN_RIGHT : LV_TEXT_ALIGN_LEFT;
            int lx = left ? (lcx - 22) : (lcx + 22);
            lv_area_t lal = {(lv_coord_t)(lx - 10), (lv_coord_t)(lcy - 7), (lv_coord_t)(lx + 10), (lv_coord_t)(lcy + 9)};
            lv_draw_label(layer, &lbl_dsc, &lal);
        };
        draw_led(sox + 32, soy + SCREEN_H - 22, VU_COL_L, "L", (now < vu_peak_led_l_until_ms), false);
        draw_led(sox + SCREEN_W - 32, soy + SCREEN_H - 22, VU_COL_R, "R", (now < vu_peak_led_r_until_ms), true);

    } else if (current_visualizer_mode == 4) {
        // --- Mode 4: Starfield ---
        if (!sf_initialized) sf_init();
        const int cx = sox + SCREEN_W / 2;
        const int cy = soy + SCREEN_H / 2;

        lv_draw_rect_dsc_t star_dsc;
        lv_draw_rect_dsc_init(&star_dsc);
        star_dsc.bg_opa = LV_OPA_COVER;

        for (int i = 0; i < sf_active_count; i++) {
            float z = sf_stars[i].z;
            if (z < SF_Z_MIN) continue;

            int sx = cx + (int)(sf_stars[i].x / z * SF_PROJ);
            int sy = cy + (int)(sf_stars[i].y / z * SF_PROJ);

            if (sx < sox || sx > sox + SCREEN_W - 1 || sy < soy || sy > soy + SCREEN_H - 1) continue;

            int sz = (int)(12.0f / z);
            if (sz < 1) sz = 1;
            if (sz > 8) sz = 8;

            if (i < SF_FFT_STARS && sf_band_hot[i]) {
                star_dsc.bg_color = lv_color_hex(sf_fft_color[i]);
            } else {
                star_dsc.bg_color = lv_color_hex(SF_COL_STAR);
            }

            star_dsc.radius = (sz > 2) ? (sz / 2) : 0;
            lv_area_t sa = {
                (lv_coord_t)(sx - sz/2), (lv_coord_t)(sy - sz/2),
                (lv_coord_t)(sx + sz/2), (lv_coord_t)(sy + sz/2)
            };
            lv_draw_rect(layer, &star_dsc, &sa);
        }

    } else if (current_visualizer_mode == 5) {
        // --- Mode 5: Warp Field ---
        if (!wf_initialized) wf_init();
        
        lv_draw_rect_dsc_t bg_dsc;
        lv_draw_rect_dsc_init(&bg_dsc);
        bg_dsc.bg_color = lv_color_hex(0x000000);
        bg_dsc.bg_opa = LV_OPA_COVER;
        lv_area_t screen_area_rect = {(lv_coord_t)sox, (lv_coord_t)soy, (lv_coord_t)(sox + SCREEN_W - 1), (lv_coord_t)(soy + SCREEN_H - 1)};
        lv_draw_rect(layer, &bg_dsc, &screen_area_rect);

        lv_draw_rect_dsc_t star_dsc;
        lv_draw_rect_dsc_init(&star_dsc);
        star_dsc.bg_opa = LV_OPA_COVER;
        star_dsc.bg_color = lv_color_hex(WF_COL_STAR);

        lv_draw_line_dsc_t tail_dsc;
        lv_draw_line_dsc_init(&tail_dsc);
        tail_dsc.color = lv_color_hex(WF_COL_TAIL);
        tail_dsc.round_start = 1;
        tail_dsc.round_end = 0;

        lv_draw_rect_dsc_t part_dsc;
        for (int i = 0; i < WF_BASE_STARS; i++) {
            float z = wf_stars[i].z;
            if (z < WF_Z_MIN) continue;

            float inv_z = 1.0f / z;
            int sx = SCREEN_W / 2 + (int)(wf_stars[i].x * inv_z * WF_PROJ);
            int sy = SCREEN_H / 2 + (int)(wf_stars[i].y * inv_z * WF_PROJ);
            
            int screen_x = sox + sx;
            int screen_y = soy + sy;

            float tail_len = (wf_stars[i].speed * (WF_PROJ * inv_z)) * 0.54f;
            if (tail_len > WF_TAIL_MAX_LEN) tail_len = WF_TAIL_MAX_LEN;
            
            float tail_norm = 1.0f - (z / WF_Z_MAX); 
            
            int sz = (int)(inv_z * 8.0f);
            if (sz < 1) sz = 1;
            if (sz > 6) sz = 6;

            if (tail_len > 1.5f) {
                float dx = wf_stars[i].nx;
                float dy = wf_stars[i].ny;
                
                float start_off = (float)sz * 0.5f;
                tail_dsc.p1.x = (lv_coord_t)(screen_x - dx * start_off);
                tail_dsc.p1.y = (lv_coord_t)(screen_y - dy * start_off);
                
                tail_dsc.opa = (lv_opa_t)(25 + 70 * tail_norm);
                tail_dsc.color = (wf_stars[i].speed > WF_SPEED_MIN + 0.1f) ? lv_color_hex(WF_COL_TAIL_WARP) : lv_color_hex(WF_COL_TAIL);
                tail_dsc.width = 1;

                tail_dsc.p2.x = (lv_coord_t)(screen_x - dx * (start_off + tail_len * 0.35f));
                tail_dsc.p2.y = (lv_coord_t)(screen_y - dy * (start_off + tail_len * 0.35f));
                lv_draw_line(layer, &tail_dsc);
                
                tail_dsc.p1 = tail_dsc.p2;
                tail_dsc.opa = (lv_opa_t)(tail_dsc.opa * 0.5f);
                tail_dsc.p2.x = (lv_coord_t)(screen_x - dx * (start_off + tail_len * 0.70f));
                tail_dsc.p2.y = (lv_coord_t)(screen_y - dy * (start_off + tail_len * 0.70f));
                lv_draw_line(layer, &tail_dsc);

                if (wf_stars[i].speed > WF_SPEED_MIN + 0.1f || z < 30.0f) {
                    tail_dsc.p1 = tail_dsc.p2;
                    tail_dsc.opa = (lv_opa_t)(tail_dsc.opa * 0.4f);
                    tail_dsc.p2.x = (lv_coord_t)(screen_x - dx * (start_off + tail_len));
                    tail_dsc.p2.y = (lv_coord_t)(screen_y - dy * (start_off + tail_len));
                    lv_draw_line(layer, &tail_dsc);
                }
            }

            if (sx >= 0 && sx < SCREEN_W && sy >= 0 && sy < SCREEN_H) {
                star_dsc.bg_opa = (lv_opa_t)(120 + 135 * (1.0f - z/WF_Z_MAX));
                lv_area_t sa = {
                    (lv_coord_t)(screen_x - sz/2), (lv_coord_t)(screen_y - sz/2),
                    (lv_coord_t)(screen_x + sz/2), (lv_coord_t)(screen_y + sz/2)
                };
                lv_draw_rect(layer, &star_dsc, &sa);
            }
        }
    } else if (current_visualizer_mode == 6) {
        // --- Mode 6: Whirlpool ---
        lv_area_t flame_area = {(lv_coord_t)sox, (lv_coord_t)soy,
                                (lv_coord_t)(sox + SCREEN_W - 1), (lv_coord_t)(soy + SCREEN_H - 1)};
        if (wp_img && !wp_alloc_failed) {
            lv_draw_image_dsc_t img_dsc;
            lv_draw_image_dsc_init(&img_dsc);
            img_dsc.src = &wp_img_dsc;
            lv_draw_image(layer, &img_dsc, &flame_area);
        } else {
            // Not yet initialised (or PSRAM alloc failed): black screen
            lv_draw_rect_dsc_t bg_dsc;
            lv_draw_rect_dsc_init(&bg_dsc);
            bg_dsc.bg_color = lv_color_hex(0x000000);
            bg_dsc.bg_opa = LV_OPA_COVER;
            lv_draw_rect(layer, &bg_dsc, &flame_area);
        }
    }
}

static void spectrum_timer_cb(lv_timer_t *timer) {
    (void)timer;
    const float attack_coeff = 0.85f;  
    const float decay_coeff = 0.75f;  
    bool changed = false;
    for(int i = 0; i < NUM_SPECTRUM_BANDS; i++) {
        float target = shared_spectrum_magnitudes[i];
        if (target > display_magnitudes[i]) display_magnitudes[i] += (target - display_magnitudes[i]) * attack_coeff;
        else display_magnitudes[i] += (target - display_magnitudes[i]) * decay_coeff;
        
        if (display_magnitudes[i] > peak_magnitudes[i]) {
            peak_magnitudes[i] = display_magnitudes[i];
        } else {
            peak_magnitudes[i] -= 0.6f; // slow decay for peak
        }
        
        if(fabsf(display_magnitudes[i] - target) > 0.1f) changed = true;
        if(peak_magnitudes[i] > DB_MIN && peak_magnitudes[i] < DB_MAX) changed = true;
    }
    
    // VU meter state update (mode 3)
    if (current_visualizer_mode == 3) {
        float l_target = dsp_get_meter_peak(1, 0);
        float r_target = dsp_get_meter_peak(1, 1);
        
        if (fabsf(l_target - l_db_display) > 0.1f) {
            l_db_display = l_target;
            changed = true;
        }
        if (fabsf(r_target - r_db_display) > 0.1f) {
            r_db_display = r_target;
            changed = true;
        }
        
        uint32_t now = lv_tick_get();
        if (l_target >= VU_PEAK_LED_DB) {
            vu_peak_led_l_until_ms = now + VU_PEAK_HOLD_MS;
            changed = true;
        }
        if (r_target >= VU_PEAK_LED_DB) {
            vu_peak_led_r_until_ms = now + VU_PEAK_HOLD_MS;
            changed = true;
        }
    }
    
    // Starfield update (mode 4)
    if (current_visualizer_mode == 4) {
        if (!sf_initialized) sf_init();
        
        float avg_db = 0.0f;
        for (int i = 0; i < NUM_SPECTRUM_BANDS; i++) avg_db += display_magnitudes[i];
        avg_db /= (float)NUM_SPECTRUM_BANDS;
        float energy = (avg_db - DB_MIN) / (DB_MAX - DB_MIN);
        if (energy < 0.0f) energy = 0.0f;
        if (energy > 1.0f) energy = 1.0f;
        
        sf_global_speed = SF_SPEED_MIN + energy * (SF_SPEED_MAX - SF_SPEED_MIN);
        sf_active_count = SF_BASE_STARS + (int)((float)(SF_MAX_STARS - SF_BASE_STARS) * energy);
        
        for (int i = 0; i < SF_FFT_STARS; i++) {
            int band = sf_fft_band[i];
            float peak = -200.0f;
            for (int b = band - 2; b <= band + 2; b++) {
                if (b >= 0 && b < NUM_SPECTRUM_BANDS) {
                    if (display_magnitudes[b] > peak) peak = display_magnitudes[b];
                }
            }
            sf_band_hot[i] = (peak >= SF_THRESHOLD);
        }
        
        for (int i = 0; i < sf_active_count; i++) {
            float speed = sf_global_speed;
            if (i < SF_FFT_STARS && sf_band_hot[i]) {
                speed *= SF_BURST_MULT;
            }
            sf_stars[i].z -= speed;
            if (sf_stars[i].z < SF_Z_MIN) {
                sf_respawn(i);
            }
        }
        changed = true;
    }

    // Warp Field (mode 5)
    if (current_visualizer_mode == 5) {
        if (!wf_initialized) wf_init();

        float avg_db = 0.0f;
        for (int i = 0; i < NUM_SPECTRUM_BANDS; i++) avg_db += display_magnitudes[i];
        avg_db /= (float)NUM_SPECTRUM_BANDS;
        float energy = (avg_db - DB_MIN) / (DB_MAX - DB_MIN);
        if (energy < 0.0f) energy = 0.0f;
        if (energy > 1.0f) energy = 1.0f;
        
        static float last_energy = 0.0f;
        bool beat_hit = (energy - last_energy > 0.12f);
        if (beat_hit) {
            wf_active_group = (wf_active_group + 1) % 5;
        }
        last_energy = energy;

        for (int i = 0; i < WF_BASE_STARS; i++) {
            // Trigger 20% of stars instead of 25% (i % 5 instead of i % 4)
            if (beat_hit && (i % 5 == wf_active_group)) {
                wf_stars[i].speed = WF_SPEED_MIN + energy * (WF_SPEED_MAX - WF_SPEED_MIN);
            }
            
            wf_stars[i].z -= wf_stars[i].speed;
            if (wf_stars[i].z < WF_Z_MIN) {
                wf_respawn(i);
            }
        }
        changed = true;
    }
    
    // Whirlpool state update (mode 6). Unlike the other modes this one
    // moves ~650 KB/frame through PSRAM, so it only runs while the spectrum
    // tile is actually visible — the persistent buffer freezes off-tile and
    // resumes seamlessly.
    if (current_visualizer_mode == 6 && !tileview_scrolling && spectrum_parent_tile != NULL) {
        lv_obj_t *ftv = lv_obj_get_parent(spectrum_parent_tile);
        if (ftv && lv_tileview_get_tile_active(ftv) == spectrum_parent_tile) {
            if (wp_ensure_init()) wp_update();
            changed = true;   // continuous animation, same as warp field
        }
    }
    
    if (changed && !tileview_scrolling && spectrum_area != NULL && spectrum_parent_tile != NULL) {
        if ((lv_tick_get() - last_scroll_end_time) < SCROLL_SETTLE_MS) return;
        lv_obj_t *tv = lv_obj_get_parent(spectrum_parent_tile);
        if (tv && lv_tileview_get_tile_active(tv) == spectrum_parent_tile) lv_obj_invalidate(spectrum_area);
    }
}

void spectrum_timer_set_paused(bool paused) {
    if (!spectrum_timer_handle) return;
    if (paused) lv_timer_pause(spectrum_timer_handle);
    else lv_timer_resume(spectrum_timer_handle);
}

void spectrum_set_hidden(bool hidden) {
    if (!spectrum_area) return;
    if (hidden) lv_obj_add_flag(spectrum_area, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(spectrum_area, LV_OBJ_FLAG_HIDDEN);
}

void spectrum_ui_create(lv_obj_t *parent) {
    spectrum_parent_tile = parent;
    for (int i = 0; i < NUM_SPECTRUM_BANDS; i++) iso_freqs[i] = 20.0f * powf(2.0f, (float)i * (10.0f / (float)(NUM_SPECTRUM_BANDS - 1)));
    log_20 = logf(20.0f);
    log_range_inv = 1.0f / (logf(20000.0f) - log_20);
    for(int i = 0; i < NUM_SPECTRUM_BANDS; i++) band_x_lut[i] = (int)(freq_to_norm_log(iso_freqs[i]) * GRAPH_W);
    for(int i = 0; i < 10; i++) grid_x_lut[i] = (int)(freq_to_norm_log((float)freq_marks[i]) * GRAPH_W);
    for(int i = 0; i < 5; i++) {
        float norm_val = (db_marks[i] - DB_MIN) * DB_RANGE_INV;
        grid_y_lut[i] = (int)((1.0f - norm_val) * GRAPH_H);
    }
    for(int i = 0; i < NUM_SPECTRUM_BANDS; i++) {
        display_magnitudes[i] = shared_spectrum_magnitudes[i] = peak_magnitudes[i] = DB_MIN;
    }
    spectrum_area = lv_obj_create(parent);
    lv_obj_remove_style_all(spectrum_area);
    lv_obj_set_size(spectrum_area, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(spectrum_area, 0, 0); 
    lv_obj_set_style_bg_opa(spectrum_area, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(spectrum_area, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(spectrum_area, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(spectrum_area, LV_OBJ_FLAG_GESTURE_BUBBLE); 
    lv_obj_add_event_cb(spectrum_area, spectrum_draw_cb, LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(spectrum_area, spectrum_gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_add_event_cb(spectrum_area, spectrum_press_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(spectrum_area, spectrum_tap_cb, LV_EVENT_CLICKED, NULL);
    spectrum_timer_handle = lv_timer_create(spectrum_timer_cb, 50, NULL);  
}
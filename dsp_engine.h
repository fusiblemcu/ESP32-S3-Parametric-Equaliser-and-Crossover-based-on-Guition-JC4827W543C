#ifndef DSP_ENGINE_H
#define DSP_ENGINE_H

#include "eq_data.h"
#include "spectrum_analyzer.h"
#include <Arduino.h>

typedef enum {
  DSP_INPUT_I2S = 0,
  DSP_INPUT_NOISE,
  DSP_INPUT_SWEEP_30_20K_30S, // 30 Hz → 20 kHz, 30 second loop
  DSP_INPUT_SWEEP_20_20K_35S, // 20 Hz → 20 kHz, 35 second loop
  DSP_INPUT_WARBLE_30_20K_30S, // 30 Hz → 20 kHz carrier, ±5% FM @ 5 Hz, 30s loop
  DSP_INPUT_TONE               // Static adjustable-frequency sine (UI-set freq/level)
} dsp_input_source_t;

typedef struct {
  float b0, b1, b2, a1, a2;
  // SVF coefficients (used if is_svf is true)
  float g, k, m0, m1, m2;
  bool is_svf;
} biquad_coeffs_t;

typedef struct {
  float s1, s2; // TDF2 states (or SVF v1, v2)
} biquad_state_t;

typedef struct {
  float threshold_lin;
  float attack_coeff;
  float release_coeff;
  float envelope;
  bool enabled;
} limiter_state_t;

// Stereo level meter state
typedef struct {
  float peak_l; // Current peak level (linear, 0.0–1.0+)
  float peak_r;
  float peak_hold_l; // Peak hold level (linear)
  float peak_hold_r;
  uint32_t hold_timer_l; // Samples remaining in hold period
  uint32_t hold_timer_r;
  uint32_t clip_timer_l; // Samples remaining in clip latch
  uint32_t clip_timer_r;
  bool clipping_l; // Currently in red/clip state
  bool clipping_r;
  bool active; // false = skip all metering work
} meter_state_t;

// Single EQ stage with independent L/R coefficient chains.
// Interpolation works in parameter space (Rule 3): UI pushes eq_band_t params
// into pending_params, sync_coeffs moves them to target_params, and
// interpolate_coeffs lerps cur_params toward target_params then calls
// calculate_biquad to produce live coeffs — never lerping coefficients directly.
typedef struct {
  biquad_coeffs_t coeffs_l[MAX_BANDS];   // Live biquad coefficients (audio path)
  biquad_coeffs_t coeffs_r[MAX_BANDS];

  eq_band_t cur_params_l[MAX_BANDS];     // Currently interpolated parameters (L)
  eq_band_t cur_params_r[MAX_BANDS];     // Currently interpolated parameters (R)
  eq_band_t target_params_l[MAX_BANDS];  // Interpolation targets (set by sync)
  eq_band_t target_params_r[MAX_BANDS];
  eq_band_t pending_params_l[MAX_BANDS]; // Pending from UI thread (written under lock)
  eq_band_t pending_params_r[MAX_BANDS];

  volatile bool dirty_l[MAX_BANDS];
  volatile bool dirty_r[MAX_BANDS];
  biquad_state_t state_l[MAX_BANDS];
  biquad_state_t state_r[MAX_BANDS];
  
  // Topology crossfade tracking (SVF <-> TDF2)
  biquad_state_t xf_state_l[MAX_BANDS];    // target filter state L
  biquad_state_t xf_state_r[MAX_BANDS];    // target filter state R
  biquad_coeffs_t xf_coeffs_l[MAX_BANDS];   // target filter coeffs L
  biquad_coeffs_t xf_coeffs_r[MAX_BANDS];   // target filter coeffs R
  
  int xf_warmup_samples_l[MAX_BANDS];       // remaining silent warmup samples L
  int xf_warmup_samples_r[MAX_BANDS];       // remaining silent warmup samples R
  int xf_topo_samples_l[MAX_BANDS];         // remaining audible crossfade samples L
  int xf_topo_samples_r[MAX_BANDS];         // remaining audible crossfade samples R
  bool xf_topo_to_svf_l[MAX_BANDS];          // true = fading toward SVF L
  bool xf_topo_to_svf_r[MAX_BANDS];          // true = fading toward SVF R

  volatile int num_active_bands_l;
  volatile int num_active_bands_r;
  int sorted_idx_l[MAX_BANDS];
  int sorted_idx_r[MAX_BANDS];
  volatile int crossfade_samples; // Samples remaining in crossfade (0 = idle)
  bool active;
} dsp_stage_t;

void dsp_init(void);
void limiter_init(float sample_rate, float threshold_db, float attack_ms,
                  float release_ms);

// channel: 0=L, 1=R
void dsp_update_band(int stage_idx, int channel, int band_idx,
                     eq_band_t *band_data);
void dsp_set_active_bands(int stage_idx, int channel, int count);

void dsp_set_input_source(dsp_input_source_t source);
void dsp_update_noise_gen(bool enabled, float level_db);
// Static tone: frequency in Hz, level in dBFS. Independent of noise level.
void dsp_set_tone_freq(float hz);
void dsp_set_tone_level(float level_db);

void dsp_set_sample_rate(float fs);
// Force-snap pending coefficients to live, zero all biquad internal state, and
// cancel any in-flight crossfade across all three EQ stages and both crossover
// filters. Also collapses gain smoothers and zeros limiter envelopes. Call
// this after a sample-rate change when the coefficient deltas are too large
// to traverse via smooth crossfade without visiting unstable intermediate
// states. Must be called with dsp_suspended = true.
void dsp_snap_and_reset_all(void);
void dsp_update_limiter(bool enabled, float threshold_db);
bool dsp_is_running(void);

void dsp_set_mute(int stage_idx, bool muted); // stage_idx: 0=input, 1=output, 2=low
bool dsp_get_mute(int stage_idx);
extern volatile bool dsp_stage_muted[3];

void dsp_set_fft_enabled(bool enabled);
bool dsp_is_fft_enabled(void);

// Input/Output gain control (channel: 0=L, 1=R)
void dsp_set_input_gain(int channel, float db);
void dsp_set_output_gain(int channel, float db);
float dsp_get_input_gain(int channel);
float dsp_get_output_gain(int channel);

// Level meter API
// stage_idx: 0 = input, 1 = output, 2 = low
// channel: 0 = L, 1 = R
void dsp_set_meter_active(int stage_idx, bool active);

// Scope ring buffer — 512 post-EQ mono samples (~10ms at 48kHz), filled every
// block independently of the FFT accumulator. Allows the scope display to
// refresh with fresh data every timer tick regardless of FFT swap cadence.
// Power-of-2 size so wrap uses bitwise AND.
#define SCOPE_BUF_SIZE 1024
extern volatile float scope_ring_buf[SCOPE_BUF_SIZE];
extern volatile uint32_t scope_ring_idx; // monotonic write cursor (unsigned: wrap is defined)

// Copy the most recent len samples (time-ordered, oldest first) into dst.
// Safe to call from any task — no locking needed for a free-running display.
void dsp_scope_get(float *dst, int len);
float dsp_get_meter_peak(int stage_idx, int channel);
float dsp_get_meter_peak_hold(int stage_idx, int channel);
bool dsp_get_meter_clipping(int stage_idx, int channel);
void dsp_clear_meter_clip(int stage_idx);

extern volatile bool dsp_suspended;
extern float current_fs;

// --- Crossover DSP structures ---

#define XOVER_MAX_BIQUADS 4   // Max biquads per filter (covers 48dB/oct)
#define DELAY_MAX_SAMPLES 384 // 8ms at 48kHz

// Single-pole filter state (for 6dB/oct)
typedef struct {
  float z1;
} onepole_state_t;

// Crossover filter chain (handles all slopes and types).
// Uses the same pending/target/crossfade pattern as the EQ stages so that
// freq/Q changes within a given topology crossfade smoothly. Topology changes
// (slope / type changes the number of biquads or switches to/from onepole)
// force a state reset inside the sync step to prevent instability.
typedef struct {
  // --- Live (audio-path) coefficients ---
  biquad_coeffs_t biquad_coeffs[XOVER_MAX_BIQUADS];
  biquad_coeffs_t biquad_target[XOVER_MAX_BIQUADS]; // Interpolation target
  biquad_state_t biquad_state_l[XOVER_MAX_BIQUADS];
  biquad_state_t biquad_state_r[XOVER_MAX_BIQUADS];
  float onepole_coeff;
  float onepole_coeff_target;
  onepole_state_t onepole_state_l;
  onepole_state_t onepole_state_r;

  // --- Live topology ---
  int num_biquads;
  bool is_onepole;
  bool enabled;

  // --- Pending values written by update(), read by sync under lock ---
  biquad_coeffs_t biquad_pending[XOVER_MAX_BIQUADS];
  float onepole_coeff_pending;
  int num_biquads_pending;
  bool is_onepole_pending;
  bool enabled_pending;

  // --- Parameters for Rule 3 interpolation ---
  float freq_cur;
  float freq_target;
  float q_cur[XOVER_MAX_BIQUADS];
  float q_target[XOVER_MAX_BIQUADS];
  
  bool is_highpass; // Filter direction (Rule 1)
  volatile bool dirty;

  int crossfade_samples; // Samples remaining in coefficient crossfade (0 = idle)
} xover_filter_t;

// Delay line for time alignment (up to 8ms at 48kHz).
// Shared ring buffer with two delay read points so delay changes crossfade
// glitchlessly — on change, `delay_target` is set and `crossfade_counter`
// starts; each sample reads from both delays and mixes. At fade end,
// delay_cur := delay_target. Dual buffers would be redundant since both
// would receive the same input anyway.
typedef struct {
  float buffer_l[DELAY_MAX_SAMPLES];
  float buffer_r[DELAY_MAX_SAMPLES];
  int write_idx;
  int delay_cur;         // Currently active delay (samples)
  int delay_target;      // Target delay during crossfade
  int crossfade_counter; // 0 = idle, else samples remaining
} delay_line_t;

// --- Crossover API (stubs in Phase 1, implemented in Phase 3) ---

void xover_init(void);
void xover_update_hp(xover_band_t *settings);
void xover_update_lp(xover_band_t *settings);
void xover_set_phase_invert(bool invert); // false = 0°, true = 180°
void xover_set_delay(float ms);           // Negative = delay main path
void xover_set_mono(bool mono);

// Low output control (stubs in Phase 1, implemented in Phase 2/3)
void low_set_output_gain(int channel, float db);
float low_get_output_gain(int channel);
void low_update_limiter(bool enabled, float threshold_db);

#endif
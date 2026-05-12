#include "dsp_engine.h"
#include "i2s_input.h"
#include "storage_backend.h" // storage_busy flag for audio-minimal mode
#include <Arduino.h>
#include <esp_dsp.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

// IRAM-resident biquad: verbatim copy of dsps_biquad_f32_aes3 asm, relocated
// to an .iram1.* section. Keeps the inner loop out of flash regardless of
// cache state and avoids per-call flash fetch overhead in the audio hot
// path. Source lives in dsps_biquad_f32_iram.S. Override the esp-dsp macro
// so all call sites pick up the new symbol transparently.
extern "C" esp_err_t dsps_biquad_f32_iram(const float *input, float *output,
                                          int len, float *coef, float *w);
#undef dsps_biquad_f32
#define dsps_biquad_f32 dsps_biquad_f32_iram

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FFT_SIZE 1024
#define I2S_NORM_FACTOR (1.0f / 2147483648.0f)
#define I2S_DENORM_FACTOR_24 8388607.0f // 2^23 - 1 (24-bit full scale)
#define I2S_BLOCK_FRAMES 256

// Level meter constants
#define METER_DECAY_DB_PER_SEC 26.0f // Main bar fall-off speed
#define METER_PEAK_HOLD_MS 500.0f    // Peak indicator hold time
#define METER_CLIP_LATCH_MS 100.0f   // Red clipping latch time
#define METER_FLOOR_DB -48.0f        // Bottom of meter range

// DRAM_ATTR pins these in internal SRAM (uncached). The audio hot path
// reads coefficients and state arrays from these every block; if they
// land in PSRAM (cached), LVGL drawing on Core 1 evicts the cache lines
// and the next audio block pays a PSRAM round-trip on every coefficient
// read. Direct DRAM is a 1-2 cycle access, immune to LVGL traffic.
DRAM_ATTR static int32_t i2s_read_buf[I2S_BLOCK_FRAMES * 2]
    __attribute__((aligned(16)));
DRAM_ATTR static int32_t i2s_write_buf[I2S_BLOCK_FRAMES * 2]
    __attribute__((aligned(16)));
DRAM_ATTR static int32_t i2s_low_write_buf[I2S_BLOCK_FRAMES * 2]
    __attribute__((aligned(16)));

// Phase 3: low-path float scratch buffers (filled in process_block, written to
// i2s_low_write_buf after limiter + delay + clamp)
DRAM_ATTR static float low_block_l[I2S_BLOCK_FRAMES];
DRAM_ATTR static float low_block_r[I2S_BLOCK_FRAMES];

// Double-buffered FFT input for decoupled FFT task
DRAM_ATTR static float dsp_fft_input_0[FFT_SIZE] __attribute__((aligned(16)));
DRAM_ATTR static float dsp_fft_input_1[FFT_SIZE] __attribute__((aligned(16)));
static float *dsp_fft_write_buf = dsp_fft_input_0; // DSP task writes here
static float *dsp_fft_read_buf = dsp_fft_input_1;  // FFT task reads here
static volatile bool fft_buffer_ready = false;     // Signal to FFT task

// Scope ring buffer — written every block alongside FFT tap, read by LVGL task
// via dsp_scope_get(). Power-of-2 so wrap is a bitwise AND.
DRAM_ATTR volatile float scope_ring_buf[SCOPE_BUF_SIZE];
volatile int scope_ring_idx = 0;

DRAM_ATTR static float dsp_fft_output[FFT_SIZE * 2]
    __attribute__((aligned(16)));
DRAM_ATTR static float dsp_hanning_window[FFT_SIZE]
    __attribute__((aligned(16)));
static int dsp_analysis_ptr = 0;

// FFT task handle
static TaskHandle_t fft_task_handle = NULL;

// DRAM_ATTR pins these in internal SRAM (uncached). The audio hot path
// reads coefficients and state arrays from these every block; if they
// land in PSRAM (cached), LVGL drawing on Core 1 evicts the cache lines
// and the next audio block pays a PSRAM round-trip on every coefficient
// read. Direct DRAM is a 1-2 cycle access, immune to LVGL traffic.
DRAM_ATTR static dsp_stage_t stage_input_dsp;
DRAM_ATTR static dsp_stage_t stage_output_dsp;
DRAM_ATTR static dsp_stage_t
    stage_low_dsp; // Phase 3: low-path EQ (0 bands until Phase 6 UI)
static volatile dsp_input_source_t dsp_input_source = DSP_INPUT_I2S;
static float dsp_noise_level = 0.1f;
static bool dsp_is_init = false;
static float dsp_iso_freqs[NUM_SPECTRUM_BANDS];

float current_fs = 48000.0f;
volatile bool dsp_suspended = false;
volatile bool dsp_stage_muted[3] = {false, false, false};
static float mute_gain_current[3] = {1.0f, 1.0f, 1.0f};  // ramped toward 0/1

void dsp_set_mute(int stage_idx, bool muted) {
  if (stage_idx >= 0 && stage_idx < 3) dsp_stage_muted[stage_idx] = muted;
}
bool dsp_get_mute(int stage_idx) {
  if (stage_idx >= 0 && stage_idx < 3) return dsp_stage_muted[stage_idx];
  return false;
}
static volatile bool fft_enabled = false; // FFT disabled by default

// Level meter state — also DRAM-pinned (touched every block in meter_process).
DRAM_ATTR static meter_state_t meter_input;
DRAM_ATTR static meter_state_t meter_output;
DRAM_ATTR static meter_state_t meter_low; // Phase 3: low-path meter

// Phase 3: crossover state. xover_settings is defined in eq_ui.cpp as the
// single source of truth; DSP reads it via the extern and writes back through
// the xover_update_* / xover_set_* API.
extern crossover_settings_t xover_settings;

DRAM_ATTR static xover_filter_t xover_hp;
DRAM_ATTR static xover_filter_t xover_lp;
DRAM_ATTR static delay_line_t delay_main;
DRAM_ATTR static delay_line_t delay_low;
static volatile bool low_phase_invert = false;
static volatile bool low_mono = false;
DRAM_ATTR static limiter_state_t low_limiter;

// Low output gain (separate from main output gain)
static volatile float low_gain_target_l = 1.0f;
static volatile float low_gain_target_r = 1.0f;
static float low_gain_current_l = 1.0f;
static float low_gain_current_r = 1.0f;
static volatile float low_gain_db_l = 0.0f;
static volatile float low_gain_db_r = 0.0f;

// Input/Output gain state (in linear scale for processing)
static volatile float input_gain_target_l = 1.0f;
static volatile float input_gain_target_r = 1.0f;
static volatile float output_gain_target_l = 1.0f;
static volatile float output_gain_target_r = 1.0f;
static float input_gain_current_l = 1.0f;
static float input_gain_current_r = 1.0f;
static float output_gain_current_l = 1.0f;
static float output_gain_current_r = 1.0f;

// Store dB values for getter functions
static volatile float input_gain_db_l = 0.0f;
static volatile float input_gain_db_r = 0.0f;
static volatile float output_gain_db_l = 0.0f;
static volatile float output_gain_db_r = 0.0f;

// Gain smoothing coefficient (per sample)
#define GAIN_SMOOTH_COEFF 0.002f

// EQ coefficient smoothing coefficient (per sample)
#define COEFF_SMOOTH_COEFF 0.005f

// Crossfade duration in samples (~100ms at 48kHz).
// Must be longer than the longest expected drag gesture so that continuous
// dragging keeps the smoothing window open — new targets arrive while the
// filter is still moving, preventing discrete steps against charged state.
#define CROSSFADE_SAMPLES 4800

// Low-block size for block-based DSP with coefficient interpolation.
// Must be 4: gives 12 kHz parameter update rate, keeping any residual stepping
// artefact above the audible range. LOW_BLOCK=32 produces a 1500 Hz update
// rate audible as a periodic tonal zipper during fast parameter drags.
#define LOW_BLOCK 32
// Alpha for 32-sample sub-block — deliberately slowed from the natural
// 1 - (1-0.005)^32 = 0.148 so that convergence takes ~44ms (longer than the
// 33ms UI tick). This means new targets arrive while the filter is still
// moving, so each tick redirects a smoothly-moving trajectory rather than
// stepping from a stationary, fully-charged state — eliminating chirp artefacts
// during fast parameter drags.
#define COEFF_SMOOTH_LOWBLOCK 0.015f

// Per-band convergence epsilon for the interp-path guard in process_block.
// A band whose freq/gain/Q deltas are all below this threshold is treated as
// converged and falls back to the cheap scalar path, even when the stage-level
// crossfade_samples window is still open. This prevents bands on other stages
// (or inactive bands on the same stage) from being dragged through the expensive
// calculate_biquad_forced path unnecessarily.
// Units: Hz for freq, dB for gain, dimensionless for Q — 0.01 is inaudible for all.
#define COEFF_SMOOTH_EPSILON 0.01f

// --- Denormal (subnormal) float trap protection ---
// ESP32-S3 FPU has no hardware subnormal handling; a single denormal value
// flowing through an IIR can trigger a software emulation exception that
// takes ~100x longer than a normal flop, spiking block time from ~0.5 ms
// up past 40 ms (enough to starve the DMA ring at 96 kHz). We defend in two
// layers: a tiny alternating-sign dither injected at the top of process_block
// so feedback paths never fully converge to zero, and an explicit flush of
// any decaying smoother / envelope that a zero input can drag toward denorm.
#define ANTI_DENORMAL_MAG 1e-18f
// Bit-pattern abs + compare — keeps this whole helper inlineable and never
// reaches into libm. Threshold 1e-20f as uint32 = 0x1A093964.
static inline IRAM_ATTR float flush_denormal(float x) {
  union {
    float f;
    uint32_t u;
  } v;
  v.f = x;
  v.u &= 0x7FFFFFFFu;
  return (v.u < 0x1A093964u) ? 0.0f : x;
}

// Inline finite check — exponent-field test, no libm call.
// Returns false for NaN or +/-Inf.
static inline IRAM_ATTR bool is_finite_fast(float x) {
  union {
    float f;
    uint32_t u;
  } v;
  v.f = x;
  return ((v.u & 0x7F800000u) != 0x7F800000u);
}

// Inline abs/max for float — single-instruction codegen on Xtensa, replaces
// libm fabsf/fmaxf which carry full function-call overhead.
static inline IRAM_ATTR float fabs_fast(float x) {
  union {
    float f;
    uint32_t u;
  } v;
  v.f = x;
  v.u &= 0x7FFFFFFFu;
  return v.f;
}
static inline IRAM_ATTR float fmax_fast(float a, float b) {
  return (a > b) ? a : b;
}

DRAM_ATTR static limiter_state_t limiter;
static portMUX_TYPE dsp_mux = portMUX_INITIALIZER_UNLOCKED;

// --- DSP profiling (Step 0) ---
#define PROFILE_INTERVAL 1000 // Print every N blocks (~5.3s at 256/48kHz)
static uint32_t prof_block_count = 0;
static uint32_t prof_worst_us = 0;
static uint32_t prof_total_us = 0;

// ============================================================
// Level meter processing
// ============================================================
static void meter_init(meter_state_t *m) {
  m->peak_l = 0.0f;
  m->peak_r = 0.0f;
  m->peak_hold_l = 0.0f;
  m->peak_hold_r = 0.0f;
  m->hold_timer_l = 0;
  m->hold_timer_r = 0;
  m->clip_timer_l = 0;
  m->clip_timer_r = 0;
  m->clipping_l = false;
  m->clipping_r = false;
  m->active = false;
}

static IRAM_ATTR void meter_process(meter_state_t *m, float *buf_l,
                                    float *buf_r, int count) {
  if (!m->active)
    return;

  // Find peak absolute value in this block
  float block_peak_l = 0.0f;
  float block_peak_r = 0.0f;
  for (int i = 0; i < count; i++) {
    float abs_l = fabs_fast(buf_l[i]);
    float abs_r = fabs_fast(buf_r[i]);
    if (abs_l > block_peak_l)
      block_peak_l = abs_l;
    if (abs_r > block_peak_r)
      block_peak_r = abs_r;
  }

  // Calculate per-block decay multiplier
  //   dB_per_block = METER_DECAY_DB_PER_SEC * (count / current_fs)
  //   decay_mult   = 10^(-dB_per_block / 20)
  // Inputs (count, current_fs) are effectively constant per session, so we
  // cache the result and skip the powf except when something actually
  // changes. Stored in DRAM so the read never touches flash.
  static DRAM_ATTR float cached_decay_mult = 0.0f;
  static DRAM_ATTR int cached_count = 0;
  static DRAM_ATTR float cached_fs = 0.0f;
  if (count != cached_count || current_fs != cached_fs) {
    float db_per_block = METER_DECAY_DB_PER_SEC * ((float)count / current_fs);
    cached_decay_mult = powf(10.0f, -db_per_block / 20.0f);
    cached_count = count;
    cached_fs = current_fs;
  }
  float decay_mult = cached_decay_mult;

  // Calculate timer decrements (in samples)
  uint32_t hold_samples =
      (uint32_t)((METER_PEAK_HOLD_MS / 1000.0f) * current_fs);
  uint32_t clip_samples =
      (uint32_t)((METER_CLIP_LATCH_MS / 1000.0f) * current_fs);

  // --- Left channel ---
  // Attack/decay for main peak
  if (block_peak_l > m->peak_l) {
    m->peak_l = block_peak_l; // Instant attack
  } else {
    m->peak_l *= decay_mult; // Decay
  }

  // Clipping detection and latch
  if (block_peak_l > 1.0f) {
    m->clipping_l = true;
    m->clip_timer_l = clip_samples;
  } else if (m->clip_timer_l > 0) {
    uint32_t decr = (uint32_t)count;
    if (decr > m->clip_timer_l)
      decr = m->clip_timer_l;
    m->clip_timer_l -= decr;
    if (m->clip_timer_l == 0) {
      m->clipping_l = false;
    }
  }

  // Peak hold
  if (m->peak_l > m->peak_hold_l) {
    m->peak_hold_l = m->peak_l;
    m->hold_timer_l = hold_samples;
  } else if (m->hold_timer_l > 0) {
    uint32_t decr = (uint32_t)count;
    if (decr > m->hold_timer_l)
      decr = m->hold_timer_l;
    m->hold_timer_l -= decr;
  } else {
    // Hold expired — peak_hold follows peak downward
    m->peak_hold_l = m->peak_l;
  }

  // --- Right channel ---
  if (block_peak_r > m->peak_r) {
    m->peak_r = block_peak_r;
  } else {
    m->peak_r *= decay_mult;
  }

  if (block_peak_r > 1.0f) {
    m->clipping_r = true;
    m->clip_timer_r = clip_samples;
  } else if (m->clip_timer_r > 0) {
    uint32_t decr = (uint32_t)count;
    if (decr > m->clip_timer_r)
      decr = m->clip_timer_r;
    m->clip_timer_r -= decr;
    if (m->clip_timer_r == 0) {
      m->clipping_r = false;
    }
  }

  if (m->peak_r > m->peak_hold_r) {
    m->peak_hold_r = m->peak_r;
    m->hold_timer_r = hold_samples;
  } else if (m->hold_timer_r > 0) {
    uint32_t decr = (uint32_t)count;
    if (decr > m->hold_timer_r)
      decr = m->hold_timer_r;
    m->hold_timer_r -= decr;
  } else {
    m->peak_hold_r = m->peak_r;
  }
}

// ============================================================
// SVF (State Variable Filter) implementation — Mandatory for low frequencies
// ============================================================
// Based on the Topology Preserving Transform (TPT) structure by Andrew Simper.
// Extremely stable even at very low frequencies (down to 1 Hz) and high Q.
static void calculate_svf_coeffs(biquad_coeffs_t *c, eq_band_t *b, float fs) {
  float freq = b->freq;
  if (freq > fs * 0.49f)
    freq = fs * 0.49f;
  if (freq < 1.0f)
    freq = 1.0f;

  // V = linear gain factor (10^(G/20))
  float V = powf(10.0f, b->gain / 20.0f);
  float g = tanf((float)M_PI * freq / fs);
  float k = 1.0f / b->q;

  c->is_svf = true;
  c->g = g;
  c->k = k;

  float h = 1.0f / (1.0f + g * (g + k));
  c->a1 = h; // Reuse a1 for the common divisor h

  switch (b->type) {
  case FTYPE_PEAK:
    // out = hp + (k*V)*bp + lp = x + k*(V-1)*bp
    c->m0 = 1.0f;
    c->m1 = k * V;
    c->m2 = 1.0f;
    break;
  case FTYPE_LOW_SHELF:
    // out = x + (V-1)*lp = hp + k*bp + V*lp
    c->m0 = 1.0f;
    c->m1 = k;
    c->m2 = V;
    break;
  case FTYPE_HIGH_SHELF:
    // out = x + (V-1)*hp = V*hp + k*bp + lp
    c->m0 = V;
    c->m1 = k;
    c->m2 = 1.0f;
    break;
  }
}

static inline IRAM_ATTR void
svf_block_stereo(float *in_l, float *out_l, float *in_r, float *out_r, int n,
                 biquad_coeffs_t *cl, biquad_state_t *sl, biquad_coeffs_t *cr,
                 biquad_state_t *sr) {

  float gl = cl->g, kl = cl->k, m0l = cl->m0, m1l = cl->m1, m2l = cl->m2,
        hl = cl->a1;
  float gr = cr->g, kr = cr->k, m0r = cr->m0, m1r = cr->m1, m2r = cr->m2,
        hr = cr->a1;
  float v1l = sl->s1, v2l = sl->s2;
  float v1r = sr->s1, v2r = sr->s2;

  for (int i = 0; i < n; i++) {
    float xl = in_l[i];
    float xr = in_r[i];

    // Left channel: Simper TPT SVF update
    float v3l = xl - v2l;
    float v1_next_l = hl * (v1l + gl * v3l);
    float v2_next_l = v2l + gl * v1_next_l;
    float bpl = v1_next_l;
    float lpl = v2_next_l;
    float hpl = xl - kl * bpl - lpl;
    out_l[i] = m0l * hpl + m1l * bpl + m2l * lpl;
    v1l = 2.0f * v1_next_l - v1l;
    v2l = 2.0f * v2_next_l - v2l;

    // Right channel: Simper TPT SVF update
    float v3r = xr - v2r;
    float v1_next_r = hr * (v1r + gr * v3r);
    float v2_next_r = v2r + gr * v1_next_r;
    float bpr = v1_next_r;
    float lpr = v2_next_r;
    float hpr = xr - kr * bpr - lpr;
    out_r[i] = m0r * hpr + m1r * bpr + m2r * lpr;
    v1r = 2.0f * v1_next_r - v1r;
    v2r = 2.0f * v2_next_r - v2r;
  }
  sl->s1 = v1l;
  sl->s2 = v2l;
  sr->s1 = v1r;
  sr->s2 = v2r;
}

static inline IRAM_ATTR void svf_block_scalar(float *in, float *out, int n,
                                              biquad_coeffs_t *c,
                                              biquad_state_t *s) {
  float g = c->g, k = c->k, m0 = c->m0, m1 = c->m1, m2 = c->m2, h = c->a1;
  float v1 = s->s1, v2 = s->s2;
  for (int i = 0; i < n; i++) {
    float x = in[i];
    float v3 = x - v2;
    float v1_next = h * (v1 + g * v3);
    float v2_next = v2 + g * v1_next;
    
    // Simper TPT Integrator Output
    float bp = v1_next;
    float lp = v2_next;
    float hp = x - k * bp - lp;
    
    out[i] = m0 * hp + m1 * bp + m2 * lp;
    
    // Correct TPT state update: the state is (2 * output - old_state)
    // for trapezoidal, but our v1/v2 are defined such that the next 
    // sample's v1 is v1_next.
    v1 = 2.0f * v1_next - v1;
    v2 = 2.0f * v2_next - v2;
  }
  s->s1 = v1;
  s->s2 = v2;
}

// Hysteresis thresholds — prevent rapid toggling near the SVF boundary
#define SVF_ENTER_HZ 55.0f    // TDF2 → SVF: freq must fall below this
#define SVF_EXIT_HZ 70.0f     // SVF → TDF2: freq must rise above this
#define SVF_ENTER_HZ_Q 110.0f // Q-conditional enter threshold
#define SVF_EXIT_HZ_Q 130.0f  // Q-conditional exit threshold
#define SVF_Q_THRESHOLD 2.0f      // Legacy reference — do not use directly in logic
#define SVF_Q_ENTER     2.1f      // TDF2 → SVF: Q must rise above this
#define SVF_Q_EXIT      1.9f      // SVF → TDF2: Q must fall below this

// Topology crossfade duration: 40 ms at 48 kHz (1920 samples)
#define SVF_XFADE_SAMPLES 1920
// Silent warmup duration: 10 ms at 48 kHz
#define PREWARM_SAMPLES 480

static inline bool should_use_svf(bool currently_svf, float freq, float q) {
  if (currently_svf) {
    // Exit SVF only when freq is high enough AND Q has fallen below exit threshold
    if (freq > SVF_EXIT_HZ && !(freq < SVF_EXIT_HZ_Q && q > SVF_Q_EXIT))
      return false;
    return true;
  } else {
    // Enter SVF when freq is low enough OR Q exceeds entry threshold in mid-bass range
    if (freq < SVF_ENTER_HZ || (freq < SVF_ENTER_HZ_Q && q > SVF_Q_ENTER))
      return true;
    return false;
  }
}

// ============================================================
// Biquad coefficient calculation — bilinear prewarped, float32 only
// ============================================================
static void calculate_tdf2_coeffs(biquad_coeffs_t *c, const eq_band_t *b,
                                  float fs) {
  float freq = b->freq;
  c->is_svf = false;
  if (freq > fs * 0.49f)
    freq = fs * 0.49f;
  if (freq < 10.0f)
    freq = 10.0f;

  // Identity bypass: disabled band or effectively 0 dB gain on gain-type
  // filters.
  if (!b->enabled || (fabsf(b->gain) < 0.01f &&
                      (b->type == FTYPE_PEAK || b->type == FTYPE_LOW_SHELF ||
                       b->type == FTYPE_HIGH_SHELF))) {
    c->b0 = 1.0f;
    c->b1 = 0.0f;
    c->b2 = 0.0f;
    c->a1 = 0.0f;
    c->a2 = 0.0f;
    return;
  }

  // Bilinear prewarped frequency: K = tan(pi*fc/fs)
  float K = tanf((float)M_PI * freq / fs);
  float KK = K * K;
  float A = powf(10.0f, b->gain / 40.0f);
  float Asq = sqrtf(A);
  float q = b->q;

  float b0, b1, b2, a0, a1, a2;

  switch (b->type) {
  case FTYPE_PEAK: {
    a0 = 1.0f + K / (A * q) + KK;
    b0 = (1.0f + A * K / q + KK) / a0;
    b1 = 2.0f * (KK - 1.0f) / a0;
    b2 = (1.0f - A * K / q + KK) / a0;
    a1 = 2.0f * (KK - 1.0f) / a0;
    a2 = (1.0f - K / (A * q) + KK) / a0;
    break;
  }
  case FTYPE_LOW_SHELF: {
    a0 = A + Asq * K / q + KK;
    b0 = A * (A * KK + Asq * K / q + 1.0f) / a0;
    b1 = 2.0f * A * (A * KK - 1.0f) / a0;
    b2 = A * (A * KK - Asq * K / q + 1.0f) / a0;
    a1 = 2.0f * (KK - A) / a0;
    a2 = (KK - Asq * K / q + A) / a0;
    break;
  }
  case FTYPE_HIGH_SHELF: {
    a0 = 1.0f + Asq * K / q + A * KK;
    b0 = A * (A + Asq * K / q + KK) / a0;
    b1 = 2.0f * A * (KK - A) / a0;
    b2 = A * (A - Asq * K / q + KK) / a0;
    a1 = 2.0f * (A * KK - 1.0f) / a0;
    a2 = (A * KK - Asq * K / q + 1.0f) / a0;
    break;
  }
  default:
    c->b0 = 1.0f;
    c->b1 = 0.0f;
    c->b2 = 0.0f;
    c->a1 = 0.0f;
    c->a2 = 0.0f;
    return;
  }

  c->b0 = b0;
  c->b1 = b1;
  c->b2 = b2;
  c->a1 = a1;
  c->a2 = a2;
}

static void calculate_svf_coeffs(biquad_coeffs_t *c, const eq_band_t *b,
                                 float fs) {
  float freq = b->freq;
  if (freq > fs * 0.49f)
    freq = fs * 0.49f;
  if (freq < 2.0f)
    freq = 2.0f;

  // Simper TPT SVF Coefficient Calculation
  float g = tanf((float)M_PI * freq / fs);
  float A = powf(10.0f, b->gain / 40.0f); // A = sqrt(gain_linear)
  float Asq = sqrtf(A);
  float k = 1.0f / b->q;
  float k_svf, m0, m1, m2;

  switch (b->type) {
  case FTYPE_PEAK: {
    if (b->gain >= 0) {
      k_svf = k / A;
      m0 = 1.0f;
      m1 = k * A;
      m2 = 1.0f;
    } else {
      k_svf = k * A;
      m0 = 1.0f;
      m1 = k / A;
      m2 = 1.0f;
    }
    break;
  }
  case FTYPE_LOW_SHELF: {
    k_svf = k / Asq;
    m0 = 1.0f;
    m1 = k * Asq;
    m2 = A;
    break;
  }
  case FTYPE_HIGH_SHELF: {
    k_svf = k * Asq;
    m0 = A;
    m1 = k * Asq;
    m2 = 1.0f;
    break;
  }
  default:
    c->g = 0.0f;
    c->k = 2.0f;
    c->m0 = 0.0f;
    c->m1 = 0.0f;
    c->m2 = 1.0f;
    c->a1 = 1.0f;
    return;
  }

  c->g = g;
  c->k = k_svf;
  c->m0 = m0;
  c->m1 = m1;
  c->m2 = m2;
  // a1 stores the pre-computed feedback compensation 'h'
  c->a1 = 1.0f / (1.0f + g * (g + k_svf));
  c->is_svf = true;
}

static IRAM_ATTR void calculate_biquad(biquad_coeffs_t *c, eq_band_t *b,
                                       float fs) {
  // Rule 9: Select SVF for low frequencies or high-Q low-mids to avoid TDF2
  // instability.
  if (should_use_svf(c->is_svf, b->freq, b->q) && b->enabled) {
    calculate_svf_coeffs(c, b, fs);
  } else {
    calculate_tdf2_coeffs(c, b, fs);
  }
}

// Computes biquad coefficients with topology forced to force_svf,
// regardless of frequency thresholds. Used during topology crossfades
// to compute target-topology coefficients at current parameter values.
static void calculate_biquad_forced(biquad_coeffs_t *c, const eq_band_t *b,
                                    float fs, bool force_svf) {
  // Create a local copy so we never mutate the caller's band params
  eq_band_t tmp = *b;
  if (force_svf) {
    calculate_svf_coeffs(c, &tmp, fs); // call SVF path directly
    c->is_svf = true;
  } else {
    calculate_tdf2_coeffs(c, &tmp, fs); // call TDF2 path directly
    c->is_svf = false;
  }
}

// Block-based Transposed Direct Form II (TDFII) processors.
// TDFII is mathematically equivalent to DFI for low-frequency precision in
// floating point, but it is 20% faster and much more SIMD-friendly for stereo
// processing.

// Stereo-Interleaved: Processes L and R simultaneously to help the compiler use
// SIMD/PIE.
static inline IRAM_ATTR void
biquad_block_tdf2_stereo(float *in_l, float *out_l, float *in_r, float *out_r,
                         int n, biquad_coeffs_t *cl, biquad_state_t *sl,
                         biquad_coeffs_t *cr, biquad_state_t *sr) {

  float b0l = cl->b0, b1l = cl->b1, b2l = cl->b2, a1l = cl->a1, a2l = cl->a2;
  float b0r = cr->b0, b1r = cr->b1, b2r = cr->b2, a1r = cr->a1, a2r = cr->a2;
  float s1l = sl->s1, s2l = sl->s2;
  float s1r = sr->s1, s2r = sr->s2;

  for (int i = 0; i < n; i++) {
    float xl = in_l[i];
    float xr = in_r[i];

    float yl = b0l * xl + s1l;
    float yr = b0r * xr + s1r;

    s1l = b1l * xl - a1l * yl + s2l;
    s1r = b1r * xr - a1r * yr + s2r;

    s2l = b2l * xl - a2l * yl;
    s2r = b2r * xr - a2r * yr;

    out_l[i] = yl;
    out_r[i] = yr;
  }
  sl->s1 = s1l;
  sl->s2 = s2l;
  sr->s1 = s1r;
  sr->s2 = s2r;
}

// Scalar version for remainder bands in split-filter mode
static inline IRAM_ATTR void biquad_block_tdf2_scalar(float *in, float *out,
                                                      int n, biquad_coeffs_t *c,
                                                      biquad_state_t *s) {
  float b0 = c->b0, b1 = c->b1, b2 = c->b2, a1 = c->a1, a2 = c->a2;
  float s1 = s->s1, s2 = s->s2;
  for (int i = 0; i < n; i++) {
    float x = in[i];
    float y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    out[i] = y;
  }
  s->s1 = s1;
  s->s2 = s2;
}

// Per-sample coefficient interpolating TDF2 filter.
// Linearly ramps all five biquad coefficients from c_start to c_end across
// the block, eliminating the staircase discontinuity that occurs when a
// step-updated coefficient hits a charged filter state. Used during active
// parameter smoothing (crossfade_samples > 0) as a drop-in replacement for
// biquad_block_tdf2_scalar. Steady-state blocks use the scalar path unchanged.
static inline IRAM_ATTR void
biquad_block_tdf2_interp(float *in, float *out, int n,
                         const biquad_coeffs_t *c_start,
                         const biquad_coeffs_t *c_end,
                         biquad_state_t *s) {
  float inv_n = 1.0f / (float)n;
  float b0 = c_start->b0, db0 = (c_end->b0 - c_start->b0) * inv_n;
  float b1 = c_start->b1, db1 = (c_end->b1 - c_start->b1) * inv_n;
  float b2 = c_start->b2, db2 = (c_end->b2 - c_start->b2) * inv_n;
  float a1 = c_start->a1, da1 = (c_end->a1 - c_start->a1) * inv_n;
  float a2 = c_start->a2, da2 = (c_end->a2 - c_start->a2) * inv_n;
  float s1 = s->s1, s2 = s->s2;
  for (int i = 0; i < n; i++) {
    float x = in[i];
    float y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    out[i] = y;
    b0 += db0; b1 += db1; b2 += db2;
    a1 += da1; a2 += da2;
  }
  s->s1 = s1;
  s->s2 = s2;
}

// Per-sample coefficient interpolating SVF filter.
// Ramps g, k, h (a1), m0, m1, m2 linearly across the block. The small
// h = 1/(1+g*(g+k)) error from independent linear interpolation is
// second-order in the per-block delta and negligible at COEFF_SMOOTH_LOWBLOCK
// magnitudes.
static inline IRAM_ATTR void
svf_block_interp(float *in, float *out, int n,
                 const biquad_coeffs_t *c_start,
                 const biquad_coeffs_t *c_end,
                 biquad_state_t *s) {
  float inv_n = 1.0f / (float)n;
  float g  = c_start->g,   dg  = (c_end->g   - c_start->g)   * inv_n;
  float k  = c_start->k,   dk  = (c_end->k   - c_start->k)   * inv_n;
  float h  = c_start->a1,  dh  = (c_end->a1  - c_start->a1)  * inv_n;
  float m0 = c_start->m0,  dm0 = (c_end->m0  - c_start->m0)  * inv_n;
  float m1 = c_start->m1,  dm1 = (c_end->m1  - c_start->m1)  * inv_n;
  float m2 = c_start->m2,  dm2 = (c_end->m2  - c_start->m2)  * inv_n;
  float v1 = s->s1, v2 = s->s2;
  for (int i = 0; i < n; i++) {
    float x = in[i];
    float v3 = x - v2;
    float v1n = h * (v1 + g * v3);
    float v2n = v2 + g * v1n;
    float bp = v1n, lp = v2n;
    float hp = x - k * bp - lp;
    out[i] = m0 * hp + m1 * bp + m2 * lp;
    v1 = 2.0f * v1n - v1;
    v2 = 2.0f * v2n - v2;
    g += dg; k += dk; h += dh;
    m0 += dm0; m1 += dm1; m2 += dm2;
  }
  s->s1 = v1;
  s->s2 = v2;
}

// Rule 3.4 stability priority (lower processed first)
static int get_band_priority(eq_band_t *b) {
  if (!b->enabled)
    return 100;
  // SVF section is always last (priority 60)
  if (b->freq < SVF_ENTER_HZ ||
      (b->freq < SVF_ENTER_HZ_Q && b->q > SVF_Q_ENTER))
    return 60;
  if (b->gain < 0) {
    if (b->q > SVF_Q_THRESHOLD)
      return 10; // High-Q cut
    return 20;   // Low-Q cut / neutral
  }
  if (b->type != FTYPE_PEAK)
    return 40; // Shelves
  if (b->q <= SVF_Q_THRESHOLD)
    return 30; // Broad boost
  return 50;   // High-Q boost
}

static inline IRAM_ATTR void sync_coeffs(dsp_stage_t *stage) {
  taskENTER_CRITICAL(&dsp_mux);
  bool any_smooth = false;
  bool sort_l = false, sort_r = false;
  for (int b = 0; b < MAX_BANDS; b++) {
    if (stage->dirty_l[b]) {
      stage->dirty_l[b] = false;
      sort_l = true;
      eq_band_t *tgt = &stage->pending_params_l[b];
      eq_band_t *cur = &stage->cur_params_l[b];
      bool current_svf = stage->coeffs_l[b].is_svf;
      bool target_svf =
          should_use_svf(current_svf, tgt->freq, tgt->q) && tgt->enabled;

      bool type_or_enable_change =
          (tgt->type != cur->type) || (tgt->enabled != cur->enabled);
      bool svf_boundary_change = (target_svf != current_svf);

      stage->target_params_l[b] = *tgt;

      // Abort any in-progress topology xfade whose direction no longer matches
      // the current target — i.e. the user swept back across the boundary before
      // the xfade completed. The main filter is unchanged so no state reset needed.
      if ((stage->xf_warmup_samples_l[b] > 0 || stage->xf_topo_samples_l[b] > 0) &&
          stage->xf_topo_to_svf_l[b] != target_svf) {
        stage->xf_warmup_samples_l[b] = 0;
        stage->xf_topo_samples_l[b] = 0;
      }

      bool just_enabled  = tgt->enabled  && !cur->enabled;
      bool just_disabled = !tgt->enabled && cur->enabled;

      if (just_enabled) {
        // Fade gain in from silence — prevents zero-state startup transient
        // when a full-gain filter suddenly appears in a live signal chain.
        stage->cur_params_l[b] = *tgt;
        stage->cur_params_l[b].gain = 0.0f;
        calculate_biquad(&stage->coeffs_l[b], &stage->cur_params_l[b],
                         current_fs);
        memset(&stage->state_l[b], 0, sizeof(biquad_state_t));
        memset(&stage->xf_state_l[b], 0, sizeof(biquad_state_t));
        stage->xf_warmup_samples_l[b] = 0;
        stage->xf_topo_samples_l[b] = 0;
        any_smooth = true;
      } else if (just_disabled) {
        // Ramp gain to zero before disappearing — no snap, no state reset.
        // cur_params stays as-is; interpolation ramps gain toward 0.
        // Bypass check (|gain| < 0.01) silences the band once converged.
        // Final snap sets cur_params = target_params (enabled=false, gain=0).
        stage->target_params_l[b].gain = 0.0f;
        any_smooth = true;
      } else if (type_or_enable_change) {
        // Filter type change while enabled (peak↔shelf etc.): snap immediately.
        // Crossfading between filter types is meaningless.
        stage->cur_params_l[b] = *tgt;
        calculate_biquad(&stage->coeffs_l[b], &stage->cur_params_l[b],
                         current_fs);
        memset(&stage->state_l[b], 0, sizeof(biquad_state_t));
        memset(&stage->xf_state_l[b], 0, sizeof(biquad_state_t));
        stage->xf_warmup_samples_l[b] = 0;
        stage->xf_topo_samples_l[b] = 0;
      } else if (svf_boundary_change && stage->xf_topo_samples_l[b] <= 0 &&
                 stage->xf_warmup_samples_l[b] <= 0) {
        // Topology boundary crossed: start silent pre-warm, then parallel
        // crossfade.
        stage->xf_topo_to_svf_l[b] = target_svf;
        // Cross-topology (SVF↔TDF2): state registers mean physically different
        // things in each topology, so a direct copy is invalid — zero instead.
        // The PREWARM_SAMPLES silent warmup gives the xfade filter time to
        // partially charge from the live signal before blending begins.
        // Same-topology path (if ever triggered): copy main state so the xfade
        // filter starts from the correct operating point with no transient.
        if (current_svf != target_svf) {
          memset(&stage->xf_state_l[b], 0, sizeof(biquad_state_t));
        } else {
          stage->xf_state_l[b] = stage->state_l[b];
        }
        calculate_biquad_forced(&stage->xf_coeffs_l[b], cur, current_fs,
                                target_svf);
        stage->xf_warmup_samples_l[b] = PREWARM_SAMPLES;
        stage->xf_topo_samples_l[b] = 0;
        any_smooth = true;
        // Extend global timer to cover both warmup and crossfade (20ms total)
        if (stage->crossfade_samples < PREWARM_SAMPLES + SVF_XFADE_SAMPLES) {
          stage->crossfade_samples = PREWARM_SAMPLES + SVF_XFADE_SAMPLES;
        }
      } else {
        any_smooth = true;
      }
    }
    if (stage->dirty_r[b]) {
      stage->dirty_r[b] = false;
      sort_r = true;
      eq_band_t *tgt = &stage->pending_params_r[b];
      eq_band_t *cur = &stage->cur_params_r[b];
      bool current_svf = stage->coeffs_r[b].is_svf;
      bool target_svf =
          should_use_svf(current_svf, tgt->freq, tgt->q) && tgt->enabled;

      bool type_or_enable_change =
          (tgt->type != cur->type) || (tgt->enabled != cur->enabled);
      bool svf_boundary_change = (target_svf != current_svf);

      stage->target_params_r[b] = *tgt;

      // Abort any in-progress topology xfade whose direction no longer matches
      // the current target — i.e. the user swept back across the boundary before
      // the xfade completed. The main filter is unchanged so no state reset needed.
      if ((stage->xf_warmup_samples_r[b] > 0 || stage->xf_topo_samples_r[b] > 0) &&
          stage->xf_topo_to_svf_r[b] != target_svf) {
        stage->xf_warmup_samples_r[b] = 0;
        stage->xf_topo_samples_r[b] = 0;
      }

      bool just_enabled  = tgt->enabled  && !cur->enabled;
      bool just_disabled = !tgt->enabled && cur->enabled;

      if (just_enabled) {
        stage->cur_params_r[b] = *tgt;
        stage->cur_params_r[b].gain = 0.0f;
        calculate_biquad(&stage->coeffs_r[b], &stage->cur_params_r[b],
                         current_fs);
        memset(&stage->state_r[b], 0, sizeof(biquad_state_t));
        memset(&stage->xf_state_r[b], 0, sizeof(biquad_state_t));
        stage->xf_warmup_samples_r[b] = 0;
        stage->xf_topo_samples_r[b] = 0;
        any_smooth = true;
      } else if (just_disabled) {
        stage->target_params_r[b].gain = 0.0f;
        any_smooth = true;
      } else if (type_or_enable_change) {
        stage->cur_params_r[b] = *tgt;
        calculate_biquad(&stage->coeffs_r[b], &stage->cur_params_r[b],
                         current_fs);
        memset(&stage->state_r[b], 0, sizeof(biquad_state_t));
        memset(&stage->xf_state_r[b], 0, sizeof(biquad_state_t));
        stage->xf_warmup_samples_r[b] = 0;
        stage->xf_topo_samples_r[b] = 0;
      } else if (svf_boundary_change && stage->xf_topo_samples_r[b] <= 0 &&
                 stage->xf_warmup_samples_r[b] <= 0) {
        // Topology boundary crossed: start silent pre-warm, then parallel
        // crossfade.
        stage->xf_topo_to_svf_r[b] = target_svf;
        if (current_svf != target_svf) {
          memset(&stage->xf_state_r[b], 0, sizeof(biquad_state_t));
        } else {
          stage->xf_state_r[b] = stage->state_r[b];
        }
        calculate_biquad_forced(&stage->xf_coeffs_r[b], cur, current_fs,
                                target_svf);
        stage->xf_warmup_samples_r[b] = PREWARM_SAMPLES;
        stage->xf_topo_samples_r[b] = 0;
        any_smooth = true;
        // Extend global timer to cover both warmup and crossfade (20ms total)
        if (stage->crossfade_samples < PREWARM_SAMPLES + SVF_XFADE_SAMPLES) {
          stage->crossfade_samples = PREWARM_SAMPLES + SVF_XFADE_SAMPLES;
        }
      } else {
        any_smooth = true;
      }
    }
  }

  // Apply priority sorting (Rule 3.4) only when no smoothing is active.
  // Deferring the sort prevents state-mismatch transients that occur when
  // processing order changes mid-gesture (e.g. Q crossing SVF_Q_THRESHOLD,
  // or gain sign flip changing cut/boost priority). Evaluated AFTER any_smooth
  // is set so the sort never fires on the same call a new dirty flag lands.
  bool sort_idle = !any_smooth && (stage->crossfade_samples <= 0);
  if (sort_l && sort_idle) {
    for (int i = 0; i < MAX_BANDS; i++)
      stage->sorted_idx_l[i] = i;
    for (int i = 0; i < MAX_BANDS - 1; i++) {
      for (int j = i + 1; j < MAX_BANDS; j++) {
        if (get_band_priority(&stage->target_params_l[stage->sorted_idx_l[i]]) >
            get_band_priority(
                &stage->target_params_l[stage->sorted_idx_l[j]])) {
          int tmp = stage->sorted_idx_l[i];
          stage->sorted_idx_l[i] = stage->sorted_idx_l[j];
          stage->sorted_idx_l[j] = tmp;
        }
      }
    }
  }
  if (sort_r && sort_idle) {
    for (int i = 0; i < MAX_BANDS; i++)
      stage->sorted_idx_r[i] = i;
    for (int i = 0; i < MAX_BANDS - 1; i++) {
      for (int j = i + 1; j < MAX_BANDS; j++) {
        if (get_band_priority(&stage->target_params_r[stage->sorted_idx_r[i]]) >
            get_band_priority(
                &stage->target_params_r[stage->sorted_idx_r[j]])) {
          int tmp = stage->sorted_idx_r[i];
          stage->sorted_idx_r[i] = stage->sorted_idx_r[j];
          stage->sorted_idx_r[j] = tmp;
        }
      }
    }
  }

  if (any_smooth) {
    // Unconditional reset — keeps the smoothing window open for the full
    // CROSSFADE_SAMPLES every time a new target lands, so continuous dragging
    // never allows the window to expire mid-gesture.
    stage->crossfade_samples = CROSSFADE_SAMPLES;
  }
  taskEXIT_CRITICAL(&dsp_mux);
}

// Lerp parameters (freq, Q, gain) toward their targets in parameter space,
// then recalculate biquad coefficients from the interpolated parameters.
// Called once per LOW_BLOCK during an active crossfade.
// Never lerps biquad coefficients directly — coefficient-space lerp produces
// invalid intermediate filter shapes (Rule 3).
static inline IRAM_ATTR void interpolate_coeffs(dsp_stage_t *stage, int n) {
  if (stage->crossfade_samples <= 0)
    return;
  stage->crossfade_samples -= n;

  for (int b = 0; b < MAX_BANDS; b++) {
    // --- L channel ---
    eq_band_t *cur_l = &stage->cur_params_l[b];
    const eq_band_t *tgt_l = &stage->target_params_l[b];
    cur_l->freq += COEFF_SMOOTH_LOWBLOCK * (tgt_l->freq - cur_l->freq);
    cur_l->gain += COEFF_SMOOTH_LOWBLOCK * (tgt_l->gain - cur_l->gain);
    cur_l->q += COEFF_SMOOTH_LOWBLOCK * (tgt_l->q - cur_l->q);

    if (stage->xf_warmup_samples_l[b] > 0) {
      // Update TARGET filter (which is silent/warming up)
      calculate_biquad_forced(&stage->xf_coeffs_l[b], cur_l, current_fs,
                              stage->xf_topo_to_svf_l[b]);
      stage->xf_warmup_samples_l[b] -= n;
      if (stage->xf_warmup_samples_l[b] <= 0) {
        stage->xf_warmup_samples_l[b] = 0;
        stage->xf_topo_samples_l[b] = SVF_XFADE_SAMPLES;
      }
      // Outgoing filter is FROZEN to ensure stability during the transition
    } else if (stage->xf_topo_samples_l[b] > 0) {
      // Update TARGET filter (which is fading in)
      calculate_biquad_forced(&stage->xf_coeffs_l[b], cur_l, current_fs,
                              stage->xf_topo_to_svf_l[b]);
      stage->xf_topo_samples_l[b] -= n;
      if (stage->xf_topo_samples_l[b] <= 0) {
        stage->xf_topo_samples_l[b] = 0;
        stage->coeffs_l[b] = stage->xf_coeffs_l[b];
        stage->state_l[b] = stage->xf_state_l[b];
      }
      // Outgoing filter is FROZEN to ensure stability during the transition
    } else {
      // Normal parameter smoothing (Topology Locked)
      calculate_biquad_forced(&stage->coeffs_l[b], cur_l, current_fs,
                              stage->coeffs_l[b].is_svf);
    }

    // --- R channel ---
    eq_band_t *cur_r = &stage->cur_params_r[b];
    const eq_band_t *tgt_r = &stage->target_params_r[b];
    cur_r->freq += COEFF_SMOOTH_LOWBLOCK * (tgt_r->freq - cur_r->freq);
    cur_r->gain += COEFF_SMOOTH_LOWBLOCK * (tgt_r->gain - cur_r->gain);
    cur_r->q += COEFF_SMOOTH_LOWBLOCK * (tgt_r->q - cur_r->q);

    if (stage->xf_warmup_samples_r[b] > 0) {
      calculate_biquad_forced(&stage->xf_coeffs_r[b], cur_r, current_fs,
                              stage->xf_topo_to_svf_r[b]);
      stage->xf_warmup_samples_r[b] -= n;
      if (stage->xf_warmup_samples_r[b] <= 0) {
        stage->xf_warmup_samples_r[b] = 0;
        stage->xf_topo_samples_r[b] = SVF_XFADE_SAMPLES;
      }
    } else if (stage->xf_topo_samples_r[b] > 0) {
      calculate_biquad_forced(&stage->xf_coeffs_r[b], cur_r, current_fs,
                              stage->xf_topo_to_svf_r[b]);
      stage->xf_topo_samples_r[b] -= n;
      if (stage->xf_topo_samples_r[b] <= 0) {
        stage->xf_topo_samples_r[b] = 0;
        stage->coeffs_r[b] = stage->xf_coeffs_r[b];
        stage->state_r[b] = stage->xf_state_r[b];
      }
    } else {
      calculate_biquad_forced(&stage->coeffs_r[b], cur_r, current_fs,
                              stage->coeffs_r[b].is_svf);
    }
  }

  // Final snap to target
  if (stage->crossfade_samples <= 0) {
    stage->crossfade_samples = 0;
    for (int b = 0; b < MAX_BANDS; b++) {
      stage->cur_params_l[b] = stage->target_params_l[b];
      // Use calculate_biquad_forced to lock the current topology — prevents
      // calculate_biquad's hysteresis logic from flipping topology at snap time.
      calculate_biquad_forced(&stage->coeffs_l[b], &stage->cur_params_l[b],
                              current_fs, stage->coeffs_l[b].is_svf);
      stage->cur_params_r[b] = stage->target_params_r[b];
      calculate_biquad_forced(&stage->coeffs_r[b], &stage->cur_params_r[b],
                              current_fs, stage->coeffs_r[b].is_svf);
    }
  }
}

static void update_spectrum_from_fft(void) {
  float freq_step = current_fs / (float)FFT_SIZE;
  for (int b = 0; b < NUM_SPECTRUM_BANDS; b++) {
    int bin = (int)(dsp_iso_freqs[b] / freq_step);
    if (bin >= FFT_SIZE / 2)
      bin = FFT_SIZE / 2 - 1;
    if (bin < 0)
      bin = 0;
    float real = dsp_fft_output[bin * 2];
    float imag = dsp_fft_output[bin * 2 + 1];
    float magnitude = sqrtf(real * real + imag * imag) / (FFT_SIZE / 2);
    shared_spectrum_magnitudes[b] = 20.0f * log10f(fmaxf(magnitude, 1e-6f));
  }
}

static uint32_t dsp_rng_state = 0xACE1u;

// White noise source (xorshift32, uniform in [-1, +1]).
static inline float white_noise(void) {
  dsp_rng_state ^= dsp_rng_state << 13;
  dsp_rng_state ^= dsp_rng_state >> 17;
  dsp_rng_state ^= dsp_rng_state << 5;
  return ((float)dsp_rng_state * (2.0f / 4294967295.0f)) - 1.0f;
}

// Pink noise via Paul Kellet's "economy" 4-pole filter over white noise.
// -3 dB/octave down to ~10 Hz, within ±0.5 dB across the audible band.
// Coefficients are sample-rate independent in their pole placement (the
// filter's *shape* is scale-invariant because it's a -3 dB/oct slope), so
// no recalculation is needed when fs changes — only the total output
// amplitude depends on fs, and we normalize that with noise_level_scale.
static float pink_b0 = 0.0f, pink_b1 = 0.0f, pink_b2 = 0.0f;
static float pink_b3 = 0.0f, pink_b4 = 0.0f, pink_b5 = 0.0f, pink_b6 = 0.0f;

// Output scaling — tuned so pink noise peaks sit roughly in the same ballpark
// as the old white generator. Final gain is applied by dsp_noise_level.
static const float PINK_OUT_SCALE = 0.11f;

static inline IRAM_ATTR float fast_noise(void) {
  float w = white_noise();
  pink_b0 = 0.99886f * pink_b0 + w * 0.0555179f;
  pink_b1 = 0.99332f * pink_b1 + w * 0.0750759f;
  pink_b2 = 0.96900f * pink_b2 + w * 0.1538520f;
  pink_b3 = 0.86650f * pink_b3 + w * 0.3104856f;
  pink_b4 = 0.55000f * pink_b4 + w * 0.5329522f;
  pink_b5 = -0.7616f * pink_b5 - w * 0.0168980f;
  float pink = pink_b0 + pink_b1 + pink_b2 + pink_b3 + pink_b4 + pink_b5 +
               pink_b6 + w * 0.5362f;
  pink_b6 = w * 0.115926f;
  return pink * PINK_OUT_SCALE;
}

// ============================================================
// Sweep and warble test signal generators (Phase 13)
// ============================================================
// Logarithmic frequency sweeps (exponential ramp) and warble tone (sweep
// with 5 Hz FM) for EQ/room tuning. All loop continuously — elapsed time
// wraps back to 0 at the end of each cycle.

// --- Sine Look-Up Table (Optimization 2) ---
#define DSP_SINE_LUT_SIZE 1024
static float dsp_sine_lut[DSP_SINE_LUT_SIZE];
static bool dsp_sine_lut_ready = false;

static void dsp_init_sine_lut(void) {
  if (dsp_sine_lut_ready)
    return;
  for (int i = 0; i < DSP_SINE_LUT_SIZE; i++) {
    dsp_sine_lut[i] =
        sinf((float)i * 2.0f * (float)M_PI / (float)DSP_SINE_LUT_SIZE);
  }
  dsp_sine_lut_ready = true;
}

static inline IRAM_ATTR float fast_sine(float phase) {
  // phase must be 0 to 2*PI
  float idx_f = phase * (float)DSP_SINE_LUT_SIZE / (2.0f * (float)M_PI);
  int idx = (int)idx_f;
  if (idx >= DSP_SINE_LUT_SIZE)
    idx = DSP_SINE_LUT_SIZE - 1;
  if (idx < 0)
    idx = 0;

  int idx_next = (idx + 1) % DSP_SINE_LUT_SIZE;
  float frac = idx_f - (float)idx;

  return dsp_sine_lut[idx] +
         frac * (dsp_sine_lut[idx_next] - dsp_sine_lut[idx]);
}

static struct {
  float start_freq;         // Hz
  float end_freq;           // Hz
  float duration_sec;       // Total loop duration
  uint32_t elapsed_samples; // Time since last reset (number of samples)
  float phase;              // Oscillator phase accumulator (radians)
  float current_freq;       // Per-sample optimized freq
  float freq_mult;          // Per-sample exponential multiplier
} sweep_state;

static struct {
  float carrier_phase; // Carrier sweep phase (radians)
  float lfo_phase;     // 5 Hz LFO phase for FM (radians)
} warble_state;

// Reset sweep/warble state on mode change (avoids phase discontinuities)
static void reset_sweep_state(float start_hz, float end_hz, float duration_s) {
  sweep_state.start_freq = start_hz;
  sweep_state.end_freq = end_hz;
  sweep_state.duration_sec = duration_s;
  sweep_state.elapsed_samples = 0;
  sweep_state.phase = 0.0f;
  sweep_state.current_freq = start_hz;
  // Multiplier for exponential sweep: freq(n+1) = freq(n) * k
  // k = (end/start) ^ (1 / total_samples)
  sweep_state.freq_mult =
      powf(end_hz / start_hz, 1.0f / (duration_s * current_fs));
}

static void reset_warble_state(float start_hz, float end_hz, float duration_s) {
  reset_sweep_state(start_hz, end_hz, duration_s); // reuse sweep state
  warble_state.carrier_phase = 0.0f;
  warble_state.lfo_phase = 0.0f;
}

// Generate one sample of logarithmic sweep. Freq ramps exponentially from
// start_freq to end_freq over duration_sec, then wraps. Phase accumulator
// ensures continuous waveform across the ramp.
static inline IRAM_ATTR float generate_log_sweep(void) {
  const float fs = current_fs;

  if (sweep_state.elapsed_samples >=
      (uint32_t)(sweep_state.duration_sec * fs)) {
    sweep_state.elapsed_samples = 0;
    sweep_state.current_freq = sweep_state.start_freq;
  }

  float freq = sweep_state.current_freq;
  float phase_inc = 2.0f * (float)M_PI * freq / fs;
  sweep_state.phase += phase_inc;
  if (sweep_state.phase > 2.0f * (float)M_PI)
    sweep_state.phase -= 2.0f * (float)M_PI;

  sweep_state.current_freq *= sweep_state.freq_mult;
  sweep_state.elapsed_samples++;

  return fast_sine(sweep_state.phase);
}

// Generate one sample of warble tone: logarithmic sweep with ±5% FM @ 5 Hz.
// IEC 60118-7 standard warble for hearing aid testing. The FM wobbles the
// instantaneous freq around the sweep carrier, making narrow resonances
// easier to hear than a pure sweep.
static inline IRAM_ATTR float generate_warble(void) {
  const float fs = current_fs;
  const float lfo_freq = 5.0f;  // Hz
  const float fm_depth = 0.05f; // ±5%

  if (sweep_state.elapsed_samples >=
      (uint32_t)(sweep_state.duration_sec * fs)) {
    sweep_state.elapsed_samples = 0;
    sweep_state.current_freq = sweep_state.start_freq;
  }

  // Carrier freq (optimized exponential ramp)
  float carrier_freq = sweep_state.current_freq;

  // LFO for FM: 5 Hz sine wave
  float lfo_phase_inc = 2.0f * (float)M_PI * lfo_freq / fs;
  warble_state.lfo_phase += lfo_phase_inc;
  if (warble_state.lfo_phase > 2.0f * (float)M_PI)
    warble_state.lfo_phase -= 2.0f * (float)M_PI;
  float lfo = fast_sine(warble_state.lfo_phase);

  // Modulated freq: carrier ± 5%
  float mod_freq = carrier_freq * (1.0f + fm_depth * lfo);

  float carrier_phase_inc = 2.0f * (float)M_PI * mod_freq / fs;
  warble_state.carrier_phase += carrier_phase_inc;
  if (warble_state.carrier_phase > 2.0f * (float)M_PI)
    warble_state.carrier_phase -= 2.0f * (float)M_PI;

  sweep_state.current_freq *= sweep_state.freq_mult;
  sweep_state.elapsed_samples++;

  return fast_sine(warble_state.carrier_phase);
}

static IRAM_ATTR void process_limiter(limiter_state_t *lim, float *buf_l,
                                      float *buf_r, int count) {
  if (!lim->enabled)
    return;

  for (int i = 0; i < count; i++) {
    float abs_l = fabs_fast(buf_l[i]);
    float abs_r = fabs_fast(buf_r[i]);
    float max_peak = fmax_fast(abs_l, abs_r);

    if (max_peak > lim->envelope) {
      lim->envelope += lim->attack_coeff * (max_peak - lim->envelope);
    } else {
      lim->envelope += lim->release_coeff * (max_peak - lim->envelope);
    }

    float gain = 1.0f;
    if (lim->envelope > lim->threshold_lin) {
      gain = lim->threshold_lin / lim->envelope;
    }

    buf_l[i] *= gain;
    buf_r[i] *= gain;
  }
}

// DRAM-pinned: read/written every audio block by process_block + meters.
DRAM_ATTR static float block_l[I2S_BLOCK_FRAMES];
DRAM_ATTR static float block_r[I2S_BLOCK_FRAMES];

// ============================================================
// Crossover coefficient tables & calculation
// ============================================================

// Butterworth section Q values
static const float bw_q_12[] = {0.7071f};
static const float bw_q_24[] = {0.5412f, 1.3065f};
static const float bw_q_48[] = {0.5098f, 0.6013f, 0.9000f, 2.5628f};

// Linkwitz-Riley section Q values (LR is cascaded BW at same -6dB point)
static const float lr_q_12[] = {0.5000f};
static const float lr_q_24[] = {0.7071f, 0.7071f};
static const float lr_q_48[] = {0.5412f, 0.5412f, 1.3065f, 1.3065f};

// Bessel section Q values (maximally flat group delay)
static const float be_q_12[] = {0.5773f};
static const float be_q_24[] = {0.5219f, 0.8055f};
static const float be_q_48[] = {0.5006f, 0.5635f, 0.7104f, 1.0238f};

// Look up Q table + biquad count for (type, slope). Returns NULL for
// bypass/6dB.
static const float *xover_q_table(xover_filter_type_t type, xover_slope_t slope,
                                  int *num_biquads_out) {
  const float *t = NULL;
  int n = 0;
  switch (type) {
  case XOVER_BUTTERWORTH:
    if (slope == XOVER_SLOPE_12DB) {
      t = bw_q_12;
      n = 1;
    } else if (slope == XOVER_SLOPE_24DB) {
      t = bw_q_24;
      n = 2;
    } else if (slope == XOVER_SLOPE_48DB) {
      t = bw_q_48;
      n = 4;
    }
    break;
  case XOVER_LINKWITZ_RILEY:
    if (slope == XOVER_SLOPE_12DB) {
      t = lr_q_12;
      n = 1;
    } else if (slope == XOVER_SLOPE_24DB) {
      t = lr_q_24;
      n = 2;
    } else if (slope == XOVER_SLOPE_48DB) {
      t = lr_q_48;
      n = 4;
    }
    break;
  case XOVER_BESSEL:
    if (slope == XOVER_SLOPE_12DB) {
      t = be_q_12;
      n = 1;
    } else if (slope == XOVER_SLOPE_24DB) {
      t = be_q_24;
      n = 2;
    } else if (slope == XOVER_SLOPE_48DB) {
      t = be_q_48;
      n = 4;
    }
    break;
  default:
    break;
  }
  *num_biquads_out = n;
  return t;
}

// Standard RBJ biquad for HP/LP at cutoff fc with given Q.
// is_highpass: true = HP, false = LP. Coefficients normalized by a0.
static void calc_xover_biquad(biquad_coeffs_t *c, float fc, float q, float fs,
                              bool is_highpass) {
  float safe_fc = fc;
  if (safe_fc > fs * 0.49f)
    safe_fc = fs * 0.49f;
  if (safe_fc < 10.0f)
    safe_fc = 10.0f;

  // Rule 1: Bilinear prewarping for accuracy at low frequencies
  float K = tanf((float)M_PI * safe_fc / fs);
  float KK = K * K;
  float norm = 1.0f / (1.0f + K / q + KK);

  c->is_svf = false; // Crossover always uses TDF2 for SIMD performance
  if (is_highpass) {
    c->b0 = norm;
    c->b1 = -2.0f * norm;
    c->b2 = norm;
  } else {
    c->b0 = KK * norm;
    c->b1 = 2.0f * KK * norm;
    c->b2 = KK * norm;
  }
  c->a1 = 2.0f * (KK - 1.0f) * norm;
  c->a2 = (1.0f - K / q + KK) * norm;
}

// One-pole IIR coefficient for 6dB/oct. We ALWAYS store the LP alpha
// (coeff = wc/(1+wc)); HP is implemented in xover_process_hp_sample() as
// x - LP(x), which is exactly equivalent to a first-order HP but needs only
// one state variable instead of two (no need to track previous input).
static void calc_onepole(float *coeff, float fc, float fs) {
  float safe_fc = fc;
  if (safe_fc > fs * 0.49f)
    safe_fc = fs * 0.49f;
  if (safe_fc < 10.0f)
    safe_fc = 10.0f;
  float w = 2.0f * (float)M_PI * safe_fc / fs;
  *coeff = w / (1.0f + w);
}

// Pass-through coefficients (unity gain) — used when xover is disabled/bypass.
static inline biquad_coeffs_t xover_passthrough_coeffs(void) {
  biquad_coeffs_t c = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  return c;
}

// Compute target (pending) coefficients for a crossover filter from settings.
// Writes into *_pending members and sets the dirty flag. Called from
// xover_update_hp / xover_update_lp, always outside the audio task.
static void xover_build_pending(xover_filter_t *xf, xover_band_t *settings,
                                bool is_highpass, float fs) {
  taskENTER_CRITICAL(&dsp_mux);

  bool en = settings->enabled && settings->type != XOVER_BYPASS;
  xf->enabled_pending = en;

  if (!en) {
    // Bypass — everything pass-through, single-biquad topology for simplicity
    xf->biquad_pending[0] = xover_passthrough_coeffs();
    for (int i = 1; i < XOVER_MAX_BIQUADS; i++)
      xf->biquad_pending[i] = xover_passthrough_coeffs();
    xf->onepole_coeff_pending = 0.0f;
    xf->num_biquads_pending = 0;
    xf->is_onepole_pending = false;
    xf->dirty = true;
    taskEXIT_CRITICAL(&dsp_mux);
    return;
  }

  if (settings->slope == XOVER_SLOPE_6DB) {
    calc_onepole(&xf->onepole_coeff_pending, settings->freq, fs);
    xf->is_onepole_pending = true;
    xf->num_biquads_pending = 0;
    // Fill unused biquads with pass-through for safety
    for (int i = 0; i < XOVER_MAX_BIQUADS; i++)
      xf->biquad_pending[i] = xover_passthrough_coeffs();
  } else {
    int num = 0;
    const float *q_table = xover_q_table(settings->type, settings->slope, &num);
    if (q_table == NULL || num == 0) {
      // Shouldn't happen with valid enum, but be safe
      for (int i = 0; i < XOVER_MAX_BIQUADS; i++)
        xf->biquad_pending[i] = xover_passthrough_coeffs();
      xf->is_onepole_pending = false;
      xf->num_biquads_pending = 0;
      xf->enabled_pending = false;
    } else {
      for (int i = 0; i < num; i++) {
        calc_xover_biquad(&xf->biquad_pending[i], settings->freq, q_table[i],
                          fs, is_highpass);
      }
      for (int i = num; i < XOVER_MAX_BIQUADS; i++)
        xf->biquad_pending[i] = xover_passthrough_coeffs();
      xf->is_onepole_pending = false;
      xf->num_biquads_pending = num;
      xf->freq_target = settings->freq;
      for (int i = 0; i < num; i++)
        xf->q_target[i] = q_table[i];
    }
    xf->onepole_coeff_pending = 0.0f;
  }

  xf->dirty = true;
  taskEXIT_CRITICAL(&dsp_mux);
}

// Audio-side: atomically copy pending → target, start crossfade.
// If topology changed (num_biquads or is_onepole differ from live), zero the
// filter states and snap immediately rather than crossfade — crossfading
// between different topologies produces unstable output.
static inline IRAM_ATTR void xover_sync(xover_filter_t *xf) {
  taskENTER_CRITICAL(&dsp_mux);
  if (!xf->dirty) {
    taskEXIT_CRITICAL(&dsp_mux);
    return;
  }

  bool topology_changed = (xf->num_biquads_pending != xf->num_biquads) ||
                          (xf->is_onepole_pending != xf->is_onepole) ||
                          (xf->enabled_pending != xf->enabled);

  for (int i = 0; i < XOVER_MAX_BIQUADS; i++) {
    xf->biquad_target[i] = xf->biquad_pending[i];
  }
  xf->onepole_coeff_target = xf->onepole_coeff_pending;

  if (topology_changed) {
    xf->num_biquads = xf->num_biquads_pending;
    xf->is_onepole = xf->is_onepole_pending;
    xf->enabled = xf->enabled_pending;
    xf->freq_cur = xf->freq_target;
    for (int i = 0; i < XOVER_MAX_BIQUADS; i++) {
      xf->q_cur[i] = xf->q_target[i];
      calc_xover_biquad(&xf->biquad_coeffs[i], xf->freq_cur, xf->q_cur[i],
                        current_fs, xf->is_highpass);
      memset(&xf->biquad_state_l[i], 0, sizeof(biquad_state_t));
      memset(&xf->biquad_state_r[i], 0, sizeof(biquad_state_t));
    }
    xf->onepole_coeff = xf->onepole_coeff_target;
    xf->onepole_state_l.z1 = 0.0f;
    xf->onepole_state_r.z1 = 0.0f;
    xf->crossfade_samples = 0;
  } else {
    xf->crossfade_samples = CROSSFADE_SAMPLES;
  }

  xf->dirty = false;
  taskEXIT_CRITICAL(&dsp_mux);
}

// Sub-block crossover coefficient interpolation
static inline IRAM_ATTR void xover_interpolate(xover_filter_t *xf, int n) {
  if (xf->crossfade_samples <= 0)
    return;
  xf->crossfade_samples -= n;

  // Rule 3: Parameter-space interpolation
  xf->freq_cur += COEFF_SMOOTH_LOWBLOCK * (xf->freq_target - xf->freq_cur);
  for (int i = 0; i < xf->num_biquads; i++) {
    xf->q_cur[i] += COEFF_SMOOTH_LOWBLOCK * (xf->q_target[i] - xf->q_cur[i]);
    calc_xover_biquad(&xf->biquad_coeffs[i], xf->freq_cur, xf->q_cur[i],
                      current_fs, xf->is_highpass);
  }

  xf->onepole_coeff +=
      COEFF_SMOOTH_LOWBLOCK * (xf->onepole_coeff_target - xf->onepole_coeff);

  if (xf->crossfade_samples <= 0) {
    xf->crossfade_samples = 0;
    xf->freq_cur = xf->freq_target;
    for (int i = 0; i < xf->num_biquads; i++) {
      xf->q_cur[i] = xf->q_target[i];
      calc_xover_biquad(&xf->biquad_coeffs[i], xf->freq_cur, xf->q_cur[i],
                        current_fs, xf->is_highpass);
    }
    xf->onepole_coeff = xf->onepole_coeff_target;
  }
}

// Typed one-pole helpers — always use the LP alpha form (wc/(1+wc)) internally;
// HP is derived as x - LP(x). Mathematically identical to a textbook 1st-order
// HP but needs only one state variable instead of two.
static inline IRAM_ATTR float onepole_lp_step(onepole_state_t *s, float x,
                                              float alpha) {
  s->z1 += alpha * (x - s->z1);
  return s->z1;
}

// ============================================================
// Delay line (shared ring buffer, dual read points for crossfade)
// ============================================================
static void delay_line_init(delay_line_t *d) {
  memset(d->buffer_l, 0, sizeof(d->buffer_l));
  memset(d->buffer_r, 0, sizeof(d->buffer_r));
  d->write_idx = 0;
  d->delay_cur = 0;
  d->delay_target = 0;
  d->crossfade_counter = 0;
}

static inline IRAM_ATTR void delay_line_process_steady(delay_line_t *d,
                                                       float *buf_l,
                                                       float *buf_r,
                                                       int count) {
  int write_idx = d->write_idx;
  int delay_cur = d->delay_cur;
  for (int i = 0; i < count; i++) {
    d->buffer_l[write_idx] = buf_l[i];
    d->buffer_r[write_idx] = buf_r[i];

    int read_idx = write_idx - delay_cur;
    // Branchless wrap: DELAY_MAX_SAMPLES is 384, not power of 2, so we use
    // ternary (Still a branch but better than if/else if compiler can cmov it)
    // Actually, let's use a safe wrap for any size:
    read_idx = (read_idx < 0) ? read_idx + DELAY_MAX_SAMPLES : read_idx;

    buf_l[i] = d->buffer_l[read_idx];
    buf_r[i] = d->buffer_r[read_idx];

    write_idx++;
    if (write_idx >= DELAY_MAX_SAMPLES)
      write_idx = 0;
  }
  d->write_idx = write_idx;
}

static inline IRAM_ATTR void delay_line_process_fade(delay_line_t *d,
                                                     float *buf_l, float *buf_r,
                                                     int count) {
  int write_idx = d->write_idx;
  int delay_cur = d->delay_cur;
  int delay_target = d->delay_target;
  int fade = d->crossfade_counter;

  for (int i = 0; i < count; i++) {
    d->buffer_l[write_idx] = buf_l[i];
    d->buffer_r[write_idx] = buf_r[i];

    int r_cur = write_idx - delay_cur;
    r_cur = (r_cur < 0) ? r_cur + DELAY_MAX_SAMPLES : r_cur;
    int r_tgt = write_idx - delay_target;
    r_tgt = (r_tgt < 0) ? r_tgt + DELAY_MAX_SAMPLES : r_tgt;

    float t = 1.0f - ((float)fade / (float)CROSSFADE_SAMPLES);
    buf_l[i] = (1.0f - t) * d->buffer_l[r_cur] + t * d->buffer_l[r_tgt];
    buf_r[i] = (1.0f - t) * d->buffer_r[r_cur] + t * d->buffer_r[r_tgt];

    if (fade > 0)
      fade--;
    write_idx++;
    if (write_idx >= DELAY_MAX_SAMPLES)
      write_idx = 0;
  }
  d->write_idx = write_idx;
  d->crossfade_counter = fade;
  if (fade == 0)
    d->delay_cur = delay_target;
}

static inline IRAM_ATTR void delay_line_process(delay_line_t *d, float *buf_l,
                                                float *buf_r, int count) {
  if (d->delay_cur == 0 && d->delay_target == 0 && d->crossfade_counter == 0)
    return;

  // Select path outside the loop (Rule 11)
  if (d->crossfade_counter > 0) {
    delay_line_process_fade(d, buf_l, buf_r, count);
  } else {
    delay_line_process_steady(d, buf_l, buf_r, count);
  }
}

static IRAM_ATTR void process_block(float *buf_l, float *buf_r, int count) {
  const int in_l = stage_input_dsp.num_active_bands_l;
  const int in_r = stage_input_dsp.num_active_bands_r;
  const int out_l = stage_output_dsp.num_active_bands_l;
  const int out_r = stage_output_dsp.num_active_bands_r;
  const int low_bl = stage_low_dsp.num_active_bands_l;
  const int low_br = stage_low_dsp.num_active_bands_r;
  const bool mono_flag = low_mono;
  const bool phase_flag = low_phase_invert;
  const bool xover_active = xover_hp.enabled || xover_lp.enabled;

  static float denorm_sign = 1.0f;
  for (int i = 0; i < count; i++) {
    buf_l[i] += ANTI_DENORMAL_MAG * denorm_sign;
    buf_r[i] += ANTI_DENORMAL_MAG * denorm_sign;
    denorm_sign = -denorm_sign;
  }

  for (int off = 0; off < count; off += LOW_BLOCK) {
    int n = count - off;
    if (n > LOW_BLOCK)
      n = LOW_BLOCK;

    // Interpolation happens at the END of the block to ensure the current loop
    // uses the coefficients and crossfade state from the START of the block.
    // This prevents skipping the final handover block of a transition.

    float xf_buf_l[LOW_BLOCK];
    float xf_buf_r[LOW_BLOCK];

    for (int i = 0; i < n; i++) {
      input_gain_current_l +=
          GAIN_SMOOTH_COEFF * (input_gain_target_l - input_gain_current_l);
      input_gain_current_r +=
          GAIN_SMOOTH_COEFF * (input_gain_target_r - input_gain_current_r);
      buf_l[off + i] *= input_gain_current_l;
      buf_r[off + i] *= input_gain_current_r;
    }

    // --- INPUT STAGE EQ ---
    dsp_stage_t *stage = &stage_input_dsp;
    for (int b = 0; b < MAX_BANDS; b++) {
      // Process Left Channel
      int idx_l = stage->sorted_idx_l[b];
      bool l_bypass = (fabsf(stage->cur_params_l[idx_l].gain) < 0.01f) &&
                      (stage->xf_topo_samples_l[idx_l] <= 0) &&
                      (stage->xf_warmup_samples_l[idx_l] <= 0);
      if (!l_bypass) {
        if (stage->xf_warmup_samples_l[idx_l] > 0) {
          // Silent pre-warm: run xf filter on scratch, but keep output silent.
          // Scalar path is fine here — output is discarded, no audible artifact.
          memcpy(xf_buf_l, buf_l + off, n * sizeof(float));
          if (stage->xf_coeffs_l[idx_l].is_svf)
            svf_block_scalar(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                             &stage->xf_state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(xf_buf_l, xf_buf_l, n,
                                     &stage->xf_coeffs_l[idx_l],
                                     &stage->xf_state_l[idx_l]);

          // Run main filter normally
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(buf_l + off, buf_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
        } else if (stage->xf_topo_samples_l[idx_l] > 0) {
          float samples_left = (float)stage->xf_topo_samples_l[idx_l];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_l, buf_l + off, n * sizeof(float));
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(buf_l + off, buf_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
          {
            // Incoming (target) filter: predict end-of-block coefficients and
            // use interp path so its parameters glide per-sample rather than
            // stepping every LOW_BLOCK during active dragging.
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_l[idx_l];
            const eq_band_t *xf_tgt = &stage->target_params_l[idx_l];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_l[idx_l]);
            if (stage->xf_coeffs_l[idx_l].is_svf)
              svf_block_interp(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                               &xf_end_c, &stage->xf_state_l[idx_l]);
            else
              biquad_block_tdf2_interp(xf_buf_l, xf_buf_l, n,
                                       &stage->xf_coeffs_l[idx_l], &xf_end_c,
                                       &stage->xf_state_l[idx_l]);
          }

          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            buf_l[off + i] = buf_l[off + i] * (1.0f - wt) + xf_buf_l[i] * wt;
          }
        } else {
          bool _band_moving_l = (fabsf(stage->cur_params_l[idx_l].freq - stage->target_params_l[idx_l].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].gain - stage->target_params_l[idx_l].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].q    - stage->target_params_l[idx_l].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_l) {
            // Predict end-of-block coefficients by simulating the lerp that
            // interpolate_coeffs will apply after this block, then ramp
            // per-sample — continuous coefficient trajectory, no staircase step.
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_l[idx_l];
            const eq_band_t *tgt = &stage->target_params_l[idx_l];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_l[idx_l].is_svf);
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_interp(buf_l + off, buf_l + off, n,
                               &stage->coeffs_l[idx_l], &end_c,
                               &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_interp(buf_l + off, buf_l + off, n,
                                       &stage->coeffs_l[idx_l], &end_c,
                                       &stage->state_l[idx_l]);
          } else {
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_scalar(buf_l + off, buf_l + off, n,
                               &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                       &stage->coeffs_l[idx_l],
                                       &stage->state_l[idx_l]);
          }
        }
      }

      // Process Right Channel
      int idx_r = stage->sorted_idx_r[b];
      bool r_bypass = (fabsf(stage->cur_params_r[idx_r].gain) < 0.01f) &&
                      (stage->xf_topo_samples_r[idx_r] <= 0) &&
                      (stage->xf_warmup_samples_r[idx_r] <= 0);
      if (!r_bypass) {
        if (stage->xf_warmup_samples_r[idx_r] > 0) {
          // Warmup: output discarded, scalar is sufficient
          memcpy(xf_buf_r, buf_r + off, n * sizeof(float));
          if (stage->xf_coeffs_r[idx_r].is_svf)
            svf_block_scalar(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                             &stage->xf_state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(xf_buf_r, xf_buf_r, n,
                                     &stage->xf_coeffs_r[idx_r],
                                     &stage->xf_state_r[idx_r]);
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(buf_r + off, buf_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
        } else if (stage->xf_topo_samples_r[idx_r] > 0) {
          float samples_left = (float)stage->xf_topo_samples_r[idx_r];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_r, buf_r + off, n * sizeof(float));
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(buf_r + off, buf_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
          {
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_r[idx_r];
            const eq_band_t *xf_tgt = &stage->target_params_r[idx_r];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_r[idx_r]);
            if (stage->xf_coeffs_r[idx_r].is_svf)
              svf_block_interp(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                               &xf_end_c, &stage->xf_state_r[idx_r]);
            else
              biquad_block_tdf2_interp(xf_buf_r, xf_buf_r, n,
                                       &stage->xf_coeffs_r[idx_r], &xf_end_c,
                                       &stage->xf_state_r[idx_r]);
          }
          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            buf_r[off + i] = buf_r[off + i] * (1.0f - wt) + xf_buf_r[i] * wt;
          }
        } else {
          bool _band_moving_r = (fabsf(stage->cur_params_r[idx_r].freq - stage->target_params_r[idx_r].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].gain - stage->target_params_r[idx_r].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].q    - stage->target_params_r[idx_r].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_r) {
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_r[idx_r];
            const eq_band_t *tgt = &stage->target_params_r[idx_r];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_r[idx_r].is_svf);
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_interp(buf_r + off, buf_r + off, n,
                               &stage->coeffs_r[idx_r], &end_c,
                               &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_interp(buf_r + off, buf_r + off, n,
                                       &stage->coeffs_r[idx_r], &end_c,
                                       &stage->state_r[idx_r]);
          } else {
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_scalar(buf_r + off, buf_r + off, n,
                               &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                       &stage->coeffs_r[idx_r],
                                       &stage->state_r[idx_r]);
          }
        }
      }
    }

    // --- INPUT MUTE GATE (ramped, click-free) ---
    {
      float target = dsp_stage_muted[0] ? 0.0f : 1.0f;
      for (int i = 0; i < n; i++) {
        mute_gain_current[0] += GAIN_SMOOTH_COEFF * (target - mute_gain_current[0]);
        buf_l[off + i] *= mute_gain_current[0];
        buf_r[off + i] *= mute_gain_current[0];
      }
    }

    // --- OUTPUT STAGE EQ ---
    stage = &stage_output_dsp;
    for (int b = 0; b < MAX_BANDS; b++) {
      // Left
      int idx_l = stage->sorted_idx_l[b];
      bool l_bypass = (fabsf(stage->cur_params_l[idx_l].gain) < 0.01f) &&
                      (stage->xf_topo_samples_l[idx_l] <= 0) &&
                      (stage->xf_warmup_samples_l[idx_l] <= 0);
      if (!l_bypass) {
        if (stage->xf_warmup_samples_l[idx_l] > 0) {
          // Warmup: output discarded, scalar is sufficient
          memcpy(xf_buf_l, buf_l + off, n * sizeof(float));
          if (stage->xf_coeffs_l[idx_l].is_svf)
            svf_block_scalar(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                             &stage->xf_state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(xf_buf_l, xf_buf_l, n,
                                     &stage->xf_coeffs_l[idx_l],
                                     &stage->xf_state_l[idx_l]);
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(buf_l + off, buf_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
        } else if (stage->xf_topo_samples_l[idx_l] > 0) {
          float samples_left = (float)stage->xf_topo_samples_l[idx_l];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_l, buf_l + off, n * sizeof(float));
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(buf_l + off, buf_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
          {
            // Incoming (target) filter: predict end-of-block coefficients and
            // use interp path so its parameters glide per-sample rather than
            // stepping every LOW_BLOCK during active dragging.
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_l[idx_l];
            const eq_band_t *xf_tgt = &stage->target_params_l[idx_l];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_l[idx_l]);
            if (stage->xf_coeffs_l[idx_l].is_svf)
              svf_block_interp(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                               &xf_end_c, &stage->xf_state_l[idx_l]);
            else
              biquad_block_tdf2_interp(xf_buf_l, xf_buf_l, n,
                                       &stage->xf_coeffs_l[idx_l], &xf_end_c,
                                       &stage->xf_state_l[idx_l]);
          }
          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            buf_l[off + i] = buf_l[off + i] * (1.0f - wt) + xf_buf_l[i] * wt;
          }
        } else {
          bool _band_moving_l = (fabsf(stage->cur_params_l[idx_l].freq - stage->target_params_l[idx_l].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].gain - stage->target_params_l[idx_l].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].q    - stage->target_params_l[idx_l].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_l) {
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_l[idx_l];
            const eq_band_t *tgt = &stage->target_params_l[idx_l];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_l[idx_l].is_svf);
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_interp(buf_l + off, buf_l + off, n,
                               &stage->coeffs_l[idx_l], &end_c,
                               &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_interp(buf_l + off, buf_l + off, n,
                                       &stage->coeffs_l[idx_l], &end_c,
                                       &stage->state_l[idx_l]);
          } else {
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_scalar(buf_l + off, buf_l + off, n,
                               &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_scalar(buf_l + off, buf_l + off, n,
                                       &stage->coeffs_l[idx_l],
                                       &stage->state_l[idx_l]);
          }
        }
      }

      // Right
      int idx_r = stage->sorted_idx_r[b];
      bool r_bypass = (fabsf(stage->cur_params_r[idx_r].gain) < 0.01f) &&
                      (stage->xf_topo_samples_r[idx_r] <= 0) &&
                      (stage->xf_warmup_samples_r[idx_r] <= 0);
      if (!r_bypass) {
        if (stage->xf_warmup_samples_r[idx_r] > 0) {
          // Warmup: output discarded, scalar is sufficient
          memcpy(xf_buf_r, buf_r + off, n * sizeof(float));
          if (stage->xf_coeffs_r[idx_r].is_svf)
            svf_block_scalar(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                             &stage->xf_state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(xf_buf_r, xf_buf_r, n,
                                     &stage->xf_coeffs_r[idx_r],
                                     &stage->xf_state_r[idx_r]);
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(buf_r + off, buf_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
        } else if (stage->xf_topo_samples_r[idx_r] > 0) {
          float samples_left = (float)stage->xf_topo_samples_r[idx_r];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_r, buf_r + off, n * sizeof(float));
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(buf_r + off, buf_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
          {
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_r[idx_r];
            const eq_band_t *xf_tgt = &stage->target_params_r[idx_r];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_r[idx_r]);
            if (stage->xf_coeffs_r[idx_r].is_svf)
              svf_block_interp(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                               &xf_end_c, &stage->xf_state_r[idx_r]);
            else
              biquad_block_tdf2_interp(xf_buf_r, xf_buf_r, n,
                                       &stage->xf_coeffs_r[idx_r], &xf_end_c,
                                       &stage->xf_state_r[idx_r]);
          }

          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            buf_r[off + i] = buf_r[off + i] * (1.0f - wt) + xf_buf_r[i] * wt;
          }
        } else {
          bool _band_moving_r = (fabsf(stage->cur_params_r[idx_r].freq - stage->target_params_r[idx_r].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].gain - stage->target_params_r[idx_r].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].q    - stage->target_params_r[idx_r].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_r) {
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_r[idx_r];
            const eq_band_t *tgt = &stage->target_params_r[idx_r];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_r[idx_r].is_svf);
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_interp(buf_r + off, buf_r + off, n,
                               &stage->coeffs_r[idx_r], &end_c,
                               &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_interp(buf_r + off, buf_r + off, n,
                                       &stage->coeffs_r[idx_r], &end_c,
                                       &stage->state_r[idx_r]);
          } else {
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_scalar(buf_r + off, buf_r + off, n,
                               &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_scalar(buf_r + off, buf_r + off, n,
                                       &stage->coeffs_r[idx_r],
                                       &stage->state_r[idx_r]);
          }
        }
      }
    }

    memcpy(&low_block_l[off], &buf_l[off], n * sizeof(float));
    memcpy(&low_block_r[off], &buf_r[off], n * sizeof(float));

    if (xover_hp.enabled) {
      if (xover_hp.is_onepole) {
        for (int i = 0; i < n; i++) {
          float lp_l = onepole_lp_step(&xover_hp.onepole_state_l,
                                       buf_l[off + i], xover_hp.onepole_coeff);
          buf_l[off + i] -= lp_l;
          float lp_r = onepole_lp_step(&xover_hp.onepole_state_r,
                                       buf_r[off + i], xover_hp.onepole_coeff);
          buf_r[off + i] -= lp_r;
        }
      } else {
        for (int b = 0; b < xover_hp.num_biquads; b++) {
          biquad_block_tdf2_stereo(
              buf_l + off, buf_l + off, buf_r + off, buf_r + off, n,
              &xover_hp.biquad_coeffs[b], &xover_hp.biquad_state_l[b],
              &xover_hp.biquad_coeffs[b], &xover_hp.biquad_state_r[b]);
        }
      }
    }

    if (xover_lp.enabled) {
      if (xover_lp.is_onepole) {
        for (int i = 0; i < n; i++) {
          low_block_l[off + i] =
              onepole_lp_step(&xover_lp.onepole_state_l, low_block_l[off + i],
                              xover_lp.onepole_coeff);
          low_block_r[off + i] =
              onepole_lp_step(&xover_lp.onepole_state_r, low_block_r[off + i],
                              xover_lp.onepole_coeff);
        }
      } else {
        for (int b = 0; b < xover_lp.num_biquads; b++) {
          biquad_block_tdf2_stereo(
              low_block_l + off, low_block_l + off, low_block_r + off,
              low_block_r + off, n, &xover_lp.biquad_coeffs[b],
              &xover_lp.biquad_state_l[b], &xover_lp.biquad_coeffs[b],
              &xover_lp.biquad_state_r[b]);
        }
      }
    }

    if (mono_flag) {
      for (int i = 0; i < n; i++) {
        float m = (low_block_l[off + i] + low_block_r[off + i]) * 0.5f;
        low_block_l[off + i] = m;
        low_block_r[off + i] = m;
      }
    }
    if (phase_flag) {
      for (int i = 0; i < n; i++) {
        low_block_l[off + i] = -low_block_l[off + i];
        low_block_r[off + i] = -low_block_r[off + i];
      }
    }

    // --- LOW STAGE EQ ---
    stage = &stage_low_dsp;
    for (int b = 0; b < MAX_BANDS; b++) {
      // Left
      int idx_l = stage->sorted_idx_l[b];
      bool l_bypass = (fabsf(stage->cur_params_l[idx_l].gain) < 0.01f) &&
                      (stage->xf_topo_samples_l[idx_l] <= 0) &&
                      (stage->xf_warmup_samples_l[idx_l] <= 0);
      if (!l_bypass) {
        if (stage->xf_warmup_samples_l[idx_l] > 0) {
          // Warmup: output discarded, scalar is sufficient
          memcpy(xf_buf_l, low_block_l + off, n * sizeof(float));
          if (stage->xf_coeffs_l[idx_l].is_svf)
            svf_block_scalar(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                             &stage->xf_state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(xf_buf_l, xf_buf_l, n,
                                     &stage->xf_coeffs_l[idx_l],
                                     &stage->xf_state_l[idx_l]);
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(low_block_l + off, low_block_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(low_block_l + off, low_block_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
        } else if (stage->xf_topo_samples_l[idx_l] > 0) {
          // Audible crossfade: ramp Target from 0.0 -> 1.0
          float samples_left = (float)stage->xf_topo_samples_l[idx_l];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_l, low_block_l + off, n * sizeof(float));
          if (stage->coeffs_l[idx_l].is_svf)
            svf_block_scalar(low_block_l + off, low_block_l + off, n,
                             &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
          else
            biquad_block_tdf2_scalar(low_block_l + off, low_block_l + off, n,
                                     &stage->coeffs_l[idx_l],
                                     &stage->state_l[idx_l]);
          {
            // Incoming (target) filter: predict end-of-block coefficients and
            // use interp path so its parameters glide per-sample rather than
            // stepping every LOW_BLOCK during active dragging.
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_l[idx_l];
            const eq_band_t *xf_tgt = &stage->target_params_l[idx_l];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_l[idx_l]);
            if (stage->xf_coeffs_l[idx_l].is_svf)
              svf_block_interp(xf_buf_l, xf_buf_l, n, &stage->xf_coeffs_l[idx_l],
                               &xf_end_c, &stage->xf_state_l[idx_l]);
            else
              biquad_block_tdf2_interp(xf_buf_l, xf_buf_l, n,
                                       &stage->xf_coeffs_l[idx_l], &xf_end_c,
                                       &stage->xf_state_l[idx_l]);
          }
                                     
          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            low_block_l[off + i] = low_block_l[off + i] * (1.0f - wt) + xf_buf_l[i] * wt;
          }
        } else {
          bool _band_moving_l = (fabsf(stage->cur_params_l[idx_l].freq - stage->target_params_l[idx_l].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].gain - stage->target_params_l[idx_l].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_l[idx_l].q    - stage->target_params_l[idx_l].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_l) {
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_l[idx_l];
            const eq_band_t *tgt = &stage->target_params_l[idx_l];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_l[idx_l].is_svf);
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_interp(low_block_l + off, low_block_l + off, n,
                               &stage->coeffs_l[idx_l], &end_c,
                               &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_interp(low_block_l + off, low_block_l + off, n,
                                       &stage->coeffs_l[idx_l], &end_c,
                                       &stage->state_l[idx_l]);
          } else {
            if (stage->coeffs_l[idx_l].is_svf)
              svf_block_scalar(low_block_l + off, low_block_l + off, n,
                               &stage->coeffs_l[idx_l], &stage->state_l[idx_l]);
            else
              biquad_block_tdf2_scalar(low_block_l + off, low_block_l + off, n,
                                       &stage->coeffs_l[idx_l],
                                       &stage->state_l[idx_l]);
          }
        }
      }

      // Right
      int idx_r = stage->sorted_idx_r[b];
      bool r_bypass = (fabsf(stage->cur_params_r[idx_r].gain) < 0.01f) &&
                      (stage->xf_topo_samples_r[idx_r] <= 0) &&
                      (stage->xf_warmup_samples_r[idx_r] <= 0);
      if (!r_bypass) {
        if (stage->xf_warmup_samples_r[idx_r] > 0) {
          // Warmup: output discarded, scalar is sufficient
          memcpy(xf_buf_r, low_block_r + off, n * sizeof(float));
          if (stage->xf_coeffs_r[idx_r].is_svf)
            svf_block_scalar(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                             &stage->xf_state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(xf_buf_r, xf_buf_r, n,
                                     &stage->xf_coeffs_r[idx_r],
                                     &stage->xf_state_r[idx_r]);
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(low_block_r + off, low_block_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(low_block_r + off, low_block_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
        } else if (stage->xf_topo_samples_r[idx_r] > 0) {
          float samples_left = (float)stage->xf_topo_samples_r[idx_r];
          float w_tgt_start = 1.0f - (samples_left / SVF_XFADE_SAMPLES);
          float w_tgt_end   = 1.0f - ((samples_left - n) / SVF_XFADE_SAMPLES);
          float w_tgt_step  = (w_tgt_end - w_tgt_start) / (float)n;

          memcpy(xf_buf_r, low_block_r + off, n * sizeof(float));
          if (stage->coeffs_r[idx_r].is_svf)
            svf_block_scalar(low_block_r + off, low_block_r + off, n,
                             &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
          else
            biquad_block_tdf2_scalar(low_block_r + off, low_block_r + off, n,
                                     &stage->coeffs_r[idx_r],
                                     &stage->state_r[idx_r]);
          {
            biquad_coeffs_t xf_end_c;
            eq_band_t xf_pred = stage->cur_params_r[idx_r];
            const eq_band_t *xf_tgt = &stage->target_params_r[idx_r];
            xf_pred.freq += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->freq - xf_pred.freq);
            xf_pred.gain += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->gain - xf_pred.gain);
            xf_pred.q    += COEFF_SMOOTH_LOWBLOCK * (xf_tgt->q    - xf_pred.q);
            calculate_biquad_forced(&xf_end_c, &xf_pred, current_fs,
                                    stage->xf_topo_to_svf_r[idx_r]);
            if (stage->xf_coeffs_r[idx_r].is_svf)
              svf_block_interp(xf_buf_r, xf_buf_r, n, &stage->xf_coeffs_r[idx_r],
                               &xf_end_c, &stage->xf_state_r[idx_r]);
            else
              biquad_block_tdf2_interp(xf_buf_r, xf_buf_r, n,
                                       &stage->xf_coeffs_r[idx_r], &xf_end_c,
                                       &stage->xf_state_r[idx_r]);
          }

          for (int i = 0; i < n; i++) {
            float wt = w_tgt_start + (float)i * w_tgt_step;
            wt = fmaxf(0.0f, fminf(1.0f, wt));
            low_block_r[off + i] = low_block_r[off + i] * (1.0f - wt) + xf_buf_r[i] * wt;
          }
        } else {
          bool _band_moving_r = (fabsf(stage->cur_params_r[idx_r].freq - stage->target_params_r[idx_r].freq) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].gain - stage->target_params_r[idx_r].gain) > COEFF_SMOOTH_EPSILON ||
                              fabsf(stage->cur_params_r[idx_r].q    - stage->target_params_r[idx_r].q)    > COEFF_SMOOTH_EPSILON);
          if (stage->crossfade_samples > 0 && _band_moving_r) {
            biquad_coeffs_t end_c;
            eq_band_t pred = stage->cur_params_r[idx_r];
            const eq_band_t *tgt = &stage->target_params_r[idx_r];
            pred.freq += COEFF_SMOOTH_LOWBLOCK * (tgt->freq - pred.freq);
            pred.gain += COEFF_SMOOTH_LOWBLOCK * (tgt->gain - pred.gain);
            pred.q    += COEFF_SMOOTH_LOWBLOCK * (tgt->q    - pred.q);
            calculate_biquad_forced(&end_c, &pred, current_fs,
                                    stage->coeffs_r[idx_r].is_svf);
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_interp(low_block_r + off, low_block_r + off, n,
                               &stage->coeffs_r[idx_r], &end_c,
                               &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_interp(low_block_r + off, low_block_r + off, n,
                                       &stage->coeffs_r[idx_r], &end_c,
                                       &stage->state_r[idx_r]);
          } else {
            if (stage->coeffs_r[idx_r].is_svf)
              svf_block_scalar(low_block_r + off, low_block_r + off, n,
                               &stage->coeffs_r[idx_r], &stage->state_r[idx_r]);
            else
              biquad_block_tdf2_scalar(low_block_r + off, low_block_r + off, n,
                                       &stage->coeffs_r[idx_r],
                                       &stage->state_r[idx_r]);
          }
        }
      }
    }

    for (int i = 0; i < n; i++) {
      output_gain_current_l +=
          GAIN_SMOOTH_COEFF * (output_gain_target_l - output_gain_current_l);
      output_gain_current_r +=
          GAIN_SMOOTH_COEFF * (output_gain_target_r - output_gain_current_r);
      low_gain_current_l +=
          GAIN_SMOOTH_COEFF * (low_gain_target_l - low_gain_current_l);
      low_gain_current_r +=
          GAIN_SMOOTH_COEFF * (low_gain_target_r - low_gain_current_r);
      // Output and low mute ramps — click-free, same smoother as gain
      mute_gain_current[1] += GAIN_SMOOTH_COEFF * ((dsp_stage_muted[1] ? 0.0f : 1.0f) - mute_gain_current[1]);
      mute_gain_current[2] += GAIN_SMOOTH_COEFF * ((dsp_stage_muted[2] ? 0.0f : 1.0f) - mute_gain_current[2]);
      buf_l[off + i]       *= output_gain_current_l * mute_gain_current[1];
      buf_r[off + i]       *= output_gain_current_r * mute_gain_current[1];
      low_block_l[off + i] *= low_gain_current_l    * mute_gain_current[2];
      low_block_r[off + i] *= low_gain_current_r    * mute_gain_current[2];
    }

    float fft_active_f = (fft_enabled && !storage_busy) ? 1.0f : 0.0f;
    float xo_active_f = xover_active ? 1.0f : 0.0f;
    for (int i = 0; i < n; i++) {
      float mono_main = (buf_l[off + i] + buf_r[off + i]) * 0.5f;
      float mono_full = (buf_l[off + i] + buf_r[off + i] +
                         low_block_l[off + i] + low_block_r[off + i]) *
                        0.25f;
      float mono = (1.0f - xo_active_f) * mono_main + xo_active_f * mono_full;
      int can_write = (int)fft_active_f & (dsp_analysis_ptr < FFT_SIZE);
      dsp_fft_write_buf[dsp_analysis_ptr] = mono;
      dsp_analysis_ptr += can_write;
      // Scope ring buffer — always filled regardless of FFT enable state
      scope_ring_buf[scope_ring_idx & (SCOPE_BUF_SIZE - 1)] = mono;
      scope_ring_idx++;
    }
    interpolate_coeffs(&stage_input_dsp, n);
    interpolate_coeffs(&stage_output_dsp, n);
    interpolate_coeffs(&stage_low_dsp, n);
    xover_interpolate(&xover_hp, n);
    xover_interpolate(&xover_lp, n);
  }

  for (int i = 0; i < count; i++) {
    if (!is_finite_fast(buf_l[i]) || !is_finite_fast(buf_r[i]) ||
        !is_finite_fast(low_block_l[i]) || !is_finite_fast(low_block_r[i])) {
      memset(stage_input_dsp.state_l, 0, sizeof(stage_input_dsp.state_l));
      memset(stage_input_dsp.state_r, 0, sizeof(stage_input_dsp.state_r));
      memset(stage_output_dsp.state_l, 0, sizeof(stage_output_dsp.state_l));
      memset(stage_output_dsp.state_r, 0, sizeof(stage_output_dsp.state_r));
      memset(stage_low_dsp.state_l, 0, sizeof(stage_low_dsp.state_l));
      memset(stage_low_dsp.state_r, 0, sizeof(stage_low_dsp.state_r));
      for (int b = 0; b < XOVER_MAX_BIQUADS; b++) {
        memset(&xover_hp.biquad_state_l[b], 0, sizeof(biquad_state_t));
        memset(&xover_hp.biquad_state_r[b], 0, sizeof(biquad_state_t));
        memset(&xover_lp.biquad_state_l[b], 0, sizeof(biquad_state_t));
        memset(&xover_lp.biquad_state_r[b], 0, sizeof(biquad_state_t));
      }
      memset(buf_l, 0, count * sizeof(float));
      memset(buf_r, 0, count * sizeof(float));
      memset(low_block_l, 0, count * sizeof(float));
      memset(low_block_r, 0, count * sizeof(float));
      Serial.println("[DSP] NaN detected — filter states reset");
      break;
    }
  }

  process_limiter(&limiter, buf_l, buf_r, count);
  process_limiter(&low_limiter, low_block_l, low_block_r, count);

  input_gain_current_l = flush_denormal(input_gain_current_l);
  input_gain_current_r = flush_denormal(input_gain_current_r);
  output_gain_current_l = flush_denormal(output_gain_current_l);
  output_gain_current_r = flush_denormal(output_gain_current_r);
  low_gain_current_l = flush_denormal(low_gain_current_l);
  low_gain_current_r = flush_denormal(low_gain_current_r);
  limiter.envelope = flush_denormal(limiter.envelope);
  low_limiter.envelope = flush_denormal(low_limiter.envelope);

  delay_line_process(&delay_main, buf_l, buf_r, count);
  delay_line_process(&delay_low, low_block_l, low_block_r, count);
}

// Separate FFT task - runs on Core 1 at low priority to avoid blocking audio
static void fft_task(void *pvParameters) {
  while (1) {
    // Wait for notification from DSP task (with timeout to avoid deadlock)
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

    if (!fft_buffer_ready || !fft_enabled)
      continue;
    fft_buffer_ready = false;

    // Apply window and prepare complex array
    for (int n = 0; n < FFT_SIZE; n++) {
      dsp_fft_output[n * 2] = dsp_fft_read_buf[n] * dsp_hanning_window[n];
      dsp_fft_output[n * 2 + 1] = 0.0f;
    }

    // Run FFT (this is the expensive part, now off the audio path)
    dsps_fft2r_fc32(dsp_fft_output, FFT_SIZE);
    dsps_bit_rev_fc32(dsp_fft_output, FFT_SIZE);

    // Convert to dB magnitudes
    update_spectrum_from_fft();
  }
}

static IRAM_ATTR void dsp_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    if (dsp_suspended) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Snapshot the storage flag once per block. While set, drop
    // non-essential work so the cache-disable / slow-write window
    // doesn't blow the per-block budget. Autosave debounce guarantees
    // there are no pending coefficient updates at the moment this
    // fires, so skipping sync_coeffs is safe — they pick up next
    // block. NaN guard stays on (cheap insurance).
    const bool minimal = storage_busy;

    if (!minimal) {
      sync_coeffs(&stage_input_dsp);
      sync_coeffs(&stage_output_dsp);
      sync_coeffs(&stage_low_dsp);
      xover_sync(&xover_hp);
      xover_sync(&xover_lp);
    }

    int frame_count = 0;
    if (dsp_input_source == DSP_INPUT_I2S && i2s_audio_is_running()) {
      frame_count = i2s_input_read(i2s_read_buf, I2S_BLOCK_FRAMES, 100);
      for (int i = 0; i < frame_count; i++) {
        // Mask out the bottom 8 bits to ensure we only process the 24-bit ADC
        // signal
        int32_t l_raw = i2s_read_buf[i * 2] & 0xFFFFFF00;
        int32_t r_raw = i2s_read_buf[i * 2 + 1] & 0xFFFFFF00;
        block_l[i] = (float)l_raw * I2S_NORM_FACTOR;
        block_r[i] = (float)r_raw * I2S_NORM_FACTOR;
      }
    } else {
      // Test signal generation (Phase 13): noise, sweeps, warble
      // 128 frames: half block size, keeps test signal generation well inside
      // budget
      frame_count = 128;
      float test_lvl =
          dsp_noise_level; // Shared level control for all test signals

      for (int i = 0; i < frame_count; i++) {
        float sig = 0.0f;

        switch (dsp_input_source) {
        case DSP_INPUT_NOISE:
          sig = fast_noise();
          break;
        case DSP_INPUT_SWEEP_30_20K_30S:
          sig = generate_log_sweep();
          break;
        case DSP_INPUT_SWEEP_20_20K_35S:
          sig = generate_log_sweep();
          break;
        case DSP_INPUT_WARBLE_30_20K_30S:
          sig = generate_warble();
          break;
        default: // DSP_INPUT_I2S when I2S not running → silence
          sig = 0.0f;
          break;
        }

        sig *= test_lvl;
        block_l[i] = sig;
        block_r[i] = sig;
      }
    }

    // INPUT METER TAP: raw I2S samples after int→float, before processing
    if (frame_count > 0 && !minimal) {
      meter_process(&meter_input, block_l, block_r, frame_count);
    }

    if (frame_count > 0) {
      // esp_timer_get_time() lives in IRAM and has lower call overhead
      // than Arduino's micros() wrapper.
      uint32_t t0 = (uint32_t)esp_timer_get_time();
      process_block(block_l, block_r, frame_count);
      uint32_t dt = (uint32_t)esp_timer_get_time() - t0;

      // Diagnostic: Every 1000 blocks (~5s), print actual rate
      static uint32_t last_diag_ms = 0;
      static uint32_t total_frames = 0;
      total_frames += frame_count;
      if (millis() - last_diag_ms > 5000) {
          float actual_fs = (float)total_frames / ((millis() - last_diag_ms) / 1000.0f);
          Serial.printf("DSP Diag: Target FS=%.0f, Measured FS=%.1f\n", current_fs, actual_fs);
          last_diag_ms = millis();
          total_frames = 0;
      }

      prof_total_us += dt;
      if (dt > prof_worst_us)
        prof_worst_us = dt;
      prof_block_count++;

      if (prof_block_count >= PROFILE_INTERVAL) {
        int total_biquads = stage_input_dsp.num_active_bands_l +
                            stage_input_dsp.num_active_bands_r +
                            stage_output_dsp.num_active_bands_l +
                            stage_output_dsp.num_active_bands_r +
                            stage_low_dsp.num_active_bands_l +
                            stage_low_dsp.num_active_bands_r;
        int xover_bq = (xover_hp.enabled ? xover_hp.num_biquads * 2 : 0) +
                       (xover_lp.enabled ? xover_lp.num_biquads * 2 : 0);
        Serial.printf(
            "[DSP] %d frames | avg %lu us | worst %lu us | budget "
            "%lu us | biquads: eq=%d xover=%d\n",
            frame_count, (unsigned long)(prof_total_us / prof_block_count),
            (unsigned long)prof_worst_us,
            (unsigned long)(frame_count * 1000000UL / (uint32_t)current_fs),
            total_biquads, xover_bq);
        prof_block_count = 0;
        prof_worst_us = 0;
        prof_total_us = 0;
      };
    }

    // OUTPUT METER TAP: main path, post-limiter (and post-delay),
    // pre-clamp/write
    if (frame_count > 0 && !minimal) {
      meter_process(&meter_output, block_l, block_r, frame_count);
    }
    // LOW METER TAP: low path, post-limiter (and post-delay), pre-clamp/write
    if (frame_count > 0 && !minimal) {
      meter_process(&meter_low, low_block_l, low_block_r, frame_count);
    }

    // Write processed audio to I2S output (DAC)
    if (frame_count > 0 && i2s_audio_is_running()) {
      for (int i = 0; i < frame_count; i++) {
        float l = block_l[i];
        float r = block_r[i];
        if (l > 1.0f)
          l = 1.0f;
        else if (l < -1.0f)
          l = -1.0f;
        if (r > 1.0f)
          r = 1.0f;
        else if (r < -1.0f)
          r = -1.0f;

        // --- Final Output Stage: TPDF Dither + 24-bit Rounding ---
        // TPDF = sum of two independent uniform random values (white_noise()),
        // each ±0.5 LSB. L and R receive independent dither samples.
        // Rule 10: TPDF peak-to-peak must be 1.0 LSB (sum of two ±0.5 LSB).
        const float dither_amp = 0.5f / 16777216.0f;
        l += (white_noise() + white_noise()) * dither_amp;
        r += (white_noise() + white_noise()) * dither_amp;

        // Round to 24-bit range and shift to MSB (bottom 8 bits identically
        // zero)
        i2s_write_buf[i * 2] = lrintf(l * I2S_DENORM_FACTOR_24) << 8;
        i2s_write_buf[i * 2 + 1] = lrintf(r * I2S_DENORM_FACTOR_24) << 8;
      }
      // Increased timeout to prevent audio dropouts during UI operations
      i2s_output_write(i2s_write_buf, frame_count, 50);

      // Phase 3: low DAC gets its own processed audio (LP → low EQ → low
      // limiter → delay). Timeout=0: non-blocking. If low DMA is full, drop
      // the frame rather than stalling the main audio path.
      if (i2s_low_is_running()) {
        for (int i = 0; i < frame_count; i++) {
          float l = low_block_l[i];
          float r = low_block_r[i];
          if (l > 1.0f)
            l = 1.0f;
          else if (l < -1.0f)
            l = -1.0f;
          if (r > 1.0f)
            r = 1.0f;
          else if (r < -1.0f)
            r = -1.0f;

          // TPDF dither: independent white noise for each channel (Rule 10)
          const float dither_amp = 0.5f / 16777216.0f;
          l += (white_noise() + white_noise()) * dither_amp;
          r += (white_noise() + white_noise()) * dither_amp;

          // Round to 24-bit range and shift to MSB
          i2s_low_write_buf[i * 2] = lrintf(l * I2S_DENORM_FACTOR_24) << 8;
          i2s_low_write_buf[i * 2 + 1] = lrintf(r * I2S_DENORM_FACTOR_24) << 8;
        }
        i2s_low_output_write(i2s_low_write_buf, frame_count, 0);
      }
    }

    if (fft_enabled && !minimal && dsp_analysis_ptr >= FFT_SIZE) {
      // Swap buffers and signal FFT task (non-blocking handoff)
      float *tmp = dsp_fft_write_buf;
      dsp_fft_write_buf = dsp_fft_read_buf;
      dsp_fft_read_buf = tmp;
      dsp_analysis_ptr = 0;
      fft_buffer_ready = true;
      if (fft_task_handle) {
        xTaskNotifyGive(fft_task_handle);
      }
    }

    // Yield control to the OS.
    // 1. If in I2S mode, i2s_input_read blocks and handles timing.
    // 2. In Test mode, we always delay 1ms to ensure system tasks
    // (WiFi/Watchdog)
    //    run, while i2s_output_write handles the fine-grained audio sync.
    if (dsp_input_source != DSP_INPUT_I2S) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    } else if (!i2s_audio_is_running() || frame_count == 0) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
  }
}

static void init_stage(dsp_stage_t *stage) {
  biquad_coeffs_t pass = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  biquad_state_t zero = {0.0f, 0.0f};
  // Default parameter state: disabled pass-through peak at 1 kHz, Q=0.707
  eq_band_t default_band = {1000.0f, 0.0f, 0.707f, FTYPE_PEAK, false};
  for (int i = 0; i < MAX_BANDS; i++) {
    stage->coeffs_l[i] = pass;
    stage->coeffs_r[i] = pass;
    stage->cur_params_l[i] = default_band;
    stage->cur_params_r[i] = default_band;
    stage->target_params_l[i] = default_band;
    stage->target_params_r[i] = default_band;
    stage->pending_params_l[i] = default_band;
    stage->pending_params_r[i] = default_band;
    stage->state_l[i] = zero;
    stage->state_r[i] = zero;
    stage->sorted_idx_l[i] = i;
    stage->sorted_idx_r[i] = i;
    stage->dirty_l[i] = false;
    stage->dirty_r[i] = false;
  }
  stage->num_active_bands_l = 0;
  stage->num_active_bands_r = 0;
  stage->crossfade_samples = 0;

  memset(stage->xf_state_l, 0, sizeof(stage->xf_state_l));
  memset(stage->xf_state_r, 0, sizeof(stage->xf_state_r));
  memset(stage->xf_coeffs_l, 0, sizeof(stage->xf_coeffs_l));
  memset(stage->xf_coeffs_r, 0, sizeof(stage->xf_coeffs_r));
  memset(stage->xf_warmup_samples_l, 0, sizeof(stage->xf_warmup_samples_l));
  memset(stage->xf_warmup_samples_r, 0, sizeof(stage->xf_warmup_samples_r));
  memset(stage->xf_topo_samples_l, 0, sizeof(stage->xf_topo_samples_l));
  memset(stage->xf_topo_samples_r, 0, sizeof(stage->xf_topo_samples_r));
  memset(stage->xf_topo_to_svf_l, 0, sizeof(stage->xf_topo_to_svf_l));
  memset(stage->xf_topo_to_svf_r, 0, sizeof(stage->xf_topo_to_svf_r));
}

void dsp_init(void) {
  if (dsp_is_init)
    return;
  dsp_init_sine_lut();
  dsps_fft2r_init_fc32(NULL, FFT_SIZE);
  dsps_wind_hann_f32(dsp_hanning_window, FFT_SIZE);
  for (int i = 0; i < NUM_SPECTRUM_BANDS; i++)
    dsp_iso_freqs[i] =
        20.0f *
        powf(2.0f, (float)i * (10.0f / (float)(NUM_SPECTRUM_BANDS - 1)));

  init_stage(&stage_input_dsp);
  init_stage(&stage_output_dsp);
  init_stage(&stage_low_dsp); // Phase 3: low-path EQ stage

  // Initialize level meters
  meter_init(&meter_input);
  meter_init(&meter_output);
  meter_init(&meter_low); // Phase 3: low-path meter

  // Default main limiter: 2ms attack, 50ms release, -1dB threshold
  limiter_init(current_fs, -1.0f, 2.0f, 50.0f);

  // Phase 3 low limiter: same attack/release as main, same -1dB threshold,
  // disabled by default (user enables via Config page in Phase 8).
  low_limiter.threshold_lin = powf(10.0f, -1.0f / 20.0f);
  low_limiter.attack_coeff =
      1.0f - expf(-1.0f / ((2.0f / 1000.0f) * current_fs));
  low_limiter.release_coeff =
      1.0f - expf(-1.0f / ((50.0f / 1000.0f) * current_fs));
  low_limiter.envelope = 0.0f;
  low_limiter.enabled = false;

  // Phase 3 crossover init (zero states, all filters bypassed)
  xover_init();

  // Push whatever is currently in xover_settings (populated by eq_ui.cpp
  // before dsp_init() is called) into the DSP. This is the single source
  // of truth — UI dropdowns and DSP filters both read from this struct.
  xover_update_hp(&xover_settings.hp);
  xover_update_lp(&xover_settings.lp);
  xover_set_mono(xover_settings.low_mono);
  xover_set_phase_invert(xover_settings.phase_invert);
  xover_set_delay(xover_settings.delay_ms);
  low_set_output_gain(0, 0.0f);
  low_set_output_gain(1, 0.0f);

  // Reduced stack size to prevent memory issues
  if (xTaskCreatePinnedToCore(dsp_task, "DSP_Task", 8192, NULL, 2, NULL, 0) !=
      pdPASS) {
    Serial.println("DSP task creation FAILED (insufficient heap)");
    return;
  }

  // FFT task on Core 1 at low priority (below UI) - won't block audio
  if (xTaskCreatePinnedToCore(fft_task, "FFT_Task", 4096, NULL, 1,
                              &fft_task_handle, 1) != pdPASS) {
    Serial.println("FFT task creation FAILED (non-critical)");
    // Continue anyway - FFT is optional
  }

  dsp_is_init = true;
  Serial.println("DSP engine initialised (stereo dual-mono, Core 0)");
}

// Helper: dispatch stage_idx → dsp_stage_t*. Returns NULL for invalid idx.
// 0 = input, 1 = output, 2 = low.
static inline dsp_stage_t *stage_ptr(int stage_idx) {
  switch (stage_idx) {
  case 0:
    return &stage_input_dsp;
  case 1:
    return &stage_output_dsp;
  case 2:
    return &stage_low_dsp;
  default:
    return NULL;
  }
}

// Helper: dispatch stage_idx → meter_state_t*.
static inline meter_state_t *meter_ptr(int stage_idx) {
  switch (stage_idx) {
  case 0:
    return &meter_input;
  case 1:
    return &meter_output;
  case 2:
    return &meter_low;
  default:
    return NULL;
  }
}

void dsp_set_active_bands(int stage_idx, int channel, int count) {
  dsp_stage_t *stage = stage_ptr(stage_idx);
  if (!stage || count < 0 || count > MAX_BANDS)
    return;
  if (channel == 0)
    stage->num_active_bands_l = count;
  else
    stage->num_active_bands_r = count;
}

void dsp_update_band(int stage_idx, int channel, int band_idx,
                     eq_band_t *band_data) {
  if (band_idx < 0 || band_idx >= MAX_BANDS || !band_data)
    return;
  dsp_stage_t *stage = stage_ptr(stage_idx);
  if (!stage)
    return;

  // Store raw parameters only — coefficient calculation happens inside
  // interpolate_coeffs (every LOW_BLOCK samples during the crossfade).
  // This ensures calculate_biquad is never called from outside the DSP loop
  // with stale sample-rate or from the UI thread (Rule 3, Rule 7).
  taskENTER_CRITICAL(&dsp_mux);
  if (channel == 0) {
    stage->pending_params_l[band_idx] = *band_data;
    stage->dirty_l[band_idx] = true;
    if (band_data->enabled && band_idx + 1 > stage->num_active_bands_l)
      stage->num_active_bands_l = band_idx + 1;
  } else {
    stage->pending_params_r[band_idx] = *band_data;
    stage->dirty_r[band_idx] = true;
    if (band_data->enabled && band_idx + 1 > stage->num_active_bands_r)
      stage->num_active_bands_r = band_idx + 1;
  }
  taskEXIT_CRITICAL(&dsp_mux);
}

void dsp_set_input_source(dsp_input_source_t source) {
  dsp_input_source = source;

  // Reset generator state on mode change to avoid phase discontinuities
  switch (source) {
  case DSP_INPUT_SWEEP_30_20K_30S:
    reset_sweep_state(30.0f, 20000.0f, 30.0f);
    break;
  case DSP_INPUT_SWEEP_20_20K_35S:
    reset_sweep_state(20.0f, 20000.0f, 35.0f);
    break;
  case DSP_INPUT_WARBLE_30_20K_30S:
    reset_warble_state(30.0f, 20000.0f, 30.0f);
    break;
  default:
    break; // I2S and noise don't need state reset
  }
}

void dsp_update_noise_gen(bool enabled, float level_db) {
  (void)enabled; // Dispatch is via dsp_input_source, not this flag
  dsp_noise_level = powf(10.0f, level_db / 20.0f);
}

void dsp_set_sample_rate(float fs) {
  current_fs = fs;
  // Update limiter coefficients for new sample rate
  limiter.attack_coeff = 1.0f - expf(-1.0f / ((2.0f / 1000.0f) * fs));
  limiter.release_coeff = 1.0f - expf(-1.0f / ((50.0f / 1000.0f) * fs));
  low_limiter.attack_coeff = 1.0f - expf(-1.0f / ((2.0f / 1000.0f) * fs));
  low_limiter.release_coeff = 1.0f - expf(-1.0f / ((50.0f / 1000.0f) * fs));
}

// ============================================================
// Snap-and-reset for sample-rate change
// ============================================================
// On a sample-rate change the new biquad coefficients can differ enough from
// the live set that the standard crossfade would visit intermediate
// coefficient values that are unstable at the new state. The caller's pattern
// should be:
//   1. Set dsp_suspended = true, wait ~20 ms for the task to park.
//   2. Restart I2S at the new rate, call dsp_set_sample_rate(new_fs).
//   3. Push all EQ bands + crossover filters with the new fs.
//   4. Call dsp_snap_and_reset_all() to collapse pending→live and zero state.
//   5. Clear dsp_suspended.
// This prevents the "pop" from applying new coefficients to old state, and
// sidesteps the unstable-crossfade-through-the-origin problem.

static void stage_snap_and_reset(dsp_stage_t *stage) {
  taskENTER_CRITICAL(&dsp_mux);
  for (int i = 0; i < MAX_BANDS; i++) {
    if (stage->dirty_l[i]) {
      stage->target_params_l[i] = stage->pending_params_l[i];
      stage->dirty_l[i] = false;
    }
    if (stage->dirty_r[i]) {
      stage->target_params_r[i] = stage->pending_params_r[i];
      stage->dirty_r[i] = false;
    }
    // Snap cur → target, recompute coefficients, zero IIR state
    stage->cur_params_l[i] = stage->target_params_l[i];
    stage->cur_params_r[i] = stage->target_params_r[i];
    calculate_biquad(&stage->coeffs_l[i], &stage->cur_params_l[i], current_fs);
    calculate_biquad(&stage->coeffs_r[i], &stage->cur_params_r[i], current_fs);
    stage->state_l[i].s1 = 0.0f;
    stage->state_l[i].s2 = 0.0f;
    stage->state_r[i].s1 = 0.0f;
    stage->state_r[i].s2 = 0.0f;
    stage->xf_warmup_samples_l[i] = 0;
    stage->xf_warmup_samples_r[i] = 0;
    stage->xf_topo_samples_l[i] = 0;
    stage->xf_topo_samples_r[i] = 0;
  }
  stage->crossfade_samples = 0;
  taskEXIT_CRITICAL(&dsp_mux);
}

static void xover_snap_and_reset(xover_filter_t *xf) {
  taskENTER_CRITICAL(&dsp_mux);
  // Collapse pending → live, zero biquad + onepole state, cancel crossfade.
  for (int i = 0; i < XOVER_MAX_BIQUADS; i++) {
    xf->biquad_target[i] = xf->biquad_pending[i];
    xf->biquad_coeffs[i] = xf->biquad_pending[i];
    xf->biquad_state_l[i].s1 = 0.0f;
    xf->biquad_state_l[i].s2 = 0.0f;
    xf->biquad_state_r[i].s1 = 0.0f;
    xf->biquad_state_r[i].s2 = 0.0f;
  }
  xf->onepole_coeff_target = xf->onepole_coeff_pending;
  xf->onepole_coeff = xf->onepole_coeff_pending;
  xf->onepole_state_l.z1 = 0.0f;
  xf->onepole_state_r.z1 = 0.0f;
  xf->num_biquads = xf->num_biquads_pending;
  xf->is_onepole = xf->is_onepole_pending;
  xf->enabled = xf->enabled_pending;
  xf->crossfade_samples = 0;
  xf->dirty = false;
  taskEXIT_CRITICAL(&dsp_mux);
}

void dsp_snap_and_reset_all(void) {
  stage_snap_and_reset(&stage_input_dsp);
  stage_snap_and_reset(&stage_output_dsp);
  stage_snap_and_reset(&stage_low_dsp);
  xover_snap_and_reset(&xover_hp);
  xover_snap_and_reset(&xover_lp);

  // Collapse gain smoothers to their targets instantly so we don't take a
  // perceptible gain ramp across the rate switch.
  input_gain_current_l = input_gain_target_l;
  input_gain_current_r = input_gain_target_r;
  output_gain_current_l = output_gain_target_l;
  output_gain_current_r = output_gain_target_r;
  low_gain_current_l = low_gain_target_l;
  low_gain_current_r = low_gain_target_r;
  for (int i = 0; i < 3; i++)
    mute_gain_current[i] = dsp_stage_muted[i] ? 0.0f : 1.0f;

  // Zero limiter envelopes — old envelope values are meaningless after fs
  // change because attack/release coefficients have been recomputed.
  limiter.envelope = 0.0f;
  low_limiter.envelope = 0.0f;

  // Reset pink noise filter state.
  pink_b0 = pink_b1 = pink_b2 = pink_b3 = 0.0f;
  pink_b4 = pink_b5 = pink_b6 = 0.0f;
}

void dsp_update_limiter(bool enabled, float threshold_db) {
  limiter.enabled = enabled;
  limiter.threshold_lin = powf(10.0f, threshold_db / 20.0f);
}

void limiter_init(float sample_rate, float threshold_db, float attack_ms,
                  float release_ms) {
  limiter.threshold_lin = powf(10.0f, threshold_db / 20.0f);
  limiter.attack_coeff =
      1.0f - expf(-1.0f / ((attack_ms / 1000.0f) * sample_rate));
  limiter.release_coeff =
      1.0f - expf(-1.0f / ((release_ms / 1000.0f) * sample_rate));
  limiter.envelope = 0.0f;
  limiter.enabled = true;
}

bool dsp_is_running(void) { return dsp_is_init; }

void dsp_set_fft_enabled(bool enabled) {
  fft_enabled = enabled;
  if (!enabled) {
    dsp_analysis_ptr = 0; // Reset buffer pointer when disabled
  }
}

bool dsp_is_fft_enabled(void) { return fft_enabled; }

void dsp_scope_get(float *dst, int len) {
  if (!dst || len <= 0) return;
  if (len > SCOPE_BUF_SIZE) len = SCOPE_BUF_SIZE;
  // Snapshot the most recent `len` samples in time order (oldest first).
  // Read the write cursor once to get a consistent window.
  int end = (int)scope_ring_idx;
  int start = (end - len) & (SCOPE_BUF_SIZE - 1);
  int first = SCOPE_BUF_SIZE - start;
  if (first >= len) {
    memcpy(dst, (const float *)scope_ring_buf + start, len * sizeof(float));
  } else {
    memcpy(dst,         (const float *)scope_ring_buf + start, first         * sizeof(float));
    memcpy(dst + first, (const float *)scope_ring_buf,         (len - first) * sizeof(float));
  }
}

void dsp_set_input_gain(int channel, float db) {
  float lin = (db <= -60.0f) ? 0.0f : powf(10.0f, db / 20.0f);
  if (channel == 0) {
    input_gain_target_l = lin;
    input_gain_db_l = db;
  } else {
    input_gain_target_r = lin;
    input_gain_db_r = db;
  }
}

void dsp_set_output_gain(int channel, float db) {
  float lin = (db <= -60.0f) ? 0.0f : powf(10.0f, db / 20.0f);
  if (channel == 0) {
    output_gain_target_l = lin;
    output_gain_db_l = db;
  } else {
    output_gain_target_r = lin;
    output_gain_db_r = db;
  }
}

float dsp_get_input_gain(int channel) {
  return (channel == 0) ? input_gain_db_l : input_gain_db_r;
}

float dsp_get_output_gain(int channel) {
  return (channel == 0) ? output_gain_db_l : output_gain_db_r;
}

// ============================================================
// Level meter public API
// ============================================================
void dsp_set_meter_active(int stage_idx, bool active) {
  meter_state_t *m = meter_ptr(stage_idx);
  if (!m)
    return;
  m->active = active;
  if (!active) {
    // Reset state when deactivated
    m->peak_l = 0.0f;
    m->peak_r = 0.0f;
    m->peak_hold_l = 0.0f;
    m->peak_hold_r = 0.0f;
    m->hold_timer_l = 0;
    m->hold_timer_r = 0;
    m->clip_timer_l = 0;
    m->clip_timer_r = 0;
    m->clipping_l = false;
    m->clipping_r = false;
  }
}

float dsp_get_meter_peak(int stage_idx, int channel) {
  meter_state_t *m = meter_ptr(stage_idx);
  if (!m)
    return METER_FLOOR_DB;
  float peak = (channel == 0) ? m->peak_l : m->peak_r;
  // Convert linear to dB, clamp to floor
  if (peak < 1e-10f)
    return METER_FLOOR_DB;
  float db = 20.0f * log10f(peak);
  if (db < METER_FLOOR_DB)
    db = METER_FLOOR_DB;
  return db;
}

float dsp_get_meter_peak_hold(int stage_idx, int channel) {
  meter_state_t *m = meter_ptr(stage_idx);
  if (!m)
    return METER_FLOOR_DB;
  float peak = (channel == 0) ? m->peak_hold_l : m->peak_hold_r;
  if (peak < 1e-10f)
    return METER_FLOOR_DB;
  float db = 20.0f * log10f(peak);
  if (db < METER_FLOOR_DB)
    db = METER_FLOOR_DB;
  return db;
}

bool dsp_get_meter_clipping(int stage_idx, int channel) {
  meter_state_t *m = meter_ptr(stage_idx);
  if (!m)
    return false;
  return (channel == 0) ? m->clipping_l : m->clipping_r;
}

void dsp_clear_meter_clip(int stage_idx) {
  meter_state_t *m = meter_ptr(stage_idx);
  if (!m)
    return;
  m->clipping_l = false;
  m->clipping_r = false;
  m->clip_timer_l = 0;
  m->clip_timer_r = 0;
}

// ============================================================
// Phase 3 crossover implementation
// ============================================================

// Note: low_gain_db_l/r, low_gain_target_l/r, low_gain_current_l/r,
// xover_hp, xover_lp, delay_main, delay_low, low_limiter, low_mono,
// low_phase_invert, and stage_low_dsp are all declared at the top of the
// file with the other DSP state.

void xover_init(void) {
  // Zero filter states, init to disabled/bypass. Coefficient calculation
  // happens lazily when xover_update_hp/lp is called.
  memset(&xover_hp, 0, sizeof(xover_hp));
  memset(&xover_lp, 0, sizeof(xover_lp));
  // Make all biquad coeffs explicit pass-through
  biquad_coeffs_t pass = xover_passthrough_coeffs();
  for (int i = 0; i < XOVER_MAX_BIQUADS; i++) {
    xover_hp.biquad_coeffs[i] = pass;
    xover_hp.biquad_target[i] = pass;
    xover_hp.biquad_pending[i] = pass;
    xover_lp.biquad_coeffs[i] = pass;
    xover_lp.biquad_target[i] = pass;
    xover_lp.biquad_pending[i] = pass;
  }
  xover_hp.enabled = false;
  xover_lp.enabled = false;
  xover_hp.enabled_pending = false;
  xover_lp.enabled_pending = false;
  xover_hp.is_highpass = true;
  xover_lp.is_highpass = false;

  delay_line_init(&delay_main);
  delay_line_init(&delay_low);
}

void xover_update_hp(xover_band_t *settings) {
  if (!settings)
    return;
  xover_build_pending(&xover_hp, settings, /*is_highpass=*/true, current_fs);
}

void xover_update_lp(xover_band_t *settings) {
  if (!settings)
    return;
  xover_build_pending(&xover_lp, settings, /*is_highpass=*/false, current_fs);
}

void xover_set_phase_invert(bool invert) { low_phase_invert = invert; }

void xover_set_delay(float ms) {
  // Convention: positive ms → delay LOW path; negative → delay MAIN path.
  // Clamp to [-8, +8] ms (matches plan's ±8ms spec and DELAY_MAX_SAMPLES-1).
  if (ms > 8.0f)
    ms = 8.0f;
  if (ms < -8.0f)
    ms = -8.0f;

  int total_samples = (int)(fabsf(ms) * current_fs / 1000.0f + 0.5f);
  if (total_samples >= DELAY_MAX_SAMPLES)
    total_samples = DELAY_MAX_SAMPLES - 1;

  int main_new = (ms < 0.0f) ? total_samples : 0;
  int low_new = (ms > 0.0f) ? total_samples : 0;

  taskENTER_CRITICAL(&dsp_mux);
  if (main_new != delay_main.delay_target) {
    delay_main.delay_target = main_new;
    delay_main.crossfade_counter = CROSSFADE_SAMPLES;
  }
  if (low_new != delay_low.delay_target) {
    delay_low.delay_target = low_new;
    delay_low.crossfade_counter = CROSSFADE_SAMPLES;
  }
  taskEXIT_CRITICAL(&dsp_mux);
}

void xover_set_mono(bool mono) { low_mono = mono; }

void low_set_output_gain(int channel, float db) {
  float lin = (db <= -60.0f) ? 0.0f : powf(10.0f, db / 20.0f);
  if (channel == 0) {
    low_gain_target_l = lin;
    low_gain_db_l = db;
  } else {
    low_gain_target_r = lin;
    low_gain_db_r = db;
  }
}

float low_get_output_gain(int channel) {
  return (channel == 0) ? low_gain_db_l : low_gain_db_r;
}

void low_update_limiter(bool enabled, float threshold_db) {
  // Same coefficients as main limiter (2ms attack, 50ms release) — the only
  // per-low knobs exposed are enable + threshold. If per-low attack/release
  // is wanted later, add a low_limiter_init() variant.
  low_limiter.enabled = enabled;
  low_limiter.threshold_lin = powf(10.0f, threshold_db / 20.0f);
}

// compute_xover_response lives here (prototype in eq_data.h).
// Returns dB magnitude for HP or LP filter at given frequency using textbook
// biquad response. Uses the shared xover_q_table() helper and Q tables defined
// at file scope (see top of crossover implementation block).
float compute_xover_response(xover_band_t *xover, float freq_hz, float fs,
                             bool is_highpass) {
  // Math runs in double precision to avoid catastrophic cancellation at low
  // f/fs ratios (see compute_band_response for the full explanation). Xover
  // display curves cascade up to 4 biquads; single-precision loses even more
  // digits in that multiplicative chain near DC.
  if (!xover || !xover->enabled || xover->type == XOVER_BYPASS)
    return 0.0f;

  const double PI_D = 3.14159265358979323846;

  if (xover->slope == XOVER_SLOPE_6DB) {
    // Single-pole magnitude at freq_hz
    //   LP: |H| = 1 / sqrt(1 + (f/fc)^2)
    //   HP: |H| = (f/fc) / sqrt(1 + (f/fc)^2)
    double w = 2.0 * PI_D * (double)xover->freq / (double)fs;
    double wq = 2.0 * PI_D * (double)freq_hz / (double)fs;
    double ratio = wq / w;
    double mag_sq = is_highpass ? (ratio * ratio) / (1.0 + ratio * ratio)
                                : 1.0 / (1.0 + ratio * ratio);
    if (mag_sq < 1e-20)
      return -100.0f;
    return (float)(10.0 * log10(mag_sq));
  }

  int num_biquads = 0;
  const float *q_table = xover_q_table(xover->type, xover->slope, &num_biquads);
  if (!q_table || num_biquads == 0)
    return 0.0f;

  double K = tan(PI_D * (double)xover->freq / (double)fs);
  double KK = K * K;
  double w = 2.0 * PI_D * (double)freq_hz / (double)fs;
  double cw = cos(w);
  double c2w = cos(2.0 * w);
  double sw = sin(w);
  double s2w = sin(2.0 * w);

  double total_mag_sq = 1.0;

  for (int i = 0; i < num_biquads; i++) {
    double q = (double)q_table[i];
    double b0, b1, b2, a0, a1, a2;

    if (is_highpass) {
      a0 = 1.0 + K / q + KK;
      b0 = 1.0 / a0;
      b1 = -2.0 / a0;
      b2 = 1.0 / a0;
      a1 = 2.0 * (KK - 1.0) / a0;
      a2 = (1.0 - K / q + KK) / a0;
    } else {
      a0 = 1.0 + K / q + KK;
      b0 = KK / a0;
      b1 = 2.0 * KK / a0;
      b2 = KK / a0;
      a1 = 2.0 * (KK - 1.0) / a0;
      a2 = (1.0 - K / q + KK) / a0;
    }

    // Evaluate H(e^jw)
    double nr = b0 + b1 * cw + b2 * c2w;
    double ni = -b1 * sw - b2 * s2w;
    double dr = 1.0 + a1 * cw + a2 * c2w;
    double di = -a1 * sw - a2 * s2w;

    double mag_sq = (nr * nr + ni * ni) / (dr * dr + di * di + 1e-30);
    total_mag_sq *= mag_sq;
  }

  if (total_mag_sq < 1e-20)
    return -100.0f;
  double db = 10.0 * log10(total_mag_sq);
  if (isnan(db) || isinf(db))
    return 0.0f;
  if (db < -100.0)
    return -100.0f;
  if (db > 100.0)
    return 100.0f;
  return (float)db;
}

// Precomputes the UI render state for a crossover
void ui_xover_calc(xover_band_t *xover, float fs, bool is_highpass,
                   ui_xover_t *out) {
  if (!xover || !xover->enabled || xover->type == XOVER_BYPASS) {
    out->active = false;
    return;
  }

  out->active = true;
  out->is_highpass = is_highpass;
  const double PI_D = 3.14159265358979323846;
  double wc = 2.0 * PI_D * (double)xover->freq / (double)fs;

  if (xover->slope == XOVER_SLOPE_6DB) {
    out->is_onepole = true;
    out->onepole_wc = wc;
    out->num_biquads = 0;
    return;
  }

  out->is_onepole = false;
  int num_biquads = 0;
  const float *q_table = xover_q_table(xover->type, xover->slope, &num_biquads);
  if (!q_table || num_biquads == 0 || num_biquads > 4) {
    out->active = false;
    return;
  }

  out->num_biquads = num_biquads;
  double K = tan(PI_D * (double)xover->freq / (double)fs);
  double KK = K * K;

  for (int i = 0; i < num_biquads; i++) {
    double q = (double)q_table[i];
    double b0, b1, b2, a0, a1, a2;

    if (is_highpass) {
      a0 = 1.0 + K / q + KK;
      b0 = 1.0 / a0;
      b1 = -2.0 / a0;
      b2 = 1.0 / a0;
      a1 = 2.0 * (KK - 1.0) / a0;
      a2 = (1.0 - K / q + KK) / a0;
    } else {
      a0 = 1.0 + K / q + KK;
      b0 = KK / a0;
      b1 = 2.0 * KK / a0;
      b2 = KK / a0;
      a1 = 2.0 * (KK - 1.0) / a0;
      a2 = (1.0 - K / q + KK) / a0;
    }

    out->bqs[i].active = true;
    out->bqs[i].b0_a0 = b0;
    out->bqs[i].b1_a0 = b1;
    out->bqs[i].b2_a0 = b2;
    out->bqs[i].a1_a0 = a1;
    out->bqs[i].a2_a0 = a2;
  }
}
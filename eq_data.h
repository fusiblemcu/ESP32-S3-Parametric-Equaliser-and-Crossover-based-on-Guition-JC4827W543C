#ifndef EQ_DATA_H
#define EQ_DATA_H

#include <math.h>
#include <stdint.h>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define NUM_BANDS 6
#define MAX_BANDS 15
#define CURVE_POINTS 100
#define DB_RANGE 20.0f

typedef enum {
  FTYPE_PEAK = 0,
  FTYPE_LOW_SHELF,
  FTYPE_HIGH_SHELF
} filter_type_t;

typedef struct {
  float freq;
  float gain;
  float q;
  filter_type_t type;
  bool enabled;
} eq_band_t;

// Lever precision widget state
typedef struct {
  bool active;
  bool beam_visible;
  int band_idx;
  int param_type; // 0=freq, 1=gain, 2=q
  float start_norm;
  float cur_norm;
  int anchor_x, anchor_y;
  int finger_x, finger_y;
  int side; // 1=right, -1=left
  float commit_hdist;
  float last_hraw;
  int last_y;
  bool at_limit;
  char limit_cap; // 't'=top, 'b'=bottom, 0=none
  int beam_x;
  int beam_top, beam_bot;
  float beam_len;
} lever_state_t;

// Parameter ranges
static const float FREQ_MIN = 20.0f;
static const float FREQ_MAX = 20000.0f;
static const float GAIN_MIN = -15.0f;
static const float GAIN_MAX = 15.0f;
static const float Q_MIN = 0.1f;
static const float Q_MAX = 50.0f;
static const float H_DEADZONE = 4.0f;

// --- Conversion helpers ---

inline float freq_to_log(float f) {
  return (logf(f) - logf(FREQ_MIN)) / (logf(FREQ_MAX) - logf(FREQ_MIN));
}

inline float log_to_freq(float t) {
  return expf(logf(FREQ_MIN) + t * (logf(FREQ_MAX) - logf(FREQ_MIN)));
}

inline float value_to_norm(int param, float val) {
  switch (param) {
  case 0:
    return freq_to_log(val);
  case 1:
    return (val - GAIN_MIN) / (GAIN_MAX - GAIN_MIN);
  case 2:
    return (logf(val) - logf(Q_MIN)) / (logf(Q_MAX) - logf(Q_MIN));
  }
  return 0.0f;
}

inline float norm_to_value(int param, float n) {
  if (n < 0.0f)
    n = 0.0f;
  if (n > 1.0f)
    n = 1.0f;
  switch (param) {
  case 0:
    return log_to_freq(n);
  case 1:
    return GAIN_MIN + n * (GAIN_MAX - GAIN_MIN);
  case 2:
    return expf(logf(Q_MIN) + n * (logf(Q_MAX) - logf(Q_MIN)));
  }
  return 0.0f;
}

inline float get_band_param(eq_band_t *b, int param) {
  switch (param) {
  case 0:
    return b->freq;
  case 1:
    return b->gain;
  case 2:
    return b->q;
  }
  return 0.0f;
}

inline void set_band_param(eq_band_t *b, int param, float val) {
  switch (param) {
  case 0:
    b->freq = val;
    break;
  case 1:
    b->gain = val;
    break;
  case 2:
    b->q = val;
    break;
  }
}

// --- Frequency response computation ---

// Curve-evaluation math is done in double precision to avoid catastrophic
// subtractive cancellation in the direct-form transfer-function evaluation
// at very low f/fs ratios: when w0 is small, b1 and a1 are both ≈ -2·cos(w0)
// and the sums `1 + a1·cos(w) + a2·cos(2w)` become the difference of two
// near-equal numbers, which blows away most of float32's 7 decimal digits.
// The DSP audio path stays single-precision — it uses DF-II state variables
// that carry precision forward and doesn't suffer this issue.
inline float compute_band_response(eq_band_t *b, float freq_hz, float fs) {
  if (!b->enabled || b->gain == 0.0f)
    return 0.0f;

  const double PI_D = 3.14159265358979323846;
  double A = pow(10.0, (double)b->gain / 40.0);
  double w0 = 2.0 * PI_D * (double)b->freq / (double)fs;
  double alpha = sin(w0) / (2.0 * (double)b->q);
  double b0, b1, b2, a0, a1, a2;

  switch (b->type) {
  case FTYPE_PEAK:
    b0 = 1.0 + alpha * A;
    b1 = -2.0 * cos(w0);
    b2 = 1.0 - alpha * A;
    a0 = 1.0 + alpha / A;
    a1 = -2.0 * cos(w0);
    a2 = 1.0 - alpha / A;
    break;
  case FTYPE_LOW_SHELF: {
    double sq = 2.0 * sqrt(A) * alpha;
    b0 = A * ((A + 1) - (A - 1) * cos(w0) + sq);
    b1 = 2.0 * A * ((A - 1) - (A + 1) * cos(w0));
    b2 = A * ((A + 1) - (A - 1) * cos(w0) - sq);
    a0 = (A + 1) + (A - 1) * cos(w0) + sq;
    a1 = -2.0 * ((A - 1) + (A + 1) * cos(w0));
    a2 = (A + 1) + (A - 1) * cos(w0) - sq;
    break;
  }
  case FTYPE_HIGH_SHELF: {
    double sq = 2.0 * sqrt(A) * alpha;
    b0 = A * ((A + 1) + (A - 1) * cos(w0) + sq);
    b1 = -2.0 * A * ((A - 1) + (A + 1) * cos(w0));
    b2 = A * ((A + 1) + (A - 1) * cos(w0) - sq);
    a0 = (A + 1) - (A - 1) * cos(w0) + sq;
    a1 = 2.0 * ((A - 1) - (A + 1) * cos(w0));
    a2 = (A + 1) - (A - 1) * cos(w0) - sq;
    break;
  }
  default:
    return 0.0f;
  }

  double w = 2.0 * PI_D * (double)freq_hz / (double)fs;
  double cw = cos(w), sw = sin(w);
  double c2w = cos(2.0 * w), s2w = sin(2.0 * w);

  double nr = b0 / a0 + (b1 / a0) * cw + (b2 / a0) * c2w;
  double ni = -(b1 / a0) * sw - (b2 / a0) * s2w;
  double dr = 1.0 + (a1 / a0) * cw + (a2 / a0) * c2w;
  double di = -(a1 / a0) * sw - (a2 / a0) * s2w;

  double mag_sq = (nr * nr + ni * ni) / (dr * dr + di * di + 1e-20);
  double db = 10.0 * log10(fmax(mag_sq, 1e-20));

  // Bulletproof NaN/Infinity guard for rendering stability
  if (isnan(db) || isinf(db)) return 0.0f;
  if (db < -100.0f) return -100.0f;
  if (db > 100.0f) return 100.0f;
  return (float)db;
}

inline float compute_total_response(eq_band_t *bands, int count,
                                    float freq_hz, float fs) {
  float total = 0.0f;
  for (int i = 0; i < count; i++) {
    total += compute_band_response(&bands[i], freq_hz, fs);
  }
  return total;
}

// --- Precomputed UI rendering structures (Optimization 3) ---
// Pre-calculating a0/b0 etc removes thousands of trig/pow calculations per frame
typedef struct {
    double b0_a0, b1_a0, b2_a0, a1_a0, a2_a0;
    bool active;
} ui_biquad_t;

typedef struct {
    ui_biquad_t bqs[4];
    int num_biquads;
    bool is_onepole;
    bool is_highpass;
    double onepole_wc;
    bool active;
} ui_xover_t;

inline void ui_biquad_calc(eq_band_t *b, float fs, ui_biquad_t *out) {
    if (!b || !b->enabled || b->gain == 0.0f) {
        out->active = false;
        return;
    }
    out->active = true;
    const double PI_D = 3.14159265358979323846;
    double A = pow(10.0, (double)b->gain / 40.0);
    double w0 = 2.0 * PI_D * (double)b->freq / (double)fs;
    double alpha = sin(w0) / (2.0 * (double)b->q);
    double b0, b1, b2, a0, a1, a2;

    switch (b->type) {
    case FTYPE_PEAK:
      b0 = 1.0 + alpha * A; b1 = -2.0 * cos(w0); b2 = 1.0 - alpha * A;
      a0 = 1.0 + alpha / A; a1 = -2.0 * cos(w0); a2 = 1.0 - alpha / A;
      break;
    case FTYPE_LOW_SHELF: {
      double sq = 2.0 * sqrt(A) * alpha;
      b0 = A * ((A + 1) - (A - 1) * cos(w0) + sq);
      b1 = 2.0 * A * ((A - 1) - (A + 1) * cos(w0));
      b2 = A * ((A + 1) - (A - 1) * cos(w0) - sq);
      a0 = (A + 1) + (A - 1) * cos(w0) + sq;
      a1 = -2.0 * ((A - 1) + (A + 1) * cos(w0));
      a2 = (A + 1) + (A - 1) * cos(w0) - sq;
      break;
    }
    case FTYPE_HIGH_SHELF: {
      double sq = 2.0 * sqrt(A) * alpha;
      b0 = A * ((A + 1) + (A - 1) * cos(w0) + sq);
      b1 = -2.0 * A * ((A - 1) + (A + 1) * cos(w0));
      b2 = A * ((A + 1) + (A - 1) * cos(w0) - sq);
      a0 = (A + 1) - (A - 1) * cos(w0) + sq;
      a1 = 2.0 * ((A - 1) - (A + 1) * cos(w0));
      a2 = (A + 1) - (A - 1) * cos(w0) - sq;
      break;
    }
    default:
      out->active = false;
      return;
    }

    out->b0_a0 = b0 / a0;
    out->b1_a0 = b1 / a0;
    out->b2_a0 = b2 / a0;
    out->a1_a0 = a1 / a0;
    out->a2_a0 = a2 / a0;
}

inline double ui_biquad_eval(ui_biquad_t *bq, int count, float freq_hz, float fs) {
    if (count == 0) return 0.0;
    
    double w = 2.0 * 3.14159265358979323846 * (double)freq_hz / (double)fs;
    double cw = cos(w), sw = sin(w);
    double c2w = cos(2.0 * w), s2w = sin(2.0 * w);

    double total_mag_sq = 1.0;
    bool has_active = false;

    for (int i = 0; i < count; i++) {
        if (!bq[i].active) continue;
        has_active = true;
        double nr = bq[i].b0_a0 + bq[i].b1_a0 * cw + bq[i].b2_a0 * c2w;
        double ni = -bq[i].b1_a0 * sw - bq[i].b2_a0 * s2w;
        double dr = 1.0 + bq[i].a1_a0 * cw + bq[i].a2_a0 * c2w;
        double di = -bq[i].a1_a0 * sw - bq[i].a2_a0 * s2w;

        double mag_sq = (nr * nr + ni * ni) / (dr * dr + di * di + 1e-20);
        total_mag_sq *= fmax(mag_sq, 1e-20);
    }
    
    if (!has_active) return 0.0;
    
    double db = 10.0 * log10(total_mag_sq);
    if (isnan(db) || isinf(db)) return 0.0;
    if (db < -100.0) return -100.0;
    if (db > 100.0) return 100.0;
    return db;
}

inline double ui_xover_eval(ui_xover_t *xo, float freq_hz, float fs) {
    if (!xo || !xo->active) return 0.0;
    
    if (xo->is_onepole) {
        double wq = 2.0 * 3.14159265358979323846 * (double)freq_hz / (double)fs;
        double ratio = wq / xo->onepole_wc;
        double mag_sq = xo->is_highpass ? (ratio * ratio) / (1.0 + ratio * ratio)
                                        : 1.0 / (1.0 + ratio * ratio);
        if (mag_sq < 1e-20) return -100.0;
        return 10.0 * log10(mag_sq);
    }
    
    return ui_biquad_eval(xo->bqs, xo->num_biquads, freq_hz, fs);
}

// --- String formatting ---

inline void format_freq(char *buf, size_t len, float f) {
  if (f >= 1000.0f)
    snprintf(buf, len, "%.1fk", f / 1000.0f);
  else
    snprintf(buf, len, "%.0f", f);
}

inline void format_gain(char *buf, size_t len, float g) {
  snprintf(buf, len, "%+.1f", g);
}

inline void format_q(char *buf, size_t len, float q) {
  snprintf(buf, len, "%.2f", q);
}

inline void format_param(char *buf, size_t len, int param, float val) {
  switch (param) {
  case 0:
    format_freq(buf, len, val);
    break;
  case 1:
    format_gain(buf, len, val);
    break;
  case 2:
    format_q(buf, len, val);
    break;
  }
}

inline const char *filter_type_str(filter_type_t t) {
  switch (t) {
  case FTYPE_PEAK:
    return "Peak";
  case FTYPE_LOW_SHELF:
    return "LoShelf";
  case FTYPE_HIGH_SHELF:
    return "HiShelf";
  }
  return "?";
}

// --- Crossover filter types ---

typedef enum {
    XOVER_BYPASS = 0,
    XOVER_BUTTERWORTH,
    XOVER_LINKWITZ_RILEY,
    XOVER_BESSEL
} xover_filter_type_t;

typedef enum {
    XOVER_SLOPE_6DB  = 0,  // 1st order, single pole
    XOVER_SLOPE_12DB,      // 2nd order, 1 biquad
    XOVER_SLOPE_24DB,      // 4th order, 2 biquads
    XOVER_SLOPE_48DB       // 8th order, 4 biquads
} xover_slope_t;

typedef struct {
    xover_filter_type_t type;
    xover_slope_t slope;
    float freq;
    bool enabled;  // false = bypass
} xover_band_t;

typedef struct {
    xover_band_t hp;
    xover_band_t lp;
    bool phase_invert;   // false = 0°, true = 180°
    float delay_ms;      // -8.0 to +8.0 (negative = delay main instead)
    bool low_mono;       // true = L+R summed to mono
} crossover_settings_t;

// Returns dB magnitude for HP or LP filter at given frequency
float compute_xover_response(xover_band_t *xover, float freq_hz, float fs, bool is_highpass);

// Precomputes the UI render state for a crossover
void ui_xover_calc(xover_band_t *xover, float fs, bool is_highpass, ui_xover_t *out);

#endif
#ifndef PRESET_STORE_H
#define PRESET_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "eq_data.h"

#define NUM_PRESET_SLOTS 4

// Legacy single-blob layout (Phase 9 format). Still used on disk for the
// numbered slot-N saves and for migrating old "state" blobs into the new
// split layout. The working-state auto-save no longer writes this — it
// writes per-domain keys (bands/stage/xover/sys/ui). See preset_store.cpp.
typedef struct {
    uint8_t version;     // Phase 9: bumped to 2 when xover/sub fields were added to flash
    uint8_t input_muted;   // was _pad[0] — repurposed Phase 15 for per-stage mute persistence
    uint8_t output_muted;  // was _pad[1]
    uint8_t low_muted;     // was _pad[2] — zero in old saves → mute=false on migration
    eq_band_t input_bands_l[MAX_BANDS];
    eq_band_t input_bands_r[MAX_BANDS];
    int input_num_bands_l;
    int input_num_bands_r;
    int input_channel_mode;
    float input_gain_l_db;
    float input_gain_r_db;
    eq_band_t output_bands_l[MAX_BANDS];
    eq_band_t output_bands_r[MAX_BANDS];
    int output_num_bands_l;
    int output_num_bands_r;
    int output_channel_mode;
    float output_gain_l_db;
    float output_gain_r_db;
    bool limiter_enabled;
    float limiter_threshold;
    int backlight_val;
    int input_source_idx;
    bool noise_gen_enabled;
    float noise_gen_db;
    uint8_t visualizer_mode;
    uint32_t magic;

    // Phase 9: crossover and low output fields are now persisted.
    crossover_settings_t xover;
    eq_band_t low_bands_l[MAX_BANDS];
    eq_band_t low_bands_r[MAX_BANDS];
    int low_num_bands_l;
    int low_num_bands_r;
    int low_channel_mode;
    float low_gain_l_db;
    float low_gain_r_db;
    bool low_limiter_enabled;
    float low_limiter_threshold;
} preset_data_t;

typedef struct {
    preset_data_t current_state;
    int active_preset_slot;
    int active_tile;
    uint32_t magic;
} working_state_t;

// Magic bumped to 0x50455133 ("PEQ3") for the 48 kHz hardening change:
// sample_rate_idx removed from preset_data_t.  Old saves will fail the
// magic check and fall back to in-memory defaults (clean wipe).
#define PRESET_MAGIC 0x50455133
#define STATE_MAGIC  0x53545434
#define PRESET_VERSION 5

void preset_store_init(void);

// Mark non-UI state dirty. Debounced (5 s) write of any changed domain
// (bands / stage_meta / xover / sys). Per-domain CRC skip means calling
// this after a no-op edit doesn't touch flash.
void preset_mark_dirty(void);

// Mark only the UI domain dirty (active_tile, active_preset_slot).
// Uses a much longer debounce (30 s) and writes only the tiny "ui" key —
// so tab-switching doesn't spam flash. Separate entry point exists so
// hot UI paths don't trigger the full-state save cadence.
void preset_mark_ui_dirty(void);

void preset_check_save(void);
void preset_load_working_state(void);
void preset_force_save_working_state(void);
bool preset_save_slot(int slot);
bool preset_load_slot(int slot);
bool preset_slot_is_valid(int slot);
int preset_get_active_slot(void);
void preset_clear_active_slot(void);

#endif
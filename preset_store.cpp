#include "preset_store.h"
#include "eq_ui.h"
#include "dsp_engine.h"
#include "storage_backend.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

static bool state_dirty = false;
static uint32_t dirty_timestamp = 0;
static const uint32_t DEBOUNCE_MS = 3000;
static int active_preset_slot = -1;

// Audio-minimal-mode flag. See storage_backend.h for the full contract.
// Wrapped around save entry points that can erase NVS sectors / write SD
// blocks. Read by the audio task and the main loop().
volatile bool storage_busy = false;

// ============================================================
// Async storage writer task (Phase 10)
// ============================================================
// The debounced auto-save path used to call the storage write directly from
// the LVGL/timer context on Core 1. NVS writes lock the SPI flash bus, which
// stalls Core 0's audio task whenever it tries to fetch instructions from
// flash — causing audible dropouts. SD writes don't disable the I-cache but
// can still take 100s of ms.
//
// Solution: a dedicated task pinned to Core 1 owns the actual write. Calls
// route through storage_active->write_bytes(...), so the same task serves
// whichever backend (NVS or SD) is active. preset_check_save() packs the
// state into a staging buffer under a mutex and notifies the writer task.
// The writer wakes, takes the mutex, drains the staged buffer, and performs
// the backend write. Audio on Core 0 is unaffected (well, less affected —
// for NVS the IDF I2S/NVS internals still touch shared resources, but the
// worst-case window is much smaller).
//
// Synchronous saves (preset_save_slot, preset_force_save_working_state used
// at shutdown) bypass the task and write inline — those are user-initiated
// where the user expects the save to complete before returning.

static TaskHandle_t      nvs_writer_task_handle = NULL;
static SemaphoreHandle_t nvs_staging_mutex      = NULL;
static working_state_t   nvs_staging_buffer;     // produced by preset_check_save, consumed by task
static bool              nvs_staging_pending    = false;

static void pack_preset_from_stages(preset_data_t *p);
static void unpack_preset_to_stages(const preset_data_t *p);

static void nvs_writer_task(void *pvParameters) {
    (void)pvParameters;
    // Task-local snapshot buffer kept off the FreeRTOS task stack. The
    // 8 KB task stack has to absorb ~2 KB of FATFS internals during SD
    // writes; pushing the ~2.6 KB working_state_t onto it as well leaves
    // almost no headroom. Static is safe — only this task touches it.
    static working_state_t local;
    while (1) {
        // Block until preset_check_save() notifies us that there's something to write.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Snapshot the staging buffer under mutex so the producer can stage
        // a new save while we're writing.
        if (xSemaphoreTake(nvs_staging_mutex, portMAX_DELAY) == pdTRUE) {
            if (!nvs_staging_pending) {
                xSemaphoreGive(nvs_staging_mutex);
                continue;  // spurious wake
            }
            memcpy(&local, &nvs_staging_buffer, sizeof(local));
            nvs_staging_pending = false;
            xSemaphoreGive(nvs_staging_mutex);
        }

        // Perform the actual write outside the mutex. Goes to whichever
        // backend (NVS or SD) the boot selector chose. Audio task and LVGL
        // both observe storage_busy and step down to minimal mode for the
        // duration — so the cache-disable window (NVS) or slow SPI window
        // (SD) doesn't compete with non-essential cache traffic.
        uint32_t t0 = millis();
        storage_busy = true;
        storage_active->write_bytes(STORAGE_NS_WORK, "state", &local, sizeof(local));
        storage_busy = false;
        Serial.printf("[BUSY] autosave write: %lu ms\n", (unsigned long)(millis() - t0));
    }
}

void preset_store_init(void) {
    storage_active->begin();

    // Phase 10: async writer task. 8KB stack — NVS internals (the SPI
    // flash driver and underlying mbedtls/CRC code) consume more than the
    // initial 4KB guess. Priority 1 is just above idle — we want this to
    // yield to anything more urgent. Pinned to Core 1 so the flash bus lock
    // happens on the same core as LVGL (which can tolerate it) rather than
    // Core 0 (audio).
    nvs_staging_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(nvs_writer_task, "NVS_Writer", 8192, NULL, 1,
                            &nvs_writer_task_handle, 1);
}

void preset_mark_dirty(void) {
    state_dirty = true;
    dirty_timestamp = millis();
    active_preset_slot = -1; 
}

void preset_check_save(void) {
    if (!state_dirty) return;
    if ((millis() - dirty_timestamp) < DEBOUNCE_MS) return;

    // Pack the current state into the staging buffer and notify the writer
    // task. The actual flash write happens off the LVGL/audio path.
    if (nvs_staging_mutex && nvs_writer_task_handle) {
        if (xSemaphoreTake(nvs_staging_mutex, 0) == pdTRUE) {
            pack_preset_from_stages(&nvs_staging_buffer.current_state);
            nvs_staging_buffer.active_preset_slot = active_preset_slot;
            nvs_staging_buffer.active_tile        = sys_config.active_tile;
            nvs_staging_buffer.magic              = STATE_MAGIC;
            nvs_staging_pending                   = true;
            xSemaphoreGive(nvs_staging_mutex);
            xTaskNotifyGive(nvs_writer_task_handle);
            state_dirty = false;
        }
        // If we couldn't take the mutex (the writer task is mid-write),
        // leave state_dirty = true so we'll retry on the next tick.
    } else {
        // Fallback (should not happen post preset_store_init): synchronous
        // write so state isn't lost if the task hasn't been created.
        preset_force_save_working_state();
        state_dirty = false;
    }
}

void preset_force_save_working_state(void) {
    // Function-static so the ~2.6 KB blob doesn't live on the loopTask
    // stack. SD/FATFS uses ~2 KB of stack itself during writes, so a stack
    // local here was tipping us into a stack-canary panic. Single-thread
    // use only (LVGL UI / setup), so static is safe.
    static working_state_t ws;
    pack_preset_from_stages(&ws.current_state);
    ws.active_preset_slot = active_preset_slot;
    ws.active_tile = sys_config.active_tile;
    ws.magic = STATE_MAGIC;
    storage_active->write_bytes(STORAGE_NS_WORK, "state", &ws, sizeof(ws));
}

void preset_load_working_state(void) {
    static working_state_t ws;
    size_t len = 0;
    bool ok = storage_active->read_bytes(STORAGE_NS_WORK, "state",
                                          &ws, sizeof(ws), &len);
    if (ok && len == sizeof(ws) && ws.magic == STATE_MAGIC) {
        unpack_preset_to_stages(&ws.current_state);
        active_preset_slot = ws.active_preset_slot;
        sys_config.active_tile = ws.active_tile;
        Serial.printf("[PRESET] Session restored, slot=%d\n", active_preset_slot);
    } else {
        sys_config.active_tile = 1; // Default to Input EQ on fresh boot
    }
}

bool preset_save_slot(int slot) {
    if (slot < 0 || slot >= NUM_PRESET_SLOTS) return false;

    // Function-static — see preset_force_save_working_state for rationale.
    // preset_save_slot also calls force_save_working_state below, so
    // without this we'd have ~5 KB of large blobs on the loopTask stack
    // simultaneously plus FATFS internals.
    static preset_data_t p;
    pack_preset_from_stages(&p);
    
    char key[12];
    snprintf(key, sizeof(key), "slot%d", slot);

    // Remove first, then write. On NVS this signals the garbage collector
    // that the old block is reclaimable, reducing compaction pressure.
    // On SD, remove + write is just a normal file replace.
    uint32_t t0 = millis();
    storage_busy = true;
    storage_active->remove_key(STORAGE_NS_PRESETS, key);
    storage_active->write_bytes(STORAGE_NS_PRESETS, key, &p, sizeof(p));
    active_preset_slot = slot;
    preset_force_save_working_state();
    storage_busy = false;
    Serial.printf("[BUSY] slot save: %lu ms\n", (unsigned long)(millis() - t0));

    Serial.printf("[PRESET] Saved to slot %d (Cleared old blob first)\n", slot);
    return true;
}

bool preset_load_slot(int slot) {
    if (slot < 0 || slot >= NUM_PRESET_SLOTS) return false;
    static preset_data_t p;   // off-stack, see preset_save_slot
    char key[12];
    snprintf(key, sizeof(key), "slot%d", slot);
    size_t len = 0;
    bool ok = storage_active->read_bytes(STORAGE_NS_PRESETS, key,
                                          &p, sizeof(p), &len);
    if (ok && len == sizeof(p) && p.magic == PRESET_MAGIC) {
        unpack_preset_to_stages(&p);
        active_preset_slot = slot;
        // The trailing force_save below is the actual write; wrap it.
        storage_busy = true;
        preset_force_save_working_state(); // Sync to session memory
        storage_busy = false;
        state_dirty = true;
        dirty_timestamp = millis();
        return true;
    }
    return false;
}

bool preset_slot_is_valid(int slot) {
    if (slot < 0 || slot >= NUM_PRESET_SLOTS) return false;
    static preset_data_t p;   // off-stack
    char key[12];
    snprintf(key, sizeof(key), "slot%d", slot);
    size_t len = 0;
    bool ok = storage_active->read_bytes(STORAGE_NS_PRESETS, key,
                                          &p, sizeof(p), &len);
    return (ok && len == sizeof(p) && p.magic == PRESET_MAGIC);
}

int preset_get_active_slot(void) { return active_preset_slot; }
void preset_clear_active_slot(void) { active_preset_slot = -1; }

static void pack_preset_from_stages(preset_data_t *p) {
    memset(p, 0, sizeof(*p));
    p->version = PRESET_VERSION;
    memcpy(p->input_bands_l, stage_input.bands_l, sizeof(p->input_bands_l));
    memcpy(p->input_bands_r, stage_input.bands_r, sizeof(p->input_bands_r));
    p->input_num_bands_l = stage_input.num_bands_l;
    p->input_num_bands_r = stage_input.num_bands_r;
    p->input_channel_mode = (int)stage_input.channel_mode;
    p->input_gain_l_db = stage_input.stage_gain_l_db;
    p->input_gain_r_db = stage_input.stage_gain_r_db;
    memcpy(p->output_bands_l, stage_output.bands_l, sizeof(p->output_bands_l));
    memcpy(p->output_bands_r, stage_output.bands_r, sizeof(p->output_bands_r));
    p->output_num_bands_l = stage_output.num_bands_l;
    p->output_num_bands_r = stage_output.num_bands_r;
    p->output_channel_mode = (int)stage_output.channel_mode;
    p->output_gain_l_db = stage_output.stage_gain_l_db;
    p->output_gain_r_db = stage_output.stage_gain_r_db;
    p->limiter_enabled = sys_config.limiter_enabled;
    p->limiter_threshold = sys_config.limiter_threshold;
    p->backlight_val = sys_config.backlight_val;
    p->input_source_idx = sys_config.input_source_idx;
    // Phase 10: noise gen is no longer persisted.
    p->noise_gen_enabled = false;
    p->noise_gen_db      = -40.0f;
    // Phase 14: Persist visualizer mode.
    p->visualizer_mode   = (uint8_t)sys_config.visualizer_mode;

    // Phase 9: crossover + low output state
    p->xover = xover_settings;
    memcpy(p->low_bands_l, stage_low.bands_l, sizeof(p->low_bands_l));
    memcpy(p->low_bands_r, stage_low.bands_r, sizeof(p->low_bands_r));
    p->low_num_bands_l = stage_low.num_bands_l;
    p->low_num_bands_r = stage_low.num_bands_r;
    p->low_channel_mode = (int)stage_low.channel_mode;
    p->low_gain_l_db = stage_low.stage_gain_l_db;
    p->low_gain_r_db = stage_low.stage_gain_r_db;
    p->low_limiter_enabled = sys_config.low_limiter_enabled;
    p->low_limiter_threshold = sys_config.low_limiter_threshold;

    // Phase 15: per-stage mute persistence (stored in formerly-padding bytes)
    p->input_muted  = dsp_get_mute(0) ? 1 : 0;
    p->output_muted = dsp_get_mute(1) ? 1 : 0;
    p->low_muted    = dsp_get_mute(2) ? 1 : 0;

    p->magic = PRESET_MAGIC;
}

static void unpack_preset_to_stages(const preset_data_t *p) {
    memcpy(stage_input.bands_l, p->input_bands_l, sizeof(stage_input.bands_l));
    memcpy(stage_input.bands_r, p->input_bands_r, sizeof(stage_input.bands_r));
    stage_input.num_bands_l = p->input_num_bands_l;
    stage_input.num_bands_r = p->input_num_bands_r;
    stage_input.channel_mode = (channel_mode_t)p->input_channel_mode;
    stage_input.stage_gain_l_db = p->input_gain_l_db;
    stage_input.stage_gain_r_db = p->input_gain_r_db;
    stage_input.curve_dirty = true;
    memcpy(stage_output.bands_l, p->output_bands_l, sizeof(stage_output.bands_l));
    memcpy(stage_output.bands_r, p->output_bands_r, sizeof(stage_output.bands_r));
    stage_output.num_bands_l = p->output_num_bands_l;
    stage_output.num_bands_r = p->output_num_bands_r;
    stage_output.channel_mode = (channel_mode_t)p->output_channel_mode;
    stage_output.stage_gain_l_db = p->output_gain_l_db;
    stage_output.stage_gain_r_db = p->output_gain_r_db;
    stage_output.curve_dirty = true;
    sys_config.limiter_enabled = p->limiter_enabled;
    sys_config.limiter_threshold = p->limiter_threshold;
    sys_config.backlight_val = p->backlight_val;
    sys_config.input_source_idx = p->input_source_idx;
    // Phase 10/13: test signal mode is not restored from flash — always resets
    // to OFF at boot regardless of what's in the blob.
    sys_config.test_signal_mode = 0;
    sys_config.test_signal_db   = -40.0f;
    
    // Phase 14: restore persisted visualizer mode.
    sys_config.visualizer_mode   = p->visualizer_mode;
    sys_config.fft_enabled       = (p->visualizer_mode > 0);

    // Phase 9: crossover + low output state
    xover_settings = p->xover;
    memcpy(stage_low.bands_l, p->low_bands_l, sizeof(stage_low.bands_l));
    memcpy(stage_low.bands_r, p->low_bands_r, sizeof(stage_low.bands_r));
    stage_low.num_bands_l = p->low_num_bands_l;
    stage_low.num_bands_r = p->low_num_bands_r;
    stage_low.channel_mode = (channel_mode_t)p->low_channel_mode;
    stage_low.stage_gain_l_db = p->low_gain_l_db;
    stage_low.stage_gain_r_db = p->low_gain_r_db;
    stage_low.curve_dirty = true;
    sys_config.low_limiter_enabled = p->low_limiter_enabled;
    sys_config.low_limiter_threshold = p->low_limiter_threshold;
    // Push main-path settings to DSP
    dsp_update_limiter(sys_config.limiter_enabled, sys_config.limiter_threshold);
    dsp_set_input_gain(0, stage_input.stage_gain_l_db);
    dsp_set_input_gain(1, stage_input.stage_gain_r_db);
    dsp_set_output_gain(0, stage_output.stage_gain_l_db);
    dsp_set_output_gain(1, stage_output.stage_gain_r_db);
    dsp_set_fft_enabled(sys_config.fft_enabled);
    dsp_set_input_source(DSP_INPUT_I2S);
    dsp_update_noise_gen(true, sys_config.test_signal_db);
    hardware_set_backlight(sys_config.backlight_val);

    // Phase 9: push restored crossover + low settings to DSP.
    xover_update_hp(&xover_settings.hp);
    xover_update_lp(&xover_settings.lp);
    xover_set_phase_invert(xover_settings.phase_invert);
    xover_set_delay(xover_settings.delay_ms);
    xover_set_mono(xover_settings.low_mono);

    // Push low EQ bands (stage_idx = 2) and low stage gain
    for (int i = 0; i < stage_low.num_bands_l; i++) {
        dsp_update_band(2, 0, i, &stage_low.bands_l[i]);
    }
    for (int i = 0; i < stage_low.num_bands_r; i++) {
        dsp_update_band(2, 1, i, &stage_low.bands_r[i]);
    }
    dsp_set_active_bands(2, 0, stage_low.num_bands_l);
    dsp_set_active_bands(2, 1, stage_low.num_bands_r);
    low_set_output_gain(0, stage_low.stage_gain_l_db);
    low_set_output_gain(1, stage_low.stage_gain_r_db);
    low_update_limiter(sys_config.low_limiter_enabled, sys_config.low_limiter_threshold);

    // Phase 15: restore per-stage mute. Old saves have zero here → mute=false.
    dsp_set_mute(0, p->input_muted  != 0);
    dsp_set_mute(1, p->output_muted != 0);
    dsp_set_mute(2, p->low_muted    != 0);

    // Main EQ bands: Phase 5 and earlier did not push these in this function
    // (they're re-pushed from eq_ui.cpp on band events), but to be consistent
    // on preset load we push them too so the DSP state is in sync without
    // needing a UI round-trip.
    for (int i = 0; i < stage_input.num_bands_l; i++) {
        dsp_update_band(0, 0, i, &stage_input.bands_l[i]);
    }
    for (int i = 0; i < stage_input.num_bands_r; i++) {
        dsp_update_band(0, 1, i, &stage_input.bands_r[i]);
    }
    dsp_set_active_bands(0, 0, stage_input.num_bands_l);
    dsp_set_active_bands(0, 1, stage_input.num_bands_r);
    for (int i = 0; i < stage_output.num_bands_l; i++) {
        dsp_update_band(1, 0, i, &stage_output.bands_l[i]);
    }
    for (int i = 0; i < stage_output.num_bands_r; i++) {
        dsp_update_band(1, 1, i, &stage_output.bands_r[i]);
    }
    dsp_set_active_bands(1, 0, stage_output.num_bands_l);
    dsp_set_active_bands(1, 1, stage_output.num_bands_r);
}
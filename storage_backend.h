#ifndef STORAGE_BACKEND_H
#define STORAGE_BACKEND_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Storage backend abstraction
// ============================================================
// preset_store.cpp routes every persistence read/write through this vtable
// instead of calling Preferences (NVS) or SD directly. That lets us swap
// the underlying medium at boot — pick NVS for the standard build or SD
// for a card-backed build, without touching the preset/state code.
//
// Logical namespace names — backends map them to their own concrete storage
// (NVS namespace name, SD subdirectory, etc.). Use these constants instead of
// hardcoding strings at call sites.
#define STORAGE_NS_WORK    "work"      // live state autosave
#define STORAGE_NS_PRESETS "presets"   // numbered preset slots

typedef struct {
    // Initialize the backend (open NVS namespaces, mount SD, etc.).
    // Returns true on success. Idempotent; safe to call more than once.
    bool        (*begin)(void);

    // Read up to buf_len bytes for (ns, key) into buf. On success returns true
    // and sets *out_len to the number of bytes actually copied. On miss
    // (key absent / empty blob) returns false and *out_len is unchanged.
    bool        (*read_bytes)(const char *ns, const char *key,
                              void *buf, size_t buf_len, size_t *out_len);

    // Write len bytes from buf to (ns, key). Returns true if every byte
    // landed; false on partial write or hardware failure.
    bool        (*write_bytes)(const char *ns, const char *key,
                               const void *buf, size_t len);

    // Remove (ns, key). Returns true if the key was removed (or didn't exist).
    bool        (*remove_key)(const char *ns, const char *key);

    // Is the backend currently usable? NVS: true after begin(). SD: true if
    // the card is mounted and writable.
    bool        (*available)(void);

    // Human-readable backend name for logs / UI status. "NVS", "SD".
    const char *(*name)(void);
} storage_backend_t;

extern storage_backend_t storage_nvs;
extern storage_backend_t storage_sd;

// Set true by preset_store at the start of any save sequence and cleared
// at the end. Read by:
//   - dsp_task on Core 0  → skip meters / FFT capture / coeff sync while set
//   - main loop()         → skip lv_task_handler() while set, pausing LVGL
// On NVS the underlying erase disables the I-cache for 20-40 ms. Pausing
// LVGL means no Core 1 flash fetches refilling I-cache with drawing code,
// so audio-path flash code stays cached through the disable window.
// Single-byte volatile read/write on Xtensa is atomic; no mutex needed.
extern volatile bool storage_busy;

// Active backend pointer. preset_store reads/writes route through this.
// Defaults to NVS at link time; the boot-time selector (added later) can
// flip it to &storage_sd before the first read/write.
extern storage_backend_t *storage_active;

#ifdef __cplusplus
}
#endif

#endif // STORAGE_BACKEND_H

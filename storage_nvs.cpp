// ============================================================
// NVS backend implementation of storage_backend_t
// ============================================================
// Wraps Arduino Preferences for the two namespaces preset_store uses:
//   STORAGE_NS_WORK    -> "paraeq_work"   (live autosave)
//   STORAGE_NS_PRESETS -> "paraeq_pre"    (numbered slots)
//
// All Preferences calls live in this file. preset_store.cpp goes through
// the vtable in storage_backend.h.

#include "storage_backend.h"
#include <Preferences.h>
#include <string.h>

static Preferences s_prefs_work;
static Preferences s_prefs_presets;
static bool        s_begun = false;

static Preferences *prefs_for(const char *ns) {
    if (!ns) return NULL;
    if (strcmp(ns, STORAGE_NS_WORK)    == 0) return &s_prefs_work;
    if (strcmp(ns, STORAGE_NS_PRESETS) == 0) return &s_prefs_presets;
    return NULL;
}

static bool nvs_begin(void) {
    if (s_begun) return true;
    if (!s_prefs_work.begin   ("paraeq_work", false)) return false;
    if (!s_prefs_presets.begin("paraeq_pre",  false)) return false;
    s_begun = true;
    return true;
}

static bool nvs_read_bytes(const char *ns, const char *key,
                           void *buf, size_t buf_len, size_t *out_len) {
    Preferences *p = prefs_for(ns);
    if (!p || !key || !buf) return false;
    // Preferences::getBytes returns 0 if the key is absent or empty.
    // Caller treats both as "no data" and falls back to defaults.
    size_t copied = p->getBytes(key, buf, buf_len);
    if (copied == 0) return false;
    if (out_len) *out_len = copied;
    return true;
}

static bool nvs_write_bytes(const char *ns, const char *key,
                            const void *buf, size_t len) {
    Preferences *p = prefs_for(ns);
    if (!p || !key || !buf) return false;
    return p->putBytes(key, buf, len) == len;
}

static bool nvs_remove_key(const char *ns, const char *key) {
    Preferences *p = prefs_for(ns);
    if (!p || !key) return false;
    // Skip the underlying erase if the key was never written. NVS logs
    // "nvs_erase_key fail: <key> NOT_FOUND" via ESP_LOGE on a missing-key
    // remove, which is noisy for the first-ever save to a slot.
    if (!p->isKey(key)) return true;
    return p->remove(key);
}

static bool        nvs_available(void) { return s_begun; }
static const char *nvs_name     (void) { return "NVS"; }

storage_backend_t storage_nvs = {
    nvs_begin,
    nvs_read_bytes,
    nvs_write_bytes,
    nvs_remove_key,
    nvs_available,
    nvs_name,
};

// Default to NVS until/unless the boot selector (future step) flips it.
storage_backend_t *storage_active = &storage_nvs;

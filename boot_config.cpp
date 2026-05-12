// ============================================================
// Boot configuration — bootstrap selector for storage backend
// ============================================================
// Lives in its own NVS namespace ("cfg_boot") with a single key ("storage").
// Read at the very start of setup() so the rest of the firmware knows
// where to put presets, state, etc. This is the only NVS write that
// persists in SD mode — flipping the selector and rebooting is the
// supported path for switching backends.

#include "boot_config.h"
#include "storage_backend.h"
#include <Arduino.h>
#include <Preferences.h>
#include <string.h>

#define BOOT_NS  "cfg_boot"
#define KEY_STG  "storage"

// ------------------------------------------------------------
// Selector read/write
// ------------------------------------------------------------

uint8_t boot_config_get_storage(void) {
    Preferences p;
    if (!p.begin(BOOT_NS, true)) {       // read-only
        // Namespace doesn't exist yet (first boot) — that's fine.
        return BOOT_STORAGE_NVS;
    }
    uint8_t v = p.getUChar(KEY_STG, BOOT_STORAGE_NVS);
    p.end();
    if (v != BOOT_STORAGE_NVS && v != BOOT_STORAGE_SD) v = BOOT_STORAGE_NVS;
    return v;
}

bool boot_config_set_storage(uint8_t backend) {
    if (backend != BOOT_STORAGE_NVS && backend != BOOT_STORAGE_SD) return false;
    Preferences p;
    if (!p.begin(BOOT_NS, false)) return false;
    bool ok = (p.putUChar(KEY_STG, backend) == 1);
    p.end();
    return ok;
}

void boot_config_apply_storage(void) {
    uint8_t v = boot_config_get_storage();
    if (v == BOOT_STORAGE_SD) {
        storage_active = &storage_sd;
        Serial.println("[BOOT] Storage backend: SD");
    } else {
        storage_active = &storage_nvs;
        Serial.println("[BOOT] Storage backend: NVS");
    }
}

// ------------------------------------------------------------
// Serial command parsing
// ------------------------------------------------------------

static void handle_command(const char *cmd) {
    if (strcmp(cmd, ">storage") == 0) {
        Serial.printf("[BOOT] Current backend: %s\n",
                      storage_active ? storage_active->name() : "(none)");
        return;
    }
    if (strcmp(cmd, ">storage nvs") == 0) {
        if (boot_config_set_storage(BOOT_STORAGE_NVS)) {
            Serial.println("[BOOT] Storage selector = NVS. Rebooting...");
            delay(300);
            ESP.restart();
        } else {
            Serial.println("[BOOT] Failed to write selector");
        }
        return;
    }
    if (strcmp(cmd, ">storage sd") == 0) {
        if (boot_config_set_storage(BOOT_STORAGE_SD)) {
            Serial.println("[BOOT] Storage selector = SD. Rebooting...");
            delay(300);
            ESP.restart();
        } else {
            Serial.println("[BOOT] Failed to write selector");
        }
        return;
    }
    // Unknown command — silently ignore (avoid log spam from random serial bytes).
}

void boot_config_serial_poll(void) {
    static char   line[64];
    static size_t line_len = 0;

    while (Serial.available()) {
        int c = Serial.read();
        if (c < 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            line[line_len] = '\0';
            if (line_len > 0) handle_command(line);
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = (char)c;
        } else {
            // Overflow — reset and drop the line.
            line_len = 0;
        }
    }
}

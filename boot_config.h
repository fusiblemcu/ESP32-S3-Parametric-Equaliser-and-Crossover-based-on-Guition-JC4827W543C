#ifndef BOOT_CONFIG_H
#define BOOT_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Boot configuration
// ============================================================
// A tiny NVS namespace ("cfg_boot") that holds bootstrap settings the rest
// of the firmware needs *before* the main storage backend is initialized.
//
// Currently one byte: which storage backend the user wants for everything
// else (presets, live state, etc.). Stored in NVS regardless of that choice
// — it's the chicken-and-egg pointer to where everything else lives.
//
// Values for the "storage" byte:
#define BOOT_STORAGE_NVS  0
#define BOOT_STORAGE_SD   1

// Read cfg_boot/storage and set storage_active accordingly. Call this in
// setup() *before* preset_store_init(). If the key is absent (first boot)
// or invalid, defaults to NVS.
void boot_config_apply_storage(void);

// Read the currently-stored selector. Defaults to BOOT_STORAGE_NVS if absent.
uint8_t boot_config_get_storage(void);

// Persist a new selector value. Returns true on success. Does NOT reboot
// or change storage_active — caller should reboot for the change to apply.
bool boot_config_set_storage(uint8_t backend);

// Poll Serial for boot-config commands. Call from loop().
//   >storage         — print current backend
//   >storage nvs     — switch to NVS, write selector, reboot
//   >storage sd      — switch to SD, write selector, reboot
void boot_config_serial_poll(void);

#ifdef __cplusplus
}
#endif

#endif // BOOT_CONFIG_H

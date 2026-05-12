// ============================================================
// SD-card backend implementation of storage_backend_t
// ============================================================
// Backs preset/state storage with files on a microSD card over SPI (FSPI).
// Bus is dedicated to SD on this board — touch is on I2C, LCD is parallel.
//
// File layout on the card:
//   /paraeq/work/state.bin
//   /paraeq/presets/slot0.bin
//   /paraeq/presets/slot1.bin
//   ...
//
// Each file:
//   sd_file_header_t  (16 bytes: magic, version, payload_len, crc32)
//   payload bytes     (matches the blob format the caller passes)
//
// CRC32 catches torn writes (power loss between FAT update and data write).
// On read, mismatched CRC → treat as "key absent" so caller falls back to
// defaults instead of unpacking corrupt bytes.

#include "storage_backend.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <esp_rom_crc.h>
#include <string.h>

// --- Pin assignment (board-specific; matches the pinout the user supplied) ---
// Pin assignment confirmed against board schematic:
//   IO10 -> CD/DAT3 (CS in SPI mode)
//   IO11 -> CMD     (MOSI from MCU's perspective)
//   IO12 -> CLK
//   IO13 -> DAT0    (MISO from MCU's perspective)
// The original pinout doc labeled IO11 "TF_MISO" and IO13 "TF_MOSI", which
// was misleading — those labels were from the *card's* perspective, not
// the MCU's. The schematic is authoritative.
#define SD_CS_PIN    10
#define SD_MOSI_PIN  11
#define SD_CLK_PIN   12
#define SD_MISO_PIN  13
// 4 MHz for diagnostic — many SD cards fail to enumerate at higher speeds
// over GPIO-matrix-routed SPI on ESP32-S3 (HSPI here is not IOMUX-direct on
// these pins). Once mount succeeds we can crank this back up.
#define SD_FREQ_HZ   4000000

#define SD_FILE_MAGIC   0x53514550u   // 'PEQS' little-endian
#define SD_FILE_VERSION 1u

#define ROOT_DIR        "/paraeq"
#define WORK_DIR        "/paraeq/work"
#define PRESETS_DIR     "/paraeq/presets"

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t payload_len;
    uint32_t crc32;
} sd_file_header_t;

// IMPORTANT: must NOT be FSPI (SPI2_HOST) — the JC4827W543's LCD driver
// claims FSPI internally and routes it to its own pins via the GPIO matrix.
// Calling SPIClass(FSPI).begin(...) would hijack the LCD's SPI peripheral
// and remap it to our SD pins, corrupting the framebuffer (green tinge) and
// breaking SD init. HSPI (SPI3_HOST) is unused on this board.
static SPIClass s_sd_spi(HSPI);
static bool     s_begun = false;

// ------------------------------------------------------------
// Helpers
// ------------------------------------------------------------

// Build "/paraeq/<ns>/<key>.bin" into out. Returns false if it doesn't fit.
static bool path_for(const char *ns, const char *key, char *out, size_t out_len) {
    if (!ns || !key || !out) return false;
    int n = snprintf(out, out_len, "%s/%s/%s.bin", ROOT_DIR, ns, key);
    return (n > 0 && (size_t)n < out_len);
}

static bool ensure_dir(const char *path) {
    if (SD.exists(path)) return true;
    return SD.mkdir(path);
}

// ------------------------------------------------------------
// Vtable functions
// ------------------------------------------------------------

static bool sd_begin(void) {
    if (s_begun) return true;

    // Internal pull-ups on CS and MISO. Many SD breakouts rely on the host
    // for these; if the board doesn't have them, MISO floats (and CMD0
    // returns "no token received" because the card never sees CS go low
    // cleanly or the response bits never reach the MCU).
    pinMode(SD_CS_PIN,   INPUT_PULLUP);
    pinMode(SD_MISO_PIN, INPUT_PULLUP);
    digitalWrite(SD_CS_PIN, HIGH);   // ensure CS deasserted before bus init
    pinMode(SD_CS_PIN, OUTPUT);
    delay(5);

    s_sd_spi.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, s_sd_spi, SD_FREQ_HZ)) {
        Serial.println("[SD] mount failed (no card / bad format / wiring?)");
        return false;
    }

    uint8_t card_type = SD.cardType();
    if (card_type == CARD_NONE) {
        Serial.println("[SD] no card detected after mount");
        SD.end();
        return false;
    }

    // Make sure the directory tree exists. mkdir is idempotent via ensure_dir.
    if (!ensure_dir(ROOT_DIR))    { Serial.println("[SD] mkdir /paraeq failed");          SD.end(); return false; }
    if (!ensure_dir(WORK_DIR))    { Serial.println("[SD] mkdir /paraeq/work failed");     SD.end(); return false; }
    if (!ensure_dir(PRESETS_DIR)) { Serial.println("[SD] mkdir /paraeq/presets failed");  SD.end(); return false; }

    uint64_t card_mb = SD.cardSize() / (1024ULL * 1024ULL);
    Serial.printf("[SD] mounted, type=%u size=%llu MB\n", (unsigned)card_type, card_mb);

    s_begun = true;
    return true;
}

static bool sd_read_bytes(const char *ns, const char *key,
                          void *buf, size_t buf_len, size_t *out_len) {
    if (!s_begun || !buf) return false;

    char path[64];
    if (!path_for(ns, key, path, sizeof(path))) return false;
    if (!SD.exists(path)) return false;

    File f = SD.open(path, FILE_READ);
    if (!f) return false;

    sd_file_header_t hdr;
    if (f.read((uint8_t *)&hdr, sizeof(hdr)) != sizeof(hdr)) { f.close(); return false; }
    if (hdr.magic   != SD_FILE_MAGIC)   { f.close(); return false; }
    if (hdr.version != SD_FILE_VERSION) { f.close(); return false; }
    if (hdr.payload_len == 0)           { f.close(); return false; }

    // We need the FULL payload to verify the CRC even if the caller's buf
    // is smaller. To stay heap-light we read into the caller's buf first
    // (truncated), then stream the rest just for CRC. If buf is at least
    // payload_len, single read covers it.
    size_t to_copy = (buf_len < hdr.payload_len) ? buf_len : hdr.payload_len;
    if (f.read((uint8_t *)buf, to_copy) != (int)to_copy) { f.close(); return false; }

    uint32_t crc = esp_rom_crc32_le(0, (const uint8_t *)buf, to_copy);

    // Stream any remaining bytes through the CRC without storing them.
    size_t remaining = hdr.payload_len - to_copy;
    while (remaining > 0) {
        uint8_t tmp[64];
        size_t chunk = (remaining < sizeof(tmp)) ? remaining : sizeof(tmp);
        if (f.read(tmp, chunk) != (int)chunk) { f.close(); return false; }
        crc = esp_rom_crc32_le(crc, tmp, chunk);
        remaining -= chunk;
    }
    f.close();

    if (crc != hdr.crc32) {
        Serial.printf("[SD] CRC mismatch on %s — treating as missing\n", path);
        return false;
    }

    if (out_len) *out_len = to_copy;
    return true;
}

static bool sd_write_bytes(const char *ns, const char *key,
                           const void *buf, size_t len) {
    if (!s_begun || !buf || len == 0) return false;

    char path[64];
    if (!path_for(ns, key, path, sizeof(path))) return false;

    sd_file_header_t hdr;
    hdr.magic       = SD_FILE_MAGIC;
    hdr.version     = SD_FILE_VERSION;
    hdr.payload_len = (uint32_t)len;
    hdr.crc32       = esp_rom_crc32_le(0, (const uint8_t *)buf, len);

    File f = SD.open(path, FILE_WRITE);   // FILE_WRITE truncates on open
    if (!f) {
        Serial.printf("[SD] open-for-write failed: %s\n", path);
        return false;
    }
    bool ok = (f.write((const uint8_t *)&hdr, sizeof(hdr)) == sizeof(hdr))
           && (f.write((const uint8_t *)buf,  len)         == len);
    f.close();
    return ok;
}

static bool sd_remove_key(const char *ns, const char *key) {
    if (!s_begun) return false;
    char path[64];
    if (!path_for(ns, key, path, sizeof(path))) return false;
    if (!SD.exists(path)) return true;     // already absent — caller's intent satisfied
    return SD.remove(path);
}

static bool        sd_available(void) { return s_begun && (SD.cardType() != CARD_NONE); }
static const char *sd_name     (void) { return "SD"; }

storage_backend_t storage_sd = {
    sd_begin,
    sd_read_bytes,
    sd_write_bytes,
    sd_remove_key,
    sd_available,
    sd_name,
};

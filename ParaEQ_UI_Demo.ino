// Board: "ESP32S3 Dev Module" from esp32 Arduino Core by Espressif (v3.2.0)
// Libraries required:
//   - lvgl v9.2.2                (Library Manager)
//   - GFX Library for Arduino    (Library Manager, v1.5.6)
//   - Dev Device Pins            (Library Manager, v0.0.2)
//   - TAMC_GT911                 (Library Manager, v1.0.2)
//
// IMPORTANT: You must configure lv_conf.h before compiling.
// See README.md for required settings.

#include <lvgl.h>
#include <PINS_JC4827W543.h>
#include "TAMC_GT911.h"
#include "eq_ui.h"
#include "dsp_engine.h"
#include "i2s_input.h"
#include "preset_store.h"
#include "boot_config.h"
#include "storage_backend.h"   // storage_busy flag for the loop()-side LVGL pause

// Radio shutdown — we don't use WiFi or Bluetooth and they cost us:
//   - ~20–30 KB of IRAM (modem/baseband stubs) that competes with our
//     audio-path IRAM budget,
//   - ~40–60 KB of DRAM reserved for control blocks / buffers,
//   - periodic high-priority ISRs that can preempt the Core 0 audio task.
// Stock arduino-esp32 links both stacks in; we can't remove them from the
// image without an sdkconfig rebuild, but we can stop the controllers and
// release their reserved RAM back to the heap at runtime.
#include <esp_wifi.h>
#include <esp_bt.h>

// Touch Controller
#define TOUCH_SDA    8
#define TOUCH_SCL    4
#define TOUCH_INT    3
#define TOUCH_RST    38
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 272
TAMC_GT911 touchController = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);

// Display global variables
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

// LVGL tick source
uint32_t millis_cb(void) {
    return millis();
}

// LVGL display flush callback
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_disp_flush_ready(disp);
}

// LVGL touchpad read callback
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    touchController.read();
    if (touchController.isTouched && touchController.touches > 0) {
        data->point.x = touchController.points[0].x;
        data->point.y = touchController.points[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ParaEQ UI Demo starting...");

    // Shut down radios before anything else allocates, so the reclaimed
    // DRAM goes to our heap budget rather than staying pinned. Errors here
    // are non-fatal — on a freshly-booted chip these controllers are already
    // in a stopped state and return ESP_ERR_INVALID_STATE.
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_mem_release(ESP_BT_MODE_BTDM);  // irreversibly frees BT RAM
    Serial.println("Radios off: WiFi + BT disabled, RAM released");

    // Check initial memory
    Serial.printf("Initial free heap: %d bytes\n", ESP.getFreeHeap());

    // Init Display
    Serial.println("Initializing display...");
    if (!gfx->begin()) {
        Serial.println("ERROR: gfx->begin() failed!");
        while (true) { }
    }
    Serial.println("Display initialized OK");

    // --- FIXED NATIVE ESP32 PWM SETUP ---
    // Attaches pin to a hardware timer running at 40kHz with 8-bit resolution (0-255).
    // Bypasses the standard Arduino analogWrite wrapper to guarantee the high frequency sticks.
    ledcAttach(GFX_BL, 40000, 8); 
    ledcWrite(GFX_BL, 255); // Set to 100% brightness initially
    
    gfx->fillScreen(RGB565_BLACK);

    // Init touch
    Serial.println("Initializing touch controller...");
    touchController.begin();
    touchController.setRotation(ROTATION_INVERTED);
    Serial.println("Touch controller initialized");

    // Init LVGL
    Serial.println("Initializing LVGL...");
    lv_init();
    lv_tick_set_cb(millis_cb);
    Serial.println("LVGL core initialized");

    screenWidth = gfx->width();
    screenHeight = gfx->height();
    Serial.printf("Screen dimensions: %d x %d\n", screenWidth, screenHeight);

    // Draw buffer - larger buffer = fewer flushes per frame. 68 rows is optimized for S3.
    Serial.println("Allocating LVGL draw buffer...");
    bufSize = screenWidth * 68;
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
        Serial.println("First draw buffer allocation failed, trying again...");
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
    }
    if (!disp_draw_buf) {
        Serial.println("ERROR: LVGL draw buffer allocation failed!");
        while (true) { }
    }
    Serial.println("Draw buffer allocated successfully");

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    Serial.println("LVGL display created");

    // Create input device
    Serial.println("Creating input device...");
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    
    // Default scroll limit is 10. Lowering it makes the screen slightly more sensitive to swipes.
    lv_indev_set_scroll_limit(indev, 8);
    
    Serial.println("Input device created");

    // Build the EQ UI
    Serial.println("Creating EQ UI...");
    eq_ui_create();
    Serial.println("EQ UI created");

    // Initialize I2S full-duplex (PCM1808 ADC + PCM5102A DAC)
    Serial.println("Initializing I2S full-duplex...");
    if (!i2s_audio_init()) {
        Serial.println("CRITICAL ERROR: I2S init failed!");
        Serial.println("System will continue with noise generation only");
    } else {
        Serial.println("I2S initialized successfully");
    }

    // Initialize the DSP engine on Core 1
    Serial.println("Initializing DSP engine...");
    dsp_init();
    Serial.println("DSP engine initialized");

    // Apply boot-time storage backend selection. Reads cfg_boot/storage from
    // NVS and points storage_active at either &storage_nvs or &storage_sd.
    // Must run BEFORE preset_store_init() so the right backend gets begin()'d.
    boot_config_apply_storage();

    // Initialize preset storage and load working state
    Serial.println("Initializing preset storage...");
    preset_store_init();
    preset_load_working_state();
    eq_ui_update_preset_display();
    Serial.println("Preset system ready");

    Serial.println("=== SETUP COMPLETE ===");
    Serial.printf("Final free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Free internal: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void loop() {
    boot_config_serial_poll();   // dev-only command channel; cheap when idle
    // While a save is in progress (NVS sector erase or SD slow write), pause
    // LVGL so Core 1 doesn't refill the I-cache with drawing code and evict
    // audio-path flash code that the DSP task is also re-fetching post-cache-
    // resume. UI freezes briefly (~50-200 ms); audio stays smooth.
    if (!storage_busy) {
        lv_task_handler();
    }
    delay(5);
}

// Hardware callback from the UI layer
void hardware_set_backlight(int percent) {
    if (percent < 5) percent = 5;   // minimum floor — prevents black screen
    if (percent > 100) percent = 100;
    
    // Map 0-100% to 0-255 PWM duty cycle
    int pwm_val = (percent * 255) / 100;
    
    // --- FIXED NATIVE ESP32 PWM WRITE ---
    // Bypasses analogWrite() so the ESP32 core doesn't secretly revert our 40kHz timer to 1kHz
    ledcWrite(GFX_BL, pwm_val);
}
// Board: "ESP32S3 Dev Module" from esp32 Arduino Core by Espressif (v3.2.0)
// Libraries required:
//   - lvgl v9.2.2                (Library Manager)
//   - GFX Library for Arduino    (Library Manager, v1.5.6)
//   - Dev Device Pins            (Library Manager, v0.0.2)
//   - TAMC_GT911                 (Library Manager, v1.0.2)
//
// IMPORTANT: You must configure lv_conf.h before compiling.
// See README.md for required settings.

#include <lvgl.h>
#include <PINS_JC4827W543.h>
#include "TAMC_GT911.h"
#include "eq_ui.h"
#include "dsp_engine.h"
#include "i2s_input.h"
#include "preset_store.h"
#include "boot_config.h"
#include "storage_backend.h"   // storage_busy flag for the loop()-side LVGL pause

// Radio shutdown — we don't use WiFi or Bluetooth and they cost us:
//   - ~20–30 KB of IRAM (modem/baseband stubs) that competes with our
//     audio-path IRAM budget,
//   - ~40–60 KB of DRAM reserved for control blocks / buffers,
//   - periodic high-priority ISRs that can preempt the Core 0 audio task.
// Stock arduino-esp32 links both stacks in; we can't remove them from the
// image without an sdkconfig rebuild, but we can stop the controllers and
// release their reserved RAM back to the heap at runtime.
#include <esp_wifi.h>
#include <esp_bt.h>

// Touch Controller
#define TOUCH_SDA    8
#define TOUCH_SCL    4
#define TOUCH_INT    3
#define TOUCH_RST    38
#define TOUCH_WIDTH  480
#define TOUCH_HEIGHT 272
TAMC_GT911 touchController = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);

// Display global variables
uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;
lv_display_t *disp;
lv_color_t *disp_draw_buf;

// LVGL tick source
uint32_t millis_cb(void) {
    return millis();
}

// LVGL display flush callback
void my_disp_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    uint32_t w = lv_area_get_width(area);
    uint32_t h = lv_area_get_height(area);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
    lv_disp_flush_ready(disp);
}

// LVGL touchpad read callback
void my_touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
    touchController.read();
    if (touchController.isTouched && touchController.touches > 0) {
        data->point.x = touchController.points[0].x;
        data->point.y = touchController.points[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("ParaEQ UI Demo starting...");

    // Shut down radios before anything else allocates, so the reclaimed
    // DRAM goes to our heap budget rather than staying pinned. Errors here
    // are non-fatal — on a freshly-booted chip these controllers are already
    // in a stopped state and return ESP_ERR_INVALID_STATE.
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_mem_release(ESP_BT_MODE_BTDM);  // irreversibly frees BT RAM
    Serial.println("Radios off: WiFi + BT disabled, RAM released");

    // Check initial memory
    Serial.printf("Initial free heap: %d bytes\n", ESP.getFreeHeap());

    // Init Display
    Serial.println("Initializing display...");
    if (!gfx->begin()) {
        Serial.println("ERROR: gfx->begin() failed!");
        while (true) { }
    }
    Serial.println("Display initialized OK");

    // --- FIXED NATIVE ESP32 PWM SETUP ---
    // Attaches pin to a hardware timer running at 40kHz with 8-bit resolution (0-255).
    // Bypasses the standard Arduino analogWrite wrapper to guarantee the high frequency sticks.
    ledcAttach(GFX_BL, 40000, 8); 
    ledcWrite(GFX_BL, 255); // Set to 100% brightness initially
    
    gfx->fillScreen(RGB565_BLACK);

    // Init touch
    Serial.println("Initializing touch controller...");
    touchController.begin();
    touchController.setRotation(ROTATION_INVERTED);
    Serial.println("Touch controller initialized");

    // Init LVGL
    Serial.println("Initializing LVGL...");
    lv_init();
    lv_tick_set_cb(millis_cb);
    Serial.println("LVGL core initialized");

    screenWidth = gfx->width();
    screenHeight = gfx->height();
    Serial.printf("Screen dimensions: %d x %d\n", screenWidth, screenHeight);

    // Draw buffer - larger buffer = fewer flushes per frame. 68 rows is optimized for S3.
    Serial.println("Allocating LVGL draw buffer...");
    bufSize = screenWidth * 68;
    disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!disp_draw_buf) {
        Serial.println("First draw buffer allocation failed, trying again...");
        disp_draw_buf = (lv_color_t *)heap_caps_malloc(bufSize * 2, MALLOC_CAP_8BIT);
    }
    if (!disp_draw_buf) {
        Serial.println("ERROR: LVGL draw buffer allocation failed!");
        while (true) { }
    }
    Serial.println("Draw buffer allocated successfully");

    disp = lv_display_create(screenWidth, screenHeight);
    lv_display_set_flush_cb(disp, my_disp_flush);
    lv_display_set_buffers(disp, disp_draw_buf, NULL, bufSize * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    Serial.println("LVGL display created");

    // Create input device
    Serial.println("Creating input device...");
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_touchpad_read);
    
    // Default scroll limit is 10. Lowering it makes the screen slightly more sensitive to swipes.
    lv_indev_set_scroll_limit(indev, 8);
    
    Serial.println("Input device created");

    // Build the EQ UI
    Serial.println("Creating EQ UI...");
    eq_ui_create();
    Serial.println("EQ UI created");

    // Initialize I2S full-duplex (PCM1808 ADC + PCM5102A DAC)
    Serial.println("Initializing I2S full-duplex...");
    if (!i2s_audio_init()) {
        Serial.println("CRITICAL ERROR: I2S init failed!");
        Serial.println("System will continue with noise generation only");
    } else {
        Serial.println("I2S initialized successfully");
    }

    // Initialize the DSP engine on Core 1
    Serial.println("Initializing DSP engine...");
    dsp_init();
    Serial.println("DSP engine initialized");

    // Apply boot-time storage backend selection. Reads cfg_boot/storage from
    // NVS and points storage_active at either &storage_nvs or &storage_sd.
    // Must run BEFORE preset_store_init() so the right backend gets begin()'d.
    boot_config_apply_storage();

    // Initialize preset storage and load working state
    Serial.println("Initializing preset storage...");
    preset_store_init();
    preset_load_working_state();
    eq_ui_update_preset_display();
    Serial.println("Preset system ready");

    Serial.println("=== SETUP COMPLETE ===");
    Serial.printf("Final free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("Free internal: %d bytes\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

void loop() {
    boot_config_serial_poll();   // dev-only command channel; cheap when idle
    // While a save is in progress (NVS sector erase or SD slow write), pause
    // LVGL so Core 1 doesn't refill the I-cache with drawing code and evict
    // audio-path flash code that the DSP task is also re-fetching post-cache-
    // resume. UI freezes briefly (~50-200 ms); audio stays smooth.
    if (!storage_busy) {
        lv_task_handler();
    }
    delay(5);
}

// Hardware callback from the UI layer
void hardware_set_backlight(int percent) {
    if (percent < 5) percent = 5;   // minimum floor — prevents black screen
    if (percent > 100) percent = 100;
    
    // Map 0-100% to 0-255 PWM duty cycle
    int pwm_val = (percent * 255) / 100;
    
    // --- FIXED NATIVE ESP32 PWM WRITE ---
    // Bypasses analogWrite() so the ESP32 core doesn't secretly revert our 40kHz timer to 1kHz
    ledcWrite(GFX_BL, pwm_val);
}

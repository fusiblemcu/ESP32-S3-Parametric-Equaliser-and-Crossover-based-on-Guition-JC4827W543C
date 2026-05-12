#include "i2s_input.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>

// Channel handles for full-duplex operation
static i2s_chan_handle_t tx_chan = NULL;
static i2s_chan_handle_t rx_chan = NULL;
static i2s_chan_handle_t low_tx_chan = NULL; // Phase 2: second DAC output
static volatile bool i2s_running = false;
static volatile bool low_running = false;

bool i2s_audio_init(void) {
  if (i2s_running)
    return true;

  // Step 1: Allocate BOTH tx and rx channels in one call for full-duplex
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.dma_desc_num = I2S_DMA_BUF_COUNT;
  chan_cfg.dma_frame_num = I2S_DMA_BUF_LEN;
  chan_cfg.auto_clear = true;

  // Pass BOTH handles - this creates full-duplex mode
  esp_err_t err = i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);
  if (err != ESP_OK) {
    Serial.printf("I2S channel create failed: %s\n", esp_err_to_name(err));
    return false;
  }

  // Step 2: Create ONE config with BOTH dout and din
  // This SAME config is used for both TX and RX init.
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = (gpio_num_t)I2S_MCLK_PIN,
              .bclk = (gpio_num_t)I2S_BCK_PIN,
              .ws = (gpio_num_t)I2S_WS_PIN,
              .dout = (gpio_num_t)I2S_DOUT_PIN, // TX data to DAC
              .din = (gpio_num_t)I2S_DIN_PIN,   // RX data from ADC
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  // Initialize TX channel with the shared config
  err = i2s_channel_init_std_mode(tx_chan, &std_cfg);
  if (err != ESP_OK) {
    Serial.printf("I2S TX init failed: %s\n", esp_err_to_name(err));
    i2s_del_channel(tx_chan);
    i2s_del_channel(rx_chan);
    tx_chan = NULL;
    rx_chan = NULL;
    return false;
  }

  // Initialize RX channel with the SAME config
  err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
  if (err != ESP_OK) {
    Serial.printf("I2S RX init failed: %s\n", esp_err_to_name(err));
    i2s_del_channel(tx_chan);
    i2s_del_channel(rx_chan);
    tx_chan = NULL;
    rx_chan = NULL;
    return false;
  }

  // Master channels are enabled later (after Slave) to sync clocks

  i2s_running = true;
  Serial.println("I2S audio initialised: Full-duplex, Master mode with MCLK");
  Serial.printf("  BCK=%d, WS=%d, DIN=%d, DOUT=%d, MCLK=%d\n", I2S_BCK_PIN,
                I2S_WS_PIN, I2S_DIN_PIN, I2S_DOUT_PIN, I2S_MCLK_PIN);

  // ---------------------------------------------------------------
  // Phase 2: I2S_NUM_1 slave TX for low DAC (GPIO 15)
  // Master (I2S_NUM_0) must be running before allocating the slave so
  // that BCK/WS are already driven on the GPIO matrix.
  //
  // CRITICAL: We MUST use I2S_GPIO_UNUSED for bclk/ws in the slave
  // config. If we pass the actual GPIO numbers, the driver calls
  // gpio_reset_pin which disconnects I2S0's output signal routes,
  // killing all master clocks. Instead, after init, we manually
  // connect I2S1's input signals to the same pins using
  // esp_rom_gpio_connect_in_signal — this adds input routes WITHOUT
  // disturbing any existing output routes.
  // ---------------------------------------------------------------
  // Phase 2: I2S_NUM_1 slave TX for low DAC (GPIO 15)
  i2s_chan_config_t low_chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
  low_chan_cfg.dma_desc_num = I2S_DMA_BUF_COUNT;
  low_chan_cfg.dma_frame_num = I2S_DMA_BUF_LEN;
  low_chan_cfg.auto_clear = true;

  err = i2s_new_channel(&low_chan_cfg, &low_tx_chan, NULL);
  if (err != ESP_OK) {
    Serial.printf("I2S low channel create failed: %s\n", esp_err_to_name(err));
  } else {
    // Provide real GPIO pins so driver enables slave clock logic.
    // Even though low_tx_chan is a slave (external BCK/WS), the driver
    // still tracks sample_rate for DMA interrupt cadence; keep it in sync.
    i2s_std_config_t low_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk =
                    (gpio_num_t)I2S_BCK_PIN,  // Real pin - enables slave logic
                .ws = (gpio_num_t)I2S_WS_PIN, // Real pin - enables slave logic
                .dout = (gpio_num_t)I2S_LOW_DOUT_PIN,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {.mclk_inv = false,
                                 .bclk_inv = false,
                                 .ws_inv = false},
            },
    };

    err = i2s_channel_init_std_mode(low_tx_chan, &low_std_cfg);
    if (err != ESP_OK) {
      Serial.printf("I2S low TX init failed: %s\n", esp_err_to_name(err));
      i2s_del_channel(low_tx_chan);
      low_tx_chan = NULL;
    } else {
      // gpio_reset_pin was called by driver - re-route I2S0 master outputs
      // immediately
      esp_rom_gpio_connect_out_signal(I2S_BCK_PIN, I2S0O_BCK_OUT_IDX, false,
                                      false);
      esp_rom_gpio_connect_out_signal(I2S_WS_PIN, I2S0O_WS_OUT_IDX, false,
                                      false);

      // Now add I2S1 slave INPUT routes (uses I2S1I_*_IN_IDX, not
      // I2S1O_*_OUT_IDX)
      esp_rom_gpio_connect_in_signal(I2S_BCK_PIN, I2S1I_BCK_IN_IDX, false);
      esp_rom_gpio_connect_in_signal(I2S_WS_PIN, I2S1I_WS_IN_IDX, false);

      err = i2s_channel_enable(low_tx_chan);
      if (err != ESP_OK) {
        Serial.printf("I2S low TX enable failed: %s\n", esp_err_to_name(err));
        i2s_del_channel(low_tx_chan);
        low_tx_chan = NULL;
      } else {
        low_running = true;
        Serial.printf("I2S low output initialised: Slave TX on GPIO %d\n",
                      I2S_LOW_DOUT_PIN);
      }
    }
  }

  // Phase 3: Start the Master channels. tx_chan starts the clocks,
  // syncing all armed slaves simultaneously.
  err = i2s_channel_enable(rx_chan);
  if (err != ESP_OK) {
    Serial.printf("I2S RX enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = i2s_channel_enable(tx_chan);
  if (err != ESP_OK) {
    Serial.printf("I2S TX enable failed: %s\n", esp_err_to_name(err));
    i2s_channel_disable(rx_chan);
    return false;
  }

  return true;
}

bool i2s_audio_restart(uint32_t new_rate) {
  if (!tx_chan || !rx_chan)
    return false;

  // Disable both master channels
  i2s_channel_disable(tx_chan);
  i2s_channel_disable(rx_chan);

  // Also disable low channel while master clocks change
  if (low_tx_chan && low_running) {
    i2s_channel_disable(low_tx_chan);
  }

  // Reconfigure clock on master TX/RX.
  i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(new_rate);

  esp_err_t err = i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
  if (err != ESP_OK) {
    Serial.printf("I2S TX clock reconfig failed: %s\n", esp_err_to_name(err));
    return false;
  }

  err = i2s_channel_reconfig_std_clock(rx_chan, &clk_cfg);
  if (err != ESP_OK) {
    Serial.printf("I2S RX clock reconfig failed: %s\n", esp_err_to_name(err));
    return false;
  }

  // Low channel is a slave (external BCK/WS) but the driver still tracks
  // sample_rate for DMA buffer interrupt cadence. Without this reconfig the
  // low DMA frame timing stays on the old rate — at 96 kHz host with a
  // 48 kHz-configured slave the buffer underflows periodically.
  if (low_tx_chan && low_running) {
    err = i2s_channel_reconfig_std_clock(low_tx_chan, &clk_cfg);
    if (err != ESP_OK) {
      Serial.printf("I2S low TX clock reconfig failed: %s\n",
                    esp_err_to_name(err));
      // Non-fatal: keep going, just log.
    }
  }

  // 1. Re-enable low slave (clock reconfig already done above). Arms it.
  if (low_tx_chan && low_running) {
    err = i2s_channel_enable(low_tx_chan);
    if (err != ESP_OK) {
      Serial.printf("I2S low TX re-enable failed: %s\n", esp_err_to_name(err));
      low_running = false; // Non-fatal
    }
  }

  // 2. Re-enable master RX
  err = i2s_channel_enable(rx_chan);
  if (err != ESP_OK) {
    Serial.printf("I2S RX re-enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  // 3. Re-enable master TX (starts clocks)
  err = i2s_channel_enable(tx_chan);
  if (err != ESP_OK) {
    Serial.printf("I2S TX re-enable failed: %s\n", esp_err_to_name(err));
    return false;
  }

  Serial.printf("I2S restarted at %lu Hz\n", new_rate);
  return true;
}

void i2s_audio_deinit(void) {
  if (low_tx_chan) {
    i2s_channel_disable(low_tx_chan);
    i2s_del_channel(low_tx_chan);
    low_tx_chan = NULL;
    low_running = false;
  }
  if (tx_chan) {
    i2s_channel_disable(tx_chan);
    i2s_del_channel(tx_chan);
    tx_chan = NULL;
  }
  if (rx_chan) {
    i2s_channel_disable(rx_chan);
    i2s_del_channel(rx_chan);
    rx_chan = NULL;
  }
  i2s_running = false;
}

IRAM_ATTR size_t i2s_input_read(int32_t *buf, size_t frames,
                                uint32_t timeout_ms) {
  if (!rx_chan || !i2s_running)
    return 0;

  size_t bytes_to_read = frames * 2 * sizeof(int32_t);
  size_t bytes_read = 0;

  esp_err_t err = i2s_channel_read(rx_chan, buf, bytes_to_read, &bytes_read,
                                   pdMS_TO_TICKS(timeout_ms));
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
    return 0;

  return bytes_read / (2 * sizeof(int32_t));
}

IRAM_ATTR size_t i2s_output_write(const int32_t *buf, size_t frames,
                                  uint32_t timeout_ms) {
  if (!tx_chan || !i2s_running)
    return 0;

  size_t bytes_to_write = frames * 2 * sizeof(int32_t);
  size_t bytes_written = 0;

  esp_err_t err = i2s_channel_write(tx_chan, buf, bytes_to_write,
                                    &bytes_written, pdMS_TO_TICKS(timeout_ms));
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
    return 0;

  return bytes_written / (2 * sizeof(int32_t));
}

IRAM_ATTR bool i2s_audio_is_running(void) { return i2s_running; }

IRAM_ATTR size_t i2s_low_output_write(const int32_t *buf, size_t frames,
                                      uint32_t timeout_ms) {
  if (!low_tx_chan || !low_running)
    return 0;

  size_t bytes_to_write = frames * 2 * sizeof(int32_t);
  size_t bytes_written = 0;

  esp_err_t err = i2s_channel_write(low_tx_chan, buf, bytes_to_write,
                                    &bytes_written, pdMS_TO_TICKS(timeout_ms));
  if (err != ESP_OK && err != ESP_ERR_TIMEOUT)
    return 0;

  return bytes_written / (2 * sizeof(int32_t));
}

IRAM_ATTR bool i2s_low_is_running(void) { return low_running; }
#ifndef I2S_INPUT_H
#define I2S_INPUT_H

#include <stdint.h>
#include <stddef.h>

// I2S pin assignments - shared clocks for full-duplex
#define I2S_BCK_PIN    46
#define I2S_WS_PIN     9
#define I2S_MCLK_PIN   18
#define I2S_DIN_PIN    14   // RX from PCM1808 ADC
#define I2S_DOUT_PIN   17   // TX to PCM5102A DAC (main)
#define I2S_LOW_DOUT_PIN 15 // TX to second PCM5102A DAC (low)

// I2S configuration
#define I2S_SAMPLE_RATE   48000
#define I2S_DMA_BUF_COUNT 16
#define I2S_DMA_BUF_LEN   256

/**
 * Initialize I2S in full-duplex mode (TX + RX on same port, shared clocks).
 */
bool i2s_audio_init(void);

/**
 * Restart I2S with a new sample rate.
 */
bool i2s_audio_restart(uint32_t new_rate);

/**
 * Deinitialize I2S.
 */
void i2s_audio_deinit(void);

/**
 * Read stereo samples from I2S RX (ADC).
 */
size_t i2s_input_read(int32_t *buf, size_t frames, uint32_t timeout_ms);

/**
 * Write stereo samples to I2S TX (DAC).
 */
size_t i2s_output_write(const int32_t *buf, size_t frames, uint32_t timeout_ms);

/**
 * Check if I2S is running.
 */
bool i2s_audio_is_running(void);

/**
 * Write stereo samples to I2S low output (second DAC, I2S_NUM_1).
 */
size_t i2s_low_output_write(const int32_t *buf, size_t frames, uint32_t timeout_ms);

/**
 * Check if the low I2S channel is running.
 */
bool i2s_low_is_running(void);

#endif // I2S_INPUT_H
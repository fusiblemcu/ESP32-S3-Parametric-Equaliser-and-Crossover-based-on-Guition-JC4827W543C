# ESP32-S3-Parametric-Equaliser-and-Crossover-based-on-Guition-JC4827W543C
Touchscreen Parametric Equaliser and Crossover based on Guition JC4827W543C with I2S input and output

Flash settings:

* USB CDC On Boot: "Enabled"
* CPU Frequency: "240MHz (WiFi)"
* Core Debug Level: "Info"
* USB DFU On Boot: "Disabled"
* Erase All Flash Before Sketch Upload: "Disabled"
* Events Run On: "Core 1"
* Flash Mode: "QIO 80MHz"
* Flash Size: "4MB (32Mb)"
* JTAG Adapter: "Disabled"
* Arduino Runs On: "Core 1"
* USB Firmware MSC On Boot: "Disabled"
* Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
* PSRAM: "OPI PSRAM"
* Upload Mode: "UART0 / Hardware CDC"
* Upload Speed: "921600"
* USB Mode: "Hardware CDC and JTAG"
* Zigbee Mode: "Disabled"

---

## I2S Audio Connections

The firmware uses a single I2S peripheral in **full-duplex mode** with shared clock lines, driving one ADC input and two DAC outputs simultaneously.

### Shared Clocks

| GPIO | Signal | Description |
|------|--------|-------------|
| 18 | MCLK | Master clock — output to PCM1808 and PCM5102A(s) |
| 46 | BCK | Bit clock — shared by all I2S devices |
| 9 | WS | Word select / LRCK — shared by all I2S devices |

### Audio Input

| GPIO | Signal | Device | Direction |
|------|--------|--------|-----------|
| 14 | DIN | PCM1808 ADC | ESP32 ← ADC |

### Audio Output

| GPIO | Signal | Device | Description |
|------|--------|--------|-------------|
| 17 | DOUT | PCM5102A (main) | High / full-range output |
| 15 | DOUT | PCM5102A (low) | Low / subwoofer output |

> **Note:** Both DAC data lines share the same BCK, WS, and MCLK. The two DOUT lines carry the same clock domain but independent stereo data, allowing independent high/low channel DSP routing.

---
NOTE: Place lv_conf.h in arduino/libraries folder for compiling

UPDATE 08/07/26 

New GUI, test tone, visualiser mode, DSP code fixes 

## Changes (LLM Generated text)

### New
- **Whirlpool visualizer (mode 6)** — audio-reactive fractal flame (chaos-game IFS)
  rendered into a persistent PSRAM framebuffer. Eight spectral bands drive eight
  morphing transforms; response is level-independent (dB-relative + AGC), so it
  works at low listening volumes. Swipe up/down on the visualizer page to reach it.

### Audio fixes
- **SVF EQ coefficients corrected** (sub-55 Hz bands and high-Q low-mid bands).
  Previous formulas deviated up to 12 dB from the displayed curve, with
  wrong-signed shelf overshoot and asymmetric bell cuts. Now matched to the
  Simper reference: measured agreement with the on-screen curve ≤ 0.0012 dB,
  and the TDF2↔SVF topology handover is response-invisible. Low-frequency
  presets tuned by ear against the old response may need retouching.
- **Coefficient motion frozen during NVS saves** — parameter smoothing no longer
  calls flash-resident math while the flash cache is off, removing that
  contribution to save-time stalls.

### Stability / diagnostics
- Fixed a one-past-end buffer write in the FFT tap (latent memory corruption).
- Sub-DAC write shortfalls are now counted and reported on the `[DSP]` line
  (`lowdrop`) — makes any main/sub time slip visible instead of silent.
- Audio-task serial diagnostics are now guarded (never block the audio path)
  and can be compiled out via `DSP_SERIAL_DIAG` for silent release builds.

### Internals
- Removed dead code: unreachable duplicate SVF coefficient function and the
  unused IRAM assembly biquad (frees IRAM).
- Scope ring cursor made unsigned (defined wrap on multi-day uptimes);
  stale LOW_BLOCK documentation corrected.

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

Here's a README-ready section for the I2S pins:

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

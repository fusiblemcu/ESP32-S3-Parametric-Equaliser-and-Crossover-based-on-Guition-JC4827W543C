// IRAM-resident copy of ESP-DSP's dsps_biquad_f32_aes3.
//
// Verbatim from espressif/esp-dsp (Apache-2.0), with two changes:
//   1. Section changed from .text (flash) to .iram1.* (internal SRAM), so the
//      function survives the I-cache disable that happens during NVS flash
//      erase/write. Without this, every biquad call during an NVS commit stalls
//      for the full erase window (20–40 ms) and audio drops out at 96 kHz.
//   2. Symbol renamed from dsps_biquad_f32_aes3 to dsps_biquad_f32_iram to
//      avoid multiple-definition conflict with the precompiled libesp-dsp.a
//      already linked in by arduino-esp32.
//
// Upstream source:
//   https://github.com/espressif/esp-dsp/blob/master/modules/iir/biquad/dsps_biquad_f32_aes3.S
//
// Licensed under the Apache License, Version 2.0.

    .section .iram1.dsps_biquad_f32_iram,"ax",@progbits
    .align  4
    .global dsps_biquad_f32_iram
    .type   dsps_biquad_f32_iram,@function

// esp_err_t dsps_biquad_f32_iram(const float* input, float* output, int len, float* coef, float* w);
// DF-II biquad, identical semantics to dsps_biquad_f32_aes3.

dsps_biquad_f32_iram:
// input   - a2
// output  - a3
// len     - a4
// coeffs  - a5
// w       - a6

// f0 - b0
// f1 - b1
// f2 - b2
// f3 - a1
// f4 - a2

// f5 - w0
// f6 - w1

    entry   a1, 16
    // Array increment for floating point data should be 4
    lsi   f0, a5, 0
    lsi   f1, a5, 4
    lsi   f2, a5, 8
    lsi   f3, a5, 12
    lsi   f4, a5, 16


    neg.s  f5, f3   // -a[1]
    neg.s  f6, f4   // -a[2]

    lsi   f7, a6, 0 // w[0]
    lsi   f8, a6, 4 // w[1]

    addi    a3, a3, -4       // i-- // preset a3
    lsi     f9, a2, 0        // f9 = x[i]
    loopnez a4, .loop_bq_end_m_iram
        madd.s  f9, f7, f5   // f9 += -a1*w0
        addi    a3, a3, 4    // out++;
        mul.s   f10, f1, f7  // f10 = b1*w0
        madd.s  f9, f8, f6   // f9 += -a2*w1
        madd.s  f10, f9, f0  // f10 += b0*d0
        addi    a2, a2, 4    // in++;
        madd.s  f10, f2, f8  // f10+= b2*w1, f10 - result
        mov.s   f8, f7       // w1 = w0
        mov.s   f7, f9       // w0 = d0
        lsi     f9, a2,  0   // f9 = x[i]
        ssi     f10, a3, 0   // y[i] = result
.loop_bq_end_m_iram:
        // Store delay line
        ssi     f7, a6, 0
        ssi     f8, a6, 4

    movi.n  a2, 0 // return status ESP_OK
    retw.n

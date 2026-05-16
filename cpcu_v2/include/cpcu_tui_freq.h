/**
 *  @file   cpcu_tui_freq.h
 *  @brief  Frequency estimation utilities for the TUI waveform display.
 */

#ifndef CPCU_TUI_FREQ_H
#define CPCU_TUI_FREQ_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Sample rate / scale — change here if your build uses different values */
#ifndef FREQ_SAMPLE_RATE
#define FREQ_SAMPLE_RATE        2000.0f
#endif
#ifndef FREQ_ADC_MAX
#define FREQ_ADC_MAX            4095.0f
#endif
#ifndef FREQ_ADC_VREF
#define FREQ_ADC_VREF           3.3f
#endif

/* FFT size. Must be a power of two AND must equal WAVE_BUF_SIZE in cpcu_tui.c
 * (otherwise we'd need to copy/zero-pad).                                  */
#ifndef FREQ_FFT_N
#define FREQ_FFT_N              512
#endif
#define FREQ_NUM_BINS           (FREQ_FFT_N / 2)        /* 256 useful bins */
#define FREQ_BIN_HZ             (FREQ_SAMPLE_RATE / (float)FREQ_FFT_N)

/* EMG band of interest (used by dominant + mean to ignore DC and AC pickup
 * outside the surface-EMG range).                                          */
#ifndef FREQ_BAND_LO_HZ
#define FREQ_BAND_LO_HZ         15.0f
#endif
#ifndef FREQ_BAND_HI_HZ
#define FREQ_BAND_HI_HZ         500.0f
#endif


/* ──────────────────────────────────────────────────────────────────────
 *  RADIX-2 IN-PLACE FFT  (Cooley-Tukey, classical iterative form)
 *  ──────────────────────────────────────────────────────────────────── */

static inline void freq_fft_inplace(float *re, float *im, int N)
{
    /* Bit-reversal permutation */
    for(int i = 1, j = 0; i < N; i++)
    {
        int bit = N >> 1;
        for(; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if(i < j)
        {
            float t;
            t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    /* Butterflies */
    for(int len = 2; len <= N; len <<= 1)
    {
        float ang     = -2.0f * (float)M_PI / (float)len;
        float wlen_re = cosf(ang);
        float wlen_im = sinf(ang);
        for(int i = 0; i < N; i += len)
        {
            float w_re = 1.0f, w_im = 0.0f;
            for(int k = 0; k < len/2; k++)
            {
                float u_re = re[i+k];
                float u_im = im[i+k];
                float t_re = re[i+k+len/2]*w_re - im[i+k+len/2]*w_im;
                float t_im = re[i+k+len/2]*w_im + im[i+k+len/2]*w_re;
                re[i+k]         = u_re + t_re;
                im[i+k]         = u_im + t_im;
                re[i+k+len/2]   = u_re - t_re;
                im[i+k+len/2]   = u_im - t_im;
                float nw_re = w_re*wlen_re - w_im*wlen_im;
                float nw_im = w_re*wlen_im + w_im*wlen_re;
                w_re = nw_re; w_im = nw_im;
            }
        }
    }
}


/* ──────────────────────────────────────────────────────────────────────
 *  HANN WINDOW + DC-REMOVAL → MAGNITUDE SPECTRUM
 *  ──────────────────────────────────────────────────────────────────── */

/*  Compute the magnitude spectrum of the latest FREQ_FFT_N samples in a
 *  circular buffer. Writes FREQ_NUM_BINS magnitudes (volts) to `mag`.
 *  Returns false if there isn't enough data yet.
 *
 *  Internal scratch buffers are static — single-threaded callers only,
 *  which is fine for cpcu_tui and signal_testbench.                    */
static inline bool freq_spectrum(const uint16_t *buf,
                                 uint32_t buf_capacity,
                                 uint32_t start,
                                 uint32_t avail,
                                 float    mag[FREQ_NUM_BINS])
{
    if(avail < (uint32_t)FREQ_FFT_N) return false;

    static float fft_re[FREQ_FFT_N];
    static float fft_im[FREQ_FFT_N];

    /* Compute mean for DC removal */
    double sum = 0.0;
    uint32_t s = (start + avail - FREQ_FFT_N) % buf_capacity;
    for(int i = 0; i < FREQ_FFT_N; i++)
    {
        uint32_t idx = (s + i) % buf_capacity;
        sum += (double)buf[idx];
    }
    float mean_v = (float)(sum / (double)FREQ_FFT_N) / FREQ_ADC_MAX * FREQ_ADC_VREF;

    /* Window + DC-remove */
    for(int i = 0; i < FREQ_FFT_N; i++)
    {
        uint32_t idx = (s + i) % buf_capacity;
        float    v   = (float)buf[idx] / FREQ_ADC_MAX * FREQ_ADC_VREF - mean_v;
        /* Hann window — reduces spectral leakage from finite buffer. */
        float    w   = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i /
                                            (float)(FREQ_FFT_N - 1)));
        fft_re[i] = v * w;
        fft_im[i] = 0.0f;
    }

    freq_fft_inplace(fft_re, fft_im, FREQ_FFT_N);

    /* Scale: 2/N for one-sided spectrum (skip DC), then take magnitude */
    const float scale = 2.0f / (float)FREQ_FFT_N;
    for(int b = 0; b < FREQ_NUM_BINS; b++)
    {
        float re = fft_re[b], im = fft_im[b];
        mag[b] = scale * sqrtf(re*re + im*im);
    }
    return true;
}


/* ──────────────────────────────────────────────────────────────────────
 *  PUBLIC API  (compatible with v1.0 of this header)
 *  ──────────────────────────────────────────────────────────────────── */

/*  Dominant frequency in the EMG band. Returns 0 on insufficient data. */
static inline float freq_dominant_hz(const uint16_t *buf,
                                     uint32_t buf_capacity,
                                     uint32_t start,
                                     uint32_t avail)
{
    static float mag[FREQ_NUM_BINS];
    if(!freq_spectrum(buf, buf_capacity, start, avail, mag))
        return 0.0f;

    int b_lo = (int)(FREQ_BAND_LO_HZ / FREQ_BIN_HZ);
    int b_hi = (int)(FREQ_BAND_HI_HZ / FREQ_BIN_HZ);
    if(b_hi >= FREQ_NUM_BINS) b_hi = FREQ_NUM_BINS - 1;

    int   best = b_lo;
    float bm   = 0.0f;
    for(int b = b_lo; b <= b_hi; b++)
    {
        if(mag[b] > bm) { bm = mag[b]; best = b; }
    }
    return (float)best * FREQ_BIN_HZ;
}


/*  Mean frequency (centroid) of the EMG band: Σ(f·P)/Σ(P), P = |X|². */
static inline float freq_mean_hz(const uint16_t *buf,
                                 uint32_t buf_capacity,
                                 uint32_t start,
                                 uint32_t avail)
{
    static float mag[FREQ_NUM_BINS];
    if(!freq_spectrum(buf, buf_capacity, start, avail, mag))
        return 0.0f;

    int b_lo = (int)(FREQ_BAND_LO_HZ / FREQ_BIN_HZ);
    int b_hi = (int)(FREQ_BAND_HI_HZ / FREQ_BIN_HZ);
    if(b_hi >= FREQ_NUM_BINS) b_hi = FREQ_NUM_BINS - 1;

    double num = 0.0, den = 0.0;
    for(int b = b_lo; b <= b_hi; b++)
    {
        double p = (double)mag[b] * (double)mag[b];
        num += (double)b * FREQ_BIN_HZ * p;
        den += p;
    }
    if(den < 1e-12) return 0.0f;
    return (float)(num / den);
}


/*  Get the full spectrum for callers that want their own viz. The output
 *  is FREQ_NUM_BINS = N/2 = 256 floats (magnitudes in V).                */
static inline bool freq_full_spectrum(const uint16_t *buf,
                                      uint32_t buf_capacity,
                                      uint32_t start,
                                      uint32_t avail,
                                      float    out[FREQ_NUM_BINS])
{
    return freq_spectrum(buf, buf_capacity, start, avail, out);
}


/*  Tiny ASCII spectrum bar. `width` columns get mapped from
 *  [b_lo, b_hi] across the EMG band. 7-bit gradient so it works
 *  on every terminal.                                              */
static inline void freq_render_bar(int row, int col, int width,
                                   const float mag[FREQ_NUM_BINS])
{
    static const char gradient[] = " .:-=+*#";
    const int n_levels = (int)sizeof(gradient) - 1;

    int b_lo = (int)(FREQ_BAND_LO_HZ / FREQ_BIN_HZ);
    int b_hi = (int)(FREQ_BAND_HI_HZ / FREQ_BIN_HZ);
    if(b_hi >= FREQ_NUM_BINS) b_hi = FREQ_NUM_BINS - 1;
    int n_bins = b_hi - b_lo + 1;
    if(n_bins < 1) return;

    /* Find peak in band for normalization */
    float peak = 1e-9f;
    for(int b = b_lo; b <= b_hi; b++)
        if(mag[b] > peak) peak = mag[b];

    for(int x = 0; x < width; x++)
    {
        int   b = b_lo + (x * n_bins) / width;
        if(b > b_hi) b = b_hi;
        float frac = mag[b] / peak;
        if(frac < 0.0f) frac = 0.0f;
        if(frac > 1.0f) frac = 1.0f;
        int   lvl  = (int)(frac * (float)(n_levels - 1) + 0.5f);
        mvaddch(row, col + x, gradient[lvl]);
    }
}

#endif /* CPCU_TUI_FREQ_H */


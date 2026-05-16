/**
 *  @file   demo_signals.h
 *  @brief  Synthetic EMG signal generators for TUI demo mode.
 */

#ifndef DEMO_SIGNALS_H
#define DEMO_SIGNALS_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/* M_PI is a POSIX extension — not in strict ISO C. Define locally if
 * math.h didn't expose it (e.g. under -std=c11 without _GNU_SOURCE). */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*============= WAVEFORM TYPES =============================================================*/

typedef enum {
    WAVE_SINE       = 1,
    WAVE_SQUARE     = 2,
    WAVE_TRIANGLE   = 3,
    WAVE_SAWTOOTH   = 4,
    WAVE_NOISE      = 5,
    WAVE_EMG_BURST  = 6,
    WAVE_ECG        = 7,
    WAVE_CHIRP      = 8,
} DemoWave;

/*============= COMMON CONSTANTS ===========================================================*/

#ifndef DEMO_MIDRAIL_V
#define DEMO_MIDRAIL_V      1.65f       /* Mid-rail (1/2 Vref) */
#endif

#ifndef DEMO_AMPLITUDE_V
#define DEMO_AMPLITUDE_V    0.6f        /* Default peak amplitude */
#endif

/*============= LABELS =====================================================================*/

static inline const char *demo_wave_label(DemoWave w)
{
    switch(w)
    {
        case WAVE_SINE:         return "SINE";
        case WAVE_SQUARE:       return "SQUARE";
        case WAVE_TRIANGLE:     return "TRI";
        case WAVE_SAWTOOTH:     return "SAW";
        case WAVE_NOISE:        return "NOISE";
        case WAVE_EMG_BURST:    return "EMG";
        case WAVE_ECG:          return "ECG";
        case WAVE_CHIRP:        return "CHIRP";
        default:                return "?";
    }
}

/*============= INDIVIDUAL GENERATORS ======================================================*/

static inline float demo_gen_sine(float t, float freq, float phase_off)
{
    return DEMO_MIDRAIL_V
         + DEMO_AMPLITUDE_V * sinf(2.0f * (float)M_PI * freq * t + phase_off);
}

static inline float demo_gen_square(float t, float freq, float phase_off)
{
    float s = sinf(2.0f * (float)M_PI * freq * t + phase_off);
    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * ((s >= 0.0f) ? 1.0f : -1.0f);
}

static inline float demo_gen_triangle(float t, float freq, float phase_off)
{
    float phase  = fmodf(freq * t + phase_off / (2.0f * (float)M_PI), 1.0f);
    if(phase < 0.0f) phase += 1.0f;
    /* 0..0.5 → -1..+1, 0.5..1 → +1..-1 */
    float v = (phase < 0.5f)
              ? (4.0f * phase - 1.0f)
              : (3.0f - 4.0f * phase);
    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * v;
}

static inline float demo_gen_sawtooth(float t, float freq, float phase_off)
{
    float phase = fmodf(freq * t + phase_off / (2.0f * (float)M_PI), 1.0f);
    if(phase < 0.0f) phase += 1.0f;
    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * (2.0f * phase - 1.0f);
}

static inline float demo_gen_noise(float t, float freq, float phase_off)
{
    (void)t; (void)freq; (void)phase_off;
    /* Uniform white noise spanning the full ±amplitude range */
    float r = (float)rand() / (float)RAND_MAX;        /* 0..1 */
    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * (2.0f * r - 1.0f);
}

/**
 *  EMG burst: 1 s quiet baseline + 1 s contraction. Each contraction is
 *  high-frequency noise gated by an envelope that rises and falls like a
 *  real muscle activation. Channel phase_off staggers the burst start so
 *  different channels light up at slightly different moments, matching
 *  what you'd see with electrodes on different muscle bellies.
 */
static inline float demo_gen_emg_burst(float t, float freq, float phase_off)
{
    (void)freq;

    /* 2-second cycle, shifted per channel */
    float cycle_t = fmodf(t + phase_off * 0.2f, 2.0f);
    if(cycle_t < 0.0f) cycle_t += 2.0f;

    /* Envelope: 0 during rest (0..1 s), triangle rise/fall during burst (1..2 s) */
    float env;
    if(cycle_t < 1.0f)
    {
        env = 0.02f;                                     /* Baseline noise */
    }
    else
    {
        float b = cycle_t - 1.0f;                        /* 0..1 inside burst */
        env = (b < 0.3f) ? (b / 0.3f)                    /* Rise */
            : (b < 0.7f) ? 1.0f                          /* Plateau */
                         : (1.0f - (b - 0.7f) / 0.3f);   /* Fall */
    }

    /* High-frequency filler — tight pulses that read as "muscle activity" */
    float r = (float)rand() / (float)RAND_MAX;
    float hf = 2.0f * r - 1.0f;

    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * env * hf;
}

/**
 *  ECG: piecewise PQRST approximation. Heart rate = freq (interpreted as BPM).
 *      P  small positive hump around   10 % of the beat
 *      Q  tiny negative dip at         20 %
 *      R  big positive spike at        22 %
 *      S  negative overshoot at        24 %
 *      T  medium positive hump at      40 %
 */
static inline float demo_gen_ecg(float t, float freq, float phase_off)
{
    float bpm   = freq;
    if(bpm < 20.0f)   bpm = 60.0f;
    float beat_s = 60.0f / bpm;
    float phase  = fmodf(t + phase_off * beat_s / (2.0f * (float)M_PI), beat_s);
    if(phase < 0.0f) phase += beat_s;
    float p = phase / beat_s;                            /* 0..1 inside beat */

    /* Gaussian helper */
    #define GAUSS(x, mu, sig) expf(-((x)-(mu))*((x)-(mu)) / (2.0f*(sig)*(sig)))

    float v = 0.0f;
    v += 0.15f * GAUSS(p, 0.10f, 0.025f);                /* P */
    v -= 0.10f * GAUSS(p, 0.20f, 0.010f);                /* Q */
    v += 1.00f * GAUSS(p, 0.22f, 0.008f);                /* R (big spike) */
    v -= 0.20f * GAUSS(p, 0.24f, 0.010f);                /* S */
    v += 0.30f * GAUSS(p, 0.40f, 0.035f);                /* T */

    #undef GAUSS

    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * v;
}

/**
 *  Linear chirp from freq to 5*freq over a 2-second sweep, then repeats.
 *  Useful for spotting frequency-dependent artifacts in the signal chain.
 */
static inline float demo_gen_chirp(float t, float freq, float phase_off)
{
    float sweep_dur = 2.0f;
    float cycle_t   = fmodf(t, sweep_dur);
    if(cycle_t < 0.0f) cycle_t += sweep_dur;
    float f0 = freq;
    float f1 = freq * 5.0f;
    /* Instantaneous frequency: f0 + (f1-f0) * cycle_t/sweep_dur.
     * Phase integral: 2π (f0 t + (f1-f0)/(2 sweep_dur) t²). */
    float rate  = (f1 - f0) / sweep_dur;
    float phase = 2.0f * (float)M_PI * (f0 * cycle_t + 0.5f * rate * cycle_t * cycle_t)
                  + phase_off;
    return DEMO_MIDRAIL_V + DEMO_AMPLITUDE_V * sinf(phase);
}

/*============= DISPATCH ===================================================================*/

/**
 *  Top-level generator: returns a voltage sample for the given waveform,
 *  simulated time t (seconds), nominal frequency (Hz), and per-channel
 *  phase offset (radians).
 *
 *  Always returns a value in [0, 3.3] V — clamps if math overshoots.
 */
static inline float demo_gen(DemoWave wave, float t, float freq, float phase_off)
{
    float v;
    switch(wave)
    {
        case WAVE_SINE:       v = demo_gen_sine(t, freq, phase_off);       break;
        case WAVE_SQUARE:     v = demo_gen_square(t, freq, phase_off);     break;
        case WAVE_TRIANGLE:   v = demo_gen_triangle(t, freq, phase_off);   break;
        case WAVE_SAWTOOTH:   v = demo_gen_sawtooth(t, freq, phase_off);   break;
        case WAVE_NOISE:      v = demo_gen_noise(t, freq, phase_off);      break;
        case WAVE_EMG_BURST:  v = demo_gen_emg_burst(t, freq, phase_off);  break;
        case WAVE_ECG:        v = demo_gen_ecg(t, freq, phase_off);        break;
        case WAVE_CHIRP:      v = demo_gen_chirp(t, freq, phase_off);      break;
        default:              v = demo_gen_sine(t, freq, phase_off);       break;
    }
    if(v < 0.0f)    v = 0.0f;
    if(v > 3.3f)    v = 3.3f;
    return v;
}

#endif /* DEMO_SIGNALS_H */


#ifndef OB1203_HR_H
#define OB1203_HR_H

#include <stdint.h>

typedef struct
{
    float     fs;                /* Sampling rate in Hz */
    float     dc;                /* DC baseline estimate */
    float     alpha_dc;          /* Smoothing factor for DC (0..1) */

    float     prev1;             /* Previous filtered sample (n-1) */
    float     prev2;             /* Previous filtered sample (n-2) */

    float     threshold;         /* Adaptive peak threshold */
    float     threshold_decay;   /* Threshold decay factor */

    uint32_t  sample_index;      /* Global sample counter */
    uint32_t  last_peak_index;   /* Index of last detected peak */

    float     bpm;               /* Smoothed heart rate estimate */
    uint8_t   has_bpm;           /* 0 = no valid BPM yet, 1 = valid */

    /* Refractory period in samples (min distance between peaks).
     * Example: 0.3s at Fs=50Hz -> 15 samples.
     */
    uint32_t  refractory_samples;
} ob1203_hr_state_t;

/* Initialize heart rate estimator */
void ob1203_hr_init(ob1203_hr_state_t * s, float fs_hz);

/* Feed one new raw IR sample.
 * Returns 1 if BPM was updated, 0 otherwise.
 */
uint8_t ob1203_hr_update(ob1203_hr_state_t * s, uint32_t ir_raw);

#endif /* OB1203_HR_H */

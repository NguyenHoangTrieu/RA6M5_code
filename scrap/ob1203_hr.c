#include "ob1203_hr.h"
#include <string.h>   /* memset */
#include <math.h>     /* fabsf */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void ob1203_hr_init(ob1203_hr_state_t * s, float fs_hz)
{
    if (!s)
    {
        return;
    }

    memset(s, 0, sizeof(*s));

    s->fs       = fs_hz;       /* Effective sample rate (Hz) */
    s->alpha_dc = 0.005f;       /* Slow DC tracking (0.01 ~ 1% step) */

    s->threshold        = 0.0f;
    s->threshold_decay  = 0.999f;  /* Threshold decays over time */
    s->refractory_samples = (uint32_t)(0.5f * fs_hz); /* 300ms min between peaks */

    s->bpm     = 0.0f;
    s->has_bpm = 0U;
}

/* Internal helper to update adaptive threshold */
static void ob1203_hr_update_threshold(ob1203_hr_state_t * s, float peak_val)
{
    float abs_peak = fabsf(peak_val);

    if (s->threshold < 1e-3f)
    {
        /* First time: seed threshold with initial amplitude */
        s->threshold = abs_peak * 0.5f;
        return;
    }

    /* Threshold decays slowly over time */
    s->threshold *= s->threshold_decay;

    /* If we have a new, stronger peak, adjust threshold upwards */
    if (abs_peak > s->threshold)
    {
        s->threshold = 0.7f * s->threshold + 0.3f * abs_peak;
    }
}

/* Feed one IR sample; returns 1 if BPM updated, else 0 */
uint8_t ob1203_hr_update(ob1203_hr_state_t * s, uint32_t ir_raw)
{
    if (!s)
    {
        return 0U;
    }

    /* Convert raw 18-bit IR sample to float.
     * You can scale or normalize if needed.
     */
    float x_raw = (float) ir_raw;

    /* --- DC removal (high-pass) --- */
    if (s->dc == 0.0f)
    {
        /* Initialize baseline with first sample */
        s->dc = x_raw;
    }
    else
    {
        s->dc += s->alpha_dc * (x_raw - s->dc);
    }

    float x = x_raw - s->dc;   /* AC component */

    /* --- Peak detection using previous two samples (n-1, n-2) --- */
    uint8_t bpm_updated = 0U;

    if (s->sample_index >= 2U)
    {
        float x_n   = x;
        float x_n1  = s->prev1;  /* sample at (n-1) */
        float x_n2  = s->prev2;  /* sample at (n-2) */

        /* We consider x_n1 as a candidate peak if:
         *  - It is greater than both neighbors (x_n2 and x_n)
         *  - It is positive and above threshold
         *  - Enough time has passed since last peak (refractory period)
         */
        uint32_t idx_n1 = s->sample_index - 1U;
        uint32_t dt     = idx_n1 - s->last_peak_index;

        if ((x_n1 > x_n2) &&
            (x_n1 > x_n)  &&
            (x_n1 > 0.0f) &&
            (fabsf(x_n1) > s->threshold * 0.8f) &&
            (dt > s->refractory_samples))
        {
            /* Peak detected at sample index (n-1) */
            if (s->last_peak_index > 0U)
            {
                uint32_t samples_between = idx_n1 - s->last_peak_index;

                if (samples_between > 0U)
                {
                    /* Instantaneous BPM */
                    float inst_bpm = 60.0f * s->fs / (float) samples_between;

                    /* Basic sanity range: 30..220 BPM */
                    if ((inst_bpm > 45.0f) && (inst_bpm < 140.0f))
                    {
                        if (!s->has_bpm)
                        {
                            s->bpm     = inst_bpm;
                            s->has_bpm = 1U;
                        }
                        else
                        {
                            if (fabsf(inst_bpm - s->bpm) > 40.0f)
                            {
                            }
                            else
                            {
                            /* Smooth BPM with simple low-pass filter */
                            const float alpha_bpm = 0.1f;   /* 0..1, higher = faster tracking */
                            s->bpm = (1.0f - alpha_bpm) * s->bpm +
                                     alpha_bpm * inst_bpm;
                            }
                        }

                        bpm_updated = 1U;
                    }
                }
            }

            /* Update last_peak_index */
            s->last_peak_index = idx_n1;

            /* Update adaptive threshold using peak amplitude */
            ob1203_hr_update_threshold(s, x_n1);
        }
        else
        {
            /* No peak this sample; threshold slowly decays toward zero */
            s->threshold *= s->threshold_decay;
        }
    }

    /* Shift history: prev2 <- prev1, prev1 <- x */
    s->prev2 = s->prev1;
    s->prev1 = x;

    /* Advance global sample counter */
    s->sample_index++;

    return bpm_updated;
}

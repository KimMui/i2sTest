/*
* Copyright (c) 2015 Google Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
* contributors may be used to endorse or promote products derived from this
* software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
* PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*!
Resample PCM data to hard coded sample rate.
 - Scaler operations (no floating point operations used)
 - liner resamping algorithm
 - Only Upscale sampling (only for frequencies lower or equal to output sample rate)
 - Only 16bit samples supported
*/

#include "stdafx.h"
#include "rescale_pcm.h"

//maximum number of channels for upscaling
#define MAX_CHANNELS 6

// Target Frequency we are upscaling to
#define MATCH_FREQ 48000

#define MAX_INT 0xFFFFFFFF
#define INTEGER_SCALE_FACTOR (MAX_INT/MATCH_FREQ)

//#define FIND_UPSCALE_POINTS
#define FIND_OFFSET 0xccc


typedef struct {
    uint32_t channels;
    uint32_t multiplier;
    bool sub_sample;
    uint32_t sub_sample_count;
} resample_type;

#define RESAMPLE_STREAM_SIGNATURE 0XDDEEFFBB
typedef struct {
    uint32_t signature;
    resample_type match_resample;
    resample_type current_resample_counts;
    uint32_t current_num_resamples;
    uint32_t last_decode_freq;
} resample_stream_type;

// forward decl
static void calc_resample_rates(resample_stream_type *p_stream,
                                uint32_t input_freq,
                                uint32_t channels,
                                resample_type *p_resample);
static void calc_next_resample(resample_stream_type *p_stream);

/*!
Initialize internal variables
@return pointer to memory alocated for audio stream upscaling
*/
STREAM_HANDLE upscale_engine_init()
{
    resample_stream_type *p_stream;
    p_stream = (resample_stream_type *)malloc(sizeof(resample_stream_type));

    if (p_stream)
    {
        p_stream->signature = RESAMPLE_STREAM_SIGNATURE;
        memset(&p_stream->match_resample, 0, sizeof(p_stream->match_resample));
        memset(&p_stream->current_resample_counts, 0, sizeof(p_stream->current_resample_counts));
        p_stream->current_num_resamples = 0;
        p_stream->last_decode_freq = 0;
    }

    return p_stream;
}

/*!
Free allocated memory
@param handle pointer to resample_stream_type
@return true if handle pointer tested valid and was freed
*/
int upscale_engine_deinit(STREAM_HANDLE stream_handle)
{
    int ret_value = RESCALE_PCM_OK;
    resample_stream_type *p_stream = (resample_stream_type *)stream_handle;

    if (p_stream->signature != RESAMPLE_STREAM_SIGNATURE) {
        ret_value = RESCALE_PCM_HANDLE_INVALID;
    }
    else {
        p_stream->signature = 0;  //just in case someone passes a stale pointer.
        free(stream_handle);
    }

    return ret_value;
}

/*!
Precompute internal values for resampling from a lower rate to a higher rate
@param p_stream pointer struct per each audio stream
@param input_freq sample rate of source signal
@param channels number of channels in source signal
@param pResample pointer to struct
*/
static void calc_resample_rates(resample_stream_type *p_stream,
                                uint32_t input_freq,
                                uint32_t channels, 
                                resample_type *p_resample)
{
    p_resample->channels = channels;

    // Calculate each time a lower frequency signal must be resampled to generate a higher frequency signal
    p_resample->multiplier = MATCH_FREQ / input_freq;
    if (MATCH_FREQ % input_freq) {
        p_resample->sub_sample = true;

        /* Disregarding integer scaling, this is what is happening in terms of floating point.
                = MATCH_FREQ/input_freq = mantissa.exponent
                = mantissa.exponent - mantissa = exponent
                the exponent is added for each sample until it adds up to a whole number.
                once it adds up to a whole number add a resample.
                A whole number is INTEGER_SCALE_FACTOR
                Thus when the added sub_sample_count is larger than the INTEGER_SCALE_FACTOR we add a sample
                */
        p_resample->sub_sample_count = (MATCH_FREQ / (float)input_freq - p_resample->multiplier) * INTEGER_SCALE_FACTOR;
    }
    else {
        p_resample->sub_sample = false;
    }
}

/*!
Update the output resample count for the next input sample.
@param p_stream pointer struct per each audio stream
*/
static void calc_next_resample(resample_stream_type *p_stream)
{
    p_stream->current_num_resamples += p_stream->match_resample.multiplier;

    if (p_stream->match_resample.sub_sample) {
        // sum p_stream->current_resample_counts
        p_stream->current_resample_counts.sub_sample_count += p_stream->match_resample.sub_sample_count;
        if (INTEGER_SCALE_FACTOR < p_stream->current_resample_counts.sub_sample_count) {
            p_stream->current_resample_counts.sub_sample_count %= INTEGER_SCALE_FACTOR;
            p_stream->current_num_resamples++;
        }
    }
}

/*!
Resample input, copying result to output.

Samples are processed by available space.

If an overrun condition occurs (not enough space available to expand all samples)
 - The output buffer will be filled completely.  
 - Its possible the number of samples per channel are not fully expanded.
   In this case the index of the sample per channel is saved
   The next call to this function will start the previous index of expansion

@param handle pointer to resample_stream_type
@param p_input_sample_buffer pointer to input samples
@param p_input_samples IN pointer to number of input samples
                       OUT pointer to number of input samples processed
@param p_output_sample_buffer pointer to output sample buffer
@param p_output_samples IN pointer to number of output samples available.
                        OUT pointer to number of output samples filled.
@return error conditon non-zero
*/
int upscale_engine_resample_copy(STREAM_HANDLE stream_handle,
                                 int16_t *p_input_sample_buffer,
                                 uint32_t *p_input_samples,
                                 int16_t *p_output_sample_buffer,
                                 uint32_t *p_output_samples)
{
    uint32_t curr_input_index = 0;
    Int32 curr_output_index = 0;
    int16_t linear_inc[MAX_CHANNELS];
    int16_t linear_sum[MAX_CHANNELS];
    uint32_t inc_channel;

    int ret_value = RESCALE_PCM_OK;
    resample_stream_type *p_stream = (resample_stream_type *)stream_handle;

    if (p_stream->signature != RESAMPLE_STREAM_SIGNATURE) {
        ret_value = RESCALE_PCM_HANDLE_INVALID;
    }
    else {
        if (p_stream->last_decode_freq == MATCH_FREQ) {
            // No rate conversion required, copy input to output verbatim
            if (*p_input_samples <= *p_output_samples) {
                memcpy(p_output_sample_buffer, p_input_sample_buffer, *p_input_samples*sizeof(int16_t));
                *p_output_samples = *p_input_samples;
            }
            else {
                ret_value = RESCALE_PCM_OVERRUN;
                memcpy(p_output_sample_buffer, p_input_sample_buffer, *p_output_samples*sizeof(int16_t));
                *p_input_samples = *p_output_samples;
            }
        }
        else {
            // Adjust the output index to a negative number in case the last number of channels was incomplete
            if (0 != p_stream->current_resample_counts.channels) {
                curr_output_index = p_stream->current_resample_counts.channels - (p_stream->match_resample.channels - 1);
                GT_1trace(curTrace, GT_4CLASS, "current channel count was non-zero - resuming at output index %u\n", curr_output_index);
            }

            while ((curr_input_index < *p_input_samples) && (curr_output_index < (Int32)*p_output_samples)) {
                // linear interpolate
                inc_channel = 0;
                while (p_stream->match_resample.channels > inc_channel) {
                    if (1 < p_stream->current_num_resamples) {
                        if (curr_input_index == (*p_input_samples - p_stream->match_resample.channels)) {
                            //can't adjust to the next sample
                            linear_inc[inc_channel] = 0;
                        }
                        else {
                            // difference between current and next sample
                            linear_inc[inc_channel] = p_input_sample_buffer[curr_input_index + p_stream->match_resample.channels + inc_channel] - p_input_sample_buffer[curr_input_index + inc_channel];

                            // Divide by the number of repeated samples
                            linear_inc[inc_channel] = linear_inc[inc_channel] / ((int16_t)(p_stream->current_num_resamples));
                        }
                    }
                    linear_sum[inc_channel] = 0;
                    inc_channel++;
                }

                // Check for resamples to copy into input buffer
                // Note, this will catch left over resamples from previous call
                while ((curr_output_index < (Int32)*p_output_samples) && p_stream->current_num_resamples) {
                    // Copy data for each channel
                    while (p_stream->match_resample.channels > p_stream->current_resample_counts.channels) {
                        // Copy samples from lower frequency to higher frequency
                        p_output_sample_buffer[curr_output_index + p_stream->current_resample_counts.channels] = p_input_sample_buffer[curr_input_index + p_stream->current_resample_counts.channels] + linear_sum[p_stream->current_resample_counts.channels];

                        linear_sum[p_stream->current_resample_counts.channels] += linear_inc[p_stream->current_resample_counts.channels];
                        p_stream->current_resample_counts.channels++;
                        if (curr_output_index >= (Int32)*p_output_samples) {
                            break;
                        }

                    }

                    if (p_stream->match_resample.channels <= p_stream->current_resample_counts.channels) {
                        // All channels processed, move to the next sample
                        p_stream->current_resample_counts.channels = 0;
                        curr_output_index += p_stream->match_resample.channels;
                        p_stream->current_num_resamples--;
                    }
                }

                // Next input sample
                curr_input_index += p_stream->match_resample.channels;

                // Compute resample counts for next input sample
                calc_next_resample(p_stream);
            }

            if (curr_output_index >= (Int32)*p_output_samples) {
                ret_value = RESCALE_PCM_OVERRUN;
            }

            // Return actual input and output sample counts
            *p_input_samples = curr_input_index;
            *p_output_samples = curr_output_index;
        }
    }

    return ret_value;
}


/*!
Calculate the number of samples that the 48KHz conversion would generate, given an input buffer size.
The calculation uses the current multiplier and subsample count.
@param handle pointer to resample_stream_type
@param num_input_samples number of 16-bit samples which would be input.
@param  p_output_samples number of 16-bit samples which would be produced after conversion to 48KHz sample rate.
@return error conditon non-zero
*/
int upscale_engine_calc_output_buffer_size(STREAM_HANDLE stream_handle,
                                           uint32_t num_input_samples,
                                           uint32_t *p_output_samples)
{
    uint32_t result = 0;
    int ret_value = RESCALE_PCM_OK;
    resample_stream_type *p_stream = (resample_stream_type *)stream_handle;

    if (p_stream->signature != RESAMPLE_STREAM_SIGNATURE) {
        ret_value = RESCALE_PCM_HANDLE_INVALID;
    }
    else {
        result = num_input_samples * p_stream->match_resample.multiplier;

        if (p_stream->match_resample.sub_sample) {
            result += (num_input_samples * p_stream->match_resample.sub_sample_count) / INTEGER_SCALE_FACTOR;
        }

        if (!p_output_samples) {
            ret_value = RESCALE_PCM_ARG_INVALID;
        }
        else {
            *p_output_samples = result;
        }
    }

    return ret_value;
}


/*!
Set the current sample rate.
The calculation uses the current multiplier and subsample count.
@param handle pointer to resample_stream_type
@param sample_rate new sample rate
@param channels new number of channels
@return error conditon non-zero
*/
int upscale_engine_set_current_sample_rate(STREAM_HANDLE stream_handle,
                                           uint32_t sample_rate,
                                           uint32_t channels)
{
    int ret_value = RESCALE_PCM_OK;
    resample_stream_type *p_stream = (resample_stream_type *)stream_handle;

    if (p_stream->signature != RESAMPLE_STREAM_SIGNATURE) {
        ret_value = RESCALE_PCM_HANDLE_INVALID;
    }
    else {
        // If new sample frequency prime all counts
        if (sample_rate != p_stream->last_decode_freq) {
            p_stream->last_decode_freq = sample_rate;

            GT_1trace(curTrace, GT_4CLASS, "New MP3 Freq detected %d.\n", sample_rate);
            calc_resample_rates(p_stream, sample_rate, channels, &p_stream->match_resample);

            //initialize current sample counts
            memcpy(&p_stream->current_resample_counts, &p_stream->match_resample, sizeof(p_stream->match_resample));
            p_stream->current_resample_counts.channels = 0;  //reset to zero because count up, not down
            //Prime the resample counts for the first sample
            calc_next_resample(p_stream);
        }
    }

    return ret_value;
}

/*!
Return the current sample rate.
@param handle pointer to resample_stream_type
@param sample_rate current sample rate.
@return error conditon non-zero
*/
int upscale_engine_get_current_sample_rate(STREAM_HANDLE stream_handle,
                                           uint32_t *p_sample_rate)
{
    int ret_value = RESCALE_PCM_OK;
    resample_stream_type *p_stream = (resample_stream_type *)stream_handle;

    if (p_stream->signature != RESAMPLE_STREAM_SIGNATURE) {
        ret_value = RESCALE_PCM_HANDLE_INVALID;
    }
    else {
        if (!p_sample_rate) {
            ret_value = RESCALE_PCM_ARG_INVALID;
        }
        else {
            *p_sample_rate = p_stream->last_decode_freq;
        }
    }

return ret_value;
}


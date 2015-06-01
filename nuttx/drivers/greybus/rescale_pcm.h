

#ifndef _RESCALE_PCM_H_
#define _RESCALE_PCM_H_

//#include <xdc/std.h>

//#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stdafx.h"
//#include <nuttx/arch.h>

//error codes
#define RESCALE_PCM_OK              0
#define RESCALE_PCM_HANDLE_INVALID  1
#define RESCALE_PCM_OVERRUN         2
#define RESCALE_PCM_ARG_INVALID     3

typedef void* STREAM_HANDLE;

STREAM_HANDLE upscale_engine_init(void);
int upscale_engine_deinit(STREAM_HANDLE stream_handle);
int upscale_engine_resample_copy(STREAM_HANDLE stream_handle,
                                 int16_t *p_input_sample_buffer,
                                 uint32_t *p_input_samples,
                                 int16_t *p_output_sample_buffer,
                                 uint32_t *p_output_samples);
int upscale_engine_calc_output_buffer_size(STREAM_HANDLE stream_handle,
                                           uint32_t num_input_samples,
                                           uint32_t *p_output_samples);
int upscale_engine_set_current_sample_rate(STREAM_HANDLE stream_handle,
                                           uint32_t sample_rate,
                                           uint32_t channels);
int upscale_engine_get_current_sample_rate(STREAM_HANDLE stream_handle,
                                           uint32_t *p_sample_rate);

#endif //_RESCALE_PCM_H_

/**
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
 *
 * @author Mark Greer
 * @brief Greybus I2S Protocol Driver
 */
/*
 * The overall data flow is from a Greybus transmitter module to one ore more
 * Greybus receiver modules.  However, a Greybus transmitter will the receiver
 * on its local i2s connection.  Similarly, a Greybus receiver will be the
 * transmitter on its local i2s connection.  This can get confusing in the
 * code.  To help with this, all of the routines defined in this file refer
 * to the Greybus point of view.  So routine names, struct members, etc.
 * that have "transmit" or "tx" in their name are related to receiving data
 * from the local i2s connection and transmitting it over the UniPro network.
 */

/* TODO:
 *
 * The definition of "bundle" in this file is wrong so fix it.
 *
 * Currently supports only one I2S Bundle because manifest data doesn't
 * provide any info on which bundle a CPort belongs to.
 *
 * Supports only one Receiver CPort per I2S Bundle because all received
 * audio data goes to the one physical low-level controller.  So if there
 * is a mixer with two I2S connections, say, then use two separate I2S Bundles.
 *
 * For now, a FAST_HANDLER is not used for handling Greybus SEND_DATA
 * requests because the Greybus code on the AP assumes that every Greybus
 * message gets a response.
 */
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/list.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>
#include <nuttx/util.h>
#include <nuttx/wdog.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>

#include <arch/tsb/unipro.h>

#include <arch/armv7-m/byteorder.h>

#include "i2s-gb.h"

// These environments are defined in make file already, they should be removed!!!
// We added them here so ECLIPSE can show which part of code is ifdefed out.
#undef ENABLE_DUAL_I2S_PORTS

#define ENABLE_DUAL_I2S_PORTS
#define USE_THREAD_FOR_I2S_RB

#undef ENABLE_UPSCALE_RESAMPLING
#define ENABLE_UPSCALE_RESAMPLING
// #define DISABLE_TRANSMIT


#define GB_I2S_VERSION_MAJOR            0
#define GB_I2S_VERSION_MINOR            1

#define GB_I2S_CPORT_INVALID            (CPORT_MAX + 1)

#define GB_I2S_BUNDLE_0_ID              0
#define GB_I2S_BUNDLE_0_DEV_ID          0

#define GB_I2S_SAMPLES_PER_MSG_DEFAULT  1

#define GB_I2S_TX_SEND_DOWNSTREAM       8
#define GB_I2S_TX_TIMER_FUDGE_NS        (5 * 1000)

#define GB_I2S_TX_RING_BUF_PAD          8
#define GB_I2S_RX_RING_BUF_PAD          8

#define GB_I2S_CONFIG_MAX               32

#define GB_I2S_FLAGS_CONFIGURED         BIT(0)
#define GB_I2S_FLAGS_RX_PREPARED        BIT(1)
#define GB_I2S_FLAGS_RX_STARTED         BIT(2)
#define GB_I2S_FLAGS_TX_PREPARED        BIT(3)
#define GB_I2S_FLAGS_TX_STARTED         BIT(4)
#define GB_I2S_FLAGS_TX_DELAYING        BIT(5)
#define GB_I2S_FLAGS_TX_STOPPING        BIT(6)

#ifdef ENABLE_DUAL_I2S_PORTS
#define GB_I2S_FLAGS_MIXER_STARTED      BIT(7)
#endif

#ifndef SIGKILL
#define SIGKILL     9
#endif

enum gb_i2s_cport_type {
    GB_I2S_CPORT_TYPE_INVALID,
    GB_I2S_CPORT_TYPE_TRANSMITTER,
    GB_I2S_CPORT_TYPE_RECEIVER,
};

enum gb_i2s_cport_state {
    GB_I2S_CPORT_STATE_INVALID,
    GB_I2S_CPORT_STATE_INACTIVE,
    GB_I2S_CPORT_STATE_ACTIVE,
};

struct gb_i2s_cport_list_entry {
    struct list_head        list;
    struct gb_i2s_info      *info;
    uint16_t                cport;
    enum gb_i2s_cport_type  type;
    enum gb_i2s_cport_state state;
};


#ifdef ENABLE_DUAL_I2S_PORTS
#define GB_I2S_BUNDLE_1_ID              1
#define GB_I2S_BUNDLE_1_DEV_ID          1

#define MIXER_I2S_MSG_SIZE				4096
#define MIXER_SAMPLES_PER_FRAME         (MIXER_I2S_MSG_SIZE >> 2) // left and right and 2 bytes sample
// We fake it for now.
#define I2S_OUTPUT_SAMPLE_RATE          48000
#define SAMPLES_PER_MESSAGE             8
#define NUMBER_OF_AUDIO_CHANNELS	    2
#define DATA_SIZE_PER_SAMPLE            2

#ifdef ENABLE_UPSCALE_RESAMPLING
#include "rescale_pcm.h"
#endif

extern int get_cport_bundle(int cport);

#define MAX_AUDIO_CHANNELS			2
enum gb_mixer_op_type {
    GB_I2S_INSERT_AUDIO_FRAME,
    GB_I2S_MIX_AUDIO_CHANNELS,
};
#endif

struct gb_i2s_info { /* One per I2S Bundle */
    uint16_t                    mgmt_cport;
    uint32_t                    flags;
#ifndef ENABLE_DUAL_I2S_PORTS
    struct device               *dev;
#else
    int                         channel_id;
#endif
#ifdef ENABLE_UPSCALE_RESAMPLING
	STREAM_HANDLE               stream_handle;
	unsigned int                output_samples_size;
    uint32_t                    rx_rb_mixing_count;
#endif
    struct gb_i2s_configuration config;
    uint32_t                    delay;
    uint16_t                    samples_per_message;
    unsigned int                next_rx_sample;
    unsigned int                next_tx_sample;
    unsigned int                sample_size;
    unsigned int                msg_data_size;
    uint8_t                     *dummy_data;
    unsigned int                active_tx_cports;
    unsigned int                active_rx_cports;
    struct list_head            cport_list;
    struct ring_buf             *rx_rb;
#ifndef DISABLE_TRANSMIT
    struct ring_buf             *tx_rb;
    sem_t                       tx_stop_sem;
    sem_t                       tx_rb_sem;
    atomic_t                    tx_rb_count;
    pthread_t                   tx_rb_thread;
    WDOG_ID                     tx_wdog;
#endif

    /* XXX temporary */
    uint32_t                    rx_rb_count;
    uint32_t                    rx_rb_underrun_count;
};

struct gb_i2s_dev_info {
    uint16_t            bundle_id;      /* TODO: Fix when bundles implemented */
    char                *dev_type;
    unsigned int        dev_id;
    struct gb_i2s_info  *info;
};

#ifdef ENABLE_DUAL_I2S_PORTS
struct gb_audio_channel_info {
	struct gb_i2s_info  *channel_info;
	struct ring_buf     *rx_rb;
};

struct gb_mixer_info {
    uint32_t                    flags;
    struct device               *dev;
    struct ring_buf             *rx_rb;
    unsigned int                msg_data_size;

    struct gb_i2s_configuration config;

    atomic_t                    rx_channel_count;
    atomic_t                    active_rx_channel_count;
    atomic_t                    total_channel_count;


#ifdef USE_THREAD_FOR_I2S_RB
    sem_t                       rx_rb_sem;
    sem_t                       rx_stop_sem;
    pthread_t                   rx_rb_thread;
    int                         rx_thread_terminate;
#endif

    /* XXX temporary */
    uint32_t                    rx_rb_count;
    uint32_t                    rx_rb_total_count;
    uint32_t                    rx_rb_underrun_count;

    struct gb_audio_channel_info audio_channels[MAX_AUDIO_CHANNELS];
};

static struct gb_mixer_info gb_mixer = {
    .flags             = 0,
    .dev               = NULL,
	.rx_rb             = NULL,
#ifdef ENABLE_UPSCALE_RESAMPLING
	.msg_data_size     = MIXER_I2S_MSG_SIZE,
#endif
	.rx_rb_count       = 0,
	.rx_rb_total_count = 0,
	.rx_rb_underrun_count = 0,
	.rx_channel_count     = 0,
	.total_channel_count  = 0,
#ifdef USE_THREAD_FOR_I2S_RB
	.rx_thread_terminate   = 1,
#endif
};
#endif

static struct gb_i2s_dev_info gb_i2s_dev_info_map[] = {
    {
        .bundle_id  = GB_I2S_BUNDLE_0_ID, /*TODO: Fix when bundles implemented*/
        .dev_type   = DEVICE_TYPE_I2S_HW,
        .dev_id     = GB_I2S_BUNDLE_0_DEV_ID,
    },
#ifdef ENABLE_DUAL_I2S_PORTS
    {
        .bundle_id  = GB_I2S_BUNDLE_1_ID, /*TODO: Fix when bundles implemented*/
        .dev_type   = DEVICE_TYPE_I2S_HW,
        .dev_id     = GB_I2S_BUNDLE_1_DEV_ID,
    }
#endif
};

static struct gb_i2s_dev_info *gb_i2s_get_dev_info(uint16_t bundle_id)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++)
        if (gb_i2s_dev_info_map[i].bundle_id == bundle_id)
            return &gb_i2s_dev_info_map[i];

    return NULL;
}

static struct gb_i2s_info *gb_i2s_get_info(uint16_t mgmt_cport)
{
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++)
        if (gb_i2s_dev_info_map[i].info &&
            (gb_i2s_dev_info_map[i].info->mgmt_cport == mgmt_cport)) {

            return gb_i2s_dev_info_map[i].info;
        }

     return NULL;
}

static struct gb_i2s_cport_list_entry *gb_i2s_get_cple(
                                    struct gb_i2s_info *info, uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;
    struct list_head *iter;

    list_foreach(&info->cport_list, iter) {
        cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

        if (cple->cport == cport)
            return cple;
    }

    return NULL;
}

static struct gb_i2s_cport_list_entry *gb_i2s_find_cple(uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;
    unsigned int i;

    for (i = 0; i < ARRAY_SIZE(gb_i2s_dev_info_map); i++) {
        if (gb_i2s_dev_info_map[i].info) {
            cple = gb_i2s_get_cple(gb_i2s_dev_info_map[i].info, cport);
            if (cple)
                return cple;
        }
    }

    return NULL;
}

static struct gb_i2s_info *gb_i2s_get_info_by_cport(uint16_t cport)
{
    struct gb_i2s_cport_list_entry *cple;

    cple = gb_i2s_find_cple(cport);
    if (cple)
        return cple->info;

    return NULL;
}

static void gb_i2s_report_event(struct gb_i2s_info *info, uint32_t event)
{
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;
    struct gb_i2s_report_event_request *request;
    int ret;

    operation = gb_operation_create(info->mgmt_cport,
                                    GB_I2S_MGMT_TYPE_REPORT_EVENT,
                                    sizeof(*hdr) + sizeof(*request));
    if (!operation)
        return;

    request = gb_operation_get_request_payload(operation);
    request->event = event;

    /* XXX Shut things down if can't reach AP? */
    ret = gb_operation_send_request(operation, NULL, false);
    if (ret)
        lldbg("--- Can't report event %d: %d\n", event, ret); /* XXX */

    /*
    gb_operation_destroy(operation);
    */
}

static void gb_i2s_ll_tx(struct gb_i2s_info *info, uint8_t *data); /* XXX */

#ifndef ENABLE_DUAL_I2S_PORTS
/* Callback for low-level i2s transmit operations (GB receives) */
static void gb_i2s_ll_tx_cb(struct ring_buf *rb,
                            enum device_i2s_event event, void *arg)
{
    struct gb_i2s_info *info = arg;
    uint32_t gb_event = 0;

    switch (event) {
    case DEVICE_I2S_EVENT_TX_COMPLETE:
        /* ring_buf_reset(rb); will be called in gb_i2s_ll_tx() */
        /* should have been called by driver too */

        info->rx_rb_count--;

        if (info->rx_rb_count < 2) { /* XXX */
            gb_i2s_ll_tx(info, info->dummy_data);
            info->rx_rb_underrun_count++;
        }

        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        gb_event = GB_I2S_EVENT_UNDERRUN;
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_event = GB_I2S_EVENT_OVERRUN;
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_event = GB_I2S_EVENT_CLOCKING;
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_event = GB_I2S_EVENT_DATA_LEN;
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_event = GB_I2S_EVENT_UNSPECIFIED;
        break;
    default:
        gb_event = GB_I2S_EVENT_INTERNAL_ERROR;
    }

    if (gb_event) {
lldbg("--- overrun from driver: %d\n", event);
        gb_i2s_report_event(info, gb_event);
        /* All events are halt streaming right now */
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
    }
}
#else
static void gb_i2s_mixer_report_event(uint32_t event)
{
	int    index;

	for (index = 0; index < MAX_AUDIO_CHANNELS; index++) {
		struct gb_i2s_info *info = gb_mixer.audio_channels[index].channel_info;

		if (info != NULL) {
			gb_i2s_report_event(info, event);
		}
	}
}

static uint8_t gb_mixer_start_transmitter(enum gb_mixer_op_type op_type);

static void gb_i2s_mixer_ll_tx_cb(struct ring_buf *rb,
                                  enum device_i2s_event event, void *arg)
{
    struct gb_i2s_info *info = arg;
    uint32_t gb_event = 0;

    switch (event) {
    case DEVICE_I2S_EVENT_TX_COMPLETE:
        /* ring_buf_reset(rb); will be called in gb_i2s_ll_tx() */
        /* should have been called by driver too */

        gb_mixer.rx_rb_count--;

        if (gb_mixer.rx_rb_count < 2) { /* XXX */
            // gb_i2s_ll_tx(info, info->dummy_data);

            gb_mixer_start_transmitter(GB_I2S_INSERT_AUDIO_FRAME);

            // gb_mixer.rx_rb_underrun_count++;
        } else {
        	if (gb_mixer.rx_rb_count < (GB_I2S_RX_RING_BUF_PAD >> 1)) {
        		sem_post(&gb_mixer.rx_rb_sem);
        	}
        }

        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
    	gb_mixer.flags &= ~GB_I2S_FLAGS_RX_STARTED;
		sem_post(&gb_mixer.rx_rb_sem);
        gb_event = GB_I2S_EVENT_UNDERRUN;
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_event = GB_I2S_EVENT_OVERRUN;
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_event = GB_I2S_EVENT_CLOCKING;
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_event = GB_I2S_EVENT_DATA_LEN;
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_event = GB_I2S_EVENT_UNSPECIFIED;
        break;
    default:
        gb_event = GB_I2S_EVENT_INTERNAL_ERROR;
    }

    if (gb_event) {
// lldbg("--- event = %d from driver!!!\n", event);
lldbg("event=%d, (%d, %d) (%d, %d)!!!\n", event, gb_mixer.rx_rb_count, gb_mixer.rx_rb_total_count, info->rx_rb_count, info->next_rx_sample);
        gb_i2s_report_event(info, gb_event);
        /* All events are halt streaming right now */
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
    }
}

static void *gb_i2s_mixer_rb_thread(void *data)
{
	lldbg("started.\n");

	while (!gb_mixer.rx_thread_terminate)
	{
		sem_wait(&gb_mixer.rx_rb_sem);

		if (gb_mixer.rx_thread_terminate)
		{
			break;
		}
		gb_mixer_start_transmitter(GB_I2S_MIX_AUDIO_CHANNELS);
	}

	lldbg("ended!!!\n");

	sem_post(&gb_mixer.rx_stop_sem);

	return NULL;
}

static int gb_i2s_start_audio_mixer(void)
{
	int ret = 0;

	sem_init(&gb_mixer.rx_rb_sem, 0, 0);
	sem_init(&gb_mixer.rx_stop_sem, 0, 0);
	gb_mixer.rx_thread_terminate = 0;

	ret = pthread_create(&gb_mixer.rx_rb_thread, NULL, gb_i2s_mixer_rb_thread, NULL);

	if (ret) {
		gb_mixer.rx_thread_terminate = 1;
	} else {
		gb_mixer.flags |= GB_I2S_FLAGS_MIXER_STARTED;
	}

	return ret;
}

static int gb_i2s_mixer_prepare_receiver(struct gb_i2s_info *info, unsigned int entries)
{
	int ret        = 0;
    int channel_id = 0;
    int number_of_rx_channels = atomic_inc(&gb_mixer.rx_channel_count);

    if ((number_of_rx_channels > MAX_AUDIO_CHANNELS) || (number_of_rx_channels < 0)) {
        lldbg("Total number of channels = %d.\n", number_of_rx_channels);
    	atomic_dec(&gb_mixer.rx_channel_count);
        return GB_OP_MALFUNCTION;
    }

    for (channel_id = 0; channel_id < MAX_AUDIO_CHANNELS; channel_id++) {
        lldbg(" channel = %d, (%x).\n", channel_id, gb_mixer.audio_channels[channel_id].channel_info);
        if (!gb_mixer.audio_channels[channel_id].channel_info) {
            gb_mixer.audio_channels[channel_id].channel_info = info;
            gb_mixer.audio_channels[channel_id].rx_rb = info->rx_rb;

            info->channel_id = channel_id;
            break;
        }
    }

    if (channel_id == MAX_AUDIO_CHANNELS) {
        lldbg(" not available channel(channel = %d).\n", channel_id);
    	atomic_dec(&gb_mixer.rx_channel_count);
        return GB_OP_MALFUNCTION;
    }

    lldbg(" channel = %d.\n", channel_id);

    //if (!(gb_mixer.flags & GB_I2S_FLAGS_RX_PREPARED)) {
    if (number_of_rx_channels == 1) {
#ifndef ENABLE_UPSCALE_RESAMPLING
        gb_mixer.msg_data_size = info->msg_data_size;
#else
        /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
        entries = ((I2S_OUTPUT_SAMPLE_RATE * info->delay) /
                   (MIXER_SAMPLES_PER_FRAME * 1000000)) +
                   GB_I2S_RX_RING_BUF_PAD;

        gb_mixer.msg_data_size = MIXER_I2S_MSG_SIZE;
#endif

        lldbg(" entries=%d, size=%d, fr=%d, dl=%d, spm=%d\n",
    	    	entries, gb_mixer.msg_data_size, info->config.sample_frequency, info->delay, info->samples_per_message);

        gb_mixer.rx_rb = ring_buf_alloc_ring(entries,
                                        sizeof(struct gb_operation_hdr) +
                                        sizeof(struct gb_i2s_send_data_request),
                                        gb_mixer.msg_data_size, 0, NULL, NULL, NULL);

        if (!gb_mixer.rx_rb) {
        	ret = GB_OP_NO_MEMORY;
        }

    	gb_mixer.flags &= ~GB_I2S_FLAGS_MIXER_STARTED;
    	gb_mixer.flags &= ~GB_I2S_FLAGS_RX_PREPARED;
    	gb_mixer.flags &= ~GB_I2S_FLAGS_RX_STARTED;

  		gb_mixer.rx_rb_count = 0;
    	gb_mixer.rx_rb_underrun_count = 0;
    	gb_mixer.rx_rb_total_count = 0;

        if (!ret) {
    	    ret = gb_i2s_start_audio_mixer();
        }
    }

    if (ret)
    {
        gb_mixer.audio_channels[channel_id].channel_info = NULL;
        gb_mixer.audio_channels[channel_id].rx_rb = NULL;

		if (gb_mixer.rx_rb == NULL) {
			ring_buf_free_ring(gb_mixer.rx_rb, NULL, NULL);
    		gb_mixer.rx_rb = NULL;
		}

        lldbg(" failed to allocate RB or start thread!\n");

        atomic_dec(&gb_mixer.rx_channel_count);
    }

    return ret;
}

#endif



static uint8_t gb_i2s_prepare_receiver(struct gb_i2s_info *info)
{
    unsigned int entries;
    int ret = 0;

    if (info->flags & GB_I2S_FLAGS_RX_PREPARED)
        return GB_OP_PROTOCOL_BAD;

#ifndef ENABLE_UPSCALE_RESAMPLING
    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((info->config.sample_frequency * info->delay) /
                    (info->samples_per_message * 1000000)) +
               GB_I2S_RX_RING_BUF_PAD;

    lldbg(" entries=%d, size=%d, fr=%d, dl=%d, spm=%d\n",
    		entries, info->msg_data_size, info->config.sample_frequency, info->delay, info->samples_per_message);

    info->rx_rb = ring_buf_alloc_ring(entries,
                                    sizeof(struct gb_operation_hdr) +
                                        sizeof(struct gb_i2s_send_data_request),
                                    info->msg_data_size, 0, NULL, NULL, NULL);
#else
    uint32_t sample_frequency = info->config.sample_frequency;
	// uint32_t sample_frequency = FAKE_INPUT_SAMPLE_RATE;
    uint32_t output_samples_per_frame  = 0;

    STREAM_HANDLE stream_handle = upscale_engine_init();

  	if (stream_handle == NULL) {
   		ret = GB_OP_NO_MEMORY;
   	} else {
   		if(upscale_engine_set_current_sample_rate(stream_handle, sample_frequency, 2) != RESCALE_PCM_OK) {
   			lldbg(" failed to set sample rate.\n");
   			upscale_engine_deinit(stream_handle);
   			ret = GB_OP_MALFUNCTION;
   		} else {
   			info->stream_handle = stream_handle;
   		}
    }

  	if (!ret)
  	{
        ret = upscale_engine_calc_output_buffer_size(info->stream_handle,
        		                                     info->samples_per_message,
												     &output_samples_per_frame);
        if (ret != RESCALE_PCM_OK)
        {
        	lldbg(" failed to calculate up-sampling output size(ret=%d).\n", ret);
            ret = GB_OP_MALFUNCTION;
        }
  	}

    if (ret)
    {
    	ring_buf_free_ring(info->rx_rb, NULL, NULL);
    	info->rx_rb = NULL;

    	return ret;
    }

    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((sample_frequency * info->delay) /
               (output_samples_per_frame * 1000000)) +
               GB_I2S_RX_RING_BUF_PAD;

    lldbg(" input_size=%d, input_fr=%d, input_dl=%d, input_spm=%d\n",
    		info->msg_data_size, info->config.sample_frequency, info->delay, info->samples_per_message);
    lldbg(" entries=%d, output_size=%d, output_fr=%d, output_dl=%d, output_spm=%d\n",
    		entries, output_samples_per_frame << 2, I2S_OUTPUT_SAMPLE_RATE, info->delay, output_samples_per_frame);

    info->rx_rb = ring_buf_alloc_ring(entries,
                                      sizeof(struct gb_operation_hdr) +
                                      sizeof(struct gb_i2s_send_data_request),
									  output_samples_per_frame << 2, 0, NULL, NULL, NULL);

    info->output_samples_size = (info->rx_rb) ? output_samples_per_frame << 2 : 0;
#endif

    if (!info->rx_rb) {
    	lldbg("Out of memory.\n");
        return GB_OP_NO_MEMORY;
    }

    /* Greybus i2s message receiver is local i2s transmitter */
#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_prepare_transmitter(info->dev, info->rx_rb,
    	    gb_i2s_ll_tx_cb, info);
#else
  	ret = gb_i2s_mixer_prepare_receiver(info, entries);

  	lldbg("=1==> %d, %d\n", ret, gb_mixer.flags);
    if (!ret && !(gb_mixer.flags & GB_I2S_FLAGS_RX_PREPARED)) {
      	lldbg("=2==>\n");
        ret = device_i2s_prepare_transmitter(gb_mixer.dev, gb_mixer.rx_rb,
        	    gb_i2s_mixer_ll_tx_cb, info);

        if (ret) {
        	lldbg("failed to start i2s transmitter.\n");
            ring_buf_free_ring(gb_mixer.rx_rb, NULL, NULL);
        	gb_mixer.rx_rb = NULL;
        } else {
        	gb_mixer.flags |= GB_I2S_FLAGS_RX_PREPARED;
        }
    }
#endif
    if (ret) {
        ring_buf_free_ring(info->rx_rb, NULL, NULL);
        info->rx_rb = NULL;

        return GB_OP_MALFUNCTION;
    }

    info->flags |= GB_I2S_FLAGS_RX_PREPARED;

    return GB_OP_SUCCESS;
}



static void gb_i2s_ll_tx(struct gb_i2s_info *info, uint8_t *data)
{
    ring_buf_reset(info->rx_rb);

    /*
     * Unfortunately, we have to copy the data even though its a fast path
     * handler.  This is because the unipro susbystem reuses the buffer
     * immediately while the data could still be sitting in the ring buffer.
     */
#ifdef ENABLE_UPSCALE_RESAMPLING
    uint32_t  number_of_input_samples = info->samples_per_message;
    uint32_t  number_of_output_samples = (info->output_samples_size >> 2);

    // lldbg("=> %d, %d.\n", number_of_input_samples, number_of_output_samples);
    upscale_engine_resample_copy(info->stream_handle,
    		                     (int16_t *)data,
								 &number_of_input_samples,
								 (int16_t *)ring_buf_get_tail(info->rx_rb),
								 &number_of_output_samples);

    // lldbg("==> %d, %d.\n", number_of_input_samples, number_of_output_samples);
    ring_buf_put(info->rx_rb, (number_of_output_samples << 2));
#else
    memcpy(ring_buf_get_tail(info->rx_rb), data, info->msg_data_size);

    ring_buf_put(info->rx_rb, info->msg_data_size);
#endif

    ring_buf_pass(info->rx_rb);

    // info->next_rx_sample += info->samples_per_message;
    info->next_rx_sample++;
    info->rx_rb = ring_buf_get_next(info->rx_rb);

    irqstate_t irq_flags = irq_flags = irqsave();
    info->rx_rb_count++;
    irqrestore(irq_flags);
}

#ifndef ENABLE_DUAL_I2S_PORTS
static uint8_t gb_i2s_receiver_send_data_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_send_data_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;
    irqstate_t flags;
    int ret;

    info = gb_i2s_get_info_by_cport(operation->cport);
    if (!info || !(info->flags & GB_I2S_FLAGS_RX_PREPARED)) {
        gb_i2s_report_event(info, GB_I2S_EVENT_PROTOCOL_ERROR);
        return 0;
    }

    if (le32_to_cpu(request->size) != info->msg_data_size) {
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        return 0;
    }

    flags = irqsave();

    if (!ring_buf_is_producers(info->rx_rb)) {
        irqrestore(flags);
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
        return 0; /* Discard the message */
    }

    /* Fill in any missing data */
    while (ring_buf_is_producers(info->rx_rb) &&
           (le32_to_cpu(request->sample_number) > info->next_rx_sample))
        gb_i2s_ll_tx(info, info->dummy_data);

    gb_i2s_ll_tx(info, request->data);

    irqrestore(flags);

#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_start_transmitter(info->dev);
#else
    ret = device_i2s_start_transmitter(gb_mixer.dev);
#endif

    if (ret) {
        gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
        return ret;
    }

    info->flags |= GB_I2S_FLAGS_RX_STARTED;

    return 0;
}
#else

#if 1
static unsigned int gb_mixer_get_available_audio_samples(
	struct gb_audio_channel_info *audio_channels[],
	unsigned int *avail_channels,
	enum gb_mixer_op_type op_type)
{
	int index;
	unsigned int number_of_samples = gb_mixer.msg_data_size;
	struct gb_audio_channel_info *audio_channel;

	*avail_channels = 0;
	for (index = 0; index < MAX_AUDIO_CHANNELS; index++) {
		audio_channel = &gb_mixer.audio_channels[index];

		if (audio_channel->channel_info != NULL) {
		   if ((audio_channel->channel_info->flags & GB_I2S_FLAGS_RX_STARTED) &&
			   (ring_buf_is_consumers(audio_channel->rx_rb)))
		   {
			   unsigned int rb_avail_samples;

			   audio_channels[*avail_channels] = audio_channel;
			   rb_avail_samples = ring_buf_len(audio_channels[*avail_channels]->rx_rb);

			   if (number_of_samples > rb_avail_samples) {
				   number_of_samples = rb_avail_samples;
			   }
			   (*avail_channels)++;
		   } else {
			   if (op_type == GB_I2S_INSERT_AUDIO_FRAME) {
			       // gb_i2s_report_event(audio_channel->channel_info, GB_I2S_EVENT_UNDERRUN);
			       audio_channel->channel_info->rx_rb_underrun_count++;
			   }
		   }
	   }
	}

	if (*avail_channels == 0) {
		number_of_samples = 0;
	}

    return number_of_samples;
}


static int gb_mixer_ring_buf_get_next(
    struct gb_audio_channel_info *audio_channels[],
  	unsigned int avail_channels,
	unsigned int samples_count,
	struct ring_buf *src_rb[])
{
    int index;

    for (index = 0; index < avail_channels; index++)
    {
    	src_rb[index] = audio_channels[index]->rx_rb;
        if (ring_buf_len(audio_channels[index]->rx_rb) <= samples_count) {
        	audio_channels[index]->rx_rb = ring_buf_get_next(audio_channels[index]->rx_rb);
        	audio_channels[index]->channel_info->rx_rb_count--;

        	if (avail_channels != 1) {
        		audio_channels[index]->channel_info->rx_rb_mixing_count++;
        	}
        }
    }
    return 0;
}

static int gb_mixer_mix_audio_channels(
	struct ring_buf *dest_rb,
	struct ring_buf *src_rb[],
  	unsigned int src_channel_count,
	unsigned int mixing_data_length)
{
    int sample_index  = 0;
    int channel_index = 0;
    unsigned int *dest = (unsigned int*)ring_buf_get_head(dest_rb);
    unsigned int *src[MAX_AUDIO_CHANNELS] = {NULL};

    if (mixing_data_length & 0x3) {
    	lldbg("Data length must be multiple of 4.\n");
    	return 1;
    }

    for (channel_index = 0; channel_index < src_channel_count; channel_index++) {
    	src[channel_index] = (unsigned int*)ring_buf_get_head(src_rb[channel_index]);
    }

    int sample_count  = mixing_data_length >> 2;
    for (sample_index = 0; sample_index < sample_count; sample_index++)
    {
    	uint32_t output_sample = 0;

        for (channel_index = 0; channel_index < src_channel_count; channel_index++) {
        	output_sample += (uint32_t)src[channel_index][sample_index];
        }

        // TODO: Need to do better than this.
        dest[sample_index] = (unsigned int)(output_sample + (src_channel_count >> 1)) / src_channel_count;
    }

    return 0;
}

static int gb_mixer_ring_buf_pull_and_pass(
    struct ring_buf *src_rb[],
  	unsigned int channel_count,
  	unsigned int mixing_data_length
	)
{
    int index;

    for (index = 0; index < channel_count; index++)
    {
    	ring_buf_pull(src_rb[index], mixing_data_length);

    	if (ring_buf_len(src_rb[index]) == 0) {
        	ring_buf_pass(src_rb[index]);
    	}
    }
    return 0;
}

static uint8_t gb_mixer_start_transmitter(enum gb_mixer_op_type op_type)
{
    int ret;
    int frames_sent = 0;
    irqstate_t irq_flags      = 0;
    struct ring_buf *dest_rb  = NULL;

	irq_flags = irqsave();

    while (ring_buf_is_producers(dest_rb = gb_mixer.rx_rb))
    {
    	unsigned int mixing_data_length = 0;
    	unsigned int avail_channels = 0;

    	struct gb_audio_channel_info *audio_channels[MAX_AUDIO_CHANNELS] = {NULL};

        // src_rb = gb_mixer.audio_channels[0].rx_rb;
    	mixing_data_length = gb_mixer_get_available_audio_samples(&audio_channels[0],
        		                                              &avail_channels,
															  op_type);

    	// if (ring_buf_is_consumers(src_rb))
        if ((mixing_data_length > 0) &&
        	((avail_channels == gb_mixer.active_rx_channel_count) || (op_type == GB_I2S_INSERT_AUDIO_FRAME))) //(avail_channels > 0))
    	{
        	struct ring_buf *src_rb[MAX_AUDIO_CHANNELS]  = {NULL};

            gb_mixer.rx_rb = ring_buf_get_next(dest_rb);
            gb_mixer_ring_buf_get_next(&audio_channels[0],
            		avail_channels,
					mixing_data_length,
					src_rb);

            gb_mixer.rx_rb_count++;
            gb_mixer.rx_rb_total_count++;

            irqrestore(irq_flags);

    		ring_buf_reset(dest_rb);
    		if (avail_channels == 1) {
    			//lldbg("copy %d bytes\n", mixing_data_length);
    		    memcpy(ring_buf_get_head(dest_rb), ring_buf_get_head(src_rb[0]), mixing_data_length);
    		} else {
    			gb_mixer_mix_audio_channels(dest_rb, &src_rb[0], avail_channels, mixing_data_length);
    		}

            ring_buf_put(dest_rb, mixing_data_length);
            ring_buf_pass(dest_rb);

            gb_mixer_ring_buf_pull_and_pass(&src_rb[0], avail_channels, mixing_data_length);

        	irq_flags = irqsave();

            frames_sent++;
            op_type = GB_I2S_MIX_AUDIO_CHANNELS;
    	}
    	else
    	{
            if ((frames_sent == 0) && (op_type == GB_I2S_INSERT_AUDIO_FRAME)) {
            	gb_mixer.rx_rb = ring_buf_get_next(dest_rb);
            	gb_mixer.rx_rb_count++;

            	ring_buf_reset(dest_rb);
            	memset(ring_buf_get_head(dest_rb), 0, gb_mixer.msg_data_size);
                ring_buf_put(dest_rb, gb_mixer.msg_data_size);
                ring_buf_pass(dest_rb);
                frames_sent++;

                gb_mixer.rx_rb_underrun_count++;
            }
    		break;
    	}
    }
    irqrestore(irq_flags);

    if (frames_sent && !(gb_mixer.flags & GB_I2S_FLAGS_RX_STARTED)) {
    	ret = device_i2s_start_transmitter(gb_mixer.dev);

    	if (ret) {
            gb_i2s_mixer_report_event(GB_I2S_EVENT_FAILURE);
            gb_i2s_mixer_report_event(GB_I2S_EVENT_HALT);
    		lldbg(" failed to start I2S HW(ret=%d)!!!\n", ret);
            return ret;
        }

        gb_mixer.flags |= GB_I2S_FLAGS_RX_STARTED;
    }

    return 0;
}
#else
static uint8_t gb_mixer_start_transmitter(enum gb_mixer_op_type op_type)
{
    int ret;
    int frame_sent = 0;
    irqstate_t irq_flags      = 0;
    int buffer_length         = 0;
    struct ring_buf *dest_rb  = NULL;
    struct ring_buf *src_rb   = NULL;

	irq_flags = irqsave();

    while (ring_buf_is_producers(dest_rb = gb_mixer.rx_rb))
    {
    	src_rb = gb_mixer.audio_channels[0].rx_rb;

    	if (ring_buf_is_consumers(src_rb))
    	{
            gb_mixer.audio_channels[0].rx_rb = ring_buf_get_next(src_rb);
            gb_mixer.rx_rb = ring_buf_get_next(dest_rb);

            gb_mixer.rx_rb_count++;
            gb_mixer.rx_rb_total_count++;

            irqrestore(irq_flags);

            buffer_length = ring_buf_len(src_rb);

    		ring_buf_reset(dest_rb);
    		memcpy(ring_buf_get_head(dest_rb), ring_buf_get_head(src_rb), buffer_length);

            ring_buf_put(dest_rb, buffer_length);

            ring_buf_pass(src_rb);
            ring_buf_pass(dest_rb);

        	irq_flags = irqsave();
            frame_sent++;
    	}
    	else
    	{
            if ((frame_sent == 0) && (op_type == GB_I2S_INSERT_AUDIO_FRAME)) {
            	gb_mixer.rx_rb = ring_buf_get_next(dest_rb);
            	gb_mixer.rx_rb_count++;

            	ring_buf_reset(dest_rb);
                ring_buf_put(dest_rb, gb_mixer.msg_data_size);
                ring_buf_pass(dest_rb);
                frame_sent++;
            }

    		break;
    	}
    }
    irqrestore(irq_flags);

    if (frame_sent != 0) {
    	ret = device_i2s_start_transmitter(gb_mixer.dev);

    	if (ret) {
            //gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
            //gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
            gb_i2s_mixer_report_event(GB_I2S_EVENT_FAILURE);
            gb_i2s_mixer_report_event(GB_I2S_EVENT_HALT);

    		lldbg(" failed to start I2S HW(ret=%d)!!!\n", ret);
            return ret;
        }

        gb_mixer.flags |= GB_I2S_FLAGS_RX_STARTED;
    }

    return 0;
}
#endif

static uint8_t gb_i2s_receiver_send_data_req_handler(
                                                struct gb_operation *operation)
{
#ifndef USE_THREAD_FOR_I2S_RB
    int ret = 0;
#endif
    struct gb_i2s_send_data_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;

    info = gb_i2s_get_info_by_cport(operation->cport);
    if (!info || !(info->flags & GB_I2S_FLAGS_RX_PREPARED)) {
        gb_i2s_report_event(info, GB_I2S_EVENT_PROTOCOL_ERROR);
        return 0;
    }

    // lldbg("===> cport=%d, sn=%d sz=%d(msgdatasz=%d).\n", operation->cport, request->sample_number, request->size, info->msg_data_size);

    if (le32_to_cpu(request->size) != info->msg_data_size) {
    	lldbg("data length error: reqSize=%d, dataSize=%d\n", request->size, info->msg_data_size);
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        return 0;
    }


    if (!ring_buf_is_producers(info->rx_rb)) {
		lldbg("Audio Channel overrun(%d, %d, %d)!!\n", gb_mixer.rx_rb_count, info->rx_rb_count, info->next_rx_sample);
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
        return 0; /* Discard the message */
    }

    // lldbg("Send frame to I2S ring buffer.\n");

    /* Fill in any missing data */
    while (ring_buf_is_producers(info->rx_rb) &&
           (le32_to_cpu(request->sample_number) > info->next_rx_sample))
        gb_i2s_ll_tx(info, info->dummy_data);

    if (!ring_buf_is_producers(info->rx_rb)) {
		lldbg("Audio Channel overrun(%d, %d, %d)!!!\n", gb_mixer.rx_rb_count, info->rx_rb_count, info->next_rx_sample);
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
        return 0; /* Discard the message */
    }

	gb_i2s_ll_tx(info, request->data);

#ifdef USE_THREAD_FOR_I2S_RB
    if (!(info->flags & GB_I2S_FLAGS_RX_STARTED)) {
    	uint32_t packets_before_start;

    	packets_before_start = (info->config.sample_frequency * info->delay) /
                               (info->samples_per_message * 1000000);

//    	lldbg("%d, %d, %d, %d, %d(%x)\n", packets_before_start, info->rx_rb_count,
//    		  info->config.sample_frequency, info->delay, info->samples_per_message,
//			  gb_mixer.flags);

    	if (packets_before_start >= info->rx_rb_count) {
            return 0;
    	}

        info->flags |= GB_I2S_FLAGS_RX_STARTED;
        atomic_inc(&gb_mixer.active_rx_channel_count);
    }

    if (!(gb_mixer.flags & GB_I2S_FLAGS_RX_STARTED)) {
    	sem_post(&gb_mixer.rx_rb_sem);
    }
#else
#ifndef ENABLE_DUAL_I2S_PORTS
    ret = gb_mixer_start_transmitter(GB_I2S_MIX_AUDIO_CHANNELS);
#else
    ret = device_i2s_start_transmitter(info->dev);
#endif

    if (ret) {
        gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
        return ret;
    }

    info->flags |= GB_I2S_FLAGS_RX_STARTED;
#endif

    return 0;
}
#endif

static uint8_t gb_i2s_shutdown_receiver(struct gb_i2s_info *info)
{
    if (!(info->flags & GB_I2S_FLAGS_RX_PREPARED))
        return GB_OP_PROTOCOL_BAD;

    if (info->flags & GB_I2S_FLAGS_RX_STARTED) {
        info->flags &= ~GB_I2S_FLAGS_RX_STARTED;

#ifndef ENABLE_DUAL_I2S_PORTS
        device_i2s_stop_transmitter(info->dev);
#else
        int channel_id = info->channel_id;

#ifdef ENABLE_UPSCALE_RESAMPLING
  		info->stream_handle = NULL;
#endif
  		atomic_dec(&gb_mixer.active_rx_channel_count);

  		if (atomic_dec(&gb_mixer.rx_channel_count) == 0)
        {
#ifdef USE_THREAD_FOR_I2S_RB
        	gb_mixer.rx_thread_terminate = 1;
        	sem_post(&gb_mixer.rx_rb_sem);
        	sem_wait(&gb_mixer.rx_stop_sem);
#endif
        	device_i2s_stop_transmitter(gb_mixer.dev);

        	ring_buf_free_ring(gb_mixer.rx_rb, NULL, NULL);

        	gb_mixer.flags &= ~GB_I2S_FLAGS_MIXER_STARTED;
        	gb_mixer.flags &= ~GB_I2S_FLAGS_RX_PREPARED;
        	gb_mixer.flags &= ~GB_I2S_FLAGS_RX_STARTED;

      		lldbg("rbCnt=%d, id=%d, undrun=%d(%d), mixingCnt=%d.\n",
      			  info->rx_rb_count, info->next_rx_sample, info->rx_rb_underrun_count,
      			  gb_mixer.rx_rb_underrun_count, info->rx_rb_mixing_count);

      		gb_mixer.rx_rb_count = 0;
        	gb_mixer.rx_rb_underrun_count = 0;
        	gb_mixer.rx_rb_total_count = 0;
        } else {
      		lldbg("rbCnt=%d, id=%d, undrun=%d(%d), mixingCnt=%d.\n",
      			  info->rx_rb_count, info->next_rx_sample, info->rx_rb_underrun_count,
      			  gb_mixer.rx_rb_underrun_count, info->rx_rb_mixing_count);
        }

        lldbg("closing audio chanbel %d.\n", channel_id);

    	gb_mixer.audio_channels[channel_id].rx_rb = NULL;
    	gb_mixer.audio_channels[channel_id].channel_info = NULL;

    	info->rx_rb_mixing_count = 0;
#endif
        // KIMMUI
        info->rx_rb_count = 0;
        info->next_rx_sample = 0;
        info->rx_rb_underrun_count = 0;
    }

#ifndef ENABLE_DUAL_I2S_PORTS
    device_i2s_shutdown_transmitter(info->dev);
#else
    device_i2s_shutdown_transmitter(gb_mixer.dev);
#endif

    ring_buf_free_ring(info->rx_rb, NULL, NULL);
    info->rx_rb = NULL;

    info->flags &= ~GB_I2S_FLAGS_RX_PREPARED;

    return GB_OP_SUCCESS;
}

/*
 * Use 'tx_rb_sem' semaphore to regulate when a sample is sent to the
 * receiver.  When the start delay time passes, 'GB_I2S_TX_SEND_DOWNSTREAM'
 * samples will be sent to the receiver to help with jitter (see
 * gb_i2s_tx_delay_timeout()).  The rest will remain on transmitter so
 * receiver isn't swamped.
 */
#ifndef DISABLE_TRANSMIT
static void *gb_i2s_tx_rb_thread(void *data)
{
    struct gb_i2s_info *info = data;
    struct gb_operation *operation;
    struct gb_i2s_send_data_request *request;
    struct ring_buf *rb;
    struct list_head *iter;
    struct gb_i2s_cport_list_entry *cple;
    int ret;

    /* XXX Waits indefinitely when gb_i2s_ll_rx_cb() not called */
    while (1) {
        sem_wait(&info->tx_rb_sem);

        if (!atomic_get(&info->tx_rb_count))
            continue;

        rb = info->tx_rb;
        operation = rb->priv;

        request = gb_operation_get_response_payload(operation);
        request->sample_number = cpu_to_le32(info->next_tx_sample);
        info->next_tx_sample += info->samples_per_message;

        list_foreach(&info->cport_list, iter) {
            cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

            if ((cple->type != GB_I2S_CPORT_TYPE_TRANSMITTER) ||
                (cple->state != GB_I2S_CPORT_STATE_ACTIVE))
                continue;

            operation->cport = cple->cport;

            ret = gb_operation_send_request(operation, NULL, 0);
            if (ret) /* XXX Add report_event throttling */
                gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        }

        ring_buf_reset(rb);
        ring_buf_pass(rb);

#ifndef ENABLE_DUAL_I2S_PORTS
        ret = device_i2s_start_receiver(info->dev);
#else
        ret = device_i2s_start_receiver(gb_mixer.dev);
#endif
        if (ret)
            gb_i2s_report_event(info, GB_OP_MALFUNCTION);

        atomic_dec(&info->tx_rb_count);
        info->tx_rb = ring_buf_get_next(rb);

        if ((info->flags & GB_I2S_FLAGS_TX_STOPPING) &&
            !atomic_get(&info->tx_rb_count)) {

            sem_post(&info->tx_stop_sem);
        }
    }

    /* NOTREACHED */
    return NULL;
}
#endif

#ifndef DISABLE_TRANSMIT
/* Callback for low-level i2s receive operations (GB transmits) */
static void gb_i2s_ll_rx_cb(struct ring_buf *rb,
                            enum device_i2s_event event, void *arg)
{
    struct gb_i2s_info *info = arg;
    uint32_t gb_event = 0;

    if (!(info->flags & GB_I2S_FLAGS_TX_STARTED))
        return;

    switch (event) {
    case DEVICE_I2S_EVENT_RX_COMPLETE:
        atomic_inc(&info->tx_rb_count);

        if (!(info->flags & GB_I2S_FLAGS_TX_DELAYING))
            sem_post(&info->tx_rb_sem);
        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        gb_event = GB_I2S_EVENT_UNDERRUN;
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_event = GB_I2S_EVENT_OVERRUN;
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_event = GB_I2S_EVENT_CLOCKING;
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_event = GB_I2S_EVENT_DATA_LEN;
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_event = GB_I2S_EVENT_UNSPECIFIED;
        break;
    default:
        gb_event = GB_I2S_EVENT_INTERNAL_ERROR;
        break;
    }

    if (gb_event) {
        gb_i2s_report_event(info, gb_event);
        /* All events currently halt streaming */
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
    }
}

static void gb_i2s_tx_delay_timeout(int argc, uint32_t arg, ...)
{
    struct gb_i2s_info *info = (struct gb_i2s_info *)arg;
    irqstate_t flags;
    uint32_t i, limit;

    flags = irqsave();

    if (info->flags & GB_I2S_FLAGS_TX_DELAYING) {
        limit = atomic_get(&info->tx_rb_count);
        limit = MIN(limit, GB_I2S_TX_SEND_DOWNSTREAM);

        for (i = 0; i < limit; i++)
            sem_post(&info->tx_rb_sem);

        info->flags &= ~GB_I2S_FLAGS_TX_DELAYING;
    }

    irqrestore(flags);
}

static int gb_i2s_rb_alloc_gb_op(struct ring_buf *rb, void *arg)
{
    struct gb_i2s_info *info = arg;
    struct gb_operation *operation;
    struct gb_operation_hdr *hdr;
    struct gb_i2s_send_data_request *request;

    operation = gb_operation_create(info->mgmt_cport,
                                    GB_I2S_DATA_TYPE_SEND_DATA,
                                    sizeof(*request) + info->msg_data_size);
    if (!operation)
        return -ENOMEM;

    ring_buf_init(rb, operation->request_buffer,
                      sizeof(*hdr) + sizeof(*request), info->msg_data_size);

    request = gb_operation_get_response_payload(operation);
    request->size = cpu_to_le32(info->msg_data_size);

    ring_buf_set_priv(rb, operation);

    return 0;
}

static void gb_i2s_rb_free_gb_op(struct ring_buf *rb, void *arg)
{
    gb_operation_destroy(ring_buf_get_priv(rb));
}
#endif

static uint8_t gb_i2s_prepare_transmitter(struct gb_i2s_info *info)
{
#ifndef DISABLE_TRANSMIT
    unsigned int entries;
    int ret;

    if (info->flags & GB_I2S_FLAGS_TX_PREPARED)
        return GB_OP_PROTOCOL_BAD;

    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((info->config.sample_frequency * info->delay) /
                    (info->samples_per_message * 1000000)) +
               GB_I2S_TX_RING_BUF_PAD;

    info->tx_rb = ring_buf_alloc_ring(entries, 0, 0, 0,
                                          gb_i2s_rb_alloc_gb_op,
                                          gb_i2s_rb_free_gb_op, info);
    if (!info->tx_rb)
        return GB_OP_NO_MEMORY;

    /* Greybus i2s message transmitter is local i2s receiver */
#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_prepare_receiver(info->dev, info->tx_rb,
                                          gb_i2s_ll_rx_cb, info);
#else
    ret = device_i2s_prepare_receiver(gb_mixer.dev, info->tx_rb,
                                          gb_i2s_ll_rx_cb, info);
#endif
    if (ret) {
        ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
        info->tx_rb = NULL;

        return GB_OP_MALFUNCTION;
    }

    info->flags |= GB_I2S_FLAGS_TX_PREPARED;
#endif

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_start_transmitter(struct gb_i2s_info *info)
{
#ifndef DISABLE_TRANSMIT
    irqstate_t flags;
    int ret;

    if (!(info->flags & GB_I2S_FLAGS_TX_PREPARED) ||
        (info->flags & GB_I2S_FLAGS_TX_STARTED)) {

        return GB_OP_PROTOCOL_BAD;
    }

    flags = irqsave();

    if (info->delay) {
        info->flags |= GB_I2S_FLAGS_TX_DELAYING;

        /* Should subtract out estimate of time it took to get here */
        wd_start(info->tx_wdog, info->delay / CLOCKS_PER_SEC,
                 gb_i2s_tx_delay_timeout, 1, info);
    }

#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_start_receiver(info->dev);
#else
    ret = device_i2s_start_receiver(gb_mixer.dev);
#endif
    if (ret) {
        if (info->flags & GB_I2S_FLAGS_TX_DELAYING) {
            wd_cancel(info->tx_wdog);
            info->flags &= ~GB_I2S_FLAGS_TX_DELAYING;
        }

        irqrestore(flags);
        return GB_OP_MALFUNCTION;
    }

    info->flags |= GB_I2S_FLAGS_TX_STARTED;

    irqrestore(flags);
#endif

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_stop_transmitter(struct gb_i2s_info *info)
{
#ifndef DISABLE_TRANSMIT
    irqstate_t flags;
    uint32_t i, limit;

    if (!(info->flags & GB_I2S_FLAGS_TX_STARTED))
        return GB_OP_PROTOCOL_BAD;

#ifndef ENABLE_DUAL_I2S_PORTS
    device_i2s_stop_receiver(info->dev);
#else
    device_i2s_stop_receiver(gb_mixer.dev);
#endif

    flags = irqsave();

    if (info->flags & GB_I2S_FLAGS_TX_DELAYING) {
        wd_cancel(info->tx_wdog);
        info->flags &= ~GB_I2S_FLAGS_TX_DELAYING;
    }

    irqrestore(flags);

    info->flags |= GB_I2S_FLAGS_TX_STOPPING;

    limit = atomic_get(&info->tx_rb_count);

    /* Flush buffered samples to receiver */
    for (i = 0; i < limit; i++)
        sem_post(&info->tx_rb_sem);

    sem_wait(&info->tx_stop_sem);

    info->flags &= ~GB_I2S_FLAGS_TX_STOPPING;
    info->flags &= ~GB_I2S_FLAGS_TX_STARTED;
#endif

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_shutdown_transmitter(struct gb_i2s_info *info)
{
#ifndef DISABLE_TRANSMIT
    if (!(info->flags & GB_I2S_FLAGS_TX_PREPARED) ||
        (info->flags & GB_I2S_FLAGS_TX_STARTED)) {

        return GB_OP_PROTOCOL_BAD;
    }

#ifndef ENABLE_DUAL_I2S_PORTS
    device_i2s_shutdown_receiver(info->dev);
#else
    device_i2s_shutdown_receiver(gb_mixer.dev);
#endif

    ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
    info->tx_rb = NULL;

    info->flags &= ~GB_I2S_FLAGS_TX_PREPARED;
#endif

    return GB_OP_SUCCESS;
}

static void gb_i2s_copy_cfg_gb2dev(struct gb_i2s_configuration *gb_cfg,
                                   struct device_i2s_configuration *dev_cfg)
{
    dev_cfg->sample_frequency = le32_to_cpu(gb_cfg->sample_frequency);
    dev_cfg->num_channels = gb_cfg->num_channels;
    dev_cfg->bytes_per_channel = gb_cfg->bytes_per_channel;
    dev_cfg->byte_order = gb_cfg->byte_order;
    dev_cfg->spatial_locations = le32_to_cpu(gb_cfg->spatial_locations);
    dev_cfg->ll_protocol = le32_to_cpu(gb_cfg->ll_protocol);
    dev_cfg->ll_mclk_role = gb_cfg->ll_mclk_role;
    dev_cfg->ll_bclk_role = gb_cfg->ll_bclk_role;
    dev_cfg->ll_wclk_role = gb_cfg->ll_wclk_role;
    dev_cfg->ll_wclk_polarity = gb_cfg->ll_wclk_polarity;
    dev_cfg->ll_wclk_change_edge = gb_cfg->ll_wclk_change_edge;
    dev_cfg->ll_data_tx_edge = gb_cfg->ll_data_tx_edge;
    dev_cfg->ll_data_rx_edge = gb_cfg->ll_data_rx_edge;
    dev_cfg->ll_data_offset = gb_cfg->ll_data_offset;
}

static void gb_i2s_copy_cfg_dev2gb(const struct device_i2s_configuration
                                                                    *dev_cfg,
                                   struct gb_i2s_configuration *gb_cfg)
{
    gb_cfg->sample_frequency = cpu_to_le32(dev_cfg->sample_frequency);
    gb_cfg->num_channels = dev_cfg->num_channels;
    gb_cfg->bytes_per_channel = dev_cfg->bytes_per_channel;
    gb_cfg->byte_order = dev_cfg->byte_order;
    gb_cfg->spatial_locations = cpu_to_le32(dev_cfg->spatial_locations);
    gb_cfg->ll_protocol = cpu_to_le32(dev_cfg->ll_protocol);
    gb_cfg->ll_mclk_role = dev_cfg->ll_mclk_role;
    gb_cfg->ll_bclk_role = dev_cfg->ll_bclk_role;
    gb_cfg->ll_wclk_role = dev_cfg->ll_wclk_role;
    gb_cfg->ll_wclk_polarity = dev_cfg->ll_wclk_polarity;
    gb_cfg->ll_wclk_change_edge = dev_cfg->ll_wclk_change_edge;
    gb_cfg->ll_data_tx_edge = dev_cfg->ll_data_tx_edge;
    gb_cfg->ll_data_rx_edge = dev_cfg->ll_data_rx_edge;
    gb_cfg->ll_data_offset = dev_cfg->ll_data_offset;
}

/* Greybus Operation Handlers */

static uint8_t gb_i2s_protocol_version_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_proto_version_response *response;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_I2S_VERSION_MAJOR;
    response->minor = GB_I2S_VERSION_MINOR;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_get_supported_configurations_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_get_supported_configurations_response *response;
    struct gb_i2s_info *info;
    const struct device_i2s_configuration *dev_cfg;
    struct gb_i2s_configuration *gb_cfg;
    uint16_t dev_cfg_cnt;
    size_t size;
    uint16_t i;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_get_supported_configurations(info->dev, &dev_cfg_cnt,
                                                  &dev_cfg);
#else
    ret = device_i2s_get_supported_configurations(gb_mixer.dev, &dev_cfg_cnt,
                                                  &dev_cfg);
#endif
    if (ret)
        return GB_OP_MALFUNCTION;

    dev_cfg_cnt = MIN(dev_cfg_cnt, GB_I2S_CONFIG_MAX); /* Limit for sanity */
    size = dev_cfg_cnt * sizeof(*gb_cfg);

    response = gb_operation_alloc_response(operation, sizeof(*response) + size);
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config_count = dev_cfg_cnt;
    gb_cfg = response->config;

    for (i = 0; i < dev_cfg_cnt; i++)
        gb_i2s_copy_cfg_dev2gb(dev_cfg++, gb_cfg++);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_configuration_req_handler(struct gb_operation
                                                                    *operation)
{
    struct gb_i2s_set_configuration_request *request =
                gb_operation_get_request_payload(operation);
    struct device_i2s_configuration dev_cfg;
    struct gb_i2s_info *info;
    int ret = 0;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    gb_i2s_copy_cfg_gb2dev(&request->config, &dev_cfg);

#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_set_configuration(info->dev, &dev_cfg);
    if (ret)
        return GB_OP_MALFUNCTION;

    memcpy(&info->config, &request->config, sizeof(info->config));
#else
    if (!(gb_mixer.flags & GB_I2S_FLAGS_CONFIGURED)) {
    	lldbg("sampleFrequency=%d\n", request->config.sample_frequency);
    	lldbg("sf=%d, nc=%d, bpc=%d, bo=%d, sl=%d, llp=%d, llm=%d, llb=%d, llw=(%d, %d, %d), lld=(%d, %d, %d)\n",
    			request->config.sample_frequency, request->config.num_channels, request->config.bytes_per_channel, request->config.byte_order,
    			request->config.spatial_locations, request->config.ll_protocol, request->config.ll_mclk_role, request->config.ll_bclk_role,
    			request->config.ll_wclk_role, request->config.ll_wclk_polarity, request->config.ll_wclk_change_edge,
    			request->config.ll_data_tx_edge, request->config.ll_data_rx_edge, request->config.ll_data_offset);

    	memset(&dev_cfg, 0, sizeof(*request));
        dev_cfg.sample_frequency    = I2S_OUTPUT_SAMPLE_RATE;
        dev_cfg.num_channels        = NUMBER_OF_AUDIO_CHANNELS;
        dev_cfg.bytes_per_channel   = DATA_SIZE_PER_SAMPLE;
        dev_cfg.byte_order          = 4;
        dev_cfg.pad                 = 0;
        dev_cfg.spatial_locations   = 3;
        dev_cfg.ll_protocol         = 2;
        dev_cfg.ll_mclk_role        = 1;
        dev_cfg.ll_bclk_role        = 1;
        dev_cfg.ll_wclk_role        = 1;
        dev_cfg.ll_wclk_polarity    = 1;
        dev_cfg.ll_wclk_change_edge = 1;
        dev_cfg.ll_data_tx_edge     = 2;
        dev_cfg.ll_data_rx_edge     = 1;
        dev_cfg.ll_data_offset      = 1;

    	ret = device_i2s_set_configuration(gb_mixer.dev, &dev_cfg);

        if (ret) {
        	lldbg("failed to set i2s configuration.\n");
            return GB_OP_MALFUNCTION;
        }

        gb_i2s_copy_cfg_dev2gb(&dev_cfg, &gb_mixer.config);
        gb_mixer.flags |= GB_I2S_FLAGS_CONFIGURED;
    }

    // memcpy(&info->config, &gb_mixer.config, sizeof(info->config));
    memcpy(&info->config, &request->config, sizeof(info->config));
#endif

    info->sample_size = request->config.num_channels *
                        request->config.bytes_per_channel;

    info->flags |= GB_I2S_FLAGS_CONFIGURED;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_samples_per_message_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_samples_per_message_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    info->samples_per_message = le16_to_cpu(request->samples_per_message);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_get_processing_delay_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_get_processing_delay_response *response;
    struct gb_i2s_info *info;
    uint32_t microseconds;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

#ifndef ENABLE_DUAL_I2S_PORTS
    ret = device_i2s_get_processing_delay(info->dev, &microseconds);
#else
    ret = device_i2s_get_processing_delay(gb_mixer.dev, &microseconds);
#endif
    if (ret)
        return GB_OP_MALFUNCTION;

    /* TODO Figure out a real delay value */
    response->microseconds = cpu_to_le32(microseconds + 0);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_start_delay_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_start_delay_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    info->delay = le32_to_cpu(request->microseconds);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_activate_cport_req_handler(struct gb_operation *operation)
{
    struct gb_i2s_activate_cport_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_cport_list_entry *cple;
    struct gb_i2s_info *info;
    int allocated_dummy_data = 0;
    uint8_t ret = GB_OP_SUCCESS;

    lldbg("Reveiced activate_cport %d.\n", request->cport);

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if (!(info->flags & GB_I2S_FLAGS_CONFIGURED))
        return GB_OP_PROTOCOL_BAD;

    cple = gb_i2s_get_cple(info, le16_to_cpu(request->cport));
    if (!cple)
        return GB_OP_INVALID;

    if (cple->state != GB_I2S_CPORT_STATE_INACTIVE)
        return GB_OP_PROTOCOL_BAD;

    if ((info->active_tx_cports + info->active_rx_cports) == 0) {
        info->msg_data_size = info->sample_size * info->samples_per_message;

        info->dummy_data = zalloc(info->msg_data_size);
        if (!info->dummy_data) {
            return GB_OP_NO_MEMORY;
        }

        allocated_dummy_data = 1;
    }

    switch (cple->type) {
    case GB_I2S_CPORT_TYPE_RECEIVER:
    	lldbg("GB_I2S_CPORT_TYPE_RECEIVER\n");
        if (!info->active_rx_cports) {
            ret = gb_i2s_prepare_receiver(info);
            if (ret != GB_OP_SUCCESS)
                break;
            /* i2s driver started when first Send Data Request arrives */
        }

        cple->state = GB_I2S_CPORT_STATE_ACTIVE;
        info->active_rx_cports++;
        break;
    case GB_I2S_CPORT_TYPE_TRANSMITTER:
    	lldbg("GB_I2S_CPORT_TYPE_TRANSMITTER\n");
        if (!info->active_tx_cports) {
            ret = gb_i2s_prepare_transmitter(info);
            if (ret != GB_OP_SUCCESS)
                break;

            ret = gb_i2s_start_transmitter(info);
            if (ret != GB_OP_SUCCESS) {
                gb_i2s_shutdown_transmitter(info);
                break;
            }
        }

        cple->state = GB_I2S_CPORT_STATE_ACTIVE;
        info->active_tx_cports++;
        break;
    default:
    	lldbg("UNKNOWN TYPE!!\n");
        ret = GB_OP_MALFUNCTION;
    }

    if ((ret != GB_OP_SUCCESS) && allocated_dummy_data) {
        free(info->dummy_data);
        info->dummy_data = NULL;
    }

    return ret;
}

static uint8_t gb_i2s_deactivate_cport_req_handler(struct gb_operation
                                                                    *operation)
{
    struct gb_i2s_deactivate_cport_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_cport_list_entry *cple;
    struct gb_i2s_info *info;
    uint8_t ret = GB_OP_SUCCESS;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    cple = gb_i2s_get_cple(info, le16_to_cpu(request->cport));
    if (!cple)
        return GB_OP_INVALID;

    if (cple->state != GB_I2S_CPORT_STATE_ACTIVE)
        return GB_OP_PROTOCOL_BAD;

    cple->state = GB_I2S_CPORT_STATE_INACTIVE;

    switch (cple->type) {
    case GB_I2S_CPORT_TYPE_RECEIVER:
        info->active_rx_cports--;

        if (!info->active_rx_cports)
            gb_i2s_shutdown_receiver(info);
        break;
    case GB_I2S_CPORT_TYPE_TRANSMITTER:
        info->active_tx_cports--;

        if (!info->active_tx_cports) {
            gb_i2s_stop_transmitter(info);
            gb_i2s_shutdown_transmitter(info);
        }
        break;
    default:
        ret = GB_OP_MALFUNCTION;
    }

    if (info->dummy_data &&
        (info->active_tx_cports + info->active_rx_cports) == 0) {

        free(info->dummy_data);
        info->dummy_data = NULL;
    }

    return ret;
}

int gb_i2s_mgmt_init(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_info *info;
    int ret;

#ifndef ENABLE_DUAL_I2S_PORTS
    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
#else
    dev_info = gb_i2s_get_dev_info(get_cport_bundle(cport)); /* XXX */
    // dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
#endif

    if (!dev_info)
        return -EINVAL;

    if (dev_info->info)
        return -EBUSY;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

lldbg("==>0x%08p\n", info); /* XXX */

    info->mgmt_cport = cport;
    info->samples_per_message = GB_I2S_SAMPLES_PER_MSG_DEFAULT;
#ifndef DISABLE_TRANSMIT
    info->tx_wdog = wd_create();
#endif

    list_init(&info->cport_list);
#ifndef DISABLE_TRANSMIT
    sem_init(&info->tx_stop_sem, 0, 0);
    sem_init(&info->tx_rb_sem, 0, 0);
    atomic_init(&info->tx_rb_count, 0);

    ret = pthread_create(&info->tx_rb_thread, NULL, gb_i2s_tx_rb_thread,
                         info);
    if (ret) {
        ret = -ret;
        goto err_free_wd_info;
    }
#endif

#ifndef NO_I2S_AUDIO_DEVICE
#ifndef ENABLE_DUAL_I2S_PORTS
    info->dev = device_open(dev_info->dev_type, dev_info->dev_id);
    if (!info->dev) {
#else
    if ((atomic_inc(&gb_mixer.total_channel_count) == 1) &&
    	(gb_mixer.dev == NULL)) {
        int index;

        for (index = 0; index < MAX_AUDIO_CHANNELS; index++) {
        	gb_mixer.audio_channels[index].channel_info = NULL;
        	gb_mixer.audio_channels[index].rx_rb = NULL;
        }

        gb_mixer.dev = device_open(dev_info->dev_type, dev_info->dev_id);
    }
    if (!gb_mixer.dev) {
#endif
        ret = -EIO;
        goto err_kill_pthread;
    }
#endif

    dev_info->info = info;

    return 0;

#ifndef DISABLE_TRANSMIT
#ifndef NO_I2S_AUDIO_DEVICE
err_kill_pthread:
    pthread_kill(info->tx_rb_thread, SIGKILL);
#endif
err_free_wd_info:
    wd_delete(info->tx_wdog);
#endif

    free(info);

    return ret;
}

void gb_i2s_mgmt_exit(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;

#ifndef ENABLE_DUAL_I2S_PORTS
    // KIMMUI: ??? Is this right?
    dev_info = gb_i2s_get_dev_info(cport);
#else
    dev_info = gb_i2s_get_dev_info(get_cport_bundle(cport));
#endif

    if (!dev_info && !dev_info->info)
        return;

#ifndef ENABLE_DUAL_I2S_PORTS
    device_close(dev_info->info->dev);
#else
    if ((atomic_dec(&gb_mixer.total_channel_count) == 0) &&
    	(gb_mixer.dev != NULL)) {
        device_close(gb_mixer.dev);
        gb_mixer.dev = NULL;

    }
#endif

#ifndef DISABLE_TRANSMIT
    pthread_kill(dev_info->info->tx_rb_thread, SIGKILL);
    wd_cancel(dev_info->info->tx_wdog);
    wd_delete(dev_info->info->tx_wdog);
#endif

    free(dev_info->info);
    dev_info->info = NULL;
}

static struct gb_operation_handler gb_i2s_mgmt_handlers[] = {
    GB_HANDLER(GB_I2S_MGMT_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_SUPPORTED_CONFIGURATIONS,
               gb_i2s_get_supported_configurations_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_CONFIGURATION,
               gb_i2s_set_configuration_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE,
               gb_i2s_set_samples_per_message_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_GET_PROCESSING_DELAY,
               gb_i2s_get_processing_delay_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_SET_START_DELAY,
               gb_i2s_set_start_delay_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_ACTIVATE_CPORT,
               gb_i2s_activate_cport_req_handler),
    GB_HANDLER(GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT,
               gb_i2s_deactivate_cport_req_handler),
    /* GB_I2S_TYPE_REPORT_EVENT should only be received by the AP */
};

struct gb_driver i2s_mgmt_driver = {
    .init               = gb_i2s_mgmt_init,
    .exit               = gb_i2s_mgmt_exit,
    .op_handlers        = gb_i2s_mgmt_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_mgmt_handlers),
};

void gb_i2s_mgmt_register(int cport)
{
lldbg("cport %d\n", cport);
    gb_register_driver(cport, &i2s_mgmt_driver);
}

static int gb_i2s_cple_init(unsigned int cport, enum gb_i2s_cport_type type)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_cport_list_entry *cple;

#ifndef ENABLE_DUAL_I2S_PORTS
    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
#else
    dev_info = gb_i2s_get_dev_info(get_cport_bundle(cport)); /* XXX */
#endif

    lldbg("cport %d, %x\n", cport, dev_info);
    if (!dev_info || !dev_info->info)
        return -EINVAL;

    cple = zalloc(sizeof(*cple));
    if (!cple)
        return -ENOMEM;

    cple->info = dev_info->info;
    cple->cport = cport;
    cple->type = type;
    cple->state = GB_I2S_CPORT_STATE_INACTIVE;

    list_add(&dev_info->info->cport_list, &cple->list);

    return 0;
}

static int gb_i2s_cple_destroy(unsigned int cport)
{
    struct gb_i2s_info *info;
    struct gb_i2s_cport_list_entry *cple;

    info = gb_i2s_get_info_by_cport(cport);
    if (!info)
        return -EINVAL;

    cple = gb_i2s_find_cple(cport);
    if (!cple)
        return -EINVAL;

    list_del(&cple->list);
    free(cple);

    return 0;
}

int gb_i2s_receiver_init(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_cport_list_entry *cple;
    struct list_head *iter;

#ifndef ENABLE_DUAL_I2S_PORTS
    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
#else
    //dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
    dev_info = gb_i2s_get_dev_info(get_cport_bundle(cport)); /* XXX */
#endif

    if (!dev_info || !dev_info->info)
        return -EINVAL;

    /* Can have only one Receiver CPort in a bundle */
    list_foreach(&dev_info->info->cport_list, iter) {
        cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

        if (cple->type == GB_I2S_CPORT_TYPE_RECEIVER)
        {
            return -EINVAL;
        }
    }

    return gb_i2s_cple_init(cport, GB_I2S_CPORT_TYPE_RECEIVER);
}

void gb_i2s_receiver_exit(unsigned int cport)
{
    gb_i2s_cple_destroy(cport);
}

static struct gb_operation_handler gb_i2s_receiver_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
    GB_HANDLER(GB_I2S_DATA_TYPE_SEND_DATA,
               gb_i2s_receiver_send_data_req_handler),
};

struct gb_driver i2s_receiver_driver = {
    .init               = gb_i2s_receiver_init,
    .exit               = gb_i2s_receiver_exit,
    .op_handlers        = gb_i2s_receiver_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_receiver_handlers),
};

void gb_i2s_receiver_register(int cport)
{
    gb_register_driver(cport, &i2s_receiver_driver);
}

int gb_i2s_transmitter_init(unsigned int cport)
{
    return gb_i2s_cple_init(cport, GB_I2S_CPORT_TYPE_TRANSMITTER);
}

void gb_i2s_transmitter_exit(unsigned int cport)
{
    gb_i2s_cple_destroy(cport);
}

static struct gb_operation_handler gb_i2s_transmitter_handlers[] = {
    GB_HANDLER(GB_I2S_DATA_TYPE_PROTOCOL_VERSION,
               gb_i2s_protocol_version_req_handler),
};

struct gb_driver i2s_transmitter_driver = {
    .init               = gb_i2s_transmitter_init,
    .exit               = gb_i2s_transmitter_exit,
    .op_handlers        = gb_i2s_transmitter_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_transmitter_handlers),
};

void gb_i2s_transmitter_register(int cport)
{
    gb_register_driver(cport, &i2s_transmitter_driver);
}

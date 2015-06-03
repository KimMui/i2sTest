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
 * @brief Low-level TSB I2S device driver test program
 */
/*
 * Just a quick hack pgm to test out i2s & lr_stereo
 * (48KHz sampling, 2 channels per sample, 16-bits per channel).
 * Audio format confirmed by an I2S analyzer.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>
#include <nuttx/clock.h>

#include <nuttx/greybus/types.h>
#include <arch/armv7-m/byteorder.h>
#include <nuttx/greybus/greybus.h>
#include <../drivers/greybus/i2s-gb.h>

#define RESCHEDULE_TIME             4000    // Change this to simulate CPU load by vary how often work item is scheduled
                                            // Increasing RESCHEDULE_TIME resulting less CPU load.

#define MGMT_CPORT                  4
#define DATA_CPORT                  6
#define REMOTE_I2S_RX_CPORT         10

#define SAMPLE_FREQUENCY            48000
#define SAMPLES_PER_MESSAGE         192
#define NUMBER_OF_AUDIO_CHANNELS	2
#define DATA_SIZE_PER_SAMPLE        2
#define ONE_MICROSECOND             1000
#define ONE_SECOND                  1000000000L

#define I2S_START_DELAY             5000   // 5ms start delay

#define USE_SEND_REQUEST_SYNC

#ifndef USE_SEND_REQUEST_SYNC
#define SEND_REQEST(op)            gb_operation_send_request(op, NULL, false)
#else
#define SEND_REQEST(op)            gb_operation_send_request_sync(op)
#endif


static int set_i2s_configuration(unsigned int sample_rate)
{
    int ret = 0;
	struct gb_i2s_configuration *request;
    struct gb_operation *operation = NULL;

    operation = gb_operation_create(MGMT_CPORT,
    		                        GB_I2S_MGMT_TYPE_SET_CONFIGURATION,
                                    sizeof(*request));
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    memset(request, 0, sizeof(*request));
    request->sample_frequency    = sample_rate,
    request->num_channels        = NUMBER_OF_AUDIO_CHANNELS,
	request->bytes_per_channel   = DATA_SIZE_PER_SAMPLE,
	request->byte_order          = 4,
	request->pad                 = 0,
	request->spatial_locations   = 3,
	request->ll_protocol         = 2,
	request->ll_mclk_role        = 1,
	request->ll_bclk_role        = 1,
	request->ll_wclk_role        = 1,
	request->ll_wclk_polarity    = 1,
	request->ll_wclk_change_edge = 1,
	request->ll_data_tx_edge     = 2,
	request->ll_data_rx_edge     = 1,
	request->ll_data_offset      = 1,

    ret = SEND_REQEST(operation);

    lldbg("gb_operation_send_request returns %d\n", ret);

    return ret;
}

static int set_samples_per_msg(uint16_t samples_per_message)
{
    int ret = 0;
    struct gb_i2s_set_samples_per_message_request *request;
    struct gb_operation *operation = NULL;

    operation = gb_operation_create(MGMT_CPORT,
    		                        GB_I2S_MGMT_TYPE_SET_SAMPLES_PER_MESSAGE,
                                    sizeof(*request));
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    memset(request, 0, sizeof(*request));
    request->samples_per_message = cpu_to_le16(samples_per_message);

    ret = SEND_REQEST(operation);

    lldbg("gb_operation_send_request returns %d\n", ret);

    return ret;
}

static int set_start_delay(__le32 delay_time)
{
    int ret = 0;
    struct gb_i2s_set_start_delay_request *request;
    struct gb_operation *operation = NULL;

    operation = gb_operation_create(MGMT_CPORT,
    		                        GB_I2S_MGMT_TYPE_SET_START_DELAY,
                                    sizeof(*request));
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    memset(request, 0, sizeof(*request));
    request->microseconds = cpu_to_le32(delay_time);

    ret = SEND_REQEST(operation);

    lldbg("gb_operation_send_request returns %d\n", ret);

    return ret;
}


static int set_i2s_activate_rx_cport(void)
{
    int ret = 0;
	struct gb_i2s_activate_cport_request *request;
    struct gb_operation *operation = NULL;

    operation = gb_operation_create(MGMT_CPORT,
    	 	                        GB_I2S_MGMT_TYPE_ACTIVATE_CPORT,
                                    sizeof(*request));
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    request->cport = cpu_to_le16(REMOTE_I2S_RX_CPORT);
    ret = SEND_REQEST(operation);

    lldbg("gb_operation_send_request returns %d\n", ret);

    return ret;
}

static int set_i2s_deactivate_rx_cport(void)
{
    int ret = 0;
	struct gb_i2s_deactivate_cport_request *request;
    struct gb_operation *operation = NULL;

    operation = gb_operation_create(MGMT_CPORT,
    		                        GB_I2S_MGMT_TYPE_DEACTIVATE_CPORT,
                                    sizeof(*request));
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    request->cport = cpu_to_le16(REMOTE_I2S_RX_CPORT);
    ret = SEND_REQEST(operation);

    lldbg("gb_operation_send_request returns %d\n", ret);

    return ret;
}

static int send_data(unsigned int number_of_samples, __le32 sample_number)
{
    int ret = 0;
    struct gb_i2s_send_data_request *request;
    struct gb_operation *operation = NULL;
    __le32 payload_size            = (number_of_samples * NUMBER_OF_AUDIO_CHANNELS * DATA_SIZE_PER_SAMPLE);

    operation = gb_operation_create(DATA_CPORT,
    		                        GB_I2S_DATA_TYPE_SEND_DATA,
                                    sizeof(*request) + payload_size);
    if (!operation)
    {
    	lldbg("Failed to create greybus operation object.\n");
        return 1;
    }

    request = gb_operation_get_request_payload(operation);
    if (!request) {
    	lldbg("Failed to get request.\n");
        return 1;
    }

    request->sample_number = cpu_to_le32(sample_number);
    request->size = cpu_to_le32(payload_size);
    memset(&request->data[0], 0x5a, payload_size);
    ret = SEND_REQEST(operation);

    // gb_operation_destroy(operation);

    if (ret) {
    	lldbg("gb_operation_send_request returns %d\n", ret);
    }

    return ret;
}

static int stream_audio(unsigned int samples_per_packet, unsigned int packets_to_send, unsigned int packet_delay)
{
    struct timespec tm;
    sem_t           sem;
    int             ret = 0;
    int             loop_count = 0;

    sem_init(&sem, 0, 0);

    clock_gettime(CLOCK_REALTIME, &tm);
    tm.tv_sec += 1;

#if 0
    // Use this to test if system tick is correct!
    while (loop_count < 20)
    {
        ret = sem_timedwait(&sem, &tm);

        lldbg("====> %d\n", MSEC_PER_TICK);
        if (ret && (errno != ETIMEDOUT)) {
            lldbg("sem_timedwait failed. loopCount=%d, rc=%d errno=%d!\n", context.loop_count, ret, errno);
        }

        if (errno == EINTR)
        {
            lldbg("wait for sem. count=%d rc=%d errno=%d!\n", context.loop_count, ret, errno);
        } else {
            tm.tv_sec++;
        }
    	loop_count++;
    }

    return 0;
#endif

    while (packets_to_send > loop_count)
    {
        ret = sem_timedwait(&sem, &tm);

        if (ret && (errno != ETIMEDOUT)) {
            lldbg("sem_timedwait failed. loopCount=%d, rc=%d errno=%d!\n", loop_count, ret, errno);
        }

        if (errno == EINTR)
        {
            lldbg("wait for sem. count=%d rc=%d errno=%d!\n", loop_count, ret, errno);
        } else {
            send_data(samples_per_packet, loop_count);

            tm.tv_nsec += (packet_delay * ONE_MICROSECOND);
            if (tm.tv_nsec >= ONE_SECOND)
            {
                tm.tv_sec += 1;
                tm.tv_nsec -= ONE_SECOND;
            }
        }
        loop_count++;
    }

    lldbg("Send Data ended(loopCount=%d)!\n", loop_count);

    if (ret != 0)
    {
        lldbg("failed to wait for thread to terminate.\n");
        return 0;
    }

    return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int audio_stream_main(int argc, char *argv[])
#endif
{
    int ret = 0;
    unsigned int samples_rate            = SAMPLE_FREQUENCY;
    unsigned int sample_count_per_packet = 0;
    unsigned int packet_rate             = RESCHEDULE_TIME;
    unsigned int packet_count            = 10000;
    unsigned int start_delay             = I2S_START_DELAY;
    int          option;

	lldbg("=>%d, %s\n", argc, argv[0] );
    while ((option = getopt(argc, argv, "ar:s:f:p:d:")) != ERROR) {
    	lldbg("==> opt=%d, %s\n", option, optarg);
        switch(option) {
        case 'r':
        	samples_rate = (optarg == NULL) ? SAMPLE_FREQUENCY : atoi(optarg);
            break;
        case 's':
      	    sample_count_per_packet = (optarg == NULL) ? 0 : atoi(optarg);
            break;
        case 'f':
        	packet_rate = (optarg == NULL) ? RESCHEDULE_TIME : atoi(optarg);
            break;
        case 'p':
    	    packet_count = (optarg == NULL) ? 10000 : atoi(optarg);
            break;
        case 'd':
    	    packet_count = (optarg == NULL) ? I2S_START_DELAY : atoi(optarg);
            break;
        default:
            break;
        }
    }

    if (!packet_rate) {
    	packet_rate = 1000;
    }

    if (!sample_count_per_packet) {
    	sample_count_per_packet = ((samples_rate/1000) * packet_rate) / 1000;
    }

    lldbg("freq=%d, spp=%d, delay=%d, count=%d.\n",
    		samples_rate, sample_count_per_packet, packet_rate, packet_count);

    // return 0;

    ret = set_i2s_configuration(samples_rate);
    if (ret)
    	return 0;

    ret = set_samples_per_msg(sample_count_per_packet);
    if (ret)
    	return 0;

    ret = set_start_delay(I2S_START_DELAY);
    if (ret)
    	return 0;

    ret = set_i2s_activate_rx_cport();
    if (ret)
    	return 0;

    ret = stream_audio(sample_count_per_packet, packet_count, packet_rate);
    if (ret)
    	return 0;

    ret = set_i2s_deactivate_rx_cport();
    if (ret)
    	return 0;

    return 0;
}

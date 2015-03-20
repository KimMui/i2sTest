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
 * @brief TSB I2S stub device driver
 */
/*
 * Stub driver that does not access hardware but generates fake
 * audio data to pass to client and discard any data sent to it
 * by client.
 */
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/device.h>
#include <nuttx/device_table.h>
#include <nuttx/device_i2s.h>
#include <nuttx/ring_buf.h>

#include "up_arch.h"

#ifndef SIGTERM
#define SIGTERM		15
#endif

#define TSB_I2S_STUB_SAMPLE_FREQ            48000 /* Could make CONFIG option */

#define TSB_I2S_STUB_FLAG_OPEN              BIT(0)
#define TSB_I2S_STUB_FLAG_CONFIGURED        BIT(1)
#define TSB_I2S_STUB_FLAG_RX_PREPARED       BIT(2)
#define TSB_I2S_STUB_FLAG_RX_ACTIVE         BIT(3)
#define TSB_I2S_STUB_FLAG_TX_PREPARED       BIT(4)
#define TSB_I2S_STUB_FLAG_TX_ACTIVE         BIT(5)

struct tsb_i2s_stub_sample {
    uint16_t    left;
    uint16_t    right;
};

struct tsb_i2s_stub_info {
    struct device                   *dev;
    uint32_t                        flags;
    struct ring_buf                 *rx_rb;
    void                            (*rx_callback)(struct ring_buf *rb,
                                                   enum device_i2s_event event,
                                                   void *arg);
    void                            *rx_arg;
    struct ring_buf                 *tx_rb;
    pthread_mutex_t                 rx_lock;
    pthread_cond_t                  rx_cond;
    pthread_t					    rx_thread;
    void                            (*tx_callback)(struct ring_buf *rb,
                                                   enum device_i2s_event event,
                                                   void *arg);
    void                            *tx_arg;
    pthread_mutex_t                 tx_lock;
    pthread_cond_t                  tx_cond;
    pthread_t					    tx_thread;
    struct device_i2s_configuration config;
    unsigned long                   samples_per_rb;
	struct timespec				    delay;
};

static const struct device_i2s_configuration tsb_i2s_stub_config_table[] = {
    {
        .sample_frequency       = 48000,
        .num_channels           = 2,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_I2S,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 1,
    },
};

static int tsb_i2s_one_bit_is_set(uint64_t bits)
{
    return bits && !(bits & (bits - 1));
}

static int tsb_i2s_stub_config_is_valid(
                                struct device_i2s_configuration *configuration)
{
    const struct device_i2s_configuration *config;
    unsigned int i;

    if (!tsb_i2s_one_bit_is_set(configuration->byte_order) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_protocol) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_bclk_role) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_wclk_role) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_wclk_polarity) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_wclk_change_edge) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_wclk_tx_edge) ||
        !tsb_i2s_one_bit_is_set(configuration->ll_wclk_rx_edge)) {

        return 0;
    }

    for (i = 0, config = tsb_i2s_stub_config_table;
         i < ARRAY_SIZE(tsb_i2s_stub_config_table);
         i++, config++) {

        if ((config->sample_frequency == configuration->sample_frequency) &&
            (config->num_channels == configuration->num_channels) &&
            (config->bytes_per_channel == configuration->bytes_per_channel) &&
            (config->byte_order & configuration->byte_order) &&
            (config->spatial_locations & configuration->spatial_locations) &&
            (config->ll_protocol & configuration->ll_protocol) &&
            (config->ll_bclk_role & configuration->ll_bclk_role) &&
            (config->ll_wclk_role & configuration->ll_wclk_role) &&
            (configuration->ll_bclk_role ==
                             configuration->ll_wclk_role) && /* Must be = */
            (config->ll_wclk_polarity & configuration->ll_wclk_polarity) &&
            (config->ll_wclk_change_edge &
                                     configuration->ll_wclk_change_edge) &&
            (config->ll_wclk_tx_edge & configuration->ll_wclk_tx_edge) &&
            (config->ll_wclk_rx_edge & configuration->ll_wclk_rx_edge) &&
            (config->ll_data_offset == configuration->ll_data_offset)) {

            return 1;
        }
    }

    return 0;
}

static int tsb_i2s_stub_device_is_open(struct tsb_i2s_stub_info *info)
{
    return !!(info->flags & TSB_I2S_STUB_FLAG_OPEN);
}

static int tsb_i2s_stub_device_is_configured(struct tsb_i2s_stub_info *info)
{
    return tsb_i2s_stub_device_is_open(info) &&
           (info->flags & TSB_I2S_STUB_FLAG_CONFIGURED);
}

static int tsb_i2s_stub_rx_is_prepared(struct tsb_i2s_stub_info *info)
{
    return tsb_i2s_stub_device_is_configured(info) &&
           (info->flags & TSB_I2S_STUB_FLAG_RX_PREPARED);
}

static int tsb_i2s_stub_tx_is_prepared(struct tsb_i2s_stub_info *info)
{
    return tsb_i2s_stub_device_is_configured(info) &&
           (info->flags & TSB_I2S_STUB_FLAG_TX_PREPARED);
}

static int tsb_i2s_stub_rx_is_active(struct tsb_i2s_stub_info *info)
{
    return tsb_i2s_stub_rx_is_prepared(info) &&
           (info->flags & TSB_I2S_STUB_FLAG_RX_ACTIVE);
}

static int tsb_i2s_stub_tx_is_active(struct tsb_i2s_stub_info *info)
{
    return tsb_i2s_stub_tx_is_prepared(info) &&
           (info->flags & TSB_I2S_STUB_FLAG_TX_ACTIVE);
}

static int tsb_i2s_fill_rb(struct ring_buf *rb)
{
    struct tsb_i2s_stub_sample *sample;
    unsigned int i;

    sample = ring_buf_get_tail(rb);

    for (i = 0; i < (ring_buf_space(rb) / sizeof(*sample)); i++) {
        sample[i].left = 0xdead;
        sample[i].right = 0xbeef;
    }

    ring_buf_put(rb, i * sizeof(*sample));

    return 0;
}

static void *tsb_i2s_stub_rx_thread(void *data)
{
	struct tsb_i2s_stub_info *info = data;
	enum device_i2s_event event;
	struct ring_buf *rb;
	struct timespec rqt;

    pthread_mutex_lock(&info->rx_lock);
    while (!tsb_i2s_stub_rx_is_active(info))
        pthread_cond_wait(&info->rx_cond, &info->rx_lock);
    pthread_mutex_unlock(&info->rx_lock);

	rb = info->rx_rb;

	while (1) {
		if (ring_buf_is_producers(rb) && !ring_buf_is_full(rb)) {
			/* Discard data, zero buffer */
			memset(ring_buf_get_head(rb), 0, ring_buf_len(rb));

			tsb_i2s_fill_rb(rb);
			ring_buf_pass(rb);

			if (ring_buf_len(rb) % 4)
				event = DEVICE_I2S_EVENT_DATA_LEN;
			else
				event = DEVICE_I2S_EVENT_RX_COMPLETE;

			info->rx_callback(rb, event, info->rx_arg);

			rb = ring_buf_get_next(rb);
		}

		memcpy(&rqt, &info->delay, sizeof(rqt));

		while (nanosleep(&rqt, &rqt) && (errno == EINTR));
	}

    return NULL;
}



unsigned int mag_tx_count = 0;

static void *tsb_i2s_stub_tx_thread(void *data)
{
	struct tsb_i2s_stub_info *info = data;
	enum device_i2s_event event;
	struct ring_buf *rb;
	struct timespec rqt;

    pthread_mutex_lock(&info->tx_lock);
    while (!tsb_i2s_stub_tx_is_active(info))
        pthread_cond_wait(&info->tx_cond, &info->tx_lock);
    pthread_mutex_unlock(&info->tx_lock);

	rb = info->tx_rb;

	while (1) {
		if (ring_buf_is_consumers(rb) && !ring_buf_is_empty(rb)) {
			/* Discard data, zero buffer */
			memset(ring_buf_get_head(rb), 0, ring_buf_len(rb));

			ring_buf_reset(rb);
			ring_buf_pass(rb);

			if (ring_buf_len(rb) % 4)
				event = DEVICE_I2S_EVENT_DATA_LEN;
			else
				event = DEVICE_I2S_EVENT_TX_COMPLETE;

			info->tx_callback(rb, event, info->tx_arg);

            mag_tx_count++;

			rb = ring_buf_get_next(rb);
		}

		memcpy(&rqt, &info->delay, sizeof(rqt));

		while (nanosleep(&rqt, &rqt) && (errno == EINTR));
	}

    return NULL;
}

static int tsb_i2s_stub_op_get_processing_delay(struct device *dev,
                                           uint32_t *processing_delay)
{
    *processing_delay = 0; /* XXX */

    return 0;
}

static int tsb_i2s_stub_op_get_supported_configurations(struct device *dev,
                        uint16_t *configuration_count,
                        const struct device_i2s_configuration *configurations[])
{
    *configuration_count = ARRAY_SIZE(tsb_i2s_stub_config_table);
    *configurations = tsb_i2s_stub_config_table;

    return 0;
}

static int tsb_i2s_stub_op_set_configuration(struct device *dev,
                                 struct device_i2s_configuration *configuration)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_config_is_valid(configuration))
        return -EINVAL;

    if (tsb_i2s_stub_rx_is_prepared(info) || tsb_i2s_stub_tx_is_prepared(info))
        return -EBUSY;

    memcpy(&info->config, configuration, sizeof(info->config));

    info->flags |= TSB_I2S_STUB_FLAG_CONFIGURED;

    return 0;
}

static int tsb_i2s_stub_op_prepare_receiver(struct device *dev,
                                            struct ring_buf *rx_rb,
                                            void (*callback)(
                                                    struct ring_buf *rb,
                                                    enum device_i2s_event event,
                                                    void *arg),
                                            void *arg)
{
    struct tsb_i2s_stub_info *info = dev->private;
	unsigned long ns;

    if (!rx_rb || !callback)
        return -EINVAL;

    if (!tsb_i2s_stub_device_is_configured(info) ||
        tsb_i2s_stub_rx_is_prepared(info) ||
        tsb_i2s_stub_tx_is_prepared(info)) {

        return -EIO;
    }

    info->rx_rb = rx_rb;
    info->rx_callback = callback;
    info->rx_arg = arg;

    /* Assumed rb entries are empty */
    info->samples_per_rb = ring_buf_space(info->rx_rb) /
                           sizeof(struct tsb_i2s_stub_sample);

    ns = (info->samples_per_rb * 1000000) / (TSB_I2S_STUB_SAMPLE_FREQ / 1000);

	info->delay.tv_sec = ns / 1000000000;
	info->delay.tv_nsec = ns % 1000000000;

    info->flags |= TSB_I2S_STUB_FLAG_RX_PREPARED;

    return 0;
}

static int tsb_i2s_stub_op_start_receiver(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_rx_is_prepared(info) || tsb_i2s_stub_rx_is_active(info))
        return -EIO;

    info->flags |= TSB_I2S_STUB_FLAG_RX_ACTIVE;

    pthread_cond_signal(&info->rx_cond);

    return 0;
}

static int tsb_i2s_stub_op_stop_receiver(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_rx_is_active(info))
        return -EIO;

    info->flags &= ~TSB_I2S_STUB_FLAG_RX_ACTIVE;

    return 0;
}

static int tsb_i2s_stub_op_shutdown_receiver(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (tsb_i2s_stub_rx_is_active(info))
        return -EBUSY;

    if (!tsb_i2s_stub_rx_is_prepared(info))
        return -EIO;

    info->rx_rb = NULL;
    info->rx_callback = NULL;
    info->rx_arg = NULL;

    info->flags &= ~TSB_I2S_STUB_FLAG_RX_PREPARED;

    return 0;
}

static int tsb_i2s_stub_op_prepare_transmitter(struct device *dev,
                                               struct ring_buf *tx_rb,
                                               void (*callback)(
                                                    struct ring_buf *rb,
                                                    enum device_i2s_event event,
                                                    void *arg),
                                               void *arg)
{
    struct tsb_i2s_stub_info *info = dev->private;
	unsigned long ns;

    if (!tx_rb || !callback)
        return -EINVAL;

    if (!tsb_i2s_stub_device_is_configured(info) ||
        tsb_i2s_stub_rx_is_prepared(info) ||
        tsb_i2s_stub_tx_is_prepared(info)) {

        return -EIO;
    }

    info->tx_rb = tx_rb;
    info->tx_callback = callback;
    info->tx_arg = arg;

    /* Assumed rb entries are filled */
    info->samples_per_rb = ring_buf_len(info->tx_rb) /
                           sizeof(struct tsb_i2s_stub_sample);

    ns = (info->samples_per_rb * 1000000) / (TSB_I2S_STUB_SAMPLE_FREQ / 1000);

	info->delay.tv_sec = ns / 1000000000;
	info->delay.tv_nsec = ns % 1000000000;

    info->flags |= TSB_I2S_STUB_FLAG_TX_PREPARED;

    return 0;
}

static int tsb_i2s_stub_op_start_transmitter(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_tx_is_prepared(info) || tsb_i2s_stub_tx_is_active(info))
        return -EIO;

    info->flags |= TSB_I2S_STUB_FLAG_TX_ACTIVE;

    pthread_cond_signal(&info->tx_cond);

    return 0;
}

static int tsb_i2s_stub_op_stop_transmitter(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_tx_is_active(info))
        return -EIO;

    info->flags &= ~TSB_I2S_STUB_FLAG_TX_ACTIVE;

    return 0;
}

static int tsb_i2s_stub_op_shutdown_transmitter(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (tsb_i2s_stub_tx_is_active(info))
        return -EBUSY;

    if (!tsb_i2s_stub_tx_is_prepared(info))
        return -EIO;

    info->tx_rb = NULL;
    info->tx_callback = NULL;
    info->tx_arg = NULL;

    info->flags &= ~TSB_I2S_STUB_FLAG_TX_PREPARED;

    return 0;
}

static int tsb_i2s_stub_dev_open(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;
    int ret;

    if (tsb_i2s_stub_device_is_open(info))
        return -EBUSY;

    ret = pthread_mutex_init(&info->rx_lock, NULL);
    if (ret)
        return -ret;

    ret = pthread_cond_init(&info->rx_cond, NULL);
    if (ret)
        return -ret;

    ret = pthread_create(&info->rx_thread, NULL, tsb_i2s_stub_rx_thread,
                         info);
    if (ret)
        return -ret;

    ret = pthread_mutex_init(&info->tx_lock, NULL);
    if (ret)
        goto err_kill_rx_thread;

    ret = pthread_cond_init(&info->tx_cond, NULL);
    if (ret)
        goto err_kill_rx_thread;

    ret = pthread_create(&info->tx_thread, NULL, tsb_i2s_stub_tx_thread,
                         info);
    if (ret)
        goto err_kill_rx_thread;

    info->flags = TSB_I2S_STUB_FLAG_OPEN;

    return 0;

err_kill_rx_thread:
    pthread_kill(info->rx_thread, SIGTERM);
    pthread_join(info->rx_thread, NULL);

    return -ret;
}

static void tsb_i2s_stub_dev_close(struct device *dev)
{
    struct tsb_i2s_stub_info *info = dev->private;

    if (!tsb_i2s_stub_device_is_open(info))
        return;

    if (tsb_i2s_stub_rx_is_active(info))
        tsb_i2s_stub_op_stop_receiver(dev);

    if (tsb_i2s_stub_tx_is_active(info))
        tsb_i2s_stub_op_stop_transmitter(dev);

    if (tsb_i2s_stub_rx_is_prepared(info))
        tsb_i2s_stub_op_shutdown_receiver(dev);

    if (tsb_i2s_stub_tx_is_prepared(info))
        tsb_i2s_stub_op_shutdown_transmitter(dev);

    pthread_cancel(info->tx_thread);
    pthread_join(info->tx_thread, NULL);

    pthread_cancel(info->rx_thread);
    pthread_join(info->rx_thread, NULL);

    info->flags = 0;
}

static int tsb_i2s_stub_dev_probe(struct device *dev)
{
    struct tsb_i2s_stub_info *info;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

    info->dev = dev;
    dev->private = info;

    return OK;
}

static void tsb_i2s_stub_dev_remove(struct device *dev)
{
    free(dev->private);
    dev->private = NULL;
}

static struct device_i2s_class_ops tsb_i2s_stub_class_ops = {
    .get_processing_delay         = tsb_i2s_stub_op_get_processing_delay,
    .get_supported_configurations =
                                   tsb_i2s_stub_op_get_supported_configurations,
    .set_configuration            = tsb_i2s_stub_op_set_configuration,
    .prepare_receiver             = tsb_i2s_stub_op_prepare_receiver,
    .start_receiver               = tsb_i2s_stub_op_start_receiver,
    .stop_receiver                = tsb_i2s_stub_op_stop_receiver,
    .shutdown_receiver            = tsb_i2s_stub_op_shutdown_receiver,
    .prepare_transmitter          = tsb_i2s_stub_op_prepare_transmitter,
    .start_transmitter            = tsb_i2s_stub_op_start_transmitter,
    .stop_transmitter             = tsb_i2s_stub_op_stop_transmitter,
    .shutdown_transmitter         = tsb_i2s_stub_op_shutdown_transmitter,
};

static struct device_driver_ops tsb_i2s_stub_driver_ops = {
    .probe          = tsb_i2s_stub_dev_probe,
    .remove         = tsb_i2s_stub_dev_remove,
    .open           = tsb_i2s_stub_dev_open,
    .close          = tsb_i2s_stub_dev_close,
    .class_ops.i2s  = &tsb_i2s_stub_class_ops,
};

struct device_driver tsb_i2s_stub_driver = {
    .class      = DEVICE_CLASS_I2S_HW,
    .name       = "tsb_i2s",
    .desc       = "TSB I2S Stub Driver",
    .ops        = &tsb_i2s_stub_driver_ops,
};

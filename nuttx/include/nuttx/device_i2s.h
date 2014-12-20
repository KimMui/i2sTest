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
 */

#ifndef __ARCH_ARM_DEVICE_I2S_H
#define __ARCH_ARM_DEVICE_I2S_H

#include <nuttx/util.h>
#include <nuttx/device.h>
#include <nuttx/ring_buf.h>

#define DEVICE_CLASS_I2S_HW                     "i2s"

#define DEVICE_I2S_BYTE_ORDER_NA                BIT(0)
#define DEVICE_I2S_BYTE_ORDER_BE                BIT(1)
#define DEVICE_I2S_BYTE_ORDER_LE                BIT(2)

#define DEVICE_I2S_SPATIAL_LOCATION_FL          BIT(0)
#define DEVICE_I2S_SPATIAL_LOCATION_FR          BIT(1)
#define DEVICE_I2S_SPATIAL_LOCATION_FC          BIT(2)
#define DEVICE_I2S_SPATIAL_LOCATION_LFE         BIT(3)
#define DEVICE_I2S_SPATIAL_LOCATION_BL          BIT(4)
#define DEVICE_I2S_SPATIAL_LOCATION_BR          BIT(5)
#define DEVICE_I2S_SPATIAL_LOCATION_FLC         BIT(6)
#define DEVICE_I2S_SPATIAL_LOCATION_FRC         BIT(7)
#define DEVICE_I2S_SPATIAL_LOCATION_C           BIT(8) /* BC in USB Spec */
#define DEVICE_I2S_SPATIAL_LOCATION_SL          BIT(9)
#define DEVICE_I2S_SPATIAL_LOCATION_SR          BIT(10)
#define DEVICE_I2S_SPATIAL_LOCATION_TC          BIT(11)
#define DEVICE_I2S_SPATIAL_LOCATION_TFL         BIT(12)
#define DEVICE_I2S_SPATIAL_LOCATION_TFC         BIT(13)
#define DEVICE_I2S_SPATIAL_LOCATION_TFR         BIT(14)
#define DEVICE_I2S_SPATIAL_LOCATION_TBL         BIT(15)
#define DEVICE_I2S_SPATIAL_LOCATION_TBC         BIT(16)
#define DEVICE_I2S_SPATIAL_LOCATION_TBR         BIT(17)
#define DEVICE_I2S_SPATIAL_LOCATION_TFLC        BIT(18)
#define DEVICE_I2S_SPATIAL_LOCATION_TFRC        BIT(19)
#define DEVICE_I2S_SPATIAL_LOCATION_LLFE        BIT(20)
#define DEVICE_I2S_SPATIAL_LOCATION_RLFE        BIT(21)
#define DEVICE_I2S_SPATIAL_LOCATION_TSL         BIT(22)
#define DEVICE_I2S_SPATIAL_LOCATION_TSR         BIT(23)
#define DEVICE_I2S_SPATIAL_LOCATION_BC          BIT(24)
#define DEVICE_I2S_SPATIAL_LOCATION_BLC         BIT(25)
#define DEVICE_I2S_SPATIAL_LOCATION_BRC         BIT(26)
#define DEVICE_I2S_SPATIAL_LOCATION_RD          BIT(31)

#define DEVICE_I2S_PROTOCOL_PCM                 BIT(0)
#define DEVICE_I2S_PROTOCOL_I2S                 BIT(1)
#define DEVICE_I2S_PROTOCOL_LR_STEREO           BIT(2)

#define DEVICE_I2S_ROLE_MASTER                  BIT(0)
#define DEVICE_I2S_ROLE_SLAVE                   BIT(1)

#define DEVICE_I2S_POLARITY_NORMAL              BIT(0)
#define DEVICE_I2S_POLARITY_REVERSED            BIT(1)

#define DEVICE_I2S_EDGE_RISING                  BIT(0)
#define DEVICE_I2S_EDGE_FALLING                 BIT(1)

enum device_i2s_event {
    DEVICE_I2S_EVENT_INVALID,
    DEVICE_I2S_EVENT_NONE,
    DEVICE_I2S_EVENT_UNSPECIFIED, /* Catch-all */
    DEVICE_I2S_EVENT_TX_COMPLETE,
    DEVICE_I2S_EVENT_RX_COMPLETE,
    DEVICE_I2S_EVENT_UNDERRUN,
    DEVICE_I2S_EVENT_OVERRUN,
    DEVICE_I2S_EVENT_CLOCKING,
    DEVICE_I2S_EVENT_DATA_LEN,
};

struct device_i2s_configuration {
    uint32_t    sample_frequency;
    uint8_t     num_channels;
    uint8_t     bytes_per_channel;
    uint8_t     byte_order;
    uint8_t     pad;
    uint32_t    spatial_locations;
    uint32_t    ll_protocol;
    uint8_t     ll_bclk_role;
    uint8_t     ll_wclk_role;
    uint8_t     ll_wclk_polarity;
    uint8_t     ll_wclk_change_edge;
    uint8_t     ll_wclk_tx_edge;
    uint8_t     ll_wclk_rx_edge;
    uint8_t     ll_data_offset;
    uint8_t     ll_pad;
};

struct device_i2s_class_ops {
    int (*get_processing_delay)(struct device *dev,
                                uint32_t *processing_delay);
    int (*get_supported_configurations)(struct device *dev,
                                        uint16_t *configuration_count,
                                        const struct device_i2s_configuration
                                                             *configurations[]);
    int (*set_configuration)(struct device *dev,
                             struct device_i2s_configuration *configuration);
    int (*prepare_receiver)(struct device *dev,
                            struct ring_buf *rx_rb,
                            void (*callback)(struct ring_buf *rb,
                                             enum device_i2s_event event,
                                             void *arg),
                            void *arg);
    int (*start_receiver)(struct device *dev);
    int (*stop_receiver)(struct device *dev);
    int (*shutdown_receiver)(struct device *dev);
    int (*prepare_transmitter)(struct device *dev,
                               struct ring_buf *tx_rb,
                               void (*callback)(struct ring_buf *rb,
                                                enum device_i2s_event event,
                                                void *arg),
                               void *arg);
    int (*start_transmitter)(struct device *dev);
    int (*stop_transmitter)(struct device *dev);
    int (*shutdown_transmitter)(struct device *dev);
};

static inline int device_i2s_get_processing_delay(struct device *dev,
                                                  uint32_t *processing_delay)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->get_processing_delay)
        return dev->driver->ops->class_ops.i2s->get_processing_delay(dev,
                                                              processing_delay);

    return -EOPNOTSUPP;
}

static inline int device_i2s_get_supported_configurations(
                        struct device *dev,
                        uint16_t *configuration_count,
                        const struct device_i2s_configuration *configurations[])
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->get_supported_configurations)
        return dev->driver->ops->class_ops.i2s->
                    get_supported_configurations(dev, configuration_count,
                                                 configurations);

    return -EOPNOTSUPP;
}

static inline int device_i2s_set_configuration(struct device *dev,
                                 struct device_i2s_configuration *configuration)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->set_configuration)
        return dev->driver->ops->class_ops.i2s->set_configuration(dev,
                                                                 configuration);

    return -EOPNOTSUPP;
}

static inline int device_i2s_prepare_receiver(struct device *dev,
                                              struct ring_buf *rx_rb,
                                              void (*callback)(
                                                    struct ring_buf *rb,
                                                    enum device_i2s_event event,
                                                    void *arg),
                                              void *arg)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->prepare_receiver)
        return dev->driver->ops->class_ops.i2s->prepare_receiver(dev, rx_rb,
                                                                 callback, arg);

    return -EOPNOTSUPP;
}

static inline int device_i2s_start_receiver(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->start_receiver)
        return dev->driver->ops->class_ops.i2s->start_receiver(dev);

    return -EOPNOTSUPP;
}

static inline int device_i2s_stop_receiver(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->stop_receiver)
        return dev->driver->ops->class_ops.i2s->stop_receiver(dev);

    return -EOPNOTSUPP;
}

static inline int device_i2s_shutdown_receiver(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->shutdown_receiver)
        return dev->driver->ops->class_ops.i2s->shutdown_receiver(dev);

    return -EOPNOTSUPP;
}

static inline int device_i2s_prepare_transmitter(struct device *dev,
                                                 struct ring_buf *tx_rb,
                                                 void (*callback)(
                                                    struct ring_buf *rb,
                                                    enum device_i2s_event event,
                                                    void *arg),
                                                 void *arg)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->prepare_transmitter)
        return dev->driver->ops->class_ops.i2s->prepare_transmitter(dev, tx_rb,
                                                                    callback,
                                                                    arg);
    return -EOPNOTSUPP;
}

static inline int device_i2s_start_transmitter(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->start_transmitter)
        return dev->driver->ops->class_ops.i2s->start_transmitter(dev);

    return -EOPNOTSUPP;
}

static inline int device_i2s_stop_transmitter(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->stop_transmitter)
        return dev->driver->ops->class_ops.i2s->stop_transmitter(dev);

    return -EOPNOTSUPP;
}

static inline int device_i2s_shutdown_transmitter(struct device *dev)
{
    if (dev->state != DEVICE_STATE_OPEN)
        return -ENODEV;

    if (dev->driver->ops->class_ops.i2s->shutdown_transmitter)
        return dev->driver->ops->class_ops.i2s->shutdown_transmitter(dev);

    return -EOPNOTSUPP;
}

#endif /* __ARCH_ARM_DEVICE_I2S_H */

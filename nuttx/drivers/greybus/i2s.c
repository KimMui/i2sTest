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
 * The overall data flos is from a Greybus transmitter module to one ore more
 * Greybus receiver modules.  However, a Greybus transmitter will the receiver
 * on its local i2s connection.  Similarly, a Greybus receiver will be the
 * transmitter on its local i2s connection.  This can get confusing in the
 * code.  To help with this, all of the routines defined in this file refer
 * to the Greybus point of view.  So routine names, struct members, etc.
 * that have "transmit" or "tx" in their name are related to receiving data
 * from the local i2s connection and transmitting it over the UniPro network.
 */


#undef GB_I2S_FAST_HANDLER /* XXX */
#undef GB_I2S_OLD_TX_METHOD /* XXX */

/* XXX
 * Currently supports only one I2S Bundle because manifest data doesn't
 * provide any info on which bundle a CPort belongs to.
 *
 * Supports only one Receiver CPort per I2S Bundle because all received
 * audio data goes to the one physical low-level controller.  So if there
 * is a mixer with two I2S connections, say, then use two separate I2S Bundles.
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

#include "i2s-gb.h"

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

#define GB_I2S_FLAGS_CONFIGURED         BIT(0)
#define GB_I2S_FLAGS_RX_PREPARED        BIT(1)
#define GB_I2S_FLAGS_RX_STARTED         BIT(2)
#define GB_I2S_FLAGS_TX_PREPARED        BIT(3)
#define GB_I2S_FLAGS_TX_STARTED         BIT(4)
#define GB_I2S_FLAGS_TX_DELAYING        BIT(5)
#define GB_I2S_FLAGS_TX_STOPPING        BIT(6)

#ifndef SIGKILL
#define SIGKILL     9
#endif

#ifndef MIN
#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
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

struct gb_i2s_info;
struct gb_i2s_cport_list_entry {
    struct list_head        list;
    struct gb_i2s_info      *info;
    uint16_t                cport;
    enum gb_i2s_cport_type  type;
    enum gb_i2s_cport_state state;
};

struct gb_i2s_info { /* One per I2S Bundle */
    uint16_t                    mgmt_cport;
    uint32_t                    flags;
    struct device               *dev;
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
    struct ring_buf             *tx_rb;
    sem_t                       tx_stop_sem;
    sem_t                       tx_rb_sem;
    atomic_t                    tx_rb_count;
    pthread_t                   tx_rb_thread;
    WDOG_ID                     tx_wdog;

    /* XXX temporary */
    uint32_t                    rx_rb_count;
    uint32_t                    rx_rb_underrun_count;
};

struct gb_i2s_dev_info {
    uint16_t            bundle_id;      /* XXX Fix when bundles implemented */
    char                *dev_class;
    unsigned int        dev_id;
    struct gb_i2s_info  *info;
};

static struct gb_i2s_dev_info gb_i2s_dev_info_map[] = {
    {
        .bundle_id  = GB_I2S_BUNDLE_0_ID, /* XXX Fix when bundles implemented */
        .dev_class  = DEVICE_CLASS_I2S_HW,
        .dev_id     = GB_I2S_BUNDLE_0_DEV_ID,
    },
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

#if 0 /* XXX - causes panic on AP (need rx handler in i2s_mgmt.c) */
    operation = gb_operation_create(info->mgmt_cport,
                                    GB_I2S_MGMT_TYPE_REPORT_EVENT,
                                    sizeof(*hdr) + sizeof(*request));
    if (!operation)
        return;

    request = gb_operation_get_response_payload(operation);
    request->event = event;

    /* XXX Shut things down if can't reach AP? */
    gb_operation_send_request_sync(operation);
#else
lldbg("--- Report event: %d\n", event); /* XXX */
#endif
}

static void gb_i2s_ll_tx(struct gb_i2s_info *info, uint8_t *data); /* XXX */

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

    if (gb_event)
        gb_i2s_report_event(info, gb_event);
}

static uint8_t gb_i2s_prepare_receiver(struct gb_i2s_info *info)
{
    unsigned int entries;
    int ret;

    if (info->flags & GB_I2S_FLAGS_RX_PREPARED)
        return GB_OP_PROTOCOL_BAD;

    /* (sample_freq / samples_per_msg) * (delay_in_us / 1,000,000) */
    entries = ((info->config.sample_frequency * info->delay) /
                    (info->samples_per_message * 1000000)) +
               GB_I2S_RX_RING_BUF_PAD;

lldbg("--- prepare_receiver: entries = %d\n", entries); /* XXX */

    info->rx_rb = ring_buf_alloc_ring(entries,
                                    sizeof(struct gb_operation_hdr) +
                                        sizeof(struct gb_i2s_send_data_request),
                                    info->msg_data_size, 0, NULL, NULL, NULL);
    if (!info->rx_rb)
        return GB_OP_NO_MEMORY;

    /* Greybus i2s message receiver is local i2s transmitter */
    ret = device_i2s_prepare_transmitter(info->dev, info->rx_rb,
                                             gb_i2s_ll_tx_cb, info);
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
    irqstate_t flags;

    flags = irqsave();

    ring_buf_reset(info->rx_rb);

    /*
     * Unfortunately, we have to copy the data even though its a fast path
     * handler.  This is because the unipro susbystem reuses the buffer
     * immediately while the data could still be sitting in the ring buffer.
     */
    memcpy(ring_buf_get_tail(info->rx_rb), data, info->msg_data_size);

    ring_buf_put(info->rx_rb, info->msg_data_size);
    ring_buf_pass(info->rx_rb);

    info->next_rx_sample += info->samples_per_message;
    info->rx_rb = ring_buf_get_next(info->rx_rb);

    info->rx_rb_count++;

    irqrestore(flags);
}

uint32_t mag_gb_rx_count = 0; /* XXX */
uint32_t mag_gb_or = 0; /* XXX */

#ifdef GB_I2S_FAST_HANDLER
static void gb_i2s_receiver_send_data_req_handler(unsigned int cport,
                                                  void *data)
{
    struct gb_i2s_info *info;
    struct gb_i2s_send_data_request *request;
    int ret;

    info = gb_i2s_get_info_by_cport(cport);
    if (!info || !(info->flags & GB_I2S_FLAGS_RX_PREPARED)) {
        gb_i2s_report_event(info, GB_I2S_EVENT_PROTOCOL_ERROR);
        return;
    }

    request = data + sizeof(struct gb_operation_hdr);

    if (request->size != info->msg_data_size) {
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        return;
    }

#if 0 /* XXX send anyway */
    /* XXX Handle wrap around to 0 */
    if (request->sample_number < info->next_rx_sample) {
        gb_i2s_report_event(info, GB_I2S_EVENT_OUT_OF_SEQUENCE);
        return;
    }
#endif

    /* Fill in any missing data */
    while (ring_buf_is_producers(info->rx_rb) &&
           (request->sample_number > info->next_rx_sample))
        gb_i2s_ll_tx(info, info->dummy_data);

    if (!ring_buf_is_producers(info->rx_rb)) {
#if 0 /* XXX */
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
#else
        mag_gb_or++; /* XXX */
#endif
        return;
    }

    gb_i2s_ll_tx(info, request->data);

#ifdef GB_I2S_OLD_TX_METHOD
    if (!(info->flags & GB_I2S_FLAGS_RX_STARTED)) {
        ret = device_i2s_start_transmitter(info->dev);
        if (ret) {
            gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
            gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
            return ret;
        }

        info->flags |= GB_I2S_FLAGS_RX_STARTED;
    }
#else
    mag_gb_rx_count++; /* XXX */

    ret = device_i2s_start_transmitter(info->dev);
    if (ret) {
        gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
        gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
        return;
    }

    info->flags |= GB_I2S_FLAGS_RX_STARTED;
#endif
}
#else
static uint8_t gb_i2s_receiver_send_data_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_send_data_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_info *info;
    int ret;

    info = gb_i2s_get_info_by_cport(operation->cport);
    if (!info || !(info->flags & GB_I2S_FLAGS_RX_PREPARED)) {
        gb_i2s_report_event(info, GB_I2S_EVENT_PROTOCOL_ERROR);
        return 0;
    }

    if (request->size != info->msg_data_size) {
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        return 0;
    }

#if 0 /* XXX send anyway */
    /* XXX Handle wrap around to 0 */
    if (request->sample_number < info->next_rx_sample) {
        gb_i2s_report_event(info, GB_I2S_EVENT_OUT_OF_SEQUENCE);
        return 0;
    }
#endif

    /* Fill in any missing data */
    while (ring_buf_is_producers(info->rx_rb) &&
           (request->sample_number > info->next_rx_sample)) {
lldbg("--- fill missing data - req #: 0x%x, info #: 0x%x\n", /* XXX */
		request->sample_number, info->next_rx_sample);
        gb_i2s_ll_tx(info, info->dummy_data);
    }

    if (!ring_buf_is_producers(info->rx_rb)) {
#if 0 /* XXX */
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
#else
        mag_gb_or++; /* XXX */
#endif
        return 0;
    }

    gb_i2s_ll_tx(info, request->data);

#ifdef GB_I2S_OLD_TX_METHOD
    if (!(info->flags & GB_I2S_FLAGS_RX_STARTED)) {
        ret = device_i2s_start_transmitter(info->dev);
        if (ret) {
            gb_i2s_report_event(info, GB_I2S_EVENT_FAILURE);
            gb_i2s_report_event(info, GB_I2S_EVENT_HALT);
            return ret;
        }

        info->flags |= GB_I2S_FLAGS_RX_STARTED;
    }
#else
    mag_gb_rx_count++; /* XXX */

    ret = device_i2s_start_transmitter(info->dev);
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
        device_i2s_stop_transmitter(info->dev);
        info->flags &= ~GB_I2S_FLAGS_RX_STARTED;
    }

    device_i2s_shutdown_transmitter(info->dev);

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
        request->sample_number = info->next_tx_sample;
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
        gb_i2s_report_event(info, GB_I2S_EVENT_UNDERRUN);
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        gb_i2s_report_event(info, GB_I2S_EVENT_OVERRUN);
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        gb_i2s_report_event(info, GB_I2S_EVENT_CLOCKING);
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        gb_i2s_report_event(info, GB_I2S_EVENT_DATA_LEN);
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        gb_i2s_report_event(info, GB_I2S_EVENT_UNSPECIFIED);
        break;
    default:
        gb_i2s_report_event(info, GB_I2S_EVENT_INTERNAL_ERROR);
        break;
    }

    if (gb_event)
        gb_i2s_report_event(info, gb_event);
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
    request->size = info->msg_data_size;

    ring_buf_set_priv(rb, operation);

    return 0;
}

static void gb_i2s_rb_free_gb_op(struct ring_buf *rb, void *arg)
{
    gb_operation_destroy(ring_buf_get_priv(rb));
}

static uint8_t gb_i2s_prepare_transmitter(struct gb_i2s_info *info)
{
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
    ret = device_i2s_prepare_receiver(info->dev, info->tx_rb,
                                          gb_i2s_ll_rx_cb, info);
    if (ret) {
        ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
        info->tx_rb = NULL;

        return GB_OP_MALFUNCTION;
    }

    info->flags |= GB_I2S_FLAGS_TX_PREPARED;

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_start_transmitter(struct gb_i2s_info *info)
{
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

    ret = device_i2s_start_receiver(info->dev);
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
    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_stop_transmitter(struct gb_i2s_info *info)
{
    irqstate_t flags;
    uint32_t i, limit;

    if (!(info->flags & GB_I2S_FLAGS_TX_STARTED))
        return GB_OP_PROTOCOL_BAD;

    device_i2s_stop_receiver(info->dev);

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

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_shutdown_transmitter(struct gb_i2s_info *info)
{
    if (!(info->flags & GB_I2S_FLAGS_TX_PREPARED) ||
        (info->flags & GB_I2S_FLAGS_TX_STARTED)) {

        return GB_OP_PROTOCOL_BAD;
    }

    device_i2s_shutdown_receiver(info->dev);

    ring_buf_free_ring(info->tx_rb, gb_i2s_rb_free_gb_op, info);
    info->tx_rb = NULL;

    info->flags &= ~GB_I2S_FLAGS_TX_PREPARED;

    return GB_OP_SUCCESS;
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
    const struct device_i2s_configuration *dev_device_cfgs;
    /* XXX
    const struct gb_i2s_configuration *gb_device_cfgs;
    */
    uint16_t device_cfg_count;
    size_t size;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    ret = device_i2s_get_supported_configurations(info->dev,
                                                      &device_cfg_count,
                                                      &dev_device_cfgs);
    if (ret)
        return GB_OP_MALFUNCTION;

    size = device_cfg_count * sizeof(*dev_device_cfgs);

    response = gb_operation_alloc_response(operation, sizeof(*response) + size);
    if (!response)
        return GB_OP_NO_MEMORY;

    response->config_count = device_cfg_count;
    memcpy(response->config, dev_device_cfgs, size);

    return GB_OP_SUCCESS;
}

static uint8_t gb_i2s_set_configuration_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_set_configuration_request *request =
                gb_operation_get_request_payload(operation);
    struct device_i2s_configuration dev_cfg;
    struct gb_i2s_info *info;
    int ret;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if ((info->active_tx_cports + info->active_rx_cports) > 0)
        return GB_OP_PROTOCOL_BAD;

    memcpy(&dev_cfg, &request->config, sizeof(dev_cfg));

    ret = device_i2s_set_configuration(info->dev, &dev_cfg);
    if (ret)
        return GB_OP_MALFUNCTION;

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

    info->samples_per_message = request->samples_per_message;

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

    ret = device_i2s_get_processing_delay(info->dev, &microseconds);
    if (ret)
        return GB_OP_MALFUNCTION;

    response->microseconds = microseconds + 0; /* XXX no delay for now */

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

    info->delay = request->microseconds;

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

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    if (!(info->flags & GB_I2S_FLAGS_CONFIGURED))
        return GB_OP_PROTOCOL_BAD;

    cple = gb_i2s_get_cple(info, request->cport);
    if (!cple)
        return GB_OP_INVALID;

    if (cple->state != GB_I2S_CPORT_STATE_INACTIVE)
        return GB_OP_PROTOCOL_BAD;

    if ((info->active_tx_cports + info->active_rx_cports) == 0) {
        info->msg_data_size = info->sample_size * info->samples_per_message;

        info->dummy_data = zalloc(info->msg_data_size);
        if (!info->dummy_data)
            return GB_OP_NO_MEMORY;

        allocated_dummy_data = 1;
    }

    switch (cple->type) {
    case GB_I2S_CPORT_TYPE_RECEIVER:
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
        ret = GB_OP_MALFUNCTION;
    }

    if ((ret != GB_OP_SUCCESS) && allocated_dummy_data) {
        free(info->dummy_data);
        info->dummy_data = NULL;
    }

    return ret;
}

static uint8_t gb_i2s_deactivate_cport_req_handler(
                                                struct gb_operation *operation)
{
    struct gb_i2s_deactivate_cport_request *request =
                gb_operation_get_request_payload(operation);
    struct gb_i2s_cport_list_entry *cple;
    struct gb_i2s_info *info;
    uint8_t ret = GB_OP_SUCCESS;

    info = gb_i2s_get_info(operation->cport);
    if (!info)
        return GB_OP_INVALID;

    cple = gb_i2s_get_cple(info, request->cport);
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

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
    if (!dev_info)
        return -EINVAL;

    if (dev_info->info)
        return -EBUSY;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

lldbg("GB i2s info struct: 0x%08p\n", info); /* XXX */

    info->mgmt_cport = cport;
    info->samples_per_message = GB_I2S_SAMPLES_PER_MSG_DEFAULT;
    info->tx_wdog = wd_create();

    list_init(&info->cport_list);
    sem_init(&info->tx_stop_sem, 0, 0);
    sem_init(&info->tx_rb_sem, 0, 0);
    atomic_init(&info->tx_rb_count, 0);

    ret = pthread_create(&info->tx_rb_thread, NULL, gb_i2s_tx_rb_thread,
                         info);
    if (ret) {
        ret = -ret;
        goto err_free_wd_info;
    }

    info->dev = device_open(dev_info->dev_class, dev_info->dev_id);
    if (!info->dev) {
        ret = -EIO;
        goto err_kill_pthread;
    }

    dev_info->info = info;

    return 0;

err_kill_pthread:
    pthread_kill(info->tx_rb_thread, SIGKILL);
err_free_wd_info:
    wd_delete(info->tx_wdog);
    free(info);

    return ret;
}

void gb_i2s_mgmt_exit(unsigned int cport)
{
    struct gb_i2s_dev_info *dev_info;

    dev_info = gb_i2s_get_dev_info(cport);
    if (!dev_info && !dev_info->info)
        return;

    device_close(dev_info->info->dev);

    pthread_kill(dev_info->info->tx_rb_thread, SIGKILL);
    wd_cancel(dev_info->info->tx_wdog);
    wd_delete(dev_info->info->tx_wdog);

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
lldbg("gb_i2s_mgmt_register: cport %d\n", cport);
    gb_register_driver(cport, &i2s_mgmt_driver);
}

static int gb_i2s_cple_init(unsigned int cport, enum gb_i2s_cport_type type)
{
    struct gb_i2s_dev_info *dev_info;
    struct gb_i2s_cport_list_entry *cple;

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
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

    dev_info = gb_i2s_get_dev_info(GB_I2S_BUNDLE_0_ID); /* XXX */
    if (!dev_info || !dev_info->info)
        return -EINVAL;

    /* Can have only one Receiver CPort in a bundle */
    list_foreach(&dev_info->info->cport_list, iter) {
        cple = list_entry(iter, struct gb_i2s_cport_list_entry, list);

        if (cple->type == GB_I2S_CPORT_TYPE_RECEIVER)
            return -EINVAL;
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
#ifdef GB_I2S_FAST_HANDLER
    GB_FAST_HANDLER(GB_I2S_DATA_TYPE_SEND_DATA,
                    gb_i2s_receiver_send_data_req_handler),
#else
    GB_HANDLER(GB_I2S_DATA_TYPE_SEND_DATA,
               gb_i2s_receiver_send_data_req_handler),
#endif
};

struct gb_driver i2s_receiver_driver = {
    .init               = gb_i2s_receiver_init,
    .exit               = gb_i2s_receiver_exit,
    .op_handlers        = gb_i2s_receiver_handlers,
    .op_handlers_count  = ARRAY_SIZE(gb_i2s_receiver_handlers),
};

void gb_i2s_receiver_register(int cport)
{
lldbg("gb_i2s_receiver_register: cport %d\n", cport);
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
lldbg("gb_i2s_transmitter_register: cport %d\n", cport);
    gb_register_driver(cport, &i2s_transmitter_driver);
}

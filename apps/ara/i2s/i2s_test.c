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

#ifndef SIGKILL
#define SIGKILL     9
#endif

#ifndef SIGTERM
#define SIGTERM     15
#endif

#undef I2S_TEST_CHECK_RX_DATA

#define SAMPLES_PER_ENTRY   100

#define RB_ENTRIES          64
#define RB_HEADROOM         0
#define RB_TAILROOM         0

static struct {
    unsigned int    tx_cnt;
    unsigned int    tx_err_cnt;
    unsigned int    rx_cnt;
    unsigned int    rx_err_cnt;
    unsigned int    rx_bad_data_cnt;
} i2s_test_stats;

static sem_t    done_sem;

struct i2s_sample {
    uint16_t    left;
    uint16_t    right;
};

static void print_usage(char *argv[])
{
    printf("Usage: %s <-m|-s> <-r|-t> <-i|-l>\n", argv[0]);
    printf("\t-m    be mclk, bclk, lrclk master)\n");
    printf("\t-s    be mclk, bclk, lrclk slave)\n");
    printf("\t-r    receive i2s data\n");
    printf("\t-t    transmit i2s data\n");
    printf("\t-i    use I2S protocol\n");
    printf("\t-l    use LR Stereo protocol\n");
}

static int parse_cmdline(int argc, char *argv[], unsigned char *is_master,
                         unsigned char *is_transmitter, unsigned char *is_i2s)
{
    int mcnt, scnt, rcnt, tcnt, icnt, lcnt, errcnt;
    int option;

    mcnt = scnt = rcnt = tcnt = icnt = lcnt = errcnt = 0;

    *is_master = 0;
    *is_transmitter = 0;
    *is_i2s = 0;

    if (argc != 4)
        return -EINVAL;

    while ((option = getopt(argc, argv, "msrtil")) != ERROR) {
        switch(option) {
        case 'm':
            *is_master = 1;
            mcnt++;
            break;
        case 's':
            *is_master = 0;
            scnt++;
            break;
        case 'r':
            *is_transmitter = 0;
            rcnt++;
            break;
        case 't':
            *is_transmitter = 1;
            tcnt++;
            break;
        case 'i':
            *is_i2s = 1;
            icnt++;
            break;
        case 'l':
            *is_i2s = 0;
            lcnt++;
            break;
        default:
            errcnt++;
        }
    }

    if (((mcnt + scnt) != 1) || ((rcnt + tcnt) != 1) || ((icnt + lcnt) != 1) ||
        errcnt) {

        return -EINVAL;
    }

    return 0;
}

static int rb_fill_and_pass(struct ring_buf *rb, void *arg)
{
    struct i2s_sample *sample;
    unsigned int i;
    static unsigned int cnt = 0;

    ring_buf_reset(rb);
    sample = ring_buf_get_tail(rb);

    for (i = 0; i < (ring_buf_space(rb) / sizeof(*sample)); i++) {
        sample[i].left = cnt & 0x0000ffff;
        sample[i].right = i & 0x0000ffff;
    }

    ring_buf_put(rb, i * sizeof(*sample));
    ring_buf_pass(rb);

    cnt++;

    return 0;
}

static void i2s_tx_callback(struct ring_buf *rb, enum device_i2s_event event,
                            void *arg)
{
    if (event != DEVICE_I2S_EVENT_TX_COMPLETE)
        i2s_test_stats.tx_err_cnt++;

    /* Don't use printf since its called from irq context */
    switch (event) {
    case DEVICE_I2S_EVENT_TX_COMPLETE:
        i2s_test_stats.tx_cnt++;
        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNDERRUN\n");
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_OVERRUN\n");
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_CLOCKING\n");
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_DATA_LEN\n");
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNSPECIFIED\n");
        break;
    default:
        lldbg("CB EVENT: Unknown\n");
    }

    if (ring_buf_is_producers(rb))
        rb_fill_and_pass(rb, NULL);
}

static int start_transmitter(struct device *dev)
{
    struct ring_buf *rb;
    int ret;

    /* 16-bit stereo */
    rb = ring_buf_alloc_ring(RB_ENTRIES, RB_HEADROOM,
                                 SAMPLES_PER_ENTRY * sizeof(struct i2s_sample),
                                 RB_TAILROOM, rb_fill_and_pass, NULL, NULL);
    if (!rb) {
        fprintf(stderr, "ring_buf_alloc_ring failed\n");
        return -EIO;
    }

    ret = device_i2s_prepare_transmitter(dev, rb, i2s_tx_callback, NULL);
    if (ret) {
        fprintf(stderr, "prepare_tx failed\n");
        goto err_free_ring;
    }

    ret = device_i2s_start_transmitter(dev);
    if (ret) {
        fprintf(stderr, "start_tx failed\n");
        goto err_shutdown_transmitter;
    }

    return 0;

err_shutdown_transmitter:
    device_i2s_shutdown_transmitter(dev);
err_free_ring:
    ring_buf_free_ring(rb, NULL, NULL);

    return ret;
}

static void stop_transmitter(struct device *dev)
{
    int ret;

    ret = device_i2s_stop_transmitter(dev);
    if (ret)
        fprintf(stderr, "stop_tx failed\n");

    ret = device_i2s_shutdown_transmitter(dev);
    if (ret)
        fprintf(stderr, "shutdown_tx failed\n");
}

#ifdef I2S_TEST_CHECK_RX_DATA
static void i2s_rx_check_data(struct ring_buf *rb)
{
    struct i2s_sample *sample;
    unsigned int i;

    sample = ring_buf_get_head(rb);

    for (i = 0; i < (ring_buf_len(rb) / sizeof(*sample)); i++) {
        if ((sample[i].left || sample[i].right) &&
            ((sample[i].left != 0xdead) || (sample[i].right != 0xbeef))) {

            i2s_test_stats.rx_bad_data_cnt++;
            lldbg("*** BAD DATA 0x%04x%04x***\n",
                  sample[i].left, sample[i].right);
        }
    }
}
#endif

static void i2s_rx_callback(struct ring_buf *rb,
                               enum device_i2s_event event,
                               void *arg)
{
    if (event != DEVICE_I2S_EVENT_RX_COMPLETE)
        i2s_test_stats.rx_err_cnt++;

    /* Don't use printf since its called from irq context */
    switch (event) {
    case DEVICE_I2S_EVENT_RX_COMPLETE:
        i2s_test_stats.rx_cnt++;
#ifdef I2S_TEST_CHECK_RX_DATA
        i2s_rx_check_data(rb);
#endif
        break;
    case DEVICE_I2S_EVENT_UNDERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNDERRUN\n");
        break;
    case DEVICE_I2S_EVENT_OVERRUN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_OVERRUN\n");
        break;
    case DEVICE_I2S_EVENT_CLOCKING:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_CLOCKING\n");
        break;
    case DEVICE_I2S_EVENT_DATA_LEN:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_DATA_LEN\n");
        break;
    case DEVICE_I2S_EVENT_UNSPECIFIED:
        lldbg("CB EVENT: DEVICE_I2S_EVENT_UNSPECIFIED\n");
        break;
    default:
        lldbg("CB EVENT: Unknown\n");
    }

    if (ring_buf_is_consumers(rb)) {
        ring_buf_reset(rb);
        ring_buf_pass(rb);
    }
}

static int start_receiver(struct device *dev)
{
    struct ring_buf *rb;
    int ret;

    /* 16-bit stereo */
    rb = ring_buf_alloc_ring(RB_ENTRIES, RB_HEADROOM,
                                 SAMPLES_PER_ENTRY * sizeof(struct i2s_sample),
                                 RB_TAILROOM, NULL, NULL, NULL);
    if (!rb) {
        fprintf(stderr, "ring_buf_alloc_ring failed\n");
        return -EIO;
    }

    ret = device_i2s_prepare_receiver(dev, rb, i2s_rx_callback, NULL);
    if (ret) {
        fprintf(stderr, "prepare_rx failed\n");
        goto err_free_ring;
    }

    ret = device_i2s_start_receiver(dev);
    if (ret) {
        fprintf(stderr, "start_rx failed\n");
        goto err_shutdown_receiver;
    }

    return 0;

err_shutdown_receiver:
    device_i2s_shutdown_receiver(dev);
err_free_ring:
    ring_buf_free_ring(rb, NULL, NULL);

    return ret;
}

static void stop_receiver(struct device *dev)
{
    int ret;

    ret = device_i2s_stop_receiver(dev);
    if (ret)
        fprintf(stderr, "stop_rx failed\n");

    ret = device_i2s_shutdown_receiver(dev);
    if (ret)
        fprintf(stderr, "shutdown_rx failed\n");
}

static int start_streaming(unsigned char is_master,
                           unsigned char is_transmitter,
                           unsigned char is_i2s)
{
    struct device *dev;
    uint16_t config_count;
    const struct device_i2s_configuration *configs, *cfg;
    struct device_i2s_configuration config;
    unsigned int i;
    int ret;

    dev = device_open(DEVICE_CLASS_I2S_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_i2s_get_supported_configurations(dev, &config_count, &configs);
    if (ret) {
        fprintf(stderr, "get_supported_configs failed\n");
        goto err_dev_close;
    }

    /* Pick 48KHz 16-bits/channel */
    for (i = 0, cfg = configs; i < config_count; i++, cfg++) {
        if ((cfg->sample_frequency == 48000) &&
            (cfg->num_channels == 2) &&
            (cfg->bytes_per_channel == 2) &&
            (cfg->byte_order & DEVICE_I2S_BYTE_ORDER_LE) &&
            (cfg->spatial_locations ==
                 (DEVICE_I2S_SPATIAL_LOCATION_FL |
                  DEVICE_I2S_SPATIAL_LOCATION_FR)) &&
            ((is_i2s && (cfg->ll_protocol & DEVICE_I2S_PROTOCOL_I2S)) ||
             (!is_i2s && (cfg->ll_protocol & DEVICE_I2S_PROTOCOL_LR_STEREO))) &&
            ((is_master && (cfg->ll_bclk_role & DEVICE_I2S_ROLE_MASTER) &&
              (cfg->ll_wclk_role & DEVICE_I2S_ROLE_MASTER)) ||
             (!is_master && (cfg->ll_bclk_role & DEVICE_I2S_ROLE_SLAVE) &&
              (cfg->ll_wclk_role & DEVICE_I2S_ROLE_SLAVE))) &&
            (cfg->ll_wclk_polarity & DEVICE_I2S_POLARITY_NORMAL) &&
            (cfg->ll_wclk_change_edge & DEVICE_I2S_EDGE_FALLING) &&
            (cfg->ll_wclk_tx_edge & DEVICE_I2S_EDGE_FALLING) &&
            (cfg->ll_wclk_rx_edge & DEVICE_I2S_EDGE_RISING) &&
            ((is_i2s && (cfg->ll_data_offset == 1)) ||
             (!is_i2s && (cfg->ll_data_offset == 0)))) {

            break;
        }
    }

    if (i >= config_count) {
        fprintf(stderr, "no valid configuration\n");
        ret = -EINVAL;
        goto err_dev_close;
    }

    memcpy(&config, cfg, sizeof(config));

    config.byte_order = DEVICE_I2S_BYTE_ORDER_LE;

    if (is_i2s)
        config.ll_protocol = DEVICE_I2S_PROTOCOL_I2S;
    else
        config.ll_protocol = DEVICE_I2S_PROTOCOL_LR_STEREO;

    if (is_master) {
        config.ll_bclk_role = DEVICE_I2S_ROLE_MASTER;
        config.ll_wclk_role = DEVICE_I2S_ROLE_MASTER;
    } else {
        config.ll_bclk_role = DEVICE_I2S_ROLE_SLAVE;
        config.ll_wclk_role = DEVICE_I2S_ROLE_SLAVE;
    }

    config.ll_wclk_polarity = DEVICE_I2S_POLARITY_NORMAL;
    config.ll_wclk_change_edge = DEVICE_I2S_EDGE_RISING;
    config.ll_wclk_tx_edge = DEVICE_I2S_EDGE_FALLING;
    config.ll_wclk_rx_edge = DEVICE_I2S_EDGE_RISING;

    ret = device_i2s_set_configuration(dev, &config);
    if (ret) {
        fprintf(stderr, "set_configs failed\n");
        goto err_dev_close;
    }

    if (is_transmitter)
        ret = start_transmitter(dev);
    else
        ret = start_receiver(dev);

    if (ret)
        goto err_dev_close;

    /*
     * Wait forever.  Can't just exit because callback is still being
     * called by driver to keep filling/draining the ring buffer.
     */
    while (sem_wait(&done_sem) && (errno == EINTR));

    if (is_transmitter)
        stop_transmitter(dev);
    else
        stop_receiver(dev);

    device_close(dev);

    return 0;

err_dev_close:
    device_close(dev);

    return ret;
}

static void sig_handler(int sig)
{
    errno = 0;
    sem_post(&done_sem);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int i2s_test_main(int argc, char *argv[])
#endif
{
    unsigned char is_master, is_transmitter, is_i2s;
    struct sigaction sig_act;
    int ret;

    ret = parse_cmdline(argc, argv, &is_master, &is_transmitter, &is_i2s);
    if (ret) {
        print_usage(argv);
        return 1;
    }

    memset(&i2s_test_stats, 0, sizeof(i2s_test_stats));

    sem_init(&done_sem, 0, 0);

    sig_act.sa_handler = sig_handler;
    sig_act.sa_flags = 0;

    ret = sigaction(SIGKILL, &sig_act, NULL);
    if (ret != OK) {
        fprintf(stderr, "SIGKILL sigaction failed: %d - %s\n", errno,
                strerror(errno));
        return 1;
    }

    ret = sigaction(SIGTERM, &sig_act, NULL);
    if (ret != OK) {
        fprintf(stderr, "SIGTERM sigaction failed: %d - %s\n", errno,
                strerror(errno));
        return 1;
    }

    ret = start_streaming(is_master, is_transmitter, is_i2s);
    if (ret)
        return 1;

    return 0;
}

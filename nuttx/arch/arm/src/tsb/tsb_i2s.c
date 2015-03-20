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
 * @brief TSB I2S device driver
 */
/*
 * Clocks:
 *  MCLK    - Master Clock: used to drive the other clocks.
 *  BCLK    - Bit Clock: used for clocking each bit in/out.
 *            Also referred to as SCLK (Serial Clock).
 *  WCLK    - Word Clock: determines which channel audio data is for
 *            (i.e., left or right channel).  Also referred to as
 *            LRCLK (Left-Right Clock).
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <nuttx/lib.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/device.h>
#include <nuttx/device_i2s.h>
#include <nuttx/audio/i2s.h>

#include "up_arch.h"
#include "tsb_scm.h"
#include "tsb_i2s.h"

#define TSB_I2S_ENABLE_SLAVE_MODE   /* XXX - slave mode broken right now */
                                    /* change mclk_role in device_table.c */

#define TSB_I2S_DRIVER_NAME                             "tsb i2s driver"

#if 0 /* XXX */
#define TSB_I2S_TX_START_THRESHOLD                      0x20
#else /* Buffer up as much as possible before starting to reduce underruns */
#define TSB_I2S_TX_START_THRESHOLD                      0x3f
#endif

/* System Controller/Bridge Registers */
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR                0x0440
#define TSB_CG_BRIDGE_PLLA_INIT                         0x0900
#define TSB_CG_BRIDGE_PLLA_DIVIDE_MODE                  0x0904
#define TSB_CG_BRIDGE_PLLA_CLK_FACTOR                   0x0908
#define TSB_CG_BRIDGE_PLLA_CLKFREQ_MODE                 0x090c
#define TSB_CG_BRIDGE_PLLA_SSCG_MODE                    0x0910
#define TSB_CG_BRIDGE_PLLA_SSCG_PARAM                   0x0914

#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL   BIT(2)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL    BIT(1)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL BIT(0)
#define TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASK                                 \
                        (TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL      | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL       | \
                         TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL)

/* I2S Registers */
#define TSB_I2S_REG_START                               0x0004
#define TSB_I2S_REG_BUSY                                0x0008
#define TSB_I2S_REG_STOP                                0x000c
#define TSB_I2S_REG_AUDIOSET                            0x0010
#define TSB_I2S_REG_INTSTAT                             0x0014
#define TSB_I2S_REG_INTMASK                             0x0018
#define TSB_I2S_REG_INTCLR                              0x001c
#define TSB_I2S_REG_MUTE                                0x0024
#define TSB_I2S_REG_EPTR                                0x0028
#define TSB_I2S_REG_TX_SSIZE                            0x0030
#define TSB_I2S_REG_REGBUSY                             0x0040
#define TSB_I2S_REG_MODESET                             0x00f8
#define TSB_I2S_REG_LMEM00                              0x0100

#define TSB_I2S_REG_START_START                         BIT(8)
#define TSB_I2S_REG_START_SPK_MIC_START                 BIT(0)

#define TSB_I2S_REG_BUSY_LRERRBUSY                      BIT(17)
#define TSB_I2S_REG_BUSY_ERRBUSY                        BIT(16)
#define TSB_I2S_REG_BUSY_BUSY                           BIT(8)
#define TSB_I2S_REG_BUSY_SERIBUSY                       BIT(1)
#define TSB_I2S_REG_BUSY_SPK_MIC_BUSY                   BIT(0)

#define TSB_I2S_REG_STOP_I2S_STOP                       BIT(0)

#define TSB_I2S_REG_AUDIOSET_DTFMT                      BIT(16)
#define TSB_I2S_REG_AUDIOSET_SDEDGE                     BIT(12)
#define TSB_I2S_REG_AUDIOSET_EDGE                       BIT(11)
#define TSB_I2S_REG_AUDIOSET_SCLKTOWS                   BIT(8)
#define TSB_I2S_REG_AUDIOSET_WORDLEN_MASK               0x3f
#define TSB_I2S_REG_AUDIOSET_MASK           (TSB_I2S_REG_AUDIOSET_DTFMT     | \
                                             TSB_I2S_REG_AUDIOSET_SDEDGE    | \
                                             TSB_I2S_REG_AUDIOSET_EDGE      | \
                                             TSB_I2S_REG_AUDIOSET_SCLKTOWS  | \
                                             TSB_I2S_REG_AUDIOSET_WORDLEN_MASK)

#define TSB_I2S_REG_INT_DMACMSK                         BIT(16)
#define TSB_I2S_REG_INT_LRCK                            BIT(3)
#define TSB_I2S_REG_INT_UR                              BIT(2)
#define TSB_I2S_REG_INT_OR                              BIT(1)
#define TSB_I2S_REG_INT_INT                             BIT(0)
#define TSB_I2S_REG_INT_ERROR_MASK          (TSB_I2S_REG_INT_LRCK   | \
                                             TSB_I2S_REG_INT_UR     | \
                                             TSB_I2S_REG_INT_OR)

#define TSB_I2S_REG_MUTE_MUTEN                          BIT(0)

#define TSB_I2S_REG_EPTR_ERRPOINTER_GET(a)              ((a) & 0x3f)
#define TSB_I2S_REG_EPTR_ERRPOINTER_SET(a)              ((a) & 0x3f)

#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_GET(r)         ((r) & 0x3f)
#define TSB_I2S_REG_TX_SSIZE_TXSTARTSIZE_SET(v)         ((v) & 0x3f)

#define TSB_I2S_REG_REGBUSY_MODESETPEND                 BIT(19)
#define TSB_I2S_REG_REGBUSY_TXSSIZEPEND                 BIT(18)
#define TSB_I2S_REG_REGBUSY_MUTEPEND                    BIT(17)
#define TSB_I2S_REG_REGBUSY_AUDIOSETPEND                BIT(16)
#define TSB_I2S_REG_REGBUSY_MODESETBUSY                 BIT(3)
#define TSB_I2S_REG_REGBUSY_TXSSIZEBUSY                 BIT(2)
#define TSB_I2S_REG_REGBUSY_MUTEBUSY                    BIT(1)
#define TSB_I2S_REG_REGBUSY_AUDIOSETBUSY                BIT(0)

#define TSB_I2S_REG_MODESET_I2S_STEREO                  0x0
#define TSB_I2S_REG_MODESET_LR_STEREO                   0x2
#define TSB_I2S_REG_MODESET_LR_STEREO_REV_POL           0x3
#define TSB_I2S_REG_MODESET_PCM_MONO                    0x4
#define TSB_I2S_REG_MODESET_PCM_MONO_REV_POL            0x5
#define TSB_I2S_REG_MODESET_WS_MASK                     0x7

#define TSB_I2S_FLAG_OPEN                               BIT(0)
#define TSB_I2S_FLAG_CONFIGURED                         BIT(1)
#define TSB_I2S_FLAG_ENABLED                            BIT(2)
#define TSB_I2S_FLAG_RX_PREPARED                        BIT(3)
#define TSB_I2S_FLAG_RX_ACTIVE                          BIT(4)
#define TSB_I2S_FLAG_TX_PREPARED                        BIT(5)
#define TSB_I2S_FLAG_TX_ACTIVE                          BIT(6)

enum tsb_i2s_block {
    TSB_I2S_BLOCK_INVALID,
    TSB_I2S_BLOCK_BRIDGE,
    TSB_I2S_BLOCK_SC,
    TSB_I2S_BLOCK_SO,
    TSB_I2S_BLOCK_SI,
};

struct tsb_i2s_info {
    struct device                   *dev;
    uint32_t                        flags;
    struct ring_buf                 *rx_rb;
    void                            (*rx_callback)(struct ring_buf *rb,
                                                   enum device_i2s_event event,
                                                   void *arg);
    void                            *rx_arg;
    struct ring_buf                 *tx_rb;
    void                            (*tx_callback)(struct ring_buf *rb,
                                                   enum device_i2s_event event,
                                                   void *arg);
    void                            *tx_arg;
    uint32_t                        cg_base;
    uint32_t                        sc_base;
    uint32_t                        so_base;
    uint32_t                        si_base;
    int                             soerr_irq;
    int                             so_irq;
    int                             sierr_irq;
    int                             si_irq;
    enum tsb_i2s_clk_role           mclk_role;
    uint32_t                        mclk_freq;
    struct device_i2s_configuration config;

    /* XXX for testing only -- remove */
    uint32_t                        so_int;
    uint32_t                        so_err;
    uint32_t                        si_int;
    uint32_t                        si_err;
    uint32_t                        rx_count;
    uint32_t                        tx_count;
};

static void tsb_i2s_stop_receiver(struct tsb_i2s_info *info);
static void tsb_i2s_stop_transmitter(struct tsb_i2s_info *info);

/*
 * Only valid for internally generated MCLK.  Table changes with external MCLK.
 *
 * XXX BCLK & WCLK are either both internal or both external so need
 * separate entries for each set.  IOW, these entries need to be
 * split in two.
 */
static const struct device_i2s_configuration tsb_i2s_config_table[] = {
    {
        .sample_frequency       = 192000,
        .num_channels           = 1,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FC,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_PCM,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 96000,
        .num_channels           = 1,
        .bytes_per_channel      = 4,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FC,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_PCM,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 96000,
        .num_channels           = 1,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FC,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_PCM,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 96000,
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
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING,
        .ll_data_offset         = 1,
    },
    {
        .sample_frequency       = 96000,
        .num_channels           = 2,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_LR_STEREO,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 48000,
        .num_channels           = 1,
        .bytes_per_channel      = 4,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FC,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_PCM,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 48000,
        .num_channels           = 2,
        .bytes_per_channel      = 4,
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
    {
        .sample_frequency       = 48000,
        .num_channels           = 2,
        .bytes_per_channel      = 4,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_LR_STEREO,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
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
    {
        .sample_frequency       = 48000,
        .num_channels           = 2,
        .bytes_per_channel      = 2,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_LR_STEREO,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
    {
        .sample_frequency       = 24000,
        .num_channels           = 2,
        .bytes_per_channel      = 4,
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
    {
        .sample_frequency       = 24000,
        .num_channels           = 2,
        .bytes_per_channel      = 4,
        .byte_order             = DEVICE_I2S_BYTE_ORDER_BE |
                                  DEVICE_I2S_BYTE_ORDER_LE,
        .spatial_locations      = DEVICE_I2S_SPATIAL_LOCATION_FL |
                                  DEVICE_I2S_SPATIAL_LOCATION_FR,
        .ll_protocol            = DEVICE_I2S_PROTOCOL_LR_STEREO,
        .ll_bclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_role           = DEVICE_I2S_ROLE_MASTER |
                                  DEVICE_I2S_ROLE_SLAVE,
        .ll_wclk_polarity       = DEVICE_I2S_POLARITY_NORMAL |
                                  DEVICE_I2S_POLARITY_REVERSED,
        .ll_wclk_change_edge    = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_tx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_wclk_rx_edge        = DEVICE_I2S_EDGE_RISING |
                                  DEVICE_I2S_EDGE_FALLING,
        .ll_data_offset         = 0,
    },
};

/*
 * The stupid nuttx IRQ handler interface doesn't provide an 'arg' parameter
 * where 'dev' can be passed so use a global for now.  Must be fixed to
 * support more than one tsb i2s controller.
 */
static struct device *saved_dev;

static int tsb_i2s_one_bit_is_set(uint64_t bits)
{
    return bits && !(bits & (bits - 1));
}

static int tsb_i2s_config_is_valid(struct device_i2s_configuration
                                                                 *configuration)
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

    for (i = 0, config = tsb_i2s_config_table;
         i < ARRAY_SIZE(tsb_i2s_config_table);
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

/* XXX Put these in a common file somewhere */
static uint16_t swap16(uint16_t h)
{
    return ((h & 0x00ff) << 8) | ((h & 0xff00) >> 8);
}

static uint32_t swap32(uint32_t w)
{
    return ((w & 0xff000000) >> 24) | ((w & 0x00ff0000) >>  8) |
           ((w & 0x0000ff00) <<  8) | ((w & 0x000000ff) << 24);
}

static int tsb_i2s_device_is_open(struct tsb_i2s_info *info)
{
    return !!(info->flags & TSB_I2S_FLAG_OPEN);
}

static int tsb_i2s_device_is_configured(struct tsb_i2s_info *info)
{
    return tsb_i2s_device_is_open(info) &&
           (info->flags & TSB_I2S_FLAG_CONFIGURED);
}

static int tsb_i2s_device_is_enabled(struct tsb_i2s_info *info)
{
    return tsb_i2s_device_is_configured(info) &&
           (info->flags & TSB_I2S_FLAG_ENABLED);
}

static int tsb_i2s_rx_is_prepared(struct tsb_i2s_info *info)
{
    return tsb_i2s_device_is_enabled(info) &&
           (info->flags & TSB_I2S_FLAG_RX_PREPARED);
}

static int tsb_i2s_tx_is_prepared(struct tsb_i2s_info *info)
{
    return tsb_i2s_device_is_enabled(info) &&
           (info->flags & TSB_I2S_FLAG_TX_PREPARED);
}

static int tsb_i2s_rx_is_active(struct tsb_i2s_info *info)
{
    return tsb_i2s_rx_is_prepared(info) &&
           (info->flags & TSB_I2S_FLAG_RX_ACTIVE);
}

static int tsb_i2s_tx_is_active(struct tsb_i2s_info *info)
{
    return tsb_i2s_tx_is_prepared(info) &&
           (info->flags & TSB_I2S_FLAG_TX_ACTIVE);
}

static uint32_t tsb_i2s_get_block_base(struct tsb_i2s_info *info,
                                       enum tsb_i2s_block block)
{
    uint32_t base;

    switch (block) {
    case TSB_I2S_BLOCK_BRIDGE:
        base = info->cg_base;
        break;
    case TSB_I2S_BLOCK_SC:
        base = info->sc_base;
        break;
    case TSB_I2S_BLOCK_SO:
        base = info->so_base;
        break;
    case TSB_I2S_BLOCK_SI:
        base = info->si_base;
        break;
    default:
        lldbg("Bogus block: %d\n", block);
        base = 0;
    }

    return base;
}

static uint32_t tsb_i2s_read(struct tsb_i2s_info *info,
                             enum tsb_i2s_block block, unsigned int reg)
{
    uint32_t base;

    base = tsb_i2s_get_block_base(info, block);
    if (!base)
        return 0;

    return getreg32(base + reg);
}

static void tsb_i2s_write(struct tsb_i2s_info *info, enum tsb_i2s_block block,
                          unsigned int reg, uint32_t val)
{
    uint32_t base;

    base = tsb_i2s_get_block_base(info, block);
    if (!base)
        return;

    putreg32(val, base + reg);
}

static void tsb_i2s_write_field(struct tsb_i2s_info *info,
                                enum tsb_i2s_block block, unsigned int reg,
                                uint32_t mask, uint32_t val)
{
    uint32_t v;

    v = tsb_i2s_read(info, block, reg);
    v &= ~mask;
    v |= (val & mask);
    tsb_i2s_write(info, block, reg, v);
}

static void tsb_i2s_mask_irqs(struct tsb_i2s_info *info,
                              enum tsb_i2s_block block, uint32_t mask)
{
    uint32_t v;

    v = tsb_i2s_read(info, block, TSB_I2S_REG_INTMASK);
    tsb_i2s_write(info, block, TSB_I2S_REG_INTMASK, v | mask);
}

static void tsb_i2s_unmask_irqs(struct tsb_i2s_info *info,
                                enum tsb_i2s_block block, uint32_t mask)
{
    uint32_t v;

    v = tsb_i2s_read(info, block, TSB_I2S_REG_INTMASK);
    tsb_i2s_write(info, block, TSB_I2S_REG_INTMASK, v & ~mask);
}

static void tsb_i2s_clear_irqs(struct tsb_i2s_info *info,
                               enum tsb_i2s_block block, uint32_t mask)
{
    tsb_i2s_write(info, block, TSB_I2S_REG_INTCLR, mask);
}

/* XXX Move PLL set up to separate file */
static void tsb_i2s_config_plla(struct tsb_i2s_info *info,
                                unsigned int plla_freq)
{
    /* XXX
     * Right now, only 12.288MHz is supported.
     * The delays and 'nop' are there because of Toshiba's app note and code.
     */
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_INIT, 0x7);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_DIVIDE_MODE,
                  0x01000001);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_CLK_FACTOR,
                  0x20);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_CLKFREQ_MODE,
                  0x02071c31);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_SSCG_MODE,
                  0x4);
    /* XXX Toshiba code writes 0 even thouigh manual says 0 is invalid */
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_SSCG_PARAM,
                  0x0);
    up_udelay(100);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_INIT, 0x3);
    __asm("nop");
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_INIT, 0x2);
    up_udelay(100);
    tsb_i2s_write(info, TSB_I2S_BLOCK_BRIDGE, TSB_CG_BRIDGE_PLLA_INIT, 0x0);
}

static void tsb_i2s_init_block(struct tsb_i2s_info *info,
                               enum tsb_i2s_block block)
{
    tsb_i2s_write(info, block, TSB_I2S_REG_TX_SSIZE,
                  TSB_I2S_TX_START_THRESHOLD);

    tsb_i2s_mask_irqs(info, block,
                      TSB_I2S_REG_INT_DMACMSK | TSB_I2S_REG_INT_LRCK |
                      TSB_I2S_REG_INT_UR | TSB_I2S_REG_INT_OR |
                      TSB_I2S_REG_INT_INT);

    tsb_i2s_clear_irqs(info, block,
                       TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
                       TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT);
}

static void tsb_i2s_mute(struct tsb_i2s_info *info,
                             enum tsb_i2s_block block, int enable)
{
    tsb_i2s_write_field(info, block, TSB_I2S_REG_MUTE, TSB_I2S_REG_MUTE_MUTEN,
                        enable ? 0 : TSB_I2S_REG_MUTE_MUTEN);
}

static int tsb_i2s_config_hw(struct tsb_i2s_info *info)
{
    struct device_i2s_configuration *config = &info->config;
    uint32_t i2s_clk_sel, sc_audioset, so_audioset, si_audioset, modeset;

    i2s_clk_sel = 0;
    sc_audioset = 0;
    so_audioset = 0;
    si_audioset = 0;
    modeset = 0;

    /* config->num_channels ignored since its defined by ll_protocol */
    /* config->spatial_locations ignored since its defined by ll_protocol */
    /* config->ll_data_offset ignored since its defined by ll_protocol */

    /*
     * The master clock (MCLK) may use an internal or external source.
     * When internal, it uses output from the PLL; when external, it
     * uses the I2S_MCLK pin.  This is a hardware configuration option
     * and the driver is told which by init_data in the device_table[].
     *
     * XXX Slave mode can change tsb_i2s_config_table[] so need to create
     * dynamically or punt by putting the table in its own file which is
     * initialized by the h/w designer.
     */
    if (info->mclk_role != TSB_I2S_CLK_ROLE_MASTER)
        i2s_clk_sel |= TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASTER_CLOCK_SEL;

    /*
     * BCLK/SCLK and WCLK/LRCLK may be masters or slaves.  When masters,
     * they use either the external MCLK or the output from the PLL
     * (specified by info->mclk_role); when slaves, they use the signal
     * on the I2S_ABCKI and I2S_ALRCKI pins.  This is specified in the
     * configuration data and both BCLK and WCLK roles must be the same
     * (verified in tsb_i2s_config_is_valid()).
     */
    if (config->ll_bclk_role == DEVICE_I2S_ROLE_SLAVE)
        i2s_clk_sel |= TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_LR_BCLK_SEL;

    if (config->bytes_per_channel > 2)
        sc_audioset |= TSB_I2S_REG_AUDIOSET_SCLKTOWS;

    if (config->ll_wclk_change_edge == DEVICE_I2S_EDGE_RISING)
        sc_audioset |= TSB_I2S_REG_AUDIOSET_EDGE;

    if (config->ll_wclk_tx_edge == DEVICE_I2S_EDGE_RISING)
        so_audioset |= TSB_I2S_REG_AUDIOSET_SDEDGE;

    if (config->ll_wclk_rx_edge == DEVICE_I2S_EDGE_RISING)
        si_audioset |= TSB_I2S_REG_AUDIOSET_SDEDGE;

    so_audioset |= (config->bytes_per_channel * 8);
    si_audioset |= (config->bytes_per_channel * 8);

    switch (config->ll_protocol) {
    case DEVICE_I2S_PROTOCOL_PCM:
        if (config->ll_wclk_polarity == DEVICE_I2S_POLARITY_REVERSED)
            modeset |= TSB_I2S_REG_MODESET_PCM_MONO_REV_POL;
        else
            modeset |= TSB_I2S_REG_MODESET_PCM_MONO;
        break;
    case DEVICE_I2S_PROTOCOL_I2S:
        modeset |= TSB_I2S_REG_MODESET_I2S_STEREO;
        break;
    case DEVICE_I2S_PROTOCOL_LR_STEREO:
        if (config->ll_wclk_polarity == DEVICE_I2S_POLARITY_REVERSED)
            modeset |= TSB_I2S_REG_MODESET_LR_STEREO;
        else
            modeset |= TSB_I2S_REG_MODESET_LR_STEREO_REV_POL;
        break;
    default:
        return -EINVAL; /* config already checked so should never happen */
    }

    switch (config->sample_frequency) {
    case 192000:
        break;
    case 96000:
        if ((config->ll_protocol == DEVICE_I2S_PROTOCOL_PCM) &&
             (config->bytes_per_channel <= 2)) {

            i2s_clk_sel |= TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL;
        }
        break;
    case 48000:
        if (((config->ll_protocol == DEVICE_I2S_PROTOCOL_PCM) &&
             (config->bytes_per_channel > 2)) ||
            ((config->ll_protocol != DEVICE_I2S_PROTOCOL_PCM) &&
             (config->bytes_per_channel <= 2))) {

            i2s_clk_sel |= TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL;
        }
        break;
    case 24000:
        i2s_clk_sel |= TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_DAC16BIT_SEL;
        break;
    default:
        return -EINVAL; /* config already checked so should never happen */
    }

    tsb_i2s_init_block(info, TSB_I2S_BLOCK_SO);
    tsb_i2s_init_block(info, TSB_I2S_BLOCK_SI);

    tsb_i2s_mute(info, TSB_I2S_BLOCK_SO, 1);
    tsb_i2s_mute(info, TSB_I2S_BLOCK_SI, 1);

    tsb_i2s_write(info, TSB_I2S_BLOCK_SC, TSB_I2S_REG_AUDIOSET, sc_audioset);
    tsb_i2s_write(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_AUDIOSET,
                  sc_audioset | so_audioset);
    tsb_i2s_write(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_AUDIOSET,
                  sc_audioset | si_audioset);

    tsb_i2s_write(info, TSB_I2S_BLOCK_SC, TSB_I2S_REG_MODESET, modeset);
    tsb_i2s_write(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_MODESET, modeset);
    tsb_i2s_write(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_MODESET, modeset);

    if (info->mclk_role == TSB_I2S_CLK_ROLE_MASTER)
        tsb_i2s_config_plla(info, 48000 /* XXX */);

    /* This write starts mclk & bclk if they are clock masters */
    tsb_i2s_write_field(info, TSB_I2S_BLOCK_BRIDGE,
                        TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR,
                        TSB_CG_BRIDGE_I2S_CLOCK_SELECTOR_MASK, i2s_clk_sel);

    return 0;
}

static void tsb_i2s_enable(struct tsb_i2s_info *info)
{
    if (tsb_i2s_device_is_enabled(info))
        return;

    tsb_clk_enable(TSB_CLK_I2SSYS);
    tsb_clk_enable(TSB_CLK_I2SBIT);

    tsb_reset(TSB_RST_I2SSYS);
    tsb_reset(TSB_RST_I2SBIT);

    info->flags |= TSB_I2S_FLAG_ENABLED;
}

static void tsb_i2s_disable(struct tsb_i2s_info *info)
{
    if (!tsb_i2s_device_is_enabled(info))
        return;

    tsb_clk_disable(TSB_CLK_I2SBIT);
    tsb_clk_disable(TSB_CLK_I2SSYS);

    info->flags &= ~TSB_I2S_FLAG_ENABLED;
}

static int tsb_i2s_block_is_busy(struct tsb_i2s_info *info,
                                 enum tsb_i2s_block block)
{
    uint32_t busy_bit, v;

    switch (block) {
    case TSB_I2S_BLOCK_SC:
        busy_bit = TSB_I2S_REG_BUSY_BUSY;
        break;
    case TSB_I2S_BLOCK_SO:
    case TSB_I2S_BLOCK_SI:
        busy_bit = TSB_I2S_REG_BUSY_SPK_MIC_BUSY;
        break;
    default:
        lldbg("Bogus block: %d\n", block);
        return 0;
    }

    v = tsb_i2s_read(info, block, TSB_I2S_REG_BUSY);
    if (v & busy_bit) {
        return 1;
    }

    return 0;
}

static int tsb_i2s_start_block(struct tsb_i2s_info *info,
                               enum tsb_i2s_block block)
{
    uint32_t mask = TSB_I2S_REG_START_START;
    unsigned int limit;

    if (tsb_i2s_block_is_busy(info, block))
        return 0;

    if ((block == TSB_I2S_BLOCK_SO) || (block == TSB_I2S_BLOCK_SI)) {
        tsb_i2s_mute(info, TSB_I2S_BLOCK_SO, 0);

        mask |= TSB_I2S_REG_START_SPK_MIC_START;
    }

    tsb_i2s_write(info, block, TSB_I2S_REG_START, mask);

    for (limit = 1000; !tsb_i2s_block_is_busy(info, block) && --limit; );

    if (!limit)
        return -EIO;

    return 0;
}

static int tsb_i2s_stop_block(struct tsb_i2s_info *info,
                              enum tsb_i2s_block block)
{
    unsigned int limit;

    /* XXX Look at this closer */

    if ((block == TSB_I2S_BLOCK_SO) || (block == TSB_I2S_BLOCK_SI)) {
        for (limit = 1000;
             (tsb_i2s_read(info, block, TSB_I2S_REG_BUSY) &
                                            TSB_I2S_REG_BUSY_ERRBUSY) &&
              --limit; );

        if (!limit)
            return -EIO;

        tsb_i2s_mute(info, TSB_I2S_BLOCK_SO, 1);

        for (limit = 1000;
             (tsb_i2s_read(info, block, TSB_I2S_REG_BUSY) &
                                            TSB_I2S_REG_BUSY_SERIBUSY) &&
              --limit; );

        if (!limit)
            return -EIO;
    }

    if (!tsb_i2s_block_is_busy(info, block))
        return 0;

    tsb_i2s_write(info, block, TSB_I2S_REG_STOP, TSB_I2S_REG_STOP_I2S_STOP);

    for (limit = 1000; tsb_i2s_block_is_busy(info, block) && --limit; );

    if (!limit)
        return -EIO;

    return 0;
}

static int tsb_i2s_start_clocking(struct tsb_i2s_info *info,
                                  enum tsb_i2s_block block)
{
    int ret;

    if (info->config.ll_bclk_role == DEVICE_I2S_ROLE_MASTER) {
        ret = tsb_i2s_start_block(info, TSB_I2S_BLOCK_SC);
        if (ret)
            return ret;
    }

    ret = tsb_i2s_start_block(info, block);
    if (ret && (info->config.ll_bclk_role == DEVICE_I2S_ROLE_MASTER))
        tsb_i2s_stop_block(info, TSB_I2S_BLOCK_SC);

    return ret;
}

static void tsb_i2s_stop_clocking(struct tsb_i2s_info *info,
                                  enum tsb_i2s_block block)
{
    tsb_i2s_stop_block(info, block);

    if (info->config.ll_bclk_role == DEVICE_I2S_ROLE_MASTER)
        tsb_i2s_stop_block(info, TSB_I2S_BLOCK_SC);
}

static enum device_i2s_event tsb_i2s_intstat2event(uint32_t intstat)
{
    enum device_i2s_event event;

    if (!(intstat & TSB_I2S_REG_INT_ERROR_MASK))
        event = DEVICE_I2S_EVENT_NONE;
    else if (intstat & TSB_I2S_REG_INT_OR)
        event = DEVICE_I2S_EVENT_OVERRUN;
    else if (intstat & TSB_I2S_REG_INT_UR)
        event = DEVICE_I2S_EVENT_UNDERRUN;
    else if (intstat & TSB_I2S_REG_INT_LRCK)
        event = DEVICE_I2S_EVENT_CLOCKING;
    else
        event = DEVICE_I2S_EVENT_UNSPECIFIED;

    return event;
}

static uint32_t tsb_i2s_fifo_data(struct tsb_i2s_info *info, uint32_t w)
{
    uint16_t *hp;
    uint32_t fifo_w;

    /* FIFO channel data is little endian and sent over wire MSB first */

    switch(info->config.bytes_per_channel) {
    case 1:
        fifo_w = w;
        break;
    case 2:
        hp = (uint16_t *)&w;

        if (info->config.byte_order == DEVICE_I2S_BYTE_ORDER_BE) {
            hp[0] = swap16(hp[0]);
            hp[1] = swap16(hp[1]);
        }

        fifo_w = (hp[1] << 16) | hp[0];
        break;
    case 3:
        /* XXX Implement */
        fifo_w = w;
        break;
    case 4:
        if (info->config.byte_order == DEVICE_I2S_BYTE_ORDER_BE)
            fifo_w = swap32(w);
        else
            fifo_w = w;
        break;
    default: /* Should never happen */
        fifo_w = w;
        break;
    }

    return fifo_w;
}

static int tsb_i2s_drain_fifo(struct tsb_i2s_info *info,
                              enum device_i2s_event *event)
{
    unsigned int i;
    uint32_t intstat;
    uint32_t *dp;
    int ret = 0;

    dp = ring_buf_get_tail(info->rx_rb);

    for (i = 0; i < ring_buf_space(info->rx_rb); i += sizeof(*dp)) {
        intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INTSTAT);

        if (intstat & TSB_I2S_REG_INT_ERROR_MASK) {
            *event = tsb_i2s_intstat2event(intstat);
            ret = -EIO;
            break;
        }

        if (!(intstat & TSB_I2S_REG_INT_INT))
            break;

        *dp++ = tsb_i2s_fifo_data(info, tsb_i2s_read(info, TSB_I2S_BLOCK_SI,
                                                     TSB_I2S_REG_LMEM00));

        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, intstat);
    }

    ring_buf_put(info->rx_rb, i);

    info->rx_count += i;

    return ret;
}

static int tsb_i2s_rx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int ret = 0;

    while (ring_buf_is_producers(info->rx_rb) &&
           !ring_buf_is_full(info->rx_rb)) {

        if (ring_buf_len(info->rx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            ret = -EINVAL;
            break;
        }

        ret = tsb_i2s_drain_fifo(info, &event);
        if (ret)
            break;

        if (!ring_buf_is_full(info->rx_rb)) /* FIFO must be empty */
            break;

        ring_buf_pass(info->rx_rb);

        if (info->rx_callback)
            info->rx_callback(info->rx_rb, DEVICE_I2S_EVENT_RX_COMPLETE,
                              info->rx_arg);

        info->rx_rb = ring_buf_get_next(info->rx_rb);
    }

    if (ret) {
        tsb_i2s_stop_receiver(info);

        if (info->rx_callback)
            info->rx_callback(info->rx_rb, event, info->rx_arg);
    } else {
        tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SI,
                            TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT);
    }

    return ret;
}

static int tsb_i2s_start_receiver(struct tsb_i2s_info *info)
{
    int ret;

    ret = tsb_i2s_start_clocking(info, TSB_I2S_BLOCK_SI);
    if (ret)
        return ret;

    info->flags |= TSB_I2S_FLAG_RX_ACTIVE;

    return tsb_i2s_rx_data(info);
}

static void tsb_i2s_stop_receiver(struct tsb_i2s_info *info)
{
    uint32_t mask;

    tsb_i2s_stop_clocking(info, TSB_I2S_BLOCK_SI);

    mask = TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
           TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT;

    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SI, mask);
    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, mask);

    info->flags &= ~TSB_I2S_FLAG_RX_ACTIVE;
}

static int tsb_i2s_fill_fifo(struct tsb_i2s_info *info,
                             enum device_i2s_event *event)
{
    unsigned int i;
    uint32_t intstat;
    uint32_t *dp;
    int ret = 0;

    dp = (uint32_t *)ring_buf_get_head(info->tx_rb);

    for (i = 0; i < ring_buf_len(info->tx_rb); i += sizeof(*dp)) {
        intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INTSTAT);

        if (intstat & TSB_I2S_REG_INT_ERROR_MASK) {
lldbg("--- fill_fifo: intstat: 0x%x\n", intstat);
            *event = tsb_i2s_intstat2event(intstat);
            ret = -EIO;
            break;
        }

        if (!(intstat & TSB_I2S_REG_INT_INT))
            break;

        tsb_i2s_write(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_LMEM00,
                      tsb_i2s_fifo_data(info, *dp++));

        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, intstat);
    }

    ring_buf_pull(info->tx_rb, i);

    info->tx_count += i;

    return ret;
}

static int tsb_i2s_tx_data(struct tsb_i2s_info *info)
{
    enum device_i2s_event event = DEVICE_I2S_EVENT_NONE;
    int ret = 0;

    while (ring_buf_is_consumers(info->tx_rb)) {
        if (ring_buf_len(info->tx_rb) % 4) {
            event = DEVICE_I2S_EVENT_DATA_LEN;
            ret = -EINVAL;
            break;
        }

        ret = tsb_i2s_fill_fifo(info, &event);
        if (ret) {
lldbg("****** ahhh - %d\n", -ret);
            /* XXX do something here */
            break;
        }

        /*
         * FIFO must be full so unmask irq and exit.  When there is
         * room in the FIFO, irq will be asserted and irq handler will call
         * this routine again to put more data in the FIFO.
         */
        if (!ring_buf_is_empty(info->tx_rb)) {
            tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_INT);
            return 0;
        }

        /*
         * All the data in the rb was written to the fifo so call the
         * callback routine.  This means the callback is called once its
         * data is in the fifo but not necessarily sent on the wire.  This
         * is intentional in case the callback is what feeds the the driver
         * the next rb entry.
         */
        ring_buf_reset(info->tx_rb);
        ring_buf_pass(info->tx_rb);

        if (info->tx_callback)
            info->tx_callback(info->tx_rb, DEVICE_I2S_EVENT_TX_COMPLETE,
                              info->tx_arg);

        info->tx_rb = ring_buf_get_next(info->tx_rb);
    }

    if (ret) {
        tsb_i2s_stop_transmitter(info);

        if (info->tx_callback)
            info->tx_callback(info->tx_rb, event, info->tx_arg);
    }

    /* XXX fix error handling */
    /* No more data to send so mask off irq so we're not flooded */
    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_INT);

    return ret;
}

static int tsb_i2s_start_transmitter(struct tsb_i2s_info *info)
{
    int ret;

    if (!tsb_i2s_tx_is_active(info)) {
        ret = tsb_i2s_start_clocking(info, TSB_I2S_BLOCK_SO);
        if (ret) {
lldbg("--- ********** start_clocking failed: %d\n", -ret);
            return ret;
        }

        tsb_i2s_unmask_irqs(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INT_UR);

        info->flags |= TSB_I2S_FLAG_TX_ACTIVE;
    }

    return tsb_i2s_tx_data(info);
}

static void tsb_i2s_stop_transmitter(struct tsb_i2s_info *info)
{
    uint32_t mask;

    tsb_i2s_stop_clocking(info, TSB_I2S_BLOCK_SO);

    mask = TSB_I2S_REG_INT_LRCK | TSB_I2S_REG_INT_UR |
           TSB_I2S_REG_INT_OR | TSB_I2S_REG_INT_INT;

    tsb_i2s_mask_irqs(info, TSB_I2S_BLOCK_SO, mask);
    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, mask);

    info->flags &= ~TSB_I2S_FLAG_TX_ACTIVE;
}

static int tsb_i2s_irq_so_err_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = saved_dev->private;
    enum device_i2s_event event;
    uint32_t intstat;

    info->so_err++;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INTSTAT);
    event = tsb_i2s_intstat2event(intstat);

    if (event == DEVICE_I2S_EVENT_NONE) {
        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, intstat);
        return OK;
    }

    tsb_i2s_stop_transmitter(info);

    if (info->tx_callback)
        info->tx_callback(info->tx_rb, event, info->tx_arg);

    return OK;
}

static int tsb_i2s_irq_so_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = saved_dev->private;
    uint32_t intstat;

    info->so_int++;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SO, TSB_I2S_REG_INTSTAT);

    if (intstat & TSB_I2S_REG_INT_INT)
        tsb_i2s_tx_data(info);

#if 0 /* XXX */
    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SO, intstat);
#endif

    return OK;
}

static int tsb_i2s_irq_si_err_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = saved_dev->private;
    enum device_i2s_event event;
    uint32_t intstat;

    info->si_err++;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INTSTAT);
    event = tsb_i2s_intstat2event(intstat);

    if (event == DEVICE_I2S_EVENT_NONE) {
        tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, intstat);
        return OK;
    }

    tsb_i2s_stop_receiver(info);

    if (info->rx_callback)
        info->rx_callback(info->rx_rb, event, info->rx_arg);

    return OK;
}

static int tsb_i2s_irq_si_handler(int irq, void *context)
{
    struct tsb_i2s_info *info = saved_dev->private;
    uint32_t intstat;

    info->si_int++;

    intstat = tsb_i2s_read(info, TSB_I2S_BLOCK_SI, TSB_I2S_REG_INTSTAT);

    if (intstat & TSB_I2S_REG_INT_INT)
        tsb_i2s_rx_data(info);

    tsb_i2s_clear_irqs(info, TSB_I2S_BLOCK_SI, intstat);

    return OK;
}

static int tsb_i2s_op_get_processing_delay(struct device *dev,
                                           uint32_t *processing_delay)
{
    *processing_delay = 0; /* XXX */

    return 0;
}

static int tsb_i2s_op_get_supported_configurations(struct device *dev,
                        uint16_t *configuration_count,
                        const struct device_i2s_configuration *configurations[])
{
    *configuration_count = ARRAY_SIZE(tsb_i2s_config_table);
    *configurations = tsb_i2s_config_table;

    return 0;
}

static int tsb_i2s_op_set_configuration(struct device *dev,
                                 struct device_i2s_configuration *configuration)
{
    struct tsb_i2s_info *info = dev->private;

    if (!tsb_i2s_config_is_valid(configuration))
        return -EINVAL;

    if (tsb_i2s_rx_is_prepared(info) || tsb_i2s_tx_is_prepared(info))
        return -EBUSY;

    memcpy(&info->config, configuration, sizeof(info->config));

    info->flags |= TSB_I2S_FLAG_CONFIGURED;

    return 0;
}

static int tsb_i2s_op_prepare_receiver(struct device *dev,
                                       struct ring_buf *rx_rb,
                                       void (*callback)(
                                               struct ring_buf *rb,
                                               enum device_i2s_event event,
                                               void *arg),
                                       void *arg)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret;

    if (!rx_rb)
        return -EINVAL;

    flags = irqsave();

    if (!tsb_i2s_device_is_configured(info) ||
        tsb_i2s_rx_is_prepared(info) ||
        tsb_i2s_tx_is_prepared(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    tsb_i2s_enable(info);

    ret = tsb_i2s_config_hw(info);
    if (ret) {
        ret = -EIO;
        goto err_disable;
    }

    info->rx_rb = rx_rb;
    info->rx_callback = callback;
    info->rx_arg = arg;

    up_enable_irq(info->sierr_irq);
    up_enable_irq(info->si_irq);

    info->flags |= TSB_I2S_FLAG_RX_PREPARED;

    irqrestore(flags);

    return 0;

err_disable:
    if (!(info->flags & TSB_I2S_FLAG_TX_PREPARED))
        tsb_i2s_disable(info);
err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_start_receiver(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (!tsb_i2s_rx_is_prepared(info) || tsb_i2s_rx_is_active(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    ret = tsb_i2s_start_receiver(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_stop_receiver(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (!tsb_i2s_rx_is_active(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    tsb_i2s_stop_receiver(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_shutdown_receiver(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (tsb_i2s_rx_is_active(info)) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    if (!tsb_i2s_rx_is_prepared(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    info->rx_rb = NULL;
    info->rx_callback = NULL;
    info->rx_arg = NULL;

    up_disable_irq(info->si_irq);
    up_disable_irq(info->sierr_irq);

    info->flags &= ~TSB_I2S_FLAG_RX_PREPARED;

    if (!(info->flags & TSB_I2S_FLAG_TX_PREPARED))
        tsb_i2s_disable(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_prepare_transmitter(struct device *dev,
                                          struct ring_buf *tx_rb,
                                          void (*callback)(
                                               struct ring_buf *rb,
                                               enum device_i2s_event event,
                                               void *arg),
                                          void *arg)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    if (!tx_rb)
        return -EINVAL;

    flags = irqsave();

    if (!tsb_i2s_device_is_configured(info) ||
        tsb_i2s_rx_is_prepared(info) ||
        tsb_i2s_tx_is_prepared(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    tsb_i2s_enable(info);

    ret = tsb_i2s_config_hw(info);
    if (ret) {
        ret = -EIO;
        goto err_disable;
    }

    info->tx_rb = tx_rb;
    info->tx_callback = callback;
    info->tx_arg = arg;

    up_enable_irq(info->soerr_irq);
    up_enable_irq(info->so_irq);

    info->flags |= TSB_I2S_FLAG_TX_PREPARED;

    irqrestore(flags);

    return 0;

err_disable:
    if (!(info->flags & TSB_I2S_FLAG_RX_PREPARED))
        tsb_i2s_disable(info);
err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_start_transmitter(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (!tsb_i2s_tx_is_prepared(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    ret = tsb_i2s_start_transmitter(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_stop_transmitter(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (!tsb_i2s_tx_is_active(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    tsb_i2s_stop_transmitter(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_op_shutdown_transmitter(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (tsb_i2s_tx_is_active(info)) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    if (!tsb_i2s_tx_is_prepared(info)) {
        ret = -EIO;
        goto err_irqrestore;
    }

    info->tx_rb = NULL;
    info->tx_callback = NULL;
    info->tx_arg = NULL;

    up_disable_irq(info->so_irq);
    up_disable_irq(info->soerr_irq);

    info->flags &= ~TSB_I2S_FLAG_TX_PREPARED;

    if (!(info->flags & TSB_I2S_FLAG_RX_PREPARED))
        tsb_i2s_disable(info);

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static int tsb_i2s_dev_open(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;
    int ret = 0;

    flags = irqsave();

    if (tsb_i2s_device_is_open(info)) {
        ret = -EBUSY;
        goto err_irqrestore;
    }

    info->flags = TSB_I2S_FLAG_OPEN;

err_irqrestore:
    irqrestore(flags);

    return ret;
}

static void tsb_i2s_dev_close(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    if (!tsb_i2s_device_is_open(info))
        goto err_irqrestore;

    if (tsb_i2s_rx_is_active(info))
        tsb_i2s_op_stop_receiver(dev);

    if (tsb_i2s_tx_is_active(info))
        tsb_i2s_op_stop_transmitter(dev);

    if (tsb_i2s_rx_is_prepared(info))
        tsb_i2s_op_shutdown_receiver(dev);

    if (tsb_i2s_tx_is_prepared(info))
        tsb_i2s_op_shutdown_transmitter(dev);

    info->flags = 0;

err_irqrestore:
    irqrestore(flags);
}

static int tsb_i2s_extract_resources(struct device *dev,
                                     struct tsb_i2s_info *info)
{
    struct device_resource *r;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "cg_bridge");
    if (!r)
        return -EINVAL;

    info->cg_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "i2slp_sc");
    if (!r)
        return -EINVAL;

    info->sc_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "i2slp_so");
    if (!r)
        return -EINVAL;

    info->so_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_REGS, "i2slp_si");
    if (!r)
        return -EINVAL;

    info->si_base = r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_IRQ, "i2soerr");
    if (!r)
        return -EINVAL;

    info->soerr_irq = (int)r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_IRQ, "i2so");
    if (!r)
        return -EINVAL;

    info->so_irq = (int)r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_IRQ, "i2sierr");
    if (!r)
        return -EINVAL;

    info->sierr_irq = (int)r->start;

    r = device_resource_get_by_name(dev, DEVICE_RESOUCE_TYPE_IRQ, "i2si");
    if (!r)
        return -EINVAL;

    info->si_irq = (int)r->start;

    return 0;
}

static int tsb_i2s_dev_probe(struct device *dev)
{
    struct tsb_i2s_info *info;
    struct tsb_i2s_init_data *init_data = dev->init_data;
    irqstate_t flags;
    int ret;

    info = zalloc(sizeof(*info));
    if (!info)
        return -ENOMEM;

lldbg("LL i2s info struct: 0x%08p\n", info); /* XXX */

    ret = tsb_i2s_extract_resources(dev, info);
    if (ret)
        goto err_free_info;

    flags = irqsave();

    ret = irq_attach(info->soerr_irq, tsb_i2s_irq_so_err_handler);
    if (ret != OK)
        goto err_irqrestore;

    ret = irq_attach(info->so_irq, tsb_i2s_irq_so_handler);
    if (ret != OK)
        goto err_detach_soerr_irq;

    ret = irq_attach(info->sierr_irq, tsb_i2s_irq_si_err_handler);
    if (ret != OK)
        goto err_detach_so_irq;

    ret = irq_attach(info->si_irq, tsb_i2s_irq_si_handler);
    if (ret != OK)
        goto err_detach_sierr_irq;

    tsb_clr_pinshare(TSB_PIN_ETM);

    info->mclk_role = init_data->mclk_role;
    info->mclk_freq = init_data->mclk_freq;

#ifndef TSB_I2S_ENABLE_SLAVE_MODE
    /* External MCLK not supported -- changes tsb_i2s_config_table[] */
    if (info->mclk_role != TSB_I2S_CLK_ROLE_MASTER) {
        ret = ERROR;
        goto err_detach_si_irq;
    }
#endif

    info->dev = dev;
    dev->private = info;
    saved_dev = dev;

    irqrestore(flags);

    return OK;

#ifndef TSB_I2S_ENABLE_SLAVE_MODE
err_detach_si_irq:
    irq_detach(info->si_irq);
#endif
err_detach_sierr_irq:
    irq_detach(info->sierr_irq);
err_detach_so_irq:
    irq_detach(info->so_irq);
err_detach_soerr_irq:
    irq_detach(info->soerr_irq);
err_irqrestore:
    irqrestore(flags);
err_free_info:
    free(info);

    return ret;
}

static void tsb_i2s_dev_remove(struct device *dev)
{
    struct tsb_i2s_info *info = dev->private;
    irqstate_t flags;

    flags = irqsave();

    irq_detach(info->si_irq);
    irq_detach(info->sierr_irq);
    irq_detach(info->so_irq);
    irq_detach(info->soerr_irq);

    saved_dev = NULL;
    dev->private = NULL;

    irqrestore(flags);

    free(info);
}

static struct device_i2s_class_ops tsb_i2s_class_ops = {
    .get_processing_delay         = tsb_i2s_op_get_processing_delay,
    .get_supported_configurations = tsb_i2s_op_get_supported_configurations,
    .set_configuration            = tsb_i2s_op_set_configuration,
    .prepare_receiver             = tsb_i2s_op_prepare_receiver,
    .start_receiver               = tsb_i2s_op_start_receiver,
    .stop_receiver                = tsb_i2s_op_stop_receiver,
    .shutdown_receiver            = tsb_i2s_op_shutdown_receiver,
    .prepare_transmitter          = tsb_i2s_op_prepare_transmitter,
    .start_transmitter            = tsb_i2s_op_start_transmitter,
    .stop_transmitter             = tsb_i2s_op_stop_transmitter,
    .shutdown_transmitter         = tsb_i2s_op_shutdown_transmitter,
};

static struct device_driver_ops tsb_i2s_driver_ops = {
    .probe          = tsb_i2s_dev_probe,
    .remove         = tsb_i2s_dev_remove,
    .open           = tsb_i2s_dev_open,
    .close          = tsb_i2s_dev_close,
    .class_ops.i2s  = &tsb_i2s_class_ops,
};

struct device_driver tsb_i2s_driver = {
    .class      = DEVICE_CLASS_I2S_HW,
    .name       = "tsb_i2s",
    .desc       = "TSB I2S Driver",
    .ops        = &tsb_i2s_driver_ops,
};

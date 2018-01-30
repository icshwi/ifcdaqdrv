#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_acq420.h"
#include "ifcdaqdrv_scope.h"

// Bug in current firmware with decimations...
//static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200};
static const uint32_t decimations[] = {1, 0};
static const uint32_t averages[] = {1, 0};
static const double   valid_clocks[] = {100e6, 0};

ifcdaqdrv_status acq420_register(struct ifcdaqdrv_dev *ifcdevice){
    ifcdaqdrv_status status = 0;
    char            *p, *end;
    uint32_t         i;
    uint32_t          nsamples_max;
    ifcdevice->init_called           = 0;
    ifcdevice->init_adc              = acq420_init;
    ifcdevice->get_signature         = acq420_get_signature;
    ifcdevice->set_led               = acq420_set_led;
    ifcdevice->get_gain              = acq420_get_channel_gain;
    ifcdevice->set_gain              = acq420_set_channel_gain;
    ifcdevice->set_nsamples          = ifcdaqdrv_scope_set_nsamples;
    ifcdevice->get_nsamples          = ifcdaqdrv_scope_get_nsamples;
    ifcdevice->set_trigger_threshold = acq420_set_trigger_threshold;
    ifcdevice->get_trigger_threshold = acq420_get_trigger_threshold;
    ifcdevice->set_clock_frequency   = acq420_set_clock_frequency;
    ifcdevice->get_clock_frequency   = acq420_get_clock_frequency;
    ifcdevice->set_clock_source      = acq420_set_clock_source;
    ifcdevice->get_clock_source      = acq420_get_clock_source;
    ifcdevice->set_clock_divisor     = acq420_set_clock_divisor;
    ifcdevice->get_clock_divisor     = acq420_get_clock_divisor;
    ifcdevice->set_pattern           = acq420_set_pattern;
    ifcdevice->get_pattern           = acq420_get_pattern;

    ifcdevice->read_ai_ch            = ifcdaqdrv_scope_read_ai_ch;
    ifcdevice->read_ai               = ifcdaqdrv_scope_read_ai;

    ifcdevice->normalize_ch          = acq420_read_ch;
    ifcdevice->normalize             = acq420_read;

    ifcdevice->mode_switch = ifcdaqdrv_scope_switch_mode;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    //ifcdevice->mode        = ifcdaqdrv_acq_mode_smem;
    ifcdevice->sample_size = 4;
    ifcdevice->nchannels   = 4;

    memcpy(ifcdevice->decimations, decimations, sizeof(decimations));
    memcpy(ifcdevice->averages, averages, sizeof(averages));
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 1024*1024;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 1000;

    status                 = acq420_get_sram_nsamples_max(ifcdevice, &nsamples_max);
    if (status) {
        return status;
    }
    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size;
    ifcdevice->smem_size = 256 * 1024 * 1024;

    // Figure out the resolution and max frequency based on part number.
    // part_num typically looks like:
    // * "ACQ420FMC-4-2000 N=4 M=A1", or
    // * "ACQ420FMC-4-1000-18 N=4 M=A1"
    p = ifcdevice->fru_id->part_num;
    i = 0;
    while ((end = strchr(p, '-')) != NULL) {
        switch (i++) {
        case 0:
            // Product name
            break;
        case 1:
            // Channel numbers?
            break;
        case 2:
            // Clock Frequency
            ifcdevice->divisor_min = 100e6 / ((double)strtol(p, &end, 10) * 1000);
            break;
        }
        p = end + 1;
    }
    if (ifcdevice->divisor_min != 0) {
        ifcdevice->resolution = strtol(p, &end, 10);
    } else {
        ifcdevice->divisor_min = 100e6 / ((double)strtol(p, &end, 10) * 1000);
        ifcdevice->resolution    = 16;
    }

    ifcdevice->sample_resolution = 32;
    ifcdevice->vref_max = 10;

    return status_success;
}

ifcdaqdrv_status acq420_init(struct ifcdaqdrv_dev *ifcdevice) {
    int32_t          i32_reg_val;
    ifcdaqdrv_status status;
    uint8_t          revision = 0, version = 0;

    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, 0, &i32_reg_val);
    LOG((5, "Signature 0x%08x\n", i32_reg_val));

    // Global reset
    ifc_fmc_tcsr_write(ifcdevice, DTACQ_MCSR_REG, 0);

    acq420_get_signature(ifcdevice, &revision, &version, 0);

    // Firmware version 1.2 and higher supports higher frequencies
    if ((version >= 2) || (version >= 1 && revision >= 2)) {
        uint32_t bitw = 0, speed = 0;
        switch ((uint32_t)ifcdevice->divisor_min) {
        case 200:
            speed = 0;
            break;
        case 100:
            speed = 1;
            break;
        case 62:
            speed = 2;
            break;
        case 50:
            speed = 3;
            break;
        }
        switch (ifcdevice->resolution) {
        case 16:
            bitw = 0;
            break;
        case 18:
            bitw = 1;
            break;
        case 20:
            bitw = 2;
            break;
        }
#if 1
        // Workaround broken firmware
        if (speed == 3) {
            speed = 2;
            ifcdevice->divisor_min = 62;
        }
#endif
        // Calibration values idelay 0 - 31, stepdel 0 - 3
        uint32_t idelay = 0x01;
        uint32_t stepdel = 0x0;
        ifc_fmc_tcsr_write(ifcdevice, DTACQ_DEL_REG,
                           idelay << 24 | stepdel << 30 |
                           idelay << 16 | stepdel << 22 |
                           idelay << 8 | stepdel << 14 |
                           idelay | stepdel << 6
                           );
        LOG((5, "Setting up speed %d (%d) and resolution %d (%d)\n", ifcdevice->divisor_min, speed, ifcdevice->resolution, bitw));

        ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_MCSR_REG, bitw << 26 | speed << 24, 0x3 << 26 | 0x3 << 24);
    } else {
        ifcdevice->divisor_min = 200;
    }

    // Check if accessory is present
    status = ifc_fmc_tcsr_read(ifcdevice, DTACQ_MCSR_REG, &i32_reg_val);

    if (!(i32_reg_val & DTACQ_MCSR_A420_ACC_PRSNT)) {
        LOG((5, "%s(): No accessory detected! Check cabling.\n", __FUNCTION__));
    }

    // Set default clock: internal with highest frequency.
    acq420_set_clock_source(ifcdevice, ifcdaqdrv_clock_internal);
    acq420_set_clock_divisor(ifcdevice, ifcdevice->divisor_min);

    // Don't use any test pattern.
    acq420_set_pattern(ifcdevice, 0, ifcdaqdrv_pattern_none);
    acq420_set_pattern(ifcdevice, 1, ifcdaqdrv_pattern_none);
    acq420_set_pattern(ifcdevice, 2, ifcdaqdrv_pattern_none);
    acq420_set_pattern(ifcdevice, 3, ifcdaqdrv_pattern_none);

    // Enable FMC Interface
    acq420_fmc_if_en(ifcdevice);

    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, DTACQ_MCSR_REG, &i32_reg_val);
    LOG((6, "acq420 MCSR content 0x%08x\n", i32_reg_val));
    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH0_REG, &i32_reg_val);
    LOG((6, "acq420 CH0  content 0x%08x\n", i32_reg_val));
    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH1_REG, &i32_reg_val);
    LOG((6, "acq420 CH1  content 0x%08x\n", i32_reg_val));
    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH2_REG, &i32_reg_val);
    LOG((6, "acq420 CH2  content 0x%08x\n", i32_reg_val));
    if(DEBUG) ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH3_REG, &i32_reg_val);
    LOG((6, "acq420 CH3  content 0x%08x\n", i32_reg_val));

    return status;
}

ifcdaqdrv_status acq420_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock){
    int32_t          i;
    ifcdaqdrv_status status;

    for (i = 0; i < 4; ++i) {
        status = acq420_set_clock_source_ch(ifcdevice, i, clock);
        if (status) {
            return status;
        }
    }
    return status_success;
}

ifcdaqdrv_status acq420_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock){
    return acq420_get_clock_source_ch(ifcdevice, 0, clock);
}

ifcdaqdrv_status acq420_set_clock_source_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_clock clock){
    ifcdaqdrv_status  status;
    ifcdaqdrv_pattern pattern;

    status = acq420_get_pattern(ifcdevice, channel, &pattern);
    if (status) {
        return status;
    }

    /* If 'pattern != none' we now that the internal clock already is used. */
    if (clock == ifcdaqdrv_clock_internal && pattern == ifcdaqdrv_pattern_none) {
        status = ifc_fmc_tcsr_setclr(
            ifcdevice,
            DTACQ_CH0_REG + channel,
            (1 << DTACQ_CHx_ACQ_TCNT_ENA_SHIFT) | (DTACQ_MODE_FIFO_INT_CLK << DTACQ_CHx_ACQ_MOD_SHIFT),
            DTACQ_CHx_ACQ_TCNT_ENA_MASK | DTACQ_CHx_ACQ_MOD_MASK);
    } else if (clock == ifcdaqdrv_clock_external && pattern != ifcdaqdrv_pattern_none) {
        return status_no_support;
    } else if (clock == ifcdaqdrv_clock_external) {
        // ifc_fmc_tcsr_setclr(ifcdevice, 2, 0, 1); // Set direction of EXT_CLK to input.
        status = ifc_fmc_tcsr_setclr(
            ifcdevice,
            DTACQ_CH0_REG + channel,
            DTACQ_MODE_FIFO_EXT_CLK << DTACQ_CHx_ACQ_MOD_SHIFT,
            DTACQ_CHx_ACQ_TCNT_ENA_MASK | DTACQ_CHx_ACQ_MOD_MASK);
    }

    return status;
}

ifcdaqdrv_status acq420_get_clock_source_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_clock *clock){
    int32_t          i32_reg_val;
    ifcdaqdrv_status status;

    status = ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH0_REG + channel, &i32_reg_val);
    switch ((i32_reg_val & DTACQ_CHx_ACQ_MOD_MASK) >> 28) {
    case 0x8:
    case 0x9:
    case 0xC:
    case 0xD:
        *clock = ifcdaqdrv_clock_external;
        break;
    case 0x1:
    case 0x2:
    case 0x3:
    case 0xA:
    case 0xB:
    case 0xE:
    case 0xF:
    default:
        *clock = ifcdaqdrv_clock_internal;
        break;
    }
    return status;
}

ifcdaqdrv_status acq420_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency){
    UNUSED(ifcdevice);
    if(frequency != 100e6) {
        return status_argument_range;
    }
    return status_success;
}

ifcdaqdrv_status acq420_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor){
    uint32_t i;
    ifcdaqdrv_status status;

    for (i = 0; i < 4; ++i) {
        status = acq420_set_clock_divisor_ch(ifcdevice, i, divisor);
        if(status) {
            return status;
        }
    }

    return status_success;
}

ifcdaqdrv_status acq420_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency){
    UNUSED(ifcdevice);
    *frequency = 100e6;
    return status_success;
}

/* Currently limited to 2'000ns (500kHz) period time in firmware */
ifcdaqdrv_status acq420_set_clock_divisor_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t divisor) {
    // 0x1<<20:   Enable LCLK
    return ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_CH0_REG + channel, (divisor - 1) | 1 << 20, DTACQ_CHx_ACQ_TCNT_MASK);
}

ifcdaqdrv_status acq420_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor) {
    return acq420_get_clock_divisor_ch(ifcdevice, 0, divisor);
}

ifcdaqdrv_status acq420_get_clock_divisor_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t *divisor) {
    int32_t          i32_reg_val;
    ifcdaqdrv_status status;
    status     = ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH0_REG + channel, &i32_reg_val);

    *divisor = (i32_reg_val & DTACQ_CHx_ACQ_TCNT_MASK) + 1;
    return status;
}

ifcdaqdrv_status acq420_set_pattern(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern pattern) {
    ifcdaqdrv_status status;
    ifcdaqdrv_clock  clock;
    int32_t          mode;

    status = acq420_get_clock_source_ch(ifcdevice, channel, &clock);
    if (status) {
        return status;
    }

    if (pattern != ifcdaqdrv_pattern_none && clock == ifcdaqdrv_clock_external) {
        // Test patterns are not available with external clock.
        return status_no_support;
    }

    switch (pattern) {
    case ifcdaqdrv_pattern_none:
        if (clock == ifcdaqdrv_clock_internal) {
            mode = DTACQ_MODE_FIFO_INT_CLK;
        } else {
            mode = DTACQ_MODE_FIFO_EXT_CLK;
        }
        break;
    case ifcdaqdrv_pattern_ramp_inc:
        mode = DTACQ_MODE_FIFO_TEST_INC;
        break;
    case ifcdaqdrv_pattern_ramp_dec:
        mode = DTACQ_MODE_FIFO_TEST_DEC;
        break;
    default:
        return status_no_support;
    }

    return ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_CH0_REG + channel, mode << DTACQ_CHx_ACQ_MOD_SHIFT,
                               DTACQ_CHx_ACQ_MOD_MASK);
}

ifcdaqdrv_status acq420_get_pattern(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern *pattern) {
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH0_REG + channel, &i32_reg_val);

    switch ((i32_reg_val & DTACQ_CHx_ACQ_MOD_MASK) >> DTACQ_CHx_ACQ_MOD_SHIFT) {
    case DTACQ_MODE_FIFO_TEST_INC:
        *pattern = ifcdaqdrv_pattern_ramp_inc;
        break;
    case DTACQ_MODE_FIFO_TEST_DEC:
        *pattern = ifcdaqdrv_pattern_ramp_dec;
        break;
    default:
        *pattern = ifcdaqdrv_pattern_none;
        break;
    }

    return status;
}

/* Enable FMC Interface */
ifcdaqdrv_status acq420_fmc_if_en(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    // Enable/reset clock
    status = ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_MCSR_REG, DTACQ_MCSR_A420_LCLK_GENA, 0);
    if (status) {
        return status;
    }

    // Interface enable (will also synchronize clocks)
    status = ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_MCSR_REG, DTACQ_MCSR_A420_GLOB_ENA, 0);
    if (status) {
        return status;
    }

    /* Clear general control register and enable specific acquisition mode.. */
    if (ifcdevice->fmc == 1) {
        status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC1_ACQRUN_SHIFT, 0xffffffff);
    } else { /* fmc is FMC2 */
        status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC2_ACQRUN_SHIFT, 0xffffffff);
    }
    return status;
}

#if 0
ifcdaqdrv_status acq420_acq_read(struct ifcdaqdrv_dev *ifcdevice, struct pev_ioctl_buf *pev_buf, size_t buflen){
    printf("%s(): Buffer Length %d KiB (%d)\n", __FUNCTION__, buflen * 4 / 1024, buflen);
    ifcdaqdrv_status      status;
    struct pev_ioctl_rdwr rdwr = {0};

    rdwr.buf        = pev_buf->u_addr;
    rdwr.offset     = 0x100000;
    rdwr.len        = buflen;
    rdwr.mode.ds    = RDWR_INT;
    rdwr.mode.swap  = RDWR_NOSWAP;
    rdwr.mode.dir   = RDWR_READ;
    rdwr.mode.space = RDWR_MEM;
    rdwr.mode.am    = fmc == IFC_FMC1 ? 0x40 : 0x50;

    status          = pevx_rdwr(crate, &rdwr);

    if (status != 0) {
        printf("%s(): ERROR pevx_rdwr() return %d\n", __FUNCTION__, status);
    }
    return status;
}
#endif

ifcdaqdrv_status acq420_get_channel_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain) {
    int32_t          i32_reg_val;
    ifcdaqdrv_status status;

    status = ifc_fmc_tcsr_read(ifcdevice, DTACQ_CH0_REG + channel, &i32_reg_val);
    if (status) {
        return status;
    }

    *gain = 1 << ((i32_reg_val & DTACQ_CHx_ACQ_PREA_MASK) >> DTACQ_CHx_ACQ_PREA_SHIFT);
    return status;
}

ifcdaqdrv_status acq420_set_channel_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain) {
    int32_t i32_reg_val = 0;

    if (gain >= 8.0) {
        i32_reg_val = 3;
    } else if (gain >= 4.0) {
        i32_reg_val = 2;
    } else if (gain >= 2.0) {
        i32_reg_val = 1;
    }

    return ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_CH0_REG + channel, i32_reg_val << DTACQ_CHx_ACQ_PREA_SHIFT,
                               DTACQ_CHx_ACQ_PREA_MASK);
}

ifcdaqdrv_status acq420_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state){
    int32_t i32_reg_val = 0;
    UNUSED(led); // Should support both leds in future.
    switch (led_state) {
    case ifcdaqdrv_led_color_green:
        i32_reg_val = DTACQ_MCSR_IFC_LED_GREEN;
        break;
    case ifcdaqdrv_led_color_red:
        i32_reg_val = DTACQ_MCSR_IFC_LED_RED;
        break;
    case ifcdaqdrv_led_off:
    default:
        i32_reg_val = 0;
        break;
    }
    return ifc_fmc_tcsr_setclr(ifcdevice, DTACQ_MCSR_REG, i32_reg_val, DTACQ_MCSR_IFC_LED_MASK);
}

/**
 * Data is stored interleaved in SMEM. This function will de-interleave the data from *data to *res.
 *
 * Channel 0 data will be in res[0]
 * Channel 1 data will be in res[1*nelm]
 * Channel 2 data will be in res[2*nelm]
 * Channel 3 data will be in res[3*nelm]
 *
 * nelm is the number of elements in each channel.
 */

ifcdaqdrv_status acq420_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples) {
    int32_t *target; /* Copy to this address */
    int32_t *itr;    /* Iterator for iterating over "data" */
    int32_t *origin; /* Copy from this address */

    // This printf is used to debug firmware "Last address" issues.
    //printf("dst_offset %zu src_offset %zu nelm %zu nsamples %zu\n", dst_offset, src_offset, nelm, channel_nsamples);

    UNUSED(ifcdevice);
    /* Multiply offsets by number of channels */
    target = ((int32_t *)dst) + dst_offset;
    origin = ((int32_t *)src) + src_offset * 4;

    for (itr = origin; itr < origin + nelm * 4; ++target, itr += 4) {
        *(target + 0 * channel_nsamples) = *(itr + 0);
        *(target + 1 * channel_nsamples) = *(itr + 1);
        *(target + 2 * channel_nsamples) = *(itr + 2);
        *(target + 3 * channel_nsamples) = *(itr + 3);
    }
    return status_success;

}

ifcdaqdrv_status acq420_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                             size_t nelm){
    int32_t *target;                 /* Copy to this address */
    int32_t *itr;                    /* Iterator for iterating over "data" */
    int32_t *origin;                 /* Copy from this address */

    UNUSED(ifcdevice);
    origin =  (int32_t *)data + offset;
    target = res;

    if(ifcdevice->mode == ifcdaqdrv_acq_mode_smem) {
        for (itr = origin; itr < origin + nelm * 4; target++, itr += 4) {
            *target = *(itr + channel);
        }
        return status_success;
    }

    /* 1. This right shift is compiler dependent. According to wikipedia most compilers do
     *    arithmetic right shift which is the right thing in this case.
     * 2. Values are stored left aligned in 32 bit words... */
    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = *itr;
    }
    return status_success;
}

ifcdaqdrv_status acq420_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold) {
    ifcdaqdrv_status status;
    threshold >>= 12; // Threshold should be left-aligned ADC value.
    int32_t          i32_reg_val;

    /* bits [3:0] are stored in the "trigger level extended" register. This register is common for all modes (hence
     * ifc_xuser_tcsr_setclr and not ifc_scope_acq_tcsr_setclr... */
    if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram && ifcdevice->fmc == 1) {
        i32_reg_val = (threshold & 0xF) << IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_SHIFT;
        status      = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_TRGLEVEXT_REG, i32_reg_val,
                                            IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_MASK);
    } else if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram) { // fmc == 2
        i32_reg_val = (threshold & 0xF) << IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_SHIFT;
        status      = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_TRGLEVEXT_REG, i32_reg_val,
                                            IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_MASK);
    } else if (ifcdevice->mode == ifcdaqdrv_acq_mode_smem && ifcdevice->fmc == 1) {
        i32_reg_val = (threshold & 0xF) << IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_SHIFT;
        status      = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_TRGLEVEXT_REG, i32_reg_val,
                                            IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_MASK);
    } else { // smem && fmc == 2
        i32_reg_val = (threshold & 0xF) << IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_SHIFT;
        status      = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_TRGLEVEXT_REG, i32_reg_val,
                                            IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_MASK);
    }

    if (status) {
        return status;
    }

    /* bits [19:4] are stored in the trigger register */
    return ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, (threshold >> 4) &
                                     IFC_SCOPE_TCSR_TRG_Trig_Level_MASK, IFC_SCOPE_TCSR_TRG_Trig_Level_MASK);
}

ifcdaqdrv_status acq420_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold) {
    ifcdaqdrv_status status;
    int32_t          threshold_adc;
    int32_t          i32_reg_val;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, &i32_reg_val);
    if (status) {
        return status;
    }

    /* bits [19:4] are stored in the trigger register */
    threshold_adc = (i32_reg_val & IFC_SCOPE_TCSR_TRG_Trig_Level_MASK) << 4;

    /* Sign extension */
    if (threshold_adc & 0x80000) {
        threshold_adc |= 0xFFF00000;
    }

    /* bits [3:0] are stored in the "trigger level extended" register. This register is common for all modes (hence
     * ifc_xuser_tcsr_read and not ifc_scope_acq_tcsr_read... */
    status = ifc_xuser_tcsr_read(ifcdevice, IFC_SCOPE_DTACQ_TCSR_TRGLEVEXT_REG, &i32_reg_val);
    if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram && ifcdevice->fmc == 1) {
        threshold_adc |= (i32_reg_val & IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_MASK) >> IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_SHIFT;
    } else if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram) {
        threshold_adc |= (i32_reg_val & IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_MASK) >> IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_SHIFT;
    } else if (ifcdevice->mode == ifcdaqdrv_acq_mode_smem && ifcdevice->fmc == 1) {
        threshold_adc |= (i32_reg_val & IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_MASK) >> IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_SHIFT;
    } else {
        threshold_adc |= (i32_reg_val & IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_MASK) >> IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_SHIFT;
    }

    /* Threshold is stored left aligned with 20 bits */
    *threshold = threshold_adc >> (20 - ifcdevice->resolution);

    return status;
}

ifcdaqdrv_status acq420_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                      uint16_t *board_id) {
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DTACQ_SIGN_REG, &i32_reg_val);

    if (revision) {
        *revision = (i32_reg_val & DTACQ_SIGN_HWREV_MASK) >> DTACQ_SIGN_HWREV_SHIFT;
    }

    if (version) {
        *version = (i32_reg_val & DTACQ_SIGN_HWVER_MASK) >> DTACQ_SIGN_HWVER_SHIFT;
    }

    if (board_id) {
        *board_id = (i32_reg_val & DTACQ_SIGN_BOARDID_MASK) >> DTACQ_SIGN_BOARDID_SHIFT;
    }

    return status;
}

ifcdaqdrv_status acq420_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){
    int32_t i32_reg_val;
    int     status;
    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (i32_reg_val & IFC_SCOPE_TCSR_CS_SRAM_Buffer_Size_MASK) {
        *nsamples_max = 16 * 1024;
    } else {
        *nsamples_max = 8 * 1024;
    }
    return status;
}

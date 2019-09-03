#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>

#include "tscioctl.h"
#include "tsculib.h"

#include <epicsThread.h>
#include <epicsTime.h>

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"
// #include "ifcdaqdrv_acq420.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_adc3117.h"
#include "ifcdaqdrv_dio3118.h"
// #include "ifcdaqdrv_adc3112.h"
//typedef long dma_addr_t;

ifcdaqdrv_status ifcdaqdrv_scope_register(struct ifcdaqdrv_dev *ifcdevice){
    char *p;
    p = ifcdevice->fru_id->product_name;

    if (p) {
        if (strcmp(p, "ACQ420FMC") == 0) {
            INFOLOG(("No support for ACQ420FMC yet!\n"));
            return status_incompatible;
        } else if (strcmp(p, "ADC3110") == 0) {
            INFOLOG(("Identified ADC3110 on FMC %d\n", ifcdevice->fmc));
            adc3110_register(ifcdevice);
        } else if (strcmp(p, "ADC3111") == 0) {
            INFOLOG(("Identified ADC3111 on FMC %d\n", ifcdevice->fmc));
            adc3111_register(ifcdevice);
        } else if (strcmp(p, "ADC_3111") == 0) {
            INFOLOG(("Identified %s on FMC %d\n", p, ifcdevice->fmc));
            adc3111_register(ifcdevice);
        } else if (strcmp(p, "ADC3112") == 0) {
            INFOLOG(("No support for ADC3112 yet\n"));
            return status_incompatible;
        } else if (strcmp(p, "ADC3117") == 0) {
            INFOLOG(("Identified ADC3117 on FMC %d\n", ifcdevice->fmc));
            adc3117_register(ifcdevice);
        } else if (strcmp(p, "DIO3118") == 0) {
            INFOLOG(("Identified DIO3118 on FMC %d\n", ifcdevice->fmc));
            dio3118_register(ifcdevice);
        }else {
            LOG((LEVEL_ERROR, "No recognized device %s - but will force ADC3117\n", p));
            return status_incompatible;
        }
    } else {
        LOG((4, "Internal error, no product_name\n"));
        return status_internal;
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_set_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples){
    int32_t i32_reg_val = 0;

    switch (nsamples) {
    case 32 * 1024:
        i32_reg_val = 0;
        break;
    case 16 * 1024:
        i32_reg_val = 1;
        break;
    case 8 * 1024:
        i32_reg_val = 2;
        break;
    case 4 * 1024:
        i32_reg_val = 3;
        break;
    case 2 * 1024:
        i32_reg_val = 4;
        break;
    case 1 * 1024:
        i32_reg_val = 5;
        break;
    default:
        return status_argument_range;
    }
    LOG((5, "acq %d, reg_val %08x\n", nsamples, i32_reg_val));
    return ifc_scope_acq_tcsr_setclr(ifcdevice, 0, i32_reg_val << 12, IFC_SCOPE_TCSR_CS_SRAM_ACQ_Size_MASK);
}

ifcdaqdrv_status ifcdaqdrv_scope_get_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples){
    int32_t i32_reg_val;
    
    int     status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    switch ((i32_reg_val & IFC_SCOPE_TCSR_CS_SRAM_ACQ_Size_MASK) >> 12) {
    case 0:
        *nsamples = 32 * 1024;
        break;
    case 1:
        *nsamples = 16 * 1024;
        break;
    case 2:
        *nsamples = 8 * 1024;
        break;
    case 3:
        *nsamples = 4 * 1024;
        break;
    case 4:
        *nsamples = 2 * 1024;
        break;
    case 5:
        *nsamples = 1 * 1024;
        break;
    default:
        return status_internal;
    }
    LOG((7, "acq %d, reg_val %08x\n", *nsamples, i32_reg_val));

    return status;
}

/*
 * Get Number of samples per channel
 */

ifcdaqdrv_status ifcdaqdrv_scope_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples) {
    int32_t  i32_reg_val;
    ifcdaqdrv_status status;
    uint32_t acq_size;
    uint32_t average;

    if(!ifcdevice->nchannels || !ifcdevice->sample_size) {
        return status_internal;
    }

    status   = ifc_scope_acq_tcsr_read(ifcdevice, 3, &i32_reg_val);
    acq_size = (i32_reg_val & 0xFF) * 1024 * 1024;
    if(!acq_size) { // 0 = 256 MiB.
        acq_size = 256 * 1024 * 1024;
    }

    status = ifcdaqdrv_scope_get_average(ifcdevice, &average);

    /* Exception: ADC311X limits to two channels if averaging is disabled */
    if(ifcdevice->nchannels == 8 && average == 1) {
        *nsamples = acq_size / 2 / ifcdevice->sample_size;
        return status;
    }

    *nsamples = acq_size / ifcdevice->nchannels / ifcdevice->sample_size;

    return status;
}

/*
 * Set Number of samples per channel in SMEM mode.
 *
 * Valid highest input is 256MiB / nchannels / sample_size.
 * If number of samples is N, N * nchannels * sample_size has to be an even MiB.
 */

ifcdaqdrv_status ifcdaqdrv_scope_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples) {
    int32_t i32_reg_val;
    uint32_t average;
    ifcdaqdrv_status status;
    uint32_t nchannels;

    status = ifcdaqdrv_scope_get_average(ifcdevice, &average);

    if(ifcdevice->nchannels == 8 && average == 1) {
        nchannels = 2;
    } else {
        nchannels = ifcdevice->nchannels;
    }

    i32_reg_val = nsamples * nchannels * ifcdevice->sample_size / (1024 * 1024); // 0 = 256MiB
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, 3, i32_reg_val & 0xFF, 0xFF);
    return status;
}

/* Returns last sample index written to in circular buffer. */
ifcdaqdrv_status ifcdaqdrv_get_sram_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address){
    int32_t i32_reg_val;
    int     status;

    status = ifc_xuser_tcsr_read(ifcdevice, ifc_get_scope_tcsr_offset(ifcdevice) + 2, &i32_reg_val);

    if (ifcdevice->sample_size == 4) {
        *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SRAMx_LA_Last_Address_MASK) >> 2;
    } else { // sample_size == 2
        *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SRAMx_LA_Last_Address_MASK) >> 1;
    }

    return status;
}

ifcdaqdrv_status ifcdaqdrv_get_smem_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address){
    int32_t i32_reg_val;
    int     status;

    status = ifc_xuser_tcsr_read(ifcdevice, ifc_get_scope_tcsr_offset(ifcdevice) + 2, &i32_reg_val);

    *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SMEMx_LA_Last_Address_MASK) >> 4;

    return status;
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq){
    if (ptq > 7) {
        return status_argument_range;
    }

    /* TODO: check if quota 0 is valid */
    return ifc_xuser_tcsr_setclr(ifcdevice, ifc_get_scope_tcsr_offset(ifcdevice), ptq << 5,
                                 IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK);
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq){
    int32_t i32_reg_val;
    int     status;

    status = ifc_xuser_tcsr_read(ifcdevice, ifc_get_scope_tcsr_offset(ifcdevice), &i32_reg_val);
    if (status) {
        return status;
    }
    if (!ptq) {
        return status_argument_invalid;
    }

    *ptq = ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK) >> 5);

    return status;
}

ifcdaqdrv_status ifcdaqdrv_scope_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data) {
    ifcdaqdrv_status      status;
    int32_t               offset       = 0;
    int32_t              *origin;
    int32_t              *res          = data;
    uint32_t              last_address = 0, nsamples = 0, npretrig = 0, ptq = 0;
    uint32_t channel;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        for(channel = 0; channel < ifcdevice->nchannels; ++channel) {
            ifcdevice->read_ai_ch(ifcdevice, channel, ((int32_t *)data) + nsamples * channel);
        }
        break;
    case ifcdaqdrv_acq_mode_smem:
        offset = 0;

        ifcdaqdrv_start_tmeas();
        status = ifcdaqdrv_read_smem_unlocked(ifcdevice, ifcdevice->all_ch_buf, ifcdevice->smem_dma_buf, offset, nsamples * ifcdevice->nchannels * ifcdevice->sample_size);
        if (status) {
            return status;
        }
        ifcdaqdrv_end_tmeas();
        

        status = ifcdaqdrv_get_smem_la(ifcdevice, &last_address);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_get_ptq(ifcdevice, &ptq);
        if (status) {
            return status_device_access;
        }

        origin   = ifcdevice->all_ch_buf;
        npretrig = (nsamples * ptq) / 8;

#if PRETRIG_ORGANIZE

        ifcdaqdrv_start_tmeas();

        // * For ADC311X increase Last address with 2 (because there are 2 samples per "acquisition block" in DDR).
        // * For FMC420 decrease Last address with 3 (becayse LA is 4 samples to much high, other offsets has also been seen).
        int32_t samples_per_block = ifcdevice->sample_size == 2 ? 2 : -3;
        if (npretrig > 0) {
            /* Copy from "Last Address + 1 sample block" to end of pretrig buffer */
            status = ifcdevice->normalize(ifcdevice, res, 0, origin, (last_address + samples_per_block), npretrig - (last_address + samples_per_block), nsamples);
            if (status) {
                return status;
            }

            /* Copy from 0 to Last Address + 1 sample block */
            status = ifcdevice->normalize(ifcdevice, res, npretrig - (last_address + samples_per_block), origin, 0, last_address + samples_per_block, nsamples);
            if (status) {
                return status;
            }
        }

        /* Copy from end of pretrig buffer to end of samples */
        status = ifcdevice->normalize(ifcdevice, res, npretrig, origin, npretrig, nsamples - npretrig, nsamples);
        if (status) {
            return status;
        }

        ifcdaqdrv_end_tmeas();
        

#if 0
        int32_t *itr;
        printf("%s(): u_base %p, acq_size %db, nsamples %d, npretrig %d, la %d, ptq %d, origin %p\n", __FUNCTION__,
                ifcdevice->smem_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, last_address, ptq,
                origin);

        printf("0x%08x: ", (int32_t) origin);
        for (itr = origin; itr < origin + 16; ++itr) {
            printf("%08x ", *itr);
        }
        printf("\n");
#endif
#else
        UNUSED(npretrig);
        status = ifcdevice->normalize(ifcdevice, res, 0, origin, 0, nsamples, nsamples);
        if (status) {
            return status;
        }
#endif
    }
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data) {
    ifcdaqdrv_status  status;
    int32_t           offset;
    int16_t          *origin;
    int32_t          *res;
    uint32_t          last_address,  nsamples,  npretrig,  ptq;


    offset = 0;
    origin = NULL;
    res = data;
    last_address = 0;
    nsamples = 0;
    npretrig = 0;
    ptq = 0;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        if (ifcdevice->board_id == 0x3117) {
            //if (ifcdevice->fmc == 1) {
            if (channel < 2) {
                offset = IFC_SCOPE_LITE_SRAM_FMC1_SAMPLES_OFFSET + (channel * 0x40000);
            } else {
                offset = IFC_SCOPE_LITE_SRAM_FMC2_SAMPLES_OFFSET + (channel * 0x40000);
            }
        } else {
            offset = IFC_SCOPE_SRAM_SAMPLES_OFFSET + (channel << 16);
        }

        status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }
        
        status = ifcdaqdrv_get_sram_la(ifcdevice, &last_address);
        if (status) {
            return status;
        }

        status = ifcdaqdrv_get_ptq(ifcdevice, &ptq);
        if (status) {
            return status;
        }

        if (ifcdaqdrv_is_byte_order_ppc())
            ifcdaqdrv_manualswap((uint16_t*) ifcdevice->sram_dma_buf->u_base,nsamples);

        origin   = ifcdevice->sram_dma_buf->u_base;
        npretrig = (nsamples * ptq) / 8;
        break;
    

    case ifcdaqdrv_acq_mode_smem:
#if 0
        status = ifcdevice->get_smem_base_address(ifcdevice, &offset);
        if (status) {
            return status_device_access;
        }
#endif
        offset = 0;

        printf("Entered ifcdaqdrv_scope_read_ai_ch with case ifcdaqdrv_acq_mode_smem\n");

        status = ifcdaqdrv_read_smem_unlocked(ifcdevice, ifcdevice->all_ch_buf, ifcdevice->smem_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }

        status = ifcdaqdrv_get_smem_la(ifcdevice, &last_address);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_get_ptq(ifcdevice, &ptq);
        if (status) {
            return status_device_access;
        }

        origin   = ifcdevice->all_ch_buf;
        npretrig = (nsamples * ptq) / 8;
        break;
    }

#if 0
    int16_t *itr;
    printf("%s(): u_base %p, acq_size %db, nsamples %d, npretrig %d, la %d, ptq %d, origin %p\n", __FUNCTION__,
            ifcdevice->sram_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, last_address, ptq,
            origin);

    printf("0x%08x: ", (int16_t) origin);
    for (itr = origin; itr < origin + 16; ++itr) {
        printf("%08x ", *itr);
    }
    printf("\n");
#endif

#if 1//PRETRIG_ORGANIZE
    /* When a pretrigger amount is selected, the first chunk of the memory is a circular buffer. The memory is
     * therefore structured in the following parts:
     *
     *    From device              To user
     * 1. LA+1 to npretrig      -> 0 to LA+1
     * 2. 0 to LA+1             -> LA+1 to npretrig
     * 3. npretrig to nsamples  -> npretrig to nsamples
     *
     * Last Address (LA)
     *     is the "Last address written to in the circular buffer". This imples that the oldest value in the
     *     circular buffer is in LA+1.
     * npretrig
     *     is the size of the circular buffer.
     * nsamples
     *     is the total number of samples.
     */
    if (npretrig > 0) {
        /* Copy from "Last Address + 1" to end of pretrig buffer */
        status = ifcdevice->normalize_ch(ifcdevice, channel, res, origin, last_address + 1, npretrig - (last_address + 1));
        if (status) {
            return status;
        }

        /* Copy from 0 to Last Address + 1 */
        status = ifcdevice->normalize_ch(ifcdevice, channel, res + (npretrig - (last_address + 1)), origin, 0,
                last_address + 1);
        if (status) {
            return status;
        }
    }

    /* Copy from end of pretrig buffer to end of samples */
    status = ifcdevice->normalize_ch(ifcdevice, channel, res + npretrig, origin, npretrig, nsamples - npretrig);
    if (status) {
        return status;
    }
#else
    UNUSED(npretrig);
    status = ifcdevice->normalize_ch(ifcdevice, channel, res, origin, 0, nsamples);
    if (status) {
        return status;
    }
#endif

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples) {
    int32_t *target; /* Copy to this address */
    int16_t *itr;    /* Iterator for iterating over "data" */
    int16_t *origin; /* Copy from this address */
    int16_t channel;

    /* Multiply offsets by number of channels */
    target = ((int32_t *)dst) + dst_offset;
    origin = ((int16_t *)src) + src_offset * ifcdevice->nchannels;

    if (ifcdevice->board_id == 0x3117) {
        for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; target += 2, itr += (ifcdevice->nchannels * 2)) {
            for (channel = 0; channel < ifcdevice->nchannels; channel++) {
                *((target + 0) + channel * channel_nsamples) = (int16_t)*(itr + channel * 2);
                *((target + 1) + channel * channel_nsamples) = (int16_t)*(itr + (channel * 2 + 1));
            }
        }
    }
    else {
        for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; target += 2, itr += (ifcdevice->nchannels * 2)) {
            for (channel = 0; channel < ifcdevice->nchannels; channel++) {
                *((target + 0) + channel * channel_nsamples) = (int16_t)(*(itr + channel * 2) - 32768);
                *((target + 1) + channel * channel_nsamples) = (int16_t)(*(itr + (channel * 2 + 1)) - 32768);
            }
        }
    }
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                              size_t nelm) {
    int16_t *origin = (int16_t *)data + offset;
    int16_t *itr;
    int32_t *target = res;

    if(ifcdevice->mode == ifcdaqdrv_acq_mode_smem) {
        if (ifcdevice->board_id == 0x3117) {
            for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; ++target, itr += ifcdevice->nchannels) {
                *target = (int16_t)*(itr + channel);
            }
        }
        else {
            for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; ++target, itr += ifcdevice->nchannels) {
                *target = (int16_t)(*(itr + channel) - 32768);
            }
        }
        return status_success;
    }

    if (ifcdevice->board_id == 0x3117) {
        for (itr = origin; itr < origin + nelm; ++target, ++itr) {
            *target = (int16_t)(*itr);
        }
    }
    else {
        for (itr = origin; itr < origin + nelm; ++target, ++itr) {
            *target = (int16_t)(*itr - 32768);
        }
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_switch_mode(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_acq_store_mode mode) {
    int32_t i32_reg_val;
    int32_t cs_reg;
    int32_t trig_reg;

    ifcdaqdrv_status status;

    /* Return immediately if device already is in correct mode */
    if(mode == ifcdevice->mode) {
        return status_success;
    }

    // CS register
    // - Move decimation
    // - Move Pre-trigger size
    // - Move decimation/averaging mode bit
    // Trigger register
    // - Move whole register

    switch(mode){
    case ifcdaqdrv_acq_mode_sram:
        INFOLOG(("Switching acquisition to SRAM mode\n"));
        break;
    case ifcdaqdrv_acq_mode_smem:
        INFOLOG(("Switching acquisition to SMEM mode\n"));
        break;
    }


    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
    cs_reg = i32_reg_val & (IFC_SCOPE_TCSR_CS_ACQ_Single_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK);
    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, &i32_reg_val);
    trig_reg = i32_reg_val;

    // Clear SCOPE app
    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_CS_REG, 0);
    // Clear SCOPE trigger
    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, 0);

    ifcdevice->mode = mode;


/* TODO: check the usage of register 0x63 (IFC_SCOPE_DTACQ_TCSR_GC) */

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

    if (status) {
        LOG((LEVEL_ERROR, "Error trying to access firmware registers (returned %d )", status));
        return status;
    }
  
    status = ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_CS_REG, cs_reg);
    status |= ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, trig_reg);

    if (status) {
        LOG((LEVEL_ERROR, "Error trying to access firmware registers (returned %d )", status));
        return status;
    }

    switch(mode){
    case ifcdaqdrv_acq_mode_sram:
        break;
    case ifcdaqdrv_acq_mode_smem:
        break;
    }

    return status;
}

/* The rules for setting number of samples:
 *
 * 1. SRAM is limited to even power-of-2 from 1k up to 16k or 32k samples.
 * 2. SMEM is limited to even MiB up to 256MiB which means:
 *    a) on DTACQ this is divided by 4*4 (nchannels * sample_size)
 *       lowest: 64 kibi
 *       highest: 16 Mibi
 *    b) on ADC311X this is either
 *       - If average == 1: divided by 2*2 (2 channels * sample_size)
 *         lowest:  256 kibi
 *         highest: 64 Mibi
 *       - Else: divided by 8*2 (8 channels * sample_size)
 *         lowest: 64 kibi
 *         highest: 16 Mibi
 *
 * The next series of IFC will support all 8 channels without average. Therefore
 * this is not implemented right now.
 */

ifcdaqdrv_status ifcdaqdrv_scope_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples)
{
    ifcdaqdrv_status status;
    
    // If samples fit in sram use sram.
    if (nsamples * ifcdevice->sample_size <= ifcdevice->sram_size) 
    {

        // WORK AROUND TO AVOID DMA FAILURES WHEN NSAMPLES < 4096
        if ((ifcdevice->board_id != 0x3117) && (nsamples < 4096)) {
            nsamples = 4096;
        }

        /* Check if nsamples is power of two */
        if ((nsamples & (nsamples - 1)) != 0) {
            return status_argument_range;
        }

        ifcdevice->mode_switch(ifcdevice, ifcdaqdrv_acq_mode_sram);
        status = ifcdaqdrv_scope_set_sram_nsamples(ifcdevice, nsamples);

        return status;
    }

/***************************************************************************
 *	SMEM MODE HAS NOT BEEN VALIDATED FOR IFC1410, ONLY SRAM IS FUNCTIONAL	
 ***************************************************************************/

#ifndef SMEM_MODE_ENABLED
  	return status_argument_range;

#else 
    uint32_t average;

    status = ifcdaqdrv_scope_get_average(ifcdevice, &average);
    if (status) {
        LOG((LEVEL_ERROR,"ifcdaqdrv_scope_get_average returned %d\n", status));
    } 
        
    if(ifcdevice->nchannels == 8 && average < 4) {
        INFOLOG(("Can't set average < 4 in SMEM mode\n"));
        return status_config;
    }

    // Check if it fits and is an even amount of Mibi.
    if(nsamples * ifcdevice->nchannels * ifcdevice->sample_size <= ifcdevice->smem_size &&
            !((nsamples * ifcdevice->nchannels * ifcdevice->sample_size) % (1024 * 1024))) {
        ifcdevice->mode_switch(ifcdevice, ifcdaqdrv_acq_mode_smem);
        return ifcdaqdrv_scope_set_smem_nsamples(ifcdevice, nsamples);
    }
    return status_argument_range;
#endif
}

ifcdaqdrv_status ifcdaqdrv_scope_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples)
{

    if(ifcdevice->mode == ifcdaqdrv_acq_mode_sram) {
        return ifcdaqdrv_scope_get_sram_nsamples(ifcdevice, nsamples);
    }

    return ifcdaqdrv_scope_get_smem_nsamples(ifcdevice, nsamples);
}

/* Valid number of pre-trigger samples are divisables with 8 */

ifcdaqdrv_status ifcdaqdrv_scope_set_npretrig(struct ifcdaqdrv_dev *ifcdevice, uint32_t npretrig)
{
    ifcdaqdrv_status      __attribute__((unused)) status;
    uint32_t              nsamples;
    uint32_t              ptq;

    /* Soft triggering doesn't work well enough with pre-trigger buffer */
    if(ifcdevice->trigger_type == ifcdaqdrv_trigger_soft && npretrig > 0) {
        return status_config;
    }

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    ptq = (npretrig * 8) / nsamples;
    if (ptq > 7 || (npretrig * 8) % nsamples != 0) {
        return status_argument_range;
    }

    return ifcdaqdrv_set_ptq(ifcdevice, ptq);
}


ifcdaqdrv_status ifcdaqdrv_scope_get_npretrig(struct ifcdaqdrv_dev *ifcdevice, uint32_t *npretrig)
{
    ifcdaqdrv_status      status;
    uint32_t              nsamples, ptq;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    nsamples /= 8;

    status    = ifcdaqdrv_get_ptq(ifcdevice, &ptq);
    if (status) {
        return status;
    }

    *npretrig = ptq * nsamples;

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_get_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average) {
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    
    if(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK) {
        *average = ifcdevice->averages[(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT];
        
        return status;
    }

    *average = 1;
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_set_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t average) {
    uint32_t              i;
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if(status){
        return status;
    }

    if (average == 0) {
        return status_argument_range;
    }

    /* Special case, ADC311X that is currently sampling to SMEM is not allowed to switch to average 1. */
    if (ifcdevice->mode == ifcdaqdrv_acq_mode_smem && average < 4) {
        return status_config;
    }

    // If average is 1 disable averaging
    if(average == 1){
        status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 0, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK);
        if(status) {
            return status;
        }

        status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 0, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
        return status;
    }

    // If averaging is not enabled and down-sampling is not 1, decimation is being used.
    // It is invalid configuration to have averaging and decimation at the same time.

    // if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK) && (i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK)) {
    //     printf("%s\n", );
    //     return status_config;
    // }

    for (i = 0; i < MAX_DECIMATIONS; ++i) {
        if (ifcdevice->averages[i] == average) {

            // Enable averaging.
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 1 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_SHIFT, 0);
            if(status) {
                return status;
            }

            // Check if averaging was successfully enabled, otherwise no support :(
            status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
            if(status) {
                return status;
            }

            if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK)) {
                return status_no_support;
            }

            // Set the avaraging factor.
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, i << IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
            return status;
        }
    }
    return status_argument_invalid;
}

ifcdaqdrv_status ifcdaqdrv_scope_init_smem_mode(struct ifcdaqdrv_dev *ifcdevice)
{
    ifcdaqdrv_status      status;

    ifcdevice->mode = ifcdaqdrv_acq_mode_smem;

    /* Set ACQ_downSMP_MOD to 1 (bit 15 = 1) */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 1 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_SHIFT, 0);
    if (status) return status;

    /* Set ACQ_downSMP to 000 (1:4 with averaging) */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 0, 0x7 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT);
    if (status) return status;

    LOG((5, "SCOPE application configured to SMEM mode\n"));
    return status_success;
}

/* Support functions to manipulate registers 0x61 / 0x62 (Data Acquisition Support ) */
#define IFC_SCOPE_FMC1_DTACQ_SUPPORT 0x01
#define IFC_SCOPE_FMC2_DTACQ_SUPPORT 0x02
#define IFC_SCOPE_DTACQ_CLR_CMD 0x01010101
#define IFC_SCOPE_DTACQ_RUNFIFO_CMD 0x06060606


ifcdaqdrv_status ifcdaqdrv_scope_smem_clearacq(struct ifcdaqdrv_dev *ifcdevice)
{
    ifcdaqdrv_status      status;

    /* Argument validation */
    if ((ifcdevice->fmc != 1) && (ifcdevice->fmc != 2)) 
    	return status_argument_range;

    /* Set CLR bits */
    if (ifcdevice->fmc == 1)
    	status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_FMC1_DTACQ_SUPPORT, IFC_SCOPE_DTACQ_CLR_CMD, 0xffffffff);
    else
    	status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_FMC2_DTACQ_SUPPORT, IFC_SCOPE_DTACQ_CLR_CMD, 0xffffffff);
    
    return status;
}

ifcdaqdrv_status ifcdaqdrv_scope_smem_configacq(struct ifcdaqdrv_dev *ifcdevice)
{
    ifcdaqdrv_status      status;

    /* Argument validation */
    if ((ifcdevice->fmc != 1) && (ifcdevice->fmc != 2)) 
    	return status_argument_range;

    /* Set CLR bits */
    if (ifcdevice->fmc == 1)
    	status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_FMC1_DTACQ_SUPPORT, IFC_SCOPE_DTACQ_RUNFIFO_CMD, 0xffffffff);
    else
    	status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_FMC2_DTACQ_SUPPORT, IFC_SCOPE_DTACQ_RUNFIFO_CMD, 0xffffffff);
    
    return status;
}



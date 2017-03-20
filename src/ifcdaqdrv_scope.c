#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>

#ifdef TOSCA_USRLIB
// #include <pevioctl.h>
// #include <pevxulib.h>
// #include <pevulib.h>
#include <tscioctl.h>
#include <tsculib.h>
#endif

#include <epicsThread.h>
#include <epicsTime.h>

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"
// #include "ifcdaqdrv_acq420.h"
#include "ifcdaqdrv_adc3110.h"
// #include "ifcdaqdrv_adc3112.h"

ifcdaqdrv_status ifcdaqdrv_scope_register(struct ifcdaqdrv_dev *ifcdevice){
    char *p;
    p = ifcdevice->fru_id->product_name;
    if (p) {
        // if (strcmp(p, "ACQ420FMC") == 0) {
        //     LOG((5, "Identified ACQ420FMC\n"));
        //     acq420_register(ifcdevice);
        // } else if (strcmp(p, "ADC3110") == 0) {
        //     LOG((5, "Identified ADC3110\n"));
        //     adc3110_register(ifcdevice);
        // } else if (strcmp(p, "ADC3111") == 0) {
        //     LOG((5, "Identified ADC3111\n"));
        //     adc3111_register(ifcdevice);
        // } else if (strcmp(p, "ADC3112") == 0) {
        //     LOG((5, "No support for ADC3112 yet\n"));
        // } else {
        //     LOG((5, "No recognized device %s\n", p));
        //     return status_incompatible;
        // }

        if (strcmp(p, "ADC3110") == 0) {
            LOG((5, "Identified ADC3110\n"));
            adc3110_register(ifcdevice);
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

#if 0
ifcdaqdrv_status ifc_scope_arm(struct ifcdaqdrv_dev *ifcdevice){
    // Function is never called
    return -1;
    int res = 0;

    int fmc_acq_reg; // fmc data acquistion support register

    fmc_acq_reg = ifcdevice->fmc == 1 ? 0x61 : 0x62;


    ifc_xuser_tcsr_write(ifcdevice, fmc_acq_reg, 0x01010101); // ACQ_CLR = 1 clear data acquisition DPRAM
    ifc_xuser_tcsr_write(ifcdevice, fmc_acq_reg, 0x00000000); // ACQ_CLR = 0

    ifc_xuser_tcsr_write(ifcdevice, fmc_acq_reg, 0x0E0E0E0E); // Run data acquisition DPRAM One Shot ACQ_RUN = 1 ACQ_MODE = 11


    uint32_t ACQ_HALT_MASK = 0x10101010;


    // TODO timeout ...
    // acquistion time depends on size and ADC clock ....

    // slowest: 10MHz with 16Ksps ~ 2ms

    int tmo = 100;   // 100 * 0.1ms = 10ms
    while (--tmo) {
        usleep(100); // wait 0.1ms

        int32_t status = ifc_xuser_tcsr_read(crate, fmc_acq_reg);

        // printf("fmc_acq_reg=%08x\n",status);

        // check ACQ_HALT
        if ((status & ACQ_HALT_MASK) == ACQ_HALT_MASK) {
            break;
        }
    }



    // check ACQ_HALT
    if ((ifc_xuser_tcsr_read(crate, fmc_acq_reg) & ACQ_HALT_MASK) == ACQ_HALT_MASK) {
        // ACQ_HALT = 1 -> acquisition stopped

        if (DBG(DBG_LEVEL3)) {
            printf("%s() acquisition done!\n", __FUNCTION__);
        }
    } else {
        // ACQ_HALT = 0 -> acquisition not stopped

        printf("Error: ifc_adc3110_ADS_acqdpram() acquisition timeout! (ACQ_HALT == 0) fmc_acq_reg=%08x\n",
               ifc_xuser_tcsr_read(crate, fmc_acq_reg));

        res = -1;
    }

    ifc_xuser_tcsr_write(crate, fmc_acq_reg, 0x00000000);   // clear ACQ_RUN bit's
}

return res;
}
#endif


#if 0 // TODO
ifcdaqdrv_status ifc_scope_vmeio_int(struct ifcdaqdrv_dev *ifcdevice, int enable, TRIGGERSLOPE slope, const
                                     char *vmeiosource){
    int res     = 0;
    int fmc_reg = 0x69;


    // set VME IO selector
    uint32_t vme_io_selector = 0;

    vmeioNameToSelector(vmeiosource, &vme_io_selector);

    vme_io_selector &= 0x7F; // 7 bits

    printf("%s: selector 0x%08x\n", __FUNCTION__, vme_io_selector);


    // read INTA and INTB selector
    int32_t regvalue = ifc_xuser_tcsr_read(ifcdevice, fmc_reg);


    printf("%s: read 0x%08x\n", __FUNCTION__, regvalue);

    int32_t mode = 0; // disabled

    // VME_Px_INTx_Sel[6:0]
    mode |= (vme_io_selector & 0x7F); // 7 bits

    // VME_Px_INTx_POL [7]
    if (slope == TSLOPE_POSITIVE) {
        // set bit 7
        mode |= (1 << 7);
    }

    // VME_Px_INTx_ENA
    if (enable) {
        // set bit 15
        mode |= (1 << 15);
    }


    printf("%s: mode 0x%08x\n", __FUNCTION__, mode);


    if (ifcdevice->fmc == 1) {
        // set INTA selector
        // bits [15:0]
        regvalue &= 0xFFFF0000;
        regvalue |= (mode & 0xFFFF);
    } else {
        // set INTB selector
        // bits [31:16]
        regvalue &= 0x0000FFFF;
        regvalue |= ((mode & 0xFFFF) << 16);
    }

    printf("%s: write 0x%08x\n", __FUNCTION__, regvalue);

    ifc_xuser_tcsr_write(ifcdevice, fmc_reg, regvalue);
    return res;
}
#endif

#if 0
ifcdaqdrv_status ifc_scope_set_trigger(struct ifcdaqdrv_dev *ifcdevice, TRIGGERSOURCE source, TRIGGERSLOPE slope,
                                       uint32_t level, const char *vmeiosource, int gpioGateEnable, int
                                       gpioGateActiveHigh){
    int res                 = 0;
    uint32_t tmode          = 0;     /* Trigger bitmask */
    uint32_t trig_level_ext = 0;     /* Bitmask for extended trigger for ACQ420 */
    uint16_t ui16_reg_val;

    int reg = IFC_SCOPE_TCSR_SRAMx_TRG_REG(ifcdevice->fmc);

    if (DBG(DBG_LEVEL2)) {
        printf("%s(source=%d,slope=%d,level=%u,gpioGateEnable=%d,gpioGateActiveHigh=%d)\n", __FUNCTION__, source, slope,
               level, gpioGateEnable, gpioGateActiveHigh);
    }


    // set GPIO gate and gatePol
    SETBIT(&tmode, 18, gpioGateEnable);
    SETBIT(&tmode, 19, gpioGateActiveHigh);

    switch (app) {
    case SAPP_XUSER_SCOPE_DTACQ:
        tmode |= (level >> 4) & IFC_SCOPE_TCSR_TRG_Trig_Level_MASK;
        if (fmc == IFC_FMC1) {
            trig_level_ext |= (level & 0xF) << 4;
        } else {
            trig_level_ext |= (level & 0xF) << 12;
        }
        break;
    default:
        tmode |= level & IFC_SCOPE_TCSR_TRG_Trig_Level_MASK;
        break;
    }
    tmode |= (slope & IFC_SCOPE_TCSR_TRG_Trig_Polarity_MASK);

    /*
     * Setting the trigger source is done using three fields:
     *   Trig_GPIO_Sel
     *     00 -> Enable Trig_Source. Use ADC channel level as trigger source.
     *     10 -> Use FMC trigger input as trigger source.
     *     11 -> Use VME_P0/2 as trigger source.
     *   Trig_GPIO_Gate (Use FMC trigger &(logical AND) ADC channel level trigger)
     *     0  -> Disabled
     *     1  -> Enabled
     *   Trig_Source
     *     Select ADC channel 0-7 (0-4) to trigger on.
     */
    switch (source) {
    case TSOURCE_EXT_GPIO:
        tmode |= (0x2 << 16);
        break;
    case TSOURCE_EXT_VMEIO:
        // set source VMEIO
        tmode |= (0x3 << 16);
        // printf("EXT_VMEIO: A 0x%08x\n",tmode);
        // set VME IO selector
        uint32_t vme_io_selector = 0;
        vmeioNameToSelector(vmeiosource, &vme_io_selector);
        vme_io_selector &= 0x7F;         // 7 bits
        // printf("EXT_VMEIO: selector 0x%08x\n",vme_io_selector);
        tmode           |= (vme_io_selector << 20);
        // printf("EXT_VMEIO: B 0x%08x\n",tmode);
        if (DBG(DBG_LEVEL5)) {
            printf("%s() set EXT-VME source selector=0x%02x!\n", __FUNCTION__, vme_io_selector);
        }
        break;
    default:
        // Shift in the channel number if triggering on channel
        tmode |= source << 28;
        break;
    }

    // Set trigger enable bit
    tmode |= IFC_SCOPE_TCSR_TRG_Trig_Enable_MASK;

    if (DBG(DBG_LEVEL3)) {
        printf("%s() trigger_reg=0x%02x  tmode=0x%08x\n", __FUNCTION__, reg, tmode);
    }

    ifc_xuser_tcsr_setclr(ifcdevice, reg, tmode, 0xFFFFFFFF);

    return res;
}

ifcdaqdrv_status ifc_scope_vmeio_init(struct ifcdaqdrv_dev *ifcdevice){
    // enable SERDES (doc: XUSER_SCOPE_UG_A0 4.2.1 TCSR FPGA Interconnect SERDES Controller

    int32_t serdes = 0;
    ifc_xuser_tcsr_read(ifcdevice, 0x24, &serdes); // GPIO TCSR SERDES

    if ((serdes & 0x02) == 0) {
        // SERDES not enabled

#if DEBUG
        printf("%s: GPIO SERDES =  0x%08x\n", __FUNCTION__, serdes);
        printf("%s: Enable IO SERDES..\n",    __FUNCTION__);
#endif

        serdes |= 0x3; // enable Serdes

        ifc_xuser_tcsr_write(ifcdevice, 0x24, serdes);
    }

    // enable ME_P2 user's IO controlled by the AP_Specific block (SCOPE)
    // set bit P2_APS_ENA to 0

    // set VME_P2 A/C/D and Z to Input
    ifc_xuser_tcsr_write(ifcdevice, 0x6, 0x00000000); // VME_P2_A_DIR all INPUT
    ifc_xuser_tcsr_write(ifcdevice, 0x9, 0x00000000); // VME_P2_C_DIR all INPUT
    ifc_xuser_tcsr_write(ifcdevice, 0xC, 0x00000000); // VME_P2_D_DIR all INPUT
    ifc_xuser_tcsr_write(ifcdevice, 0xF, 0x00000000); // VME_P2_Z_DIR all INPUT and P2_APS_ENA=0

    return 0;
}
#endif

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

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        ifcdaqdrv_scope_get_sram_nsamples(ifcdevice, &nsamples);
        for(channel = 0; channel < ifcdevice->nchannels; ++channel) {
            ifcdevice->read_ai_ch(ifcdevice, channel, ((int32_t *)data) + nsamples * channel);
        }
        break;
    case ifcdaqdrv_acq_mode_smem:
        offset = 0;

        status = ifcdaqdrv_scope_get_smem_nsamples(ifcdevice, &nsamples);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_read_smem_unlocked(ifcdevice, ifcdevice->all_ch_buf, ifcdevice->smem_dma_buf, offset, nsamples * ifcdevice->nchannels * ifcdevice->sample_size);
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

#if PRETRIG_ORGANIZE
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
#if DEBUG
        int32_t *itr;
        printf("%s(): u_base %p, acq_size %db, nsamples %d, npretrig %d, la %d, ptq %d, origin %p\n", __FUNCTION__,
                ifcdevice->smem_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, last_address, ptq,
                origin);

        printf("0x%08x: ", origin);
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
    int32_t          *origin;
    int32_t          *res;
    uint32_t          last_address,  nsamples,  npretrig,  ptq;

    offset = 0;
    origin = NULL;
    res = data;
    last_address = 0;
    nsamples = 0;
    npretrig = 0;
    ptq = 0;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        offset = IFC_SCOPE_SRAM_SAMPLES_OFFSET + (channel << 16);

        status = ifcdaqdrv_scope_get_sram_nsamples(ifcdevice, &nsamples);
        if (status) {
            return status;
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

        /* TODO: check the usage of dma_buf for TOSCA user library */
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

        status = ifcdaqdrv_scope_get_smem_nsamples(ifcdevice, &nsamples);
        if (status) {
            return status_device_access;
        }

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

#if DEBUG
    int32_t *itr;
    printf("%s(): u_base %p, acq_size %db, nsamples %d, npretrig %d, la %d, ptq %d, origin %p\n", __FUNCTION__,
            ifcdevice->sram_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, last_address, ptq,
            origin);

    printf("0x%08x: ", origin);
    for (itr = origin; itr < origin + 16; ++itr) {
        printf("%08x ", *itr);
    }
    printf("\n");
#endif

#if PRETRIG_ORGANIZE
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
// TODO Temporary include
//#include "ifcdaqdrv_acq420.h"

ifcdaqdrv_status ifcdaqdrv_scope_switch_mode(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_acq_store_mode mode) {
    int32_t i32_reg_val;
    int32_t cs_reg;
    int32_t trig_reg;
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
        break;
    case ifcdaqdrv_acq_mode_smem:
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

    /* Clear general control register and enable specific acquisition mode.. */
    if (ifcdevice->fmc == 1) {
        ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC1_ACQRUN_SHIFT, 0xffffffff);
    } else { /* fmc is FMC2 */
        ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC2_ACQRUN_SHIFT, 0xffffffff);
    }

    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_CS_REG, cs_reg);
    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, trig_reg);

    //ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
    //printf("CS REG after switch %08x\n", i32_reg_val);

    //ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, &i32_reg_val);
    //printf("TRIG REG after switch %08x\n", i32_reg_val);

    switch(mode){
    case ifcdaqdrv_acq_mode_sram:
        break;
    case ifcdaqdrv_acq_mode_smem:
        break;
    }

    return status_success;
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
    uint32_t average;

    // If samples fit in sram use sram.
    if (nsamples * ifcdevice->sample_size <= ifcdevice->sram_size) {

        /* Check if nsamples is power of two */
        if ((nsamples & (nsamples - 1)) != 0) {
            return status_argument_range;
        }

        ifcdevice->mode_switch(ifcdevice, ifcdaqdrv_acq_mode_sram);
        status = ifcdaqdrv_scope_set_sram_nsamples(ifcdevice, nsamples);

        return status;
    }

    ifcdaqdrv_scope_get_average(ifcdevice, &average);

    if(ifcdevice->nchannels == 8 && average < 4) {
        return status_config;
    }

    // Check if it fits and is an even amount of Mibi.
    if(nsamples * ifcdevice->nchannels * ifcdevice->sample_size <= ifcdevice->smem_size &&
            !((nsamples * ifcdevice->nchannels * ifcdevice->sample_size) % (1024 * 1024))) {
        ifcdevice->mode_switch(ifcdevice, ifcdaqdrv_acq_mode_smem);
        return ifcdaqdrv_scope_set_smem_nsamples(ifcdevice, nsamples);
    }
    return status_argument_range;
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
    ifcdaqdrv_status      status;
    uint32_t              nsamples;
    uint32_t              ptq;

    /* Soft triggering doesn't work well enough with pre-trigger buffer */
    if(ifcdevice->trigger_type == ifcdaqdrv_trigger_soft && npretrig > 0) {
        return status_config;
    }

    switch(ifcdevice->mode){
    case ifcdaqdrv_acq_mode_sram:
        status = ifcdaqdrv_scope_get_sram_nsamples(ifcdevice, &nsamples);
        break;
    case ifcdaqdrv_acq_mode_smem:
        status = ifcdaqdrv_scope_get_smem_nsamples(ifcdevice, &nsamples);
        break;
    default:
        return status_internal;
    }
    if (status) {
        return status;
    }

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

    switch(ifcdevice->mode){
    case ifcdaqdrv_acq_mode_sram:
        status = ifcdaqdrv_scope_get_sram_nsamples(ifcdevice, &nsamples);
        break;
    case ifcdaqdrv_acq_mode_smem:
        status = ifcdaqdrv_scope_get_smem_nsamples(ifcdevice, &nsamples);
        break;
    default:
        return status_internal;
    }
    if (status) {
        return status;
    }

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
    if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK) && (i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK)) {
        return status_config;
    }

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

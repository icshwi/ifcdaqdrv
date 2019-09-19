#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>

#include "tscioctl.h"
#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope_lite.h"
#include "ifcdaqdrv_adc3117.h"

ifcdaqdrv_status ifcdaqdrv_scope_lite_register(struct ifcdaqdrv_dev *ifcdevice)
{
    char *p;
    p = ifcdevice->fru_id->product_name;

    if (p) {
        if (strcmp(p, "ADC3117") == 0) {
            INFOLOG(("Identified ADC3117 on FMC %d\n", ifcdevice->fmc));
            adc3117_register(ifcdevice);
        }else {
            LOG((LEVEL_ERROR, "No recognized device %s\n", p));
            return status_incompatible;
        }
    } else {
        LOG((4, "Internal error, no product_name\n"));
        return status_internal;
    }

    ifcdevice->arm_device       = ifcdaqdrv_scope_lite_arm_device;
    ifcdevice->disarm_device    = ifcdaqdrv_scope_lite_disarm_device;
    ifcdevice->wait_acq_end     = ifcdaqdrv_scope_lite_wait_acq_end;
    ifcdevice->set_trigger      = ifcdaqdrv_scope_lite_set_trigger;
    ifcdevice->get_trigger      = ifcdaqdrv_scope_lite_get_trigger;
    ifcdevice->read_ai_ch       = ifcdaqdrv_scope_lite_read_ai_ch;
    ifcdevice->read_ai          = ifcdaqdrv_scope_lite_read_ai;
    ifcdevice->normalize_ch     = ifcdaqdrv_scope_lite_read_ch;
    ifcdevice->normalize        = ifcdaqdrv_scope_lite_read;
    ifcdevice->set_average      = NULL;
    ifcdevice->get_average      = NULL;
    ifcdevice->set_decimation   = NULL;
    ifcdevice->get_decimation   = NULL;
    ifcdevice->set_ptq          = NULL;
    ifcdevice->get_ptq          = NULL;

    ifcdevice->mode             = ifcdaqdrv_acq_mode_sram;
    ifcdevice->smem_size        = 4 * 1024 * 1024;
    ifcdevice->sram_size        = 2 * 1024 * ifcdevice->sample_size;
    ifcdevice->smem_sg_dma      = 0;

    return status_success;
}

/*
 * Arm device.
 *
 * Device can only be armed _after_ the ADCs has been initialized.
 *
 * Soft triggering has to estimate the amount of time the acquisition will take since it depends on the acquisition
 * length before it actually knows if it succeded.
 */
ifcdaqdrv_status ifcdaqdrv_scope_lite_arm_device(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;
    uint8_t               i; // iterator to test ACQ_CLKERR

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(ifcdevice->init_called == 0){
        LOG((LEVEL_WARNING,"WARNING: init_adc() has not been called\n"));
    }

    pthread_mutex_lock(&ifcdevice->lock);

    for (i = 0; i < 5; i++)
    {
        status = ifc_scope_lite_tcsr_read(ifcdevice, IFC_SCOPE_LITE_TCSR_FMC_STATUS_REG, &i32_reg_val);
        if ((i32_reg_val & 0x3) == 0x3) // Clock ready and init done
        {
            break;
        }
        usleep(20000);
    }

    /* if the "for" didn't break, then ACQ_CLKERR is still HIGH, which means that clock is not locked */
    if (i == 5)
    {
        LOG((LEVEL_ERROR,"Error: %s() ADC acquisition clock reference PLL is unlocked!\n", __FUNCTION__));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_unknown;
    }

    /* Arm device */
    status = ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CHANNEL_SEL_REG, 0xFFFFF000);
    status += ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG, 0x202);
    status += ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_SBUF_SELECT_REG, 0xFFFFF000);
    status += ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_SBUF_CONTROL_STATUS_REG, 0x2);

    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    ifcdevice->armed = 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Disarm device
 */
ifcdaqdrv_status ifcdaqdrv_scope_lite_disarm_device(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->armed)
        return status_success;

    pthread_mutex_lock(&ifcdevice->lock);

    ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_SBUF_CONTROL_STATUS_REG, 0x4);
    ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG, 0x0);

    ifcdevice->armed = 0;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Wait for acquisition to end.
 *
 * Currently implemented by polling device for "Acquisition ended". Should be implemented with interrupts.
 */
ifcdaqdrv_status ifcdaqdrv_scope_lite_wait_acq_end(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    do {
        status = ifc_scope_lite_tcsr_read(ifcdevice, IFC_SCOPE_LITE_TCSR_SBUF_CONTROL_STATUS_REG, &i32_reg_val);
        usleep(ifcdevice->poll_period);
    } while (!status && ifcdevice->armed && ((i32_reg_val & 0x00000080) != 0x00000080));

    if (!ifcdevice->armed) {
        return status_cancel;
    }

    ifcdaqdrv_disarm_device(ifcuser);
    return status;
}

/*
 * Set trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_scope_lite_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Forces SCOPE_LITE trigger to be software until we don't have the support in firmware */
    /* TODO: enable backplane trigger for SCOPE_LITE */
    if (trigger != ifcdaqdrv_trigger_soft) {
        LOG((LEVEL_ERROR, "ADC3117 only supports software (manual) trigger"));
        trigger = ifcdaqdrv_trigger_soft;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->trigger_type = trigger;

    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}

/*
 * Get trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_scope_lite_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (trigger) {
        *trigger = ifcdevice->trigger_type;
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data)
{
    uint32_t nsamples = 0;
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
        return status_no_support;
    }
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data)
{
    ifcdaqdrv_status status;
    int32_t offset = 0;
    int16_t *origin = NULL;
    int32_t *res = data;
    uint32_t nsamples = 0;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        offset = (IFC_SCOPE_LITE_SRAM_SAMPLES_OFFSET * ifcdevice->fmc) + (channel << 12);
        status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }

        if (ifcdaqdrv_is_byte_order_ppc())
            ifcdaqdrv_manualswap((uint16_t*) ifcdevice->sram_dma_buf->u_base,nsamples);

        origin   = ifcdevice->sram_dma_buf->u_base;
        break;
    case ifcdaqdrv_acq_mode_smem:
        return status_no_support;
    }

    status = ifcdevice->normalize_ch(ifcdevice, channel, res, origin, 0, nsamples);
    if (status) {
        return status;
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_lite_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples)
{
    int32_t *target;
    int16_t *itr;
    int16_t *origin;
    int16_t channel;

    target = ((int32_t *)dst) + dst_offset;
    origin = ((int16_t *)src) + src_offset * ifcdevice->nchannels;

    for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; target += 2, itr += (ifcdevice->nchannels * 2)) {
        for (channel = 0; channel < ifcdevice->nchannels; channel++) {
            *((target + 0) + channel * channel_nsamples) = (int16_t)*(itr + channel * 2);
            *((target + 1) + channel * channel_nsamples) = (int16_t)*(itr + (channel * 2 + 1));
        }
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm)
{
    int16_t *origin = (int16_t *)data + offset;
    int16_t *itr;
    int32_t *target = res;

    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = (int16_t)(*itr);
    }

    return status_success;
}

/* Functions for accessing 0x60 to 0x6F (SCOPE LITE MAIN TCSR) */
ifcdaqdrv_status ifc_scope_lite_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_lite_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, value);
}

ifcdaqdrv_status ifc_scope_lite_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, setmask, clrmask);
}

/* Functions for accessing 0x70-0x7A (SCOPE LITE Waveform generator) registers */
ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, 0x70 + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, 0x70 + register_idx, value);
}

ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, 0x70 + register_idx, setmask, clrmask);
}

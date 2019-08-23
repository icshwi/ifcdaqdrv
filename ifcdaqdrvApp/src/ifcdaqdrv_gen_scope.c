#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include "tscioctl.h"
#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3117.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_gen_scope.h"

static const uint32_t decimations[] = {1, 2, 4, 8, 16, 32, 64, 0};
static const uint32_t averages[] = {1, 2, 4, 8, 16, 32, 64, 0};

ifcdaqdrv_status ifcdaqdrv_gen_scope_register(struct ifcdaqdrv_dev *ifcdevice)
{
    int32_t i = 0;
    char *p;
    p = ifcdevice->fru_id->product_name;

    if (p) {
        if (strcmp(p, "ADC3110") == 0) {
            INFOLOG(("Identified ADC3110 on FMC %d\n", ifcdevice->fmc));
            adc3110_register(ifcdevice);
            ifcdevice->nchannels = 8;
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
            ifcdevice->nchannels = 20;
        } else {
            LOG((LEVEL_ERROR, "No recognized device %s\n", p));
            return status_incompatible;
        }
    } else {
        LOG((4, "Internal error, no product_name\n"));
        return status_internal;
    }

    ifcdevice->arm_device       = ifcdaqdrv_gen_scope_arm_device;
    ifcdevice->disarm_device    = ifcdaqdrv_gen_scope_disarm_device;
    ifcdevice->wait_acq_end     = ifcdaqdrv_gen_scope_wait_acq_end;
    ifcdevice->set_trigger      = ifcdaqdrv_gen_scope_set_trigger;
    ifcdevice->get_trigger      = ifcdaqdrv_gen_scope_get_trigger;
    ifcdevice->set_average      = ifcdaqdrv_gen_scope_set_average;
    ifcdevice->get_average      = ifcdaqdrv_gen_scope_get_average;
    ifcdevice->set_decimation   = ifcdaqdrv_gen_scope_set_decimation;
    ifcdevice->get_decimation   = ifcdaqdrv_gen_scope_get_decimation;
    ifcdevice->set_ptq          = ifcdaqdrv_gen_scope_set_ptq;
    ifcdevice->get_ptq          = ifcdaqdrv_gen_scope_get_ptq;
    ifcdevice->set_nsamples     = ifcdaqdrv_gen_scope_set_nsamples;
    ifcdevice->get_nsamples     = ifcdaqdrv_gen_scope_get_nsamples;
    ifcdevice->read_ai_ch       = ifcdaqdrv_gen_scope_read_ai_ch;
    ifcdevice->read_ai          = ifcdaqdrv_gen_scope_read_ai;
    ifcdevice->normalize_ch     = ifcdaqdrv_gen_scope_read_ch;
    ifcdevice->normalize        = ifcdaqdrv_gen_scope_read;
    ifcdevice->mode_switch      = NULL;
    ifcdevice->set_trigger_threshold      = ifcdaqdrv_gen_scope_set_trigger_threshold;
    ifcdevice->get_trigger_threshold      = ifcdaqdrv_gen_scope_get_trigger_threshold;

    if ((ifcdevice->app_signature & IFCDAQDRV_APP_SIGNATURE_MASK) == 0x12350000)
        ifcdevice->mode             = ifcdaqdrv_acq_mode_smem;
    else
        ifcdevice->mode             = ifcdaqdrv_acq_mode_sram;

    ifcdevice->smem_size        = 32 * 1024 * ifcdevice->sample_size;
    ifcdevice->sram_size        = 32 * 1024 * ifcdevice->sample_size;
    ifcdevice->smem_sg_dma      = 0;

    for (i = 0; i < MAX_DECIMATIONS; i++) {
        ifcdevice->decimations[i] = 0;
        ifcdevice->averages[i] = 0;
    }
    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    return status_success;
}

/* When the device is in soft trigger mode, the arm function will try to manually trigger the device a limited amount of
 * times. The following number is the least amount of retries. */
#define SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES 10000

/*
 * Arm device.
 *
 * Device can only be armed _after_ the ADCs has been initialized.
 *
 * Soft triggering has to estimate the amount of time the acquisition will take since it depends on the acquisition
 * length before it actually knows if it succeded.
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_arm_device(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;
    //uint8_t               i; // iterator to test ACQ_CLKERR

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(ifcdevice->init_called == 0){
        LOG((LEVEL_WARNING,"WARNING: init_adc() has not been called\n"));
    }

    pthread_mutex_lock(&ifcdevice->lock);

    ifc_gen_scope_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, 0x0);
    ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, &i32_reg_val);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x20000000);
    usleep(2000);
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x40000000);
    usleep(2000);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010000);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10001000);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010001);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10021002);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010002);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10031003);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010003);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10041004);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010004);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10051005);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010005);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10061006);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010006);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10071007);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0x00010007);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); // Flush write cycle
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG, 0x10081008);

    status = ifc_gen_scope_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, 0x900000FF);
    usleep(2000);
    if (status) {
        INFOLOG(("Enh scope arm device failed 1!\n"));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, 0xC0010000);
    usleep(2000);
    ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val); /* Flush write cycle */

    do {
        usleep(1000);
        ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val);
    } while (i32_reg_val & 0x40000000);
    INFOLOG(("RG BUF clear is done 0x70 val %08x\n", i32_reg_val));

    ifc_gen_scope_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, 0x80000000);

    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x20000000);
    usleep(2000);
    ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x40000000);
    usleep(2000);

    INFOLOG(("Enh scope arm device!\n"));
    status = ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x10000001);
    //usleep(2000);
    if (status) {
        INFOLOG(("Enh scope arm device failed 2!\n"));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);
    INFOLOG(("Enh scope device armed! %08x\n", i32_reg_val));

#if 0
    /* If software trigger is selected, try manually trigger. */
    if (ifcdevice->trigger_type == ifcdaqdrv_trigger_soft) {
        /* Estimate time to complete sample */
        double acquisition_time;
        double frequency;
        uint32_t divisor;
        uint32_t nsamples;
        uint32_t average;
        uint32_t decimation;
        int32_t timeo;
        ifcdevice->get_clock_frequency(ifcdevice, &frequency);
        ifcdevice->get_clock_divisor(ifcdevice, &divisor);
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
        ifcdaqdrv_scope_get_average(ifcdevice, &average);
        ifcdaqdrv_get_decimation(ifcuser, &decimation);

        acquisition_time = ((nsamples * average * decimation) / (frequency / divisor));

        /* Poll for the expected acquisition time before giving up */
        timeo = SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES + acquisition_time / (ifcdevice->poll_period * 1e-6);
        do {
            status  = ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_LA_REG, 1 << IFC_SCOPE_TCSR_LA_Spec_CMD_SHIFT);
            usleep(ifcdevice->poll_period);
            status |= ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
        } while (!status && ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT) < 2 && timeo-- > 0);

        if (status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_device_access;
        }

        if(((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT) < 2) {
            // Failed to self-trigger.
            LOG((LEVEL_INFO, "CS register value is %08x after %d iterations\n", i32_reg_val, (int32_t) (SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES + acquisition_time / (ifcdevice->poll_period * 1e-6))));
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
    }
#endif
    ifcdevice->armed = 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Disarm device
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_disarm_device(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;
    int32_t               i;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->armed)
        return status_success;

    pthread_mutex_lock(&ifcdevice->lock);

    /* Set "Acquisition STOP/ABORT" bit and "ACQ mode single" bit. Single
     * mode has to be set to disable a continuous acquisition. */
    status = ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x20000001);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);

    if(i32_reg_val & 0xF0000000) {
        // If ACQ_Status hasn't gone to idle (0) disarming failed.
        // Try ten times to disarm the board and then report internal error.
        for(i = 0; i < 10; ++i) {
            status = ifc_gen_scope_acq_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, 0x40000001);
            if (status) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_access;
            }
            status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);
            if(!(i32_reg_val & 0xF0000000)) {
                goto disarm_success;
            }
        }

        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

disarm_success:
    /* Should we disble FE here? */
    ifc_gen_scope_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, 0x00000000);
    ifcdevice->armed = 0;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Wait for acquisition to end.
 *
 * Currently implemented by polling device for "Acquisition ended". Should be implemented with interrupts.
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_wait_acq_end(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    ifc_gen_scope_tcsr_write(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, 0x40000000); //Manual trigger for testing
    INFOLOG(("Enh scope wait acq end!\n"));

    do {
        status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);
        //INFOLOG(("Enh scope acq wait loop! %08x\n", i32_reg_val));
        usleep(ifcdevice->poll_period);
    } while (!status && ifcdevice->armed && ((i32_reg_val & 0xF0000000) != 0x60000000));
    INFOLOG(("Enh scope acq end! %08x\n", i32_reg_val));

    if (!ifcdevice->armed) {
        return status_cancel;
    }

    ifcdaqdrv_disarm_device(ifcuser);
    return status;
}

/*
 * Set trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge)
{
    ifcdaqdrv_status      status       = 0;
    struct ifcdaqdrv_dev *ifcdevice;
    //int32_t               i32_cs_val   = 0; // Control & Status value
    int32_t               i32_trig_val = 0; // Trigger value
    uint32_t              channel      = 0;
    uint32_t              gpio         = 0;
    uint32_t              channel_mask = 0;
    uint32_t              ptq          = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    switch (trigger) {
    /* Changed mask register to adapt for mTCA instead of VME */
    case ifcdaqdrv_trigger_backplane:
        //i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger
        i32_trig_val |= ((mask & 0x03) | IFC_GEN_SCOPE_AMC_RX_IN_SELECT) << 16;  // Set MTCA line
        i32_trig_val |= 1 << 23;           // Set backplane trigger
        if (rising_edge & 0x7FFFFFFF) {
            i32_trig_val |= 2 << 24; /* Positive edge */
        } else {
            i32_trig_val |= 3 << 24; /* Negative edge */
        }
        break;
    case ifcdaqdrv_trigger_frontpanel:
        gpio         = mask & 0x40000000;
        channel_mask = mask & 0x3fffffff;
        while (channel_mask >>= 1) {
            ++channel;
        }

        if (channel >= ifcdevice->nchannels) {
            return status_argument_range;
        }

        //i32_cs_val   = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger
        if (!gpio) {
            /* Trigger only on channel */
            i32_trig_val |= channel << 16;
            if (rising_edge & (1 << channel)) {
                i32_trig_val |= 2 << 24; /* Positive edge */
            } else {
                i32_trig_val |= 3 << 24; /* Negative edge */
            }
        } else {
            /* Trigger only on GPIO */
            i32_trig_val |= 1 << 23;       // Set GPIO trigger
            if (ifcdevice->fmc == 1)
                i32_trig_val |= 0xC << 16;
            else
                i32_trig_val |= 0xD << 16;
            if (rising_edge & 0x40000000) {
                i32_trig_val |= 2 << 24; /* Positive edge */
            } else {
                i32_trig_val |= 3 << 24; /* Negative edge */
            }
        }
        break;
    case ifcdaqdrv_trigger_auto: // Auto is not supported anymore (will be interpreted as soft trigger)
    case ifcdaqdrv_trigger_soft:
        status = ifcdaqdrv_gen_scope_get_ptq(ifcdevice, &ptq);
        if(status) {
            return status;
        }
        if(ptq != 0) {
            // It doesn't make sense to have pre-trigger samples when triggering manually.
            return status_config;
        }

    /* Treat trigger_none as soft trigger */    
    case ifcdaqdrv_trigger_none:
    default:
        //i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        break;
    }

    LOG((LEVEL_DEBUG, "Will set trig val 0x%08x\n", i32_trig_val));

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->trigger_type = trigger;

    /*status = ifc_gen_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_TCSR_CS_REG, i32_cs_val, IFC_SCOPE_TCSR_CS_ACQ_Single_MASK | IFC_SCOPE_TCSR_CS_ACQ_Auto_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }*/

    status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, i32_trig_val, 0xFFFFFFFF);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    if (ifcdevice->set_trigger_threshold) {
        status = ifcdevice->set_trigger_threshold(ifcdevice, threshold);
    }

#if DEBUG
    /* This is interesting because set_trigger_threshold may modify the content of trigger register */
    ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, &i32_trig_val);
    LOG((LEVEL_INFO, "Is set trig val %08x\n", i32_trig_val));
#endif

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}

/*
 * Get trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge)
{
    ifcdaqdrv_status status = 0;
    int32_t i32_trig_val = 0; // Trigger value
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Lock device so that the trigger configuration can be read out atomically. */
    pthread_mutex_lock(&ifcdevice->lock);

    status = ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, &i32_trig_val);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    pthread_mutex_unlock(&ifcdevice->lock);

    if (trigger) {
        *trigger = ifcdevice->trigger_type;
    }

    if (threshold && ifcdevice->get_trigger_threshold) {
        status = ifcdevice->get_trigger_threshold(ifcdevice, threshold);
        if (status) {
            return status;
        }
    }

    if (mask && rising_edge) {
        *mask = 0;
        *rising_edge = 0;
        if (((i32_trig_val >> 23) & 1) == 1) { /* Digital trigger */
            if (ifcdevice->trigger_type == ifcdaqdrv_trigger_backplane) {
                *mask = 0x80000000 | ((i32_trig_val >> 16) & 3);
                *rising_edge = 0x80000000 | ((i32_trig_val >> 16) & 3);
            } else { /* GPIO */
                *mask = 0x40000000;
                *rising_edge = 0x40000000;
            }
        } else { /* Analog trigger */
            *mask = 1 << ((i32_trig_val >> 16) & 0x7F);
            if (((i32_trig_val >> 24) & 3) == 2)
                *rising_edge = 1 << ((i32_trig_val >> 16) & 0x7F);
        }
    } else {
        return status_argument_invalid;
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold)
{
    uint16_t ui16_reg_val = (uint16_t)threshold + 32768;

    return ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, ui16_reg_val & 0xFFFF, 0xFFFF);
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold)
{
    ifcdaqdrv_status status = 0;
    int32_t          threshold_adc;
    int32_t          i32_reg_val;

    status = ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG, &i32_reg_val);
    if (status) {
        return status;
    }
    threshold_adc = (i32_reg_val & 0xFFFF) - 32768;

    /* Sign extension */
    if (threshold_adc & 0x8000) {
        threshold_adc |= 0xFFFF0000;
    }

    *threshold = threshold_adc;

    return status;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples)
{
    int32_t i32_reg_val;
    int status;

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG, &i32_reg_val);
    if (status) {
        return status;
    }

    switch ((i32_reg_val & IFC_GEN_SCOPE_DPRAM_BUF_SIZE_MASK) >> 10) {
    case 0:
        *nsamples = 16 * 1024;
        break;
    case 1:
        *nsamples = 32 * 1024;
        break;
    case 2:
        *nsamples = 64 * 1024;
        break;
    case 3:
        *nsamples = 128 * 1024;
        break;
    default:
        return status_internal;
    }
    LOG((5, "acq %d, reg_val %08x\n", *nsamples, i32_reg_val));

    return status;
}

/*
 * Get Number of samples per channel
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples)
{
    if(!ifcdevice || !ifcdevice->smem_size || !ifcdevice->sample_size) {
        return status_internal;
    }

    *nsamples = ifcdevice->smem_size / ifcdevice->sample_size;

    return status_success;
}

/*
 * Set Number of samples per channel in SMEM mode.
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples)
{
    if(!ifcdevice) {
        return status_internal;
    }

    ifcdevice->smem_size = nsamples * ifcdevice->sample_size;

    return status_success;
}

/* Returns offset in ring buffer of the sample causing the trigger. */
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger_marker(struct ifcdaqdrv_dev *ifcdevice, uint32_t *trigger_marker)
{
    ifcdaqdrv_status status = 0;
    int32_t i32_reg_val;

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_TRIGGER_MARKER_REG, &i32_reg_val);
    if (status)
        return status_device_access;

    *trigger_marker = (i32_reg_val & IFC_GEN_SCOPE_TRIGGER_MARKER_MASK) >> 2;

    return status;
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq)
{
    if (ptq < 0 || ptq > 7) {
        return status_argument_range;
    }

    return ifc_gen_scope_acq_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, ptq << 5,
                                 IFC_GEN_SCOPE_ACQ_BUFFER_MODE_MASK);
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq)
{
    ifcdaqdrv_status status = 0;
    int32_t i32_reg_val;

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);
    if (status) {
        return status;
    }
    if (!ptq) {
        return status_argument_invalid;
    }

    *ptq = ((i32_reg_val & IFC_GEN_SCOPE_ACQ_BUFFER_MODE_MASK) >> 5);

    return status;
}

/*
 * Set number of samples
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples)
{
    if (nsamples != 2 * 1024 &&
        nsamples != 4 * 1024 &&
        nsamples != 8 * 1024 &&
        nsamples != 16 * 1024 &&
        nsamples != 32 * 1024 &&
        nsamples != 64 * 1024 &&
        nsamples != 128 * 1024)
    {
        return status_argument_invalid;
    }

    if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram)
    {
        return status_no_support;
    }
    else if (ifcdevice->mode == ifcdaqdrv_acq_mode_smem)
    {
        return ifcdaqdrv_gen_scope_set_smem_nsamples(ifcdevice, nsamples);
    }

    return status_internal;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples)
{
    if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram)
    {
        return ifcdaqdrv_gen_scope_get_sram_nsamples(ifcdevice, nsamples);
    }
    else if (ifcdevice->mode == ifcdaqdrv_acq_mode_smem)
    {
        return ifcdaqdrv_gen_scope_get_smem_nsamples(ifcdevice, nsamples);
    }

    return status_internal;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average)
{
    ifcdaqdrv_status status = 0;
    int32_t i32_reg_val;

    status = ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, &i32_reg_val);
    
    if(i32_reg_val & IFC_GEN_SCOPE_FE_AVERAGING_MASK) {
        *average = ifcdevice->averages[(i32_reg_val & IFC_GEN_SCOPE_FE_PRESCL_MASK) >> 24];
        return status;
    }

    *average = 1;
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_set_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t average)
{
    uint32_t              i = 0;
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    if (average == 0) {
        return status_argument_range;
    }

    // If average is 1 disable averaging
    if(average == 1){
        status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, 0 << 27, IFC_GEN_SCOPE_FE_AVERAGING_MASK);
        if(status) {
            return status;
        }

        status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, 0 << 24, IFC_GEN_SCOPE_FE_PRESCL_MASK);
        return status;
    }

    while (ifcdevice->averages[i] != 0) {
        if (ifcdevice->averages[i] == average) {

            // Enable averaging.
            status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, 1 << 27, IFC_GEN_SCOPE_FE_AVERAGING_MASK);
            if(status) {
                return status;
            }

            // Check if averaging was successfully enabled, otherwise no support :(
            status = ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, &i32_reg_val);
            if(status) {
                return status;
            }

            if(!(i32_reg_val & IFC_GEN_SCOPE_FE_AVERAGING_MASK)) {
                return status_no_support;
            }

            // Set the avaraging factor.
            status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, i << 24, IFC_GEN_SCOPE_FE_PRESCL_MASK);
            return status;
        }
        i++;
    }
    return status_argument_invalid;
}


/*
 * Set decimation
 * 
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation)
{
    uint32_t              i = 0;
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (decimation == 0) {
        return status_argument_invalid;
    }

    // Decimation is not supported when averaging is enabled. 
    status = ifc_gen_scope_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, &i32_reg_val);
    if(status) {
        return status;
    }
    if((decimation != 1) && (i32_reg_val & IFC_GEN_SCOPE_FE_AVERAGING_MASK)) {
        return status_config;
    }

    while (ifcdevice->decimations[i] != 0) {
        if (ifcdevice->decimations[i] == decimation) {
            pthread_mutex_lock(&ifcdevice->lock);

            if (ifcdevice->armed) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_armed;
            }

            status = ifc_gen_scope_tcsr_setclr(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, i << 24, IFC_GEN_SCOPE_FE_PRESCL_MASK);
	    
            pthread_mutex_unlock(&ifcdevice->lock);
            return status;
        }
        i++;
    }
    return status_argument_invalid;
}

/*
 * Get decimation
 */
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation)
{
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!decimation) {
        return status_argument_invalid;
    }

    status = ifc_gen_scope_acq_tcsr_read(ifcdevice, IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG, &i32_reg_val);
    if (status)
    {
        return status;
    }

    if(!(i32_reg_val & IFC_GEN_SCOPE_FE_AVERAGING_MASK)) {
        *decimation = ifcdevice->decimations[(i32_reg_val & IFC_GEN_SCOPE_FE_PRESCL_MASK) >> 24];
        return status;
    }

    *decimation = 0;
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data)
{
    ifcdaqdrv_status      status;
    int32_t               offset       = 0;
    int32_t              *origin;
    int32_t              *res          = data;
    uint32_t              trigger_marker = 0, nsamples = 0, npretrig = 0, ptq = 0;
    uint32_t channel;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        nsamples = 32 * 1024;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        for(channel = 0; channel < ifcdevice->nchannels; ++channel) {
            ifcdevice->read_ai_ch(ifcdevice, channel, ((int32_t *)data) + nsamples * channel);
        }
        break;
    case ifcdaqdrv_acq_mode_smem:
        offset = 0x10010000;

        ifcdaqdrv_start_tmeas();
        status = ifcdaqdrv_read_smem_unlocked(ifcdevice, ifcdevice->all_ch_buf, ifcdevice->smem_dma_buf, offset, nsamples * ifcdevice->nchannels * ifcdevice->sample_size);
        if (status) {
            return status;
        }
        ifcdaqdrv_end_tmeas();
        

        status = ifcdaqdrv_gen_scope_get_trigger_marker(ifcdevice, &trigger_marker);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_gen_scope_get_ptq(ifcdevice, &ptq);
        if (status) {
            return status_device_access;
        }

        origin   = ifcdevice->all_ch_buf;
        npretrig = (nsamples * ptq) / 8;

#if PRETRIG_ORGANIZE

        ifcdaqdrv_start_tmeas();

        // * For ADC311X increase Last address with 2 (because there are 2 samples per "acquisition block" in DDR).
        int32_t samples_per_block = 2;
        if (npretrig > 0) {
            /* Copy from "Last Address + 1 sample block" to end of pretrig buffer */
            status = ifcdevice->normalize(ifcdevice, res, 0, origin, (trigger_marker + samples_per_block), npretrig - (trigger_marker + samples_per_block), nsamples);
            if (status) {
                return status;
            }

            /* Copy from 0 to Last Address + 1 sample block */
            status = ifcdevice->normalize(ifcdevice, res, npretrig - (trigger_marker + samples_per_block), origin, 0, trigger_marker + samples_per_block, nsamples);
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
                ifcdevice->smem_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, trigger_marker, ptq,
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

ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data)
{
    ifcdaqdrv_status  status;
    int32_t           offset;
    int16_t          *origin;
    int32_t          *res;
    uint32_t          trigger_marker,  nsamples,  npretrig,  ptq;


    offset = 0;
    origin = NULL;
    res = data;
    trigger_marker = 0;
    nsamples = 0;
    npretrig = 0;
    ptq = 0;

    if (ifcdevice->get_nsamples)
        ifcdevice->get_nsamples(ifcdevice, &nsamples);
    else
        return status_no_support;

    switch(ifcdevice->mode) {
    case ifcdaqdrv_acq_mode_sram:
        offset = IFC_GEN_SCOPE_SRAM_SAMPLES_OFFSET + (channel << 16);
        status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }
        
        status = ifcdaqdrv_gen_scope_get_trigger_marker(ifcdevice, &trigger_marker);
        if (status) {
            return status;
        }

        status = ifcdaqdrv_gen_scope_get_ptq(ifcdevice, &ptq);
        if (status) {
            return status;
        }

        if (ifcdaqdrv_is_byte_order_ppc())
            ifcdaqdrv_manualswap((uint16_t*) ifcdevice->sram_dma_buf->u_base,nsamples);

        origin   = ifcdevice->sram_dma_buf->u_base;
        npretrig = (nsamples * ptq) / 8;
        break;
    

    case ifcdaqdrv_acq_mode_smem:
        offset = 0;

        status = ifcdaqdrv_read_smem_unlocked(ifcdevice, ifcdevice->all_ch_buf, ifcdevice->smem_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }

        status = ifcdaqdrv_gen_scope_get_trigger_marker(ifcdevice, &trigger_marker);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_gen_scope_get_ptq(ifcdevice, &ptq);
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
            ifcdevice->sram_dma_buf->u_base, nsamples * ifcdevice->sample_size, nsamples, npretrig, trigger_marker, ptq,
            origin);

    printf("0x%08x: ", (int16_t) origin);
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
        status = ifcdevice->normalize_ch(ifcdevice, channel, res, origin, trigger_marker + 1, npretrig - (trigger_marker + 1));
        if (status) {
            return status;
        }

        /* Copy from 0 to Last Address + 1 */
        status = ifcdevice->normalize_ch(ifcdevice, channel, res + (npretrig - (trigger_marker + 1)), origin, 0,
                trigger_marker + 1);
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

ifcdaqdrv_status ifcdaqdrv_gen_scope_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples)
{
    int32_t *target; /* Copy to this address */
    int16_t *itr;    /* Iterator for iterating over "data" */
    int16_t *origin; /* Copy from this address */
    int16_t channel;

    /* Multiply offsets by number of channels */
    target = ((int32_t *)dst) + dst_offset;
    origin = ((int16_t *)src) + src_offset * ifcdevice->nchannels;

    for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; target += 2, itr += (ifcdevice->nchannels * 2)) {
        for (channel = 0; channel < ifcdevice->nchannels; channel++) {
            *((target + 0) + channel * channel_nsamples) = (int16_t)(*(itr + channel * 2) - 32768);
            *((target + 1) + channel * channel_nsamples) = (int16_t)(*(itr + (channel * 2 + 1)) - 32768);
        }
    }

    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                              size_t nelm)
{
    int16_t *origin = (int16_t *)data + offset;
    int16_t *itr;
    int32_t *target = res;

    if(ifcdevice->mode == ifcdaqdrv_acq_mode_smem) {
        for (itr = origin; itr < origin + nelm * ifcdevice->nchannels; ++target, itr += ifcdevice->nchannels) {
            *target = (int16_t)(*(itr + channel) - 32768);
        }
        return status_success;
    }

    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = (int16_t)(*itr - 32768);
    }

    return status_success;
}

/* Functions for accessing GEN SCOPE MAIN TCSR (0x60 to 0x6F) */
ifcdaqdrv_status ifc_gen_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_tcsr_offset(ifcdevice), i32_reg_val);
}

ifcdaqdrv_status ifc_gen_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_tcsr_offset(ifcdevice), value);
}

ifcdaqdrv_status ifc_gen_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_tcsr_offset(ifcdevice), setmask, clrmask);
}

/* Functions for accessing GEN SCOPE ACQ TCSR (0x70 to 0x7F) */
ifcdaqdrv_status ifc_gen_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_acq_tcsr_offset(ifcdevice), i32_reg_val);
}

ifcdaqdrv_status ifc_gen_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_acq_tcsr_offset(ifcdevice), value);
}

ifcdaqdrv_status ifc_gen_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_gen_scope_acq_tcsr_offset(ifcdevice), setmask, clrmask);
}


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
#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_adc3112.h"

static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200, 0};
static const uint32_t averages[] = {1, 4, 8, 16, 32, 64, 128, 256, 0};

ifcdaqdrv_status ifcdaqdrv_scope_register(struct ifcdaqdrv_dev *ifcdevice)
{
    uint32_t i = 0;
    char *p;
    p = ifcdevice->fru_id->product_name;

    if (p) {
        if (strcmp(p, "ADC3110") == 0) {
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
        }else {
            LOG((LEVEL_ERROR, "No recognized device %s - but will force ADC3117\n", p));
            return status_incompatible;
        }
    } else {
        LOG((4, "Internal error, no product_name\n"));
        return status_internal;
    }

    ifcdevice->arm_device       = ifcdaqdrv_scope_arm_device;
    ifcdevice->disarm_device    = ifcdaqdrv_scope_disarm_device;
    ifcdevice->wait_acq_end     = ifcdaqdrv_scope_wait_acq_end;
    ifcdevice->set_trigger      = ifcdaqdrv_scope_set_trigger;
    ifcdevice->get_trigger      = ifcdaqdrv_scope_get_trigger;
    ifcdevice->set_average      = ifcdaqdrv_scope_set_average;
    ifcdevice->get_average      = ifcdaqdrv_scope_get_average;
    ifcdevice->set_decimation   = ifcdaqdrv_scope_set_decimation;
    ifcdevice->get_decimation   = ifcdaqdrv_scope_get_decimation;
    ifcdevice->set_ptq          = ifcdaqdrv_scope_set_ptq;
    ifcdevice->get_ptq          = ifcdaqdrv_scope_get_ptq;
    ifcdevice->set_nsamples     = ifcdaqdrv_scope_set_nsamples;
    ifcdevice->get_nsamples     = ifcdaqdrv_scope_get_nsamples;
    ifcdevice->read_ai_ch       = ifcdaqdrv_scope_read_ai_ch;
    ifcdevice->read_ai          = ifcdaqdrv_scope_read_ai;
    ifcdevice->normalize_ch     = ifcdaqdrv_scope_read_ch;
    ifcdevice->normalize        = ifcdaqdrv_scope_read;
    ifcdevice->mode_switch      = ifcdaqdrv_scope_switch_mode;
    ifcdevice->set_trigger_threshold      = ifcdaqdrv_scope_set_trigger_threshold;
    ifcdevice->get_trigger_threshold      = ifcdaqdrv_scope_get_trigger_threshold;

    ifcdevice->mode             = ifcdaqdrv_acq_mode_sram;
    ifcdevice->smem_size        = 4 * 1024 * 1024;
    ifcdevice->sram_size        = 32 * 1024 * ifcdevice->sample_size;
    ifcdevice->smem_sg_dma      = 0;

    for (i = 0; i < MAX_DECIMATIONS; i++) {
        ifcdevice->decimations[i] = 0;
        ifcdevice->averages[i] = 0;
    }

    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    /* Remove these */
    ifcdevice->set_npretrig     = ifcdaqdrv_scope_set_npretrig;
    ifcdevice->get_npretrig     = ifcdaqdrv_scope_get_npretrig;

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
ifcdaqdrv_status ifcdaqdrv_scope_arm_device(struct ifcdaqdrv_usr *ifcuser)
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

#if 0
    /* Stop any running acquisition */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, 1 << 31 | 1, IFC_SCOPE_TCSR_CS_ACQ_Command_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }
#endif

    /* ACQ_CLKERR returns HIGH when we first try to ARM the device, and then LOW
     * on the subsequent execution. This is a iterative verification with 100 ms timeout
     */
    for (i = 0; i < 5; i++)
    {
        status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
        if ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_CLKERR_MASK) == 0)
        {
            break;
        }
        usleep(20000);
    }

    /* if the "for" didn't break, then ACQ_CLKERR is still HIGH, which means that clock is not locked */
    /*if (i == 5)
    {
        LOG((LEVEL_ERROR,"Error: %s() ADC acquisition clock reference PLL is unlocked!\n", __FUNCTION__));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_unknown;
    }*/

    /* Arm device */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 1 << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT, 0);

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
            status  = ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_ACQ_TCSR_LA_REG, 1 << IFC_SCOPE_TCSR_LA_Spec_CMD_SHIFT);
            usleep(ifcdevice->poll_period);
            status |= ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
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

    ifcdevice->armed = 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Disarm device
 */
ifcdaqdrv_status ifcdaqdrv_scope_disarm_device(struct ifcdaqdrv_usr *ifcuser)
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
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG,
                                   IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ABORT << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT
                                   | IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE,
                                   IFC_SCOPE_TCSR_CS_ACQ_Command_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);

    if(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) {
        // If ACQ_Status hasn't gone to idle (0) disarming failed.
        // Try ten times to disarm the board and then report internal error.
        for(i = 0; i < 10; ++i) {
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG,
                                           IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ABORT << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT
                                           | IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE,
                                           IFC_SCOPE_TCSR_CS_ACQ_Command_MASK);
            if (status) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_access;
            }
            status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
            if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK)) {
                goto disarm_success;
            }
        }

        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

disarm_success:
    ifcdevice->armed = 0;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Wait for acquisition to end.
 *
 * Currently implemented by polling device for "Acquisition ended". Should be implemented with interrupts.
 */
ifcdaqdrv_status ifcdaqdrv_scope_wait_acq_end(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    do {
        status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
        usleep(ifcdevice->poll_period);
    } while (!status && ifcdevice->armed && (
            (i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT !=
              IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_DONE));

    if (!ifcdevice->armed) {
        return status_cancel;
    }

    ifcdaqdrv_disarm_device(ifcuser);
    return status;
}


/*
 * Set trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_scope_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_cs_val   = 0; // Control & Status value
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
        i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger

        i32_trig_val |= (mask & 0x07) << 20;  // Set MTCA line
        i32_trig_val |= 0x03 << 16;           // Set backplane trigger
        if (rising_edge & 0x7FFFFFFF) {
            i32_trig_val |= 1 << 27;
        }
        break;

    case ifcdaqdrv_trigger_frontpanel:
        gpio = mask & 0x40000000;

        if (!gpio) {
            return status_argument_range;
        }

        i32_cs_val   = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger

        i32_trig_val |= 2 << 16;       // Set GPIO trigger
        if (rising_edge & 0x40000000) { // Set edge of trigger detection
            i32_trig_val |= 1 << 27;
        }
        break;

    case ifcdaqdrv_trigger_adc: 
        channel_mask = mask & 0x3fffffff;
        while (channel_mask >>= 1) {
            ++channel;
        }

        if (channel >= ifcdevice->nchannels) {
            return status_argument_range;
        }

        i32_cs_val   = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger

        /* Trigger only on channel */
        i32_trig_val |= channel << 28;
        if (rising_edge & (1 << channel)) {
            i32_trig_val |= 1 << 27;
        }
        break;

    case ifcdaqdrv_trigger_soft:
        status = ifcdaqdrv_scope_get_ptq(ifcdevice, &ptq);
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
        i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        break;
    }

    LOG((LEVEL_DEBUG, "Will set cs val 0x%08x, trig val 0x%08x\n", i32_cs_val, i32_trig_val));

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->trigger_type = trigger;

    /* Enable backplane lines (MLVDS) */
    if (ifcdevice->trigger_type == ifcdaqdrv_trigger_backplane)
        ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_MLVDS_CONTROL_REG, 1<<IFC_SCOPE_MLVDS_ENABLE_SHIFT, 0);
    else
        ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_MLVDS_CONTROL_REG, 0, 1<<IFC_SCOPE_MLVDS_ENABLE_SHIFT);

    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, i32_cs_val, IFC_SCOPE_TCSR_CS_ACQ_Single_MASK | IFC_SCOPE_TCSR_CS_ACQ_Auto_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, i32_trig_val, 0xFFFFFFFF);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    if (ifcdevice->set_trigger_threshold) {
        status = ifcdevice->set_trigger_threshold(ifcdevice, threshold);
    }

#if 0
    /* This is interesting because set_trigger_threshold may modify the content of trigger register */
    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, &i32_trig_val);
    LOG((LEVEL_INFO, "Is set trig val %08x\n", i32_trig_val));
#endif

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}

/*
 * Get trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_scope_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge)
{
    ifcdaqdrv_status      status;
    int32_t               i32_cs_val   = 0; // Control & Status value
    int32_t               i32_trig_val = 0; // Trigger value
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Lock device so that the trigger configuration can be read out atomically. */
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_cs_val);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, &i32_trig_val);
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
    }

    if (mask) {
        *mask = 0;
        if (i32_trig_val & 1 << 18) {
            // GPIO and Channel
            *mask  = 0x40000000;
            *mask |= 1 << ((i32_trig_val >> 28) & 0x7);
        } else if (((i32_trig_val >> 16) & 3) == 2) {
            // GPIO only
            *mask = 0x40000000;
        } else if (((i32_trig_val >> 16) & 3) == 3) {
            // Backplane (VME) only
            *mask = 0x80000000 | ((i32_trig_val >> 26) & 0x7F);
        } else {
            // Channel only
            *mask = 1 << ((i32_trig_val >> 28) & 0x7);
        }
    }

    if (rising_edge) {
        *rising_edge = 0;
        if(i32_trig_val & (1 << 27)) {
            /* Trigger polarity is positive edge */
            if (((i32_trig_val >> 16) & 3) == 2) {
                // GPIO only
                *rising_edge = 0x40000000;
            } else if (((i32_trig_val >> 16) & 3) == 3) {
                // Backplane (VME) only
                *rising_edge = 0x80000000 | ((i32_trig_val >> 26) & 0x7F);
            } else {
                // Channel only
                *rising_edge = 1 << ((i32_trig_val >> 28) & 0x7);
            }
        }
        if(i32_trig_val & (1 << 19)) {
            /* GPIO gate is active high */
            *rising_edge |= 0x40000000;
        }
        *rising_edge = (i32_trig_val & (1 << 27)) ? 0x1FF : 0;
    }

    return status;
}

ifcdaqdrv_status ifcdaqdrv_scope_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold)
{
    uint16_t ui16_reg_val = (uint16_t)threshold + 32768;
    // threshold += 32768; // Threshold should be ADC value (unsigned).

    return ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, ui16_reg_val & 0xFFFF, 0xFFFF);
}

ifcdaqdrv_status ifcdaqdrv_scope_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold)
{
    ifcdaqdrv_status status;
    int32_t          threshold_adc;
    int32_t          i32_reg_val;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, &i32_reg_val);
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

ifcdaqdrv_status ifcdaqdrv_scope_set_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples)
{
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
    return ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, i32_reg_val << 12, IFC_SCOPE_TCSR_CS_SRAM_ACQ_Size_MASK);
}

ifcdaqdrv_status ifcdaqdrv_scope_get_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples)
{
    int32_t i32_reg_val;
    
    int     status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
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
ifcdaqdrv_status ifcdaqdrv_scope_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples)
{
    int32_t  i32_reg_val;
    ifcdaqdrv_status status;
    uint32_t acq_size;
    uint32_t average;

    if(!ifcdevice->nchannels || !ifcdevice->sample_size) {
        return status_internal;
    }

    status   = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_SMEM_BASE_SIZE_REG, &i32_reg_val);
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
ifcdaqdrv_status ifcdaqdrv_scope_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples)
{
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
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_SMEM_BASE_SIZE_REG, i32_reg_val & 0xFF, 0xFF);
    return status;
}

/* Returns last sample index written to in circular buffer. */
ifcdaqdrv_status ifcdaqdrv_scope_get_sram_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address)
{
    int32_t i32_reg_val;
    int     status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_LA_REG, &i32_reg_val);

    if (ifcdevice->sample_size == 4) {
        *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SRAMx_LA_Last_Address_MASK) >> 2;
    } else { // sample_size == 2
        *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SRAMx_LA_Last_Address_MASK) >> 1;
    }

    return status;
}

ifcdaqdrv_status ifcdaqdrv_scope_get_smem_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address)
{
    int32_t i32_reg_val;
    int     status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_LA_REG, &i32_reg_val);

    *last_address = (i32_reg_val & IFC_SCOPE_TCSR_SMEMx_LA_Last_Address_MASK) >> 4;

    return status;
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_scope_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq)
{
    if (ptq > 7) {
        return status_argument_range;
    }

    return ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, ptq << 5,
                                 IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK);
}

/**
 * Pre-trigger quota is 0-7 eighths of the acquisition.
 *
 * @return pretrigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_scope_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq)
{
    int32_t i32_reg_val;
    int     status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
    if (status) {
        return status;
    }
    if (!ptq) {
        return status_argument_invalid;
    }

    *ptq = ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK) >> 5);

    return status;
}

ifcdaqdrv_status ifcdaqdrv_scope_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data)
{
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
        

        status = ifcdaqdrv_scope_get_smem_la(ifcdevice, &last_address);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_scope_get_ptq(ifcdevice, &ptq);
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

ifcdaqdrv_status ifcdaqdrv_scope_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data)
{
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
        offset = IFC_SCOPE_SRAM_SAMPLES_OFFSET + (channel << 16);
        status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
        if (status) {
            return status;
        }
        
        status = ifcdaqdrv_scope_get_sram_la(ifcdevice, &last_address);
        if (status) {
            return status;
        }

        status = ifcdaqdrv_scope_get_ptq(ifcdevice, &ptq);
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

        status = ifcdaqdrv_scope_get_smem_la(ifcdevice, &last_address);
        if (status) {
            return status_device_access;
        }

        status = ifcdaqdrv_scope_get_ptq(ifcdevice, &ptq);
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

ifcdaqdrv_status ifcdaqdrv_scope_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples)
{
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
                              size_t nelm)
{
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

ifcdaqdrv_status ifcdaqdrv_scope_switch_mode(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_acq_store_mode mode)
{
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


    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
    cs_reg = i32_reg_val & (IFC_SCOPE_TCSR_CS_ACQ_Single_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK |
                            IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK);
    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, &i32_reg_val);
    trig_reg = i32_reg_val;

    // Clear SCOPE app
    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 0);
    // Clear SCOPE trigger
    ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, 0);

    ifcdevice->mode = mode;


/* TODO: check the usage of register 0x63 (IFC_SCOPE_DTACQ_TCSR_GC) */

    /* Clear general control register and enable specific acquisition mode.. */
    /*if (ifcdevice->fmc == 1) {
        status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC1_ACQRUN_SHIFT, 0xffffffff);
    } else {
        status = ifc_scope_tcsr_setclr(ifcdevice, IFC_SCOPE_DTACQ_TCSR_GC, (IFC_SCOPE_DTACQ_TCSR_GC_ACQRUN_MASK |
                                                                            IFC_SCOPE_DTACQ_TCSR_GC_ACQFIFO_MASK) <<
                                       IFC_SCOPE_DTACQ_TCSR_GC_FMC2_ACQRUN_SHIFT, 0xffffffff);
    }

    if (status) {
        LOG((LEVEL_ERROR, "Error trying to access firmware registers (returned %d )", status));
        return status;
    }*/
  
    status = ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, cs_reg);
    status |= ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_ACQ_TCSR_TRIG_REG, trig_reg);

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
 *    a) on ADC311X this is either
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
        if (nsamples < 4096) {
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

ifcdaqdrv_status ifcdaqdrv_scope_get_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average)
{
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
    
    if(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK) {
        *average = ifcdevice->averages[(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT];
        
        return status;
    }

    *average = 1;
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_set_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t average)
{
    uint32_t              i;
    int32_t               i32_reg_val;
    ifcdaqdrv_status      status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
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
        status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 0, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK);
        if(status) {
            return status;
        }

        status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 0, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
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
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 1 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_SHIFT, 0);
            if(status) {
                return status;
            }

            // Check if averaging was successfully enabled, otherwise no support :(
            status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
            if(status) {
                return status;
            }

            if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK)) {
                return status_no_support;
            }

            // Set the avaraging factor.
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, i << IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
            return status;
        }
    }
    return status_argument_invalid;
}


/*
 * Set decimation
 * 
 */

ifcdaqdrv_status ifcdaqdrv_scope_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation)
{
    uint32_t              i;
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
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
    if((decimation != 1) && (i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK)) {
        return status_config;
    }

    for (i = 0; i < MAX_DECIMATIONS; ++i) {
        if (ifcdevice->decimations[i] == decimation) {
            pthread_mutex_lock(&ifcdevice->lock);

            if (ifcdevice->armed) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_armed;
            }

            status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, i << 2, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
	    
            pthread_mutex_unlock(&ifcdevice->lock);
            return status;
        }
    }
    return status_argument_invalid;
}

/*
 * Get decimation
 */

ifcdaqdrv_status ifcdaqdrv_scope_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation)
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

    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, &i32_reg_val);
    if (status)
    {
        return status_read;
    }

    if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK)) {
        *decimation = ifcdevice->decimations[(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT];
        return status;
    }

    *decimation = 0;
    return status_success;
}

ifcdaqdrv_status ifcdaqdrv_scope_init_smem_mode(struct ifcdaqdrv_dev *ifcdevice)
{
    ifcdaqdrv_status      status;

    ifcdevice->mode = ifcdaqdrv_acq_mode_smem;

    /* Set ACQ_downSMP_MOD to 1 (bit 15 = 1) */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 1 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_SHIFT, 0);
    if (status) return status;

    /* Set ACQ_downSMP to 000 (1:4 with averaging) */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_ACQ_TCSR_CS_REG, 0, 0x7 << IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT);
    if (status) return status;

    LOG((5, "SCOPE application configured to SMEM mode\n"));
    return status_success;
}

/* Functions for accessing 0x60 to 0x6F (SCOPE MAIN TCSR) */
ifcdaqdrv_status ifc_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, value);
}

ifcdaqdrv_status ifc_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, setmask, clrmask);
}

/* Functions for accessing SCOPE ACQ TCSR  (0x70-0x73, 0x74-0x77, 0x78-0x7B, 0x7C-0x7F (SCOPE FMC1/FMC2 and SRAM/SMEM specific)) */
ifcdaqdrv_status ifc_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val)
{
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_acq_tcsr_offset(ifcdevice) + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value)
{
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_acq_tcsr_offset(ifcdevice) + register_idx, value);
}

ifcdaqdrv_status ifc_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask)
{
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_acq_tcsr_offset(ifcdevice) + register_idx, setmask, clrmask);
}

//Remove these
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

    return ifcdaqdrv_scope_set_ptq(ifcdevice, ptq);
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

    status    = ifcdaqdrv_scope_get_ptq(ifcdevice, &ptq);
    if (status) {
        return status;
    }

    *npretrig = ptq * nsamples;

    return status_success;
}


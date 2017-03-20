#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#ifdef TOSCA_USRLIB
// #include <pevioctl.h>
// #include <pevxulib.h>
#include <tscioctl.h>
#include <tsculib.h>
#endif

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_fmc.h"
// #include "ifcdaqdrv_acq420.h"
#include "ifcdaqdrv_adc3110.h"
// #include "ifcfastintdrv.h"
// #include "ifcfastintdrv_utils.h"

LIST_HEAD(ifcdaqdrv_devlist);
pthread_mutex_t ifcdaqdrv_devlist_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * Open device. Keep a list of users to support multiple open calls in the same process
 */

ifcdaqdrv_status ifcdaqdrv_open_device(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    //struct pevx_node     *node;
    int                   node; /* TOSCA file descriptor */
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;



    if (!ifcuser || ifcuser->card >= MAX_PEV_CARDS || (ifcuser->fmc != 1 && ifcuser->fmc != 2)) {
        return status_argument_invalid;
    }

    LOG((5, "Level %d tracing set.\n", ifcdaqdrvDebug));

    pthread_mutex_lock(&ifcdaqdrv_devlist_lock);

    /* Check if device already is opened in list. */
    list_for_each_entry(ifcdevice, &ifcdaqdrv_devlist, list){
        if (ifcdevice->card == ifcuser->card && ifcdevice->fmc == ifcuser->fmc) {
            /* Try reading from the device to check that it is physically present. */
            status = ifc_xuser_tcsr_read(ifcuser->device, 0, &i32_reg_val);

            if (status) {
                continue;
            }

            ifcuser->device = ifcdevice;
            ifcdevice->count++;
            pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
            return status_success;
        }
    }

    /* Initialize pevx library */

#ifdef TOSCA_USRLIB
    // node = pevx_init(ifcuser->card);
    // if (!node) {
    //     status = status_no_device;
    //     goto err_pevx_init;
    // }

    node = tsc_init();
    if (node < 0) {
        status = status_no_device;
        goto err_pevx_init;
    }

#endif

    /* Allocate private structure */
    ifcdevice = calloc(1, sizeof(struct ifcdaqdrv_dev));
    if (!ifcdevice) {
        status = status_internal;
        goto err_dev_alloc;
    }

    ifcdevice->card  = ifcuser->card;
    ifcdevice->fmc   = ifcuser->fmc;
    ifcdevice->node  = node;
    ifcdevice->count = 1;

    pthread_mutex_init(&ifcdevice->lock, NULL);

    ifcuser->device = ifcdevice;

    /* Read TOSCA signature and verify that board is a TOSCA board. */
    status = ifc_xuser_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (status) {
        status = status_internal;
        goto err_read;
    }
    if (i32_reg_val != IFC1210SCOPEDRV_TOSCA_SIGNATURE) {
        // Bug in current firmware, TOSCA signature is not at address 0.
        // printf("reg: %x, sig: %x\n", i32_reg_val, IFC1210SCOPEDRV_TOSCA_SIGNATURE);
        // ifcdaqdrv_free(ifcdevice);
        // pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
        // return status_incompatible;
    }
    ifcdevice->tosca_signature = i32_reg_val;

    /* Read APP signature */
    status = ifc_scope_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (status) {
        status = status_internal;
        goto err_read;
    }

    switch (i32_reg_val) {
    case IFC1210SCOPEDRV_SCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_SCOPE_DTACQ_SIGNATURE:
        LOG((5, "Generic DAQ Application\n"));
        /* Recognized scope implementation. */
        break;
    // case IFC1210FASTINT_APP_SIGNATURE:
    //     LOG((5, "Fast Interlock Application\n"));
    //     /* Recognized fast interlock implementation */
    //     break;
    default:
        // Skip all signature verification for now...
        //status = status_incompatible;
        //goto err_read;
        break;
    }
    ifcdevice->app_signature = i32_reg_val;
#if DEBUG
    printf("TOSCA signature: %08x, APP signature: %08x, ", ifcdevice->tosca_signature, ifcdevice->app_signature);
    /* Read FMC FDK signature */
    status = ifc_fmc_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (status) {
        status = status_internal;
        goto err_read;
    }
    printf("FMC FDK signature: %08x\n", i32_reg_val);
#endif

    /* Determine what type of FMC that is mounted. */
    ifcdevice->fru_id = calloc(1, sizeof(struct fmc_fru_id));
    if (!ifcdevice->fru_id) {
        status = status_internal;
        goto err_read;
    }

    /* ifc_fmc_eeprom_read_fru will allocate two char arrays that have to be freed by us. */
    status = ifc_fmc_eeprom_read_fru(ifcdevice, ifcdevice->fru_id);
    if (status == status_fru_info_invalid) {
        /* .... fall back to IOxOS proprietary signature */
        status = ifc_read_ioxos_signature(ifcdevice, ifcdevice->fru_id);
        if (status) {
            status = status_internal;
            goto err_read;
        }
    } else if (status) {
        status = status_internal;
        goto err_read;
    }

    /*
     * Register the correct functions with the ifcdevice and
     * allocate all memory necessary for DMA transfers
     */
    switch (ifcdevice->app_signature) {
    case IFC1210SCOPEDRV_SCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_SCOPE_DTACQ_SIGNATURE:
        status = ifcdaqdrv_scope_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }
        break;
    // case IFC1210FASTINT_APP_SIGNATURE:
    //     status = ifcfastintdrv_register(ifcdevice);
    //     if(status) {
    //         goto err_dev_alloc;
    //     }
    //     status = ifcfastintdrv_dma_allocate(ifcdevice);
    //     if(status) {
    //         goto err_read;
    //     }
    //     break;
    default:
        break;
    }

    /* Add device to the list of opened devices */
    list_add_tail(&ifcdevice->list, &ifcdaqdrv_devlist);
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
    return status_success;

err_read:
    /* Free ifcdevice (This will also free fru_id for us. */
    ifcdaqdrv_free(ifcdevice);

err_dev_alloc:
    /* Close pevx library */
#ifdef TOSCA_USRLIB
    // pevx_exit(ifcdevice->card);
    tsc_exit();
#endif  

err_pevx_init:
    /* Unlock device list */
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
    return status;
}

/*
 * Close the device. Check if this was the last user that closed the device and in that case clean up.
 */

ifcdaqdrv_status ifcdaqdrv_close_device(struct ifcdaqdrv_usr *ifcuser) {
    /* TODO When signals are implemented, cleanup that as well */
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Close device if this is the last user. */
    pthread_mutex_lock(&ifcdaqdrv_devlist_lock);
    if (--ifcdevice->count == 0) {
        list_del(&ifcdevice->list);
        ifcdaqdrv_free(ifcdevice);
#ifdef TOSCA_USRLIB
        // pevx_exit(ifcdevice->card);
        tsc_exit();
#endif
        free(ifcdevice);
    }
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);

    ifcuser->device = NULL;

    return status_success;
}

/*
 * Initialize the ADCs. Call the FMC specific ADC initializer function. Keep track of the fact that it has been called
 * with init_called counter to avoid that users arm the device before calling this function.
 */

ifcdaqdrv_status ifcdaqdrv_init_adc(struct ifcdaqdrv_usr *ifcuser) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->init_adc) {
        return status_no_support;
    }

    ifcdevice->init_called++;

    return ifcdevice->init_adc(ifcdevice);
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
ifcdaqdrv_status ifcdaqdrv_arm_device(struct ifcdaqdrv_usr *ifcuser){
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(ifcdevice->init_called == 0){
        fprintf(stderr, "WARNING: init_adc() has not been called\n");
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

    // check ACQ Clock
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
    if ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_CLKERR_MASK) != 0) {
#if DEBUG
        printf("Error: %s() ADC acquisition clock reference PLL is unlocked!\n", __FUNCTION__);
#endif
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_unknown;
    }

    /* Arm device */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_TCSR_CS_REG, 1 << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT, 0);

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
        LOG((6, "%f %d iterations\n", acquisition_time, (int32_t) (SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES + acquisition_time / (ifcdevice->poll_period * 1e-6))));
        do {
            status  = ifc_scope_acq_tcsr_write(ifcdevice, IFC_SCOPE_TCSR_LA_REG, 1 << IFC_SCOPE_TCSR_LA_Spec_CMD_SHIFT);
            usleep(ifcdevice->poll_period);
            status |= ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
        } while (!status && ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT) < 2 && timeo-- > 0);

        if (status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_device_access;
        }

        if(((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT) < 2) {
            // Failed to self-trigger.
            LOG((6, "CS register value is %08x after %d iterations\n", i32_reg_val, (int32_t) (SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES + acquisition_time / (ifcdevice->poll_period * 1e-6))));
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
ifcdaqdrv_status ifcdaqdrv_disarm_device(struct ifcdaqdrv_usr *ifcuser){
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t               i32_reg_val;
    int32_t               i;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Set "Acquisition STOP/ABORT" bit and "ACQ mode single" bit. Single
     * mode has to be set to disable a continuous acquisition. */
    status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0,
                                       IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ABORT << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT
                                       | IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE,
                                       IFC_SCOPE_TCSR_CS_ACQ_Command_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_access;
    }

    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);

    if(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) {
        // If ACQ_Status hasn't gone to idle (0) disarming failed.
        // Try ten times to disarm the board and then report internal error.
        for(i = 0; i < 10; ++i) {
            status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0,
                                               IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ABORT << IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT
                                               | IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE,
                                               IFC_SCOPE_TCSR_CS_ACQ_Command_MASK);
            if (status) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_access;
            }
            status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
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
 * Currently implemented by polling device for "Acquisition ended". Could be implemented with interrupts.
 */

ifcdaqdrv_status ifcdaqdrv_wait_acq_end(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    do {
        status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
        LOG((LEVEL_DEBUG, "TCSR %02x 0x%08x\n", ifc_get_scope_tcsr_offset(ifcdevice), i32_reg_val));
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
 * Read out samples. Call FMC specific function.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai(struct ifcdaqdrv_usr *ifcuser, void *data) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if(ifcdevice->read_ai) {
        status = ifcdevice->read_ai(ifcdevice, data);
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

/*
 * Read out samples for one channel. Call FMC specific function.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai_ch(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, void *data) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (channel >= ifcdevice->nchannels) {
        return status_argument_range;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if(ifcdevice->read_ai_ch) {
        status = ifcdevice->read_ai_ch(ifcdevice, channel, data);
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

/*
 * Set clock source.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock clock) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->set_clock_source) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdevice->set_clock_source(ifcdevice, clock);

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}

/*
 * Get clock source.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock *clock) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->get_clock_source) {
        return status_no_support;
    }

    return ifcdevice->get_clock_source(ifcdevice, clock);
}

/*
 * Set clock frequency.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if (ifcdevice->set_clock_frequency) {
        status = ifcdevice->set_clock_frequency(ifcdevice, frequency);
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

/*
 * Get clock frequency.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double *frequency) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_clock_frequency) {
        return status_no_support;
    }
    if (!frequency) {
        return status_argument_invalid;
    }

    return ifcdevice->get_clock_frequency(ifcdevice, frequency);
}

/*
 * Get valid clock frequencies. ifcdevice->valid_clocks must be a 0 terminated array.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_frequencies_valid(struct ifcdaqdrv_usr *ifcuser, double *frequencies, size_t buf_len, size_t *data_len) {
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t              last;
    double               *target;
    double                *source;

    last = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!frequencies) {
        return status_argument_invalid;
    }

    for(target = frequencies, source = ifcdevice->valid_clocks; target < frequencies + buf_len; ++target, ++source) {
        *target = *source;
        /* If the next value is 0 we managed to copy all values to the user */
        if(!*(source+1)) {
            last = 1;
            break;
        }
    }

    if(data_len) {
        *data_len = source - ifcdevice->valid_clocks + 1;
    }

    if (!last) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Set clock divisor.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t divisor) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(divisor < ifcdevice->divisor_min || divisor > ifcdevice->divisor_max) {
        return status_argument_range;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if (ifcdevice->set_clock_divisor) {
        status = ifcdevice->set_clock_divisor(ifcdevice, divisor);
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

/*
 * Get clock divisor
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!divisor) {
        return status_argument_invalid;
    }

    if (!ifcdevice->get_clock_divisor) {
        *divisor = 1;
        return status_success;
    }

    return ifcdevice->get_clock_divisor(ifcdevice, divisor);
}

/*
 * Get clock divisor range
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_divisor_range(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor_min, uint32_t *divisor_max) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (divisor_max) {
        *divisor_max = ifcdevice->divisor_max;
    }

    if (divisor_min) {
        *divisor_min = ifcdevice->divisor_min;
    }

    return status_success;
}

/*
 * Set trigger configuration
 */

ifcdaqdrv_status ifcdaqdrv_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge) {
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
    case ifcdaqdrv_trigger_backplane:
        i32_trig_val |= (mask & 0x7f) << 20;  // Set VME pin
        i32_trig_val |= 0x11 << 16;           // Set backplane trigger
        if (rising_edge & 0x7FFFFFFF) {
            i32_trig_val |= 1 << 27;
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

        i32_cs_val   = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger
        if (!gpio) {
            /* Trigger only on channel */
            i32_trig_val |= channel << 28;
            if (rising_edge & (1 << channel)) {
                i32_trig_val |= 1 << 27;
            }
        } else if (gpio && (mask & 0xff)) {
            /* Trigger on GPIO & channel */
            i32_trig_val |= channel << 28;
            if (rising_edge & (1 << channel)) {
                i32_trig_val |= 1 << 27;
            }
            if (rising_edge & mask & 0x40000000) {
                i32_trig_val |= 1 << 19; // 1 = Active high
            }
        } else {
            /* Trigger only on GPIO */
            i32_trig_val |= 2 << 16;       // Set GPIO trigger
            if (rising_edge & 0x40000000) {
                i32_trig_val |= 1 << 27;
            }
        }
        break;
    case ifcdaqdrv_trigger_auto: // Auto is not supported anymore (will be interpreted as soft trigger)
    case ifcdaqdrv_trigger_soft:
        status = ifcdaqdrv_get_ptq(ifcdevice, &ptq);
        if(status) {
            return status;
        }
        if(ptq != 0) {
            // It doesn't make sense to have pre-trigger samples when triggering manually.
            return status_config;
        }
    case ifcdaqdrv_trigger_none:
        i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
    default:
        break;
    }

    LOG((LEVEL_INFO, "Will set cs val 0x%08x, trig val 0x%08x\n", i32_cs_val, i32_trig_val));

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->trigger_type = trigger;

    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_TCSR_CS_REG, i32_cs_val, IFC_SCOPE_TCSR_CS_ACQ_Single_MASK | IFC_SCOPE_TCSR_CS_ACQ_Auto_MASK);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    status = ifc_scope_acq_tcsr_setclr(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, i32_trig_val, 0xFFFFFFFF);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    if (ifcdevice->set_trigger_threshold) {
        status = ifcdevice->set_trigger_threshold(ifcdevice, threshold);
    }

#if DEBUG
    /* This is interesting because set_trigger_threshold may modify the content of trigger register */
    ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, &i32_trig_val);
    LOG((LEVEL_INFO, "Is set trig val %08x\n", i32_trig_val));
#endif

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}

/*
 * Get trigger configuration
 */

ifcdaqdrv_status ifcdaqdrv_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge) {
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
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_cs_val);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_TRIG_REG, &i32_trig_val);
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

/*
 * Set average
 */

ifcdaqdrv_status ifcdaqdrv_set_average(struct ifcdaqdrv_usr *ifcuser, uint32_t average) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdaqdrv_status status;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdaqdrv_scope_set_average(ifcdevice, average);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get average
 */

ifcdaqdrv_status ifcdaqdrv_get_average(struct ifcdaqdrv_usr *ifcuser, uint32_t *average) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!average) {
        return status_argument_invalid;
    }

    return ifcdaqdrv_scope_get_average(ifcdevice, average);

}

/*
 * Get valid averages
 */

ifcdaqdrv_status ifcdaqdrv_get_averages_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *averages, size_t buf_len, size_t *data_len) {
    int32_t              last = 0;
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t *target;
    uint32_t *source;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    for(target = averages, source = ifcdevice->averages; target < averages + buf_len; target++, source++) {
        *target = *source;
        if(!*(source+1)) {
            last = 1;
            break;
        }
    }

    if(data_len) {
        *data_len = source - ifcdevice->averages + 1;
    }

    if (!last) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Set decimation
 */

ifcdaqdrv_status ifcdaqdrv_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation) {
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
    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if(decimation != 1 && i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK) {
        return status_config;
    }

    for (i = 0; i < MAX_DECIMATIONS; ++i) {
        if (ifcdevice->decimations[i] == decimation) {
            pthread_mutex_lock(&ifcdevice->lock);

            if (ifcdevice->armed) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_armed;
            }

            status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, i << 2, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
            pthread_mutex_unlock(&ifcdevice->lock);
            return status;
        }
    }
    return status_argument_invalid;
}

/*
 * Get decimation
 */

ifcdaqdrv_status ifcdaqdrv_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation) {
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

    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if(!(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK)) {
        *decimation = ifcdevice->decimations[(i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT];
        return status;
    }

    *decimation   = 1;

    return status;
}

/*
 * Get valid decimations.
 */

ifcdaqdrv_status ifcdaqdrv_get_decimations_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimations, size_t buf_len, size_t *data_len) {
    uint32_t              last = 0;
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t *target, *source;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    for(target = decimations, source = ifcdevice->decimations; target < decimations + buf_len; target++, source++) {
        *target = *source;
        if(!*(source+1)) {
            last = 1;
            break;
        }
    }

    if(data_len) {
        *data_len = source - ifcdevice->decimations + 1;
    }

    if (!last) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Set number of samples.
 */

ifcdaqdrv_status ifcdaqdrv_set_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t nsamples) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!ifcdevice->set_nsamples) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }
    status = ifcdevice->set_nsamples(ifcdevice, nsamples);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get number or samples.
 */

ifcdaqdrv_status ifcdaqdrv_get_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t *nsamples) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!nsamples) {
        return status_argument_invalid;
    }
    return ifcdevice->get_nsamples(ifcdevice, nsamples);
}

/*
 * Set number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_set_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t npretrig) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdaqdrv_scope_set_npretrig(ifcdevice, npretrig);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_get_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t *npretrig) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!npretrig) {
        return status_argument_invalid;
    }

    /* Take mutex since we have to read out multiple values */
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcdaqdrv_scope_get_npretrig(ifcdevice, npretrig);
    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}

/*
 * Set gain
 */

ifcdaqdrv_status ifcdaqdrv_set_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double gain) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_gain) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdevice->set_gain(ifcuser->device, channel, gain);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get gain
 */

ifcdaqdrv_status ifcdaqdrv_get_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double *gain) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_gain) {
        *gain = 1.0;
        return status_success;
    }

    return ifcdevice->get_gain(ifcdevice, channel, gain);
}

/*
 * Get maximum measurable voltage
 */

ifcdaqdrv_status ifcdaqdrv_get_vref_max(struct ifcdaqdrv_usr *ifcuser, double *vref_max) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if(!vref_max) {
        return status_argument_invalid;
    }

    if(!ifcdevice->vref_max) {
        return status_internal;
    }

    *vref_max = ifcdevice->vref_max;
    return status_no_support;
}

/*
 * Get actual sample resolution
 */

ifcdaqdrv_status ifcdaqdrv_get_resolution(struct ifcdaqdrv_usr *ifcuser, uint32_t *resolution) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if (!resolution) {
        return status_argument_invalid;
    }

    if (!ifcdevice->sample_resolution) {
        return status_internal;
    }

    *resolution = ifcdevice->sample_resolution;
    return status_success;
}

/*
 * Set test pattern. Not all FMCs implement all patterns.
 */

ifcdaqdrv_status ifcdaqdrv_set_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern pattern) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_pattern) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdevice->set_pattern(ifcuser->device, channel, pattern);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get test pattern.
 */

ifcdaqdrv_status ifcdaqdrv_get_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern *pattern) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_pattern) {
        *pattern = ifcdaqdrv_pattern_none;
        return status_success;
    }

    return ifcdevice->get_pattern(ifcdevice, channel, pattern);
}

/*
 * Get number of channels
 */

ifcdaqdrv_status ifcdaqdrv_get_nchannels(struct ifcdaqdrv_usr *ifcuser, uint32_t *nchannels) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!nchannels) {
        return status_argument_invalid;
    }

    if (!ifcdevice->nchannels) {
        return status_internal;
    }

    *nchannels = ifcdevice->nchannels;
    return status_success;
}

/*
 * Get manufacturer string
 */

ifcdaqdrv_status ifcdaqdrv_get_manufacturer(struct ifcdaqdrv_usr *ifcuser, char *manufacturer, size_t buf_len){
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!manufacturer) {
        return status_argument_invalid;
    }
    if (!ifcdevice->fru_id || !ifcdevice->fru_id->manufacturer) {
        return status_internal;
    }

    manufacturer[0] = '\0';
    strncat(manufacturer, ifcdevice->fru_id->manufacturer, buf_len - 1);

    if (buf_len < strlen(ifcdevice->fru_id->manufacturer)) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Get product name string
 */

ifcdaqdrv_status ifcdaqdrv_get_product_name(struct ifcdaqdrv_usr *ifcuser, char *product_name, size_t buf_len){
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!product_name) {
        return status_argument_invalid;
    }
    if (!ifcdevice->fru_id || !ifcdevice->fru_id->product_name) {
        return status_internal;
    }

    product_name[0] = '\0';
    strncat(product_name, ifcdevice->fru_id->product_name, buf_len - 1);

    if (buf_len < strlen(ifcdevice->fru_id->product_name)) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Get fw version
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_version(struct ifcdaqdrv_usr *ifcuser, uint8_t *version) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!version) {
        return status_argument_invalid;
    }

    if (!ifcdevice->get_signature) {
        return status_no_support;
    }

    status = ifcdevice->get_signature(ifcdevice, 0, version, 0);

    return status;
}

/*
 * Get fw revision
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_revision(struct ifcdaqdrv_usr *ifcuser, uint8_t *revision) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!revision) {
        return status_argument_invalid;
    }

    status = ifcdevice->get_signature(ifcdevice, revision, 0, 0);

    return status;
}

/*
 * Non-supported debug function
 */

ifcdaqdrv_status ifcdaqdrv_debug(struct ifcdaqdrv_usr *ifcuser) {
    struct ifcdaqdrv_dev *ifcdevice;
    int i;
    int32_t               i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    for (i = 0; i < 8; ++i) {
        ifc_fmc_tcsr_read(ifcdevice, i, &i32_reg_val);
        printf("[0x%02x]: 0x%08x\n", i, i32_reg_val);
    }
    return status_success;
}

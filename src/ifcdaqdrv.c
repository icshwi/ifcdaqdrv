#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

/* TOSCA API */
#include "tscioctl.h"
#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_adc3117.h"
#include "ifcfastintdrv.h"
#include "ifcfastintdrv_utils.h"

#define IFC1410_FMC_EN_DATA   0xC0000000
#define IFC1410_FMC_EN_OFFSET 0x000C

LIST_HEAD(ifcdaqdrv_devlist);
pthread_mutex_t ifcdaqdrv_devlist_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * Open device. Keep a list of users to support multiple open calls in the same process
 */

ifcdaqdrv_status ifcdaqdrv_open_device(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    
    int                   node; /* TOSCA file descriptor */
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;
    int                   data = IFC1410_FMC_EN_DATA;

    if (!ifcuser || ifcuser->card >= MAX_PEV_CARDS || (ifcuser->fmc != 1 && ifcuser->fmc != 2)) {
        return status_argument_invalid;
    }

    ifcdaqdrvDebug = 4; // Manually set level for debug
    LOG((LEVEL_NOTICE, "Level %d tracing set.\n", ifcdaqdrvDebug));

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

    /* Initialize tsc library */

    node = tsc_init();
    if (node < 0) {
        status = status_no_device;
        goto err_pevx_init;
    }

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
        LOG((LEVEL_NOTICE, "Generic DAQ Application\n"));
        /* Recognized scope implementation. */
        break;
    case IFC1210FASTINT_APP_SIGNATURE:
        LOG((LEVEL_NOTICE, "Fast Interlock Application\n"));
        /* Recognized fast interlock implementation */
        break;
    default:
        LOG((LEVEL_NOTICE, "Firmware application not supported\n"));
        // Skip all signature verification for now...
        //status = status_incompatible;
        //goto err_read;
        break;
    }
    ifcdevice->app_signature = i32_reg_val;

    INFOLOG(("TOSCA signature: %08x, APP signature: %08x\n", ifcdevice->tosca_signature, ifcdevice->app_signature));

    /* Read FMC FDK signature */
    status = ifc_fmc_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (status) {
        status = status_internal;
        goto err_read;
    }
    INFOLOG(("FMC FDK signature: %08x\n", i32_reg_val));

    /* Determine what type of FMC that is mounted. */
    ifcdevice->fru_id = calloc(1, sizeof(struct fmc_fru_id));
    if (!ifcdevice->fru_id) {
        status = status_internal;
        goto err_read;
    }

    ifcdevice->fru_id->product_name = calloc(1, sizeof(uint8_t)*8);
    if (!ifcdevice->fru_id->product_name) {
        status = status_internal;
        goto err_read;
    }

#ifdef ENABLE_I2C
    LOG((LEVEL_NOTICE, "Trying to read EEPROM\n"));
    ifc_fmc_eeprom_read_sig(ifcdevice, (uint8_t *)ifcdevice->fru_id->product_name);
#endif
    
    /*
     * Register the correct functions with the ifcdevice and
     * allocate all memory necessary for DMA transfers
     */
    switch (ifcdevice->app_signature) {
    case IFC1210SCOPEDRV_SCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_SCOPE_DTACQ_SIGNATURE:
    case IFC1410SCOPEDRV_SCOPE_SIGNATURE:
        status = ifcdaqdrv_scope_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        if (ifcdevice->board_id != 0x3118) {
            status = ifcdaqdrv_dma_allocate(ifcdevice);
            if(status) {
                goto err_read;
            }
        }
        break;
    case IFC1210FASTINT_APP_SIGNATURE:
        status = ifcfastintdrv_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        status = ifcfastintdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }
        break;
    default:
        break;
    }

    /* Enable the CENTRAL FPGA using the PON FPGA (pi c c0000000)*/
    tsc_pon_write(IFC1410_FMC_EN_OFFSET, &data);

    /* Add device to the list of opened devices */
    list_add_tail(&ifcdevice->list, &ifcdaqdrv_devlist);
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
    return status_success;

err_read:
    /* Free ifcdevice (This will also free fru_id for us. */
    ifcdaqdrv_free(ifcdevice);

err_dev_alloc:
    /* Close pevx library */
    tsc_exit();

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
        tsc_exit();
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

    if (ifcdevice->board_id == 0x3117) {
        for (i = 0; i < 5; i++)
        {
            status = ifc_scope_tcsr_read(ifcdevice, 0x5, &i32_reg_val);
            if ((i32_reg_val & 0x3) == 0x3) // Clock ready and init done
            {
                break;
            }
            usleep(20000);
        }
    } else {
        /* ACQ_CLKERR returns HIGH when we first try to ARM the device, and then LOW
         * on the subsequent execution. This is a iterative verification with 100 ms timeout
         */
        for (i = 0; i < 5; i++)
        {
            status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
            if ((i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_CLKERR_MASK) == 0)
            {
                break;
            }
            usleep(20000);
        }
    }

    /* if the "for" didn't break, then ACQ_CLKERR is still HIGH, which means that clock is not locked */
    if (i == 5)
    {
        LOG((LEVEL_ERROR,"Error: %s() ADC acquisition clock reference PLL is unlocked!\n", __FUNCTION__));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_unknown;
    }

    /* Arm device */
    if (ifcdevice->board_id == 0x3117) {
        status = ifc_scope_tcsr_write(ifcdevice, 0x1, 0xFFFFF000);
        status += ifc_scope_tcsr_write(ifcdevice, 0x2, 0x2);
        status += ifc_scope_tcsr_write(ifcdevice, 0x8, 0xFFFFF000);
        status += ifc_scope_tcsr_write(ifcdevice, 0x9, 0x2);
        
        if (status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_device_access;
        }

    } else {
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
                LOG((LEVEL_INFO, "CS register value is %08x after %d iterations\n", i32_reg_val, (int32_t) (SOFT_TRIG_LEAST_AMOUNT_OF_CYCLES + acquisition_time / (ifcdevice->poll_period * 1e-6))));
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_internal;
            }

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

    if (ifcdevice->board_id == 0x3117) {
        ifc_scope_tcsr_write(ifcdevice, 0x9, 0x4);
        ifc_scope_tcsr_write(ifcdevice, 0x2, 0x0);
    } else {
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

    if (ifcdevice->board_id == 0x3117) {
        do {
            status = ifc_scope_tcsr_read(ifcdevice, 0x2, &i32_reg_val);
            usleep(ifcdevice->poll_period);
        } while (!status && ifcdevice->armed && ((i32_reg_val & 0x00080000) != 0x0));
    } else {
        do {
            status = ifc_scope_acq_tcsr_read(ifcdevice, IFC_SCOPE_TCSR_CS_REG, &i32_reg_val);
	        usleep(ifcdevice->poll_period);
        } while (!status && ifcdevice->armed && (
                (i32_reg_val & IFC_SCOPE_TCSR_CS_ACQ_Status_MASK) >> IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT !=
                  IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_DONE));
    }

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

    /* Prevent from using external clock */
    if (clock == ifcdaqdrv_clock_external)
        clock = ifcdaqdrv_clock_internal;

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

    /* Forces ADC3117 trigger to be software until we don't have the support in firmware */
    /* TODO: enable backplane trigger for ADC3117 */
    if (ifcdevice->board_id == 0x3117) {
        if (trigger != ifcdaqdrv_trigger_soft) {
            LOG((LEVEL_ERROR, "ADC3117 only supports software (manual) trigger"));
        }
        trigger = ifcdaqdrv_trigger_soft;
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

    /* Additional cases needed to test the backplane triggering */
    case ifcdaqdrv_trigger_testmanual:
    case ifcdaqdrv_trigger_testauto:

        i32_cs_val = IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE;
        i32_trig_val = 1 << 31; // Enable trigger

        /* MASK will be either 11000 or 11001 */
        i32_trig_val |= (mask & 0x1F) << 20;  // Set self trigger/ self periodic trigger 
        i32_trig_val |= 0x03 << 16;           // keeps backplane trigger [17:16] = 11
        if (rising_edge & 0x7FFFFFFF) {
            i32_trig_val |= 1 << 27;
        }


        break;
    default:
        break;
    }

    LOG((LEVEL_DEBUG, "Will set cs val 0x%08x, trig val 0x%08x\n", i32_cs_val, i32_trig_val));

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

	    	//printf("Decimation [%d] is valid, but ACQ_downSMP is forced to be zero (1:4 with avg)\n", decimation);  
            //status = ifc_scope_acq_tcsr_setclr(ifcdevice, 0, i << 2, IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK);
	    
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
    if(!ifcdevice->get_nsamples) {
        return status_no_support;
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

    if (ifcdevice->board_id == 0x3117) {
        LOG((LEVEL_DEBUG, "Not valid operation on ADC3117\n"));
        return status_no_support; //Maybe return status_no_support?
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

    if (ifcdevice->board_id == 0x3117) {
        LOG((LEVEL_DEBUG, "Not valid operation on ADC3117\n"));
        *npretrig = 0;
        return status_no_support; //Maybe return status_no_support?
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
 * Set sample rate
 */

ifcdaqdrv_status ifcdaqdrv_set_sample_rate(struct ifcdaqdrv_usr *ifcuser, double sample_rate) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->set_sample_rate) {
        return status_internal;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdevice->set_sample_rate(ifcdevice, sample_rate);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

/*
 * Get sample rate
 */

ifcdaqdrv_status ifcdaqdrv_get_sample_rate(struct ifcdaqdrv_usr *ifcuser, double *sample_rate) {
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if (ifcdevice->board_id != 0x3117) {
        /* This should be moved to ADC3110 driver */
        double clock_frequency;
        uint32_t clock_divisor;
        ifcdaqdrv_get_clock_frequency(ifcuser, &clock_frequency);
        ifcdaqdrv_get_clock_divisor(ifcuser, &clock_divisor);
        *sample_rate = clock_frequency/clock_divisor;
        return status_success;
    }

    if (!ifcdevice->get_sample_rate) {
        return status_internal;
    }

    return ifcdevice->get_sample_rate(ifcdevice, sample_rate);
}

ifcdaqdrv_status ifcdaqdrv_send_configuration_command(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->configuration_command) {
        return status_internal;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    status = ifcdevice->configuration_command(ifcdevice);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
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


/* Function to simulate backplane trigger */
ifcdaqdrv_status ifcdaqdrv_set_simtrigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_simtrigger_action function, int32_t clk_divider) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t __attribute__((unused)) ui32_regval;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    status = status_success;

    //TODO: remove this function from the library - it is not used anymore

#if 0
    switch (function)
    {
        case ifcdaqdrv_simtrig_stop: // stop periodic trigger            
            if (ifc_xuser_tcsr_setclr(ifcdevice, 0x6F, 0x0, 0x3)) // clear bits 1:0
                status = status_device_access;
            else
                printf("Stopping simulated trigger\n");
            
            break;

        case ifcdaqdrv_simtrig_startauto:
            if (ifc_xuser_tcsr_setclr(ifcdevice, 0x6F, 0x2, 0x1)) // set bit 2
                status = status_device_access;            
            else
                printf("Starting periodic simulated trigger \n");
            
            break;

        case ifcdaqdrv_simtrig_manual:
            if (ifc_xuser_tcsr_setclr(ifcdevice, 0x6F, 0x1, 0x2)) // set bit 1
                status = status_device_access;
            else
                printf("Sending manual simulated trigger\n");
            
            break;

        case ifcdaqdrv_simtrig_setfreq:

            /* prevent negative values*/
            if (clk_divider <= 0) clk_divider = 4;
            
            /* calculate divisor */
            ui32_regval = 125000000 / clk_divider;

            if (ifc_xuser_tcsr_setclr(ifcdevice, 0x6F, (ui32_regval & 0xFFFFFFFC), ((~ui32_regval) & 0xFFFFFFFC)))
                status = status_device_access;
            else              
                printf("Changing clock divider to 0x%08x \n", (clk_divider & 0xFFFFFFFC));
            
            break;
        
        case ifcdaqdrv_simtrig_readreg:
            if (ifc_xuser_tcsr_read(ifcdevice, 0x6F, (int32_t*) &ui32_regval))
                status = status_device_access;
            else
                printf("Register 0x6F = 0x%08x \n", ui32_regval);

            break;
        
        default:
            printf("Invalid set_trigger_command\n");
            break;
    }   

    if (status) printf("Error while trying to access register 0x6F\n");     
#endif

    return status;
}

#define MAX_SAMPLE_RATES 100

struct sample_rate {
    double   frequency;
    int32_t divisor;
    int32_t decimation;
    int32_t average;
    double   sample_rate;
};

// First priority is sample_rate, second divisor
static int compare_sample_rates(const void *a, const void *b) {
    const struct sample_rate *da = (const struct sample_rate *) a;
    const struct sample_rate *db = (const struct sample_rate *) b;
    int32_t sample_diff = (da->sample_rate > db->sample_rate) - (da->sample_rate < db->sample_rate);
    if(!sample_diff) {
        return da->divisor - db->divisor;
    }
    return sample_diff;
}

ifcdaqdrv_status ifcdaqdrv_calc_sample_rate(struct ifcdaqdrv_usr *ifcuser, int32_t *averaging, int32_t *decimation, int32_t *divisor, double *freq, double *sample_rate, uint8_t sample_rate_changed) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;


    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if ((ifcdevice->board_id == 0x3117) && (sample_rate_changed == 1)) {
        status = ifcdaqdrv_set_sample_rate(ifcuser, *sample_rate);
    }
    else if ((ifcdevice->board_id == 0x3117) && (sample_rate_changed == 0)) {
        return status_success;
    }
    else {
        if (!sample_rate_changed) {
            *sample_rate = *freq / *divisor / *averaging / *decimation;
            return status_success;
        } else {
            struct sample_rate sample_rates[MAX_SAMPLE_RATES] = {};
            double   frequencies[5];
            size_t nfrequencies;
            uint32_t decimations[8];
            size_t ndecimations;
            uint32_t averages[8];
            size_t naverages;
            uint32_t *downsamples;
            size_t ndownsamples;
            uint32_t  divisor_tmp, div_min, div_max;
            uint32_t i, j, nsample_rates;
            int32_t k;
            double sample_rate_tmp;

            status = ifcdaqdrv_get_clock_divisor_range(ifcuser, &div_min, &div_max);
            status += ifcdaqdrv_get_clock_frequencies_valid(ifcuser, frequencies, sizeof(frequencies)/sizeof(double), &nfrequencies);
            status += ifcdaqdrv_get_decimations_valid(ifcuser, decimations, sizeof(decimations)/sizeof(uint32_t), &ndecimations);
            status += ifcdaqdrv_get_averages_valid(ifcuser, averages, sizeof(averages)/sizeof(uint32_t), &naverages);
            if (status) {
                LOG((LEVEL_NOTICE, "Getting values failed\n"));
                return status_device_access;
            }

            /*
             * Try to find the combination of clock frequency, clock divisor, decimation and average which
             * is closest (but higher) to the requested sample rate.
             *
             * The algorithm is as follows:
             * 1. For every available clock frequency
             *      For every available downsample (decimation and average)
             *         Start with the highest divisor and test all divisors until there is a sample rate higher than requested.
             *         If such a sample rate is found, add it to the list of sample rates.
             * 2. Sort list of sample rates. Lowest sample rate first. If equal prioritize lowest clock divisor.
             * 3. Pick the first combination in the list.
             */

            nsample_rates = 0;
            for(i = 0; i < nfrequencies; ++i) {
                for(j = 0; j < 2; ++j) {
                    if (j == 0) {
                        downsamples = decimations;
                        ndownsamples = ndecimations;
                    } else {
                        downsamples = averages;
                        ndownsamples = naverages;
                    }
                    for(k = ndownsamples - 1; k >= 0 ; --k) {
                        sample_rates[nsample_rates].frequency = frequencies[i];
                        sample_rates[nsample_rates].divisor = div_min;
                        sample_rates[nsample_rates].sample_rate = frequencies[i] / downsamples[k] / div_min;
                        sample_rates[nsample_rates].decimation = 1;
                        sample_rates[nsample_rates].average = 1;
                        for(divisor_tmp = div_max; divisor_tmp >= div_min; --divisor_tmp) {
                            sample_rate_tmp = frequencies[i] / downsamples[k] / divisor_tmp;
                            /*ndsDebugStream(m_node) << "Try Frequency: " << frequencies[i]
                                                   << ", Divisor: "     << divisor
                                                   << ", Downsample: "  << downsamples[i]
                                                   << " : "             << sample_rate_tmp << std::endl;*/
                            if(sample_rate_tmp >= *sample_rate) {
                                sample_rates[nsample_rates].frequency = frequencies[i];
                                sample_rates[nsample_rates].divisor = divisor_tmp;
                                sample_rates[nsample_rates].sample_rate = sample_rate_tmp;
                                if(j == 0) {
                                    sample_rates[nsample_rates].decimation = downsamples[k];
                                } else {
                                    sample_rates[nsample_rates].average = downsamples[k];
                                }
                                /*ndsDebugStream(m_node) << "OK Frequency: "  << sample_rates[nsample_rates].frequency
                                                       << ", Divisor: "     << sample_rates[nsample_rates].divisor
                                                       << ", Decimation: "  << sample_rates[nsample_rates].decimation
                                                       << ", Average: "     << sample_rates[nsample_rates].average
                                                       << ". Sample Rate: " << sample_rates[nsample_rates].sample_rate << std::endl;*/
                                nsample_rates++;
                                break;
                            }
                        }
                    }
                }
            }

            // Sort lowest sample rates firsts.
            qsort(sample_rates, nsample_rates, sizeof(struct sample_rate), compare_sample_rates);

            /*ndsInfoStream(m_node) << "Will set Frequency: " << sample_rates[0].frequency
                              << ", Divider: "          << sample_rates[0].divisor
                              << ", Decimation: "       << sample_rates[0].decimation
                              << ", Average: "          << sample_rates[0].average
                              << ". Sample Rate: "      << sample_rates[0].sample_rate << std::endl;*/

            *freq = sample_rates[0].frequency;
            *divisor = sample_rates[0].divisor;
            *decimation = sample_rates[0].decimation;
            *averaging = sample_rates[0].average;

            status = ifcdaqdrv_set_clock_frequency(ifcuser, *freq);
            status += ifcdaqdrv_set_clock_divisor(ifcuser, *divisor);
            if(*decimation > 1) {
                status += ifcdaqdrv_set_average(ifcuser, 1);
                status += ifcdaqdrv_set_decimation(ifcuser, *decimation);
            } else {
                status += ifcdaqdrv_set_decimation(ifcuser, 1);
                status += ifcdaqdrv_set_average(ifcuser, *averaging);
            }
            if (status) {
                LOG((LEVEL_NOTICE, "Setting values failed\n"));
                return status_device_access;
            }
        }
    }

    return status;
}


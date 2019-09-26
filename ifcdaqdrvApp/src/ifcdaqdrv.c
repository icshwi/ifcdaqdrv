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
#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"

#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_scope_lite.h"
#include "ifcdaqdrv_gen_scope.h"
#include "ifcdaqdrv_scope4ch.h"
#include "ifcdaqdrv_scope20ch.h"

#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_adc3117.h"

#include "ifcfastintdrv2.h"
#include "ifcfastintdrv_utils.h"

LIST_HEAD(ifcdaqdrv_devlist);
pthread_mutex_t ifcdaqdrv_devlist_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * Open device. Keep a list of users to support multiple open calls in the same process
 */

ifcdaqdrv_status ifcdaqdrv_open_device(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    int                   node; /* TOSCA file descriptor */
    int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    if (!ifcuser || ifcuser->card >= MAX_CARDS || (ifcuser->fmc != 1 && ifcuser->fmc != 2)) {
        return status_argument_invalid;
    }

    ifcdaqdrvDebug = 5; // Manually set level for debug
    LOG((LEVEL_NOTICE, "Level %d tracing set.\n", ifcdaqdrvDebug));

    pthread_mutex_lock(&ifcdaqdrv_devlist_lock);

    /* Check if device already is opened in list. */
    list_for_each_entry(ifcdevice, &ifcdaqdrv_devlist, list){
        if (ifcdevice->card == ifcuser->card && ifcdevice->fmc == ifcuser->fmc) {
            /* Try reading from the device to check that it is physically present. */
            status = ifc_xuser_tcsr_read(ifcdevice, 0, &i32_reg_val);
            if (status) {
                continue;
            }

            INFOLOG(("Device already opened!\n"));
            ifcuser->device = ifcdevice;
            ifcdevice->count++;
            pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
            return status_success;
        }
    }

    /* Initialize tsc library */
    node = tsc_init(ifcuser->card);
    if (node < 0) {
        status = status_no_device;
        goto err_tsc_init;
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
    ifcdevice->tosca_signature = i32_reg_val;
    
    /* Read APP signature */
    status = ifc_scope_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (status) {
        status = status_internal;
        goto err_read;
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

    if (ifcdaqdrv_is_byte_order_ppc()) {
        LOG((LEVEL_NOTICE, "Trying to read EEPROM\n"));
        ifc_fmc_eeprom_read_sig(ifcdevice, (uint8_t *)ifcdevice->fru_id->product_name);
    } else {
        switch (i32_reg_val >> 16) {
        case 0x3110:
            strcpy(ifcdevice->fru_id->product_name, "ADC3110");
            break;
        case 0x3111:
            strcpy(ifcdevice->fru_id->product_name, "ADC3111");
            break;
        case 0x3117:
            strcpy(ifcdevice->fru_id->product_name, "ADC3117");
            break;
        }
    }

    /*
     * Register the correct functions with the ifcdevice and
     * allocate all memory necessary for DMA transfers
     */
    switch (ifcdevice->app_signature) {
    case IFC1210SCOPEDRV_SCOPE_SIGNATURE:
    case IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE:
    case IFC1410SCOPEDRV_SCOPE_SIGNATURE:
        status = ifcdaqdrv_scope_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }
        break;
    
    case IFC1410SCOPEDRV_SCOPE_LITE_4CHANNELS:
        status = scope4ch_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
    
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }

        break;

    case IFC1410SCOPEDRV_SCOPE_LITE_20CHANNELS:
        status = scope20ch_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
    
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
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
    case IFC1410SCOPEDRV_SCOPE_LITE_SIGNATURE:
        status = ifcdaqdrv_scope_lite_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }
        break;       
    case IFC1410SCOPEDRV_GEN_3110_3110_SIGNATURE:
    case IFC1410SCOPEDRV_GEN_3117_3117_SIGNATURE:
        status = ifcdaqdrv_gen_scope_register(ifcdevice);
        if(status) {
            goto err_dev_alloc;
        }
        status = ifcdaqdrv_dma_allocate(ifcdevice);
        if(status) {
            goto err_read;
        }
        break;
    default:
        LOG((LEVEL_ERROR, "Application signature 0x%08x is not recognized\n", ifcdevice->app_signature));
        status = status_internal;
        goto err_read;
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
    tsc_exit(ifcdevice->node);

err_tsc_init:
    /* Unlock device list */
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
    return status;
}

/*
 * Close the device. Check if this was the last user that closed the device and in that case clean up.
 */

ifcdaqdrv_status ifcdaqdrv_close_device(struct ifcdaqdrv_usr *ifcuser)
{
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
        tsc_exit(ifcdevice->node);
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

ifcdaqdrv_status ifcdaqdrv_init_adc(struct ifcdaqdrv_usr *ifcuser)
{
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

/*
 * Arm device.
 */
ifcdaqdrv_status ifcdaqdrv_arm_device(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->arm_device) {
        return status_no_support;
    }

    return ifcdevice->arm_device(ifcuser);
}

/*
 * Disarm device
 */
ifcdaqdrv_status ifcdaqdrv_disarm_device(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->disarm_device) {
        return status_no_support;
    }

    return ifcdevice->disarm_device(ifcuser);
}

/*
 * Wait for acquisition to end.
 */
ifcdaqdrv_status ifcdaqdrv_wait_acq_end(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->wait_acq_end) {
        return status_no_support;
    }

    return ifcdevice->wait_acq_end(ifcuser);
}

/*
 * Read out samples. Call application specific function.
 */
ifcdaqdrv_status ifcdaqdrv_read_ai(struct ifcdaqdrv_usr *ifcuser, void *data)
{
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
ifcdaqdrv_status ifcdaqdrv_read_ai_ch(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, void *data)
{
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
ifcdaqdrv_status ifcdaqdrv_set_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock clock)
{
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
ifcdaqdrv_status ifcdaqdrv_get_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock *clock)
{
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
ifcdaqdrv_status ifcdaqdrv_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency)
{
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
ifcdaqdrv_status ifcdaqdrv_get_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double *frequency)
{
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
ifcdaqdrv_status ifcdaqdrv_get_clock_frequencies_valid(struct ifcdaqdrv_usr *ifcuser, double *frequencies, size_t buf_len, size_t *data_len)
{
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
ifcdaqdrv_status ifcdaqdrv_set_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t divisor)
{
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
ifcdaqdrv_status ifcdaqdrv_get_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor)
{
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
ifcdaqdrv_status ifcdaqdrv_get_clock_divisor_range(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor_min, uint32_t *divisor_max)
{
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
                                       uint32_t mask, uint32_t rising_edge)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_trigger) {
        return status_no_support;
    }

    return ifcdevice->set_trigger(ifcuser, trigger, threshold, mask, rising_edge);
}

/*
 * Get trigger configuration
 */
ifcdaqdrv_status ifcdaqdrv_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_trigger) {
        return status_no_support;
    }

    return ifcdevice->get_trigger(ifcuser, trigger, threshold, mask, rising_edge);
}

/*
 * Set average
 */
ifcdaqdrv_status ifcdaqdrv_set_average(struct ifcdaqdrv_usr *ifcuser, uint32_t average)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_average) {
        return status_no_support;
    }

    return ifcdevice->set_average(ifcdevice, average);
}

/*
 * Get average
 */
ifcdaqdrv_status ifcdaqdrv_get_average(struct ifcdaqdrv_usr *ifcuser, uint32_t *average)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_average) {
        return status_no_support;
    }

    return ifcdevice->get_average(ifcdevice, average);
}

/*
 * Get valid averages
 */
ifcdaqdrv_status ifcdaqdrv_get_averages_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *averages, size_t buf_len, size_t *data_len)
{
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
 * 
 */
ifcdaqdrv_status ifcdaqdrv_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_decimation) {
        return status_no_support;
    }

    return ifcdevice->set_decimation(ifcuser, decimation);
}

/*
 * Get decimation
 */
ifcdaqdrv_status ifcdaqdrv_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_decimation) {
        return status_no_support;
    }

    return ifcdevice->get_decimation(ifcuser, decimation);
}

/*
 * Get valid decimations.
 */
ifcdaqdrv_status ifcdaqdrv_get_decimations_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimations, size_t buf_len, size_t *data_len)
{
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
ifcdaqdrv_status ifcdaqdrv_set_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t nsamples)
{
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
ifcdaqdrv_status ifcdaqdrv_get_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t *nsamples)
{
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
 * Set pre-trigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_set_ptq(struct ifcdaqdrv_usr *ifcuser, uint32_t ptq)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_ptq) {
        return status_no_support;
    }

    return ifcdevice->set_ptq(ifcdevice, ptq);
}

/*
 * Get pre-trigger quota.
 */
ifcdaqdrv_status ifcdaqdrv_get_ptq(struct ifcdaqdrv_usr *ifcuser, uint32_t *ptq)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_ptq) {
        return status_no_support;
    }

    return ifcdevice->get_ptq(ifcdevice, ptq);
}

/*
 * Set gain
 */
ifcdaqdrv_status ifcdaqdrv_set_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double gain)
{
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
ifcdaqdrv_status ifcdaqdrv_get_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double *gain)
{
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
ifcdaqdrv_status ifcdaqdrv_get_vref_max(struct ifcdaqdrv_usr *ifcuser, double *vref_max)
{
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
ifcdaqdrv_status ifcdaqdrv_get_resolution(struct ifcdaqdrv_usr *ifcuser, uint32_t *resolution)
{
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
ifcdaqdrv_status ifcdaqdrv_set_sample_rate(struct ifcdaqdrv_usr *ifcuser, double sample_rate)
{
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
ifcdaqdrv_status ifcdaqdrv_get_sample_rate(struct ifcdaqdrv_usr *ifcuser, double *sample_rate)
{
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

ifcdaqdrv_status ifcdaqdrv_send_configuration_command(struct ifcdaqdrv_usr *ifcuser)
{
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
ifcdaqdrv_status ifcdaqdrv_set_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern pattern)
{
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
ifcdaqdrv_status ifcdaqdrv_get_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern *pattern)
{
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
ifcdaqdrv_status ifcdaqdrv_get_nchannels(struct ifcdaqdrv_usr *ifcuser, uint32_t *nchannels)
{
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
ifcdaqdrv_status ifcdaqdrv_get_manufacturer(struct ifcdaqdrv_usr *ifcuser, char *manufacturer, size_t buf_len)
{
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
ifcdaqdrv_status ifcdaqdrv_get_product_name(struct ifcdaqdrv_usr *ifcuser, char *product_name, size_t buf_len)
{
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
ifcdaqdrv_status ifcdaqdrv_get_fw_version(struct ifcdaqdrv_usr *ifcuser, uint8_t *version)
{
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
ifcdaqdrv_status ifcdaqdrv_get_fw_revision(struct ifcdaqdrv_usr *ifcuser, uint8_t *revision)
{
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
#if 0
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
#endif


ifcdaqdrv_status ifcdaqdrv_calc_sample_rate(struct ifcdaqdrv_usr *ifcuser, int32_t *averaging, int32_t *decimation, int32_t *divisor,
                                            double *freq, double *sample_rate, uint8_t sample_rate_changed)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->calc_sample_rate) {
        return status_no_support;
    }

    return ifcdevice->calc_sample_rate(ifcuser, averaging, decimation, divisor, freq, sample_rate, sample_rate_changed);
}

ifcdaqdrv_status ifcdaqdrv_set_digiout(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, uint32_t value)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_digiout) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = ifcdevice->set_digiout(ifcuser->device, channel, value);
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}


ifcdaqdrv_status ifcdaqdrv_get_digiout(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, uint32_t *value)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->get_digiout) {
        return status_no_support;
    }

    return ifcdevice->get_digiout(ifcuser->device, channel, value);
}

ifcdaqdrv_status ifcdaqdrv_is_bigendian(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!ifcdevice->set_digiout) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = (ifcdaqdrv_status) ifcdaqdrv_is_byte_order_ppc();
    
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

ifcdaqdrv_status ifcdaqdrv_subs_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    struct tsc_ioctl_user_irq tscirq;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    tscirq.irq  = irqn;
    tscirq.mask = 1 <<irqn;
    tscirq.wait_mode = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4);

    status = tsc_user_irq_subscribe(ifcdevice->node, &tscirq);

    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

ifcdaqdrv_status ifcdaqdrv_unsubs_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    struct tsc_ioctl_user_irq tscirq;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    tscirq.irq  = irqn;
    tscirq.mask = 1 <<irqn;
    tscirq.wait_mode = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4);

    status = tsc_user_irq_unsubscribe(ifcdevice->node, &tscirq);

    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}

ifcdaqdrv_status ifcdaqdrv_wait_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    struct tsc_ioctl_user_irq tscirq;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    tscirq.irq  = irqn;
    tscirq.mask = 1 <<irqn;
    tscirq.wait_mode = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4);

    status = tsc_user_irq_wait(ifcdevice->node, &tscirq);    

    return status;
}

/* ##################################################################################### */
/*          For compatibility with IFC14edrv                                             */
/* ##################################################################################### */

/*
 * Set number of pre-trigger samples.
 */
ifcdaqdrv_status ifcdaqdrv_set_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t npretrig)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (ifcdevice->board_id == 0x3117) {
        LOG((LEVEL_DEBUG, "Not valid operation on ADC3117\n"));
        return status_no_support;
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
ifcdaqdrv_status ifcdaqdrv_get_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t *npretrig)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (ifcdevice->board_id == 0x3117) {
        LOG((LEVEL_DEBUG, "Not valid operation on ADC3117\n"));
        *npretrig = 0;
        return status_no_support;
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

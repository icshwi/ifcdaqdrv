#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <time.h>


// #include <pevioctl.h>
// #include <pevxulib.h>

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_sim.h"
// #include "ifcdaqdrv_utils.h"
// #include "ifcdaqdrv_scope.h"
// #include "ifcdaqdrv_fmc.h"
// #include "ifcdaqdrv_acq420.h"
// #include "ifcdaqdrv_adc3110.h"
// #include "ifcfastintdrv.h"
// #include "ifcfastintdrv_utils.h"

LIST_HEAD(ifcdaqdrv_devlist);
pthread_mutex_t ifcdaqdrv_devlist_lock = PTHREAD_MUTEX_INITIALIZER;

/*
 * Open device. Keep a list of users to support multiple open calls in the same process
 */

ifcdaqdrv_status ifcdaqdrv_open_device(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    //int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    if (!ifcuser || ifcuser->card >= MAX_PEV_CARDS || (ifcuser->fmc != 1 && ifcuser->fmc != 2)) {
        return status_argument_invalid;
    }

    ifcdaqdrvDebug = 0;
    LOG((5, "Level %d tracing set.\n", ifcdaqdrvDebug));

    pthread_mutex_lock(&ifcdaqdrv_devlist_lock);

    /* Check if device already is opened in list. */
    list_for_each_entry(ifcdevice, &ifcdaqdrv_devlist, list){
        if (ifcdevice->card == ifcuser->card && ifcdevice->fmc == ifcuser->fmc) {

            ifcuser->device = ifcdevice;
            ifcdevice->count++;
            pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
            return status_success;
        }
    }

    /* Allocate private structure */
    ifcdevice = calloc(1, sizeof(struct ifcdaqdrv_dev));
    if (!ifcdevice) {
        status = status_internal;
        goto err_dev_alloc;
    }

    ifcdevice->card  = ifcuser->card;
    ifcdevice->fmc   = ifcuser->fmc;
    ifcdevice->count = 1;

    pthread_mutex_init(&ifcdevice->lock, NULL);

    ifcuser->device = ifcdevice;

    /* Register simulated board */
    status = ADCSim_register(ifcdevice);
    if (status)
    {
        goto err_read;
    }

    /* Allocate memory */
    status = ifcdaqdrv_dma_allocate(ifcdevice);
    if(status) {
        goto err_read;
    }

    /* Add device to the list of opened devices */
    list_add_tail(&ifcdevice->list, &ifcdaqdrv_devlist);
    pthread_mutex_unlock(&ifcdaqdrv_devlist_lock);
    return status_success;

err_read:
    /* Free ifcdevice (This will also free fru_id for us. */
    ifcdaqdrv_free(ifcdevice);

err_dev_alloc:
// /* Close pevx library */
// err_pevx_init:

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
    int i;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // set configurations
    ifcdevice->rt_clock_source = ifcdaqdrv_clock_internal;
    ifcdevice->rt_clock_frequency = 250e6;
    ifcdevice->rt_clock_divisor = 1;

    // set gain to 1
    for (i=0; i<8; i++)
        ifcdevice->rt_channel_gain[i] = 1.0;

    // set unsigned data format
    ifcdevice->rt_dataformat = ifcdaqdrv_dataformat_unsigned;    

    ifcdevice->init_called++;

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
ifcdaqdrv_status ifcdaqdrv_arm_device(struct ifcdaqdrv_usr *ifcuser){
    //ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    //int32_t               i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(ifcdevice->init_called == 0){
        fprintf(stderr, "WARNING: init_adc() has not been called\n");
    }

    pthread_mutex_lock(&ifcdevice->lock);

    ifcdevice->armed = 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Disarm device
 */
ifcdaqdrv_status ifcdaqdrv_disarm_device(struct ifcdaqdrv_usr *ifcuser){
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    // int32_t               i32_reg_val;
    // int32_t               i;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

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
    // ifcdaqdrv_status      status;
    // int32_t               i32_reg_val;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->databuffer) {
        return status_no_device;
    }

    srand((unsigned int) time(NULL));

    uint32_t i;
    for (i = 0; i < ifcdevice->buffersize; i++)
    {
        ifcdevice->databuffer[i] = (int32_t) ((rand()%65536) - 32768);
    }

    if (!ifcdevice->armed) {
        return status_cancel;
    }

    ifcdaqdrv_disarm_device(ifcuser);
    return status_success;
}

/*
 * Read out samples. Call FMC specific function.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai(struct ifcdaqdrv_usr *ifcuser, void *data) {
    // ifcdaqdrv_status      status;
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

    // if(ifcdevice->read_ai) {
    //     status = ifcdevice->read_ai(ifcdevice, data);
    //     pthread_mutex_unlock(&ifcdevice->lock);
    //     return status;
    // }

    if (!ifcdevice->databuffer)
    {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_no_support;
    }

    /* Fill the data pointer with databuffer data */
    uint32_t totalsamples = ifcdevice->nchannels * ifcdevice->rt_nsamples;
    int32_t *target = (int32_t*) data;
   
    memcpy(target, ifcdevice->databuffer, totalsamples*sizeof(int32_t));

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

/*
 * Read out samples for one channel. Call FMC specific function.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai_ch(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, void *data) {
    // ifcdaqdrv_status      status;
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

    // if(ifcdevice->read_ai_ch) {
    //     status = ifcdevice->read_ai_ch(ifcdevice, channel, data);
    //     pthread_mutex_unlock(&ifcdevice->lock);
    //     return status;
    // }

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

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->rt_clock_source = clock;
    status = status_success;

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

    *clock = ifcdevice->rt_clock_source;

    return status_success;
}

/*
 * Set clock frequency.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency) {
    // ifcdaqdrv_status      status;
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

    ifcdevice->rt_clock_frequency = frequency;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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
    if (!frequency) {
        return status_argument_invalid;
    }

    *frequency = ifcdevice->rt_clock_frequency;

    return status_success;
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
    // ifcdaqdrv_status      status;
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

    ifcdevice->rt_clock_divisor = divisor;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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

    if (!ifcdevice->rt_clock_divisor)
        ifcdevice->rt_clock_divisor = 1;

    *divisor = ifcdevice->rt_clock_divisor;

    return status_success;
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
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    // int32_t               i32_cs_val   = 0; // Control & Status value
    // int32_t               i32_trig_val = 0; // Trigger value
    // uint32_t              channel      = 0;
    // uint32_t              gpio         = 0;
    // uint32_t              channel_mask = 0;
    // uint32_t              ptq          = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->trigger_type = trigger;

    ifcdevice->rt_tr_trigger = trigger;
    ifcdevice->rt_tr_edge = rising_edge;
    ifcdevice->rt_tr_threshold = threshold;
    ifcdevice->rt_tr_mask = mask;

    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}

/*
 * Get trigger configuration
 */

ifcdaqdrv_status ifcdaqdrv_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Lock device so that the trigger configuration can be read out atomically. */
    pthread_mutex_lock(&ifcdevice->lock);

    if (trigger) {
        *trigger = ifcdevice->trigger_type;
    }

    *rising_edge = ifcdevice->rt_tr_edge;
    *threshold = ifcdevice->rt_tr_threshold;
    *mask = ifcdevice->rt_tr_mask;

    return status_success;
}

/*
 * Set average
 */

ifcdaqdrv_status ifcdaqdrv_set_average(struct ifcdaqdrv_usr *ifcuser, uint32_t average) {
    struct ifcdaqdrv_dev *ifcdevice;
    // ifcdaqdrv_status status;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if (average == 0)
        return status_argument_range;

    ifcdevice->rt_average = average;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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

    *average = ifcdevice->rt_average;

    return status_success;

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
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    // int32_t               i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (decimation == 0) {
        return status_argument_invalid;
    }

    // Decimation is not supported when averaging is enabled.

    // Check if decimation is valid 
    for (i = 0; i < MAX_DECIMATIONS; ++i) {
        if (ifcdevice->decimations[i] == decimation) {
            pthread_mutex_lock(&ifcdevice->lock);

            if (ifcdevice->armed) {
                pthread_mutex_unlock(&ifcdevice->lock);
                return status_device_armed;
            }

            ifcdevice->rt_decimation = decimation;
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_success;
        }
    }
    return status_argument_invalid;
}

/*
 * Get decimation
 */

ifcdaqdrv_status ifcdaqdrv_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation) {
    struct ifcdaqdrv_dev *ifcdevice;
    // int32_t               i32_reg_val;
    // ifcdaqdrv_status      status;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!decimation) {
        return status_argument_invalid;
    }

    if (!ifcdevice->rt_decimation)
        ifcdevice->rt_decimation = 1;

    *decimation = ifcdevice->rt_decimation;

    return status_success;
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
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Check if nsamples is power of two */
    if ((nsamples & (nsamples - 1)) != 0) {
        return status_argument_range;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }
    
    ifcdevice->rt_nsamples = nsamples;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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
    
    *nsamples = ifcdevice->rt_nsamples;

    return status_success;
}

/*
 * Set number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_set_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t npretrig) {
    // ifcdaqdrv_status      status;
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

    ifcdevice->rt_npretrig = npretrig;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/*
 * Get number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_get_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t *npretrig) {
    // ifcdaqdrv_status      status;
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
    
    *npretrig = ifcdevice->rt_npretrig;

    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}

/*
 * Set gain
 */

ifcdaqdrv_status ifcdaqdrv_set_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double gain) {
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    // if (!ifcdevice->set_gain) {
    //     return status_no_support;
    // }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    ifcdevice->rt_channel_gain[channel] = gain;    

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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

    if (channel > 7)
        return status_argument_invalid;

    if (!ifcdevice->rt_channel_gain[channel]) {
        ifcdevice->rt_channel_gain[channel] = 1.0;
    }

    *gain = ifcdevice->rt_channel_gain[channel];

    return status_success;
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
    // ifcdaqdrv_status      status;
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

    //status = ifcdevice->set_pattern(ifcuser->device, channel, pattern);
    ifcdevice->rt_ch_pattern[channel] = pattern;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
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
    // if (!ifcdevice->get_pattern) {
    //     *pattern = ifcdaqdrv_pattern_none;
    //     return status_success;
    // }

    *pattern = ifcdevice->rt_ch_pattern[channel];

    return status_success;
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
    char *simulated = "ESS ICS";

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!manufacturer) {
        return status_argument_invalid;
    }

    manufacturer[0] = '\0';
    strncat(manufacturer, simulated, buf_len - 1);

    if (buf_len < strlen(simulated)) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Get product name string
 */

ifcdaqdrv_status ifcdaqdrv_get_product_name(struct ifcdaqdrv_usr *ifcuser, char *product_name, size_t buf_len){
    struct ifcdaqdrv_dev *ifcdevice;
    char *simulated = "SIM ADC";

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }
    if (!product_name) {
        return status_argument_invalid;
    }

    product_name[0] = '\0';
    strncat(product_name, simulated, buf_len - 1);

    if (buf_len < strlen(simulated)) {
        return status_buf_len;
    }

    return status_success;
}

/*
 * Get fw version
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_version(struct ifcdaqdrv_usr *ifcuser, uint8_t *version) {
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!version) {
        return status_argument_invalid;
    }

    *version = 0;

    //status = ifcdevice->get_signature(ifcdevice, 0, version, 0);

    return status_success;
}

/*
 * Get fw revision
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_revision(struct ifcdaqdrv_usr *ifcuser, uint8_t *revision) {
    // ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!revision) {
        return status_argument_invalid;
    }

    //status = ifcdevice->get_signature(ifcdevice, revision, 0, 0);
    *revision = 0;

    return status_success;
}

/*
 * Non-supported debug function
 */

ifcdaqdrv_status ifcdaqdrv_debug(struct ifcdaqdrv_usr *ifcuser) {
    struct ifcdaqdrv_dev *ifcdevice;
    int i;
    int32_t               i32_reg_val = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    for (i = 0; i < 8; ++i) {
        //ifc_fmc_tcsr_read(ifcdevice, i, &i32_reg_val);
        printf("[0x%02x]: 0x%08x\n", i, i32_reg_val);
    }
    return status_success;
}

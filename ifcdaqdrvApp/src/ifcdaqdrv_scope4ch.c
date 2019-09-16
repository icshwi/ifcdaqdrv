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
#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_scope_lite.h"
#include "ifcdaqdrv_scope4ch.h"


ifcdaqdrv_status scope4ch_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    uint32_t nsamples_max = SCOPE4CH_MAX_SAMPLES; //131072 samples

    status = adc3117_get_signature(ifcdevice, NULL, NULL, &ifcdevice->board_id);
    if (status)
        return status;

    /* Activate FMC */
    status = ifc_fmc_tcsr_write(ifcdevice, ADC3117_SIGN_REG, 0x31170000);
    usleep(200000);	
    
    ifcdevice->init_adc              = adc3117_init_adc;
    ifcdevice->get_signature         = adc3117_get_signature;
    ifcdevice->set_led               = adc3117_set_led;
    ifcdevice->get_gain              = adc3117_get_gain;
    ifcdevice->set_gain              = adc3117_set_gain;
    ifcdevice->get_offset            = adc3117_get_offset;
    ifcdevice->set_offset            = adc3117_set_offset;
    ifcdevice->get_nsamples          = scope4ch_get_nsamples;
    ifcdevice->set_nsamples          = scope4ch_set_nsamples;
    ifcdevice->set_trigger_threshold = NULL;
    ifcdevice->get_trigger_threshold = NULL;
    ifcdevice->set_clock_frequency   = adc3117_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3117_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3117_set_clock_source;
    ifcdevice->get_clock_source      = adc3117_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3117_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3117_get_clock_divisor;

    ifcdevice->arm_device            = scope4ch_arm_acquisition;
    ifcdevice->disarm_device         = scope4ch_disarm_acquisition;
    ifcdevice->wait_acq_end          = scope4ch_wait_acq_end;

    ifcdevice->set_pattern           = adc3117_set_test_pattern;
    ifcdevice->get_pattern           = adc3117_get_test_pattern;

    ifcdevice->read_ai_ch            = scope4ch_read_ai_ch;
    ifcdevice->read_ai               = scope4ch_read_allchannels;

    ifcdevice->normalize_ch          = scope4ch_normalize_ch;
    ifcdevice->normalize             = ifcdaqdrv_scope_read;

    ifcdevice->mode_switch           = NULL; 

    ifcdevice->set_adc_channel       = adc3117_set_adc_channel;
    ifcdevice->get_adc_channel       = adc3117_get_adc_channel;
    ifcdevice->set_adc_channel_mask  = adc3117_set_adc_channel_mask;
    ifcdevice->get_adc_channel_mask  = adc3117_get_adc_channel_mask;

    ifcdevice->set_adc_channel_negative_input = adc3117_set_adc_channel_negative_input;
    ifcdevice->get_adc_channel_negative_input = adc3117_get_adc_channel_negative_input;
    ifcdevice->set_adc_channel_positive_input = adc3117_set_adc_channel_positive_input;
    ifcdevice->get_adc_channel_positive_input = adc3117_get_adc_channel_positive_input;

    ifcdevice->set_sample_rate       = adc3117_set_sample_rate;
    ifcdevice->get_sample_rate       = adc3117_get_sample_rate;
    ifcdevice->calc_sample_rate      = NULL;

    ifcdevice->trigger_type     = ifcdaqdrv_trigger_soft;
    ifcdevice->set_trigger      = scope4ch_set_trigger;
    ifcdevice->get_trigger      = scope4ch_get_trigger;

    ifcdevice->set_average      = NULL;
    ifcdevice->get_average      = NULL;
    ifcdevice->set_decimation   = NULL;
    ifcdevice->get_decimation   = NULL;
    ifcdevice->set_ptq          = NULL;
    ifcdevice->get_ptq          = NULL;

    ifcdevice->configuration_command = adc3117_configuration_command;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 4;
    ifcdevice->resolution  = 16;
    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 10.24;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 10;

    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size * ifcdevice->nchannels;
    ifcdevice->smem_size = nsamples_max * ifcdevice->sample_size * ifcdevice->nchannels;
    ifcdevice->smem_sg_dma = 0;

#if 0
    /* Remote procdure call functions */
    ifcdevice->write_generic = scope4ch_write_generic;
    ifcdevice->read_generic = scope4ch_read_generic;
#endif

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

ifcdaqdrv_status scope4ch_arm_acquisition(struct ifcdaqdrv_usr *ifcuser) 
{
    ifcdaqdrv_status status;
    struct ifcdaqdrv_dev *ifcdevice;
    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    INFOLOG(("Arming acquisition ...\n"));

    /* autorun (reg 0x62 bit 1) is enabled by default 
    * we need to set bit 1 of register 0x69 (loc_SBUF_CTL(i).RUN <=  aps_TCSR_DATW( 1);)
    */
    ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_REG_69, 1<<1, 0x00);
    ifcdevice->armed = 1;

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;
}


/*
 * Disarm device
 */
ifcdaqdrv_status scope4ch_disarm_acquisition(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->armed)
        return status_success;

    pthread_mutex_lock(&ifcdevice->lock);

    INFOLOG(("Disarming acquisition\n"));

    /* Register 0x66 
    *   bit 16 -> reset acquisition
    *   bit 12 -> ack interruption
    */
    ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 0x11000, 0x00);

    /* Register 0x69 bit 2 -> halt */
    ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_SBUF_CONTROL_STATUS_REG, 0x4);

    /* Register 0x62 */
    ifc_scope_lite_tcsr_write(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG, 0x0);

    ifcdevice->armed = 0;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/* Using interrupts to wait for acquisition */
ifcdaqdrv_status scope4ch_wait_acq_end(struct ifcdaqdrv_usr *ifcuser)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;

    if (!ifcdevice) {
        return status_no_device;
    }

    /* Subscribe to interrupt number ZERO */
    status = ifcdaqdrv_subs_intr(ifcuser, 0);
    if (status) {
        ifcdaqdrv_disarm_device(ifcuser);
        return status;        
    }

    /* If software trigger, generate it manually */
    if (ifcdevice->trigger_type == ifcdaqdrv_trigger_soft)
        ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 1<<8, 0x00); // register 0x66 bit 8

    INFOLOG(("Waiting for interrupt...\n"));

    /* Wait for interrupt */
    status = ifcdaqdrv_wait_intr(ifcuser, 0);
    if (status) {
        /* Interrupt never happened - generate soft trigger and catch new interrupt */
        ifcdaqdrv_subs_intr(ifcuser, 0);
        ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 1<<8, 0x00); // register 0x66 bit 8
        ifcdaqdrv_wait_intr(ifcuser, 0);
    }

    ifcdaqdrv_disarm_device(ifcuser);
    return status_success;
}


ifcdaqdrv_status scope4ch_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge)
{
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

    /* There are only two options: backplane or software trigger */
    switch (trigger) {
        case ifcdaqdrv_trigger_backplane:

            INFOLOG(("Selecting backplane trigger with line %d", (mask&0x0f)));

            ifcdevice->trigger_type = ifcdaqdrv_trigger_backplane;
            
            /* Enable MLVDS lines*/
            ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_MLVDS_CONTROL_REG, 1<<IFC_SCOPE_MLVDS_ENABLE_SHIFT, 0);

            /* Select trigger line in register 0x66 (mask should contain the backplane line)*/
            ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, (mask & 0x0f), 0x0f);
            break;
        
        /* Treat any other trigger as soft trigger */    
        default:
            ifcdevice->trigger_type = ifcdaqdrv_trigger_soft;

            INFOLOG(("Selecting SOFTWARE trigger\n"));

            /* disable MLVDS lines */
            ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_MLVDS_CONTROL_REG, 0, 1<<IFC_SCOPE_MLVDS_ENABLE_SHIFT);

            /* reset bits 7:0 of register 0x66 */
            ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 0x00, 0x0f);
            break;
    }
  
    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}


/*
 * Get trigger configuration
 */
ifcdaqdrv_status scope4ch_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge)
{
    ifcdaqdrv_status      status;
    int32_t               i32_trig_val = 0; // Trigger value
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Lock device so that the trigger configuration can be read out atomically. */
    pthread_mutex_lock(&ifcdevice->lock);

    *trigger = ifcdevice->trigger_type;
    status = ifc_xuser_tcsr_read(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, &i32_trig_val);
        if (status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;        
    }

    pthread_mutex_unlock(&ifcdevice->lock);

    /* Return the backplane trigger line using mask variable */
    if (ifcdevice->trigger_type == ifcdaqdrv_trigger_backplane)
        *mask = (uint32_t) (i32_trig_val & 0x0f);
    else
        *mask = 0;

    return status;
}



#if 0
ifcdaqdrv_status scope4ch_write_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data)
{
    ifcdaqdrv_status status;
    uint32_t *valid_data;

    switch (function) {
        case SCOPE4CH_WRITE_ENABLE_BACKPLANE:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_enable_backplane(ifcdevice, *valid_data);
            break;

        case SCOPE4CH_WRITE_DISABLE_BACKPLANE:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_disable_backplane(ifcdevice, *valid_data);
            break;

        case SCOPE4CH_WRITE_ACK_ACQUISITION:
            status = scope4ch_ack_acquisition(ifcdevice);
            break;

        case SCOPE4CH_WRITE_SOFT_TRIGGER:
            status = scope4ch_generate_trigger(ifcdevice);
            break;
        
        default:
            status = status_no_support;
            break;
    }
    
    return status;
}

ifcdaqdrv_status scope4ch_read_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data)
{
    ifcdaqdrv_status status;
    uint32_t *valid_data;
    
    switch (function) {
        case SCOPE4CH_READ_TRIGGER_COUNT:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_read_backplane_trgcnt(ifcdevice, valid_data);
            break;

        case SCOPE4CH_READ_ACQ_COUNT:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_read_acq_count(ifcdevice, valid_data);
            break;

        case SCOPE4CH_READ_SCOPE_STATUS:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_read_scopestatus(ifcdevice, valid_data);
            break;

        case SCOPE4CH_READ_ACQ_DONE:
            if (!data) {
                status = status_argument_invalid;
                break;
            }
            valid_data = (uint32_t*) data;
            status = scope4ch_read_acqdone(ifcdevice, valid_data);
            break;

        default:
            status = status_no_support;
            break;
    }
    
    return status;
}
#endif

ifcdaqdrv_status scope4ch_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){

    // Returns the maximum possible amount of samples
    *nsamples_max = ifcdevice->sram_size / ifcdevice->nchannels / ifcdevice->sample_size;

    return status_success;
}

ifcdaqdrv_status scope4ch_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples)
{
    if (nsamples > (ifcdevice->sram_size / ifcdevice->nchannels / ifcdevice->sample_size))
        return status_argument_range;
    else
        return status_success;
}

ifcdaqdrv_status scope4ch_enable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines) 
{
    ifcdaqdrv_status status;
    backplane_lines = backplane_lines & 0xff;
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, backplane_lines, 0x00);
    return status;
}

ifcdaqdrv_status scope4ch_disable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines) 
{
    ifcdaqdrv_status status;
    backplane_lines = backplane_lines & 0xff;
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 0x00, backplane_lines);
    return status;
}


ifcdaqdrv_status scope4ch_read_backplane_trgcnt(struct ifcdaqdrv_dev *ifcdevice, uint32_t *trig_cnt) 
{
    ifcdaqdrv_status status;
    int32_t i32_reg_val;
    status = ifc_xuser_tcsr_read(ifcdevice, IFC_SCOPE_BACKPLANE_TRIGCNT_REG, &i32_reg_val);
    *trig_cnt = (uint32_t) (i32_reg_val >> 16) & 0x0f;
    return status;
}


ifcdaqdrv_status scope4ch_read_acq_count(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acq_cnt) 
{
    ifcdaqdrv_status status;
    int32_t i32_reg_val;
    status = ifc_xuser_tcsr_read(ifcdevice, IFC_SCOPE_BACKPLANE_TRIGCNT_REG, &i32_reg_val);
    *acq_cnt = (uint32_t) (i32_reg_val >> 8) & 0x0f;
    return status;
}


ifcdaqdrv_status scope4ch_ack_acquisition(struct ifcdaqdrv_dev *ifcdevice) 
{
    ifcdaqdrv_status status;
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 0x110FF, 0x00);
    return status;
}




ifcdaqdrv_status scope4ch_generate_trigger(struct ifcdaqdrv_dev *ifcdevice) 
{
    ifcdaqdrv_status status;
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFC_SCOPE_BACKPLANE_MASK_REG, 1<<8, 0x00);
    return status;
}


ifcdaqdrv_status scope4ch_read_scopestatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t *scopest) 
{
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_xuser_tcsr_read(ifcdevice, 0x66, &i32_reg_val);
    *scopest = (uint32_t) (i32_reg_val >> 12) & 0x0f;

    return status;
}

ifcdaqdrv_status scope4ch_read_acqdone(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acqdone) 
{
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_xuser_tcsr_read(ifcdevice, 0x69, &i32_reg_val);
    *acqdone = (uint32_t) (i32_reg_val >> 7) & 0x01;

    return status;
}


/* Simplified version of ifcdaqdrv_scope_read_ai function */
ifcdaqdrv_status scope4ch_read_allchannels(struct ifcdaqdrv_dev *ifcdevice, void *data) {
    //ifcdaqdrv_status status;
    uint32_t nsamples = 0;
    uint32_t channel;

    /* TODO: fix NSAMPLES - currently will always return max samples */
    ifcdevice->get_nsamples(ifcdevice, &nsamples);

    /* Read the channels sequentially */
    for(channel = 0; channel < ifcdevice->nchannels; ++channel) {
        ifcdevice->read_ai_ch(ifcdevice, channel, ((int32_t *)data) + nsamples * channel);
    }

    return status_success;
}

/* Simplified version of ifcdaqdrv_scope_read_ai_ch function */
ifcdaqdrv_status scope4ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data) {
    ifcdaqdrv_status  status;
    int32_t offset;
    int16_t *origin;
    int32_t *res;
    uint32_t last_address,  nsamples;

    offset = 0;
    origin = NULL;
    res = data;
    last_address = 0;
    nsamples = 0;

    INFOLOG(("Entering scope4ch_read_ai_ch for channel %d", channel));

    /* TODO: fix NSAMPLES - currently will always return max samples */
    ifcdevice->get_nsamples(ifcdevice, &nsamples);

    /* ------------ ADDRESS DEFINITIONS -------------- 
    * This customized version of the SCOPE_LITE firmware stores data in the following format:
    * FMC 1 channel 0 -> 0x00100000 to 0x0013ffff
    * FMC 1 channel 1 -> 0x00140000 to 0x0017ffff
    * FMC 1 channel 2 -> 0x00200000 to 0x0023ffff
    * FMC 1 channel 3 -> 0x00240000 to 0x0027ffff
    */

    if (channel < 2) {
        offset = IFC_SCOPE4CH_FMC1_SRAM_SAMPLES_OFFSET + (channel * 0x40000);
    } else {
        offset = IFC_SCOPE4CH_FMC2_SRAM_SAMPLES_OFFSET + (channel * 0x40000);
    }

    status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
    if (status) {
        return status;
    }
    
    /* TODO: port this function to this file */
    status = ifcdaqdrv_scope_get_sram_la(ifcdevice, &last_address);
    if (status) {
        return status;
    }

    if (ifcdaqdrv_is_byte_order_ppc())
        ifcdaqdrv_manualswap((uint16_t*) ifcdevice->sram_dma_buf->u_base,nsamples);

    origin   = ifcdevice->sram_dma_buf->u_base;

    /* Copy from end of pretrig buffer to end of samples */
    status = ifcdevice->normalize_ch(ifcdevice, channel, res, origin, 0, nsamples);
    if (status) {
        return status;
    }

    return status_success;
}

/* Simplified version of ifcdaqdrv_scope_read_ch function (ifcdevice->normalize_ch) */
ifcdaqdrv_status scope4ch_normalize_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm) {
    int16_t *origin = (int16_t *)data + offset;
    int16_t *itr;
    int32_t *target = res;

    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = (int16_t)(*itr);
    }
    return status_success;
}

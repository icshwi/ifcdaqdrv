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
#include "ifcdaqdrv_scope4ch.h"
#include "ifcdaqdrv_scope20ch.h"


/* Very similar to scope4ch_register because the firmware is almost the same */
ifcdaqdrv_status scope20ch_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    uint32_t nsamples_max = SCOPE20CH_MAX_SAMPLES; //2048 samples

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

    ifcdevice->read_ai_ch            = scope20ch_read_ai_ch;
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
    ifcdevice->nchannels   = 20;
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

/* Simplified version of ifcdaqdrv_scope_read_ai_ch function */
ifcdaqdrv_status scope20ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data) {
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

    /* TODO: fix NSAMPLES - currently will always return max samples */
    ifcdevice->get_nsamples(ifcdevice, &nsamples);

    /* ADDRESS DEFINITIONS */
    if (ifcdevice->fmc == 1) {
        offset = IFC_SCOPE4CH_FMC1_SRAM_SAMPLES_OFFSET + (channel << 12);
    } else {
        offset = IFC_SCOPE4CH_FMC2_SRAM_SAMPLES_OFFSET + (channel << 12);
    }

    status = ifcdaqdrv_read_sram_unlocked(ifcdevice, ifcdevice->sram_dma_buf, offset, nsamples * ifcdevice->sample_size);
    if (status) {
        return status;
    }
    
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


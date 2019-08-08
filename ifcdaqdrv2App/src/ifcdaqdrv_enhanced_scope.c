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
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3117.h"
#include "ifcdaqdrv_scope.h"
#include "ifcdaqdrv_enhanced_scope.h"


ifcdaqdrv_status enhanced_scope_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    uint32_t nsamples_max = 0;

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
    ifcdevice->get_nsamples          = adc3117_get_nsamples;
    ifcdevice->set_trigger_threshold = adc3117_set_trigger_threshold;
    ifcdevice->get_trigger_threshold = adc3117_get_trigger_threshold;
    ifcdevice->set_clock_frequency   = adc3117_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3117_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3117_set_clock_source;
    ifcdevice->get_clock_source      = adc3117_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3117_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3117_get_clock_divisor;

    ifcdevice->set_pattern           = adc3117_set_test_pattern;
    ifcdevice->get_pattern           = adc3117_get_test_pattern;

    ifcdevice->read_ai_ch            = ifcdaqdrv_scope_read_ai_ch;
    ifcdevice->read_ai               = ifcdaqdrv_scope_read_ai;

    ifcdevice->normalize_ch          = ifcdaqdrv_scope_read_ch;
    ifcdevice->normalize             = ifcdaqdrv_scope_read;

    ifcdevice->mode_switch           = ifcdaqdrv_scope_switch_mode; // Changed default mode to SMEM

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

    ifcdevice->configuration_command = adc3117_configuration_command;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 20;
    ifcdevice->resolution  = 16;
    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 10.24;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 10;

	nsamples_max = 4096;

    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size * ifcdevice->nchannels;
    ifcdevice->smem_size = 4 * 1024 * 1024;
    ifcdevice->smem_sg_dma = 0;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

/* Functions for accessing any ENHANCED SCOPE XUSER TCSR - 0x60 0x7F*/

ifcdaqdrv_status ifc_enhscope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val) {
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_enhscope_fmc_offset(ifcdevice), i32_reg_val);
}

ifcdaqdrv_status ifc_enhscope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value) {
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_enhscope_fmc_offset(ifcdevice), value);
}

ifcdaqdrv_status ifc_enhscope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask) {
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, register_idx + ifc_get_enhscope_fmc_offset(ifcdevice), setmask, clrmask);
}


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
#include "ifcdaqdrv_scope_lite.h" /* Remove when get/set sampling rate is fixed */

static ifcdaqdrv_status adc3117_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){
    int32_t i32_reg_val;
    int     status;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_MCSR_REG, &i32_reg_val);

    switch ((i32_reg_val & 0x0C000000) >> 26) {
    case 0:
        *nsamples_max = 2 * 1024;
        break;
    case 1:
        *nsamples_max = 4 * 1024;
        break;
    case 2:
        *nsamples_max = 8 * 1024;
        break;
    default:
        printf("Invalid DPRAM buffer size!\n");
    }

    *nsamples_max = 128 * 1024;

    return status;
}

static ifcdaqdrv_status adc3117_get_smem_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){

    *nsamples_max = ifcdevice->smem_size / ifcdevice->nchannels / ifcdevice->sample_size;

    return status_success;
}

ifcdaqdrv_status adc3117_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;

    status = adc3117_get_signature(ifcdevice, NULL, NULL, &ifcdevice->board_id);
    if (status)
        return status;

    /* Activate FMC */
    status = ifc_fmc_tcsr_write(ifcdevice, ADC3117_SIGN_REG, 0x31170000);
    //usleep(200000);	
    
    ifcdevice->init_adc              = adc3117_init_adc;
    ifcdevice->get_signature         = adc3117_get_signature;
    ifcdevice->set_led               = adc3117_set_led;
    ifcdevice->get_gain              = adc3117_get_gain;
    ifcdevice->set_gain              = adc3117_set_gain;
    ifcdevice->get_offset            = adc3117_get_offset;
    ifcdevice->set_offset            = adc3117_set_offset;
    ifcdevice->get_nsamples          = adc3117_get_nsamples;
    ifcdevice->set_clock_frequency   = adc3117_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3117_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3117_set_clock_source;
    ifcdevice->get_clock_source      = adc3117_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3117_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3117_get_clock_divisor;
    ifcdevice->set_pattern           = adc3117_set_test_pattern;
    ifcdevice->get_pattern           = adc3117_get_test_pattern;
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

    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 20;
    ifcdevice->resolution  = 16;
    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 10.24;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 10;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);
   
    return status;
}

ifcdaqdrv_status adc3117_init_adc(struct ifcdaqdrv_dev *ifcdevice){
    ifcdaqdrv_status status;

    if (ifcdevice->board_id != 0x3117) {
        printf("Error: %s: No ADC3117 installed on fmc%d [%04x]", __FUNCTION__, ifcdevice->fmc, ifcdevice->board_id);
        return status_incompatible;
    }

    status = adc3117_fmc_reinit(ifcdevice); // Enable XRA1405 sequencer
    if (status)
        return status;
    usleep(1000);

    status = adc3117_set_adc_channel_mask(ifcdevice, 0xFFFFF); // Select all channels
    if (status)
        return status;

/*    status = ifc_fmc_tcsr_write(ifcdevice, ADC3117_SERIAL_BUS_DATA_REG, 0x80); //0x80=5V, 0x30=2V
    if (status)
        return status;
    status = ifc_fmc_tcsr_write(ifcdevice, ADC3117_SERIAL_BUS_CONTROL_REG, 0xCE000000);
    if (status)
        return status;
    usleep(1000);*/

    status = ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_GENERAL_CONFIG_REG, 0x1, 0x3); // VCAL = 0x1 Ref 4.128V, 0x0 var_ref
    if (status)
        return status;

    status = adc3117_set_adc_channel_negative_input(ifcdevice, FROM_CONNECTOR); // ADC channel negative input
    if (status)
        return status;

    status = adc3117_set_adc_channel_positive_input(ifcdevice, FROM_CONNECTOR); // ADC channel positive input
    if (status)
        return status;

    status = adc3117_set_gain(ifcdevice, 0xFFFFF000, 0x1); // Disable gain all channels
    if (status)
        return status;

    status = adc3117_set_offset(ifcdevice, 0x0000); // Disable offset
    if (status)
        return status;

    status = adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_color_green); // Green led on
    status += adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_off); // Red led off
    if (status)
        return status;

    status = adc3117_configuration_command(ifcdevice); // Send config
    if (status)
        return status;

    printf("ADC3117 was correctly initiated!\n");
    

    return status;
}

ifcdaqdrv_status adc3117_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state){
    uint32_t reg   = 0;
    uint32_t value = 0;
    uint32_t mask  = 0;

    if (led == ifcdaqdrv_led_ifc) {
        reg = ADC3117_SUPPORT_REG;
        mask = ADC3117_SUPPORT_IFC_LED_MASK;

        switch (led_state) {
        case ifcdaqdrv_led_color_green:
            value = 0x1;
            break;
        case ifcdaqdrv_led_color_red:
            value = 0x2;
            break;
        case ifcdaqdrv_led_off:
            value = 0x0;
            break;
        default:
            printf("Error: %s: Unknown led state %d", __FUNCTION__, led_state);
            return status_argument_range;
        }
    } else if (led == ifcdaqdrv_led_fmc0) {
        reg = ADC3117_ADCLED_REG;
        mask = ADC3117_SUPPORT_ADC_GREEN_LED_MASK;

        switch (led_state) {
        case ifcdaqdrv_led_color_green:
            value = 0x1;
            break;
        case ifcdaqdrv_led_blink_fast:
            value = 0x3;
            break;
        case ifcdaqdrv_led_blink_slow:
            value = 0x2;
            break;
        case ifcdaqdrv_led_off:
            value = 0x0;
            break;
        default:
            printf("Error: %s: Unknown led state %d", __FUNCTION__, led_state);
            return status_argument_range;
        }
    } else if (led == ifcdaqdrv_led_fmc1) {
        reg = ADC3117_ADCLED_REG;
        mask = ADC3117_SUPPORT_ADC_RED_LED_MASK;

        switch (led_state) {
        case ifcdaqdrv_led_color_red:
            value = 0x4;
            break;
        case ifcdaqdrv_led_blink_fast:
            value = 0xC;
            break;
        case ifcdaqdrv_led_blink_slow:
            value = 0x8;
            break;
        case ifcdaqdrv_led_off:
            value = 0x0;
            break;
        default:
            printf("Error: %s: Unknown led state %d", __FUNCTION__, led_state);
            return status_argument_range;
        }
    } else {
        printf("Error: %s: Unknown led %d", __FUNCTION__, led);
        return status_argument_range;
    }

    return ifc_fmc_tcsr_setclr(ifcdevice, reg, value, mask);;
}

ifcdaqdrv_status adc3117_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency) {

    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency) {

    if (frequency != NULL) {
        *frequency = 5000000.0;
    }
    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock){

    if (clock != NULL) {
        *clock = ifcdaqdrv_clock_internal;
    }

    return status_success;
}

ifcdaqdrv_status adc3117_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock){

    return status_success;
}

ifcdaqdrv_status adc3117_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor){

    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor){

    if (divisor != NULL) {
        *divisor = 1;
    }

    return status_success;
}

ifcdaqdrv_status adc3117_set_offset(struct ifcdaqdrv_dev *ifcdevice, uint16_t offset){

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, (offset << 16), 0xFFFF0000);
}

ifcdaqdrv_status adc3117_get_offset(struct ifcdaqdrv_dev *ifcdevice, uint16_t *offset){
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, &i32_reg_val);
    *offset = (i32_reg_val >> 16) & 0xFFFF;

    return status;
}

ifcdaqdrv_status adc3117_set_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain){

    if ((gain < 1) || (gain > 10))
        return status_argument_range;

    adc3117_set_adc_channel(ifcdevice, channel);

    switch ((int)gain) {
    case 1:
        gain = GAIN_x1;
        break;
    case 2:
        gain = GAIN_x2;
        break;
    case 5:
        gain = GAIN_x5;
        break;
    case 10:
        gain = GAIN_x10;
        break;
    default:
        printf("Invalid gain!\n");
        return status_argument_range;
    }

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, ((int)gain << 4), 0x30);
}

ifcdaqdrv_status adc3117_get_gain(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, double *gain) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = adc3117_set_adc_channel(ifcdevice, channel);
    if (status)
        return status;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, &i32_reg_val);
    switch ((i32_reg_val >> 4) & 0x3) {
    case 0:
        *gain = 1;
        break;
    case 1:
        *gain = 2;
        break;
    case 2:
        *gain = 5;
        break;
    case 3:
        *gain = 10;
        break;
    default:
        printf("Invalid gain!\n");
    }

    return status;
}

ifcdaqdrv_status adc3117_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern){

    return status_success;
}

ifcdaqdrv_status adc3117_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern){

    return status_success;
}

ifcdaqdrv_status adc3117_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                       uint16_t *board_id) {
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_SIGN_REG, &i32_reg_val);

    if (revision) {
        *revision = i32_reg_val & 0x000000ff;
    }

    if (version) {
        *version = (i32_reg_val & 0x0000ff00) >> 8;
    }

    if (board_id) {
        *board_id = (i32_reg_val & 0xffff0000) >> 16;
    }

    return status;
}

ifcdaqdrv_status adc3117_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){

    if (ifcdevice->mode == ifcdaqdrv_acq_mode_sram) {
        return adc3117_get_sram_nsamples_max(ifcdevice, nsamples_max);
    }

    return adc3117_get_smem_nsamples_max(ifcdevice, nsamples_max);
}

ifcdaqdrv_status adc3117_set_adc_channel(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel) {

    if (channel > 20)
        return status_argument_range;

    return ifc_fmc_tcsr_write(ifcdevice, ADC3117_CHANNEL_SELECT_REG, channel & 0x1F);
}

ifcdaqdrv_status adc3117_get_adc_channel(struct ifcdaqdrv_dev *ifcdevice, uint32_t *channel) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_SELECT_REG, &i32_reg_val);
    *channel = i32_reg_val & 0x1F;

    return status;
}

ifcdaqdrv_status adc3117_set_adc_channel_mask(struct ifcdaqdrv_dev *ifcdevice, uint32_t mask) {

    return ifc_fmc_tcsr_write(ifcdevice, ADC3117_CHANNEL_SELECT_REG, (mask << 12) & 0xFFFFF000);
}

ifcdaqdrv_status adc3117_get_adc_channel_mask(struct ifcdaqdrv_dev *ifcdevice, uint32_t *mask) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_SELECT_REG, &i32_reg_val);
    *mask = (i32_reg_val >> 12) & 0x000FFFFF;

    return status;
}

ifcdaqdrv_status adc3117_set_adc_channel_negative_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t input) {

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, input, 0x3);
}

ifcdaqdrv_status adc3117_get_adc_channel_negative_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, &i32_reg_val);
    *input = i32_reg_val & 0x3;

    return status;
}

ifcdaqdrv_status adc3117_set_adc_channel_positive_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t input) {

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, (input << 2), 0xC);
}

ifcdaqdrv_status adc3117_get_adc_channel_positive_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, &i32_reg_val);
    *input = (i32_reg_val >> 2) & 0x3;

    return status;
}

ifcdaqdrv_status adc3117_set_sample_rate(struct ifcdaqdrv_dev *ifcdevice, double sample_rate) {
    uint32_t value;

    if ((sample_rate < 1602) || (sample_rate > 5000000))
        return status_argument_range;

    value = (105000000 / sample_rate) - 1;

    /* 1 - 19 are invalid */
    if ((value > 0) && (value < 20))
        value = 20;

    return ifc_scope_lite_tcsr_setclr(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG, value << 16, 0xFFFF0000);
}

ifcdaqdrv_status adc3117_get_sample_rate(struct ifcdaqdrv_dev *ifcdevice, double *sample_rate) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_scope_lite_tcsr_read(ifcdevice, IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG, &i32_reg_val);
    if ((i32_reg_val >> 16) == 0)
        *sample_rate = 5000000;
    else
        *sample_rate = 105000000 / ((i32_reg_val >> 16) + 1);

    return status;
}

ifcdaqdrv_status adc3117_fmc_reinit(struct ifcdaqdrv_dev *ifcdevice) {

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_MCSR_REG, 0x2, 0x2);
}

ifcdaqdrv_status adc3117_configuration_command(struct ifcdaqdrv_dev *ifcdevice) {

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_MCSR_REG, 0x1, 0x1);
}


ifcdaqdrv_status adc3117_scopelite_test(struct ifcdaqdrv_dev *ifcdevice) {
    uint32_t nsamples_max = 0;
    
    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 4;
    ifcdevice->resolution  = 16;
    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 10.24;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 10;

    nsamples_max = (0x114000 - 0x100000) / 2;

    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size * ifcdevice->nchannels;
    ifcdevice->smem_size = 4 * 1024 * 1024;
    ifcdevice->smem_sg_dma = 0;

    return status_success;
}
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

/* OLIVER: check these values */
static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200, 0};

static const uint32_t averages[] = {1, 4, 8, 16, 32, 64, 128, 256, 0};
//static const uint32_t averages[] = {4, 8, 16, 32, 64, 128, 256, 512, 0}; // TODO: use different vectors for SRAM SMEM
static const double   valid_clocks[] = {2400e6, 2500e6, 0};

static ifcdaqdrv_status adc3117_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice);
static uint32_t adc3117_SerialBus_prepare_command(ADC3117_SPI_DEVICE device, int addr, int writecmd);

ifcdaqdrv_status adc3117_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    uint32_t nsamples_max;

    ifcdevice->init_adc              = adc3117_init_adc;
    ifcdevice->get_signature         = adc3117_get_signature;
    ifcdevice->set_led               = adc3117_set_led;
    ifcdevice->get_gain              = adc3117_get_gain;
    ifcdevice->set_gain              = adc3117_set_gain;
    ifcdevice->get_offset            = adc3117_get_offset;
    ifcdevice->set_offset            = adc3117_set_offset;
    ifcdevice->set_nsamples          = ifcdaqdrv_scope_set_nsamples;
    ifcdevice->get_nsamples          = ifcdaqdrv_scope_get_nsamples;
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
    ifcdevice->read_ai               = ifcdaqdrv_scope_read_ai;;

    ifcdevice->normalize_ch          = adc3117_read_ch;
    ifcdevice->normalize             = adc3117_read;

    ifcdevice->mode_switch           = ifcdaqdrv_scope_switch_mode; // Changed default mode to SMEM

    ifcdevice->set_adc_channel       = adc3117_set_adc_channel;
    ifcdevice->get_adc_channel       = adc3117_get_adc_channel;
    ifcdevice->set_adc_channel_mask  = adc3117_set_adc_channel_mask;
    ifcdevice->get_adc_channel_mask  = adc3117_get_adc_channel_mask;

    ifcdevice->set_adc_channel_negative_input = adc3117_set_adc_channel_negative_input;
    ifcdevice->get_adc_channel_negative_input = adc3117_get_adc_channel_negative_input;
    ifcdevice->set_adc_channel_positive_input = adc3117_set_adc_channel_positive_input;
    ifcdevice->get_adc_channel_positive_input = adc3117_get_adc_channel_positive_input;

    ifcdevice->adc_configuration_command = adc3117_configuration_command;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 20;

    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    ifcdevice->resolution  = 16;
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 125; //1045;
    ifcdevice->divisor_min = 8; //1;

    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 0.5; /* OLIVER: double check this */

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 1000;

    status = adc3117_get_sram_nsamples_max(ifcdevice, &nsamples_max);
    if (status) {
        return status;
    }
    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size;
    ifcdevice->smem_size = 256 * 1024 * 1024;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

ifcdaqdrv_status adc3117_init_adc(struct ifcdaqdrv_dev *ifcdevice){
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_SIGN_REG, &i32_reg_val);
    if (status)
        return status;

    if ((i32_reg_val & 0xFFFF0000) != 0x31170000) {
        printf("Error: %s: No ADC3117 installed on fmc%d [%08x]", __FUNCTION__, ifcdevice->fmc, i32_reg_val);
        return status_incompatible;
    }

    status = adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_off); // Green led off
    status += adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_blink_fast); // Red led blink fast during init
    if (status)
        return status;

    status = adc3117_set_adc_channel_mask(ifcdevice, 0xFFFFF); // Select all channels
    if (status)
        return status;

    status = ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_GENERAL_CONFIG_REG, 0x1, 0x3); // VCAL = Ref 4.128V
    if (status)
        return status;

    status = adc3117_set_adc_channel_negative_input(ifcdevice, GND);
    if (status)
        return status;

    status = adc3117_set_adc_channel_positive_input(ifcdevice, GND);
    if (status)
        return status;

    status = adc3117_set_gain(ifcdevice, 0, GAIN_x1);
    if (status)
        return status;

    status = adc3117_set_offset(ifcdevice, 0x0000);
    if (status)
        return status;

    /* OLIVER: not sure what this is/does */
    // Issues with signed dataformat.
    //adc3117_set_dataformat(ifcdevice, ifcdaqdrv_dataformat_unsigned);

    status = adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_color_green); // Green led on
    status += adc3117_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_off); // Red led off

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

/* SerialBus functions are not used. Need proper ADC3117 implementation. */
static ifcdaqdrv_status adc3117_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice){
    int32_t i32_reg_val;
    ifcdaqdrv_status status;
    int state_loop = 0;

    // check if Serial bus is ready (bit 31 is '0')
    while (1) {
        status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_SERIAL_BUS_CONTROL_REG, &i32_reg_val);
        if (status)
            return status;

        if ((i32_reg_val & 0x80000000) == 0) {
            // serial bus is ready
            return status_success;
        }

        state_loop++;
        if (state_loop > 50) {
            // break check loop (timeout)
            // serial bus is not ready
            return status_spi_busy;
        }

        usleep(1);
    }

    // serial bus is not ready
    return status_spi_busy;
}

static uint32_t adc3117_SerialBus_prepare_command(ADC3117_SPI_DEVICE device, int addr, int writecmd){
    return 0;
}

/*
 * Serialbus read and write depend on accessing two registers sequentially and
 * are therefore locked with the "subdevice" lock.
 */
ifcdaqdrv_status adc3117_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3117_SPI_DEVICE device, int addr, uint32_t
                                         data){
    int      status;
    uint32_t cmd = adc3117_SerialBus_prepare_command(device, addr, 1); // create write command

    pthread_mutex_lock(&ifcdevice->sub_lock);
    // check if serial bus is ready
    if (adc3117_SerialBus_isReady(ifcdevice) != 0) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status_i2c_busy;
    }

    status = ifc_fmc_tcsr_write(ifcdevice, 4, data);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status;
    }

    status = ifc_fmc_tcsr_write(ifcdevice, 3, cmd);
    pthread_mutex_unlock(&ifcdevice->sub_lock);

    return status;
}

ifcdaqdrv_status adc3117_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3117_SPI_DEVICE device, int addr,
                                                 uint32_t *value){
    uint32_t cmd = adc3117_SerialBus_prepare_command(device, addr, 0); // create read command
    int      status;

    pthread_mutex_lock(&ifcdevice->sub_lock);
    // check if serial bus is ready
    if (adc3117_SerialBus_isReady(ifcdevice) != 0) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status_i2c_busy;
    }

    status = ifc_fmc_tcsr_write(ifcdevice, 3, cmd);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status;
    }

    // check if serial bus is ready
    if (adc3117_SerialBus_isReady(ifcdevice) != 0) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status_i2c_busy;
    }

    status = ifc_fmc_tcsr_read(ifcdevice, 4, (int32_t *) value);
    pthread_mutex_unlock(&ifcdevice->sub_lock);

    return status;
}

/* OLIVER: This is not used. The function in ADC3110 driver has a generic name. Must be changed. */
/* Two strings on the heap should be returned since the freeing is done elsewhere */
ifcdaqdrv_status adc3117_read_ioxos_signature(struct ifcdaqdrv_dev *ifcdevice, struct fmc_fru_id *fru_id){
    int  status;
    char signature[ADC3117_SIGNATURELEN + 1];
    char *p;

    printf("Trying to read EEPROM signature!!! \n");
    status = ifc_fmc_eeprom_read_string(ifcdevice, 0x7000, 8, signature, sizeof(signature));
    if (status) {
        return status;
    }

    if (strcmp(signature, "ADC_3117") == 0 ||
               strcmp(signature, "ADC3117") == 0 ||
               strcmp(signature, "ADC3117 ") == 0 ||
               strcmp(signature, " ADC3117") == 0) {
        p = calloc(strlen("ADC3117") + 1, 1);
        strcpy(p, "ADC3117");
    } else {
        return status_internal;
    }
    fru_id->product_name = p;

    p = calloc(strlen("IOxOS") + 1, 1);
    strcpy(p, "IOxOS");
    fru_id->manufacturer = p;

    return status_success;
}

/* OLIVER: Implement clock functions */
ifcdaqdrv_status adc3117_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency) {

    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency) {

    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock){

    return status_success;
}

ifcdaqdrv_status adc3117_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock){

    return status_success;
}

ifcdaqdrv_status adc3117_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor){

    return status_success;
}

ifcdaqdrv_status adc3117_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor){

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
    UNUSED(channel);

    if ((gain < GAIN_x1) || (gain > GAIN_x10))
        return status_argument_range;

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, ((int)gain << 4), 0x30);
}

ifcdaqdrv_status adc3117_get_gain(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, double *gain) {
    UNUSED(channel);
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, ADC3117_CHANNEL_CONFIG_REG, &i32_reg_val);
    *gain = (i32_reg_val >> 4) & 0x3;

    return status;
}

ifcdaqdrv_status adc3117_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern){

    return status_success;
}

ifcdaqdrv_status adc3117_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern){

    return status_success;
}

/* OLIVER: Check the trigger functions */
ifcdaqdrv_status adc3117_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold) {
    uint16_t ui16_reg_val = (uint16_t)threshold + 32768;
    // threshold += 32768; // Threshold should be ADC value (unsigned).

    return ifc_scope_acq_tcsr_setclr(ifcdevice, 1, ui16_reg_val & 0xFFFF, 0xFFFF);
}

ifcdaqdrv_status adc3117_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold) {
    ifcdaqdrv_status status;
    int32_t          threshold_adc;
    int32_t          i32_reg_val;

    status = ifc_scope_acq_tcsr_read(ifcdevice, 1, &i32_reg_val);
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

/* OLIVER: Check the read functions */
ifcdaqdrv_status adc3117_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples) {
    UNUSED(ifcdevice);

    int32_t *target; /* Copy to this address */
    int16_t *itr;    /* Iterator for iterating over "data" */
    int16_t *origin; /* Copy from this address */

    /* Multiply offsets by number of channels */
    target = ((int32_t *)dst) + dst_offset;
    origin = ((int16_t *)src) + src_offset * 8;

    for (itr = origin; itr < origin + nelm * 8; target += 2, itr += 16) {
        *((target + 0) + 0 * channel_nsamples) = (int16_t)(*(itr + 0) - 32768);
        *((target + 1) + 0 * channel_nsamples) = (int16_t)(*(itr + 1) - 32768);
        *((target + 0) + 1 * channel_nsamples) = (int16_t)(*(itr + 2) - 32768);
        *((target + 1) + 1 * channel_nsamples) = (int16_t)(*(itr + 3) - 32768);
        *((target + 0) + 2 * channel_nsamples) = (int16_t)(*(itr + 4) - 32768);
        *((target + 1) + 2 * channel_nsamples) = (int16_t)(*(itr + 5) - 32768);
        *((target + 0) + 3 * channel_nsamples) = (int16_t)(*(itr + 6) - 32768);
        *((target + 1) + 3 * channel_nsamples) = (int16_t)(*(itr + 7) - 32768);
        *((target + 0) + 4 * channel_nsamples) = (int16_t)(*(itr + 8) - 32768);
        *((target + 1) + 4 * channel_nsamples) = (int16_t)(*(itr + 9) - 32768);
        *((target + 0) + 5 * channel_nsamples) = (int16_t)(*(itr + 10) - 32768);
        *((target + 1) + 5 * channel_nsamples) = (int16_t)(*(itr + 11) - 32768);
        *((target + 0) + 6 * channel_nsamples) = (int16_t)(*(itr + 12) - 32768);
        *((target + 1) + 6 * channel_nsamples) = (int16_t)(*(itr + 13) - 32768);
        *((target + 0) + 7 * channel_nsamples) = (int16_t)(*(itr + 14) - 32768);
        *((target + 1) + 7 * channel_nsamples) = (int16_t)(*(itr + 15) - 32768);
    }
    return status_success;
}

ifcdaqdrv_status adc3117_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                              size_t nelm) {
    UNUSED(ifcdevice);
    int16_t *origin = (int16_t *)data + offset;
    int16_t *itr;
    int32_t *target = res;

    if(ifcdevice->mode == ifcdaqdrv_acq_mode_smem) {
        for (itr = origin; itr < origin + nelm * 8; ++target, itr += 8) {
            *target = (int16_t)(*(itr + channel) - 32768);
        }
        return status_success;
    }

    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = (int16_t)(*itr - 32768);
    }

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

ifcdaqdrv_status adc3117_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){
    int32_t i32_reg_val;
    int     status;

    status = ifc_scope_acq_tcsr_read(ifcdevice, 0, &i32_reg_val);
    if (i32_reg_val & IFC_SCOPE_TCSR_CS_SRAM_Buffer_Size_MASK) {
        *nsamples_max = 32 * 1024;
    } else {
        *nsamples_max = 16 * 1024;
    }
    return status;
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

ifcdaqdrv_status adc3117_configuration_command(struct ifcdaqdrv_dev *ifcdevice) {

    return ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_MCSR_REG, 0x1, 0x1);
}

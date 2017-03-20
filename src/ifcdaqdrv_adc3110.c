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
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_scope.h"

static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200, 0};
static const uint32_t averages[] = {1, 4, 8, 16, 32, 64, 128, 256, 0};
static const double   valid_clocks[] = {2400e6, 2500e6, 0};

static int adc3110_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice);
static uint32_t adc3110_SerialBus_prepare_command(ADC3110_SBCDEVICE device, int addr, int writecmd);


static ADC3110_SBCDEVICE adc3110_get_sbc_device(unsigned channel);

static ADC3110_SBCDEVICE adc3110_get_sbc_device(unsigned channel){
    switch (channel) {
    case 0:
    case 1:
        return ADS01;
    case 2:
    case 3:
        return ADS23;
    case 4:
    case 5:
        return ADS45;
    case 6:
    case 7:
        return ADS67;
    default:
#if DEBUG
        printf("%s(): Error: (channel=%d)\n", __FUNCTION__, channel);
#endif
        return ADS01;
    }
}

ifcdaqdrv_status adc3111_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    status = adc3110_register(ifcdevice);
    ifcdevice->vref_max = 0.5;
    return status;
}

ifcdaqdrv_status adc3110_register(struct ifcdaqdrv_dev *ifcdevice) {
    int status = 0;
    uint32_t nsamples_max;
    ifcdevice->init_adc              = adc3110_init_adc;
    ifcdevice->get_signature         = adc3110_get_signature;
    ifcdevice->set_led               = adc3110_set_led;
    ifcdevice->get_gain              = adc3110_get_gain;
    ifcdevice->set_gain              = adc3110_set_gain;
    ifcdevice->set_nsamples          = ifcdaqdrv_scope_set_nsamples;
    ifcdevice->get_nsamples          = ifcdaqdrv_scope_get_nsamples;
    ifcdevice->set_trigger_threshold = adc3110_set_trigger_threshold;
    ifcdevice->get_trigger_threshold = adc3110_get_trigger_threshold;
    ifcdevice->set_clock_frequency   = adc3110_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3110_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3110_set_clock_source;
    ifcdevice->get_clock_source      = adc3110_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3110_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3110_get_clock_divisor;

    ifcdevice->set_pattern           = adc3110_set_test_pattern;
    ifcdevice->get_pattern           = adc3110_get_test_pattern;

    ifcdevice->read_ai_ch            = ifcdaqdrv_scope_read_ai_ch;
    ifcdevice->read_ai               = ifcdaqdrv_scope_read_ai;;

    ifcdevice->normalize_ch          = adc3110_read_ch;
    ifcdevice->normalize             = adc3110_read;

    ifcdevice->mode_switch = ifcdaqdrv_scope_switch_mode;

    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 8;

    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    ifcdevice->resolution  = 16;
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 125; //1045;
    ifcdevice->divisor_min = 8; //1;

    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 1;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 1000;

    status                 = adc3110_get_sram_nsamples_max(ifcdevice, &nsamples_max);
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

ifcdaqdrv_status adc3110_init_adc(struct ifcdaqdrv_dev *ifcdevice){
    int res = 0;

    // led off
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_off);
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_off);

    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x1900); // ReleasePowerdown ADC #01
    usleep(1000);

    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x1100); // ReleasePowerdown ADC #23
    usleep(1000);

    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x0100); // ReleasePowerdown ADC #4567
    usleep(1000);

    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x4100); // RESET PLL Rx CLK ADC3110 + RESET ADC
    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x0100); // Release RESET PLL Rx CLK ADC3110
    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x0000); // Release ADC3110 ADS42LB69 RESET
    usleep(2000);

    /*
     * Setup LMK04906
     */
    // Reset device
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x00, 0x00020000);
    usleep(2000);

    // Power down Clock 0 (testpoint) and Clock 5 (fmc GPIO)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x00, 0x80000000); // ClkOut0_PD = 1
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x05, 0x80000000); // ClkOut5_PD = 1

    // Set LMK04906 outputs to LVDS (This MUST be done before enabling CCHD575)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x06, 0x01100000); // ClkOut0_Type/Clk_Out1_Type = 1 (LVDS)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x07, 0x01100000); // ClkOut2_Type/Clk_Out3_Type = 1 (LVDS)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x08, 0x01010000); // ClkOut4_Type/Clk_Out5_Type = 1 (LVDS)

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x09, 0x55555540); // For "Proper" operation...

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0D, 0x3B700240); // HOLDOVER pin uWRITE SDATOUT ClkIn_SELECT_MODE = ClkIn1  Enable CLKin1 = 1
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0E, 0x00000000); // Bipolar Mode CLKin1 INPUT
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0F, 0x00000000); // DAC unused

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x10, 0x01550400); // OSC IN level

    // PLL1 is never used.
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x18, 0x00000000); // PLL1 not used / PLL2 used
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x19, 0x00000000); // DAC config not used

    adc3110_set_clock_source(ifcdevice, ifcdaqdrv_clock_internal);

    adc3110_adc_init_priv(ifcdevice, ADS01);
    adc3110_adc_init_priv(ifcdevice, ADS23);
    adc3110_adc_init_priv(ifcdevice, ADS45);
    adc3110_adc_init_priv(ifcdevice, ADS67);

    // disable gain, set gain to 1 on all channels
    int i = 0;
    for (i = 0; i < 8; ++i) {
        adc3110_set_gain(ifcdevice, i, 1);
    }

    // Issues with signed dataformat.
    adc3110_set_dataformat(ifcdevice, ifcdaqdrv_dataformat_unsigned);

    // adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc1, res ? ifcdaqdrv_led_off : ifcdaqdrv_led_color_green);

    // adc3110_ctrl_front_panel_gpio(card->pevCrate,card->FMCslot, 0xC0000000);

    return res;
}

ifcdaqdrv_status adc3110_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state){
    uint32_t reg   = 0;
    uint32_t value = 0;
    uint32_t mask  = ADC3110_SUPPORT_IFC_LED_MASK;

    if (led == ifcdaqdrv_led_ifc) {
        reg = ADC3110_SUPPORT_REG;

        switch (led_state) {
        case ifcdaqdrv_led_color_green:
            value = 1;
            break;
        case ifcdaqdrv_led_color_red:
            value = 2;
            break;
        default:
            value = 0;
        }
    } else {
        reg = ADC3110_ADCLED_REG;

        switch (led_state) {
        case ifcdaqdrv_led_color_green:
            value = 1;
            break;
        case ifcdaqdrv_led_blink_fast:
            value = 2;
            break;
        case ifcdaqdrv_led_blink_slow:
            value = 3;
            break;
        default:
            value = 0;
        }

        if (led == ifcdaqdrv_led_fmc1) {
            value <<= 2;
            mask  <<= 2;
        }
    }

    ifc_fmc_tcsr_setclr(ifcdevice, reg, value, mask);
    return status_success;
}

ifcdaqdrv_status adc3110_set_dataformat(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat){
    int res = 0;
    int i;
    for (i = 0; i < 4; ++i) {
        uint32_t val;
        adc3110_SerialBus_read(ifcdevice, adc3110_get_sbc_device(i * 2), 0x08, &val);

        switch (dataformat) {
        case ifcdaqdrv_dataformat_signed:
            val &= ~(1 << 4);
            break;
        case ifcdaqdrv_dataformat_unsigned:
            val |= (1 << 4);
            break;
        default:
            // wrong dataformat
#if DEBUG
            printf("Error: %s(crate=%d,fmc=%d,device=%d,dataformat=%d) wrong dataformat!\n", __FUNCTION__,
                   ifcdevice->card, ifcdevice->fmc, adc3110_get_sbc_device(i * 2), dataformat);
#endif
            return -1;
            break;
        }

        res += adc3110_SerialBus_write(ifcdevice, adc3110_get_sbc_device(i * 2), 0x08, val);
    }
    return res ? -1 : 0;
}

ifcdaqdrv_status adc3110_adc_resolution(struct ifcdaqdrv_dev *ifcdevice, unsigned *resolution){
    UNUSED(ifcdevice);
    *resolution = 16;
    return status_success;
}

static int adc3110_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice){
    // Doc: ADC3110_UG_A0 3.2.4 TCSR ADC_3110Serial_IF Control Register

    int base       = (ifcdevice->fmc == 1) ? 0x80 : 0xC0;
    int state_loop = 0;
    int state;

    // check if Serial bus is ready (bit 31 is '0')
    while (1) {
        ifc_xuser_tcsr_read(ifcdevice, base + 0x03, &state);

        if ((state & 0x80000000) == 0) {
            // serial bus is ready
            return 0;
        }

        state_loop++;
        if (state_loop > 50) {
            // break check loop (timeout)
            // serial bus is not ready
            return -1;
        }

        usleep(1);
    }

    // serial bus is not ready
    return -1;
}

static uint32_t adc3110_SerialBus_prepare_command(ADC3110_SBCDEVICE device, int addr, int writecmd){
    // Doc: ADC3110_UG_A0 3.2.4 TCSR ADC_3110Serial_IF Control Register
    // Doc: ADC3110_UG_A0 3.2.5 TCSR ADC_3110Serial_IF Data Register

    uint32_t cmd = 0;

    switch (device) {
    case LMK04906:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x0E000000;    // DEVSEL[25:24]="10"(LMK04906) LMK_2CLK_LE[27:26]="11"(3 extra uWIRE clock with LE='1')
        break;
    case ADS01:
    case ADS23:
    case ADS45:
    case ADS67:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr

        switch (device) {
        case ADS01:
            cmd |= 0x01000000;     // DEVSEL[25:24]="01"(ADS42LB69) ADS42LB69_SEL[17:16]="00"(device #01)
            break;
        case ADS23:
            cmd |= 0x01010000;     // DEVSEL[25:24]="01"(ADS42LB69) ADS42LB69_SEL[17:16]="01"(device #23)
            break;
        case ADS45:
            cmd |= 0x01020000;     // DEVSEL[25:24]="01"(ADS42LB69) ADS42LB69_SEL[17:16]="10"(device #45)
            break;
        case ADS67:
        default:
            cmd |= 0x01030000;     // DEVSEL[25:24]="01"(ADS42LB69) ADS42LB69_SEL[17:16]="11"(device #67)
            break;
        }

        break;
    default:

#if DEBUG
        printf("ERROR: %s(device=%d,addr=%d,writecmd=%d) unknown device %d", __FUNCTION__, device, addr, writecmd,
               device);
#endif

        return 0;
        break;
    }

    if (writecmd) {
        // write command
        cmd |= 0xC0000000;    // CMD[31:30]="11"(write operation)
    } else {
        // read command
        cmd |= 0x80000000;    // CMD[31:30]="10"(write operation)
    }

    return cmd;
}

/*
 * Serialbus read and write depend on accessing two registers sequentially and
 * are therefore locked with the "subdevice" lock.
 */

ifcdaqdrv_status adc3110_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device, int addr, uint32_t
                                         data){
    int      status;
    uint32_t cmd = adc3110_SerialBus_prepare_command(device, addr, 1); // create write command

    pthread_mutex_lock(&ifcdevice->sub_lock);
    // check if serial bus is ready
    if (adc3110_SerialBus_isReady(ifcdevice) != 0) {
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

ifcdaqdrv_status adc3110_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device, int addr,
                                                 uint32_t *value){
    uint32_t cmd = adc3110_SerialBus_prepare_command(device, addr, 0); // create read command
    int      status;

    pthread_mutex_lock(&ifcdevice->sub_lock);
    // check if serial bus is ready
    if (adc3110_SerialBus_isReady(ifcdevice) != 0) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status_i2c_busy;
    }

    status = ifc_fmc_tcsr_write(ifcdevice, 3, cmd);
    if (status) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status;
    }

    // check if serial bus is ready
    if (adc3110_SerialBus_isReady(ifcdevice) != 0) {
        pthread_mutex_unlock(&ifcdevice->sub_lock);
        return status_i2c_busy;
    }

    status = ifc_fmc_tcsr_read(ifcdevice, 4, (int32_t *) value);
    pthread_mutex_unlock(&ifcdevice->sub_lock);
    return status;
}

/* Two strings on the heap should be returned since the freeing is done elsewhere */

ifcdaqdrv_status ifc_read_ioxos_signature(struct ifcdaqdrv_dev *ifcdevice, struct fmc_fru_id *fru_id){
    int  status;
    char signature[ADC3110_SIGNATURELEN + 1];
    char *p;

    status = ifc_fmc_eeprom_read_string(ifcdevice, 0x7000, 8, signature, sizeof(signature));
    if (status) {
        return status;
    }

    if (strcmp(signature, "ADC_3110") == 0 ||
               strcmp(signature, "ADC3110") == 0 ||
               strcmp(signature, "ADC3110 ") == 0 ||
               strcmp(signature, " ADC3110") == 0) {
        p = calloc(strlen("ADC3110") + 1, 1);
        strcpy(p, "ADC3110");
    } else if (strcmp(signature, "ADC_3111") == 0 ||
               strcmp(signature, "ADC3111") == 0 ||
               strcmp(signature, "ADC3111 ") == 0 ||
               strcmp(signature, " ADC3111") == 0) {
        p = calloc(strlen("ADC3111") + 1, 1);
        strcpy(p, "ADC3111");
    } else if (strcmp(signature, "ADC_3112") == 0 ||
               strcmp(signature, "ADC3112") == 0 ||
               strcmp(signature, "ADC3112 ") == 0 ||
               strcmp(signature, " ADC3112") == 0) {
        p = calloc(strlen("ADC3112") + 1, 1);
        strcpy(p, "ADC3112");
    } else {
        return status_internal;
    }
    fru_id->product_name = p;

    p = calloc(strlen("IOxOS") + 1, 1);
    strcpy(p, "IOxOS");
    fru_id->manufacturer = p;

    return status_success;
}

/* seems unneccessary */
ifcdaqdrv_status adc3110_ctrl_front_panel_gpio(struct ifcdaqdrv_dev *ifcdevice, int32_t value){
    return ifc_fmc_tcsr_write(ifcdevice, 5, value);
}

ifcdaqdrv_status ifc_fmc_eeprom_read_ChannelOffsetCompensation(struct ifcdaqdrv_dev *ifcdevice, unsigned channel,
                                                               int32_t *offsetcompensation){
    *offsetcompensation = 0;
    int status;

    if (channel >= ifcdevice->nchannels) {
        return status_argument_invalid;
    }

#define OFFSETSTRLEN 4
    char offstr[OFFSETSTRLEN + 1];

    status = ifc_fmc_eeprom_read_string(ifcdevice, 0x7000 + 44 + (channel * 4), OFFSETSTRLEN, offstr, sizeof(offstr));
    if (status) {
        return status;
    }

#if DEBUG
    char testDate[8 + 1];
    testDate[0] = '\n';
    status      = ifc_fmc_eeprom_read_string(ifcdevice, 0x7000 + 28, 8, testDate, sizeof(testDate));
    if (status) {
        return status;
    }

    char latestCalibrationDate[8 + 1];
    latestCalibrationDate[0] = '\0';
    status = ifc_fmc_eeprom_read_string(ifcdevice, 0x7000 + 36, 8, latestCalibrationDate,
                                        sizeof(latestCalibrationDate));
    if (status) {
        return status;
    }

    printf("%s(channel=%d) -> offset = %s -> %d  /testDate=%s calibrationDate=%s\n", __FUNCTION__, channel, offstr,
           *offsetcompensation, testDate, latestCalibrationDate);
#endif

    *offsetcompensation = strtol(offstr, NULL, 16);

    return 0;
}

ifcdaqdrv_status adc3110_tmp102_read(struct ifcdaqdrv_dev *ifcdevice, unsigned reg, uint32_t *ui32_reg_val){
    int      status;
    uint32_t device = 0x01040048; // I2C Bus device tmp102

    device |= ifcdevice->fmc == 1 ? IFC_FMC1_I2C_BASE : IFC_FMC2_I2C_BASE;

#ifdef TOSCA_USRLIB
    //status  = pevx_i2c_read(ifcdevice->card, device, reg, ui32_reg_val);

    /*TODO: check usage of i2c_read. Who is the first argument ? */

    status  = tsc_i2c_read(ifcdevice->card, device, reg, ui32_reg_val);


    /* TODO fix bit mask */
    if ((status & I2C_CTL_EXEC_MASK) == I2C_CTL_EXEC_ERR) {
        return status_i2c_nack;
    }

#else
    status = 0;
#endif


    return status_success;
}

ifcdaqdrv_status adc3110_tmp102_read_temperature(struct ifcdaqdrv_dev *ifcdevice, unsigned reg, float *temp){
    uint32_t extended_mode;
    uint32_t ui32_reg_val;
    int      status;

    status = adc3110_tmp102_read(ifcdevice, 0, &extended_mode);
    if (status) {
        return status;
    }
    status = adc3110_tmp102_read(ifcdevice, reg, &ui32_reg_val);
    if (status) {
        return status;
    }

    if (extended_mode & 0x100) { // Check if temperature sensor is in extended mode
        ui32_reg_val = ((ui32_reg_val & 0xff) << 5) + ((ui32_reg_val & 0xf8) >> 3);
    } else {
        ui32_reg_val = ((ui32_reg_val & 0xff) << 4) + ((ui32_reg_val & 0xf0) >> 4);
    }
    *temp = (float)ui32_reg_val / 16;
    return status_success;
}

ifcdaqdrv_status adc3110_temperatur(struct ifcdaqdrv_dev *ifcdevice, float *temp){
    return adc3110_tmp102_read_temperature(ifcdevice, 0, temp);
}

ifcdaqdrv_status adc3110_temperatur_low(struct ifcdaqdrv_dev *ifcdevice, float *temp){
    return adc3110_tmp102_read_temperature(ifcdevice, 2, temp);
}

ifcdaqdrv_status adc3110_temperatur_high(struct ifcdaqdrv_dev *ifcdevice, float *temp){
    return adc3110_tmp102_read_temperature(ifcdevice, 3, temp);
}

// PLL2_N * PLL2_P becomes the complete divisor in the feedback path to the PLL2 phase detector
//
// See PLL Programming in p.84 LMK04906 datasheet

ifcdaqdrv_status adc3110_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency) {
    uint32_t PLL2_P      = 2;  // 2..8
    uint32_t PLL2_N      = 25; // 1..262143
    int32_t  i32_reg_val = 0;

    // For now, only support 2400 and 2500 Mhz
    if (!(frequency == 2400e6 || frequency == 2500e6)) {
        return status_argument_range;
    }
    PLL2_N       = frequency / 1e8;

    i32_reg_val  = 1 << 23;       // PLL Fast Phase Detector Frequency (>100Mhz)
    i32_reg_val |= 1 << 24;       // OscIn Freq 63 to 127Mhz
    i32_reg_val |= (PLL2_N << 5);

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1D, i32_reg_val);

    i32_reg_val = (PLL2_P << 24) | (PLL2_N << 5);

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1E, i32_reg_val);

    // usleep(2000);

    // adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1E, i32_reg_val); // PLL2 P/N Recallibration
    // adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0C, 0x03800000); // PLL2 LD pin programmable
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0C, 0x02800000); // PLL2 LD pin programmable
    return status_success;
}

ifcdaqdrv_status adc3110_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency) {
    uint32_t         ui32_reg_val;
    ifcdaqdrv_status status;

    status = adc3110_SerialBus_read(ifcdevice, LMK04906, 0x1E, &ui32_reg_val);
    if (status) {
        return status;
    }
    if (frequency) {
        *frequency = ((ui32_reg_val >> 5) & 0x1FFFF) * 1e8;
    }

    return status;
}

ifcdaqdrv_status adc3110_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock){
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, 2, &i32_reg_val);
    if (status) {
        return status;
    }

    if (i32_reg_val & 0x80000000) {
        *clock = ifcdaqdrv_clock_internal;
    } else {
        *clock = ifcdaqdrv_clock_external;
    }

    return status;
}

ifcdaqdrv_status adc3110_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock){
    switch (clock) {
    case ifcdaqdrv_clock_internal:
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0A, 0x11404200);     // OscOut_Type = 1 (LVDS) Powerdown OscIn PowerDown = 0 VCO_DIV = 2
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0B, 0x37f28000);     // Device MODE=0x6 + No SYNC output
        adc3110_set_clock_frequency(ifcdevice, 2500e6);
        adc3110_set_clock_divisor(ifcdevice, 10);
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1A, 0x8FA00000);     // PLL2 used / ICP = 3200uA
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1B, 0x00000000);     // PLL1 not used
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1C, 0x00200000);     // PLL2_R = 2 / PLL1 N divisor=00
        // Enable Internal 100MHz clock from CCHD-575
        ifc_fmc_tcsr_setclr(ifcdevice, 2, 0x80000000, 0x0);
        break;
    case ifcdaqdrv_clock_external:
        // Disable Internal 100MHz clock OSC CCHD-575
        ifc_fmc_tcsr_setclr(ifcdevice, 2, 0x0, 0x80000000);
        adc3110_set_clock_divisor(ifcdevice, 1);
        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0A, 0x11484200);     // LMK04906_R10 OscOut_Type = 1 (LVDS) OscIn Powerdown = 1

        adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0B, 0x87f08000);     // LMK04906_R11 Device MODE=0x10(Clock Distribution) + No SYNC output
        break;
    }

    // Verification Clock has started
    // Warning: ads42lb69 01 shall be initialized
    const int timeout = 60; // 30ms

    int       timo    = 0;
    int32_t   value;

    timo = 0;
    while (timo < timeout) {
        timo++;

        ifc_fmc_tcsr_read(ifcdevice, 1, &value);

        if (value & 0x00008000) {
            // MMCM is locked
            break;
        } else {
            // MMCM not locked
        }

        usleep(500);
    }

    /* Check if MMC is locked */
    if (value & 0x00008000) {
        return status_success;
    }

    // Failed to set clock
    // return status_internal;
#if DEBUG
    printf("%s(): Warning: Failed to lock clock..\n", __FUNCTION__);
#endif
    return status_success;
}

ifcdaqdrv_status adc3110_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor){
    int32_t i32_reg_val;

    if (divisor < 1 || divisor > 1045) {
        return status_argument_range;
    }

    i32_reg_val = divisor << 5; // Enable ClkOutX + ClkOutX_DIV = divisor

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x01, i32_reg_val);
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x02, i32_reg_val);
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x03, i32_reg_val);
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x04, i32_reg_val);

    return status_success;
}

ifcdaqdrv_status adc3110_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor){
    ifcdaqdrv_status status;
    uint32_t         ui32_reg_val;

    status = adc3110_SerialBus_read(ifcdevice, LMK04906, 0x1, &ui32_reg_val);
    if (status) {
        return status;
    }
    if (divisor) {
        *divisor = (ui32_reg_val >> 5) & 0x3FF;
    }

    return status;
}

ifcdaqdrv_status adc3110_adc_init_priv(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device){
    int res = 0;

    res += adc3110_SerialBus_write(ifcdevice, device, 0x08, 0x19); // ADS42LB69_Reg 0x08 RESET device

    // ????
    res += adc3110_SerialBus_write(ifcdevice, device, 0x04, 0x00); // ADS42LB69_Reg 0x04 LVDS electrical level
    // ????
    res += adc3110_SerialBus_write(ifcdevice, device, 0x05, 0x00); // ADS42LB69_Reg 0x05 LVDS electrical level

    res += adc3110_SerialBus_write(ifcdevice, device, 0x06, 0x00); // ADS42LB69_Reg 0x06 LVDS CLKDIV=0 : Bypassed
    res += adc3110_SerialBus_write(ifcdevice, device, 0x07, 0x00); // ADS42LB69_Reg 0x07 SYNC_IN delay = 0 ps
    res += adc3110_SerialBus_write(ifcdevice, device, 0x08, 0x18); // ADS42LB69_Reg 0x08 Data format = 2s complement ????

    res += adc3110_SerialBus_write(ifcdevice, device, 0x0B, 0x00); // ADS42LB69_Reg 0x0B Channel A(even) Gain disabled 0dB / No Flip
    res += adc3110_SerialBus_write(ifcdevice, device, 0x0C, 0x00); // ADS42LB69_Reg 0x0C Channel B(odd) Gain disabled 0dB / No Flip

    res += adc3110_SerialBus_write(ifcdevice, device, 0x0D, 0x00); // ADS42LB69_Reg 0x0D OVR pin normal

    res += adc3110_SerialBus_write(ifcdevice, device, 0x0F, 0x00); // ADS42LB69_Reg 0x0F Normal operation

    res += adc3110_SerialBus_write(ifcdevice, device, 0x14, 0x00); // ADS42LB69_Reg 0x14 LVDS strenght (default DATA/CLK)

    res += adc3110_SerialBus_write(ifcdevice, device, 0x15, 0x01); // ADS42LB69_Reg 0x15 DDR LVDS Mode enabled

    res += adc3110_SerialBus_write(ifcdevice, device, 0x16, 0x00); // ADS42LB69_Reg 0x16 DDR Timing 0ps (default)

    res += adc3110_SerialBus_write(ifcdevice, device, 0x17, 0x00); // ADS42LB69_Reg 0x17 QDR CLKOUT delay
    res += adc3110_SerialBus_write(ifcdevice, device, 0x18, 0x00); // ADS42LB69_Reg 0x18 QDR CLKOUT timing

    // ????
    res += adc3110_SerialBus_write(ifcdevice, device, 0x1F, 0xFF); // ADS42LB69_Reg 0x1F Fast OVR

    // ????
    // 0x20 ....

    // ????
    res += adc3110_SerialBus_write(ifcdevice, device, 0x30, 0x00); // ADS42LB69_Reg 0x30 SYNC IN disabled

    return res != 0 ? -1 : status_success;
}

ifcdaqdrv_status adc3110_ADC_setOffset(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, uint16_t offset){
    int               status = 0;
    int32_t           reg;

    ADC3110_SBCDEVICE device = adc3110_get_sbc_device(channel);

    // check device
    switch (device) {
    case ADS01:
        reg = 0x8;
        break;
    case ADS23:
        reg = 0x9;
        break;
    case ADS45:
        reg = 0xA;
        break;
    case ADS67:
        reg = 0xB;
        break;
    default:
        // wrong device
#if DEBUG
        printf("Error: adc3110_ADC_setOffset(device=%d,channel=%d) wrong device!\n", device, channel);
#endif
        return -1;
        break;
    }

    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, reg, &i32_reg_val); // read offset compensation value
    if (status) {
        return status;
    }

    // set new offset
    if (channel % 2 == 0) {
        // channel A -> bits [15:0]
        i32_reg_val &= 0xFFFF0000;
        i32_reg_val |= (offset & 0xFFFF);
    } else {
        // channel B -> bits [31:16]
        i32_reg_val &= 0x0000FFFF;
        i32_reg_val |= (offset & 0xFFFF) << 16;
    }

    return ifc_fmc_tcsr_write(ifcdevice, reg, i32_reg_val); // write modified offset compensation value
}

ifcdaqdrv_status adc3110_set_gain(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, double gain){
    ifcdaqdrv_status  status;
    ADC3110_SBCDEVICE device      = adc3110_get_sbc_device(channel);
    int32_t           i32_reg_val = 0;

    // adjust gain Vpp to supported values
    if (gain <= 2 / 2.5) {
        i32_reg_val = 3;
    } else if (gain <= 2 / 2.4) {
        i32_reg_val = 4;
    } else if (gain <= 2 / 2.2) {
        i32_reg_val = 5;
    } else if (gain <= 2 / 2.1) {
        i32_reg_val = 6;
    } else if (gain <= 2 / 2.0) {
        i32_reg_val = 0; // Disable gain
    } else if (gain <= 2 / 1.9) {
        i32_reg_val = 8;
    } else if (gain <= 2 / 1.8) {
        i32_reg_val = 9;
    } else if (gain <= 2 / 1.7) {
        i32_reg_val = 10;
    } else if (gain <= 2 / 1.6) {
        i32_reg_val = 11;
    } else if (gain <= 2 / 1.5) {
        i32_reg_val = 12;
    } else if (gain <= 2 / 1.4) {
        i32_reg_val = 13;
    } else if (gain <= 2 / 1.3) {
        i32_reg_val = 14;
    } else if (gain <= 2 / 1.25) {
        i32_reg_val = 15;
    } else if (gain <= 2 / 1.2) {
        i32_reg_val = 16;
    } else if (gain <= 2 / 1.1) {
        i32_reg_val = 17;
    } else if (gain <= 2 / 1.05) {
        i32_reg_val = 18;
    } else { // gain > 2/1.05
        i32_reg_val = 19;
    }

    int addr = (channel % 2 == 0) ? 0x0B : 0x0C;

    // read i32_reg_val control register
    uint32_t value;
    adc3110_SerialBus_read(ifcdevice, device, addr, &value);

    value &= 0x03; // clear bits [7:2] -> leave bit 0 and 1 untouched

    /* Enable gain if gain != 1.0 */
    if (i32_reg_val == 0) {
        value &= ~(1 << 2);
    } else {
        value |= (1 << 2);
    }

    // set CHx_GAIN - bits [7:3]
    value |= (i32_reg_val << 3);


    // write i32_reg_val control register
    status = adc3110_SerialBus_write(ifcdevice, device, addr, value);

    return status;
}

ifcdaqdrv_status adc3110_get_gain(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, double *gain) {
    uint32_t         ui32_reg_val;
    ifcdaqdrv_status status;
    int              addr = (channel % 2 == 0) ? 0x0B : 0x0C;

    status = adc3110_SerialBus_read(ifcdevice, adc3110_get_sbc_device(channel), addr, &ui32_reg_val);
    if (status) {
        return status;
    }

    if (!(ui32_reg_val & (1 << 2))) {
        // Gain is disabled
        *gain = 1.0;
        return status;
    }

    // printf("gain %d\n", i32_reg_val >> 3);

    switch ((ui32_reg_val >> 3) & 0x1F) {
    default:
    case 0: // Not defined
    case 1: // Not defined
    case 2: // Not defined
        *gain = 2 / 2.0;
        break;
    case 3:
        *gain = 2 / 2.5;
        break;
    case 4:
        *gain = 2 / 2.4;
        break;
    case 5:
        *gain = 2 / 2.2;
        break;
    case 6:
        *gain = 2 / 2.1;
        break;
    case 7:
        *gain = 2 / 2.0;
        break;
    case 8:
        *gain = 2 / 1.9;
        break;
    case 9:
        *gain = 2 / 1.8;
        break;
    case 10:
        *gain = 2 / 1.7;
        break;
    case 11:
        *gain = 2 / 1.6;
        break;
    case 12:
        *gain = 2 / 1.5;
        break;
    case 13:
        *gain = 2 / 1.4;
        break;
    case 14:
        *gain = 2 / 1.3;
        break;
    case 15:
        *gain = 2 / 1.25;
        break;
    case 16:
        *gain = 2 / 1.2;
        break;
    case 17:
        *gain = 2 / 1.1;
        break;
    case 18:
        *gain = 2 / 1.05;
        break;
    case 19:
        *gain = 2 / 1.0;
        break;
    }

    return status;
}

ifcdaqdrv_status adc3110_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern){
    ifcdaqdrv_status  status;
    ADC3110_SBCDEVICE device;
    uint32_t          i32_reg_val;

    device = adc3110_get_sbc_device(channel);

    status = adc3110_SerialBus_read(ifcdevice, device, 0x0F, &i32_reg_val);

    if (status) {
        return status;
    }

    int32_t clearmask = 0xF;
    int32_t setmask   = 0;

    switch (pattern) {
    case ifcdaqdrv_pattern_zero:
        setmask = 0x01;
        break;
    case ifcdaqdrv_pattern_one:
        setmask = 0x02;
        break;
    case ifcdaqdrv_pattern_toggle:
        setmask = 0x03;
        break;
    case ifcdaqdrv_pattern_ramp_inc:
        setmask = 0x04;
        break;
    case ifcdaqdrv_pattern_8psin:
        setmask = 0x0B;
        break;
    default:
        setmask = 0;
        break;
    }

    if (channel % 2 == 0) {
        // channel 0,2,4,6 bits 7:4
        clearmask = (clearmask << 4);
        setmask   = (setmask << 4);
    } else {
        // channel 1,3,5,7 bits 3:0
    }

    i32_reg_val &= ~clearmask; // clearmask
    i32_reg_val |= setmask;    // setmask

    return adc3110_SerialBus_write(ifcdevice, device, 0x0F, i32_reg_val);
}

ifcdaqdrv_status adc3110_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern){
    ifcdaqdrv_status  status;
    ADC3110_SBCDEVICE device;
    uint32_t          i32_reg_val;

    device = adc3110_get_sbc_device(channel);

    status = adc3110_SerialBus_read(ifcdevice, device, 0x0F, &i32_reg_val);

    if (status) {
        return status;
    }
    if (channel % 2 == 0) {
        // channel 0,2,4,6 bits 7:4
        i32_reg_val >>= 4;
    } else {
        // channel 1,3,5,7 bits 3:0
    }
    switch(i32_reg_val & 0xf) {
    case 0x01:
        *pattern = ifcdaqdrv_pattern_zero;
        break;
    case 0x02:
        *pattern = ifcdaqdrv_pattern_one;
        break;
    case 0x03:
        *pattern = ifcdaqdrv_pattern_toggle;
        break;
    case 0x04:
        *pattern = ifcdaqdrv_pattern_ramp_inc;
        break;
    case 0x0B:
        *pattern = ifcdaqdrv_pattern_8psin;
        break;
    default:
        *pattern = ifcdaqdrv_pattern_none;
    }

    return status_success;
}

ifcdaqdrv_status adc3110_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold) {
    uint16_t ui16_reg_val = (uint16_t)threshold + 32768;
    // threshold += 32768; // Threshold should be ADC value (unsigned).

    return ifc_scope_acq_tcsr_setclr(ifcdevice, 1, ui16_reg_val & 0xFFFF, 0xFFFF);
}

ifcdaqdrv_status adc3110_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold) {
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

ifcdaqdrv_status adc3110_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples) {
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

ifcdaqdrv_status adc3110_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
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

    // printf("origin: %u %08x, nelm %u %08x, origin+nelm %u %08x, res %u %08x, res+nelm %u %08x\n", origin, origin, nelm, nelm, origin+nelm, origin+nelm, res, res, res+nelm, res+nelm);

    for (itr = origin; itr < origin + nelm; ++target, ++itr) {
        *target = (int16_t)(*itr - 32768);
    }

    return status_success;
}

ifcdaqdrv_status adc3110_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                       uint16_t *board_id) {
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, 0, &i32_reg_val);

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

ifcdaqdrv_status adc3110_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max){
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

#if 0
ifcdaqdrv_status adc3110_ADC_setStandby(uint crate, IFC_FMC_SLOT fmc, int standby, int waitForWakeupTime){
    int                     res             = 0;

    const int               STDBY_BitNr     = 5; // standby -> bit 5 register 0x08

    const ADC3110_SBCDEVICE allADCDevices[] = {ADS01, ADS23, ADS45, ADS67};

#if DEBUG_OUT || 1
    printf("%s(crate=%u,fmc=%u,standby=%d,wait=%d)\n", __FUNCTION__, crate, fmc, standby, waitForWakeupTime);
#endif

    ADC3110_SBCDEVICE device;
    uint32_t          value;
    int               idx;

    for (idx = 0; idx < (sizeof(allADCDevices) / sizeof(allADCDevices[0])); idx++) {
        device = allADCDevices[idx];


        // read standby control register
        value = adc3110_SerialBus_read_unlocked(crate, fmc, device, 0x08);

#if DEBUG_OUT
        printf("%s(crate=%u,fmc=%u,standby=%d,wait=%d) device=%d read value = %08x\n", __FUNCTION__, crate, fmc,
               standby, waitForWakeupTime, device, value);
#endif

        if (standby) {
            // set standby
            value |= (1 << STDBY_BitNr);
        } else {
            // clear standby
            value &= ~(1 << STDBY_BitNr);
        }

#if DEBUG_OUT
        printf("%s(crate=%u,fmc=%u,standby=%d,wait=%d) device=%d write value = %08x\n", __FUNCTION__, crate, fmc,
               standby, waitForWakeupTime, device, value);
#endif

        // write standby control register
        res = adc3110_SerialBus_write(crate, fmc, device, 0x08, value);
    }

    if (!standby && waitForWakeupTime) {
        // wakeup time from standby ~100usec

        // make some register reads

        // TODO ... need some fine tune ......

        int n;
        for (n = 0; n < 200; n++) {
            adc3110_SerialBus_read_unlocked(crate, fmc, ADS01, 0x08);
        }
    }

    return res;
}
#endif

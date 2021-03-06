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
#include "ifcdaqdrv_adc3110.h"

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
        LOG((LEVEL_ERROR,"%s(): Error: (channel=%d)\n", __FUNCTION__, channel));
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

    /* Activate FMC */
    status = ifc_fmc_tcsr_write(ifcdevice, 0, 0x31100000);

    ifcdevice->init_adc              = adc3110_init_adc_alternative;
    ifcdevice->get_signature         = adc3110_get_signature;
    ifcdevice->set_led               = adc3110_set_led;
    ifcdevice->get_gain              = adc3110_get_gain;
    ifcdevice->set_gain              = adc3110_set_gain;
    ifcdevice->set_clock_frequency   = adc3110_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3110_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3110_set_clock_source;
    ifcdevice->get_clock_source      = adc3110_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3110_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3110_get_clock_divisor;
    ifcdevice->set_pattern           = adc3110_set_test_pattern;
    ifcdevice->get_pattern           = adc3110_get_test_pattern;
    ifcdevice->calc_sample_rate      = adc3110_calc_sample_rate;

    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 8;

    ifcdevice->resolution  = 16;
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 125; //1045;
    ifcdevice->divisor_min = 8; //1;

    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 1;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 10;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

ifcdaqdrv_status adc3110_init_adc(struct ifcdaqdrv_dev *ifcdevice){
    int res = 0;
    
    // led off
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_blink_fast);
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_blink_slow);

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
            INFOLOG(("Error: %s(crate=%d,fmc=%d,device=%d,dataformat=%d) wrong dataformat!\n", __FUNCTION__,
                   ifcdevice->card, ifcdevice->fmc, adc3110_get_sbc_device(i * 2), dataformat));
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
        INFOLOG(("ERROR: %s(device=%d,addr=%d,writecmd=%d) unknown device %d", __FUNCTION__, device, addr, writecmd,
               device));
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

/* seems unneccessary */
ifcdaqdrv_status adc3110_ctrl_front_panel_gpio(struct ifcdaqdrv_dev *ifcdevice, int32_t value){
    return ifc_fmc_tcsr_write(ifcdevice, 5, value);
}

ifcdaqdrv_status adc3110_tmp102_read(struct ifcdaqdrv_dev *ifcdevice, unsigned reg, uint32_t *ui32_reg_val){
    int      status;
    uint32_t device = 0x01040048; // I2C Bus device tmp102

    device |= ifcdevice->fmc == 1 ? IFC_FMC1_I2C_BASE : IFC_FMC2_I2C_BASE;

    /*TODO: check usage of i2c_read. Who is the first argument ? */
    
    if (ifcdaqdrv_is_byte_order_ppc()) {
        status  = tsc_i2c_read(ifcdevice->node, device, reg, ui32_reg_val);

        /* TODO fix bit mask */
        if ((status & I2C_CTL_EXEC_MASK) == I2C_CTL_EXEC_ERR) {
            return status_i2c_nack;
        }
    }
    else { //x86_64
        /* keep compatibility and remove warnings*/
        status = 0;
        *ui32_reg_val = (uint32_t) status;
    }
    
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

    /*********************************************************************************/
    //usleep(2000);

    //adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1E, i32_reg_val); // PLL2 P/N Recallibration
    //adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0C, 0x03800000); // PLL2 LD pin programmable

    /********************************************************************************************/
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
    const int timeout = 120; // 60ms

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
    INFOLOG(("%s(): Warning: Failed to lock clock..\n", __FUNCTION__));
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
        INFOLOG(("Error: adc3110_ADC_setOffset(device=%d,channel=%d) wrong device!\n", device, channel));
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

/* This function is used because it is compatible with the IFC1410. The original "init_adc" was 
created for IFC1210*/
ifcdaqdrv_status adc3110_init_adc_alternative(struct ifcdaqdrv_dev *ifcdevice)
{
    // led off
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc0, ifcdaqdrv_led_blink_fast);
    adc3110_set_led(ifcdevice, ifcdaqdrv_led_fmc1, ifcdaqdrv_led_blink_slow);

    /* -----------------------------Starting initialization procedure -------------------------- */

    // Power down ADS #01 #23 #4567 
    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0x1D00);
    usleep(2000);

    ifc_fmc_tcsr_write(ifcdevice, 0x01, 0); // Release RESET on all ADS
    ifc_fmc_tcsr_write(ifcdevice, 0x02, 0); // LED_MGT FP Led  power-off + oscillator 100  power-off

    /*
     * Setup LMK04906
     */
    /* ---------------------------------Configuring LMK04906 ----------------------------------- */
  
    // Reset device
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x00, 0x00020000);
    usleep(2000);

    //Enable the six clock outputs divided by 10
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x00, 0x00000140); // LMK04906_R00 Enable + ClkOUT0_DIV = 10  <(Test point R184))
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x01, 0x00000140); // LMK04906_R01 Enable  ClkOut_1 + ClkOUT0_DIV = 10  -> 2500/10 = 250 MHz
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x02, 0x00000140); // LMK04906_R02 Enable  ClkOut_2 + ClkOUT0_DIV = 10  
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x03, 0x00000140); // LMK04906_R02 Enable  ClkOut_3 + ClkOUT0_DIV = 10  
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x04, 0x00000140); // LMK04906_R02 Enable  ClkOut_4 + ClkOUT0_DIV = 10  
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x05, 0x00000140); // LMK04906_R05 Enable  ClkOut_5 + ClkOUT0_DIV = 10  -> 2500/10 = 250 MHz

    // Set LMK04906 outputs to LVDS (This MUST be done before enabling CCHD575)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x06, 0x01100000); // ClkOut0_Type/Clk_Out1_Type = 1 (LVDS)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x07, 0x01100000); // ClkOut2_Type/Clk_Out3_Type = 1 (LVDS)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x08, 0x01010000); // ClkOut4_Type/Clk_Out5_Type = 1 (LVDS)

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x09, 0x55555540); // For "Proper" operation...

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0A, 0x11404200); // LMK04906_R10 OscOUT_Type = 1 (LVDS)  Powerdown    
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0B, 0x34028000); // LMK04906_R11 Device MODE=0x6 + No SYNC
    //adc3110_SerialBus_write(ifcdevice, LMK04906,0x0B,0x37f28000); // Device MODE=0x6 + No SYNC output

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0C, 0x13000000); // LMK04906_R12 LD pin programmable (-> PLL_2 DLD output)    

    //adc3110_SerialBus_write(ifcdevice,LMK04906,0x0D, 0x3B700240); // HOLDOVER pin uWRITE SDATOUT ClkIn_SELECT_MODE = ClkIn1  Enable CLKin1 = 1
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0D, 0x3B7002c8); // HOLDOVER pin uWIRE SDATOUT  Enable CLKin1
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0E, 0x00000000); // Bipolar Mode CLKin1 INPUT
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x0F, 0x00000000); // DAC unused

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x10, 0x01550400); // OSC IN level

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x18, 0x00000000); // PLL1 not used / PLL2 used
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x19, 0x00000000); // DAC config not used

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1A, 0x8FA00000); // PLL2 used / ICP = 3200uA
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1B, 0x00000000); // PLL1 not used
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1C, 0x00200000); // PLL2_R = 2 / PLL1 N divisor=00
    
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1D, 0x01800320); // LMK04906_R29 OSCIN_FREQ /PLL2_NCAL = 25)
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1E, 0x02000320); // LMK04906_R30 /PLL2_P = 2 PLL2_N = 25
    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1F, 0x00000000); // LMK04906_R31 uWIRE Not LOCK

    /* ----------------------Enable Internal 100 MHz clock from  +OSC575 ------------------------ */
    ifc_fmc_tcsr_write(ifcdevice, 0x02, 0x80000003);
    usleep(2000);

    adc3110_SerialBus_write(ifcdevice, LMK04906, 0x1E, 0x02000320); // LMK04906_R30 /PLL2_P = 2 PLL2_N = 25
    usleep(20000);

    /* --------------------------------Configuring the ADCs --------------------------------------*/
    
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


    /* -------------------------------Checking if CLK is locked ----------------------------------- */

    // Verification Clock has started
    // Warning: ads42lb69 01 shall be initialized
    const int timeout = 120; // 60ms

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

        char *p;
        p = ifcdevice->fru_id->product_name;
        
        INFOLOG(("Initialization of %s is complete\n",p));
        return status_success;
    }

    // Failed to set clock
    // return status_internal;
    INFOLOG(("%s(): Warning: Failed to lock clock..\n", __FUNCTION__));
    
    return status_success;
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

ifcdaqdrv_status adc3110_calc_sample_rate(struct ifcdaqdrv_usr *ifcuser, int32_t *averaging, int32_t *decimation, int32_t *divisor, double *freq, double *sample_rate, uint8_t sample_rate_changed)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;


    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }


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

    return status;
}


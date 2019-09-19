#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <libudev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>

#include "tscioctl.h"
#include "tsculib.h"

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_adc3112.h"

static int ifc_adc3112_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice);
static uint32_t ifc_adc3112_SerialBus_prepare_command(ADC3112_SBCDEVICE device, int addr, int writecmd);

static ADC3112_SBCDEVICE adc3112_get_sbc_device(int channel);

static ADC3112_SBCDEVICE adc3112_get_sbc_device(int channel) {
    switch (channel) {
    case 0:
    case 1:
        return ADS5409_01;
    case 2:
    case 3:
        return ADS5409_23;
    }
    return ADS5409_01;
}

ifcdaqdrv_status adc3112_init_adc(struct ifcdaqdrv_dev *ifcdevice){
    return -1;
#if 0
    clock = 250;
    if (ifc_adc3112_init(card->pevCrate, card->FMCslot, &clock) != 0) {
        printf("%s: ERROR IFC1210Scope module: ADC3112 init failed!\n", MY_ID_STR);


        printf("... IGNORE for test ...\n");

        // free(card);
        // return NULL;
    } else {
        int verbose = 0;

        if (DBG(DBG_LEVEL2)) {
            verbose = 1;
        }

        // calibrate ADS5409 and TTIM ISERDES
        ifc_adc3112_fastscope_calibrate_ISERDES(card->pevCrate, card->FMCslot, 1000.0, 1, 1, 0, verbose);

        // set clock
        ifc_adc3112_setClock_internalCLKREF(card->pevCrate, card->FMCslot, clock, &clock);
    }
#endif
}

ifcdaqdrv_status adc3112_set_dataformat(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat){
    int res = 0;
    int i;

    for (i = 0; i < 2; ++i) {
        uint32_t val = ifc_adc3112_SerialBus_read(ifcdevice, adc3112_get_sbc_device(i * 2), 0x01);

        switch (dataformat) {
        case ifcdaqdrv_dataformat_signed:
            val &= ~(1 << 3);
            break;
        case ifcdaqdrv_dataformat_unsigned:
            val |= (1 << 3);
            break;
        default:
            // wrong dataformat
            printf("Error: %s(crate=%d,fmc=%d,device=%d,dataformat=%d) wrong dataformat!\n", __FUNCTION__,
                   ifcdevice->card, ifcdevice->fmc, adc3112_get_sbc_device(i * 2), dataformat);
            return -1;
            break;
        }

        res += ifc_adc3112_SerialBus_write(ifcdevice, adc3112_get_sbc_device(i * 2), 0x01, val); // ADS5409_R01 bit 3 Data format
    }
    return res ? -1 : 0;
}

static int ifc_adc3112_SerialBus_isReady(struct ifcdaqdrv_dev *ifcdevice){
    // Doc: ADC3112_UG_A0 3.2.4 TCSR ADC_3112 Serial_IF Control Register

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

static uint32_t ifc_adc3112_SerialBus_prepare_command(ADC3112_SBCDEVICE device, int addr, int writecmd){
    // Doc: ADC3112_UG_A0 3.2.4 TCSR ADC_3112 Serial_IF Control Register
    // Doc: ADC3112_UG_A0 3.2.5 TCSR ADC_3112 Serial_IF Data Register

    uint32_t cmd = 0;

    switch (device) {
    // --------------------------------------------------------------------
    // PORTSEL = "01"
    // --------------------------------------------------------------------
    case ADS5409_01:
    case ADS5409_23:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr

        switch (device) {
        case ADS5409_01:
            cmd |= 0x01000000;     // PORTSEL[25:24]="01" DEV_SEL[17:16]="00"
            break;
        case ADS5409_23:
        default:
            cmd |= 0x01010000;     // PORTSEL[25:24]="01" DEV_SEL[17:16]="01"
            break;
        }
        break;

    case XRA01:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x01020000;    // PORTSEL[25:24]="01" DEV_SEL[17:16]="10"
        break;
    case XRA23:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x01030000;    // PORTSEL[25:24]="01" DEV_SEL[17:16]="11"
        break;

    // --------------------------------------------------------------------
    // PORTSEL = "11"
    // --------------------------------------------------------------------

    case DACCMP:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x03000000;    // PORTSEL[25:24]="11" DEV_SEL[17:16]="00"
        break;

    case XRATRIG:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x03010000;    // PORTSEL[25:24]="11" DEV_SEL[17:16]="01"
        break;

    // --------------------------------------------------------------------
    // PORTSEL = "10"
    // --------------------------------------------------------------------

    case LMK04803B:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x0E000000;    // PORTSEL[25:24]="10" DEV_SEL[17:16]="00" LMK_2CLK_LE[27:26]="11"(3 extra uWIRE clock with LE='1')
        break;

    case SY:
        cmd  = 0;
        cmd |= (addr & 0xff); // SER_ADD[15:0]=addr
        cmd |= 0x02010000;    // PORTSEL[25:24]="10" DEV_SEL[17:16]="01"
        break;

    default:

        printf("ERROR: %s(device=%d,addr=%d,writecmd=%d) unknown device %d", __FUNCTION__, device, addr, writecmd,
               device);

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

ifcdaqdrv_status ifc_adc3112_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3112_SBCDEVICE device, int addr,
                                             uint32_t data){
    // Doc: ADC3112_UG_A0 3.2.4 TCSR ADC_3112 Serial_IF Control Register
    // Doc: ADC3112_UG_A0 3.2.5 TCSR ADC_3112 Serial_IF Data Register

    int      base = (ifcdevice->fmc == 1) ? 0x80 : 0xC0;

    uint32_t cmd  = ifc_adc3112_SerialBus_prepare_command(device, addr, 1); // create write command

    // check if serial bus is ready
    if (ifc_adc3112_SerialBus_isReady(ifcdevice) != 0) {
        printf("Error: %s(crate=%u,fmc=%u,device=%u,addr=%d,data=%d) serial bus is not ready!", __FUNCTION__,
               ifcdevice->card, ifcdevice->fmc, device, addr, data);
        return -1;
    }

    ifc_xuser_tcsr_write(ifcdevice, base + 0x04, data);
    ifc_xuser_tcsr_write(ifcdevice, base + 0x03,  cmd);

    return 0;
}

uint32_t ifc_adc3112_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3112_SBCDEVICE device, int addr){
    // Doc: ADC3112_UG_A0 3.2.4 TCSR ADC_3112 Serial_IF Control Register
    // Doc: ADC3112_UG_A0 3.2.5 TCSR ADC_3112 Serial_IF Data Register

    int      base = (ifcdevice->fmc == 1) ? 0x80 : 0xC0;

    uint32_t cmd  = ifc_adc3112_SerialBus_prepare_command(device, addr, 0); // create read command

    // check if serial bus is ready
    if (ifc_adc3112_SerialBus_isReady(ifcdevice) != 0) {
        printf("Error: %s(crate=%u,fmc=%u,device=%u,addr=%d) serial bus is not ready!", __FUNCTION__, ifcdevice->card,
               ifcdevice->fmc, device, addr);
        return -1;
    }

    ifc_xuser_tcsr_write(ifcdevice, base + 0x03, cmd);

    // check if serial bus is ready
    if (ifc_adc3112_SerialBus_isReady(ifcdevice) != 0) {
        printf("Error: %s(crate=%u,fmc=%u,device=%u,addr=%d) write command timeout!", __FUNCTION__, ifcdevice->card,
               ifcdevice->fmc, device, addr);
        return -1;
    }

    int32_t data;
    ifc_fmc_tcsr_read(ifcdevice, 0x04, &data);

    return data;
}

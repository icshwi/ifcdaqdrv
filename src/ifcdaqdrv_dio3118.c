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
#include "ifcdaqdrv_dio3118.h"
#include "ifcdaqdrv_scope.h"

static ifcdaqdrv_status dio3118_check_ttl_cfg(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_CFG_REG, &i32_reg_val);
    if (i32_reg_val != 0x01011010) {
        printf("ERROR: %s: TTL configuration is invalid %08x\n", __FUNCTION__, i32_reg_val);
        return status_incompatible;
    } else {
        printf("TTL configuration is OK!\n");
    }
    return status;
}

static ifcdaqdrv_status dio3118_check_strapping(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_MCSR_REG, &i32_reg_val);
    if ((i32_reg_val >> 28) == 0x0) {
        printf("DIO3118 is TTL strapped\n");
        status = dio3118_check_ttl_cfg(ifcdevice);
        if (status)
            return status;
    } else if ((i32_reg_val >> 28) == 0x1)
        printf("DIO3118 is LVDS strapped\n");
    else
        printf("ERROR: %s: Unknown strapping 0x%x\n", __FUNCTION__, (i32_reg_val >> 28));

    return status;
}

static ifcdaqdrv_status dio3118_check_ready(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;
    int32_t i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_MCSR_REG, &i32_reg_val);
    if (i32_reg_val & 0x000100A1) {
        printf("DIO3118 is ready\n");
    } else {
        printf("ERROR: %s: DIO3118 is not ready 0x%08x\n", __FUNCTION__, i32_reg_val);
        return status_internal;
    }

    return status;
}

static ifcdaqdrv_status dio3118_get_temperature(struct ifcdaqdrv_dev *ifcdevice, uint32_t *data) {
    ifcdaqdrv_status  status;
    uint32_t device = 0x40040048;
    uint32_t reg_val;

    if (!data) {
        return status_argument_invalid;
    }

    if (ifcdevice->fmc == 1) {
        device |= 0x80000000;
    }
    else if (ifcdevice->fmc == 2) {
        device |= 0xa0000000;
    }

    status = tsc_i2c_read(device, 0, &reg_val);
    data[0] = reg_val;
    status = tsc_i2c_read(device, 1, &reg_val);
    data[1] = reg_val;
    status = tsc_i2c_read(device, 2, &reg_val);
    data[2] = reg_val;
    status = tsc_i2c_read(device, 3, &reg_val);
    data[3] = reg_val;

    return status;
}

static ifcdaqdrv_status dio3118_test_loopback(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status  status;
    int32_t i32_reg_val, i;
    uint32_t output = 0x00000001, input = 0x00008000;

    printf("\nAll input must be zero: ");
    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_DSI_DAT_REG, &i32_reg_val);
    printf("0x%08x : %s\n", i32_reg_val, i32_reg_val ? "NOK" : "OK");

    for (i = 15; i >= 0; i--) {
        status += ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSO_DAT_REG, output); //Set output
        printf("Input %d should be set: ", i);
        status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_DSI_DAT_REG, &i32_reg_val);
        printf("0x%08x : %s\n", i32_reg_val, i32_reg_val & input ? "OK" : "NOK"); //Check input
        status += ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSO_DAT_REG, 0x00000000);

        input = input >> 1;
        output = output << 1;
    }

    status += ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSO_DAT_REG, 0x00100000); //Set LVDS output
    printf("LVDS input should be set: ");
    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_DSI_DAT_REG, &i32_reg_val);
    printf("0x%08x : %s\n", i32_reg_val, i32_reg_val & 0x00100000 ? "OK" : "NOK"); //Check LVDS input
    status += ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSO_DAT_REG, 0x00000000);

    return status;
}

ifcdaqdrv_status dio3118_register(struct ifcdaqdrv_dev *ifcdevice) {
    ifcdaqdrv_status status;


    status = dio3118_get_signature(ifcdevice, NULL, NULL, &ifcdevice->board_id);
    if (status)
        return status;

    /* Check strapping 0 = TTL, 1 = LVDS */
    status = dio3118_check_strapping(ifcdevice);
    if (status)
        return status;

    /* Activate FMC */
    status = ifc_fmc_tcsr_write(ifcdevice, DIO3118_SIGN_REG, 0x31180000);
    usleep(1000);

    /* Enable to show temperature, Not supported if card is in FMC 2 */
    /*uint32_t temp[4];
    status = dio3118_get_temperature(ifcdevice, temp);
    if (status)
        printf("Can't read temp sensor!\n");
    else
        printf("Temperature: %.2f\n", (float)(temp[0]&0xfff));*/

    ifcdevice->init_adc              = dio3118_init_dio;
    ifcdevice->get_signature         = dio3118_get_signature;
    ifcdevice->set_led               = dio3118_set_led;

    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    return status;
}

ifcdaqdrv_status dio3118_init_dio(struct ifcdaqdrv_dev *ifcdevice){
    ifcdaqdrv_status status;

    if (ifcdevice->board_id != 0x3118) {
        printf("Error: %s: No DIO3118 installed on fmc%d [%04x]\n", __FUNCTION__, ifcdevice->fmc, ifcdevice->board_id);
        return status_incompatible;
    }

    status = dio3118_check_ready(ifcdevice);
    if (status)
        return status;

    status = ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSO_DAT_REG, 0x00000000); //Zero on all outputs
    status += ifc_fmc_tcsr_write(ifcdevice, DIO3118_DSOE_DAT_REG, 0x0010FFFF); //Enable all outputs

    /* Enable to run loopback test, connect camera side to output */
    //status += dio3118_test_loopback(ifcdevice);

    return status;
}

ifcdaqdrv_status dio3118_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state){
    uint32_t reg   = 0;
    uint32_t value = 0;
    uint32_t mask  = 0;

    if (led == ifcdaqdrv_led_ifc) {
        reg = DIO3118_IFC_MGT_REG;
        mask = DIO3118_IFC_MGT_LED_MASK;

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
            printf("Error: %s: Unknown led state %d\n", __FUNCTION__, led_state);
            return status_argument_range;
        }
    } else {
        printf("Error: %s: Unknown led %d\n", __FUNCTION__, led);
        return status_argument_range;
    }

    return ifc_fmc_tcsr_setclr(ifcdevice, reg, value, mask);
}

ifcdaqdrv_status dio3118_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                       uint16_t *board_id) {
    ifcdaqdrv_status status;
    int32_t          i32_reg_val;

    status = ifc_fmc_tcsr_read(ifcdevice, DIO3118_SIGN_REG, &i32_reg_val);

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



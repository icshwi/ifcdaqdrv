#ifndef _IFC_DIO3118_H_
#define _IFC_DIO3118_H_ 1

#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"

#define DIO3118_SIGN_REG                0x0
#define DIO3118_MCSR_REG                0x1
#define DIO3118_CFG_REG                 0x2
#define DIO3118_DSI_DAT_REG             0x3
#define DIO3118_DSO_DAT_REG             0x4
#define DIO3118_DSO_CLR_REG             0x5
#define DIO3118_DSO_SET_REG             0x6
#define DIO3118_IFC_MGT_REG             0x7
#define DIO3118_MISSING_IN_DOC_REG      0x8
#define DIO3118_DSOE_DAT_REG            0x9
#define DIO3118_DSOE_CLR_REG            0xA
#define DIO3118_DSOE_SET_REG            0xB

#define DIO3118_IFC_MGT_LED_MASK        0x00000003

#define DIO3118_SIGNATURELEN 8

ifcdaqdrv_status dio3118_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status dio3118_init_dio(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status dio3118_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state);
ifcdaqdrv_status dio3118_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,  uint16_t *board_id);

#endif // _IFC_DIO3118_H_

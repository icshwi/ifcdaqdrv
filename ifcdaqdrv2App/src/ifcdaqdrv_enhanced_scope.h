#ifndef _IFC_ENHSCOPE_H_
#define _IFC_ENHSCOPE_H_ 1

#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"

// TODO: fix the offset, is different from registers 0x60 to 0x70
static inline int32_t ifc_get_enhscope_fmc_offset(struct ifcdaqdrv_dev *ifcdevice) {
    if (ifcdevice->fmc == 1) return 0x00;
    else return 0x04;
}

ifcdaqdrv_status ifc_enhscope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_enhscope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_enhscope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask);

ifcdaqdrv_status enhanced_scope_register(struct ifcdaqdrv_dev *ifcdevice);


#endif // _IFC_ENHSCOPE_H_


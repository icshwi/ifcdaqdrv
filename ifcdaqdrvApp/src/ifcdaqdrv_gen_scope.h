#ifndef _IFCDAQDRV_GEN_SCOPE_H_
#define _IFCDAQDRV_GEN_SCOPE_H_ 1

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"

#define IFC1410SCOPEDRV_GEN_3110_3110_SIGNATURE 0x12353110
#define IFC1410SCOPEDRV_GEN_3117_3117_SIGNATURE 0x12353117

#define IFC_GEN_SCOPE_SRAM_SAMPLES_OFFSET 0x0

/* GEN SCOPE MAIN TCSR (0x60 to 0x6F) */
#define IFC_GEN_SCOPE_TCSR_APP_SIGN_REG          0x0 /* FMC2 0x4 */
#define IFC_GEN_SCOPE_TCSR_DEFMAP_REG            0x1 /* FMC2 0x5 */
#define IFC_GEN_SCOPE_TCSR_RW_TEST_REG           0x3 /* FMC2 0x7 */
#define IFC_GEN_SCOPE_TCSR_FE_FMC_SIGN_REG       0x8 /* FMC2 0xC */
#define IFC_GEN_SCOPE_TCSR_FE_CONTROL_STATUS_REG 0x9 /* FMC2 0xD */
#define IFC_GEN_SCOPE_TCSR_FE_TRIGGER_REG        0xB /* FMC2 0xF */

/* GEN SCOPE ACQ TCSR (0x70-0x7F) */
#define IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_CFG_REG       0x0 /* FMC2 0x8 */
#define IFC_GEN_SCOPE_TCSR_ACQ_RGBUF_BASE_ADDR_REG 0x1 /* FMC2 NOT USED */
#define IFC_GEN_SCOPE_TCSR_ACQ_CONTROL_STATUS_REG  0x2 /* FMC2 0xA */
#define IFC_GEN_SCOPE_TCSR_ACQ_TRIGGER_MARKER_REG  0x3 /* FMC2 0xB */
#define IFC_GEN_SCOPE_TCSR_ACQ_GLOBAL_TIME_REG     0x6 /* FMC2 0xE */

#define IFC_GEN_SCOPE_TRIGGER_MARKER_MASK          0x3FFFC
#define IFC_GEN_SCOPE_ACQ_BUFFER_MODE_MASK         0xE0
#define IFC_GEN_SCOPE_FE_AVERAGING_MASK            0x8000000
#define IFC_GEN_SCOPE_FE_PRESCL_MASK               0x7000000
#define IFC_GEN_SCOPE_DPRAM_BUF_SIZE_MASK          0xC00

#define IFC_GEN_SCOPE_AMC_RX_IN_SELECT             0x1C

/* Functions for accessing GEN SCOPE MAIN TCSR (0x60 to 0x6F) */
static inline int32_t ifc_get_gen_scope_tcsr_offset(struct ifcdaqdrv_dev *ifcdevice)
{
    if (ifcdevice->fmc == 1) return 0x60;
    else return 0x64;
}
ifcdaqdrv_status ifc_gen_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_gen_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_gen_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask);

/* Functions for accessing GEN SCOPE ACQ TCSR (0x70-0x7F) */
static inline int32_t ifc_get_gen_scope_acq_tcsr_offset(struct ifcdaqdrv_dev *ifcdevice)
{
    if (ifcdevice->fmc == 1) return 0x70;
    else return 0x78;
}
ifcdaqdrv_status ifc_gen_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_gen_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_gen_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask);

ifcdaqdrv_status ifcdaqdrv_gen_scope_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcdaqdrv_gen_scope_arm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_gen_scope_disarm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_gen_scope_wait_acq_end(struct ifcdaqdrv_usr *ifcuser);

ifcdaqdrv_status ifcdaqdrv_gen_scope_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold, uint32_t mask, uint32_t rising_edge);
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger, int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples);

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples);
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples);

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_trigger_marker(struct ifcdaqdrv_dev *ifcdevice, uint32_t *trigger_marker);

ifcdaqdrv_status ifcdaqdrv_gen_scope_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq);
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq);

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples);
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);

ifcdaqdrv_status ifcdaqdrv_gen_scope_get_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average);
ifcdaqdrv_status ifcdaqdrv_gen_scope_set_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t average);

ifcdaqdrv_status ifcdaqdrv_gen_scope_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation);
ifcdaqdrv_status ifcdaqdrv_gen_scope_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation);

ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data);
ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);
ifcdaqdrv_status ifcdaqdrv_gen_scope_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples);
ifcdaqdrv_status ifcdaqdrv_gen_scope_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm);

#endif // _IFCDAQDRV_GEN_SCOPE_H_


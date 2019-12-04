#ifndef _IFCDAQDRV_SCOPE_LITE_H_
#define _IFCDAQDRV_SCOPE_LITE_H_ 1

#define IFC1410SCOPEDRV_SCOPE_LITE_SIGNATURE    0x73570001

#define IFC_SCOPE_LITE_SRAM_SAMPLES_OFFSET   	0x00100000

/* SCOPE LITE MAIN TCSR (0x60 to 0x6F) */
#define IFC_SCOPE_LITE_TCSR_APP_SIGN_REG            0x0
#define IFC_SCOPE_LITE_TCSR_ACQ_CHANNEL_SEL_REG     0x1
#define IFC_SCOPE_LITE_TCSR_ACQ_CONTROL_STATUS_REG  0x2
#define IFC_SCOPE_LITE_TCSR_ACQ_STATUS_1_REG        0x3
#define IFC_SCOPE_LITE_TCSR_ACQ_STATUS_2_REG        0x4
#define IFC_SCOPE_LITE_TCSR_FMC_STATUS_REG          0x5
#define IFC_SCOPE_LITE_TCSR_SBUF_SELECT_REG         0x8
#define IFC_SCOPE_LITE_TCSR_SBUF_CONTROL_STATUS_REG 0x9
#define IFC_SCOPE_LITE_TCSR_SBUF_STATUS_1_REG       0xA
#define IFC_SCOPE_LITE_TCSR_SBUF_STATUS_2_REG       0xB

/* SCOPE LITE WGEN TCSR (0x70-0x7A */
#define IFC_SCOPE_LITE_TCSR_WGEN_CONTROL_STATUS_REG 0x0
#define IFC_SCOPE_LITE_TCSR_WGEN_DAC_DEF_01_REG     0x1
#define IFC_SCOPE_LITE_TCSR_WGEN_DAC_DEF_23_REG     0x2
#define IFC_SCOPE_LITE_TCSR_WGEN_INIT_ADDR_0_REG    0x3
#define IFC_SCOPE_LITE_TCSR_WGEN_STEP_0_REG         0x4
#define IFC_SCOPE_LITE_TCSR_WGEN_INIT_ADDR_1_REG    0x5
#define IFC_SCOPE_LITE_TCSR_WGEN_STEP_1_REG         0x6
#define IFC_SCOPE_LITE_TCSR_WGEN_INIT_ADDR_2_REG    0x7
#define IFC_SCOPE_LITE_TCSR_WGEN_STEP_2_REG         0x8
#define IFC_SCOPE_LITE_TCSR_WGEN_INIT_ADDR_3_REG    0x9
#define IFC_SCOPE_LITE_TCSR_WGEN_STEP_3_REG         0xA

/* Functions for accessing SCOPE LITE MAIN TCSR (0x60 to 0x6F) */
ifcdaqdrv_status ifc_scope_lite_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_lite_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_lite_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask);

/* Functions for accessing SCOPE LITE WGEN TCSR (0x70 to 0x7A) */
ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_lite_wgen_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t clrmask);

ifcdaqdrv_status ifcdaqdrv_scope_lite_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcdaqdrv_scope_lite_arm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_scope_lite_disarm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_scope_lite_wait_acq_end(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_scope_lite_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold, uint32_t mask, uint32_t rising_edge);
ifcdaqdrv_status ifcdaqdrv_scope_lite_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger, int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);
ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data);
ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);
ifcdaqdrv_status ifcdaqdrv_scope_lite_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples);
ifcdaqdrv_status ifcdaqdrv_scope_lite_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm);

#endif //_IFCDAQDRV_SCOPE_LITE_H_

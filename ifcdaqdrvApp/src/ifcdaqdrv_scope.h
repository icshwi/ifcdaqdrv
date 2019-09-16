#ifndef _IFCDAQDRV_SCOPE_H_
#define _IFCDAQDRV_SCOPE_H_ 1

#define IFC1210SCOPEDRV_SCOPE_SIGNATURE       0x12110301
#define IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE   0x12100501

#define IFC1410SCOPEDRV_SCOPE_SIGNATURE       0x14100301

/*
 * The SCOPE TCSR has two registers, 0x70 and 0x74, for control & status of SRAM1/2 which are used by FMC1/2 specific
 * details such as Acquisition, trigger and downsampling mode. It also has two registers, 0x71 and 0x75, for control of
 * the triggers. Finally it has two registers, 0x72 and 0x76, which store the last pre-trigger address.
 *
 * Then there are two more duplicates of all these registers to control SMEM.
 *
 *   XUSER_SCOPE_UG       chapter 5.3.8
 *   XUSER_FASTSCOPE_UG   chapter 5.4.9
 */

#define IFC_SCOPE_TCSR_APP_SIGN_REG             0
#define IFC_SCOPE_TCSR_APP_INT_STATUS_REG       8

#define IFC_SCOPE_ACQ_TCSR_CS_REG               0
#define IFC_SCOPE_ACQ_TCSR_TRIG_REG             1
#define IFC_SCOPE_ACQ_TCSR_LA_REG               2
#define IFC_SCOPE_ACQ_TCSR_SMEM_BASE_SIZE_REG   3

#define IFC_SCOPE_TCSR_CS_ACQ_Single_MASK            0x00000001
#define IFC_SCOPE_TCSR_CS_ACQ_Auto_MASK              0x00000002
#define IFC_SCOPE_TCSR_CS_ACQ_downSMP_MASK           0x0000001C
#define IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_MASK       0x000000E0
#define IFC_SCOPE_TCSR_CS_FMC_Installed_MASK         0x00000700
#define IFC_SCOPE_TCSR_CS_SRAM_Buffer_Size_MASK      0x00000800
#define IFC_SCOPE_TCSR_CS_SRAM_ACQ_Size_MASK         0x00007000
#define IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_MASK       0x00008000
#define IFC_SCOPE_TCSR_CS_ACQ_CLKERR_MASK            0x00080000
#define IFC_SCOPE_TCSR_CS_ACQ_Event_CNT_MASK         0x0FF00000
#define IFC_SCOPE_TCSR_CS_ACQ_Status_MASK            0x30000000
#define IFC_SCOPE_TCSR_CS_ACQ_Command_MASK           0xC0000000

#define IFC_SCOPE_TCSR_CS_ACQ_Single_SHIFT           0
#define IFC_SCOPE_TCSR_CS_ACQ_Auto_SHIFT             1
#define IFC_SCOPE_TCSR_CS_ACQ_downSMP_SHIFT          2
#define IFC_SCOPE_TCSR_CS_ACQ_Buffer_Mode_SHIFT      5
#define IFC_SCOPE_TCSR_CS_FMC_Installed_SHIFT        8
#define IFC_SCOPE_TCSR_CS_SRAM_Buffer_Size_SHIFT     11
#define IFC_SCOPE_TCSR_CS_SRAM_ACQ_Size_SHIFT        12
#define IFC_SCOPE_TCSR_CS_ACQ_downSMP_MOD_SHIFT      15
#define IFC_SCOPE_TCSR_CS_ACQ_CLKERR_SHIFT           19
#define IFC_SCOPE_TCSR_CS_ACQ_Event_CNT_SHIFT        20
#define IFC_SCOPE_TCSR_CS_ACQ_Status_SHIFT           28
#define IFC_SCOPE_TCSR_CS_ACQ_Command_SHIFT          30

#define IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_CONT        0
#define IFC_SCOPE_TCSR_CS_ACQ_Single_VAL_SINGLE      1

#define IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_IDLE       0
#define IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_PRETRIG    1
#define IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_POSTTRIG   2
#define IFC_SCOPE_TCSR_CS_ACQ_Status_VAL_DONE       3

#define IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_NO_ACTION  0
#define IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ARM        1
#define IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_ABORT      2
#define IFC_SCOPE_TCSR_CS_ACQ_Command_VAL_REARM      3

#define IFC_SCOPE_TCSR_TRG_Trig_Level_MASK           0x0000FFFF
#define IFC_SCOPE_TCSR_TRG_Trig_GPIO_Sel_MASK        0x00030000
#define IFC_SCOPE_TCSR_TRG_Trig_GPIO_Gate_MASK       0x00040000
#define IFC_SCOPE_TCSR_TRG_Trig_GPIO_GatePol_MASK    0x00080000
#define IFC_SCOPE_TCSR_TRG_VME_Px_IO_Sel_MASK        0x07F00000
#define IFC_SCOPE_TCSR_TRG_Trig_Polarity_MASK        0x08000000
#define IFC_SCOPE_TCSR_TRG_Trig_Source_MASK          0x70000000
#define IFC_SCOPE_TCSR_TRG_Trig_Enable_MASK          0x80000000

#define IFC_SCOPE_TCSR_SRAMx_LA_Last_Address_MASK    0x0000FFFC
#define IFC_SCOPE_TCSR_SMEMx_LA_Last_Address_MASK    0x3FFFFFF8
#define IFC_SCOPE_TCSR_LA_Spec_CMD_MASK              0xC0000000

#define IFC_SCOPE_TCSR_LA_Last_Address_SHIFT         0
#define IFC_SCOPE_TCSR_LA_Spec_CMD_SHIFT             30

#define IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_MASK          0x000000F0
#define IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_MASK          0x0000F000
#define IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_MASK          0x00F00000
#define IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_MASK          0xF0000000

#define IFC_SCOPE_TCSR_TRGLEVEXT_SRAM1_SHIFT         4
#define IFC_SCOPE_TCSR_TRGLEVEXT_SRAM2_SHIFT         12
#define IFC_SCOPE_TCSR_TRGLEVEXT_SMEM1_SHIFT         20
#define IFC_SCOPE_TCSR_TRGLEVEXT_SMEM2_SHIFT         28

#define IFC_SCOPE_SRAM_SAMPLES_OFFSET        		0x01100000
#define IFC_SCOPE_SMEM_FMC2_SAMPLES_OFFSET   		0x10000000

#define IFC_SCOPE_MLVDS_CONTROL_REG					0x09
#define IFC_SCOPE_MLVDS_ENABLE_SHIFT				31


// The following option enables rearranging of pretrigger circular buffer.
#ifndef PRETRIG_ORGANIZE
#define PRETRIG_ORGANIZE 1
#endif

/* Functions for accessing SCOPE ACQ TCSR  (0x70-0x73, 0x74-0x77, 0x78-0x7B, 0x7C-0x7F (SCOPE FMC1/FMC2 and SRAM/SMEM specific)) */
static inline int32_t ifc_get_scope_acq_tcsr_offset(struct ifcdaqdrv_dev *ifcdevice) {
    if(ifcdevice->fmc == 1) {
        if(ifcdevice->mode == ifcdaqdrv_acq_mode_sram) {
            return 0x70;
        } else {
            return 0x78;
        }
    } else {
        if(ifcdevice->mode == ifcdaqdrv_acq_mode_sram){
            return 0x74;
        } else {
            return 0x7C;
        }
    }
}
/* Functions for accessing SCOPE MAIN TCSR (0x60 to 0x6F) */
ifcdaqdrv_status ifc_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                       clrmask);

/* Functions for accessing SCOPE ACQ TCSR  (0x70-0x73, 0x74-0x77, 0x78-0x7B, 0x7C-0x7F (SCOPE FMC1/FMC2 and SRAM/SMEM specific)) */
ifcdaqdrv_status ifc_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                           clrmask);

ifcdaqdrv_status ifcdaqdrv_scope_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcdaqdrv_scope_arm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_scope_disarm_device(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcdaqdrv_scope_wait_acq_end(struct ifcdaqdrv_usr *ifcuser);

ifcdaqdrv_status ifcdaqdrv_scope_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold, uint32_t mask, uint32_t rising_edge);
ifcdaqdrv_status ifcdaqdrv_scope_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger, int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);
ifcdaqdrv_status ifcdaqdrv_scope_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status ifcdaqdrv_scope_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);

ifcdaqdrv_status ifcdaqdrv_scope_get_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples);
ifcdaqdrv_status ifcdaqdrv_scope_set_sram_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples);

ifcdaqdrv_status ifcdaqdrv_scope_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned *nsamples);
ifcdaqdrv_status ifcdaqdrv_scope_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, unsigned nsamples);

//ifcdaqdrv_status ifc_scope_arm(struct ifcdaqdrv_dev *ifcdevice);

//typedef int  TRIGGERSLOPE;

//ifcdaqdrv_status ifc_scope_vmeio_int(struct ifcdaqdrv_dev *ifcdevice, int enable, TRIGGERSLOPE slope, const
//                                     char *vmeiosource);

// int ifc_scope_set_trigger(struct ifcdaqdrv_dev *ifcdevice, TRIGGERSOURCE source, TRIGGERSLOPE slope, uint32_t level, const char *vmeiosource, int gpioGateEnable, int gpioGateActiveHigh);


//ifcdaqdrv_status ifc_scope_manual_trigger(struct ifcdaqdrv_dev *ifcdevice);

/*
 *  enable VMEIO P2 for Application
 */
//ifcdaqdrv_status ifc_scope_vmeio_init(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status ifcdaqdrv_scope_get_sram_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address);
ifcdaqdrv_status ifcdaqdrv_scope_get_smem_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address);

ifcdaqdrv_status ifcdaqdrv_scope_read_ai(struct ifcdaqdrv_dev *ifcdevice, void *data);
ifcdaqdrv_status ifcdaqdrv_scope_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);

ifcdaqdrv_status ifcdaqdrv_scope_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src,
                                        size_t src_offset, size_t nelm, size_t channel_nsamples);
ifcdaqdrv_status ifcdaqdrv_scope_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data,
                                        size_t offset, size_t nelm);

ifcdaqdrv_status ifcdaqdrv_scope_switch_mode(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_acq_store_mode mode);

ifcdaqdrv_status ifcdaqdrv_scope_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples);
ifcdaqdrv_status ifcdaqdrv_scope_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);

ifcdaqdrv_status ifcdaqdrv_scope_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq);
ifcdaqdrv_status ifcdaqdrv_scope_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq);

ifcdaqdrv_status ifcdaqdrv_scope_get_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average);
ifcdaqdrv_status ifcdaqdrv_scope_set_average(struct ifcdaqdrv_dev *ifcdevice, uint32_t average);

ifcdaqdrv_status ifcdaqdrv_scope_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation);
ifcdaqdrv_status ifcdaqdrv_scope_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation);

ifcdaqdrv_status ifcdaqdrv_scope_init_smem_mode(struct ifcdaqdrv_dev *ifcdevice);

//Remove these
ifcdaqdrv_status ifcdaqdrv_scope_set_npretrig(struct ifcdaqdrv_dev *ifcdevice, uint32_t npretrig);
ifcdaqdrv_status ifcdaqdrv_scope_get_npretrig(struct ifcdaqdrv_dev *ifcdevice, uint32_t *npretrig);

#endif // _IFCDAQDRV_SCOPE_H_

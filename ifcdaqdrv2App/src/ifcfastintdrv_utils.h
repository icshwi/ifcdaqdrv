#ifndef _IFCFASTINT_UTILS_H_
#define _IFCFASTINT_UTILS_H_ 1

#include <stddef.h>

#include <pthread.h>
#include <linux/types.h>

#include "ifcdaqdrv2.h"
#include "ifcfastintdrv2.h"


#define IFCFASTINT_SRAM_PP_OFFSET 0x00100000
#define IFCFASTINT_SRAM_PP_STATUS 0x00101000

#define IFCFASTINT_SIGN_REG 0x60
#define IFCFASTINT_FMC1_CSR_REG 0x61
#define IFCFASTINT_FMC2_CSR_REG 0x62
#define IFCFASTINT_GENERAL_CSR_REG 0x63
#define IFCFASTINT_FSM_MAN_REG 0x64
#define IFCFASTINT_DIGITAL_PP_STATUS_REG 0x65
#define IFCFASTINT_ANALOG_PP_STATUS_REG 0x66
#define IFCFASTINT_DEBUG_REG 0x67
#define IFCFASTINT_BUF_SIZE_REG 0x68
#define IFCFASTINT_BUF_W_PTR_REG 0x69
#define IFCFASTINT_BUF_R_PTR_REG 0x6A

#define IFCFASTINT_FSM_MAN_SIGN_MASK                 0x000000FF
#define IFCFASTINT_FSM_MAN_HISTORY_STATUS_MASK       0x00000300
#define IFCFASTINT_FSM_MAN_HISTORY_RING_OVER_FF_MASK 0x00000400
#define IFCFASTINT_FSM_MAN_HISTORY_OVER_FF_MASK      0x00000800
#define IFCFASTINT_FSM_MAN_HISTORY_MODE_MASK         0x00007000
#define IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK          0x00008000
#define IFCFASTINT_FSM_MAN_FSM_FRQ_MASK              0x00030000
#define IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_MASK  0x00040000
#define IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_MASK   0x00080000
#define IFCFASTINT_FSM_MAN_FSM_OUT_MASK              0x00F00000
#define IFCFASTINT_FSM_MAN_FSM_STA_MASK              0x0F000000
#define IFCFASTINT_FSM_MAN_FSM_CMD_MASK              0xF0000000

#define IFCFASTINT_FSM_MAN_SIGN_SHIFT                 0
#define IFCFASTINT_FSM_MAN_HISTORY_STATUS_SHIFT       8
#define IFCFASTINT_FSM_MAN_HISTORY_RING_OVER_FF_SHIFT 10
#define IFCFASTINT_FSM_MAN_HISTORY_OVER_FF_SHIFT      11
#define IFCFASTINT_FSM_MAN_HISTORY_MODE_SHIFT         12
#define IFCFASTINT_FSM_MAN_HISTORY_ENA_SHIFT          15
#define IFCFASTINT_FSM_MAN_FSM_FRQ_SHIFT              16
#define IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_SHIFT  18
#define IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT   19
#define IFCFASTINT_FSM_MAN_FSM_OUT_SHIFT              20
#define IFCFASTINT_FSM_MAN_FSM_STA_SHIFT              24
#define IFCFASTINT_FSM_MAN_FSM_CMD_SHIFT              28

#define IFCFASTINT_DIGITAL_PP_STATUS_QOUT_MASK 0x00ffffff
#define IFCFASTINT_ANALOG_PP_STATUS_QOUT_MASK 0x00ffffff

/* Help to printout the 64-bit register content */
#define IFCFASTINT_ANALOGPP_REG_PPACT_MASK 		0x1000000000000000
#define IFCFASTINT_ANALOGPP_REG_PPACT_SHIFT		63
#define IFCFASTINT_ANALOGPP_REG_PPEMU_MASK		0x4000000000000000
#define IFCFASTINT_ANALOGPP_REG_PPEMU_SHIFT		62
#define IFCFASTINT_ANALOGPP_REG_PPMODE_MASK		0x0f00000000000000
#define IFCFASTINT_ANALOGPP_REG_PPMODE_SHIFT	56
#define IFCFASTINT_ANALOGPP_REG_PPVAL1_MASK		0x00000000ffff0000
#define IFCFASTINT_ANALOGPP_REG_PPVAL1_SHIFT	16
#define IFCFASTINT_ANALOGPP_REG_PPVAL2_MASK		0x00ffffff00000000
#define IFCFASTINT_ANALOGPP_REG_PPVAL2_SHIFT	32
#define IFCFASTINT_ANALOGPP_REG_PPCVAL_MASK		0x000000000000ffff
#define IFCFASTINT_ANALOGPP_REG_PPCVAL_SHIFT	0

#define IFCFASTINT_BUF_TST_MASK 		0x80000000	
#define IFCFASTINT_BUF_BIGENDIAN_MASK 	0x40000000	



ifcdaqdrv_status ifcfastintdrv_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcfastintdrv_write_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t pp_options);
ifcdaqdrv_status ifcfastintdrv_read_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *pp_options);
ifcdaqdrv_status ifcfastintdrv_read_pp_status(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *pp_status);
ifcdaqdrv_status ifcfastintdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcfastintdrv_printregister(uint64_t *pp_register);

void ifcfastintdrv_history_reset(struct ifcdaqdrv_dev *ifcdevice);

void ifcfastint_print_status(int32_t reg);

uint64_t u64_setclr(uint64_t input, uint64_t bits, uint64_t mask, uint32_t offset);


ifcdaqdrv_status ifcfastintdrv_read_smem_historybuffer( struct ifcdaqdrv_dev *ifcdevice, 
                                                        void *dest_buffer, 
                                                        struct tsc_ioctl_kbuf_req *dma_buf, 
                                                        uint32_t smem_addr, 
                                                        uint32_t size);

ifcdaqdrv_status ifcfastintdrv_read_rtstatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *rt_status);

ifcdaqdrv_status ifcfastintdrv_eeprom_read(struct ifcdaqdrv_dev *ifcdevice, 
                                           ifcfastint_aichannel_param param,
                                           int channel,
                                           double *value);
ifcdaqdrv_status ifcfastintdrv_eeprom_write(struct ifcdaqdrv_dev *ifcdevice, 
                                           ifcfastint_aichannel_param param,
                                           int channel,
                                           double value);

ifcdaqdrv_status ifcfastintdrv_read_rtstatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *rt_status);


#endif /* _IFCFASTINT_UTILS_H_ */

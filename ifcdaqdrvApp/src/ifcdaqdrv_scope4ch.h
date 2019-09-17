#ifndef _IFC_SCOPE4CH_H_
#define _IFC_SCOPE4CH_H_ 1

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"

#define IFC1410SCOPEDRV_SCOPE_LITE_4CHANNELS  0x73571704

#define IFC_SCOPE4CH_FMC1_SRAM_SAMPLES_OFFSET   	0x00100000
#define IFC_SCOPE4CH_FMC2_SRAM_SAMPLES_OFFSET   	0x00200000

#define SCOPE4CH_MAX_SAMPLES 131072 // Limited to 128k samples (128*1024)

#define IFC_SCOPE_BACKPLANE_MASK_REG				0x66
#define IFC_SCOPE_BACKPLANE_TRIGCNT_REG				0x67
#define IFC_SCOPE_REG_69							0x69


/**
 * @brief Constants to select backplane lines - DEPRECATED
 */
#define IFCDAQDRV_BACKPLANE_RX17    0x01
#define IFCDAQDRV_BACKPLANE_RX18    0x02
#define IFCDAQDRV_BACKPLANE_RX19    0x04
#define IFCDAQDRV_BACKPLANE_RX20    0x08
#define IFCDAQDRV_BACKPLANE_TX17    0x10
#define IFCDAQDRV_BACKPLANE_TX18    0x20
#define IFCDAQDRV_BACKPLANE_TX19    0x40
#define IFCDAQDRV_BACKPLANE_TX20    0x80
#define IFCDAQDRV_BACKPLANE_ALL     0xFF


ifcdaqdrv_status scope4ch_register(struct ifcdaqdrv_dev *ifcdevice);

#if 0
ifcdaqdrv_status scope4ch_write_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data);
ifcdaqdrv_status scope4ch_read_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data);
#endif

ifcdaqdrv_status scope4ch_disarm_acquisition(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status scope4ch_wait_acq_end(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status scope4ch_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge);
ifcdaqdrv_status scope4ch_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);

ifcdaqdrv_status scope4ch_enable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines);
ifcdaqdrv_status scope4ch_disable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines); 
ifcdaqdrv_status scope4ch_read_backplane_trgcnt(struct ifcdaqdrv_dev *ifcdevice, uint32_t *trig_cnt); 
ifcdaqdrv_status scope4ch_read_acq_count(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acq_cnt); 
ifcdaqdrv_status scope4ch_ack_acquisition(struct ifcdaqdrv_dev *ifcdevice); 
ifcdaqdrv_status scope4ch_arm_acquisition(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status scope4ch_generate_trigger(struct ifcdaqdrv_dev *ifcdevice); 
ifcdaqdrv_status scope4ch_read_scopestatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t *scopest); 
ifcdaqdrv_status scope4ch_read_acqdone(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acqdone); 

ifcdaqdrv_status scope4ch_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);
ifcdaqdrv_status scope4ch_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max);

ifcdaqdrv_status scope4ch_read_allchannels(struct ifcdaqdrv_dev *ifcdevice, void *data);
ifcdaqdrv_status scope4ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);
ifcdaqdrv_status scope4ch_normalize_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm);


#endif // _IFC_SCOPE4CH_H_


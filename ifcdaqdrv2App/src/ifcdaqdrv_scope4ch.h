#ifndef _IFC_SCOPE4CH_H_
#define _IFC_SCOPE4CH_H_ 1

#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"

#define SCOPE4CH_MAX_SAMPLES 122880 // Limited to 120k samples (120*1024)

ifcdaqdrv_status scope4ch_register(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status scope4ch_write_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data);
ifcdaqdrv_status scope4ch_read_generic(struct ifcdaqdrv_dev *ifcdevice, int function, void *data);

ifcdaqdrv_status scope4ch_enable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines);
ifcdaqdrv_status scope4ch_disable_backplane(struct ifcdaqdrv_dev *ifcdevice, uint32_t backplane_lines); 
ifcdaqdrv_status scope4ch_read_backplane_trgcnt(struct ifcdaqdrv_dev *ifcdevice, uint32_t *trig_cnt); 
ifcdaqdrv_status scope4ch_read_acq_count(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acq_cnt); 
ifcdaqdrv_status scope4ch_ack_acquisition(struct ifcdaqdrv_dev *ifcdevice); 
ifcdaqdrv_status scope4ch_arm_acquisition(struct ifcdaqdrv_dev *ifcdevice); 
ifcdaqdrv_status scope4ch_generate_trigger(struct ifcdaqdrv_dev *ifcdevice); 
ifcdaqdrv_status scope4ch_read_scopestatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t *scopest); 
ifcdaqdrv_status scope4ch_read_acqdone(struct ifcdaqdrv_dev *ifcdevice, uint32_t *acqdone); 

ifcdaqdrv_status scope4ch_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);
ifcdaqdrv_status scope4ch_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max);

ifcdaqdrv_status scope4ch_read_allchannels(struct ifcdaqdrv_dev *ifcdevice, void *data);
ifcdaqdrv_status scope4ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);
ifcdaqdrv_status scope4ch_normalize_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset, size_t nelm);


#endif // _IFC_SCOPE4CH_H_


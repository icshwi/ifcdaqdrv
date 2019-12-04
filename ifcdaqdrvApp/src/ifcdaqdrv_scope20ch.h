#ifndef _IFC_SCOPE20CH_H_
#define _IFC_SCOPE20CH_H_ 1

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"

#define IFC1410SCOPEDRV_SCOPE_LITE_20CHANNELS 0x73571720

#define SCOPE20CH_MAX_SAMPLES 2048 /* Hardcoded limitation */ 

ifcdaqdrv_status scope20ch_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status scope20ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);
ifcdaqdrv_status scope20ch_read_memory(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *dma_buf, uint32_t offset, uint32_t size);

#endif // _IFC_SCOPE20CH_H_


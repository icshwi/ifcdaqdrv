#ifndef _IFC_SCOPE20CH_H_
#define _IFC_SCOPE20CH_H_ 1

#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
#include "ifcdaqdrv_scope.h"

#define SCOPE20CH_MAX_SAMPLES 2048 /* Hardcoded limitation */ 

ifcdaqdrv_status scope20ch_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status scope20ch_read_ai_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data);

#endif // _IFC_SCOPE20CH_H_


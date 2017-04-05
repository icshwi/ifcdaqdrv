// #include <stdlib.h>
// #include <string.h>
// #include <stdint.h>
// #include <unistd.h>
// #include <libudev.h>
// #include <fcntl.h>
// #include <sys/ioctl.h>
// #include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>


// #include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_sim.h"

static const uint32_t decimations[] = {1, 2, 5, 10, 20, 50, 100, 200, 0};
static const uint32_t averages[] = {1, 4, 8, 16, 32, 64, 128, 256, 0};
static const double   valid_clocks[] = {2400e6, 2500e6, 0};

ifcdaqdrv_status ADCSim_register(struct ifcdaqdrv_dev *ifcdevice) {
    uint32_t nsamples_max;
    int i;
    
    ifcdevice->mode        = ifcdaqdrv_acq_mode_sram;
    ifcdevice->sample_size = 2;
    ifcdevice->nchannels   = 8;

    memcpy(ifcdevice->decimations,  decimations,  sizeof(decimations));
    memcpy(ifcdevice->averages,     averages,     sizeof(averages));

    ifcdevice->resolution  = 16;
    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 125; //1045;
    ifcdevice->divisor_min = 8; //1;

    ifcdevice->sample_resolution = 16;
    ifcdevice->vref_max = 1;

    ifcdevice->armed       = 0;
    ifcdevice->poll_period = 1000;

    nsamples_max = 16 * 1024;

    ifcdevice->sram_size = nsamples_max * ifcdevice->sample_size;
    
    //ifcdevice->smem_size = 256 * 1024 * 1024;
    ifcdevice->smem_size = 1 * 1024 * 1024;

    /* The subsystem lock is used to serialize access to the serial interface
     * since it requires several write/read pci accesses */
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);

    /* Runtime data initalization */
    ifcdevice->rt_clock_source = ifcdaqdrv_clock_internal;

    for (i=0; i<8; i++)
    {
    	ifcdevice->rt_channel_gain[i] = 1.0;
    	ifcdevice->rt_ch_pattern[i] = ifcdaqdrv_pattern_none;
    }

    ifcdevice->rt_dataformat = ifcdaqdrv_dataformat_unsigned;
    ifcdevice->rt_clock_frequency = 2500e6;
    ifcdevice->rt_clock_divisor = 1;
    ifcdevice->rt_tr_trigger = ifcdaqdrv_trigger_soft;
    ifcdevice->rt_tr_threshold = 0;
    ifcdevice->rt_tr_mask = 0;
    ifcdevice->rt_tr_edge = 0;
    ifcdevice->rt_average = 1;
    ifcdevice->rt_decimation = 1;
    ifcdevice->rt_nsamples = 16*1024;
    ifcdevice->rt_npretrig = 0;

    ifcdevice->databuffer = NULL;
    ifcdevice->buffersize = 0;

    return status_success;
}

void ifcdaqdrv_free(struct ifcdaqdrv_dev *ifcdevice){
    // // TODO finish free implementaiton
    // if(ifcdevice->all_ch_buf) {
    //     free(ifcdevice->all_ch_buf);
    //     ifcdevice->all_ch_buf = NULL;
    // }

    // if(ifcdevice->smem_dma_buf) {
    //     pevx_buf_free(ifcdevice->card, ifcdevice->smem_dma_buf);
    //     free(ifcdevice->smem_dma_buf);
    //     ifcdevice->smem_dma_buf = NULL;
    // }

    // if(ifcdevice->sram_dma_buf){
    //     pevx_buf_free(ifcdevice->card, ifcdevice->sram_dma_buf);
    //     free(ifcdevice->sram_dma_buf);
    //     ifcdevice->sram_dma_buf = NULL;
    // }

    // if(ifcdevice->fru_id) {
    //     if(ifcdevice->fru_id->product_name) {
    //         free(ifcdevice->fru_id->product_name);
    //     }
    //     if(ifcdevice->fru_id->manufacturer) {
    //         free(ifcdevice->fru_id->manufacturer);
    //     }
    //     free(ifcdevice->fru_id);
    //     ifcdevice->fru_id = NULL;
    // }

    if (ifcdevice->databuffer)
    {
    	free(ifcdevice->databuffer);
    	ifcdevice->databuffer = NULL;
    	ifcdevice->buffersize = 0;
    }

}

ifcdaqdrv_status ifcdaqdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice) {
//     void *p;

    ifcdevice->databuffer = calloc(4*1024*1024, sizeof(int32_t));
    if (!ifcdevice->databuffer) {
        return status_internal;
    }

    ifcdevice->buffersize = 4*1024*1024;


//     ifcdevice->sram_dma_buf->size = ifcdevice->sram_size;

//     TRACE((5, "Trying to allocate %dkiB in kernel\n", ifcdevice->sram_size / 1024));
//     if (pevx_buf_alloc(ifcdevice->card, ifcdevice->sram_dma_buf) == NULL) {
//         goto err_sram_buf;
//     }

//     ifcdevice->smem_dma_buf = calloc(1, sizeof(struct pev_ioctl_buf));
//     if (!ifcdevice->smem_dma_buf) {
//         goto err_smem_ctl;
//     }

//     // Try to allocate as large dma memory as possible
//     ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
//     do {
//         TRACE((5, "Trying to allocate %dMiB in kernel\n", ifcdevice->smem_dma_buf->size / 1024 / 1024));
//         p = pevx_buf_alloc(ifcdevice->card, ifcdevice->smem_dma_buf);
//     } while (p == NULL && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

//     if(!p) {
//         goto err_smem_buf;
//     }

//     TRACE((5, "Trying to allocate %dMiB in userspace\n", ifcdevice->smem_size / 1024 / 1024));
//     ifcdevice->all_ch_buf = calloc(ifcdevice->smem_size, 1);
//     if(!ifcdevice->all_ch_buf){
//         goto err_smem_user_buf;
//     }

//     return status_success;

// err_smem_user_buf:
//     pevx_buf_free(ifcdevice->card, ifcdevice->smem_dma_buf);

// err_smem_buf:
//     free(ifcdevice->smem_dma_buf);

// err_smem_ctl:
//     pevx_buf_free(ifcdevice->card, ifcdevice->sram_dma_buf);

// err_sram_buf:
//     free(ifcdevice->sram_dma_buf);

// err_sram_ctl:
//     return status_internal;

	return 0;
}



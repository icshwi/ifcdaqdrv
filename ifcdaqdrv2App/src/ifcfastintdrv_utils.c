#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

//#include <pevxulib.h>
#include "tscioctl.h"
#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_adc3117.h"
#include "ifcfastintdrv_utils.h"

static const double   valid_clocks[] = {5e6, 0};

static inline int32_t swap_mask(struct ifcdaqdrv_dev *ifcdevice) {
    switch (ifcdevice->sample_size) {
    case 2:
        return DMA_SPACE_WS;
    case 4:
        return DMA_SPACE_DS;
    case 8:
        return DMA_SPACE_QS;
    }
    return 0;
}

// static inline void INIT_SECONDARY(struct ifcdaqdrv_dev *ifcdevice, struct ifcdaqdrv_dev *secondary) {
//     secondary->card = ifcdevice->card;
//     secondary->fmc = 2;
//     secondary->node = ifcdevice->node;
//     pthread_mutex_init(&secondary->sub_lock, NULL);
// }

/* Since the IFC Fast Interlock uses both FMCs we initialize both */
ifcdaqdrv_status ifcfastintdrv_init_adc(struct ifcdaqdrv_dev *ifcdevice) {
    // Initialize the ADCs, this will
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);
    adc3117_init_adc(ifcdevice);
 
    /* No secundary FMC is needed */
    // struct ifcdaqdrv_dev secondary;
    // INIT_SECONDARY(ifcdevice, &secondary);
    // adc3110_init_adc(&secondary);

    int32_t i32regval;

    /* Set correct pins on P2 to IN */
    //ifc_xuser_tcsr_write(ifcdevice, 0x0f, 0);
    ifc_xuser_tcsr_write(ifcdevice, 0x06, 0);
    ifc_xuser_tcsr_write(ifcdevice, 0x09, 0);
    ifc_xuser_tcsr_write(ifcdevice, 0x0f, 1<<31);
    //ifc_xuser_tcsr_write(ifcdevice, 0x04,
    //    1<<7 | 1<<8 | 1<<10 | 1<<11 | 1<<12 | 1<<14 | 1<<15 | 1<<16);
    //ifc_xuser_tcsr_write(ifcdevice, 0x07,
    //    1<<7 | 1<<8 | 1<<10 | 1<<11 | 1<<12 | 1<<14 | 1<<15 | 1<<16);
    
    // Initialize V6-S6 Link
    ifc_xuser_tcsr_write(ifcdevice, 0x24, 0);
    usleep(10*1000);
    ifc_xuser_tcsr_write(ifcdevice, 0x24, 1);
    usleep(10*1000);
    ifc_xuser_tcsr_write(ifcdevice, 0x24, 2);
    usleep(10*1000);
    ifc_xuser_tcsr_write(ifcdevice, 0x24, 3);
    usleep(10*1000);

    /* Check if V6-S6 is operating OK */
    ifc_xuser_tcsr_read(ifcdevice, 0x24, &i32regval);
    if ((i32regval & 0xF000C00F) != (0xC0008003))
    	LOG((LEVEL_ERROR, "Register 0x24 = 0x%8x", i32regval));

    /* Reset and set DIRECT mode of FMC support module */
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x01010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x02020202);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x01010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x02020202);

    /* Clear ISERDES (should be 0x00010001) */
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x80010025);
    //ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x00010001);
    usleep(100000);
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x00000000);

    return status_success;
}

void ifcfastintdrv_history_reset(struct ifcdaqdrv_dev *ifcdevice) {

        /* Turn off history to clear history errors */
        ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0, IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK);

        /* Turn on history buffer acq, keeping the actual HISTORY MODE */
        ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK, 0);

}

/* Debug function that prints a human readable string of the status register */
void ifcfastint_print_status(int32_t reg) {
        int32_t tmp;
        int32_t i;

        printf("Status FSM: %08x, ", reg);

        tmp = (reg & IFCFASTINT_FSM_MAN_FSM_CMD_MASK) >> IFCFASTINT_FSM_MAN_FSM_CMD_SHIFT;
        switch(tmp) {
        case 0:
                printf("CMD: No action");
                break;
        }
        printf(", ");
        printf("State: ");
        tmp = (reg & IFCFASTINT_FSM_MAN_FSM_STA_MASK) >> IFCFASTINT_FSM_MAN_FSM_STA_SHIFT;
        switch(tmp) {
        case 0:
                printf("IDLE");
                break;
        case 1:
                printf("ABO");
                break;
        case 2:
                printf("PRE");
                break;
        case 3:
                printf("RUN");
                break;
        }
        printf(", ");
        tmp = (reg & IFCFASTINT_FSM_MAN_FSM_OUT_MASK) >> IFCFASTINT_FSM_MAN_FSM_OUT_SHIFT;
        printf("OUT: ");
        for(i=0; i<4; ++i) {
                if(tmp & 1<<i) {
                        printf("1");
                } else {
                        printf("0");
                }
        }
        printf(", ");
        printf("Dynamic Analog: ");
        if(reg & IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_MASK) {
                printf("On");
        } else {
                printf("Off");
        }
        printf(", ");
        printf("Dynamic Digital: ");
        if(reg & IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_MASK) {
                printf("On");
        } else {
                printf("Off");
        }
        printf(", ");
        tmp = (reg & IFCFASTINT_FSM_MAN_FSM_FRQ_MASK) >> IFCFASTINT_FSM_MAN_FSM_FRQ_SHIFT;
        switch(tmp) {
        case 0:
                printf("Disabled");
                break;
        case 1:
                printf("1MHz");
                break;
        case 2:
                printf("500kHz");
                break;
        case 3:
                printf("200kHz");
                break;
        }
        printf(", ");
        printf("History EN: ");
        if(reg & IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK) {
                printf("On");
        } else {
                printf("Off");
        }
        printf(", ");
        tmp = (reg & IFCFASTINT_FSM_MAN_HISTORY_MODE_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_MODE_SHIFT;
        printf("History Mode: ");
        switch(tmp) {
        case 0:
                printf("RUN");
                break;
        case 1:
                printf("PRE+RUN");
                break;
        case 2:
                printf("IDLE+PRE+RUN");
                break;
        default:
                printf("Reserved");
                break;
        }
        printf(", ");
        printf("History SMEM error: ");
        if(reg & IFCFASTINT_FSM_MAN_HISTORY_OVER_FF_MASK) {
                printf("Set");
        } else {
                printf("Clear");
        }
        printf(", ");
        printf("History Ring overflow: ");
        if(reg & IFCFASTINT_FSM_MAN_HISTORY_RING_OVER_FF_MASK) {
                printf("Set");
        } else {
                printf("Clear");
        }
        printf(", ");
        tmp = (reg & IFCFASTINT_FSM_MAN_HISTORY_STATUS_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_STATUS_SHIFT;
        printf("History Status: ");
        switch(tmp) {
        case 0:
                printf("IDLE");
                break;
        case 1:
                printf("Running");
                break;
        case 2:
                printf("Filling");
                break;
        case 3:
                printf("Ended");
                break;
        }

        printf("\n");
}

/* Register ADC3117 FMC */

ifcdaqdrv_status ifcfastintdrv_register(struct ifcdaqdrv_dev *ifcdevice){
    
    /* Activate FMC */
    ifc_fmc_tcsr_write(ifcdevice, 0, 0x31170000);
    usleep(900000);

    ifcdevice->init_adc = ifcfastintdrv_init_adc;
    ifcdevice->nchannels = 20;
    ifcdevice->smem_size = 256<<20; //256 MB
    ifcdevice->vref_max = 10.24;
    ifcdevice->sample_resolution = 16;
    ifcdevice->sample_size = 2; // Important for endianess
    ifcdevice->poll_period = 1000;

    /* Functions to configure clock */
    ifcdevice->set_clock_frequency   = adc3117_set_clock_frequency;
    ifcdevice->get_clock_frequency   = adc3117_get_clock_frequency;
    ifcdevice->set_clock_source      = adc3117_set_clock_source;
    ifcdevice->get_clock_source      = adc3117_get_clock_source;
    ifcdevice->set_clock_divisor     = adc3117_set_clock_divisor;
    ifcdevice->get_clock_divisor     = adc3117_get_clock_divisor;

    memcpy(ifcdevice->valid_clocks, valid_clocks, sizeof(valid_clocks));
    ifcdevice->divisor_max = 1; //1045;
    ifcdevice->divisor_min = 1; //1;

    return status_success;
}

#define PP_OFFSET 0x100000

ifcdaqdrv_status ifcfastintdrv_read_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *pp_options) {
    ifcdaqdrv_status status;
    

/********************* Read TMEM using DMA engine - ccurrently not working  ********************************/
#if 0
    struct tsc_ioctl_dma_req dma_req = {0};
    uint32_t valid_dma_status;

    struct tsc_ioctl_dma_sts dma_sts;
    struct tsc_ioctl_kbuf_req *dma_buf;

    /* workaround */
#ifndef DMA_SPACE_USR1
    #define DMA_SPACE_USR1 0x04
#endif

#ifndef DMA_SPACE_USR2
    #define DMA_SPACE_USR2 0x05
#endif

    dma_buf = ifcdevice->smem_dma_buf;

    dma_req.src_addr = (dma_addr_t) PP_OFFSET + addr;
    dma_req.src_space = DMA_SPACE_USR1;
    dma_req.src_mode = DMA_PCIE_RR2;

    dma_req.des_addr = dma_buf->b_base;
    //dma_req.des_space = DMA_SPACE_PCIE | swap_mask(ifcdevice);
    dma_req.des_space = DMA_SPACE_PCIE | DMA_SPACE_DS;
    dma_req.des_mode = DMA_PCIE_RR2;

    dma_req.end_mode   = 0;
    dma_req.start_mode = DMA_START_CHAN(0);
    dma_req.intr_mode  = DMA_INTR_ENA;
    dma_req.wait_mode  = 0;

    dma_req.size = 4096 | DMA_SIZE_PKT_128;

    status = tsc_dma_alloc(0);
    if (status) 
    {
      LOG((LEVEL_WARNING, "%s() tsc_dma_alloc(0) == %d \n", __FUNCTION__, status));
      return status;
    }

    dma_sts.dma.chan = 0;
    status = tsc_dma_status(&dma_sts);
    if (status) 
    {
      LOG((LEVEL_ERROR, "%s() tsc_dma_status() == %d \n", __FUNCTION__, status));
      return status;
    }

    status = tsc_dma_move(&dma_req);
    if (status != 0) {
        LOG((LEVEL_WARNING, "%s() tsc_dma_move() == %d status = 0x%08x\n", __FUNCTION__, status, dma_req.dma_status));
        return status_read;
    }

    dma_sts.dma.chan = 0;
    status = tsc_dma_status(&dma_sts);
    
    if (status) 
    {
      LOG((LEVEL_WARNING, "%s() tsc_dma_status() == %d \n", __FUNCTION__, status));
      return status;
    }

    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4); // Timeout after 50 ms
    status = tsc_dma_wait(&dma_req);

    if (status) {
        LOG((LEVEL_ERROR, "%s() tsc_dma_wait() returned %d\n", __FUNCTION__, status));
    }
    
    // * 0x2 << 28 is the interrupt number.
    // * The board can only read from the shared memory. If we are not reading
    //   from shared memory the "WR0" will run because the DMA engine first has
    //   to write the data into the shared memory.
    valid_dma_status = (0x2 << 28) | DMA_STATUS_DONE | DMA_STATUS_ENDED | DMA_STATUS_RUN_RD0;
    if(!(dma_req.src_space & DMA_SPACE_SHM)) {
        valid_dma_status |= DMA_STATUS_RUN_WR0;
    }

    if (dma_req.dma_status != valid_dma_status) {
        LOG((LEVEL_ERROR, "Error: %s() DMA error 0x%08x\n", __FUNCTION__, dma_req.dma_status));       
        tsc_dma_free(0);
        return status_read;
    }
    
    /*free DMA channel 0 */
    status = tsc_dma_free(0);
    if (status) 
    {
      LOG((LEVEL_ERROR, "%s() tsc_dma_free() == %d\n", __FUNCTION__, status));
      return status_read;
    }

    /* Copy from kernel space (dma_buf) to userspace (pp_options)
     * TODO: check for errors and fill buffer with zeros??
     */
    memcpy((void *)pp_options, dma_buf->u_base, sizeof(*pp_options));
#endif

    /***************************************** Legacy code from VME ***************************************/
#if 0
    struct tsc_ioctl_dma_req dma_req = {0};
    struct tsc_ioctl_kbuf_req dma_buf  = {0};

    dma_buf.size = sizeof(*pp_options);

    pevx_buf_alloc(ifcdevice->card, &dma_buf);

    /* PEV Userlib DMA operation call */
    dma_req.src_addr = PP_OFFSET + addr;
    dma_req.src_space = DMA_SPACE_USR1;
    dma_req.src_mode = DMA_PCIE_RR2;

    dma_req.des_addr = (long)dma_buf.b_addr;
    //dma_req.des_space = DMA_SPACE_PCIE | swap_mask(ifcdevice);
    dma_req.des_space = DMA_SPACE_PCIE | DMA_SPACE_QS;
    dma_req.des_mode = DMA_PCIE_RR2;

    dma_req.start_mode = DMA_MODE_PIPE;
    dma_req.intr_mode = DMA_INTR_ENA;
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_10MS | (5 << 4); // Timeout after 50 ms

    dma_req.size = dma_buf.size | DMA_SIZE_PKT_128;

    status = pevx_dma_move(ifcdevice->card, &dma_req);

    /* Copy from kernel space (dma_buf) to userspace (pp_options)
     * TODO: check for errors and fill buffer with zeros??
     */
    memcpy((void *)pp_options, dma_buf.u_addr, sizeof(*pp_options));

    pevx_buf_free(ifcdevice->card, &dma_buf);

    LOG((LEVEL_DEBUG, " addr: 0x%08x, r: 0x%016" PRIx64 ", len:%d\n", PP_OFFSET + addr, *pp_options, dma_buf.size));

    if(status != 0) {
        LOG((LEVEL_ERROR, "dma_status %08x\n", status));
        return status_write;
    }
#endif

/********************* Reading TMEM memory space using CPU copy *******************************************/

    int blkvar = RDWR_MODE_SET( 0x44, RDWR_SPACE_USR1, 0);
    void *mybuffer = calloc(1024*1024,1);

    ulong src_addr = PP_OFFSET + addr;

    usleep(2000);
    status = tsc_read_blk(src_addr, (char*) mybuffer, sizeof(*pp_options), blkvar);

    if (status) {
        LOG((LEVEL_ERROR,"tsc_blk_read() returned %d\n", status));
        free(mybuffer);
        return status;
    }

    memcpy((void *)pp_options, mybuffer, sizeof(*pp_options));
    free(mybuffer);

/********************************************************************************************************/

    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_write_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t pp_options) {
    ifcdaqdrv_status status;

/******************************** Legacy code from VME *************************************************/
#if 0
    struct tsc_ioctl_dma_req dma_req = {0};
    struct tsc_ioctl_kbuf_req dma_buf  = {0};

    dma_buf.size = sizeof(pp_options);

    pevx_buf_alloc(ifcdevice->card, &dma_buf);

    /* Copy from userspace to kernel space (dma_buf) */
    memcpy(dma_buf.u_addr, (void *)&pp_options, sizeof(pp_options));

    dma_req.src_addr = (long)dma_buf.b_addr;
    //dma_req.src_space = DMA_SPACE_PCIE | swap_mask(ifcdevice);
    dma_req.src_space = DMA_SPACE_PCIE | DMA_SPACE_QS;
    dma_req.src_mode = DMA_PCIE_RR2;

    dma_req.des_addr = PP_OFFSET + addr;
    dma_req.des_space = DMA_SPACE_USR1;
    dma_req.des_mode = DMA_PCIE_RR2;

    dma_req.start_mode = DMA_MODE_PIPE;
    dma_req.intr_mode = DMA_INTR_ENA;
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_10MS | (5 << 4); // Timeout after 50 ms

    dma_req.size = dma_buf.size | DMA_SIZE_PKT_128;

    status = pevx_dma_move(ifcdevice->card, &dma_req);

    pevx_buf_free(ifcdevice->card, &dma_buf);

    LOG((LEVEL_DEBUG, "addr: 0x%08x, w: 0x%016" PRIx64 "\n", PP_OFFSET + addr, pp_options));

    if(status != 0) {
        LOG((LEVEL_ERROR, "dma_status %08x\n", status));
        return status_write;
    }
#endif

/*************** Write TMEM address space using CPU copy  ****************************************************/
    void *mybuffer = calloc(1024*1024,1);
    memcpy(mybuffer, (void *)&pp_options, sizeof(pp_options));

    ulong dest_addr = PP_OFFSET + addr;
    int blkvar = RDWR_MODE_SET( 0x44, RDWR_SPACE_USR1, 0);

    usleep(2000);
    status = tsc_write_blk(dest_addr, (char*) mybuffer, sizeof(pp_options), blkvar);
    
    free(mybuffer);

    if (status) {
        LOG((LEVEL_ERROR,"tsc_blk_write() returned %d\n", status));
        return status;
    }
    
/**************************************************************************************************************/

    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice) {
    //void *p;
    int ret;

/************** Currently there is no need for kernel allocation with TSC ***************************/

#if 1    
    ifcdevice->smem_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->smem_dma_buf) {
        return status_internal;
    }

    // Try to allocate as large dma memory as possible
    ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
    do {
        LOG((LEVEL_INFO, "Trying to allocate %d bytes in kernel\n", ifcdevice->smem_dma_buf->size));
        ret = tsc_kbuf_alloc(ifcdevice->smem_dma_buf);
    } while ((ret < 0) && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

    if(ret) {
        free(ifcdevice->smem_dma_buf);
        return status_internal;
    }

#endif

#if 0
    LOG((5, "Trying to mmap %dkiB in kernel for SMEM acquisition\n", ifcdevice->smem_dma_buf->size / 1024));
    if (tsc_kbuf_mmap(ifcdevice->smem_dma_buf) < 0)  {
        //goto err_mmap_smem;
        fprintf(stderr, "ERROR: tsc_kbuf_mmap(ifcdevice->smem_dma_buf) failed\n");
    }
    LOG((LEVEL_INFO, "Successfully allocated space in kernel! [%d bytes]\n",ifcdevice->smem_dma_buf->size));
#endif

#if 1
    
    /* Temporary solution to allocate memory for use with tsc_blk calls */
    LOG((5, "Trying to allocate 1 MiB in userspace for tsc_read_blk() calls\n"));
    ifcdevice->sram_blk_buf = calloc(1024*1024, 1);
    if(!ifcdevice->sram_blk_buf){
        fprintf(stderr, "calloc for the sram_blk_buf() failed\n");
        return status_internal;
    }

#endif
    
    return status_success;
}

inline uint64_t u64_setclr(uint64_t input, uint64_t bits, uint64_t mask, uint32_t offset) {
    uint64_t output = input & ~(mask << offset); // Clear mask
    output |= bits << offset; // Set bits
    return output;
}

ifcdaqdrv_status ifcfastintdrv_printregister(uint64_t *pp_register) {
    uint64_t tmp;
    uint32_t value;

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPACT_MASK) >> IFCFASTINT_ANALOGPP_REG_PPACT_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_ACT  =   %d\n", value);

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPEMU_MASK) >> IFCFASTINT_ANALOGPP_REG_PPEMU_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_EMUL =   %d\n", value);

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPMODE_MASK) >> IFCFASTINT_ANALOGPP_REG_PPMODE_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_MODE = 0x%04x\n", value);

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPVAL1_MASK) >> IFCFASTINT_ANALOGPP_REG_PPVAL1_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_VAL1 = 0x%08x\n", value);

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPVAL2_MASK) >> IFCFASTINT_ANALOGPP_REG_PPVAL2_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_VAL2 = 0x%08x\n", value);

    tmp = (*pp_register & IFCFASTINT_ANALOGPP_REG_PPCVAL_MASK) >> IFCFASTINT_ANALOGPP_REG_PPCVAL_SHIFT;
    value = (uint32_t) tmp;
    printf("pp_option_CVAL = 0x%08x\n", value);

    printf("\n");
    return status_success;
}

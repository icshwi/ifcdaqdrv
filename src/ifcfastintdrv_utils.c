#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
/* Oliver adc3110 -> adc3117 */
#include "ifcdaqdrv_adc3110.h"
#include "ifcfastintdrv_utils.h"

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

static inline void INIT_SECONDARY(struct ifcdaqdrv_dev *ifcdevice, struct ifcdaqdrv_dev *secondary) {
    secondary->card = ifcdevice->card;
    secondary->fmc = 2;
    secondary->node = ifcdevice->node;
    pthread_mutex_init(&secondary->sub_lock, NULL);
}

/* Oliver adc3110 -> adc3117 */
/* Since the IFC Fast Interlock uses both FMCs we initialize both */
ifcdaqdrv_status ifcfastintdrv_init_adc(struct ifcdaqdrv_dev *ifcdevice) {
    // Initialize the ADCs, this will
    pthread_mutex_init(&ifcdevice->sub_lock, NULL);
    adc3110_init_adc(ifcdevice);
    struct ifcdaqdrv_dev secondary;
    INIT_SECONDARY(ifcdevice, &secondary);
    adc3110_init_adc(&secondary);

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


    /* Reset and set DIRECT mode of FMC support module */
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x01010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x02020202);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x01010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x02020202);

    /* Clear ISERDES (should be 0x00010001) */
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x80010025);
    usleep(100000);
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x00000000);

    return status_success;
}

void ifcfastintdrv_history_reset(struct ifcdaqdrv_dev *ifcdevice) {
        int32_t i32_reg_val;
        /* Turno off history. Clear history errors */
        ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0, IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK);
        /* Init history */
        i32_reg_val =
                1 << IFCFASTINT_FSM_MAN_HISTORY_ENA_SHIFT |
                6 << IFCFASTINT_FSM_MAN_HISTORY_MODE_SHIFT;

        ifc_xuser_tcsr_setclr(
                ifcdevice,
                IFCFASTINT_FSM_MAN_REG,
                i32_reg_val,
                IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK |
                  IFCFASTINT_FSM_MAN_HISTORY_MODE_MASK);
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

ifcdaqdrv_status ifcfastintdrv_register(struct ifcdaqdrv_dev *ifcdevice){
    ifcdevice->init_adc = ifcfastintdrv_init_adc;
    ifcdevice->nchannels = 8;
    ifcdevice->smem_size = 256<<20; //256 MB
    ifcdevice->smem_size = 1<<19; //256 MB
    ifcdevice->vref_max = 0.5;
    ifcdevice->sample_resolution = 16;
    ifcdevice->sample_size = 2; // Important for endianess
    ifcdevice->poll_period = 1000;
    return status_success;
}

#define PP_OFFSET 0x100000

ifcdaqdrv_status ifcfastintdrv_read_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *pp_options) {
    ifcdaqdrv_status status;
    struct tsc_ioctl_dma_req dma_req = {0};
    struct tsc_ioctl_kbuf_req dma_buf  = {0};

    dma_buf.size = sizeof(*pp_options);

    tsc_kbuf_alloc(&dma_buf);

    dma_req.src_addr = PP_OFFSET + addr;
    dma_req.src_space = DMA_SPACE_USR;
    dma_req.src_mode = DMA_PCIE_RR2;

    dma_req.des_addr = (long)dma_buf.b_base;
    //dma_req.des_space = DMA_SPACE_PCIE | swap_mask(ifcdevice);
    dma_req.des_space = DMA_SPACE_PCIE | DMA_SPACE_QS;
    dma_req.des_mode = DMA_PCIE_RR2;

    dma_req.start_mode = DMA_START_CHAN(0);
    dma_req.intr_mode = DMA_INTR_ENA;
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_10MS | (5 << 4); // Timeout after 50 ms

    dma_req.size = dma_buf.size | DMA_SIZE_PKT_128;

    status = tsc_dma_move(&dma_req);

    memcpy((void *)pp_options, dma_buf.u_base, sizeof(*pp_options));

    tsc_kbuf_free(&dma_buf);

    LOG((LEVEL_DEBUG, " addr: 0x%08x, r: 0x%016" PRIx64 ", len:%d\n", PP_OFFSET + addr, *pp_options, dma_buf.size));

    if(status != 0) {
        LOG((LEVEL_ERROR, "dma_status %08x\n", status));
        return status_write;
    }

    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_write_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t pp_options) {
    ifcdaqdrv_status status;
    struct tsc_ioctl_dma_req dma_req = {0};
    struct tsc_ioctl_kbuf_req dma_buf  = {0};

    dma_buf.size = sizeof(pp_options);

    tsc_kbuf_alloc(&dma_buf);

    memcpy(dma_buf.u_base, (void *)&pp_options, sizeof(pp_options));

    dma_req.src_addr = (long)dma_buf.b_base;
    //dma_req.src_space = DMA_SPACE_PCIE | swap_mask(ifcdevice);
    dma_req.src_space = DMA_SPACE_PCIE | DMA_SPACE_QS;
    dma_req.src_mode = DMA_PCIE_RR2;

    dma_req.des_addr = PP_OFFSET + addr;
    dma_req.des_space = DMA_SPACE_USR;
    dma_req.des_mode = DMA_PCIE_RR2;

    dma_req.start_mode = DMA_START_CHAN(0);
    dma_req.intr_mode = DMA_INTR_ENA;
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_10MS | (5 << 4); // Timeout after 50 ms

    dma_req.size = dma_buf.size | DMA_SIZE_PKT_128;

    status = tsc_dma_move(&dma_req);

    tsc_kbuf_free(&dma_buf);

    LOG((LEVEL_DEBUG, "addr: 0x%08x, w: 0x%016" PRIx64 "\n", PP_OFFSET + addr, pp_options));

    if(status != 0) {
        LOG((LEVEL_ERROR, "dma_status %08x\n", status));
        return status_write;
    }
    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice) {
    int ret;
    ifcdevice->smem_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->smem_dma_buf) {
        return status_internal;
    }

    // Try to allocate as large dma memory as possible
    ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
    do {
        LOG((LEVEL_INFO, "Trying to allocate %dMiB in kernel\n", ifcdevice->smem_dma_buf->size / 1024 / 1024));
        ret = tsc_kbuf_alloc(ifcdevice->smem_dma_buf);
    } while ((ret < 0) && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

    if(ret) {
        free(ifcdevice->smem_dma_buf);
        return status_internal;
    }

    return status_success;
}

inline uint64_t u64_setclr(uint64_t input, uint64_t bits, uint64_t mask, uint32_t offset) {
    uint64_t output = input & ~(mask << offset); // Clear mask
    output |= bits << offset; // Set bits
    return output;
}


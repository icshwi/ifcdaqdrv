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

static inline ifcdaqdrv_status ifcfastintdrv_alloc_tscbuf(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *kbuf_req);
static inline ifcdaqdrv_status ifcfastintdrv_free_tscbuf(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *kbuf_req);

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

/* 8-byte long word endianess conversion */
static inline void ifcdaqdrv_swap64(uint64_t* destination)
{
    uint64_t source = *destination;
    uint8_t *byte_ppo;
    uint8_t *byte_aux;

    byte_ppo = (uint8_t*) destination;
    byte_aux = (uint8_t*) &source;

    byte_ppo[0] = byte_aux[7];
    byte_ppo[1] = byte_aux[6];
    byte_ppo[2] = byte_aux[5];
    byte_ppo[3] = byte_aux[4];  
    byte_ppo[4] = byte_aux[3];
    byte_ppo[5] = byte_aux[2];
    byte_ppo[6] = byte_aux[1];
    byte_ppo[7] = byte_aux[0];
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
    ifcdaqdrv_status status;

    pthread_mutex_init(&ifcdevice->sub_lock, NULL);
    status = adc3117_init_adc(ifcdevice);
    if (status) 
        return status;
 
    /* Set ADC inputs to the front panel connector */
    status = adc3117_set_adc_channel_negative_input(ifcdevice, GND); // ADC channel negative input
    if (status)
        return status;

    status = adc3117_set_adc_channel_positive_input(ifcdevice, FROM_CONNECTOR); // ADC channel positive input
    //status = adc3117_set_adc_channel_positive_input(ifcdevice, VCAL); // ADC channel positive input
    if (status)
        return status;

    // status = ifc_fmc_tcsr_write(ifcdevice, 0x4, 0x30); //0x80=5V, 0x30=2V
    // if (status)
    //     return status;

    // status = ifc_fmc_tcsr_write(ifcdevice, 0x3, 0xCE000000);
    // if (status)
    //     return status;
    // usleep(1000);

    // status = ifc_fmc_tcsr_setclr(ifcdevice, ADC3117_GENERAL_CONFIG_REG, 0x0, 0x3); // VCAL = 0x1 Ref 4.128V, 0x0 var_ref
    // if (status)
    //     return status;

    status = adc3117_configuration_command(ifcdevice); // Send config
    if (status)
        return status;

    /* Legacy code from VME version */

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

#if 0 // ONLY VALID FOR VME VERSION
    /* Check if V6-S6 is operating OK */
    ifc_xuser_tcsr_read(ifcdevice, 0x24, &i32regval);
    if ((i32regval & 0xF000C00F) != (0xC0008003))
    {    
        LOG((LEVEL_ERROR, "Register 0x24 = 0x%08x\n", i32regval));
    }
#endif

    /* Reset and set DIRECT mode of FMC support module */
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x05010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x61, 0x0A020202);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x05010101);
    ifc_xuser_tcsr_write(ifcdevice, 0x62, 0x0A020202);

    /* Clear ISERDES (should be 0x00010001) */
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x80010025);
    //ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x00010001);
    usleep(100000);
    ifc_xuser_tcsr_write(ifcdevice, 0x63, 0x00000000);

    INFOLOG(("Successfully started ADC3117\n"));

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
    
    ifcdaqdrv_status status;
    status = adc3117_get_signature(ifcdevice, NULL, NULL, &ifcdevice->board_id);
    if (status)
        return status;

    /* Activate FMC */
    ifc_fmc_tcsr_write(ifcdevice, 0, 0x31170000);
    usleep(10000);

    ifcdevice->init_adc = ifcfastintdrv_init_adc;
    ifcdevice->nchannels = 16; //20 is not working !!!
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

ifcdaqdrv_status ifcfastintdrv_read_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *pp_options) {
    
    ifcdaqdrv_status status;
    
    /* Prepares the command word for CPU copy|read using tsclib (based on TscMon source code)*/
    int cmdword = RDWR_MODE_SET( 0x44, RDWR_SPACE_USR1, 0);
    void *mybuffer = calloc(1024*1024,1);

    /* if running on little endian machine the command word needs to be converted */
    if (!ifcdaqdrv_is_byte_order_ppc()) {
        cmdword = tsc_swap_32(cmdword);
    }

    /* address that holds the configuration PP_OPTION for the specific pp block */
    ulong src_addr = IFCFASTINT_SRAM_PP_OFFSET + addr;

    /* using tsclib to read what is in SRAM 1 area */
    usleep(1000);
    status = tsc_read_blk(ifcdevice->node, src_addr, (char*) mybuffer, sizeof(*pp_options), cmdword);

    if (status) {
        LOG((LEVEL_ERROR,"tsc_blk_read() returned %d\n", status));
        free(mybuffer);
        return status;
    }

    /* Copying 8 bytes to the 64 bit variable that will be decoded */
    memcpy((void *)pp_options, mybuffer, sizeof(*pp_options));

    /* if running on BIG ENDIAN machine the 8-byte stream needs to be swapped */
    if (ifcdaqdrv_is_byte_order_ppc()) {
    	ifcdaqdrv_swap64(pp_options);
    }
    free(mybuffer);

    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_write_pp_conf(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t pp_options) {
    ifcdaqdrv_status status;
    void *mybuffer = calloc(1024*1024,1);

    /* if running on BIG endian machine, the 8-byte long word needs to be converted to big endian */
    if (ifcdaqdrv_is_byte_order_ppc()) {
    	ifcdaqdrv_swap64(&pp_options);
    }
    memcpy(mybuffer, (void *)&pp_options, sizeof(pp_options));

    /* Sets the PP_OPTION addres of the specific pp block */
    ulong dest_addr = IFCFASTINT_SRAM_PP_OFFSET + addr;

    /* Prepares the command word for CPU copy using tsclib (based on TscMon source code)*/
    int cmdword = RDWR_MODE_SET( 0x44, RDWR_SPACE_USR1, 0);
    
    /* if running on little endian machine the command word needs to be converted */
    if (!ifcdaqdrv_is_byte_order_ppc()) {
        cmdword = tsc_swap_32(cmdword);
    }

    usleep(1000);
    status = tsc_write_blk(ifcdevice->node, dest_addr, (char*) mybuffer, sizeof(pp_options), cmdword);
    
    free(mybuffer);

    if (status) {
        LOG((LEVEL_ERROR,"tsc_blk_write() returned %d\n", status));
        return status;
    }

    return status_success;
}

static inline ifcdaqdrv_status ifcfastintdrv_alloc_tscbuf(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *kbuf_req) {
    int ret;

    if (!kbuf_req) {
        return status_internal;
    }

    do {
        LOG((LEVEL_INFO, "Trying to allocate %d bytes in kernel\n", kbuf_req->size));
        ret = tsc_kbuf_alloc(ifcdevice->node, kbuf_req);
    } while ((ret < 0) && (kbuf_req->size >>= 1) > 0);

    if(ret) {
        fprintf(stderr, "Impossible to allocate tsc kbuf\n");
        return status_internal;
    }

    /* Mapping kbuf in userspace */
    if (tsc_kbuf_mmap(ifcdevice->node, kbuf_req) < 0)
    {
        /* Free the kbuf */
        tsc_kbuf_free(ifcdevice->node, kbuf_req);
        fprintf(stderr, "ERROR: tsc_kbuf_mmap(kbuf_req) failed\n");
        return status_internal;
    }

    return status_success;
}

static inline ifcdaqdrv_status ifcfastintdrv_free_tscbuf(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *kbuf_req) {
    if (tsc_kbuf_free(ifcdevice->node, kbuf_req))
    {
        fprintf(stderr, "[ERROR] tsclib returned error when trying tsc_kbuf_free\n");
        return status_internal;
    }
    return status_success;
}


ifcdaqdrv_status ifcfastintdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice) {
    //void *p;
    int ret;

    /* Allocate tsc kbuf for the DMA operations */
    ifcdevice->smem_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->smem_dma_buf) {
        return status_internal;
    }

    // Try to allocate as large dma memory as possible
    ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
    do {
        LOG((LEVEL_INFO, "Trying to allocate %d bytes in kernel\n", ifcdevice->smem_dma_buf->size));
        ret = tsc_kbuf_alloc(ifcdevice->node, ifcdevice->smem_dma_buf);
    } while ((ret < 0) && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

    if(ret) {
        free(ifcdevice->smem_dma_buf);
        return status_internal;
    }

    /* Map kernel space in userspace */
    LOG((5, "Trying to mmap %dkiB in kernel for SMEM acquisition\n", ifcdevice->smem_dma_buf->size / 1024));
    if (tsc_kbuf_mmap(ifcdevice->node, ifcdevice->smem_dma_buf) < 0)
    {
        /* Free the kbuf */
        tsc_kbuf_free(ifcdevice->node, ifcdevice->smem_dma_buf);
        free(ifcdevice->smem_dma_buf);
        fprintf(stderr, "ERROR: tsc_kbuf_mmap(ifcdevice->smem_dma_buf) failed\n");
        return status_internal;
    }
    INFOLOG(("Successfully allocated space in kernel! [%d bytes]\n",ifcdevice->smem_dma_buf->size));
    return status_success;
}

uint64_t u64_setclr(uint64_t input, uint64_t bits, uint64_t mask, uint32_t offset) {
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

#define TSCDMA_PKT_SIZE 0x400 // 
#define CURR_DMA_CHAN DMA_CHAN_2

ifcdaqdrv_status ifcfastintdrv_read_smem_historybuffer( struct ifcdaqdrv_dev *ifcdevice, 
                                                        void *dest_buffer, 
                                                        struct tsc_ioctl_kbuf_req *dma_buf, 
                                                        uint32_t smem_addr, 
                                                        uint32_t size) 
{
    //return status_success;

    ifcdaqdrv_status status;
    struct tsc_ioctl_dma_req dma_req = {0};

    dma_req.src_addr = (uint64_t) smem_addr;
    dma_req.src_space = 2; //DMA_SPACE_SMEM
    dma_req.src_mode = 0; //DMA_PCIE_RR2;

    dma_req.des_addr = dma_buf->b_base;
    dma_req.des_space = ifcdaqdrv_is_byte_order_ppc() ? DMA_SPACE_PCIE : DMA_SPACE_PCIE1,
    dma_req.des_mode = 0; //DMA_PCIE_RR2;

    dma_req.end_mode   = 0;
    dma_req.start_mode = (char) DMA_START_CHAN(CURR_DMA_CHAN);
    //dma_req.intr_mode  = DMA_INTR_ENA;
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_1S | (5<<4);

    //dma_req.size = size; //dma_buf->size | DMA_SIZE_PKT_128;
    dma_req.size = size;

    status = tsc_dma_alloc(ifcdevice->node, CURR_DMA_CHAN);
    if (status) 
    {
        LOG((LEVEL_ERROR, "%s() tsc_dma_alloc(CURR_DMA_CHAN) == %d \n", __FUNCTION__, status));
        tsc_dma_clear(ifcdevice->node, CURR_DMA_CHAN);
        tsc_dma_free(ifcdevice->node, CURR_DMA_CHAN);
        return status;
    }

    status = tsc_dma_move(ifcdevice->node, &dma_req);
    if (status) 
    {
        LOG((LEVEL_ERROR, "%s() tsc_dma_move() == %d status = 0x%08x\n", __FUNCTION__, status, dma_req.dma_status));
        tsc_dma_clear(ifcdevice->node, CURR_DMA_CHAN);
        tsc_dma_free(ifcdevice->node, CURR_DMA_CHAN);
        return status;
    }

    /* Check for errors */
    if (dma_req.dma_status & DMA_STATUS_TMO)
    {
        LOG((LEVEL_ERROR, "DMA ERROR -> timeout | status = %08x\n",  dma_req.dma_status));
        tsc_dma_clear(ifcdevice->node, CURR_DMA_CHAN);
        tsc_dma_free(ifcdevice->node, CURR_DMA_CHAN);
        return status;
    }
    else if(dma_req.dma_status & DMA_STATUS_ERR)
    {
        LOG((LEVEL_ERROR, "DMA ERROR -> unknown error | status = %08x\n",  dma_req.dma_status));
        tsc_dma_clear(ifcdevice->node, CURR_DMA_CHAN);
        tsc_dma_free(ifcdevice->node, CURR_DMA_CHAN);
        return status;
    }
    
    /*free DMA channel 0 */
    status = tsc_dma_free(ifcdevice->node, CURR_DMA_CHAN);
    if (status) 
    {
      LOG((LEVEL_ERROR, "%s() tsc_dma_free() == %d\n", __FUNCTION__, status));
      tsc_dma_clear(ifcdevice->node, CURR_DMA_CHAN);
      return status;
    }

    /* Copy from kernel space (dma_buf) to userspace (pp_options)
     * TODO: check for errors and fill buffer with zeros??
     */       
    memcpy(dest_buffer, dma_buf->u_base, (size_t) size);
    return 0;
}

#define RT_OFFSET 0x300000
ifcdaqdrv_status ifcfastintdrv_read_rtstatus(struct ifcdaqdrv_dev *ifcdevice, uint32_t addr, uint64_t *rt_status) 
{
    ifcdaqdrv_status status;
    int cmdword = RDWR_MODE_SET( 0x44, RDWR_SPACE_USR1, 0);
    void *mybuffer = calloc(1024*1024,1);

    if (!ifcdaqdrv_is_byte_order_ppc()) {
        cmdword = tsc_swap_32(cmdword);
    }

    ulong src_addr = RT_OFFSET + addr;

    usleep(1000);
    status = tsc_read_blk(ifcdevice->node, src_addr, (char*) mybuffer, sizeof(*rt_status), cmdword);

    if (status) {
        LOG((LEVEL_ERROR,"tsc_blk_read() returned %d\n", status));
        free(mybuffer);
        return status;
    }

    memcpy((void *)rt_status, mybuffer, sizeof(*rt_status));

    if (ifcdaqdrv_is_byte_order_ppc()) {
    	ifcdaqdrv_swap64(rt_status);
	}
    free(mybuffer);

    return status_success;
}

ifcdaqdrv_status ifcfastintdrv_eeprom_read(struct ifcdaqdrv_dev *ifcdevice, 
                                           ifcfastint_aichannel_param param,
                                           int channel,
                                           uint32_t *value)
{
    uint32_t data = 0;
    uint32_t addr = 0x2000;

    /**/
    switch (param)
    {
        case ifcfastint_aichannel_gain:
            addr = 0x2000 + (channel*4);
            *value = 0x8ccc; // Default value is 1
            break;

        case ifcfastint_aichannel_offset:
            addr = 0x2002 + (channel*4);
            *value = 0x8000;  // Default value is 0
            break;

        default:
            break;
    }

    if (ifcdaqdrv_is_byte_order_ppc()) {
        *value = 0;

        tsc_i2c_read(ifcdevice->node, 0x40010050, addr, &data);
        usleep(5000);
        *value = data;

        tsc_i2c_read(ifcdevice->node, 0x40010050, addr+1, &data);
        *value |= (data<<8);
        usleep(2500);
    } 

    return status_success;
} 

ifcdaqdrv_status ifcfastintdrv_eeprom_write(struct ifcdaqdrv_dev *ifcdevice, 
                                           ifcfastint_aichannel_param param,
                                           int channel,
                                           uint32_t value)
{
    uint32_t data = 0;
    uint32_t addr = 0x2000;

    /**/
    switch (param)
    {
        case ifcfastint_aichannel_gain:
            addr = 0x2000 + (channel*4);
            break;

        case ifcfastint_aichannel_offset:
            addr = 0x2002 + (channel*4);
            break;

        default:
            break;
    }

    if (ifcdaqdrv_is_byte_order_ppc()) {
        data = value & 0x000000ff;
        tsc_i2c_write(ifcdevice->node, 0x40010050, addr, data);
        usleep(5000);

        data = (value>>8) & 0x000000ff;
        tsc_i2c_write(ifcdevice->node, 0x40010050, addr+1, data);
        usleep(5000);
    }

    return status_success;
}

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>

#ifdef TOSCA_USRLIB
// #include <pevioctl.h>
// #include <pevxulib.h>
// #include <pevulib.h>
#include <tscioctl.h>
#include <tsculib.h>
#endif

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"
// #include "ifcdaqdrv_acq420.h"
#include "ifcdaqdrv_adc3110.h"
#include "ifcdaqdrv_scope.h"

ifcdaqdrv_status ifc_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t *i32_reg_val){
#if DEBUG
    if ((register_idx < 0) || (register_idx >= 0x3FF)) {
        // error message
        printf(
            "Error: ifc_tcsr_read(crate=%d,offset=0x%x,idx=%d) idx is out of range! (valid range is 0..0x3ff (1024))",
            ifcdevice->card, offset, register_idx);
        return -1;
    }
#endif

#ifdef TOSCA_USRLIB
    // *i32_reg_val = pevx_csr_rd(ifcdevice->card, TCSR_ACCESS_ADJUST + offset + (register_idx * 4));
    *i32_reg_val = tsc_csr_rd(TCSR_ACCESS_ADJUST + offset + (register_idx * 4));
#else
    *i32_reg_val = 0;
#endif

    return 0;
}

ifcdaqdrv_status ifc_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t value){
#if DEBUG
    if ((register_idx < 0) || (register_idx >= 0x3FF)) {
        // error message
        printf(
            "Error: ifc_tcsr_read(crate=%d,offset=0x%x:idx=%d) idx is out of range! (valid range is 0..0x3ff (1024))",
            ifcdevice->card, offset, register_idx);
        return -1;
    }
    int32_t i32_reg_val;
    ifc_tcsr_read(ifcdevice, offset, register_idx, &i32_reg_val);

    LOG((7, "crate %d: CSR[0x%x:%02x] %08x (befor write)\n", ifcdevice->card, offset, register_idx, i32_reg_val));
    LOG((7, "crate %d: CSR[0x%x:%02x] %08x (write)\n",       ifcdevice->card, offset, register_idx, value));
#endif

#ifdef TOSCA_USRLIB
    // pevx_csr_wr(ifcdevice->card, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), value);
    tsc_csr_wr(TCSR_ACCESS_ADJUST + offset + (register_idx * 4), value);
#endif

#if DEBUG
    ifc_tcsr_read(ifcdevice, offset, register_idx, &i32_reg_val);
    LOG((7, "crate %d: CSR[0x%x:%02x] %08x (after write)\n", ifcdevice->card, offset, register_idx, i32_reg_val));
#endif
    return 0;
}

ifcdaqdrv_status ifc_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t setmask, int32_t
                                 clrmask){
    int32_t i32_reg_val;
#if DEBUG
    if ((register_idx < 0) || (register_idx >= 1024)) {
        // error message
        printf("Error: ifc_tcsr_read(crate=%d,offset=0x%x,idx=%d) idx is out of range! (valid range is 0..1024)",
               ifcdevice->card, offset, register_idx);
        return -1;
    }
#endif

#ifdef TOSCA_USRLIB
    //i32_reg_val = pevx_csr_rd(ifcdevice->card, TCSR_ACCESS_ADJUST + offset + (register_idx * 4));
    i32_reg_val = tsc_csr_rd(TCSR_ACCESS_ADJUST + offset + (register_idx * 4));
#else
    i32_reg_val = 0;
#endif

#if DEBUG
    LOG((7, "crate %d: CSR[0x%x:%02x] %08x (befor setclr)\n", ifcdevice->card, offset, register_idx, i32_reg_val));
    LOG((7, "crate %d: CSR[0x%x:%02x] set=%08x clr=%08x\n",   ifcdevice->card, offset, register_idx, setmask, clrmask));
#endif

    i32_reg_val &= ~clrmask;
    i32_reg_val |= setmask;

#ifdef TOSCA_USRLIB
    //pevx_csr_wr(ifcdevice->card, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), i32_reg_val);
    tsc_csr_wr(TCSR_ACCESS_ADJUST + offset + (register_idx * 4), i32_reg_val);
#endif

#if DEBUG
    ifc_tcsr_read(ifcdevice, offset, register_idx, &i32_reg_val);
    LOG((7, "crate %d: CSR[0x%x:%02x] %08x (after setclr)\n", ifcdevice->card, offset, register_idx, i32_reg_val));
#endif
    return 0;
}

/* Functions for accessing any XUSER TCSR */

ifcdaqdrv_status ifc_xuser_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val){
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_xuser_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value){
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, register_idx, value);
}

ifcdaqdrv_status ifc_xuser_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                       clrmask){
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, register_idx, setmask, clrmask);
}

/* Functions for accessing 0x60 to 0x6F (SCOPE MAIN TCSR) */

ifcdaqdrv_status ifc_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val){
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value){
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, value);
}

ifcdaqdrv_status ifc_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                       clrmask){
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, 0x60 + register_idx, setmask, clrmask);
}

/* Functions for accessing 0x70-0x73, 0x74-0x77, 0x78-0x7B, 0x7C-0x7F (SCOPE FMC1/FMC2 and SRAM/SMEM specific) registers */

ifcdaqdrv_status ifc_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val){
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_tcsr_offset(ifcdevice) + register_idx, i32_reg_val);
}

ifcdaqdrv_status ifc_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value){
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_tcsr_offset(ifcdevice) + register_idx, value);
}

ifcdaqdrv_status ifc_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                           clrmask){
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, ifc_get_scope_tcsr_offset(ifcdevice) + register_idx, setmask, clrmask);
}

/* Functions for accessing 0x80-0xBF or 0xC0-0xFF based on FMC1/FMC2. */

ifcdaqdrv_status ifc_fmc_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *reg_val){
    return ifc_tcsr_read(ifcdevice, OFFSET_XUSER_CSR, ifc_get_fmc_tcsr_offset(ifcdevice) + register_idx, reg_val);
}

ifcdaqdrv_status ifc_fmc_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value){
    return ifc_tcsr_write(ifcdevice, OFFSET_XUSER_CSR, ifc_get_fmc_tcsr_offset(ifcdevice) + register_idx, value);
}

ifcdaqdrv_status ifc_fmc_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                     clrmask){
    return ifc_tcsr_setclr(ifcdevice, OFFSET_XUSER_CSR, ifc_get_fmc_tcsr_offset(ifcdevice) + register_idx, setmask, clrmask);
}

void ifcdaqdrv_free(struct ifcdaqdrv_dev *ifcdevice){
    // TODO finish free implementaiton
    if(ifcdevice->all_ch_buf) {
        free(ifcdevice->all_ch_buf);
        ifcdevice->all_ch_buf = NULL;
    }

    if(ifcdevice->smem_dma_buf) {
#ifdef TOSCA_USRLIB
        //pevx_buf_free(ifcdevice->card, ifcdevice->smem_dma_buf);
        tsc_kbuf_free(ifcdevice->smem_dma_buf);
#endif
        free(ifcdevice->smem_dma_buf);
        ifcdevice->smem_dma_buf = NULL;
    }

    if(ifcdevice->sram_dma_buf){
#ifdef TOSCA_USRLIB
        // pevx_buf_free(ifcdevice->card, ifcdevice->sram_dma_buf);
        tsc_kbuf_free(ifcdevice->sram_dma_buf);
#endif
        free(ifcdevice->sram_dma_buf);
        ifcdevice->sram_dma_buf = NULL;
    }

    if(ifcdevice->fru_id) {
        if(ifcdevice->fru_id->product_name) {
            free(ifcdevice->fru_id->product_name);
        }
        if(ifcdevice->fru_id->manufacturer) {
            free(ifcdevice->fru_id->manufacturer);
        }
        free(ifcdevice->fru_id);
        ifcdevice->fru_id = NULL;
    }
}

ifcdaqdrv_status ifcdaqdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice) {
    void *p;

    //ifcdevice->sram_dma_buf = calloc(1, sizeof(struct pev_ioctl_buf));
    ifcdevice->sram_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->sram_dma_buf) {
        goto err_sram_ctl;
    }

    ifcdevice->sram_dma_buf->size = ifcdevice->sram_size;

    LOG((5, "Trying to allocate %dkiB in kernel\n", ifcdevice->sram_size / 1024));

#ifdef TOSCA_USRLIB
    // if (pevx_buf_alloc(ifcdevice->card, ifcdevice->sram_dma_buf) == NULL) {
    //     goto err_sram_buf;
    // }

    if (tsc_kbuf_alloc(ifcdevice->sram_dma_buf) == NULL) {
        goto err_sram_buf;
    }

#endif

    //ifcdevice->smem_dma_buf = calloc(1, sizeof(struct pev_ioctl_buf));
    ifcdevice->smem_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->smem_dma_buf) {
        goto err_smem_ctl;
    }

    // Try to allocate as large dma memory as possible
    ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
    do {
        LOG((5, "Trying to allocate %dMiB in kernel\n", ifcdevice->smem_dma_buf->size / 1024 / 1024));

#ifdef TOSCA_USRLIB
        //p = pevx_buf_alloc(ifcdevice->card, ifcdevice->smem_dma_buf);
        p = tsc_kbuf_alloc(ifcdevice->smem_dma_buf);
#else
        p = NULL;
#endif


    } while (p == NULL && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

    if(!p) {
        goto err_smem_buf;
    }

    LOG((5, "Trying to allocate %dMiB in userspace\n", ifcdevice->smem_size / 1024 / 1024));
    ifcdevice->all_ch_buf = calloc(ifcdevice->smem_size, 1);
    if(!ifcdevice->all_ch_buf){
        goto err_smem_user_buf;
    }

    return status_success;

err_smem_user_buf:
#ifdef TOSCA_USRLIB
    //pevx_buf_free(ifcdevice->card, ifcdevice->smem_dma_buf);
    tsc_kbuf_free(ifcdevice->smem_dma_buf);
#endif
 

err_smem_buf:
    free(ifcdevice->smem_dma_buf);

err_smem_ctl:
#ifdef TOSCA_USRLIB
    //pevx_buf_free(ifcdevice->card, ifcdevice->smem_dma_buf);
    tsc_kbuf_free(ifcdevice->sram_dma_buf);
#endif

err_sram_buf:
    free(ifcdevice->sram_dma_buf);

err_sram_ctl:
    return status_internal;
}

/**
 * This is a helper function that reads from FPGA Block RAM named USR1 for FMC1 or USR2 for FMC2.
 * b_addr (bus address) should be a pev_ioctl_dma_req compatible des_addr pointer.
 *
 * @param ifdevice
 * @param offset Byte addressed offset
 * @param size Size in bytes
 */
ifcdaqdrv_status ifcdaqdrv_dma_read_unlocked(struct ifcdaqdrv_dev *ifcdevice, uint32_t src_addr, uint8_t src_space, uint8_t src_mode, uint32_t des_addr, uint8_t des_space, uint8_t des_mode, uint32_t size) {
    //struct pev_ioctl_dma_req dma_req = {0};
    struct tsc_ioctl_dma_req dma_req = {0};

    int status;
    uint32_t valid_dma_status;

    dma_req.src_addr  = src_addr;
    dma_req.src_space = src_space;
    dma_req.src_mode  = src_mode;

    dma_req.des_addr  = des_addr;
    dma_req.des_space = des_space;
    dma_req.des_mode  = des_mode;

    dma_req.size       = size;

    dma_req.start_mode = DMA_MODE_PIPE;
    // dma_req.start_mode = DMA_MODE_BLOCK;
    // dma_req.end_mode   = 0;

    dma_req.intr_mode  = DMA_INTR_ENA;                             // enable interrupt
    dma_req.wait_mode  = DMA_WAIT_INTR | DMA_WAIT_10MS | (5 << 4); // Timeout after 50 ms

    // dma_req.intr_mode = 0;
    // dma_req.wait_mode = DMA_WAIT_1S | (5<<4);
    // dma_req.dma_status = 0;

#ifdef TOSCA_USRLIB
    // status = pevx_dma_move(ifcdevice->card, &dma_req);
    status = tsc_dma_move(&dma_req);
#else
    status = status_success;
#endif

    if (status != 0) {
#if DEBUG
        LOG((4, "%s() pevx_dma_move() == %d status = 0x%08x\n", __FUNCTION__, status, dma_req.dma_status));
#endif
        return status_read;
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
#if DEBUG
        LOG((4, "Error: %s() DMA error 0x%08x\n", __FUNCTION__, dma_req.dma_status));
#endif
        return status_read;
    }
    return 0;
}

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

/*
 * This function reads /size/ bytes from /offset/ in SRAM (FPGA Block Ram) to /dma_buf/.
 *
 * @param ifcdevice Device structure..
 * @param dma_buf PEV DMA buffer to read into.
 * @param offset Offset in bytes to read from.
 * @param size Size in bytes to read.
 */

ifcdaqdrv_status ifcdaqdrv_read_sram_unlocked(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *dma_buf, uint32_t offset, uint32_t size) {
    int status;

    /* TODO: check usage of base address of the buffer. Tosca doesn't use pointer */
    //if (!dma_buf || !dma_buf->b_addr) {
    if (!dma_buf || !dma_buf->b_base) {
        return status_internal;
    }

    status = ifcdaqdrv_dma_read_unlocked(
            ifcdevice,
            offset, ifcdevice->fmc == 1 ? DMA_SPACE_USR1 : DMA_SPACE_USR2, DMA_PCIE_RR2,
            (ulong)dma_buf->u_base, DMA_SPACE_PCIE | swap_mask(ifcdevice), DMA_PCIE_RR2,
            size | DMA_SIZE_PKT_1K);

    return status;
}

// This is a fixed offset for FMC2 since SCOPE2 stores samples at this offset (256 MiB).
static inline int32_t smem_fmc_offset(struct ifcdaqdrv_dev *ifcdevice){
    return ifcdevice->fmc == 1 ? 0 : IFC_SCOPE_SMEM_FMC2_SAMPLES_OFFSET;
}

void ifc_stop_timer(struct ifcdaqdrv_dev *ifcdevice) {
#ifdef TOSCA_USRLIB
    //pevx_timer_stop(ifcdevice->card);
    tsc_timer_stop();
#endif
}

void ifc_init_timer(struct ifcdaqdrv_dev *ifcdevice){
#ifdef TOSCA_USRLIB
    // pevx_timer_start(ifcdevice->card, TIMER_1MHZ ,0);
    tsc_timer_start(TIMER_1MHZ, 0);
#endif
}

uint64_t ifc_get_timer(struct ifcdaqdrv_dev *ifcdevice){

#ifdef TOSCA_USRLIB
    // struct pevx_time tim;  
    // pevx_timer_read(ifcdevice->card, &tim);
    struct tsc_time tim;
    tsc_timer_read(ifcdevice->card, &tim);

    return ((uint64_t)tim.time * 1000) + (uint64_t)(tim.utime & 0x1ffff) / 100;
#else
    return 0;
#endif
    
}

/*
 * This function reads /size/ bytes from /offset/ in SMEM (DDR3) to /res/. It uses dma_buf as intermediate storage since
 * you typically cannot allocate arbitrarily large bus pointers.
 *
 * @param ifcdevice Device structure.
 * @param res Buffer to read into.
 * @param dma_buf PEV DMA buffer to use as intermediate storage.
 * @param offset Offset in bytes to read from.
 * @param size Size in bytes to read.
 */

ifcdaqdrv_status ifcdaqdrv_read_smem_unlocked(struct ifcdaqdrv_dev *ifcdevice, void *res, struct tsc_ioctl_kbuf_req *dma_buf, uint32_t offset, uint32_t size) {
    int status;
    intptr_t src_addr;
    uint32_t current_size;
    uint32_t total_size; /* Debug variable */
    uint64_t total_time; /* Debug variable */

    if(DEBUG) total_size = size;
    LOG((LEVEL_DEBUG, "Copying from: 0x%08x, amount: %u\n", offset, size));

    /* TODO: check usage of base address of the buffer. Tosca doesn't use pointer */
    //if (!dma_buf || !dma_buf->b_addr) {
    if (!dma_buf || !dma_buf->b_base) {
        return status_internal;
    }

    src_addr = smem_fmc_offset(ifcdevice) + offset;
    current_size = dma_buf->size;
    if(DEBUG) ifc_init_timer(ifcdevice);
    while(size != 0) {
        if(size < dma_buf->size) {
            current_size = size;
        }

        status = ifcdaqdrv_dma_read_unlocked(
                ifcdevice,
                src_addr, DMA_SPACE_SHM, DMA_PCIE_RR2,
                (ulong)dma_buf->b_base, DMA_SPACE_PCIE | swap_mask(ifcdevice), DMA_PCIE_RR2,
                current_size | DMA_SIZE_PKT_1K);

        if (status != 0) {
            return status;
        }

        memcpy(res + src_addr - (smem_fmc_offset(ifcdevice) + offset), dma_buf->u_base, current_size);

        src_addr += current_size;
        size     -= current_size;
    }

    if(DEBUG) total_time = ifc_get_timer(ifcdevice);
    if(DEBUG) ifc_stop_timer(ifcdevice);
    LOG((LEVEL_DEBUG, "read_smem_unlocked %.2f MB took %llu ms\n", (total_size)/1024.0/1024.0, total_time));

    return status_success;
}


// The following part is kept because we might implement interrupt handling with signals in the future...
#if 0
void init_eventqueue(){
    int crate                 = cardNum;

    struct pev_ioctl_evt *evt = pevx_evt_queue_alloc(crate, SIGUSR2);

    if (DBG(DBG_LEVEL4)) {
        printf("%s: allocate event queue crate=%d evt=%p (signal=%d)\n", __FUNCTION__, crate, evt, SIGUSR2);
    }

    if (evt) {
        int src_id;

#if 0
        int i;
        src_id = 0x40;
        for (i = 0; i < 16; i++) {
            int res = pevx_evt_register(crate, evt, src_id + i);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id + i, res);


            // res = pevx_evt_unmask(crate,evt,src_id + i);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id+i,res);
        }
#else
        // Doc: XUSER_SCOPE_UG_A0 5.3.8.6 Register_68 SCOPE Interrupt Status ....

        {         // AP_Specific Interrupt 0 - Scope SRAM1 Single Ended
            src_id = InterruptSourceID_SRAM1_SingleEnded;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }
        {         // AP_Specific Interrupt 1 - Scope SRAM2 Single Ended
            src_id = InterruptSourceID_SRAM2_SingleEnded;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }
        {         // AP_Specific Interrupt 2 - Scope SMEM1 Single Ended
            src_id = InterruptSourceID_SMEM1_SingleEnded;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }
        {         // AP_Specific Interrupt 3 - Scope SMEM2 Single Ended
            src_id = InterruptSourceID_SMEM2_SingleEnded;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }


        {         // AP_Specific Interrupt 6 - AP_Specific IP VME_P2/VME_P0 selectable source #A -> SCOPE 1
            src_id = InterruptSourceID_SCOPE1_Interlock;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }
        {         // AP_Specific Interrupt 7 - AP_Specific IP VME_P2/VME_P0 selectable source #B -> SCOPE 2
            src_id = InterruptSourceID_SCOPE2_Interlock;
            int res = pevx_evt_register(crate, evt, src_id);

            printf("pevx_evt_register(crate=%d,src_id=%d) res = %d\n", crate, src_id, res);

            // res = pevx_evt_unmask(crate,evt,src_id);
            // printf("pevx_evt_unmask(src_id=%d) res = %d\n",src_id,res);
        }
#endif

        evt->wait = -1;

        signal(evt->sig, myPevEventHandler);         // ?????

        printf("a0\n");
        pevx_evt_queue_enable(crate, evt);
        printf("a1\n");
    }

    pevCrateEventQueue[cardNum] = evt;

    if (pevCrateEventQueue[cardNum] == NULL) {
        printf("%s: ERROR cardNum=%d: initialization of PEV1100 event queue for crate %d failed!\n", MY_ID_STR, cardNum,
               cardNum);
        return NULL;
    }
}

printf("A0\n");
registerCleanupHandle();
printf("A1\n");
}

static void cleanupSigHandler(int sig, siginfo_t *info, void *ctx){
    /* Signal handler for "terminate" signals:
     *  SIGHUP, SIGINT, SIGPIPE, SIGALRM, SIGTERM
     *  as well as "core dump" signals:
     *  SIGQUIT, SIGILL, SIGABRT, SIGFPE, SIGSEGV
     */

    printf("%s: sig=%d\n", __FUNCTION__, sig);


    // cleanup
    drv_cleanup();


    /* try to clean up before exit */
    epicsExitCallAtExits();
    signal(sig, SIG_DFL);
    raise(sig);
}

static int registerCleanupHandleDone = 0;
static void registerCleanupHandle(){
    if (registerCleanupHandleDone) {
        return;
    }

    registerCleanupHandleDone = 1;

    // register only once ....


    printf("%s:\n", __FUNCTION__);


    struct sigaction sa;

    /* make sure that even at abnormal termination the clean up is called */
    sa.sa_sigaction = cleanupSigHandler;
    sa.sa_flags     = SA_SIGINFO;

    sigaction(SIGHUP,  &sa, NULL);
    sigaction(SIGINT,  &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);
    sigaction(SIGALRM, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGQUIT, &sa, NULL);
    sigaction(SIGILL,  &sa, NULL);
    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGFPE,  &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
}

// missing extern declaration ...
extern struct pev_ioctl_evt *pevx_evt_queue_alloc(int crate, int sig);

static void myPevEventHandler(int sig){
    int debugout = DBG(DBG_LEVEL2);


    if (debugout) {
        printf("%s: sig=%d\n", __FUNCTION__, sig);
    }

    int crate;
    for (crate = 0; crate < MAX_CARD; crate++) {
        struct pev_ioctl_evt *evt = pevCrateEventQueue[crate];

        if (evt) {
            int cnt = 0;
            do {
                pevx_evt_read(crate, evt, 0);
                cnt = evt->evt_cnt;
                if (evt->src_id) {
                    // clear event -> ready for next event
                    pevx_evt_unmask(crate, evt, evt->src_id);


                    if (debugout) {
                        printf("%s: crate=%d src_id=0x%04x vec_id=0x%04x evt_cnt=%d cnt=%d\n", __FUNCTION__, crate,
                               evt->src_id, evt->vec_id, evt->evt_cnt, cnt);
                    }


                    switch (evt->src_id) {
                    case InterruptSourceID_SRAM1_SingleEnded:
                    case InterruptSourceID_SRAM2_SingleEnded:
                    {
                        ScopeData *card = ScopeFind (crate, (evt->src_id == InterruptSourceID_SRAM1_SingleEnded) ?
                                                     IFC_FMC1 : IFC_FMC2);
                        if (card) {
                            // TODO ..

                            if (debugout) {
                                printf("%s: %s\n", __FUNCTION__, (evt->src_id == InterruptSourceID_SRAM1_SingleEnded) ?
                                       "SRAM1_SingleEnded" : "SRAM2_SingleEnded");
                            }
                        }
                    }
                    break;
                    case InterruptSourceID_SMEM1_SingleEnded:
                    case InterruptSourceID_SMEM2_SingleEnded:
                    {
                        ScopeData *card = ScopeFind (crate, (evt->src_id == InterruptSourceID_SMEM1_SingleEnded) ?
                                                     IFC_FMC1 : IFC_FMC2);
                        if (card) {
                            // TODO ..

                            if (debugout) {
                                printf("%s: %s\n", __FUNCTION__, (evt->src_id == InterruptSourceID_SMEM1_SingleEnded) ?
                                       "SMEM1_SingleEnded" : "SMEM2_SingleEnded");
                            }
                        }
                    }
                    break;
                    case InterruptSourceID_SCOPE1_Interlock:
                    case InterruptSourceID_SCOPE2_Interlock:
                    {
                        ScopeData *card = ScopeFind (crate, (evt->src_id == InterruptSourceID_SCOPE1_Interlock) ?
                                                     IFC_FMC1 : IFC_FMC2);
                        if (card) {
                            triggerInterlock(card, 0);

                            if (debugout) {
                                printf("%s: %s\n", __FUNCTION__, (evt->src_id == InterruptSourceID_SCOPE1_Interlock) ?
                                       "SCOPE1_Interlock" : "SCOPE2_Interlock");
                            }
                        }
                    }
                    break;

                    default:
                        break;
                    }
                } else {
                    // evt queue is emtpy
                    printf("%s: evt queue is empty!\n", __FUNCTION__);
                }
            } while (cnt);
        }
    }

    return;
}
#endif

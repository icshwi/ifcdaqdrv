#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <inttypes.h>
#include <time.h>

#include "tscioctl.h"
#include "tsculib.h"

#include "debug.h"
#include "ifcdaqdrv.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"

// Variables for the time-measure tool
struct timespec ts_start, ts_end;

ifcdaqdrv_status ifc_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t *i32_reg_val)
{
    int ret;
    if ((register_idx < 0) || (register_idx >= 0x3FF)) {
        fprintf(stderr,"ERROR: ifc_tcsr_read(offset=0x%x,idx=%d) idx is out of range! (valid range is 0..0x3ff (1024))",
            offset, register_idx);
        return -1;
    }

    ret = tsc_csr_read(ifcdevice->node, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), i32_reg_val);
    if (ret) {
        fprintf(stderr,"ERROR: tsc_csr_read() tsc library returned %d\n", ret);
    }
    return ret;
}

ifcdaqdrv_status ifc_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t value)
{
    if ((register_idx < 0) || (register_idx >= 0x3FF)) {
        fprintf(stderr,"ERROR: ifc_tcsr_write(offset=0x%x,idx=%d) idx is out of range! (valid range is 0..0x3ff (1024))",
            offset, register_idx);
        return -1;
    }

    int ret;
    ret = tsc_csr_write(ifcdevice->node, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), &value);

    if (ret) {
        fprintf(stderr,"ERROR: tsc_csr_write() tsc library returned %d\n", ret);
    }
    return ret;
}

ifcdaqdrv_status ifc_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t setmask, int32_t clrmask)
{
    int32_t i32_reg_val;
    int ret;

    ret = tsc_csr_read(ifcdevice->node, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), &i32_reg_val);
    if (ret) {
        return ret;
    }

    i32_reg_val &= ~clrmask;
    i32_reg_val |= setmask;

    ret = tsc_csr_write(ifcdevice->node, TCSR_ACCESS_ADJUST + offset + (register_idx * 4), &i32_reg_val);
    return ret;
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
        if (ifcdevice->smem_sg_dma)
            free(ifcdevice->smem_dma_buf->u_base);
        else {
            tsc_kbuf_munmap(ifcdevice->smem_dma_buf);
            tsc_kbuf_free(ifcdevice->node, ifcdevice->smem_dma_buf);
        }
        free(ifcdevice->smem_dma_buf);
        ifcdevice->smem_dma_buf = NULL;
    }

    if(ifcdevice->sram_dma_buf){
        // pevx_buf_free(ifcdevice->card, ifcdevice->sram_dma_buf);
        tsc_kbuf_munmap(ifcdevice->sram_dma_buf);
        tsc_kbuf_free(ifcdevice->node, ifcdevice->sram_dma_buf);
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
    int ret;

    ifcdevice->sram_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->sram_dma_buf) {
        goto err_sram_ctl;
    }

    ifcdevice->sram_dma_buf->size = ifcdevice->sram_size;

    LOG((5, "Trying to allocate %dkiB in kernel for SRAM acquisition with tsc_kbuf_alloc()\n", ifcdevice->sram_size / 1024));
    if (tsc_kbuf_alloc(ifcdevice->node, ifcdevice->sram_dma_buf) < 0)  {
        fprintf(stderr, "ERROR: tsc_kbuf_alloc() failed\n");
        goto err_sram_buf;
    }

    LOG((5, "Trying to mmap %dkiB in kernel for SRAM acquisition with tsc_kbuf_mmap()\n", ifcdevice->sram_dma_buf->size / 1024));
    if (tsc_kbuf_mmap(ifcdevice->node, ifcdevice->sram_dma_buf) < 0)  {
        fprintf(stderr, "ERROR: tsc_kbuf_mmap(ifcdevice->sram_dma_buf) failed\n");
        goto err_mmap_sram;
    }

    ifcdevice->smem_dma_buf = calloc(1, sizeof(struct tsc_ioctl_kbuf_req));
    if (!ifcdevice->smem_dma_buf) {
        fprintf(stderr, "ERROR: calloc(1, sizeof(struct tsc_ioctl_kbuf_req)) failed\n");
        goto err_smem_ctl;
    }

    if (ifcdevice->smem_sg_dma) {
        if (ifcdevice->smem_size > TSC_MAX_SG_DMA_LEN)
            ifcdevice->smem_size = TSC_MAX_SG_DMA_LEN;
        ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
        LOG((5, "Trying to allocate %dkiB in userspace for scatter gather SMEM acquisition\n", ifcdevice->smem_dma_buf->size / 1024));
        ifcdevice->smem_dma_buf->u_base = aligned_alloc(sysconf(_SC_PAGESIZE), ifcdevice->smem_dma_buf->size);
        if(!ifcdevice->smem_dma_buf->u_base) {
            fprintf(stderr, "ERROR: aligned_alloc() failed\n");
            goto err_smem_buf;
        }
    }
    else {
        // Try to allocate as large dma memory as possible but maximum TSC_MAX_DMA_LEN (15MB) due to Tosca limit
        if (ifcdevice->smem_size > TSC_MAX_DMA_LEN)
            ifcdevice->smem_size = TSC_MAX_DMA_LEN;
        ifcdevice->smem_dma_buf->size = ifcdevice->smem_size;
        do {
            LOG((5, "Trying to allocate %dkiB in kernel for SMEM acquisition\n", ifcdevice->smem_dma_buf->size / 1024));
            ret = tsc_kbuf_alloc(ifcdevice->node, ifcdevice->smem_dma_buf);
        } while ((ret < 0) && (ifcdevice->smem_dma_buf->size >>= 1) > 0);

        if(ret) {
            fprintf(stderr, "ERROR: tsc_kbuf_alloc() failed\n");
            goto err_smem_buf;
        }

        LOG((5, "Trying to mmap %dkiB in kernel for SMEM acquisition\n", ifcdevice->smem_dma_buf->size / 1024));
        if (tsc_kbuf_mmap(ifcdevice->node, ifcdevice->smem_dma_buf) < 0)  {
            fprintf(stderr, "ERROR: tsc_kbuf_mmap(ifcdevice->smem_dma_buf) failed\n");
            tsc_kbuf_free(ifcdevice->node, ifcdevice->smem_dma_buf);
            goto err_smem_buf;
        }
    }

    LOG((5, "Trying to allocate %dkiB in userspace\n", ifcdevice->smem_size / 1024));
    ifcdevice->all_ch_buf = calloc(ifcdevice->smem_size, 1);
    if(!ifcdevice->all_ch_buf){
        fprintf(stderr, "calloc(ifcdevice->smem_size, 1) failed\n");
        goto err_all_ch_buf;
    }

    LOG((5, "Trying to allocate 1 MiB in userspace for tsc_read_blk() calls\n"));
    ifcdevice->sram_blk_buf = calloc(1024*1024, 1);
    if(!ifcdevice->sram_blk_buf){
        fprintf(stderr, "calloc for the sram_blk_buf() failed\n");
        goto err_sram_blk_buf;
    }


    /* This routine was used during development to check the memory allocation */

#if 0
    uint64_t tp_sram;
    uint64_t tp_smem;
    tp_sram = (uint64_t) ifcdevice->sram_dma_buf->u_base;
    tp_smem = (uint64_t) ifcdevice->smem_dma_buf->u_base;     

    /* PRIx64 macro is from inttypes.h */
    printf("tsc_kbuf_alloc() was successful, buffers were filled with:\n");
    printf("########################################################################\n");
    printf("sram_dma_buf->size = %d\n", ifcdevice->sram_dma_buf->size);
    printf("sram_dma_buf->b_base = 0x%" PRIx64"\n", ifcdevice->sram_dma_buf->b_base);
    printf("sram_dma_buf->u_base = 0x%" PRIXPTR "\n", (uintptr_t)ifcdevice->sram_dma_buf->u_base);
    printf("########################################################################\n");
    printf("smem_dma_buf->size = %d\n", ifcdevice->smem_dma_buf->size);
    printf("smem_dma_buf->b_base = 0x%" PRIx64"\n", ifcdevice->smem_dma_buf->b_base);
    printf("smem_dma_buf->u_base = 0x%" PRIXPTR"\n", (uintptr_t) ifcdevice->smem_dma_buf->u_base);
    printf("########################################################################\n");
#endif

    return status_success;

err_sram_blk_buf:
    free(ifcdevice->all_ch_buf);

err_all_ch_buf:
    if (ifcdevice->smem_sg_dma)
        free(ifcdevice->smem_dma_buf->u_base);
    else {
        tsc_kbuf_munmap(ifcdevice->smem_dma_buf);
        tsc_kbuf_free(ifcdevice->node, ifcdevice->smem_dma_buf);
    }

err_smem_buf:
    free(ifcdevice->smem_dma_buf);

err_smem_ctl:
    tsc_kbuf_munmap(ifcdevice->sram_dma_buf);

err_mmap_sram:
    tsc_kbuf_free(ifcdevice->node, ifcdevice->sram_dma_buf);

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
ifcdaqdrv_status 
ifcdaqdrv_dma_read_unlocked(struct ifcdaqdrv_dev *ifcdevice,
			    uint64_t src_addr,
			    uint8_t src_space,
			    uint8_t src_mode,
			    uint64_t des_addr,
			    uint8_t des_space,
			    uint8_t des_mode,
			    uint32_t size)
{
    struct tsc_ioctl_dma_req dma_req = {0};
    int status;

    dma_req.src_addr  = src_addr;
    dma_req.src_space = src_space;
    dma_req.src_mode  = 0;
    dma_req.des_addr  = des_addr;
    dma_req.des_space = des_space;
    dma_req.des_mode  = 0;
    dma_req.size       = size;
    dma_req.end_mode   = 0;
    dma_req.intr_mode  = DMA_INTR_ENA;
    dma_req.wait_mode  = 0;

    status = tsc_dma_transfer(ifcdevice->node, &dma_req);
    if (status) {
        LOG((LEVEL_ERROR, "tsc_dma_transfer() returned 0x%x, DMA status 0x%08x\n", status, dma_req.dma_status));
        return status;
    }

    return  status_success;
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

    if (!dma_buf || !dma_buf->b_base) {
        return status_internal;
    }

    status = ifcdaqdrv_dma_read_unlocked(ifcdevice,
					 (uint64_t) offset,
					 ifcdevice->fmc == 1 ? DMA_SPACE_USR : DMA_SPACE_USR2,
					 DMA_PCIE_RR2,
					 dma_buf->b_base,
					 ifcdaqdrv_is_byte_order_ppc() ? DMA_SPACE_PCIE : DMA_SPACE_PCIE1,
					 DMA_PCIE_RR2,
					 size);

    return status;
}

void ifc_stop_timer(struct ifcdaqdrv_dev *ifcdevice) {
    tsc_timer_stop(ifcdevice->node);
}

void ifc_init_timer(struct ifcdaqdrv_dev *ifcdevice){
    tsc_timer_start(ifcdevice->node, TIMER_1MHZ, 0);
}

uint64_t ifc_get_timer(struct ifcdaqdrv_dev *ifcdevice){
    struct tsc_time tim;
    tsc_timer_read(ifcdevice->node, &tim);

    return ((uint64_t)tim.msec * 1000) + (uint64_t)(tim.usec & 0x1ffff) / 100;
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
    uint32_t current_size;
    uint32_t total_size = 0;
    uint64_t __attribute__((unused)) total_time; /* Debug variable */

    //long meastime = 0;

    LOG((LEVEL_DEBUG, "Copying from: 0x%08x, amount: %u\n", offset, size));

    if (!ifcdevice->smem_sg_dma && (!dma_buf || !dma_buf->b_base)) {
        return status_internal;
    }
    else if (ifcdevice->smem_sg_dma && (!dma_buf || !dma_buf->u_base)) {
        return status_internal;
    }

    current_size = dma_buf->size;
    //if(DEBUG) ifc_init_timer(ifcdevice);
    while(size != 0) {
        if(size < dma_buf->size) {
            current_size = size;
        }

#if 0
	printf(" [smem_read] dma_buf->size = %"PRIu32" \n", dma_buf->size);
	printf(" [smem_read] dma_buf->size = 0x%08x \n", dma_buf->size);
	printf(" [smem_read] curr_size = %"PRIu32" \n", current_size);
	printf(" [smem_read] curr_size = 0x%08x \n", current_size);
	printf(" [smem_read] size = %"PRIu32" \n", size);
	printf(" [smem_read] size = 0x%08x \n", size);
#endif

        status = ifcdaqdrv_dma_read_unlocked(ifcdevice,
					     (uint64_t) offset,
					     ifcdevice->fmc == 1 ? DMA_SPACE_SHM : DMA_SPACE_SHM2,
					     DMA_PCIE_RR2,
					     ifcdevice->smem_sg_dma ? (uint64_t)dma_buf->u_base : dma_buf->b_base,
					     ifcdaqdrv_is_byte_order_ppc() ? DMA_SPACE_PCIE | DMA_SPACE_WS : DMA_SPACE_PCIE1 | DMA_SPACE_WS,
					     DMA_PCIE_RR2,
					     current_size | DMA_SIZE_PKT_1K);

        if (status != 0) {
            return status;
        }

        memcpy(res + total_size, dma_buf->u_base, current_size);

        offset += current_size;
        size -= current_size;
        total_size += current_size;
    }

    //if(DEBUG) total_time = ifc_get_timer(ifcdevice);
    //if(DEBUG) ifc_stop_timer(ifcdevice);
    // LOG((LEVEL_DEBUG, "read_smem_unlocked %.2f MB took %llu ms\n", (total_size)/1024.0/1024.0, total_time));
    LOG((LEVEL_DEBUG, "read_smem_unlocked %.2f MB  - SUCCESS\n", (total_size)/1024.0/1024.0));

    return status_success;
}

void ifcdaqdrv_manualswap(uint16_t *buffer, int nsamples)
{
    uint16_t aux;
    int i;

    for (i = 0; i < nsamples; i++)
    {
        aux = (buffer[i] & 0xff00) >> 8;
        buffer[i] = ((buffer[i] & 0x00ff) << 8) | aux;

    }
}

void ifcdaqdrv_start_tmeas(void)
{
    clock_gettime(CLOCK_REALTIME, &ts_start);
}

void ifcdaqdrv_end_tmeas(void)
{
    clock_gettime(CLOCK_REALTIME, &ts_end);
}


long ifcdaqdrv_elapsedtime(void)
{
    struct timespec temp;
    if ((ts_end.tv_nsec-ts_start.tv_nsec)<0) {
        temp.tv_sec = ts_end.tv_sec-ts_start.tv_sec-1;
        temp.tv_nsec = 1000000000+ts_end.tv_nsec-ts_start.tv_nsec;
    } else {
        temp.tv_sec = ts_end.tv_sec-ts_start.tv_sec;
        temp.tv_nsec = ts_end.tv_nsec-ts_start.tv_nsec;
    }
    return (temp.tv_nsec / 1000);
}

int ifcdaqdrv_is_byte_order_ppc(void)
{
    if (CheckByteOrder()){
        return 1; // Big endian ppc
    }
    else{
        return 0; // Little endian x86_64
    }
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

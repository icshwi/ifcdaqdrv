#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#include <ifcdaqdrv.h>

#include "debug.h"
#include "ifcdaqdrv_utils.h"
#include "ifcfastintdrv.h"
#include "ifcfastintdrv_utils.h"
#include "ifcdaqdrv_adc3110.h"

ifcdaqdrv_status ifcfastint_init_fsm(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Set all channels to "DIRECT" mode */

    /* Enable acquisition on all channels */
    //status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_FMC1_CSR_REG, 0x02020202);
    //if(status) {
    //    pthread_mutex_unlock(&ifcdevice->lock);
    //    return status_internal;
    //}

    //status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_FMC2_CSR_REG, 0x02020202);
    //if(status) {
    //    pthread_mutex_unlock(&ifcdevice->lock);
    //    return status_internal;
    //}

    /* clear fsm and history registers */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0, IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK);

    /* Initialize history buffer size and pointers */
    /* Maximize to 65k values for now */
//    intptr_t buf_start = 0x00100000;
//    intptr_t buf_end   = 0x00300000;

    /* Use maximum size of the memory (256 MB) */
    intptr_t buf_start = 0x00000000;
    intptr_t buf_end   = 0x0FF00000;

    i32_reg_val = buf_end + (buf_start >> 16);

    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_SIZE_REG, i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    /* Read current WRITE POINTER location */
    //status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, buf_start + 64); // First frame is special...
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, &i32_reg_val); // First frame is special...
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    /* Set the read pointer just after the current write location */
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, i32_reg_val + 64); // First frame is special...
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    //usleep(10000);

    /* Reset FSM. Set it to IDLE. */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
            3 << IFCFASTINT_FSM_MAN_FSM_CMD_SHIFT,
            IFCFASTINT_FSM_MAN_FSM_CMD_MASK);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    ifcfastintdrv_history_reset(ifcdevice);

    /* Enable QOUT memorization */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_DIGITAL_PP_STATUS_REG, 1 << 31, 0);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_ANALOG_PP_STATUS_REG, 1 << 31, 0);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

//    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
//            2 << IFCFASTINT_FSM_MAN_FSM_CMD_SHIFT,
//            IFCFASTINT_FSM_MAN_FSM_CMD_MASK);
//    if(status) {
//        pthread_mutex_unlock(&ifcdevice->lock);
//        return status_internal;
//    }
//
    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;

}

/* This is only safe to call when state == abort, otherwise it will hang indefinitely */
ifcdaqdrv_status ifcfastint_wait_abort_done(struct ifcdaqdrv_usr *ifcuser) {

    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;
    int32_t timeout = 2500;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Read History_Status and wait for 11 -> History acquisition ended */
    do {
        status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        usleep(ifcdevice->poll_period);
        timeout--;
    } while((((i32_reg_val >> 8) & 0x3) != 3) && (timeout > 0));

    if (timeout == 0)
    	printf("[TIMEOUT] Exiting wait_abort_done... \n");

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}


ifcdaqdrv_status ifcfastint_get_pp_out(struct ifcdaqdrv_usr *ifcuser,
                                       uint32_t *digital,
                                       uint32_t *analog) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Read the digital outputs */
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_DIGITAL_PP_STATUS_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    *digital = i32_reg_val && IFCFASTINT_DIGITAL_PP_STATUS_QOUT_MASK;

    /* Read the analog outputs */
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_ANALOG_PP_STATUS_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    *analog = i32_reg_val && IFCFASTINT_ANALOG_PP_STATUS_QOUT_MASK;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_read_lastframe(struct ifcdaqdrv_usr *ifcuser, void *data)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!data) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    // Get circular buffer size
    // Top 16 bits are buffer end pointer, lower 16 bits are buffer start pointer.
    size_t buf_size;
    intptr_t buf_start;
    intptr_t buf_end;
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_SIZE_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    buf_start = ((i32_reg_val & 0xFFFF) << 16) + 64; // First history slot is special
    buf_end = (i32_reg_val & 0xFFFF0000); // This points to first item *after* the buffer
    buf_size = buf_end - buf_start;

    intptr_t content_start;
    intptr_t content_end;

    // Store write pointer in `content_end`
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    /*
     * Never read out the last item, it will make hardware think it overflowed.
     * W_PTR points to "next empty slot". Which means that we need to back it 2 steps.
     */
    content_end = i32_reg_val - 64;
    if(content_end < buf_start) {
            content_end += buf_size;
    }

    /* Read 64 bytes from SMEM */
    size = 64;
    content_start = content_end - size;

    ifcdaqdrv_read_smem_unlocked(
            ifcdevice,
            data,
            ifcdevice->smem_dma_buf,
            content_start,
            size
    );

    // Update Read PTR
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, content_end);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_read_history(struct ifcdaqdrv_usr *ifcuser, size_t count,
        void *data, size_t *nelm) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;
    size_t size;
    LOG((LEVEL_TRACE, "%s\n", "Enter"));

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!nelm || !data) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if(DEBUG) {
        status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
        if(ifcdaqdrvDebug >= LEVEL_DEBUG) {
            ifcfastint_print_status(i32_reg_val);
        }
    }

    // Get circular buffer size
    // Top 16 bits are buffer end pointer, lower 16 bits are buffer start pointer.
    size_t buf_size;
    intptr_t buf_start;
    intptr_t buf_end;
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_SIZE_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    buf_start = ((i32_reg_val & 0xFFFF) << 16) + 64; // First history slot is special
    buf_end = (i32_reg_val & 0xFFFF0000); // This points to first item *after* the buffer
    buf_size = buf_end - buf_start;

    intptr_t content_start;
    intptr_t content_end;
    // Store read pointer in `content_start`
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    content_start = i32_reg_val;

    // Store write pointer in `content_end`
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    /*
     * Never read out the last item, it will make hardware think it overflowed.
     * W_PTR points to "next empty slot". Which means that we need to back it 2 steps.
     */
    content_end = i32_reg_val - 3*64;
    if(content_end < buf_start) {
            content_end += buf_size;
    }

    if(content_end < buf_start || content_start < buf_start) {
        LOG((LEVEL_ERROR, "bs: 0x%08" PRIxPTR " be: 0x%08" PRIxPTR " cs: 0x%08" PRIxPTR " ce: 0x%08" PRIxPTR "\n", buf_start, buf_end, content_start, content_end));
        LOG((LEVEL_ERROR, "%s\n", "Internal error.. Content pointers are outside buffer area."));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    LOG((LEVEL_DEBUG, "buffer start: 0x%08" PRIxPTR " end: 0x%08" PRIxPTR " content start: 0x%08" PRIxPTR " end: 0x%08" PRIxPTR "\n", buf_start, buf_end, content_start, content_end));

    if (!count && content_end > content_start) {
        size = content_end - content_start;
    } else if (!count) {
        size = buf_size - (content_start - content_end);
    } else if (count * 64 > buf_size) {
        size = buf_size;
    } else {
        size = count * 64;
    }
    *nelm = size/64;

    // New content_start. `content_start` might end up outside the buffer
    content_start = content_end - size;

    // Bail early if 0 frames was found
    if(!size) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_success;
    }

    // Check if `content_start` is outside buffer
    if(content_start > buf_start) {
        LOG((LEVEL_DEBUG, "reading out %zd (%zd bytes) in order\n", *nelm, size));
        ifcdaqdrv_read_smem_unlocked(
                ifcdevice,
                data,
                ifcdevice->smem_dma_buf,
                content_start,
                size
        );
    } else {
        content_start += buf_size;
        LOG((LEVEL_DEBUG, "reading out %zd (%zd bytes) out of order. CE %08" PRIxPTR ", BS %08" PRIxPTR "\n",
                    *nelm, size, content_end - size, buf_start));
        ifcdaqdrv_read_smem_unlocked(
                ifcdevice,
                data,
                ifcdevice->smem_dma_buf,
                content_start,
                buf_end - content_start
        );
        ifcdaqdrv_read_smem_unlocked(
                ifcdevice,
                data + (buf_end - content_start),
                ifcdevice->smem_dma_buf,
                buf_start,
                content_end - buf_start
        );
    }

    // Update Read PTR
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, content_end);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_fsm_reset(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Clear and enable history */
    ifcfastintdrv_history_reset(ifcdevice);

    // Force to idle
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
            3 << IFCFASTINT_FSM_MAN_FSM_CMD_SHIFT,
            IFCFASTINT_FSM_MAN_FSM_CMD_MASK);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_get_fsm_state(struct ifcdaqdrv_usr *ifcuser,
                                          ifcfastint_fsm_state *state) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!state) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    switch((i32_reg_val & IFCFASTINT_FSM_MAN_FSM_STA_MASK) >> IFCFASTINT_FSM_MAN_FSM_STA_SHIFT) {
    case 0:
        *state = ifcfastint_fsm_state_idle;
        break;
    case 1:
        *state = ifcfastint_fsm_state_abort;
        break;
    case 2:
        *state = ifcfastint_fsm_state_pre;
        break;
    case 3:
        *state = ifcfastint_fsm_state_run;
        break;
    default:
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_fsm_state(struct ifcdaqdrv_usr *ifcuser, ifcfastint_fsm_state state) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    //int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = status_success;

    switch(state){
    case ifcfastint_fsm_state_idle:
        status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0x3<<28, 0xf<<28);
        break;
    case ifcfastint_fsm_state_pre:
        status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0x1<<28, 0xf<<28);
        break;
    case ifcfastint_fsm_state_run:
        status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0x2<<28, 0xf<<28);
        break;
    case ifcfastint_fsm_state_arm:
        // state removed
        //status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0x2<<28, 0xf<<28);
        break;
    case ifcfastint_fsm_state_abort:
        status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0x4<<28, 0xf<<28);
        break;
    default:
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_argument_invalid;
    }
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_get_fsm_do(struct ifcdaqdrv_usr *ifcuser,
                                       uint32_t *channel_mask) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!channel_mask) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    *channel_mask = (i32_reg_val & IFCFASTINT_FSM_MAN_FSM_OUT_MASK) >> IFCFASTINT_FSM_MAN_FSM_OUT_SHIFT;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_fsm_frequency(struct ifcdaqdrv_usr *ifcuser,
                                              uint32_t frequency) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    switch(frequency) {
    case 0:
        i32_reg_val = 0;
        break;
    case 1000:
        i32_reg_val = 1;
        break;
    case 500:
        i32_reg_val = 2;
        break;
    case 200:
        i32_reg_val = 3;
        break;
    default:
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
                          i32_reg_val << IFCFASTINT_FSM_MAN_FSM_FRQ_SHIFT,
                          IFCFASTINT_FSM_MAN_FSM_FRQ_MASK);

    pthread_mutex_unlock(&ifcdevice->lock);

    if(status) {
        return status_internal;
    }

    return status_success;
}

ifcdaqdrv_status ifcfastint_get_fsm_frequency(struct ifcdaqdrv_usr *ifcuser,
                                              uint32_t *frequency) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!frequency) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);

    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    switch((i32_reg_val & IFCFASTINT_FSM_MAN_FSM_FRQ_MASK) >> IFCFASTINT_FSM_MAN_FSM_FRQ_SHIFT) {
    case 0:
        *frequency = 0;
        break;
    case 1:
        *frequency = 1000;
        break;
    case 2:
        *frequency = 500;
        break;
    case 3:
        *frequency = 200;
        break;
    default:
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_get_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
											   ifcfastint_analog_pp ppblock,
                                               struct ifcfastint_analog_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val = 0;
    uint32_t fpga_mem_address = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Current firmware supports only 20 analog inputs */
    if(block >= 20) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    if ((ppblock != ifcfastint_analog_pp_channel)&&(ppblock != ifcfastint_analog_pp_pwravg))
    {
    	/* Read the configuration (OPTION1) of the standard analog input pre-processing block. */
    	fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + block*8; // OPTION1 registers starts at 0x200

    	pthread_mutex_lock(&ifcdevice->lock);
    	status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
    	if(status) {
    		pthread_mutex_unlock(&ifcdevice->lock);
    		return status_internal;
    	}

    	option->mode = (pp_options >> 56) & 0xF;
    	option->emulation_en = (pp_options >> 62) & 1;
    	option->val1 = (pp_options >> 16) & 0xFFFF;
    	option->val2 = (pp_options >> 32) & 0x00FFFFFF; // VAL2 now is 24 bit register
    	option->cval = pp_options & 0xFFFF;

    	/* TODO: There is no need for VAL3 and VAL4 anymore in this block */
    	option->val3 = 0xFFFF;
    	option->val4 = 0xFFFF;
    }

    /* Read IDLE->PRE qualifier TODO(nc): unmagicify ??? */
    status = ifc_xuser_tcsr_read(ifcdevice, 0x70, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->idle2pre = (i32_reg_val >> block) & 1; // A_in bits 19:0

    /* Read PRE->RUN qualifier */
    status = ifc_xuser_tcsr_read(ifcdevice, 0x74, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->pre2run |= (i32_reg_val >> block) & 1;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_get_conf_pwravg_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               struct ifcfastint_pwravg_option *option)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val = 0;
    uint32_t fpga_mem_address = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Current firmware supports only 4 pwravg blocks */
    if(block >= 4) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    /* Sets the offset address on TMEM4_1 that holds OPTION1 configuration */
    fpga_mem_address = 0x600 + (block*16);

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    option->mode = (pp_options >> 56) & 0xF;
    option->emulation_en = (pp_options >> 62) & 1;
    option->val1 = (pp_options >> 16) & 0xFFFF;
    option->val2 = (pp_options >> 32) & 0xFFFF;
    option->cval = pp_options & 0xFFFF;

    /* Read which analog in channels are connected to this block */
    option->u_in = option->val2 & 0x1F;
    option->i_in = (option->val2 >> 8) & 0x1F;

    /* Prepares to read OPTION2 configuration (starts at 0x600) */
   	fpga_mem_address = 0x608 + (block*16);
   	status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
   	if(status) {
   		pthread_mutex_unlock(&ifcdevice->lock);
   		return status_internal;
   	}

   	option->val3 = pp_options & 0xFFFF;
   	option->val4 = (pp_options >> 32) & 0xFFFF;

    /* Read IDLE->PRE qualifier */
    status = ifc_xuser_tcsr_read(ifcdevice, 0x70, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
   	option->idle2pre = (i32_reg_val >> (block+24)) & 1; // PWRAVG bits 27:24

    /* Read PRE->RUN qualifier */
    status = ifc_xuser_tcsr_read(ifcdevice, 0x74, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
   	option->pre2run |= (i32_reg_val >> (block+24)) & 1;

    pthread_mutex_unlock(&ifcdevice->lock);
	return status_success;
}

ifcdaqdrv_status ifcfastint_set_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
											   ifcfastint_analog_pp ppblock,
                                               struct ifcfastint_analog_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val;
    uint64_t active = 0;
    uint32_t fpga_mem_address = 0;


    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Current firmware supports only 20 analog inputs */
    if(block >= 20) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    // TODO: check why this is written
    // The PP block should always be active, otherwise we don't get any measurements...
    //if(option->mode > 0 && option->mode <= 12) {
    //    active =  1;
    //}
    active = 1;

    if ((ppblock != ifcfastint_analog_pp_channel)&&(ppblock != ifcfastint_analog_pp_pwravg))
    {
		/* TMEM4_1 address */
		fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + block*8; // OPTION1 registers starts at 0x200

		/* Read current configuration */
		pthread_mutex_lock(&ifcdevice->lock);
		status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
		if(status) {
			pthread_mutex_unlock(&ifcdevice->lock);
			return status_internal;
		}

		/* Writing a new mode */
		if(write_mask & IFCFASTINT_ANALOG_MODE_W) {
			pp_options = u64_setclr(pp_options, active, 1, 63);
			pp_options = u64_setclr(pp_options, option->mode, 0xF, 56);
		}

		/* Enabling emulation */
		if(write_mask & IFCFASTINT_ANALOG_EMULATION_EN_W) {
			pp_options = u64_setclr(pp_options, option->emulation_en, 1, 62);
		}

		/* Writing VAL1 register */
		if(write_mask & IFCFASTINT_ANALOG_VAL1_W) {
			pp_options = u64_setclr(pp_options, option->val1, 0xFFFF, 16);
		}

		/* Writing VAL2 register */
		if(write_mask & IFCFASTINT_ANALOG_VAL2_W) {
			pp_options = u64_setclr(pp_options, (option->val2 & 0x00FFFFFF), 0xFFFFFF, 32);
		}

		/* Writing emulation value register */
		if(write_mask & IFCFASTINT_ANALOG_CVAL_W) {
			pp_options = u64_setclr(pp_options, (uint16_t)option->cval, 0xFFFF, 0);
		}

    }
    /*
     * Return write error if configuration is locked.
     * A bit value of 1 indicates that it is unlocked.
     * ONLY CHECK ANALOG PERMISSION !!!
     */
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(!(i32_reg_val & (1 << IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT)))
    {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_write;
    }

    /* Writes OPTION1 configuration space */
    fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + block*8;
    status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    /* OPTION2 is not necessary if pp block is not pwravg */
//    status = ifcfastintdrv_read_pp_conf(ifcdevice, 0x300 + block*8, &pp_options);
//    if(status) {
//        pthread_mutex_unlock(&ifcdevice->lock);
//        return status_internal;
//    }
//    if(write_mask & IFCFASTINT_ANALOG_VAL3_W) {
//        pp_options = u64_setclr(pp_options, (uint16_t)option->val3, 0xFFFF, 0);
//    }
//    if(write_mask & IFCFASTINT_ANALOG_VAL4_W) {
//        pp_options = u64_setclr(pp_options, (uint16_t)option->val4, 0xFFFF, 32);
//    }
//
//    status = ifcfastintdrv_write_pp_conf(ifcdevice, 0x300 + block*8, pp_options);
//    if(status) {
//        pthread_mutex_unlock(&ifcdevice->lock);
//        return status_internal;
//    }

    // TODO(nc): unmagicify
    if(write_mask & IFCFASTINT_ANALOG_IDLE2PRE_W) {
        i32_reg_val = option->idle2pre << block;
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x70, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x72, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

    }

    if(write_mask & IFCFASTINT_ANALOG_PRE2RUN_W) {
        i32_reg_val = option->pre2run << block;
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x74, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x76, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_conf_pwravg_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
											   struct ifcfastint_pwravg_option *option)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val;
    uint64_t active = 0;
    uint32_t fpga_mem_address = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Currently only 4 blocks (0-3) are instantiated */
    if(block >= 4) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    // TODO: check why this is written
    // The PP block should always be active, otherwise we don't get any measurements...
    //if(option->mode > 0 && option->mode <= 12) {
    //    active =  1;
    //}
    active = 1;

    /* TMEM4_1 address space for POWER AVERAGING block */
    fpga_mem_address = 0x600 + (block*16);

    /* Read current configuration */
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    /* Writing a new mode */
    if(write_mask & IFCFASTINT_ANALOG_MODE_W) {
        pp_options = u64_setclr(pp_options, active, 1, 63);
        pp_options = u64_setclr(pp_options, option->mode, 0xF, 56);
    }

    /* Enabling emulation */
    if(write_mask & IFCFASTINT_ANALOG_EMULATION_EN_W) {
        pp_options = u64_setclr(pp_options, option->emulation_en, 1, 62);
    }

    /* Writing VAL1 register */
    if(write_mask & IFCFASTINT_ANALOG_VAL1_W) {
        // Cast to avoid sign extension
        pp_options = u64_setclr(pp_options, (uint16_t)option->val1, 0xFFFF, 16);
    }

    /* Writing VAL2 register - use u_in and i_in fields */
    if(write_mask & IFCFASTINT_ANALOG_VAL2_W) {
        option->val2 = (((option->i_in & 0x1F) << 8) | (option->u_in & 0x1F));
    	pp_options = u64_setclr(pp_options, (uint16_t)option->val2, 0xFFFF, 32);
    }

    /* Writing emulation value register */
    if(write_mask & IFCFASTINT_ANALOG_CVAL_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->cval, 0xFFFF, 0);
    }

    /*
     * Return write error if configuration is locked.
     * A bit value of 1 indicates that it is unlocked.
     * ONLY CHECK ANALOG PERMISSION !!!
     */
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(!(i32_reg_val & (1 << IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT)))
    {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_write;
    }

    /* Writes OPTION1 configuration space */
    fpga_mem_address = 0x600 + (block*16);
    status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    /* Write OPTION2 configuration space */

    fpga_mem_address = 0x608 + (block*16);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    if(write_mask & IFCFASTINT_ANALOG_VAL3_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->val3, 0xFFFF, 0);
    }
    if(write_mask & IFCFASTINT_ANALOG_VAL4_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->val4, 0xFFFF, 32);
    }

    fpga_mem_address = 0x608 + (block*16);
    status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    // TODO(nc): unmagicify
    if(write_mask & IFCFASTINT_ANALOG_IDLE2PRE_W) {
        i32_reg_val = option->idle2pre << (block+24);
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x70, i32_reg_val, 1 << (block+24));
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x72, i32_reg_val, 1 << (block+24));
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

    }

    if(write_mask & IFCFASTINT_ANALOG_PRE2RUN_W) {
        i32_reg_val = option->pre2run << (block+24);
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x74, i32_reg_val, 1 << (block+24));
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x76, i32_reg_val, 1 << (block+24));
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}


ifcdaqdrv_status ifcfastint_get_conf_digital_pp(struct ifcdaqdrv_usr *ifcuser,
                                           uint32_t block,
                                           struct ifcfastint_digital_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options1 = 0;
    int32_t i32_reg_val;
    uint32_t fpga_mem_address = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(block >= 32) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    /*
     * Read the configuration block.
     */
    fpga_mem_address = 0x100 + block*8;

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options1);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    /* Fill extracted information on the struct */
    option->mode = (pp_options1 >> 56) & 0xF;
    option->emulation_en = (pp_options1 >> 62) & 1;
    option->val1 = (pp_options1 >> 32) & 0xFFFF;
    option->val2 = (pp_options1 >> 16) & 0xFFFF;
    option->cval = pp_options1 & 0x1;

    // TODO(nc): unmagicify
    status = ifc_xuser_tcsr_read(ifcdevice, 0x71, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->idle2pre = (i32_reg_val >> block) & 1;

    status = ifc_xuser_tcsr_read(ifcdevice, 0x75, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->pre2run = (i32_reg_val >> block) & 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_conf_digital_pp(struct ifcdaqdrv_usr *ifcuser,
                                           uint32_t block,
                                           uint32_t write_mask,
                                           struct ifcfastint_digital_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val;
    uint64_t active = 0;
    uint32_t fpga_mem_address = 0;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(block >= 32) {
        return status_argument_range;
    }

    if(!option) {
        return status_argument_invalid;
    }

    // The PP block should always be active, otherwise we don't get any measurements...
    //if(option->mode > 0 && option->mode <= 3) {
    //    active =  1;
    //}
    active = 1;

    /* Modify the block specified */
    //pp_options =
    //    1LL << 63 | /* ACT */
    //    (uint64_t)emulation_en << 62 | /* EMUL */
    //    (uint64_t)(mode & 0xF) << 56 | /* MODE */
    //    (uint64_t)(val1 & 0xFFFF) << 32 |
    //    (uint64_t)(val2 & 0xFFFF) << 16 |
    //    (cval & 0xFFFF);

    /* Write the configuration block. */

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, block*8, &pp_options);
    if(write_mask & IFCFASTINT_DIGITAL_MODE_W) {
        pp_options = u64_setclr(pp_options, active, 1, 63);
        pp_options = u64_setclr(pp_options, option->mode, 0xF, 56);
    }
    if(write_mask & IFCFASTINT_DIGITAL_EMULATION_EN_W) {
        pp_options = u64_setclr(pp_options, option->emulation_en, 1, 62);
    }
    if(write_mask & IFCFASTINT_DIGITAL_VAL1_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->val1, 0xFFFF, 16);
    }
    if(write_mask & IFCFASTINT_DIGITAL_VAL2_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->val2, 0xFFFF, 32);
    }
    if(write_mask & IFCFASTINT_DIGITAL_CVAL_W) {
        pp_options = u64_setclr(pp_options, option->cval, 1, 0);
    }

    fpga_mem_address = 0x100 + block*8;
    status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }


    // TODO(nc): unmagicify
    if(write_mask & IFCFASTINT_DIGITAL_IDLE2PRE_W) {
        i32_reg_val = option->idle2pre << block;
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x71, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x73, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
    }

    if(write_mask & IFCFASTINT_DIGITAL_PRE2RUN_W) {
        i32_reg_val = option->pre2run << block;
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x75, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        status = ifc_xuser_tcsr_setclr(ifcdevice, 0x77, i32_reg_val, 1 << block);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

// FASTINT V2 verified
ifcdaqdrv_status ifcfastint_conf_lock(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Clear bits 18/19 of register 64, disabling dynamic configuration */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
                          0,
                          1 << IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT |
                          1 << IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_SHIFT);
    pthread_mutex_unlock(&ifcdevice->lock);

    if(status) {
        return status_internal;
    }

    return status_success;
}

// FASTINT V2 verified
ifcdaqdrv_status ifcfastint_conf_unlock(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Sets bits 18/19 of register 64, enabling configuration of PP options */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG,
                          1 << IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT |
                          1 << IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_SHIFT,
                          0);

    pthread_mutex_unlock(&ifcdevice->lock);

    if(status) {
        return status_internal;
    }

    return status_success;
}

ifcdaqdrv_status ifcfastint_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency, uint32_t fmc)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
    	pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if (ifcdevice->set_clock_frequency) {
        fmc_aux = ifcdevice->fmc;
        ifcdevice->fmc = fmc;
        status = ifcdevice->set_clock_frequency(ifcdevice, frequency);
        ifcdevice->fmc = fmc_aux;
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}

ifcdaqdrv_status ifcfastint_get_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double *frequency, uint32_t fmc)
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->get_clock_frequency) {
    	return status_no_support;
    }

    if (!frequency) {
        return status_argument_invalid;
    }

    fmc_aux = ifcdevice->fmc;
    ifcdevice->fmc = fmc;

    status = ifcdevice->get_clock_frequency(ifcdevice, frequency);

    ifcdevice->fmc = fmc_aux;
    return status;
}

ifcdaqdrv_status ifcfastint_set_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t divisor, uint32_t fmc)
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(divisor < ifcdevice->divisor_min || divisor > ifcdevice->divisor_max) {
        return status_argument_range;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

    if (ifcdevice->set_clock_divisor) {

    	fmc_aux = ifcdevice->fmc;
        ifcdevice->fmc = fmc;
    	status = ifcdevice->set_clock_divisor(ifcdevice, divisor);
    	ifcdevice->fmc = fmc_aux;

        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_no_support;
}


ifcdaqdrv_status ifcfastint_get_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor, uint32_t fmc)
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!divisor) {
        return status_argument_invalid;
    }

    if (!ifcdevice->get_clock_divisor) {
        *divisor = 1;
        return status_success;
    }

	fmc_aux = ifcdevice->fmc;
    ifcdevice->fmc = fmc;
    status = ifcdevice->get_clock_divisor(ifcdevice, divisor);
    ifcdevice->fmc = fmc_aux;

    return status;
}

ifcdaqdrv_status ifcfastint_set_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock clock, uint32_t fmc)
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->set_clock_source) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    if (ifcdevice->armed) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_device_armed;
    }

	fmc_aux = ifcdevice->fmc;
    ifcdevice->fmc = fmc;
    status = ifcdevice->set_clock_source(ifcdevice, clock);
    ifcdevice->fmc = fmc_aux;

    pthread_mutex_unlock(&ifcdevice->lock);

    return status;

}


ifcdaqdrv_status ifcfastint_get_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock *clock, uint32_t fmc)
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
    uint32_t			 fmc_aux;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdevice->get_clock_source) {
        return status_no_support;
    }

    fmc_aux = ifcdevice->fmc;
    ifcdevice->fmc = fmc;
    status = ifcdevice->get_clock_source(ifcdevice, clock);
    ifcdevice->fmc = fmc_aux;

    return status;
}

ifcdaqdrv_status ifcfastint_set_history_mode(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histmode hist_mode)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    // Clear bits 14:12
    i32_reg_val &= ~(IFCFASTINT_FSM_MAN_HISTORY_MODE_MASK);

    // Set the new mode - the enum should correspond to the regmap
    i32_reg_val |= ((int32_t)hist_mode & 0x07) << IFCFASTINT_FSM_MAN_HISTORY_MODE_SHIFT;

    printf("[ifcdaqdrv - set history mode] Writing 0x%08x to register 0x64\n", i32_reg_val);

    //write register 0x64
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_FSM_MAN_REG, i32_reg_val);

    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}
ifcdaqdrv_status ifcfastint_get_history_mode(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histmode *hist_mode)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    i32_reg_val = (i32_reg_val & IFCFASTINT_FSM_MAN_HISTORY_MODE_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_MODE_SHIFT;
    switch (i32_reg_val)
    {
    	case 0:
    		*hist_mode = ifcfastint_histmode_0;
    		break;
    	case 1:
    		*hist_mode = ifcfastint_histmode_1;
    		break;
    	case 2:
    		*hist_mode = ifcfastint_histmode_2;
    		break;
    	case 3:
    		*hist_mode = ifcfastint_histmode_3;
    		break;
    	case 4:
    		*hist_mode = ifcfastint_histmode_4;
    		break;
    	case 5:
    		*hist_mode = ifcfastint_histmode_5;
    		break;
    	case 6:
    		*hist_mode = ifcfastint_histmode_6;
    		break;
    	case 7:
    		*hist_mode = ifcfastint_histmode_7;
    		break;
    	default:
    		*hist_mode = ifcfastint_histmode_0;
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_history_status(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histcontrol hist_enabled)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Enable/disable bit 15 on register 0x64
    if (hist_enabled == ifcfastint_history_disabled)
    	status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0, 1<<IFCFASTINT_FSM_MAN_HISTORY_ENA_SHIFT);
    else
    	status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 1<<IFCFASTINT_FSM_MAN_HISTORY_ENA_SHIFT, 0);

    pthread_mutex_unlock(&ifcdevice->lock);
	return status;
}
ifcdaqdrv_status ifcfastint_get_history_status(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histcontrol *hist_enabled)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    i32_reg_val = (i32_reg_val & IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_ENA_SHIFT;

    if (i32_reg_val)
    	*hist_enabled = ifcfastint_history_enabled;
    else
    	*hist_enabled = ifcfastint_history_disabled;

    pthread_mutex_unlock(&ifcdevice->lock);
	return status_success;
}

ifcdaqdrv_status ifcfastint_get_history_flags(struct ifcdaqdrv_usr *ifcuser, int32_t *over_ff, int32_t *ring_over_ff)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    *over_ff = (i32_reg_val & IFCFASTINT_FSM_MAN_HISTORY_OVER_FF_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_OVER_FF_SHIFT;
    *ring_over_ff = (i32_reg_val & IFCFASTINT_FSM_MAN_HISTORY_RING_OVER_FF_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_RING_OVER_FF_SHIFT;

    pthread_mutex_unlock(&ifcdevice->lock);
	return status_success;
}
ifcdaqdrv_status ifcfastint_get_history_acqstate(struct ifcdaqdrv_usr *ifcuser, ifcfastint_hist_state *state)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    i32_reg_val = (i32_reg_val & IFCFASTINT_FSM_MAN_HISTORY_STATUS_MASK) >> IFCFASTINT_FSM_MAN_HISTORY_STATUS_SHIFT;
    *state = (ifcfastint_hist_state) i32_reg_val;

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_get_statusreg(struct ifcdaqdrv_usr *ifcuser, int32_t *regval)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    // Get status register
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    *regval = i32_reg_val;
    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}

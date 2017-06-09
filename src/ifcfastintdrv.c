#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#include <ifcdaqdrv2.h>

#include "debug.h"
#include "ifcdaqdrv_utils.h"
#include "ifcfastintdrv.h"
#include "ifcfastintdrv_utils.h"
/* Oliver adc3110 -> adc3117 (unused?)*/
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
    intptr_t buf_start = 0x00100000;
    intptr_t buf_end   = 0x00300000;
    i32_reg_val = buf_end + (buf_start >> 16);

    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_SIZE_REG, i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, buf_start + 64); // First frame is special...
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, buf_start + 64); // First frame is special...
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

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    do {
        status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }
        usleep(ifcdevice->poll_period);
    } while(((i32_reg_val >> 8) & 0x3) != 3);

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
                                               struct ifcfastint_analog_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val = 0;

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
     * The analog values are 0x100 bytes offset.
     */

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, 0x100 + block*8, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    option->mode = (pp_options >> 56) & 0xF;
    option->emulation_en = (pp_options >> 62) & 1;
    option->val1 = (pp_options >> 16) & 0xFFFF;
    option->val2 = (pp_options >> 32) & 0xFFFF;
    option->cval = pp_options & 0xFFFF;

    status = ifcfastintdrv_read_pp_conf(ifcdevice, 0x300 + block*8, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    option->val3 = pp_options & 0xFFFF;
    option->val4 = (pp_options >> 32) & 0xFFFF;

    // TODO(nc): unmagicify
    status = ifc_xuser_tcsr_read(ifcdevice, 0x70, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->idle2pre = (i32_reg_val >> block) & 1;

    status = ifc_xuser_tcsr_read(ifcdevice, 0x74, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    option->pre2run |= (i32_reg_val >> block) & 1;
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

ifcdaqdrv_status ifcfastint_set_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
                                               struct ifcfastint_analog_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val;
    uint64_t active = 0;

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
    //if(option->mode > 0 && option->mode <= 12) {
    //    active =  1;
    //}
    active = 1;

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, 0x100 + block*8, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }
    if(write_mask & IFCFASTINT_ANALOG_MODE_W) {
        pp_options = u64_setclr(pp_options, active, 1, 63);
        pp_options = u64_setclr(pp_options, option->mode, 0xF, 56);
    }
    if(write_mask & IFCFASTINT_ANALOG_EMULATION_EN_W) {
        pp_options = u64_setclr(pp_options, option->emulation_en, 1, 62);
    }
    if(write_mask & IFCFASTINT_ANALOG_VAL1_W) {
        // Cast to avoid sign extension
        pp_options = u64_setclr(pp_options, (uint16_t)option->val1, 0xFFFF, 16);
    }
    if(write_mask & IFCFASTINT_ANALOG_VAL2_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->val2, 0xFFFF, 32);
    }
    if(write_mask & IFCFASTINT_ANALOG_CVAL_W) {
        pp_options = u64_setclr(pp_options, (uint16_t)option->cval, 0xFFFF, 0);
    }
    /*
     * Return write error if configuration is locked.
     * A bit value of 1 indicates that it is unlocked.
     */
    //status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_FSM_MAN_REG, &i32_reg_val);
    //if(!(i32_reg_val & (1 << IFCFASTINT_FSM_MAN_DYN_ANALOG_OPT_ENA_SHIFT |
    //                  1 << IFCFASTINT_FSM_MAN_DYN_DIGITAL_OPT_ENA_SHIFT))){
    //    pthread_mutex_unlock(&ifcdevice->lock);
    //    return status_write;
    //}
    status = ifcfastintdrv_write_pp_conf(ifcdevice, 0x100 + block*8, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    status = ifcfastintdrv_read_pp_conf(ifcdevice, 0x300 + block*8, &pp_options);
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

    status = ifcfastintdrv_write_pp_conf(ifcdevice, 0x300 + block*8, pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

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

ifcdaqdrv_status ifcfastint_get_conf_digital_pp(struct ifcdaqdrv_usr *ifcuser,
                                           uint32_t block,
                                           struct ifcfastint_digital_option *option) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    int32_t i32_reg_val;

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
     * The analog values are 0x100 bytes offset.
     */

    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_pp_conf(ifcdevice, block*8, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    option->mode = (pp_options >> 56) & 0xF;
    option->emulation_en = (pp_options >> 62) & 1;
    option->val1 = (pp_options >> 32) & 0xFFFF;
    option->val2 = (pp_options >> 16) & 0xFFFF;
    option->cval = pp_options & 0x1;

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
    status = ifcfastintdrv_write_pp_conf(ifcdevice, block*8, pp_options);
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

ifcdaqdrv_status ifcfastint_conf_lock(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

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

ifcdaqdrv_status ifcfastint_conf_unlock(struct ifcdaqdrv_usr *ifcuser) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

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

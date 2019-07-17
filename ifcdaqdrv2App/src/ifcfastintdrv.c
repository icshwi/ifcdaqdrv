#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unistd.h>

#include "tscioctl.h"
#include "tsculib.h"

#include <ifcdaqdrv2.h>
#include "ifcdaqdrv_dio3118.h"

#include "debug.h"
#include "ifcdaqdrv_utils.h"
#include "ifcfastintdrv2.h"
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

    /* Turn off history buffer just to configure it */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_FSM_MAN_REG, 0, IFCFASTINT_FSM_MAN_HISTORY_ENA_MASK);

    /* Setup the size of the ring buffer */
    
    intptr_t buf_start = 0x00000000;
    intptr_t buf_end   = 0xF<<20;//0x0ff00000; //max allowed = 255 MB

    i32_reg_val = buf_end + (buf_start >> 16);

    /* if running on BIG endian machine, enable the firmware option to record the history buffer
	 * as big endian. It will then swap the 64 bytes frame on a 4 byte basis, causing the 16-bit
	 * analog channels neigbours to be inverted.
	 *
	 * |-------LITTE ENDIAN--------|--------|-------- BIG ENDIAN---------|
	 * |ch0_b0|ch0_b1|ch1_b0|ch1_b1| ---->> |ch1_b1||ch1_b0|ch0_b1|ch0_b0| 
	 *
     */

    if (ifcdaqdrv_is_byte_order_ppc()) {
	    /* Configure the history frame to be saved as BIG ENDIAN */
	    i32_reg_val |= IFCFASTINT_BUF_BIGENDIAN_MASK;   
	}

    /* Write to register 0x68 */
    status = ifc_xuser_tcsr_write(ifcdevice, IFCFASTINT_BUF_SIZE_REG, i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    /* Read current WRITE POINTER location */
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, &i32_reg_val); 
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

    /* Disable/Enable history buffer to produce a reset and put it into RUNNING state*/
    ifcfastintdrv_history_reset(ifcdevice);

    /* Enable QOUT memorization when interlock occurs */
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

    /* TESTING: enable bit 0 of the timing input mask (register 6C) */
    status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_TIMING_CTL, 0x01, 0x00);
    if(status) {
	pthread_mutex_unlock(&ifcdevice->lock);
	return status_internal;
    }

    /* Basic initialization is done - current STATE of the main state machine is preserved */
    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

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

    i32_reg_val &= 0x3fffffff; // CLEANING TST AND ENDIAN BITS!!!!

    buf_start = ((i32_reg_val & 0xFFFF) << 16) + 64; // First history slot is special
    buf_end = (i32_reg_val & 0xFFFF0000); // This points to first item *after* the buffer
    buf_size = buf_end - buf_start;

    /**************************************************************************/
    /* Current RFLPS FIM firmware is using only 512 kB of internal FPGA memory*/
    /* buffer boundaries are FORCED to 0x0000 ---> 0x80000                     */
    /**************************************************************************/

    //buf_start = 0; // First history slot is special
    //buf_end = 0x80000;
    //buf_size = buf_end - buf_start;

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
    //content_end = i32_reg_val - 64;
    content_end = i32_reg_val;
    if(content_end < buf_start) {
            content_end += buf_size;
    }

    /* Read 64 bytes from SMEM */
    size_t size = 64;
    content_start = content_end - size;


    ifcfastintdrv_read_smem_historybuffer(
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

ifcdaqdrv_status ifcfastint_read_history(struct ifcdaqdrv_usr *ifcuser, size_t count, void *data, size_t *nelm, double *wrpointer) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    int32_t i32_reg_val;
    size_t size;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!nelm || !data) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    // Get circular buffer size
    // Top 16 bits are buffer end pointer, lower 16 bits are buffer start pointer.
    uint32_t buf_size;
    uint32_t buf_start;
    uint32_t buf_end;
    
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_SIZE_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    

    buf_start = (((uint32_t)i32_reg_val & 0x0FF0) << 16); // MBytes granularity
    buf_end = ((uint32_t)i32_reg_val & 0x0FF00000); // This points to first item *after* the buffer
    buf_size = buf_end - buf_start;

    uint32_t content_start;
    uint32_t content_end;
    
    // Store read pointer in `content_start`
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_R_PTR_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
    content_start = (uint32_t) i32_reg_val;

    /* IFDEF to select from where to read the ring buffer: randomly or from the latched write pointer */


#if 1 
    //Now the write pointer is triggered by timing
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_TIM, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }

    /* THIS IS TO BE EXPOSED BY EPICS !!! */
    *wrpointer = (double) i32_reg_val;
#endif

#if 0 // randomly
    // Store write pointer in `content_end`
    status = ifc_xuser_tcsr_read(ifcdevice, IFCFASTINT_BUF_W_PTR_REG, &i32_reg_val);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status;
    }
#endif

    /*
     * Never read out the last item, it will make hardware think it overflowed.
     * W_PTR points to "next empty slot". Which means that we need to back it 2 steps.
     */

    content_end = (uint32_t) i32_reg_val - 2*64;
    if(content_end < buf_start) {
            content_end += buf_size;
    }


    if(content_end < buf_start || content_start < buf_start) {
        //LOG((LEVEL_ERROR, "bs: 0x%08" PRIxPTR " be: 0x%08" PRIxPTR " cs: 0x%08" PRIxPTR " ce: 0x%08" PRIxPTR "\n", buf_start, buf_end, content_start, content_end));
        //LOG((LEVEL_ERROR, "%s\n", "Internal error.. Content pointers are outside buffer area."));
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

    /* Guard for number of frames */
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

    // Bail early if 0 frames was found
    if(!size) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_success;
    }

    // New content_start. `content_start` might end up outside the buffer
    //content_start = content_end - size;
    if(content_end > buf_start + size) {
        ifcfastintdrv_read_smem_historybuffer(
                ifcdevice,
                data,
                ifcdevice->smem_dma_buf,
                (content_end - size),
                size
        );
    } else {
        //content_start is outside the buffer, adjust it to the current size
        content_start = buf_end - (size - (content_end - buf_start)); 
        ifcfastintdrv_read_smem_historybuffer(
                ifcdevice,
                data,
                ifcdevice->smem_dma_buf,
                content_start,
                buf_end - content_start
        );
        ifcfastintdrv_read_smem_historybuffer(
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

ifcdaqdrv_status ifcfastint_read_measurements(struct ifcdaqdrv_usr *ifcuser, void *data) {
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if(!data) {
        return status_argument_invalid;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    // Read 400 kB, from 0x101200 to 0x101600
    status = ifcfastintdrv_read_sram_measurements(
            ifcdevice,
            data,
            IFCFASTINT_SRAM_PP_MEASURE
    );
   
    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
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

ifcdaqdrv_status ifcfastint_history_reset(struct ifcdaqdrv_usr *ifcuser) {
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    /* Clear and enable history */
    ifcfastintdrv_history_reset(ifcdevice);

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

    /*
    * IDLE and ABO are states that corresponds to STDBY state on the SIM 
    * PRE corresponds to HV ON
    * RUN corresponds to RF ON
    *
    */

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

    pthread_mutex_lock(&ifcdevice->lock);

    if ((ppblock != ifcfastint_analog_pp_channel)&&(ppblock != ifcfastint_analog_pp_pwravg))
    {
    	/* Read the configuration (OPTION1) of the standard analog input pre-processing block. */
    	fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + block*8; // OPTION1 registers starts at 0x200

    	usleep(1000);
        /* Read first time */
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
    usleep(1000);
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
    usleep(1000);
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
    uint64_t pp_options_rb = 0;
    int32_t i32_reg_val;
    const uint64_t active = 1;
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

    pthread_mutex_lock(&ifcdevice->lock);

    if ((ppblock != ifcfastint_analog_pp_channel)&&(ppblock != ifcfastint_analog_pp_pwravg))
    {
		/* TMEM4_1 address */
		fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + block*8; // OPTION1 registers starts at 0x200

		/* Read current configuration */	
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

        // Force autorun
        pp_options = u64_setclr(pp_options, 1, 1, 61);

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

    /* Check if block was correctly written */
    int max_atempts = 10;
    for (; max_atempts > 0; max_atempts--)
    {
        /* Write attempt */
        status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        /* Read back attempt */
        status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options_rb);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        if (pp_options_rb == pp_options) {
            break;
        }
    }

    if (max_atempts == 0) {
        INFOLOG(("Failed WRITE ANALOG PP BLOCK operation at address 0x%08x\n", fpga_mem_address));
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

ifcdaqdrv_status ifcfastint_set_conf_pwravg_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
											   struct ifcfastint_pwravg_option *option)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint64_t pp_options = 0;
    uint64_t pp_options_rb = 0;
    int32_t i32_reg_val;
    const uint64_t active = 1;
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

    /* Check if block was correctly written */
    fpga_mem_address = 0x600 + (block*16);

    /* WORK AROUND TO READ/WRITE OPERATIONS */
    int max_atempts = 10;
    for (; max_atempts > 0; max_atempts--)
    {
        status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options_rb);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        if (pp_options_rb == pp_options) {
            break;
        }
    }

    if (max_atempts == 0) {
        INFOLOG(("Failed WRITE PWRAVG PP BLOCK operation at address 0x%08x\n", fpga_mem_address));
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

    /* Check if block was correctly written */
    for (max_atempts=10; max_atempts > 0; max_atempts--)
    {
        status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options_rb);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        if (pp_options_rb == pp_options) {
            break;
        }
    }

    if (max_atempts == 0) {
        INFOLOG(("Failed WRITE PWRAVG PP BLOCK operation at address 0x%08x\n", fpga_mem_address));
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

    /* DIO3118 has only 16 digital inputs */
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
    usleep(1000);
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
    uint64_t pp_options_rb = 0;
    int32_t i32_reg_val;
    const uint64_t active = 1;
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

    /* Write the configuration block. */
    pthread_mutex_lock(&ifcdevice->lock);

    fpga_mem_address = 0x100 + block*8;
    status = ifcfastintdrv_read_pp_conf(ifcdevice, fpga_mem_address, &pp_options);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        return status_internal;
    }

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

    /* Check if block was correctly written */
    int max_atempts = 5;
    for (; max_atempts > 0; max_atempts--)
    {

        status = ifcfastintdrv_write_pp_conf(ifcdevice, fpga_mem_address, pp_options);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        status = ifcfastintdrv_read_pp_conf(ifcdevice, block*8, &pp_options_rb);
        if(status) {
            pthread_mutex_unlock(&ifcdevice->lock);
            return status_internal;
        }

        if (pp_options_rb == pp_options) {
            break;
        }
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
    		*hist_mode = ifcfastint_histmode_4;
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


/* Function to read RT_Status implementation - not yet tested */
ifcdaqdrv_status ifcfastint_get_rtstatus(struct ifcdaqdrv_usr *ifcuser,
                                         uint32_t aichannel,
                                         uint32_t *value,
                                         ifcfastint_analog_pp analog_pp_type)
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    uint32_t fpga_mem_address;
    uint64_t rt_status_result;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    /* Current firmware supports only 4 pwravg blocks */
    if(aichannel >= 16) {
        return status_argument_range;
    }

    if(!value) {
        return status_argument_invalid;
    }

    /* Currently only supports PULSE WIDTH and PULSE RATE */
    if ((analog_pp_type != ifcfastint_analog_pp_pulshp) && (analog_pp_type != ifcfastint_analog_pp_pulrate))
    {
        return status_argument_invalid;   
    }

    /* Sets the offset address on TMEM4_1 that holds OPTION1 configuration */
    if (analog_pp_type == ifcfastint_analog_pp_pulshp)
        fpga_mem_address = (aichannel*8);

    if (analog_pp_type == ifcfastint_analog_pp_pulrate)
        fpga_mem_address = 0x100 + (aichannel*8);
    
    pthread_mutex_lock(&ifcdevice->lock);
    status = ifcfastintdrv_read_rtstatus(ifcdevice, fpga_mem_address, &rt_status_result);
    if(status) {
        pthread_mutex_unlock(&ifcdevice->lock);
        *value = 0;
        return status_internal;
    }

    /* Extracts the desired value  */
    if (analog_pp_type == ifcfastint_analog_pp_pulshp){
        *value = (uint32_t) ((rt_status_result & 0x0000FFFF00000000) >> 32);
    }

    if (analog_pp_type == ifcfastint_analog_pp_pulrate){
        *value = (uint32_t) ((rt_status_result & 0x00FFFFFF00000000) >> 32);
    }

    pthread_mutex_unlock(&ifcdevice->lock);
    return status_success;
}

/* New function necessary to RFLPS MicroTCA version */
ifcdaqdrv_status ifcfastint_init_dio3118(struct ifcdaqdrv_usr *ifcuser)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);
   
    /* Activate FMC Write 0x3118 in the signature register -> 0x03000050*/
    ifc_tcsr_write(ifcdevice, 0x1000, 0xC0, 0x31180000);
    usleep(1000);

    ifc_tcsr_write(ifcdevice, 0x1000, 0xC1, 0x03000050);

    /* output enables */
    ifc_tcsr_write(ifcdevice, 0x1000, 0xC9, 0x001fffff);

    pthread_mutex_unlock(&ifcdevice->lock); 

    return status_success;
}

/****************************************************************/

ifcdaqdrv_status ifcfastint_set_eeprom_param(struct ifcdaqdrv_usr *ifcuser, int channel, ifcfastint_aichannel_param aiparam, double value)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdaqdrv_is_byte_order_ppc()) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);
    ifcfastintdrv_eeprom_write(ifcdevice, aiparam, channel, value);
    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}

ifcdaqdrv_status ifcfastint_get_eeprom_param(struct ifcdaqdrv_usr *ifcuser, int channel, ifcfastint_aichannel_param aiparam, double *value)
{
    struct ifcdaqdrv_dev *ifcdevice;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    if (!ifcdaqdrv_is_byte_order_ppc()) {
        return status_no_support;
    }

    pthread_mutex_lock(&ifcdevice->lock);
    ifcfastintdrv_eeprom_read(ifcdevice, aiparam, channel, value);
    pthread_mutex_unlock(&ifcdevice->lock);

    return status_success;
}


ifcdaqdrv_status ifcfastint_get_diagnostics(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcfastint_analog_pp ppblock, struct ifcfastint_analog_diag *diag_info) 
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;
	uint64_t pp_status = 0;
	uint32_t fpga_mem_address = 0;

	ifcdevice = ifcuser->device;
	if (!ifcdevice) {
		return status_no_device;
	}

/* Current firmware supports only 20 analog inputs */
	if(channel >= 20) {
		return status_argument_range;
	}

	if(!diag_info) {
		return status_argument_invalid;
	}

	pthread_mutex_lock(&ifcdevice->lock);

	if ((ppblock != ifcfastint_analog_pp_channel)&&(ppblock != ifcfastint_analog_pp_pwravg))
	{
	/* Read the READ_OUT 64-bits register of the standard analog input pre-processing block. */
		fpga_mem_address = 0x200 + ((uint32_t)ppblock*0x100) + channel*8; // READ_OUT registers starts at 0x200

		usleep(500);
		/* Read first time */
		status = ifcfastintdrv_read_pp_status(ifcdevice, fpga_mem_address, &pp_status);
		if(status) {
			pthread_mutex_unlock(&ifcdevice->lock);
			return status_internal;
		}

		diag_info->process_out = (bool) (pp_status >> 63) & 1;
		diag_info->dev_val = 0;
		diag_info->trig_dev_val = 0;
		diag_info->pres_val = 0;
		diag_info->trig_val = 0; 

		switch(ppblock) {
			case ifcfastint_analog_pp_lvlmon:
				diag_info->pres_val = (pp_status >> 16) & 0xFFFF;
				diag_info->trig_val = pp_status & 0xFFFF; 
				break;

			case ifcfastint_analog_pp_pulshp:
			case ifcfastint_analog_pp_pulrate:
				diag_info->pres_val = (pp_status >> 32) & 0xFFFFFF;
				diag_info->trig_val = pp_status & 0xFFFFFF; 
				break;

			case ifcfastint_analog_pp_devmon:
				diag_info->dev_val = (pp_status >> 16) & 0xFFFF;
				diag_info->trig_dev_val = pp_status & 0xFFFF;
				break;

			default:
				break;

		}
	}
	
	pthread_mutex_unlock(&ifcdevice->lock);
	return status_success;
}

ifcdaqdrv_status ifcfastint_set_timingmask(struct ifcdaqdrv_usr *ifcuser, uint32_t mask) 
{
	ifcdaqdrv_status      status;
	struct ifcdaqdrv_dev *ifcdevice;

	ifcdevice = ifcuser->device;
	if (!ifcdevice) {
		return status_no_device;
	}

	pthread_mutex_lock(&ifcdevice->lock);

	mask = 0x00000001;
	
        status = ifc_xuser_tcsr_setclr(ifcdevice, IFCFASTINT_TIMING_CTL, 0x01, 0x00);
	if(status) {
		pthread_mutex_unlock(&ifcdevice->lock);
		return status_internal;
	}

	pthread_mutex_unlock(&ifcdevice->lock);
	return status_success;
}

ifcdaqdrv_status ifcfastint_subs_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    struct tsc_ioctl_user_irq tscirq;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    pthread_mutex_lock(&ifcdevice->lock);

    tscirq.irq  = irqn;
    tscirq.mask = 1 <<irqn;
    tscirq.wait_mode = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4);

    status = tsc_user_irq_subscribe(ifcdevice->node, &tscirq);

    pthread_mutex_unlock(&ifcdevice->lock);
    return status;
}


ifcdaqdrv_status ifcfastint_wait_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn) 
{
    ifcdaqdrv_status      status;
    struct ifcdaqdrv_dev *ifcdevice;
    struct tsc_ioctl_user_irq tscirq;

    ifcdevice = ifcuser->device;
    if (!ifcdevice) {
        return status_no_device;
    }

    tscirq.irq  = irqn;
    tscirq.mask = 1 <<irqn;
    tscirq.wait_mode = DMA_WAIT_INTR | DMA_WAIT_1S | (5 << 4);

    status = tsc_user_irq_wait(ifcdevice->node, &tscirq);    

    return status;
}

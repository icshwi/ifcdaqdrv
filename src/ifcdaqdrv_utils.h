#ifndef _IFC1210SCOPEDRV_UTILS_H_
#define _IFC1210SCOPEDRV_UTILS_H_ 1

#include <pthread.h>

#include "ifcdaqdrv_list.h"

#define MAX_PEV_CARDS 16
#define MAX_DECIMATIONS 20

#define TCSR_ACCESS_ADJUST 0x80000000
#define OFFSET_XUSER_CSR 0x1000

#define min(a, b) \
    ({ __typeof__ (a)_a = (a); \
       __typeof__ (b)_b = (b); \
       _a < _b ? _a : _b; })

#define max(a, b) \
    ({ __typeof__ (a)_a = (a); \
       __typeof__ (b)_b = (b); \
       _a > _b ? _a : _b; })

/**
 * Enumeration for Acquisition modes
 */

typedef enum {
    ifcdaqdrv_acq_mode_sram,
    ifcdaqdrv_acq_mode_smem
} ifcdaqdrv_acq_store_mode;

/**
 * @brief Enumeration for LED states
 */

typedef enum {
    ifcdaqdrv_led_off,
    ifcdaqdrv_led_color_green,
    ifcdaqdrv_led_color_red,
    ifcdaqdrv_led_blink_fast,
    ifcdaqdrv_led_blink_slow
} ifcdaqdrv_led_state;

/**
 * @brief Enumeration for LEDs
 *
 * The IFC has one led for each FMC which is controlled by the FMC. This is ifcdaqdrv_led_ifc. Furthermore an FMC may have
 * as many as 2 LEDs that can be individually controlled.
 *
 * The ADC3110 maps fmc0 to frontpanel led and fmc1 to rear side PCB led.
 * The DTACQ doesn't have any FMC led.
 */

typedef enum {
    ifcdaqdrv_led_ifc,       // FMC1 or FMC2 LED on IFC
    ifcdaqdrv_led_fmc0,
    ifcdaqdrv_led_fmc1
} ifcdaqdrv_led;

/* Definition of TOSCA structures when we don't have the driver  */
#ifndef TOSCA_USRLIB

typedef long dma_addr_t;

struct tsc_ioctl_kbuf_req
{
  uint size;
  void *k_base;
  dma_addr_t b_base;
  void *u_base;
};

struct tsc_ioctl_dma_req
{
  dma_addr_t src_addr;
  dma_addr_t des_addr;
  uint size;
  unsigned char src_space; unsigned char src_mode; unsigned char des_space; unsigned char des_mode;
  unsigned char start_mode; unsigned char end_mode; unsigned char intr_mode; unsigned char wait_mode;
  uint dma_status;
};


#define DMA_SPACE_PCIE       0x00
#define DMA_SPACE_SHM        0x02
#define DMA_SPACE_USR        0x03
#define DMA_SPACE_KBUF       0x08
#define DMA_SPACE_MASK       0x07

#define DMA_START_PIPE_NO    0x00
#define DMA_START_PIPE       0x01
#define DMA_START_CHAN(chan)  ((chan&1) << 4)
#define DMA_SPACE_WS      0x10
#define DMA_SPACE_DS      0x20
#define DMA_SPACE_QS      0x30

#define DMA_INTR_ENA      0x01

#define DMA_WAIT_INTR     0x01
#define DMA_WAIT_1MS      0x02
#define DMA_WAIT_10MS     0x04
#define DMA_WAIT_100MS    0x06
#define DMA_WAIT_1S       0x08
#define DMA_WAIT_10S      0x0a
#define DMA_WAIT_100S     0x0c

#define DMA_PCIE_TC0      0x00  /* Traffic Class 0 */
#define DMA_PCIE_TC1      0x01  /* Traffic Class 1 */
#define DMA_PCIE_TC2      0x02  /* Traffic Class 2 */
#define DMA_PCIE_TC3      0x03  /* Traffic Class 3 */
#define DMA_PCIE_TC4      0x04  /* Traffic Class 4 */
#define DMA_PCIE_TC5      0x05  /* Traffic Class 5 */
#define DMA_PCIE_TC6      0x06  /* Traffic Class 6 */
#define DMA_PCIE_TC7      0x07  /* Traffic Class 7 */
#define DMA_PCIE_RR1      0x00  /* 1 outstanding read request */
#define DMA_PCIE_RR2      0x10  /* 2 outstanding read request */
#define DMA_PCIE_RR3      0x20  /* 3 outstanding read request */

#define DMA_SIZE_PKT_128  0x00000000 
#define DMA_SIZE_PKT_256  0x40000000 
#define DMA_SIZE_PKT_512  0x80000000 
#define DMA_SIZE_PKT_1K   0xc0000000 

#define DMA_STATE_IDLE          0x00
#define DMA_STATE_ALLOCATED     0x01
#define DMA_STATE_STARTED       0x02
#define DMA_STATE_WAITING       0x03
#define DMA_STATE_DONE          0x04

#define DMA_STATUS_RUN_RD0             0x01
#define DMA_STATUS_RUN_RD1             0x02
#define DMA_STATUS_RUN_WR0             0x04
#define DMA_STATUS_RUN_WR1             0x08
#define DMA_STATUS_DONE                0x10
#define DMA_STATUS_WAITING             0x000
#define DMA_STATUS_ENDED               0x100
#define DMA_STATUS_TMO                 0x80
#define DMA_STATUS_ERR                 0x40
#define DMA_STATUS_BUSY                0x20

/* definitions created to enable compilation
 * THESE ARE NOT ON TSCIOCTL.H !!!
 */
#define DMA_SPACE_USR1        0x04
#define DMA_SPACE_USR2        0x05
#define DMA_MODE_PIPE         0x01

#endif


/**
 * @brief Device private struct.
 *
 * Contains objects necessary for device access serialization,
 * end of acquisition signaling, device bookkeeping and status information.
 */
struct ifcdaqdrv_dev {
    struct list_head   list;            /**< Entry in the list of opened devices, sis8300drv_devlist. */
    uint32_t           card;            /**< Card/Crate number selected by rotational on-board switch. */
    uint32_t           fmc;             /**< FMC slot, 1 or 2. */
    //struct pevx_node  *node;            /**< PEV node structure */
    int                node;
    int                count;           /**< Number of times this device has been opened. */
    uint32_t           init_called;     /**< Positive if init_adc has been called. */
    uint32_t           ch_enabled;      /**< bitmask of enabled channels */

    struct fmc_fru_id *fru_id;
    uint32_t           tosca_signature; /**< Device type */
    uint32_t           app_signature;   /**< App type */

    int                (*init_adc)(struct ifcdaqdrv_dev *ifcdevice);
    int                (*get_signature)(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                        uint16_t *board_id);
    int                (*set_led)(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state color);
    int                (*set_dataformat)(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat);
    int                (*set_pattern)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern pattern);
    int                (*get_pattern)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern *pattern);

    int                (*set_gain)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain);
    int                (*get_gain)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain);

    int                (*set_nsamples)(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);
    int                (*get_nsamples)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples);

    int                (*set_trigger_threshold)(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
    int                (*get_trigger_threshold)(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);
    int                (*set_clock_frequency)(struct ifcdaqdrv_dev *ifcdevice, double frequency);
    int                (*get_clock_frequency)(struct ifcdaqdrv_dev *ifcdevice, double *frequency);
    int                (*set_clock_divisor)(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor);
    int                (*get_clock_divisor)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor);
    int                (*set_clock_source)(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock);
    int                (*get_clock_source)(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock);

    int                (*read_ai_ch)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *data); /* Put channel's data into data */
    int                (*read_ai)(struct ifcdaqdrv_dev *ifcdevice, void *data); /* Put all channels' data into data */

    int                (*normalize_ch)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                               size_t nelm); /* Convert raw channel data into standardized int32_t */
    int                (*normalize)(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset,
                               size_t nelm, size_t channel_nsamples); /* Convert all channels' raw data into standardized int32_t */

    int                (*mode_switch)(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_acq_store_mode mode);           /**< Switch between SRAM and SMEM */

    ifcdaqdrv_acq_store_mode mode;           /**< In which memory to store acquistition SRAM/SMEM */
    ifcdaqdrv_trigger_type   trigger_type;

    struct tsc_ioctl_kbuf_req    *sram_dma_buf;                /**< Buffer for SRAM DMA transfers */
    struct tsc_ioctl_kbuf_req    *smem_dma_buf;                /**< Buffer for SMEM DMA transfers */
    void                    *all_ch_buf;                  /**< Buffer to store raw SMEM data */

    uint32_t                 sample_size;                  /**< Sample size in bytes, TODO: Function pointer instead? */
    uint32_t                 nchannels;                    /**< Number of channels */
    uint32_t                 decimations[MAX_DECIMATIONS]; /**< hardare supported decimations */
    uint32_t                 averages[MAX_DECIMATIONS];    /**< hardare supported averages */

    uint32_t                 resolution;                   /**< ADC Resolution of FMC */
    double                   valid_clocks[MAX_DECIMATIONS];             /**< Clock frequencies supported by FMC. */
    uint32_t                 divisor_max;                  /**< Maximum clock divisor supported */
    uint32_t                 divisor_min;                  /**< Minimum clock divisor supported */

    uint32_t                 sample_resolution;            /**< Resolution of samples read out by read_ai */
    double                   vref_max;                     /**< Maximum measurable voltage */

    int      armed;                                        /**< Is device armed, 0 - not armed, 1 - armed. */
    uint32_t poll_period;                                  /**< Poll period in microseconds used when checking weather acquisition is finished. */

    uint32_t sram_size;                                    /**< Size of SRAM per channel in bytes  */
    uint32_t smem_size;                                    /**< Size of shared RAM in bytes (512/2 MB in IFC1210). */

    pthread_mutex_t lock;                                  /**< Lock that serializes access to the device. */
    pthread_mutex_t sub_lock;                              /**< Lock that serializes access to part of the device. */
};

inline static void setbit(uint32_t *val, int bitnr, int on){
    if (on) {
        *val |= (1 << bitnr);
    } else {
        *val &= ~(1 << bitnr);
    }
}

/* Functions for accessing any TCSR */

ifcdaqdrv_status ifc_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t value);
ifcdaqdrv_status ifc_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int offset, int register_idx, int32_t setmask, int32_t
                                 clrmask)
;

/* Functions for accessing any XUSER TCSR */

ifcdaqdrv_status ifc_xuser_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *value);
ifcdaqdrv_status ifc_xuser_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_xuser_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                       clrmask);

/* Functions for accessing SCOPE MAIN TCSR (0x60 to 0x6F) */

ifcdaqdrv_status ifc_scope_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                       clrmask);

/* Functions for accessing SCOPE ACQ TCSR  (0x70-0x73, 0x74-0x77, 0x78-0x7B, 0x7C-0x7F (SCOPE FMC1/FMC2 and SRAM/SMEM specific)) */

static inline int32_t ifc_get_scope_tcsr_offset(struct ifcdaqdrv_dev *ifcdevice) {
    if(ifcdevice->fmc == 1) {
        if(ifcdevice->mode == ifcdaqdrv_acq_mode_sram) {
            return 0x70;
        } else {
            return 0x78;
        }
    } else {
        if(ifcdevice->mode == ifcdaqdrv_acq_mode_sram){
            return 0x74;
        } else {
            return 0x7C;
        }
    }
}

ifcdaqdrv_status ifc_scope_acq_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *i32_reg_val);
ifcdaqdrv_status ifc_scope_acq_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_scope_acq_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                           clrmask);

/* Functions for accessing 0x80-0xBF or 0xC0-0xFF based on FMC1/FMC2. */

static inline int32_t ifc_get_fmc_tcsr_offset(struct ifcdaqdrv_dev *ifcdevice) {
    if(ifcdevice->fmc == 1) {
        return 0x80;
    } else {
        return 0xC0;
    }
}

ifcdaqdrv_status ifc_fmc_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *reg_val);
ifcdaqdrv_status ifc_fmc_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_fmc_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
                                     clrmask);

void ifcdaqdrv_free(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status ifcdaqdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcdaqdrv_read_sram_unlocked(struct ifcdaqdrv_dev *ifcdevice, struct tsc_ioctl_kbuf_req *dma_buf, uint32_t offset, uint32_t size);
ifcdaqdrv_status ifcdaqdrv_read_smem_unlocked(struct ifcdaqdrv_dev *ifcdevice, void *res, struct tsc_ioctl_kbuf_req *dma_buf, uint32_t offset, uint32_t size);
ifcdaqdrv_status ifcdaqdrv_get_sram_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address);
ifcdaqdrv_status ifcdaqdrv_get_smem_la(struct ifcdaqdrv_dev *ifcdevice, uint32_t *last_address);
// int ifcdaqdrv_get_sram_pretrig_size(struct ifcdaqdrv_dev *ifcdevice, int *size);
ifcdaqdrv_status ifcdaqdrv_set_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq);
ifcdaqdrv_status ifcdaqdrv_get_ptq(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq);
#endif // _IFC1210SCOPEDRV_UTILS_H_




#if 0
// Imported from old project

typedef enum {
    TMODE_STOP       = 0, // 1)stop pending trigger/acquisition
    TMODE_NORMAL     = 1, // 1)wait for trigger 2)Acquisition 3)trigger holdoff 4)goto(1)
    TMODE_AUTO       = 2, // 1)wait for trigger(max Xms) 2)Acquisition 3)trigger holdoff 4)goto(1)
    TMODE_SINGLESHOT = 3, // 1)wait for trigger 2)Acquisition 3)set mode TMODE_STOP
    TMODE_MANUAL     = 4  // 1)manual start Aquisition 2)Acquisition 3)set mode TMODE_STOP
} TRIGGERMODE;

typedef enum {
    TSLOPE_POSITIVE = 0x00000000,
    TSLOPE_NEGATIVE = 0x08000000
} TRIGGERSLOPE;

typedef enum {
    TLEVELMODE_RAW  = 0,
    TLEVELMODE_VOLT = 1
} TRIGGERLEVELMODE;
#endif

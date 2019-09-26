#ifndef _IFCDAQDRV_UTILS_H_
#define _IFCDAQDRV_UTILS_H_ 1

#include <pthread.h>
#include <linux/types.h>

#include "ifcdaqdrv_list.h"

#define MAX_CARDS 16
#define MAX_DECIMATIONS 20

#define TCSR_ACCESS_ADJUST 0x00000000
#define OFFSET_XUSER_CSR 0x1000

#define I2C_CTL_EXEC_IDLE 0x00000000
#define I2C_CTL_EXEC_RUN  0x00100000
#define I2C_CTL_EXEC_DONE 0x00200000
#define I2C_CTL_EXEC_ERR  0x00300000
#define I2C_CTL_EXEC_MASK 0x00300000

#define min(a, b) \
    ({ __typeof__ (a)_a = (a); \
       __typeof__ (b)_b = (b); \
       _a < _b ? _a : _b; })

#define max(a, b) \
    ({ __typeof__ (a)_a = (a); \
       __typeof__ (b)_b = (b); \
       _a > _b ? _a : _b; })

#define IFCDAQDRV_TOSCA_SIGNATURE_MASK  0xffff0000
#define IFCDAQDRV_APP_SIGNATURE_MASK    0xffff0000

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
 * The ADC3117 maps fmc0 to frontpanel green led and fmc1 to frontpanel red led.
 */

typedef enum {
    ifcdaqdrv_led_ifc,       // FMC1 or FMC2 LED on IFC
    ifcdaqdrv_led_fmc0,
    ifcdaqdrv_led_fmc1
} ifcdaqdrv_led;


/**
 * @brief Device private struct.
 *
 * Contains objects necessary for device access serialization,
 * end of acquisition signaling, device bookkeeping and status information.
 */
struct ifcdaqdrv_dev {
    struct list_head   list;            /**< Entry in the list of opened devices, ifcdaqdrv_devlist. */
    uint32_t           card;            /**< Card/Crate number selected by rotational on-board switch. */
    uint32_t           fmc;             /**< FMC slot, 1 or 2. */

    int                node;
    int                count;           /**< Number of times this device has been opened. */
    uint32_t           init_called;     /**< Positive if init_adc has been called. */
    uint32_t           ch_enabled;      /**< bitmask of enabled channels */

    struct fmc_fru_id *fru_id;
    uint32_t           tosca_signature; /**< Device type */
    uint32_t           app_signature;   /**< App type */
    uint16_t           board_id;        /**< FMC board id*/

    int                (*init_adc)(struct ifcdaqdrv_dev *ifcdevice);
    int                (*arm_device)(struct ifcdaqdrv_usr *ifcuser);
    int                (*disarm_device)(struct ifcdaqdrv_usr *ifcuser);
    int                (*wait_acq_end)(struct ifcdaqdrv_usr *ifcuser);
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

    int                (*set_adc_channel)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel); /* Select adc channel */
    int                (*get_adc_channel)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *channel); /* Get adc channel */
    int                (*set_adc_channel_mask)(struct ifcdaqdrv_dev *ifcdevice, uint32_t mask); /* Set adc channel mask */
    int                (*get_adc_channel_mask)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *mask); /* Get adc channel mask */
    int                (*set_adc_channel_negative_input)(struct ifcdaqdrv_dev *ifcdevice, uint8_t input);
    int                (*get_adc_channel_negative_input)(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input);
    int                (*set_adc_channel_positive_input)(struct ifcdaqdrv_dev *ifcdevice, uint8_t input);
    int                (*get_adc_channel_positive_input)(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input);
    int                (*set_offset)(struct ifcdaqdrv_dev *ifcdevice, uint16_t offset);
    int                (*get_offset)(struct ifcdaqdrv_dev *ifcdevice, uint16_t *offset);
    int                (*set_sample_rate)(struct ifcdaqdrv_dev *ifcdevice, double sample_rate);
    int                (*get_sample_rate)(struct ifcdaqdrv_dev *ifcdevice, double *sample_rate);
    int                (*calc_sample_rate)(struct ifcdaqdrv_usr *ifcuser, int32_t *averaging, int32_t *decimation, int32_t *divisor, double *freq, double *sample_rate, uint8_t sample_rate_changed);
    int                (*configuration_command)(struct ifcdaqdrv_dev *ifcdevice); /* Transfer configuration bits to all devices */

    int                (*set_trigger)(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold, uint32_t mask, uint32_t rising_edge);
    int                (*get_trigger)(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger, int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);
    int                (*set_average)(struct ifcdaqdrv_dev *ifcdevice, uint32_t average);
    int                (*get_average)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *average);
    int                (*set_decimation)(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation);
    int                (*get_decimation)(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation);
    int                (*set_ptq)(struct ifcdaqdrv_dev *ifcdevice, uint32_t ptq);
    int                (*get_ptq)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *ptq);

    /* Remove these */
    int                (*set_npretrig)(struct ifcdaqdrv_dev *ifcdevice, uint32_t npretrig);
    int                (*get_npretrig)(struct ifcdaqdrv_dev *ifcdevice, uint32_t *npretrig);

    ifcdaqdrv_acq_store_mode mode;           /**< In which memory to store acquistition SRAM/SMEM */
    ifcdaqdrv_trigger_type   trigger_type;

    struct tsc_ioctl_kbuf_req    *sram_dma_buf;                /**< Buffer for SRAM DMA transfers */
    struct tsc_ioctl_kbuf_req    *smem_dma_buf;                /**< Buffer for SMEM DMA transfers */
    void                         *all_ch_buf;                  /**< Buffer to store raw SMEM data */
    void                         *sram_blk_buf;                /* Buffer to store raw SRAM data */

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
    uint32_t smem_sg_dma;                                  /**< Set to use scatter gather DMA transfer */

    pthread_mutex_t lock;                                  /**< Lock that serializes access to the device. */
    pthread_mutex_t sub_lock;                              /**< Lock that serializes access to part of the device. */
    int                 (*set_digiout)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t value);
    int                 (*get_digiout)(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t *value);
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
                                 clrmask);

/* Functions for accessing any XUSER TCSR */
ifcdaqdrv_status ifc_xuser_tcsr_read(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t *value);
ifcdaqdrv_status ifc_xuser_tcsr_write(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t value);
ifcdaqdrv_status ifc_xuser_tcsr_setclr(struct ifcdaqdrv_dev *ifcdevice, int register_idx, int32_t setmask, int32_t
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

void ifcdaqdrv_manualswap(uint16_t *buffer, int nsamples);
void ifcdaqdrv_start_tmeas(void);
void ifcdaqdrv_end_tmeas(void);
long ifcdaqdrv_elapsedtime(void);
int ifcdaqdrv_is_byte_order_ppc(void);

#endif // _IFCDAQDRV_UTILS_H_


#ifndef _IFCDAQDRV_SIM_H_
#define _IFCDAQDRV_SIM_H_

#include <pthread.h>
#include "ifcdaqdrv_list.h"

#define MAX_PEV_CARDS 16
#define MAX_DECIMATIONS 20

#define ADC3110_CLOCK_MIN 40.0e6
#define ADC3110_CLOCK_MAX 330.0e6


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
 * @brief Device private struct.
 *
 * Contains objects necessary for device access serialization,
 * end of acquisition signaling, device bookkeeping and status information.
 */
struct ifcdaqdrv_dev {
    struct list_head   list;            /**< Entry in the list of opened devices, sis8300drv_devlist. */
    uint32_t           card;            /**< Card/Crate number selected by rotational on-board switch. */
    uint32_t           fmc;             /**< FMC slot, 1 or 2. */
    int                count;           /**< Number of times this device has been opened. */
    uint32_t           init_called;     /**< Positive if init_adc has been called. */
    uint32_t           ch_enabled;      /**< bitmask of enabled channels */

    ifcdaqdrv_acq_store_mode mode;           /**< In which memory to store acquistition SRAM/SMEM */
    ifcdaqdrv_trigger_type   trigger_type;

    // struct pev_ioctl_buf    *sram_dma_buf;                /**< Buffer for SRAM DMA transfers */
    // struct pev_ioctl_buf    *smem_dma_buf;                /**< Buffer for SMEM DMA transfers */
    // void                    *all_ch_buf;                  /**< Buffer to store raw SMEM data */

    uint32_t                 sample_size;                  /**< Sample size in bytes, TODO: Function pointer instead? */
    uint32_t                 nchannels;                    /**< Number of channels */
    uint32_t                 decimations[MAX_DECIMATIONS]; /**< hardare supported decimations */
    uint32_t                 averages[MAX_DECIMATIONS];    /**< hardare supported averages */

    uint32_t                 resolution;                   /**< ADC Resolution of FMC */
    double                   valid_clocks[MAX_DECIMATIONS];/**< Clock frequencies supported by FMC. */
    uint32_t                 divisor_max;                  /**< Maximum clock divisor supported */
    uint32_t                 divisor_min;                  /**< Minimum clock divisor supported */

    uint32_t                 sample_resolution;            /**< Resolution of samples read out by read_ai */
    double                   vref_max;                     /**< Maximum measurable voltage */

    int      armed;                                        /**< Is device armed, 0 - not armed, 1 - armed. */
    uint32_t poll_period;                                  /**< Poll period in microseconds used when checking weather acquisition is finished. */

    uint32_t sram_size;                                    /**< Size of SRAM per channel in bytes  */
    uint32_t smem_size;                                    /**< Size of shared RAM in bytes (512/2 MB in IFC1210). */

    int32_t *databuffer;                                    /* Buffer to hold simulated data */
    uint32_t buffersize;

    pthread_mutex_t lock;                                  /**< Lock that serializes access to the device. */
    pthread_mutex_t sub_lock;                              /**< Lock that serializes access to part of the device. */

    /*runtime simulated parameters!! */
    ifcdaqdrv_clock         rt_clock_source;
    double                  rt_channel_gain[8];
    ifcdaqdrv_dataformat    rt_dataformat;
    double                  rt_clock_frequency;
    uint32_t                rt_clock_divisor;

    ifcdaqdrv_trigger_type  rt_tr_trigger;
    int32_t                 rt_tr_threshold;
    uint32_t                rt_tr_mask;
    uint32_t                rt_tr_edge;

    uint32_t                rt_average;
    uint32_t                rt_decimation;

    uint32_t                rt_nsamples;
    uint32_t                rt_npretrig;

    ifcdaqdrv_pattern       rt_ch_pattern[8];
};

ifcdaqdrv_status ADCSim_register(struct ifcdaqdrv_dev *ifcdevice);

void ifcdaqdrv_free(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status ifcdaqdrv_dma_allocate(struct ifcdaqdrv_dev *ifcdevice);



#endif //_IFCDAQDRV_SIM_H_
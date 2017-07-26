#ifndef _IFC1210SCOPEDRV_H_
#define _IFC1210SCOPEDRV_H_ 1

/* Types from the following libraries are used */
#include <stddef.h>
#include <inttypes.h>


#ifdef __cplusplus
extern "C" {
#endif

#define IFC1210SCOPEDRV_TOSCA_SIGNATURE       0x12350201 // IFC1410
#define IFC1210SCOPEDRV_TOSCA_SIGNATURE_MASK  0xffff0000

#define IFC1210SCOPEDRV_SCOPE_SIGNATURE       0x12110301 // IFC1410
#define IFC1210SCOPEDRV_FASTSCOPE_SIGNATURE   0x12100501
#define IFC1210SCOPEDRV_SCOPE_DTACQ_SIGNATURE 0x12101201


/**
 * @brief Enumeration of possible error codes.
 */

typedef enum {
    status_success          = 0,    /**< Operation successful. */
    status_device_armed     = -1,   /**< The device is armed. */
    status_argument_range   = -2,   /**< Argument out of range. */
    status_argument_invalid = -3,   /**< Invalid argument choice. */
    status_device_access    = -4,   /**< Error in reading/writing device registers. */
    status_read             = -5,   /**< Error in reading device memory. */
    status_write            = -6,   /**< Error in writing to device memory. */
    status_internal         = -7,   /**< Internal library error. */
    status_no_device        = -8,   /**< Device not opened. */
    status_device_state     = -9,   /**< Device in an undefined state. */
    status_irq_error        = -10,  /**< Waiting for irq resulted in error from device. */
    status_irq_timeout      = -11,  /**< Waiting for irq timeout. */
    status_irq_release      = -12,  /**< Waiting for irq was stopped. */
    status_i2c_busy         = -13,  /**< i2c bus busy. */
    status_i2c_nack         = -14,  /**< i2c transfer not acknowledged. */
    status_spi_busy         = -15,  /**< spi interface busy. */
    status_flash_busy       = -16,  /**< Flash chip busy. */
    status_flash_failed     = -17,  /**< Flashing image failed. */
    status_incompatible     = -18,  /**< Incompatible device. */
    status_no_support       = -19,  /**< Unsupported feature. */
    status_fru_info_invalid = -20,  /**< Invalid FRU information. */
    status_buf_len          = -21,  /**< Invalid argument buffer length. */
    status_cancel           = -22,  /**< Operation was canceled */
    status_config           = -23,  /**< Configuration not allowed */
    status_unknown          = -24
} ifcdaqdrv_status;

/**
 * @brief Strings for error codes.
 */

static const char *ifcdaqdrv_errstr[] =
{
    "operation successful",
    "the device is armed",
    "argument out of range",
    "invalid argument choice",
    "error reading/writing device registers",
    "error reading device memory",
    "error writing to device memory",
    "internal library error",
    "device not opened",
    "device in an undefined state",
    "irq wait failed",
    "irq timeout",
    "irq stopped",
    "i2c busy",
    "i2c not acknowledged",
    "spi busy",
    "flash busy",
    "flashing image failed",
    "incompatible device",
    "unsupported feature",
    "invalid FRU information",
    "invalid argument buffer length",
    "acquisition canceled",
    "configuration not allowed",

    "unknown error code"
};

/**
 * @brief Return String interpretation of error code.
 */

static inline const char *ifcdaqdrv_strerror(ifcdaqdrv_status status) {
    if (status < status_unknown || status > 0) {
        status = status_unknown;
    }
    return ifcdaqdrv_errstr[-status];
}

/**
 * @brief Enumeration for trigger.
 */

typedef enum {
    ifcdaqdrv_trigger_none,       /**< Disable triggers */
    ifcdaqdrv_trigger_backplane,  /**< Interpret channel mask as a VME trigger */
    ifcdaqdrv_trigger_frontpanel, /**< Interpret channel mask as DAQ channel or FMC GPIO */
    ifcdaqdrv_trigger_auto,       /**< Deprecated */
    ifcdaqdrv_trigger_soft,       /**< Manually trigger from software immediately after arming device */
    ifcdaqdrv_trigger_testmanual, /**< TEST ONLY: simulated backplane trigger under demand */
    ifcdaqdrv_trigger_testauto    /**< TEST ONLY: simulated backplane trigger using periodic clk division */
} ifcdaqdrv_trigger_type;

/**
 * @brief Enumeration for data format.
 *
 * Not used so far.
 */

typedef enum {
    ifcdaqdrv_dataformat_signed,   /**< Set ADC dataformat to signed two-complements */
    ifcdaqdrv_dataformat_unsigned  /**< Set ADC dataformat to unsigned (0 is lowest, 0xFF.. is highest value) */
} ifcdaqdrv_dataformat;

/**
 * @brief Enumeration for clocks.
 */

typedef enum {
    ifcdaqdrv_clock_internal,  /**< Set clock source to internal */
    ifcdaqdrv_clock_external   /**< Set clock source to external */
} ifcdaqdrv_clock;

/**
 * @brief Enumeration for test patterns.
 */

typedef enum {
    ifcdaqdrv_pattern_none,     /**< Normal mode, no test pattern */
    ifcdaqdrv_pattern_zero,     /**< Every bit is zero (0x00..) */
    ifcdaqdrv_pattern_one,      /**< Every bit is one (0xFF..) */
    ifcdaqdrv_pattern_toggle,   /**< Toggle between every other bit is zero and one (0xaa/0x55)*/
    ifcdaqdrv_pattern_ramp_inc, /**< Increasing counter */
    ifcdaqdrv_pattern_ramp_dec, /**< Decreasing counter */
    ifcdaqdrv_pattern_8psin     /**< 8 point repeating sinusiodal */
} ifcdaqdrv_pattern;


/**
 * @brief Library user context struct.
 */

struct ifcdaqdrv_usr {
    uint32_t card;       /**< Card/Crate number selected by rotational on-board switch. */
    uint32_t fmc;        /**< FMC slot, 1 or 2. */
    void    *device;     /**< Device private context.  */
};

/**
 * @brief Open the device.
 *
 * @param[in] ifcuser User struct. Should be populated with card and fmc number.
 */

ifcdaqdrv_status ifcdaqdrv_open_device(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Close the device.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcdaqdrv_close_device(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Initialize ADCs.
 *
 * This has to be called before arming the board for the first time.  If it is called twice it will re-initialize the
 * ADCs the second time, in practice reseting them.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcdaqdrv_init_adc(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Arm device.
 *
 * It is not allowed to change any configuration while the board is armed.  While the device is armed all functions
 * except #ifcdaqdrv_disarm_device and #ifcdaqdrv_wait_acq_end will return error code "board armed".
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcdaqdrv_arm_device(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Disarm device.
 *
 * This functions is called to cancel an acquisition. It will disarm the device
 * and make #ifcdaqdrv_wait_acq_end return with "acquisition cancelled".
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcdaqdrv_disarm_device(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Wait until acquisition is done. Blocking call.
 *
 * This blocking call is called after #ifcdaqdrv_arm_device to ensure that the acquisition is done. After this function
 * has returned it is possible to read out the last samples with #ifcdaqdrv_read_ai or #ifcdaqdrv_read_ai_ch.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcdaqdrv_wait_acq_end(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Read Analog Input samples. Returns valid data immediately after #ifcdaqdrv_wait_acq_end finishes.
 *
 * @param[in]  ifcuser User struct.
 * @param[out] data    The buffer to store the samples. Shall be sizeof(int32_t) * nchannels * nsamples in size. Data is always
 *                     32 bit integers but depending on #ifcdaqdrv_get_resolution should be interpreted as int32_t
 *                     or int16_t.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai(struct ifcdaqdrv_usr *ifcuser, void *data);

/**
 * @brief Read Analog Input samples of a single channel. Returns valid data immediately after #ifcdaqdrv_wait_acq_end
 * finishes.
 *
 * @warning  Due to the layout of the data by some firmwares this function might have to transfer ALL
 *          channels even though only one is read...
 *
 * @param[in]  ifcuser User struct.
 * @param[in]  channel The channel to read out. Valid values are 0 to nchannels - 1.
 * @param[out] data    The buffer to store the samples. Shall be sizeof(int32_t) * nsamples in size. Data is always 32 bit integers
 *                     but depending on #ifcdaqdrv_get_resolution should be interpreted as int32_t or int16_t.
 */

ifcdaqdrv_status ifcdaqdrv_read_ai_ch(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, void *data);

/**
 * @brief Enable channel mask.
 *
 * If a channel is disabled it will not be copied from the device (if possible). mask[n] is channel n enable. Set bit
 * to 1 for Enable, 0 for Disable.
 *
 * @param[in] ifcuser User struct.
 * @param[in] mask    The channel enable mask.
 */

ifcdaqdrv_status ifcdaqdrv_channel_enable(struct ifcdaqdrv_usr *ifcuser, uint64_t mask);

/**
 * @brief Set clock source.
 *
 * @param[in] ifcuserser struct.
 * @param[in] clock The clock source.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock clock);

/**
 * @brief Get clock source.
 *
 * @param[in]  ifcuserser struct.
 * @param[out] clock The clock source.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock *clock);

/**
 * @brief Set clock frequency.
 *
 * @param[in] ifcuser   User struct.
 * @param[in] frequency The clock frequency.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency);

/**
 * @brief Get clock frequency.
 *
 * @param[in]  ifcuser   User struct.
 * @param[out] frequency The clock frequency.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double *frequency);

/**
 * @brief Get supported clock frequencies.
 *
 * @param[in]  ifcuser     User struct.
 * @param[out] frequencies NULL terminated array with supported clock frequencies.
 * @param[in]  buf_len     The length of the user provided buffer.
 * @param[out] data_len    The number of frequencies returned.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_frequencies_valid(struct ifcdaqdrv_usr *ifcuser, double *frequencies, size_t buf_len, size_t *data_len);

/**
 * @brief Set clock divisor.
 *
 * @param[in] ifcuser User struct.
 * @param[in] divisor The clock divisor. The valid divisor range can be read out with
 *                    #ifcdaqdrv_get_clock_divisor_range.
 */

ifcdaqdrv_status ifcdaqdrv_set_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t divisor);

/**
 * @brief Get clock divisor.
 *
 * @param[in]  ifcuser User struct.
 * @param[out] divisor The clock divisor.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor);

/**
 * @brief Get clock divisor minimum and maximum.
 *
 * @param[in]  ifcuser     User struct.
 * @param[out] divisor_min The minumum clock divisor.
 * @param[out] divisor_max The maximum clock divisor.
 */

ifcdaqdrv_status ifcdaqdrv_get_clock_divisor_range(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor_min, uint32_t *divisor_max);


/**
 * @brief Set trigger configuration.
 *
 * @param[in] ifcuser   User struct.
 * @param[in] trigger   The trigger source.
 * @param[in] threshold The voltage threshold for frontpanel ADC channel trigger. Given in ADC value: 32 or 16 bit depending
 *                      on #ifcdaqdrv_get_resolution.
 * @param[in] mask      The channel select mask. MSB bit indicates frontpanel or backplane.
 *                      - 0x0000 0000 to 0x7FFF FFFF is frontpanel mask, 0x4000 0000 is GPIO pin, mask[n] is channel n mask bit.
 *                      - 0x8000 0000 to 0xFFFF FFFF is backplane mask, depending on platform there are different amount of triggers.
 * @param[in] rising_edge The rising edge mask. Set to 1 to trigger on rising edge.
 *                        - 0x0000 0000 to 0x7FF FFFF is frontpanel mask. 0: falling edge, 1: rising edge. Combination of
 *                          one channel and the GPIO pin is allowed. If GPIO pin bit is set to 1 it is active high.
 *                        - 0x8000 0000 -> Backplane polarity falling.
 *                        - 0x8FFF FFFF -> Backplane polarity rising.
 */

ifcdaqdrv_status ifcdaqdrv_set_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type trigger, int32_t threshold,
                                       uint32_t mask, uint32_t rising_edge);

/**
 * @brief Get trigger configuration.
 *
 * @param[in] ifcuser    User struct.
 * @param[out] trigger   The trigger source.
 * @param[out] threshold The voltage threshold for frontpanel ADC channel trigger.
 * @param[out] mask      The channel select mask.
 * @param[out] polarity  The channel polarity mask.
 */

ifcdaqdrv_status ifcdaqdrv_get_trigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_trigger_type *trigger,
                                       int32_t *threshold, uint32_t *mask, uint32_t *rising_edge);

/**
 * @brief Set averaging.
 *
 * Hardware averaging will downsample the data without changing the number of samples, which means that the same amount
 * of values are copied from hardware to software but a larger part of the analog signal is acquired. Every sample will
 * be the average of N samples if an average factor of N is chosen.
 *
 * On some FMCs it is not possible to have both decimation and averaging simultaneously.
 *
 * @param[in] ifcuser User struct.
 * @param[in] average The averaging factor.
 */

ifcdaqdrv_status ifcdaqdrv_set_average(struct ifcdaqdrv_usr *ifcuser, uint32_t average);

/**
 * @brief Get averaging.
 *
 * @param[in]  ifcuser User struct.
 * @param[out] average The averaging factor.
 */

ifcdaqdrv_status ifcdaqdrv_get_average(struct ifcdaqdrv_usr *ifcuser, uint32_t *average);

/**
 * @brief Get hardware supported averaging lengths.
 *
 * @param[in]  ifcuser  User struct.
 * @param[out] averages NULL terminated array with supported averages.
 * @param[in]  buf_len  The length of the user provided buffer.
 * @param[out] data_len The number of frequencies returned.
 */

ifcdaqdrv_status ifcdaqdrv_get_averages_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *averages, size_t buf_len, size_t *data_len);

/**
 * @brief Set decimation.
 *
 * Hardware decimation will downsample the data without changing the number of samples, which means that the same amount
 * of values are copied from hardware to software but a larger part of the analog signal is acquired. Every sample will
 * be the N-th sample if an decimation factor of N is chosen.
 *
 * On some FMCs it is not possible to have both decimation and averaging simultaneously.
 *
 * @param[in] ifcuser    User struct.
 * @param[in] decimation The decimation factor.
 */

ifcdaqdrv_status ifcdaqdrv_set_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t decimation);

/**
 * @brief Get decimation.
 *
 * @param[in]  ifcuser    User struct.
 * @param[out] decimation The decimation factor.
 */

ifcdaqdrv_status ifcdaqdrv_get_decimation(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimation);

/**
 * @brief Get hardware supported decimations.
 *
 * @param[in]  ifcuser     User struct.
 * @param[out] decimations NULL terminated array with supported averages.
 * @param[in]  buf_len     The length of the user provided buffer.
 * @param[out] data_len    The number of frequencies returned.
 */

ifcdaqdrv_status ifcdaqdrv_get_decimations_valid(struct ifcdaqdrv_usr *ifcuser, uint32_t *decimations, size_t buf_len, size_t *data_len);

/**
 * @brief Set number of samples
 *
 * @param[in] ifcuser  User struct.
 * @param[in] nsamples The number of samples.
 */

ifcdaqdrv_status ifcdaqdrv_set_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t nsamples);

/**
 * @brief Get number of samples
 *
 * @param[in]  ifcuser  User struct.
 * @param[out] nsamples The number of samples.
 */

ifcdaqdrv_status ifcdaqdrv_get_nsamples(struct ifcdaqdrv_usr *ifcuser, uint32_t *nsamples);

/**
 * @brief Set number of pre-trigger samples.
 *
 * @param[in] ifcuser  User struct.
 * @param[in] npretrig The number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_set_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t npretrig);

/**
 * @brief Get number of pre-trigger samples.
 *
 * @param[in]  ifcuser  User struct.
 * @param[out] npretrig The number of pre-trigger samples.
 */

ifcdaqdrv_status ifcdaqdrv_get_npretrig(struct ifcdaqdrv_usr *ifcuser, uint32_t *npretrig);

/**
 * @brief Set gain per channel. Supported gain varies between FMCs.
 *
 * Gain is the amount of amplification or attenuation given to the signal _before_ it is sampled with the ADC. If the
 * requested gain is lower than the lowest supported gain, the lowest will be set. If the requested gain is higher than
 * the highest supported gain, the highest will be set.
 *
 * @param[in] ifcuser User struct.
 * @param[in] channel The channel to apply gain to.
 * @param[in] gain    The gain (1.0 will disable gain).
 */

ifcdaqdrv_status ifcdaqdrv_set_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double gain);

/**
 * @brief Get gain per channel.
 *
 * @param[in]  ifcuser User struct.
 * @param[in]  channel The channel to get gain from.
 * @param[out] gain    The gain.
 */

ifcdaqdrv_status ifcdaqdrv_get_gain(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, double *gain);

/**
 * @brief Get maximum voltage that can be measured by the board.
 *
 * This is the value given by the manufacturer in the datasheet. Together with the ADC resolution, it can be used to
 * calculate the correct V/bit conversion.
 *
 * @param[in]  ifcuser  User struct.
 * @param[out] vref_max The maximum measurable voltage.
 */

ifcdaqdrv_status ifcdaqdrv_get_vref_max(struct ifcdaqdrv_usr *ifcuser, double *vref_max);

/**
 * @brief Get the actual resolution of the samples read out by #ifcdaqdrv_read_ai and #ifcdaqdrv_read_ai_ch.
 *
 * @param[in]  ifcuser    User struct.
 * @param[out] resolution The sample resolution.
 */

ifcdaqdrv_status ifcdaqdrv_get_resolution(struct ifcdaqdrv_usr *ifcuser, uint32_t *resolution);

/**
 * @brief Set testpattern or normal mode on channel
 *
 * @param[in] ifcuser User struct.
 * @param[in] pattern The test pattern.
 */

ifcdaqdrv_status ifcdaqdrv_set_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern pattern);

/**
 * @brief Get testpattern on channel.
 *
 * @param[in]  ifcuser User struct.
 * @param[out] pattern The test pattern.
 */

ifcdaqdrv_status ifcdaqdrv_get_pattern(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcdaqdrv_pattern *pattern);

/**
 * @brief Get the number of channels the device has.
 *
 * @param[in]  ifcuser   User struct.
 * @param[out] nchannels The number of channels.
 */

ifcdaqdrv_status ifcdaqdrv_get_nchannels(struct ifcdaqdrv_usr *ifcuser, uint32_t *nchannels);

/**
 * @brief Get the Manufacturer
 *
 * @param[in]  ifcuser      User struct.
 * @param[out] manufacturer The buffer to hold the manufacturer string.
 * @param[in]  buf_len      The length of the user provided buffer.
 * @return status_success or status_buf_len (Indicating buffer not long enough)
 */

ifcdaqdrv_status ifcdaqdrv_get_manufacturer(struct ifcdaqdrv_usr *ifcuser, char *manufacturer, size_t buf_len);

/**
 * Get the Product name
 *
 * @param[in]  ifcuser      User struct.
 * @param[out] product_name The buffer to hold the product name string.
 * @param[in]  buf_len      The length of the user provided buffer.
 * @return status_success or status_buf_len (Indicating buffer not long enough)
 */
ifcdaqdrv_status ifcdaqdrv_get_product_name(struct ifcdaqdrv_usr *ifcuser, char *product_name, size_t buf_len);


/*
 * Firmware information. Exposed temporarily.
 * These functions may change when it is more clear which numbers actually matters to API user.
 */

/*
 * Get the FMC HW Revision
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_revision(struct ifcdaqdrv_usr *ifcuser, uint8_t *revision);

/*
 * Get the FMC HW Version
 */

ifcdaqdrv_status ifcdaqdrv_get_fw_version(struct ifcdaqdrv_usr *ifcuser, uint8_t *version);


/*
 * Test function for the backplane triggering 
 */
typedef enum {
    ifcdaqdrv_simtrig_stop,     
    ifcdaqdrv_simtrig_startauto,   
    ifcdaqdrv_simtrig_manual,   
    ifcdaqdrv_simtrig_setfreq,   
    ifcdaqdrv_simtrig_readreg
 } ifcdaqdrv_simtrigger_action;

ifcdaqdrv_status ifcdaqdrv_set_simtrigger(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_simtrigger_action function, int32_t clk_divider);




#ifdef __cplusplus
}
#endif

#endif

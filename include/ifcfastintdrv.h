#ifndef _IFCFASTINT_H_
#define _IFCFASTINT_H_ 1

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <ifcdaqdrv2.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Oliver double check */
#define IFC1210FASTINT_XUSER_SIGNATURE 0x12100901
#define IFC1210FASTINT_APP_SIGNATURE   0x12340101

/**
 * @brief Analog mode for analog pre-processing
 */

typedef enum {
    ifcfastint_amode_zero=0,
    ifcfastint_amode_one=1,
    ifcfastint_amode_gte1=2,
    ifcfastint_amode_lte2=3,
    ifcfastint_amode_gte1_lte2=4,
    ifcfastint_amode_pulse_mon=8,
    ifcfastint_amode_pd_avg=10,
    ifcfastint_amode_pd_sum=11,
    ifcfastint_amode_dev_mon=12
} ifcfastint_amode;

/**
 * @brief Digital mode for analog pre-processing
 */

typedef enum {
    ifcfastint_dmode_zero,
    ifcfastint_dmode_one,
    ifcfastint_dmode_pass_through,
    ifcfastint_dmode_invert
    //ifcfastint_dmode_varcalc1,
    //ifcfastint_dmode_varcalc2
} ifcfastint_dmode;

/**
 * @brief FSM states
 */

typedef enum {
    ifcfastint_fsm_state_idle,
    ifcfastint_fsm_state_arm,
    ifcfastint_fsm_state_pre,
    ifcfastint_fsm_state_run,
    ifcfastint_fsm_state_abort
} ifcfastint_fsm_state;

#define IFCFASTINT_ANALOG_MODE_W         (1<<0)
#define IFCFASTINT_ANALOG_EMULATION_EN_W (1<<1)
#define IFCFASTINT_ANALOG_VAL1_W         (1<<2)
#define IFCFASTINT_ANALOG_VAL2_W         (1<<3)
#define IFCFASTINT_ANALOG_VAL3_W         (1<<4)
#define IFCFASTINT_ANALOG_VAL4_W         (1<<5)
#define IFCFASTINT_ANALOG_CVAL_W         (1<<6)
#define IFCFASTINT_ANALOG_IDLE2PRE_W     (1<<7)
#define IFCFASTINT_ANALOG_PRE2RUN_W      (1<<8)

struct ifcfastint_analog_option {
    ifcfastint_amode mode;
    bool emulation_en;
    int16_t val1;
    int16_t val2;
    int16_t val3;
    int16_t val4;
    int16_t cval;
    bool idle2pre;
    bool pre2run;
};

#define IFCFASTINT_DIGITAL_MODE_W         (1<<0)
#define IFCFASTINT_DIGITAL_EMULATION_EN_W (1<<1)
#define IFCFASTINT_DIGITAL_VAL1_W         (1<<2)
#define IFCFASTINT_DIGITAL_VAL2_W         (1<<3)
#define IFCFASTINT_DIGITAL_CVAL_W         (1<<4)
#define IFCFASTINT_DIGITAL_IDLE2PRE_W     (1<<5)
#define IFCFASTINT_DIGITAL_PRE2RUN_W      (1<<6)

struct ifcfastint_digital_option {
    ifcfastint_dmode mode;
    bool emulation_en;
    int16_t val1;
    int16_t val2;
    bool cval;
    bool idle2pre;
    bool pre2run;
};

/**
 * @brief Initialize and start FSM
 */

ifcdaqdrv_status ifcfastint_init_fsm(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Wait until the abort acquisition has ended.
 * This will hang indefinitely if state != abort.
 */

ifcdaqdrv_status ifcfastint_wait_abort_done(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Read out the current output values of the pre-processing blocks.
 *
 * @param[in] ifcuser User struct.
 * @param[out] digital Bitfield with digital channels.
 * @param[out] analog Bitfield with analog channels.
 */

ifcdaqdrv_status ifcfastint_get_pp_out(struct ifcdaqdrv_usr *ifcuser,
                                       uint32_t *digital,
                                       uint32_t *analog);

/**
 * @brief Read "count" frames from the history data.
 *
 * This will empty the queue of any existing history events.
 *
 * If count is less than the existing events it will return the most recent.
 *
 * Every history event consists of 64 bytes:
 *
 * - 8  Bytes (4 bytes sequence id, signature and fsm status).
 * - 8  Bytes (4 bytes digital pre-processing out and 4 bytes digital pre-processing out).
 * - 4  Bytes Digital input values.
 * - 40 Bytes Analog input values.
 * - 2  Bytes checksum.
 * - 2  Bytes magic word.
 *
 * @param[in] ifcuser User struct.
 * @param[out] data History data.
 * @param[out] nelm Number of history events.
 */


ifcdaqdrv_status ifcfastint_read_history(struct ifcdaqdrv_usr *ifcuser,
                                         size_t count,
                                         void *data,
                                         size_t *nelm);

/**
 * @brief Reset the FSM.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcfastint_fsm_reset(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Get FSM state.
 *
 * @param[in] ifcuser User struct.
 * @param[out] state
 */

ifcdaqdrv_status ifcfastint_get_fsm_state(struct ifcdaqdrv_usr *ifcuser,
                                          ifcfastint_fsm_state *state);

/**
 * @brief Interface to simulate different states.
 *
 * @param[in] ifcuser User struct.
 * @param[in] state
 */

ifcdaqdrv_status ifcfastint_set_fsm_state(struct ifcdaqdrv_usr *ifcuser,
                                          ifcfastint_fsm_state state);
/**
 * @brief Get FSM digital output as a mask.
 *
 * The following bits can be set:
 *
 * 3. HV ENABLE   (VME_P2_C1)
 * 2. RF ENABLE   (VME_P2_C3)
 * 1. SPARE/ABORT (VME_P2_C5)
 * 0. LLRF ENABLE (VME_P2_C7/8/9)
 *
 * @param[in] ifcuser User struct.
 * @param[out] channel_mask Bits set according to list above.
 */

ifcdaqdrv_status ifcfastint_get_fsm_do(struct ifcdaqdrv_usr *ifcuser,
                                       uint32_t *channel_mask);

/**
 * @brief Set FSM update frequency in kHz. Only valid frequencies are 0, 200, 500 and 1000.
 */

ifcdaqdrv_status ifcfastint_set_fsm_frequency(struct ifcdaqdrv_usr *ifcuser,
                                              uint32_t frequency);

/**
 * @brief Get FSM update frequency in kHz. Only valid frequencies are 0, 200, 500 and 1000.
 */

ifcdaqdrv_status ifcfastint_get_fsm_frequency(struct ifcdaqdrv_usr *ifcuser,
                                              uint32_t *frequency);

/**
 * @brief Get configuration of analog channel pre-processing. Only return values if the pointer != NULL.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-23)
 * @param[out] mode Determine the arithmetic operation performed on input.
 * @param[out] emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[out] val1 Threshold 1.
 * @param[out] val2 Threshold 2.
 * @param[out] val3 Threshold 3.
 * @param[out] val4 Threshold 4.
 * @param[out] cval Emulated current analog value.
 * @param[out] transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */

ifcdaqdrv_status ifcfastint_get_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               struct ifcfastint_analog_option *option);
/**
 * @brief Set configuration of analog channel pre-processing. Only write values according to write_mask.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-23)
 * @param[in] write_mask Specify which options to write [transition_mask, cval, val4, val3, val2, val1, emulation_en, mode]
 * @param[in] mode Determine the arithmetic operation performed on input.
 * @param[in] emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[in] val1 Threshold 1.
 * @param[in] val2 Threshold 2.
 * @param[in] val3 Threshold 3.
 * @param[in] val4 Threshold 4.
 * @param[in] cval Emulated current analog value.
 * @param[in] transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */

ifcdaqdrv_status ifcfastint_set_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
                                               struct ifcfastint_analog_option *option);

/**
 * @brief Get configuration of digital channel pre-processing.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-23)
 * @param[out] mode Determine the arithmetic operation performed on input.
 * @param[out] emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[out] val1 Value 1, parameter 1 for VAR-CALC.
 * @param[out] val2 Value 2, parameter 2 for VAR-CALC.
 * @param[out] cval Emulated current digital value.
 */

ifcdaqdrv_status ifcfastint_get_conf_digital_pp(struct ifcdaqdrv_usr *ifcuser,
                                           uint32_t block,
                                           struct ifcfastint_digital_option *option);

/**
 * @brief Set configuration of digital channel pre-processing.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-23)
 * @param[in] mode Determine the arithmetic operation performed on input.
 * @param[in] emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[in] val1 Value 1, parameter 1 for VAR-CALC.
 * @param[in] val2 Value 2, parameter 2 for VAR-CALC.
 * @param[in] cval Emulated current digital value.
 */

ifcdaqdrv_status ifcfastint_set_conf_digital_pp(struct ifcdaqdrv_usr *ifcuser,
                                           uint32_t block,
                                           uint32_t write_mask,
                                           struct ifcfastint_digital_option *option);

/**
 * @brief Lock the configuration so that configuration cannot be written.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcfastint_conf_lock(struct ifcdaqdrv_usr *ifcuser);

/**
 * @brief Unlock the configuration so that configuration can be written.
 *
 * @param[in] ifcuser User struct.
 */

ifcdaqdrv_status ifcfastint_conf_unlock(struct ifcdaqdrv_usr *ifcuser);

#ifdef __cplusplus
}
#endif

#endif /* _IFCFASTINT_H_ */

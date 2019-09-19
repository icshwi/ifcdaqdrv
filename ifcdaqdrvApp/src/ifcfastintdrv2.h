#ifndef _IFCFASTINT_H_
#define _IFCFASTINT_H_ 1

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "ifcdaqdrv.h"

#ifdef __cplusplus
extern "C" {
#endif


#define IFC1210FASTINT_XUSER_SIGNATURE 0x12100901
#define IFC1210FASTINT_APP_SIGNATURE   0x12340201

#define IFCFASTINT_CHANGESTATE  0x00
#define IFCFASTINT_KEEPSTATE    0x01

#define IFCFASTINT_RAND_WRPOINTER 0x00
#define IFCFASTINT_TRIG_WRPOINTER 0x01


/*
 * @brief Pre-processing modes for analog input
 */
typedef enum {
	ifcfastint_analog_pp_lvlmon,
	ifcfastint_analog_pp_pulshp,
	ifcfastint_analog_pp_pulrate,
	ifcfastint_analog_pp_devmon,
	ifcfastint_analog_pp_pwravg,
	ifcfastint_analog_pp_channel
} ifcfastint_analog_pp;

/*
 * @brief Pre-processing modes for analog input with register code
 */
typedef enum {
	ifcfastint_amode_zero   = 0x00,
	ifcfastint_amode_one    = 0x01,
	ifcfastint_amode_level2 = 0x02,
	ifcfastint_amode_level3 = 0x03,
	ifcfastint_amode_level4 = 0x04,
	ifcfastint_amode_level5 = 0x05,
	ifcfastint_amode_shape  = 0x08,
	ifcfastint_amode_rate   = 0x09,
	ifcfastint_amode_devmon = 0x0C,
	ifcfastint_amode_rsvd   = 0x0D,
	ifcfastint_amode_avg1   = 0x0A,
	ifcfastint_amode_avg2   = 0x0B
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

  /*
  * IDLE and ABO are states that corresponds to STDBY state on the SIM 
  * PRE corresponds to HV ON
  * RUN corresponds to RF ON
  *
  */

typedef enum {
    ifcfastint_fsm_state_idle,
    ifcfastint_fsm_state_arm,
    ifcfastint_fsm_state_pre,
    ifcfastint_fsm_state_run,
    ifcfastint_fsm_state_abort
} ifcfastint_fsm_state;

/*
 * @brief History Mode Configurations
 */
typedef enum {
	ifcfastint_histmode_0=0,
	ifcfastint_histmode_1=1,
	ifcfastint_histmode_2=2,
	ifcfastint_histmode_3=3,
	ifcfastint_histmode_4=4,
	ifcfastint_histmode_5=5,
	ifcfastint_histmode_6=6,
	ifcfastint_histmode_7=7
} ifcfastint_histmode;

typedef enum {
	ifcfastint_history_disabled,
	ifcfastint_history_enabled
} ifcfastint_histcontrol;

typedef enum {
	ifcfastint_history_noflags,
	ifcfastint_history_overff,
	ifcfastint_history_ringoverff,
	ifcfastint_history_
} ifcfastint_hist_flags;

typedef enum {
	ifcfastint_history_idle,
	ifcfastint_history_running,
	ifcfastint_history_postmortem,
	ifcfastint_history_ended
} ifcfastint_hist_state;

typedef enum {
  ifcfastint_aichannel_ergain,
  ifcfastint_aichannel_eroffset,
  ifcfastint_aichannel_egumax,
  ifcfastint_aichannel_egumin
} ifcfastint_aichannel_param;


/* Masks used on history mode configuration to identify which options will write */
#define IFCFASTINT_HISTORY_ENABLE_W	(1<<0)
#define IFCFASTINT_HISTORY_MODE_W	(1<<1)


/* Masks used on option struct to identify which options will write */
#define IFCFASTINT_ANALOG_MODE_W         (1<<0)
#define IFCFASTINT_ANALOG_EMULATION_EN_W (1<<1)
#define IFCFASTINT_ANALOG_VAL1_W         (1<<2)
#define IFCFASTINT_ANALOG_VAL2_W         (1<<3)
#define IFCFASTINT_ANALOG_VAL3_W         (1<<4)
#define IFCFASTINT_ANALOG_VAL4_W         (1<<5)
#define IFCFASTINT_ANALOG_CVAL_W         (1<<6)
#define IFCFASTINT_ANALOG_IDLE2PRE_W     (1<<7)
#define IFCFASTINT_ANALOG_PRE2RUN_W      (1<<8)

/*
 * @brief Struct to hold analog PP configuration block (level, shape, rate, deviation)
 */
struct ifcfastint_analog_option {
	ifcfastint_amode mode;
    bool emulation_en;
    uint32_t val1;
    uint32_t val2;
    uint32_t val3;
    uint32_t val4;
    uint32_t cval;
    bool idle2pre;
    bool pre2run;
};

/*
 * @brief Struct to hold Power Averaging analog PP configuration block
 */
struct ifcfastint_pwravg_option {
	ifcfastint_amode mode;
	bool emulation_en;
	int16_t u_in;
	int16_t i_in;
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

struct ifcfastint_analog_diag {
	bool process_out;
	uint32_t pres_val;
	uint32_t trig_val;
	int16_t dev_val;
	int16_t trig_dev_val;
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
                                         size_t *nelm,
                                         int readtype);

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
 * @param[in] block Specify which analog input block to configure (0-19)
 * @param[in] ppblock Specify which one of the 4 types of standard pre-processing this configuration refers (level, shape, rate, deviation)
 * @param[out] option->mode Determine the arithmetic operation performed on input.
 * @param[out] option->emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[out] option->val1 Threshold 1.
 * @param[out] option->val2 Threshold 2.
 * @param[out] option->val3 Threshold 3.
 * @param[out] option->val4 Threshold 4.
 * @param[out] option->cval Emulated current analog value.
 * @param[out] option->transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */

ifcdaqdrv_status ifcfastint_get_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
											   ifcfastint_analog_pp ppblock,
                                               struct ifcfastint_analog_option *option);

/**
 * @brief Get configuration of power averaging pre-processing block. Only return values if the pointer != NULL.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which pwravg block to configure (0-3)
 *
 * @param[out] option->mode Determine the arithmetic operation performed on input.
 * @param[out] option->u_in Analog input channel number used as voltage input of this pwravg block
 * @param[out] option->i_in Analog input channel number used as current input of this pwravg block
 * @param[out] option->emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[out] option->val1 Threshold 1.
 * @param[out] option->val2 Threshold 2.
 * @param[out] option->val3 Threshold 3.
 * @param[out] option->val4 Threshold 4.
 * @param[out] option->cval Emulated current analog value.
 * @param[out] option->transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */

ifcdaqdrv_status ifcfastint_get_conf_pwravg_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               struct ifcfastint_pwravg_option *option);

/**
 * @brief Set configuration of analog channel pre-processing. Only write values according to write_mask.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-23)
 * @param[in] write_mask Specify which options to write [transition_mask, cval, val4, val3, val2, val1, emulation_en, mode]
 * @param[in] ppblock Specify which one of the 4 types of standard pre-processing this configuration refers (level, shape, rate, deviation)
 *
 * @param[in] option->mode Determine the arithmetic operation performed on input.
 * @param[in] option->emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[in] option->val1 Threshold 1.
 * @param[in] option->val2 Threshold 2.
 * @param[in] option->val3 Threshold 3.
 * @param[in] option->val4 Threshold 4.
 * @param[in] option->cval Emulated current analog value.
 * @param[in] option->transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */

ifcdaqdrv_status ifcfastint_set_conf_analog_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
											   ifcfastint_analog_pp ppblock,
                                               struct ifcfastint_analog_option *option);
/**
 * @brief Set configuration of analog channel pre-processing. Only write values according to write_mask.
 *
 * @param[in] ifcuser User struct.
 * @param[in] block Specify which block to configure (0-3)
 * @param[in] write_mask Specify which options to write [transition_mask, cval, val4, val3, val2, val1, emulation_en, mode]
 *
 * @param[in] option->mode Determine the arithmetic operation performed on input.
 * @param[in] option->u_in Analog input channel number used as voltage input of this pwravg block
 * @param[in] option->i_in Analog input channel number used as current input of this pwravg block
 * @param[in] option->emulation_en Enable Emulation, if this is true then cval is used instead of the real adc value.
 * @param[in] option->val1 Threshold 1.
 * @param[in] option->val2 Threshold 2.
 * @param[in] option->val3 Threshold 3.
 * @param[in] option->val4 Threshold 4.
 * @param[in] option->cval Emulated current analog value.
 * @param[in] option->transition_mask [0]: IDLE->PRE, [1] PRE->RUN.
 */
ifcdaqdrv_status ifcfastint_set_conf_pwravg_pp(struct ifcdaqdrv_usr *ifcuser,
                                               uint32_t block,
                                               uint32_t write_mask,
											   struct ifcfastint_pwravg_option *option);


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

/* Special functions to configure FMC clocks */
ifcdaqdrv_status ifcfastint_set_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double frequency, uint32_t fmc);
ifcdaqdrv_status ifcfastint_get_clock_frequency(struct ifcdaqdrv_usr *ifcuser, double *frequency, uint32_t fmc);

ifcdaqdrv_status ifcfastint_set_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t divisor, uint32_t fmc);
ifcdaqdrv_status ifcfastint_get_clock_divisor(struct ifcdaqdrv_usr *ifcuser, uint32_t *divisor, uint32_t fmc);

ifcdaqdrv_status ifcfastint_set_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock clock, uint32_t fmc);
ifcdaqdrv_status ifcfastint_get_clock_source(struct ifcdaqdrv_usr *ifcuser, ifcdaqdrv_clock *clock, uint32_t fmc);

ifcdaqdrv_status ifcfastint_set_history_mode(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histmode hist_mode);
ifcdaqdrv_status ifcfastint_get_history_mode(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histmode *hist_mode);
ifcdaqdrv_status ifcfastint_set_history_status(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histcontrol hist_enabled);
ifcdaqdrv_status ifcfastint_get_history_status(struct ifcdaqdrv_usr *ifcuser, ifcfastint_histcontrol *hist_enabled);
ifcdaqdrv_status ifcfastint_get_history_flags(struct ifcdaqdrv_usr *ifcuser, int32_t *over_ff, int32_t *ring_over_ff);
ifcdaqdrv_status ifcfastint_get_history_acqstate(struct ifcdaqdrv_usr *ifcuser, ifcfastint_hist_state *state);
ifcdaqdrv_status ifcfastint_get_statusreg(struct ifcdaqdrv_usr *ifcuser, int32_t *regval);
ifcdaqdrv_status ifcfastint_read_lastframe(struct ifcdaqdrv_usr *ifcuser, void *data);

ifcdaqdrv_status ifcfastint_get_rtstatus(struct ifcdaqdrv_usr *ifcuser,
                                         uint32_t aichannel,
                                         uint32_t *value,
                                         ifcfastint_analog_pp analog_pp_type);

ifcdaqdrv_status ifcfastint_init_dio3118(struct ifcdaqdrv_usr *ifcuser);
ifcdaqdrv_status ifcfastint_history_reset(struct ifcdaqdrv_usr *ifcuser);

ifcdaqdrv_status ifcfastint_set_eeprom_param(struct ifcdaqdrv_usr *ifcuser, int channel, ifcfastint_aichannel_param aiparam, double value);
ifcdaqdrv_status ifcfastint_get_eeprom_param(struct ifcdaqdrv_usr *ifcuser, int channel, ifcfastint_aichannel_param aiparam, double *value);

ifcdaqdrv_status ifcfastint_get_diagnostics(struct ifcdaqdrv_usr *ifcuser, uint32_t channel, ifcfastint_analog_pp ppblock, struct ifcfastint_analog_diag *diag_info);
ifcdaqdrv_status ifcfastint_read_measurements(struct ifcdaqdrv_usr *ifcuser, void *data);
ifcdaqdrv_status ifcfastint_set_timingmask(struct ifcdaqdrv_usr *ifcuser, uint32_t mask);

ifcdaqdrv_status ifcfastint_subs_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn);
ifcdaqdrv_status ifcfastint_wait_intr(struct ifcdaqdrv_usr *ifcuser, uint32_t irqn);


#ifdef __cplusplus
}
#endif

#endif /* _IFCFASTINT_H_ */

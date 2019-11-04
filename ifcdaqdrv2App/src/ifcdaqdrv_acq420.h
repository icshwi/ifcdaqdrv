#ifndef _ACQ420_H_
#define _ACQ420_H_ 1

#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"

#define DTACQ_SIGN_REG 0
#define DTACQ_MCSR_REG 1
#define DTACQ_TRIG_REG 2
#define DTACQ_DEL_REG  3
#define DTACQ_CH0_REG  4
#define DTACQ_CH1_REG  5
#define DTACQ_CH2_REG  6
#define DTACQ_CH3_REG  7

#define DTACQ_SIGN_HWREV_MASK       0x000000FF
#define DTACQ_SIGN_ADCRES_MASK      0x00000300
#define DTACQ_SIGN_ADCSPEED_MASK    0x00000C00
#define DTACQ_SIGN_HWVER_MASK       0x0000F000
#define DTACQ_SIGN_BOARDID_MASK     0xFFFF0000

#define DTACQ_SIGN_HWREV_SHIFT       0
#define DTACQ_SIGN_ADCRES_SHIFT      8
#define DTACQ_SIGN_ADCSPEED_SHIFT    10
#define DTACQ_SIGN_HWVER_SHIFT       12
#define DTACQ_SIGN_BOARDID_SHIFT     16

#define DTACQ_MCSR_IFC_LED_RED      0x00000001
#define DTACQ_MCSR_IFC_LED_GREEN    0x00000002
#define DTACQ_MCSR_A420_ACC_PRSNT   0x20000000
#define DTACQ_MCSR_A420_LCLK_GENA   0x40000000
#define DTACQ_MCSR_A420_GLOB_ENA    0x80000000

#define DTACQ_MCSR_IFC_LED_MASK     0x00000003

/* Data acquisition sampling rate with 10ns resolution */
#define DTACQ_CHx_ACQ_TCNT_MASK     0x000FFFFF
/* Enable Data acquisition sampling rate clock */
#define DTACQ_CHx_ACQ_TCNT_ENA_MASK 0x00100000
/* Backend interface Error */
#define DTACQ_CHx_ACQ_ERR_MASK      0x03000000
/* AD8251 pre-amplifier (1x, 2x, 4x, 8x)*/
#define DTACQ_CHx_ACQ_PREA_MASK     0x0C000000
/* Data acquisition mode 0x1-0xF */
#define DTACQ_CHx_ACQ_MOD_MASK      0xF0000000

#define DTACQ_CHx_ACQ_TCNT_SHIFT       0
#define DTACQ_CHx_ACQ_TCNT_ENA_SHIFT  20
#define DTACQ_CHx_ACQ_ERR_SHIFT       24
#define DTACQ_CHx_ACQ_PREA_SHIFT      26
#define DTACQ_CHx_ACQ_MOD_SHIFT       28

#define DTACQ_MODE_DIRECT_TEST_INC     0x1
#define DTACQ_MODE_FIFO_TEST_INC       0x2
#define DTACQ_MODE_FIFO_TEST_DEC       0x3
#define DTACQ_MODE_DIRECT_EXT_CLK      0x8
#define DTACQ_MODE_DIRECT_EXT_CLK_INFO 0x9
#define DTACQ_MODE_DIRECT_INT_CLK      0xa
#define DTACQ_MODE_DIRECT_INT_CLK_INFO 0xb
#define DTACQ_MODE_FIFO_EXT_CLK        0xc
#define DTACQ_MODE_FIFO_EXT_CLK_INFO   0xd
#define DTACQ_MODE_FIFO_INT_CLK        0xe
#define DTACQ_MODE_FIFO_INT_CLK_INFO   0xf

ifcdaqdrv_status acq420_register(struct ifcdaqdrv_dev *ifcdevice);

/*
 * Initialize d-tAcq ACQ420FMC
 */
ifcdaqdrv_status acq420_init(struct ifcdaqdrv_dev *ifcdevice);

/*
 * Set clock of channel 'channel' to frequency 'freq'. Valid range is
 * 1/2'000 ns to 1/10'485'760 ns.
 */
//ifcdaqdrv_status acq420_set_clock(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double frequency);
//ifcdaqdrv_status acq420_get_clock(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *frequency);

ifcdaqdrv_status acq420_enable_counter(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel);

ifcdaqdrv_status acq420_set_pattern(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern pattern);
ifcdaqdrv_status acq420_get_pattern(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_pattern *pattern);

ifcdaqdrv_status acq420_fmc_if_en(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status acq420_get_channel_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain);
ifcdaqdrv_status acq420_set_channel_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain);

ifcdaqdrv_status acq420_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state);

// int acq420_get_adc_resolution(struct ifcdaqdrv_dev *ifcdevice, uint32_t *resolution);

// int acq420_acq_read(struct ifcdaqdrv_dev *ifcdevice, struct pev_ioctl_buf *pev_buf, int buflen);

ifcdaqdrv_status acq420_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                             size_t nelm);
ifcdaqdrv_status acq420_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples);

ifcdaqdrv_status acq420_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status acq420_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);

ifcdaqdrv_status acq420_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency);
ifcdaqdrv_status acq420_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency);
//ifcdaqdrv_status acq420_set_clock_frequency_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double frequency);
//ifcdaqdrv_status acq420_get_clock_frequency_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *frequency);

ifcdaqdrv_status acq420_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock);
ifcdaqdrv_status acq420_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock);
ifcdaqdrv_status acq420_set_clock_source_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_clock clock);
ifcdaqdrv_status acq420_get_clock_source_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, ifcdaqdrv_clock *clock);

ifcdaqdrv_status acq420_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor);
ifcdaqdrv_status acq420_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t  *divisor);
ifcdaqdrv_status acq420_set_clock_divisor_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t  divisor);
ifcdaqdrv_status acq420_get_clock_divisor_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, uint32_t  *divisor);

ifcdaqdrv_status acq420_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                      uint16_t *board_id);

ifcdaqdrv_status acq420_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max);

ifcdaqdrv_status acq420_set_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples);
ifcdaqdrv_status acq420_get_smem_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples);

#endif // _ACQ420_H_

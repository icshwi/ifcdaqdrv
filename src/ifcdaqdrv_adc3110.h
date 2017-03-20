#ifndef _IFC_ADC3110_H_
#define _IFC_ADC3110_H_ 1

#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"

#define ADC3110_SIGN_REG                0x0
#define ADC3110_MCSR_REG                0x1
#define ADC3110_ADCLED_REG              0x2
#define ADC3110_SERIAL_BUS_CONTROL_REG  0x3
#define ADC3110_SERIAL_BUS_DATA_REG     0x4
#define ADC3110_GPIO_REG                0x5
#define ADC3110_DISC_REG                0x6
#define ADC3110_SUPPORT_REG             0x7
#define ADC3110_CH01_OFFSET_REG         0x8
#define ADC3110_CH23_OFFSET_REG         0x9
#define ADC3110_CH45_OFFSET_REG         0xA
#define ADC3110_CH56_OFFSET_REG         0xB

#define ADC3110_SUPPORT_IFC_LED_MASK     0x00000003

#define ADC3110_SIGNATURELEN 8

#define ADC3110_CLOCK_MIN 40.0e6
#define ADC3110_CLOCK_MAX 330.0e6

typedef enum {LMK04906, // LMK04906
              ADS01,    // ADS42LB69 device #01 adc channel 0 and 1
              ADS23,    // ADS42LB69 device #23 adc channel 2 and 3
              ADS45,    // ADS42LB69 device #45 adc channel 4 and 5
              ADS67     // ADS42LB69 device #67 adc channel 6 and 7
} ADC3110_SBCDEVICE;

ifcdaqdrv_status adc3110_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status adc3111_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status adc3110_init_adc(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status adc3110_adc_init_priv(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device);

ifcdaqdrv_status adc3110_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state);
ifcdaqdrv_status adc3110_set_dataformat(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat);

ifcdaqdrv_status adc3110_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency);
ifcdaqdrv_status adc3110_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency);

ifcdaqdrv_status adc3110_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor);
ifcdaqdrv_status adc3110_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor);

ifcdaqdrv_status adc3110_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock);
ifcdaqdrv_status adc3110_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock);

ifcdaqdrv_status adc3110_set_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain);
ifcdaqdrv_status adc3110_get_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain);

ifcdaqdrv_status adc3110_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern);
ifcdaqdrv_status adc3110_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern);

ifcdaqdrv_status adc3110_read(struct ifcdaqdrv_dev *ifcdevice, void *dst, size_t dst_offset, void *src, size_t src_offset, size_t nelm, size_t channel_nsamples);
ifcdaqdrv_status adc3110_read_ch(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, void *res, void *data, size_t offset,
                              size_t nelm);

ifcdaqdrv_status adc3110_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device, int addr, uint32_t
                                         data);
ifcdaqdrv_status adc3110_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3110_SBCDEVICE device, int addr,
                                                 uint32_t *value);
ifcdaqdrv_status adc3110_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status adc3110_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);

ifcdaqdrv_status ifc_read_ioxos_signature(struct ifcdaqdrv_dev *ifcdevice, struct fmc_fru_id *fru_id);

ifcdaqdrv_status adc3110_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                       uint16_t *board_id);
ifcdaqdrv_status adc3110_get_sram_nsamples_max(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max);

#endif // _IFC_ADC3110_H_

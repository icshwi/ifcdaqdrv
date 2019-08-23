#ifndef _IFCDAQDRV_ADC3117_H_
#define _IFCDAQDRV_ADC3117_H_ 1

#include "ifcdaqdrv_utils.h"

#define ADC3117_SIGN_REG                0x0
#define ADC3117_MCSR_REG                0x1
#define ADC3117_ADCLED_REG              0x2
#define ADC3117_SERIAL_BUS_CONTROL_REG  0x3
#define ADC3117_SERIAL_BUS_DATA_REG     0x4
#define ADC3117_GPIO_REG                0x5
#define ADC3117_DISC_CONTROL_REG        0x6
#define ADC3117_SUPPORT_REG             0x7
#define ADC3117_GENERAL_CONFIG_REG      0x8
#define ADC3117_CHANNEL_SELECT_REG      0x9
#define ADC3117_CHANNEL_CONFIG_REG      0xA
#define ADC3117_DISC_STATUS_REG         0xB

#define ADC3117_SIGNATURELEN 8

#define ADC3117_SUPPORT_IFC_LED_MASK         0x00000003
#define ADC3117_SUPPORT_ADC_GREEN_LED_MASK   0x00000003
#define ADC3117_SUPPORT_ADC_RED_LED_MASK     0x0000000C

typedef enum {
              ADC00,
              ADC01,
              ADC02,
              ADC03,
              ADC04,
              ADC05,
              ADC06,
              ADC07,
              ADC08,
              ADC09,
              ADC10,
              ADC11,
              ADC12,
              ADC13,
              ADC14,
              ADC15,
              ADC16,
              ADC17,
              ADC18,
              ADC19
} ADC3117_CHANNEL;

typedef enum {
              ADC_OFFSET_COMP_DAC0,
              ADC_OFFSET_COMP_DAC1,
              ADC_OFFSET_COMP_DAC2,
              ADC_OFFSET_COMP_DAC3,
              ADC_OFFSET_COMP_DAC4,
              ADC_OFFSET_COMP_DAC5,
              ADC_OFFSET_COMP_DAC6,
              ADC_OFFSET_COMP_DAC7,
              ADC_OFFSET_COMP_DAC8,
              ADC_OFFSET_COMP_DAC9,
              ADC3117_UNUSED1,
              ADC3117_UNUSED2,
              DAC0_OUTPUT_DIGITAL_POT,
              DAC1_OUTPUT_DIGITAL_POT,
              VCAL_DIGITAL_POT,
              DAC_OUTPUT
} ADC3117_SPI_DEVICE;

typedef enum {
              FROM_CONNECTOR,
              GND,
              OFFSET_COMP_VOLTAGE,
              VCAL
} ADC3117_INPUT_FUNCTION;

typedef enum {
              GAIN_x1,
              GAIN_x2,
              GAIN_x5,
              GAIN_x10
} ADC3117_GAIN;

ifcdaqdrv_status adc3117_register(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status adc3117_init_adc(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status adc3117_set_led(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_led led, ifcdaqdrv_led_state led_state);

ifcdaqdrv_status adc3117_set_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double frequency);
ifcdaqdrv_status adc3117_get_clock_frequency(struct ifcdaqdrv_dev *ifcdevice, double *frequency);

ifcdaqdrv_status adc3117_set_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t divisor);
ifcdaqdrv_status adc3117_get_clock_divisor(struct ifcdaqdrv_dev *ifcdevice, uint32_t *divisor);

ifcdaqdrv_status adc3117_set_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock clock);
ifcdaqdrv_status adc3117_get_clock_source(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_clock *clock);

ifcdaqdrv_status adc3117_set_offset(struct ifcdaqdrv_dev *ifcdevice, uint16_t offset);
ifcdaqdrv_status adc3117_get_offset(struct ifcdaqdrv_dev *ifcdevice, uint16_t *offset);

ifcdaqdrv_status adc3117_set_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double gain);
ifcdaqdrv_status adc3117_get_gain(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel, double *gain);

ifcdaqdrv_status adc3117_set_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern pattern);
ifcdaqdrv_status adc3117_get_test_pattern(struct ifcdaqdrv_dev *ifcdevice, unsigned channel, ifcdaqdrv_pattern *pattern);

ifcdaqdrv_status adc3117_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3117_SPI_DEVICE device, int addr, uint32_t
                                         data);
ifcdaqdrv_status adc3117_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3117_SPI_DEVICE device, int addr,
                                                 uint32_t *value);
ifcdaqdrv_status adc3117_set_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t threshold);
ifcdaqdrv_status adc3117_get_trigger_threshold(struct ifcdaqdrv_dev *ifcdevice, int32_t *threshold);

ifcdaqdrv_status adc3117_get_signature(struct ifcdaqdrv_dev *ifcdevice, uint8_t *revision, uint8_t *version,
                                       uint16_t *board_id);
ifcdaqdrv_status adc3117_get_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t *nsamples_max);
ifcdaqdrv_status adc3117_set_nsamples(struct ifcdaqdrv_dev *ifcdevice, uint32_t nsamples_max);

ifcdaqdrv_status adc3117_set_adc_channel(struct ifcdaqdrv_dev *ifcdevice, uint32_t channel);
ifcdaqdrv_status adc3117_get_adc_channel(struct ifcdaqdrv_dev *ifcdevice, uint32_t *channel);

ifcdaqdrv_status adc3117_set_adc_channel_mask(struct ifcdaqdrv_dev *ifcdevice, uint32_t mask);
ifcdaqdrv_status adc3117_get_adc_channel_mask(struct ifcdaqdrv_dev *ifcdevice, uint32_t *mask);

ifcdaqdrv_status adc3117_set_adc_channel_negative_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t input);
ifcdaqdrv_status adc3117_get_adc_channel_negative_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input);
ifcdaqdrv_status adc3117_set_adc_channel_positive_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t input);
ifcdaqdrv_status adc3117_get_adc_channel_positive_input(struct ifcdaqdrv_dev *ifcdevice, uint8_t *input);

ifcdaqdrv_status adc3117_set_sample_rate(struct ifcdaqdrv_dev *ifcdevice, double sample_rate);
ifcdaqdrv_status adc3117_get_sample_rate(struct ifcdaqdrv_dev *ifcdevice, double *sample_rate);

ifcdaqdrv_status adc3117_fmc_reinit(struct ifcdaqdrv_dev *ifcdevice);

ifcdaqdrv_status adc3117_configuration_command(struct ifcdaqdrv_dev *ifcdevice);
ifcdaqdrv_status adc3117_scopelite_test(struct ifcdaqdrv_dev *ifcdevice);

#endif // _IFC_ADC3117_H_

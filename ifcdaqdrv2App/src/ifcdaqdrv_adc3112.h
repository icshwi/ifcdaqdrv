#ifndef _IFC_ADC3112_H_
#define _IFC_ADC3112_H_ 1

#define ADC3112_CLOCK_MAX 1000.0

typedef enum {LMK04803B = 100, // LMK04803B / start with enum value 100 to separate them from ADC3110_SBCDEVICE enums
              ADS5409_01,      // ADS5409 device #01 adc channel 0 and 1
              ADS5409_23,      // ADS5409 device #23 adc channel 2 and 3
              DACCMP,          // DAC8563 GPIO CMP level
              SY,              // SY89297 clock delay
              XRA01,           // XRA1404 IN 01
              XRA23,           // XRA1404 IN 23
              XRATRIG          // XRA1404 Trigger
} ADC3112_SBCDEVICE;

int adc3112_init_adc(struct ifcdaqdrv_dev *ifcdevice);

int adc3112_set_dataformat(struct ifcdaqdrv_dev *ifcdevice, ifcdaqdrv_dataformat dataformat);

int ifc_adc3112_SerialBus_write(struct ifcdaqdrv_dev *ifcdevice, ADC3112_SBCDEVICE device, int addr, uint32_t data);
uint32_t ifc_adc3112_SerialBus_read(struct ifcdaqdrv_dev *ifcdevice, ADC3112_SBCDEVICE device, int addr);

#endif // _IFC_ADC3112_H_

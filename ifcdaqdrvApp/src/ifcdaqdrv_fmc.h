#ifndef _IFCDAQDRV_FMC_H_
#define _IFCDAQDRV_FMC_H_ 1

struct fmc_fru_id { char *manufacturer;
                    char *product_name;
                    char *part_num;
                    char *serial;
};

#define IFC_FMC1_I2C_BASE 0x80000000
#define IFC_FMC2_I2C_BASE 0xA0000000

/* IPMI FRU defines */
#define FRU_COUNT_BYTES_MASK 0x3f

ifcdaqdrv_status ifc_fmc_eeprom_read_field(struct ifcdaqdrv_dev *ifcdevice, uint16_t offset, char **field,
                                           uint8_t *len);
ifcdaqdrv_status ifc_fmc_eeprom_read_string(struct ifcdaqdrv_dev *ifcdevice, uint16_t address, int len, char *buf, int
                                            buflen);
ifcdaqdrv_status ifc_fmc_eeprom_read_fru(struct ifcdaqdrv_dev *ifcdevice, struct fmc_fru_id *fru_id);
ifcdaqdrv_status ifc_fmc_eeprom_read(struct ifcdaqdrv_dev *ifcdevice, uint16_t address, uint8_t *data);

ifcdaqdrv_status ifc_fmc_eeprom_read_sig(struct ifcdaqdrv_dev *ifcdevice, uint8_t *data);

#endif // _IFCDAQDRV_FMC_H_

#ifndef _IFC1210SCOPEDRV_FMC_H_
#define _IFC1210SCOPEDRV_FMC_H_ 1

typedef enum {
    FMC_NOTINSTALLED,

    FMC_ADC_3110,
    FMC_ADC_3111,
    FMC_ADC_3112,
    FMC_ADC_3113,

    FMC_DTACQ_ACQ420,

    FMC_UNKNOWN
} FMCTYP;

const char *fmctypToName(FMCTYP fmctyp);

/* Temporary, don't think I need this.. */
struct fmcinfo {
    struct fmc_fru_id *fru_id;
    uint32_t           Signature;
    FMCTYP             fmcTyp;
    uint32_t           HWRevision;
    uint32_t           HWVersion;
    const char        *description;
    const char        *descriptionshort;
    uint32_t           acqDPRAM_BufferSize;          // number of int16_t samples available
    uint32_t           acqDPRAM_BufferSize_Reserved; // number of int16_t samples reserved space (z.b XUSER SCOPE 64KByte(32ksps))
    uint32_t           nChannels;                    // number of channels
};

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



#endif // _IFC1210SCOPEDRV_FMC_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>

#ifdef TOSCA_USRLIB
// #include <pevioctl.h>
// #include <pevxulib.h>
// #include <pevulib.h>
#include <tscioctl.h>
#include <tsculib.h>
#endif

#include "debug.h"
#include "ifcdaqdrv2.h"
#include "ifcdaqdrv_utils.h"
#include "ifcdaqdrv_fmc.h"


ifcdaqdrv_status ifc_fmc_eeprom_read(struct ifcdaqdrv_dev *ifcdevice, uint16_t address, uint8_t *data){
    int  status = 0;
    uint device = 0x01010051;
    uint reg_val;

    if (!data) {
        return status_argument_invalid;
    }

    if (address > 0x7fff) {
        return status_argument_range;
    }

    switch (ifcdevice->fmc) {
    case 1:
        device |= IFC_FMC1_I2C_BASE;
        device += 1;     // Don't know why this is needed, found in  XprsMon/adc3110.c
        break;
    case 2:
        device |= IFC_FMC2_I2C_BASE;
        break;
    }
    
#ifdef TOSCA_USRLIB
    /* TODO: check usage of i2c_read */
    //status = pevx_i2c_read(ifcdevice->card, device, pev_swap_16(address), &reg_val);
    status = tsc_i2c_read(ifcdevice->card, device, tsc_swap_16(address), &reg_val);

    if (!((status & I2C_CTL_EXEC_MASK) == I2C_CTL_EXEC_DONE)) {
        return status_i2c_nack;
    }
#else
    reg_val = 0;
#endif


    *data = (uint8_t)reg_val;
    return status_success;

#if DEBUG
    printf("%s(fmc=%d,address=0x%04x) -> uint8_t [%c]\n", __FUNCTION__, ifcdevice->fmc, address, *data);
#endif

    return -1;
}

ifcdaqdrv_status ifc_fmc_eeprom_read_string(struct ifcdaqdrv_dev *ifcdevice, uint16_t address, int len, char *buf, int
                                            buflen){
    uint16_t         i;
    uint8_t          data;
    char            *p;
    ifcdaqdrv_status status;

    if (!buf) {
        return status_argument_invalid;
    }

    p  = buf;
    *p = '\0';

    if (len > (buflen - 1)) {
        len = buflen - 1;
    }

    for (i = 0; i < len; i++, p++) {
        status = ifc_fmc_eeprom_read(ifcdevice, address + i, &data);
        if (status) {
            return status;
        }
        *p = data;
    }
    *p = '\0';

#if DEBUG
    printf("%s(): address=0x%04x -> string = %s\n", __FUNCTION__, address, buf);
#if 0
    printf("HEX: ");
    for (i = 0; i < buflen; ++i) {
        printf("%x", buf[i]);
    }
    printf("\n");
    printf("CHAR: ");
    for (i = 0; i < buflen; ++i) {
        printf("%c", buf[i]);
    }
    printf("\n");
#endif
#endif

    return status_success;
}

/* Allocate memory here because fields are variable length */
ifcdaqdrv_status ifc_fmc_eeprom_read_field(struct ifcdaqdrv_dev *ifcdevice, uint16_t offset, char **field,
                                           uint8_t *len){
    ifcdaqdrv_status status;
    uint8_t          read_len;
    char            *p;
    if (!field) {
        return status_argument_invalid;
    }

    status = ifc_fmc_eeprom_read(ifcdevice, offset, &read_len);
    if (status) {
        return status;
    }
#if DEBUG
    printf("Type/Length: %d%d %d\n", (read_len & 0x80) >> 7, (read_len & 0x40) >> 6, read_len & 0x3f);
#endif
    if (read_len == 0xC1) {
        // This indicates End of Fields.
        return status_read;
    }
    read_len &= FRU_COUNT_BYTES_MASK;
    p         = calloc(sizeof(char), read_len + 1);   // String and null terminator
    status    = ifc_fmc_eeprom_read_string(ifcdevice, offset + 1, read_len, p, read_len + 1);
    if (status) {
        return status;
    }
    p[read_len] = '\0';
#if DEBUG
    printf("Value: %.*s\n", read_len, p);
#endif
    *field = p;
    *len   = read_len;
    return 0;
}

ifcdaqdrv_status ifc_fmc_eeprom_read_fru(struct ifcdaqdrv_dev *ifcdevice, struct fmc_fru_id *fru_id){
    char             fru_header[8];
    ifcdaqdrv_status status;

    if (!fru_id) {
        return status_argument_invalid;
    }

    status = ifc_fmc_eeprom_read_string(ifcdevice, 0, sizeof(fru_header), fru_header, sizeof(fru_header));
    if (status) {
        return status;
    }
#if DEBUG
    int i;
    printf("FRU HEADER: ");
    for (i = 0; i < sizeof(fru_header); ++i) {
        printf("%x", fru_header[i]);
    }
    printf("\n");
    printf("Version:           0x%02x\n", fru_header[0]);
    printf("Internal use:      0x%02x\n", fru_header[1]);
    printf("Chassis info area: 0x%02x\n", fru_header[2]);
    printf("Board area:        0x%02x\n", fru_header[3]);
    printf("Product info area: 0x%02x\n", fru_header[4]);
    printf("MultiRecord area:  0x%02x\n", fru_header[5]);
    printf("Padding            0x%02x\n", fru_header[6]);
    printf("Checksum:          0x%02x\n", fru_header[7]);
#endif

    /* Only version 1 of the IPMI specification is supported */
    if (!((fru_header[0] & 0xf) & 0x1)) {
        return status_fru_info_invalid;
    }

    uint16_t offset = fru_header[3] * 8;
    uint8_t  board_area_version;
    uint8_t  read_len;

    status = ifc_fmc_eeprom_read(ifcdevice, offset, &board_area_version);
    if (status) {
        return status;
    }

    /* Only version 1 of the Board Info Area specification is supported */
    if (!((board_area_version & 0xf) & 0x1)) {
        return status_fru_info_invalid;
    }

    offset += 6;
    ifc_fmc_eeprom_read_field(ifcdevice, offset, &fru_id->manufacturer, &read_len);

    offset += 1 + read_len;
    ifc_fmc_eeprom_read_field(ifcdevice, offset, &fru_id->product_name, &read_len);

    offset += 1 + read_len;
    ifc_fmc_eeprom_read_field(ifcdevice, offset, &fru_id->serial,       &read_len);

    offset += 1 + read_len;
    ifc_fmc_eeprom_read_field(ifcdevice, offset, &fru_id->part_num,     &read_len);

#if DEBUG
    offset += 1 + read_len;
    char *fru_file_id;
    ifc_fmc_eeprom_read_field(ifcdevice, offset, &fru_file_id, &read_len);

    offset += 1 + read_len;
    status  = ifc_fmc_eeprom_read_field(ifcdevice, offset, NULL, &read_len);
    if (status) {
        printf("end of field\n");
    }
    printf("Manufacturor: %s\nProduct name: %s\nSerial Number: %s\nPart Number:%s\nFRU File ID:%s\n",
           fru_id->manufacturer, fru_id->product_name, fru_id->serial, fru_id->part_num, fru_file_id);
#endif

    return status_success;
}

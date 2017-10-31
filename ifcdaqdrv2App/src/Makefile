TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================

# build a support library
LIBRARY_linux-ppc64e6500 += ifcdaqdrv2
LIBRARY = $(LIBRARY_$(T_A))

# install ifcdaqdrv2.dbd into <top>/dbd
DBD += ifcdaqdrv2.dbd
ifcdaqdrv2_DBD += debugEpics.dbd

# specify all source files to be compiled and added to the library
ifcdaqdrv2_SRCS += debugEpics.c ifcdaqdrv_acq420.c ifcdaqdrv_adc3110.c ifcdaqdrv_adc3112.c ifcdaqdrv_adc3117.c
ifcdaqdrv2_SRCS += ifcdaqdrv.c ifcdaqdrv_fmc.c ifcdaqdrv_scope.c ifcdaqdrv_utils.c
ifcdaqdrv2_SRCS += ifcfastintdrv.c ifcfastintdrv_utils.c

# tsclib should be defined in configure/RELEASE
ifcdaqdrv2_LIBS += tsclib
ifcdaqdrv2_LIBS += $(EPICS_BASE_IOC_LIBS)

INC += ifcdaqdrv2.h
INC += ifcfastintdrv.h

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


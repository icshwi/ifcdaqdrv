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
ifcdaqdrv2_SRCS += debugEpics.c ifcdaqdrv_adc3110.c ifcdaqdrv.c ifcdaqdrv_fmc.c 
ifcdaqdrv2_SRCS += ifcdaqdrv_scope.c ifcdaqdrv_utils.c

# tsclib should be defined in configure/RELEASE
ifcdaqdrv2_LIBS += tsclib
ifcdaqdrv2_LIBS += $(EPICS_BASE_IOC_LIBS)

INC += ifcdaqdrv2.h

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Use typed rset structure (see 3.16.1 release notes)
#USR_CPPFLAGS += -DUSE_TYPED_RSET

# build a support library only for ppc64e6500 architecture
#LIBRARY_linux-ppc64e6500 += ifcdaqdrv
#LIBRARY = $(LIBRARY_$(T_A))

# build an IOC support library
LIBRARY_IOC += ifcdaqdrv

# install ifcdaqdrv.dbd into <top>/dbd
DBD += ifcdaqdrv.dbd
ifcdaqdrv_DBD += debugEpics.dbd

# specify all source files to be compiled and added to the library
ifcdaqdrv_SRCS += debugEpics.c ifcdaqdrv_adc3110.c ifcdaqdrv_adc3112.c ifcdaqdrv_adc3117.c ifcdaqdrv_dio3118.c
ifcdaqdrv_SRCS += ifcdaqdrv.c ifcdaqdrv_fmc.c ifcdaqdrv_scope.c ifcdaqdrv_scope_lite.c ifcdaqdrv_utils.c ifcdaqdrv_gen_scope.c
ifcdaqdrv_SRCS += ifcfastintdrv.c ifcfastintdrv_utils.c

# tsclib should be defined in configure/RELEASE
ifcdaqdrv2_LIBS += tsclib
ifcdaqdrv2_LIBS += $(EPICS_BASE_IOC_LIBS)

# files to be installed on include folder
INC += ifcdaqdrv.h
INC += ifcfastintdrv2.h

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = ifcdaqdrvVersion.h
# Macro name
GENVERSIONMACRO = ifcdaqdrvVERSION

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


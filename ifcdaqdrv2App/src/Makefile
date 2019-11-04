TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Use typed rset structure (see 3.16.1 release notes)
#USR_CPPFLAGS += -DUSE_TYPED_RSET

# build a support library only for ppc64e6500 architecture
#LIBRARY_linux-ppc64e6500 += ifcdaqdrv2
#LIBRARY = $(LIBRARY_$(T_A))

# build an IOC support library
LIBRARY_IOC += ifcdaqdrv2

# install ifcdaqdrv2.dbd into <top>/dbd
DBD += ifcdaqdrv2.dbd
ifcdaqdrv2_DBD += debugEpics.dbd

# specify all source files to be compiled and added to the library
ifcdaqdrv2_SRCS += debugEpics.c ifcdaqdrv_acq420.c ifcdaqdrv_adc3110.c ifcdaqdrv_adc3112.c ifcdaqdrv_adc3117.c ifcdaqdrv_dio3118.c
ifcdaqdrv2_SRCS += ifcdaqdrv.c ifcdaqdrv_fmc.c ifcdaqdrv_scope.c ifcdaqdrv_utils.c
ifcdaqdrv2_SRCS += ifcfastintdrv.c ifcfastintdrv_utils.c

# tsclib should be defined in configure/RELEASE
ifcdaqdrv2_LIBS += tsclib
ifcdaqdrv2_LIBS += $(EPICS_BASE_IOC_LIBS)

# files to be installed on include folder
INC += ifcdaqdrv2.h
INC += ifcfastintdrv2.h

# Auto-generate a header file containing a version string.
# Version comes from the VCS if available, else date+time.
GENVERSION = ifcdaqdrv2Version.h
# Macro name
GENVERSIONMACRO = ifcdaqdrv2VERSION

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE


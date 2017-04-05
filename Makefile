PROJECT=ifcdaqdrv2

include ${EPICS_ENV_PATH}/module.Makefile

EXCLUDE_ARCHS = eldk56

SOURCES = $(wildcard src/*.c)
HEADERS = $(wildcard include/*.h)

# Compile away some debug code if compiling with optimizations
#OPT_CPPFLAGS_YES = -DNDEBUG

# Enable debug, disabled by default.
# HOST_OPT = NO
# CROSS_OPT = NO

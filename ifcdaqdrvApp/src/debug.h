/*
 * Debug printing
 *
 * This debug printing functionality is implemented so that the compiler always sees the debug printing code.
 *
 * - Compile with -DNDEBUG to disable DEBUG and TRACE print levels.
 *   Use if(DEBUG){} to remove code that is only needed for DEBUG/TRACE levels.
 * - Set ifcdaqdrvDebug to a LEVEL to print messages on that level and below. Will always print errors.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>

#define UNUSED(x) (void)(x)

/*
 * Usage LOG((LEVEL, fmt, args...))
 *
 * Typical levels are 5 (notice), 6 (info), 7 (debug).
 *
 * -   LEVEL  debug verbosity number
 * -     fmt  printf format
 * - args...  printf arguments
 */

#define LEVEL_ERROR 3
#define LEVEL_WARNING 4
#define LEVEL_NOTICE 5
#define LEVEL_INFO 6
#define LEVEL_DEBUG 7
#define LEVEL_TRACE 8

static const char __attribute__((unused)) *level_str[] = {"", "", "", "E", "W", "N", "I", "D", "T"};

#ifdef NDEBUG
#define DEBUG 0
#else
#define DEBUG 1
#endif /* NDEBUG */

#define LOG(x)    do { LOG_ARGS x; } while (0)

#define LOG_ARGS(level, fmt, args...) \
    if(DEBUG || level <= LEVEL_INFO) { \
      if(level <= ifcdaqdrvDebug || level <= 3) { \
        fprintf(stderr, "[IFC1410 driver] [%s] %s:%d:%s(): ", level_str[level], __FILE__, __LINE__, __func__); \
        fprintf(stderr, fmt, ## args); \
      } \
    }

#define INFOLOG(msg)	do { INFOLOG_ARGS msg; } while (0)
#define INFOLOG_ARGS(fmt, args...) \
    fprintf(stderr, "[IFC1410 driver] "); \
    fprintf(stderr, fmt, ## args);

extern int32_t ifcdaqdrvDebug;

#endif /* DEBUG_H */

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

static const char *level_str[] = {"", "", "", "E", "W", "N", "I", "D", "T"};

#ifdef NDEBUG
#define DEBUG 0
#else
#define DEBUG 1
#endif /* NDEBUG */
 // #define DEBUG 1


#define ENABLE_TRACE_IOC
#define NENABLE_TRACE_SERIAL

// #define ENABLE_I2C

#define I2C_CTL_EXEC_IDLE 0x00000000
#define I2C_CTL_EXEC_RUN  0x00100000
#define I2C_CTL_EXEC_DONE 0x00200000
#define I2C_CTL_EXEC_ERR  0x00300000
#define I2C_CTL_EXEC_MASK 0x00300000

#ifdef ENABLE_TRACE_IOC
	#define TRACE_IOC fprintf(stderr,"[TRACE] Program has enter %s\n",__func__)
	// #define TRACE_PARAMD(par, val) printf("  [SET PARAM] %s = %f\n", par, val)
	// #define TRACE_PARAM(par, val) printf("  [SET PARAM] %s = %d\n", par, val)

	// #define TRACE_GET_PARAMD(par, val) printf("  [PARAM] %s = %f\n", par, val)
	// #define TRACE_GET_PARAM(par, val) printf("  [PARAM] %s = %d\n", par, val)

	#define TRACE_INIT(msg) printf("[INITIALIZATION] %s\n", msg)

#else
	#define TRACE_INIT(msg) fprintf(stderr,"")
	#define TRACE_IOC fprintf(stderr,"")
	// #define TRACE_PARAMD(par, val) printf("")
	// #define TRACE_PARAM(par, val) printf("")
	// #define TRACE_GET_PARAMD(par, val) printf("")
	// #define TRACE_GET_PARAM(par, val) printf("")

#endif


#define LOG(x)    do { LOG_ARGS x; } while (0)

#define LOG_ARGS(level, fmt, args...) \
    if(DEBUG || level <= LEVEL_INFO) { \
      if(level <= ifcdaqdrvDebug || level <= 3) { \
        fprintf(stderr, "[%s] %s:%d:%s(): ", level_str[level], __FILE__, __LINE__, __func__); \
        fprintf(stderr, fmt, ## args); \
      } \
    }

#define UNUSED(x) (void)(x)

extern int32_t ifcdaqdrvDebug;

#endif /* DEBUG_H */

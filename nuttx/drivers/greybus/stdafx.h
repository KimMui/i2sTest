// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#if 0
#ifndef _WIN32_WINNT		// Allow use of features specific to Windows XP or later.                   
#define _WIN32_WINNT 0x0501	// Change this to the appropriate value to target other versions of Windows.
#endif						

#include "windows.h"
#include "tchar.h"
#include "wavefile.h"
#endif

//#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <nuttx/arch.h>

//typedef int16_t Int16;
//typedef int32_t Int32;

// Use TI Types
#if 0
typedef INT16 Int16;
typedef INT32 Int32;
typedef UINT32 Uint32;
typedef VOID Void;
typedef BOOL Bool;

typedef INT32 XDAS_Int32;
typedef INT8 XDAS_Int8;
#else
typedef int16_t Int16;
typedef int16_t INT16;
typedef int32_t INT32;
typedef int32_t Int32;
typedef uint16_t UINT16;
typedef uint32_t UINT32;
typedef uint32_t Uint32;
typedef void Void;
typedef bool Bool;

typedef int32_t XDAS_Int32;
typedef int8_t  XDAS_Int8;
#endif



#define XDM_MAX_IO_BUFFERS      16          /**< Max I/O Buffers */
/**
 *  @brief      Single buffer descriptor.
 */
typedef struct XDM1_SingleBufDesc {
    XDAS_Int8   *buf;       /**< Pointer to a buffer address. */
    XDAS_Int32  bufSize;    /**< Size of @c buf in 8-bit bytes. */
    XDAS_Int32  accessMask; /**< Mask filled by the algorithm, declaring
                             *   how the buffer was accessed <b>by the
                             *   algorithm processor</b>.
                             *
                             *   @remarks  If the buffer was <b>not</b>
                             *             accessed by the algorithm
                             *             processor (e.g., it was filled
                             *             via DMA or other hardware
                             *             accelerator that <i>doesn't</i>
                             *             write through the algorithm's
                             *             CPU), then no bits in this mask
                             *             should be set.
                             *
                             *   @remarks  It is acceptible (and
                             *             appropriate!)to set several
                             *             bits in this mask if the
                             *             algorithm accessed the buffer
                             *             in several ways.
                             *
                             *   @remarks  This mask is often used by the
                             *             application and/or framework
                             *             to appropriately manage cache
                             *             on cache-based systems.
                             *
                             *   @sa XDM_AccessMode
                             */
} XDM1_SingleBufDesc;


typedef struct XDM1_BufDesc {
    XDAS_Int32   numBufs;   /**< Number of buffers. */
    XDM1_SingleBufDesc descs[XDM_MAX_IO_BUFFERS]; /** Array of buffer
                             * descriptors.
                             */
} XDM1_BufDesc;

/**
 *  @brief      Defines the run time output arguments for
 *              all IAUDDEC1 instance objects.
 *
 *  @extensibleStruct
 *
 *  @sa         IAUDDEC1_Fxns::process()
 */
typedef struct IAUDDEC1_OutArgs {
    XDAS_Int32 size;            /**< @sizeField */
    XDAS_Int32 extendedError;   /**< @extendedErrorField */
    XDAS_Int32 bytesConsumed;   /**< Number of bytes consumed during the
                                 *   process() call.
                                 */
    XDAS_Int32 numSamples;      /**< Number of output samples per channel. */
    XDAS_Int32 channelMode;     /**< Output Channel Configuration.
                                 *
                                 *   @sa    IAUDIO_ChannelMode
                                 */
    XDAS_Int32 lfeFlag;         /**< Flag indicating the precense of LFE
                                 *   channel in the output.  When the LFE
                                 *   channel is not in the output, this is set
                                 *   to XDAS_FALSE.
                                 *
                                 *   Valid values for this field are XDAS_TRUE
                                 *   and XDAS_FALSE.
                                 */
    XDAS_Int32 dualMonoMode;    /**< Mode to indicate type of Dual Mono.
                                 *   Only used in case of Dual Mono output.
                                 *
                                 *   @sa IAUDIO_DualMonoMode
                                 */
    XDAS_Int32 sampleRate;      /**< Sampling frequency, in Hz.
                                 *   For example, if the sampling
                                 *   frequency is 44.1 kHz, this field will
                                 *   be 44100.
                                 */
} IAUDDEC1_OutArgs;

/*!
Type definition for a 16-bit signed, stereo sample (interleaved)
*/
typedef struct _S16_t
{
    Int16 l;
    Int16 r;
} Sample;

#if 0
#define GT_0trace(zone,TIclass,string) _tprintf(TEXT(string));
#define GT_1trace(zone,TIclass,string,arg_1) _tprintf(TEXT(string),arg_1);
#define GT_2trace(zone,TIclass,string,arg_1,arg_2) _tprintf(TEXT(string),arg_1,arg_2);
#else
#define GT_0trace(zone,TIclass,string)
#define GT_1trace(zone,TIclass,string,arg_1)
#define GT_2trace(zone,TIclass,string,arg_1,arg_2)
#endif

// TODO: reference additional headers your program requires here

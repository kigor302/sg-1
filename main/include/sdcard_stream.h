/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef _SDCARD_STREAM_H_
#define _SDCARD_STREAM_H_

#include "audio_error.h"
#include "audio_element.h"
#include "audio_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_DATA_BLOCKS     (160)
/**
 * @brief   SDCARD Stream configurations, if any entry is zero then the configuration will be set to default values
 */
typedef struct {
    audio_stream_type_t     type;           /*!< Stream type */
    int                     buf_sz;         /*!< Audio Element Buffer size */
    int                     out_rb_size;    /*!< Size of output ringbuffer */
    int                     task_stack;     /*!< Task stack size */
    int                     task_core;      /*!< Task running in core (0 or 1) */
    int                     task_prio;      /*!< Task priority (based on freeRTOS priority) */
    int                     buffered_size;  /*!< Number of blocks in the buffer */
} sdcard_stream_cfg_t;


#define SDCARD_STREAM_BUF_SIZE            (2048)
#define SDCARD_STREAM_TASK_STACK          (3072)
#define SDCARD_STREAM_TASK_CORE           (0)
#define SDCARD_STREAM_TASK_PRIO           (4)
#define SDCARD_STREAM_RINGBUFFER_SIZE     (8 * 1024)
#define SDCARD_STREAM_BUFFERED_SIZE       (100)

#define SDCARD_STREAM_CFG_DEFAULT() {\
    .task_prio = SDCARD_STREAM_TASK_PRIO, \
    .task_core = SDCARD_STREAM_TASK_CORE, \
    .task_stack = SDCARD_STREAM_TASK_STACK, \
    .out_rb_size = SDCARD_STREAM_RINGBUFFER_SIZE, \
    .buf_sz = SDCARD_STREAM_BUF_SIZE, \
    .buffered_size = SDCARD_STREAM_BUFFERED_SIZE, \
}

/**
 * @brief      Create a handle to an Audio Element to stream data from SDCARD to another Element
 *             or get data from other elements written to SDCARD, depending on the configuration
 *             the stream type, either AUDIO_STREAM_READER or AUDIO_STREAM_WRITER.
 *
 * @param      config  The configuration
 *
 * @return     The Audio Element handle
 */
audio_element_handle_t sdcard_stream_init(sdcard_stream_cfg_t *config);

#ifdef __cplusplus
}
#endif

#endif

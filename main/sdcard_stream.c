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

#include <sys/unistd.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <esp_types.h>
#include <stdatomic.h>
#include "errno.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sdcard_stream.h"
#include "audio_common.h"
#include "audio_mem.h"
#include "audio_element.h"
#include "wav_head.h"
#include "esp_log.h"
#include "audio_mutex.h"

#define FILE_WAV_SUFFIX_TYPE  "wav"
#define FILE_OPUS_SUFFIX_TYPE "opus"
#define FILE_AMR_SUFFIX_TYPE "amr"
#define FILE_AMRWB_SUFFIX_TYPE "Wamr"


#define MIN_WAIT_TICKS (10 / portTICK_PERIOD_MS)
#define min(a,b) ((a) < (b) ? (a) : (b))

static const char *TAG = "SDCARD_STREAM";

typedef enum {
    STREAM_TYPE_UNKNOW,
    STREAM_TYPE_WAV,
    STREAM_TYPE_OPUS,
    STREAM_TYPE_AMR,
    STREAM_TYPE_AMRWB,
} wr_stream_type_t;


typedef struct evt_block
{
    #define BL_FLAG_BUSY   (1<<0)
    #define BL_FLAG_WRITE  (1<<1)
    #define BL_FLAG_DONE   (1<<2)
    uint16_t flags;
    uint16_t len;
    FILE *   fd;
    uint8_t  data[SDCARD_STREAM_BUF_SIZE];
}evt_block_t;


typedef struct sdcard_main_obj
{
    #define JOB_QUEUE_READ   0
    #define JOB_QUEUE_WRITE  1
    bool           isRunning;
    QueueHandle_t  job_queue[2];
    int            wait_time[2];
    int            block_nums;
    evt_block_t  * blocks;
    xSemaphoreHandle lock;
    TaskHandle_t     task_handle;
}sdcard_main_obj_t;

typedef struct sdcard_stream {
    audio_stream_type_t type;
    bool is_open;
    FILE *file;
    wr_stream_type_t w_type;
    uint8_t  buffered_size;
    uint8_t  blk_head, blk_tail;
    uint8_t  blk_queue[MAX_DATA_BLOCKS];
} sdcard_stream_t;

static sdcard_main_obj_t * m_obj = NULL;
static int m_reference = 0;

/*==========================================================*/
static uint8_t sdcard_take_blk(sdcard_main_obj_t * obj)
{
    mutex_lock(obj->lock);
    for (uint8_t i=0; i<obj->block_nums; i++)
        if ( !(obj->blocks[i].flags & BL_FLAG_BUSY) )
        {
            obj->blocks[i].flags |= BL_FLAG_BUSY;
            //ESP_LOGW(TAG, " - Took buf %d", i);
            mutex_unlock(obj->lock);
            return i;
        }
    mutex_unlock(obj->lock);
    //ESP_LOGW(TAG, " - No buf %d",obj->blocks[0].flags);
    return MAX_DATA_BLOCKS;
}

static void sdcard_release_blk(sdcard_main_obj_t * obj, uint8_t idx)
{
    mutex_lock(obj->lock);
    if (idx < obj->block_nums)
        obj->blocks[idx].flags &= ~BL_FLAG_BUSY;
    mutex_unlock(obj->lock);
    //ESP_LOGW(TAG, " - Released buf %d", idx);
}

static evt_block_t * sdcard_waiting_msg(sdcard_main_obj_t * obj, int queue_type)
{
    evt_block_t * msg = NULL;
    if (obj->job_queue[queue_type])
    {
        if (xQueueReceive(obj->job_queue[queue_type], (void *)&msg, obj->wait_time[queue_type]) == pdTRUE)
        {
            //ESP_LOGW(TAG, " - Receive from queue %p message=%p", obj->job_queue[queue_type], msg);
        }
    }
        
    return msg;
}

static esp_err_t sdcard_send_msg(sdcard_main_obj_t * obj, int queue_type, evt_block_t * msg)
{
    if (obj->job_queue[queue_type])
        if (xQueueSend(obj->job_queue[queue_type], (void *)&msg, 0) != pdPASS)
        {
            ESP_LOGW(TAG, " - There are no space to dispatch queue %d", queue_type);    
            return ESP_FAIL;
        }

    return ESP_OK;
}

static void sdcard_proccess_work(void * ctx)
{
    sdcard_main_obj_t * obj = m_obj;
    evt_block_t  * blk;
    //FILE * fd_sync;

    while (obj->isRunning)
    {
        while ((blk = sdcard_waiting_msg(obj, JOB_QUEUE_READ)) != NULL)
        {
            if (blk->fd)
               blk->len = fread(blk->data, 1, ((blk->len)?blk->len:SDCARD_STREAM_BUF_SIZE), blk->fd);
                
            blk->flags |= BL_FLAG_DONE;
            //ESP_LOGW(TAG, "- readed done %d", blk->len);
        }

        //fd_sync = NULL;
        while ((blk = sdcard_waiting_msg(obj, JOB_QUEUE_WRITE)) != NULL)
        {
            if (blk->fd)
            {
               blk->len = fwrite(blk->data, 1, ((blk->len)?blk->len:SDCARD_STREAM_BUF_SIZE), blk->fd);
            }
                
            blk->flags |= BL_FLAG_DONE;
            //fd_sync = blk->fd;
        }
        //if (fd_sync)
        //    fsync(fileno(fd_sync));
    }

    ESP_LOGD(TAG, "- Task exited and deleted %d", uxTaskGetStackHighWaterMark(NULL));
    vTaskDelete(NULL);
}


static void sdcard_clear_internals(void)
{
    if (m_obj)
    {
        m_obj->isRunning = false;
        vTaskDelay(m_obj->wait_time[JOB_QUEUE_READ] + m_obj->wait_time[JOB_QUEUE_WRITE]);
        
        /*
        while (eTaskGetState(m_obj->task_handle) == eRunning || 
               eTaskGetState(m_obj->task_handle) == eBlocked)
            vTaskDelay(1);
        */

        if (m_obj->job_queue[JOB_QUEUE_READ])
            vQueueDelete(m_obj->job_queue[JOB_QUEUE_READ]);
        if (m_obj->job_queue[JOB_QUEUE_WRITE])
            vQueueDelete(m_obj->job_queue[JOB_QUEUE_WRITE]);
        if (m_obj->lock)
            mutex_destroy(m_obj->lock);
        if (m_obj->blocks)
            free(m_obj->blocks);
        
        free(m_obj);
        m_obj = NULL;
    }
}

static esp_err_t sdcard_init_internals(uint8_t block_nums, int stack_size, int task_prio, int task_core)
{
    if (m_obj == NULL)
    {
        if ( (m_obj = calloc(1, sizeof(sdcard_main_obj_t))) == NULL ||
             (m_obj->blocks = calloc(block_nums, sizeof(evt_block_t))) == NULL)
        {
            ESP_LOGE(TAG, " - Error memory allocation");
            sdcard_clear_internals();
            return ESP_FAIL;
        }
        
        //memset(m_obj->blocks,'\0', sizeof(evt_block_t)*block_nums);

        m_obj->block_nums = block_nums;
        m_obj->lock = mutex_create();
        m_obj->isRunning = true;

        m_obj->wait_time[JOB_QUEUE_READ] = m_obj->wait_time[JOB_QUEUE_WRITE] = MIN_WAIT_TICKS;
        if ((m_obj->job_queue[JOB_QUEUE_READ] = xQueueCreate(block_nums, sizeof(void *))) == NULL ||
            (m_obj->job_queue[JOB_QUEUE_WRITE] = xQueueCreate(block_nums, sizeof(void *))) == NULL)
        {
            ESP_LOGE(TAG, "- Error create queues");
            sdcard_clear_internals();
            return ESP_FAIL;
        }

        if (xTaskCreatePinnedToCore(sdcard_proccess_work, "sd_stream", stack_size, (void *)m_obj, task_prio, &m_obj->task_handle, task_core) != pdPASS) 
        {
            ESP_LOGE(TAG, "- Error create internal task");
            sdcard_clear_internals();
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}



static wr_stream_type_t get_type(const char *str)
{
    char *relt = strrchr(str, '.');
    if (relt != NULL) {
        relt ++;
        ESP_LOGD(TAG, "result = %s", relt);
        if (strncasecmp(relt, FILE_WAV_SUFFIX_TYPE, 3) == 0) {
            return STREAM_TYPE_WAV;
        } else if (strncasecmp(relt, FILE_OPUS_SUFFIX_TYPE, 4) == 0) {
            return STREAM_TYPE_OPUS;
        } else if (strncasecmp(relt, FILE_AMR_SUFFIX_TYPE, 3) == 0) {
            return STREAM_TYPE_AMR;
        } else if (strncasecmp(relt, FILE_AMRWB_SUFFIX_TYPE, 4) == 0) {
            return STREAM_TYPE_AMRWB;
        } else {
            return STREAM_TYPE_UNKNOW;
        }
    } else {
        return STREAM_TYPE_UNKNOW;
    }
}


static esp_err_t _sdcard_open(audio_element_handle_t self)
{
    sdcard_stream_t *sdcard = (sdcard_stream_t *)audio_element_getdata(self);

    audio_element_info_t info;
    char *uri = audio_element_get_uri(self);
    if (uri == NULL) {
        ESP_LOGE(TAG, "Error, uri is not set");
        return ESP_FAIL;
    }
    ESP_LOGD(TAG, "_sdcard_open, uri:%s", uri);
    char *path = strstr(uri, "/sdcard");
    audio_element_getinfo(self, &info);
    if (path == NULL) {
        ESP_LOGE(TAG, "Error, need file path to open");
        return ESP_FAIL;
    }
    if (sdcard->is_open) {
        ESP_LOGE(TAG, "already opened");
        return ESP_FAIL;
    }
    if (sdcard->type == AUDIO_STREAM_READER) {
        sdcard->file = fopen(path, "r");
        struct stat siz =  { 0 };
        stat(path, &siz);
        info.total_bytes = siz.st_size;
        ESP_LOGI(TAG, "File size is %d byte,pos:%d", (int)siz.st_size, (int)info.byte_pos);
        if (sdcard->file && (info.byte_pos > 0)) {
            if (fseek(sdcard->file, info.byte_pos, SEEK_SET) != 0) {
                ESP_LOGE(TAG, "Error seek file");
                return ESP_FAIL;
            }
        }
    } else if (sdcard->type == AUDIO_STREAM_WRITER) {
        sdcard->file = fopen(path, "w+");
        sdcard->w_type =  get_type(path);
        if (sdcard->file && STREAM_TYPE_WAV == sdcard->w_type) {
            wav_header_t info = {0};
            fwrite(&info, 1, sizeof(wav_header_t), sdcard->file);
            fsync(fileno(sdcard->file));
        } else if (sdcard->file && (STREAM_TYPE_AMR == sdcard->w_type)) {
            fwrite("#!AMR\n", 1, 6, sdcard->file);
            fsync(fileno(sdcard->file));
        } else if (sdcard->file && (STREAM_TYPE_AMRWB == sdcard->w_type)) {
            fwrite("#!AMR-WB\n", 1, 9, sdcard->file);
            fsync(fileno(sdcard->file));
        }
    } else {
        ESP_LOGE(TAG, "SDCARD must be Reader or Writer");
        return ESP_FAIL;
    }
    if (sdcard->file == NULL) {
        ESP_LOGE(TAG, "Failed to open file %s", path);
        return ESP_FAIL;
    }
    sdcard->is_open = true;
    if (info.byte_pos && fseek(sdcard->file, info.byte_pos, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "Failed to seek to %d/%d", (int)info.byte_pos, (int)info.total_bytes);
        return ESP_FAIL;
    }

    return audio_element_setinfo(self, &info);
}

static int _sdcard_read(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    sdcard_stream_t *sdcard = (sdcard_stream_t *)audio_element_getdata(self);
    audio_element_info_t info;

    audio_element_getinfo(self, &info);
    ESP_LOGD(TAG, "read len=%d, pos=%d/%d", len, (int)info.byte_pos, (int)info.total_bytes);

    int blks_in_queue = (sdcard->blk_head >= sdcard->blk_tail)? (sdcard->blk_head - sdcard->blk_tail):
                                                                (MAX_DATA_BLOCKS - (sdcard->blk_tail - sdcard->blk_head));
    while (blks_in_queue < (sdcard->buffered_size-1))
    {
        if ( (sdcard->blk_queue[sdcard->blk_head] = sdcard_take_blk(m_obj)) >= MAX_DATA_BLOCKS)
        {
            ESP_LOGW(TAG, "Ooops no more blocks on read !!");
            break;
        }

        evt_block_t * msg = &m_obj->blocks[sdcard->blk_queue[sdcard->blk_head]];
        msg->flags &= ~(BL_FLAG_WRITE | BL_FLAG_DONE);
        msg->len = 0;
        msg->fd = sdcard->file;

        if (sdcard_send_msg(m_obj, JOB_QUEUE_READ, msg) != ESP_OK)
        {
            sdcard_release_blk(m_obj, sdcard->blk_queue[sdcard->blk_head]);
            break;
        }

        sdcard->blk_head = (sdcard->blk_head + 1) % MAX_DATA_BLOCKS;
        blks_in_queue++;
    }

    if (sdcard->blk_tail == sdcard->blk_head)
    {
        ESP_LOGW(TAG, "No more data to readed !!");
        return 0;
    }

    evt_block_t * msg = &m_obj->blocks[sdcard->blk_queue[sdcard->blk_tail]];
    int ticks = 0;
    while ( !(msg->flags & BL_FLAG_DONE) )
    {
        vTaskDelay(2);
        if ( !(++ticks % 10) )
        {
            ESP_LOGW(TAG, " - Ooops we have blocked on read %d", ticks);
        }
    }

    int cpy_size = min(len, msg->len);
    if (msg->len <= 0)
    {
        ESP_LOGW(TAG, "No more data, ret:%d", cpy_size);
    }
    else
    {
        memcpy(buffer, msg->data, cpy_size);
        info.byte_pos += cpy_size;
        audio_element_setinfo(self, &info);
    }

    if (cpy_size < msg->len)
    {
        msg->len -= cpy_size;
        memmove(&msg->data[0], &msg->data[cpy_size], msg->len);
    }
    else
    {
        sdcard_release_blk(m_obj, sdcard->blk_queue[sdcard->blk_tail]);
        sdcard->blk_tail = (sdcard->blk_tail + 1) % MAX_DATA_BLOCKS;
    }

    return cpy_size;
    

    /* IGOR ------------------ *
   int64_t start = esp_timer_get_time( );

   int rlen = fread(buffer, 1, len, sdcard->file);
    if (rlen <= 0) {
        ESP_LOGW(TAG, "No more data,ret:%d", rlen);
    } else {
        info.byte_pos += rlen;
        audio_element_setinfo(self, &info);
    }
    int64_t end = esp_timer_get_time( );
    if ( (end - start) > 50000 )
    {
        ESP_LOGE(TAG, "! read took :%d usec", (int)(end - start));
    }

    return rlen;
    *----------------------- */
}

static int _sdcard_write(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    sdcard_stream_t *sdcard = (sdcard_stream_t *)audio_element_getdata(self);
    audio_element_info_t info;
    evt_block_t * msg;
    int  rlen = 0;
    audio_element_getinfo(self, &info);

    if (ticks_to_wait == 0)
        ticks_to_wait++;

    while (ticks_to_wait) 
    {
        if (sdcard->blk_tail != sdcard->blk_head)
        {
            msg = &m_obj->blocks[sdcard->blk_queue[sdcard->blk_tail]];
            if (msg->flags & BL_FLAG_DONE)
            {
                sdcard_release_blk(m_obj, sdcard->blk_queue[sdcard->blk_tail]);
                sdcard->blk_tail = (sdcard->blk_tail + 1) % MAX_DATA_BLOCKS;
                continue;
            }
        }

        int blks_in_queue = (sdcard->blk_head >= sdcard->blk_tail)? (sdcard->blk_head - sdcard->blk_tail):
                                                (MAX_DATA_BLOCKS - (sdcard->blk_tail - sdcard->blk_head));
        if ( (blks_in_queue >= (MAX_DATA_BLOCKS - 1)) )
        {
            vTaskDelay(MIN_WAIT_TICKS);
            ticks_to_wait -= min((MIN_WAIT_TICKS), ticks_to_wait);
            continue;
        }

        if ((sdcard->blk_queue[sdcard->blk_head] = sdcard_take_blk(m_obj)) >= MAX_DATA_BLOCKS)
        {
            vTaskDelay(MIN_WAIT_TICKS);
            ticks_to_wait -= min((MIN_WAIT_TICKS), ticks_to_wait);
            continue;
        }

        msg = &m_obj->blocks[sdcard->blk_queue[sdcard->blk_head]];
        msg->flags &= ~(BL_FLAG_DONE);
        msg->flags |= BL_FLAG_WRITE;
        msg->len = min(SDCARD_STREAM_BUF_SIZE, len);
        msg->fd = sdcard->file;
        memcpy(msg->data, buffer, msg->len);

        if (sdcard_send_msg(m_obj, JOB_QUEUE_WRITE, msg) != ESP_OK)
        {
            sdcard_release_blk(m_obj, sdcard->blk_queue[sdcard->blk_head]);
            break;
        }

        info.byte_pos += msg->len;
        audio_element_setinfo(self, &info);

        ESP_LOGD(TAG, "write,%d, pos:%d", msg->len, (int)info.byte_pos);

        sdcard->blk_head = (sdcard->blk_head + 1) % MAX_DATA_BLOCKS;
        rlen = msg->len;

        break;
    }

    return rlen;


    /* IGOR ------------------ *
    int64_t start = esp_timer_get_time( );

    int wlen =  fwrite(buffer, 1, len, sdcard->file);
    fsync(fileno(sdcard->file));
    
    int64_t end = esp_timer_get_time( );
    if ( (end - start) > 50000 )
    {
        ESP_LOGE(TAG, "!!! write took :%d usec", (int)(end - start));
    }
    ESP_LOGD(TAG, "write,%d, errno:%d,pos:%d", wlen, errno, (int)info.byte_pos);
    if (wlen > 0) {
        info.byte_pos += wlen;
        audio_element_setinfo(self, &info);
    }
    return wlen;
    *----------------------- */
}

static int _sdcard_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
    int r_size = audio_element_input(self, in_buffer, in_len);
    int w_size = 0;
    if (r_size > 0) {
        w_size = audio_element_output(self, in_buffer, r_size);
    } else {
        w_size = r_size;
    }
    return w_size;
}

static esp_err_t _sdcard_close(audio_element_handle_t self)
{
    sdcard_stream_t *sdcard = (sdcard_stream_t *)audio_element_getdata(self);

    /* wait to release all blocks */
    while (sdcard->blk_tail != sdcard->blk_head) 
    {
        evt_block_t * msg = &m_obj->blocks[sdcard->blk_queue[sdcard->blk_tail]];
        if (msg->flags & BL_FLAG_DONE)
        {
            sdcard_release_blk(m_obj, sdcard->blk_queue[sdcard->blk_tail]);
            sdcard->blk_tail = (sdcard->blk_tail + 1) % MAX_DATA_BLOCKS;
        }
        else
            vTaskDelay(MIN_WAIT_TICKS);
    }

    if (AUDIO_STREAM_WRITER == sdcard->type
        && sdcard->file
        && STREAM_TYPE_WAV == sdcard->w_type) 
    {
        wav_header_t *wav_info = (wav_header_t *) audio_malloc(sizeof(wav_header_t));

        AUDIO_MEM_CHECK(TAG, wav_info, return ESP_ERR_NO_MEM);

        if (fseek(sdcard->file, 0, SEEK_SET) != 0) 
        {
            ESP_LOGE(TAG, "Error seek file ,line=%d", __LINE__);
        }
        audio_element_info_t info;
        audio_element_getinfo(self, &info);
        wav_head_init(wav_info, info.sample_rates, info.bits, info.channels);
        wav_head_size(wav_info, (uint32_t)info.byte_pos);

        fwrite(wav_info, 1, sizeof(wav_header_t), sdcard->file);
        fsync(fileno(sdcard->file));


        audio_free(wav_info);
    }

    if (sdcard->is_open) 
    {
        fclose(sdcard->file);
        sdcard->is_open = false;
    }
    if (AEL_STATE_PAUSED != audio_element_get_state(self)) 
    {
        audio_element_report_info(self);
        audio_element_info_t info = {0};
        audio_element_getinfo(self, &info);
        info.byte_pos = 0;
        audio_element_setinfo(self, &info);
    }
    return ESP_OK;
}

static esp_err_t _sdcard_destroy(audio_element_handle_t self)
{
    sdcard_stream_t *sdcard = (sdcard_stream_t *)audio_element_getdata(self);
    audio_free(sdcard);

    if (m_obj && (--m_reference == 0))
        sdcard_clear_internals();

    return ESP_OK;
}

audio_element_handle_t sdcard_stream_init(sdcard_stream_cfg_t *config)
{
    audio_element_handle_t el;
    sdcard_stream_t *sdcard = audio_calloc(1, sizeof(sdcard_stream_t));
    AUDIO_MEM_CHECK(TAG, sdcard, return NULL);

    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.open = _sdcard_open;
    cfg.close = _sdcard_close;
    cfg.process = _sdcard_process;
    cfg.destroy = _sdcard_destroy;
    cfg.task_stack = config->task_stack;
    cfg.task_prio = config->task_prio;
    cfg.task_core = config->task_core;
    cfg.out_rb_size = config->out_rb_size;
    cfg.buffer_len = config->buf_sz;
    if (cfg.buffer_len == 0) 
    {
        cfg.buffer_len = SDCARD_STREAM_BUF_SIZE;
    }

    cfg.tag = "file";
    sdcard->type = config->type;
    sdcard->buffered_size = config->buffered_size;

    if (config->type == AUDIO_STREAM_WRITER) 
    {
        cfg.write = _sdcard_write;
    } 
    else 
    {
        cfg.read = _sdcard_read;
    }
    el = audio_element_init(&cfg);

    AUDIO_MEM_CHECK(TAG, el, goto _sdcard_init_exit);
    audio_element_setdata(el, sdcard);
    
    if (ESP_OK != sdcard_init_internals(MAX_DATA_BLOCKS, SDCARD_STREAM_TASK_STACK, SDCARD_STREAM_TASK_PRIO, SDCARD_STREAM_TASK_CORE))
        goto _sdcard_init_exit;

    m_reference++;

    return el;
_sdcard_init_exit:
    audio_free(sdcard);
    return NULL;
}

/********************************* UNI TEST SDCARD PERFORMANCE ********************************************/
#define TASK_LOADER_STACK_SIZE  (1024 * 4)
#define TASK_LOADER_PRI         (4)
#define TASK_LOADER_CORE        (0)
#define SD_TEST_DIR             "/sdcard/test"
#define SD_TEST_WR_FILE         SD_TEST_DIR"/twrite2.bin"
#define SD_TEST_RD_FILE         SD_TEST_DIR"/tread2.bin"
#define SD_RD_BLK_SIZE         (1024 * 16)
#define SD_WR_BLK_SIZE         (1024 * 16)
#define SD_POST_QUEUE_LEN      (16)
#define SD_TOTAL_FILE_SIZE     (1024 * 1024 * 100) /* 100 Mbytes of file */

typedef struct _evt_data
{
    uint8_t indx;
    bool    bReady;
}evt_data_t;

typedef struct _sd_test_streamer
{
    bool bWriterReader;
    const char *  file_path;
    FILE * file_handle;
    uint block_size;
    uint block_idx;
    QueueHandle_t queue;
    TaskHandle_t  task_handle;
}sd_test_streamer_t;


static sd_test_streamer_t m_stream_reader, m_stream_writer;
static bool               m_RunningCount = 0;

static void sdcard_loader_read(void * ctx)
{
    char * tmp_data = malloc(m_stream_reader.block_size);
    evt_data_t * evt_data;

    if (!tmp_data)
    {
        vTaskDelete(NULL);
        return;
    }

    m_RunningCount++;

    while (xQueueReceive(m_stream_reader.queue, (void *)&evt_data, (100/portTICK_PERIOD_MS)) == pdTRUE)
    {
        int len = 0;
        while (len < m_stream_reader.block_size)
        {
            int rd = fread(tmp_data, 1, (m_stream_reader.block_size-len), m_stream_reader.file_handle);
            if (rd <= 0)
            {
                ESP_LOGE(TAG, "IO-error: sdcard_loader_read %d", rd);
                break;
            }
            len += rd;
        }

        if ( len < m_stream_reader.block_size || 
            (tmp_data[0] != evt_data->indx || tmp_data[len-1] != evt_data->indx) )
        {
            if (len == m_stream_reader.block_size)
            {
                ESP_LOGE(TAG, "IO-error: verification read failed on %d index", evt_data->indx);
            }
            break;
        }

        evt_data->bReady = false;
    }

    m_RunningCount--;
    free(tmp_data);

    vTaskDelete(NULL);
}

#define MAX_RETRIES 6
static void sdcard_loader_write(void * ctx)
{
    char * tmp_data = (char *)malloc(m_stream_writer.block_size);
    evt_data_t * evt_data=NULL;

    if (!tmp_data)
    {
        vTaskDelete(NULL);
        return;
    }

    m_RunningCount++;

    while (xQueueReceive(m_stream_writer.queue, (void *)&evt_data, (100/portTICK_PERIOD_MS)) == pdTRUE)
    {
        int len = 0;

        ESP_LOGW(TAG, "before %d indx %d", m_stream_writer.block_size, evt_data->indx); vTaskDelay(200/portTICK_PERIOD_MS);

        memset(tmp_data, evt_data->indx, m_stream_writer.block_size);

        //ESP_LOGE(TAG, "after %d indx %d", m_stream_writer.block_size, evt_data->indx); vTaskDelay(50/portTICK_PERIOD_MS);

        while (len < m_stream_writer.block_size)
        {
            int wr = fwrite(&tmp_data[len], 1, (m_stream_writer.block_size-len), m_stream_writer.file_handle);
            taskYIELD();

            if (wr <= 0)
            {
                ESP_LOGE(TAG, "IO-error: sdcard_writer %d", wr);
                break;
            }

            //fsync(fileno(m_stream_writer.file_handle)); 
            len += wr;
        }

        if ( len < m_stream_writer.block_size )
            break;

        evt_data->bReady = false;
    }

    ESP_LOGE(TAG, "Exit %d ", m_stream_writer.block_size); vTaskDelay(50/portTICK_PERIOD_MS);

    m_RunningCount--;

    free(tmp_data);
    vTaskDelete(NULL);
}


static int simpleTest(bool bWriteOnly, const char * wrFile, const char * rdFile)
{
    struct stat st;
    if (stat(wrFile, &st) == 0) {
        unlink(wrFile);  // if old file exists, delete it
    }

    ESP_LOGI(TAG, "Opening %s for write", wrFile);
    FILE* f = fopen(wrFile, "w"), *fr;
    if (f==NULL){
        ESP_LOGE(TAG, "Failed to open file %s for writing", wrFile);
        return 1;
    }

    if (!bWriteOnly) {
        ESP_LOGI(TAG, "Opening %s for read", wrFile);
        fr = fopen(rdFile, "r");
        if (fr == NULL) {
            fclose(f);
            ESP_LOGE(TAG, "Failed to open file %s for reading", rdFile);
            return 1;
        }
    }

    int BUF_SIZE = (4 * 1024);
    char * buf= malloc(BUF_SIZE);
    if (buf == NULL) {
        ESP_LOGI(TAG, "No more memory for !!!");
        fclose(f);
        if (!bWriteOnly)
            fclose(fr);
        return 1;
    }
    memset(buf, 'x', BUF_SIZE) ;

    int64_t start = esp_timer_get_time( );

    // An fwrite(..) will consistently fail in this loop after random/various iterations
    int i, wrote, num_records = 3 * 60 * 60  ; // NEVER finishes this many
    for(i = 1; i <= num_records; i++) {
        wrote = fwrite(buf, 1, BUF_SIZE, f) ;
        fsync(fileno(f));
        if (wrote != BUF_SIZE) {
            ESP_LOGE(TAG, "Failed on fwrite(..) with: %d", wrote) ;
            break ;  // after 1st failure, all successive fwrite(..)'s also fail...
        }
        taskYIELD();
        if (!bWriteOnly) {
            wrote = fread(buf, 1, BUF_SIZE, fr) ;
            if (wrote != BUF_SIZE) {
                ESP_LOGE(TAG, "Failed on fread(..) with: %d", wrote) ;
                break ;  // after 1st failure, all successive fwrite(..)'s also fail...
            }
        }
        if (i % 100 == 0){
            ESP_LOGW(TAG, "written records: %d, bytes: %d", i, i * BUF_SIZE) ;
            vTaskDelay(10 / portTICK_PERIOD_MS);  // appease watchdog
        }
    }
    fclose(f);
    if (!bWriteOnly)
        fclose(fr);

    int64_t end = esp_timer_get_time( );

    ESP_LOGE(TAG, "Finish with %dms ", (int)((end - start)/1000)); vTaskDelay(50/portTICK_PERIOD_MS);

    free(buf) ;

    return 1;
}

void runSDTest()
{
    ESP_LOGW(TAG, "Start SD test"); vTaskDelay(50/portTICK_PERIOD_MS);

    if (simpleTest(true, "/sdcard/data2.txt", NULL))
        return;

    memset(&m_stream_reader, 0, sizeof(sd_test_streamer_t));
    memset(&m_stream_writer, 0, sizeof(sd_test_streamer_t));

    mkdir(SD_TEST_DIR, 0777);

    /* Prepare file for reading */
    m_stream_writer.bWriterReader = true;
    m_stream_writer.file_path = SD_TEST_RD_FILE;
    m_stream_writer.block_size = SD_RD_BLK_SIZE;

    if ((m_stream_writer.file_handle = fopen(m_stream_writer.file_path, "w+")) == NULL ||
        (m_stream_reader.queue = xQueueCreate(SD_POST_QUEUE_LEN, sizeof(void *))) == NULL ||
        (m_stream_writer.queue = xQueueCreate(SD_POST_QUEUE_LEN, sizeof(void *))) == NULL )
    {
        ESP_LOGE(TAG, "Cannot initialize reader or writer queue");
        if (m_stream_writer.file_handle)
            fclose(m_stream_writer.file_handle);
        if (m_stream_reader.queue)
            vQueueDelete(m_stream_reader.queue);
        return;
    }
    fseek(m_stream_writer.file_handle, 0 , SEEK_SET);

    ESP_LOGE(TAG, "Step1 test"); vTaskDelay(50/portTICK_PERIOD_MS);

    if (xTaskCreatePinnedToCore(sdcard_loader_write, "sd_wrstream", TASK_LOADER_STACK_SIZE, (void *)&m_stream_writer,
                                                                    TASK_LOADER_PRI, &m_stream_writer.task_handle, TASK_LOADER_CORE) != pdPASS) 
    {
        ESP_LOGE(TAG, "-Prepare: Error create sdcard_loader_write task");
        return;
    }

    ESP_LOGE(TAG, "Step2 test"); vTaskDelay(50/portTICK_PERIOD_MS);

    evt_data_t evt_data[SD_POST_QUEUE_LEN];
    uint max_delay = 0, dly; 

    int64_t start = esp_timer_get_time( );
    
    for (uint i=0; (m_RunningCount > 0) && i<(SD_TOTAL_FILE_SIZE/SD_RD_BLK_SIZE); i++)
    {
        uint evt_idx = (i%SD_POST_QUEUE_LEN);
        evt_data_t * p_data = &evt_data[evt_idx];

        p_data->indx = (i&0xFF);
        p_data->bReady = true;

        dly = 0;
        while (m_RunningCount > 0 && uxQueueSpacesAvailable(m_stream_writer.queue) <= 0)
        {
            vTaskDelay(10/portTICK_PERIOD_MS);
            dly += 10;
        }

        if (max_delay < dly)
            max_delay = dly;

        xQueueSend(m_stream_writer.queue, (void *)&p_data, 0);
    }

    int64_t end = esp_timer_get_time( );


    ESP_LOGE(TAG, "Finish main %d ", m_stream_writer.block_size); vTaskDelay(50/portTICK_PERIOD_MS);


    //Wait until done
    do { vTaskDelay(portTICK_PERIOD_MS); } while(m_RunningCount > 0);

    fclose(m_stream_writer.file_handle);

    ESP_LOGW(TAG, "!! Write only took %d(ms) avg speed = %d(kbps) max delay = %dms !!", 
            (int)((end - start)/1000), (int)((SD_TOTAL_FILE_SIZE*8)/((end - start)/1000)), max_delay);

    
    /* Start testing read and write together */
    m_stream_writer.bWriterReader = true;
    m_stream_writer.file_path = SD_TEST_WR_FILE;
    m_stream_writer.block_size = SD_WR_BLK_SIZE;

    m_stream_reader.bWriterReader = false;
    m_stream_reader.file_path = SD_TEST_RD_FILE;
    m_stream_reader.block_size = SD_RD_BLK_SIZE;

    if ((m_stream_writer.file_handle = fopen(m_stream_writer.file_path, "w")) == NULL ||
        (m_stream_reader.file_handle = fopen(m_stream_reader.file_path, "r")) == NULL)
    {
        ESP_LOGE(TAG, "-Cannot initialize reader or writer queue");
        if (m_stream_writer.file_handle)
            fclose(m_stream_writer.file_handle);
        return;
    }

    fseek(m_stream_writer.file_handle, 0 , SEEK_SET);
    fseek(m_stream_reader.file_handle, 0 , SEEK_SET);

    if (xTaskCreatePinnedToCore(sdcard_loader_write, "sd_wrstream", TASK_LOADER_STACK_SIZE, (void *)&m_stream_writer,
                                                                    TASK_LOADER_PRI, &m_stream_writer.task_handle, TASK_LOADER_CORE) != pdPASS || 
        xTaskCreatePinnedToCore(sdcard_loader_read, "sd_rdstream",  TASK_LOADER_STACK_SIZE, (void *)&m_stream_reader,
                                                                    TASK_LOADER_PRI, &m_stream_reader.task_handle, TASK_LOADER_CORE) != pdPASS) 
    {
        ESP_LOGE(TAG, "-Prepare: Error create reader/writer task");
        return;
    }


    evt_data_t evt_data_rd[SD_POST_QUEUE_LEN];
    evt_data_t evt_data_wr[SD_POST_QUEUE_LEN];
    max_delay = 0; 

    start = esp_timer_get_time( );
    
    for (uint i=0; (m_RunningCount > 1) && i<(SD_TOTAL_FILE_SIZE/SD_RD_BLK_SIZE); i++)
    {
        uint evt_idx = (i%SD_POST_QUEUE_LEN);
        evt_data_t * p_data_rd = &evt_data_rd[evt_idx];
        evt_data_t * p_data_wr = &evt_data_wr[evt_idx];

        p_data_rd->indx = (i&0xFF);
        p_data_rd->bReady = true;
        p_data_wr->indx = (i&0xFF);
        p_data_wr->bReady = true;

        dly = 0;
        while (m_RunningCount > 1 && uxQueueSpacesAvailable(m_stream_reader.queue) <= 0)
        {
            dly+=10;
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        xQueueSend(m_stream_reader.queue, (void *)&p_data_rd, 0);

        while (m_RunningCount > 1 && uxQueueSpacesAvailable(m_stream_writer.queue) <= 0)
        {
            dly+=10;
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        xQueueSend(m_stream_writer.queue, (void *)&p_data_wr, 0);

        if (max_delay < dly)
            max_delay = dly;
    }

    end = esp_timer_get_time( );

    //Wait until done
    do { vTaskDelay(portTICK_PERIOD_MS); } while(m_RunningCount > 0);

    fclose(m_stream_reader.file_handle);
    fclose(m_stream_writer.file_handle);

    ESP_LOGW(TAG, "!! Write/Read took %d(ms) avg speed = %d(kbps) max delay = %dms !!", 
            (int)((end - start)/1000), (int)((SD_TOTAL_FILE_SIZE*8)/((end - start)/1000)), max_delay);

    vQueueDelete(m_stream_reader.queue);
    vQueueDelete(m_stream_writer.queue);

    /*
    remove(m_stream_reader.file_path);
    remove(m_stream_writer.file_path);
    */
}


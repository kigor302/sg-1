/* Play mp3 songs from SD card and record to SD card as wav file.
*/

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "tcpip_adapter.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
//#include "sdcard_stream.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "wav_decoder.h"
#include "wav_encoder.h"
#include "filter_resample.h"
#include "http_stream.h"
#include "raw_stream.h"
#include "esp_sr_iface.h"
#include "esp_sr_models.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_touch.h"
#include "periph_button.h"
#include "periph_wifi.h"
#include "input_key_service.h"
#include "periph_adc_button.h"
#include "board.h"
#include "sg1.h"
#include "sg_ctrl.h"


static const char *TAG = "SG-1";

//#define USE_WIFI

/* Local variabbles */
static audio_board_handle_t board_handle;
static audio_pipeline_handle_t pipeline_for_record, pipeline_for_play;
static audio_element_handle_t  fatfs_stream_reader, i2s_stream_writer, /*mp3_decoder,*/ wav_decoder, resample_for_play;
static audio_element_handle_t  fatfs_stream_writer, i2s_stream_reader, wav_encoder, /*raw_reader,*/ resample_for_rec;
static song_t m_songs[MAX_SONGS] = {0};
static player_state_t m_state = { .song=&m_songs[0], .play_selected_tracks=0x0, .rec_selected_track=1,
                                  .cursor=0, .play_state=P_STOPED, .rec_state=R_STOPED, 
                                  .rec_opt={SRC_LINEIN, true, 1},
                                  .equalizer={{0,0,0,0,0,0}},
                                  .volume={60, 50, true, 0},
                                  .display=D_SONG };


static void button_ctrl_proc(CTRL_BUTTON_E bt, EVT_BUTTON_E evt)
{
    if (evt == EVT_RELEASED)
        return;

    switch (bt)
    {
        case BT_LEFT:
            if (m_state.display == D_SONG)
                do {
                    m_state.song->cursor = (m_state.song->cursor==0)? (MAX_TRACKS*2-1): (m_state.song->cursor-1);
                } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
            else if (m_state.display == D_VOLUME)
                m_state.volume.play_vol_selected = !m_state.volume.play_vol_selected;
            else if (m_state.display == D_EQUALIZER)
                m_state.equalizer.cursor = (m_state.equalizer.cursor==0)? (MAX_EQ_BANDS-1): (m_state.equalizer.cursor-1);
            else if (m_state.display == D_REC_OPT)
                m_state.rec_opt.cursor = !m_state.rec_opt.cursor;
            break; 
        case BT_VOLUME_SW:
        case BT_RIGTH:
            if (m_state.display == D_SONG)
                do {
                    m_state.song->cursor = (m_state.song->cursor + 1) % (MAX_TRACKS*2);    
                } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
                
            else if (m_state.display == D_VOLUME)
                m_state.volume.play_vol_selected = !m_state.volume.play_vol_selected;
            else if (m_state.display == D_EQUALIZER)
                m_state.equalizer.cursor = (m_state.equalizer.cursor+1) % MAX_EQ_BANDS;
            else if (m_state.display == D_REC_OPT)
                m_state.rec_opt.cursor = !m_state.rec_opt.cursor;
            break; 
        case BT_UP:
            m_state.display = (DISPLAY_E)((m_state.display == D_SONG)? (D_MAX_OPTIONS-1): (m_state.display-1));  
            break; 
        case BT_DOWN:
            m_state.display = (DISPLAY_E)((m_state.display + 1) % D_MAX_OPTIONS);
            break; 
        
        case BT_SET:
            if (m_state.display == D_SONG)
            {
                if (m_state.song->cursor >= MAX_TRACKS) 
                    m_state.rec_selected_track = (m_state.rec_selected_track != (m_state.song->cursor-MAX_TRACKS+1))? 
                                                  (m_state.song->cursor-MAX_TRACKS+1): 0;
                else 
                    m_state.play_selected_tracks = (m_state.play_selected_tracks & (1<<m_state.song->cursor))? 
                                           (m_state.play_selected_tracks & ~(1<<m_state.song->cursor)):
                                           (m_state.play_selected_tracks | (1<<m_state.song->cursor));
            }
            else if (m_state.display == D_VOLUME)
            {
                if (m_state.volume.play_vol_selected)
                {
                    m_state.volume.play_volume = ((m_state.volume.play_volume+5) % 100);
                    audio_hal_set_volume(board_handle->audio_hal, ((m_state.volume.play_volume*63)/100) );
                }
                else
                {
                    m_state.volume.rec_volume = ((m_state.volume.rec_volume+5) % 100);
                    audio_hal_set_recvolume(board_handle->audio_hal, m_state.volume.rec_volume);
                }
            }
            else if (m_state.display == D_EQUALIZER)
            {
                m_state.equalizer.bands[m_state.equalizer.cursor] = 
                                           ((m_state.equalizer.bands[m_state.equalizer.cursor]+1+20) % 40) - 20;
                //TODO:  Set equalizer in range -20db..20db
            }
            else if (m_state.display == D_REC_OPT)
            {
                if (m_state.rec_opt.cursor==0)
                {
                    m_state.rec_opt.rec_source = (REC_SOURCE_E)!m_state.rec_opt.rec_source;
                    audio_hal_ctrl_codec(board_handle->audio_hal, 
                        ((m_state.rec_opt.rec_source == SRC_MIC)? AUDIO_HAL_CODEC_MODE_DECODE:
                                                                  AUDIO_HAL_CODEC_MODE_LINE_IN),
                        AUDIO_HAL_CTRL_START);
                }
                else
                {
                    m_state.rec_opt.bmonitor = !m_state.rec_opt.bmonitor;
                    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_PASSTHROUGH, 
                                        (m_state.rec_opt.bmonitor? AUDIO_HAL_CTRL_START: AUDIO_HAL_CTRL_STOP));
                }
            }
            break; 

        case BT_PLAY:
            {
            audio_pipeline_handle_t pipeline = pipeline_for_play;
            int track_num = (m_state.play_selected_tracks & 0x1)? 1:
                            (m_state.play_selected_tracks & 0x2)? 2:
                            (m_state.play_selected_tracks & 0x4)? 3:
                            (m_state.play_selected_tracks & 0x8)? 4:
                            (m_state.play_selected_tracks & 0x10)? 5:
                            0;
            audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline, "file");
            audio_element_handle_t i2s_el = audio_pipeline_get_el_by_tag(pipeline, "i2s");
            audio_element_state_t el_state = audio_element_get_state(i2s_el);

            switch (el_state) {
                case AEL_STATE_RUNNING :
                    ESP_LOGI(TAG, "[ * ] Pausing play audio pipeline");
                    audio_pipeline_pause(pipeline);
                    m_state.play_state = P_PAUSE;
                    set_led(OUT_LED_GREEN, false);
                    break;
                case AEL_STATE_PAUSED :
                    ESP_LOGI(TAG, "[ * ] Resuming play audio pipeline");
                    audio_pipeline_resume(pipeline);
                    m_state.play_state = P_PLAYING;
                    set_led(OUT_LED_GREEN, true);
                    break;
                case AEL_STATE_INIT :
                    ESP_LOGI(TAG, "[ * ] Starting play audio pipeline");
                case AEL_STATE_STOPPED:
                case AEL_STATE_FINISHED:
                    if ( fatfs_el && track_num )
                    {
                        char track_name[32];
                        sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num, track_num);
                        audio_element_set_uri(fatfs_el, track_name);
                        ESP_LOGI(TAG, "[ * ] Starting to play track: %s", track_name);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_run(pipeline);
                        m_state.play_state = P_PLAYING;
                        set_led(OUT_LED_GREEN, true);
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        case BT_REC:
            {
            audio_pipeline_handle_t pipeline = pipeline_for_record;
            int track_num = m_state.rec_selected_track;
            audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline, "file");
            audio_element_handle_t i2s_el = audio_pipeline_get_el_by_tag(pipeline, "i2s");
            audio_element_state_t el_state = audio_element_get_state(i2s_el);
            switch (el_state) {
                case AEL_STATE_RUNNING :
                    ESP_LOGI(TAG, "[ * ] Mute rec audio pipeline");
                    audio_hal_set_mute(board_handle->audio_hal, true);
                    m_state.rec_state = R_MUTE;
                    set_led(OUT_LED_RED, false);
                    break;
                case AEL_STATE_PAUSED :
                    ESP_LOGI(TAG, "[ * ] Unmute rec audio pipeline");
                    audio_hal_set_mute(board_handle->audio_hal, false);
                    m_state.rec_state = R_RECORDING;
                    set_led(OUT_LED_RED, true);
                    break;
                case AEL_STATE_INIT :
                    ESP_LOGI(TAG, "[ * ] Starting rec audio pipeline");
                case AEL_STATE_STOPPED:
                case AEL_STATE_FINISHED:
                    if ( fatfs_el && track_num)
                    {
                        char track_name[32];
                        sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num, track_num);
                        audio_element_set_uri(fatfs_el, track_name);
                        ESP_LOGI(TAG, "[ * ] Starting to record track: %s", track_name);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_run(pipeline);
                        m_state.rec_state = R_RECORDING;
                        set_led(OUT_LED_RED, true);
                        m_state.recording_track = track_num;
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        case BT_STOP:
            if (AEL_STATE_FINISHED != audio_element_get_state(i2s_stream_writer/*i2s_stream_reader*/))
            {
                audio_pipeline_stop(pipeline_for_play);
                audio_pipeline_wait_for_stop(pipeline_for_play);
            }

            if (AEL_STATE_RUNNING == audio_element_get_state(i2s_stream_reader))
            {
                audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline_for_record, "file");
                audio_element_info_t info;
                
                if ((m_state.rec_state == R_MUTE || m_state.rec_state == R_RECORDING) && audio_element_getinfo(fatfs_el, &info) == ESP_OK)
                    m_state.song->tracks[m_state.recording_track].len_in_sec = (info.byte_pos / (2 * 48000));

                audio_pipeline_stop(pipeline_for_record);
                audio_pipeline_wait_for_stop(pipeline_for_record);

            }
            audio_pipeline_reset_ringbuffer(pipeline_for_play);
            audio_pipeline_reset_ringbuffer(pipeline_for_record);
            m_state.play_state = P_STOPED;
            m_state.rec_state = R_STOPED;
            set_led(OUT_LED_GREEN, false);
            set_led(OUT_LED_RED, false);
            break;

        case BT_FORWARD:
            if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
            {
                int next = (m_state.song->num+1) % MAX_SONGS;
                m_state.song = &m_songs[next];
                m_state.song->num = next;
            }
            /* Fall down */
        case BT_REWARD:
            if (bt == BT_REWARD)
            {
                if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                    m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
                {
                    int prev = (m_state.song->num==0)? (MAX_SONGS-1): (m_state.song->num-1);
                    m_state.song = &m_songs[prev];
                    m_state.song->num = prev;
                }
            }

            {
                char song_dir[] = { '/','s','d','c','a','r','d','/','s','o','n','g','_',('0'+m_state.song->num),'\0' };
                mkdir(song_dir, 0755);
                for (int i=0; i<MAX_TRACKS; i++)
                {
                    struct stat st;
                    char filename[32];
                    sprintf(filename, "%s/track_%d.wav",song_dir, i);
                    m_state.song->tracks[i].len_in_sec = (stat(filename, &st) == ESP_OK)?
                                                         (st.st_size / (2 * 48000)): 0;
                }

                m_state.display = D_SONG;
            }
            break; 

        case EN_VOLUME_UP:
        case EN_VOLUME_DOWN:
            if (m_state.display == D_SONG || m_state.display == D_REC_OPT)
                m_state.display = D_VOLUME;

            if (m_state.display == D_VOLUME)
            {
                if (m_state.volume.play_vol_selected)
                {
                    m_state.volume.play_volume = (m_state.volume.play_volume+((bt==EN_VOLUME_UP)?5:-5)) % 100;
                    audio_hal_set_volume(board_handle->audio_hal, ((m_state.volume.play_volume*63)/100) );
                }
                else
                {
                    m_state.volume.rec_volume = (m_state.volume.rec_volume+((bt==EN_VOLUME_UP)?5:-5)) % 100;
                    audio_hal_set_recvolume(board_handle->audio_hal, m_state.volume.rec_volume);
                }
            }
            else if (m_state.display == D_EQUALIZER)
            {
                m_state.equalizer.bands[m_state.equalizer.cursor] = 
                                           ((m_state.equalizer.bands[m_state.equalizer.cursor]+((bt==EN_VOLUME_UP)?2:-2)+20) % 40) - 20;
                //TODO:  Set equalizer in range -20db..20db
            }
            break; 
        default:
            ESP_LOGW(TAG, "[ * ] %d Unknown control", bt);
            break;
    }

    display_player_state(&m_state);
}


//#define MAX_TRACKS  (3)
//static int  play_track = 1, record_track = 0;

/* Events proccessing */
//typedef enum { SG_MODE_VOL=0, SG_MODE_PLAY, SG_MODE_REC, SG_MODE_MONITOR, SG_MAX_MODE }sg_mode_t; 
//const char * set_modes[] = {  "VOLUME", "PLAY TRACK", "RECORD TRACK", "MONITOR Enable/Disable", "N/A" };
//static sg_mode_t current_mode = SG_MODE_VOL;

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
#if 0    
    /* Handle touch pad events
           to start, pause, resume, finish current song and adjust volume
    */
    audio_board_handle_t board_handle = (audio_board_handle_t) ctx;
    int  player_volume = 50;

    /* Handle debouncing and push release effect */
    static uint last_tick = 0,  last_button = 0;
    if ((last_button == (int)evt->data) && (xTaskGetTickCount() - last_tick) < pdMS_TO_TICKS(1000))// == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) 
    {
        last_button = 0; 
        last_tick = 0;
        return ESP_OK;
    }
    last_tick = xTaskGetTickCount();
    last_button = (int)evt->data;
    
    audio_hal_get_volume(board_handle->audio_hal, &player_volume);

    switch ((int)evt->data) {
        
        case BUTTON_REC_ID:
        //case BUTTON_PLAY_ID:
            {
            bool bPlay = ((int)evt->data != BUTTON_REC_ID);
            audio_pipeline_handle_t pipeline = bPlay? pipeline_for_play: pipeline_for_record;
            int  * track_num =  bPlay? &play_track:  &record_track;
            audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline, "file");
            audio_element_handle_t i2s_el = audio_pipeline_get_el_by_tag(pipeline, "i2s");
            
            ESP_LOGI(TAG, "[ * ] [%s] input key event",  (bPlay?"Play":"Record"));
            audio_element_state_t el_state = audio_element_get_state(i2s_el);
            switch (el_state) {
                case AEL_STATE_RUNNING :
                    ESP_LOGI(TAG, "[ * ] Pausing audio pipeline");
                    audio_pipeline_pause(pipeline);
                    break;
                case AEL_STATE_PAUSED :
                    ESP_LOGI(TAG, "[ * ] Resuming audio pipeline");
                    audio_pipeline_resume(pipeline);
                    break;
                case AEL_STATE_INIT :
                    ESP_LOGI(TAG, "[ * ] Starting audio pipeline");
                case AEL_STATE_STOPPED:
                case AEL_STATE_FINISHED:
                    if ( fatfs_el )
                    {
                        char new_track[] = { '/','s','d','c','a','r','d','/','t','r','a','c','k','_','0'+*track_num,'.','w','a','v','\0' };
                        audio_element_set_uri(fatfs_el, new_track);
                        ESP_LOGI(TAG, "[ * ] Starting %s new track: %s", (bPlay?"Play":"Record"), new_track);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_run(pipeline);
                    }
                    break;

                default :
                    ESP_LOGW(TAG, "[ * ] Not supported state %d for element %p (%p)", el_state, i2s_el, fatfs_el);
            }
            }
            break;

        case BUTTON_SET_ID:
            ESP_LOGI(TAG, "[ * ] [Set] input key event (stop recording and playing)");
            if (AEL_STATE_FINISHED != audio_element_get_state(i2s_stream_reader))
            {
                audio_pipeline_stop(pipeline_for_play);
                audio_pipeline_wait_for_stop(pipeline_for_play);
            }

            if (AEL_STATE_RUNNING == audio_element_get_state(i2s_stream_reader))
            {
                audio_pipeline_stop(pipeline_for_record);
                audio_pipeline_wait_for_stop(pipeline_for_record);
            }
            audio_pipeline_reset_ringbuffer(pipeline_for_play);
            audio_pipeline_reset_ringbuffer(pipeline_for_record);
            break;
/*
        case BUTTON_VOLUP_ID:
            if (current_mode == SG_MODE_VOL)
            {
                player_volume += 10;
                if (player_volume > 63) {
                    player_volume = 63;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            }
            else if (current_mode == SG_MODE_PLAY)
            {
                play_track = ((play_track + 1) % MAX_TRACKS);
                ESP_LOGI(TAG, "[ * ] Play track changed to: %d", play_track);
            }
            else if (current_mode == SG_MODE_REC)
            {
                record_track = ((record_track + 1) % MAX_TRACKS);
                ESP_LOGI(TAG, "[ * ] Record track changed to: %d", record_track);
            }
            else if (current_mode == SG_MODE_MONITOR)
            {
                audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_PASSTHROUGH, AUDIO_HAL_CTRL_START);    
                ESP_LOGI(TAG, "[ * ] Monitor is enabled");
            }
            break;

        case BUTTON_VOLDOWN_ID:
            if (current_mode == SG_MODE_VOL)
            {
                player_volume -= 10;
                if (player_volume < 0) {
                    player_volume = 0;
                }
                audio_hal_set_volume(board_handle->audio_hal, player_volume);
                ESP_LOGI(TAG, "[ * ] Volume set to %d %%", player_volume);
            }
            else if (current_mode == SG_MODE_PLAY)
            {
                record_track = (!play_track?MAX_TRACKS:play_track) - 1;
                ESP_LOGI(TAG, "[ * ] Play track changed to: %d", play_track);
            }
            else if (current_mode == SG_MODE_REC)
            {
                record_track = (!record_track?MAX_TRACKS:record_track) - 1;
                ESP_LOGI(TAG, "[ * ] Record track changed to: %d", record_track);
            }
            else if (current_mode == SG_MODE_MONITOR)
            {
                audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_PASSTHROUGH, AUDIO_HAL_CTRL_STOP);    
                ESP_LOGI(TAG, "[ * ] Monitor is disabled");
            }
            break;
*/
        case BUTTON_MODE_ID:
            current_mode = (sg_mode_t)((current_mode+1) % SG_MAX_MODE);
            ESP_LOGI(TAG, "[ * ] %s select mode",set_modes[current_mode]);
            break;
    }
#endif
    return ESP_OK;
}

void i2s_fatfs_event(audio_event_iface_msg_t * msg)
{
    //ESP_LOGI(TAG, "[ * ] fatfs event=%d data=%d", msg->cmd, (int)msg->data);
}

void i2s_stream_event(audio_event_iface_msg_t * msg)
{
    switch (msg->cmd) {
        case AEL_MSG_CMD_REPORT_STATUS:
            if ((int)msg->data == AEL_STATUS_STATE_STOPPED || (int)msg->data == AEL_STATUS_STATE_FINISHED)
            {
                audio_pipeline_handle_t pipeline = (msg->source == (void *)i2s_stream_writer)? pipeline_for_play: pipeline_for_record;
                ESP_LOGW(TAG, "[ * ] Stop event received from i2s stream %s",((msg->source == (void *)i2s_stream_writer)? "Play": "Record"));

                audio_pipeline_stop(pipeline);
                audio_pipeline_wait_for_stop(pipeline);
                audio_pipeline_reset_ringbuffer(pipeline);

                if (pipeline==pipeline_for_play)
                {
                    set_led(OUT_LED_GREEN, false);
                    m_state.play_state = P_STOPED;
                }
                else
                {
                    set_led(OUT_LED_RED, false);
                    m_state.rec_state = P_STOPED;
                }
                display_player_state(&m_state);
            }
            break;
        default:
            ESP_LOGI(TAG, "[ * ] wave i2s stream event=%d data=%d", msg->cmd, (int)msg->data);
            break;
    }
}


static void wav_codec_event(audio_event_iface_msg_t * msg)
{
    bool bDecoder = (msg->source == wav_decoder);
    switch (msg->cmd)
    {
        case AEL_MSG_CMD_REPORT_MUSIC_INFO:
            {
                audio_element_info_t music_info = {0};
                audio_element_getinfo((bDecoder? wav_decoder: wav_encoder), &music_info);

                ESP_LOGI(TAG, "[ * ] Receive music info from wav %s, sample_rates=%d, bits=%d, ch=%d",
                                            (bDecoder?"decoder":"encoder"), music_info.sample_rates, music_info.bits, music_info.channels);

                //audio_element_setinfo(i2s_stream_writer, &music_info);
                //audio_element_setinfo(i2s_stream_reader, &music_info);
                rsp_filter_set_src_info((bDecoder?resample_for_play:resample_for_rec), music_info.sample_rates, music_info.channels);
            }
            break;
        
        default:
            ESP_LOGI(TAG, "[ * ] wave %s event=%d data=%d", (bDecoder?"decoder":"encoder"), msg->cmd, (int)msg->data);
            break;
    }
}


static audio_pipeline_handle_t setup_record_pipeline(const char * url, int sample_rates, int channels)
{
    ESP_LOGI(TAG, "- Create audio pipeline for record");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "- Save record data as %dHz, %dChannels wav file named [%s]", sample_rates, channels, url);
    /*
    sdcard_stream_cfg_t sdcard_write_cfg = SDCARD_STREAM_CFG_DEFAULT();
    //fatfs_write_cfg.out_rb_size = (32 * 1024);
    sdcard_write_cfg.type = AUDIO_STREAM_WRITER;
    fatfs_stream_writer = sdcard_stream_init(&sdcard_write_cfg);
    */
    fatfs_stream_cfg_t fatfs_write_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_write_cfg.out_rb_size = (160 * 1024);
    fatfs_write_cfg.type = AUDIO_STREAM_WRITER;
    fatfs_stream_writer = fatfs_stream_init(&fatfs_write_cfg);
    audio_element_set_uri(fatfs_stream_writer, url);

    i2s_stream_cfg_t i2s_file_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_file_cfg.type = AUDIO_STREAM_READER;
    i2s_file_cfg.i2s_config.sample_rate = 48000;
    i2s_file_cfg.task_prio = 4;
    i2s_stream_reader = i2s_stream_init(&i2s_file_cfg);

    wav_encoder_cfg_t wav_file_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    wav_encoder = wav_encoder_init(&wav_file_cfg);

    rsp_filter_cfg_t rsp_filter_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_filter_cfg.src_rate = 48000;
    rsp_filter_cfg.src_ch = 2;
    rsp_filter_cfg.dest_rate = sample_rates;
    rsp_filter_cfg.dest_ch = channels;
    rsp_filter_cfg.mode = RESAMPLE_DECODE_MODE;
    resample_for_rec = rsp_filter_init(&rsp_filter_cfg);

    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s");
    audio_pipeline_register(pipeline, wav_encoder, "wav");
    audio_pipeline_register(pipeline, resample_for_rec, "filter");
    audio_pipeline_register(pipeline, fatfs_stream_writer, "file");

    audio_pipeline_link(pipeline, (const char *[]) {"i2s", /*"filter",*/ "wav", "file"}, 3/*4*/);
    
    return pipeline;
}

 static struct audio_pipeline * setup_play_pipeline(const char * url)
{
    ESP_LOGI(TAG, "- Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "- Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.i2s_config.sample_rate = 48000;
    i2s_cfg.task_prio = 4;
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "- Create wav decoder to decode wav file");
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    wav_decoder = wav_decoder_init(&wav_cfg);


    ESP_LOGI(TAG, "- Play [%s] from sdcard", url);
    /*
    sdcard_stream_cfg_t sdcard_read_cfg = SDCARD_STREAM_CFG_DEFAULT();
    sdcard_read_cfg.type = AUDIO_STREAM_READER;
    //fatfs_read_cfg.out_rb_size = (100 * 1024);
    fatfs_stream_reader = sdcard_stream_init(&sdcard_read_cfg);
    */
    fatfs_stream_cfg_t fatfs_read_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_read_cfg.type = AUDIO_STREAM_READER;
    fatfs_read_cfg.out_rb_size = (160 * 1024);
    fatfs_stream_reader = fatfs_stream_init(&fatfs_read_cfg);

    audio_element_set_uri(fatfs_stream_reader, url);


    /* ZL38063 audio chip on board of ESP32-LyraTD-MSC does not support 44.1 kHz sampling frequency,
       so resample filter has been added to convert audio data to other rates accepted by the chip.
       You can resample the data to 16 kHz or 48 kHz.
    */
    ESP_LOGI(TAG, "- Create resample filter");
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.dest_rate = rsp_cfg.src_rate = 48000;
    resample_for_play = rsp_filter_init(&rsp_cfg);

    ESP_LOGI(TAG, "- Create a ringbuffer and insert it between mp3 decoder and i2s writer");
    //ringbuf_handle_t ringbuf = rb_create(16 * 1024, 1);
    //audio_element_set_input_ringbuf(i2s_stream_writer, ringbuf);
    //audio_element_set_output_ringbuf(wav_decoder, ringbuf);

    ESP_LOGI(TAG, "- Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, fatfs_stream_reader, "file");
    audio_pipeline_register(pipeline, wav_decoder, "wav");
    audio_pipeline_register(pipeline, resample_for_play, "filter");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "- Link it together [file]-->wav_decoder-->i2s_stream-->[codec_chip]");
    audio_pipeline_link(pipeline, (const char *[]) {"file", "wav", /*"filter",*/ "i2s"}, 3/*4*/);

    return pipeline;
}


void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
#ifdef USE_WIFI    
    tcpip_adapter_init();
#endif

    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_LOGI(TAG, "[ 1 ] Initialize the peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[ 1.1 ] Initialize sd card");
    audio_board_sdcard_init(set);
    
    ESP_LOGI(TAG, "[ 1.2 ] Initialize keys");
    audio_board_key_init(set);

#ifdef USE_WIFI    
    ESP_LOGI(TAG, "[ 1.3 ] Initialize wifi");
    periph_wifi_cfg_t wifi_cfg = {
        .ssid = CONFIG_WIFI_SSID,
        .password = CONFIG_WIFI_PASSWORD,
    };
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);
#endif

    board_handle = audio_board_init();
#ifdef CONFIG_INPUT_MIC
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    ESP_LOGI(TAG, "[ 2 ] Start codec chip (MIC)");
#else
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
    ESP_LOGI(TAG, "[ 2 ] Start codec chip (LINE-IN)");
#endif
    audio_hal_set_volume(board_handle->audio_hal, (m_state.volume.play_volume*63)/100);
    audio_hal_set_recvolume(board_handle->audio_hal, m_state.volume.rec_volume);
        
    ESP_LOGI(TAG, "[ 3 ] Create and start input key service");
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    periph_service_handle_t input_ser = input_key_service_create(set);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, (void *)board_handle);

    pipeline_for_play = setup_play_pipeline("/sdcard/track_0.wav");
    pipeline_for_record = setup_record_pipeline("/sdcard/track_0.wav", 48000 /*sampling rate HZ*/, 2/*channels*/);

    ESP_LOGI(TAG, "[ 4 ] Set up event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from pipelines");
    audio_pipeline_set_listener(pipeline_for_play, evt);
    audio_pipeline_set_listener(pipeline_for_record, evt);

    ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[4.3] Initialize control board");
    if (ESP_OK == init_ctrl_board())
        registrate_cb(button_ctrl_proc);

    button_ctrl_proc(BT_FORWARD, EVT_PRESSED);

    ESP_LOGW(TAG, "[ 5 ] Press the keys to control music player:");
    ESP_LOGW(TAG, "      [Play/Record] to start/Record, pause/mute and resume, [<< / >>] Stop and go to next song.");
    ESP_LOGW(TAG, "      [Vol-] or [Vol+] to adjust volume and equalizer");

    while (1) {
        audio_event_iface_msg_t msg;
        
        //wait for event (from pheriferial or pipelines)
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && ((msg.source == (void *) wav_encoder) || 
                                                              (msg.source == (void *) wav_decoder)))
            wav_codec_event(&msg);
        else if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && ((msg.source == (void *) fatfs_stream_reader) || 
                                                                   (msg.source == (void *) fatfs_stream_writer)))
            i2s_fatfs_event(&msg);
        else if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && ((msg.source == (void *) i2s_stream_writer) || 
                                                                   (msg.source == (void *) i2s_stream_reader)))
            i2s_stream_event(&msg);
        else if (msg.source_type == PERIPH_ID_BUTTON)
        {
             periph_service_event_t evt = { .type = msg.source_type, .source=msg.source, .data = msg.data, .len = msg.data_len };
             input_key_service_cb(input_ser, &evt, board_handle);
        }
        else {
            ESP_LOGW(TAG, "[ * ] event from type=%d, source=%p", msg.source_type, msg.source);
        }

        memset(&msg, 0, sizeof(audio_event_iface_msg_t));
    }

    ESP_LOGI(TAG, "[ 6 ] release all resources");
    audio_pipeline_stop(pipeline_for_record);
    audio_pipeline_wait_for_stop(pipeline_for_record);
    audio_pipeline_terminate(pipeline_for_record);

    audio_pipeline_stop(pipeline_for_play);
    audio_pipeline_wait_for_stop(pipeline_for_play);
    audio_pipeline_terminate(pipeline_for_play);

    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);
    audio_event_iface_destroy(evt);
    esp_periph_set_destroy(set);
}




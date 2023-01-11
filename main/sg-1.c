/* Play mp3 songs from SD card and record to SD card as wav file.
*/

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//#include "tcpip_adapter.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "sdcard_stream.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "mp3_decoder.h"
#include "wav_decoder.h"
#include "wav_encoder.h"
#include "filter_resample.h"
#include "http_stream.h"
#include "raw_stream.h"
#include "esp_netif.h"
//#include "esp_sr_iface.h"
//#include "esp_sr_models.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_touch.h"
#include "periph_button.h"
#include "periph_wifi.h"
#include "input_key_service.h"
#include "periph_adc_button.h"
#include "board.h"
#include "equalizer.h"
#include "sg1.h"
#include "sg_ctrl.h"


static const char *TAG = "SG-1";

//#define USE_WIFI

/* Local variabbles */
static audio_board_handle_t board_handle;
static audio_pipeline_handle_t pipeline_for_record, pipeline_for_play;
static audio_element_handle_t  fatfs_stream_reader, i2s_stream_writer, /*mp3_decoder,*/ wav_decoder, resample_for_play, equalizer;
static audio_element_handle_t  fatfs_stream_writer, i2s_stream_reader, wav_encoder, /*raw_reader,*/ resample_for_rec;
static song_t m_songs[MAX_SONGS] = {0};
static player_state_t m_state = { .version = PLAYER_CONFIG_VERSION,
                                  .song=&m_songs[0], .play_selected_tracks=0x0, .rec_selected_track=1, .played_times=0,
                                  .cursor=0, .play_state=P_STOPED, .rec_state=R_STOPED, 
                                  .rec_opt={SRC_LINEIN, false, false, 0},
                                  .equalizer={{0,0,0,0,0,0,0,0,0,0}},
                                  .volume={{50, 50, 50, 50 /*, 50, 50, 50*/}, 0},
                                  .display=D_SONG,
                                  .song_idx=0 };

#define GPIO_OUTPUT_IO_MICSEL   22
#define GPIO_OUTPUT_AUDIO_VCC   22
#define GPIO_OUTPUT_IO_PREAMP   19
#define GPIO_OUTPUT_PIN_SEL     ((1ULL<<GPIO_OUTPUT_IO_PREAMP) | (1ULL<<GPIO_OUTPUT_IO_MICSEL))

static void allgpios_config()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

static void gpio_set(int gpio, int bSet)
{
    gpio_set_level(gpio, (bSet&1));
}


static bool saveload_config(bool bsave)
{
    char config_file_name[] = { '/','s','d','c','a','r','d','/','c','f','g','_',m_state.version,'.','b','i','n','\0' };
    char songs_file_name[] = { '/','s','d','c','a','r','d','/','s','o','n','g','s','_',m_state.version,'.','b','i','n','\0' };
    FILE * fc = fopen(config_file_name,(bsave)?"wb":"rb");
    FILE * fs = fopen(songs_file_name,(bsave)?"wb":"rb");
    bool bresult = false;

    if (fc != NULL && fs != NULL)
    {
        if (bsave) {
            bresult = (0 < fwrite(&m_state,1,sizeof(player_state_t),fc) && 
                       0 < fwrite(&m_songs,MAX_SONGS,sizeof(song_t),fs));

        }
        else {
            bresult = (0 < fread(&m_state,1,sizeof(player_state_t),fc) &&
                       0 < fread(&m_songs,MAX_SONGS,sizeof(song_t),fs));
            if (bresult) {
                m_state.song_idx = m_state.song_idx % MAX_SONGS;
                m_state.song = &m_songs[m_state.song_idx];
            }
        }
    }

    if (fc != NULL)
        fclose(fc);
    if (fs != NULL)
        fclose(fs);
    
    return (bresult);
}

static void pots_ctrl_proc(POTS_E pot, int value)
{
    static DISPLAY_E saved_display = D_MAX_OPTIONS;

    /* 0 - play vloume, 1 - record volume*/
    int curs =  (pot == POTS_PLAY)? AUDIO_HAL_VOL_OUT_DAC: /*0:*/
                (pot == POTS_REC)?  2:
                (pot == POTS_TONE)? 3:
                -1;
    
    if (curs >= 0)
    {
        if (pot == POTS_REC || pot == POTS_TONE)
        {
            audio_hal_volume_channel_t ch = (pot == POTS_REC)? AUDIO_HAL_VOL_CHANNEL_LEFT:
                                                               AUDIO_HAL_VOL_CHANNEL_RIGHT;

            m_state.volume.bands[curs] = value;
            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[curs], 
                                    AUDIO_HAL_VOL_IN_MIC, ch);
            /*
            int offs = (m_state.volume.bands[curs] - value);
            m_state.volume.bands[curs] -= offs;
            m_state.volume.bands[curs-1] -= offs;
            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[curs-1], 
                       (audio_hal_volume_src_t)(curs-1), AUDIO_HAL_VOL_CHANNEL_LEFT);
            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[curs], 
                       (audio_hal_volume_src_t)curs, AUDIO_HAL_VOL_CHANNEL_RIGHT);
            */
        }
        else
        {
            m_state.volume.bands[curs] = value;
            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[curs], 
                                   (audio_hal_volume_src_t)curs, AUDIO_HAL_VOL_CHANNEL_BOTH);
            /*
            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[curs], 
                                   AUDIO_HAL_VOL_OUT_HEADPHONE or AUDIO_HAL_VOL_OUT_SPK , AUDIO_HAL_VOL_CHANNEL_BOTH);
            */
        }


        if (saved_display == D_MAX_OPTIONS) {
            saved_display = m_state.display;
            m_state.display = D_VOLUME;
        }

        if (m_state.display == D_VOLUME)
            display_player_state(&m_state);
    }
    else if (saved_display != D_MAX_OPTIONS) {
        m_state.display = saved_display;
        saved_display = D_MAX_OPTIONS; 
    }

    ESP_LOGW(TAG, "[ * ] POTS control %d, value %d", pot, value);
}

static void button2_ctrl_proc(CTRL2_BUTTON_E bt, EVT_BUTTON_E evt)
{
    if (evt == EVT_RELEASED)
        return;

    ESP_LOGW(TAG, "[ * ] BT2 control %d, value %d", bt, evt);
}

static void button_ctrl_proc(CTRL_BUTTON_E bt, EVT_BUTTON_E evt)
{
    if (evt == EVT_RELEASED || (evt == EVT_LONGPRESS && bt != ENC1_SW/*BT_SET*/))
        return;

    if (evt == EVT_LONGPRESS)
    {
        #if 0
        if (m_state.display == D_SONG)
        {
            if (bt == ENC1_SW/*BT_SET*/)
            {
                int track_num = 1 + ((m_state.song->cursor >= MAX_TRACKS)? m_state.song->cursor-MAX_TRACKS: m_state.song->cursor);
                char track_name[64];
                sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num+1, track_num);
                ESP_LOGI(TAG, "[ * ] Removing track %s", track_name);
                remove(track_name);
                m_state.song->tracks[track_num-1].len_in_sec = 0;
            }
        }
        #endif
        m_state.display = (DISPLAY_E)((m_state.display + 1) % D_MAX_OPTIONS);

        display_player_state(&m_state);
        return;
    }

    switch (bt)
    {
        #if 0
        case BT_LEFT:
            if (m_state.display == D_SONG)
                do {
                    m_state.song->cursor = (m_state.song->cursor==0)? (MAX_TRACKS*2-1): (m_state.song->cursor-1);
                } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
            else if (m_state.display == D_VOLUME)
                m_state.volume.cursor = (m_state.volume.cursor==0)? (MAX_VOL_BANDS-1): (m_state.volume.cursor-1);
            else if (m_state.display == D_EQUALIZER)
                m_state.equalizer.cursor = (m_state.equalizer.cursor==0)? (MAX_EQ_BANDS-1): (m_state.equalizer.cursor-1);
            else if (m_state.display == D_REC_OPT)
                m_state.rec_opt.cursor = (m_state.rec_opt.cursor==0)? (MAX_REC_OPTONS-1): (m_state.rec_opt.cursor-1);
            break; 
        case BT_VOLUME_SW:
        case BT_RIGTH:
            if (m_state.display == D_SONG)
                do {
                    m_state.song->cursor = (m_state.song->cursor + 1) % (MAX_TRACKS*2);    
                } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
                
            else if (m_state.display == D_VOLUME)
                m_state.volume.cursor = (m_state.volume.cursor+1) % MAX_VOL_BANDS;
            else if (m_state.display == D_EQUALIZER)
                m_state.equalizer.cursor = (m_state.equalizer.cursor+1) % MAX_EQ_BANDS;
            else if (m_state.display == D_REC_OPT)
                m_state.rec_opt.cursor = (m_state.rec_opt.cursor+1) % MAX_REC_OPTONS;
            break; 
        case BT_UP:
            m_state.display = (DISPLAY_E)((m_state.display == D_SONG)? (D_MAX_OPTIONS-1): (m_state.display-1));  
            break; 
        case BT_DOWN:
            m_state.display = (DISPLAY_E)((m_state.display + 1) % D_MAX_OPTIONS);
            break; 
        #endif
        
        case ENC1_SW/*BT_SET*/:
            if (m_state.display == D_SELECT_SONG)
            {
                button_ctrl_proc(BT_REFRESH, EVT_PRESSED);
                return;
            }
            else if (m_state.display == D_SONG)
            {
                if (m_state.song->cursor >= MAX_TRACKS) 
                    m_state.rec_selected_track = (m_state.rec_selected_track != (m_state.song->cursor-MAX_TRACKS+1))? 
                                                  (m_state.song->cursor-MAX_TRACKS+1): 0;
                else 
                    m_state.play_selected_tracks = (m_state.play_selected_tracks & (1<<m_state.song->cursor))? 0:
                                                   (1<<m_state.song->cursor); //Right now only one track is selected

                    //m_state.play_selected_tracks = (m_state.play_selected_tracks & (1<<m_state.song->cursor))? 
                    //                       (m_state.play_selected_tracks & ~(1<<m_state.song->cursor)):
                    //                       (m_state.play_selected_tracks | (1<<m_state.song->cursor));
            }
            else if (m_state.display == D_VOLUME)
            {
                //m_state.volume.bands[m_state.volume.cursor] = ((m_state.volume.bands[m_state.volume.cursor]+10) % 100);
                m_state.volume.cursor = (m_state.volume.cursor+1) % MAX_VOL_BANDS;
                //audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[m_state.volume.cursor], 
                //                        (audio_hal_volume_src_t)m_state.volume.cursor, AUDIO_HAL_VOL_CHANNEL_BOTH);
            }
            else if (m_state.display == D_EQUALIZER)
            {
                m_state.equalizer.cursor = (m_state.equalizer.cursor+1) % MAX_EQ_BANDS;
                //m_state.equalizer.bands[m_state.equalizer.cursor] = 
                //                           ((m_state.equalizer.bands[m_state.equalizer.cursor]+1+20) % 40) - 20;

                //TODO:  Set equalizer in range -20db..20db
            }
            else if (m_state.display == D_REC_OPT)
            {
                if (m_state.rec_opt.cursor == 0) // Record source
                {
                    m_state.rec_opt.rec_source = (REC_SOURCE_E)!m_state.rec_opt.rec_source;
                    audio_hal_ctrl_codec(board_handle->audio_hal, 
                        ((m_state.rec_opt.rec_source == SRC_MIC)? AUDIO_HAL_CODEC_MODE_ENCODE:
                                                                  AUDIO_HAL_CODEC_MODE_LINE_IN),
                        AUDIO_HAL_CTRL_START);
                }
                else if (m_state.rec_opt.cursor == 1) //Monitor enable (AC101 codec only)
                {
                    m_state.rec_opt.bmonitor = !m_state.rec_opt.bmonitor;
                    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_PASSTHROUGH, 
                                        (m_state.rec_opt.bmonitor? AUDIO_HAL_CTRL_START: AUDIO_HAL_CTRL_STOP));
                }
                else if (m_state.rec_opt.cursor == 2) // Toogle record mixer (AC101 codec only)
                {
                    m_state.rec_opt.brecordmix = !m_state.rec_opt.brecordmix;
                    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_RECORD_MIX, 
                                        (m_state.rec_opt.brecordmix? AUDIO_HAL_CTRL_START: AUDIO_HAL_CTRL_STOP));
                }
                else if (m_state.rec_opt.cursor == 3) //Save configuration 
                {
                    bool bsaved = saveload_config(true);
                    ESP_LOGI(TAG, "[ * ] Configuration save %s !!", (bsaved?"success":"failed"));
                }
            }

            /*********** For internal test *********************/
            {
                int underun=0, overflow=0, start=0, stop=0;
                audio_element_handle_t i2s_el = audio_pipeline_get_el_by_tag(pipeline_for_play, "i2s");
                audio_element_handle_t i2s_el2 = audio_pipeline_get_el_by_tag(pipeline_for_record, "i2s");
                if (audio_element_get_state(i2s_el) == AEL_STATE_RUNNING)
                    i2s_get_errors(i2s_el, &underun, NULL, &start, &stop);
                if (audio_element_get_state(i2s_el2) == AEL_STATE_RUNNING)
                    i2s_get_errors(i2s_el2, NULL, &overflow, &start, &stop);
                if (underun || overflow || start || stop)
                {
                    ESP_LOGW(TAG, "[ !!! ] TX-underuns=%d, RX-overflows=%d  Start=%d/%d", underun, overflow, start, stop);
                }
            }
            /****************************************************/

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
                    ESP_LOGI(TAG, "[ * ] Pausing play audio pipeline (not ready yet)");
                    //audio_pipeline_pause(pipeline);
                    //m_state.play_state = P_PAUSE;
                    //set_led(OUT_LED_GREEN, false);
                    break;
                case AEL_STATE_PAUSED :
                    ESP_LOGI(TAG, "[ * ] Resuming play audio pipeline (not ready yet)");
                    //audio_pipeline_resume(pipeline);
                    //m_state.play_state = P_PLAYING;
                    //set_led(OUT_LED_GREEN, true);
                    break;
                case AEL_STATE_INIT :
                    //fall throu
                case AEL_STATE_STOPPED:
               
                case AEL_STATE_FINISHED:
                    if (el_state == AEL_STATE_INIT)
                    {
                        ESP_LOGI(TAG, "[ * ] Starting play audio pipeline");
                    }

                    if ( fatfs_el && track_num )
                    {
                        char track_name[64];
                        sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num+1, track_num);
                        audio_element_set_uri(fatfs_el, track_name);
                        ESP_LOGI(TAG, "[ * ] Starting to play track: %s", track_name);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_run(pipeline);
                        m_state.play_state = P_PLAYING;
                        set_led(OUT_LED_GREEN, true);
                        m_state.playing_tracks = track_num;
                        m_state.played_times = 0;
                    }
                    else if (!track_num)
                    {
                        ESP_LOGI(TAG, "[ * ] Please, select at least one track");
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
                case AEL_STATE_PAUSED :
                    if (m_state.rec_state == R_RECORDING)
                    {
                        ESP_LOGI(TAG, "[ * ] Mute rec audio pipeline");
                        audio_hal_set_mute(board_handle->audio_hal, true);
                        m_state.rec_state = R_MUTE;
                        set_led(OUT_LED_RED, false);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "[ * ] Unmute rec audio pipeline");
                        audio_hal_set_mute(board_handle->audio_hal, false);
                        m_state.rec_state = R_RECORDING;
                        set_led(OUT_LED_RED, true);
                    }
                    break;
                case AEL_STATE_INIT :
                case AEL_STATE_STOPPED:
                
                case AEL_STATE_FINISHED:

                    if (AEL_STATE_INIT == el_state)
                    {
                        ESP_LOGI(TAG, "[ * ] Starting rec audio pipeline");
                    }
                    if ( fatfs_el && track_num)
                    {
                        char track_name[64];
                        sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num+1, track_num);
                        fclose(fopen(track_name, "wb"));
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
        {
            audio_element_state_t stw, str;
            if (AEL_STATE_FINISHED != (stw = audio_element_get_state(i2s_stream_writer)))
            {
                audio_pipeline_stop(pipeline_for_play);
                audio_pipeline_wait_for_stop(pipeline_for_play);
            }

            if (AEL_STATE_RUNNING == (str = audio_element_get_state(i2s_stream_reader)))
            {
                audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline_for_record, "file");
                audio_element_info_t info;
                
                if ((m_state.rec_state == R_MUTE || m_state.rec_state == R_RECORDING) && audio_element_getinfo(fatfs_el, &info) == ESP_OK)
                    if (m_state.recording_track > 0)
                        m_state.song->tracks[m_state.recording_track-1].len_in_sec = (info.byte_pos / (2 * CONFIG_BITRATE_SAMPLING));

                audio_pipeline_stop(pipeline_for_record);
                audio_pipeline_wait_for_stop(pipeline_for_record);
            }
            audio_pipeline_reset_ringbuffer(pipeline_for_play);
            audio_pipeline_reset_ringbuffer(pipeline_for_record);
            m_state.play_state = P_STOPED;
            m_state.rec_state = R_STOPED;
            m_state.recording_track = m_state.playing_tracks = 0;
            set_led(OUT_LED_GREEN, false);
            set_led(OUT_LED_RED, false);

            ESP_LOGI(TAG, "[ * ] Stop activated (%d,%d)", stw, str);

            if (m_state.display == D_SELECT_SONG) 
            {
                button_ctrl_proc(BT_REFRESH, EVT_PRESSED);
                return;
            }

            // LUC - setting display to D_SELECT_SONG when pressing STOP twice
            if ( stw != AEL_STATE_RUNNING && stw != AEL_STATE_PAUSED && str != AEL_STATE_RUNNING && str != AEL_STATE_PAUSED)
                m_state.display = D_SELECT_SONG;
        }

            break;

        case BT_FORWARD:
            /* Fall down */
        case BT_REWARD:
        case BT_REFRESH:
            if (bt == BT_REWARD)
            {
                if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                    m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
                {
                    int prev = (m_state.song_idx==0)? (MAX_SONGS-1): (m_state.song_idx-1);
                    m_state.song = &m_songs[prev];
                    m_state.song->num = prev;
                }
            }
            else if (bt == BT_FORWARD)
            {
                if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                    m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
                {
                    int next = (m_state.song_idx+1) % MAX_SONGS;
                    m_state.song = &m_songs[next];
                    m_state.song->num = next;
                }
            }
            else if (bt == BT_REFRESH)
            {
                m_state.song = &m_songs[m_state.song_idx];
                m_state.song->num = m_state.song_idx;
                bool bsaved = saveload_config(true);
                ESP_LOGI(TAG, "[ * ] Configuration save %s !!", (bsaved?"success":"failed"));
            }

            {
                char song_dir[32];
                sprintf(song_dir, "/sdcard/song_%d", (1+m_state.song->num));
                mkdir(song_dir, 0777);
                m_state.song->cursor = 0;
                for (int i=0; i<MAX_TRACKS; i++)
                {
                    struct stat st;
                    char filename[64];
                    sprintf(filename, "%s/track_%d.wav",song_dir, (i+1));
                    m_state.song->tracks[i].len_in_sec = (stat(filename, &st) == ESP_OK)?
                                                         (st.st_size / (2 * CONFIG_BITRATE_SAMPLING)): 0;
                    if (m_state.song->cursor == i && !m_state.song->tracks[i].len_in_sec)
                        m_state.song->cursor++;
                }

                //m_state.rec_selected_track = 0;
                //m_state.play_selected_tracks = 0;
                m_state.display = D_SONG;
            }
            break;


        case ENC1_CW/*EN_VOLUME_UP*/:
        case ENC1_CCW/*EN_VOLUME_DOWN*/:
            /*
            if (m_state.display == D_SONG || m_state.display == D_REC_OPT)
                m_state.display = D_VOLUME;
            */
            if (m_state.display == D_SONG)
            {
                if (bt == ENC1_CW)
                    do {
                        m_state.song->cursor = (m_state.song->cursor + 1) % (MAX_TRACKS*2);    
                    } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
                else
                    do {
                        m_state.song->cursor = (m_state.song->cursor==0)? (MAX_TRACKS*2-1): (m_state.song->cursor-1);
                    } while (m_state.song->cursor < MAX_TRACKS && m_state.song->tracks[m_state.song->cursor].len_in_sec == 0);
            }
            else if (m_state.display == D_REC_OPT)
            {
                if (bt == ENC1_CW)
                    m_state.rec_opt.cursor = (m_state.rec_opt.cursor+1) % MAX_REC_OPTONS;
                else
                    m_state.rec_opt.cursor = (m_state.rec_opt.cursor==0)? (MAX_REC_OPTONS-1): (m_state.rec_opt.cursor-1);
            }
            else if (m_state.display == D_VOLUME)
            {
                m_state.volume.bands[m_state.volume.cursor] = (uint)(m_state.volume.bands[m_state.volume.cursor]+((bt==ENC1_CW/*EN_VOLUME_UP*/)?5:-5));
                if (m_state.volume.bands[m_state.volume.cursor] > 100)
                    m_state.volume.bands[m_state.volume.cursor] = 100;
                if (m_state.volume.bands[m_state.volume.cursor] < 0)
                    m_state.volume.bands[m_state.volume.cursor] = 0;


                audio_hal_volume_channel_t ch = (m_state.volume.cursor == AUDIO_HAL_VOL_IN_LINEIN)? AUDIO_HAL_VOL_CHANNEL_LEFT:
                                                (m_state.volume.cursor == AUDIO_HAL_VOL_IN_MIC)? AUDIO_HAL_VOL_CHANNEL_RIGHT:
                                                AUDIO_HAL_VOL_CHANNEL_BOTH;

                audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[m_state.volume.cursor], 
                                    (audio_hal_volume_src_t)m_state.volume.cursor+((ch==AUDIO_HAL_VOL_CHANNEL_LEFT)?1:0), ch);
            }
            else if (m_state.display == D_EQUALIZER)
            {
                m_state.equalizer.bands[m_state.equalizer.cursor] = 
                                           (m_state.equalizer.bands[m_state.equalizer.cursor]+((bt==ENC1_CW/*EN_VOLUME_UP*/)?2:-2)+25) - 25;
                if (m_state.equalizer.bands[m_state.equalizer.cursor] > 25)
                    m_state.equalizer.bands[m_state.equalizer.cursor] = 25;
                if (m_state.equalizer.bands[m_state.equalizer.cursor] < -25)
                    m_state.equalizer.bands[m_state.equalizer.cursor] = -25;

                //TODO:  Set equalizer in range -20db..20db
                equalizer_set_gain_info(equalizer, m_state.equalizer.cursor, m_state.equalizer.bands[m_state.equalizer.cursor], true);
            }
            else if (m_state.display == D_SELECT_SONG)
            { 
               if (bt == ENC1_CW)
               {
                    if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                        m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
                    {
                        int next = (m_state.song_idx+1) % MAX_SONGS;
                        m_state.song = &m_songs[next];
                        m_state.song_idx = m_state.song->num = next;
                    }
                }
                else if (bt == ENC1_CCW) 
                {
                    if (m_state.rec_state != R_MUTE && m_state.rec_state != R_RECORDING &&
                        m_state.play_state != P_PAUSE && m_state.play_state != P_PLAYING)
                    {
                        int prev = (m_state.song_idx==0)? (MAX_SONGS-1): (m_state.song_idx-1);
                        m_state.song = &m_songs[prev];
                        m_state.song_idx = m_state.song->num = prev;
                    }
                }
            }
            break; 
        default:
            ESP_LOGW(TAG, "[ * ] %d Unknown control", bt);
            break;
    }
    

    display_player_state(&m_state);
}



void i2s_fatfs_event(audio_event_iface_msg_t * msg)
{
    ESP_LOGI(TAG, "[ * ] fatfs event=%d data=%d", msg->cmd, (int)msg->data);
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

                if (pipeline == pipeline_for_play)
                {
                    if (++m_state.played_times < 4)
                    {
                        audio_element_handle_t fatfs_el = audio_pipeline_get_el_by_tag(pipeline, "file");
                        //audio_element_handle_t i2s_el = audio_pipeline_get_el_by_tag(pipeline, "i2s");
                        //audio_element_state_t el_state = audio_element_get_state(i2s_el);

                        if ( fatfs_el && m_state.playing_tracks )
                        {
                            char track_name[64];
                            sprintf(track_name, "/sdcard/song_%d/track_%d.wav", m_state.song->num+1, m_state.playing_tracks);
                            audio_element_set_uri(fatfs_el, track_name);
                            ESP_LOGI(TAG, "[ * ] Starting to play track: %s", track_name);
                        }

                        ESP_LOGI(TAG, "[ * ] Loop track: %d", m_state.played_times);
                        audio_pipeline_reset_elements(pipeline);
                        audio_pipeline_run(pipeline);
                    }
                    else
                    {
                        set_led(OUT_LED_GREEN, false);
                        m_state.play_state = P_STOPED;
                    }
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
                /*
                if (m_state.rec_state == P_STOPED)
                {
                    audio_element_setinfo(i2s_stream_writer, &music_info);
                    audio_element_setinfo(i2s_stream_reader, &music_info);
                }
                */
                rsp_filter_set_src_info((bDecoder?resample_for_play:resample_for_rec), music_info.sample_rates, music_info.channels);
            }
            break;
        
        default:
            ESP_LOGI(TAG, "[ * ] wave %s event=%d data=%d", (bDecoder?"decoder":"encoder"), msg->cmd, (int)msg->data);
            break;
    }
}


static audio_element_handle_t setup_fatfs(bool bPlayer, const char * url)
{
    audio_element_info_t info;
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.out_rb_size = (256 * 1024); //bPlayer? (128 * 1024): (192 * 1024);
    fatfs_cfg.buf_sz = (64 * 1024); //bPlayer? (32 * 1024): (24 * 1024);
    fatfs_cfg.type = bPlayer? AUDIO_STREAM_READER: AUDIO_STREAM_WRITER;
    audio_element_handle_t fatfs_stream = fatfs_stream_init(&fatfs_cfg);
    audio_element_set_uri(fatfs_stream, url);
    audio_element_getinfo(fatfs_stream, &info);
    info.sample_rates = CONFIG_BITRATE_SAMPLING;
    audio_element_setinfo(fatfs_stream, &info);

    return fatfs_stream;
}

static audio_element_handle_t setup_wav_codec(bool bPlayer)
{
    audio_element_info_t info;
    audio_element_handle_t wav_coder;
    if (bPlayer) {
        wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
        wav_coder = wav_decoder_init(&wav_cfg);
    }
    else {
        wav_encoder_cfg_t wav_file_cfg = DEFAULT_WAV_ENCODER_CONFIG();
        wav_coder = wav_encoder_init(&wav_file_cfg);
    }
    audio_element_getinfo(wav_coder, &info);
    info.sample_rates = CONFIG_BITRATE_SAMPLING;
    audio_element_setinfo(wav_coder, &info);

    return wav_coder;
}

static audio_element_handle_t setup_equalizer_filter()
{
    audio_element_info_t info;
    equalizer_cfg_t eq_cfg = DEFAULT_EQUALIZER_CONFIG();
    static int set_gain[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    eq_cfg.set_gain   = set_gain; // The size of gain array should be the multiplication of NUMBER_BAND and number channels of audio stream data. The minimum of gain is -13 dB.
    eq_cfg.samplerate = CONFIG_BITRATE_SAMPLING;
    eq_cfg.channel    = 2;
    audio_element_handle_t eq = equalizer_init(&eq_cfg);
    audio_element_getinfo(eq, &info);
    info.sample_rates = CONFIG_BITRATE_SAMPLING;
    audio_element_setinfo(eq, &info);

    return eq;
}


static audio_element_handle_t setup_resample_filter()
{
    audio_element_info_t info;
    rsp_filter_cfg_t rsp_filter_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_filter_cfg.src_rate = rsp_filter_cfg.dest_rate = CONFIG_BITRATE_SAMPLING;
    rsp_filter_cfg.src_ch = rsp_filter_cfg.dest_ch = 2;
    audio_element_handle_t resample_filter = rsp_filter_init(&rsp_filter_cfg);
    audio_element_getinfo(resample_filter, &info);
    info.sample_rates = CONFIG_BITRATE_SAMPLING;
    audio_element_setinfo(resample_filter, &info);

    return resample_filter;
}

static audio_element_handle_t setup_i2s(bool bPlayer)
{
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = bPlayer? AUDIO_STREAM_WRITER: AUDIO_STREAM_READER;
    i2s_cfg.i2s_config.sample_rate = CONFIG_BITRATE_SAMPLING;
    i2s_cfg.out_rb_size = (16 * 1024);
    audio_element_handle_t i2s_stream = i2s_stream_init(&i2s_cfg);

    return i2s_stream;
}

static audio_pipeline_handle_t setup_record_pipeline(const char * url)
{
    ESP_LOGI(TAG, "- Create audio pipeline for record");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    audio_pipeline_register(pipeline, (i2s_stream_reader = setup_i2s(false)),           "i2s");
    audio_pipeline_register(pipeline, (wav_encoder = setup_wav_codec(false)),           "wav");
    audio_pipeline_register(pipeline, (resample_for_rec = setup_resample_filter()),     "filter");
    audio_pipeline_register(pipeline, (fatfs_stream_writer = setup_fatfs(false, url)),  "file");

    audio_pipeline_link(pipeline, (const char *[]) {"i2s", "filter", "wav", "file"}, 4);
    
    return pipeline;
}

static struct audio_pipeline * setup_play_pipeline(const char * url)
{
    ESP_LOGI(TAG, "- Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    audio_pipeline_register(pipeline, (i2s_stream_writer = setup_i2s(true)),           "i2s");
    audio_pipeline_register(pipeline, (wav_decoder = setup_wav_codec(true)),           "wav");
    audio_pipeline_register(pipeline, (resample_for_play = setup_resample_filter()),   "filter");
    audio_pipeline_register(pipeline, (equalizer = setup_equalizer_filter()),          "equlizer");
    audio_pipeline_register(pipeline, (fatfs_stream_reader = setup_fatfs(true, url)),  "file");

    audio_pipeline_link(pipeline, (const char *[]) {"file", "wav", "filter", "equlizer", "i2s"}, 5);
    /*
    ESP_LOGI(TAG, "- Create a ringbuffer and insert it between mp3 decoder and i2s writer");
    ringbuf_handle_t ringbuf = rb_create(16 * 1024, 1);
    audio_element_set_input_ringbuf(i2s_stream_writer, ringbuf);
    audio_element_set_output_ringbuf(wav_decoder, ringbuf);
    */

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
    audio_board_sdcard_init(set, SD_MODE_4_LINE);
    
    ESP_LOGI(TAG, "[ 1.2 ] Initialize keys");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ - ] Try to load configuration from sd card");
    saveload_config(false);
    //runSDTest();

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

    if ( (board_handle = audio_board_init()) )
    {
        audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_PASSTHROUGH, 
                            (m_state.rec_opt.bmonitor? AUDIO_HAL_CTRL_START: AUDIO_HAL_CTRL_STOP));

        audio_hal_ctrl_codec(board_handle->audio_hal, 
            ((m_state.rec_opt.rec_source == SRC_MIC)? AUDIO_HAL_CODEC_MODE_ENCODE:
                                                      AUDIO_HAL_CODEC_MODE_LINE_IN),
                            AUDIO_HAL_CTRL_START);

        // Not supported in ES8388
        //audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_RECORD_MIX, 
        //                    (m_state.rec_opt.brecordmix? AUDIO_HAL_CTRL_START: AUDIO_HAL_CTRL_STOP));
/*
#ifdef CONFIG_INPUT_MIC
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
    ESP_LOGI(TAG, "[ 2 ] Start codec chip (MIC)");
#else
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
    ESP_LOGI(TAG, "[ 2 ] Start codec chip (LINE-IN)");
#endif
*/
        /* set all volumes */
        for (int v=0; v<MAX_VOL_BANDS; v++) {
            audio_hal_volume_channel_t ch = (v == AUDIO_HAL_VOL_IN_LINEIN)? AUDIO_HAL_VOL_CHANNEL_LEFT:
                                            (v == AUDIO_HAL_VOL_IN_MIC)? AUDIO_HAL_VOL_CHANNEL_RIGHT:
                                            AUDIO_HAL_VOL_CHANNEL_BOTH;

            audio_hal_set_volume_ex(board_handle->audio_hal, m_state.volume.bands[v], 
                                    (audio_hal_volume_src_t)v+((ch==AUDIO_HAL_VOL_CHANNEL_LEFT)?1:0), ch);
        }
    }

    pipeline_for_play = setup_play_pipeline(NULL);
    pipeline_for_record = setup_record_pipeline(NULL);

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
        registrate_cb(button_ctrl_proc, button2_ctrl_proc, pots_ctrl_proc);

    ESP_LOGI(TAG, "[4.4] Set pream off & mic to internal %d", gpio_get_level(GPIO_NUM_39));
    allgpios_config();
    //gpio_set(GPIO_OUTPUT_IO_MICSEL, 0);
    gpio_set(GPIO_OUTPUT_AUDIO_VCC, 1);
    gpio_set(GPIO_OUTPUT_IO_PREAMP, 1);

    button_ctrl_proc(BT_REFRESH, EVT_PRESSED);

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
            /* Proccess on board buttons (int 4 lanes mode there are only 2 buttons available record and set) */
            if ((int)msg.data == INPUT_KEY_USER_ID_REC)
                button_ctrl_proc(BT_REC, EVT_PRESSED);
            else if ((int)msg.data == INPUT_KEY_USER_ID_SET)
                button_ctrl_proc(BT_STOP, EVT_PRESSED);
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

    audio_board_deinit(board_handle);
}




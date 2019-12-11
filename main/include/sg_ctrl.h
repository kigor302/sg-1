#ifndef _SG_CTRL_H_
#define _SG_CTRL_H_

#include "esp_action_def.h"
#include "audio_service.h"
#include "periph_service.h"
#include "display_service.h"
#include "esp_dispatcher.h"


#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INT_IO_PIN       5
#define GPIO_SCL_PIN		  23
#define GPIO_SDA_PIN		  18
#define GPIO_INPUT_PIN_SEL    (1ULL<<GPIO_INT_IO_PIN)

#define MAX_SONGS			  (9)
#define MAX_TRACKS			  (5)
#define MAX_EQ_BANDS		  (6)

typedef enum { BT_LEFT=0, BT_RIGTH, BT_UP, BT_DOWN, BT_SET, BT_PLAY, BT_REC, BT_STOP, BT_FORWARD, 
	           BT_REWARD, EN_VOLUME_UP, EN_VOLUME_DOWN, BT_VOLUME_SW, OUT_LED_GREEN, OUT_LED_RED, OUT_OLED }CTRL_BUTTON_E;

typedef enum { EVT_PRESSED=0, EVT_RELEASED, EVT_STEP }EVT_BUTTON_E;

typedef enum { P_STOPED=0, P_PLAYING,   P_PAUSE }PLAY_STATE_E;
typedef enum { R_STOPED=0, R_RECORDING, R_MUTE }REC_STATE_E;
typedef enum { SRC_LINEIN=0, SRC_MIC }REC_SOURCE_E;
typedef enum { D_SONG =0, D_VOLUME, D_EQUALIZER, D_REC_OPT, D_MAX_OPTIONS }DISPLAY_E;

typedef struct _track
{
	int num;
	int len_in_sec;
}track_t;

typedef struct _song
{
	int num;
	track_t tracks[MAX_TRACKS];
	int cursor; /* 0 - (1 play track selection),  5 - (1 recording track selection) */
}song_t;

typedef struct _record_opt
{
	REC_SOURCE_E  rec_source;
	bool          bmonitor;
	int           cursor; /* 0 - source selection, 1 - Monitor selection */
}record_opt_t;

typedef struct _volume
{
	int play_volume;
	int rec_volume;
	bool play_vol_selected;
	uint last_time_selected;
}volume_t;

typedef struct _equalizer
{
	int bands[MAX_EQ_BANDS];
	int cursor; /* 0 - first band selection */
}equalizer_t;

typedef struct _player_state
{
	song_t *  	song;
	volume_t  	volume;
	equalizer_t	equalizer;
	record_opt_t rec_opt;

	uint8_t play_selected_tracks; /* bits: <0 | 1 | 2 | 3 | 4>  (0 - no playing tracks) */
	uint8_t rec_selected_track;   /* track num <1 - 5>  (0 - no selected tracks) */  
	uint8_t cursor;               /* 0-4 played tracks,  5-9 rec tracks */

	PLAY_STATE_E  play_state;
	REC_STATE_E   rec_state;
	int           recording_track;
	int           playing_tracks;

	DISPLAY_E   display;

}player_state_t;


typedef void (*button_cb_t)(CTRL_BUTTON_E bt, EVT_BUTTON_E evt);

/* Control board initialization */
esp_err_t init_ctrl_board();

/* Control board initialization */
void registrate_cb(button_cb_t cb);

/* Control LED on board */
void set_led(CTRL_BUTTON_E led, bool bOn);

/* Show current state to OLED display */
void display_player_state(player_state_t * state);

#endif


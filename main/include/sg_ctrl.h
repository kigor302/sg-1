#ifndef _SG_CTRL_H_
#define _SG_CTRL_H_

#include "esp_action_def.h"
#include "audio_service.h"
#include "periph_service.h"
#include "display_service.h"
#include "esp_dispatcher.h"


#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INT_IO_PIN       5
//#define GPIO_INT2_IO_PIN      19
#define GPIO_SCL_PIN		  23
#define GPIO_SDA_PIN		  18
#define GPIO_INPUT_PIN_SEL    (1ULL<<GPIO_INT_IO_PIN)
#define GPIO_PREAMP_IO_PIN	  (19)

#define MAX_SONGS			  (9)
#define MAX_TRACKS			  (5)
#define MAX_EQ_BANDS		  (10)
#define MAX_VOL_BANDS		  (4) /*8*/
#define MAX_REC_OPTONS		  (4)

typedef enum { POTS_PLAY=0, POTS_REC, POTS_TONE, MAX_POTS }POTS_E;

typedef enum { LVL_6=0, LVL_5, LVL_4, LVL_3, LVL_2, LVL_1, RESERVED_A, RESERVED_B, 
			   IO1_0, IO1_1,  IO1_2,  IO1_3,  IO1_4,  IO1_5,  IO1_6,  IO1_7 }CTRL2_BUTTON_E;

//typedef enum { ENC1_SW=0, ENC1_CW, ENC1_CCW, OUT_MIC_VCC, OUT_PREAMP_EN, LOOP_SW, PWM_0, PWM_1,
//			   OUT_9V,  PWM_EN, PWM_2, OUT_OLED_KEY_EN, BT_BUTTON1, BT_BUTTON2, BT_BUTTON3, BT_BUTTON4 }CTRL2_BUTTON_E;

#define OUT_LED_GREEN       PLAY_LED
#define OUT_LED_RED         REC_LED
typedef enum { ENC1_SW=0, ENC1_CW,  ENC1_CCW,  BT_REC,   BT_STOP,   BT_PLAY,    PLAY_LED,    REC_LED, 
               POTS_EN,    BRG_A,  BRG_B,    BRG_INH,   STOP_LED,   BT_FORWARD/*RESERVED_X*/, BT_REWARD/*RESERVED_Y*/, OUT_OLED }CTRL_BUTTON_E;

//typedef enum { BT_LEFT=0, BT_RIGTH, BT_UP, BT_DOWN, BT_SET, BT_PLAY, BT_REC, BT_STOP, BT_FORWARD, 
//	             BT_REWARD, EN_VOLUME_UP, EN_VOLUME_DOWN, BT_VOLUME_SW, OUT_LED_GREEN, OUT_LED_RED, OUT_OLED }CTRL_BUTTON_E;

/*
typedef enum { BT_LEFT=16, BT_RIGTH, BT_UP, BT_DOWN, BT_SET, 
	           BT_FORWARD, BT_REWARD, EN_VOLUME_UP, EN_VOLUME_DOWN,
	           BT_VOLUME_SW, OUT_LED_GREEN, OUT_LED_RED,
	           OUT_MIC_VCC, OUT_PREAMP_EN, LOOP_SW, PWM_0, PWM_1, 
	           OUT_9V,  PWM_EN, PWM_2, OUT_OLED_KEY_EN } CTRL_WORKAROND_E;
*/


typedef enum { EVT_PRESSED=0, EVT_RELEASED, EVT_STEP, EVT_LONGPRESS }EVT_BUTTON_E;

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
	bool          bmonitor; /* record to outpt passtrough */
	bool          brecordmix; /* mix play and record together */
	int           cursor; /* 0 - source selection, 1 - Monitor selection, 2 - Record mixer*/
}record_opt_t;

typedef struct _volume
{
	int bands[MAX_VOL_BANDS];
	int cursor; /* 0 - play vloume, 1 - record volume, 2 - line-in, 3 - microphone, 4 - headset volume, 5 - speaker volume */
	uint last_time_selected;
}volume_t;

typedef struct _equalizer
{
	int bands[MAX_EQ_BANDS];
	int cursor; /* 0 - first band selection */
}equalizer_t;

#define PLAYER_CONFIG_VERSION	'2'
typedef struct _player_state
{
	char		version;
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
typedef void (*button2_cb_t)(CTRL2_BUTTON_E bt, EVT_BUTTON_E evt);
typedef void (*pots_cb_t)(POTS_E pot, int value);

/* Control board initialization */
esp_err_t init_ctrl_board();

/* Control board initialization */
void registrate_cb(button_cb_t cb, button2_cb_t cb2, pots_cb_t cb3);

/* Control LED on board */
void set_led(CTRL_BUTTON_E led, bool bOn);

/* Show current state to OLED display */
void display_player_state(player_state_t * state);

void display_clear(void);

/* Control second board outputs */
void set_gpio_out_1(CTRL2_BUTTON_E gpio, bool bOn);
void set_gpio_out_0(CTRL_BUTTON_E gpio, bool bOn);


#endif


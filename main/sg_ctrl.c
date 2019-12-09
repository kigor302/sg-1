#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <esp_timer.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include "sdkconfig.h"
#include "ssd1306.h"
#include "font.h"
#include "iface_esp32_i2c.h"
#include "iface_virtual.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sg_ctrl.h"
#include "ads111x.h"
#include "pca9555.h"


/* #define USE_ADS111X */  /* <=== Uncomment if using ADS111X converter */


/* Local variables */
static xQueueHandle m_gpio_evt_queue = NULL;
static button_cb_t  m_evt_callback_func = NULL;
static struct SSD1306_Device m_Dev_I2C;
static i2c_dev_t m_pca9555_I2C;
static uint16_t  m_last_pca9555_value;

#ifdef USE_ADS111X
	static i2c_dev_t m_ads111x_I2C;
#endif

static const char *TAG = "SG-CTRL";
static bool m_active = false;

static const uint8_t pca9555_io_mapping[16] = { BT_VOLUME_SW,  EN_VOLUME_UP, EN_VOLUME_DOWN,  BT_STOP,   BT_PLAY, BT_REC,  BT_REWARD, BT_FORWARD,
										 	    /*pin0*/        /*pin1(A)*/   /*pin2(B)*/      /*pin3*/  /*pin4*/ /*pin5*/ /*pin6*/   /*pin7*/
                                                BT_UP,    BT_DOWN,  BT_LEFT,  BT_RIGTH,  BT_SET,   OUT_LED_GREEN, OUT_LED_RED, OUT_OLED };
                                                /*pin8*/  /*pin9*/ /*pin10*/  /*Pin11*/ /*pin12*/  /*pin13*/      /*pin14*/    /*pin16*/


static const uint16_t pca9555_input_mask = 0x1FFF;
static const uint16_t pca9555_led_bits[2] = { (1<<13), (1<<14) };
static const uint16_t pca9555_oled_bit = (1<<15);


/******************************************** TEST functions ***************************************************
static void test_display();
****************************************************************************************************************/

/*---------------------------------------------------------*/
static int64_t GetMicro( void ) {
    return esp_timer_get_time( );
}

static void pca9555_change_proc(uint16_t value)
{	
	/* proccess all buttons and encoders first */
	uint16_t xor = (m_last_pca9555_value ^ value) & pca9555_input_mask;

	for (int i=0; xor; i++, xor>>=1)
	{
		if ((xor & 0x1) && m_evt_callback_func)
		{
			//Check if control is encoder.
			if (pca9555_io_mapping[i] == EN_VOLUME_UP || pca9555_io_mapping[i] == EN_VOLUME_DOWN)
			{
				if (!!(value & (1<<i)) != !!(value & (1<<((i&1)?(i+1):(i-1)))))
					m_evt_callback_func(pca9555_io_mapping[i], EVT_STEP);
			}
			else
				/* TODO: add read procedure and add the call to calback to the main app */
				m_evt_callback_func(pca9555_io_mapping[i], ((value & (1<<i))?EVT_RELEASED:EVT_PRESSED));
		}
	}

	m_last_pca9555_value = value;
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_get_level(gpio_num) == 0)
    	xQueueSendFromISR(m_gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_loop(void* arg)
{
    uint32_t io_num;
    
    while (m_active) {
        if (xQueueReceive(m_gpio_evt_queue, &io_num, (1000 / portTICK_PERIOD_MS))) {

        	if (gpio_get_level(io_num) == 0)
        	{
        		uint16_t sr = PCA9555_get(&m_pca9555_I2C);
            	ESP_LOGW(TAG, "GPIO[%d] intr, val: %d sr=0x%04x\n", io_num, gpio_get_level(io_num), sr);

            	pca9555_change_proc(sr);
            }
        }

#ifdef USE_ADS111X
        else
        {
        	int16_t value = 0;
			ads111x_get_value(&m_ads111x_I2C, &value);
            ESP_LOGW(TAG, "Analog, input on AI0 val: %d ", value);
            ads111x_start_conversion(&m_ads111x_I2C);
        }
#endif

    }

    vTaskDelete(NULL);
}

#define GPIO36  36
esp_err_t init_ctrl_board()
{
	esp_err_t ret = ESP_OK;
    gpio_config_t io_conf;
    uint8_t tmp;

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (GPIO_INPUT_PIN_SEL | (1ULL << GPIO36));
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    
    if (ESP_OK != (ret = gpio_config(&io_conf)))
    	return ret;

    //create a queue to handle gpio event from isr
    if (NULL == (m_gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t))))
    	return ESP_FAIL;
    
    //start gpio task
    m_active = true;
    xTaskCreate(gpio_task_loop, "gpio_task_loop", 4096, NULL, 10, NULL);

    //install gpio isr service (already installed by other service)
    //if (ESP_OK != (ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT)))
    //	return ret;

    if ( ESP32_InitI2CMaster( GPIO_SDA_PIN, GPIO_SCL_PIN ) ) 
    {
        if ( SSD1306_Init_I2C( &m_Dev_I2C, 128, 64, 0x3C, 0, ESP32_WriteCommand_I2C, ESP32_WriteData_I2C, NULL ) == 1 ) 
        {
        	ESP_LOGW(TAG, "* Found SSD1306 display"); 

            SSD1306_SetFont( &m_Dev_I2C, /*&Font_Liberation_Sans_15x16*/&Font_Liberation_Serif_19x19/*&Font_Comic_Neue_25x28*/);

        	//test_display();
        	//test_display_song();
            //FontDrawAnchoredString( &m_Dev_I2C, "Smile!", TextAnchor_Center, true );
            //SSD1306_Update( &m_Dev_I2C );
        }
        else
        {
     		ESP_LOGW(TAG, "* SSD1306_Init_I2C init failed"); 
        }
    }
    else
    {
    	ESP_LOGW(TAG, "* Master I2C init failed"); 
    	m_active = false;
    	return ESP_FAIL;
    }

#ifdef USE_ADS111X
	/* Initialize ADC */
	m_ads111x_I2C.addr = ADS111X_ADDR_GND;
	if (ads111x_set_input_mux(&m_ads111x_I2C, ADS111X_MUX_0_GND) == ESP_OK)
	{
		int16_t value = 0;

		ads111x_set_mode(&m_ads111x_I2C, ADS111X_MODE_SINGLE_SHOT);
		ads111x_set_gain(&m_ads111x_I2C, ADS111X_GAIN_4V096);
		ads111x_set_data_rate(&m_ads111x_I2C, ADS111X_DATA_RATE_128);
		ads111x_set_comp_mode(&m_ads111x_I2C, ADS111X_COMP_MODE_NORMAL);
		ads111x_set_comp_polarity(&m_ads111x_I2C, ADS111X_COMP_POLARITY_LOW);
		ads111x_set_comp_latch(&m_ads111x_I2C, ADS111X_COMP_LATCH_ENABLED);
		ads111x_set_comp_queue(&m_ads111x_I2C, ADS111X_COMP_QUEUE_1/*ADS111X_COMP_QUEUE_DISABLED*/);
		// User Alert/RDY as RDY interrupt
		ads111x_set_comp_low_thresh(&m_ads111x_I2C, 0x0000);
		ads111x_set_comp_high_thresh(&m_ads111x_I2C, 0x8000);
		//Start conversation
		ads111x_start_conversion(&m_ads111x_I2C);
		vTaskDelay(20);
		ads111x_get_value(&m_ads111x_I2C, &value);
		ESP_LOGW(TAG, "* Found ADS111X device and read %d ", value); 
		ads111x_start_conversion(&m_ads111x_I2C);
	}
#endif	

	m_pca9555_I2C.addr = (PCA9555_DEVICE_ADDRESS | PCA9555_DEV_000);
	if ( i2c_dev_read_reg(&m_pca9555_I2C, 0, &tmp, 1) == ESP_OK )
	{
		ESP_LOGW(TAG, "* Found PCA9555 device and read %d ", tmp); 
		PCA9555_set(&m_pca9555_I2C, (pca9555_led_bits[0] | pca9555_led_bits[1]));
		PCA9555_dir(&m_pca9555_I2C, ~(pca9555_led_bits[0] | pca9555_led_bits[1] | pca9555_oled_bit));
		m_last_pca9555_value = PCA9555_get(&m_pca9555_I2C);

	    //hook isr handler for specific gpio pin
	    if (ESP_OK != gpio_isr_handler_add(GPIO_INT_IO_PIN, gpio_isr_handler, (void*) GPIO_INT_IO_PIN) )
	    {
	    	m_active = false;
	    	return ESP_FAIL;
	    }
	}

    return ESP_OK;
}

void clear_ctrl_board()
{
	m_active = false;
}

/* Control board initialization */
void registrate_cb(button_cb_t cb)
{
	m_evt_callback_func = cb;
}

/* Control LED on board */
void set_led(CTRL_BUTTON_E led, bool bOn)
{
	static uint16_t lastValue = 0;
	if ( (led-OUT_LED_GREEN) < (sizeof(pca9555_led_bits)/sizeof(pca9555_led_bits[0])) )
	{
		uint16_t bit = pca9555_led_bits[(led-OUT_LED_GREEN)];

		lastValue = (lastValue & ~(bit)) | (bOn?0:bit);  
		PCA9555_set(&m_pca9555_I2C, lastValue); 
	}
}

/******************************* Display *************************/
static void show_display_volume(player_state_t * state)
{
	char txtMsg[16];
	int  vol = (state->volume.play_vol_selected? state->volume.play_volume: state->volume.rec_volume);
	sprintf(txtMsg, "%s vol (%d)", (state->volume.play_vol_selected?"Play":"Rec"), vol);

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);
	
	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, txtMsg, TextAnchor_North, true);

	/* Draw horizontal line */
	SSD1306_DrawHLine(&m_Dev_I2C, 10, 41, 110, true);
	SSD1306_DrawHLine(&m_Dev_I2C, 10, 40, 10+vol, true);
	SSD1306_DrawHLine(&m_Dev_I2C, 10, 42, 10+vol, true);

	/* Draw vertical bar */
	SSD1306_DrawVLine(&m_Dev_I2C, 10+vol,   40-4, 42+4, true);
	SSD1306_DrawVLine(&m_Dev_I2C, 10+vol+1, 40-4, 42+4, true);

	SSD1306_Update(&m_Dev_I2C);
}


static const uint8_t track_matix[19][8] = { 
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x03, 0x80, 0x7C, 0x03, 0xE0, 0x03, 0x81, 0xFE},
	{0x03, 0x80, 0xFF, 0x0F, 0xF0, 0x07, 0x81, 0xFE},
	{0x07, 0x81, 0xFF, 0x0F, 0xF8, 0x07, 0x83, 0xFE},
	{0x1F, 0x83, 0xC7, 0x9E, 0x38, 0x0F, 0x83, 0x80},
	{0x1F, 0x83, 0x83, 0x9C, 0x38, 0x1F, 0x83, 0x80},
	{0x1B, 0x83, 0x83, 0x80, 0x78, 0x1F, 0x83, 0xF8},
	{0x03, 0x80, 0x03, 0x81, 0xF0, 0x3B, 0x83, 0xFC},
	{0x03, 0x80, 0x07, 0x01, 0xE0, 0x73, 0x87, 0xFE},
	{0x03, 0x80, 0x0F, 0x01, 0xF8, 0x73, 0x87, 0x0F},
	{0x03, 0x80, 0x1E, 0x00, 0x3C, 0xE3, 0x80, 0x07},
	{0x03, 0x80, 0x3C, 0x00, 0x1C, 0xC3, 0x80, 0x07},
	{0x03, 0x80, 0x78, 0x1C, 0x1D, 0xFF, 0xE7, 0x07},
	{0x03, 0x80, 0xF0, 0x1C, 0x1D, 0xFF, 0xE7, 0x07},
	{0x03, 0x81, 0xE0, 0x1E, 0x3D, 0xFF, 0xE7, 0x8E},
	{0x03, 0x81, 0xFF, 0x8F, 0xF8, 0x03, 0x83, 0xFE},
	{0x03, 0x83, 0xFF, 0x87, 0xF0, 0x03, 0x81, 0xFC},
	{0x03, 0x83, 0xFF, 0x83, 0xE0, 0x03, 0x80, 0xF8},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

static const uint8_t delimeters[] = { 2, 12, 26, 38, 52, 64 }; 

static void draw_track(struct SSD1306_Device * DeviceHandle, int track_num, bool bSelected, int offs, bool bcursor)
{
	uint32_t x = delimeters[track_num]-2 + offs, y = (DeviceHandle->Height - 19);
	uint32_t offset = DeviceHandle->Width * (y/8) + x;
	uint8_t *display = DeviceHandle->Framebuffer + offset;
	uint8_t display_bit, matrix_bit, *ptr;
	bool bPixel; 

	for (int c=delimeters[track_num]; c<delimeters[track_num+1]; c++, display++)
	{
		display_bit = (1 << (y&7));
		matrix_bit  = (0x80 >> (c & 7));
		ptr = display;  

		for (int r=y; r<(y+19); r++)
		{
			bPixel = !!(track_matix[r-y][c/8] & matrix_bit);
			if (bPixel ^ bSelected) 
				*ptr |= display_bit;
			if (!(display_bit <<= 1))
			{
				display_bit = 1;
				ptr += DeviceHandle->Width;
			}
		}
	}

	if (bcursor)
	{
		SSD1306_DrawEmptyRect(DeviceHandle, x, y, x+delimeters[track_num+1]-2, (y+18), !bSelected);
	}
}

static void show_display_song(player_state_t * state)
{
	static const char * state_play_msg[] = { "Stop", "Play", "Pause" };
	static const char * state_rec_msg[] = { "Stop", "Rec", "Pause" };
	char txtMsg[16];
	
	if (!state || !state->song || !m_active || m_Dev_I2C.Width < 128)
		return;

	sprintf(txtMsg, "Song_%d", state->song->num);

	int64_t start = GetMicro( );

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);
	
	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, txtMsg, TextAnchor_North, true);

	/* Draw vertical bar */
	SSD1306_DrawVLine(&m_Dev_I2C, m_Dev_I2C.Width/2, 20, m_Dev_I2C.Height-1, true);
	
	/* drow state line */
	FontDrawAnchoredString(&m_Dev_I2C, state_play_msg[state->play_state % 3], TextAnchor_West, true);
	FontDrawAnchoredString(&m_Dev_I2C, state_rec_msg[state->rec_state % 3], TextAnchor_East, true);

	for (int i=0; i<MAX_TRACKS; i++)
	{
		if (state->song->tracks[i].len_in_sec > 0)
			draw_track(&m_Dev_I2C, i, state->play_selected_tracks & (1<<i), 0, (state->cursor==i));
		draw_track(&m_Dev_I2C, i, ((i+1)==state->rec_selected_track), m_Dev_I2C.Width/2+1, (state->cursor==(i+MAX_TRACKS)));
	}
	
	SSD1306_Update(&m_Dev_I2C);

	int64_t stop = GetMicro();
	ESP_LOGW(TAG, "* It take %d usec to draw song!!!", (int)(stop - start)); 
}

static void show_display_equlizer(player_state_t * state)
{
	static const char * bands_msg[MAX_EQ_BANDS] = { "60", /*"170",*/ "310", /*"600",*/ "1k", "3k", /*"6k",*/ "12k", /*"14k",*/ "16k" };

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	SSD1306_SetFont(&m_Dev_I2C, &Font_Ubuntu_Mono_6x10);

	int x = 11, ytop=11, ybottom=m_Dev_I2C.Height-14;
	int step = (m_Dev_I2C.Width-x*2)/(MAX_EQ_BANDS-1);

	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, "Equalizer", TextAnchor_North, true);

	for (int i=0; i<MAX_EQ_BANDS; i++, x+=step)
	{
		/* Draw vertical bar */
		SSD1306_DrawVLine(&m_Dev_I2C, x, ytop, ybottom, true);
		
		int h = ((ybottom-ytop)*(state->equalizer.bands[i]+50))/100;
		SSD1306_DrawVLine(&m_Dev_I2C, x-1, ybottom-h, ybottom, true);
		SSD1306_DrawVLine(&m_Dev_I2C, x+1, ybottom-h, ybottom, true);

		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+0, x+5, true);
		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+1, x+5, true);

		if (state->equalizer.cursor == i)
		{
			SSD1306_DrawEmptyRect(&m_Dev_I2C, x-8, ytop-2, x+8, ybottom+1, true);
		}

		int xstart = (x - FontMeasureString(&Font_Ubuntu_Mono_6x10, bands_msg[i])/2);
		if (xstart < 0)
			xstart = 0;
		FontDrawStringUnaligned(&m_Dev_I2C, bands_msg[i], xstart, ybottom+3, true);
	}

	SSD1306_Update(&m_Dev_I2C);

	/* Return back right font selection */
	SSD1306_SetFont(&m_Dev_I2C, &Font_Liberation_Serif_19x19);
}

void show_display_recoptions(player_state_t * state)
{
	char msg_s[32], msg_m[32];
	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, "Rec options", TextAnchor_North, true);

	sprintf(msg_s, "Source: %s", (state->rec_opt.rec_source == SRC_LINEIN)? "line-in": "mic");
	FontDrawStringUnaligned(&m_Dev_I2C, msg_s, 1, 20, true);
	sprintf(msg_m, "Monitor: %s", (state->rec_opt.bmonitor)? "on": "off");
	FontDrawStringUnaligned(&m_Dev_I2C, msg_m, 1, 40, true);

	int y = (state->rec_opt.cursor==0)? 20: 40;
	int x = FontMeasureString(&Font_Liberation_Serif_19x19, ((state->rec_opt.cursor==0)?msg_s:msg_m));
	SSD1306_DrawEmptyRect(&m_Dev_I2C, 0, y, x+1, y+19, true);

	SSD1306_Update(&m_Dev_I2C);
}


void display_player_state(player_state_t * state)
{
	if (!state )
		return;

	switch (state->display)
	{
		case D_SONG:		show_display_song(state);       break;
		case D_VOLUME:		show_display_volume(state);     break;
		case D_EQUALIZER:   show_display_equlizer(state);   break;
		case D_REC_OPT:     show_display_recoptions(state); break;
		default:
		{
			ESP_LOGW(TAG, "* Display %d not implemented yet !!!", state->display);
		}
	}
}



/******************************************** TEST functions ***************************************************
static void test_display()
{
	song_t song = { .num=1, .tracks={{1,10},{2,10},{3,10},{4,10},{5,10}} };
	player_state_t state = { .song=&song, .play_selected_tracks=0x5, .rec_selected_track=2,
	                         .cursor=0, .play_state=P_PLAYING, .rec_state=R_RECORDING, 
	                         .rec_opt={SRC_LINEIN, true, 1},
	                         .equalizer={{20,10,-4,-20,-10,40}},
	                         .volume={50, 50, true, 0},
	                         .display=D_REC_OPT };

	display_player_state(&state);
}
****************************************************************************************************************/



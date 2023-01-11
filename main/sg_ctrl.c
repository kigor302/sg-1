#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <esp_timer.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include "driver/adc.h"
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
#include "esp_adc_cal.h"



/* Local variables */
static xQueueHandle m_gpio_evt_queue = NULL;
static button_cb_t  m_evt_callback_func = NULL;
static button2_cb_t m_evt_callback2_func = NULL;
static pots_cb_t	m_pots_callback_func = NULL;
static struct SSD1306_Device m_Dev_I2C;
static i2c_dev_t m_pca9555_I2C, m_pca9555_I2C_2;
static uint16_t  m_last_pca9555_value, m_last_pca9555_value_2;

#ifdef USE_ADS111X
	static i2c_dev_t m_ads111x_I2C;
#endif

static const char *TAG = "SG-CTRL";
static bool m_active = false;

//static const uint8_t pca9555_io2_mapping[16] = { LVL_6,  LVL_5,   LVL_4,    LVL_3,      LVL_2,   LVL_1,   RESERVED_A, RESERVED_B, 
										 	    /*pin0*/  /*pin1*/  /*pin2*/  /*pin3*/  /*pin4*/  /*pin5*/  /*pin6*/      /*pin7*/
//                                                IO1_0,      IO1_1,   IO1_2,    IO1_3,     IO1_4,      IO1_5,      IO1_6,    IO1_7 };
                                                /*pin8*/  /*pin9*/  /*pin10*/  /*Pin11*/  /*pin12*/  /*pin13*/   /*pin14*/   /*pin16*/
//static uint32_t pca9555_io2_pressed_time = 0;
//static int pca9555_io2_pressed_key = -1;
static const uint16_t pca9555_input2_mask = 0xFEFF; //0x072F;
//static uint32_t pca9555_io2_trigger_ticks[3];


static const uint8_t pca9555_io_mapping[16] = { ENC1_SW,    ENC1_CW,     ENC1_CCW,      BT_REC,    BT_STOP,   BT_PLAY,   PLAY_LED,   REC_LED,
										 	    /*pin0*/    /*pin1(A)*/   /*pin2(B)*/    /*pin3*/  /*pin4*/    /*pin5*/    /*pin6*/   /*pin7*/
                                                POTS_EN,    BRG_A,  BRG_B,    BRG_INH,   STOP_LED,   BT_FORWARD/*OUT_LED_GREEN*/, BT_REWARD/*OUT_LED_RED*/, OUT_OLED };
                                                /*pin8*/  /*pin9*/ /*pin10*/  /*Pin11*/ /*pin12*/  /*pin13*/      /*pin14*/    /*pin16*/
static uint32_t pca9555_io_pressed_time = 0;
static int pca9555_io_pressed_key = -1;


static const uint16_t pca9555_input_mask = 0x603F; //0x1FFF;
static const uint16_t pca9555_led_bits[3] = { (1<<PLAY_LED), (1<<REC_LED), (1<<STOP_LED) };
static const uint16_t pca9555_oled_bit = (1<<OUT_OLED);


/******************************************** TEST functions ***************************************************
static void test_display();
****************************************************************************************************************/
static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGW(TAG,"eFuse Two Point: Supported");
    } else {
        ESP_LOGW(TAG,"eFuse Two Point: NOT supported");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGW(TAG,"eFuse Vref: Supported");
    } else {        ESP_LOGW(TAG,"eFuse Vref: NOT supported");
    }
}

#define NO_OF_SAMPLES 16
static uint32_t getAdcValue()
{
	uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)ADC1_CHANNEL_0);
    }
    adc_reading /= NO_OF_SAMPLES;
    //ESP_LOGW(TAG,"Raw adc is: %d\n", adc_reading);
    return adc_reading;
}


/*---------------------------------------------------------*
static int64_t GetMicro( void ) {
    return esp_timer_get_time( );
}
*----------------------------------------------------------*/
static void pca9555_change_proc(uint32_t io_num, uint16_t value)
{	
	if (io_num == GPIO_INT_IO_PIN)
	{
		/* proccess all buttons and encoders first */
		uint16_t xor = (m_last_pca9555_value ^ value) & pca9555_input_mask;
		m_last_pca9555_value = value;

		for (int i=0; xor; i++, xor>>=1)
		{
			if ((xor & 0x1) && m_evt_callback_func)
			{
				//Check if control is encoder.
				//if (pca9555_io_mapping[i] == EN_VOLUME_UP || pca9555_io_mapping[i] == EN_VOLUME_DOWN)
				//{
				//	if (!!(value & (1<<i)) != !!(value & (1<<((i&1)?(i+1):(i-1)))))
				//		m_evt_callback_func(pca9555_io_mapping[i], EVT_STEP);
				//	pca9555_io_pressed_key = -1;
				//}
				//else
				//Check if control is encoder.
				if (pca9555_io_mapping[i] == ENC1_CW || pca9555_io_mapping[i] == ENC1_CCW)
				{
					if (!!(value & (1<<i)) != !!(value & (1<<((i&1)?(i+1):(i-1)))))
						m_evt_callback_func(pca9555_io_mapping[i], EVT_STEP);
					pca9555_io_pressed_key = -1;
				}
				else
				{
					bool bRelease = (value & (1<<i));
					EVT_BUTTON_E evt = EVT_PRESSED;
					if (bRelease)
					{
						evt = (pca9555_io_pressed_key == i && ((xTaskGetTickCount() - pca9555_io_pressed_time)*portTICK_PERIOD_MS) > 1000)? 
							  EVT_LONGPRESS : EVT_RELEASED;
						pca9555_io_pressed_key = -1;
					}
					else
					{
						pca9555_io_pressed_key = i;
						pca9555_io_pressed_time = xTaskGetTickCount();
					}
					/* TODO: add read procedure and add the call to calback to the main app */
					m_evt_callback_func(pca9555_io_mapping[i], evt);
				}
			}
		}
	}
	/*
	else if (io_num == GPIO_INT2_IO_PIN)
	{
		// proccess all buttons and encoders first 
		uint16_t xor = (m_last_pca9555_value_2 ^ value) & pca9555_input2_mask;

		for (int i=0; xor; i++, xor>>=1)
		{
			if ((xor & 0x1) && m_evt_callback2_func)
			{
				//Check if control is encoder.
				if (pca9555_io2_mapping[i] == ENC1_CW || pca9555_io2_mapping[i] == ENC1_CCW)
				{
					if (!!(value & (1<<i)) != !!(value & (1<<((i&1)?(i+1):(i-1)))))
						m_evt_callback2_func(pca9555_io2_mapping[i], EVT_STEP);
					pca9555_io2_pressed_key = -1;
				}
				//Check if control is PWM.
				if (pca9555_io2_mapping[i] == PWM_0 || pca9555_io2_mapping[i] == PWM_1 || pca9555_io2_mapping[i] == PWM_2)
				{
					int idx = 	(pca9555_io2_mapping[i] == PWM_0)?0:
								(pca9555_io2_mapping[i] == PWM_1)?1:
								2;

					if (value & (1<<i))
						pca9555_io2_trigger_ticks[idx] = xTaskGetTickCount();
					else
					{
						int period = ((xTaskGetTickCount() - pca9555_io2_trigger_ticks[idx]) * portTICK_PERIOD_MS);
						if (period <= 250)
							m_evt_callback2_func(pca9555_io2_mapping[i], (period*250)/100);
					}
					pca9555_io2_pressed_key = -1;
				}
				else
				{
					bool bRelease = (value & (1<<i));
					EVT_BUTTON_E evt = EVT_PRESSED;
					if (bRelease)
					{
						evt = (pca9555_io2_pressed_key == i && ((xTaskGetTickCount() - pca9555_io2_pressed_time)*portTICK_PERIOD_MS) > 1000)? 
							  EVT_LONGPRESS : EVT_RELEASED;
						pca9555_io2_pressed_key = -1;
					}
					else
					{
						pca9555_io2_pressed_key = i;
						pca9555_io2_pressed_time = xTaskGetTickCount();
					}
					// TODO: add read procedure and add the call to calback to the main app 
					m_evt_callback2_func(pca9555_io2_mapping[i], evt);
				}
			}
		}

		m_last_pca9555_value = value;
	}
	*/
	
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if (gpio_get_level(gpio_num) == 0)
    	xQueueSendFromISR(m_gpio_evt_queue, &gpio_num, NULL);
}


static int pots_values[MAX_POTS] = {0};
static POTS_E cur_pot = POTS_PLAY;
static int cur_pot_ticks = 0;
static void gpio_task_loop(void* arg)
{
    uint32_t io_num;
    
    while (m_active) {
        if (xQueueReceive(m_gpio_evt_queue, &io_num, (100 / portTICK_PERIOD_MS))) 
        {
        	if (gpio_get_level(io_num) == 0)
        	{
        		uint16_t sr = PCA9555_get((io_num==GPIO_INT_IO_PIN)?&m_pca9555_I2C:&m_pca9555_I2C_2);
            	ESP_LOGI(TAG, "GPIO[%d] intr, sr=0x%04x\n", io_num, sr);
            	pca9555_change_proc(io_num, sr);
            	continue;
            }
        }
        else if (gpio_get_level(GPIO_INT_IO_PIN) == 0)
        {
        	 uint16_t sr = PCA9555_get(&m_pca9555_I2C);
        	 ESP_LOGW(TAG, "GPIO[%d] intr, val: %d sr=0x%04x\n", GPIO_INT_IO_PIN, gpio_get_level(GPIO_INT_IO_PIN), sr);
        	 continue;
        }

        //Get analog voltage from POTS
        int val = (int)getAdcValue();
        int diff = abs(pots_values[cur_pot] - val);
        if (diff >= 100) {

        	ESP_LOGW(TAG, "cur_pot[%d] val: %d diff: %d\n", cur_pot, val, diff);

        	m_pots_callback_func(cur_pot, (val + 25)/41);
        	cur_pot_ticks =8;
        	pots_values[cur_pot] = val;
        }
        else if (cur_pot_ticks == 2) {
        	cur_pot_ticks = 1;
        	m_pots_callback_func(MAX_POTS, 0);
        } 
        if (--cur_pot_ticks <= 0) {
        	cur_pot = (cur_pot + 1) % MAX_POTS;
        	cur_pot_ticks = 1;

			uint16_t bits = ((cur_pot & 1)?(1<<BRG_A):0) | ((cur_pot & 2)?(1<<BRG_B):0);
			m_last_pca9555_value = (m_last_pca9555_value & ~((1<<BRG_A)|(1<<BRG_B))) | bits;  
			PCA9555_set(&m_pca9555_I2C, m_last_pca9555_value); 
        }
        //else if (gpio_get_level(GPIO_INT2_IO_PIN) == 0)
        //{
        //	 uint16_t sr = PCA9555_get(&m_pca9555_I2C_2);
        //	 ESP_LOGW(TAG, "GPIO[%d] intr, val: %d sr=0x%04x\n", GPIO_INT2_IO_PIN, gpio_get_level(GPIO_INT2_IO_PIN), sr);
        //}

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


int ResetSSDDisplay (struct SSD1306_Device* DeviceHandle)
{
	ESP_LOGW(TAG, "SSD1306 display reset");
	set_gpio_out_1(IO1_0, false);
	vTaskDelay(10);
	set_gpio_out_1(IO1_0, true);
	vTaskDelay(10);
	return true;
}


#define GPIO36  36
esp_err_t init_ctrl_board()
{
	esp_err_t ret = ESP_OK;
    uint8_t tmp;

    gpio_config_t io_conf;
    //interrupt of falling edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = (GPIO_INPUT_PIN_SEL /*| (1ULL << GPIO36)*/);
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    if (ESP_OK != (ret = gpio_config(&io_conf)))
    {
    	return ret;
    }

	//esp_log_level_set(TAG, ESP_LOG_INFO);

    //create a queue to handle gpio event from isr
    if (NULL == (m_gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t))))
    	return ESP_FAIL;
    
    //start gpio task
    m_active = true;
    xTaskCreatePinnedToCore(gpio_task_loop, "gpio_task_loop", 4096, NULL, 10, NULL, 1);

    //install gpio isr service (already installed by other service)
    //if (ESP_OK != (ret = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT)))
    //	return ret;

    if ( ESP32_InitI2CMaster( GPIO_SDA_PIN, GPIO_SCL_PIN ) ) 
    {
		m_pca9555_I2C.addr = (PCA9555_DEVICE_ADDRESS | PCA9555_DEV_000);
		if ( i2c_dev_read_reg(&m_pca9555_I2C, 0, &tmp, 1) == ESP_OK )
		{
			ESP_LOGW(TAG, "* Found PCA9555 first device and read %d ", tmp); 
			PCA9555_set(&m_pca9555_I2C, (pca9555_led_bits[0] | pca9555_led_bits[1] | pca9555_led_bits[2] | pca9555_oled_bit));
			PCA9555_dir(&m_pca9555_I2C, pca9555_input_mask);
			m_last_pca9555_value = PCA9555_get(&m_pca9555_I2C);

		    //hook isr handler for specific gpio pin
		    if (ESP_OK != gpio_isr_handler_add(GPIO_INT_IO_PIN, gpio_isr_handler, (void*) GPIO_INT_IO_PIN) )
		    {
		    	m_active = false;
		    	return ESP_FAIL;
		    }
		}

		m_pca9555_I2C_2.addr = (PCA9555_DEVICE2_ADDRESS | PCA9555_DEV_000);
		if ( i2c_dev_read_reg(&m_pca9555_I2C_2, 0, &tmp, 1) == ESP_OK )
		{
			ESP_LOGW(TAG, "* Found PCA9555 second device and read %d ", tmp); 
			PCA9555_set(&m_pca9555_I2C_2, (uint16_t)~pca9555_input2_mask /*& ~(1<<PWM_EN)*/);
			PCA9555_dir(&m_pca9555_I2C_2, pca9555_input2_mask);
			m_last_pca9555_value_2 = PCA9555_get(&m_pca9555_I2C_2);

		    //hook isr handler for specific gpio pin (no interrupt for second extender)
		    //if (ESP_OK != gpio_isr_handler_add(GPIO_INT2_IO_PIN, gpio_isr_handler, (void*)GPIO_INT2_IO_PIN) )
		    //{
		    //	m_active = false;
		    //	return ESP_FAIL;
		    //}
		}

        if ( SSD1306_Init_I2C( &m_Dev_I2C, 128, 64, 0x3C, 0, ESP32_WriteCommand_I2C, ESP32_WriteData_I2C, ResetSSDDisplay) == 1 ) 
        {
        	ESP_LOGW(TAG, "* Found SSD1306 display"); 

            SSD1306_SetFont( &m_Dev_I2C, /*&Font_Liberation_Sans_15x16*/&Font_Liberation_Serif_19x19/*&Font_Comic_Neue_25x28*/);

        	//test_display();
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

    set_gpio_out_0(POTS_EN, true);
    set_gpio_out_0(BRG_A, false);
    set_gpio_out_0(BRG_B, false);
    set_gpio_out_0(BRG_INH, false);

	check_efuse();
    adc_gpio_init(ADC_UNIT_1, ADC1_CHANNEL_0);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    return ESP_OK;
}

void clear_ctrl_board()
{
	m_active = false;
}

/* Control board initialization */
void registrate_cb(button_cb_t cb, button2_cb_t cb2, pots_cb_t cb3)
{
	m_evt_callback_func = cb;
	m_evt_callback2_func = cb2;
	m_pots_callback_func = cb3;
}

/* Control LED on board */
void set_led(CTRL_BUTTON_E led, bool bOn)
{
	uint16_t lastValue = m_last_pca9555_value; //PCA9555_get(&m_pca9555_I2C);;
	if ( led-OUT_LED_GREEN < (sizeof(pca9555_led_bits)/sizeof(pca9555_led_bits[0])) )
	{
		uint16_t bit = pca9555_led_bits[led-OUT_LED_GREEN];

		m_last_pca9555_value = lastValue = (lastValue & ~(bit)) | (bOn?0:bit);  
		PCA9555_set(&m_pca9555_I2C, lastValue); 
	}
}


/* Control second board outputs */
void set_gpio_out_1(CTRL2_BUTTON_E io, bool bOn)
{
	if ( (1<<io) & (~pca9555_input2_mask) )
	{
		uint16_t lastValue = PCA9555_get(&m_pca9555_I2C_2);
		uint16_t bit = (1<<io);

		lastValue = (lastValue & ~(bit)) | (bOn?bit:0);  
		PCA9555_set(&m_pca9555_I2C_2, lastValue); 
	}
}

/* Control second board outputs */
void set_gpio_out_0(CTRL_BUTTON_E io, bool bOn)
{
	if ( (1<<io) & (~pca9555_input_mask) )
	{
		uint16_t lastValue = m_last_pca9555_value; //PCA9555_get(&m_pca9555_I2C);
		uint16_t bit = (1<<io);

		m_last_pca9555_value = lastValue = (lastValue & ~(bit)) | (bOn?bit:0);  
		PCA9555_set(&m_pca9555_I2C, lastValue); 
	}
}


/******************************* Display *************************/
static void show_display_volume(player_state_t * state)
{
	static const char * bands_msg[MAX_VOL_BANDS] = { "dac", "adc", "mL", "mR" /*, "hp", "spk", "boost", "aux"*/ };
	int page_offset = (state->volume.cursor >= (MAX_VOL_BANDS/1/*2*/))? MAX_VOL_BANDS/1/*2*/ : 0;

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	SSD1306_SetFont(&m_Dev_I2C, &Font_Ubuntu_Mono_6x10);

	int x = 11, ytop=11, ybottom=m_Dev_I2C.Height-14;
	int step = (m_Dev_I2C.Width-x*2)/(MAX_VOL_BANDS/1/*2*/-1);

	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, "Volume control", TextAnchor_North, true);

	for (int i=0; i<MAX_VOL_BANDS/1/*2*/; i++, x+=step)
	{
		/* Draw vertical bar */
		SSD1306_DrawVLine(&m_Dev_I2C, x, ytop, ybottom, true);
		
		int h = ((ybottom-ytop)*(state->volume.bands[i+page_offset]))/100;
		SSD1306_DrawVLine(&m_Dev_I2C, x-1, ybottom-h, ybottom, true);
		SSD1306_DrawVLine(&m_Dev_I2C, x+1, ybottom-h, ybottom, true);

		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+0, x+5, true);
		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+1, x+5, true);

		if (state->volume.cursor == (i+page_offset))
		{
			SSD1306_DrawEmptyRect(&m_Dev_I2C, x-8, ytop-2, x+8, ybottom+1, true);
		}

		int xstart = (x - FontMeasureString(&Font_Ubuntu_Mono_6x10, bands_msg[i+page_offset])/2);
		if (xstart < 0)
			xstart = 0;
		FontDrawStringUnaligned(&m_Dev_I2C, bands_msg[i+page_offset], xstart, ybottom+3, true);
	}

	SSD1306_Update(&m_Dev_I2C);

	/* Return back right font selection */
	SSD1306_SetFont(&m_Dev_I2C, &Font_Liberation_Serif_19x19);
}

static uint8_t fonts_12x8[5][18] = {
/* 1 */{0x00, 0x0C, 0x0C, 0xFE, 0xFE, 0x00, 0x00, 0x00, 0, 0x0, 0x6, 0x6, 0x7, 0x7, 0x6, 0x6, 0x0, 0}, 
/* 2 */{0x00, 0x0C, 0x06, 0x86, 0xC6, 0x66, 0x3E, 0x1C, 0, 0x0, 0x6, 0x7, 0x7, 0x6, 0x6, 0x6, 0x6, 0},
/* 3 */{0x00, 0x0C, 0x66, 0x66, 0x66, 0x66, 0xFE, 0xDC, 0, 0x0, 0x3, 0x6, 0x6, 0x6, 0x6, 0x7, 0x3, 0},
/* 4 */{0x00, 0xC0, 0xF0, 0xBC, 0x8E, 0xFE, 0xFE, 0x80, 0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x7, 0x7, 0x1, 0},
/* 5 */{0x00, 0x7E, 0x7E, 0x66, 0x66, 0x66, 0xE6, 0xC6, 0, 0x0, 0x3, 0x6, 0x6, 0x6, 0x6, 0x7, 0x3, 0}
};

static void draw_track(struct SSD1306_Device * DeviceHandle, int track_num, bool bSelected, int offs, bool bcursor)
{
	uint32_t x = 2 + (13 * track_num) + offs, y = (DeviceHandle->Height - 16);
	uint32_t offset = DeviceHandle->Width * (y/8) + x;
	uint8_t *display = DeviceHandle->Framebuffer + offset;
	
	/* Print track align with 16-bit */	
	for (int i=0; i<9; i++)
		display[i] = bSelected? (0xFF ^ fonts_12x8[track_num][i]): fonts_12x8[track_num][i];
	display += DeviceHandle->Width;
	for (int i=0; i<9; i++)
		display[i] = bSelected? (0x0F ^ fonts_12x8[track_num][9+i]): (fonts_12x8[track_num][9+i] & 0x0F);

	if (bcursor)
	{
		SSD1306_DrawEmptyRect(DeviceHandle, x-2, y-2, x+8+2, (y+12+2), true);
	}
}


static void show_display_song(player_state_t * state)
{
	static const char * state_play_msg[] = { "Stop", "Play", "Pause" };
	static const char * state_rec_msg[] = { "Stop", "Rec", "Pause" };
	char txtMsg[64];
	
	if (!state || !state->song || !m_active || m_Dev_I2C.Width < 128)
		return;

	sprintf(txtMsg, "Song_%d", state->song->num+1);

	//int64_t start = GetMicro( );

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	//Display off when recording
	//if ((state->rec_state % 3)==1)
	//{
	//	SSD1306_DisplayOff(&m_Dev_I2C);
	//	return;
	//}
	//else
	//	SSD1306_DisplayOn(&m_Dev_I2C);
	
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
			draw_track(&m_Dev_I2C, i, state->play_selected_tracks & (1<<i), 0, (state->song->cursor==i));
		draw_track(&m_Dev_I2C, i, ((i+1)==state->rec_selected_track), m_Dev_I2C.Width/2 + 1, (state->song->cursor==(i+MAX_TRACKS)));
	}
	
	SSD1306_Update(&m_Dev_I2C);

	//int64_t stop = GetMicro();
	//ESP_LOGW(TAG, "* It take %d usec to draw song!!!", (int)(stop - start)); 
}

static void show_display_equlizer(player_state_t * state)
{
	static const char * bands_msg[MAX_EQ_BANDS] = { "31", "62", "125", "250", "500", "1k", "2k", "4k", "8k", "16k" };
	int page_offset = (state->equalizer.cursor >= (MAX_EQ_BANDS/2))? MAX_EQ_BANDS/2 : 0;

	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	SSD1306_SetFont(&m_Dev_I2C, &Font_Ubuntu_Mono_6x10);

	int x = 11, ytop=11, ybottom=m_Dev_I2C.Height-14;
	int step = (m_Dev_I2C.Width-x*2)/(MAX_EQ_BANDS/2-1);

	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, "Equalizer", TextAnchor_North, true);

	for (int i=0; i<MAX_EQ_BANDS/2; i++, x+=step)
	{
		/* Draw vertical bar */
		SSD1306_DrawVLine(&m_Dev_I2C, x, ytop, ybottom, true);
		
		int h = ((ybottom-ytop)*(state->equalizer.bands[i+page_offset]+25))/50;
		SSD1306_DrawVLine(&m_Dev_I2C, x-1, ybottom-h, ybottom, true);
		SSD1306_DrawVLine(&m_Dev_I2C, x+1, ybottom-h, ybottom, true);

		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+0, x+5, true);
		SSD1306_DrawHLine(&m_Dev_I2C, x-5, ybottom-h+1, x+5, true);

		if (state->equalizer.cursor == (i+page_offset))
		{
			SSD1306_DrawEmptyRect(&m_Dev_I2C, x-8, ytop-2, x+8, ybottom+1, true);
		}

		int xstart = (x - FontMeasureString(&Font_Ubuntu_Mono_6x10, bands_msg[i+page_offset])/2);
		if (xstart < 0)
			xstart = 0;
		FontDrawStringUnaligned(&m_Dev_I2C, bands_msg[i+page_offset], xstart, ybottom+3, true);
	}

	SSD1306_Update(&m_Dev_I2C);

	/* Return back right font selection */
	SSD1306_SetFont(&m_Dev_I2C, &Font_Liberation_Serif_19x19);
}

void show_display_recoptions(player_state_t * state)
{
	int  y_offset = 0;
	char msg[MAX_REC_OPTONS][32];
	/* Clear screen */
	SSD1306_Clear(&m_Dev_I2C, false);

	SSD1306_SetFont(&m_Dev_I2C, &Font_Ubuntu_Mono_6x10);

	/* drow text line */
	FontDrawAnchoredString(&m_Dev_I2C, "Record options", TextAnchor_North, true);

	sprintf(msg[0], "Source: %s", (state->rec_opt.rec_source == SRC_LINEIN)? "line-in": "mic");
	FontDrawStringUnaligned(&m_Dev_I2C, msg[0], 1, (y_offset += 12), true);
	sprintf(msg[1], "Monitor: %s", (state->rec_opt.bmonitor)? "on": "off");
	FontDrawStringUnaligned(&m_Dev_I2C, msg[1], 1, (y_offset += 12), true);
	sprintf(msg[2], "Record mix: %s", (state->rec_opt.brecordmix)? "yes": "no");
	FontDrawStringUnaligned(&m_Dev_I2C, msg[2], 1, (y_offset += 12), true);
	sprintf(msg[3], "Save Configuration");
	FontDrawStringUnaligned(&m_Dev_I2C, msg[3], 1, (y_offset += 12), true);

	int y = 11 + (state->rec_opt.cursor * 12);
	int x = FontMeasureString(&Font_Ubuntu_Mono_6x10, msg[(state->rec_opt.cursor%MAX_REC_OPTONS)]);
	SSD1306_DrawEmptyRect(&m_Dev_I2C, 0, y-1, x+1, y+11, true);

	SSD1306_Update(&m_Dev_I2C);

	/* Return back right font selection */
	SSD1306_SetFont(&m_Dev_I2C, &Font_Liberation_Serif_19x19);
}

static void show_display_selectsong(player_state_t * state)
{
	char msg[32];

	sprintf(msg, "Select Song_%d", (state->song->num+1));

	SSD1306_Clear(&m_Dev_I2C, false);
	SSD1306_SetFont(&m_Dev_I2C, &Font_Ubuntu_Mono_6x10);

	FontDrawAnchoredString(&m_Dev_I2C, msg, TextAnchor_North, true);

	SSD1306_Update(&m_Dev_I2C);
}

void display_player_state(player_state_t * state)
{
	if (!state )
		return;

	switch (state->display)
	{
		case D_SONG:		show_display_song(state);       break;
		case D_SELECT_SONG: show_display_selectsong(state); break;
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



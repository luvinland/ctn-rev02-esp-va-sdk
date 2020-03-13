// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
// All rights reserved.

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_event_loop.h>
#include <esp_pm.h>
#include <nvs_flash.h>

#include <conn_mgr_prov.h>
#include <conn_mgr_prov_mode_ble.h>

#include <voice_assistant.h>
#include <alexa.h>
#include <alexa_local_config.h>

#include <va_mem_utils.h>
#include <scli.h>
#include <va_diag_cli.h>
#include <wifi_cli.h>
#include <media_hal.h>
#include <tone.h>
#include <avs_config.h>
#include <speech_recognizer.h>
#include "va_board.h"
#include "app_auth.h"
#include "app_wifi.h"

#ifdef CONFIG_ALEXA_ENABLE_EQUALIZER
#include "alexa_equalizer.h"
#endif

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#endif

#ifdef CONFIG_ALEXA_ENABLE_OTA
#include "app_ota.h"
#endif

#include "app_defs.h"

#if defined(BLYNK_APPS)
#include "blynk.h"
#endif

#if defined(FACTORY_RESET)
#include "va_nvs_utils.h"
#endif

#define SOFTAP_SSID_PREFIX  "ESP-Alexa-"

static const char *TAG = "[app_main]";

#if defined(BLYNK_APPS)
#include "driver/i2c.h"

#define POWEROFF	0x0
#define POWERON		0x1
#define STEP1		0x2
#define STEP2		0x3
#define STEP3		0x4
#if defined(BLYNK_I2C)
#define ESPTRIGGER	0x5
#define ESPIDLE		0x6
#endif

#define BLY_VENT_POWER 10
#define BLY_VENT_STEP1 11
#define BLY_VENT_STEP2 12
#define BLY_VENT_STEP3 13

extern bool vent_power_on;
extern uint8_t vent_step;

extern uint8_t	_binary_01_bin_start, _binary_01_bin_end, _binary_02_bin_start, _binary_02_bin_end, _binary_03_bin_start, _binary_03_bin_end,
				_binary_04_bin_start, _binary_04_bin_end, _binary_05_bin_start, _binary_05_bin_end;

#if defined(BLYNK_I2C)
void blynk_notify_i2c_i94124(uint8_t cmd)
{
	uint8_t power_off[4]			= {0xF0, 0xF1, 0xF2, 0xF3};
	uint8_t power_on[4]				= {0xE0, 0xE1, 0xE2, 0xE3};
	uint8_t step_one[4]				= {0x10, 0x11, 0x12, 0x13};
	uint8_t step_two[4]				= {0x20, 0x21, 0x22, 0x23};
	uint8_t step_three[4]			= {0x30, 0x31, 0x32, 0x33};
	uint8_t esp_trigger_detect[4]	= {0xA0, 0xA1, 0xA2, 0xA3};
	uint8_t esp_idle_state[4]		= {0xB0, 0xB1, 0xB2, 0xB3};

	switch(cmd)
	{
		case POWEROFF:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, power_off, 4, 1000 / portTICK_RATE_MS));
			break;
		case POWERON:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, power_on, 4, 1000 / portTICK_RATE_MS));
			break;
		case STEP1:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, step_one, 4, 1000 / portTICK_RATE_MS));
			break;
		case STEP2:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, step_two, 4, 1000 / portTICK_RATE_MS));
			break;
		case STEP3:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, step_three, 4, 1000 / portTICK_RATE_MS));
			break;
		case ESPTRIGGER:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, esp_trigger_detect, 4, 1000 / portTICK_RATE_MS));
			break;
		case ESPIDLE:
			ESP_LOGE(TAG, "Len (%d)", i2c_slave_write_buffer( I2C_NUM_0, esp_idle_state, 4, 1000 / portTICK_RATE_MS));
			break;
	}
}
#endif

static esp_err_t blynk_tone_play(uint8_t cmd)
{
	int res = 0;

	media_hal_audio_info_t bin_info = {0};

	bin_info.sample_rate = 16000;
	bin_info.channels = 1;
	bin_info.bits_per_sample = 16;

	switch(cmd)
	{
		case POWEROFF:
			res = tone_play_custom(&_binary_02_bin_start, &_binary_02_bin_end, &bin_info);
			break;
		case POWERON:
			res = tone_play_custom(&_binary_01_bin_start, &_binary_01_bin_end, &bin_info);
			break;
		case STEP1:
			res = tone_play_custom(&_binary_03_bin_start, &_binary_03_bin_end, &bin_info);
			break;
		case STEP2:
			res = tone_play_custom(&_binary_04_bin_start, &_binary_04_bin_end, &bin_info);
			break;
		case STEP3:
			res = tone_play_custom(&_binary_05_bin_start, &_binary_05_bin_end, &bin_info);
			break;
	}

	if(res != ESP_OK)
	{
		ESP_LOGE(TAG, "Error tone play.");
	}
	return res;
}

/* Blynk client state handler */
static void state_handler(blynk_client_t *c, const blynk_state_evt_t *ev, void *data) {
	ESP_LOGI(TAG, "state: %d\n", ev->state);
}

/* Virtual write handler */
static void vw_handler(blynk_client_t *c, uint16_t id, const char *cmd, int argc, char **argv, void *data)
{
	if (argc > 1)
	{
		switch(atoi(argv[0]))
		{
			case BLY_VENT_POWER:
				if(!atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					vent_step = 0;
				}
				vent_power_on = atoi(argv[1]);
				blynk_tone_play(atoi(argv[1]));
				break;
			case BLY_VENT_STEP1:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					blynk_tone_play(STEP1);
					vent_step = BLY_VENT_STEP1;
				}
				break;
			case BLY_VENT_STEP2:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					blynk_tone_play(STEP2);
					vent_step = BLY_VENT_STEP2;
				}
				break;
			case BLY_VENT_STEP3:
				if(atoi(argv[1]))
				{
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_tone_play(STEP3);
					vent_step = BLY_VENT_STEP3;
				}
				break;
		}
	}
}

uint8_t prev_pw_value = 0;
uint8_t prev_step_value = 0;

/* Virtual read handler */
static void vr_handler(blynk_client_t *c, uint16_t id, const char *cmd, int argc, char **argv, void *data)
{
	if (!argc) {
		return;
	}

	int pin = atoi(argv[0]);

	if(pin == BLY_VENT_POWER)
	{
		uint8_t pw_value = (uint8_t)vent_power_on;
		uint8_t step_value = vent_step;

		if(prev_pw_value != pw_value)
		{
			/* Respond with `virtual write' command */
			blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_POWER, pw_value);
			if(!pw_value)
			{
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
				blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
				vent_step = 0;
			}
#if defined(BLYNK_I2C)
			blynk_notify_i2c_i94124(pw_value);
#endif
		}
		prev_pw_value = pw_value;

		if(prev_step_value != step_value)
		{
			switch(step_value)
			{
				case BLY_VENT_STEP1:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 1);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					break;
				case BLY_VENT_STEP2:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 1);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					break;
				case BLY_VENT_STEP3:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 1);
					break;
				default:
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP1, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP2, 0);
					blynk_send(c, BLYNK_CMD_HARDWARE, 0, "sii", "vw", BLY_VENT_STEP3, 0);
					break;
			}
		}
		prev_step_value = step_value;
		
	}
}

static void esp_blynk_apps(void)
{
	blynk_err_t ret;

	blynk_client_t *client = malloc(sizeof(blynk_client_t));
	blynk_init(client);

	blynk_options_t opt = {
		.token = CONFIG_BLYNK_TOKEN,
		.server = CONFIG_BLYNK_SERVER,
		/* Use default timeouts */
	};

	blynk_set_options(client, &opt);

	/* Subscribe to state changes and errors */
	blynk_set_state_handler(client, state_handler, NULL);

	/* blynk_set_handler sets hardware (BLYNK_CMD_HARDWARE) command handler */
	blynk_set_handler(client, "vw", vw_handler, NULL);
	blynk_set_handler(client, "vr", vr_handler, NULL);

	/* Start Blynk client task */
	ret = blynk_start(client);
	ESP_LOGE(TAG, "blynk_start ret[%d]", ret);
}
#endif

static EventGroupHandle_t cm_event_group;
const int CONNECTED_BIT = BIT0;
const int PROV_DONE_BIT = BIT1;
#if defined(FACTORY_RESET)
uint8_t reset_counter = 0;
#endif

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    conn_mgr_prov_event_handler(ctx, event);

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        esp_wifi_set_storage(WIFI_STORAGE_FLASH);
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        // We already have print in SYSTEM_EVENT_STA_GOT_IP
#if defined(FACTORY_RESET)
        reset_counter = 0;
#endif
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        app_wifi_stop_timeout_timer();
        printf("%s: Connected with IP Address: %s\n", TAG, ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(cm_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
    case SYSTEM_EVENT_STA_LOST_IP:
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
        app_wifi_stop_timeout_timer();
#if defined(FACTORY_RESET)
        printf("%s: Disconnected. Event: %d. Connecting to the AP again Try %d\n", TAG, event->event_id, reset_counter++);
		if(reset_counter < 20)
	        esp_wifi_connect();
		else
		{
			reset_counter = 0;

            va_led_set(LED_OFF);
            va_nvs_flash_erase();
            va_reset();
            esp_restart();
		}
#else
        printf("%s: Disconnected. Event: %d. Connecting to the AP again Try %d\n", TAG, event->event_id);
        esp_wifi_connect();
#endif
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_init_sta()
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_start() );
#ifdef CONFIG_PM_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif
}

#define MEDIA_HAL_DEFAULT()     \
    {   \
        .op_mode    = MEDIA_HAL_MODE_SLAVE,              \
        .adc_input  = MEDIA_HAL_ADC_INPUT_LINE1,         \
        .dac_output = MEDIA_HAL_DAC_OUTPUT_ALL,          \
        .codec_mode = MEDIA_HAL_CODEC_MODE_BOTH,         \
        .bit_length = MEDIA_HAL_BIT_LENGTH_16BITS,       \
        .format     = MEDIA_HAL_I2S_NORMAL,              \
        .port_num = 0,                          \
    };

void app_prov_done_cb()
{
    xEventGroupSetBits(cm_event_group, PROV_DONE_BIT);
}

#ifdef CTN_REV01
#define TRI_LED 14
#define RES_LED 13

static esp_err_t ctn_led_init()
{
	gpio_pad_select_gpio(TRI_LED);
	gpio_set_direction(TRI_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(TRI_LED, 0);

	gpio_pad_select_gpio(RES_LED);
	gpio_set_direction(RES_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(RES_LED, 0);

	return ESP_OK;
}
#endif

void app_main()
{
    ESP_LOGI(TAG, "==== Voice Assistant SDK version: %s ====", va_get_sdk_version());

    /* This will never be freed */
    alexa_config_t *va_cfg = va_mem_alloc(sizeof(alexa_config_t), VA_MEM_EXTERNAL);

    if (!va_cfg) {
        ESP_LOGE(TAG, "Failed to alloc voice assistant config");
        abort();
    }
    va_cfg->product_id = CONFIG_ALEXA_PRODUCT_ID;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    va_board_init();
    static media_hal_config_t media_hal_conf = MEDIA_HAL_DEFAULT();
    media_hal_init(&media_hal_conf);

#ifdef CTN_REV01
	ctn_led_init();
	va_board_led_init();
#else
	va_board_button_init();
	va_board_led_init();
#endif

    scli_init();
    va_diag_register_cli();
    wifi_register_cli();
    app_wifi_reset_to_prov_init();
    app_auth_register_cli();
    cm_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    printf("\r");       // To remove a garbage print ">>"
    alexa_auth_delegate_init(NULL, NULL);
    bool provisioned = false;
    if (conn_mgr_prov_is_provisioned(&provisioned) != ESP_OK) {
        ESP_LOGE(TAG, "Error getting device provisioning state");
        abort();
    }
    if (app_wifi_get_reset_to_prov() > 0) {
        app_wifi_start_timeout_timer();
        provisioned = false;
        app_wifi_unset_reset_to_prov();
        esp_wifi_set_storage(WIFI_STORAGE_RAM);
    }

    char service_name[20];
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(service_name, sizeof(service_name), "%s%02X%02X", SOFTAP_SSID_PREFIX, mac[4], mac[5]);

    if (!provisioned) {
        va_led_set(LED_RESET);
        printf("%s: Starting provisioning\n", TAG);
        conn_mgr_prov_t prov_type = conn_mgr_prov_mode_ble;
        prov_type.event_cb = alexa_conn_mgr_prov_cb;
        prov_type.cb_user_data = (void *)va_cfg;
        int security = 1;
        const char *pop = "abcd1234";
        const char *service_key = "";
        conn_mgr_prov_start_provisioning(prov_type, security, pop, service_name, service_key);
        printf("\tproof of possession (pop): %s\n", pop);
    } else {
        va_led_set(VA_CAN_START);
        ESP_LOGI(TAG, "Already provisioned, starting station");
        conn_mgr_prov_mem_release();        // This is useful in case of BLE provisioning
        app_prov_done_cb();
        wifi_init_sta();
    }

    xEventGroupWaitBits(cm_event_group, CONNECTED_BIT | PROV_DONE_BIT, false, true, portMAX_DELAY);

    if (!provisioned) {
        va_led_set(VA_CAN_START);
    }

#ifdef CONFIG_ALEXA_ENABLE_EQUALIZER
    alexa_equalizer_init();
#endif

    ret = alexa_local_config_start(va_cfg, service_name);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start local SSDP instance. Some features might not work.");
    }

#ifdef ALEXA_BT
    alexa_bt_a2dp_sink_init();
#endif
    ret = alexa_init(va_cfg);

    if (ret != ESP_OK) {
        while(1) vTaskDelay(2);
    }

    /* This is a blocking call */
    va_dsp_init(speech_recognizer_recognize, speech_recognizer_record);

#ifdef CONFIG_ALEXA_ENABLE_OTA
    /* Doing OTA init after full alexa boot-up. */
    app_ota_init();
#endif
    /* This is only supported with minimum flash size of 8MB. */
    alexa_tone_enable_larger_tones();

#ifdef CONFIG_PM_ENABLE
    rtc_cpu_freq_t max_freq;
    rtc_clk_cpu_freq_from_mhz(CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ, &max_freq);
    esp_pm_config_esp32_t pm_config = {
            .max_cpu_freq = max_freq,
            .min_cpu_freq = RTC_CPU_FREQ_XTAL,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
            .light_sleep_enable = true
#endif
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config));
    gpio_wakeup_enable(GPIO_NUM_36, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_pm_dump_locks(stdout);
#endif

#if defined(BLYNK_APPS)
	esp_blynk_apps();
#endif
    return;
}

/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
//#include "time.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "smbus.h"
#include "i2c-lcd1602.h"
#include "qwiic_twist.h"

static const char *TAG = "IMC_PROFTAAK";

static time_t time_now;
static struct tm time_info;
static int16_t tm_zone_displacement = 0;

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
}
#endif

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

void initialize_sntp(void)
{
    time(&time_now);
    localtime_r(&time_now, &time_info);

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();

    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&time_now);
    localtime_r(&time_now, &time_info);
}

#define LCD_NUM_ROWS               2
#define LCD_NUM_COLUMNS            32
#define LCD_NUM_VISIBLE_COLUMNS    16

#define USE_STDIN  1

#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                     // disabled
#define I2C_MASTER_RX_BUF_LEN    0                     // disabled
#define I2C_MASTER_FREQ_HZ       200000
#define I2C_MASTER_SDA_IO        CONFIG_I2C_MASTER_SDA
#define I2C_MASTER_SCL_IO        CONFIG_I2C_MASTER_SCL

static void i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;  // GY-2561 provides 10kΩ pullups
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_LEN,
                       I2C_MASTER_TX_BUF_LEN, 0);
}


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#define NTP_ENDPOINT "pool.ntp.org"
const char* NTP_SERVER_ENDPOINT = NTP_ENDPOINT;
const long GMT_OFFSET_SECOND = 3600;
const long DAYLIGHT_OFFSET_SECOND = 3600;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static i2c_lcd1602_info_t* lcd_handle;
static qwiic_twist_t* qwiic_handle;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        i2c_lcd1602_clear(lcd_handle);
        i2c_lcd1602_write_string(lcd_handle, "WiFi Connection");
        i2c_lcd1602_move_cursor(lcd_handle, 0, 1);
        i2c_lcd1602_write_string(lcd_handle, "Failed");
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

enum project_screen_state {
    SCREEN_TIME         = 0x00,
    SCREEN_HUMIDITY     = 0x01,
    SCREEN_TEMPERATURE  = 0x02,
    SCREEN_DATE         = 0x03,
};

static int16_t screen_current_state = 0x00;

void click_feedback_task()
{
    qwiic_twist_set_color(qwiic_handle, 255, 255, 255);
    vTaskDelay(200 / portTICK_RATE_MS);
    qwiic_twist_set_color(qwiic_handle, 0, 0, 0);
    vTaskDelay(200 / portTICK_RATE_MS);
    qwiic_twist_set_color(qwiic_handle, 255, 255, 255);
    vTaskDelay(200 / portTICK_RATE_MS);
    qwiic_twist_set_color(qwiic_handle, 0, 0, 0);
    vTaskDelete(NULL);
}

void rotary_encoder_on_clicked()
{
    xTaskCreate(click_feedback_task, "click_feedback_task", 2048, NULL, 10, NULL);

    if (screen_current_state == SCREEN_TIME)
    {   
        tm_zone_displacement++;
        if (tm_zone_displacement > 12)
            tm_zone_displacement = -12;

        int inverted_displacement = tm_zone_displacement * -1;

        char new_timezone[6];
        new_timezone[0] = 'U';
        new_timezone[1] = 'T';
        new_timezone[2] = 'C';
        new_timezone[3] = tm_zone_displacement < 0 ? '+' : '-';

        if (tm_zone_displacement < 0)
        {
            new_timezone[4] = inverted_displacement > 9 || inverted_displacement < -9 ? inverted_displacement / 10 + '0' : inverted_displacement + '0';
            new_timezone[5] = inverted_displacement > 9 || inverted_displacement < -9 ? inverted_displacement % 10 + '0' : '\0';
        }
        else
        {
            new_timezone[4] = tm_zone_displacement > 9 || tm_zone_displacement < -9 ? tm_zone_displacement / 10 + '0' : tm_zone_displacement + '0';
            new_timezone[5] = tm_zone_displacement > 9 || tm_zone_displacement < -9 ? tm_zone_displacement % 10 + '0' : '\0';
        }

        setenv("TZ", new_timezone, 1);
        tzset();
    }
}

void switch_screen_state(int16_t state);
void rotary_encoder_on_moved(int16_t idx)
{
    ESP_LOGI(TAG, "%d", idx);

    int16_t increment = idx > 0 ? 1 : -1;
    screen_current_state += increment;
    if (screen_current_state > 0x03)
        screen_current_state = 0x00;
    if (screen_current_state < 0x00)
        screen_current_state = 0x03;

    switch_screen_state(screen_current_state);
}

void screen_time_task           (void* params);
void screen_humidity_task       (void* params);
void screen_temperature_task    (void* params);
void screen_date_task           (void* params);

static TaskHandle_t active_task_handle;
void switch_screen_state(int16_t state)
{
    if (active_task_handle != NULL)
        vTaskDelete(active_task_handle);

    switch (state)
    {
    case SCREEN_TIME:
    {
        xTaskCreate(screen_time_task, "screen_time_task", 1024*2, (void*)0, 10, &active_task_handle);
    }
    break;

    case SCREEN_HUMIDITY:
    {
        xTaskCreate(screen_humidity_task, "screen_humidity_task", 1024*2, (void*)0, 10, &active_task_handle);
    }
    break;

    case SCREEN_TEMPERATURE:
    {
        xTaskCreate(screen_temperature_task, "screen_temperature_task", 1024*2, (void*)0, 10, &active_task_handle);
    }
    break;

    case SCREEN_DATE:
    {
        xTaskCreate(screen_date_task, "screen_date_task", 1024*2, (void*)0, 10, &active_task_handle);
    }
    break;
    }
}

void screen_time_task(void* params) 
{
    i2c_lcd1602_clear(lcd_handle);
    while (true)
    {
        time(&time_now);
        localtime_r(&time_now, &time_info);

        char strbuff[9];

        // Noob programmer manier
        // strbuff[0] = (time_info.tm_hour / 10) + '0';
        // strbuff[1] = (time_info.tm_hour % 10) + '0';
        // strbuff[2] = ':';
        // strbuff[3] = (time_info.tm_min / 10) + '0';
        // strbuff[4] = (time_info.tm_min % 10) + '0';
        // strbuff[5] = ':';
        // strbuff[6] = (time_info.tm_sec / 10) + '0';
        // strbuff[7] = (time_info.tm_sec % 10) + '0';
        // strbuff[8] = '\0';

        // Proprogrammer manier
        strftime(strbuff, sizeof(strbuff), "%T", &time_info);

        ESP_LOGI(TAG, "%s", strbuff);
        int tzlen = strlen(getenv("TZ"));

        i2c_lcd1602_clear           (lcd_handle);
        i2c_lcd1602_set_cursor      (lcd_handle, true);
        i2c_lcd1602_move_cursor     (lcd_handle, 0, 0);
        i2c_lcd1602_write_string    (lcd_handle, strbuff);
        i2c_lcd1602_move_cursor     (lcd_handle, 0, 1);
        i2c_lcd1602_write_string    (lcd_handle, "<DATE");
        i2c_lcd1602_move_cursor     (lcd_handle, 16 - 5, 1);
        i2c_lcd1602_write_string    (lcd_handle, "HUMI>");
        i2c_lcd1602_move_cursor     (lcd_handle, tzlen != 5 ? 10 : 11, 0);
        i2c_lcd1602_write_string    (lcd_handle, getenv("TZ"));
        i2c_lcd1602_set_cursor      (lcd_handle, false);

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void screen_humidity_task(void* params) 
{
    i2c_lcd1602_clear(lcd_handle);
    int dots = 0;
    int dots_max = 3;
    while (true)
    {
        dots++;
        if (dots > dots_max)
            dots = 0;

        //TODO: If temp sensor available update here
        i2c_lcd1602_clear           (lcd_handle);
        i2c_lcd1602_set_cursor      (lcd_handle, true);

        i2c_lcd1602_move_cursor     (lcd_handle, 2, 0);
        i2c_lcd1602_write_string    (lcd_handle, "NO SENSOR");

        for (int i = 0; i < dots; i++)
            i2c_lcd1602_write_char(lcd_handle, '.');

        i2c_lcd1602_move_cursor     (lcd_handle, 0, 1);
        i2c_lcd1602_write_string    (lcd_handle, "<TIME");
        i2c_lcd1602_move_cursor     (lcd_handle, 16 - 5, 1);
        i2c_lcd1602_write_string    (lcd_handle, "TEMP>");
        i2c_lcd1602_set_cursor      (lcd_handle, false);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void screen_temperature_task(void* params) 
{
    i2c_lcd1602_clear(lcd_handle);
    int dots = 0;
    int dots_max = 3;
    while (true)
    {
        dots++;
        if (dots > dots_max)
            dots = 0;

        //TODO: If temp sensor available update here
        i2c_lcd1602_clear           (lcd_handle);
        i2c_lcd1602_set_cursor      (lcd_handle, true);

        i2c_lcd1602_move_cursor     (lcd_handle, 2, 0);
        i2c_lcd1602_write_string    (lcd_handle, "NO SENSOR");

        for (int i = 0; i < dots; i++)
            i2c_lcd1602_write_char(lcd_handle, '.');

        i2c_lcd1602_move_cursor     (lcd_handle, 0, 1);
        i2c_lcd1602_write_string    (lcd_handle, "<HUMI");
        i2c_lcd1602_move_cursor     (lcd_handle, 16 - 5, 1);
        i2c_lcd1602_write_string    (lcd_handle, "DATE>");
        i2c_lcd1602_set_cursor      (lcd_handle, false);
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void screen_date_task(void* params) 
{
    i2c_lcd1602_clear(lcd_handle);
    while (true)
    {
        time(&time_now);
        localtime_r(&time_now, &time_info);

        char strbuff[14];
        strftime (strbuff, sizeof(strbuff), "%D", &time_info);
        
        i2c_lcd1602_clear           (lcd_handle);
        i2c_lcd1602_set_cursor      (lcd_handle, true);
        i2c_lcd1602_move_cursor     (lcd_handle, 0, 0);
        i2c_lcd1602_write_string    (lcd_handle, strbuff);
        i2c_lcd1602_move_cursor     (lcd_handle, 0, 1);
        i2c_lcd1602_write_string    (lcd_handle, "<TEMP");
        i2c_lcd1602_move_cursor     (lcd_handle, 16 - 5, 1);
        i2c_lcd1602_write_string    (lcd_handle, "TIME>");
        i2c_lcd1602_set_cursor      (lcd_handle, false);

        vTaskDelay(10000 / portTICK_RATE_MS);
    }
}

void rotary_encoder_on_pressed()
{
    
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /*-----------------------------------------------------------------------*/
    /*---------------------------------I2C-----------------------------------*/
    /*-----------------------------------------------------------------------*/

    i2c_master_init();
    i2c_port_t i2c_num = I2C_MASTER_NUM;
    uint8_t address = CONFIG_LCD1602_I2C_ADDRESS;

    smbus_info_t * smbus_info = smbus_malloc();
    ESP_ERROR_CHECK(smbus_init(smbus_info, i2c_num, address));
    ESP_ERROR_CHECK(smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS));

    i2c_lcd1602_info_t * lcd_info = i2c_lcd1602_malloc();
    ESP_ERROR_CHECK(i2c_lcd1602_init(lcd_info, smbus_info, true,
                                     LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VISIBLE_COLUMNS));

    ESP_ERROR_CHECK(i2c_lcd1602_reset(lcd_info));
    lcd_handle = lcd_info;

    i2c_lcd1602_set_cursor(lcd_handle, true);
    i2c_lcd1602_set_backlight(lcd_handle, true);
    i2c_lcd1602_move_cursor(lcd_handle, 0, 0);
    i2c_lcd1602_write_string(lcd_handle, "BOOTING_UP...");

    qwiic_twist_t* qwiic_config = qwiic_twist_malloc();
    qwiic_config->port = I2C_MASTER_NUM;
    qwiic_config->i2c_addr = 0x3E;
    qwiic_config->onButtonClicked = &rotary_encoder_on_clicked;
    qwiic_config->onButtonPressed = &rotary_encoder_on_pressed;
    qwiic_config->onMoved = &rotary_encoder_on_moved;
    ESP_ERROR_CHECK(qwiic_twist_init(qwiic_config));
    ESP_ERROR_CHECK(qwiic_twist_set_color(qwiic_config, 0,0,0));
    qwiic_handle = qwiic_config;

    /*-----------------------------------------------------------------------*/
    /*-------------------------------WIFI------------------------------------*/
    /*-----------------------------------------------------------------------*/

    i2c_lcd1602_clear(lcd_handle);
    i2c_lcd1602_write_string(lcd_handle, "CONNECTING TO");
    i2c_lcd1602_move_cursor(lcd_handle, 0, 1);

    char* str_wifi_connect = (char*)malloc(sizeof(char) * 16);
    sprintf(str_wifi_connect, "WiFi %s", EXAMPLE_ESP_WIFI_SSID);  
    i2c_lcd1602_write_string(lcd_handle, str_wifi_connect);
    free(str_wifi_connect);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    initialize_sntp();

    time(&time_now);

    char strftime_buf[64];
    setenv("TZ", "UTC-0", 1);
    tzset();
    localtime_r(&time_now, &time_info);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &time_info);
    ESP_LOGI(TAG, "The current date/time in The Netherlands is: %s", strftime_buf);
    
    switch_screen_state(0x00);
    if (qwiic_twist_start_task(qwiic_config) != ESP_OK)
    {
        ESP_LOGE(TAG, "TASK_START_QWIIC_FAILED");
        esp_restart();
    }
}

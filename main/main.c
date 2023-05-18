#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#define RX_PIN GPIO_NUM_22
#define TX_PIN GPIO_NUM_23

#define GPS_UART_PORT UART_NUM_2
#define GPS_UART_BAUDRATE 9600
#define GPS_UART_BUF_SIZE 1024

static const char *TAG = "GPS";

typedef struct
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    char status;
    double latitude;
    char lat_dir;
    double longitude;
    char lon_dir;
    float speed;
    float course;
    uint8_t day;
    uint8_t month;
    uint16_t year;
} gps_info_t;

static gps_info_t gps_info;

uint32_t getNumberFromString(uint16_t BeginAddress, char *Buffer)
{
    uint16_t i = BeginAddress;
    uint32_t hexNumber = 0;

    while (isspace(Buffer[i]))
    {
        i++;
    }

    if (Buffer[i] == '0' && (Buffer[i + 1] == 'x' || Buffer[i + 1] == 'X'))
    {
        i += 2;
    }

    while (isxdigit(Buffer[i]))
    {
        char c = tolower(Buffer[i]);
        uint8_t nibble = (c >= 'a' && c <= 'f') ? (c - 'a' + 10) : (c - '0');
        hexNumber = (hexNumber << 4) | nibble;
        i++;
    }

    return hexNumber;
}
void process_gps_data(char *data, gps_info_t *gps_info)
{
    uint8_t pos = 0, length = 0, i = 0;
    char gpsConver[200];

    char *gpsMsg = strstr(data, "$GNRMC");
    ESP_LOGI(TAG, "Read bytes: '%s'", gpsMsg);
    if (gpsMsg != NULL)
    {
        pos += 6; // Skip "$GPRMC"
        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        // Extract hour
        gps_info->hour = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        ESP_LOGI(TAG, "Hour: %d", gps_info->hour);
        pos += 2;

        // Extract minute
        gps_info->minute = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        pos += 2;
        ESP_LOGI(TAG, "Minute: %d", gps_info->minute);

        // Extract second
        gps_info->second = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        pos += 2;
        ESP_LOGI(TAG, "Second: %d", gps_info->second);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        // Check GPS status
        if (data[pos] != 'A')
            return;
        gps_info->status = data[pos];
        ESP_LOGI(TAG, "Status: %c", gps_info->status);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;

        length = 0;
        while (gpsMsg[pos + length] != '.')
            length++;
        gps_info->latitude = 0;

        length -= 2;
        for (i = 0; i < length; i++)
        {
            gps_info->latitude = gps_info->latitude * 10 + (gpsMsg[pos++] - '0');
        }
        i = 0;
        while (gpsMsg[pos] != ',')
            gpsConver[i++] = gpsMsg[pos++];
        pos++;
        gpsConver[i] = '\0';
        gps_info->latitude += atof(gpsConver) / 60;
        gps_info->lat_dir = gpsMsg[pos];
        if (gpsMsg[pos] == 'S')
            gps_info->latitude = -gps_info->latitude;
        ESP_LOGI(TAG, "Latitude: %lf", gps_info->latitude);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        // longitude
        length = 0;
        while (gpsMsg[pos + length] != '.')
            length++;
        gps_info->longitude = 0;
        length -= 2;
        for (i = 0; i < length; i++)
        {
            gps_info->longitude = gps_info->longitude * 10 + (gpsMsg[pos++] - '0');
        }
        i = 0;
        while (gpsMsg[pos] != ',')
            gpsConver[i++] = gpsMsg[pos++];
        pos++;
        gpsConver[i] = '\0';
        gps_info->longitude += atof(gpsConver) / 60;
        gps_info->lon_dir = gpsMsg[pos];
        if (gpsMsg[pos] == 'S')
            gps_info->longitude = -gps_info->longitude;
        ESP_LOGI(TAG, "Longitude: %lf", gps_info->longitude);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        // knot to km/h
        uint32_t temp = strtoul(&gpsMsg[pos], NULL, 10);
        temp = temp * 1000 + strtoul(&gpsMsg[pos + 5], NULL, 10);
        gps_info->speed = temp * 13 / 7020;
        ESP_LOGI(TAG, "Speed: %f", gps_info->speed);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        i = 0;
        while (gpsMsg[pos] != ',')
            gpsConver[i++] = gpsMsg[pos++];
        gpsConver[i] = '\0';
        gps_info->course = strtof(gpsConver, NULL);
        ESP_LOGI(TAG, "Course: %f", gps_info->course);

        while (gpsMsg[pos] != ',')
            pos++;
        pos++;
        gps_info->day = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        pos += 2;
        gps_info->month = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        pos += 2;
        gps_info->year = (data[pos] - '0') * 10 + (data[pos + 1] - '0');
        pos += 2;
        ESP_LOGI(TAG, "Day: %d", gps_info->day);
        ESP_LOGI(TAG, "Month: %d", gps_info->month);
        ESP_LOGI(TAG, "Year: %d", gps_info->year);
    }
    else
    {
        ESP_LOGE(TAG, "FAIL");
    }
}

void gps_task(void *pvParameters)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    uint8_t *data = (uint8_t *)malloc(GPS_UART_BUF_SIZE);
    while (1)
    {
        int len = uart_read_bytes(GPS_UART_PORT, data, GPS_UART_BUF_SIZE - 1, 1000 / portTICK_RATE_MS);
        if (len > 0)
        {
            data[len] = '\0';
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", len, data);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, len, ESP_LOG_INFO);
            process_gps_data((char *)data, &gps_info);
           
        }
    }
    free(data);
}

void app_main()
{
    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    static const gps_info_t initial_gps_info = {0};           // Initialize gps_info with all fields set to 0
    memcpy(&gps_info, &initial_gps_info, sizeof(gps_info_t)); // Copy initial_gps_info to gps_info

    uart_driver_install(GPS_UART_PORT, GPS_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(GPS_UART_PORT, &uart_config);
    uart_set_pin(GPS_UART_PORT, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "Year:");
    // Create GPS task

    xTaskCreate(gps_task, "gps_task", 2048, NULL, 5, NULL);
}
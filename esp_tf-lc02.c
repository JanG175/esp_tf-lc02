/**
 * @file esp_tf-lc02.c
 * @author JanG175
 * @brief ESP-IDF driver for TF-LC02 TOF sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_tf-lc02.h"

static const char* TAG = "tf-lc02";


/**
 * @brief send data through UART
 * 
 * @param tflc configuration structure
 * @param data data to be sent
 * @param len length of data
*/
static void tflc02_uart_write(tflc02_conf_t tflc, uint8_t* data, uint32_t len)
{
    uart_write_bytes(tflc.uart_port, data, len);
    ESP_ERROR_CHECK(uart_wait_tx_done(tflc.uart_port, TFLC02_TIMEOUT));
}


/**
 * @brief receive data through UART
 * 
 * @param tflc configuration structure
 * @param response received data
 * @param len length of data
*/
static void tflc02_uart_recv(tflc02_conf_t tflc, uint8_t* response, uint32_t len)
{
    uint32_t buf = 0;
    uint8_t data[len];

    buf = uart_read_bytes(tflc.uart_port, data, len, TFLC02_TIMEOUT);
    uart_flush(tflc.uart_port);

    if (buf == len)
    {
        for (uint32_t i = 0; i < buf; i++)
            response[i] = data[i];
    }
    else
    {
        ESP_LOGE(TAG, "UART read error");
        for (uint32_t i = 0; i < len; i++)
            response[i] = 0;
    }
}


/**
 * @brief initialize UART communication
 * 
 * @param tflc configuration structure
*/
void tflc02_init(tflc02_conf_t tflc)
{
#ifdef TFLC02_INIT_UART
    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = TFLC02_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(tflc.uart_port, 2048, 0, 0, NULL, 0));
#endif // TFLC02_INIT_UART

    ESP_ERROR_CHECK(uart_param_config(tflc.uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(tflc.uart_port, tflc.tx_pin, tflc.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


/**
 * @brief measure distance
 * 
 * @param tflc configuration structure
 * @param distance measured distance
 * 
 * @return error code
*/
esp_err_t tflc02_measure_distance(tflc02_conf_t tflc, uint16_t* distance)
{
    esp_err_t err = ESP_OK;

    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_MEASURE_DISTANCE;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 8;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] == TFLC02_PACKET_HEADER_0 && response[1] == TFLC02_PACKET_HEADER_1 &&
        response[len_r-1] == TFLC02_PACKET_END && response[2] == TFLC02_MEASURE_DISTANCE && response[3] == 3)
    {
        switch (response[6])
        {
            case TFLC02_VALID_DATA:
                *distance = ((uint16_t)response[4] << 8) | (uint16_t)response[5];
                err = ESP_OK;
                break;
            case TFLC02_VCSEL_SHORT:
                ESP_LOGE(TAG, "VCSEL short-circuited");
                err = ESP_ERR_INVALID_STATE;
                break;
            case TFLC02_LOW_SIGNAL:
                ESP_LOGE(TAG, "Low signal");
                err = ESP_ERR_INVALID_STATE;
                break;
            case TFLC02_LOW_SN:
                ESP_LOGE(TAG, "Low SN");
                err = ESP_ERR_INVALID_STATE;
                break;
            case TFLC02_TOO_MUCH_AMB:
                ESP_LOGE(TAG, "Too much ambient light");
                err = ESP_ERR_INVALID_STATE;
                break;
            case TFLC02_WAF:
                ESP_LOGE(TAG, "Wrapping error");
                err = ESP_FAIL;
                break;
            case TFLC02_CAL_ERROR:
                ESP_LOGE(TAG, "Internal calculation error");
                err = ESP_FAIL;
                break;
            case TFLC02_CROSSTALK_ERROR:
                ESP_LOGE(TAG, "Crosstalk error");
                err = ESP_FAIL;
                break;
            default:
                ESP_LOGE(TAG, "Unknown error");
                err = ESP_FAIL;
                break;
        }
    }
    else
    {
        ESP_LOGW(TAG, "Invalid communication");
        err = ESP_ERR_INVALID_RESPONSE;
    }

    return err;
}


/**
 * @brief perform sensor crosstalk correction
 * 
 * @param tflc configuration structure
 * @param error error value
 * @param xtak crosstalk data factory
*/
void tflc02_crosstalk_correction(tflc02_conf_t tflc, uint8_t* error, uint16_t* xtak)
{
    ESP_LOGW(TAG, "Put the product in the dark box and there should be no barriers within 60 cm. The product runs the crosstalk correction program after receiving the command, then store correction value and return the value to MCU.");

    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_OFFSET_CORRECTION;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 8;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] == TFLC02_PACKET_HEADER_0 && response[1] == TFLC02_PACKET_HEADER_1 &&
        response[len_r-1] == TFLC02_PACKET_END && response[2] == TFLC02_OFFSET_CORRECTION && response[3] == 3)
    {
        *error = response[4];
        *xtak = ((uint16_t)response[5] << 8) | (uint16_t)response[6];
    }
    else
        ESP_LOGW(TAG, "Invalid communication");
}


/**
 * @brief perform sensor offset correction
 * 
 * @param tflc configuration structure
 * @param error error value
*/
void tflc02_offset_correction(tflc02_conf_t tflc, uint8_t* error, uint8_t* offset_short1, uint8_t* offset_short2,
                                uint8_t* offset_long1, uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration)
{
    ESP_LOGW(TAG, "Put the product in dark box and there should be 75%% grayscale white card within 10 cm. The product runs offset correction program after receiving the command, then stores correction value and return the value to MCU.");

    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_OFFSET;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 10;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] == TFLC02_PACKET_HEADER_0 && response[1] == TFLC02_PACKET_HEADER_1 &&
        response[len_r-1] == TFLC02_PACKET_END && response[2] == TFLC02_OFFSET && response[3] == 5)
    {
        *error = response[4];
        *offset_short1 = response[5];
        *offset_short2 = response[6];
        *offset_long1 = response[7];
        *offset_long2 = response[8];
    }
    else
        ESP_LOGW(TAG, "Invalid communication");
}


/**
 * @brief reset sensor
 *
 * @param tflc configuration structure
*/
void tflc02_reset(tflc02_conf_t tflc)
{
    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_RESET;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 5;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] != TFLC02_PACKET_HEADER_0 || response[1] != TFLC02_PACKET_HEADER_1 ||
        response[len_r-1] != TFLC02_PACKET_END || response[2] != TFLC02_RESET || response[3] != 0)
        ESP_LOGW(TAG, "Invalid communication");

    vTaskDelay(10 / portTICK_PERIOD_MS);
}


/**
 * @brief read sensor factory settings
 *
 * @param tflc configuration structure
 * @param offset_short short distance offset
 * @param offset_long long distance offset
 * @param crosstalk crosstalk value
 * @param calibration calibration value
*/
void tflc02_read_factory_settings(tflc02_conf_t tflc, uint8_t* offset_short1, uint8_t* offset_short2, uint8_t* offset_long1,
                                    uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration)
{
    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_FACTORY_SETTINGS;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 12;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] == TFLC02_PACKET_HEADER_0 && response[1] == TFLC02_PACKET_HEADER_1 &&
        response[len_r-1] == TFLC02_PACKET_END && response[2] == TFLC02_FACTORY_SETTINGS && response[3] == 7)
    {
        *offset_short1 = response[4];
        *offset_short2 = response[5];
        *offset_long1 = response[6];
        *offset_long2 = response[7];
        *crosstalk = ((uint16_t)response[9] << 8) | (uint16_t)response[8];
        *calibration = response[10];
    }
    else
        ESP_LOGW(TAG, "Invalid communication");
}


/**
 * @brief read sensor information
 *
 * @param tflc configuration structure
 * @param sensor_ic sensor model
 * @param support support information
 * @param version version information
*/
void tflc02_read_information(tflc02_conf_t tflc, uint8_t* sensor_ic, uint8_t* support, uint8_t* version)
{
    uint32_t len_w = 5;
    uint8_t data[len_w];

    data[0] = TFLC02_PACKET_HEADER_0;
    data[1] = TFLC02_PACKET_HEADER_1;
    data[2] = TFLC02_INFORMATION;
    data[3] = 0;
    data[4] = TFLC02_PACKET_END;

    tflc02_uart_write(tflc, data, len_w);

    uint32_t len_r = 8;
    uint8_t response[len_r];

    tflc02_uart_recv(tflc, response, len_r);

    if (response[0] == TFLC02_PACKET_HEADER_0 && response[1] == TFLC02_PACKET_HEADER_1 &&
        response[len_r-1] == TFLC02_PACKET_END && response[2] == TFLC02_INFORMATION && response[3] == 3)
    {
        *sensor_ic = response[4];
        *support = response[5];
        *version = response[6];
    }
    else
        ESP_LOGW(TAG, "Invalid communication");
}

/**
 * @file esp_tf-lc02.h
 * @author JanG175
 * @brief ESP-IDF driver for TF-LC02 TOF sensor
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TFLC02_INIT_UART          1 // uncomment to initialize UART

#define TFLC02_BAUDRATE           115200
#define TFLC02_TIMEOUT            (2 / portTICK_PERIOD_MS)

#define TFLC02_PACKET_HEADER_0    0x55
#define TFLC02_PACKET_HEADER_1    0xAA
#define TFLC02_PACKET_END         0xFA

#define TFLC02_MEASURE_DISTANCE   0x81 // Measuring distance value and measurement status (unit: mm)
#define TFLC02_OFFSET_CORRECTION  0x82 // Offset correction value and correction result
#define TFLC02_OFFSET             0x83 // Offset correction and correction result
#define TFLC02_RESET              0x84 // Reset the ToF module
#define TFLC02_FACTORY_SETTINGS   0x85 // Get factory default settings, including offset, X_TALK and check
#define TFLC02_INFORMATION        0x86 // Get product information, including model, communication mode andversion

#define TFLC02_VALID_DATA         0x00 // Valid value
#define TFLC02_VCSEL_SHORT        0x01 // When the VCSEL is short-circuited. If this error occurs, theVCSEL current will not flow inside the IC
#define TFLC02_LOW_SIGNAL         0x02 // The amount of reflected light obtained fromthe detectedobject is small
#define TFLC02_LOW_SN             0x04 // The ratio of reflected light from the detected object anddisturbance light is small
#define TFLC02_TOO_MUCH_AMB       0x08 // Disturbance light is large
#define TFLC02_WAF                0x10 // Wrapping error
#define TFLC02_CAL_ERROR          0x20 // Internal calculation error
#define TFLC02_CROSSTALK_ERROR    0x80 // Crosstalk from the panel is large

#define TFLC02_CAL_UNCHECKED      0x00 // Unchecked
#define TFLC02_CAL_CHECKED        0x01 // Crosstalk checked
#define TFLC02_CAL_OFFSET         0x02 // Offset checked
#define TFLC02_CAL_FULL           0x03 // Full checked

#define TFLC02_SENSOR_IC_0        0x02 // The sensor is gp2ap02vt
#define TFLC02_SENSOR_IC_1        0x03 // The sensor is gp2ap03vt

#define TFLC02_UART_I2C_SUPPORT   0x41 // The module supports I2C and UART
#define TFLC02_UART_SUPPORT       0x55 // The module supports UART
#define TFLC02_I2C_SUPPORT        0x49 // The module supports I2C

typedef struct tflc02_conf_t
{
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
} tflc02_conf_t;


void tflc02_init(tflc02_conf_t tflc);

esp_err_t tflc02_measure_distance(tflc02_conf_t tflc, uint16_t* distance);

void tflc02_crosstalk_correction(tflc02_conf_t tflc, uint8_t* error, uint16_t* xtak);

void tflc02_offset_correction(tflc02_conf_t tflc, uint8_t* error, uint8_t* offset_short1, uint8_t* offset_short2,
                                uint8_t* offset_long1, uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration);

void tflc02_reset(tflc02_conf_t tflc);

void tflc02_read_factory_settings(tflc02_conf_t tflc, uint8_t* offset_short1, uint8_t* offset_short2, uint8_t* offset_long1,
                                    uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration);

void tflc02_read_information(tflc02_conf_t tflc, uint8_t* sensor_ic, uint8_t* support, uint8_t* version);

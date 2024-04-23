#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TFL02_INIT_UART          1 // uncomment to initialize UART

#define TFL02_BAUDRATE           115200
#define TFL02_TIMEOUT            100 / portTICK_PERIOD_MS

#define TFL02_PACKET_HEADER_0    0x55
#define TFL02_PACKET_HEADER_1    0xAA
#define TFL02_PACKET_END         0xFA

#define TFL02_MEASURE_DISTANCE   0x81 // Measuring distance value and measurement status (unit: mm)
#define TFL02_OFFSET_CORRECTION  0x82 // Offset correction value and correction result
#define TFL02_OFFSET             0x83 // Offset correction and correction result
#define TFL02_RESET              0x84 // Reset the ToF module
#define TFL02_FACTORY_SETTINGS   0x85 // Get factory default settings, including offset, X_TALK and check
#define TFL02_INFORMATION        0x86 // Get product information, including model, communication mode andversion

#define TFL02_VALID_DATA         0x00 // Valid value
#define TFL02_VCSEL_SHORT        0x01 // When the VCSEL is short-circuited. If this error occurs, theVCSEL current will not flow inside the IC
#define TFL02_LOW_SIGNAL         0x02 // The amount of reflected light obtained fromthe detectedobject is small
#define TFL02_LOW_SN             0x04 // The ratio of reflected light from the detected object anddisturbance light is small
#define TFL02_TOO_MUCH_AMB       0x08 // Disturbance light is large
#define TFL02_WAF                0x10 // Wrapping error
#define TFL02_CAL_ERROR          0x20 // Internal calculation error
#define TFL02_CROSSTALK_ERROR    0x80 // Crosstalk from the panel is large

#define TFL02_CAL_UNCHECKED      0x00 // Unchecked
#define TFL02_CAL_CHECKED        0x01 // Crosstalk checked
#define TFL02_CAL_OFFSET         0x02 // Offset checked
#define TFL02_CAL_FULL           0x03 // Full checked

#define TFL02_SENSOR_IC_0        0x02 // The sensor is gp2ap02vt
#define TFL02_SENSOR_IC_1        0x03 // The sensor is gp2ap03vt

#define TFL02_UART_I2C_SUPPORT   0x41 // The module supports I2C and UART
#define TFL02_UART_SUPPORT       0x55 // The module supports UART
#define TFL02_I2C_SUPPORT        0x49 // The module supports I2C

typedef struct tfl02_conf_t
{
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
} tfl02_conf_t;


void tfl02_init(tfl02_conf_t tfl);

void tfl02_measure_distance(tfl02_conf_t tfl, uint16_t* distance);

void tfl02_crosstalk_correction(tfl02_conf_t tfl, uint8_t* error, uint16_t* xtak);

void tfl02_offset_correction(tfl02_conf_t tfl, uint8_t* error, uint8_t* offset_short1, uint8_t* offset_short2,
                                uint8_t* offset_long1, uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration);

void tfl02_reset(tfl02_conf_t tfl);

void tfl02_read_factory_settings(tfl02_conf_t tfl, uint8_t* offset_short1, uint8_t* offset_short2, uint8_t* offset_long1,
                                    uint8_t* offset_long2, uint16_t* crosstalk, uint8_t* calibration);

void tfl02_read_information(tfl02_conf_t tfl, uint8_t* sensor_ic, uint8_t* support, uint8_t* version);

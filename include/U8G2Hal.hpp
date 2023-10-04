#pragma once

#include "u8g2.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"

#if SOC_I2C_NUM > 1
#define I2C_MASTER_NUM I2C_NUM_1     //  I2C port number for master dev
#else
#define I2C_MASTER_NUM I2C_NUM_0     //  I2C port number for master dev
#endif


/** @public
 * HAL configuration structure.
 */
typedef struct {
union {
    /* SPI settings. */
    struct {
    /* GPIO num for clock. */
    gpio_num_t clk;
    /* GPIO num for SPI mosi. */
    gpio_num_t mosi;
    /* GPIO num for SPI slave/chip select. */
    gpio_num_t cs;
    } spi;
    /* I2C settings. */
    struct {
    /* GPIO num for I2C data. */
    gpio_num_t sda;
    /* GPIO num for I2C clock. */
    gpio_num_t scl;
    } i2c;
} bus;
/* GPIO num for reset. */
gpio_num_t reset;
/* GPIO num for DC. */
gpio_num_t dc;
    } u8g2_esp32_hal_t;

class U8G2Hal
{

    public:


    static spi_device_handle_t handle_spi;   // SPI handle.
    static i2c_cmd_handle_t handle_i2c;      // I2C handle.
    static u8g2_esp32_hal_t u8g2_esp32_hal;  // HAL state data.              

    static const constexpr uint16_t I2C_MASTER_TX_BUF_DISABLE = 0;  //  I2C master do not need buffer
    static const constexpr uint16_t  I2C_MASTER_RX_BUF_DISABLE = 0;  //  I2C master do not need buffer
    static const constexpr uint32_t  I2C_MASTER_FREQ_HZ = 400000U;     //  I2C master clock frequency
    static const constexpr uint16_t I2C_TIMEOUT_MS = 1000;
    static const constexpr uint16_t  ACK_CHECK_EN = 0x1;             //  I2C master will check ack from slave
    static const constexpr uint16_t  ACK_CHECK_DIS = 0x0;  //  I2C master will not check ack from slave
    static const constexpr gpio_num_t U8G2_ESP32_HAL_UNDEFINED = GPIO_NUM_NC;
    static const constexpr spi_host_device_t HOST = SPI2_HOST;
    static const constexpr char* TAG = "U8G2Hal";
                     
    /**
     * Construct a default HAL configuration with all fields undefined.
     */
    static const constexpr u8g2_esp32_hal_t U8G2_ESP32_HAL_DEFAULT =                                        
    {                                                                   
        .bus = {.spi = {.clk = U8G2_ESP32_HAL_UNDEFINED,                  
                        .mosi = U8G2_ESP32_HAL_UNDEFINED,                 
                        .cs = U8G2_ESP32_HAL_UNDEFINED}},                 
        .reset = U8G2_ESP32_HAL_UNDEFINED, .dc = U8G2_ESP32_HAL_UNDEFINED 
    };


    /**
     * Initialize the HAL with the given configuration.
     *
     */
    static void hal_init(u8g2_esp32_hal_t u8g2_esp32_hal_param);
    static uint8_t spi_byte_cb(u8x8_t* u8x8,
                                uint8_t msg,
                                uint8_t arg_int,
                                void* arg_ptr);
    static uint8_t i2c_byte_cb(u8x8_t* u8x8,
                                uint8_t msg,
                                uint8_t arg_int,
                                void* arg_ptr);
    static uint8_t gpio_and_delay_cb(u8x8_t* u8x8,
                                        uint8_t msg,
                                        uint8_t arg_int,
                                        void* arg_ptr);


};
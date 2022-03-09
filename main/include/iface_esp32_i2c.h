#ifndef _IFACE_ESP32_I2C_H_
#define _IFACE_ESP32_I2C_H_

#include "esp_err.h"
#include "ssd1306.h"

#define USE_THIS_I2C_PORT I2C_NUM_1

#define CONFIG_I2CDEV_TIMEOUT	1000 /* One second timout for I2C access */

/* STUBS for external source */
#define I2C_DEV_TAKE_MUTEX(dev)
#define I2C_DEV_GIVE_MUTEX(dev)
#define I2C_DEV_CHECK(dev,X) do { \
        esp_err_t ___ = X; \
        if (___ != ESP_OK) { \
            return ___; \
        } \
    } while (0)
/**
 * I2C device descriptor
 */
typedef struct
{
    uint8_t addr;            //!< Unshifted address
} i2c_dev_t;

/**
 * @brief Read from slave device
 *
 * Issue a send operation of \p out_data register adress, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 * @param[in] dev Device descriptor
 * @param[in] out_data Pointer to data to send if non-null
 * @param[in] out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param[in] in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data,
        size_t out_size, void *in_data, size_t in_size);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 * @param[in] dev Device descriptor
 * @param[in] out_reg Pointer to register address to send if non-null
 * @param[in] out_reg_size Size of register address
 * @param[in] out_data Pointer to data to send
 * @param[in] out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg,
        size_t out_reg_size, const void *out_data, size_t out_size);

/**
 * @brief Read from register with an 8-bit address
 *
 * Shortcut to i2c_dev_read().
 * @param[in] dev Device descriptor
 * @param[in] reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param[in] in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg,
        void *in_data, size_t in_size);

/**
 * @brief Write to register with an 8-bit address
 *
 * Shortcut to i2c_dev_write().
 * @param[in] dev Device descriptor
 * @param[in] reg Register address
 * @param[in] out_data Pointer to data to send
 * @param[in] out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg,
        const void *out_data, size_t out_size);

int ESP32_WriteCommand_I2C( struct SSD1306_Device* DeviceHandle, SSDCmd SSDCommand );
int ESP32_WriteData_I2C( struct SSD1306_Device* DeviceHandle, uint8_t* Data, size_t DataLength );
int ESP32_InitI2CMaster( int SDA, int SCL );

#endif

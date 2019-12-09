#include <esp_log.h>
//#include <esp_idf_lib_helpers.h>
#include "iface_esp32_i2c.h"
#include "pca9555.h"


/**
 *  Low level write command. Uses i2cmaster library to communicate with the
 *  device. Calls i2c_start_wait, which waits for ACK from the device, so if
 *  the specified device is not on the i2c bus, it will wait forever.
 *  @param dev 3 bit device address.
 *  @param reg Register to write.
 *  @param value Value to write.
 */
void PCA9555_write(i2c_dev_t *dev, PCA9555_REGISTER reg, uint16_t value)
{
	uint8_t buf[2] = {(value & 0xFF),  ((value >> 8) & 0xFF)};
    i2c_dev_write_reg(dev, reg, buf, 2);
}

/**
 *  Low level read command. Uses i2cmaster library to communicate with the
 *  device. Calls i2c_start_wait, which waits for ACK from the device, so if
 *  the specified device is not on the i2c bus, it will wait forever.
 *  @param dev 3 bit device address.
 *  @param reg Register to read.
 *  @return value Value read from the device.
 */
uint16_t PCA9555_read(i2c_dev_t *dev, PCA9555_REGISTER reg)
{
    uint8_t buf[2];
    esp_err_t res = i2c_dev_read_reg(dev, reg, buf, 2);

    return ((res == ESP_OK)? ((buf[1]<<8) | buf[0]): ESP_FAIL);
}

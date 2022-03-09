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

/**
 *  Set the direction of a port. Implemented as a macro.
 *  @param dev 3 bit device address.
 *  @param port Write to port 0 or 1.
 *  @param dir 8 bit direction of the pins. 1 means input, 0 output.
 */
void PCA9555_dir(i2c_dev_t *dev, uint16_t dir)
{
    PCA9555_write(dev, PCA9555_DIRECTION_0, dir);
}

/**
 *  Invert the polarity of pins on a port. Take care when using this function
 *  as it inverts the current polarity settings, it is not an absolute value.
 *  If the PCA9555 has not been reset but the microcontroller has been, this
 *  might not contain the value you expect. This is complicated by the fact
 *  that this chip does not have any RESET line or i2c command that can be
 *  accessed from a microcontroller. Implemented as a macro.
 *  @param dev 3 bit device address.
 *  @param port Write to port 0 or 1.
 *  @param pol 8 bit polarity of the pins. 1 means invert, 0 no change. */
void PCA9555_pol(i2c_dev_t *dev, uint16_t pol)
{
    PCA9555_write(dev, PCA9555_POLARITY_INV_0, pol);
}

/**
 *  Set values on a port. Implemented as a macro.
 *  @param dev 3 bit device address.
 *  @param port Write to port 0 or 1.
 *  @param value New 8 bit value for the port.
 */
void PCA9555_set(i2c_dev_t *dev, uint16_t value)
{
    PCA9555_write(dev, PCA9555_OUTPUT_0, value);
}

/**
 *  Get values from a port.
 *  @param dev 3 bit device address.
 *  @param port Write to port 0 or 1.
 *  @return 8 bit value on the port.
 */
uint16_t PCA9555_get(i2c_dev_t *dev)
{
    return PCA9555_read(dev, PCA9555_INPUT_0);
}

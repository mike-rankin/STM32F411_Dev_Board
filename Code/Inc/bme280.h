
/*! @file bme280.h
    @brief Sensor driver for BME280 sensor */
/*!
 * @defgroup BME280 SENSOR API
 * @{*/
#ifndef BME280_H_
#define BME280_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/* Header includes */
#include "bme280_defs.h"

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 *
 *  @param[in,out] dev : Structure instance of bme280_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_init(struct bme280_dev *dev);

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be written.
 * @param[in] reg_data : Pointer to data buffer which is to be written
 * in the sensor.
 * @param[in] len : No of bytes of data to write..
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, const struct bme280_dev *dev);

/*!
 * @brief This API reads the data from the given register address of the sensor.
 *
 * @param[in] reg_addr : Register address from where the data to be read
 * @param[out] reg_data : Pointer to data buffer to store the read data.
 * @param[in] len : No of bytes of data to be read.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, const struct bme280_dev *dev);

/*!
 * @brief This API sets the oversampling, filter and standby duration
 * (normal mode) settings in the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] desired_settings : Variable used to select the settings which
 * are to be set in the sensor.
 *
 * @note : Below are the macros to be used by the user for selecting the
 * desired settings. User can do OR operation of these macros for configuring
 * multiple settings.
 *
 * Macros		  |   Functionality
 * -----------------------|----------------------------------------------
 * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
 * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
 * BME280_OSR_HUM_SEL    |   To set humidity oversampling.
 * BME280_FILTER_SEL     |   To set filter setting.
 * BME280_STANDBY_SEL  |   To set standby duration setting.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_set_sensor_settings(uint8_t desired_settings, const struct bme280_dev *dev);

/*!
 * @brief This API gets the oversampling, filter and standby duration
 * (normal mode) settings from the sensor.
 *
 * @param[in,out] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_get_sensor_settings(struct bme280_dev *dev);

/*!
 * @brief This API sets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[in] sensor_mode : Variable which contains the power mode to be set.
 *
 *    sensor_mode           |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_set_sensor_mode(uint8_t sensor_mode,
				const struct bme280_dev *dev);

/*!
 * @brief This API gets the power mode of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 * @param[out] sensor_mode : Pointer variable to store the power mode.
 *
 *   sensor_mode            |   Macros
 * ---------------------|-------------------
 *     0                | BME280_SLEEP_MODE
 *     1                | BME280_FORCED_MODE
 *     3                | BME280_NORMAL_MODE
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_sensor_mode(uint8_t *sensor_mode, const struct bme280_dev *dev);

/*!
 * @brief This API performs the soft reset of the sensor.
 *
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
 */
int8_t bme280_soft_reset(const struct bme280_dev *dev);

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 *
 * @param[in] sensor_comp : Variable which selects which data to be read from
 * the sensor.
 *
 * sensor_comp |   Macros
 * ------------|-------------------
 *     1       | BME280_PRESS
 *     2       | BME280_TEMP
 *     4       | BME280_HUM
 *     7       | BME280_ALL
 *
 * @param[out] comp_data : Structure instance of bme280_data.
 * @param[in] dev : Structure instance of bme280_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
 */
int8_t bme280_get_sensor_data(uint8_t sensor_comp, struct bme280_data *comp_data, struct bme280_dev *dev);

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 *
 *  @param[in] reg_data     : Contains register data which needs to be parsed
 *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
 *  and humidity data.
 */
void bme280_parse_sensor_data(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected by the
 * user.
 *
 * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
 * humidity.
 * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
 * humidity data.
 * @param[out] comp_data : Contains the compensated pressure and/or temperature
 * and/or humidity data.
 * @param[in] calib_data : Pointer to the calibration data structure.
 *
 * @return Result of API execution status.
 * @retval zero -> Success / -ve value -> Error
 */
int8_t bme280_compensate_data(uint8_t sensor_comp, const struct bme280_uncomp_data *uncomp_data,
				     struct bme280_data *comp_data, struct bme280_calib_data *calib_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* BME280_H_ */
/** @}*/

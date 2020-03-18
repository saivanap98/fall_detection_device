#include <Arduino.h>
#include "BNO055_support.h"
#include <I2C_DMAC.h>

void initBNO055(struct bno055_t * bno055_sensor);
BNO055_RETURN_FUNCTION_TYPE bno055_format_accel_data(struct bno055_accel * accRaw, struct bno055_accel_float * accActual);
BNO055_RETURN_FUNCTION_TYPE bno055_format_mag_data(struct bno055_mag * magRaw, struct bno055_mag_float * magActual);
BNO055_RETURN_FUNCTION_TYPE bno055_format_gyro_data(struct bno055_gyro * gyroRaw, struct bno055_gyro_float * gyroActual);
BNO055_RETURN_FUNCTION_TYPE bno055_format_euler_data(struct bno055_euler * eulerRaw, struct bno055_euler_float * eulerActual);
BNO055_RETURN_FUNCTION_TYPE bno055_format_quaternion_data(struct bno055_quaternion * quaternionRaw, struct bno055_quaternion_float * quaternionActual);

struct bno055_t myBNO;

struct bno055_accel rawAccelData;
struct bno055_mag rawMagData;
struct bno055_gyro rawGyroData;
struct bno055_euler rawEulerData;
struct bno055_quaternion rawQuaternionData;

struct bno055_accel_float actualAccelData;
struct bno055_mag_float actualMagData;
struct bno055_gyro_float actualGyroData;
struct bno055_euler_float actualEulerData;
struct bno055_quaternion_float actualQuaternionData;

const unsigned char acc_units = 0; // 0: m/s2, 1: mG
const unsigned char gyr_units = 0; // 0: dps, 1: rps
const unsigned char eul_units = 0; // 0: Degrees, 1: Radians
const unsigned char temp_units = 1; // 0: Celsius, 1: Farenheit

unsigned long lastTime = 0;

void setup(){
								//Initialize the Serial Port to view information on the Serial Monitor
								Serial.begin(115200);
								while(!Serial) {};

								//Initialize I2C communication
								I2C.begin(400000);

								//Initialize BNO055 sensor
								initBNO055(&myBNO);


}

void loop() {
								if ((millis() - lastTime) >= 50
												) //To stream at 10Hz without using additional timers
								{
																lastTime = millis();

																bno055_read_accel_xyz(&rawAccelData);
																bno055_read_mag_xyz(&rawMagData);
																bno055_read_gyro_xyz(&rawGyroData);
																bno055_read_euler_hrp(&rawEulerData);
																bno055_read_quaternion_wxyz(&rawQuaternionData);

																bno055_format_accel_data(&rawAccelData, &actualAccelData);
																bno055_format_mag_data(&rawMagData, &actualMagData);
																bno055_format_gyro_data(&rawGyroData, &actualGyroData);
																bno055_format_euler_data(&rawEulerData, &actualEulerData);
																bno055_format_quaternion_data(&rawQuaternionData, &actualQuaternionData);
								}
}

/**
 * Initialize BNO055 Sensor
 * @method initBNO055
 * @param  bno055_sensor [pointer to sensor object]
 */
void initBNO055(struct bno055_t * bno055_sensor) {
								//Initialization of the BNO055
								BNO_Init(bno055_sensor); //Assigning the structure to hold information about the device

								//Configure units of BNO055 sensor data
								bno055_set_accel_unit(acc_units);
								bno055_set_gyro_unit(gyr_units);
								bno055_set_euler_unit(eul_units);
								bno055_set_temperature_unit(temp_units);

								//Configuration to NDoF mode
								bno055_set_operation_mode(OPERATION_MODE_NDOF);

								delay(1);

								//Read out device information
								Serial.print("Chip ID: ");
								Serial.println(bno055_sensor->chip_id, HEX);

								//Read out the software revision ID
								Serial.print("Software Revision ID: ");
								Serial.println(bno055_sensor->sw_revision_id, HEX);

								//Read out the page ID
								Serial.print("Page ID: ");
								Serial.println(bno055_sensor->page_id, HEX);

								//Read out the accelerometer revision ID
								Serial.print("Accelerometer Revision ID: ");
								Serial.println(bno055_sensor->accel_revision_id, HEX);

								//Read out the gyroscope revision ID
								Serial.print("Gyroscope Revision ID: ");
								Serial.println(bno055_sensor->gyro_revision_id, HEX);

								//Read out the magnetometer revision ID
								Serial.print("Magnetometer Revision ID: ");
								Serial.println(bno055_sensor->mag_revision_id, HEX);

								//Read out the bootloader revision ID
								Serial.print("Bootloader Revision ID: ");
								Serial.println(bno055_sensor->bootloader_revision_id, HEX);

								//Read out the device address
								Serial.print("Device Address: ");
								Serial.println(bno055_sensor->dev_addr, HEX);
}

/**
 * Formats accelerometer data based on selected units
 * @method bno055_format_accel_data
 * @param  accRaw                   [raw accelerometer data]
 * @param  accActual                [formatted accelerometer data]
 * @return                          [result of calculation]
 */
BNO055_RETURN_FUNCTION_TYPE bno055_format_accel_data(struct bno055_accel * accRaw, struct bno055_accel_float * accActual) {
								BNO055_RETURN_FUNCTION_TYPE calres = BNO055_Zero_U8X;
								unsigned char unit;
								bno055_get_accel_unit( &unit);
								if (unit == 0) {
																accActual->x = float(accRaw->x) / 100;
																accActual->y = float(accRaw->y) / 100;
																accActual->z = float(accRaw->z) / 100;
								} else if (unit == 1) {
																accActual->x = float(accRaw->x);
																accActual->y = float(accRaw->y);
																accActual->z = float(accRaw->z);
								} else {
																return ERROR1;
								}

								return calres;
}

/**
 * Formats magnetometer data
 * @method bno055_format_mag_data
 * @param  magRaw                 [raw magnetometer data]
 * @param  magActual              [formatted magnetometer data]
 * @return                        [result of calculation]
 */
BNO055_RETURN_FUNCTION_TYPE bno055_format_mag_data(struct bno055_mag * magRaw, struct bno055_mag_float * magActual) {
								BNO055_RETURN_FUNCTION_TYPE calres = BNO055_Zero_U8X;
								magActual->x = float(magRaw->x) / 16;
								magActual->y = float(magRaw->y) / 16;
								magActual->z = float(magRaw->z) / 16;

								return calres;
}

/**
 * Formats gyroscope data based on selected units
 * @method bno055_format_gyro_data
 * @param  gyroRaw                 [raw gyroscope data]
 * @param  gyroActual              [formatted gyroscope data]
 * @return                         [result of calcualtion]
 */
BNO055_RETURN_FUNCTION_TYPE bno055_format_gyro_data(struct bno055_gyro * gyroRaw, struct bno055_gyro_float * gyroActual) {
								BNO055_RETURN_FUNCTION_TYPE calres = BNO055_Zero_U8X;
								unsigned char unit;
								bno055_get_gyro_unit(&unit);
								if (unit == 0) {
																gyroActual->x = float(gyroRaw->x) / 16;
																gyroActual->y = float(gyroRaw->y) / 16;
																gyroActual->z = float(gyroRaw->z) / 16;
								} else if(unit == 1) {
																gyroActual->x = float(gyroRaw->x) / 900;
																gyroActual->y = float(gyroRaw->y) / 900;
																gyroActual->z = float(gyroRaw->z) / 900;
								}
								else {
																return ERROR1;
								}

								return calres;
}

/**
 * Formats euler data based on selected units
 * @method bno055_format_euler_data
 * @param  eulerRaw                 [raw euler data]
 * @param  eulerActual              [formatted euler data]
 * @return                          [result of calculation]
 */
BNO055_RETURN_FUNCTION_TYPE bno055_format_euler_data(struct bno055_euler * eulerRaw, struct bno055_euler_float * eulerActual) {
								BNO055_RETURN_FUNCTION_TYPE calres = BNO055_Zero_U8X;
								unsigned char unit;
								bno055_get_euler_unit(&unit);
								if (unit == 0) {
																eulerActual->h = float(eulerRaw->h) / 16;
																eulerActual->r = float(eulerRaw->r) / 16;
																eulerActual->p = float(eulerRaw->p) / 16;
								} else if(unit == 1) {
																eulerActual->h = float(eulerRaw->h) / 900;
																eulerActual->r = float(eulerRaw->r) / 900;
																eulerActual->p = float(eulerRaw->p) / 900;
								}
								else {
																return ERROR1;
								}

								return calres;
}

/**
 * Formats quaternion data
 * @method bno055_format_quaternion_data
 * @param  quaternionRaw                 [raw quaternion data]
 * @param  quaternionActual              [formatted quaternion data]
 * @return                               [result of calcualtion]
 */
BNO055_RETURN_FUNCTION_TYPE bno055_format_quaternion_data(struct bno055_quaternion * quaternionRaw, struct bno055_quaternion_float * quaternionActual) {
								BNO055_RETURN_FUNCTION_TYPE calres = BNO055_Zero_U8X;
								quaternionActual->w = float(quaternionRaw->w) / exp2f(14);
								quaternionActual->x = float(quaternionRaw->x) / exp2f(14);
								quaternionActual->y = float(quaternionRaw->y) / exp2f(14);
								quaternionActual->z = float(quaternionRaw->z) / exp2f(14);

								return calres;
}

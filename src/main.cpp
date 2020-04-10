#include <Arduino.h>
#include "BNO055_support.h"
#include "mbed.h"

void initBNO055(struct bno055_t * bno055_sensor);
uint8_t calibrateBNO055();

#define OUTPUT_DATA_RATE    100     //Output data rate in Hz
#define ASVM_THRESHOLD      2500    //asvm threshold: 2500mg = 24.525m/s^2
#define THETA_THRESHOLD     65      //theta threshold in degrees

struct bno055_t myBNO;

struct bno055_accel_float ax;
struct bno055_mag_float mag;
struct bno055_gyro_float gyr;
struct bno055_euler_float eul;
struct bno055_quaternion_float quat;

float asvm, theta;
unsigned long lastTime = 0;

mbed:: AnalogIn ain(P0_4);
float batteryLevel;

mbed::PwmOut red(P0_24);

void setup(){
		//Initialize the Serial Port to view information on the Serial Monitor
		Serial.begin(115200);
		while(!Serial) {};

		//Initialize I2C communication
		Wire.begin();
		Wire.setClock(400000);

		//Initialize BNO055 sensor
		initBNO055(&myBNO);
		calibrateBNO055();

		red.period(4.0f);
		red.write(1.0f);
}

void loop() {
		if ((millis() - lastTime) >= OUTPUT_DATA_RATE) //To stream at 10Hz without using additional timers
		{
				lastTime = millis();

				bno055_get_accel_data(&ax);
				asvm = sqrt(ax.x * ax.x + ax.y * ax.y + ax.z * ax.z);

				//Print accelerometer data to Serial Monitor
				Serial.print(asvm); Serial.print(" ");
				Serial.print(ax.x); Serial.print(" ");
				Serial.print(ax.y); Serial.print(" ");
				Serial.println(ax.z);

				//Fall Detection Algorithm
				if(asvm > ASVM_THRESHOLD) {
						int i = 0;
						while(i < 100) {
								if((millis() - lastTime) >= 12) {
										lastTime = millis();
										bno055_get_euler_data(&eul);
										theta += float(abs(eul.p));
										i++;
								}
						}
						theta /= 100;
						if(theta > THETA_THRESHOLD) {
								red.write(0.1f);
								Serial.println("Fall Detected");
						}
				}

				// /* Adafruit BNO055 bunny Processing sketch Serial data */
				// bno055_get_euler_data(&eul);
				// /* The processing sketch expects data as heading, pitch, roll */
				// Serial.print(F("Orientation: "));
				// Serial.print((float)eul.h);
				// Serial.print(F(" "));
				// Serial.print((float)eul.p);
				// Serial.print(F(" "));
				// Serial.print((float)eul.r);
				// Serial.println(F(""));
				//
				// /* Also send calibration data for each sensor. */
				// uint8_t sys, gyro, accel, mag = 0;
				// bno055_get_syscalib_status(&sys);
				// bno055_get_accelcalib_status(&accel);
				// bno055_get_gyrocalib_status(&gyro);
				// bno055_get_magcalib_status(&mag);
				// Serial.print(F("Calibration: "));
				// Serial.print(sys, DEC);
				// Serial.print(F(" "));
				// Serial.print(gyro, DEC);
				// Serial.print(F(" "));
				// Serial.print(accel, DEC);
				// Serial.print(F(" "));
				// Serial.println(mag, DEC);

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

		unsigned char acc_units = 1; // 0: m/s2, 1: mG
		unsigned char gyr_units = 0; // 0: dps, 1: rps
		unsigned char eul_units = 0; // 0: Degrees, 1: Radians
		unsigned char temp_units = 1; // 0: Celsius, 1: Farenheit
		//Configure units of BNO055 sensor data
		bno055_set_operation_mode(OPERATION_MODE_CONFIG);
		delayMicroseconds(1000);
		bno055_set_accel_unit(acc_units);
		bno055_set_gyro_unit(gyr_units);
		bno055_set_euler_unit(eul_units);
		bno055_set_temperature_unit(temp_units);
		bno055_set_axis_remap_value(REMAP_Y_Z);
		bno055_set_x_remap_sign(0); //Positive
		bno055_set_y_remap_sign(0); //Positive
		bno055_set_z_remap_sign(1); //Negative

		//Configuration to NDoF mode
		bno055_set_operation_mode(OPERATION_MODE_NDOF);

		delayMicroseconds(1000);

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

uint8_t calibrateBNO055() {
		uint8_t calibration = 0;
		uint8_t sys, gyro, accel, mag = 0;

		Serial.println("Calibrating system. Please wait...");
		if(gyro != 3) {
				Serial.println("Calibrate Gyroscope. Please place device on stable surface. DO NOT MOVE THE DEVICE.");
				while(gyro != 3) {
						bno055_get_gyrocalib_status(&gyro);
						Serial.print("Gyro Cal:"); Serial.println(gyro);
						delayMicroseconds(10000);
				}
				Serial.println("Gyroscope Calibrated!");
		}
		if(mag != 3) {
				Serial.println("Calibrate Magnetometer. Wave device in the air in a figure 8 shape.");
				while(mag != 3) {
						bno055_get_magcalib_status(&mag);
						Serial.print("Mag Cal:"); Serial.println(mag);
						delayMicroseconds(10000);
				}
				Serial.println("Magnetometer Calibrated!");
		}
		if(accel != 3) {
				Serial.println("Calibrate Accelerometer. Place device in various stable positions along 3 axes.");
				while(accel != 3) {
						bno055_get_accelcalib_status(&accel);
						Serial.print("Accel Cal:"); Serial.println(accel);
						delayMicroseconds(10000);
				}
				Serial.println("Accelerometer Calibrated!");
		}
		if(sys != 3) {
				Serial.println("Calibrate System. Place wait");
				while(sys != 3) {
						bno055_get_syscalib_status(&sys);
						Serial.print("Accel Cal:"); Serial.println(sys);
						delayMicroseconds(10000);
				}
				Serial.println("System Calibrated!");
		}
		calibration = sys;
		Serial.print("Calibration Complete: ");
		Serial.print(sys); Serial.print(" ");
		Serial.print(gyro); Serial.print(" ");
		Serial.print(accel); Serial.print(" ");
		Serial.println(mag);


		return calibration;
}

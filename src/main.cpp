#include <Arduino.h>
#include "BNO055_support.h"
#include "DebounceIn.h"
#include "ArduinoBLE.h"

void initBNO055(struct bno055_t * bno055_sensor);
uint8_t calibrateBNO055();
void togglePower(void);
void statusLED(void);
void powerMonitor(void);
void detectFall(void);
void hapticFeedback(uint32_t flags);
void resetSystem(void);
void manualTrigger(void);
void notifications(void);
void updateBLE(void);

#define SENSOR_READ_FLAG            (1UL << 0)
#define FALL_DETECTED_FLAG          (1UL << 1)
#define MANUAL_TRIGGER_FLAG         (1UL << 2)
#define RESET_FLAG                  (1UL << 3)
#define BLE_MESSAGE_FLAG            (1UL << 4)

rtos::EventFlags event_flags;

//Sensor and Fall Detection Parameters
#define OUTPUT_DATA_RATE    100     //Output data rate in Hz
#define ASVM_THRESHOLD      2500    //asvm threshold: 2500mg = 24.525m/s^2
#define THETA_THRESHOLD     65      //theta threshold in degrees

#define POWER_BUTTON        P1_11   //PinName for Power button
#define RESET_BUTTON        P1_12   //PinName for Reset button
#define MANUAL_TRIGGER      P1_13   //PinName for Manual Trigger button
#define MOTOR_PIN           P1_8

#define LED_RED             P0_24   //PinName for Red LED
#define LED_GREEN           P0_16   //PinName for Green LED
#define LED_BLUE            P0_6    //PinName for Blue LED

enum power_state {POWER_OFF=0, POWER_ON, CHARGING, CHARGED};

struct bno055_t myBNO;

struct bno055_accel_float ax;
struct bno055_mag_float mag;
struct bno055_gyro_float gyr;
struct bno055_euler_float eul;
struct bno055_quaternion_float quat;

uint8_t powerState = 0;
uint8_t systemStatus = 0;
rtos::Semaphore one_slot(1);
mbed::LowPowerTimeout timeout;

//Custom BLE GATT Service
BLEService alertService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedIntCharacteristic notificationChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);
BLEDescriptor notificationDescriptor("2904", "Fall Detection Alert");
uint8_t btValue = 0x0;

float asvm, theta;
unsigned long lastTime = 0;

mbed:: AnalogIn batIn(P0_4);
float batteryLevel;

mbed::DigitalOut red(LED_RED);
mbed::DigitalOut green(LED_GREEN);
mbed::DigitalOut blue(LED_BLUE);
mbed::DigitalOut blink(P0_13);
mbed::PwmOut motor(MOTOR_PIN);
DebounceIn pbut(POWER_BUTTON, PullUp, &one_slot);
DebounceIn reset(RESET_BUTTON, PullUp, &one_slot);
DebounceIn trigger(MANUAL_TRIGGER, PullUp, &one_slot);


rtos::Thread sensor;
rtos::Thread powerMan;
rtos::Thread notif;

void setup(){
		//Initalize variables and system states
		powerState = POWER_ON;
		systemStatus = powerState;
		red.write(1);
		green.write(1);
		blue.write(1);
		pbut.assert_low_long(togglePower);
		reset.assert_fall(resetSystem);
		trigger.assert_fall(manualTrigger);
		event_flags.clear();
		event_flags.set(SENSOR_READ_FLAG);
		motor.period(4.0f);

		//Begin Bluetooth initialization
		if (!BLE.begin()) {
				Serial.println("Starting BLE failed!");
				while (1);
		}

		BLE.setDeviceName("FallDetect");
		BLE.setAppearance(0x512);
		BLE.setLocalName("FallDetect");
		BLE.setAdvertisedService(alertService);
		alertService.addCharacteristic(notificationChar);
		BLE.addService(alertService);
		notificationChar.writeValue(btValue);
		notificationChar.addDescriptor(notificationDescriptor);
		//Start advertising
		BLE.advertise();

		//Initialize the Serial Port to view information on the Serial Monitor
		Serial.begin(115200);
		while(!Serial) {};

		//Initialize I2C communication
		Wire.begin();
		Wire.setClock(400000);

		//Initialize BNO055 sensor
		initBNO055(&myBNO);
		//calibrateBNO055();

		//Start worker threads
		sensor.start(detectFall);
		powerMan.start(powerMonitor);
		notif.start(notifications);
}

void loop() {
		//Main BLE notification thread
		BLEDevice central = BLE.central();

		uint8_t oldValue;
		notificationChar.readValue(oldValue);
		if(btValue != oldValue && systemStatus != POWER_OFF) {
				notificationChar.writeValue(btValue);
		}
		rtos::ThisThread::sleep_for(500);
		// 		/* Adafruit BNO055 bunny Processing sketch Serial data */
		// 		bno055_get_euler_data(&eul);
		// 		/* The processing sketch expects data as heading, pitch, roll */
		// 		Serial.print(F("Orientation: "));
		// 		Serial.print((float)eul.h);
		// 		Serial.print(F(" "));
		// 		Serial.print((float)eul.p);
		// 		Serial.print(F(" "));
		// 		Serial.print((float)eul.r);
		// 		Serial.println(F(""));
		//
		// 		/* Also send calibration data for each sensor. */
		// 		uint8_t sys, gyro, accel, mag = 0;
		// 		bno055_get_syscalib_status(&sys);
		// 		bno055_get_accelcalib_status(&accel);
		// 		bno055_get_gyrocalib_status(&gyro);
		// 		bno055_get_magcalib_status(&mag);
		// 		Serial.print(F("Calibration: "));
		// 		Serial.print(sys, DEC);
		// 		Serial.print(F(" "));
		// 		Serial.print(gyro, DEC);
		// 		Serial.print(F(" "));
		// 		Serial.print(accel, DEC);
		// 		Serial.print(F(" "));
		// 		Serial.println(mag, DEC);
		//
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

/**
 * Calibrate BNO055 Sensor
 * @method calibrateBNO055
 * @return system calibration level
 */
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

/**
 * Monitor system and battery power states and adjust status indicator LED
 * @method powerMonitor
 */
void powerMonitor(void) {
		while(true) {
				batteryLevel = batIn.read();
				batteryLevel = batteryLevel * 3.3 * 2;
				if(batteryLevel > 4) {
						systemStatus = CHARGING;
				}else {
						systemStatus = powerState;
				}
				statusLED();
				rtos::ThisThread::sleep_for(2000);
		}
}

/**
 * System Notification handler
 * @method notifications
 */
void notifications(void) {
		while(true) {
				event_flags.wait_any(SENSOR_READ_FLAG, osWaitForever, false);
				uint32_t _flags = event_flags.wait_any(FALL_DETECTED_FLAG | MANUAL_TRIGGER_FLAG, osWaitForever, false);
				hapticFeedback(_flags);
				rtos::ThisThread::sleep_for(3000);
		}
}

/**
 * Toggler system power states and peripheral devices
 * @method togglePower
 */
void togglePower(void) {
		if(powerState == POWER_ON) {
				powerState = POWER_OFF;
				event_flags.clear();
				bno055_set_powermode(POWER_MODE_SUSPEND);
				delay(20);
		}else {
				powerState = POWER_ON;
				bno055_set_powermode(POWER_MODE_NORMAL);
				bno055_set_operation_mode(OPERATION_MODE_NDOF);
				delay(20);
				event_flags.set(SENSOR_READ_FLAG);
		}
}

/**
 * Manual system variable and state reset
 * NOT AN ACTUAL HARDWARE RESET
 * @method resetSystem
 */
void resetSystem(void) {
		if(systemStatus != POWER_OFF) {
				event_flags.clear();
				event_flags.set(SENSOR_READ_FLAG | RESET_FLAG);
		}
}

/**
 * External button trigger handler
 * @method manualTrigger
 */
void manualTrigger(void) {
		if(systemStatus != POWER_OFF) {
				event_flags.set(MANUAL_TRIGGER_FLAG);
				timeout.attach(mbed::callback(updateBLE), 15);
		}
}

/**
 * Internal system flag handler
 * @method updateBLE
 */
void updateBLE(void) {
		btValue = (uint8_t)(event_flags.get() & (FALL_DETECTED_FLAG | MANUAL_TRIGGER_FLAG));
		blink = !blink;
}

/**
 * Status LED handler
 * @method statusLED
 */
void statusLED(void) {
		switch(systemStatus) {
		case 1: red.write(1);
				green.write(1);
				blue.write(0);
				break;
		case 2: red.write(0);
				green.write(1);
				blue.write(1);
				break;
		case 3: red.write(1);
				green.write(0);
				blue.write(1);
				break;
		default: red.write(1);
				green.write(1);
				blue.write(1);
				break;
		}
}

/**
 * Sensor read function and fall detection handler. Implements basic threshold based
 * fall detection algorithm. Threshold values can be modified in variables
 * at the start of this file.
 * @method detectFall
 */
void detectFall(void) {
		while(true) {
				event_flags.wait_all(SENSOR_READ_FLAG, osWaitForever, false);
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
								event_flags.set(FALL_DETECTED_FLAG);
								timeout.attach(mbed::callback(updateBLE), 15);
						}
				}
				rtos::ThisThread::sleep_for(100);
				one_slot.release();
		}
}

/**
 * Haptic feedback motor handler
 * @method hapticFeedback
 * @param  flags          [description]
 */
void hapticFeedback(uint32_t flags) {
		if((flags & FALL_DETECTED_FLAG)  == FALL_DETECTED_FLAG) {
				//indicate to user fall detected
				motor.write(0.8f);
				delay(5000);
				motor.write(0.0f);
		}else if((flags & MANUAL_TRIGGER_FLAG) == MANUAL_TRIGGER_FLAG) {
				//indicate to user manual trigger detected
				motor.write(0.8f);
				delay(3000);
				motor.write(0.0f);
				delay(3000);
				motor.write(0.8f);
				delay(3000);
				motor.write(0.0f);
		}
}

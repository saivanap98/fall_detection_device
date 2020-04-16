/*
 * myFirstLibrary.h - An introduction to library setup
 * Created by Christian @ Core Electronics on 1/06/18
 * Revision #5 - See readMe
 */

#ifndef DEBOUNCEIN_H
#define DEBOUNCEIN_H

#define RISE_MASK_16 0b0000000000111111
#define FALL_MASK_16 0b1111100000000000
#define HIGH_MASK_16 0b1111111111111111
#define LOW_MASK_16  0b0000000000000000

#define START_THREAD 1;

#include "Arduino.h"

class DebounceIn: public mbed::DigitalIn {
public:
		DebounceIn(PinName pin, MbedPinMode mode, rtos::Semaphore* sm);
		void set_samples(int samples);
		void assert_low(mbed::Callback<void()> function);
		void assert_high(mbed::Callback<void()> function);
		void assert_high_long(mbed::Callback<void()> function);
		void assert_low_long(mbed::Callback<void()> function);
		void assert_rise(mbed::Callback<void()> function);
		void assert_fall(mbed::Callback<void()> function);
		int read(void);
		int status;
	#ifdef MBED_OPERATORS
		operator int() {
				return read();
		}
	#endif
protected:
		void set_debounce_us(int time);
		void _callback();
		mbed::LowPowerTicker _ticker;
		mbed::Callback<void()> _callback_low;
		mbed::Callback<void()> _callback_high;
		mbed::Callback<void()> _callback_high_long;
		mbed::Callback<void()> _callback_low_long;
		mbed::Callback<void()> _callback_rise;
		mbed::Callback<void()> _callback_fall;
		uint8_t _shadow;
		int _counter;
		int _samples;
		uint16_t history;
		rtos::Thread _thread;
		rtos::Semaphore* _sm;


private:
		int flag;
};

#endif

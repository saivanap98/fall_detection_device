/**
 * This project was updated and modified based off the existing library by
 * PinDetect by Alex Kirkham
 * @see https://os.mbed.com/users/AjK/code/PinDetect/
 *
 * DebounceIn is used to debounce mechanical swithces and implements
 * Interrupt style callbacks for short and long button presses.
 *
 * This is done by collecting a button's state history and comparing it to
 * rise, fall, and hold patterns. When a pattern matches, an associated callback
 * handler is called. Bit masking and bit wise operations make for fast
 * and efficient pattern matching and debouncing.
 *
 * '0's replaces 'X's in the bit mask templates which represent times where
 * a switch may bounce. Bit masks can be tuned to a button's behavior by varrying
 * the number of 'X's which accounts for the length of bounching as well as
 * random glitches during held down or open states.
 *
 * The library also uses an RTOS thread for the running the debounce routine.
 */
#ifndef DEBOUNCEIN_H
#define DEBOUNCEIN_H

#define RISE_MASK_16 0b0000000000111111     ///<0b00000XXXXX111111>
#define FALL_MASK_16 0b1111100000000000     ///<0b11111XXXXX000000>
#define HIGH_MASK_16 0b1111111111111111     ///<0b1111111111111111>
#define LOW_MASK_16  0b0000000000000000     ///<0b0000000000000000>

#include "Arduino.h"


class DebounceIn: public mbed::DigitalIn {
public:
		DebounceIn(PinName pin, MbedPinMode mode, rtos::Semaphore* sm);
		~DebounceIn(void);
		void set_samples(int samples);
		void assert_low(mbed::Callback<void()> function);
		void assert_high(mbed::Callback<void()> function);
		void assert_high_long(mbed::Callback<void()> function);
		void assert_low_long(mbed::Callback<void()> function);
		void assert_rise(mbed::Callback<void()> function);
		void assert_fall(mbed::Callback<void()> function);
		int read(void);
	#ifdef MBED_OPERATORS
		operator int() {
				return read();
		}
	#endif
protected:
		void _callback();
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
};

#endif

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
#include "DebounceIn.h"
/**
 * DebounceIn constructor
 * @method DebounceIn::DebounceIn
 * @param  pin                    [PinName that supports DigitalIn]
 * @param  mode                   [MbedPinMode: PullUp, PullDown, PullNone, OpenDrain]
 * @param  sm                     [Semaphore pointer for thread synchronization]
 */
DebounceIn::DebounceIn(PinName pin, MbedPinMode mode, rtos::Semaphore* sm) : mbed::DigitalIn(pin, mode) {
		_counter = 0;
		_samples = 200;
		history = 0;
		_sm = (rtos::Semaphore*)(sm);
		_thread.start(mbed::callback(this, &DebounceIn::_callback));
}
/**
 * DebounceIn destructor
 * @method DebounceIn
 */
DebounceIn::~DebounceIn(void) {
		_thread.terminate();
}

/**
 * Set number of samples needed before long press is detected
 * @method DebounceIn::set_samples
 * @param  samples                 [description]
 */
void DebounceIn::set_samples(int samples) {
		_samples = samples;
}

/**
 * Callback handler for low state
 * @method DebounceIn::assert_low
 * @param  [function]                 [callback function]
 */
void DebounceIn::assert_low(mbed::Callback<void()> function) {
		if(function) {
				_callback_low = function;
		}
}

/**
 * Callback handler for high state
 * @method DebounceIn::assert_high
 * @param  [function]                  [callback function]
 */
void DebounceIn::assert_high(mbed::Callback<void()> function) {
		if(function) {
				_callback_high = function;
		}
}

/**
 * Callback handler for long press high state
 * @method DebounceIn::assert_high_long
 * @param  [function]                       [callback function]
 */
void DebounceIn::assert_high_long(mbed::Callback<void()> function) {
		if(function) {
				_callback_high_long = function;
		}
}

/**
 * Callback handler for long press low state
 * @method DebounceIn::assert_low_long
 * @param  [function]                      [callback function]
 */
void DebounceIn::assert_low_long(mbed::Callback<void()> function) {
		if(function) {
				_callback_low_long = function;
		}
}

/**
 * Callback for on button rise
 * @method DebounceIn::assert_rise
 * @param  [function]                  [callback function]
 */
void DebounceIn::assert_rise(mbed::Callback<void()> function) {
		if(function) {
				_callback_rise = function;
		}
}

/**
 * Callback for button fall
 * @method DebounceIn::assert_fall
 * @param  [function]                  [callback function]
 */
void DebounceIn::assert_fall(mbed::Callback<void()> function) {
		if(function) {
				_callback_fall = function;
		}
}

/**
 * Read the value of the button
 * @method DebounceIn::read
 * @return the value of the button
 */
int DebounceIn::read() {
		return _shadow;
}

/**
 * Debounce handler and thread worker - Default sampling rate is 10ms
 * @method DebounceIn::_callback
 */
void DebounceIn::_callback() {
		while(true) {
				history = history << 1;
				history |= DigitalIn::read();

				if((history & RISE_MASK_16) == RISE_MASK_16) {
						if(_callback_rise) _callback_rise.call();
						_counter = 0;
				}else if((history & FALL_MASK_16) == FALL_MASK_16) {
						if(_callback_fall) _callback_fall.call();
						_counter = 0;
				}else if((history & LOW_MASK_16) == LOW_MASK_16) {
						_shadow = 0;
						if(_counter >= 0) _counter++;
						if(_callback_low) _callback_low.call();
						if(_counter > _samples) {
								_counter = -1;
								if(_callback_low_long) _callback_low_long.call();
						}
				}else if((history & HIGH_MASK_16) == HIGH_MASK_16) {
						_shadow = 1;
						if(_counter >= 0) _counter++;
						if(_callback_high) _callback_high.call();
						if(_counter > _samples) {
								_counter = -1;
								if(_callback_high_long) _callback_high_long.call();
						}
				}

				rtos::ThisThread::sleep_for(10);
		}
}

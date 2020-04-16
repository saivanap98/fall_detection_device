#include "DebounceIn.h"

DebounceIn::DebounceIn(PinName pin, MbedPinMode mode, rtos::Semaphore* sm) : mbed::DigitalIn(pin, mode) {
		_counter = 0;
		_samples = 200;
		history = 0;
		flag = 0;
		_sm = (rtos::Semaphore*)(sm);
		_thread.start(mbed::callback(this, &DebounceIn::_callback));
}



void DebounceIn::set_samples(int samples) {
		_samples = samples;
}

void DebounceIn::set_debounce_us(int time) {
		// _ticker.attach_us(mbed::callback(this, &DebounceIn::_callback), time);
}

void DebounceIn::assert_low(mbed::Callback<void()> function) {
		if(function) {
				_callback_low = function;
		}
}
void DebounceIn::assert_high(mbed::Callback<void()> function) {
		if(function) {
				_callback_high = function;
		}
}
void DebounceIn::assert_high_long(mbed::Callback<void()> function) {
		if(function) {
				_callback_high_long = function;
		}
}
void DebounceIn::assert_low_long(mbed::Callback<void()> function) {
		if(function) {
				_callback_low_long = function;
		}
}
void DebounceIn::assert_rise(mbed::Callback<void()> function) {
		if(function) {
				_callback_rise = function;
		}
}
void DebounceIn::assert_fall(mbed::Callback<void()> function) {
		if(function) {
				_callback_fall = function;
		}
}
int DebounceIn::read() {
		return _shadow;
}

void DebounceIn::_callback() {
		while(true) {
				history = history << 1;
				history |= DigitalIn::read();

				if((history & RISE_MASK_16) == 0b0000000000111111) {
						if(_callback_rise) _callback_rise.call();
						_counter = 0;
				}else if((history & FALL_MASK_16) == 0b1111100000000000) {
						if(_callback_fall) _callback_fall.call();
						_counter = 0;
				}else if((history & LOW_MASK_16) == 0b0000000000000000) {
						_shadow = 0;
						flag = 0;
						if(_counter >= 0) _counter++;
						if(_callback_low) _callback_low.call();
						if(_counter > _samples) {
								flag = 1;
								_counter = -1;
								if(_callback_low_long) _callback_low_long.call();
						}
				}else if((history & HIGH_MASK_16) == 0b1111111111111111) {
						_shadow = 1;
						flag = 0;
						if(_counter >= 0) _counter++;
						if(_callback_high) _callback_high.call();
						if(_counter > _samples) {
								flag = 1;
								_counter = -1;
								if(_callback_high_long) _callback_high_long.call();
						}
				}

				rtos::ThisThread::sleep_for(10);
		}
}

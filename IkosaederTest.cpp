/* (c) 2015-03-30 Jens Hauke <jens.hauke@4k2.de>   -*- linux-c -*- */
#ifndef ARDUINO

#include <iostream>
#include <string>
#include "stdint.h"
#include <bitset>
#include <sstream>
#include "/usr/share/arduino/hardware/arduino/cores/arduino/binary.h"

using namespace std;

#define HOST_TEST 1

void delay(int ms) {
}

#define INPUT 0
void pinMode(int pin, int state) {
}

// Registers
uint8_t DDRB = 0;
uint8_t DDRD = 0;
uint8_t PORTB = 0;
uint8_t PORTD = 0;

string binary(uint8_t num) {
	bitset<8> x(num);
	stringstream out;
	out << x;
	return out.str();
}

// Set (D7 D6 D5 D4 D3 D9 D8) to lo, hi or tri-state
void set_tri(uint8_t lo_hi, uint8_t in_out) {
	char buffer [33];
	cout << "set_tri " << binary(lo_hi) << " " << binary(in_out) << endl;
}

class ASerial {
public:
	template <typename T>
	void print(T i) {
		cout << i;
	};

	template <typename T>
	void println(T i) {
		cout << i << endl;
	};

	template <typename T>
	void write(T val) {
		cout << val;
	};

	bool available(void) {
		return false;
	}

	char read(void) {
		return -1;
	}
};

ASerial Serial;

unsigned long millis_now = 0;
unsigned long millis(void) {
	return millis_now++;
}

#include "Ikosaeder.ino"


int main(int argc, char **argv)
{
	for (int i = 0; i < 100; i++) {
		loop();
		millis_now += 1000;
	}
	return 0;
}

#endif // ARDUINO

/*
 * Local Variables:
 *  compile-command: "g++ IkosaederTest.cpp -Wall -W -Wno-unused -Wno-unused-parameter -O2 -o IkosaederTest && ./IkosaederTest"
 * End:
 *
 */

//					-*- c++ -*-
// (c) 2015-03-29 Jens Hauke <jens.hauke@4k2.de>



void setup() {
	// Setup D2 - D9 as Input
	for (int8_t i = 2; i <= 9; i++) {
		pinMode(i, INPUT);
	}
}


// Set (D7 D6 D5 D4 D3 D9 D8) to lo, hi or tri-state
void set_tri(uint8_t lo_hi, uint8_t in_out) {
	uint8_t portb, portd, ddrb, ddrd;
	// Keep the state of the unused pins
	ddrd = (DDRD & B00000011) | (in_out & B11111100); // 1: Output, 0: Input
	ddrb = (DDRB & B11111100) | (in_out & B00000011);
	portd = (PORTD & B00000011) | (lo_hi & B11111100); // 0: lo, 1: hi
	portb = (PORTB & B11111100) | (lo_hi & B00000011);

	// Set registers
	PORTB = portb;
	DDRB = ddrb;
	PORTD = portd;
	DDRD = ddrd;
}


void loop() {
	set_tri(B11110000, B11111111); // hi and lo
	delay(1000);
	set_tri(B00001111, B11111111); // lo and hi
	delay(1000);
	set_tri(B00000000, B00000000); // All tri-state
	delay(1000);
}

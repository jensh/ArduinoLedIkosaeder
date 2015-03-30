//					-*- c++ -*-
// (c) 2015-03-29 Jens Hauke <jens.hauke@4k2.de>
//
// With (D7 D6 D5 D4 D3 D2 D9 D8)
// build a lo, hi, tri-state matrix.
// (Ikosaeder uses only 7 bits: (0 D6 D5 D4 D3 D2 D9 D8)

static
void help(void);

void setup() {
	// Setup D2 - D9 as Input
	for (int8_t i = 2; i <= 9; i++) {
		pinMode(i, INPUT);
	}

	Serial.begin(115200);
	// Serial.begin(9600);
	help();
}


#ifndef HOST_TEST
// Set (D7 D6 D5 D4 D3 D2 D9 D8) to lo, hi or tri-state
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
#endif

void blink_hi_lo() {
	set_tri(B11110000, B11111111); // hi and lo
	delay(1000);
	set_tri(B00001111, B11111111); // lo and hi
	delay(1000);
	set_tri(B00000000, B00000000); // All tri-state
	delay(1000);
}


// Switch on @led. Using @nbits Pins.
void set_tri_led(uint8_t nbits, int led) {
	uint8_t lo_bit = led / (nbits - 1);
	uint8_t hi_bit = (led + lo_bit) % (nbits - 1);
	if (hi_bit >= lo_bit) hi_bit++;

	// bit pos to bitmask:
	lo_bit = 1 << lo_bit;
	hi_bit = 1 << hi_bit;

	// Hi: only hi_bit, Output: lo_bit and hi_bit
	set_tri(hi_bit, lo_bit | hi_bit);
}


void tri_loop(uint8_t nbits) {
	int count = nbits * (nbits - 1);
	int i;
	for (i = 0; i < count; i++) {
		set_tri_led(nbits, i);
		delay(5);
	}
}


int cmd_number = 0;
int cmd_number2 = 0;
bool cmd_number_set = false;
bool cmd_number_neg = false;

#define ANIMATION0_TRILOOP 0
#define ANIMATION1_NONE 1
int animation = 0;
int num_pins = 7;

static
void help(void) {
	Serial.println("\nIkosaeder controller.");

	Serial.println("? : Help");
	Serial.println("<num>a : Set animation");
	Serial.println("<led number>s : Switch on led");
	Serial.println("[<in_out mask>,]<lo_hi mask>t : Set tri-state");
	Serial.println("[<num>]# : number of used pins");
}

static
void SerialComm(void) {
	while (Serial.available()) {
		bool number_cmd = false;
		uint8_t cmd;

		cmd = Serial.read();

		switch (cmd) {
		case 'a': // Animation cmd_number
			if (cmd_number_set) {
				animation = cmd_number;
			} else {
				Serial.print(animation); Serial.write('a');
			}
			break;
		case 's':
			set_tri_led(num_pins, cmd_number);
			animation = ANIMATION1_NONE;
			break;
		case 't':
			set_tri(cmd_number, cmd_number2);
			animation = ANIMATION1_NONE;
			break;
		case '#':
			if (cmd_number_set) {
				num_pins = cmd_number;
			} else {
				Serial.print(num_pins); Serial.write('#');
			}
			break;
		case ',':
			cmd_number2 = cmd_number;
			break;
		case '?':
			help();
			break;
		case '-':
			// Set cmd_number negative
			cmd_number_neg = true;
			cmd_number = 0;
			number_cmd = true;
			break;
		case '0'...'9':
			// Set cmd_number
			cmd_number *= 10;
			cmd_number += cmd_number_neg ? '0' - cmd : cmd - '0';
			cmd_number_set = true;
			number_cmd = true;
			break;
		case '\n': // LF (VT102: ^j)
			Serial.print("\r\n");
			break;
		case '\r': // CR (VT102: ^m or Enter)
			Serial.print("\r\n");
			break;
		default:
			Serial.println("Unknown cmd.");
			Serial.println("Expect one of '-0123456789,?ast#'");
			break;
		}

		if (!number_cmd) {
			cmd_number = 0;
			cmd_number_set = false;
			cmd_number_neg = false;
		}
	}
}


void loop() {
	switch (animation) {
	case ANIMATION0_TRILOOP:
		tri_loop(7);
		break;
	case ANIMATION1_NONE:
		// Nothing. Keep state.
		break;
	default:
		Serial.print("\"Unknown animation ");
		Serial.print(animation);
		Serial.println(". Using 0 now.\"");
		animation = ANIMATION0_TRILOOP;
		break;
	}
	SerialComm();
}

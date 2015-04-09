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


#define CMD_MAX_NUMBERS 3
int cmd_number[CMD_MAX_NUMBERS];
uint8_t cmd_numbers = 0;
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
	delay(200);
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
			if (cmd_numbers) {
				animation = cmd_number[0];
			} else {
				Serial.print(animation); Serial.write('a');
			}
			break;
		case 's':
			if (cmd_numbers) {
				set_tri_led(num_pins, cmd_number[0]);
			} else {
				set_tri(0, 0); // All input. (Switch off)
			}
			animation = ANIMATION1_NONE;
			break;
		case 't':
			set_tri(cmd_number[1], cmd_number[0]);
			animation = ANIMATION1_NONE;
			break;
		case '#':
			if (cmd_numbers) {
				num_pins = cmd_number[0];
			} else {
				Serial.print(num_pins); Serial.write('#');
			}
			break;
		case 'p': // debug cmd_numbers
			Serial.print(cmd_numbers); Serial.write(',');
			Serial.print(cmd_number[0]); Serial.write(',');
			Serial.print(cmd_number[1]); Serial.write(',');
			Serial.print(cmd_number[2]); Serial.write('p');
			break;
		case ',':
			cmd_number[cmd_numbers] = 0;
			if (cmd_numbers < CMD_MAX_NUMBERS) cmd_numbers++;
			cmd_number_neg = false;
			number_cmd = true;
			break;
		case '?':
			help();
			break;
		case '-':
			// Set cmd_number negative
			cmd_number_neg = true;
			number_cmd = true;
			break;
		case '0'...'9':
			// Set cmd_number
			if (!cmd_numbers) {
				cmd_number[0] = 0;
				cmd_numbers = 1;
			}
			cmd_number[cmd_numbers - 1] *= 10;
			cmd_number[cmd_numbers - 1] += cmd_number_neg ? '0' - cmd : cmd - '0';
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
			cmd_numbers = 0;
			cmd_number_neg = false;
		}
	}
}


void loop() {
	switch (animation) {
	case ANIMATION0_TRILOOP:
		tri_loop(num_pins);
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

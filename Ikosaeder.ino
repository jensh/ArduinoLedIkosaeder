//					-*- c++ -*-
// (c) 2015-03-29 Jens Hauke <jens.hauke@4k2.de>
//
// With (D7 D6 D5 D4 D3 D2 D9 D8)
// build a lo, hi, tri-state matrix.
// (Ikosaeder uses only 7 bits: (0 D6 D5 D4 D3 D2 D9 D8)

static
void help(void);

#ifndef HOST_TEST
void setup() {
	// Setup D2 - D9 as Input
	for (int8_t i = 2; i <= 9; i++) {
		pinMode(i, INPUT);
	}

	Serial.begin(115200);
	// Serial.begin(9600);
	help();
}


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


uint8_t led_map[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
uint8_t led_pos = 0;
uint16_t led_delay = 0;

void led_map_step(void) {
	uint8_t cnt = 0;
	uint8_t lmap;
	uint8_t lopin_mask;
	while (!((lmap = led_map[led_pos]))) {
		if (cnt++ >= 8) {
			 // empty map. Switch off.
			set_tri(0, 0);
			return;
		}
		led_pos = (led_pos + 1) & 7;
	}
	lopin_mask = 1 << led_pos;
	set_tri(lmap & ~lopin_mask, lmap | lopin_mask);
	led_pos = (led_pos + 1) & 7;
	if (led_delay) delay(led_delay);
}


void led_map_set(uint8_t row, uint8_t mask) {
	led_map[row & 7] = mask; // bit @row must be set to 0!
}


void led_map_led_set(uint8_t led_id, bool on = true) {
	static struct Mapping {
		unsigned row : 3;
		unsigned bit : 3;
	} mapping[42] = {
		{1, 6}, // 0  // level 2 x
		{6, 2}, // 1  // level 4 x
		{0, 1}, // 2  // level 1 x
		{1, 3}, // 3  // level 2 x
		{2, 1}, // 4  // level 3 x
		{3, 2}, // 5  // level 3 x
		{2, 5}, // 6  // level 4 x
		{2, 3}, // 7  // level 5 x
		{6, 3}, // 8  // level 2 x
		{5, 6}, // 9  // level 4 x
		{3, 0}, // 10 // level 1 x
		{3, 1}, // 11 // level 2 x
		{5, 3}, // 12 // level 3 x
		{1, 5}, // 13 // level 3 x
		{5, 4}, // 14 // level 4 x
		{3, 5}, // 15 // level 5 x
		{6, 1}, // 16 // level 2 x
		{4, 6}, // 17 // level 4 x
		{1, 0}, // 18 // level 1 x
		{5, 1}, // 19 // level 2 x
		{1, 4}, // 20 // level 3 x
		{4, 5}, // 21 // level 3 x
		{0, 4}, // 22 // level 4 x
		{4, 3}, // 23 // level 5 x
		{6, 5}, // 24 // level 2 x
		{6, 0}, // 25 // level 4 x
		{0, 5}, // 26 // level 1 x
		{5, 2}, // 27 // level 2 x
		{5, 0}, // 28 // level 3 x
		{0, 2}, // 29 // level 3 x
		{4, 0}, // 30 // level 4 x
		{0, 3}, // 31 // level 5 x
		{2, 6}, // 32 // level 2 x
		{6, 4}, // 33 // level 4 x
		{2, 0}, // 34 // level 1 x
		{1, 2}, // 35 // level 2 x
		{2, 4}, // 36 // level 3 x
		{4, 1}, // 37 // level 3 x
		{4, 2}, // 38 // level 4 x
		{3, 4}, // 39 // level 5 x
		{0, 6}, // 40 // level 0 x
		{3, 6}, // 41 // level 6 x
	};

#ifdef HOST_TEST
	cout << "Led " << (int)led_id << " mapping: " << mapping[led_id].row <<  ", " << mapping[led_id].bit << endl;
#endif
	if (on) {
		led_map[mapping[led_id].row] |= 1 << mapping[led_id].bit;
	} else {
		led_map[mapping[led_id].row] &= ~(1 << mapping[led_id].bit);
	}
}


void led_map_clear(void) {
	for (int i = 0; i < 8; i++) led_map[i] = 0;
}


void led_map_dump() {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		Serial.print(i); Serial.write(','); Serial.print(led_map[i]); Serial.write("m\r\n");
	}
}


/*
 * get_peer functions:
 */
uint8_t get_peer_count(uint8_t id) {
	return (id % 8 < 2) ? 5 : 6;
}

uint8_t get_peer(uint8_t id, uint8_t dir) {
	static uint8_t num3_peers_a[][6] = {
		{ 2, 3, 4, 5+32, 3+32 }, // 0
		{ 4, 5, 6, 7, 6+32 }, // 1
		{ 40, 2+8, 3, 0, 3+32, 2+32 }, // 2
		{ 2, 2+8, 0+8, 5, 4, 0 }, // 3
		{ 0, 3, 5, 1, 6+32, 5+32 }, // 4
		{ 3, 0+8, 4+8, 6, 1, 4 }, // 5
		{ 5, 4+8, 1+8, 7+8, 7, 1 }, // 6
		{ 41, 7+32, 6+32, 1, 6, 7+8 } // 7
	};

	switch (id) {
	case 40: return (2 + 32) - dir * 8;
	case 41: return 7 + dir * 8;
	default:
		uint8_t sid = id % 8; // 0..7
		uint8_t grp = (id - sid); // 0,8,16,24,32
		uint8_t res = num3_peers_a[sid][dir];
		if (res < 40) res = (res + grp) % 40;
		return res;
	}
}


uint8_t get_peer_dir(uint8_t id, uint8_t dir) {
	static uint8_t dirs[][6] = {
		{ 3, 5, 0, 1, 2 } ,
		{ 3, 4, 5, 3, 2 } ,
		{ 6 /* marker */, 5, 0, 0, 1, 1 } ,
		{ 2, 4, 4, 0, 1, 1 } ,
		{ 2, 4, 5, 0, 1, 2 } ,
		{ 3, 3, 5, 0, 1, 2 } ,
		{ 3, 4, 4, 2, 4, 2 } ,
		{ 6 /* marker */, 5, 3, 3, 4, 1 } ,
	};

	switch (id) {
	case 40:
	case 41: return 0;
	default:
		uint8_t sid = id % 8; // 0..7
		uint8_t res = dirs[sid][dir];
		if (res == 6) {
			uint8_t grpidx = id / 8; // group index 0..4
			res = (sid == 2) ? 4 - grpidx : grpidx;
		}
		return res;
	}
}


void swirl_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < 2000) return;
	last = now;

	led_map_clear();

	static uint8_t c_id = 0;

	led_map_led_set(c_id, true);

	c_id = (c_id + 1) % 42;
}


void swirl2_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < 30) return;
	last = now;

	led_map_clear();

	static uint8_t c_id = 0;
	static uint8_t c_dir = 0;
	static uint32_t cnt = 0;

	uint8_t p_peer_count = get_peer_count(c_id);

	c_dir = (c_dir + 3) % p_peer_count;
	if (cnt++ % 7 == 0) {
		c_dir = (c_dir + (5 * 6 - 1)) % p_peer_count;
	}

	uint8_t c_id_next = get_peer(c_id, c_dir);
	uint8_t c_dir_next = get_peer_dir(c_id, c_dir);

	c_id = c_id_next;
	c_dir = c_dir_next;

	led_map_led_set(c_id, true);
}



#define CMD_MAX_NUMBERS 3
int cmd_number[CMD_MAX_NUMBERS];
uint8_t cmd_numbers = 0;
bool cmd_number_neg = false;

#define ANIMATION0_TRILOOP 0
#define ANIMATION1_NONE 1
#define ANIMATION2_LEDMAP 2
#define ANIMATION3_SWIRL 3
#define ANIMATION4_SWIRL2 4

int animation = ANIMATION4_SWIRL2;
int num_pins = 7;

static
void help(void) {
	Serial.println("\nIkosaeder controller.");

	Serial.println("? : Help");
	Serial.println("<num>a : Set animation");
	Serial.println("<led number>s : Switch on led");
	Serial.println("[<num>]# : number of used pins for s");
	delay(200);
	Serial.println("[<in_out mask>,]<lo_hi mask>t : Set tri-state");
	Serial.println("<row>,<mask>m : led map");
	Serial.println("<delay>d : delay for each led map row");
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
		case 'd': // led delay for ledmap <delay>
			if (cmd_numbers) {
				led_delay = cmd_number[0];
			} else {
				Serial.print(led_delay); Serial.write('d');
			}
			break;
		case 'm': // set map <row>, <mask>
			if (cmd_numbers) {
				led_map_set(cmd_number[0], cmd_number[1]);
				animation = ANIMATION2_LEDMAP;
			} else {
				// dump map
				led_map_dump();
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
			Serial.println("Expect one of '-0123456789,?admst#'");
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
	case ANIMATION2_LEDMAP:
		led_map_step();
		break;
	case ANIMATION3_SWIRL:
		swirl_step();
		led_map_step();
		break;
	case ANIMATION4_SWIRL2:
		swirl2_step();
		led_map_step();
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

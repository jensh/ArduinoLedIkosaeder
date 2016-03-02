//					-*- c++ -*-
// (c) 2015-03-29 Jens Hauke <jens.hauke@4k2.de>
// (c) 2016-03-01 Jens Hauke <jens.hauke@4k2.de>
//
// With (D7 D6 D5 D4 D3 D2 D9 D8)
// build a lo, hi, tri-state matrix.
// (Ikosaeder2 uses only 6 bits (bit 7..2): (D7 D6 D5 D4 D3 D2 (D9) (D8))
//
// D10: Animation switcher.
//      Input, pull up switch. Connect to GND.
// D11: Power output. HIGH: on, LOW: off (Auto off feature)

#ifndef HOST_TEST

#define PIN11_POWER	11 /* Out: Power switch for auto off */
#define PIN10_ANIMATION	10 /* In: Animation switch and power on */
#define AUTOOFF_DELAY_MS 60000 /* auto power off after this time */
#define ANIMATION_DURATION 15000 /* duration of an animation, if animation_autoinc is true */

typedef struct {
	unsigned level : 3;
	unsigned dpos : 3;
	unsigned dir : 2;
} level_dpos_dir_t;


typedef struct {
	unsigned id : 6;
	unsigned dir : 2;
} id_dir_t;


void setup() {
	// Setup D2 - D9 as Input
	for (int8_t i = 2; i <= 9; i++) {
		pinMode(i, INPUT);
	}
	pinMode(PIN10_ANIMATION, INPUT_PULLUP);
	autooff_init();

	Serial.begin(115200);
	// Serial.begin(9600);
	help();
}

/*
 * Auto off
 */
static
unsigned long autooff_at_ms = 0;


// Keep alive ping. Stay on for further AUTOOFF_DELAY_MS ms.
void autooff_ping(void) {
	autooff_at_ms = millis() + AUTOOFF_DELAY_MS;
}


void autooff_init(void) {
	pinMode(PIN11_POWER, OUTPUT);
	digitalWrite(PIN11_POWER, HIGH);
	autooff_ping();
}


// Power off, now.
inline
void autooff_off(void) {
	digitalWrite(PIN11_POWER, LOW);
}


// Check for timeout to power off.
void autooff_check(void) {
	// Timeout to auto power off? Unsigned long calculation!
	if (autooff_at_ms - millis() >= (unsigned long) AUTOOFF_DELAY_MS) {
		// Power off, but keep running
		autooff_off();
	}
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
	const int off = 2;
	uint8_t lo_bit = off + led / (nbits - 1);
	uint8_t hi_bit = off + (led + lo_bit) % (nbits - 1);
	if (hi_bit >= lo_bit) hi_bit++;

	// bit pos to bitmask:
	lo_bit = 1 << lo_bit;
	hi_bit = 1 << hi_bit;

	// Hi: only hi_bit, Output: lo_bit and hi_bit
	set_tri(hi_bit, lo_bit | hi_bit);
}


void tri_loop(void) {
	const int nbits = 6;
	int count = nbits * (nbits - 1);
	int i;
	for (i = 0; i < count; i++) {
		set_tri_led(nbits, i);
		delay(5);
	}
}


uint8_t led_map[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };
uint8_t led_pos = 0;
uint16_t led_delay = 1;

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


void led_map_led_set2(uint8_t row, uint8_t bit, bool on = true) {
	row = row & 7;
	if (on) {
		led_map[row] |= 1 << bit;
	} else {
		led_map[row] &= ~(1 << bit);
	}
}


void led_map_led_set(uint8_t led_id, bool on = true) {
	static struct Mapping {
		unsigned row : 3;
		unsigned bit : 3;
	} mapping[30] = {
		{3, 2}, // 0  // level 0
		{2, 4}, // 1  // level 0
		{5, 2}, // 2  // level 0
		{2, 3}, // 3  // level 0
		{6, 2}, // 4  // level 0

		{4, 3}, // 5  // level 1
		{5, 4}, // 6  // level 1
		{3, 5}, // 7  // level 1
		{3, 6}, // 8  // level 1
		{6, 3}, // 9  // level 1

		{7, 3}, // 10 // level 2 r
		{4, 2}, // 12 // level 2 r
		{5, 7}, // 14 // level 2 r
		{5, 3}, // 16 // level 2 r
		{4, 6}, // 18 // level 2 r

		{7, 4}, // 11 // level 2 l
		{2, 5}, // 13 // level 2 l
		{3, 7}, // 15 // level 2 l
		{6, 5}, // 17 // level 2 l
		{3, 4}, // 19 // level 2 l

		{4, 7}, // 20 // level 3
		{2, 7}, // 21 // level 3
		{7, 2}, // 22 // level 3
		{7, 5}, // 23 // level 3
		{4, 5}, // 24 // level 3

		{7, 6}, // 25 // level 4
		{2, 6}, // 26 // level 4
		{6, 7}, // 27 // level 4
		{5, 6}, // 28 // level 4
		{6, 4}, // 29 // level 4
	};

#ifdef HOST_TEST
	cout << "Led " << (int)led_id << " mapping: " << mapping[led_id].row <<  ", " << mapping[led_id].bit << endl;
#endif
	led_map_led_set2(mapping[led_id].row, mapping[led_id].bit, on);
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

static const level_dpos_dir_t peer_d[][4] PROGMEM = {
	{ {1, 0, 1}, {0, 1, 0}, {0, 4, 3}, {1, 4, 0}},
	{ {2, 0, 3}, {3, 0, 0}, {0, 1, 1}, {0, 0, 2}},
	{ {3, 0, 1}, {1, 0, 2}, {3, 4, 3}, {4, 0, 0}},

	{ {4, 1, 1}, {2, 1, 0}, {1, 0, 3}, {2, 0, 2}},
	{ {5, 4, 3}, {5, 0, 0}, {2, 0, 1}, {3, 4, 2}},
	{ {5, 1, 1}, {4, 1, 2}, {4, 0, 3}, {5, 4, 2}}
};


id_dir_t get_peer(uint8_t id, uint8_t dir) {
	uint8_t level = id / 5;
	uint8_t pos = id % 5;
	byte ldpd_b = pgm_read_byte(&peer_d[level][dir]);
	level_dpos_dir_t ldpd;// = *((level_dpos_dir_t*)(void*)(&ldpd_b));
	memcpy(&ldpd, &ldpd_b, sizeof(ldpd));

	uint8_t nlevel = ldpd.level;
	uint8_t npos = (pos + ldpd.dpos) % 5;
	uint8_t nid = nlevel * 5 + npos;
	id_dir_t id_dir = {
		.id = nid,
		.dir = ldpd.dir
	};
	return id_dir;
}


class Walker {
public:
	id_dir_t cur;

	Walker() : cur({ 0, 0 }) {
	}

	/* Step in direction @dir.
	 * 0: forward
	 * 1: turn forward, (left if coming from 0 or 2, right if coming from 1 or 3)
	 * 2: backward,
	 * 3: turn backward
	 */
	void step(uint8_t dir) {
		uint8_t c_id = cur.id;
		uint8_t abs_dir = (cur.dir + dir +
				   (cur.dir & dir & 1) * 2) % 4;

		cur = get_peer(c_id, abs_dir);
	}

	void show(bool visible = true) {
		led_map_led_set(cur.id, visible);
	}
};


unsigned swirl_speed = 40;

void swirl_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < swirl_speed) return;
	last = now;

	led_map_clear();

	static Walker walker;

	walker.step(random(7) == 0 ? 1 : 0);
	walker.show();
}


inline
void swirl2_init(void) {
	led_map_clear();
}


void swirl2_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < swirl_speed) return;
	last = now;

	static Walker walker;
	const int tail_size = 6;
	static uint8_t pos = 0;
	static uint8_t tail[tail_size] = {0};

	// clear tail
	led_map_led_set(tail[pos], false);

	// next step show
	walker.step(random(7) == 0 ? 1 : 0);
	walker.show();

	// set head of tail
	tail[pos] = walker.cur.id;

	// progress
	pos = (pos + 1) % tail_size;
}


/*
 * Loop through led ids
 */
void count_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < swirl_speed) return;
	last = now;

	static struct {
		uint8_t count : 5;
		uint8_t on : 1;
	} state = { 0, 0 };

	led_map_led_set(state.count, state.on);

	// Count level 2 differently:
	//  0, 1, 2, 3, 4, 5, 6, 7, 8, 9
	// 10,15,11,16,12,17,13,18,14,19,
	// 20,21,22,23,24,25,26,27,28,29
	static const uint8_t next[] = {
		1, 2, 3, 4, 5, 6, 7, 8, 9,
		10,15,16,17,18,19,11,12,13,14,
		20,21,22,23,24,25,26,27,28,29,0
	};
	state.count = next[state.count];

	if (state.count == 0) {
		state.on = !state.on;
	}
}


/*
 * swinging hemisphere
 */
struct XYZ {
	int8_t x, y, z;

	int16_t operator*(const XYZ &b) {
		return  ((int16_t)x) * b.x +
			((int16_t)y) * b.y +
			((int16_t)z) * b.z;
	}

	void scale(uint8_t s) {
		x = (((int16_t)x) * s) / 256;
		y = (((int16_t)y) * s) / 256;
		z = (((int16_t)z) * s) / 256;
	}
};

static const
XYZ koordinates[30] = {
	{  50,  31, -81}, // 0
	{  31,  81, -50}, // 1
	{ -31,  81, -50}, // 2
	{ -50,  31, -81}, // 3
	{   0,   0,-100}, // 4
	{  81,  50, -31}, // 5
	{   0, 100,   0}, // 6
	{ -81,  50, -31}, // 7
	{ -50, -31, -81}, // 8
	{  50, -31, -81}, // 9
	{ 100,   0,   0}, // 10
	{  31,  81,  50}, // 11
	{ -81,  50,  31}, // 12
	{ -81, -50, -31}, // 13
	{  31, -81, -50}, // 14
	{  81,  50,  31}, // 15
	{ -31,  81,  50}, // 16
	{ -100,  0,   0}, // 17
	{ -31, -81, -50}, // 18
	{  81, -50, -31}, // 19
	{  81, -50,  31}, // 20
	{  50,  31,  81}, // 21
	{ -50,  31,  81}, // 22
	{ -81, -50,  31}, // 23
	{   0,-100,   0}, // 24
	{  50, -31,  81}, // 25
	{   0,   0, 100}, // 26
	{ -50, -31,  81}, // 27
	{ -31, -81,  50}, // 28
	{  31, -81,  50}, // 29
};


XYZ hemisphere_vec = {
	50, 0, 0
};
float h_x = 0.0, h_y = 0.0, h_z = 0.0;

void hemisphere_walk(int8_t &pos, float &a) {
	pos = sin(a) * 45;
	if (a > 1000) a = 0; // Limit angle range
}

void hemisphere_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < swirl_speed) return;
	last = now;


	uint8_t id;

	int16_t size = hemisphere_vec * hemisphere_vec;

	for (id = 0; id < 30; id++) {
		const XYZ &koor = koordinates[id];
		int16_t sum = hemisphere_vec * koor;

		led_map_led_set(id, sum > size);
	}

	hemisphere_walk(hemisphere_vec.x, h_x);
	hemisphere_walk(hemisphere_vec.y, h_y);
	hemisphere_walk(hemisphere_vec.z, h_z);
	h_x += 0.26;
	h_y += 0.2712;
	h_z += 0.28312;
}


#define CMD_MAX_NUMBERS 3
int cmd_number[CMD_MAX_NUMBERS];
uint8_t cmd_numbers = 0;
bool cmd_number_neg = false;

#define ANIMATION_LEDMAP 0
#define ANIMATION_NONE 1

#define ANIMATION_START ANIMATION_COUNT
#define ANIMATION_COUNT 2
#define ANIMATION_TRILOOP 3
#define ANIMATION_SWIRL2 4
#define ANIMATION_SWIRL 5
#define ANIMATION_HEMISPHERE 6
#define ANIMATION_LAST ANIMATION_HEMISPHERE

#define ANIMATION_AUTOINC 100 /* enable autoinc, keep current animation */

unsigned animation = ANIMATION_SWIRL2;
unsigned last_animation = ~0;
unsigned num_pins = 8;
bool animation_autoinc = true;


void help(void) {
	Serial.println("\nIkosaeder controller.");

	Serial.println("? : Help");
	Serial.println("<num>a : Set animation (100=auto)");
	Serial.println("<led number>s : Switch on led");
	Serial.println("[<num>]# : number of used pins for s");
	delay(200);
	Serial.println("[<in_out mask>,]<lo_hi mask>t : Set tri-state");
	Serial.println("<row>,<mask>m : led map");
	Serial.println("<id>|{<row>,<bit>}{l|L} : led map l:on L:off");
	Serial.println("<delay>d : delay for each led map row");
	Serial.println("<delay>S : Swirl delay");
}


bool set_or_query_param(unsigned &param, char cmd) {
	if (cmd_numbers) {
		// Set
		param = cmd_number[0];
		return true;
	} else {
		// Query
		Serial.print(param); Serial.write(cmd);
		return false;
	}
}


/*
 * Animation auto increment
 */
void animation_autoinc_step(void) {
	static unsigned long last = 0;
	unsigned long now = millis();

	if (now - last < ANIMATION_DURATION) return;
	last = now;

	animation++;
}


void animation_set(unsigned aanimation) {
	animation_autoinc = (aanimation == ANIMATION_AUTOINC);
	if (!animation_autoinc) {
		animation = aanimation;
	}
}


void SerialComm(void) {
	while (Serial.available()) {
		bool number_cmd = false;
		uint8_t cmd;
		unsigned param;

		autooff_ping();

		cmd = Serial.read();

		switch (cmd) {
		case 'a': // Animation cmd_number
			param = animation;
			if (set_or_query_param(param, 'a')) {
				animation_set(param);
			}
			break;
		case 's':
			if (cmd_numbers) {
				set_tri_led(num_pins, cmd_number[0]);
			} else {
				set_tri(0, 0); // All input. (Switch off)
			}
			animation_set(ANIMATION_NONE);
			break;
		case 'c':
		case 'S': // swirl speed
			set_or_query_param(swirl_speed, 'S');
			break;
		case 't':
			set_tri(cmd_number[1], cmd_number[0]);
			animation_set(ANIMATION_NONE);
			break;
		case '#': // Num pins
			set_or_query_param(num_pins, '#');
			break;
		case 'd': // led delay for ledmap <delay>
			set_or_query_param(led_delay, 'd');
			break;
		case 'm': // set map <row>, <mask>
			if (cmd_numbers) {
				led_map_set(cmd_number[0], cmd_number[1]);
				animation_set(ANIMATION_LEDMAP);
			} else {
				// dump map
				led_map_dump();
			}
			break;
		case 'l':
		case 'L':
			switch (cmd_numbers) {
			case 1:
				// set map <id>
				led_map_led_set(cmd_number[0], cmd == 'l');
				animation_set(ANIMATION_LEDMAP);
				break;
			case 2:
				// set map <row>, <bit>
				led_map_led_set2(cmd_number[0], cmd_number[1], cmd == 'l');
				animation_set(ANIMATION_LEDMAP);
				break;
			default:
				led_map_dump();
			}
			break;
		case 'p': // debug cmd_numbers
			Serial.print(cmd_numbers); Serial.write(',');
			Serial.print(cmd_number[0]); Serial.write(',');
			Serial.print(cmd_number[1]); Serial.write(',');
			Serial.print(cmd_number[2]); Serial.write('p');
			break;
		case 'h': // Set/ query hemisphere_vec
			switch (cmd_numbers) {
			default:
			case 3: hemisphere_vec.z = cmd_number[2];
			case 2: hemisphere_vec.y = cmd_number[1];
			case 1: hemisphere_vec.x = cmd_number[0];
				break;
			case 0:
				Serial.print(hemisphere_vec.x); Serial.write(',');
				Serial.print(hemisphere_vec.y); Serial.write(',');
				Serial.print(hemisphere_vec.z); Serial.write('h');
				break;
			}
			break;
/*
 *              Numbers
 */
		case ',':
			cmd_number[cmd_numbers] = 0;
			if (cmd_numbers < CMD_MAX_NUMBERS) cmd_numbers++;
			cmd_number_neg = false;
			number_cmd = true;
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
/*
 *              Misc
 */
		case '?':
			help();
			break;
		case '\n': // LF (VT102: ^j)
			Serial.print("\r\n");
			break;
		case '\r': // CR (VT102: ^m or Enter)
			Serial.print("\r\n");
			break;
		default:
			Serial.println("Unknown cmd.");
			Serial.println("Expect one of '-0123456789,?admpsSt#'");
			break;
		}

		if (!number_cmd) {
			cmd_numbers = 0;
			cmd_number_neg = false;
		}
	}
}


void PushButtonComm(void) {
	if (!digitalRead(PIN10_ANIMATION)) {
		autooff_ping();
		if (animation != ANIMATION_LAST || animation_autoinc) {
			animation_set(animation + 1);
		} else {
			animation_set(ANIMATION_AUTOINC);
			for (int i = 0; i < 3; i++) {
				tri_loop();
			}
		}
		// poor man's debounce
		delay(300);
	}
}


void loop() {
	// Initialize new animation?
	if (last_animation != animation) {
		switch (animation) {
		case ANIMATION_SWIRL2:
			swirl2_init();
			break;
		}
	}

	// Run animation
	switch (animation) {
	case ANIMATION_LEDMAP:
		// Static led map
		led_map_step();
		break;
	case ANIMATION_NONE:
		// Nothing. Keep state.
		break;
	case ANIMATION_COUNT:
		count_step();
		led_map_step();
		break;
	case ANIMATION_TRILOOP:
		tri_loop();
		break;
	case ANIMATION_SWIRL2:
		swirl2_step();
		led_map_step();
		break;
	case ANIMATION_SWIRL:
		swirl_step();
		led_map_step();
		break;
	case ANIMATION_HEMISPHERE:
		hemisphere_step();
		led_map_step();
		break;
	default:
		Serial.print("\"Unknown animation ");
		Serial.print(animation);
		Serial.println(". Using 2 now.\"");
		// fall through
	case ANIMATION_LAST + 1:
		animation = ANIMATION_START;
		break;
	}

	last_animation = animation;

	// Process commands
	SerialComm();
	PushButtonComm();
	if (animation_autoinc) animation_autoinc_step();
	autooff_check();
}


/*
 * Local Variables:
 *  compile-command: "arduino --upload --preserve-temp-files -v Ikosaeder.ino"
 * End:
 */

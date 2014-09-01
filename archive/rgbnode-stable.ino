 

#include <SoftwareSerial.h>
#include <IRremote.h>
#include "colours.h"

//#define DEBUG
#undef DEBUG

/*******************
 * Pin Assignments *
 *******************/

#define PIN_RELAY	4

#define PIN_RED		6
#define PIN_GREEN	5
#define PIN_BLUE	9

#define PIN_IRRECV	12

#define PIN_LED		13

#define PIN_SSRX	2
#define PIN_SSTX	3

/*******************
 * RGB LED Control *
 *******************/

#define RGB_MAX_INTENSITY	0xff
#define RGB_BROWNIAN_MAX	0x600

enum {
	RCM_ONE,
	RCM_MULTI,
	RCM_RANDOM
};

byte rgb_col_mode = RCM_ONE;
int rgb_delay = 2000;
char rgb_col_index = 0;
RGBcol rgb_colour = { 0xFF, 0xFF, 0xFF };

int rgb_on = 0;
int rgb_intensity = RGB_MAX_INTENSITY;

/// RGB State Machine
enum {
	RS_STOP,
	RS_SWIRL_FADE,
	RS_SWIRL_HOLD,
	RS_STROBE_ON,
	RS_STROBE_OFF,
	RS_BROWNIAN,
	RS_SOLID
};

#define RS_SWIRL	RS_SWIRL_HOLD
#define RS_STROBE	RS_STROBE_OFF

char rgb_running = 0;
byte rgb_state = RS_STOP;
RGBcol rgb_target = { 0xFF, 0xFF, 0xFF };
RGBcol rgb_output = { 0xFF, 0xFF, 0xFF };

/// If the Millisecond per Change value is postive, channel is incremented every * milliseconds.
/// If negative, channel is decremented every * milliseconds
int rgb_mpc[3] = { 0, 0, 0 };		// Millisecond per Change
int rgb_mpc_remain[3] = { 0, 0, 0 };	// Milliseconds Left Over
unsigned long rgb_mpc_last = millis();

int rgb_hold = 0;
unsigned long rgb_hold_last = millis();

int rgb_brownian = 0;

void update_rgb_state(void);
void rgb_next_state(void);
RGBcol rgb_next_colour(void);

#ifdef DEBUG
#define PRINTCOLOUR(c)			\
	Serial.print((c).r, DEC);	\
	Serial.print(' ');		\
	Serial.print((c).g, DEC);	\
	Serial.print(' ');		\
	Serial.print((c).b, DEC);	\
	Serial.print('\n');
#endif

void rgb_send_set()
{
	Serial.print('S');
	Serial.print(rgb_output.c[0], DEC);
	Serial.print(',');
	Serial.print(rgb_output.c[1], DEC);
	Serial.print(',');
	Serial.print(rgb_output.c[2], DEC);
	Serial.print('\n');
}

void rgb_send_fade()
{
	Serial.print('F');
	Serial.print(rgb_target.c[0], DEC);
	Serial.print(',');
	Serial.print(rgb_target.c[1], DEC);
	Serial.print(',');
	Serial.print(rgb_target.c[2], DEC);
	Serial.print(',');
	Serial.print(rgb_delay, DEC);
	Serial.print('\n');
}

void update_rgb()
{
	unsigned int r, g, b, i, m;

	if (rgb_on) {
		if (rgb_running)
			update_rgb_state();
		if (!rgb_running && rgb_state != RS_STOP)
			rgb_next_state();
/*
		i = rgb_intensity * rgb_intensity;
		m = RGB_MAX_INTENSITY * RGB_MAX_INTENSITY;

		r = ( ((int) output[RED]) * i ) / m;
		g = ( ((int) output[GREEN]) * i ) / m;
		b = ( ((int) output[BLUE]) * i ) / m;
*/
		r = map(rgb_output.r, 0, 255, 0, rgb_intensity);
		g = map(rgb_output.g, 0, 255, 0, rgb_intensity);
		b = map(rgb_output.b, 0, 255, 0, rgb_intensity);

		analogWrite(PIN_RED, r);
		analogWrite(PIN_GREEN, g);
		analogWrite(PIN_BLUE, b);
	}
	else {
		analogWrite(PIN_RED, 0);
		analogWrite(PIN_GREEN, 0);
		analogWrite(PIN_BLUE, 0);
	}
}

void update_rgb_state(void)
{
	char change;
	int diff, last = 0;

	diff = millis() - rgb_mpc_last;
	if (diff < 1)		// Don't update unless there was a change
		return;
	rgb_mpc_last = millis();
	for (int i = 0; i < 3; i++) {
		if (rgb_mpc[i] != 0) {
			last = diff + rgb_mpc_remain[i];
			change = last / rgb_mpc[i];
			rgb_mpc_remain[i] = last % abs(rgb_mpc[i]);
			rgb_output.c[i] += change;
			if ((change >= 1 && rgb_output.c[i] >= rgb_target.c[i])
			   || (change <= -1 && rgb_output.c[i] <= rgb_target.c[i])) {
				rgb_output.c[i] = rgb_target.c[i];
				rgb_mpc[i] = 0;
			}
		}
	}

	if (last == 0 && rgb_hold == 0)
		rgb_running = 0;
	else if ((rgb_hold && ((millis() - rgb_hold_last) > rgb_hold)))
		rgb_clear_state();

	#ifdef DEBUG
	if (change != 0) {
		Serial.print(rgb_output.r, DEC);
		Serial.print(' ');
		Serial.print(rgb_output.g, DEC);
		Serial.print(' ');
		Serial.print(rgb_output.b, DEC);
		Serial.print('\n');
	}
	#endif
}

void rgb_setup_fade(int delay)
{
	for (int i = 0; i < 3; i++) {
		if (rgb_target.c[i] == rgb_output.c[i])
			rgb_mpc[i] = 0;
		else
			rgb_mpc[i] = delay / (rgb_target.c[i] - rgb_output.c[i]);
	}
}

void rgb_next_state(void)
{
	#ifdef DEBUG
	Serial.print("Finished state ");
	Serial.print(rgb_state, DEC);
	Serial.print("\n");
	#endif

	switch (rgb_state) {
	    case RS_SWIRL_HOLD:
		rgb_state = RS_SWIRL_FADE;
		rgb_target = rgb_next_colour();
		rgb_setup_fade(rgb_delay);
		rgb_send_fade();

		#ifdef DEBUG
		Serial.print("Next colour: ");
		Serial.print(rgb_col_index, DEC);
		Serial.print("\n");
		Serial.print("Target: ");
		PRINTCOLOUR(rgb_target)
		Serial.print("Output: ");
		PRINTCOLOUR(rgb_output)

		Serial.print("FADE for ");
		Serial.print(rgb_mpc[0], DEC);
		Serial.print("ms ");
		Serial.print(rgb_mpc[1], DEC);
		Serial.print("ms ");
		Serial.print(rgb_mpc[2], DEC);
		Serial.print("ms\n");
		#endif
		break;
	    case RS_SWIRL_FADE:
		rgb_state = RS_SWIRL_HOLD;
		rgb_hold = rgb_delay;
		rgb_hold_last = millis();
		#ifdef DEBUG
		Serial.print("HOLD for ");
		Serial.print(rgb_hold, DEC);
		Serial.print("ms\n");
		#endif
		break;
	    case RS_STROBE_ON:
		rgb_state = RS_STROBE_OFF;
		rgb_output = rgb_palette[BLACK];
		rgb_hold = rgb_delay;
		rgb_hold_last = millis();
		rgb_send_set();
		break;
	    case RS_STROBE_OFF:
		rgb_state = RS_STROBE_ON;
		rgb_output = rgb_next_colour();
		rgb_hold = 20;
		rgb_hold_last = millis();
		rgb_send_set();
		break;
	    case RS_BROWNIAN:
		//int r, v;
		//r = random(0, 3);
		//v = rgb_output.c[r] + random(-1, 2);
		//if (v >= 0 && v <= 255)
		//	rgb_output.c[r] = v;

		int r;
		r = random(0, 1000);
		if (r < 3000)
			rgb_brownian -= r / 1000;
		else if (r > 7000)
			rgb_brownian += (r - 4000) / 1000;
		if (rgb_brownian >= RGB_BROWNIAN_MAX)
			rgb_brownian = 0;
		else if (rgb_brownian < 0)
			rgb_brownian = RGB_BROWNIAN_MAX - 1;

		switch (rgb_brownian >> 8) {
		    case 0:
			rgb_output.r = 0xFF;
			rgb_output.g = (byte) (rgb_brownian & 0xFF);
			rgb_output.b = 0;
			break;
		    case 1:
			rgb_output.r = (byte) (0xFF - (rgb_brownian & 0xFF));
			rgb_output.g = 0xFF;
			rgb_output.b = 0;
			break;
		    case 2:
			rgb_output.r = 0;
			rgb_output.g = 0xFF;
			rgb_output.b = (byte) (rgb_brownian & 0xFF);
			break;
		    case 3:
			rgb_output.r = 0;
			rgb_output.g = (byte) (0xFF - (rgb_brownian & 0xFF));
			rgb_output.b = 0xFF;
			break;
		    case 4:
			rgb_output.r = (byte) (rgb_brownian & 0xFF);
			rgb_output.g = 0;
			rgb_output.b = 0xFF;
			break;
		    case 5:
			rgb_output.r = 0xFF;
			rgb_output.g = 0;
			rgb_output.b = (byte) (0xFF - (rgb_brownian & 0xFF));
			break;
		    default:
			break;
		}

		rgb_hold = rgb_delay / 20;
		rgb_hold_last = millis();
		rgb_send_set();
		break;
	    case RS_SOLID:
		rgb_state = RS_STOP;
		rgb_output = rgb_target;
		rgb_send_set();
		return;
	    default:
		break;
	}
	rgb_running = 1;
}

RGBcol rgb_next_colour()
{
	if (rgb_col_mode == RCM_ONE)
		return(rgb_colour);
	else if (rgb_col_mode == RCM_MULTI) {
		if (++rgb_col_index >= RGB_COL_END)
			rgb_col_index = RGB_COL_START;
		return(rgb_palette[rgb_col_index]);
	}
	else if (rgb_col_mode == RCM_RANDOM) {
		rgb_col_index = random(RGB_COL_START, RGB_COL_END);
		return(rgb_palette[rgb_col_index]);
	}
}

void rgb_clear_state()
{
	rgb_running = 0;
	rgb_hold = 0;
	rgb_mpc[0] = 0;
	rgb_mpc[1] = 0;
	rgb_mpc[2] = 0;
}


/******************************
 * Serial Communications Code *
 ******************************/

#define SERIAL_SIZE	32
#define SERIAL_SPEED	19200

char serial_r = 0;
char serial_avail = 0;
char serial_rb[SERIAL_SIZE];

SoftwareSerial debug(PIN_SSRX, PIN_SSTX);

int read_serial()
{
	int b;

	b = Serial.read();
	if (b == -1)
		return 0;

	serial_rb[serial_r] = b;
	if (b == '\n' || b == '\r') {
		serial_rb[serial_r + 1] = '\0';
		serial_avail = 1;
	}
	if (serial_r < SERIAL_SIZE)
		serial_r++;

	b = debug.read();
	if (b != -1)
		Serial.write(b);
	return serial_avail;
}

void clear_serial()
{
	serial_avail = 0;
	serial_r = 0;
}

int serial_get_num(int &i)
{
	int start = i;

	for (; i <= serial_r; i++) {
		if (serial_rb[i] == ',' || serial_rb[i] == '\n') {
			serial_rb[i++] = '\0';
			break;
		}
		else if (serial_rb[i] < 0x30 || serial_rb[i] > 0x39)
			return(0);
	}
	return(atoi(&serial_rb[start]));
}


/********************
 * IR Recevier Code *
 ********************/

IRrecv irrecv(PIN_IRRECV);
decode_results ir_data;
int ir_code;
int ir_repeat;
int ir_repeat_count = 0;

int read_ir(void)
{
	if (!irrecv.decode(&ir_data))
		return(0);

	if (ir_data.value == 0xFFFFFFFF) {
		if (++ir_repeat_count <= (ir_repeat ? 1 : 4)) {
			irrecv.resume();
			return(0);
		}
		ir_repeat = 1;
		ir_repeat_count = 0;
	}
	else {
		ir_code = (int) ((ir_data.value & 0xFF000000) >> 16) | ((ir_data.value & 0x0000FF00) >> 8);
		ir_repeat = 0;
		ir_repeat_count = 0;
	}

	//Serial.print("I0=");
	//Serial.print(ir_code, HEX);
	//Serial.print('\n');
	irrecv.resume();
	return(1);
}


/**************************
 * Channel Behaviour Code *
 **************************/

#define MAX_CHANNEL	10
int channel = 1;
int relay_on = 0;

void update_channel()
{
	if (rgb_running)
		rgb_clear_state();

	switch (channel) {
	    case 1:
		rgb_col_index = WHITE;
		rgb_target = rgb_palette[rgb_col_index];
		rgb_state = RS_SOLID;
		break;
	    case 2:
		rgb_col_index = MEDIUM;
		rgb_target = rgb_palette[rgb_col_index];
		rgb_state = RS_SOLID;
		break;
	    case 3:
		rgb_col_index = WARM;
		rgb_target = rgb_palette[rgb_col_index];
		rgb_state = RS_SOLID;
		break;
	    case 4:
		// TODO set the colour to the current(last) selected colour
		//rgb_target = rgb_palette[rgb_cur_col];
		rgb_col_index = GOLD;
		rgb_target = rgb_palette[rgb_col_index];
		rgb_state = RS_SOLID;
		break;
	    case 5:
		// TODO set the colour to the current hue??
		rgb_state = RS_STROBE;
		rgb_col_mode = RCM_ONE;
		break;
	    case 6:
		rgb_state = RS_STROBE;
		rgb_col_mode = RCM_RANDOM;
		break;
	    case 7:
		rgb_state = RS_SWIRL;
		rgb_col_mode = RCM_MULTI;
		break;
	    case 8:
		rgb_state = RS_SWIRL;
		rgb_col_mode = RCM_RANDOM;
		break;
	    case 9:
		rgb_state = RS_BROWNIAN;
		//rgb_col_index = random(RGB_COL_START, RGB_COL_END);
		//rgb_output = rgb_palette[rgb_col_index];
		rgb_brownian = random(0, RGB_BROWNIAN_MAX);
		break;
	    default:
		break;
	}
}

byte dev_on = 0;

void process_ir(int key)
{
	int prev_channel = channel;

	if (!rgb_on && key != 0x248) {
		Serial.print('I');
		Serial.print(key, HEX);
		Serial.print('\n');
	}

	switch (key) {
	    case 0x2f0:
		relay_on = relay_on ? 0 : 1;
		digitalWrite(PIN_RELAY, relay_on);
		break;
	    case 0x248:		// Power
		if (ir_repeat)	// Don't allow repeat codes
			break;
		if (++dev_on >= 4)
			dev_on = 0;
		rgb_on = (dev_on & 0x01);
		/// Send on/off command to slave devices
		Serial.print((dev_on & 0x02) >> 1, DEC);
		Serial.print('\n');
		break;
	    case 0x280:		// One
		channel = 1;
		break;
	    case 0x240:		// Two
		channel = 2;
		break;
	    case 0x2C0:		// Three
		channel = 3;
		break;
	    case 0x220:		// Four
		channel = 4;
		break;
	    case 0x2A0:		// Five
		channel = 5;
		break;
	    case 0x260:		// Six
		channel = 6;
		break;
	    case 0x2E0:		// Seven
		channel = 7;
		break;
	    case 0x210:		// Eight
		channel = 8;
		break;
	    case 0x290:		// Nine
		channel = 9;
		break;
	    case 0x200:		// Zero
		channel = 0;
		break;
	//    case 0x208:		// Mute
	//	if (rgb_intensity_prev == -1) {
	//		rgb_intensity_prev = rgb_intensity;
	//		rgb_intensity = RGB_MAX_INTENSITY;
	//	}
	//	else {
	//		rgb_intensity = rgb_intensity_prev;
	//		rgb_intensity_prev = -1;
	//	}
	//	break;
	//
	    case 0x2D8:		// Channel Up
		if (++channel > MAX_CHANNEL)
			channel = MAX_CHANNEL;
		break;
	    case 0x2F8:		// Channel Down
		if (--channel < 1)
			channel = 1;
		break;
	    case 0x258:		// Volume Up
		rgb_intensity += (rgb_intensity >> 3) + 1;
		if (rgb_intensity > RGB_MAX_INTENSITY)
			rgb_intensity = RGB_MAX_INTENSITY;
		break;
	    case 0x278:		// Volume Down
		rgb_intensity -= (rgb_intensity >> 3) - 1;
		if (rgb_intensity < 0)
			rgb_intensity = 0;
		break;
	    case 0x241:		// Channel Set +
		if (--rgb_col_index < 0)
			rgb_col_index = 0;
		rgb_target = rgb_palette[rgb_col_index];
		if (rgb_state == RS_STOP)
			rgb_state = RS_SOLID;
		rgb_running = 1;
		break;
	    case 0x291:		// Enter
		if (++rgb_col_index >= NUM_PALETTE)
			rgb_col_index = NUM_PALETTE - 1;
		rgb_target = rgb_palette[rgb_col_index];
		if (rgb_state == RS_STOP)
			rgb_state = RS_SOLID;
		rgb_running = 1;
		break;
	    case 0x201:		// Menu
		rgb_delay += rgb_delay / 50;
		break;
	    case 0x2C1:		// Channel Set -
		rgb_delay -= rgb_delay / 50;
		break;
	    default:
		//process_channel_key(key);
		break;
	}

	if (channel != prev_channel)
		update_channel();
}

void process_serial()
{
	int i;
	char type, cmd;
	byte addr, data;
	int value;

	if (!serial_avail)
		return;

	Serial.print(serial_rb);

	switch (serial_rb[0]) {
	    case '1':
		rgb_on = 1;
		break;
	    case '0':
		rgb_on = 0;
		break;
	    case 'c':
		if (serial_rb[1] >= 0x30 && serial_rb[1] <= 0x39)
			channel	= serial_rb[1] - 0x30;
		update_channel();
		break;
	    case '-':
		//rgb_on = 0;
		break;
	    case 'I':
		i = 1;
		value = serial_get_num(i);
		if (value == -1)
			ir_repeat = 1;
		else {
			ir_repeat = 0;
			ir_code = value;
		}
		process_ir(ir_code);
		break;
	    case 'S':
		i = 1;
		rgb_state = RS_STOP;
		rgb_running = 0;
		for (int j = 0; j < 3; j++)
			rgb_output.c[j] = (byte) serial_get_num(i);
		break;
	    case 'F':
		i = 1;
		rgb_state = RS_STOP;
		rgb_running = 1;
		for (int j = 0; j < 3; j++)
			rgb_target.c[j] = (byte) serial_get_num(i);
		rgb_delay = serial_get_num(i);
		rgb_setup_fade(rgb_delay);
		break;
	    default:
		break;
	}

/*
	if (serial_rb[0] >= 0x61 && serial_rb[0] <= 0x7A)
		serial_rb[0] -= 0x20;

	type = serial_rb[0];
	for (i = 1; i <= serial_r; i++) {
		if (serial_rb[i] >= 0x30 && serial_rb[i] <= 0x39)
			continue;
		else if (serial_rb[i] == '=' || serial_rb[i] == '?') {
			cmd = serial_rb[i];
			serial_rb[i] = '\0';
			addr = atoi(&serial_rb[1]);
			data = i + 1;
			break;
		}
		else
			return;		/// Otherwise we ignore the message
	}

	Serial.write(type);
	Serial.print(addr, DEC);
	Serial.write(cmd);
	Serial.print(value, DEC);
	Serial.write('\n');
	if (cmd == '=') {
		if (type == 'R') {
			value = atoi(&serial_rb[data]);
			if (addr == 0) {
				if (value == 0)
					digitalWrite(PIN_RELAY, LOW);
				else if (value == 1)
					digitalWrite(PIN_RELAY, HIGH);
			}
		}
		else if (type == 'B') {
			value = atoi(&serial_rb[data]);
			switch (addr) {
			    case 0:
				rgb_on = value;
				break;
			    case 1:
				rgb_intensity = value;
				Serial.write('B');
				Serial.print(addr, DEC);
				Serial.write('>');
				Serial.print(value, DEC);
				Serial.write('\n');
				break;
			    default:
				break;
			}
		}
	}
	else if (cmd == '?') {

	}
*/
}


/****************
 * Setup & Loop *
 ****************/

void setup()
{
	randomSeed(analogRead(0));

	pinMode(PIN_RELAY, OUTPUT);
	pinMode(PIN_LED, OUTPUT);

	pinMode(PIN_RED, OUTPUT);
	pinMode(PIN_GREEN, OUTPUT);
	pinMode(PIN_BLUE, OUTPUT);
	analogWrite(PIN_RED, 0);
	analogWrite(PIN_GREEN, 0);
	analogWrite(PIN_BLUE, 0);

	Serial.begin(SERIAL_SPEED);
	debug.begin(SERIAL_SPEED);
	clear_serial();
	irrecv.enableIRIn();

	rgb_next_state();
}

void loop()
{
	if (read_serial()) {
		process_serial();
		clear_serial();
	}

	if (read_ir())
		process_ir(ir_code);

	update_rgb();
}







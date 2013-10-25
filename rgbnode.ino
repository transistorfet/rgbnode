 

#include <SoftwareSerial.h>
#include <IRremote.h>
#include "colours.h"


/*******************
 * Pin Assignments *
 *******************/

#define PIN_IRTX	3
#define PIN_IRRX	12

#define PIN_RELAY	4

#define PIN_RED		6
#define PIN_GREEN	5
#define PIN_BLUE	9

#define PIN_LED		13

#define PIN_DAISYRX	7
#define PIN_DAISYTX	8


/************
 * IR Codes *
 ************/

#define TV_ADDR		0x4004
#define TV_POWER	0x100BCBD
#define STEREO_POWER	0xA81

/*
#define TV_ADDR		0x4004
#define TV_POWER	0x100BCBD
#define TV_VOLUP	0x1000405
#define TV_VOLDOWN	0x1008485
#define TV_INPUT	0x100A0A1
#define TV_TWO		0x1008889
#define TV_THREE	0x1004849

unsigned long tv[] = { TV_POWER, TV_VOLUP, TV_VOLDOWN, TV_INPUT, TV_TWO, TV_THREE };

#define STEREO_POWER	0xA81
#define STEREO_VOLUP	0x481
#define STEREO_VOLDOWN	0xC81
#define STEREO_TAPE	0xC41
#define STEREO_TUNER	0x841

unsigned int stereo[] = { STEREO_POWER, STEREO_VOLUP, STEREO_VOLDOWN, STEREO_TAPE, STEREO_TUNER };
*/

typedef union {
	unsigned char bytes[4];
	unsigned long value;
} LongT;

/******************************
 * Serial Communications Code *
 ******************************/

#define SERIAL_SIZE	32
#define SERIAL_SPEED	19200
#define DAISY_SPEED	19200

char serial_r = 0;
char serial_avail = 0;
char serial_rb[SERIAL_SIZE];

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

	return serial_avail;
}

void clear_serial()
{
	serial_avail = 0;
	serial_r = 0;
}


/*********************************
 * DaisyWire Communications Code *
 *********************************/

#define DAISY_SIZE	32
#define DAISY_SPEED	19200

char daisy_r = 0;
char daisy_avail = 0;
char daisy_rb[SERIAL_SIZE];

SoftwareSerial Daisy(PIN_DAISYRX, PIN_DAISYTX);

int read_daisy()
{
	int b;

	b = Daisy.read();
	if (b == -1)
		return 0;

	daisy_rb[daisy_r] = b;
	if (b == '\n' || b == '\r') {
		daisy_rb[daisy_r + 1] = '\0';
		daisy_avail = 1;
	}
	if (daisy_r < SERIAL_SIZE)
		daisy_r++;

	//b = debug.read();
	//if (b != -1)
	//	Daisy.write(b);
	return daisy_avail;
}

void clear_daisy()
{
	daisy_avail = 0;
	daisy_r = 0;
}

int daisy_get_num(int &i)
{
	int start = i;

	for (; i <= daisy_r; i++) {
		if (daisy_rb[i] == ',' || daisy_rb[i] == '\n') {
			daisy_rb[i++] = '\0';
			break;
		}
		else if (daisy_rb[i] < 0x30 || daisy_rb[i] > 0x39)
			return(0);
	}
	return(atoi(&daisy_rb[start]));
}


/********************
 * IR Recevier Code *
 ********************/

IRrecv irrecv(PIN_IRRX);
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


/********************
 * IR Transmit Code *
 ********************/

IRsend irsend;

void ir_send_sony(unsigned int data)
{
	irsend.sendSony(data, 12);
	delay(50);
	irsend.sendSony(data, 12);
	delay(50);
	irsend.sendSony(data, 12);
	irrecv.enableIRIn();
}

void ir_send_panasonic(unsigned int address, unsigned long data)
{
	irsend.sendPanasonic(address, data);
	irrecv.enableIRIn();
}


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

void rgb_send_set()
{
	Daisy.print('S');
	Daisy.print(rgb_output.c[0], DEC);
	Daisy.print(',');
	Daisy.print(rgb_output.c[1], DEC);
	Daisy.print(',');
	Daisy.print(rgb_output.c[2], DEC);
	Daisy.print('\n');
}

void rgb_send_fade()
{
	Daisy.print('F');
	Daisy.print(rgb_target.c[0], DEC);
	Daisy.print(',');
	Daisy.print(rgb_target.c[1], DEC);
	Daisy.print(',');
	Daisy.print(rgb_target.c[2], DEC);
	Daisy.print(',');
	Daisy.print(rgb_delay, DEC);
	Daisy.print('\n');
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
	switch (rgb_state) {
	    case RS_SWIRL_HOLD:
		rgb_state = RS_SWIRL_FADE;
		rgb_target = rgb_next_colour();
		rgb_setup_fade(rgb_delay);
		rgb_send_fade();
		break;
	    case RS_SWIRL_FADE:
		rgb_state = RS_SWIRL_HOLD;
		rgb_hold = rgb_delay;
		rgb_hold_last = millis();
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

void rgb_set_index_colour(char col)
{
	rgb_col_index = col;

	if (rgb_col_index < 0)
		rgb_col_index = 0;
	else if (rgb_col_index >= NUM_PALETTE)
		rgb_col_index = NUM_PALETTE - 1;

	rgb_target = rgb_palette[rgb_col_index];
	if (rgb_state == RS_STOP)
		rgb_state = RS_SOLID;
	rgb_running = 1;
}

/**************************
 * Channel Behaviour Code *
 **************************/

#define MAX_CHANNEL	10
int channel = 1;
int relay_on = 0;

void set_channel(char ch)
{
	channel = ch;
	if (channel > MAX_CHANNEL)
		channel = MAX_CHANNEL;
	else if (channel < 1)
		channel = 1;

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
		Daisy.print('I');
		Daisy.print(key, HEX);
		Daisy.print('\n');
	}

	switch (key) {
	    case 0x2f0:		// TV/Video
		relay_on = relay_on ? 0 : 1;
		digitalWrite(PIN_RELAY, relay_on);
		break;
	    case 0x2EA:		// Cap/Text
		ir_send_panasonic(TV_ADDR, TV_POWER);
		break;
	    case 0x2C6:		// 1/2
		ir_send_sony(STEREO_POWER);
		break;
	    case 0x248:		// Power
		if (ir_repeat)	// Don't allow repeat codes
			break;
		if (++dev_on >= 4)
			dev_on = 0;
		rgb_on = (dev_on & 0x01);
		/// Send on/off command to slave devices
		Daisy.print((dev_on & 0x02) >> 1, DEC);
		Daisy.print('\n');
		break;
	    case 0x280:		// One
		set_channel(1);
		break;
	    case 0x240:		// Two
		set_channel(2);
		break;
	    case 0x2C0:		// Three
		set_channel(3);
		break;
	    case 0x220:		// Four
		set_channel(4);
		break;
	    case 0x2A0:		// Five
		set_channel(5);
		break;
	    case 0x260:		// Six
		set_channel(6);
		break;
	    case 0x2E0:		// Seven
		set_channel(7);
		break;
	    case 0x210:		// Eight
		set_channel(8);
		break;
	    case 0x290:		// Nine
		set_channel(9);
		break;
	    case 0x200:		// Zero
		set_channel(0);
		break;
	    case 0x2D8:		// Channel Up
		set_channel(channel + 1);
		break;
	    case 0x2F8:		// Channel Down
		set_channel(channel - 1);
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
		rgb_set_index_colour(rgb_col_index - 1);
		break;
	    case 0x291:		// Enter
		rgb_set_index_colour(rgb_col_index + 1);
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
}

void process_serial()
{
	LongT temp;

	if (!serial_avail)
		return;

	if (serial_rb[0] == 'I') {		// Transmit IR Code
		if (serial_rb[1] == 's') {
			temp.value = strtol(&serial_rb[2], NULL, 16);
			ir_send_sony(temp.value);
		}
		else if (serial_rb[1] == 'p') {
			int addr;
			char *endptr = NULL;

			addr = strtol(&serial_rb[2], &endptr, 16);
			if (*endptr == ',') {
				temp.value = strtol(&endptr[1], NULL, 16);
				ir_send_panasonic(addr, temp.value);
			}
		}
	}
	else if (serial_rb[0] == 'S') {		// Set RGB Colour
		if (serial_r != 8)	// 1 char + 6 data command plus linefeed
			return;
		temp.value = strtol(&serial_rb[1], NULL, 16);
		rgb_state = RS_STOP;
		rgb_running = 0;
		rgb_output.c[0] = temp.bytes[2];
		rgb_output.c[1] = temp.bytes[1];
		rgb_output.c[2] = temp.bytes[0];
	}
	else if (serial_rb[0] == 'C') {		// Set RGB Channel/Mode
		if (serial_rb[1] >= '0' && serial_rb[1] <= '9')
			set_channel(serial_rb[1] - '0');
	}
	else if (serial_rb[0] == 'L') {		// RGB Power On/Off
		if (serial_rb[1] == '0')
			rgb_on = 0;
		else if (serial_rb[1] == '1')
			rgb_on = 1;
		else if (serial_rb[1] == 'x')
			rgb_on = rgb_on ? 0 : 1;
	}
	else if (serial_rb[0] == 'D') {		// Set RGB Delay
		temp.value = strtol(&serial_rb[1], NULL, 16);
		rgb_delay = temp.value;
	}
	else if (serial_rb[0] == 'P') {		// Set RGB Palette Colour
		temp.value = strtol(&serial_rb[1], NULL, 16);
		rgb_set_index_colour(temp.value);
	}
}

void process_daisy()
{
	int i;
	char type, cmd;
	byte addr, data;
	int value;

	if (!daisy_avail)
		return;

	Daisy.print(daisy_rb);

	switch (daisy_rb[0]) {
	    case '1':
		rgb_on = 1;
		break;
	    case '0':
		rgb_on = 0;
		break;
	    case 'c':
		if (daisy_rb[1] >= '0' && daisy_rb[1] <= '9')
			set_channel(daisy_rb[1] - '0');
		break;
	    case '-':
		//rgb_on = 0;
		break;
	    case 'I':
		i = 1;
		value = daisy_get_num(i);
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
			rgb_output.c[j] = (byte) daisy_get_num(i);
		break;
	    case 'F':
		i = 1;
		rgb_state = RS_STOP;
		rgb_running = 1;
		for (int j = 0; j < 3; j++)
			rgb_target.c[j] = (byte) daisy_get_num(i);
		rgb_delay = daisy_get_num(i);
		rgb_setup_fade(rgb_delay);
		break;
	    default:
		break;
	}
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
	clear_serial();
	Daisy.begin(DAISY_SPEED);
	clear_daisy();
	irrecv.enableIRIn();

	rgb_next_state();
}

void loop()
{
	if (read_serial()) {
		process_serial();
		clear_serial();
	}

	if (read_daisy()) {
		process_daisy();
		clear_daisy();
	}

	if (read_ir())
		process_ir(ir_code);

	update_rgb();
}



 

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

#define PIN_WRELAY_ON	10
#define PIN_WRELAY_OFF	11

/************
 * IR Codes *
 ************/

// These codes are used by the NEC Remote for turning the TV and Stereo on and off
#define TV_ADDR		0x4004
#define TV_POWER	0x100BCBD
#define STEREO_POWER	0xA81

typedef union {
	unsigned char bytes[4];
	unsigned long value;
} LongT;

int wrelay = 0;

/******************************
 * Serial Communications Code *
 ******************************/

#define SERIAL_SIZE	32
#define SERIAL_SPEED	19200

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

#define DAISY_ON		0xA0
#define DAISY_SET_MODE		0xA1
#define DAISY_SET_DELAY		0xA2
#define DAISY_SET_TARGET	0xA3
#define DAISY_SET_INTENSITY	0xA4
#define DAISY_CALIBRATE		0xA5
#define DAISY_KEY		0xA6

#define DAISY_FADE		0xA9

#define DAISY_SIZE	8
#define DAISY_SPEED	19200

char daisy_r = 0;
char daisy_size = 0;
char daisy_avail = 0;
byte daisy_rb[DAISY_SIZE];

char daisy_w = 0;
byte daisy_wb[DAISY_SIZE];

SoftwareSerial Daisy(PIN_DAISYRX, PIN_DAISYTX);

int read_daisy()
{
	int b;
	byte checksum;

	b = Daisy.read();
	if (b == -1)
		return 0;

	if (!daisy_size) {
		daisy_size = b;
	}
	else {
		daisy_rb[daisy_r++] = b;
		if (daisy_r == daisy_size) {
			Serial.print("debugread ");
			Serial.print(daisy_r, DEC);
			Serial.write(' ');
			for (char i = 0; i < daisy_r; i++) {
				Serial.print((byte) daisy_rb[i], HEX);
				Serial.write(' ');
			}
			Serial.write('\n');

			checksum = daisy_checksum(daisy_rb, daisy_r - 1);
			if (daisy_rb[daisy_r - 1] == checksum)
				daisy_avail = 1;
			else {
				Serial.print("debug checksumfail\n");
				clear_daisy();
			}
		}
		if (daisy_r == SERIAL_SIZE)
			clear_daisy();
	}
	return daisy_avail;
}

void clear_daisy()
{
	daisy_avail = 0;
	daisy_size = 0;
	daisy_r = 0;
}

void daisy_write_byte(byte ch)
{
	daisy_wb[daisy_w++] = ch;
}

void daisy_send_msg()
{
	daisy_wb[daisy_w] = daisy_checksum(daisy_wb, daisy_w);
	daisy_w++;

	Serial.print("debugwrite ");
	Serial.print(daisy_w, DEC);
	Serial.write(' ');
	for (char i = 0; i < daisy_w; i++) {
		Serial.print((byte) daisy_wb[i], HEX);
		Serial.write(' ');
	}
	Serial.write('\n');

	Daisy.write((byte) daisy_w);
	for (char i = 0; i < daisy_w; i++)
		Daisy.write(daisy_wb[i]);

	// Reset message buffer
	daisy_w = 0;
}

byte daisy_checksum(byte *buffer, int len)
{
	byte ret = 0;

	for (char i = 0; i < len; i++) {
		ret ^= ~buffer[i];
	}
	return(ret);
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


/******************
 * RGB LED Engine *
 ******************/

#define RGB_MAX_INTENSITY	0xff
#define RGB_DEFAULT_INTENSITY	0x80
#define RGB_BROWNIAN_MAX	0x600

/// RGB Modes
enum {
	RM_SWITCH,
	RM_SWIRL,
	RM_STROBE,
	RM_FADE_FOLLOW,
	RM_BROWNIAN
};

/// RGB Colour Change Modes
enum {
	RCM_ONE,
	RCM_MULTI,
	RCM_RANDOM
};

char rgb_mode = RM_SWITCH;
char rgb_col_mode = RCM_ONE;
int rgb_delay = 2000;
char rgb_col_index = 0;
RGBcol rgb_colour = { 0xFF, 0xFF, 0xFF };

int rgb_on = 0;
int rgb_intensity = RGB_DEFAULT_INTENSITY;
int rgb_intensity_prev = -1;

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

/*
enum {
	RS_STOP,
	RS_FADE_ANIMATE,
	RS_HOLD,
	RS_STROBE_ON,
	RS_STROBE_OFF,
};
*/

#define RS_SWIRL	RS_SWIRL_HOLD
#define RS_STROBE	RS_STROBE_OFF

char rgb_running = 0;
byte rgb_state = RS_STOP;
RGBcol rgb_target = { 0xFF, 0xC0, 0x20 };
RGBcol rgb_output = { 0xFF, 0xC0, 0x20 };
RGBcol rgb_calibrate = { 0xFF, 0xFF, 0xFF };

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

void rgb_send_on();
void rgb_send_mode();
void rgb_send_target();
void rgb_send_delay();
void rgb_send_intensity();

void rgb_clear_state()
{
	rgb_running = 0;
	rgb_hold = 0;
	rgb_mpc[0] = 0;
	rgb_mpc[1] = 0;
	rgb_mpc[2] = 0;
}

void update_rgb()
{
	unsigned int r, g, b;

	if (rgb_on & 0x01) {
		if (rgb_running)
			update_rgb_state();
		if (!rgb_running && rgb_state != RS_STOP)
			rgb_next_state();

		r = map(rgb_output.r, 0, 255, 0, map(rgb_intensity, 0, 255, 0, rgb_calibrate.r));
		g = map(rgb_output.g, 0, 255, 0, map(rgb_intensity, 0, 255, 0, rgb_calibrate.g));
		b = map(rgb_output.b, 0, 255, 0, map(rgb_intensity, 0, 255, 0, rgb_calibrate.b));

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
	int diff, last;

	last = millis();
	diff = last - rgb_mpc_last;
	if (diff < 1)		// Don't update unless there was a change
		return;
	rgb_mpc_last = last;

	last = 0;
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
		if (rgb_mode != RM_FADE_FOLLOW)
			rgb_target = rgb_next_colour();
		rgb_setup_fade(rgb_delay);
		//rgb_send_fade();
		rgb_send_target();
		break;
	    case RS_SWIRL_FADE:
		if (rgb_mode == RM_FADE_FOLLOW) {
			rgb_state = RS_STOP;
			rgb_running = 0;
			return;
		}
		else {
			rgb_state = RS_SWIRL_HOLD;
			rgb_hold = rgb_delay;
			rgb_hold_last = millis();
		}
		break;
	    case RS_STROBE_ON:
		rgb_state = RS_STROBE_OFF;
		rgb_output = rgb_palette[BLACK];
		rgb_hold = rgb_delay;
		rgb_hold_last = millis();
		rgb_send_target();
		break;
	    case RS_STROBE_OFF:
		rgb_state = RS_STROBE_ON;
		rgb_output = rgb_next_colour();
		rgb_hold = 20;
		rgb_hold_last = millis();
		rgb_send_target();
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
		rgb_send_target();
		break;
	    case RS_SOLID:
		rgb_state = RS_STOP;
		rgb_output = rgb_target;
		rgb_send_target();
		rgb_running = 0;
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

/*
void rgb_animate()
{
	rgb_setup_fade();
	rgb_running = 1;
}
*/

/************************
 * RGB Public Functions *
 ************************/

void rgb_enable(char mode)
{
	if (mode >= 0)
		rgb_on = mode;
	else {
		if (++rgb_on >= 4)
			rgb_on = 0;
	}

	rgb_send_on();
	//rgb_send_target();
	//rgb_send_delay();
	//rgb_send_mode();
}

void rgb_set_target_by_index(char col)
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

	//rgb_send_target();
}

void rgb_set_target_by_rgb(byte r, byte g, byte b)
{
	rgb_target.r = r;
	rgb_target.g = g;
	rgb_target.b = b;

	rgb_state = RS_SOLID;
	rgb_running = 1;

	//rgb_send_target();
}

void rgb_set_intensity(int i)
{
	if (i < 0)
		i = 0;
	else if (i > RGB_MAX_INTENSITY)
		i = RGB_MAX_INTENSITY;
	rgb_intensity = (byte) i;

	rgb_send_intensity();
}

void rgb_set_delay(int delay)
{
	rgb_delay = delay;
	rgb_send_delay();
}

void rgb_set_mode(char mode, char col_mode)
{
	rgb_mode = mode;
	rgb_col_mode = col_mode;

	switch (rgb_mode) {
	    case RM_SWITCH:
		rgb_state = RS_SOLID;
		rgb_send_mode(RM_SWITCH, rgb_col_mode);
		break;
	    case RM_SWIRL:
		rgb_state = RS_SWIRL_HOLD;
		rgb_send_mode(RM_FADE_FOLLOW, rgb_col_mode);
		break;
	    case RM_STROBE:
		rgb_state = RS_STROBE_OFF;
		rgb_send_mode(RM_SWITCH, RCM_ONE);
		break;
	    case RM_FADE_FOLLOW:
		rgb_state = RS_SWIRL_HOLD;
		rgb_send_mode(rgb_mode, rgb_col_mode);
		break;
	    case RM_BROWNIAN:
		rgb_state = RS_BROWNIAN;
		rgb_brownian = random(0, RGB_BROWNIAN_MAX);
		rgb_send_mode(RM_SWITCH, RCM_ONE);
		break;
	    default:
		break;
	}
	//rgb_running = 1;
	rgb_running = 0;

}

/******************************
 * DaisyWire Control Messages *
 ******************************/

void rgb_send_on()
{
	char mode;

	mode = (rgb_on & 0x02) >> 1;

	daisy_write_byte(DAISY_ON);
	daisy_write_byte(mode);
	daisy_send_msg();
}

void rgb_send_mode(char mode, char col_mode)
{
	daisy_write_byte(DAISY_SET_MODE);
	daisy_write_byte((mode & 0x0F) | ((col_mode & 0x0F) << 4));
	daisy_send_msg();
}

void rgb_send_target()
{
	daisy_write_byte(DAISY_SET_TARGET);
	daisy_write_byte(rgb_output.c[0]);
	daisy_write_byte(rgb_output.c[1]);
	daisy_write_byte(rgb_output.c[2]);
	daisy_send_msg();
}

void rgb_send_delay()
{
	daisy_write_byte(DAISY_SET_DELAY);
	daisy_write_byte((byte) (rgb_delay & 0x00FF));
	daisy_write_byte((byte) ((rgb_delay & 0xFF00) >> 8));
	daisy_send_msg();
}

void rgb_send_intensity()
{
	daisy_write_byte(DAISY_SET_INTENSITY);
	daisy_write_byte(rgb_intensity);
	daisy_send_msg();
}

void rgb_send_fade()
{
	daisy_write_byte(DAISY_FADE);
	daisy_write_byte((byte) (rgb_delay & 0x00FF));
	daisy_write_byte((byte) ((rgb_delay & 0xFF00) >> 8));
	daisy_write_byte(rgb_output.c[0]);
	daisy_write_byte(rgb_output.c[1]);
	daisy_write_byte(rgb_output.c[2]);
	daisy_send_msg();
}

/**************************
 * Channel Behaviour Code *
 **************************/

#define MAX_CHANNEL	10
int channel = 3;

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
		//rgb_col_index = WHITE;
		//rgb_target = rgb_palette[rgb_col_index];
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(WHITE);
		break;
	    case 2:
		//rgb_col_index = MEDIUM;
		//rgb_target = rgb_palette[rgb_col_index];
		//rgb_state = RS_SOLID;
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(MEDIUM);
		break;
	    case 3:
		//rgb_col_index = WARM;
		//rgb_target = rgb_palette[rgb_col_index];
		//rgb_state = RS_SOLID;
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(WARM);
		break;
	    case 4:
		// TODO set the colour to the current(last) selected colour
		//rgb_target = rgb_palette[rgb_cur_col];
		//rgb_col_index = GOLD;
		//rgb_target = rgb_palette[rgb_col_index];
		//rgb_state = RS_SOLID;
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(GOLD);
		break;
	    case 5:
		// TODO set the colour to the current hue??
		//rgb_state = RS_STROBE;
		//rgb_col_mode = RCM_ONE;
		rgb_set_mode(RM_STROBE, RCM_ONE);
		break;
	    case 6:
		//rgb_state = RS_STROBE;
		//rgb_col_mode = RCM_RANDOM;
		rgb_set_mode(RM_STROBE, RCM_RANDOM);
		break;
	    case 7:
		//rgb_state = RS_SWIRL;
		//rgb_col_mode = RCM_MULTI;
		rgb_set_mode(RM_SWIRL, RCM_MULTI);
		break;
	    case 8:
		//rgb_state = RS_SWIRL;
		//rgb_col_mode = RCM_RANDOM;
		rgb_set_mode(RM_SWIRL, RCM_RANDOM);
		break;
	    case 9:
		//rgb_state = RS_BROWNIAN;
		//rgb_col_index = random(RGB_COL_START, RGB_COL_END);
		//rgb_output = rgb_palette[rgb_col_index];
		//rgb_brownian = random(0, RGB_BROWNIAN_MAX);
		rgb_set_mode(RM_BROWNIAN, RCM_ONE);
		break;
	    default:
		break;
	}
}

/********************
 * Input Processing *
 ********************/

int relay_on = 0;

void process_ir(int key)
{
	int prev_channel = channel;

	if (!(rgb_on & 0x01) && key != 0x248) {
		daisy_write_byte(DAISY_KEY);
		daisy_write_byte((key & 0xFF));
		daisy_write_byte(((key >> 8) & 0xFF));
		daisy_send_msg();
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
		rgb_enable(-1);
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
	    case 0x208:		// Mute
		if (rgb_intensity_prev == -1) {
			rgb_intensity_prev = rgb_intensity;
			rgb_set_intensity(RGB_MAX_INTENSITY);
		}
		else {
			rgb_set_intensity(rgb_intensity_prev);
			rgb_intensity_prev = -1;
		}
		break;
	    case 0x258:		// Volume Up
		rgb_set_intensity(rgb_intensity + (rgb_intensity >> 3) + 1);
		break;
	    case 0x278:		// Volume Down
		rgb_set_intensity(rgb_intensity - (rgb_intensity >> 3) - 1);
		break;
	    case 0x241:		// Channel Set +
		rgb_set_target_by_index(rgb_col_index - 1);
		break;
	    case 0x291:		// Enter
		rgb_set_target_by_index(rgb_col_index + 1);
		break;
	    case 0x201:		// Menu
		rgb_set_delay(rgb_delay / 50);
		break;
	    case 0x2C1:		// Channel Set -
		rgb_set_delay(rgb_delay / 50);
		break;
	    default:
		//process_channel_key(key);
		break;
	}
}

#define SERIAL_MAX_ARGS		5

void process_serial()
{
	char i;
	char nargs = 0;
	char argi[SERIAL_MAX_ARGS];

	LongT temp;

	if (!serial_avail)
		return;

	// Locate the indexes to each word and change spaces and newline to null terminations
	for (i = 0; i < serial_r && serial_rb[i] != '\0'; i++) {
		if (serial_rb[i] == ' ') {
			serial_rb[i] = '\0';
			argi[nargs] = i + 1;
			if (++nargs >= SERIAL_MAX_ARGS)
				break;
		}
		else if (serial_rb[i] == '\n') {
			serial_rb[i] = '\0';
			break;
		}
	}

	// Determine and execute command
	if (!strcmp(serial_rb, "ir")) {			// Transmit IR Code
		if (nargs < 2)
			return;
		if (serial_rb[argi[0]] == 'S') {
			temp.value = strtol(&serial_rb[argi[1]], NULL, 16);
			ir_send_sony(temp.value);
		}
		else if (serial_rb[argi[0]] == 'P') {
			int addr;

			if (nargs < 3)
				return;
			addr = strtol(&serial_rb[argi[1]], NULL, 16);
			temp.value = strtol(&serial_rb[argi[2]], NULL, 16);
			ir_send_panasonic(addr, temp.value);
		}	
	}
	else if (!strcmp(serial_rb, "key")) {		// Receive Key (IR Input for RGBNode, not for transmission)
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			process_ir((int) temp.value);
		}
	}
	else if (!strcmp(serial_rb, "power")) {		// RGB Power On/Off
		if (nargs) {
			char mode = strtol(&serial_rb[argi[0]], NULL, 16);
			if (mode >= 0 && mode <= 3)
				rgb_enable(mode);
		}
		else {
			rgb_enable(-1);
		}
	}
	else if (!strcmp(serial_rb, "color")) {		// Set RGB Colour
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_set_target_by_rgb(temp.bytes[2], temp.bytes[1], temp.bytes[0]);
		}
		else {
			// TODO return colour
			//Serial.print("color", ...);
		}
	}
	else if (!strcmp(serial_rb, "red")) {		// Set Red Value
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_output.r = temp.value;
		}
		else {
			Serial.print("red ");
			Serial.print(rgb_output.r, HEX);
			Serial.write('\n');
		}
	}
	else if (!strcmp(serial_rb, "green")) {		// Set Green Value
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_output.g = temp.value;
		}
		else {
			Serial.print("green ");
			Serial.print(rgb_output.g, HEX);
			Serial.write('\n');
		}
	}
	else if (!strcmp(serial_rb, "blue")) {		// Set Blue Value
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_output.b = temp.value;
		}
		else {
			Serial.print("blue ");
			Serial.print(rgb_output.b, HEX);
			Serial.write('\n');
		}
	}
	else if (!strcmp(serial_rb, "delay")) {		// Set RGB Delay
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			//rgb_delay = temp.value;
			rgb_set_delay(temp.value);
		}
		else {
			Serial.print("delay ");
			Serial.print(rgb_delay, HEX);
			Serial.write('\n');
		}
	}
	else if (!strcmp(serial_rb, "channel")) {	// Set RGB Channel/Mode
		if (nargs) {
			char *arg = &serial_rb[argi[0]];
			if (arg[1] == '\0' && arg[0] >= '0' && arg[0] <= '9')
				set_channel(arg[0] - '0');
		}
	}
	else if (!strcmp(serial_rb, "index")) {		// Set RGB Palette Colour
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_set_target_by_index(temp.value);
		}
	}
	else if (!strcmp(serial_rb, "intensity")) {	// Set RGB Intensity
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			rgb_set_intensity(temp.value);
		}
		else {
			Serial.print("intensity ");
			Serial.print(rgb_intensity, HEX);
			Serial.write('\n');
		}
	}
	else if (!strcmp(serial_rb, "chanup")) {	// Increment RGB Channel
		set_channel(channel + 1);
	}
	else if (!strcmp(serial_rb, "chandown")) {	// Decrement RGB Channel
		set_channel(channel - 1);
	}
	else if (!strcmp(serial_rb, "indexup")) {	// Increment RGB Index Colour
		rgb_set_target_by_index(rgb_col_index + 1);
	}
	else if (!strcmp(serial_rb, "indexdown")) {	// Decrement RGB Index Colour
		rgb_set_target_by_index(rgb_col_index - 1);
	}
	else if (!strcmp(serial_rb, "calibrate")) {	// Send calibration RGB values to slave
		if (nargs) {
			temp.value = strtol(&serial_rb[argi[0]], NULL, 16);
			daisy_write_byte(DAISY_CALIBRATE);
			daisy_write_byte(temp.bytes[2]);
			daisy_write_byte(temp.bytes[1]);
			daisy_write_byte(temp.bytes[0]);
			daisy_send_msg();
		}
	}
	else if (!strcmp(serial_rb, "relay_toggle")) {		// Control wireless relay
		if (nargs) {
			if (wrelay) {
				wrelay = 0;
				digitalWrite(PIN_WRELAY_OFF, 1);
				delay(1000);
				digitalWrite(PIN_WRELAY_OFF, 0);
			}
			else {
				wrelay = 1;
				digitalWrite(PIN_WRELAY_ON, 1);
				delay(1000);
				digitalWrite(PIN_WRELAY_ON, 0);
			}
		}
	}
}

void process_daisy()
{
	if (!daisy_avail)
		return;

	switch (daisy_rb[0]) {
	    case DAISY_ON:
		rgb_enable(daisy_rb[1]);
		break;
	    case DAISY_SET_MODE:
		rgb_set_mode((daisy_rb[1] & 0x0F), ((daisy_rb[1] & 0xF0) >> 4));
		break;
	    case DAISY_SET_TARGET:
		//rgb_state = RS_STOP;
		//rgb_running = 0;
		if (rgb_mode == RM_FADE_FOLLOW) {
			rgb_state = RS_SWIRL_HOLD;
			rgb_running = 0;
		}
		else if (rgb_state == RS_STOP) {
			rgb_state = RS_SOLID;
			rgb_running = 1;
		}

		rgb_target.c[0] = daisy_rb[1];
		rgb_target.c[1] = daisy_rb[2];
		rgb_target.c[2] = daisy_rb[3];
		break;
	/*
	    case DAISY_FADE:
		rgb_clear_state();
		//rgb_output.c[0] = rgb_target.c[0];
		//rgb_output.c[1] = rgb_target.c[1];
		//rgb_output.c[2] = rgb_target.c[2];

		rgb_state = RS_SWIRL_FADE;
		rgb_running = 1;
		rgb_delay = daisy_rb[1];
		rgb_delay |= (daisy_rb[2] << 8);
		rgb_target.c[0] = daisy_rb[3];
		rgb_target.c[1] = daisy_rb[4];
		rgb_target.c[2] = daisy_rb[5];
		rgb_setup_fade(rgb_delay);
		break;
	*/
	    case DAISY_SET_DELAY:
		rgb_delay = daisy_rb[1];
		rgb_delay |= (daisy_rb[2] << 8);
		break;
	    case DAISY_SET_INTENSITY:
		rgb_intensity = daisy_rb[1];
		break;
	    case DAISY_CALIBRATE:
		rgb_calibrate.c[0] = daisy_rb[1];
		rgb_calibrate.c[1] = daisy_rb[2];
		rgb_calibrate.c[2] = daisy_rb[3];
		break;
	    case DAISY_KEY:
		int key;

		key = (byte) daisy_rb[2];
		key <<= 8;
		key |= (byte) daisy_rb[1];
		process_ir(key);
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
	pinMode(PIN_WRELAY_ON, OUTPUT);
	pinMode(PIN_WRELAY_OFF, OUTPUT);
	pinMode(PIN_LED, OUTPUT);

	pinMode(PIN_RED, OUTPUT);
	pinMode(PIN_GREEN, OUTPUT);
	pinMode(PIN_BLUE, OUTPUT);
	analogWrite(PIN_RED, 0);
	analogWrite(PIN_GREEN, 0);
	analogWrite(PIN_BLUE, 0);

	// Put Timer1 into FastPWM mode to match Timer0.  This should eliminate the flickering effect
	bitSet(TCCR1B, WGM12);

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



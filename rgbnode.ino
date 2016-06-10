 

#include <SoftwareSerial.h>
#include <IRremote.h>

#include "colours.h"
#include "NerveSerial.h"

#define VERSION		"0.9.5"

/*******************
 * Pin Assignments *
 *******************/

#define PIN_IRTX	3
#define PIN_IRRX	12

#define PIN_RED		6
#define PIN_GREEN	5
#define PIN_BLUE	9

#define PIN_DAISYRX	7
#define PIN_DAISYTX	8


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

/******************************
 * Serial Communications Code *
 ******************************/

#define SERIAL_SPEED	19200

NerveSerial nSerial(&Serial);


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

#define DAISY_SIZE	8
#define DAISY_SPEED	19200

char daisy_r = 0;
char daisy_size = 0;
char daisy_avail = 0;
char daisy_ready = 0;
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

	// TODO you could possibly combine the size and command into one byte (A3, B3, C4, D5, E3, F5, 94 <-- possible command/size bytes for all daisywire commands)
	if (!daisy_size) {
		/// In order to start reading a new message, we must have first read a 0x00 char from the bus, followed by a
		/// non-zero number less than DAISY_SIZE.  We will ignore data until these conditions are met.
		if (b == 0)
			daisy_ready = 1;
		if (daisy_ready && b > 0 && b < DAISY_SIZE) {
			daisy_size = b;
			daisy_ready = 0;
		}
	}
	else {
		daisy_rb[daisy_r++] = b;
		if (daisy_r == daisy_size) {
			/*
			Serial.print("debugread ");
			Serial.print(daisy_r, DEC);
			Serial.write(' ');
			for (char i = 0; i < daisy_r; i++) {
				Serial.print((byte) daisy_rb[i], HEX);
				Serial.write(' ');
			}
			Serial.write('\n');
			*/

			checksum = daisy_checksum(daisy_rb, daisy_r - 1);
			if (daisy_rb[daisy_r - 1] == checksum)
				daisy_avail = 1;
			else {
				Serial.print("debug checksumfail\n");
				clear_daisy();
			}
		}
		if (daisy_r == DAISY_SIZE)
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

	/*
	Serial.print("debugwrite ");
	Serial.print(daisy_w, DEC);
	Serial.write(' ');
	for (char i = 0; i < daisy_w; i++) {
		Serial.print((byte) daisy_wb[i], HEX);
		Serial.write(' ');
	}
	Serial.write('\n');
	*/

	// TODO This is a convenient hack, but is there a better solution?  Sometimes the slave freezes,
	// and while I'm not completely sure, I think it's because it gets some garbled bytes while reading
	// the previous message (checksum failed), and misinterprets the next message, never getting back
	// in sync.

	/// Before each message, send a couple "null messages" to flush the link of any failed messages
	Daisy.write((byte) 0x00);
	Daisy.write((byte) 0x00);

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
int ir_type;
int ir_code;
int ir_repeat;
int ir_repeat_count = 0;
char *ir_types[] = { "x", "N", "S", "RC5", "RC6", "D", "SH", "P", "J", "SA", "M" };

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
		ir_type = ir_data.decode_type;
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
#define RGB_MAX_DELAY		0x7fff
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
int rgb_delay = 16000;
char rgb_col_index = 0;
RGBcol rgb_colour = { 0xFF, 0xFF, 0xFF };

int rgb_on = 0;
int rgb_intensity = RGB_DEFAULT_INTENSITY;
int rgb_intensity_prev = -1;

/// RGB Frame and State Machines
enum {
	RF_SWIRL_HOLD,
	RF_SWIRL_FADE,
	RF_STROBE_ON,
	RF_STROBE_OFF
};

enum {
	RS_STOPPED,
	RS_FADING,
	RS_HOLDING,
};

char rgb_frame = 0;
char rgb_state = RS_STOPPED;
char rgb_target_updated = 0;
RGBcol rgb_target = { 0xFF, 0xC0, 0x20 };
RGBcol rgb_output = { 0xFF, 0xC0, 0x20 };
RGBcol rgb_calibrate = { 0xFF, 0xFF, 0xFF };

/// If the Millisecond per Change value is postive, channel is incremented every * milliseconds.
/// If negative, channel is decremented every * milliseconds
int rgb_mpc[3] = { 0, 0, 0 };		// Millisecond per Change
int rgb_mpc_remain[3] = { 0, 0, 0 };	// Milliseconds Left Over
unsigned long rgb_mpc_last;

int rgb_hold = 0;
unsigned long rgb_hold_last;

int rgb_brownian = 0;

void rgb_do_fade(void);
void rgb_do_hold(void);
void rgb_next_frame(void);
RGBcol rgb_next_colour(void);

void rgb_send_on();
void rgb_send_mode();
void rgb_send_target();
void rgb_send_delay();
void rgb_send_intensity();

void update_rgb()
{
	unsigned int r, g, b;

	if (rgb_on & 0x01) {
		if (rgb_state == RS_STOPPED)
			rgb_next_frame();
		else if (rgb_state == RS_FADING)
			rgb_do_fade();
		else if (rgb_state == RS_HOLDING)
			rgb_do_hold();

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

void rgb_clear_state()
{
	rgb_state = RS_STOPPED;
	rgb_hold = 0;

	rgb_mpc_last = 0;
	rgb_mpc[0] = 0;
	rgb_mpc[1] = 0;
	rgb_mpc[2] = 0;
	rgb_mpc_remain[0] = 0;
	rgb_mpc_remain[1] = 0;
	rgb_mpc_remain[2] = 0;
}

void rgb_setup_fade(int delay)
{
	rgb_state = RS_FADING;
	rgb_mpc_last = millis();

	for (int i = 0; i < 3; i++) {
		if (rgb_target.c[i] == rgb_output.c[i])
			rgb_mpc[i] = 0;
		else
			rgb_mpc[i] = delay / (rgb_target.c[i] - rgb_output.c[i]);
	}
}

void rgb_do_fade(void)
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

	if (last == 0)
		rgb_clear_state();
}

void rgb_setup_hold(int delay)
{
	rgb_state = RS_HOLDING;
	rgb_hold = delay;
	rgb_hold_last = millis();
}

void rgb_do_hold()
{
	if (rgb_hold == 0 || (millis() - rgb_hold_last) > rgb_hold)
		rgb_clear_state();
}

void rgb_next_frame(void)
{
	switch (rgb_mode) {
	    case RM_SWITCH: {
		if (rgb_target_updated) {
			rgb_target_updated = 0;
			rgb_output = rgb_target;
			rgb_send_target();
		}
		break;
	    }
	    case RM_SWIRL: {
		if (rgb_frame == RF_SWIRL_FADE) {
			rgb_frame = RF_SWIRL_HOLD;
			rgb_setup_hold(rgb_delay);
		}
		else {
			rgb_frame = RF_SWIRL_FADE;
			rgb_target = rgb_next_colour();
			rgb_setup_fade(rgb_delay);
			rgb_send_target();
		}
		break;
	    }
	    case RM_STROBE: {
		int delay;

		if (rgb_frame == RF_STROBE_ON) {
			rgb_frame = RF_STROBE_OFF;
			rgb_output = rgb_palette[BLACK];
			delay = rgb_delay >> 4;
			if (delay < 100)
				delay = 100;
			rgb_setup_hold(delay);
		}
		else {
			rgb_frame = RF_STROBE_ON;
			rgb_output = rgb_next_colour();
			rgb_setup_hold(20);
		}
		rgb_target = rgb_output;
		rgb_send_target();
		break;
	    }
	    case RM_FADE_FOLLOW: {
		if (rgb_target_updated) {
			rgb_target_updated = 0;
			rgb_setup_fade(rgb_delay);
			rgb_send_target();
		}
		break;
	    }
	    case RM_BROWNIAN: {
		//int r, v;
		//r = random(0, 3);
		//v = rgb_output.c[r] + random(-1, 2);
		//if (v >= 0 && v <= 255)
		//	rgb_output.c[r] = v;

		// TODO this is just the old code
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

		rgb_target = rgb_output;
		rgb_send_target();
		break;
	    }
	    default:
		break;
	}
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
	//rgb_send_delay();
	rgb_send_mode(rgb_mode, rgb_col_mode);
	rgb_send_target();
}

void rgb_set_mode(char mode, char col_mode)
{
	rgb_mode = mode;
	rgb_col_mode = col_mode;

	rgb_frame = 0;
	rgb_clear_state();

	if (rgb_on & 0x01) {
		/// if this device is on, then treat the next device as a slave
		switch (rgb_mode) {
		    case RM_SWITCH:
			rgb_send_mode(RM_SWITCH, rgb_col_mode);
			break;
		    case RM_SWIRL:
			rgb_send_mode(RM_FADE_FOLLOW, rgb_col_mode);
			break;
		    case RM_STROBE:
			rgb_send_mode(RM_SWITCH, RCM_ONE);
			break;
		    case RM_FADE_FOLLOW:
			rgb_send_mode(rgb_mode, rgb_col_mode);
			break;
		    case RM_BROWNIAN:
			rgb_brownian = random(0, RGB_BROWNIAN_MAX);
			rgb_send_mode(RM_SWITCH, RCM_ONE);
			break;
		    default:
			break;
		}
	}
	else {
		/// otherwise just forward it the new mode
		rgb_send_mode(rgb_mode, rgb_col_mode);
	}

	rgb_next_frame();
}

void rgb_set_target_by_index(char col)
{
	rgb_col_index = col;

	if (rgb_col_index < 0)
		rgb_col_index = 0;
	else if (rgb_col_index >= NUM_PALETTE)
		rgb_col_index = NUM_PALETTE - 1;

	rgb_target = rgb_palette[rgb_col_index];

	rgb_target_updated = 1;
	rgb_next_frame();
}

void rgb_set_target_by_rgb(byte r, byte g, byte b)
{
	rgb_target.r = r;
	rgb_target.g = g;
	rgb_target.b = b;

	rgb_target_updated = 1;
	rgb_next_frame();
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
	if (delay < 0)
		delay = 0;
	else if (delay > RGB_MAX_DELAY)
		delay = RGB_MAX_DELAY;
	rgb_delay = delay;

	rgb_send_delay();
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
	daisy_write_byte(rgb_target.c[0]);
	daisy_write_byte(rgb_target.c[1]);
	daisy_write_byte(rgb_target.c[2]);
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

	switch (channel) {
	    case 1:
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(WHITE);
		break;
	    case 2:
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(MEDIUM);
		break;
	    case 3:
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(WARM);
		break;
	    case 4:
		// TODO set the colour to the current(last) selected colour
		rgb_set_mode(RM_SWITCH, RCM_ONE);
		rgb_set_target_by_index(GOLD);
		break;
	    case 5:
		// TODO set the colour to the current hue??
		rgb_set_mode(RM_STROBE, RCM_ONE);
		break;
	    case 6:
		rgb_set_mode(RM_STROBE, RCM_RANDOM);
		break;
	    case 7:
		rgb_set_mode(RM_SWIRL, RCM_MULTI);
		break;
	    case 8:
		rgb_set_mode(RM_SWIRL, RCM_RANDOM);
		break;
	    case 9:
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

char process_ir(int key)
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
		//relay_on = relay_on ? 0 : 1;
		//digitalWrite(PIN_RELAY, relay_on);
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
		rgb_set_delay(rgb_delay + 50);
		break;
	    case 0x2C1:		// Channel Set -
		rgb_set_delay(rgb_delay - 50);
		break;
	    default:
		return(-1);
	}
	return(0);
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
		rgb_set_target_by_rgb(daisy_rb[1], daisy_rb[2], daisy_rb[3]);
		break;
	    case DAISY_SET_DELAY:
		int delay;

		delay = daisy_rb[1];
		delay |= (daisy_rb[2] << 8);
		rgb_set_delay(delay);
		break;
	    case DAISY_SET_INTENSITY:
		rgb_set_intensity(daisy_rb[1]);
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


void command_ir()
{
	long code, addr;

	if (nSerial.get_arg(0)[0] == 'S') {
		code = strtol(nSerial.get_arg(1), NULL, 0);
		ir_send_sony(code);
	}
	else if (nSerial.get_arg(0)[0] == 'P') {
		if (nSerial.num_args() < 3)
			return;
		addr = strtol(nSerial.get_arg(1), NULL, 0);
		code = strtol(nSerial.get_arg(2), NULL, 0);
		ir_send_panasonic(addr, code);
	}
}

void command_key()
{
	int key;

	key = strtol(nSerial.get_arg(0), NULL, 0);
	process_ir(key);
}

void command_power()
{
	char arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 10);
		if (arg >= 0 && arg <= 3)
			rgb_enable(arg);
	}
	else
		rgb_enable(-1);
}

void command_color()
{
	LongT arg;

	if (nSerial.num_args() > 0) {
		arg.value = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_target_by_rgb(arg.bytes[2], arg.bytes[1], arg.bytes[0]);
	}
	Serial.write(' ');
	Serial.print((rgb_target.r & 0xf0) >> 4, HEX);
	Serial.print(rgb_target.r & 0x0f, HEX);
	Serial.print((rgb_target.g & 0xf0) >> 4, HEX);
	Serial.print(rgb_target.g & 0x0f, HEX);
	Serial.print((rgb_target.b & 0xf0) >> 4, HEX);
	Serial.print(rgb_target.b & 0x0f, HEX);
}

void command_red()
{
	int arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_target_by_rgb(arg, rgb_target.g, rgb_target.b);
	}
	nSerial.print_arg(rgb_output.r, HEX);
}

void command_green()
{
	int arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_target_by_rgb(rgb_target.r, arg, rgb_target.b);
	}
	nSerial.print_arg(rgb_output.g, HEX);
}

void command_blue()
{
	int arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_target_by_rgb(rgb_target.r, rgb_target.g, arg);
	}
	nSerial.print_arg(rgb_output.b, HEX);
}

void command_delay()
{
	int arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_delay(arg);
	}
	nSerial.print_arg(rgb_delay, HEX);
}

void command_channel()
{
	int arg;

	arg = strtol(nSerial.get_arg(0), NULL, 0);
	set_channel(arg);
	nSerial.print_arg(channel, DEC);
}

void command_index()
{
	int arg;

	arg = strtol(nSerial.get_arg(0), NULL, 0);
	rgb_set_target_by_index(arg);
	nSerial.print_arg(rgb_col_index, HEX);
}

void command_intensity()
{
	int arg;

	if (nSerial.num_args() > 0) {
		arg = strtol(nSerial.get_arg(0), NULL, 0);
		rgb_set_intensity(arg);
	}
	nSerial.print_arg(rgb_intensity, HEX);
}

void command_chanup()
{
	set_channel(channel + 1);
}

void command_chandown()
{
	set_channel(channel - 1);
}

void command_indexup()
{
	rgb_set_target_by_index(rgb_col_index + 1);
}

void command_indexdown()
{
	rgb_set_target_by_index(rgb_col_index - 1);
}

void command_calibrate()
{
	LongT arg;

	arg.value = strtol(nSerial.get_arg(0), NULL, 0);
	daisy_write_byte(DAISY_CALIBRATE);
	daisy_write_byte(arg.bytes[2]);
	daisy_write_byte(arg.bytes[1]);
	daisy_write_byte(arg.bytes[0]);
	daisy_send_msg();
	nSerial.print_arg(arg.value, HEX);
}

void command_version()
{
	nSerial.print_arg(VERSION);
}

void command_default()
{
	Serial.print("error\n");
}

NerveCommand_t command_list[] = {
	{ "ir", 2, command_ir },
	{ "key", 1, command_key },
	{ "power", 0, command_power },
	{ "color", 1, command_color },
	{ "red", 1, command_red },
	{ "green", 1, command_green },
	{ "blue", 1, command_blue },
	{ "delay", 1, command_delay },
	{ "channel", 1, command_channel },
	{ "index", 1, command_index },
	{ "intensity", 1, command_intensity },
	{ "chanup", 0, command_chanup },
	{ "chandown", 0, command_chandown },
	{ "indexup", 0, command_indexup },
	{ "indexdown", 0, command_indexdown },
	{ "calibrate", 1, command_calibrate },
	{ "version", 0, command_version },
	{ 0, 0, command_default }
};

/****************
 * Setup & Loop *
 ****************/

void setup()
{
	randomSeed(analogRead(0));

	pinMode(PIN_RED, OUTPUT);
	pinMode(PIN_GREEN, OUTPUT);
	pinMode(PIN_BLUE, OUTPUT);
	analogWrite(PIN_RED, 0);
	analogWrite(PIN_GREEN, 0);
	analogWrite(PIN_BLUE, 0);

	// Put Timer1 into FastPWM mode to match Timer0.  This should eliminate the flickering effect
	bitSet(TCCR1B, WGM12);

	Serial.begin(SERIAL_SPEED);
	nSerial.set_commands(command_list);
	Daisy.begin(DAISY_SPEED);
	clear_daisy();
	irrecv.enableIRIn();

	Daisy.write((byte) 0x00);
	Daisy.write((byte) 0x00);
	Daisy.write((byte) 0x00);
	Daisy.write((byte) 0x00);
}

void loop()
{
	nSerial.check_read();

	if (read_daisy()) {
		process_daisy();
		clear_daisy();
	}

	if (read_ir()) {
		if (process_ir(ir_code) == -1) {
			nSerial.print("irrecv ");
			nSerial.print(ir_types[(ir_data.decode_type < 0) ? 0 : ir_data.decode_type]);
			nSerial.print(":");
			if (ir_data.decode_type == PANASONIC) {
				nSerial.print(ir_data.panasonicAddress, HEX);
				nSerial.print(":");
			}
			nSerial.print(ir_data.value, HEX);
			nSerial.print("\n");
		}
	}

	update_rgb();
}



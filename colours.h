
#ifndef COLOURS_H
#define COLOURS_H

#include "Arduino.h"

typedef union {
	struct {
		byte r;
		byte g;
		byte b;
	};
	byte c[3];
} RGBcol;

enum {
	BLACK,
	RED, RED_ORANGE, ORANGE, YELLOW_ORANGE, YELLOW, YELLOW_GREEN, SOMETHING, LIGHT_GREEN,
	GREEN, COL1, COL2, COL3, COL4, COL5, COL6, COL7,
	BLUE, COL11, COL22, COL33, COL44, COL55, COL66, COL77,
	WHITE, MEDIUM, WARM, GOLD, FAV1, FAV2,
	NUM_PALETTE
};

RGBcol rgb_palette[] = {
	{   0,   0,   0 },

	{ 255,   0,   0 },
	{ 255,  32,   0 },
	{ 255,  64,   0 },
	{ 255, 128,   0 },
	{ 255, 255,   0 },
	{ 128, 255,   0 },
	{  64, 255,   0 },
	{  32, 255,   0 },

	{   0, 255,   0 },
	{   0, 255,  32 },
	{   0, 255,  64 },
	{   0, 255, 128 },
	{   0, 255, 255 },
	{   0, 128, 255 },
	{   0,  64, 255 },
	{   0,  32, 255 },

	{   0,   0, 255 },
	{  32,   0, 255 },
	{  64,   0, 255 },
	{ 128,   0, 255 },
	{ 255,   0, 255 },
	{ 255,   0, 128 },
	{ 255,   0,  64 },
	{ 255,   0,  32 },

	{ 255, 255, 255 },
	{ 255, 255, 128 },
	{ 255, 255,  64 },
	{ 255, 192,   0 },
	{  16, 255,   0 },
	{   0,  80, 255 }
};

#define RGB_COL_START	RED
#define RGB_COL_END	WHITE

#define RGB_WHITE_START	WHITE
#define RGB_WHITE_END	NUM_PALETTE

#endif
 

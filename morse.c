#include <xc.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "morse.h"



const char lookup_none[] = "";
const char lookup_A[] = ".-";
const char lookup_B[] = "-...";
const char lookup_C[] = "-.-.";
const char lookup_D[] = "-..";
const char lookup_E[] = ".";
const char lookup_F[] = "..-.";
const char lookup_G[] = "--.";
const char lookup_H[] = "....";
const char lookup_I[] = "..";
const char lookup_J[] = ".---";
const char lookup_K[] = "-.-";
const char lookup_L[] = ".-..";
const char lookup_M[] = "--";
const char lookup_N[] = "-.";
const char lookup_O[] = "---";
const char lookup_P[] = ".--.";
const char lookup_Q[] = "--.-";
const char lookup_R[] = ".-.";
const char lookup_S[] = "...";
const char lookup_T[] = "-";
const char lookup_U[] = "..-";
const char lookup_V[] = "...-";
const char lookup_W[] = ".--";
const char lookup_X[] = "-..-";
const char lookup_Y[] = "-.--";
const char lookup_Z[] = "--..";
const char lookup_0[] = "-----";
const char lookup_1[] = ".----";
const char lookup_2[] = "..---";
const char lookup_3[] = "...--";
const char lookup_4[] = "....-";
const char lookup_5[] = ".....";
const char lookup_6[] = "-....";
const char lookup_7[] = "--...";
const char lookup_8[] = "---..";
const char lookup_9[] = "----.";
const char lookup_dot[] = ".-.-.-";
const char lookup_comma[] = "--..--";
const char lookup_colon[] = "---...";
const char lookup_questionmark[] = "..--..";
const char lookup_apostrophe[] = ".----.";
const char lookup_slash[] = "-..-.";
const char lookup_at[] = ".--.-.";
const char lookup_equal[] = "-...-";


static const char* lookup(char in);
static void setBit(uint8_t *array_ptr, uint16_t position);
static void clearBit(uint8_t *array_ptr, uint16_t position);
static uint16_t writeChar(uint8_t *array_ptr, uint16_t position, char character);


static const char* lookup(char in)
{
	//switch (toupper(in))
    switch (in)
	{
		case 'A':
			return &lookup_A[0];
		case 'B':
			return &lookup_B[0];
		case 'C':
			return &lookup_C[0];
		case 'D':
			return &lookup_D[0];
		case 'E':
			return &lookup_E[0];
		case 'F':
			return &lookup_F[0];
		case 'G':
			return &lookup_G[0];
		case 'H':
			return &lookup_H[0];
		case 'I':
			return &lookup_I[0];
		case 'J':
			return &lookup_J[0];
		case 'K':
			return &lookup_K[0];
		case 'L':
			return &lookup_L[0];
		case 'M':
			return &lookup_M[0];
		case 'N':
			return &lookup_N[0];
		case 'O':
			return &lookup_O[0];
		case 'P':
			return &lookup_P[0];
		case 'Q':
			return &lookup_Q[0];
		case 'R':
			return &lookup_R[0];
		case 'S':
			return &lookup_S[0];
		case 'T':
			return &lookup_T[0];
		case 'U':
			return &lookup_U[0];
		case 'V':
			return &lookup_V[0];
		case 'W':
			return &lookup_W[0];
		case 'X':
			return &lookup_X[0];
		case 'Y':
			return &lookup_Y[0];
		case 'Z':
			return &lookup_Z[0];
		case '0':
			return &lookup_0[0];
		case '1':
			return &lookup_1[0];
		case '2':
			return &lookup_2[0];
		case '3':
			return &lookup_3[0];
		case '4':
			return &lookup_4[0];
		case '5':
			return &lookup_5[0];
		case '6':
			return &lookup_6[0];
		case '7':
			return &lookup_7[0];
		case '8':
			return &lookup_8[0];
		case '9':
			return &lookup_9[0];
		case '.':
			return &lookup_dot[0];
		case ',':
			return &lookup_comma[0];
		case ':':
			return &lookup_colon[0];
		case '?':
			return &lookup_questionmark[0];
		case '\'':
			return &lookup_apostrophe[0];
		case '/':
			return &lookup_slash[0];
		case '@':
			return &lookup_at[0];
		case '=':
			return &lookup_equal[0];
	}
	return &lookup_none[0];
}

static void setBit(uint8_t *array_ptr, uint16_t position)
{
	uint16_t byte_nbr = position >> 3;
	uint8_t bit_nbr = 7 - (position & 0b111);
    if(position<BITSTRING_LENGTH_BITS)
        array_ptr[byte_nbr] |= (0b1 << bit_nbr);
}

static void clearBit(uint8_t *array_ptr, uint16_t position)
{
	uint16_t byte_nbr = position >> 3;
	uint8_t bit_nbr = 7 - (position & 0b111);
    if(position<BITSTRING_LENGTH_BITS)
        array_ptr[byte_nbr] &= ~(0b1 << bit_nbr);
}

static uint16_t writeChar(uint8_t *array_ptr, uint16_t position, char character)
{
	const char *substring_ptr = lookup(character);
	if (*substring_ptr)
	{
		while (*substring_ptr)
		{
			if (*substring_ptr == '.')
			{
				setBit(array_ptr, position++);
			}
			else
			{
				setBit(array_ptr, position++);
				setBit(array_ptr, position++);
				setBit(array_ptr, position++);
			}
			clearBit(array_ptr, position++);
			++substring_ptr;
		}
		clearBit(array_ptr, position++);
		clearBit(array_ptr, position++);
	}
    if(position<BITSTRING_LENGTH_BITS)
        return position;
    else
        return BITSTRING_LENGTH_BITS-1;
}

uint16_t compile(char *in, uint8_t *out)
{
	uint16_t bitcount = 0;
	while (*in)
	{
        CLRWDT();
		switch (*in)
		{
			case ' ':
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				break;
			case '-':
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				clearBit(out, bitcount++);
				break;
			case '_':
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				setBit(out, bitcount++);
				break;
			default:
                bitcount = writeChar(out, bitcount, *in);
		}
		++in;
	}
	return bitcount;
}

uint8_t getBit(uint8_t *array_ptr, uint16_t position)
{
	uint16_t byte_nbr = position >> 3;
	uint8_t bit_nbr = 7 - (position & 0b111);
	if (array_ptr[byte_nbr] & (0b1 << bit_nbr))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
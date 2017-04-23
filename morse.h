/* 
 * File:   morse.h
 * Author: Luke
 *
 * Created on 8. Dezember 2016, 23:52
 */

#ifndef MORSE_H
#define	MORSE_H

#define BITSTRING_LENGTH_BYTES 200
#define BITSTRING_LENGTH_BITS 1600

uint16_t compile(char *in, uint8_t *out);
uint8_t getBit(uint8_t *array_ptr, uint16_t position);

#endif	/* MORSE_H */


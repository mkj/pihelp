//=============================================================================
// Copyright Atmel Corporation 2003. All Rights Reserved.
//
// File:			des.h
// Compiler:		IAR Atmel AVR C/EC++ Compiler
// Output Size:
// Based on work by:ØE, VU
// Created:			4-Feb-2003	JP (Atmel Finland)
// Modified:	
//
// Support Mail:	avr@atmel.com
//
// Description:		Please refer to Application Note Documentation for more
//					information.
//
//					For details on DES, please refer to the official FIPS 46-3
//					document:
//
//				http://csrc.nist.gov/publications/fips/fips46-3/fips46-3.pdf
//
//=============================================================================

#ifndef AES_H
#define AES_H

//#include "bootldr.h"

extern void aesInit( unsigned char *key, unsigned char * tempbuf );
extern void aesDecrypt(unsigned char *buffer, unsigned char *chainBlock);

#endif // AES_H


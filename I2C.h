/* I2C.h: I2C function prototypes */
/******************************************************************************/
/* This file is part of the uVision/ARM development tools. */
/* Copyright (c) 2005-2009 Keil Software. All rights reserved. */
/* This software may only be used under the terms of a valid, current, */
/* end user licence from KEIL for a compatible version of KEIL software */
/* development tools. Nothing else gives you the right to use this software. */
/******************************************************************************/

//#include "stm32f10x_lib.h"
#include "stm32f4xx.h"
//#include <stdint.h>
extern void I2C_Init(void);
extern void I2C_Start (void);
extern void I2C_Stop (void);
extern void I2C_Addr (unsigned char adr);
extern void I2C_Write (unsigned char c);
extern unsigned char I2C_Read (int ack);
extern unsigned char I2C_getbyte(unsigned char address, unsigned char cmd);
extern void I2C_putbyte(unsigned char address, unsigned char cmd, unsigned char data);
extern unsigned short int I2C_getword(unsigned char address, unsigned char cmd);

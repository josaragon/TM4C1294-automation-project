/*
 * BOSCH_datatypes.h
 *
 *  Created on: 26 oct. 2017
 *      Author: ManoloP
 */

#ifndef BOSCH_DATATYPES_H_
#define BOSCH_DATATYPES_H_

#define  MACHINE_32_BIT

/*signed integer types*/
typedef signed char  s8;/**< used for signed 8bit */
typedef signed short int s16;/**< used for signed 16bit */
typedef signed int s32;/**< used for signed 32bit */
typedef signed long long int s64;/**< used for signed 64bit */

/*unsigned integer types*/
typedef unsigned char u8;/**< used for unsigned 8bit */
typedef unsigned short int u16;/**< used for unsigned 16bit */
typedef unsigned int u32;/**< used for unsigned 32bit */
typedef unsigned long long int u64;/**< used for unsigned 64bit */
#define BME280_64BITSUPPORT_PRESENT

#endif /* BOSCH_DATATYPES_H_ */

/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef MAIN_H
#define	MAIN_H

#include <xc.h>
#include "lcd.h"
#include "hardware.h"

void cmp_sensor(void);

/*******************************************************************************
 * defines for the project 
 ******************************************************************************/

#define CKP_PIN PORTBbits.RB6

/*******************************************************************************
 * typedef for variables types (C data types)
 ******************************************************************************/
typedef unsigned char       uint8_t;  // Unsigned, 8 bits, (0 .. 255).
typedef signed char          int8_t;  // Signed, 8 bits, (-128 .. +127).
typedef unsigned short     uint16_t;  // Unsigned, 16 bits, (0 .. 65,535).
typedef signed short        int16_t;  // Signed, 16 bits, (-32,768..+32,767.
typedef unsigned long      uint32_t;  // unsigned, 32 bits, (0 .. 4,294,967,295)
typedef signed long         int32_t;  // signed, 32 bits, (-2,147,483,648..+2,147,483,647)

/*******************************************************************************
 * defines general
 ******************************************************************************/
#define YES                       1
#define NO                        0
#define ON                        1
#define OFF                       0
#define ENABLE                    1
#define DISABLE                   0 
#define HIGH                      1
#define LOW                       0
#define TRUE                      1
#define FALSE                     0
#define UNPRESSED                 1
#define PRESSED                   0
#define OR                       ||
#define AND                      &&

/******************************************************************************/
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 
#ifdef	__cplusplus
}
#endif /* __cplusplus */
#endif	/* MAIN_H */


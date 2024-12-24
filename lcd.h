/* ****************************************************************************
 * Project: Control Functions             File lcd.c                 March/2022
 * ****************************************************************************
 * File description: Functions for 16x2 LCD display control, with PIC18F4550.      
 * ****************************************************************************
 * Program environment for validation: MPLAB X IDE v6.0, XC8 v2.36, C std C90,
 * PIC18F4550 mounted on FATEC development board (FATEC board) - 20 MHz crystal.
 * ****************************************************************************
 * MIT License  (see: LICENSE em github)
 *   Copyright (c) 2022 Antonio Aparecido Ariza Castilho
 *   <https://github.com/AntonioCastilho>
 * ****************************************************************************
 * Reference:
 * * Microchip PIC18F4550 Datasheet.
 * * MicroChip Developer Help sample program files. 
 *   <https://microchipdeveloper.com/>.
 * * HD44780U dot-matrix liquid crystal display controller Datasheet
 *   <https://www.digchip.com/datasheets/parts/datasheet/740/HD44780U-pdf.php>
 * ****************************************************************************
 * Date      | Author            | Description
 * **********|*******************|*********************************************
 * 03/23/2022| Antonio Castilho  | Function has been created
 ******************************************************************************/

#ifndef LCD_16X2_H
#define	LCD_16X2_H

/******************************************************************************/
// Include header files.
/******************************************************************************/
#include <xc.h>
#include <stdlib.h>
#include <math.h>
#include "main.h"

/******************************************************************************/
// LCD Display pins setting
/******************************************************************************/
#define LCD_E          PORTDbits.RD0 
#define LCD_RS         PORTDbits.RD1
#define LCD_RW         PORTDbits.RD2
//                     PORTDbits.RD3 
#define LCD_D4         PORTDbits.RD4
#define LCD_D5         PORTDbits.RD5
#define LCD_D6         PORTDbits.RD6
#define LCD_D7         PORTDbits.RD4
/******************************************************************************/
// I/O LCD Display pins setting
/******************************************************************************/
#define LCD_TRIS     TRISD 
#define LCD_PORT     PORTD  // PORTD [RD4:RD7] 4 bits LCD Display data.
/******************************************************************************/

/******************************************************************************/
// Macros for cursor positioning
/******************************************************************************/
#define r1c1()     lcd_com(0x80)  // row 1, column 1.
#define r1c2()     lcd_com(0x81)
#define r1c3()     lcd_com(0x82)
#define r1c4()     lcd_com(0x83)
#define r1c5()     lcd_com(0x84)
#define r1c6()     lcd_com(0x85)
#define r1c7()     lcd_com(0x86)
#define r1c8()     lcd_com(0x87)
#define r1c9()     lcd_com(0x88)
#define r1c10()    lcd_com(0x89) 
#define r1c11()    lcd_com(0x8A)
#define r1c12()    lcd_com(0x8B)
#define r1c13()    lcd_com(0x8C)
#define r1c14()    lcd_com(0x8D)
#define r1c15()    lcd_com(0x8E)  
#define r1c16()    lcd_com(0x8F)
#define r2c1()     lcd_com(0xC0) // row 2 , column 1
#define r2c2()     lcd_com(0xC1)
#define r2c3()     lcd_com(0xC2)
#define r2c4()     lcd_com(0xC3)
#define r2c5()     lcd_com(0xC4)
#define r2c6()     lcd_com(0xC5)
#define r2c7()     lcd_com(0xC6)
#define r2c8()     lcd_com(0xC7)
#define r2c9()     lcd_com(0xC8)
#define r2c10()    lcd_com(0xC9)
#define r2c11()    lcd_com(0xCA)
#define r2c12()    lcd_com(0xCB)
#define r2c13()    lcd_com(0xCC)
#define r2c14()    lcd_com(0xCD)
#define r2c15()    lcd_com(0xCE)
#define r2c16()    lcd_com(0xCF)
/******************************************************************************/

/******************************************************************************
 * Macros for Display Control Functions.
 ******************************************************************************/
#define lcd_clear()          lcd_com(0x01) // Display clear.
#define lcd_cursorHome()     lcd_com(0x02) // Put cursor in row and column 0.
#define lcd_cursorOff()      lcd_com(0x0C) // Display on, cursor off.
#define lcd_cursorBlinks()   lcd_com(0x0F) // Display on, cursor blinks
#define lcd_off()            lcd_com(0x08) // Display off.
#define lcd_on()             lcd_com(0x0E) // Display on, cursor on, no blinks.
/******************************************************************************/

/******************************************************************************/
// Function prototypes
/******************************************************************************/
void lcd_com(uint8_t cmd);     // Send a command to the display
void lcd_ini(void);            // initialize the display
void lcd_prtChar(uint8_t dat); // Write char in display.
void lcd_prtStr(const uint8_t row, const uint8_t col, const uint8_t *str); //Write string.
void lcd_prtInt(const uint8_t row, const uint8_t col, const int32_t str);  //Wtite integer

uint8_t digit_counter(uint16_t number); // Counts the number of digits in an integer.

void ms_time(uint16_t ms); // Wastes processing time.
void us_time(uint16_t us); // Wastes processing time.

/******************************************************************************/

#endif	/* LCD_16X2_H */


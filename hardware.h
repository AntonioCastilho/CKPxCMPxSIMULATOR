/*
 * hardware.h
 * Author: Automotive Technologist Antonio Castilho
 * 
 * Program environment: MPLAB X IDE v6.20, XC8 v2.50, C std C90, PIC18F455
 * Reference: Microchip PIC18F4550 Datasheet.
 * 
 * MIT License  (see: LICENSE em github)
 * Copyright (c) 2022 Antonio Aparecido Ariza Castilho
 * <https://github.com/AntonioCastilho>
 *
 * Created on 8 de Dezembro de 2024, 12:01
 */

#ifndef HARDWARE_H
#define	HARDWARE_H

#include <xc.h>
#include "lcd.h"
#include "main.h"

#define _XTAL_FREQ         8000000
#define TMR2PRESCALE            16
#define TMR2POSTCALER            1
#define PWM1_FREQUENCY         900 // 900,0 rpm | 15 Hz | 900 pps (60-2))
#define DUTY_CYCLE             500 // 50,0 %

/*******************************************************************************
 * Function declaration
 ******************************************************************************/
void     pic_ini(void);
void     timer0_ini(void);
void     timer0_write(uint16_t timer0_value);
void     timer1_ini(void);
void     timer1_write(uint16_t timer1_value);
void     timer2_ini(void);
void     adc_ini(void);
void     pwm1_ini(void);
void     pwm2_ini(void);
void     pwm1_setDutyPot(uint16_t ccpr1_aux);
void     pwm2_setDutyPot(uint16_t ccpr2_aux);
void     pwm1_setPeriod(uint8_t period);
void     pwm2_setPeriod(uint8_t period);
uint16_t adc_read(uint8_t ch);
/******************************************************************************/

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* HARDWARE_H */


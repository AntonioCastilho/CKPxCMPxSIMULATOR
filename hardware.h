/*
 * hardware.h
 * Mapeamento de hardware
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
void pic_ini(void);
void timer0_ini(void);
void timer1_ini(void);
void timer1_write(uint16_t timer_value);
void timer2_ini(void);
void timer0_write(uint16_t timer_value);
void timer2_write( uint8_t timer_value);
void adc_ini(void);
uint16_t adc_read(uint8_t ch);
void pwm1_ini(void);
void pwm2_ini(void);
void pwm1_setDutyPot(uint16_t ccpr1_aux);
void pwm2_setDutyPot(uint16_t ccpr2_aux);
void pwm1_setPeriod(uint8_t period);
void pwm2_setPeriod(uint8_t period);

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* HARDWARE_H */


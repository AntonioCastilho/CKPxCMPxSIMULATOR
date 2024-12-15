/* ****************************************************************************
 * Project: Control Functions             File adc.c                 March/2022
 * ****************************************************************************
 * File description:       
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
 * ****************************************************************************
 * Date      | Author            | Description
 * **********|*******************|*********************************************
 * 04/16/2022| Antonio Castilho  | Function has been created
 ******************************************************************************/ 
#include <xc.h>
#include "adc.h"
#include "main.h"

/******************************************************************************
 * Function: void adc_ini();
 * Description: The function starts analog channels.
 * Example: adc_ini(3); // Start analog channels: 0, 1 e 2.
 * Input: Number of desired channels.
 * Output: void
 * ****************************************************************************
 * Date      | Author            | Description
 * **********|*******************|*********************************************
 * 03/22/2022| Antonio Castilho  | Function has been created
 ******************************************************************************/
void adc_ini(void)
{
    // Configure Port A as input, according to the channels that will be needed.
    TRISA = 0x0F;  // channel in AN0.
    ADCON1 = 0x0B; // 4 channels Pg 262.
    ADCON2 = 0xBE; // Right Justified, 4Tad and Fosc/32. Pg 263.
    ADCON0bits.ADON = ON;
    ADRESH=0;	   // Flush ADC output Register. Pg 261.
    ADRESL=0;
}
/* end of function void adc_ini(void)
*******************************************************************************/

/*******************************************************************************
 * Function: int16_t adc_read(uint8_t channel)
 * Description: Read the channel.
 * Input: Channel number.
 * Output: Value read between 0 and 1023, for 5V reference voltage.
 * Created in: 03/22/2022 by Antonio Aparecido Ariza Castilho
 ******************************************************************************/
uint16_t adc_read(uint8_t ch)
{
    uint16_t value;
    
    ADCON0bits.CHS = ch; // selects the channel to be read.
    ADCON0bits.GO = ON;  // start conversion.
    while(ADCON0bits.GO_DONE == TRUE); // wait for the conversion.
    value = (uint16_t)((ADRESH << 8) + ADRESL);
    return value;
}
/* end of function uint16_t adc_read(uint8_t ch)
*******************************************************************************/


uint16_t adjust_value(uint16_t value) {
    
    return ((uint16_t)((((float)value * 5)/1023)*100));
    
}
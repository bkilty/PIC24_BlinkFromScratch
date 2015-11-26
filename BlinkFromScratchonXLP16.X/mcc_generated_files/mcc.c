/**
  @Generated MPLAB® Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24F16KA102
        Driver Version    :  1.02
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

// Configuration bits: selected in the GUI

// FBS
#pragma config BSS = OFF    // Boot segment Protect->No boot program Flash segment
#pragma config BWRP = OFF    // Table Write Protect Boot->Boot segment may be written

// FGS
#pragma config GCP = OFF    // General Segment Code Flash Code Protection bit->No protection
#pragma config GWRP = OFF    // General Segment Code Flash Write Protection bit->General segment may be written

// FOSCSEL
#pragma config FNOSC = FRC    // Oscillator Select->Fast RC oscillator (FRC)
#pragma config IESO = ON    // Internal External Switch Over bit->Internal External Switchover mode enabled (Two-Speed Start-up enabled)

// FOSC
#pragma config POSCMOD = NONE    // Primary Oscillator Configuration bits->Primary oscillator disabled
#pragma config FCKSM = CSDCMD    // Clock Switching and Monitor Selection->Both Clock Switching and Fail-safe Clock Monitor are disabled
#pragma config OSCIOFNC = OFF    // CLKO Enable Configuration bit->CLKO output signal is active on the OSCO pin
#pragma config POSCFREQ = HS    // Primary Oscillator Frequency Range Configuration bits->Primary oscillator/external clock input frequency greater than 8 MHz
#pragma config SOSCSEL = SOSCHP    // SOSC Power Selection Configuration bits->Secondary oscillator configured for high-power operation

// FWDT
#pragma config WDTPS = PS32768    // Watchdog Timer Postscale Select bits->1:32,768
#pragma config FWDTEN = ON    // Watchdog Timer Enable bit->WDT enabled
#pragma config WINDIS = OFF    // Windowed Watchdog Timer Disable bit->Standard WDT selected; windowed WDT disabled
#pragma config FWPSA = PR128    // WDT Prescaler->WDT prescaler ratio of 1:128

// FPOR
#pragma config I2C1SEL = PRI    // Alternate I2C1 Pin Mapping bit->Default location for SCL1/SDA1 pins
#pragma config MCLRE = ON    // MCLR Pin Enable bit->MCLR pin enabled; RA5 input pin disabled
#pragma config PWRTEN = ON    // Power-up Timer Enable bit->PWRT enabled
#pragma config BOREN = BOR3    // Brown-out Reset Enable bits->Brown-out Reset enabled in hardware; SBOREN bit disabled
#pragma config BORV = V18    // Brown-out Reset Voltage bits->Brown-out Reset set to lowest voltage (1.8V)

// FICD
#pragma config ICS = PGx3    // ICD Pin Placement Select bits->PGC3/PGD3 are used for programming and debugging the device
#pragma config COE = OFF    // Set Clip On Emulation bit->Device will reset into Operational Mode
#pragma config BKBUG = OFF    // Background Debugger Enable bit->Background debugger disabled

// FDS
#pragma config RTCOSC = SOSC    // RTCC Reference Clock Select bit->RTCC uses SOSC as reference clock
#pragma config DSWDTEN = ON    // Deep Sleep Watchdog Timer Enable bit->DSWDT enabled
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select bit->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = ON    // Deep Sleep Zero-Power BOR Enable bit->Deep Sleep BOR enabled in Deep Sleep
#pragma config DSWDTPS = DSWDTPSF    // Deep Sleep Watchdog Timer Postscale Select bits->1:2,147,483,648 (25.7 Days)

#include "mcc.h"

void SYSTEM_Initialize(void) {
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    INTERRUPT_Initialize();
    TMR2_Initialize();
    UART2_Initialize();
    TMR3_Initialize();
}

void OSCILLATOR_Initialize(void) {
    // RCDIV FRC/1; DOZE 1:8; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3000;
    // Set the secondary oscillator
    OSCCONbits.SOSCEN = 1;
}

/**
 End of File
 */
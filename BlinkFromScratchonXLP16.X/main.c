/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB® Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.25.2
        Device            :  PIC24F16KA102
        Driver Version    :  2.00
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

#include "mcc_generated_files/mcc.h"



/*
                         Main application
 */
int main(void) {
    // initialize the device
    SYSTEM_Initialize();
    uint16_t portValue;
    char name[]="Hello PI! Brennan and Amanda sitting in a tree KISSING!\r\n";
    uint8_t uartBuffer[128];
    uint8_t i = 0;
    //uint8_t j = 0;
    uint8_t uartBytesWritten = 0;
    uint8_t* tempPtr;
    uint16_t tmrValue;
    UART2_TRANSFER_STATUS status ;
    
    //intialize the Board Power VDD_Board
    IO_RB2_SetLow(); 
    
    //Testing UART code
    while(name[i] != '\0')
    {
        uartBuffer[i] = (uint8_t)name[i];
        i++;
    }

    uartBuffer[i] = (uint8_t)name[i];
    /*
    IO_RB15_SetHigh();
    
    for(j = 0;j<10;j++)
    {
        while(!TMR2_GetElapsedThenClear())
        {
            //IO_RB8_SetHigh();
            IO_RB15_SetHigh();
        }
            
        while(!TMR2_GetElapsedThenClear())
        {
            //IO_RB8_SetLow();
            IO_RB15_SetLow();
        }
    TMR3_delay_ms(_XTAL_FREQ, 500);
       
    }
    IO_RB15_SetHigh();
     */

    while (1) {
        
        //IO_RB15_SetLow();
        
        // Add your application code
        // Read RB7
        //portValue = IO_RB7_GetValue();
        //if(portValue == 1)
        
            
        
        uartBytesWritten = 0;
        
        while(uartBytesWritten < i)
        {
            //status = UART2_TransferStatusGet ( ) ;
            tempPtr = uartBuffer + uartBytesWritten;
            uartBytesWritten += UART2_WriteBuffer(uartBuffer+uartBytesWritten, UART2_TransmitBufferSizeGet());
            UART2_TasksTransmit();
            if (!UART2_TransmitBufferIsFull())
            {
                
                //IO_RB15_SetLow();
                //TMR3_delay_ms(_XTAL_FREQ, 100);
                
            }   
            //IO_RB15_SetHigh();
            
        }
        TMR3_delay_ms(_XTAL_FREQ, 500);
        
    }

    return -1;
}


        
/**
 End of File
 */
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
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Function Prototypes*/
uint8_t uartSend(const uint8_t* message, uint8_t messageLength);
void uartPrintStatus();

/*
                         Main application
 */
int main(void) {
    // initialize the device
    SYSTEM_Initialize();
    uint16_t portValue;
    char introduction[]="\r\nHere are some messages from PIC. \r\n";

    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t sLength = 0;

    //UART Data Variables
    uint8_t uartBytes2Rx;
    uint8_t uartRxBuffer[128];
    uint8_t uartBytes2Tx;
    uint8_t uartTxBuffer[128];
    UART2_TRANSFER_STATUS uartTxStatus ;
    UART2_STATUS uartStatus;
    uint8_t uartBytesWritten = 0;
    
    
    
    //intialize the Board Power VDD_Board
    IO_RB2_SetLow(); 
    
//    //Testing UART code
//    while(name[i] != '\0')
//    {
//        uartBuffer[i] = (uint8_t)name[i];
//        i++;
//    }
//    uartBuffer[i] = (uint8_t)name[i];
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
        
        // Read RB7
        portValue = IO_RB7_GetValue();
        if(portValue == 1)
            IO_RB15_SetLow();
        else
        {
            IO_RB15_SetHigh();
            //print the status of the UART2 Peripheral
            uartPrintStatus();
        }
        
        if(i == 0)
        {
            // char * strcpy(char *, const char *);
            strcpy((char*)uartTxBuffer, (const char *)introduction);
            //size_t strlen(const char *);
            sLength = (uint8_t)strlen((char*)uartTxBuffer);
           // uartBytes2Tx = sLength;
            uartSend(uartTxBuffer,sLength);
            i++;
               
        }
        
        
        if(i!=0)
        {
            //TMR3_delay_ms(_XTAL_FREQ, 3000);//wait for 10ms in sleep mode
            uartTxStatus = UART2_TransferStatusGet();
            uartStatus = UART2_StatusGet();
            if(UART2_TRANSFER_STATUS_RX_DATA_PRESENT & uartTxStatus)
            {                              
                //Get bytes from the UART Receiver
                uartBytes2Rx = UART2_ReadBuffer(uartRxBuffer, 128); //get the bytes
                
                //echo back what what received from Raspberry PI
                uartSend(uartRxBuffer, uartBytes2Rx);
                //uartBytes2Tx = sprintf((char*) uartTxBuffer, "\r\n PIC Received %d bytes --> %s\r\r\r\n", uartBytes2Rx, uartRxBuffer);
                //uartSend(uartTxBuffer, uartBytes2Tx);
                
                //clean up the TxBuffer and RxBuffer void * memset(void *, int, size_t);
                memset((void*)uartRxBuffer, NULL, uartBytes2Rx);
                memset((void*)uartTxBuffer, NULL, uartBytes2Tx);
                 
            }
          
        }
        
        
        //if(i==1)
        //    TMR3_delay_ms(_XTAL_FREQ, 3000); //delay just so I can see the first message
        
    
    }
        
    return -1;
}


////////////////////////////////////////////////////////////////////////////////
/*uartPrintStatus -- get status of UART Channel 2 and send it over UART2
 * 
 * UART2_STATUS enumeration
        //   UART2_RX_DATA_AVAILABLE = (1 << 0) 
        //   UART2_RX_OVERRUN_ERROR = (1 << 1) 
        //   UART2_FRAMING_ERROR = (1 << 2) 
        //   UART2_PARITY_ERROR = (1 << 3) 
        //   UART2_RECEIVER_IDLE = (1 << 4) 
        //   UART2_TX_COMPLETE = (1 << 8)
        //   UART2_TX_FULL = (1 << 9) 

     
        UART2_TRANSFER_STATUS 
        //UART2_TRANSFER_STATUS_RX_FULL = (1 << 0) 
        //UART2_TRANSFER_STATUS_RX_DATA_PRESENT = (1 << 1)
        //UART2_TRANSFER_STATUS_RX_EMPTY = (1 << 2)
        //UART2_TRANSFER_STATUS_TX_FULL = (1 << 3) 
        //UART2_TRANSFER_STATUS_TX_EMPTY = (1 << 4) 
*///////////////////////////////////////////////////////////////////////////////

void uartPrintStatus()
{
    UART2_TRANSFER_STATUS uartTferStatus ;
    UART2_STATUS uartStatus;
    char statusMessage[256];
    uint8_t tempStat; //
    uint8_t sLength = 0;
    uint8_t uartBytes2Rx = 0; //how many bytes are in the receiver
    
    //Get peripheral status
    uartTferStatus = UART2_TransferStatusGet();
    uartStatus = UART2_StatusGet();
    
    uartBytes2Rx = UART2_ReceiveBufferSizeGet(); //how many bytes are in the receiver
   
    //Put status into an ascii string
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\r\n RxBytes Avail --> %d",uartBytes2Rx );
    tempStat = uartTferStatus & UART2_TRANSFER_STATUS_RX_FULL;
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\r\n UART2_TxFER_STATUS_RX_FULL --> %d",tempStat );
    tempStat = uartStatus & UART2_TRANSFER_STATUS_RX_DATA_PRESENT;
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\r\n RX_DATA_PRESENT --> %d",tempStat );
    tempStat = uartStatus & UART2_RX_OVERRUN_ERROR;
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\r\n UART2_RX_OVERRUN_ERROR ----> %d",tempStat );
    tempStat = uartStatus & UART2_FRAMING_ERROR;
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\nUART2_FRAMING_ERROR -> %d",tempStat);
    tempStat = uartStatus & UART2_PARITY_ERROR;
    sLength += (uint8_t)sprintf((char*)(statusMessage+sLength), (const char*)"\r\nUART2_PARITY_ERROR -----------> %d",tempStat);
        
    uartSend((const uint8_t*)statusMessage, sLength);
    
    return 0;
        
}


//////////////////////////////////////////////////////////////////////////////
// uartSend -- send bytes over UART channel 2
//////////////////////////////////////////////////////////////////////////////
uint8_t uartSend(const uint8_t* message, uint8_t messageLength)
{
    uint8_t uartBytesWritten = 0;
    UART2_TRANSFER_STATUS uartTStatus ;
    UART2_STATUS uartStatus;
    
            
        while( uartBytesWritten < messageLength)
        {
            uartTStatus = UART2_TransferStatusGet() ;
            if (uartTStatus & UART2_TRANSFER_STATUS_TX_EMPTY)
            {
                uartBytesWritten += UART2_WriteBuffer ( message + uartBytesWritten, messageLength - uartBytesWritten )  ;
                if(uartBytesWritten < messageLength)
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            else
            {
                continue;
            }

            // Do something else...
        }
    return uartBytesWritten;
}
/**
 End of File
 */

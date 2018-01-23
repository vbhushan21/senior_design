//*****************************************************************************
//
// project0.c - Example to demonstrate minimal TivaWare setup
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Project Zero (project0)</h1>
//!
//! This example demonstrates the use of TivaWare to setup the clocks and
//! toggle GPIO pins to make the LED blink. This is a good place to start
//! understanding your launchpad and the tools that can be used to program it.
//
//*****************************************************************************

#define USER_LED  GPIO_PIN_2

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************

void I2CInit(void);
void I2Croutine(void);
int
main(void)
{
    //
    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

		I2CInit();
		I2Croutine();
    
}
void I2CInit(void){
	//PB2 I2C0SCL
	//PB3 I2C0SDA
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
	{
	}
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	//Fast Mode 400 kbps
	//Address 0x62
	//
	//Enable I2C0 peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	//
	// Wait for the I2C0 module to be ready.
	//
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
	{
	}
	//
// Initialize Master and Slave
//
//Sets 400 kbps speed	
I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); 
}
void I2Croutine(void) {
	uint32_t readValue = 0x01;
//
// Specify slave address
//
//false specifies write mode
I2CMasterSlaveAddrSet(I2C0_BASE, 0x62, false);
//
// WRITE
//	Address of sensor is put first, then address of internal register, then write value 
// 8 bit argument
I2CMasterDataPut(I2C0_BASE, 0x00); 
//
//
	

I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
// Delay until transmission completes
//
while(I2CMasterBusBusy(I2C0_BASE))
{
}
I2CMasterDataPut(I2C0_BASE, 0x04);
I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//
// Delay until transmission completes
//
while(I2CMasterBusBusy(I2C0_BASE))
{
}
//Read from register to determine initialized-state

while ((readValue & 0x01) == 0x01) 
{
		//read register 0x01
		I2CMasterDataPut(I2C0_BASE, 0x01);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		while(I2CMasterBusBusy(I2C0_BASE))
		{
		}
		//I2C module read mode
		I2CMasterSlaveAddrSet(I2C0_BASE, 0x62, true);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		readValue = I2CMasterDataGet(I2C0_BASE);
		while(I2CMasterBusBusy(I2C0_BASE))
		{
		}
}
//read two bytes to obtain distance in centimeters
//Setting MSB of register address allows for auto-increment
//Used for burst mode read of two contiguous locations
		I2CMasterSlaveAddrSet(I2C0_BASE, 0x62, false);
		I2CMasterDataPut(I2C0_BASE, 0x8F);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		while(I2CMasterBusBusy(I2C0_BASE))
		{
		}
		I2CMasterSlaveAddrSet(I2C0_BASE, 0x62, true);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(I2CMasterBusBusy(I2C0_BASE))
		{
		}
		readValue = I2CMasterDataGet(I2C0_BASE) << 8;
		I2CMasterControl(I2C0_BASE,I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		while(I2CMasterBusBusy(I2C0_BASE))
		{
		}
		readValue |= I2CMasterDataGet(I2C0_BASE);
		//16-bit value in cm
		//UART0_OutUDec(readValue);
}



//------------UART_OutChar------------
// Output 8-bit to serial port
// Input: letter is an 8-bit ASCII character to be transferred
// Output: none
void UART0_OutChar(char data){
	UARTCharPut(UART0_BASE, data);
}
//-----------------------UART_OutUDec-----------------------
// Output a 32-bit number in unsigned decimal format
// Input: 32-bit number to be transferred
// Output: none
// Variable format 1-10 digits with no space before or after
void UART0_OutUDec(uint32_t n){
// This function uses recursion to convert decimal number
//   of unspecified length as an ASCII string
  if(n >= 10){
    UART0_OutUDec( n/10);
    n = n%10;
  }
  UART0_OutChar(n+'0'); /* n is between 0 and 9 */
}

void UARTinit(void) 
{
	/***  UART0 Setup   *****************************************************************/
	//Enable GPIOA 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(3);

	//Configure GPIOA pins for UART0
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//Enable UART0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlDelay(3);

	//Configure UART0 Baud Rate 115200
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
					(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
					
	IntEnable(INT_UART0); //Enable interrupt UART0
	IntPrioritySet(INT_UART0, 0x20); //Give priority 1
	UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}

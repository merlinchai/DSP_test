// TI File $Revision: /main/2 $
// Checkin $Date: March 1, 2007   16:06:07 $
//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitSci: 
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
void InitSci(void)
{
	// Initialize SCI-A:

	//tbd...
 	

	// Initialize SCI-B:

	//tbd...
  

	// Initialize SCI-C:

	ScicRegs.SCICCR.all = 0x67;    				// 1 stop bit,  No loopback 
                                   						// Even parity,8 char bits,
                                   						// async mode, idle-line protocol
   	ScicRegs.SCICTL1.all = 0x03;   				// enable TX, RX, internal SCICLK, 
                                   						// Disable RX ERR, SLEEP, TXWAKE
  	ScicRegs.SCICTL2.bit.TXINTENA = 0;		//Disable interrupt
   	ScicRegs.SCICTL2.bit.RXBKINTENA = 0;		//Disable interrupt
   	ScicRegs.SCIHBAUD = 0x00;				//19200bps
   	ScicRegs.SCILBAUD = 0xF3;
	
   	ScicRegs.SCIFFTX.all = 0x0000;			
   	ScicRegs.SCIFFRX.all = 0x0000;
   	ScicRegs.SCIFFCT.all = 0x00;
	ScicRegs.SCIFFTX.bit.SCIRST = 1;
	ScicRegs.SCIFFTX.bit.SCIFFENA = 1;

  	ScicRegs.SCICTL1.all =0x23;     			// Relinquish SCI from Reset 
   	ScicRegs.SCIFFTX.bit.TXFIFOXRESET = 1;
   	ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;	
}	

//---------------------------------------------------------------------------
// Example: InitSciGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
// Caution: 
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation. 
// Comment out other unwanted lines.

void InitSciGpio()
{
#if DSP28_SCIA   
   InitSciaGpio();
#endif // if DSP28_SCIA  
#if DSP28_SCIB   
   InitScibGpio();
#endif // if DSP28_SCIB  
#if DSP28_SCIC
   InitScicGpio();
#endif // if DSP28_SCIC
}

void InitSciaGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	   // Enable pull-up for GPIO29 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
	
    EDIS;
}

#if DSP28_SCIB 
void InitScibGpio()
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;     // Enable pull-up for GPIO9  (SCITXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up for GPIO14 (SCITXDB)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	   // Enable pull-up for GPIO18 (SCITXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pull-up for GPIO22 (SCITXDB)

	
//  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up for GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up for GPIO15 (SCIRXDB)
//  GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	   // Enable pull-up for GPIO19 (SCIRXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;    // Enable pull-up for GPIO23 (SCIRXDB)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
//  GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

/* Configure SCI-B pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.
// Comment out other unwanted lines.

//  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;    // Configure GPIO9 for SCITXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2;   // Configure GPIO14 for SCITXDB operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDB operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;   // Configure GPIO22 for SCITXDB operation
	
//  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;   // Configure GPIO11 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;   // Configure GPIO15 for SCIRXDB operation
//  GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDB operation
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO23 for SCIRXDB operation
	
    EDIS;
}
#endif // if DSP28_SCIB 

#if DSP28_SCIC
void InitScicGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.  
// This will enable the pullups for the specified pins.

	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;    // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	   // Enable pull-up for GPIO63 (SCITXDC)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

/* Configure SCI-C pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 for SCITXDC operation
	
    EDIS;
}
#endif // if DSP28_SCIC 

	
//===========================================================================
// End of file.
//===========================================================================

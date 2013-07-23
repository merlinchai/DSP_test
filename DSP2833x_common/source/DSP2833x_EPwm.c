// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:19 $
//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "CommonDefine.h"


//---------------------------------------------------------------------------
// InitEPwm: 
//---------------------------------------------------------------------------
// This function initializes the ePWM(s) to a known state.
//
void InitEPwm(void)
{
   // Initialize ePWM1/2/3/4/5/6

   // tbd...
 
}

//---------------------------------------------------------------------------
// Example: InitEPwmGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 


//---------------------------------------------------------------------------
// ePWM setup
//---------------------------------------------------------------------------

void SetupEPwmUExample(void)  //by yb
{
   //setup TB
   EPwm1Regs.TBPRD = EPWM_PERIOD; 	// Set timer period  100us
   EPwm1Regs.TBPHS.half.TBPHS = 0;	// Phase is 0
   EPwm1Regs.TBCTR = 0x0000;		// Clear counter

   // Setup TBCTL
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;	// Count up mode   
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;			// Disable phase shift
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;			// Use shadow registers when TBPRD changes
   													// ie only update when counter is 0
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;			// Clock ratio to SYSCLKOUT 
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;			// Counter clock is same as system clock

  // Setup CMPCTL
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every ZERO
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   // AQCSF Active Register Reload From Shadow Options: Immediate
   //EPwm1Regs.AQSFRC.bit.RLDCSF=3;

   // Setup CMP
   EPwm1Regs.CMPA.half.CMPA =EPwmU_Compare;
   
   // Set actions    
   // Set PWM1A 
   EPwm1Regs.AQCTLA.bit.CAU=AQ_SET;		// Counter increases & == comparator A: output high
   EPwm1Regs.AQCTLA.bit.CAD=AQ_CLEAR;	// Counter decreases & == comparator A: output low
   // Set PWM1B
   EPwm1Regs.AQCTLB.bit.CAU=AQ_SET;		// Counter increases & == comparator A: output high
   EPwm1Regs.AQCTLB.bit.CAD=AQ_CLEAR;	// Counter decreases & == comparator A: output low

   // Set deadband
   // Active Low PWMs - Setup Deadband
   // Use deadband module
   EPwm1Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;	// Enable deadband generator. ePWMxA and B are used
   //EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;	// Enable ePWMxA and B symmetrically and inverted
   												// (IPM module is low conduction) 
   												// Deadband is active high
   //EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;		// This module only needs ePWMxA as input
   												// Can output xA and xB with deadband
   //EPwm1Regs.DBRED = EPWMU_MIN_DB;			// Rise time delay
   //EPwm1Regs.DBFED = EPWMU_MIN_DB;			// Fall time delay

   // Interrupt settings
   EPwm1Regs.ETSEL.bit.INTEN=1;			// Use ePWM1 interrupt
   EPwm1Regs.ETSEL.bit.INTSEL=0x1;		// Choose ePWM1 arithmetic underflow
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;	// Generate interrupt on first event

   EPwm1Regs.ETCLR.bit.INT=1;			// Manually clear

   
   // Set ePWM to initialize ADCA
   EPwm1Regs.ETSEL.bit.SOCAEN=1;		// Allow ePWM1 event to initialize ADCA
   EPwm1Regs.ETSEL.bit.SOCASEL=0x1;		// When TB=0 for ePWM1, underflow triggers ADC
   EPwm1Regs.ETPS.bit.SOCAPRD=ET_1ST;   // !!!!!!!!!!!


   // Set ePWM2 for phase U gate signals

   // Setup TB
   EPwm2Regs.TBPRD = EPWM_PERIOD;		// Set timer period 50us
   EPwm2Regs.TBPHS.half.TBPHS = 0;		// Phase is 0
   EPwm2Regs.TBCTR = 0x0000;			// Clear counter

   // Setup TBCTL
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;	// Count up mode   
   EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;		// Disable phase shift
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;		// 
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;		// Clock ratio to SYSCLKOUT 
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;		// Counter clock is same as system clock

  // Setup CMPCTL
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every 0
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   
   // AQCSF Active Register Reload From Shadow Options: Immediate
   // EPwm2Regs.AQSFRC.bit.RLDCSF=3;


   // Setup CMP
   EPwm2Regs.CMPA.half.CMPA =EPwmU_Compare;
   // Set actions    
   // Set PWM2A
   EPwm2Regs.AQCTLA.bit.CAU=AQ_SET;		// Counter increases & == comparator A: output low
   EPwm2Regs.AQCTLA.bit.ZRO=AQ_CLEAR;	// Counter decreases & == comparator A: output high
   
   // Set PWM2B
   EPwm2Regs.AQCTLB.bit.CAU=AQ_CLEAR;	// Counter increases & == comparator A: output high
   EPwm2Regs.AQCTLB.bit.ZRO=AQ_SET;		// Counter decreases & == comparator A: output low 

   // Set deadband
   // Active Low PWMs - Setup deadband
   // Use deadband module
   // Only setup ePWMxA because output is complimentary
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// Enable deadband generator. Use ePWMxA and ePWMxB
   EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;		// Enable ePWMxA and ePWMxB to be complimentary and symmetrical
   													// Also inverted (IPM module is active low)
   													// Deadband is set to be active high
   EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;			// This module only needs ePWMxA as input
   													// Can produce xA and xB outputs with deadband
   EPwm2Regs.DBRED = EPWMU_MIN_DB;					// Rise time delay
   EPwm2Regs.DBFED = EPWMU_MIN_DB;					// Fall time delay

   // Initialize second set of switch to be open
   EPwm1Regs.AQSFRC.bit.RLDCSF = LD_IM;		// Force to open state
   EPwm1Regs.AQCSFRC.bit.CSFA = CSF_DIS;	// Not used
   EPwm1Regs.AQCSFRC.bit.CSFB = CSF_DIS;	// Not used
   
   EPwm2Regs.AQSFRC.bit.RLDCSF = LD_IM; 		// Force to open state
   EPwm2Regs.AQCSFRC.bit.CSFA = CSF_LOW; 		// Forced low by software
   EPwm2Regs.AQCSFRC.bit.CSFB = CSF_LOW;		// Forced low by software
   EPwm2Regs.DBCTL.bit.OUT_MODE = DB_DISABLE; 	// Deadband not used
   
}


void SetupEPwmVExample(void)  //by yb
{
   //setup TB
   EPwm3Regs.TBPRD = EPWM_PERIOD;	// Set timer period 50us
   EPwm3Regs.TBPHS.half.TBPHS = 0;  // Phase is 0
   EPwm3Regs.TBCTR = 0x0000;        // Clear counter

   // Setup TBCTL
   EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;	// Count up mode
   EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // Disable phase shift
   EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;   	// Use shadow registes when changing TBPRD
   												// Refresh when counter is 0
   EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT 
   EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;        // Counter clock is same as system clock

  // Setup CMPCTL
   EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every ZERO
   EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   //AQCSF Active Register Reload From Shadow Options: Immediate
   //EPwm1Regs.AQSFRC.bit.RLDCSF=3;

   // Setup CMP
   EPwm3Regs.CMPA.half.CMPA =EPwmV_Compare;
   
   // Set actions    
   // Set PWM3A
   EPwm3Regs.AQCTLA.bit.CAU=AQ_SET;  	// Counter increases & == comparator A: output low
   EPwm3Regs.AQCTLA.bit.ZRO=AQ_CLEAR; 	// Counter decreases & == comparator A: output high
   
   // Set PWM3B
   EPwm3Regs.AQCTLB.bit.CAU=AQ_CLEAR;  	// Counter increases & == comparator A: output high
   EPwm3Regs.AQCTLB.bit.ZRO=AQ_SET;  	// Counter decreases & == comparator A: output low

	//set Deadband
   // Active Low PWMs - Setup Deadband
   EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  	// Enable deadband generator
   													// Used by ePWMxA and ePWMxB
   EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; 		// Enable ePWMxA and ePWMxB to be complimentary and symmetrical
   													// Also inverted (IPM module is active low)
   													// Deadband is set to be active high
   EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;     		// This module only needs ePWMxA as input
   													// Can produce xA and xB outputs with deadband
   EPwm3Regs.DBRED = EPWMU_MIN_DB;      			// Rise time delay
   EPwm3Regs.DBFED = EPWMU_MIN_DB;     				// Fall time delay


   // Set ePWM4 for phase V gate signals

   // Setup TB
   EPwm4Regs.TBPRD = EPWM_PERIOD;  		// Set timer period 50us
   EPwm4Regs.TBPHS.half.TBPHS = 0;    	// Phase is 0
   EPwm4Regs.TBCTR = 0x0000;         	// Clear counter

   // Setup TBCTL
   EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; 	// Count up mode
   EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // Disable phase shift
   EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;   	// Use shadow registes when changing TBPRD
   												// Refresh when counter is 0
   EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT 
   EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;        // Counter clock is same as system clock

  // Setup CMPCTL
   EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every 0
   EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   
   
   //AQCSF Active Register Reload From Shadow Options: Immediate
   //EPwm2Regs.AQSFRC.bit.RLDCSF=3;

   // Setup CMP
   EPwm4Regs.CMPA.half.CMPA =EPwmV_Compare;
   
   // Set actions    
   // Set PWM4A
   EPwm4Regs.AQCTLA.bit.CAU=AQ_SET;  	// Counter increases & == comparator A: output low
   EPwm4Regs.AQCTLA.bit.ZRO=AQ_CLEAR;  	// Counter decreases & == comparator A: output high
   
   // Set PWM4B
   EPwm4Regs.AQCTLB.bit.CAU=AQ_CLEAR;  	// Counter increases & == comparator A: output high
   EPwm4Regs.AQCTLB.bit.ZRO=AQ_SET;  	// Counter decreases & == comparator A: output low

   // Set deadband
   // Active Low PWMs - Setup deadband
   // Use deadband module
   // Only setup ePWMxA because output is complimentary
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  	// Enable deadband generator
   													// Used by ePWMxA and ePWMxB
   EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; 		// Enable ePWMxA and ePWMxB to be complimentary and symmetrical
   													// Also inverted (IPM module is active low)
   													// Deadband is set to be active high
   EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;     		// This module only needs ePWMxA as input
   													// Can produce xA and xB outputs with deadband
   EPwm4Regs.DBRED = EPWMU_MIN_DB;      			// Rise time delay
   EPwm4Regs.DBFED = EPWMU_MIN_DB;     				// Fall time delay
   
   // Initialize second set of switch to be open
   EPwm3Regs.AQSFRC.bit.RLDCSF = LD_IM; 		// Force to open state
   EPwm3Regs.AQCSFRC.bit.CSFA = CSF_DIS; 		// Not used
   EPwm3Regs.AQCSFRC.bit.CSFB = CSF_DIS; 		// Not used
   
   EPwm4Regs.AQSFRC.bit.RLDCSF = LD_IM; 		// Force to open state
   EPwm4Regs.AQCSFRC.bit.CSFA = CSF_LOW; 		// Forced low by software
   EPwm4Regs.AQCSFRC.bit.CSFB = CSF_LOW; 		// Forced low by software
   EPwm4Regs.DBCTL.bit.OUT_MODE = DB_DISABLE;	// Deadband module not used
   
}




void SetupEPwmWExample(void)  //by yb
{
   //setup TB
   EPwm5Regs.TBPRD = EPWM_PERIOD;  		// Set timer period 50us
   EPwm5Regs.TBPHS.half.TBPHS = 0;    	// Phase is 0
   EPwm5Regs.TBCTR = 0x0000;         	// Clear counter

   // Setup TBCTL
   EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; 	// Count up mode
   EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // Disable phase shift
   EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;   	// Use shadow registes when changing TBPRD
   												// Refresh when counter is 0
   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT 
   EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;        // Counter clock is same as system clock

  // Setup CMPCTL
   EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every 0
   EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   //AQCSF Active Register Reload From Shadow Options: Immediate
   //EPwm1Regs.AQSFRC.bit.RLDCSF=3;

   // Setup CMP
   EPwm5Regs.CMPA.half.CMPA =EPwmW_Compare;
   
   // Set actions    
   // Set PWM5A
   EPwm5Regs.AQCTLA.bit.CAU=AQ_SET;  	// Counter increases & == comparator A: output low
   EPwm5Regs.AQCTLA.bit.ZRO=AQ_CLEAR;  	// Counter decreases & == comparator A: output high
   
   // Set PWM5B
   EPwm5Regs.AQCTLB.bit.CAU=AQ_CLEAR;  	// Counter increases & == comparator A: output high
   EPwm5Regs.AQCTLB.bit.ZRO=AQ_SET;  	// Counter decreases & == comparator A: output low

   // Set deadband
   // Active low PWMs - Setup Deadband
   EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  	// Enable deadband generator
   													// Used by ePWMxA and ePWMxB
   EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; 		// Enable ePWMxA and ePWMxB to be complimentary and symmetrical
   													// Also inverted (IPM module is active low)
   													// Deadband is set to be active high
   EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;     		// This module only needs ePWMxA as input
   													// Can produce xA and xB outputs with deadband
   EPwm5Regs.DBRED = EPWMU_MIN_DB;      			// Rise time delay
   EPwm5Regs.DBFED = EPWMU_MIN_DB;     				// Fall time delay


   // Set ePWM6 for phase W gate signals

   // Setup TB
   EPwm6Regs.TBPRD = EPWM_PERIOD;  		// Set timer period 50us
   EPwm6Regs.TBPHS.half.TBPHS = 0;    	// Phase is 0
   EPwm6Regs.TBCTR = 0x0000;         	// Clear counter

   // Setup TBCTL
   EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; 	// Count up mode
   EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;      // Disable phase shift
   EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;   	// Use shadow registes when changing TBPRD
   												// Refresh when counter is 0
   EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT 
   EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;        // Counter clock is same as system clock

  // Setup CMPCTL
   EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;	// Load registers every 0
   EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;	//CC_IMMEDIATE;//CC_SHADOW;
   EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;   

   
   // AQCSF Active Register Reload From Shadow Options: Immediate
   //EPwm2Regs.AQSFRC.bit.RLDCSF=3;


   // Setup CMP
   EPwm6Regs.CMPA.half.CMPA =EPwmW_Compare;
   // Set actions    
   // Set PWM6A
   EPwm6Regs.AQCTLA.bit.CAU=AQ_SET;  	// Counter increases & == comparator A: output low
   EPwm6Regs.AQCTLA.bit.ZRO=AQ_CLEAR;  	// Counter decreases & == comparator A: output high
   
   // Set PWM6B
   EPwm6Regs.AQCTLB.bit.CAU=AQ_CLEAR;  	// Counter increases & == comparator A: output high
   EPwm6Regs.AQCTLB.bit.ZRO=AQ_SET;  	// Counter decreases & == comparator A: output low

   // Set deadband
   // Active low PWMs - Setup deadband
   EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // Enable deadband generator
   													// Used by ePWMxA and ePWMxB
   EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC; 		// Enable ePWMxA and ePWMxB to be complimentary and symmetrical
   													// Also inverted (IPM module is active low)
   													// Deadband is set to be active high
   EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;     		// This module only needs ePWMxA as input
   													// Can produce xA and xB outputs with deadband
   EPwm6Regs.DBRED = EPWMU_MIN_DB;      			// Rise time delay
   EPwm6Regs.DBFED = EPWMU_MIN_DB;     				// Fall time delay

   // Initialize second set of switch to be open
   EPwm5Regs.AQSFRC.bit.RLDCSF = LD_IM; 		// Force to open state
   EPwm5Regs.AQCSFRC.bit.CSFA = CSF_DIS; 		// Not used
   EPwm5Regs.AQCSFRC.bit.CSFB = CSF_DIS; 		// Not used

   EPwm6Regs.AQSFRC.bit.RLDCSF = LD_IM; 		// Force to open state
   EPwm6Regs.AQCSFRC.bit.CSFA = CSF_LOW; 		// Forced low by software
   EPwm6Regs.AQCSFRC.bit.CSFB= CSF_LOW; 		// Forced low by software
   EPwm6Regs.DBCTL.bit.OUT_MODE = DB_DISABLE; 	// Deadband module not used

}



//void SetupEPWMRegister(void)
//{
// Force low
//	EPwm4Regs.AQCSFRC.bit.CSFA=0x1;	

//}


void InitEPwmGpio(void)
{
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEPwm3Gpio();
	#if DSP28_EPWM4
	   InitEPwm4Gpio();
	#endif // endif DSP28_EPWM4
	#if DSP28_EPWM5    
	   InitEPwm5Gpio();
	#endif // endif DSP28_EPWM5
	#if DSP28_EPWM6
	   InitEPwm6Gpio();
	#endif // endif DSP28_EPWM6 
}

void InitEPwm1Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)   
   
/* Configure ePWM-1 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
   
    EDIS;
}

void InitEPwm2Gpio(void)
{
   EALLOW;
	
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

/* Configure ePWM-2 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM2 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
   
    EDIS;
}

void InitEPwm3Gpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
       
/* Configure ePWM-3 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM3 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
	
    EDIS;
}


#if DSP28_EPWM4
void InitEPwm4Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)

/* Configure ePWM-4 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM4 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B
	
    EDIS;
}
#endif // endif DSP28_EPWM4  


#if DSP28_EPWM5
void InitEPwm5Gpio(void)
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO9 (EPWM5B)

/* Configure ePWM-5 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM5 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B
	
    EDIS;
}
#endif // endif DSP28_EPWM5


#if DSP28_EPWM6
void InitEPwm6Gpio(void)
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)

/* Configure ePWM-6 pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be ePWM6 functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
	
    EDIS;
}
#endif // endif DSP28_EPWM6  

//---------------------------------------------------------------------------
// Example: InitEPwmSyncGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as ePWM Synch pins
//

void SetupEPwm(void)
{

	// ePWM clock as system clock
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; 	// Enable ePWM clock synchronization
   											// Can make all ePWM modules synchronized with certain steps
   											// First do not enable
   											// Then configure TBCLK of each ePWM modules to be the same
   EDIS;

   // To synchronize, TBCLK of each ePWM module needs to be synchronized 
   SetupEPwmUExample();
   SetupEPwmVExample();
   SetupEPwmWExample();
  // SetupEPWMRegister();
   
   EALLOW;
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // Finally enable ePWM clock synchronization
   EDIS; 
  

}



void InitEPwmSyncGpio(void)
{

   EALLOW;

/* Configure EPWMSYNCI  */
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up on GPIO32 (EPWMSYNCI)    

/* Set qualification for selected pins to asynch only */
// This will select synch to SYSCLKOUT for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;   // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;  // Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)    

/* Configure EPwmSync pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be EPwmSync functional pins.
// Comment out other unwanted lines.   

   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;    // Enable pull-up on GPIO6 (EPWMSYNCI)
// GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;   // Enable pull-up on GPIO32 (EPWMSYNCI)    



/* Configure EPWMSYNC0  */

/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

// GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWMSYNC0)
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pull-up on GPIO33 (EPWMSYNC0)    

// GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;    // Enable pull-up on GPIO6 (EPWMSYNC0)
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;   // Enable pull-up on GPIO33 (EPWMSYNC0)    

}



//---------------------------------------------------------------------------
// Example: InitTzGpio: 
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as Trip Zone (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 

void InitTzGpio(void)
{
   EALLOW;
   
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user. 
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
//   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
//   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
//   GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ4)

//   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
// GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ5)

//   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ6) 
// GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ6)  
   
/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.  
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
//   GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
//   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
//   GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ4)

//   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ5)
// GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ5)

//   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ6) 
// GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ6)  

   
/* Configure TZ pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be TZ functional pins.
// Comment out other unwanted lines.   
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
//   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
//   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
//   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ4

//   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ5
// GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ5

//   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ6               
// GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ6  

   EDIS;
}



//===========================================================================
// End of file.
//===========================================================================

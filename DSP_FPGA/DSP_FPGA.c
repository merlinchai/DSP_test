// DSP +FPGA main program for controlling board¿ØÖÆ°åÖ÷³ÌÐò
//Last Edit Time: 20120624

//###########################################################################
//
// FILE:    Compensation-code-30kw.c
//
// TITLE:   30kw series-compensation device main program.C
//
// ASSUMPTIONS:
//
//
//       $Boot_Table:
//
//         GPIO87   GPIO86     GPIO85   GPIO84
//          XA15     XA14       XA13     XA12
//           PU       PU         PU       PU
//        ==========================================
//            1        1          1        1    Jump to Flash <- "boot to Flash"
//            1        1          1        0    SCI-A boot
//            1        1          0        1    SPI-A boot
//            1        1          0        0    I2C-A boot
//            1        0          1        1    eCAN-A boot
//            1        0          1        0    McBSP-A boot
//            1        0          0        1    Jump to XINTF x16
//            1        0          0        0    Jump to XINTF x32
//            0        1          1        1    Jump to OTP
//            0        1          1        0    Parallel GPIO I/O boot
//            0        1          0        1    Parallel XINTF boot
//            0        1          0        0    Jump to SARAM
//            0        0          1        1    Branch to check boot mode
//            0        0          1        0    Boot to flash, bypass ADC cal
//            0        0          0        1    Boot to SARAM, bypass ADC cal
//            0        0          0        0    Boot to SCI-A, bypass ADC cal
//                                              Boot_Table_End$
//
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################



#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "CommonDefine.h"  		 // Macro and global variables definitions

// .h files
#include "iqmathlib.h"		// Included to perform abc-dq transformation
#include "iclarke.h"		// Inverse abc transformation
#include "ipark.h"			// Inverse dq transformation
#include "park.h"			// dq transformation
#include "clarke.h"			// abc transformation

#include "math.h"			// Various math functions 
#include "filter.h"			// Digital filter library

// Global parameters

extern unsigned int EPWM_PERIOD_change=60000; 
extern unsigned int EPwmU_Compare=3000;
extern unsigned int EPwmV_Compare=3000;
extern unsigned int EPwmW_Compare=3000;


/////////////////////////////////////////////////////////////
// Declaration of data address for FPGA
/////////////////////////////////////////////////////////////

// PWM module definition
#define FPGA_PWMA_Period (unsigned int *)0x210000
#define FPGA_PWMA_Wait1 (unsigned int *)0x210001
#define FPGA_PWMA_Duty1 (unsigned int *)0x210002
#define FPGA_PWMA_Wait2 (unsigned int *)0x210003
#define FPGA_PWMA_Duty2 (unsigned int *)0x210004
#define FPGA_PWMA_Wait3 (unsigned int *)0x210005
#define FPGA_PWMA_Duty3 (unsigned int *)0x210006
#define FPGA_PWMA_Wait4 (unsigned int *)0x210007
#define FPGA_PWMA_Duty4 (unsigned int *)0x210008
#define FPGA_PWMA_Wait5 (unsigned int *)0x210009
#define FPGA_PWMA_Duty5 (unsigned int *)0x21000a
#define FPGA_PWMA_Wait6 (unsigned int *)0x21000b
#define FPGA_PWMA_Duty6 (unsigned int *)0x21000c
#define FPGA_PWMA_Wait7 (unsigned int *)0x21000d
#define FPGA_PWMA_Duty7 (unsigned int *)0x21000e
#define FPGA_PWMA_Wait8 (unsigned int *)0x21000f
#define FPGA_PWMA_Duty8 (unsigned int *)0x210010
#define FPGA_PWMA_Wait9 (unsigned int *)0x210011
#define FPGA_PWMA_Duty9 (unsigned int *)0x210012
#define FPGA_PWMA_Wait10 (unsigned int *)0x210013
#define FPGA_PWMA_Duty10 (unsigned int *)0x210014
#define FPGA_PWMA_Wait11 (unsigned int *)0x210015
#define FPGA_PWMA_Duty11 (unsigned int *)0x210016
#define FPGA_PWMA_Wait12 (unsigned int *)0x210017
#define FPGA_PWMA_Duty12 (unsigned int *)0x210018

#define FPGA_PWMB_Period (unsigned int *)0x210020
#define FPGA_PWMB_Wait1 (unsigned int *)0x210021
#define FPGA_PWMB_Duty1 (unsigned int *)0x210022
#define FPGA_PWMB_Wait2 (unsigned int *)0x210023
#define FPGA_PWMB_Duty2 (unsigned int *)0x210024
#define FPGA_PWMB_Wait3 (unsigned int *)0x210025
#define FPGA_PWMB_Duty3 (unsigned int *)0x210026
#define FPGA_PWMB_Wait4 (unsigned int *)0x210027
#define FPGA_PWMB_Duty4 (unsigned int *)0x210028
#define FPGA_PWMB_Wait5 (unsigned int *)0x210029
#define FPGA_PWMB_Duty5 (unsigned int *)0x21002a
#define FPGA_PWMB_Wait6 (unsigned int *)0x21002b
#define FPGA_PWMB_Duty6 (unsigned int *)0x21002c
#define FPGA_PWMB_Wait7 (unsigned int *)0x21002d
#define FPGA_PWMB_Duty7 (unsigned int *)0x21002e
#define FPGA_PWMB_Wait8 (unsigned int *)0x21003f
#define FPGA_PWMB_Duty8 (unsigned int *)0x210030
#define FPGA_PWMB_Wait9 (unsigned int *)0x210031
#define FPGA_PWMB_Duty9 (unsigned int *)0x210032
#define FPGA_PWMB_Wait10 (unsigned int *)0x210033
#define FPGA_PWMB_Duty10 (unsigned int *)0x210034
#define FPGA_PWMB_Wait11 (unsigned int *)0x210035
#define FPGA_PWMB_Duty11 (unsigned int *)0x210036
#define FPGA_PWMB_Wait12 (unsigned int *)0x210037
#define FPGA_PWMB_Duty12 (unsigned int *)0x210038

#define FPGA_PWMC_Period (unsigned int *)0x210040
#define FPGA_PWMC_Wait1 (unsigned int *)0x210041
#define FPGA_PWMC_Duty1 (unsigned int *)0x210042
#define FPGA_PWMC_Wait2 (unsigned int *)0x210043
#define FPGA_PWMC_Duty2 (unsigned int *)0x210044
#define FPGA_PWMC_Wait3 (unsigned int *)0x210045
#define FPGA_PWMC_Duty3 (unsigned int *)0x210046
#define FPGA_PWMC_Wait4 (unsigned int *)0x210047
#define FPGA_PWMC_Duty4 (unsigned int *)0x210048
#define FPGA_PWMC_Wait5 (unsigned int *)0x210049
#define FPGA_PWMC_Duty5 (unsigned int *)0x21004a
#define FPGA_PWMC_Wait6 (unsigned int *)0x21004b
#define FPGA_PWMC_Duty6 (unsigned int *)0x21004c
#define FPGA_PWMC_Wait7 (unsigned int *)0x21004d
#define FPGA_PWMC_Duty7 (unsigned int *)0x21004e
#define FPGA_PWMC_Wait8 (unsigned int *)0x21005f
#define FPGA_PWMC_Duty8 (unsigned int *)0x210050
#define FPGA_PWMC_Wait9 (unsigned int *)0x210051
#define FPGA_PWMC_Duty9 (unsigned int *)0x210052
#define FPGA_PWMC_Wait10 (unsigned int *)0x210053
#define FPGA_PWMC_Duty10 (unsigned int *)0x210054
#define FPGA_PWMC_Wait11 (unsigned int *)0x210055
#define FPGA_PWMC_Duty11 (unsigned int *)0x210056
#define FPGA_PWMC_Wait12 (unsigned int *)0x210057
#define FPGA_PWMC_Duty12 (unsigned int *)0x210058

#define FPGA_PWMD_Period (unsigned int *)0x210060
#define FPGA_PWMD_Wait1 (unsigned int *)0x210061
#define FPGA_PWMD_Duty1 (unsigned int *)0x210062
#define FPGA_PWMD_Wait2 (unsigned int *)0x210063
#define FPGA_PWMD_Duty2 (unsigned int *)0x210064
#define FPGA_PWMD_Wait3 (unsigned int *)0x210065
#define FPGA_PWMD_Duty3 (unsigned int *)0x210066
#define FPGA_PWMD_Wait4 (unsigned int *)0x210067
#define FPGA_PWMD_Duty4 (unsigned int *)0x210068
#define FPGA_PWMD_Wait5 (unsigned int *)0x210069
#define FPGA_PWMD_Duty5 (unsigned int *)0x21006a
#define FPGA_PWMD_Wait6 (unsigned int *)0x21006b
#define FPGA_PWMD_Duty6 (unsigned int *)0x21006c
#define FPGA_PWMD_Wait7 (unsigned int *)0x21006d
#define FPGA_PWMD_Duty7 (unsigned int *)0x21006e
#define FPGA_PWMD_Wait8 (unsigned int *)0x21006f
#define FPGA_PWMD_Duty8 (unsigned int *)0x210070
#define FPGA_PWMD_Wait9 (unsigned int *)0x210071
#define FPGA_PWMD_Duty9 (unsigned int *)0x210072
#define FPGA_PWMD_Wait10 (unsigned int *)0x210073
#define FPGA_PWMD_Duty10 (unsigned int *)0x210074
#define FPGA_PWMD_Wait11 (unsigned int *)0x210075
#define FPGA_PWMD_Duty11 (unsigned int *)0x210076
#define FPGA_PWMD_Wait12 (unsigned int *)0x210077
#define FPGA_PWMD_Duty12 (unsigned int *)0x210078


#define FPGA_PWME_Period (unsigned int *)0x210080
#define FPGA_PWME_Wait1 (unsigned int *)0x210081
#define FPGA_PWME_Duty1 (unsigned int *)0x210082
#define FPGA_PWME_Wait2 (unsigned int *)0x210083
#define FPGA_PWME_Duty2 (unsigned int *)0x210084
#define FPGA_PWME_Wait3 (unsigned int *)0x210085
#define FPGA_PWME_Duty3 (unsigned int *)0x210086
#define FPGA_PWME_Wait4 (unsigned int *)0x210087
#define FPGA_PWME_Duty4 (unsigned int *)0x210088
#define FPGA_PWME_Wait5 (unsigned int *)0x210089
#define FPGA_PWME_Duty5 (unsigned int *)0x21008a
#define FPGA_PWME_Wait6 (unsigned int *)0x21008b
#define FPGA_PWME_Duty6 (unsigned int *)0x21008c
#define FPGA_PWME_Wait7 (unsigned int *)0x21008d
#define FPGA_PWME_Duty7 (unsigned int *)0x21008e
#define FPGA_PWME_Wait8 (unsigned int *)0x21008f
#define FPGA_PWME_Duty8 (unsigned int *)0x210090
#define FPGA_PWME_Wait9 (unsigned int *)0x210091
#define FPGA_PWME_Duty9 (unsigned int *)0x210092
#define FPGA_PWME_Wait10 (unsigned int *)0x210093
#define FPGA_PWME_Duty10 (unsigned int *)0x210094
#define FPGA_PWME_Wait11 (unsigned int *)0x210095
#define FPGA_PWME_Duty11 (unsigned int *)0x210096
#define FPGA_PWME_Wait12 (unsigned int *)0x210097
#define FPGA_PWME_Duty12 (unsigned int *)0x210098

// I/O module
// Each number corresponds to an I/O port
// 0 is low, non-0 is high
// 1 is output, 0 is input

#define FPGA_IO1_DATA (unsigned int *)0x2100a0
#define FPGA_IO2_DATA (unsigned int *)0x2100a1
#define FPGA_IO3_DATA (unsigned int *)0x2100a2
#define FPGA_IO4_DATA (unsigned int *)0x2100a3
#define FPGA_IO5_DATA (unsigned int *)0x2100a4
#define FPGA_IO6_DATA (unsigned int *)0x2100a5
#define FPGA_IO7_DATA (unsigned int *)0x2100a6
#define FPGA_IO8_DATA (unsigned int *)0x2100a7
#define FPGA_IO9_DATA (unsigned int *)0x2100a8
#define FPGA_IO10_DATA (unsigned int *)0x2100a9
#define FPGA_IO11_DATA (unsigned int *)0x2100aa
#define FPGA_IO12_DATA (unsigned int *)0x2100ab
#define FPGA_IO13_DATA (unsigned int *)0x2100ac
#define FPGA_IO14_DATA (unsigned int *)0x2100ad
#define FPGA_IO15_DATA (unsigned int *)0x2100ae
#define FPGA_IO16_DATA (unsigned int *)0x2100af
#define FPGA_IO17_DATA (unsigned int *)0x2100b0
#define FPGA_IO18_DATA (unsigned int *)0x2100b1
#define FPGA_IO19_DATA (unsigned int *)0x2100b2
#define FPGA_IO20_DATA (unsigned int *)0x2100b3
#define FPGA_IO21_DATA (unsigned int *)0x2100b4
#define FPGA_IO22_DATA (unsigned int *)0x2100b5
#define FPGA_IO23_DATA (unsigned int *)0x2100b6
#define FPGA_IO24_DATA (unsigned int *)0x2100b7
#define FPGA_IODIR_LOW (unsigned int *)0x2100b8
#define FPGA_IODIR_HIGH (unsigned int *)0x2100b9

// DAC module
#define DAC1 (unsigned int *)0x2100e0
#define DAC2 (unsigned int *)0x2100e1
#define DAC3 (unsigned int *)0x2100e2
#define DAC4 (unsigned int *)0x2100e3

#define FPGA_DATA_Test_length 8


// Modebus communication for touchscreen
// Variable definitions
/* variable for modbus is following */

char CommIndex;
int writeadr;
char CommIndexEnd;
char CommBuf[MaxDataLen];
char SlaveOutputBuf[MaxRegLen];	// Higher 8 bit of word is front, Lower 8 bit is back
char SlaveInputBuf[MaxRegLen];

char ForceCoilBuf[10];
char count=0;
char forceval=0x00;

int ModebusTimeoutFlag=0; 	// Set when communication is continuing

int BugTrap=1;

int TestState=0;

/* Table Of CRC Values for high-order byte */
char auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
} ;
/* Table of CRC values for low-order byte */
char auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
} ;

// Data buffer registers for touchscreen
char ModebusRegsDataBuff[MaxRegsNum+MaxWaveformLen];
char ModebusBitsDataBuff[MaxCoilsNum];

int SciTxQueue[MaxDataLen];	 // Queue for touchscreen communication buffer
int SciTxLowPoint;
int SciTxHighPoint;

// Load control algorithm during startup to increase speed
//#pragma CODE_SECTION(ControlArith, "ramfuncs");

// Interrupt declarations
interrupt void cpu_timer0_isr(void);
//interrupt void ISREPwm1_Zero(void);
interrupt void xint1_isr(void);


// Prototype function declarations

// Touchscreen functions
void initvar(void);					// Variable initialization
unsigned int crc16(char *, char);	// CRC crosscheck
void AnalyzeRecieve(void);			// Analysis of received data
void slavecomm(void);				// Communication functions


// Use touchscreen to control EPWM module
//void EPWM_To_ModeBus(void);


main()
{
	int16 i;
	int16 j;
	int16 test[100];
   

// I. System initialization
// PLL clock: 150M
// hispcp = 1 100m/2
// lospcp = 2 100m/4
// Enable ePWM, ADC, cpu timer clock
	InitSysCtrl();	// Clock set to 150MHz and disable other clocks

// DELAY_US(1000000);	

// II. Initialize all GPIO as input
// Input sampling is Type 1
// Enable all pull-ups
// Disable ePWM pull-ups
	InitGpio();

// Custom I/O initialization
// To note: login input and output ports
	InitLogicIO();

// SCI initialization
	InitScicGpio();
	InitSci();
	
// Touchscreen variable initialization
	initvar();


// MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
// ePWM initialization
 	//InitEPwmGpio();	// ePWM I/O initialization
	//InitEPwm1Gpio(); 
	//InitEPwm2Gpio();	// Enable all ePWM modules
	//InitEPwm3Gpio();
	//InitEPwm4Gpio();
	//InitEPwm5Gpio();
	//InitEPwm6Gpio();


// III. Initialize interrupt:
// Disable CPU interrupt
// Disable all PIE interrupts and clear all PIE interrupt flags
	DINT;
	InitPieCtrl();
	IER=0x0000;
	IFR=0x0000;
// Initialize IPE interrupts and enable PIE
	InitPieVectTable();

// Custom interrupt initialization
	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;	//Cpu timer0 interrupt
    PieVectTable.XINT1 = &xint1_isr;
	EDIS;


// IV. Peripherials and setup peripherials initialization
    InitFlash();

    InitXintf();

// ADC initialization
	InitAdc();
// ADC setup
	SetupAdc();

//       Adc_Inquire();
// ePWM Setup
//	SetupEPwm();   // Generate PWM

// CPU timer0 initialization
	InitCpuTimers();
// Configure CPU-Timer 0 to interrupt every ms:
// 150MHz CPU Freq, 3m second Period (in us)
   	ConfigCpuTimer(&CpuTimer0, 150, 3000);
   	StartCpuTimer0();

// V. User specific code, enable interrupts:

/*** Set breakpoint at this point and step through code ***/

// 1. I/O test
	// Test program: 0-17 of GPIOA is input
	// Test program: 24-27 of GPIOA is input
	// Test program: 48-61 of GPIOB is input
	// All I/O are pulled-up, so use input = 0 when testing to protech board
   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

//   EALLOW;
//   GpioCtrlRegs.GPADIR.all = 0x0003FFFF;	// GPIO functionality GPIO0-GPIO15
//   EDIS;
//   GpioDataRegs.GPACLEAR.all = 0x0003FFFF;
//   GpioDataRegs.GPASET.all = 0x0001AAAA;
//   GpioDataRegs.GPATOGGLE.all = 0x0003FFFF;


//2. Communication Test:
	// More programs are required for this test

	// etst RS485 chip
		GpioDataRegs.GPASET.bit.GPIO21=1;
		GpioDataRegs.GPATOGGLE.bit.GPIO22=1;

	// Test CAN communication chip
		GpioDataRegs.GPATOGGLE.bit.GPIO19=1;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}
	// Test RS485 chip
		GpioDataRegs.GPASET.bit.GPIO21=1;
		GpioDataRegs.GPATOGGLE.bit.GPIO22=1;

	// Test CAN communication chip
		GpioDataRegs.GPATOGGLE.bit.GPIO19=1;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

// 3. A/D Test
	// Start A/D with software

	for(i=0;i<100;i++)
   		{
			AdcRegs.ADCTRL2.bit.SOC_SEQ1=1;
			Adc_Inquire();
   		}

// 4. PWM Test
	// Test one channel at a time so that faults can be detected easily

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}
	
		*FPGA_PWMA_Wait1=1000;
		*FPGA_PWMA_Duty1=1000;

		*FPGA_PWMA_Wait2=2000;
		*FPGA_PWMA_Duty2=2000;

		*FPGA_PWMA_Wait3=3000;
		*FPGA_PWMA_Duty3=3000;

		*FPGA_PWMA_Wait4=4000;
		*FPGA_PWMA_Duty4=4000;

		*FPGA_PWMA_Wait5=5000;
		*FPGA_PWMA_Duty5=5000;

		*FPGA_PWMA_Wait6=6000;
		*FPGA_PWMA_Duty6=6000;

		*FPGA_PWMA_Wait7=1000;
		*FPGA_PWMA_Duty7=1000;

		*FPGA_PWMA_Wait8=2000;
		*FPGA_PWMA_Duty8=2000;

		*FPGA_PWMA_Wait9=3000;
		*FPGA_PWMA_Duty9=3000;

		*FPGA_PWMA_Wait10=4000;
		*FPGA_PWMA_Duty10=4000;

		*FPGA_PWMA_Wait11=5000;
		*FPGA_PWMA_Duty11=5000;

		*FPGA_PWMA_Wait12=6000;
		*FPGA_PWMA_Duty12=6000;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

		*FPGA_PWMB_Wait1=1000;
		*FPGA_PWMB_Duty1=1000;

		*FPGA_PWMB_Wait2=2000;
		*FPGA_PWMB_Duty2=2000;

		*FPGA_PWMB_Wait3=3000;
		*FPGA_PWMB_Duty3=3000;

		*FPGA_PWMB_Wait4=4000;
		*FPGA_PWMB_Duty4=4000;

		*FPGA_PWMB_Wait5=5000;
		*FPGA_PWMB_Duty5=5000;

		*FPGA_PWMB_Wait6=6000;
		*FPGA_PWMB_Duty6=6000;

		*FPGA_PWMB_Wait7=1000;
		*FPGA_PWMB_Duty7=1000;

		*FPGA_PWMB_Wait8=2000;
		*FPGA_PWMB_Duty8=2000;

		*FPGA_PWMB_Wait9=3000;
		*FPGA_PWMB_Duty9=3000;

		*FPGA_PWMB_Wait10=4000;
		*FPGA_PWMB_Duty10=4000;

		*FPGA_PWMB_Wait11=5000;
		*FPGA_PWMB_Duty11=5000;

		*FPGA_PWMB_Wait12=6000;
		*FPGA_PWMB_Duty12=6000;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

		*FPGA_PWMC_Wait1=1000;
		*FPGA_PWMC_Duty1=1000;

		*FPGA_PWMC_Wait2=2000;
		*FPGA_PWMC_Duty2=2000;

		*FPGA_PWMC_Wait3=3000;
		*FPGA_PWMC_Duty3=3000;

		*FPGA_PWMC_Wait4=4000;
		*FPGA_PWMC_Duty4=4000;

		*FPGA_PWMC_Wait5=5000;
		*FPGA_PWMC_Duty5=5000;

		*FPGA_PWMC_Wait6=6000;
		*FPGA_PWMC_Duty6=6000;

		*FPGA_PWMC_Wait7=1000;
		*FPGA_PWMC_Duty7=1000;

		*FPGA_PWMC_Wait8=2000;
		*FPGA_PWMC_Duty8=2000;

		*FPGA_PWMC_Wait9=3000;
		*FPGA_PWMC_Duty9=3000;

		*FPGA_PWMC_Wait10=4000;
		*FPGA_PWMC_Duty10=4000;

		*FPGA_PWMC_Wait11=5000;
		*FPGA_PWMC_Duty11=5000;

		*FPGA_PWMC_Wait12=6000;
		*FPGA_PWMC_Duty12=6000;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

		*FPGA_PWMD_Wait1=1000;
		*FPGA_PWMD_Duty1=1000;

		*FPGA_PWMD_Wait2=2000;
		*FPGA_PWMD_Duty2=2000;

		*FPGA_PWMD_Wait3=3000;
		*FPGA_PWMD_Duty3=3000;

		*FPGA_PWMD_Wait4=4000;
		*FPGA_PWMD_Duty4=4000;

		*FPGA_PWMD_Wait5=5000;
		*FPGA_PWMD_Duty5=5000;

		*FPGA_PWMD_Wait6=6000;
		*FPGA_PWMD_Duty6=6000;

		*FPGA_PWMD_Wait7=1000;
		*FPGA_PWMD_Duty7=1000;

		*FPGA_PWMD_Wait8=2000;
		*FPGA_PWMD_Duty8=2000;

		*FPGA_PWMD_Wait9=3000;
		*FPGA_PWMD_Duty9=3000;

		*FPGA_PWMD_Wait10=4000;
		*FPGA_PWMD_Duty10=4000;

		*FPGA_PWMD_Wait11=5000;
		*FPGA_PWMD_Duty11=5000;

		*FPGA_PWMD_Wait12=6000;
		*FPGA_PWMD_Duty12=6000;

   	for(i=0;i<100;i++)
   		{
   			j=i;
   		}

		*FPGA_PWME_Wait1=1000;
		*FPGA_PWME_Duty1=1000;

		*FPGA_PWME_Wait2=2000;
		*FPGA_PWME_Duty2=2000;

		*FPGA_PWME_Wait3=3000;
		*FPGA_PWME_Duty3=3000;

		*FPGA_PWME_Wait4=4000;
		*FPGA_PWME_Duty4=4000;

		*FPGA_PWME_Wait5=5000;
		*FPGA_PWME_Duty5=5000;

		*FPGA_PWME_Wait6=6000;
		*FPGA_PWME_Duty6=6000;

		*FPGA_PWME_Wait7=1000;
		*FPGA_PWME_Duty7=1000;

		*FPGA_PWME_Wait8=2000;
		*FPGA_PWME_Duty8=2000;

		*FPGA_PWME_Wait9=3000;
		*FPGA_PWME_Duty9=3000;

		*FPGA_PWME_Wait10=4000;
		*FPGA_PWME_Duty10=4000;

		*FPGA_PWME_Wait11=5000;
		*FPGA_PWME_Duty11=5000;

		*FPGA_PWME_Wait12=6000;
		*FPGA_PWME_Duty12=6000;

		// PWM reset
		// After reset, all PWM are 0
		GpioDataRegs.GPACLEAR.bit.GPIO31=1;
		DELAY_US(1);
		GpioDataRegs.GPASET.bit.GPIO31=1;

// 5. D/A Test


		*DAC1=1024;
		*DAC2=2048;
		*DAC3=3072;
		*DAC4=4095;

// 6. FPGA I/O Test

	// Test whether result is 1
	for(j=0;j<10;j++)
	{
		for(i=0; i<24; i++
			{
				test[i]=*(FPGA_IO1_DATA+i);
			}
		i=0;
	}
	
	// Test whether result is 0
	for(j=0;j<100;j++)
	{
		for(i=0; i<24; i++)
			{
				test[i]=*(FPGA_IO1_DATA+i);
			}
		i=0;
	}

	// Output test
	// Only I/O pins that are set to output can output 0
	*FPGA_IODIR_LOW=0xFFFF;
	*FPGA_IODIR_HIGH=0x00FF;

	j=0;

	for(i=0; i<24; i++)
	{
		*(FPGA_IO1_DATA+i)=j;
		j=~j;
	}

	j=0;

	for(i=0; i<24; i++)
	{
		j=~j;
		*(FPGA_IO1_DATA+i)=j;
	}

// Reset above test functions
// All I/O are pulled-up and cannot output
	GpioDataRegs.GPACLEAR.bit.GPIO30=1;
	DELAY_US(1);
	GpioDataRegs.GPASET.bit.GPIO30=1;

	j=*FPGA_IODIR_LOW;
	j=*FPGA_IODIR_HIGH;

	j=0;
	
	for(i=0; i<24; i++)
		*(FPGA_IO1_DATA+i)=j;

// Enable interrupt
// Test SCI communications and communication between DSP and FPGA

	EnableInterrupts();	// Enable cputimer 0 interrupt

	i=0;
	
// VI: IDLE loop. Just sit and loop forever (optional)
  	for(;;)
   	{
	//	ScicRegs.SCITXBUF=i;
		DELAY_US(1000);

/*
		if(GpioDataRegs.GPADAT.bit.GPIO27==0)
			GpioDataRegs.GPBCLEAR.bit.GPIO50=1;
		else 
			GpioDataRegs.GPBSET.bit.GPIO50=1;
						
		if(GpioDataRegs.GPBDAT.bit.GPIO48==0)
			GpioDataRegs.GPBCLEAR.bit.GPIO51=1;	
		else 
			GpioDataRegs.GPBSET.bit.GPIO51=1;	
	//	ModebusRegsDataBuff[0]=i;
//		GpioDataRegs.GPBTOGGLE.bit.GPIO60=1;
//		GpioDataRegs.GPBTOGGLE.bit.GPIO61=1;		
//		GpioDataRegs.GPBTOGGLE.bit.GPIO58=1;
//		GpioDataRegs.GPBTOGGLE.bit.GPIO59=1;

     	GpioDataRegs.GPBCLEAR.bit.GPIO53= 1;
     	GpioDataRegs.GPBCLEAR.bit.GPIO52= 1;
						
		i++;
	//	if(i==2000)
		if(i==FPGA_DATA_Test_length)
			i=0;
	//	TestState=~TestState;

*/
	
		slavecomm();	

		
   	}

	
}


// CPU timer0 interrupt: 3ms period
interrupt void cpu_timer0_isr(void)
{
   // Acknowledge this interrupt to receive more interrupts from group 1
   	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


interrupt void xint1_isr(void)
{
//	GpioDataRegs.GPBCLEAR.all = 0x4;   // GPIO34 is low
//	Xint1Count++;
//	GpioDataRegs.GPATOGGLE.bit.GPIO24=1;
		*FPGA_PWMA_Wait1=0000;
		*FPGA_PWMA_Duty1=0000;

		*FPGA_PWMA_Wait2=2000;
		*FPGA_PWMA_Duty2=2000;

		*FPGA_PWMA_Wait3=3000;
		*FPGA_PWMA_Duty3=3000;

		*FPGA_PWMA_Wait4=4000;
		*FPGA_PWMA_Duty4=4000;

		*FPGA_PWMA_Wait5=5000;
		*FPGA_PWMA_Duty5=5000;

		*FPGA_PWMA_Wait6=6000;
		*FPGA_PWMA_Duty6=6000;

		*FPGA_PWMA_Wait7=1000;
		*FPGA_PWMA_Duty7=1000;

		*FPGA_PWMA_Wait8=2000;
		*FPGA_PWMA_Duty8=2000;

		*FPGA_PWMA_Wait9=3000;
		*FPGA_PWMA_Duty9=3000;

		*FPGA_PWMA_Wait10=4000;
		*FPGA_PWMA_Duty10=4000;

		*FPGA_PWMA_Wait11=5000;
		*FPGA_PWMA_Duty11=5000;

		*FPGA_PWMA_Wait12=6000;
		*FPGA_PWMA_Duty12=6000;

		*DAC1=1024;
		*DAC2=1024;
		*DAC3=1024;
		*DAC4=1024;

	// GpioDataRegs.GPATOGGLE.bit.GPIO24=1;
	// Acknowledge this interrupt to get more from group 1
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// Sub functions
// Communication functin of modebus
void initvar(void)
{
	int i;
	for (i=0;i<=250;i++)
		{
			asm("		NOP");
		}
	CommIndex=0;
	CommIndexEnd=0xff; // First enlarge recieve lenght
	for (i=0;i<MaxRegLen;i++)
		{
			SlaveOutputBuf[i]=0;
		}
	for (i=0;i<MaxCoilsNum;i++)
		{
			ModebusBitsDataBuff[i]=1;
		}
	for (i=0;i<(MaxRegsNum+MaxWaveformLen);i++)
		{
			ModebusRegsDataBuff[i]=0;
		}
	for (i=1;i<(MaxRegsNum+MaxWaveformLen);i=i+2)
		{
			ModebusRegsDataBuff[i]=(char)i;
		}
	SciTxHighPoint=0;
	SciTxLowPoint=0;
}


unsigned int crc16(char *puchMsg,char usDataLen) // puchMsg message to calculate CRC upon
{ 
// usDataLen quantity of bytes in message
char uchCRCHi = 0xFF;	// High byte of CRC initialized
char uchCRCLo = 0xFF;	// Low byte of CRC initialized
unsigned int uIndex;	// Index into CRC lookup table
while (usDataLen--)		// Pass through message buffer
	{
		uIndex = uchCRCHi ^ *puchMsg++;	// Calculate the CRC
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (((unsigned int)(uchCRCLo) << 8) | uchCRCHi) ;
}

void AnalyzeRecieve(void)
{
	unsigned int crc16tem;
	char i,j;
	int TempHighAddr;

	if (CommBuf[CommIndexSlaveID]==SlaveID) // The slave ID is matched
	{
		crc16tem=crc16(CommBuf,CommIndex-1); // calculate CRC check, erase two CRC byte

		if (crc16tem==(((unsigned int)(CommBuf[CommIndexEnd]) << 8) | CommBuf[CommIndexEnd-1]))
		{ 
			// crc16 check is OK
			switch (CommBuf[CommIndexFunction])
			{
	
				case ReadMulCoilSta:
					CommBuf[CommIndexSlaveID]=SlaveID;	// 0
					CommBuf[CommIndexFunction]=ReadMulCoilSta;	// 2
					CommBuf[2]=CommBuf[CommIndexNoPointLo] / 0x08;	// 0x16 bit
		
					writeadr=CommBuf[3];
					SlaveOutputBuf[0]=0;
					SlaveOutputBuf[1]=0;
					
					for(i=0;i<8;i++,writeadr++)
						{
							SlaveOutputBuf[0]=SlaveOutputBuf[0]+(ModebusBitsDataBuff[writeadr]<<i);	// Send data of master force coil val
						}
					
					for(i=0;i<8;i++,writeadr++)
						{
							SlaveOutputBuf[1]=SlaveOutputBuf[1]+(ModebusBitsDataBuff[writeadr]<<i);
						}
					i=CommBuf[2]+3;
					
					for(CommIndex=3;CommIndex<i;CommIndex++) // Send data Reg to Master
						{
							CommBuf[CommIndex]=SlaveOutputBuf[CommIndex-3]; // High 8 bit is sent first
						}
						
					crc16tem=crc16(CommBuf,CommIndex); // Then send low 8 bit data
					CommBuf[CommIndex++]=(char)(crc16tem & 0x00ff); // Send crccheck low 8 bit first
					CommBuf[CommIndex]=(char)(crc16tem>>8); // Then send high 8 bit
					CommIndexEnd=CommIndex;
					CommIndex=0;
		
					for(i=0;i<=CommIndexEnd;i++)
						{
	//						ScicRegs.SCITXBUF=CommBuf[i];
							SciTxQueue[SciTxHighPoint]=CommBuf[i];
							SciTxHighPoint++;
							if(SciTxHighPoint==MaxDataLen)
								{
									SciTxHighPoint=0;
								}
						}
					break;
	
				case ForceSingleCoil:
					// get data 0x00 or 0xff
				 	writeadr=CommBuf[3];
					if (CommBuf[4]==0xff)
						 {
							ModebusBitsDataBuff[writeadr]=1;
						 }
					else
						 {
							ModebusBitsDataBuff[writeadr]=0;
						 }
		
					CommBuf[CommIndexSlaveID]=SlaveID; // 0
					CommBuf[CommIndexFunction]=ForceSingleCoil; // 1
					CommIndex=2;
		
					CommBuf[CommIndex++]=CommBuf[CommIndexStartAdrHi];	// 2
					CommBuf[CommIndex++]=CommBuf[CommIndexStartAdrLo];	// 3
					CommBuf[CommIndex++]=CommBuf[CommIndexNoPointHi];	// 4
					CommBuf[CommIndex++]=CommBuf[CommIndexNoPointLo];	// 5
					crc16tem=crc16(CommBuf,CommIndex);	// Send low 8 bit data
					CommBuf[CommIndex++]=(char)(crc16tem & 0x00ff);	// Send crccheck low 8 bit first
					CommBuf[CommIndex]=(char)(crc16tem>>8); // Then send high 8 bit
					CommIndexEnd=CommIndex;
					CommIndex=0;
		
		
					for(i=0;i<=CommIndexEnd;i++)
						{
							ScicRegs.SCITXBUF=CommBuf[i];
							SciTxQueue[SciTxHighPoint]=CommBuf[i];
							SciTxHighPoint++;
							if(SciTxHighPoint==MaxDataLen)
								{
									SciTxHighPoint=0;
								}
						}
					break;
	
				case ReadHoldReg:
					TempHighAddr=CommBuf[2];
					writeadr=((int)CommBuf[3])+(TempHighAddr<<8);
					CommBuf[CommIndexSlaveID]=SlaveID;			// 0
					CommBuf[CommIndexFunction]=ReadHoldReg;		// 1
					CommBuf[2]=CommBuf[CommIndexNoPointLo]*2;	// 2 Byte Count
		
					for (i=0;i<CommBuf[2];i++)	// sim inputval for test
						{
							SlaveOutputBuf[i]=ModebusRegsDataBuff[writeadr++];
						}
					i=CommBuf[2]+3;
					
					for(CommIndex=3;CommIndex<i;CommIndex++) // Send data Reg to Master
						{
							CommBuf[CommIndex]=SlaveOutputBuf[CommIndex-3]; //  Send high 8 bit first
						}
		
					crc16tem=crc16(CommBuf,CommIndex); // Then send low 8 bit data
					CommBuf[CommIndex++]=(char)(crc16tem & 0x00ff); // Send crccheck low 8 bit
					CommBuf[CommIndex]=(char)(crc16tem>>8); // Then send high 8 bit
					CommIndexEnd=CommIndex;
					CommIndex=0;
		
					for(i=0;i<=CommIndexEnd;i++)
						{
//							ScicRegs.SCITXBUF=CommBuf[i];
							SciTxQueue[SciTxHighPoint]=CommBuf[i];
							SciTxHighPoint++;
							if(SciTxHighPoint==MaxDataLen)
								{
									SciTxHighPoint=0;
								}
						}
					break;
	
				case PresetMulReg:	
					TempHighAddr=CommBuf[2];
					writeadr=((int)CommBuf[3])+(TempHighAddr<<8);
					j=CommBuf[CommIndexNoPointLo]*2;
					
					for (i=0;i<j;i++)
					{
					ModebusRegsDataBuff[writeadr]=CommBuf[i+7]; // get data that master send start 7th byte
					writeadr++;
					}
		
					CommBuf[CommIndexSlaveID]=SlaveID; // 0
					CommBuf[CommIndexFunction]=PresetMulReg; // 1
					CommIndex=2;
					CommBuf[CommIndex++]=CommBuf[CommIndexStartAdrHi];	// 2
					CommBuf[CommIndex++]=CommBuf[CommIndexStartAdrLo];	// 3
					CommBuf[CommIndex++]=CommBuf[CommIndexNoPointHi];	// 4
					CommBuf[CommIndex++]=CommBuf[CommIndexNoPointLo];	// 5
					crc16tem=crc16(CommBuf,CommIndex);	// Send low 8 bit data
					CommBuf[CommIndex++]=(char)(crc16tem & 0x00ff);	// Send crccheck low 8 bit first
					CommBuf[CommIndex]=(char)(crc16tem>>8);	// Then send high 8 bit
					CommIndexEnd=CommIndex;
					CommIndex=0;
				
					for(i=0;i<=CommIndexEnd;i++)
						{
							ScicRegs.SCITXBUF=CommBuf[i];
							SciTxQueue[SciTxHighPoint]=CommBuf[i];
							SciTxHighPoint++;
							if(SciTxHighPoint==MaxDataLen)
								{
									SciTxHighPoint=0;
								}
						}
		
					break;
	
				default:
					for (i=0;i<=80;i++) // delay // error recieve again
						{
							asm("		NOP");
						}
					CommIndex=0;
					//T0=RECIEVE;
					break;
			}
		}
		else
		{
			for (i=0;i<=80;i++) // delay
				{
					asm("		NOP");
				}
			CommIndex=0;
		}
	}

}


/*------------- intr code is following ----------------*/
void slavecomm(void)
{
	unsigned int i;
	
	while(ScicRegs.SCIFFRX.bit.RXFFST>0)
		{
			ModebusTimeoutFlag=1;		// xommunication is continuing
			if (CommIndex<CommIndexEnd)	// CommIndex less than recieve length
			{
				CommBuf[CommIndex]=(char)ScicRegs.SCIRXBUF.all;
				if (CommIndex==CommIndexFunction)
				{
					switch (CommBuf[CommIndexFunction])
					{
						case ReadMulCoilSta:
							CommIndexEnd=ReadCoilStaIndexEnd;
						break;
						
						case ForceSingleCoil:
							CommIndexEnd=ForceSingleCoilIndexEnd;
						break;
						
						case ReadHoldReg:
							CommIndexEnd=ReadHoldRegIndexEnd;
						break;
						
						case PresetMulReg:
							CommIndexEnd=7;	//6+CommBuf[CommIndexNoPointLo]*2+2;
						break;
						
						default:
							break;
					}
				}
		
				if (CommIndex==CommIndexNoPointLo)
				{
					switch (CommBuf[CommIndexFunction])
					{
						case PresetMulReg:
							CommIndexEnd=6+CommBuf[CommIndexNoPointLo]*2+2;
							break;
							
						default:
							break;
					}
				}
				
				CommIndex++;
			}
			else // Finished receiving CommIndex==CommIndexEnd
				{
					CommBuf[CommIndex]=(char)ScicRegs.SCIRXBUF.all;
			
					AnalyzeRecieve();
					CommIndex=0;
			
				}
		}
		
		if(ModebusTimeoutFlag==0)	// Communication stops for a while
		{
			CommIndex=0;			// Frame count is set to 0
		}
		
		ModebusTimeoutFlag=0;
		
	while((SciTxLowPoint!=SciTxHighPoint)&&(ScicRegs.SCIFFTX.bit.TXFFST<16))	//send queue
	{
		ScicRegs.SCITXBUF=SciTxQueue[SciTxLowPoint];
		SciTxLowPoint++;
		if(SciTxLowPoint==MaxDataLen)
			{
				SciTxLowPoint=0;
			}
	}
	
	if(ScicRegs.SCIRXST.bit.RXERROR==1)
	{
		InitSci();
		// Intializaiton of parameters for touch screen
		initvar();
	}

}

//===========================================================================
// No more.
//===========================================================================

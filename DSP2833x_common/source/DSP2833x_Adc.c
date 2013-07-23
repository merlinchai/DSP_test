// TI File $Revision: /main/4 $
// Checkin $Date: July 30, 2007   14:15:53 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x Header Files V1.01 $
// $Release Date: September 26, 2007 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "CommonDefine.h"

#define ADC_usDELAY  5000L

//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;




    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_CLOCK_SPEED define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels
}

void SetupAdc(void)		//by lwc 
{
// Highest ADC clock speed is 12.5MHz
	AdcRegs.ADCTRL3.bit.ADCCLKPS=ADC_CKPS;	// ADC is divided by 4; it is now 12.5MHz
	AdcRegs.ADCTRL1.bit.CPS=ADC_CPS;		// Can be further divided by 2, not divided if 0
// ADC period
	AdcRegs.ADCTRL1.bit.ACQ_PS=ADC_SHCLK;	// Sample and hold period, 16xADC clock
// Use sequential sampling
	AdcRegs.ADCTRL3.bit.SMODE_SEL=0;		// Sequential sampling. 
											// A and B channels are not sampled concurrently
	
// Cascade mode
	AdcRegs.ADCTRL1.bit.SEQ_CASC=1; 
	
// Enable external interrupt 2 to trigger ADC
 	AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1=0;		// Disable SEQ1 interrupt (every EOS)
   	AdcRegs.ADCTRL2.bit.EXT_SOC_SEQ1=1;  	// Enable EINT2 to start SEQ1
	
	
// Maximum conversion is 15
// One sample for each channel
	AdcRegs.ADCMAXCONV.all=15;
	
// Automatic continuous conversion mode not used.
// After sampling maximum number of conversions, revert to initial state
// Use start-stop mode, because only sampled each carrier period
// Revert to initial state using software
	AdcRegs.ADCTRL1.bit.CONT_RUN=0;

// ADCA0 to ADCA2 for 3 phase input voltage (sampled twice to reduce error)
// ADCA3 to ADCA5 for 3 phase output voltage
// ADCB0 to ADCB2 for 3 phase inductor 1 current
// ADCB3 to ADCB5 for 3 phase inductor 2 current

    AdcRegs.ADCCHSELSEQ1.bit.CONV00=0x0;   
    AdcRegs.ADCCHSELSEQ1.bit.CONV01=0x1;   
    AdcRegs.ADCCHSELSEQ1.bit.CONV02=0x2;
    AdcRegs.ADCCHSELSEQ1.bit.CONV03=0x3;
    AdcRegs.ADCCHSELSEQ2.bit.CONV04=0x4;   
    AdcRegs.ADCCHSELSEQ2.bit.CONV05=0x5;   
    AdcRegs.ADCCHSELSEQ2.bit.CONV06=0x6;
    AdcRegs.ADCCHSELSEQ2.bit.CONV07=0x7;
    AdcRegs.ADCCHSELSEQ3.bit.CONV08=0x8;   
    AdcRegs.ADCCHSELSEQ3.bit.CONV09=0x9;   
    AdcRegs.ADCCHSELSEQ3.bit.CONV10=0xa;
    AdcRegs.ADCCHSELSEQ3.bit.CONV11=0xb;
    AdcRegs.ADCCHSELSEQ4.bit.CONV12=0xc;   
    AdcRegs.ADCCHSELSEQ4.bit.CONV13=0xd;
    AdcRegs.ADCCHSELSEQ4.bit.CONV14=0xe;
    AdcRegs.ADCCHSELSEQ4.bit.CONV15=0xf; 
// ADC interrupt mode selection
	AdcRegs.ADCTRL2.bit.INT_MOD_SEQ1=0;		// Interrupt mode 0
											// Interrupt flag int_seq1 is set when SEQ1 ends  
}


void Adc_Inquire(void)		// ADC inquire
{
	int16 result1, result2, result3, result4, result5, result6, result7, result8;
	int16 result9, result10, result11, result12, result13, result14, result15, result0;
	
	// Sample ADC until complete
	while (AdcRegs.ADCST.bit.INT_SEQ1== 0){}
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;	// When conversion of seq1 of ADC is complete
										// clears interrupt flag int_seq1 

	result0=(int16)(AdcRegs.ADCRESULT0>>4);
	result1=(int16)(AdcRegs.ADCRESULT1>>4);
	result2=(int16)(AdcRegs.ADCRESULT2>>4);
	result3=(int16)(AdcRegs.ADCRESULT3>>4);
	result4=(int16)(AdcRegs.ADCRESULT4>>4);
	result5=(int16)(AdcRegs.ADCRESULT5>>4);
	result6=(int16)(AdcRegs.ADCRESULT6>>4);
	result7=(int16)(AdcRegs.ADCRESULT7>>4);
	result8=(int16)(AdcRegs.ADCRESULT8>>4);
	result9=(int16)(AdcRegs.ADCRESULT9>>4);
	result10=(int16)(AdcRegs.ADCRESULT10>>4);
	result11=(int16)(AdcRegs.ADCRESULT11>>4);
	result12=(int16)(AdcRegs.ADCRESULT12>>4);
	result13=(int16)(AdcRegs.ADCRESULT13>>4);
	result14=(int16)(AdcRegs.ADCRESULT14>>4);
	result15=(int16)(AdcRegs.ADCRESULT15>>4);	

     
     /* // ADC conversion values
	voltage_in_a=((int16)(AdcRegs.ADCRESULT0>>4)+(AdcRegs.ADCRESULT1>>4))>>1; 
	voltage_in_b=((int16)(AdcRegs.ADCRESULT2>>4)+(AdcRegs.ADCRESULT3>>4))>>1;     
	voltage_in_c=((int16)(AdcRegs.ADCRESULT4>>4)+(AdcRegs.ADCRESULT5>>4))>>1; 
	voltage_out_a=(int16)(AdcRegs.ADCRESULT6>>4);
	voltage_out_b=(int16)(AdcRegs.ADCRESULT7>>4);
	voltage_out_c=(int16)(AdcRegs.ADCRESULT8>>4);
	current_1_a=(int16)(AdcRegs.ADCRESULT9>>4);
	current_1_b=(int16)(AdcRegs.ADCRESULT10>>4);
	current_1_c=(int16)(AdcRegs.ADCRESULT11>>4);
	current_2_a=(int16)(AdcRegs.ADCRESULT12>>4);
	current_2_b=(int16)(AdcRegs.ADCRESULT13>>4);
	current_2_c=(int16)(AdcRegs.ADCRESULT14>>4);
	
// Practical calculations
	A_voltage_IN=voltage_in_a-ADCIni_A_voltageIN_offset;		// Input voltage actual value
	B_voltage_IN=voltage_in_b-ADCIni_B_voltageIN_offset; 
	C_voltage_IN=voltage_in_c-ADCIni_C_voltageIN_offset;
	A_voltage_OUT=voltage_out_a-ADCIni_A_voltageOUT_offset;		// Output voltage actual value
	B_voltage_OUT=voltage_out_b-ADCIni_B_voltageOUT_offset;
	C_voltage_OUT=voltage_out_c-ADCIni_C_voltageOUT_offset;

	A_current_1=current_1_a-ADCIni_A_current1_offset;			// Inductor 1 actual value
	B_current_1=current_1_b-ADCIni_B_current1_offset;
	C_current_1=current_1_c-ADCIni_C_current1_offset;
	A_current_2=current_2_a-ADCIni_A_current2_offset;			// Inductor 1 actual value
	B_current_2=current_2_b-ADCIni_B_current2_offset;
	C_current_2=current_2_c-ADCIni_C_current2_offset;

*/

                         
 // Reinitialize for next ADC sequence
  	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1 
      
  	return;


}

//===========================================================================
// End of file.
//===========================================================================

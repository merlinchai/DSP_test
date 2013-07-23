/* ==============================================================================

System Name:  IIR Filter Demo, This system uses IIR5BIQ16 (16 bit implementation) module

File Name:    IIR16D.C

Description:  Primary System file for demonstrating the IIR Filter

Originator:   Digital control systems Group - Texas Instruments

Target dependency:  C28x

Description:
============

This software demonstrates the IIR Filter Usage

   The frame work, for demonstrating the FILTER is given below
   
   
   CH0  |--------------|           |------------|               |-----------|
   ---->|              |           |            |               |           |
   CH0  |              |           |            |               |           |         
   ---->|              |     input |            | Output        |  EVMDAC   |
   CH0  |   ADC04U_DRV |-----|---->|    IIR     |-------------->|  PWMDAC   |
   ---->|              |     |     |   FILTER   |               |  DATALOG  |
   CH0  |              |     |     |            |          |--->|           |
   --|->|              |     |     |            |          |    |           |
        |--------------|     |     |------------|          |    |-----------|
                             |                             |
                             |-----------------------------|
 
*/

#include <filter.h>         

/* Define Constant Co-efficient Array and place the .econst/.const section in
non-volatile memory */    
const int IIR_coeff[5*IIR16_NBIQ]=IIR16_COEFF;
         
/* Finter Input and Output Variables                */   
     
void IIR_LPF_Initialisation(IIR5BIQ16_handle iir, int dbuffer[])
{
/* IIR Filter Initialisation                                */ 
        iir->dbuffer_ptr=dbuffer;
        iir->coeff_ptr=(int *)IIR_coeff; 		
        iir->qfmat=IIR16_QFMAT;
        iir->nbiq=IIR16_NBIQ; 
        iir->isf=IIR16_ISF; 
        iir->init(iir);
              
}
/*---------------------------------------------------------------------------
    Nothing running in the background at present           
----------------------------------------------------------------------------*/

int IIR_LPF_Calculation(IIR5BIQ16_handle iir, int xn) 
{
		int yn;

        iir->input=xn;           
        iir->calc(iir);
        yn=iir->output;
		return yn;
}       


             





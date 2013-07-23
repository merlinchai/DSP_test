/* ==============================================================================

System Name:    Generic FIR Filter Demo, This system uses FIRFILT_GEN module

File Name:      FIRGD.C

Description:    Primary System file for demonstrating the FIR Filter

Originator:     Digital control systems Group - Texas Instruments

Target dependency:  C28x
Description:
============

This software demonstrates the FIR Filter Usage

   The frame work, for demonstrating the FILTER is given below
   
   
   CH0  |--------------|           |------------|               |-----------|
   ---->|              |           |            |               |           |
   CH0  |              |           |            |               |           |         
   ---->|              |     input |            | Output        |  EVMDAC   |
   CH0  |   ADC04U_DRV |-----|---->|    FIR     |-------------->|  PWMDAC   |
   ---->|              |     |     |   FILTER   |               |  DATALOG  |
   CH0  |              |     |     |            |          |--->|           |
   --|->|              |     |     |            |          |    |           |
        |--------------|     |     |------------|          |    |-----------|
                             |                             |
                             |-----------------------------|
 
*/

#include <fir.h>
         
/* Define Constant Co-efficient Array and place the .econst/.const section in
non-volatile memory */
const long FIR_coeff[(FIR_ORDER+2)/2]= FIR16_COEFF;
         
/* Finter Input and Output Variables                */   
     
void FIR_LPF_Initialisation(FIR16_handle fir, long dbuffer[])
{    

/* FIR Generic Filter Initialisation    */ 
       fir->order=FIR_ORDER; 
       fir->dbuffer_ptr=dbuffer;
       fir->coeff_ptr=(long *)FIR_coeff; 
       fir->init(fir);
              
}


int FIR_LPF_Calculation(FIR16_handle fir, int xn) 
{
	int yn;

    fir->input=xn;           
    fir->calc(fir);
    yn=fir->output;
    return yn;
}       


             





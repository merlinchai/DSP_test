/* ==============================================================================

System Name:  IIR Filter Demo, This system uses IIR5BIQ32 (32 bit implementation) module

File Name: IIR32D.C

Description:    Primary System file for demonstrating the IIR Filter

Originator:     Digital control systems Group - Texas Instruments

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

   


/* Define Constant Co-efficient Array  and place the
.constant section in ROM memory				*/ 
const long IIR32_coeff[5*IIR32_NBIQ]=IIR32_COEFF;
         



void IIR32_LPF_Initialisation(IIR5BIQ32_handle iir, long dbuffer[])
{
/* IIR Filter Initialisation                                */ 
       iir->dbuffer_ptr=dbuffer;
       iir->coeff_ptr=(long *)IIR32_coeff; 
       iir->qfmat=IIR32_QFMAT;
       iir->nbiq=IIR32_NBIQ; 
       iir->isf=IIR32_ISF; 
       iir->init(iir);
}              

/*---------------------------------------------------------------------------
    Nothing running in the background at present           
----------------------------------------------------------------------------*/

int IIR32_LPF_Calculation(IIR5BIQ32_handle iir, int xn) 
{
			int yn;

            iir->input=xn;           
            iir->calc(iir);
            yn=iir->output16;

			return yn;
}       


             





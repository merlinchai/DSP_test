/*=====================================================================================
 File name:        ICLARKE.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  Clarke Transformation                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "IQmathLib.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "dmctype.h"
#include "iclarke.h"

void iclarke_calc(ICLARKE *v)
{	
   v->As=v->Alpha;
   v->Bs=_IQmpy(_IQ(-0.5),v->Alpha)+_IQmpy(_IQ(0.86602540378444),v->Beta); // sqrt(3)/2 = 0.86602540378444
   v->Cs=_IQmpy(_IQ(-0.5),v->Alpha)+_IQmpy(_IQ(-0.86602540378444),v->Beta);   
}



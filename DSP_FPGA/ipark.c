/*=====================================================================================
 File name:        IPARK.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Dsscription:  Inverse Park Transformation                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "IQmathLib.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "dmctype.h"
#include "ipark.h"

void ipark_calc(IPARK *v)
{	
   
   
// Using look-up IQ sine table

     v->Alpha = _IQmpy(v->Ds,v->Cosine) - _IQmpy(v->Qs,v->Sine);
     v->Beta = _IQmpy(v->Qs,v->Cosine) + _IQmpy(v->Ds,v->Sine);  
}



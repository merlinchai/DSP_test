/*=====================================================================================
 File name:        PARK.C  (IQ version)                  
                    
 Originator:	Digital Control Systems Group
			Texas Instruments

 Alphascription:  Park Transformation                   

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/

#include "IQmathLib.h"         // Include header for IQmath library
// Don't forget to set a proper GLOBAL_Q in "IQmathLib.h" file
#include "dmctype.h"
#include "park.h"

void park_calc(PARK *v)
{	

// Using look-up IQ sine table
    // Sine = _IQsinPU(v->Angle);
     //Cosine = _IQcosPU(v->Angle);

     v->Ds = _IQmpy(v->Alpha,v->Cosine) + _IQmpy(v->Beta,v->Sine);
     v->Qs = _IQmpy(v->Beta,v->Cosine) - _IQmpy(v->Alpha,v->Sine);

}



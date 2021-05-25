/* ==============================================================================
System Name:  	HVACI_Sensorless

File Name:		HVACI_Sensorless.h

Description:	Primary system header file for the Real Implementation of Sensorless  
          		Field Orientation Control for Induction Motor

Originator:		Digital control systems Group - Texas Instruments

 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2010	Version 1.0
=================================================================================  */

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/


#include "park.h"       		// Include header for the PARK object 
#include "ipark.h"       		// Include header for the IPARK object 
#include "pid_reg3.h"       	// Include header for the PIDREG3 object 
#include "clarke.h"         	// Include header for the CLARKE object 
#include "rampgen.h"        	// Include header for the RAMPGEN object 
#include "rmp_cntl.h"       	// Include header for the RMPCNTL object 
#include "volt_calc.h"      	// Include header for the PHASEVOLTAGE object 
#include "svgen_mf.h"
#include "svgen_dq.h"

#if (DSP2833x_DEVICE_H==1)
#include "f2833xpwmdac.h"       // Include header for the PWMDAC object
#include "f2833xqep.h"        	// Include header for the QEP object
#include "f2833xcap.h"        	// Include header for the CAP object
#include "f2833xpwm.h"
#include "f2833xileg_vdc_ls.h"

#endif


//===========================================================================
// No more.
//===========================================================================

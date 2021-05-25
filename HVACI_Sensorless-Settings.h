/* =================================================================================
File name:  HVACI_Sensorless-Settings.H                     
                    
Originator:	Digital Control Systems Group
			Texas Instruments

Description: 
Incremental Build Level control file.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 02-09-2010	Version 1.0
=================================================================================  */
#ifndef PROJ_SETTINGS_H

/*------------------------------------------------------------------------------
Following is the list of the Build Level choices.
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
This line sets the BUILDLEVEL to one of the available choices.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------


#ifndef TRUE
#define FALSE 0
#define TRUE  1
#endif

#define SQRT2 _iq(1.414213562)
#define PI _iq(3.14159265358979)

// Define the system frequency (MHz)

#if (DSP2803x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 60
#elif (DSP280x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 100
#elif (DSP2833x_DEVICE_H==1)
#define SYSTEM_FREQUENCY 150
#endif


//Define system Math Type
// Select Floating Math Type for 2833x
// Select IQ Math Type for 2803x 
#if (DSP2803x_DEVICE_H==1)
#define MATH_TYPE 0 
#elif (DSP2833x_DEVICE_H==1)
#define MATH_TYPE 1
#endif



// Define the ISR frequency (kHz)
#define ISR_FREQUENCY 10

#endif



#include "DSP2833x_Device.h"   // DSP2833x header file
//#include "DSP2833x_DefaultISR.h"                // Definiciones de ISR
#include "Solar_IQ.h"

#ifndef MEDICIONES_H_
#define MEDICIONES_H_

extern void Mediciones();

extern _iq VgridU;     //Tension de fase U de la red
extern _iq VgridV;     //Tension de fase V de la red
extern _iq VgridW;     //Tension de fase W de la red
extern _iq VinversorU;      //Tension de fase U del inversor
extern _iq VinversorV;      //Tension de fase V del inversor
extern _iq VinversorW;      //Tension de fase W del inversor
extern _iq IcargaU;     //Corriente de carga U de la red
extern _iq IcargaV;    //Corriente de carga V de la red
extern _iq IcargaW;     //Corriente de carga W de la red
extern _iq IgridU;   //Corriente de fase U de la red
extern _iq IgridV;   //Corriente de fase V de la red
extern _iq IgridW;    //Corriente de fase W de la red
extern _iq IBUS;     //Corriente de BUS
extern _iq VBUS;

#endif /* MEDICIONES_H_ */

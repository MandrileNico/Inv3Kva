#include "Mediciones.h"


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
//***************************************************************************
    //Adquisición de mediciones
    //***************************************************************************
    // 1/4096=0.00024414. Resta 0.5 para eliminar la referencia de 1.5V y multiplica por 2 para lograr 1 a fondo de escala.


void Mediciones(){
        VinversorW = ((AdcMirror.ADCRESULT12)*0.00024414-0.5)*2.0;  // Phase W tensión sobre capacitores de salida del inversor.
        VinversorV = ((AdcMirror.ADCRESULT14)*0.00024414-0.5)*2.0;  // Phase V tensión sobre capacitores de salida del inversor.
        VinversorU = ((AdcMirror.ADCRESULT15)*0.00024414-0.5)*2.0;  // Phase U tensión sobre capacitores de salida del inversor.
        IcargaW = ((AdcMirror.ADCRESULT5)*0.00024414-0.5)*2.0;      // Phase W corriente de carga.
        IcargaV = ((AdcMirror.ADCRESULT13)*0.00024414-0.5)*2.0;     // Phase V corriente de carga.
        IcargaU = ((AdcMirror.ADCRESULT11)*0.00024414-0.5)*2.0;     // Phase U corriente de carga.
        VgridW = ((AdcMirror.ADCRESULT0)*0.00024414-0.5)*2.0;       // Phase W tensión de red.
        VgridV = ((AdcMirror.ADCRESULT1)*0.00024414-0.5)*2.0;       // Phase V tensión de red.
        VgridU = ((AdcMirror.ADCRESULT2)*0.00024414-0.5)*2.0;       // Phase U tensión de red.
        IgridW = ((AdcMirror.ADCRESULT8)*0.00024414-0.5)*2.0;       // Phase W corriente de red.
        IgridV = ((AdcMirror.ADCRESULT9)*0.00024414-0.5)*2.0;       // Phase V corriente de red.
        IgridU = ((AdcMirror.ADCRESULT10)*0.00024414-0.5)*2.0;      // Phase U corriente de red.
        IBUS   = ((AdcMirror.ADCRESULT4)*0.00024414-0.5)*2.0;       // Corriente de BUS.
        VBUS   = (AdcMirror.ADCRESULT7)*0.00024414;     // Tensión de BUS (NO SE RESTA NI MULTIPLICA PORQUE ES CONTINUA).

        //Temperatura = ((AdcMirror.ADCRESULT6)*0.01715);

}






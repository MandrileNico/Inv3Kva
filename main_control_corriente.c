//###########################################################################
// Description: verion actual en uso

//
//###########################################################################
// $TI Release: F2833x/F2823x Header Files and Peripheral Examples V140 $
// $Release Date: March  4, 2015 $
// $Copyright: Copyright (C) 2007-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "PeripheralHeaderIncludes.h"
#include "HVACI_Sensorless-Settings.h"
#include "IQmathLib.h"
#include "HVACI_Sensorless.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Solar_IQ.h"

#include "Mediciones.h"
#include "TripZoneEnable.h"
#include "DSP28x_Project.h"

#define T 0.0001			//Periodo PWM e interrupcion (10Khz)
#define BASE_FREQ 200		//Frecuencia de base para SVGEN (maxima frecuencia para freq=1)
#define frec50Hz 0.04		//Constante para obtener 50Hz (BASE_FREQ*freq50hz)
#define V_CONSTANTE 12.1 //Constante para obtener la relacion cantidad de cuentas por voltio (cuentas/V=12.1)
#define BASE_VOLT_CONTINUA 339.2 // voltaje maximo a medir, en por unidad seria 1
#define BASE_CURRENT_CONT 30.3
//#define BASE_CURRENT 45.4
#define BASE_VOLT_ALTERNA 448.77
#define GRID_FREQ 50
#define CONTACTOR_CARGA GpioDataRegs.GPCSET.bit.GPIO87
#define CONTACTOR_LINEA GpioDataRegs.GPASET.bit.GPIO22
#define Igrid_lim_min 1		// corriente instantanea minima
#define Igrid_lim_max 2		// corriente instantanea maxima


// Prototype statements for functions found within this file.
void delay_loop(void);
interrupt void MainISR(void);
__interrupt void epwm1_tzint_isr(void);
Uint32  EPwm1TZIntCount; // sacar luego de funcione la interrupcion
void DeviceInit();
void TZ_Protection(void);
void TZ_Clear(void);

Uint16 STOP = 0, prueba=0, I_INST_LIM_FLAG=0;
int16 VDC_FLAG = -1, cuenta=0, VEFF_FLAG=-1, IEFF_FLAG=-1, cont_ciclos_flag=1;

float32 Veff_inv=0, Veff_grid=0;
float32 Veff_sp=0;
float32 Id_sp=0;
float32 Iq_sp=0;
float32 Ieff_carga=0, Ieff_grid=0;
float32 Imax=5;
float32 buffer_VinvU[200], buffer_VinvV[200], buffer_VinvW[200], buffer_VgridW[200], buffer_VgridV[200], buffer_VgridU[200];
float32 buffer_theta[200], buffer_rampa[200];
Uint16 AcknowledgeTZ = 0;

//float32 bufferU[200], bufferV[200], bufferW[200];
//float32 buffer_VinvU[200]; buffer_VinvV[200], buffer_SP[200], buffer_Out[200], buffer_error[200], buffer_VgridV1[200];

_iq frecuencia=frec50Hz, gananciaQ=0, gananciaD=0.5, BASE_CURRENT=45.4;
_iq Vq_inv_filtrada = _IQ(0);
_iq Vd_inv_filtrada = _IQ(0);
_iq Vq_grid_filtrada = _IQ(0);
_iq Vd_grid_filtrada = _IQ(0);
_iq Iq_grid_filtrada = _IQ(0);
_iq Id_grid_filtrada = _IQ(0);
_iq Id_carga_filtrada = _IQ(0);
_iq Iq_carga_filtrada = _IQ(0);


_iq VBUS_filtrada = _IQ(0);
_iq error = _IQ(0), theta = _IQ(0);

//Declaración de variables para mediciones de tesiones y corrientes normalizadas entre -1 y 1
_iq VgridU = _IQ(0);    //Tension de fase U de la red
_iq VgridV = _IQ(0);    //Tension de fase V de la red
_iq VgridW = _IQ(0);    //Tension de fase W de la red
_iq VinversorU = _IQ(0);     //Tension de fase U del inversor
_iq VinversorV = _IQ(0);     //Tension de fase V del inversor
_iq VinversorW = _IQ(0);     //Tension de fase W del inversor
_iq IcargaU = _IQ(0);    //Corriente de carga U de la red
_iq IcargaV = _IQ(0);    //Corriente de carga V de la red
_iq IcargaW = _IQ(0);    //Corriente de carga W de la red
_iq IgridU = _IQ(0);    //Corriente de fase U de la red
_iq IgridV = _IQ(0);    //Corriente de fase V de la red
_iq IgridW = _IQ(0);    //Corriente de fase W de la red
_iq IBUS = _IQ(0);		//Corriente de BUS
_iq Vd_grid = _IQ(0);
_iq Vq_grid = _IQ(0);
_iq Id_grid = _IQ(0);
_iq Iq_grid = _IQ(0);
_iq Vd_inv = _IQ(0);
_iq Vq_inv = _IQ(0);
_iq Id_carga = _IQ(0);
_iq Iq_carga = _IQ(0);
_iq VBUS = _IQ(0);
_iq theta_PLL_sinfiltro = _IQ(0);
_iq desfasaje = _IQ(0);
_iq a = _IQ(0);
_iq theta_PLL = _IQ(0);	//Angulo de fase de la tensión de red
_iq21 B0 = _IQ21(0);	// Coeficientes filtro PLL
_iq21 B1 = _IQ21(0);	// Coeficientes filtro PLL
_iq Comp=0;	// valor de compensancion del decoupling
_iq V_D;	// V_D a la salida del lazo
_iq V_Q;	// V_Q a la salida del lazo
_iq ARRANQUE_CON_RED=0;	// variable para arrancar desde la programacion


//Declaración de instancias de las macros
PWMGEN pwm1 = PWMGEN_DEFAULTS;			//PWM
IPARK ipark_SP_tension = IPARK_DEFAULTS;			//Transformacion de park inversa
SVGENDQ vectdq = SVGENDQ_DEFAULTS;		//Generador de vector rotante
RAMPGEN rampa = RAMPGEN_DEFAULTS;		//Generador de rampa
CLARKE clarke_tension_salida = CLARKE_DEFAULTS;
CLARKE clarke_corriente = CLARKE_DEFAULTS;
PARK park_tension_salida = PARK_DEFAULTS;
PARK park_corriente = PARK_DEFAULTS;
PHASEVOLTAGE volt1 = PHASEVOLTAGE_DEFAULTS;
RMPCNTL rc1 = RMPCNTL_DEFAULTS;				//Control de rampa
RMPCNTL rcid = RMPCNTL_DEFAULTS;			//Control de rampa
RMPCNTL rciq = RMPCNTL_DEFAULTS;			//Control de rampa
RMPCNTL rc_vd_arranque = RMPCNTL_DEFAULTS;	//Control de rampa
PIDREG3 pid_vout = PIDREG3_DEFAULTS;		//PID para control de tension
PIDREG3 pid_id = PIDREG3_DEFAULTS;			//PID para control de ID
PIDREG3 pid_iq = PIDREG3_DEFAULTS;			//PID para control de IQ
ABC_DQ0_POS_IQ ABC_DQ_Vgrid;
ABC_DQ0_POS_IQ ABC_DQ_Igrid;
ABC_DQ0_POS_IQ ABC_DQ_Vinversor;
ABC_DQ0_POS_IQ ABC_DQ_Icarga;
SPLL_3ph_SRF_IQ PLL_grid;




void main(void){

    //***************************************************************************
    //INICIALIZACIONES
    //***************************************************************************



    DINT;			//Desactiva interrupciones antes de la inicializacion
	DeviceInit(); 	//Inicializacion PLL, WatchDog, enable Peripheral Clocks

	//Inicializacion del modulo PWM
	//Calculo del periodo del PWM a partir de la frecuencia del micro (en cantidad de ciclos de clock que entran en un semiperiodo)
	pwm1.PeriodMax = SYSTEM_FREQUENCY*(Uint16)(1000000*T)/2;  // Prescaler X1 (T1), ISR period = T x 1
	PWM_INIT_MACRO(pwm1)
	TripZoneEnable();
	EALLOW;  // This is needed to write to EALLOW protected registers
	   PieVectTable.EPWM1_TZINT = &epwm1_tzint_isr;
	EDIS;
	//Inicializacion del módulo ADC
	ADC_MACRO_INIT()

	//Inicializacion del generador de rampa
	rampa.Gain = _IQ(1);
	rampa.StepAngleMax = _IQ(BASE_FREQ*T);

	//Inicializacion de rampa de Veff de referencia
	rc1.RampDelayMax=10;
		//Inicializacion de rampa de SP de Id
	rcid.RampDelayMax=1;
		//Inicializacion de rampa de SP de Iq
	rciq.RampDelayMax=1;
		//Inicializacion de rampa de Vd en arranque
	rc_vd_arranque.RampDelayMax=1;
	rc_vd_arranque.TargetValue=ARRANQUE_CON_RED;
	
	IER |= M_INT2;
	PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

//	//Inicializacion de PID para control de tension
//	pid_vout.Kp = _IQ(0.01);				// Con los valores de MatLab Kp = Pi*(I0/V0)
//	pid_vout.Ki = _IQ(0.01);				// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0)
//	pid_vout.Kd = _IQ(0);
//	pid_vout.Kc = _IQ(0.1);
//	pid_vout.OutMax = _IQ(0.99);
//	pid_vout.OutMin = _IQ(0.01);
//	//Inicializacion de PID para control de ID
//	pid_id.Kp = _IQ(1);				// Con los valores de MatLab Kp = Pi*(I0/V0)
//	pid_id.Ki = _IQ(0);				// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0)
//	pid_id.Kd = _IQ(0);
//	pid_id.Kc = _IQ(0);
//	pid_id.OutMax = _IQ(0.99);
//	pid_id.OutMin = _IQ(0.01);
//	//Inicializacion de PID para control de IQ
//	pid_iq.Kp = _IQ(1);				// Con los valores de MatLab Kp = Pi*(I0/V0)
//	pid_iq.Ki = _IQ(0);				// Con los valores de MatLab Ki = (Pi/Ii)*(I0/V0)
//	pid_iq.Kd = _IQ(0);
//	pid_iq.Kc = _IQ(0);
//	pid_iq.OutMax = _IQ(0.99);
//	pid_iq.OutMin = _IQ(0.01);

	//Inicialización de transformada abc --> dq     tuve que crear macros porque las funciones daban error al linkear

//	ABC_DQ0_POS_IQ_MACRO_init(ABC_DQ_Vinversor); // Inicialización de transformada abc->dq de tensión del inversor
//	ABC_DQ0_POS_IQ_MACRO_init(ABC_DQ_Igrid); // Inicialización de transformada abc->dq de corriente de la red
//	ABC_DQ0_POS_IQ_MACRO_init(ABC_DQ_Vgrid); // Inicialización de transformada abc->dq de tensión de red
//	ABC_DQ0_POS_IQ_MACRO_init(ABC_DQ_Icarga); // Inicialización de transformada abc->dq de corriente de carga
	//ABC_DQ0_POS_IQ_init(&ABC_DQ_Vinversor);
	//ABC_DQ0_POS_IQ_init(&ABC_DQ_Vgrid);
	//ABC_DQ0_POS_IQ_init(&ABC_DQ_Igrid);

	//Inicialización del PLL		tuve que crear macros porque las funciones daban error al linkear
									//la macro usa los valores 50Hz, 0.0001s y B0 y B1 para 10Khz
//	SPLL_3ph_SRF_IQ_MACRO_init(PLL_grid);
	//SPLL_3ph_SRF_IQ_init(GRID_FREQ,_IQ21(T),&PLL_grid);
    //B0 = 180.4878;  // ver bibliografía de SolarLib para la obtención de estos valores (calculado para 10kHz)
    //B1 = -165.2122; // ver bibliografía de SolarLib para la obtención de estos valores (calculado para 10kHz)
    //PLL_grid.lpf_coeff.B0_lf = B0; // coeficientes del filtro pasabajos del PLL
    //PLL_grid.lpf_coeff.B1_lf = B1; // coeficientes del filtro pasabajos del PLL

	//***************************************************************************
	//Configuración de las interrupciones
	//***************************************************************************
	EALLOW;	// This is needed to write to EALLOW protected registers

	//Asignacion de la direccion de la rutina MainISR a la posicion de la tabla correspondiente a la interrupcion de PWM1
	PieVectTable.EPWM1_INT = &MainISR;
    //PieVectTable.XNMI= &DesatISR; habilitar protección
	EDIS;

	// Enable PIE group 3 interrupt 1 for EPWM1_INT
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;

	//Configuracion de la interrupcion del PWM1
	// Enable CNT_zero interrupt using EPWM1 Time-base
	EPwm1Regs.ETSEL.bit.INTEN = 1;   	//Enable EPWM1INT generation
	//EPwm1Regs.ETSEL.bit.INTSEL = 1;  	//Enable interrupt CNT_zero event
	EPwm1Regs.ETSEL.bit.INTSEL = 2;  	//Enable interrupt period event
	EPwm1Regs.ETPS.bit.INTPRD = 1;   	//Generate interrupt on the 1st event
	EPwm1Regs.ETCLR.bit.INT = 1;     	//Limpia flag


	// Enable CPU INT3 for EPWM1_INT:
	IER |= M_INT3;

	// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;	// Enable Global realtime interrupt DBGM

    V_D=0.75;
    V_Q=0;
    EPwm1TZIntCount = 0;
	//LOOP INFINITO
	for(;;){
		//Titila led control card
		GpioDataRegs.GPADAT.bit.GPIO31    =1;
		//GpioDataRegs.GPBDAT.bit.GPIO34    =1;
		delay_loop();

		GpioDataRegs.GPADAT.bit.GPIO31    =0;
		//GpioDataRegs.GPBDAT.bit.GPIO34    =0;
		delay_loop();
	}
}

void delay_loop(){
    volatile long i;
    for (i = 0; i < 1000000; i++) {}
}


interrupt void MainISR(void){

//	if (ARRANQUE_CON_RED>1)
//		ARRANQUE_CON_RED=1;

	    Mediciones();
//            VinversorW = ((AdcMirror.ADCRESULT12)*0.00024414-0.5)*2.0;  // Phase W tensión sobre capacitores de salida del inversor.
//            VinversorV = ((AdcMirror.ADCRESULT14)*0.00024414-0.5)*2.0;  // Phase V tensión sobre capacitores de salida del inversor.
//            VinversorU = ((AdcMirror.ADCRESULT15)*0.00024414-0.5)*2.0;  // Phase U tensión sobre capacitores de salida del inversor.
//            IcargaW = ((AdcMirror.ADCRESULT5)*0.00024414-0.5)*2.0;      // Phase W corriente de carga.
//            IcargaV = ((AdcMirror.ADCRESULT13)*0.00024414-0.5)*2.0;     // Phase V corriente de carga.
//            IcargaU = ((AdcMirror.ADCRESULT11)*0.00024414-0.5)*2.0;     // Phase U corriente de carga.
//            VgridW = ((AdcMirror.ADCRESULT0)*0.00024414-0.5)*2.0;       // Phase W tensión de red.
//            VgridV = ((AdcMirror.ADCRESULT1)*0.00024414-0.5)*2.0;       // Phase V tensión de red.
//            VgridU = ((AdcMirror.ADCRESULT2)*0.00024414-0.5)*2.0;       // Phase U tensión de red.
//            IgridW = ((AdcMirror.ADCRESULT8)*0.00024414-0.5)*2.0;       // Phase W corriente de red.
//            IgridV = ((AdcMirror.ADCRESULT9)*0.00024414-0.5)*2.0;       // Phase V corriente de red.
//            IgridU = ((AdcMirror.ADCRESULT10)*0.00024414-0.5)*2.0;      // Phase U corriente de red.
//            IBUS   = ((AdcMirror.ADCRESULT4)*0.00024414-0.5)*2.0;       // Corriente de BUS.
//            VBUS   = (AdcMirror.ADCRESULT7)*0.00024414;     // Tensión de BUS (NO SE RESTA NI MULTIPLICA PORQUE ES CONTINUA).
    //Parametrizacion de los valores de la rampa de angulo
        rampa.Freq = _IQ(frecuencia);

        //Calculo del angulo
        RG_MACRO(rampa)

    //***************************************************************************
    //Transformadas ABC --> DQ
    //***************************************************************************

	// ------------------------------------------------------------------------------
    // TENSIÓN DE RED
	// ------------------------------------------------------------------------------
//    ABC_DQ_Vgrid.a = VgridU;
//    ABC_DQ_Vgrid.b = VgridV;
//    ABC_DQ_Vgrid.c = VgridW;
//    //ABC_DQ_Vgrid.sin = sin(rampa.Out); // con angulo simulado
//    //ABC_DQ_Vgrid.cos = cos(rampa.Out); // con angulo simulado
//    ABC_DQ_Vgrid.sin = _IQsin(theta_PLL);
//    ABC_DQ_Vgrid.cos = _IQcos(theta_PLL);
//
//    ABC_DQ0_POS_IQ_MACRO(ABC_DQ_Vgrid);

//    Vd_grid = 0.5;
//    Vq_grid = 0;


	// ------------------------------------------------------------------------------
    // TENIÓN GENERADA (TENSIÓN SOBRE CAPACITORES)
    // ------------------------------------------------------------------------------

 //if (PLL_grid.fo<42 || PLL_grid.fo>58) {
	/*	ABC_DQ_Vinversor.a = VinversorU;
		ABC_DQ_Vinversor.b = VinversorV;
		ABC_DQ_Vinversor.c = VinversorW;
		//ABC_DQ_Vinversor.sin = _IQsinPU(rampa.Out);
		//ABC_DQ_Vinversor.cos = _IQcosPU(rampa.Out);
		ABC_DQ_Vinversor.sin = _IQsinPU(rampa.Out);
		ABC_DQ_Vinversor.cos = _IQcosPU(rampa.Out);

		ABC_DQ0_POS_IQ_MACRO(ABC_DQ_Vinversor);

		Vd_inv = ABC_DQ_Vinversor.d;
		Vq_inv = ABC_DQ_Vinversor.q;
	}
	*/


	//else {
//		ABC_DQ_Vinversor.a = VinversorU;
//		ABC_DQ_Vinversor.b = VinversorV;
//		ABC_DQ_Vinversor.c = VinversorW;
//		//ABC_DQ_Vinversor.sin = _IQsinPU(rampa.Out); // con angulo simulado
//		//ABC_DQ_Vinversor.cos = _IQcosPU(rampa.Out); // con angulo simulado
//		ABC_DQ_Vinversor.sin = _IQsin(desfasaje+theta_PLL);
//		ABC_DQ_Vinversor.cos = _IQcos(desfasaje+theta_PLL);
//
//		ABC_DQ0_POS_IQ_MACRO(ABC_DQ_Vinversor);
//
//		Vd_inv = ABC_DQ_Vinversor.d;
//		Vq_inv = ABC_DQ_Vinversor.q;
	// }

	// ------------------------------------------------------------------------------
    // CORRIENTE DE RED
    // ------------------------------------------------------------------------------
//	ABC_DQ_Igrid.a = IgridU;
//    ABC_DQ_Igrid.b = IgridV;
//    ABC_DQ_Igrid.c = IgridW;
//    ABC_DQ_Igrid.sin = _IQsin(theta_PLL);
//    ABC_DQ_Igrid.cos = _IQcos(theta_PLL);
//
//    ABC_DQ0_POS_IQ_MACRO(ABC_DQ_Igrid);
//
//    Id_grid = ABC_DQ_Igrid.d;
//    Iq_grid = ABC_DQ_Igrid.q;


	// ------------------------------------------------------------------------------
    // CORRIENTE DE CARGA
    // ------------------------------------------------------------------------------
//	ABC_DQ_Icarga.a = IcargaU;
//    ABC_DQ_Icarga.b = IcargaV;
//    ABC_DQ_Icarga.c = IcargaW;
//    //ABC_DQ_Icarga.sin = _IQsin(rampa.Out);
//    //ABC_DQ_Icarga.cos = _IQcos(rampa.Out);
//    ABC_DQ_Icarga.sin = _IQsin(desfasaje+theta_PLL);
//    ABC_DQ_Icarga.cos = _IQcos(desfasaje+theta_PLL);
//
//    ABC_DQ0_POS_IQ_MACRO(ABC_DQ_Icarga);
//
//    Id_carga = ABC_DQ_Icarga.d;
//    Iq_carga = ABC_DQ_Icarga.q;

    //***************************************************************************
    //***************************************************************************

    // ------------------------------------------------------------------------------
    //  Versiones filtradas de corrientes y tensiones
    // ------------------------------------------------------------------------------

//    Vq_inv_filtrada= 0.000999 * Vq_inv + 0.999001 * Vq_inv_filtrada;
//
//    Vd_inv_filtrada= 0.000999 * Vd_inv + 0.999001 * Vd_inv_filtrada;
//
//	Vq_grid_filtrada= 0.000999 * Vq_grid + 0.999001 * Vq_grid_filtrada;
//
//    Vd_grid_filtrada= 0.000999 * Vd_grid + 0.999001 * Vd_grid_filtrada;
//
//    Iq_grid_filtrada= 0.000999 * Iq_grid + 0.999001 * Iq_grid_filtrada;
//
//    Id_grid_filtrada= 0.000999 * Id_grid + 0.999001 * Id_grid_filtrada;
//
//    Iq_carga_filtrada= 0.000999 * Iq_carga + 0.999001 * Iq_carga_filtrada;
//
//    Id_carga_filtrada= 0.000999 * Id_carga + 0.999001 * Id_carga_filtrada;
//
//	VBUS_filtrada= 0.000999 * VBUS + 0.999001 * VBUS_filtrada;

    // ------------------------------------------------------------------------------
	// Cálculo de TENSIÓN EFICAZ del inversor y de la red
    // ------------------------------------------------------------------------------
	Veff_inv=_IQmag(Vq_inv_filtrada, Vd_inv_filtrada)*BASE_VOLT_ALTERNA*0.707106;	//Tension en P.U. por la base y sobre sqrt(2) para sacar la tensión eficaz
	Veff_grid=_IQmag(Vq_grid_filtrada, Vd_grid_filtrada)*BASE_VOLT_ALTERNA*0.707106;	//Tension en P.U. por la base y sobre sqrt(2) para sacar la tensión eficaz

	//Veff_sp=Veff_grid; //Se asigna como setpoint de tension la tension eficaz de la linea (para pruebas)

    // ------------------------------------------------------------------------------
	// Cálculo de CORRIENTE EFICAZ de la carga y de la red
    // ------------------------------------------------------------------------------
	Ieff_carga=_IQmag(Id_carga_filtrada, Iq_carga_filtrada)*BASE_CURRENT*0.707106;	//Corriente en P.U. por la base y sobre sqrt(2) para sacar la corriente eficaz
	Ieff_grid=_IQmag(Id_grid_filtrada, Iq_grid_filtrada)*BASE_CURRENT*0.707106;	//Corriente en P.U. por la base y sobre sqrt(2) para sacar la corriente eficaz

    // ------------------------------------------------------------------------------
	// Cálculo de TENSIÓN DE BUS
    // ------------------------------------------------------------------------------

    VBUS= VBUS_filtrada*BASE_VOLT_CONTINUA;


    //***************************************************************************
    //***************************************************************************

    //***************************************************************************
    //PLL
    //***************************************************************************
//    PLL_grid.v_q[0] = Vq_grid;
//    SPLL_3ph_SRF_IQ_MACRO(PLL_grid);
//    theta_PLL= PLL_grid.theta[1];		//Hay que restar 30º a este valor para que en el arranque quede en fase la tension generada
											//con la medida, debido al trafo triangulo estrella (ver si esta en grados o radianes)


    //***************************************************************************
    //RAMPA PARA SET POINT TENSION (VIEJO PARA CONTROL DE VEFF)
    //***************************************************************************
    rc1.TargetValue = Veff_sp/220;
    RC_MACRO(rc1);

    //***************************************************************************
    //CONTROL PID PARA SET POINT DE Veff (VIEJO PARA CONTROL DE VEFF)
    //***************************************************************************

    // ------------------------------------------------------------------------------
    //  Connect inputs of the PID_REG3 module and call the PID controller macro
    // ------------------------------------------------------------------------------
//    pid_vout.Ref = rc1.SetpointValue*220;	// Se asigna como referencia del PID el SP de tension proveniente de la rampa
//    pid_vout.Fdb = Veff_inv;	// Se asigna como variable de feedback la tensión medida a la salida del inversor (bornes de los capacitores)
//
//    PID_MACRO(pid_vout)	// Llamado a la macro


    //***************************************************************************
    //***************************************************************************

	//***************************************************************************
    //RAMPA PARA SET POINT ID
    //***************************************************************************
//    rcid.TargetValue = 1;
//    RC_MACRO(rcid);

    //***************************************************************************
    //CONTROL PID PARA SET POINT DE ID
    //***************************************************************************

//    Id_sp = pid_vout.Out; // PARA CONTROL EN CASCADA

    // ------------------------------------------------------------------------------
    //  Connect inputs of the PID_REG3 module and call the PID controller macro
    // ------------------------------------------------------------------------------
//    pid_id.Ref = Id_sp*rcid.SetpointValue;	// Se asigna como referencia del PID el SP de corriente multiplicado por la rampa
//    pid_id.Fdb = 0;	// anulado para queno moleste la medicion
//    //pid_id.Fdb = Id_grid_filtrada;	// Se asigna como variable de feedback la componente d de la corriente de red (ver si va filtrada)
//
//    PID_MACRO(pid_id)	// Llamado a la macro

    //***************************************************************************
    //***************************************************************************

    //***************************************************************************
    //RAMPA PARA SET POINT IQ
    //***************************************************************************
    rciq.TargetValue = 1;
    RC_MACRO(rciq);

    //***************************************************************************
    //CONTROL PID PARA SET POINT DE IQ
    //***************************************************************************

    // ------------------------------------------------------------------------------
    //  Connect inputs of the PID_REG3 module and call the PID controller macro
    // ------------------------------------------------------------------------------
//    pid_iq.Ref = Iq_sp*rciq.SetpointValue;	// Se asigna como referencia del PID el SP de corriente multiplicado por la rampa
//    pid_iq.Fdb = 0;	// anulado para queno moleste la medicion
//    //pid_iq.Fdb = Iq_grid_filtrada;	// Se asigna como variable de feedback la componente q de la corriente de red (ver si va filtrada)
//
//    PID_MACRO(pid_iq)	// Llamado a la macro

    //***************************************************************************
    //***************************************************************************


    //***************************************************************************
    //RAMPA PARA SET POINT TENSION EN CASCADA
    //***************************************************************************
//    rc_vd_arranque.TargetValue = ARRANQUE_CON_RED;
//    RC_MACRO(rc_vd_arranque);


    //***************************************************************************
    //SUMATORIAS A LA SALIDA DE LOS PID PARA INGRESAR A IPARK
    //***************************************************************************
//	V_D = pid_id.Out + Iq_grid_filtrada*Comp+Vd_grid_filtrada*rc_vd_arranque.SetpointValue;
//
//	V_Q = pid_iq.Out + Id_grid_filtrada*Comp+Vq_grid_filtrada*rc_vd_arranque.SetpointValue;
	//***************************************************************************
    //***************************************************************************

    //***************************************************************************
    //OBTENSIÓN DE LA TRANSFORMACIÓN INVERSA A PARTIR DE V_D Y V_Q
    //***************************************************************************

    // ------------------------------------------------------------------------------
    //Parametrizacion de los componentes D y Q del vector de tension
    // ------------------------------------------------------------------------------
	ipark_SP_tension.Ds= _IQ(V_D);
    ipark_SP_tension.Qs= _IQ(V_Q);

    // ------------------------------------------------------------------------------
	//Asignacion del angulo, necesario para la transformacion
    // ------------------------------------------------------------------------------
	//ipark_SP_tension.Sine  = _IQsin(desfasaje+theta_PLL);
	//ipark_SP_tension.Cosine= _IQcos(desfasaje+theta_PLL);

    ipark_SP_tension.Sine  = _IQsinPU(rampa.Out); // con angulo simulado
    ipark_SP_tension.Cosine= _IQcosPU(rampa.Out); // con angulo simulado

    // ------------------------------------------------------------------------------
    //Transformacion de park inversa (DQ -> ab)
    // ------------------------------------------------------------------------------
	IPARK_MACRO(ipark_SP_tension)

    //***************************************************************************
    //GENERACIÓN DEL VECTOR ESPACIAL Y APLICACIÓN PWM
    //***************************************************************************

    // ------------------------------------------------------------------------------
	//Parametrizacion del vector en el marco ab, a partir de la transformacion
    // ------------------------------------------------------------------------------
	vectdq.Ualpha = ipark_SP_tension.Alpha;
    vectdq.Ubeta = ipark_SP_tension.Beta;

	// ------------------------------------------------------------------------------
    //Calculo de los tiempos de aplicacion de cada vector
    // ------------------------------------------------------------------------------
	SVGEN_MACRO(vectdq)

	// ------------------------------------------------------------------------------
    //Asignacion del valor de duty cycle a partir de la macro SVGEN
    // ------------------------------------------------------------------------------
	pwm1.MfuncC1=_IQtoQ15(vectdq.Ta);
    pwm1.MfuncC2=_IQtoQ15(vectdq.Tb);
    pwm1.MfuncC3=_IQtoQ15(vectdq.Tc);

	// ------------------------------------------------------------------------------
    //Calculo del valor de comparacion mediante la macro PWM
    // ------------------------------------------------------------------------------
	PWM_MACRO(pwm1)

    // ------------------------------------------------------------------------------
    //Asignacion de valor de comparacion del pwm
    // ------------------------------------------------------------------------------
	EPwm1Regs.CMPA.half.CMPA=pwm1.PWM1out;	// PWM 1A - PhaseA
    EPwm2Regs.CMPA.half.CMPA=pwm1.PWM2out;	// PWM 2A - PhaseB
    EPwm3Regs.CMPA.half.CMPA=pwm1.PWM3out;	// PWM 3A - PhaseC

    //***************************************************************************
    //***************************************************************************


    // ------------------------------------------------------------------------------
    //  Measure phase voltages, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
    //	Connect inputs of the CLARKE module and call the clarke transformation macro
    // ------------------------------------------------------------------------------



    // ------------------------------------------------------------------------------
    //  Measure phase currents, subtract the offset and normalize from (-0.5,+0.5) to (-1,+1).
    //	Connect inputs of the CLARKE module and call the clarke transformation macro
    // ------------------------------------------------------------------------------




				    if (cuenta<200) {
						//bufferU[cuenta]=clarke_tension_salida.As;
						//bufferV[cuenta]=clarke_tension_salida.Bs;
						//bufferW[cuenta]=((AdcMirror.ADCRESULT12)*0.00024414-0.5)*2.0-VbOffset;
/*PARA*/				buffer_theta[cuenta]=PLL_grid.fo; // tension fase W
						buffer_rampa[cuenta]=PLL_grid.fn;
/*GRAFICAR*/			buffer_VinvW[cuenta]=VinversorW*BASE_VOLT_ALTERNA; // tension fase W
						buffer_VinvV[cuenta]=VinversorV*BASE_VOLT_ALTERNA; // tension fase V
/*CON EL*/				buffer_VinvU[cuenta]=VinversorU*BASE_VOLT_ALTERNA; // tension fase U
/*CCS*/					buffer_VgridW[cuenta]=VgridW*BASE_VOLT_ALTERNA; // tension fase W
						buffer_VgridV[cuenta]=VgridV*BASE_VOLT_ALTERNA; // tension fase V
						buffer_VgridU[cuenta]=VgridU*BASE_VOLT_ALTERNA; // tension fase U
						//buffer_SP[cuenta]= pid_vout.Ref; // Veff set point
						//buffer_Out[cuenta]=pid_vout.Fdb; // Veff salida
						//buffer_error[cuenta]=pid_vout.Err; // error
						//Isuma=Isuma+IBUS_ok;
						cuenta=cuenta++;
					}else{
						//Imedia=Isuma/cuenta;
						//Isuma=0;
						cuenta=0;
					}


    //Salida digital que togglea en cada interrupcion

    if(GpioDataRegs.GPADAT.bit.GPIO16    == 0)
       GpioDataRegs.GPADAT.bit.GPIO16    =1;
    else
       GpioDataRegs.GPADAT.bit.GPIO16    =0;



    //IUpico= (AdcMirror.ADCRESULT11/102);
    //IVpico= (AdcMirror.ADCRESULT13/102);
    //IWpico= (AdcMirror.ADCRESULT5/102);

	// ------------------------------------------------------------------------------
    //  ESBOZO DE PROTECCION INSTANTANEA DE ID E IQ
	//	Si alguna de las magnitudes instantaneas supera el limite de umbral se abren los igbt hasta que ingrese dentro de valores
	//	normales nuevamente (con histéresis)
    // ------------------------------------------------------------------------------
    cont_ciclos_flag=1;


//		if((abs(IgridU)+abs(IcargaU) >= (Igrid_lim_max/cont_ciclos_flag)) || (abs(IgridV)+abs(IcargaV) >= (Igrid_lim_max/cont_ciclos_flag)) || (abs(IgridW)+abs(IcargaW) >= (Igrid_lim_max/cont_ciclos_flag)) )
//        {
//			I_INST_LIM_FLAG=1;
//			TZ_Protection();
//		}
//		if (abs(IgridU)+abs(IcargaU) < Igrid_lim_min/cont_ciclos_flag && abs(IgridV)+abs(IcargaV) < Igrid_lim_min/cont_ciclos_flag && abs(IgridW)+abs(IcargaW) < Igrid_lim_min/cont_ciclos_flag && I_INST_LIM_FLAG)
//		{
//			I_INST_LIM_FLAG=0;
//			//cont_ciclos_flag=1;
//			TZ_Clear();
//		}
		//while (I_INST_LIM_FLAG==1)
		//	cont_ciclos_flag=cont_ciclos_flag+1;


    // ------------------------------------------------------------------------------
    //  Protecciones GRAL
    // ------------------------------------------------------------------------------
//        if(VBUS>=310 || Veff_inv>240 || Ieff_grid>=Imax || Ieff_carga>=Imax /*  || speed1.Speed>=0.45 || clarke_tension_salida.Alpha>=1 || STOP==1 || GpioDataRegs.GPADAT.bit.GPIO6 == 1 OverTemp || GpioDataRegs.GPADAT.bit.GPIO26 == 1 OCP || GpioDataRegs.GPADAT.bit.GPIO14 == 1 DESAT */)
//        {	 //Vdcd .5 clarke_tension_salida.Alpha 0.4 DESCOMENTAR OverTemp
//            if(!STOP)
//            {
//                VDC_FLAG=VBUS>=310?1:0;
//                VEFF_FLAG=Veff_inv>=240?1:0;
//                IEFF_FLAG=Ieff_grid>=Imax?1:0;
//                IEFF_FLAG=Ieff_carga>=Imax?1:0;
//                /*CLARKE_FLAG=clarke_tension_salida.Alpha>=1?1:0;
//                OVERTEMP_FLAG=GpioDataRegs.GPADAT.bit.GPIO6==1?1:0;
//                OCP_FLAG=GpioDataRegs.GPADAT.bit.GPIO26==1?1:0;
//                DESAT_FLAG=GpioDataRegs.GPADAT.bit.GPIO14==1?1:0;
//*/
//            }


    if(STOP==1){ //Flag de parada.
        if(AcknowledgeTZ==1){
            GpioDataRegs.GPADAT.bit.GPIO8 = 1;
            TZ_Clear();
            GpioDataRegs.GPBDAT.bit.GPIO34 = 1;
            STOP = 0;
            AcknowledgeTZ = 0;
        }
    }
    /*
        //ABRE LA PRIMERA RAMA DE IGBTs
            //Load immediately (the active register is directly
            //accessed by the CPU and is not loaded from the shadow register).
            EPwm1Regs.AQSFRC.bit.RLDCSF = 3;

            //Clear (low) / Clear (low)
            EPwm1Regs.AQSFRC.bit.ACTSFA = 1;
            EPwm1Regs.AQSFRC.bit.ACTSFB = 1;

            //Initiates a single software forced event
            //Initiates a single software forced event
            EPwm1Regs.AQSFRC.bit.OTSFA = 1;
            EPwm1Regs.AQSFRC.bit.OTSFB = 1;

            //Forces a continuous low on output A
            //Forces a continuous low on output B
            EPwm1Regs.AQCSFRC.bit.CSFA = 1;
            EPwm1Regs.AQCSFRC.bit.CSFB = 1;

            //Dead-band generation is bypassed for both output signals.
            //In this mode, both the EPWMxA and EPWMxB output signals
            //from the action-qualifier are passed directly to the PWM-chopper submodule.
            EPwm1Regs.DBCTL.bit.OUT_MODE = 0;

        //ABRE LA SEGUNDA RAMA DE IGBTs
            //Load immediately (the active register is directly
            //accessed by the CPU and is not loaded from the shadow register).
            EPwm2Regs.AQSFRC.bit.RLDCSF = 3;

            //Clear (low) / Clear (low)
            EPwm2Regs.AQSFRC.bit.ACTSFA = 1;
            EPwm2Regs.AQSFRC.bit.ACTSFB = 1;

            //Initiates a single software forced event
            //Initiates a single software forced event
            EPwm2Regs.AQSFRC.bit.OTSFA = 1;
            EPwm2Regs.AQSFRC.bit.OTSFB = 1;

            //Forces a continuous low on output A
            //Forces a continuous low on output B
            EPwm2Regs.AQCSFRC.bit.CSFA = 1;
            EPwm2Regs.AQCSFRC.bit.CSFB = 1;

            //Dead-band generation is bypassed for both output signals.
            //In this mode, both the EPWMxA and EPWMxB output signals
            //from the action-qualifier are passed directly to the PWM-chopper submodule.
            EPwm2Regs.DBCTL.bit.OUT_MODE = 0;

        //ABRE LA TERCERA RAMA DE IGBTs
            //Load immediately (the active register is directly
            //accessed by the CPU and is not loaded from the shadow register).
            EPwm3Regs.AQSFRC.bit.RLDCSF = 3;

            //Clear (low) / Clear (low)
            EPwm3Regs.AQSFRC.bit.ACTSFA = 1;
            EPwm3Regs.AQSFRC.bit.ACTSFB = 1;

            //Initiates a single software forced event
            //Initiates a single software forced event
            EPwm3Regs.AQSFRC.bit.OTSFA = 1;
            EPwm3Regs.AQSFRC.bit.OTSFB = 1;

            //Forces a continuous low on output A
            //Forces a continuous low on output B
            EPwm3Regs.AQCSFRC.bit.CSFA = 1;
            EPwm3Regs.AQCSFRC.bit.CSFB = 1;

            //Dead-band generation is bypassed for both output signals.
            //In this mode, both the EPWMxA and EPWMxB output signals
            //from the action-qualifier are passed directly to the PWM-chopper submodule.
            EPwm3Regs.DBCTL.bit.OUT_MODE = 0;
    */

        //}
	//}
    //Limpiar flag de evento de interrupcion
    EPwm1Regs.ETCLR.bit.INT = 1;

    //Reconocimiento de la interrupcion del grupo 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

interrupt void ProteccionExtCorriente(void)
{
	TZ_Protection();
	STOP = 0;
}

//Proteccion por TRIPZONE
void TZ_Protection(void)
{
//Requiero permiso para modificar estos registros
EALLOW;
//Fuerzo falla por software en los 3 PWM
	EPwm1Regs.TZFRC.bit.OST=1;
	EPwm2Regs.TZFRC.bit.OST=1;
	EPwm3Regs.TZFRC.bit.OST=1;
EDIS;
}


//Funcion que resetea la interrupcion del TripZone y permite al equipo continuar trabajando
void TZ_Clear(void)
{
//Requiero permiso para modificar estos registros
EALLOW;
//Fuerzo reset y deshabilito la proteccion
	EPwm1Regs.TZCLR.bit.OST = 1;
	EPwm2Regs.TZCLR.bit.OST = 1;
	EPwm3Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EPwm2Regs.TZCLR.bit.INT = 1;
    EPwm3Regs.TZCLR.bit.INT = 1;
EDIS;
}
__interrupt void epwm1_tzint_isr(void)
{
   GpioDataRegs.GPBDAT.bit.GPIO34 = 0;
   GpioDataRegs.GPADAT.bit.GPIO8 = 0;
   EPwm1TZIntCount++;
   STOP=1;
   // Acknowledge this interrupt to receive more interrupts from group 2
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}
//===========================================================================
// No more.
//===========================================================================



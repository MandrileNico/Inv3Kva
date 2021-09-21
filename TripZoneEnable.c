/*
 * TripZoneEnable.c
 *
 *  Created on: 2 ago. 2021
 *      Author: nico_
 */
#include "TripZoneEnable.h"

void TripZoneEnable() {
    EALLOW;
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm3Regs.TZSEL.bit.OSHT1 = 1;
    /*What do we want the TZ1 and TZ2 to do?*/
    EPwm1Regs.TZCTL.bit.TZA = 0x2;
    EPwm1Regs.TZCTL.bit.TZB = 0x2;
    EPwm2Regs.TZCTL.bit.TZA = 0x2;
    EPwm2Regs.TZCTL.bit.TZB = 0x2;
    EPwm3Regs.TZCTL.bit.TZA = 0x2;
    EPwm3Regs.TZCTL.bit.TZB = 0x2;

    /*Enable TZ interrupt*/
    EPwm1Regs.TZEINT.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;
    EDIS;
}





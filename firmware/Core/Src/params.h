/*
 * params.h
 *
 *  Created on: Oct 6, 2025
 *      Author: rodriguedeguerre
 */

#ifndef SRC_PARAMS_H_
#define SRC_PARAMS_H_
#include "stm32f7xx_hal.h"
#include "arm_math.h"
//#include "main.h"
//
//#ifndef ARM_MATH_CM7
//	#define ARM_MATH_CM7
//#endif
//
//#ifndef ARM_MATH_H
//	#include "arm_math.h"
//#endif

typedef struct {
    /* Physics */
    float c0;
    float rho0;

    /* Sensor and amplifier parameters */
    float sens_p;
    float i2u;

    /* Control speaker parameters */
    float Sd;
    float Bl;
    float Rms;
    float Mms;
    float Cmc;
    float f0;

    /* Target impedance parameters */
    float muM;
    float muR;
    float muC;
    float fst;

    /* Transfer function coefficients */
    float a2, a1, a0;
    float b2, b1, b0;

    float bz[3];
    float az[3];

    float fst_global;
} Params;

void init_params(Params *p);

#endif /* SRC_PARAMS_H_ */

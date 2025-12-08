/*
 * params.c
 *
 *  Created on: Oct 6, 2025
 *      Author: rodriguedeguerre
 */
#include "params.h"
#include <string.h>
#include "generated_params.h"

void init_params(Params *p) {
    /* PHYSICS */
    p->c0 = 347.13f;
    p->rho0 = 1.1839f;

    /* CONTROL SENSITIVITY */
    p->sens_p = -1.0f / SENS_P; //37.1e-3f; //39
    p->i2u = I2U;  // 100
    
    p->Sd = SD;
    p->Bl = BL;

    p->bz[0] = B0;
    p->bz[1] = B1;
    p->bz[2] = B2;

    p->az[0] = 1.000000000000e+00;
    p->az[1] = A1;
    p->az[2] = A2;
}
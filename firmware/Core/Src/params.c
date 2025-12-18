/*
 * params.c
 *
 *  Created on: Oct 6, 2025
 *      Author: rodriguedeguerre
 */
#include "params.h"
#include <string.h>
#include "generated_params.h"

void init_params(Params *p) 
{
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
    p->az[1] = -A1;
    p->az[2] = -A2;
}

void init_params_array(Params *array, size_t length, InitParamsFun fun) 
{
    for (size_t i = 0; i < length; i++) 
    {
        Params *p = &array[i];
        p->c0   = 347.13f;
        p->rho0 = 1.1839f;
        p->Sd   = SD;
        p->Bl   = BL;
        p->sens_p = -1.0f / SENS_P;

        if (fun != NULL) {
            fun(p, (int)i);
        }
    }
}

void init_params_single(Params *p, int idx)
{
    p->sens_p = -1.0f / SENS_P;
    p->i2u = I2U;
    p->bz[0] = 0; p->bz[1] = 0; p->bz[2] = 0;
    p->az[0] = 1; p->az[1] = 0; p->az[2] = 0;
}

void init_params_m3(Params *p, int idx)
{
    switch (idx) {
    case 0:
        p->i2u = I2U_1;
        p->bz[0] = coeffs_m1[0]; 
        p->bz[1] = coeffs_m1[1]; 
        p->bz[2] = coeffs_m1[2];
        p->az[0] = 1; 
        p->az[1] = coeffs_m1[3]; 
        p->az[2] = coeffs_m1[4];
        break;
    case 1: 
        p->i2u = I2U_2;
        p->bz[0] = coeffs_m2[0]; 
        p->bz[1] = coeffs_m2[1]; 
        p->bz[2] = coeffs_m2[2];
        p->az[0] = 1; 
        p->az[1] = coeffs_m2[3]; 
        p->az[2] = coeffs_m2[4];
        break;
    case 2:
        p->i2u = I2U_3;
        p->bz[0] = coeffs_m3[0]; 
        p->bz[1] = coeffs_m3[1]; 
        p->bz[2] = coeffs_m3[2];
        p->az[0] = 1; 
        p->az[1] = coeffs_m3[3]; 
        p->az[2] = coeffs_m3[4];
        break;
    default:
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    }
}

void init_params_m6(Params *p, int idx)
{
    switch (idx) {
    case 0:
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    case 1: 
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    case 2:
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    case 3:
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    case 4: 
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    case 5:
        p->sens_p = -1.0f / SENS_P;
        p->i2u = I2U;
        p->bz[0] = B0; p->bz[1] = B1; p->bz[2] = B2;
        p->az[0] = 1; p->az[1] = A1; p->az[2] = A2;
        break;
    default:
        break;
    }
}

// Function to initialize the coefficient arrays
// Note: We pass float* (pointers to the arrays)
void init_coefs(float* a, float* b, float* c)
{
    // CMSIS Biquad Order: {b0, b1, b2, a1, a2}
    a[0] = coeffs_m1[0];
    a[1] = coeffs_m1[1];
    a[2] = coeffs_m1[2];
    a[3] = coeffs_m1[3];  
    a[4] = coeffs_m1[4];

    b[0] = coeffs_m2[0]; 
    b[1] = coeffs_m2[1]; 
    b[2] = coeffs_m2[2];
    b[3] = coeffs_m2[3];
    b[4] = coeffs_m2[4];

    c[0] = coeffs_m3[0];
    c[1] = coeffs_m3[1];
    c[2] = coeffs_m3[2];
    c[3] = coeffs_m3[3];
    c[4] = coeffs_m3[4];
}
/*
 * params.c
 *
 *  Created on: Oct 6, 2025
 *      Author: rodriguedeguerre
 */


#include "params.h"
#include <string.h>

void init_params(Params *p) {
    /* PHYSICS */
    p->c0 = 347.13f;
    p->rho0 = 1.1839f;

    /* CONTROL SENSITIVITY */
    p->sens_p = -1.0f / 37.1e-3f; //39

    p->i2u = 100.0f;

    /* CONTROL SPEAKER PARAMETERS */
    p->Sd = 23.5e-4f;

//    double Re  = 7.20;
//    float Bl_val  = 2.369823e+00;
//    float Rms_val = 3.716888e-01;
//    float Mms_val = 1.255558e-03;
//    float Cmc_val = 1.650878e-04;

//    Bl_val = 1.939940e+00;
//    Rms_val = 5.186787e-01;
//    Mms_val = 1.117079e-03;
//    Cmc_val = 1.163066e-04;

//    Bl_val = 2e+00;
//    Rms_val = 5.186787e-01;
//    Mms_val = 1e-03;
//    Cmc_val = 1e-04;
//
//    p->Bl = Bl_val;
//    p->Rms = Rms_val;
//    p->Mms = Mms_val;
//    p->Cmc = Cmc_val;

    // Updated params ok
    p->Bl = 1.806225000000000;
    p->Rms = 0.742623500000000;
    p->Mms = 0.001329715000000;
    p->Cmc = 1.339291000000000e-04;


    // p->f0 =  3.495782e+02;
    p->f0 = 4.415461e+02;

    // Q = 5.975047e+00;
//    float Q = 7.419616e+00;

    /* TARGET IMPEDANCE PARAMETERS */
    float f_tgt = 290.0f;
//    p->muM = 1.0f;
//    p->muR = p->rho0 * p->c0 * p->Sd / 20;
//    p->muC = 1/p->muM * powf(2.0f * (float)M_PI * f_tgt / 2.0f * (float)M_PI * p->f0, 2.0f); // * p->Mms * p->Cmc;
//    p->fst = p->f0 * sqrtf(p->muC / p->muM);

    // Updated values
//    p->muR = 0.065024399069650;
//    p->muC = 0.340281717893114;

    // Updated Formulas
    p->muM = 1.0f;
    p->muR = 1/20 * p->rho0 * p->c0 * p->Sd / p->Rms;
    p->muC = powf(2.0f * (float)M_PI * f_tgt, 2.0f) * p->Mms * p->Cmc;


//    p->b2 = p->Sd * (p->muM - 1.0f) * p->Mms;
//    p->b1 = p->Sd * (p->muR - 1.0f) * p->Rms;
//    p->b0 = p->Sd * (p->muC - 1.0f) / p->Cmc;
//    p->a2 = p->Bl * p->muM * p->Mms;
//    p->a1 = p->Bl * p->muR * p->Rms;
//    p->a0 = p->Bl * p->muC / p->Cmc;

    // Updated formulas ok
    p->b2 = p->muM * p->Mms * p->Cmc;
    p->b1 = p->muR * p->Rms * p->Cmc;
    p->b0 = p->muC;
    p->a2 = (p->muM - 1.0f) * p->Mms * p->Cmc;
    p->a1 = (p->muR - 1.0f) * p->Rms * p->Cmc;
    p->a0 = (p->muC - 1.0f);

//    float Ts = 40e-06;

//    bilinear_discretize(
//    		p->a2, p->a1, p->a0,
//			p->b2, p->b1, p->b0,
//			Ts,
//			p->bz, p->az
//    );

    p->bz[0] = -1.549219320629e-05;
    p->bz[1] = -3.850030080815e-06;
    p->bz[2] = 1.164216312548e-05;

    p->az[0] = 1.000000000000e+00;
    p->az[1] = -1.995496902269e+00;
    p->az[2] = 9.985495606274e-01;

    p->fst_global = p->f0 * sqrtf(p->muC / p->muM);
}



// Discretize 2nd-order TF with Tustin (bilinear transform)
void bilinear_discretize(
        float a2, float a1, float a0,
        float b2, float b1, float b0,
        float Ts,
        float *B,   // B[0]=b0z, B[1]=b1z, B[2]=b2z
        float *A)   // A[0]=a1z, A[1]=a2z  (note: A[0] = a1, A[1] = a2)
{
    // Precompute substitution constant
    float K = 2.0f / Ts;

    // Build numerator and denominator polynomials in z (before normalization)
    float A0 =  a2*K*K + a1*K + a0;
    float A1 = -2*a2*K*K + 0      - 2*a0;
    float A2 =  a2*K*K - a1*K + a0;

    float B0 =  b2*K*K + b1*K + b0;
    float B1 = -2*b2*K*K + 0      - 2*b0;
    float B2 =  b2*K*K - b1*K + b0;

    // Normalize so denom leading coefficient = 1
    float invA0 = 1.0f / A0;

    B[0] = B0 * invA0;
    B[1] = B1 * invA0;
    B[2] = B2 * invA0;

    A[0] = A1 * invA0;   // "a1" term in CMSIS
    A[1] = A2 * invA0;   // "a2" term in CMSIS
}


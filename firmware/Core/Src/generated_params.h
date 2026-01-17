/* 
 * This file is auto-generated. DO NOT EDIT BY HAND.
 * Any manual changes will be overwritten.
 */
#ifndef GENERATED_PARAMS_H
#define GENERATED_PARAMS_H

#define VERSION_TYPE 3

#define ARR 10799

#define SENS_P 3.710000000000e-02f
#define SD 2.400000000000e-03f
#define BL 1.865600000000e+00f

// Current to Voltage Gains
#define I2U 1.000000000000e+02f
#define I2U_1 1.000000000000e+02f
#define I2U_2 1.000000000000e+02f
#define I2U_3 1.000000000000e+02f

// Mode 1 Macros (Legacy)
#define B0 -3.118062580219e-03f
#define B1 5.777957096885e-03f
#define B2 -2.893029328615e-03f
#define A1 1.954338268883e+00f
#define A2 -9.730991998799e-01f

// Coefficient Arrays
static float coeffs_m1[5] = { -3.118062580219e-03f, 5.777957096885e-03f, -2.893029328615e-03f, 1.954338268883e+00f, -9.730991998799e-01f };
static float coeffs_m2[5] = { -3.118062580219e-03f, 5.777957096885e-03f, -2.893029328615e-03f, 1.954338268883e+00f, -9.730991998799e-01f };
static float coeffs_m3[5] = { -3.118062580219e-03f, 5.777957096885e-03f, -2.893029328615e-03f, 1.954338268883e+00f, -9.730991998799e-01f };

static float* coeff_lut[4] = { NULL, coeffs_m1, coeffs_m2, coeffs_m3 };

static float i2u_lut[4] = { 0, I2U_1, I2U_2, I2U_3 };

#endif

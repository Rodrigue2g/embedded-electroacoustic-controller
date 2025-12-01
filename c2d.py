import numpy as np
import control


class Params:
    pass

def compute_filter_coeffs(params):
    """Computes discrete filter coefficients from physical parameters."""

    # Continuous numerator
    num_c = [
        params.Sd/params.Bl * params.a2,
        params.Sd/params.Bl * params.a1,
        params.Sd/params.Bl * params.a0
    ]
    # Continuous denominator
    den_c = [params.b2, params.b1, params.b0]

    Phi_c = control.tf(num_c, den_c)
    Phi_d = control.c2d(Phi_c, params.ts_ctr, method='tustin')

    bz = Phi_d.num[0][0]
    az = Phi_d.den[0][0]

    return bz, az   # Python lists


def initParams():
    params = Params()

    # ===== PHYSICS =====
    params.c0   = 347.13
    params.rho0 = 1.1839

    # ===== CONTROL SENSITIVITY =====
    params.sens_p = -1.0 / 37.1e-3
    params.i2u    = 100.0

    # ===== SPEAKER PARAMETERS =====
    params.Sd = 23.5e-4
    params.Bl = 1.806225000000000

    params.Rms = 0.742623500000000
    params.Mms = 0.001329715000000
    params.Cmc = 1.339291000000000e-04

    params.f0 = 441.5461  # not used for the TF directly

    # ===== TARGET parameters =====
    f_tgt = 220

    params.muM = 0.5
    params.muR = (1/10) * params.rho0 * params.c0 * params.Sd / params.Rms
    params.muC = (2*np.pi*f_tgt)**2 * params.Mms * params.Cmc

    # ===== Continuous TF coefficients =====
    params.b2 = params.muM * params.Mms * params.Cmc
    params.b1 = params.muR * params.Rms * params.Cmc
    params.b0 = params.muC

    params.a2 = (params.muM - 1.0) * params.Mms * params.Cmc
    params.a1 = (params.muR - 1.0) * params.Rms * params.Cmc
    params.a0 = (params.muC - 1.0)

    # ===== Sampling =====
    params.ts_ctr = 40e-6
    params.fs_ctr = 1 / params.ts_ctr

    return params


def test():
    params = initParams()

    # continuous-time transfer function
    num_c = [
        params.Sd/params.Bl * params.a2,
        params.Sd/params.Bl * params.a1,
        params.Sd/params.Bl * params.a0
    ]

    den_c = [params.b2, params.b1, params.b0]

    Phi_c = control.tf(num_c, den_c)

    # Discretization (Tustin)
    Phi_d = control.c2d(Phi_c, params.ts_ctr, method='tustin')

    bz = Phi_d.num[0][0]
    az = Phi_d.den[0][0]

    # CMSIS DF2T-ready coefficients
    print("\n/* === CMSIS biquad_coeffs[] === */\n")

    print(f"biquad_coeffs[0] = {bz[0]:.12e};  // b0")
    print(f"biquad_coeffs[1] = {bz[1]:.12e};  // b1")
    print(f"biquad_coeffs[2] = {bz[2]:.12e};  // b2")
    print(f"biquad_coeffs[3] = {-az[1]:.12e}; // -a1")
    print(f"biquad_coeffs[4] = {-az[2]:.12e}; // -a2")


if __name__ == "__main__":
    test()
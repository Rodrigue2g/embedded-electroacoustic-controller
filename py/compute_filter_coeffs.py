import numpy as np
import control


class Params:
    pass

def compute_filter_coeffs(params):
    scaling = params.Sd / params.Bl
    num_c = [
        scaling * params.a0,
        scaling * params.a1,
        scaling * params.a2
    ]
    den_c = [params.b0, params.b1, params.b2]
    Phi_c = control.tf(num_c, den_c)
    Phi_d = control.c2d(Phi_c, params.ts_ctr, method='tustin')
    bz = list(Phi_d.num[0][0])
    az = list(Phi_d.den[0][0])

    return bz, az
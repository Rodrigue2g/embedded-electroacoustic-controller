import numpy as np
from py.c2d import Params

def get_physics_params(values):
    p = Params()

    p.c0 = 347.13
    p.rho0 = 1.1839
    p.sens_p = values["sens_p"]
    p.i2u = values["i2u"]
    p.Sd = values["Sd"]
    p.Bl = values["Bl"]
    p.Rms = values["Rms"]
    p.Mms = values["Mms"]
    p.Cmc = values["Cmc"]
    p.ts_ctr = 100e-6 #40e-6 #50e-6

    p.f_tgt = values["f_tgt"]
    
    p.muM = values["muM"]
    p.muR = values["muR_factor"] * p.rho0 * p.c0 * p.Sd / p.Rms
    p.muC = (2*np.pi * p.f_tgt)**2 * p.Mms * p.Cmc

    # Denominator
    p.b2 = p.muM * p.Mms * p.Cmc
    p.b1 = p.muR * p.Rms * p.Cmc
    p.b0 = p.muC

    # Numerator
    p.a2 = (p.muM - 1.0) * p.Mms * p.Cmc
    p.a1 = (p.muR - 1.0) * p.Rms * p.Cmc
    p.a0 = (p.muC - 1.0)

    # scaling_factor = p.Sd / p.Bl
    # p.a2 = a2_raw * scaling_factor
    # p.a1 = a1_raw * scaling_factor
    # p.a0 = 1 # a0_raw * scaling_factor

    return p
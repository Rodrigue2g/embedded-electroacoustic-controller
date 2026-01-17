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

    # DEPRECATED
    # We now use a global user var (set in the GUI)
    # Look for `global_fs` instead
    p.ts_ctr = None #100e-6 #40e-6 #50e-6

    p.f_tgt = values["f_tgt"]
    
    p.muM = values["muM"]
    p.muR = values["muR_factor"] * p.rho0 * p.c0 * p.Sd / p.Rms
    p.muC = p.muM * ((2* np.pi * p.f_tgt)**2 *p.Mms * p.Cmc)

    # Denominator
    p.b0 = p.muM * p.Mms * p.Cmc
    p.b1 = p.muR * p.Rms * p.Cmc
    p.b2 = p.muC

    # Numerator
    p.a0 = (p.muM - 1.0) * p.Mms * p.Cmc
    p.a1 = (p.muR - 1.0) * p.Rms * p.Cmc
    p.a2 = (p.muC - 1.0)

    return p

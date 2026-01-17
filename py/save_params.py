# logic_save.py
import os
import numpy as np
from py.compute_filter_coeffs import compute_filter_coeffs
from py.physics import get_physics_params


def compute_coeffs_for_mode(ui_widget, fs):
    """Calculates coefficients for a single UI widget instance."""
    values = ui_widget.get_values()  

    p = get_physics_params(values)
    bz, az = compute_filter_coeffs(p, fs)

    # Normalization
    # if az[0] != 0:
    #     bz = bz / az[0]

    # CMSIS Format: {b0, b1, b2, -a1, -a2}
    return [bz[0], bz[1], bz[2], -az[1], -az[2]], p


def save_all_params(gui):
    """
    Generates the C header file.
    'gui' is the main window instance containing the UI tabs and path.
    """
    if not gui.project_path:
        gui.log.append("Error: No project path selected.")
        return
    
    try:
        raw_fs = gui.global_fs.currentText()
        global_fs = float(raw_fs)
    except (ValueError, AttributeError) as e:
        gui.log.append(f"❌ Error reading Sampling Frequency: {e}")
        return
    
    try:
        fs = global_fs
        # Access the widgets stored in the gui instance
        coefs1, p1 = compute_coeffs_for_mode(gui.mode1_ui, fs)
        coefs2, p2 = compute_coeffs_for_mode(gui.mode2_ui, fs)
        coefs3, p3 = compute_coeffs_for_mode(gui.mode3_ui, fs)

        ARR = int((108000000 / fs) - 1)

        content = f"""/* * This file is auto-generated. DO NOT EDIT BY HAND.
 * Any manual changes will be overwritten.
 */
#ifndef GENERATED_PARAMS_H
#define GENERATED_PARAMS_H

#define ARR {ARR}

#define SENS_P {p1.sens_p:.12e}f
#define SD {p1.Sd:.12e}f
#define BL {p1.Bl:.12e}f

// Current to Voltage Gains
#define I2U {p1.i2u:.12e}f
#define I2U_1 {p1.i2u:.12e}f
#define I2U_2 {p2.i2u:.12e}f
#define I2U_3 {p3.i2u:.12e}f

// Mode 1 Macros (Legacy)
#define B0 {coefs1[0]:.12e}f
#define B1 {coefs1[1]:.12e}f
#define B2 {coefs1[2]:.12e}f
#define A1 {coefs1[3]:.12e}f
#define A2 {coefs1[4]:.12e}f

// Coefficient Arrays
static float coeffs_m1[5] = {{ {coefs1[0]:.12e}f, {coefs1[1]:.12e}f, {coefs1[2]:.12e}f, {coefs1[3]:.12e}f, {coefs1[4]:.12e}f }};
static float coeffs_m2[5] = {{ {coefs2[0]:.12e}f, {coefs2[1]:.12e}f, {coefs2[2]:.12e}f, {coefs2[3]:.12e}f, {coefs2[4]:.12e}f }};
static float coeffs_m3[5] = {{ {coefs3[0]:.12e}f, {coefs3[1]:.12e}f, {coefs3[2]:.12e}f, {coefs3[3]:.12e}f, {coefs3[4]:.12e}f }};

#endif
"""
        header_path = os.path.join(gui.project_path, "Core", "Src", "generated_params.h")
        core_src = os.path.dirname(header_path)
        
        if not os.path.isdir(core_src):
            gui.log.append(f"[save_params] ERROR: directory does not exist:\n    {core_src}\n")
            return
        
        gui.log.append(f"Overwriting generated_params.h with:\n\n {content}\n\n")

        with open(header_path, "w") as f:
            f.write(content)
        
        gui.log.append(f"✔ Successfully saved parameters to:\n{header_path}\n")

    except Exception as e:
        gui.log.append(f"❌ Error saving parameters: {e}\n")
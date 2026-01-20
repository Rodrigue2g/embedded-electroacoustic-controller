##
# Defaults value that will be set if the user performs 
# a `factory reset`
##
FACTORY_DEFAULTS = {
    # Line input
    "sens_p": "37.1e-3",
    "Sd": "24e-4",
    "Bl": "1.8656",
    "Rms": "0.74",
    "Mms": "0.0012",
    "Cmc": "1.3638e-4",
    # Sliders
    "i2u": 100.0,
    "f_tgt": 220.0,
    "muM": 0.30,
    "muR_factor": 0.10
}

##
# Default Sampling frequency (fs)
# The value should exist in this array
# ["2000", "5000", "10000", "20000", "25000", "40000", "50000"]
##
DEFAULT_FS = "10000"

def perform_factory_reset(gui):
    """
    Logic to clear settings and reset the GUI instance.
    'gui' refers to your main GUI class instance.
    """
    gui.settings.clear()

    gui.global_fs.setCurrentText(DEFAULT_FS)

    all_modes = [
        gui.mode1_ui, gui.mode2_ui, gui.mode3_ui, 
        gui.mode4_ui, gui.mode5_ui, gui.mode6_ui
    ]
    
    for mode_ui in all_modes:
        mode_ui.set_values(FACTORY_DEFAULTS)

    gui.log.append("🔄 UI Reset: All parameters restored to factory hardcoded defaults.")

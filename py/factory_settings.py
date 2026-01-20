##
# Defaults value that will be set if the user performs 
# a `factory reset`
##
FACTORY_DEFAULTS = {
    "sens_p": "37.1e-3",
    "i2u": 100.0,
    "Sd": "23.5e-4",
    "Bl": "1.806225",
    "Rms": "0.5186787",
    "Mms": "0.001",
    "Cmc": "0.0001",
    "f_tgt": 200.0,
    "muM": 1.0,
    "muR_factor": 0.05
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

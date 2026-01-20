from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLineEdit, QGroupBox, QFormLayout, 
    QSlider, QDoubleSpinBox, QPushButton
)
from PySide6.QtCore import Qt, Signal
from py.factory_settings import FACTORY_DEFAULTS

class ParamsWidget(QWidget):
    # Define a signal that sends a dictionary of values when triggered
    request_plot = Signal(dict) 

    def __init__(self, title="Parameters"):
        super().__init__()
        
        self.layout = QFormLayout()
        self.group_box = QGroupBox(title)
        self.group_box.setLayout(self.layout)
        
        # --- INPUTS ---
        self.sens_p_input = QLineEdit(str(FACTORY_DEFAULTS.get("sens_p")))
        self.Sd_input     = QLineEdit(str(FACTORY_DEFAULTS.get("Sd")))
        self.Bl_input     = QLineEdit(str(FACTORY_DEFAULTS.get("Bl")))
        self.Rms_input    = QLineEdit(str(FACTORY_DEFAULTS.get("Rms")))
        self.Mms_input    = QLineEdit(str(FACTORY_DEFAULTS.get("Mms")))
        self.Cmc_input    = QLineEdit(str(FACTORY_DEFAULTS.get("Cmc")))

        self.layout.addRow("sens_p:", self.sens_p_input)
        
        # i2u
        i2u_default = FACTORY_DEFAULTS.get("i2u")
        self.i2u_input, self.i2u_slider = self.create_slider_input(0, 2500, i2u_default)
        self.add_slider_row("i2u:", self.i2u_input, self.i2u_slider)

        self.layout.addRow("Sd (m²):",  self.Sd_input)
        self.layout.addRow("Bl (Tm):",  self.Bl_input)
        self.layout.addRow("Rms (N·s/m):", self.Rms_input)
        self.layout.addRow("Mms (kg):", self.Mms_input)
        self.layout.addRow("Cmc (m/N):", self.Cmc_input)

        # f_target
        f_tgt_default = FACTORY_DEFAULTS.get("f_tgt")
        self.f_tgt_input, self.f_tgt_slider = self.create_slider_input(0, 1000, f_tgt_default)
        self.add_slider_row("f_target (Hz):", self.f_tgt_input, self.f_tgt_slider)

        # muM
        muM_default = FACTORY_DEFAULTS.get("muM")
        self.muM_input, self.muM_slider = self.create_slider_input(0.2, 2.0, muM_default, scale=1000)
        self.add_slider_row("muM:", self.muM_input, self.muM_slider)

        # muR
        muR_default = FACTORY_DEFAULTS.get("muR_factor")
        self.muR_input, self.muR_slider = self.create_slider_input(0.04, 1.0, muR_default, scale=1000)
        self.add_slider_row("muR factor:", self.muR_input, self.muR_slider)

        # --- PLOT BUTTON ---
        self.btn_plot = QPushButton("📈 Plot Frequency Response")
        self.btn_plot.clicked.connect(self.on_plot_clicked)
        self.layout.addRow(self.btn_plot)

        # Final layout
        main_layout = QHBoxLayout()
        main_layout.addWidget(self.group_box)
        main_layout.setContentsMargins(0,0,0,0)
        self.setLayout(main_layout)

    def create_slider_input(self, min_val, max_val, default_val, scale=1):
        spinbox = QDoubleSpinBox()
        spinbox.setRange(min_val, max_val)
        spinbox.setValue(default_val)
        spinbox.setSingleStep((max_val - min_val) / 100.0)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(int(min_val * scale), int(max_val * scale))
        slider.setValue(int(default_val * scale))

        if scale == 1:
            slider.valueChanged.connect(spinbox.setValue)
            spinbox.valueChanged.connect(lambda v: slider.setValue(int(v)))
        else:
            slider.valueChanged.connect(lambda v: spinbox.setValue(v / scale))
            spinbox.valueChanged.connect(lambda v: slider.setValue(int(v * scale)))

        return spinbox, slider

    def add_slider_row(self, label, spinbox, slider):
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(slider)
        layout.addWidget(spinbox)
        self.layout.addRow(label, container)

    def on_plot_clicked(self):
        """Collects values and emits signal to parent"""
        try:
            values = self.get_values()
            self.request_plot.emit(values)
        except ValueError:
            print("Error parsing values for plot")

    def get_values(self):
        return {
            "sens_p": float(self.sens_p_input.text()),
            "i2u": self.i2u_input.value(),
            "Sd": float(self.Sd_input.text()),
            "Bl": float(self.Bl_input.text()),
            "Rms": float(self.Rms_input.text()),
            "Mms": float(self.Mms_input.text()),
            "Cmc": float(self.Cmc_input.text()),
            "f_tgt": self.f_tgt_input.value(),
            "muM": self.muM_input.value(),
            "muR_factor": self.muR_input.value()
        }
    
    def set_values(self, data):
        """Updates the UI widgets from a dictionary retrieved from QSettings"""
        if not data or not isinstance(data, dict):
            return

        def set_text(widget, key):
            if key in data: widget.setText(str(data[key]))

        def set_val(widget, key):
            if key in data: widget.setValue(float(data[key]))

        # Update QLineEdits
        set_text(self.sens_p_input, "sens_p")
        set_text(self.Sd_input, "Sd")
        set_text(self.Bl_input, "Bl")
        set_text(self.Rms_input, "Rms")
        set_text(self.Mms_input, "Mms")
        set_text(self.Cmc_input, "Cmc")

        # Update SpinBoxes (Sliders will update automatically via signals)
        set_val(self.i2u_input, "i2u")
        set_val(self.f_tgt_input, "f_tgt")
        set_val(self.muM_input, "muM")
        set_val(self.muR_input, "muR_factor")
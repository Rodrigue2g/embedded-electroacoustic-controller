import os
import sys, subprocess
import platform
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton,
    QVBoxLayout, QTextEdit, QFileDialog, QLabel
)
from PySide6.QtWidgets import QHBoxLayout, QLineEdit, QGroupBox, QFormLayout
from c2d import Params, compute_filter_coeffs
import numpy as np
from toolchain import ToolchainManager
from fractions import Fraction


class BuilderGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Embedded Builder")

        self.layout = QVBoxLayout()

        # === Top Controls ===
        self.btn_select = QPushButton("Select Project Folder")
        self.btn_build = QPushButton("Build")
        self.btn_flash = QPushButton("Flash")
        self.status_label = QLabel("No project selected.")

        self.btn_select.clicked.connect(self.select_folder)
        self.btn_build.clicked.connect(self.build_project)
        self.btn_flash.clicked.connect(self.flash_project)

        self.layout.addWidget(self.btn_select)
        self.layout.addWidget(self.btn_build)
        self.layout.addWidget(self.btn_flash)
        self.layout.addWidget(self.status_label)

        # === Speaker + Control + Target Parameter UI ===
        params_group = QGroupBox("Physical Parameters")
        form = QFormLayout()

        # CONTROL SENSITIVITY
        self.sens_p_input = QLineEdit(str(-1.0 / 37.1e-3))
        self.i2u_input    = QLineEdit("100.0")

        # SPEAKER PARAMETERS
        self.Sd_input  = QLineEdit("23.5e-4")
        self.Bl_input  = QLineEdit("1.806225")
        self.Rms_input = QLineEdit("0.7426235")
        self.Mms_input = QLineEdit("0.001329715")
        self.Cmc_input = QLineEdit("1.339291e-4")

        self.f0_input  = QLineEdit("441.5461")
        self.f_tgt_input = QLineEdit("220")

        self.muM_input = QLineEdit("0.5")
        self.muR_factor_input = QLineEdit("1/10")


        form.addRow("sens_p:", self.sens_p_input)
        form.addRow("i2u:",    self.i2u_input)

        form.addRow("Sd (m²):",  self.Sd_input)
        form.addRow("Bl (Tm):",  self.Bl_input)
        form.addRow("Rms (N·s/m):", self.Rms_input)
        form.addRow("Mms (kg):", self.Mms_input)
        form.addRow("Cmc (m/N):", self.Cmc_input)

        form.addRow("f0 (Hz):",   self.f0_input)
        form.addRow("f_target (Hz):", self.f_tgt_input)

        form.addRow("muM:", self.muM_input)
        form.addRow("muR factor:", self.muR_factor_input)

        params_group.setLayout(form)

        self.btn_save_params = QPushButton("Save Parameters")
        self.btn_save_params.clicked.connect(self.save_params)

        self.layout.addWidget(params_group)
        self.layout.addWidget(self.btn_save_params)

        # === Log Window ===
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.layout.addWidget(self.log)

        # Set final layout
        self.setLayout(self.layout)

        # Initialize project path
        self.project_path = None
        self.project_path = self.get_project_root()
        self.status_label.setText(f"Selected: {self.project_path}")

        print("sys.frozen =", getattr(sys, 'frozen', None))
        print("sys._MEIPASS =", getattr(sys, '_MEIPASS', None))
        print("sys.executable =", sys.executable)
        self.log.append(f"sys.frozen = {getattr(sys, 'frozen', None)}\n")
        self.log.append(f"sys._MEIPASS = {getattr(sys, '_MEIPASS', None)}\n")
        self.log.append(f"sys.executable = {sys.executable}\n")

        try:
            app_root = self.get_app_root()
            print("app_root=", app_root)
            self.log.append(f"app_root= {app_root}\n")
            self.toolchain = ToolchainManager(app_root)
            self.toolchain.load()
            self.log.append("✔ Toolchain loaded.\n")
        except Exception as e:
            self.log.append(f"Toolchain error: {e}\n")


    def get_app_root(self):
        if getattr(sys, 'frozen', False):
            # Use Contents/Resources as root where data is stored
            return os.path.join(os.path.dirname(sys.executable), "..", "Resources")
        return os.path.dirname(os.path.abspath(__file__))

    def get_project_root(self):
        if getattr(sys, 'frozen', False):
            # Project is bundled in Resources
            return os.path.join(
                os.path.dirname(sys.executable),
                "..",   # Contents
                "Resources",
                "Accoustic-Controller"
            )
        else:
            # Development mode: use local folder
            return os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")

    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            self.project_path = folder
            self.status_label.setText(f"Selected: {folder}")


    def save_params(self):
        if not self.project_path:
            self.log.append("Select a project folder first.")
            return

        # --- Create params object ---
        params = Params()

        # CONSTANTS
        params.c0 = 347.13
        params.rho0 = 1.1839

        # USER INPUTS (GUI)
        params.sens_p = float(self.sens_p_input.text())
        params.i2u    = float(self.i2u_input.text())

        params.Sd  = float(self.Sd_input.text())
        params.Bl  = float(self.Bl_input.text())
        params.Rms = float(self.Rms_input.text())
        params.Mms = float(self.Mms_input.text())
        params.Cmc = float(self.Cmc_input.text())

        params.f0 = float(self.f0_input.text())
        f_tgt     = float(self.f_tgt_input.text())

        # --- Derived physics ---
        params.muM = float(self.muM_input.text())
        try:
            muR_factor = float(Fraction(self.muR_factor_input.text()))
        except ValueError as e:
            self.log.append(f"Fraction error: {e}\n")
            muR_factor = 1/20
            self.log.append(f"Will use default value: {muR_factor}\n")

        params.muR = muR_factor * params.rho0 * params.c0 * params.Sd / params.Rms
        params.muC = (2*np.pi*f_tgt)**2 * params.Mms * params.Cmc

        params.b2 = params.muM * params.Mms * params.Cmc
        params.b1 = params.muR * params.Rms * params.Cmc
        params.b0 = params.muC

        params.a2 = (params.muM - 1.0) * params.Mms * params.Cmc
        params.a1 = (params.muR - 1.0) * params.Rms * params.Cmc
        params.a0 = (params.muC - 1.0)

        params.ts_ctr = 40e-6

        # --- Run c2d ---
        bz, az = compute_filter_coeffs(params)

        # CMSIS signs
        b0, b1, b2 = bz
        a1 = -az[1]
        a2 = -az[2]

        # --- Generate header ---
        # header_path = f"{self.project_path}/Core/Src/generated_params.h"
        header_path = os.path.join(self.project_path, "Core", "Src", "generated_params.h")
        core_src = os.path.dirname(header_path)
        if not os.path.isdir(core_src):
            self.log.append(f"[save_params] ERROR: directory does not exist:\n    {core_src}\n")
            return

        content = f"""#ifndef GENERATED_PARAMS_H
#define GENERATED_PARAMS_H

#define B0 {b0:.12e}f
#define B1 {b1:.12e}f
#define B2 {b2:.12e}f
#define A1 {a1:.12e}f
#define A2 {a2:.12e}f

#endif
"""
        try:
            with open(header_path, "w") as f:
                f.write(content)
            self.log.append(f"✔ Updated {header_path}\n")
        except Exception as e:
            self.log.append(f"[save_params] WRITE FAILED: {e}\n")


    def run_cmd(self, cmd, cwd=None):
        if cwd is None:
            cwd = self.project_path

        self.log.append(f"> {cmd}\n")

        env = self.toolchain.get_env()
        # print("env[PATH]")
        # print(env["PATH"])

        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=cwd,
            env=env
        )
        for line in process.stdout:
            self.log.append(line)
        process.wait()
        self.log.append(f"\nExit code: {process.returncode}\n")

    def detect_os(self):
        system = platform.system()
        if system == "Darwin":
            return "mac"
        elif system == "Windows":
            return "win"
        else:
            return "other"

    def build_project(self):
        if not self.project_path:
            self.log.append("No project selected!")
            return

        build_dir = os.path.join(self.project_path, "Debug")
        self.log.append(f"Building in: {build_dir}\n")

        os_type = self.detect_os()

        # Adjust depending the toolchain
        # Examples:
        #   make
        #   cmake --build .
        #   stm32cubeide --launcher.suppressErrors -nosplash -application headless ... 

        if os_type == "mac":
            # Most macOS users use gcc-arm-none-eabi + make
            cmd = "make all -j8"

        elif os_type == "win":
            # Windows needs full path to make.exe (or mingw32-make)
            # Example:
            cmd = r"C:\STM32\Tools\make.exe -j8"

        else:
            self.log.append("Unsupported OS for building.")
            return

        self.run_cmd(cmd, cwd=build_dir)


    def flash_project(self):
        if not self.project_path:
            self.log.append("No project selected!")
            return

        os_type = self.detect_os()

        # cmd = "openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \"program build/firmware.elf verify reset exit\""
        if getattr(sys, "frozen", False):
            elf = os.path.join(self.app_root, "Accoustic-Controller/Debug/Accoustic-Controller.elf")
        else:
            elf = os.path.abspath("firmware/Debug/Accoustic-Controller.elf")

        self.log.append(f"ELF: {elf}")
        
        if os_type == "mac":
            cmd = (
            'openocd '
            '-f interface/stlink.cfg '
            '-f target/stm32f7x.cfg '
            f'-c "program {elf} verify reset exit"'
            )

        elif os_type == "linux":
            cmd = (
                'openocd '
                '-f interface/stlink.cfg '
                '-f target/stm32f7x.cfg '
                f'-c "program {elf} verify reset exit"'
            )

        elif os_type == "win":
            cube_cli = r'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
            cmd = (
                f'"{cube_cli}" '
                '-c port=SWD '
                f'-w "{elf}" '
                '-v -rst'
            )

        else:
            self.log.append("Unsupported OS for flashing.")
            return

        self.run_cmd(cmd)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = BuilderGUI()
    gui.resize(600, 400)
    gui.show()
    sys.exit(app.exec())

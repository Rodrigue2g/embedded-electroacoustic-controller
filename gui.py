import os
import sys, subprocess
import platform
import scipy.signal as signal
import matplotlib.pyplot as plt
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, 
    QTextEdit, QFileDialog, QLabel, QTabWidget
)
from py.c2d import Params, compute_filter_coeffs
from py.toolchain import ToolchainManager
from py.ui_components import ParamsWidget
from py.plot import handle_plot
from py.save_params import save_all_params


class BuilderGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Embedded Builder - Multi Mode")
        self.resize(800, 600)

        self.layout = QVBoxLayout()

        # === 1. TOP CONTROLS ===
        top_layout = QVBoxLayout()
        self.btn_select = QPushButton("Select Project Folder")
        self.status_label = QLabel("No project selected.")
        self.btn_select.clicked.connect(self.select_folder)
        top_layout.addWidget(self.btn_select)
        top_layout.addWidget(self.status_label)
        self.layout.addLayout(top_layout)

        # === 2. TABS FOR PARAMETERS ===
        self.tabs = QTabWidget()
        self.mode1_ui = ParamsWidget("Mode 1 (Default)")
        self.mode2_ui = ParamsWidget("Mode 2")
        self.mode3_ui = ParamsWidget("Mode 3")
        self.mode1_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 1"))
        self.mode2_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 2"))
        self.mode3_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 3"))
        self.tabs.addTab(self.mode1_ui, "Mode 1")
        self.tabs.addTab(self.mode2_ui, "Mode 2")
        self.tabs.addTab(self.mode3_ui, "Mode 3")
        self.layout.addWidget(self.tabs)

        # === 3. BUILD ACTIONS ===
        self.btn_save = QPushButton("Save All Parameters")
        self.btn_clean = QPushButton("Clean Build Folder")
        self.btn_build = QPushButton("Build Firmware")
        self.btn_flash = QPushButton("Flash Firmware")

        self.btn_save.clicked.connect(self.run_save)
        self.btn_clean.clicked.connect(self.clean_project)
        self.btn_build.clicked.connect(self.build_project)
        self.btn_flash.clicked.connect(self.flash_project)

        self.layout.addWidget(self.btn_save)
        self.layout.addWidget(self.btn_clean)
        self.layout.addWidget(self.btn_build)
        self.layout.addWidget(self.btn_flash)

        # === 4. LOG ===
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.layout.addWidget(self.log)

        self.setLayout(self.layout)
        
        # Initialize project path
        self.project_path = None
        self.project_path = self.get_project_root()
        self.status_label.setText(f"Selected: {self.project_path}")
        self.log.append(f"sys.frozen = {getattr(sys, 'frozen', None)}\n")
        self.log.append(f"sys._MEIPASS = {getattr(sys, '_MEIPASS', None)}\n")
        self.log.append(f"sys.executable = {sys.executable}\n")

        try:
            app_root = self.get_app_root()
            self.log.append(f"app_root= {app_root}\n")
            self.toolchain = ToolchainManager(app_root)
            self.toolchain.load()
            self.log.append("✔ Toolchain loaded.\n")
        except Exception as e:
            self.log.append(f"Toolchain error: {e}\n")

    ##
    # Paths
    ##
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
            # Dev mode uses the local folder
            return os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")
    def select_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            self.project_path = folder
            self.status_label.setText(f"Selected: {folder}")

    ##
    # Params + Plots
    ##
    def run_plot(self, values, mode_name):
        """
        Wrapper for the plotting logic.
        We pass 'self' so the external function can access self.log
        """
        handle_plot(self, values, mode_name)

    def run_save(self):
        """
        Wrapper for the saving logic.
        We pass 'self' so the external function can access ui widgets and paths.
        """
        save_all_params(self)

    ##
    # Firmware
    ##
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

    def clean_project(self):
        if not self.project_path:
            self.log.append("No project selected!")
            return

        build_dir = os.path.join(self.project_path, "Debug")
        
        if not os.path.isdir(build_dir):
             self.log.append(f"Build directory not found: {build_dir}\n")
             return

        self.log.append(f"Cleaning in: {build_dir}\n")

        os_type = self.detect_os()

        if os_type == "mac":
            cmd = "make clean"

        elif os_type == "win":
            cmd = r"C:\STM32\Tools\make.exe clean"

        else:
            self.log.append("Unsupported OS for cleaning.")
            return

        self.run_cmd(cmd, cwd=build_dir)

    def build_project(self):
        if not self.project_path:
            self.log.append("No project selected!")
            return

        build_dir = os.path.join(self.project_path, "Debug")
        self.log.append(f"Building in: {build_dir}\n")

        os_type = self.detect_os()

        if os_type == "mac":
            cmd = "make all -j8"

        elif os_type == "win":
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

import os
import sys, subprocess
import platform
import scipy.signal as signal
import matplotlib.pyplot as plt
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, 
    QTextEdit, QFileDialog, QLabel, QTabWidget,
    QMenuBar, QMenu, QGroupBox, QComboBox, QHBoxLayout, QFrame
)
from PySide6.QtGui import QAction
from py.c2d import Params, compute_filter_coeffs
from py.toolchain import ToolchainManager
from py.ui_components import ParamsWidget
from py.plot import handle_plot
from py.save_params import save_all_params
import shutil
from pathlib import Path
import platform

class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Embedded Builder - Multi Mode")
        self.resize(800, 700)

        # Main Layout
        self.main_layout = QVBoxLayout()
        self.layout = self.main_layout  # Alias for compatibility with existing code

        # === 0. MENU BAR ===
        self.setup_menu()

        # === 1. GLOBAL SETTINGS (Sampling Freq) ===
        self.setup_global_settings()

        # === 3. TABS FOR PARAMETERS ===
        self.tabs = QTabWidget()
        self.mode1_ui = ParamsWidget("Mode 1 (Default)")
        self.mode2_ui = ParamsWidget("Mode 2")
        self.mode3_ui = ParamsWidget("Mode 3")
        
        # Connect plot requests
        self.mode1_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 1"))
        self.mode2_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 2"))
        self.mode3_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 3"))
        
        self.tabs.addTab(self.mode1_ui, "Mode 1")
        self.tabs.addTab(self.mode2_ui, "Mode 2")
        self.tabs.addTab(self.mode3_ui, "Mode 3")
        self.main_layout.addWidget(self.tabs)

        # === 4. STANDARD ACTIONS (Always Visible) ===
        actions_group = QGroupBox("Actions")
        actions_layout = QHBoxLayout()
        
        self.btn_save = QPushButton("Save All Parameters")
        self.btn_build = QPushButton("Build Firmware")
        self.btn_flash = QPushButton("Flash Firmware")

        self.btn_save.clicked.connect(self.run_save)
        self.btn_build.clicked.connect(self.build_project)
        self.btn_flash.clicked.connect(self.flash_project)

        actions_layout.addWidget(self.btn_save)
        actions_layout.addWidget(self.btn_build)
        actions_layout.addWidget(self.btn_flash)
        actions_group.setLayout(actions_layout)
        self.main_layout.addWidget(actions_group)

        # === 2. ADVANCED / DEBUG CONTROLS (Hidden by default) ===
        self.setup_advanced_controls()

        # === 5. LOG ===
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.main_layout.addWidget(self.log)

        self.setLayout(self.main_layout)
        
        # --- Initialization Logic ---
        self.init_toolchain()

    def setup_menu(self):
        # In the app Menu bar (sys)
        self.menu_bar = QMenuBar(self)
        debug_menu = self.menu_bar.addMenu("Debug")
        
        # Toggle Action
        self.toggle_advanced_action = QAction("Show Advanced Options", self)
        self.toggle_advanced_action.setCheckable(True)
        self.toggle_advanced_action.setChecked(False) # Default
        self.toggle_advanced_action.triggered.connect(self.toggle_advanced_visibility)
        debug_menu.addAction(self.toggle_advanced_action)

        # Add menu to layout (since QWidget doesn't have setMenuBar like QMainWindow)
        self.main_layout.setMenuBar(self.menu_bar)

    def setup_global_settings(self):
        group = QGroupBox("Global Configuration")
        layout = QHBoxLayout()

        lbl_fs = QLabel("Sampling Frequency:")
        self.combo_fs = QComboBox()
        # Add common control loop frequencies
        self.combo_fs.addItems(["10000", "20000", "25000", "40000", "50000", "2000"])
        self.combo_fs.setCurrentText("25000")

        layout.addWidget(lbl_fs)
        layout.addWidget(self.combo_fs)
        layout.addStretch() # Push items to left

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def setup_advanced_controls(self):
        """Controls meant for Developer/Debug mode only"""
        self.advanced_container = QGroupBox("Advanced / Debug Settings")
        layout = QVBoxLayout()

        # Project Path Selection
        path_layout = QHBoxLayout()
        self.btn_select = QPushButton("Select Project Folder")
        self.status_label = QLabel("No project selected.")
        self.btn_select.clicked.connect(self.select_folder)
        path_layout.addWidget(self.btn_select)
        path_layout.addWidget(self.status_label)
        layout.addLayout(path_layout)

        # Clean Build
        self.btn_clean = QPushButton("Clean Build Folder")
        self.btn_clean.clicked.connect(self.clean_project)
        layout.addWidget(self.btn_clean)

        self.advanced_container.setLayout(layout)
        
        # Initial Visibility State
        self.advanced_container.setVisible(False)
        self.main_layout.addWidget(self.advanced_container)

    def toggle_advanced_visibility(self):
        is_visible = self.toggle_advanced_action.isChecked()
        self.advanced_container.setVisible(is_visible)

    def get_selected_fs(self):
        """Helper to get current Sampling Freq as float"""
        try:
            return float(self.combo_fs.currentText())
        except ValueError:
            return 25000.0
        
    def init_toolchain(self):
        """Moved initialization logic here for cleaner __init__"""
        self.project_path = self.get_project_root()
        self.status_label.setText(f"Selected: {self.project_path}")
        
        # Logging basics
        self.log.append(f"sys.frozen = {getattr(sys, 'frozen', None)}")
        self.log.append(f"sys.executable = {sys.executable}")

        try:
            app_root = self.get_app_root()
            self.toolchain = ToolchainManager(app_root)
            self.toolchain.load()
            
            env = self.toolchain.get_env()
            if shutil.which("make", path=env["PATH"]) is None:
                self.log.append("❌ make not found in PATH")
            else:
                self.log.append("✔ make found in PATH")

            self.project_path = self.ensure_user_firmware(app_root, self.log)
            self.status_label.setText(f"Selected: {self.project_path}")
        except Exception as e:
            self.log.append(f"Toolchain error: {e}")
    ##
    # Paths
    ##
    def get_app_root(self):
        if getattr(sys, 'frozen', False):
            if sys.platform == "darwin":
                return os.path.abspath(
                    os.path.join(os.path.dirname(sys.executable), "..", "Resources")
                )
            else:
                # Windows / Linux
                return os.path.dirname(sys.executable) #sys._MEIPASS
        else:
            return os.path.dirname(os.path.abspath(__file__))

    def get_project_root(self):
        if getattr(sys, 'frozen', False):
            if sys.platform == "darwin":
                # On MacOS, it is bundled in Resources
                return os.path.join(
                    os.path.dirname(sys.executable),
                    "..",   # Contents
                    "Resources",
                    "Accoustic-Controller"
                )
            else:
                # Windows / Linux
                return os.path.join(
                    os.path.dirname(sys.executable),
                    "Accoustic-Controller"
                )
                # or just
                # return os.path.join(
                #     sys._MEIPASS,
                #     "Accoustic-Controller"
                # )
        else:
            # Dev mode uses the local folder
            return os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware")
        
    def ensure_user_firmware(self, app_root, log=None):
        if getattr(sys, 'frozen', False):
            if platform.system() == "Windows":
                base = Path.home() / "Documents" / "STM32Builder"
            else:
                # macOS: you can keep current behavior or mirror this
                base = Path(app_root)

            dst = base / "Accoustic-Controller"
            src = Path(app_root) / "Accoustic-Controller"

            if not dst.exists():
                shutil.copytree(src, dst)
                if log:
                    log.append(f"✔ Firmware copied to user workspace:\n{dst}\n")
            else:
                if log:
                    log.append(f"✔ Using existing firmware:\n{dst}\n")

            return str(dst)
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
            cmd = "make clean"

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
            cmd = "make all -j8"

        else:
            self.log.append("Unsupported OS for building.")
            return

        self.run_cmd(cmd, cwd=build_dir)


    def flash_project(self):
        self.log.append(f"Flashing firmware...")
        if not self.project_path:
            self.log.append("No project selected!")
            return

        elf = os.path.join(self.project_path, "Debug", "Accoustic-Controller.elf")
        self.log.append(f"Flashing ELF: {elf}")

        os_type = self.detect_os()

        # cmd = "openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c \"program build/firmware.elf verify reset exit\""
        # if getattr(sys, "frozen", False):
        #     elf = os.path.join(self.project_path, "Accoustic-Controller/Debug/Accoustic-Controller.elf")
        # else:
        #     elf = os.path.abspath("firmware/Debug/Accoustic-Controller.elf")

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
            # cmd = (
            #     'openocd '
            #     '-f interface/stlink.cfg '
            #     '-f target/stm32f7x.cfg '
            #     f'-c "program {elf} verify reset exit"'
            # )
            cmd = (
                'STM32_Programmer_CLI.exe '
                '-c port=SWD mode=UR '
                f'-w "{elf}" 0x08000000 '
                '-v -rst'
            )
            # cube_cli = r'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
            # cmd = (
            #     f'"{cube_cli}" '
            #     '-c port=SWD '
            #     f'-w "{elf}" '
            #     '-v -rst'
            # )

        else:
            self.log.append("Unsupported OS for flashing.")
            return

        self.run_cmd(cmd)
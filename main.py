import os
import sys, subprocess
import platform
import shutil
import platform
import scipy.signal as signal
import matplotlib.pyplot as plt
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, 
    QTextEdit, QFileDialog, QLabel, QTabWidget,
    QMenuBar, QMenu, QGroupBox, QComboBox, QHBoxLayout, QFrame
)
from PySide6.QtGui import QAction, QActionGroup
from PySide6.QtCore import Qt, QSettings
from PySide6.QtWidgets import QMessageBox, QProgressDialog
from pathlib import Path
from py.plot import handle_plot
from py.progress import ProgressOverlay
from py.ui_components import ParamsWidget
from py.toolchain import ToolchainManager
from py.save_params import save_all_params
from py.factory_settings import perform_factory_reset, DEFAULT_FS
from py.strings import LOG_MESSAGES, UI_TEXT

##
# Default number of modes for the GUI 
# Switch between: [3,6]
##
MODE_TYPE = 3


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("EAR Builer")
        self.settings = QSettings("EPFL-LWE", "ear-builder")

        # Main Layout
        self.main_layout = QVBoxLayout()
        self.layout = self.main_layout

        # === MENU BAR ===
        self.setup_menu()

        # === GLOBAL SETTINGS (Sampling Freq) ===
        self.setup_global_settings()

        # === TABS FOR PARAMETERS ===
        self.tabs = QTabWidget()
        self.mode1_ui = ParamsWidget("Mode 1 (Default)")
        self.mode2_ui = ParamsWidget("Mode 2")
        self.mode3_ui = ParamsWidget("Mode 3")
        self.mode4_ui = ParamsWidget("Mode 4")
        self.mode5_ui = ParamsWidget("Mode 5")
        self.mode6_ui = ParamsWidget("Mode 6")

        # Connect plot requests
        self.mode1_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 1"))
        self.mode2_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 2"))
        self.mode3_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 3"))
        
        self.mode4_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 4"))
        self.mode5_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 5"))
        self.mode6_ui.request_plot.connect(lambda v: self.run_plot(v, "Mode 6"))


        if MODE_TYPE == 3:
            self.tabs.addTab(self.mode1_ui, "Mode 1")
            self.tabs.addTab(self.mode2_ui, "Mode 2")
            self.tabs.addTab(self.mode3_ui, "Mode 3")
        elif MODE_TYPE == 6:
            self.tabs.addTab(self.mode1_ui, "Mode 1")
            self.tabs.addTab(self.mode2_ui, "Mode 2")
            self.tabs.addTab(self.mode3_ui, "Mode 3")
            self.tabs.addTab(self.mode4_ui, "Mode 4")
            self.tabs.addTab(self.mode5_ui, "Mode 5")
            self.tabs.addTab(self.mode6_ui, "Mode 6")

        self.main_layout.addWidget(self.tabs)

        # === SAVE AS DEFAULTS ===
        self.setup_persistence_actions()

        # === Board Programming (Always Visible) ===
        actions_group = QGroupBox("Build")
        actions_layout = QHBoxLayout()
        
        self.btn_full_build = QPushButton("Program Board")
        self.btn_full_build.clicked.connect(self.run_full_build)

        actions_layout.addWidget(self.btn_full_build)
        actions_group.setLayout(actions_layout)
        self.main_layout.addWidget(actions_group)


        # === ADVANCED / DEBUG CONTROLS (Hidden by default) ===
        self.setup_build_actions()
        self.setup_advanced_controls()


        # ===  LOGs ===
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.main_layout.addWidget(self.log)

        self.setLayout(self.main_layout)
        
        # === INIT TOOLCHAIN & LOAD SETTINGS
        self.init_toolchain()
        self.load_settings()

        ##########
        #####
        ##

    def setup_menu(self):
        # Create Menu Bar
        self.menu_bar = QMenuBar(self)
        
        mode_menu = self.menu_bar.addMenu("View")
        self.mode_action_group = QActionGroup(self) # Ensure only one is selected
        
        for count in [3, 6]:
            action = QAction(f"{count} Modes", self)
            action.setCheckable(True)
            action.setData(count)
            if count == MODE_TYPE: action.setChecked(True) # Default
            action.triggered.connect(self.change_mode_count)
            self.mode_action_group.addAction(action)
            mode_menu.addAction(action)

        # Settings Menu
        settings_menu = self.menu_bar.addMenu("Settings")

        self.toggle_build_actions = QAction("Show Build Actions", self)
        self.toggle_build_actions.setCheckable(True)
        self.toggle_build_actions.setChecked(False) # Default hidden
        self.toggle_build_actions.triggered.connect(self.toggle_build_actions_visibility)
        settings_menu.addAction(self.toggle_build_actions)

        self.action_factory_reset = QAction("Factory Reset All Modes", self)
        self.action_factory_reset.triggered.connect(self.confirm_factory_reset)
        settings_menu.addAction(self.action_factory_reset)

        # Debug Menu
        debug_menu = self.menu_bar.addMenu("Debug")
        
        self.toggle_advanced_action = QAction("Show Advanced Options", self)
        self.toggle_advanced_action.setCheckable(True)
        self.toggle_advanced_action.setChecked(False) # Default hidden
        self.toggle_advanced_action.triggered.connect(self.toggle_advanced_visibility)
        debug_menu.addAction(self.toggle_advanced_action)

        # Add menu to layout
        self.main_layout.setMenuBar(self.menu_bar)

    def change_mode_count(self):
        action = self.sender()
        new_count = action.data()
        
        self.setWindowTitle(f"EAR Builder - {new_count} Modes")
        
        self.tabs.clear()
        
        self.tabs.addTab(self.mode1_ui, "Mode 1")
        self.tabs.addTab(self.mode2_ui, "Mode 2")
        self.tabs.addTab(self.mode3_ui, "Mode 3")
        
        if new_count == 6:
            self.tabs.addTab(self.mode4_ui, "Mode 4")
            self.tabs.addTab(self.mode5_ui, "Mode 5")
            self.tabs.addTab(self.mode6_ui, "Mode 6")
            
        self.log.append(LOG_MESSAGES["MODE_SWITCH"].format(new_count))

    def get_current_mode_count(self):
        """Helper to see if we are in 3 or 6 mode state"""
        return self.tabs.count()

    def setup_global_settings(self):
        group = QGroupBox("Global Configuration")
        layout = QHBoxLayout()

        lbl_fs = QLabel("Sampling Frequency:")
        self.global_fs = QComboBox()
        self.global_fs.addItems(["2000", "5000", "10000", "20000", "25000", "40000", "50000"])
        self.global_fs.setCurrentText(DEFAULT_FS)

        layout.addWidget(lbl_fs)
        layout.addWidget(self.global_fs)
        layout.addStretch() # Push items to left

        group.setLayout(layout)
        self.main_layout.addWidget(group)

    def setup_build_actions(self):
        # === STANDARD Build ACTIONS ===
        self.build_actions_container = QGroupBox("Actions")
        layout = QVBoxLayout()

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
        layout.addLayout(actions_layout)

        # Clean Build
        self.btn_clean = QPushButton("Clean Build Folder")
        self.btn_clean.clicked.connect(self.clean_project)
        layout.addWidget(self.btn_clean)

        self.build_actions_container.setLayout(layout)

        self.build_actions_container.setVisible(False)
        self.main_layout.addWidget(self.build_actions_container)

    def toggle_build_actions_visibility(self):
        is_visible = self.toggle_build_actions.isChecked()
        self.build_actions_container.setVisible(is_visible)


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

        self.advanced_container.setLayout(layout)

        # Hidden by default
        self.advanced_container.setVisible(False)
        self.main_layout.addWidget(self.advanced_container)

    def toggle_advanced_visibility(self):
        is_visible = self.toggle_advanced_action.isChecked()
        self.advanced_container.setVisible(is_visible)

    # Settings Persistecy
    def save_settings(self):
        """Saves current UI state to persistent storage"""
        self.settings.setValue("fs_index", self.global_fs.currentIndex())
        self.settings.setValue("mode1_data", self.mode1_ui.get_values())
        self.settings.setValue("mode2_data", self.mode2_ui.get_values())
        self.settings.setValue("mode3_data", self.mode3_ui.get_values())

        if self.get_current_mode_count() == 6:
            self.settings.setValue("mode4_data", self.mode4_ui.get_values())
            self.settings.setValue("mode5_data", self.mode5_ui.get_values())
            self.settings.setValue("mode6_data", self.mode6_ui.get_values())

        self.log.append(LOG_MESSAGES["SAVE_SUCCESS"])

    def load_settings(self):
        """Loads settings from storage if they exist"""
        fs_idx = self.settings.value("fs_index")
        if fs_idx is not None:
            self.global_fs.setCurrentIndex(int(fs_idx))

        m1 = self.settings.value("mode1_data")
        if m1: self.mode1_ui.set_values(m1)
            
        m2 = self.settings.value("mode2_data")
        if m2: self.mode2_ui.set_values(m2)
            
        m3 = self.settings.value("mode3_data")
        if m3: self.mode3_ui.set_values(m3)

        m4 = self.settings.value("mode4_data")
        if m4: self.mode4_ui.set_values(m4)
            
        m5 = self.settings.value("mode5_data")
        if m5: self.mode5_ui.set_values(m5)
            
        m6 = self.settings.value("mode6_data")
        if m6: self.mode6_ui.set_values(m6)



    def setup_persistence_actions(self):
        group = QGroupBox("User Preferences")
        layout = QHBoxLayout()
        
        btn_set_default = QPushButton("💾 Set Current values as Default") # (for this Mode only)
        btn_set_default.clicked.connect(self.save_settings)
        
        layout.addWidget(btn_set_default)
        group.setLayout(layout)
        
        self.main_layout.addWidget(group)

    def get_selected_fs(self):
        """Helper to get current Sampling Freq as float"""
        try:
            return float(self.global_fs.currentText())
        except ValueError:
            return 25000.0
        
    def init_toolchain(self):
        """Moved initialization logic here for cleaner __init__"""
        self.project_path = self.get_project_root()
        self.status_label.setText(f"Selected: {self.project_path}")
        
        self.log.append(f"--Debug-- sys.frozen = {getattr(sys, 'frozen', None)}")
        self.log.append(f"--Debug-- sys.executable = {sys.executable}")

        try:
            app_root = self.get_app_root()
            self.toolchain = ToolchainManager(app_root)
            self.toolchain.load()
            
            env = self.toolchain.get_env()
            if shutil.which("make", path=env["PATH"]) is None:
                self.log.append(LOG_MESSAGES["MAKE_MISSING"])
            else:
                self.log.append(LOG_MESSAGES["MAKE_FOUND"])

            self.project_path = self.ensure_user_firmware(app_root, self.log)
            self.status_label.setText(f"Selected: {self.project_path}")
        except Exception as e:
            self.log.append(LOG_MESSAGES["TOOLCHAIN_ERROR"].format(e))
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
                # Project is bundled in Resources (MacOS Only)
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
                # macOS
                base = Path(app_root)

            dst = base / "Accoustic-Controller"
            src = Path(app_root) / "Accoustic-Controller"

            if not dst.exists():
                shutil.copytree(src, dst)
                if log:
                    log.append(LOG_MESSAGES["FIRMWARE_COPY"].format(dst))
            else:
                if log:
                    log.append(LOG_MESSAGES["FIRMWARE_EXIST"].format(dst))

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
        fs = self.get_selected_fs() 
        handle_plot(self, values, mode_name, fs)

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
        
        # Progress 
        if not hasattr(self, 'overlay'):
            self.overlay = ProgressOverlay(self)
        
        self.overlay.resize(self.size())
        self.overlay.start()
        QApplication.processEvents()

        env = self.toolchain.get_env()

        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=cwd,
            env=env
        )

    #     for line in process.stdout:
    #         self.log.append(line)
        while True:
            line = process.stdout.readline()
            if not line:
                break
            self.log.append(line)
            QApplication.processEvents()

        process.wait()
        self.overlay.stop()
        self.log.append(f"\nExit code: {process.returncode}\n")

    def detect_os(self):
        system = platform.system()
        if system == "Darwin":
            return "mac"
        elif system == "Windows":
            return "win"
        else:
            return "other"
        
    def run_full_build(self):
        self.run_save()
        self.build_project()
        self.flash_project()

    def clean_project(self):
        if not self.project_path:
            self.log.append(LOG_MESSAGES["NO_PROJECT"])
            return

        build_dir = os.path.join(self.project_path, "Debug")
        
        if not os.path.isdir(build_dir):
            self.log.append(LOG_MESSAGES["BUILD_NOT_FOUND"].format(build_dir))
            return

        self.log.append(LOG_MESSAGES["CLEAN_START"].format(build_dir))

        os_type = self.detect_os()

        if os_type == "mac":
            cmd = "make clean"

        elif os_type == "win":
            # cmd = r"C:\STM32\Tools\make.exe clean"
            cmd = "make clean"

        else:
            self.log.append(LOG_MESSAGES["CLEAN_UNSUPPORTED"])
            return

        self.run_cmd(cmd, cwd=build_dir)

    def build_project(self):
        if not self.project_path:
            self.log.append(LOG_MESSAGES["NO_PROJECT"])
            return

        build_dir = os.path.join(self.project_path, "Debug")
        self.log.append(LOG_MESSAGES["BUILD_START"].format(build_dir))

        os_type = self.detect_os()

        if os_type == "mac":
            cmd = "make all -j8"

        elif os_type == "win":
            # cmd = r"C:\STM32\Tools\make.exe -j8"
            cmd = "make all -j8"

        else:
            self.log.append(LOG_MESSAGES["BUILD_UNSUPPORTED"])
            return

        self.run_cmd(cmd, cwd=build_dir)


    def flash_project(self):
        self.log.append(LOG_MESSAGES["FLASH_START"])
        if not self.project_path:
            self.log.append(LOG_MESSAGES["NO_PROJECT"])
            return

        elf = os.path.join(self.project_path, "Debug", "Accoustic-Controller.elf")
        self.log.append(LOG_MESSAGES["FLASH_ELF"].format(elf))

        os_type = self.detect_os()

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
            cmd = (
                'STM32_Programmer_CLI.exe '
                '-c port=SWD mode=UR '
                f'-w "{elf}" 0x08000000 '
                '-v -rst'
            )

        else:
            self.log.append(LOG_MESSAGES["FLASH_UNSUPPORTED"])
            return

        self.run_cmd(cmd)


    def restore_factory_settings(self):
        """Clears saved settings and resets UI to hardcoded defaults"""
        perform_factory_reset(self)
        self.log.append(LOG_MESSAGES["FACTORY_RESET_DONE"])

    def confirm_factory_reset(self):
        """Shows a modal confirmation dialog before resetting."""
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle(UI_TEXT["CONFIRM_RESET_TITLE"])
        msg_box.setText(UI_TEXT["CONFIRM_RESET_TEXT"])
        msg_box.setInformativeText(UI_TEXT["CONFIRM_RESET_INFO"])
        msg_box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        msg_box.setDefaultButton(QMessageBox.StandardButton.No)
        msg_box.setIcon(QMessageBox.Icon.Warning)

        result = msg_box.exec()

        if result == QMessageBox.StandardButton.Yes:
            self.restore_factory_settings()
            self.log.append(LOG_MESSAGES["FACTORY_RESET_CONFIRM"])
        else:
            self.log.append(LOG_MESSAGES["FACTORY_RESET_CANCEL"])



if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = GUI()
    gui.resize(600, 400)
    gui.show()
    sys.exit(app.exec())

import os
import platform

class ToolchainManager:
    """
    Loads the bundled STM32 toolchain shipped inside ./toolchain/.

    Expected structure:

        app_root/
            toolchain/
                mac/
                    bin/
                        arm-none-eabi-gcc
                        STM32_Programmer_CLI
                win/
                    bin/
                        arm-none-eabi-gcc.exe
                        STM32_Programmer_CLI.exe
    """

    TOOLCHAIN_BIN_FILENAME = "toolchain_path.txt"

    def __init__(self, app_root):
        self.app_root = app_root
        self.toolchain_path = None
        self.loaded = False

    # ------------------------------------------------
    # OS detection
    # ------------------------------------------------
    @staticmethod
    def detect_os():
        s = platform.system()
        if s == "Darwin":
            return "mac"
        if s == "Windows":
            return "win"
        return "other"

    # ------------------------------------------------
    # Find correct GCC inside a root folder
    # ------------------------------------------------
    @staticmethod
    def find_gcc_in(root, os_id):
        target = "arm-none-eabi-gcc.exe" if os_id == "win" else "arm-none-eabi-gcc"

        for current_path, dirs, files in os.walk(root):
            if target in files:
                return current_path
        return None

    # ------------------------------------------------
    # Load toolchain based on OS
    # ------------------------------------------------
    def load(self):
        os_id = self.detect_os()

        if os_id == "mac":
            tc_root = os.path.join(self.app_root, "toolchain", "mac")
        elif os_id == "win":
            tc_root = os.path.join(self.app_root, "toolchain", "win")
        else:
            raise RuntimeError("Unsupported OS for STM32 toolchain.")

        if not os.path.isdir(tc_root):
            raise RuntimeError(
                f"❌ Toolchain directory not found: {tc_root}\n"
                "Make sure the toolchain is bundled in ./toolchain/<os>/"
            )

        if os_id == "mac":
            self.toolchain_path = os.path.join(self.app_root, "toolchain", "mac", "bin")
        elif os_id == "win":
            self.toolchain_path = os.path.join(self.app_root, "toolchain", "win", "bin")
        else:
            raise RuntimeError(
                f"❌ Toolchain directory not found: {self.app_root}\n"
                "Make sure the toolchain is bundled in ./toolchain/<os>/"
            )

        # Otherwise perform fresh search
        # bin_path = self.find_gcc_in(tc_root, os_id)
        # if not bin_path:
        #     raise RuntimeError(
        #         f"❌ Could not find ARM GCC in: {tc_root}\n"
        #         "Expected bin/arm-none-eabi-gcc"
        #     )

        # self.toolchain_path = bin_path
        self.loaded = True
        print(f"✔ Found bundled toolchain: {self.toolchain_path}")
        return self.toolchain_path

    # ------------------------------------------------
    def load_or_error(self):
        if not self.loaded:
            self.load()
        return self.toolchain_path

    # ------------------------------------------------
    # Prepare build environment
    # ------------------------------------------------
    def get_env(self):
        tc_path = self.load_or_error()
        env = os.environ.copy()
        env["PATH"] = tc_path + os.pathsep + env["PATH"]
        return env

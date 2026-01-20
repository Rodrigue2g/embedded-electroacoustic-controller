# Embedded Electroacoustic Controller

*A standalone MCU-based controller for electroacoustic absorbers.*

This project implements a **real-time embedded controller** for electroacoustic absorbers running on an **STM32 microcontroller**. It provides a **portable, low-cost, and scalable alternative** to real-time platforms such as SpeedGoat.

---

## Project Overview

The controller implements a second-order control transfer function on an STM32 MCU. It performs real-time signal acquisition, control computation, and actuation at a fixed sampling rate using hardware timers and interrupts.

The firmware is designed to be used in conjunction with a host-side graphical interface (e.g. *EAR Builder*), which handles parameter configuration, digital controller synthesis, firmware generation, and board programming.

---

## Repository Structure

The repository is organized to clearly separate **embedded firmware**, **application logic**, and **build toolchain**:

    ├── .github/
    │   └── workflows/
    │       └── build-installers.yml
    ├── firmware/
    │   ├── Core/ 
    │   |   ├── Inc/
    │   |   └── Src/
    |   |       ├── main.c
    |   |       ├── params.h
    |   |       ├── generated_params.h
    |   |       └── stm32f7xx_it.c
    |   └── Drivers/
    │       ├── CMSIS/
    │       └── STM32xx_HAL_Driver/
    ├── py/
    │   ├── ui_components.py
    │   ├── factory_settings.py
    │   ├── toolchain.py
    │   ├── save_params.py    
    │   ├── compute_filter_coeffs.py
    │   ├── strings.py
    │   ├── physics.py
    │   └── plot.py
    ├── toolchain/
    │   ├── mac/
    │   |   ├── bin/
    │   |   └── lib/
    │   └── win/
    │       ├── bin/
    │       └── lib/
    ├── main.py
    ├── STM32Builder.spec
    ├── STM32Builder_win.spec
    ├── installer.iss
    ├── requirements.txt
    ├── requirements-dev.txt
    ├── README.md
    └── LICENSE

---

## Real-Time Execution Model

The control algorithm is executed inside a **hardware timer interrupt callback**, ensuring a strictly fixed sampling period. After system initialization, the `main()` loop remains idle, and all real-time processing is handled within interrupt context.

This design guarantees deterministic timing behavior, independent of background tasks or host-side interaction.

---

## Building and Flashing the Firmware

The firmware can be built and flashed using either:

- The **custom GUI application**, which automates parameter generation, compilation, and flashing, or
- **STM32CubeIDE**, for manual inspection, debugging, or development.

When using the GUI, the required STM32 toolchain is bundled with the application, and no separate IDE installation is necessary.

---

## Configuring the Number of Modes (3 vs 6)

The firmware supports multiple electroacoustic resonator configurations at **compile time**.

The number of active modes (three or six, plus an always-included off mode) is selected directly from the GUI. This selection updates the `VERSION_TYPE` macro in the auto-generated file:


`firmware/Core/Inc/generated_params.h`

Changing the mode configuration therefore requires recompilation but does not require manual modification of the firmware source code.

---

## License

This project is intended primarily for **academic and research use**.

The source code is distributed under the terms of the MIT License.  
See the `LICENSE` file for full details.

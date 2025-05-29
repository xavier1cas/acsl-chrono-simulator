# ACSL Physics Simulator - Installation Instructions

The **ACSL Physics Simulator** is a modular simulation framework built to support **Hardware-in-the-Loop (HIL)**, **Software-in-the-Loop (SIL)**, and **Model-in-the-Loop (MIL)** simulations. It is developed and maintained by the Advanced Control Systems Lab (ACSL) and leverages [Project Chrono](https://projectchrono.org/) as its physics engine.

To streamline installation and package requirement setup, a script-based installer is provided. This document outlines a step-by-step guide to install all required packages and successfully compile the simulator.

> [!IMPORTANT]
> The installer log is only created **after** the `installer.sh ` is executed and you accept the licenses.

If you encounter any issues during the setup process with the `installer.sh` script, please upload the log file located at `scripts/cache/` after cleaning it using the `clean-installer-log.sh` in the `scripts/` directory.

**How to clean up the logs:**

```bash
# Navigate to repository
cd /path/to/acsl-physics-sim/scripts/

# Make the clean-installer-log.sh executable
sudo chmod +x clean-installer-log.sh

# Execute
./clean-installer-log.sh
```

Ths will clean up the `installer-log.log` file in `scripts/cache/` directory. Upload it to the **Issues** tab of the repository on GitHub.


## Installation Workflow

The installation is divided into **three major steps** and **requires access to the internet**:

- [ACSL Physics Simulator - Installation Instructions](#acsl-physics-simulator---installation-instructions)
  - [Installation Workflow](#installation-workflow)
  - [1. Setup the Required Packages using installer.sh](#1-setup-the-required-packages-using-installersh)
  - [2. Compile the Chrono Physics Engine](#2-compile-the-chrono-physics-engine)
  - [3. Compile the ACSL Physics Simulator](#3-compile-the-acsl-physics-simulator)
  - [🛠️ Troubleshooting](#️-troubleshooting)

> [!IMPORTANT]
> **Please complete these steps in order. Skipping or reordering them may lead to build errors or incomplete installations.**

## 1. Setup the Required Packages using installer.sh

> [!CAUTION]
> This script requires **root access** and installs packages directly onto your system. **Never run scripts from the internet without reviewing them first.**
> 
> By executing `installer.sh`, you acknowledge that you do so at your own discretion and risk.
> **The Advanced Control Systems Lab (ACSL) is not responsible for any damage, misconfiguration, or data loss resulting from the use of this script.**
>
> **We strongly recommend opening** scripts/installer.sh in a text editor and inspecting its contents before running it.

This step installs all dependencies required for both Chrono and the ACSL Physics Simulator. The script also sets up environment variables and system configurations.

> [!CAUTION]
> Make sure you are inside the `acsl-physics-sim/scripts` directory before executing the installer script. **It must be run from this location**.

**Instructions:**

1. Open a terminal.
2. Navigate to the root of the ACSL Physics Simulator' scripts repository:
   ```bash
   cd path/to/acsl-physics-sim/scripts/
   ```
3. Make the script executable:
   ```bash
   chmod +x scripts/installer.sh
   ```
4. Run the installer:
   ```bash
   sudo ./installer.sh
   ```

> [!NOTE]
> - This script will install common build tools (`cmake`, `g++`, `python3`, etc.), graphics dependencies (OpenGL, GLFW), and third-party libraries needed for Chrono.
> - The installation may take several minutes depending on your system and internet speed.

## 2. Compile the Chrono Physics Engine

Once dependencies are installed, you’ll need to compile **Project Chrono**—a high-performance C++ library for multi-physics simulation. The `installer.sh` script will open a secondary terminal window in the `/path/to/acsl-physics-sim/libraries/chorno-build` where the Chrono libraries will be compiled.

Set the following settings in the `ccmake` window on the new terminal:

> [!TIP]
> - Please set the following settings one at a time, Press `space bar` to change settings in the CMake configuration screen. 
> - Then press `c` on your keyboard to configure the required cmake dependencies. 
> - Once the final setting is set and you get a successful configuration, press `g` to generate the code.

> [!CAUTION]
> Please go one setting at a time. Do not set everything at once. It makes it difficult to catch any configuration erros that way.

| CMake Option                | Value   |
|-----------------------------|---------|
| `CMAKE_BUILD_TYPE`          | Release |
| `ENABLE_MODULE_CASCADE`     | ON      |
| `ENABLE_MODULE_IRRLICHT`    | ON      |
| `ENABLE_MODULE_MULTICORE`   | ON      |
| `ENABLE_MODULE_OPENGL`      | ON      |
| `ENABLE_MODULE_POSTPROCESS` | ON      |
| `ENABLE_MODULE_ROS`         | ON      |
| `ENABLE_MODULE_VEHICLE`     | ON      |


Once the above settings are set, you are now ready to compile Project Chrono. Run in the **same window** where you finished the cmake configuration:

```bash
make -j$(nproc)
```

to compile with all available cores. This will build Project Chrono in `libraries/chrono-build/`.

> [!NOTE]
> Upon successful build, the example programs from Project Chrono will be placed in the `libraries/chrono-build/bin/` directory. You can run them to check if all modules of Chrono are compiled properly on your system.

## 3. Compile the ACSL Physics Simulator

Once Chrono is compiled, we are ready to compile the ACSL Physics Simulator codebase.

## 🛠️ Troubleshooting

- **Compilation errors?** Double-check that all dependencies were successfully installed via `installer.sh`. Debug with the generated log file in `scripts/cache/installer-log.log`.
- **Chrono build issues?** Check the [google group](https://groups.google.com/g/projectchrono?pli=1) maintained by Project Chrono.


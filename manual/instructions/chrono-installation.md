# ACSL Chrono Simulator - Installation Instructions

The **ACSL Chrono Simulator** is a modular simulation framework built to support **Hardware-in-the-Loop (HIL)**, **Software-in-the-Loop (SIL)**, and **Model-in-the-Loop (MIL)** simulations. It is developed and maintained by the Advanced Control Systems Lab (ACSL) and leverages [Project Chrono](https://projectchrono.org/) as its physics engine.

To streamline installation and package requirement setup, a script-based installer is provided. This document outlines a step-by-step guide to install all required packages and successfully compile the simulator.

> [!IMPORTANT]
> The installer log is only created **after** the `installer.sh` is executed and you accept the licenses.

If you encounter any issues during the setup process with the `installer.sh` script, please upload the log file located at `scripts/cache/` after cleaning it using the `clean-installer-log.sh` in the `scripts/` directory.

**How to clean up the logs:**

```bash
# Navigate to repository
cd /path/to/acsl-chrono-sim/scripts/

# Make the clean-installer-log.sh executable
sudo chmod +x clean-installer-log.sh

# Execute
./clean-installer-log.sh
```

This will clean up the `installer-log.log` file in the `scripts/cache/` directory. Upload it to the **Issues** tab of the repository on GitHub.

## Installation Workflow

The installation is divided into **three major steps** and **requires access to the internet**:

- [ACSL Chrono Simulator - Installation Instructions](#acsl-chrono-simulator---installation-instructions)
  - [Installation Workflow](#installation-workflow)
  - [0. Clone the Repository](#0-clone-the-repository)
  - [1. Setup the Required Packages using installer.sh](#1-setup-the-required-packages-using-installersh)
  - [2. Compile the Chrono Physics Engine](#2-compile-the-chrono-physics-engine)
  - [3. Compile the ACSL Chrono Simulator](#3-compile-the-acsl-chrono-simulator)
  - [WSL2 Notes](#wsl2-notes)
  - [🛠️ Troubleshooting](#️-troubleshooting)

> [!IMPORTANT]
> **Please complete these steps in order. Skipping or reordering them may lead to build errors or incomplete installations.**

## 0. Clone the Repository

The repository uses Git LFS (Large File Storage) for large binary assets (meshes, textures, etc.) and includes several third-party libraries as submodules. Both must be set up correctly before running the installer.

```bash
sudo apt install git-lfs
git clone -b dev --recurse-submodules https://github.com/<your-fork-or-org>/acsl-chrono-sim.git
cd ./acsl-chrono-sim/
git-lfs install
git-lfs pull
```

> [!NOTE]
> Replace the clone URL with your own fork or the lab's canonical repository as appropriate. The `--recurse-submodules` flag is required — it pulls in the `chrono`, `chrono-ros-messages` and other submodules that later steps depend on.

## 1. Setup the Required Packages using installer.sh

> [!CAUTION]
> This script requires **root access** and installs packages directly onto your system. **Never run scripts from the internet without reviewing them first.**
>
> By executing `installer.sh`, you acknowledge that you do so at your own discretion and risk.
> **The Advanced Control Systems Lab (ACSL) is not responsible for any damage, misconfiguration, or data loss resulting from the use of this script.**
>
> **We strongly recommend opening** `scripts/installer.sh` in a text editor and inspecting its contents before running it.

This step installs all dependencies required for both Chrono and the ACSL Chrono Simulator, including VSG (Vulkan-based rendering) and its dependency chain. The script also sets up environment variables and system configurations.

**Instructions:**

```bash
cd ./scripts/
chmod +x ./installer.sh
sudo ./installer.sh
```

> [!NOTE]
> - This script installs common build tools (`cmake`, `g++`, `python3`, etc.), graphics dependencies (OpenGL, Vulkan, GLFW), and third-party libraries needed for Chrono — including building VulkanSceneGraph (VSG) from source under `libraries/third-party/vsg-install/`.
> - The installation may take a while depending on your system and internet speed — building the VSG dependency chain in particular can take a significant amount of time.

## 2. Compile the Chrono Physics Engine

Once dependencies are installed, source your ROS2 environment and the Chrono ROS message package, then configure and build Chrono itself:

```bash
cd ../libraries
source /opt/ros/humble/setup.bash
source chrono-ros-messages/install/local_setup.bash
cd chrono-build/
ccmake ../chrono/
```

`installer.sh` pre-seeds the CMake cache with the module flags and third-party paths below (including `vsg_DIR`, `vsgXchange_DIR`, and `vsgImGui_DIR`, pointed at `libraries/third-party/vsg-install/`), so they should already show as set when `ccmake` opens. Confirm the following are set as shown, then proceed:

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
| `ENABLE_MODULE_VSG`         | ON      |

> [!TIP]
> Set any settings that aren't already populated one at a time — press `space bar` to toggle a setting, then `c` to reconfigure. Repeat `c` until no more red/changed entries remain, then press `g` to generate.

```bash
    -> press 'c' to configure
    -> press 'e' to exit (if prompted with configure output)
    -> press 'c' to configure again (to confirm no further changes)
    -> press 'e' to exit
    -> press 'g' to generate
```

Once configuration is generated, compile:

```bash
make -j$(nproc)
```

> [!NOTE]
> Upon successful build, the example programs from Project Chrono will be placed in `libraries/chrono-build/bin/`, including VSG demos (`demo_VSG_*`). Running one of these is a good way to confirm the VSG module built and links correctly before moving on.

## 3. Compile the ACSL Chrono Simulator

Once Chrono is compiled, build the ACSL Chrono Simulator itself:

```bash
cd ../../build/
ccmake ..
    -> press 'c' to configure
    -> press 'g' to generate
make -j$(nproc)
```

Once built, run the simulator:

```bash
./acsl_sim
```

## WSL2 Notes

If you're developing on Windows via WSL2 (Ubuntu), a few platform-specific quirks apply. These don't affect native Linux installs.

<!-- - **Segfault when Irrlicht/VSG windows try to render:** WSL2's default GPU pass-through (via Mesa's D3D12 translation layer) can be unstable for real-time 3D rendering. If the simulator crashes with a `D3D12: Removing Device` message followed by a segfault, try running with software rendering forced:
  ```bash
  LIBGL_ALWAYS_SOFTWARE=1 ./acsl_sim
  ``` -->
<!-- - **Blank/white render window after forcing software rendering:** this is a known WSLg compositor quirk. Restarting the WSL session usually clears it — from **PowerShell** (not inside WSL):
  ```powershell
  wsl --shutdown
  ```
  then reopen your WSL terminal. -->
- **Configure .wslconfig (memory + vsyscall)** Why. WSL2 runs Linux inside a lightweight VM with its own resource limits and kernel command line, separate from Windows itself, and you might run into two potential problems:

  WSL2 defaults to allocating only ~50% of your total RAM to the Linux VM, with no swap file. Compiling the full codebase is a memory-hungry process, and sometimes, the default allocation is often not enough, causing the VM to OOM mid-build with cascading, confusing errors.

  Modern WSL2 kernels disable the legacy vsyscall syscall mechanism by default (vsyscall=none) for security. Some tools are old statically-linked binaries that still rely on vsyscall. Without emulation enabled, they segfault instantly on launch.

  To fix this:

  * Open PowerShell (not WSL) and create/edit the file directly:

  ```bash
  notepad $env:UserProfile\.wslconfig
  ```

  **Note** %UserProfile% is the Windows equivalent of $HOME — it expands to C:\Users\<you>. This command opens (or creates) C:\Users\<you>\.wslconfig in Notepad.

  <!-- 
    memory=28GB
    processors=20
    swap=16GB
    swapFile=C:\\wsl-swap.vhdx
  -->
  * Paste in:
  ```bash
  ini   [wsl2]
  memory=28GB
  swap=16GB
  kernelCommandLine = vsyscall=emulate
  ```

  * Adjust memory/processors to leave some headroom for Windows itself — don't allocate 100% of your system RAM/cores.

  * Save and close Notepad.
  Restart the WSL2 VM so the new config takes effect:

  ```bash
  wsl --shutdown
  ```

  * Reopen your Ubuntu terminal. Verify the kernel picked up the vsyscall setting:

  ```bash
  cat /proc/cmdline
  ```

  You should see vsyscall=emulate in the output.
- **Hardware-accelerated Vulkan (VSG) requires a newer Mesa than Ubuntu 22.04 ships by default.** Ubuntu 22.04's stock `mesa-vulkan-drivers` package does not include the `dzn` (Vulkan-over-D3D12) driver needed for WSL2 GPU acceleration. To get it:
  ```bash
  sudo add-apt-repository ppa:kisak/turtle
  sudo apt update
  sudo apt upgrade
  ```
  Verify with:
  ```bash
  vulkaninfo --summary | grep -i deviceName
  ```
  You should see your actual GPU listed (e.g. `Microsoft Direct3D12 (NVIDIA GeForce ...)`) rather than only `llvmpipe`.
  > [!CAUTION]
  > This adds a third-party PPA and upgrades system-wide Mesa packages, which also affects your OpenGL/Irrlicht rendering path. This is **not** included in `installer.sh` automatically — it's a deliberate, manual system change. Review the PPA before adding it, and consider testing your Irrlicht-based build still works correctly afterward.
- **On laptops with both an integrated and discrete GPU** (common on Windows gaming/workstation laptops), Mesa's `dzn` driver may default to the integrated GPU. Force the discrete GPU explicitly:
  ```bash
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA   # or "AMD", matching your discrete GPU vendor
  ```
  Add this to `~/.bashrc` to make it persistent once confirmed working.
- **Even with all of the above, expect VSG performance on WSL2 to be modest compared to native Linux or Windows.** The `dzn` driver is explicitly flagged by Mesa as non-conformant/testing-only, and carries real CPU-side translation overhead per Vulkan call independent of GPU headroom. This is a platform limitation, not a misconfiguration — for best VSG/Vulkan performance, native Linux or native Windows builds will outperform WSL2.

## 🛠️ Troubleshooting

- **Compilation errors?** Double-check that all dependencies were successfully installed via `installer.sh`. Debug with the generated log file in `scripts/cache/installer-log.log`.
- **Chrono build issues?** Check the [google group](https://groups.google.com/g/projectchrono?pli=1) maintained by Project Chrono.
- **VSG-specific build issues (version mismatches between `vsg`, `vsgXchange`, and `vsgImGui`)?** These libraries are under active development and their latest tags aren't always mutually compatible, nor necessarily compatible with the specific Chrono release you're building. `installer.sh` pins known-working versions for this repository's Chrono version — if you modify `buildVSG.sh`'s version pins, verify compatibility carefully, as mismatches typically show up as `find_package(vsg)` version errors or missing-symbol compile errors deep in `chrono_vsg` source files.
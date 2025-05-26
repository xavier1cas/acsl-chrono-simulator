# Advanced Control Systems Lab Physics Simulator

[![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE)

## 📚 Overview

This codebase is a C++ implementation of Project Chrono that provides a high-fidelity physics engine to perform model-in-the-loop, software-in-the-loop, and hardware-in-the-loop simulations with the freeware, open-source, PX4-compatible [**ACSL Flightstack Winged**](https://github.com/andrealaffly/ACSL-flightstack-winged). The codebase leverages the Chrono simulation environment to test various **GNC** (Guidance, Navigation and Control) algorithms prior to real-world flights, enabling debugging, optimization, and tuning of algorithms formulated at the Advanced Control Systems Lab (ACSL) at Virginia Tech.

### System Requirements

- **RAM:** At least 16 GB
- **GPU:** Nvidia
- **Operating System:** Ubuntu 20.04 or above

#### Current Setup

| **Component**             | **Details**                                |
|---------------------------|--------------------------------------------|
| **CPU**                   | AMD Ryzen 5 9600X                          |
| **GPU**                   | Nvidia RTX 3080 (10 GB)                    |
| **Operating System**      | Ubuntu 24.04                               |
| **RAM**                   | 32 GB                                      |

## 💻 How to Use the Repository

### 1. Clone the repository

To clone this repo with all the needed submodules run the command:

```bash
git clone --recurse-submodules https://github.com/girimugundankumar/acsl-physics-sim.git
```

Update all the submodules to the stable version

```bash
# Navigate into the simulator directory
cd <path-to-repository>/acsl-physics-sim

# Update the submodules
git submodule update --init --recursive
```

### 2. Build the dependencies

After cloning the project and all its dependencies in the previous step, build the dependecies for the project [here](manual/instructions/chrono-installation.md).

### Opitional - Solidworks Add-in

After cloning the project and all its dependcies in the previous step, follow the guide [here](manual/instructions/solidworks-plugin.md) for installation and usage instructions.

>[!NOTE]
> You will need the solidworks plugin to import new UAV platforms for use in the simulator.

## 📝 License

This project is licensed under the BSD 3-Clause License. See the [![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](LICENSE) file for details. The Chrono Project is licensed under the BSD 3-Clause License. See the [![BSD License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/projectchrono/chrono/blob/9.0.1/LICENSE) file for more details.
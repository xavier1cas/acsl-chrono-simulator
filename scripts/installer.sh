#!/bin/bash

set -x  # Enable debug mode

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo"
    exit 1
fi

update_system() {
    echo "Updating and upgrading the system..."
    if command -v apt &> /dev/null; then
        apt update && apt upgrade -y | tee /dev/tty
    elif command -v dnf &> /dev/null; then
        dnf upgrade -y | tee /dev/tty
    elif command -v yum &> /dev/null; then
        yum update -y | tee /dev/tty
    elif command -v pacman &> /dev/null; then
        pacman -Syu --noconfirm | tee /dev/tty
    else
        echo "Unsupported package manager. Skipping update."
    fi
}

install_whiptail() {
    echo "Installing whiptail..."
    if command -v apt &> /dev/null; then
        apt install -y whiptail | tee /dev/tty
    elif command -v dnf &> /dev/null; then
        dnf install -y newt | tee /dev/tty
    elif command -v yum &> /dev/null; then
        yum install -y newt | tee /dev/tty
    elif command -v pacman &> /dev/null; then
        pacman -Sy --noconfirm libnewt | tee /dev/tty
    else
        echo "Unsupported package manager. Please install whiptail manually."
        exit 1
    fi
}

ensure_whiptail() {
    if ! command -v whiptail &> /dev/null; then
        install_whiptail
        if ! command -v whiptail &> /dev/null; then
            echo "Failed to install whiptail. Exiting."
            exit 1
        fi
    fi
}

show_license() {
    LICENSE_PATH="$(realpath $(dirname "$0")/../LICENSE)"
    if [ ! -f "$LICENSE_PATH" ]; then
        whiptail --title "Error" --msgbox "License file not found at: $LICENSE_PATH" 10 60
        exit 1
    fi
    LICENSE_CONTENT=$(tr -d '\r' < "$LICENSE_PATH")
    whiptail --title "Advanced Control Systems Lab Physics Simulator" \
        --scrolltext \
        --yesno "$LICENSE_CONTENT\n\nDo you accept the license agreement?" \
        25 80 \
        --yes-button "I Accept" \
        --no-button "I Decline"
    if [ $? -ne 0 ]; then
        whiptail --title "License Declined" --msgbox "Installation cannot continue without accepting the license agreement." 10 60
        exit 1
    fi
}

check_packages() {
    ORIG_DIR=$(pwd)
    output=""

    if [ -f /etc/os-release ] && grep -q 'Ubuntu' /etc/os-release; then
        UBUNTU_VERSION=$(lsb_release -rs)
        output+="Operating System: Ubuntu ${UBUNTU_VERSION}\n\n"
        case $UBUNTU_VERSION in
            "20.04") ROS2_RECOMMENDED="galactic" ;;
            "22.04") ROS2_RECOMMENDED="humble" ;;
            "24.04") ROS2_RECOMMENDED="jazzy" ;;
            *) ROS2_RECOMMENDED="not recommended for this Ubuntu version" ;;
        esac
        if [ -d "/opt/ros/${ROS2_RECOMMENDED}" ]; then
            ROS2_INSTALLED="Installed"
        else
            ROS2_INSTALLED="Not installed"
        fi
        output+="ROS2 (recommended version: ${ROS2_RECOMMENDED^}): ${ROS2_INSTALLED}\n\n"
    else
        output+="Operating System: Not Ubuntu\nROS2 check skipped\n\n"
    fi

    output+="Package presence check results:\n\n"

    # Python3
    if command -v python3 &> /dev/null; then
        output+="Python3: Installed\n"; PYTHON3_FLAG=OFF
    else
        output+="Python3: Not installed\n"; PYTHON3_FLAG=ON
    fi

    # pip3
    if command -v pip3 &> /dev/null; then
        output+="pip3: Installed\n"; PIP3_FLAG=OFF
    else
        output+="pip3: Not installed\n"; PIP3_FLAG=ON
    fi

    # GCC
    if command -v gcc &> /dev/null && command -v g++ &> /dev/null; then
        output+="GCC: Installed\n"; GCC_FLAG=OFF
    else
        output+="GCC: Not installed\n"; GCC_FLAG=ON
    fi

    # Clang
    if command -v clang &> /dev/null; then
        output+="Clang: Installed\n"; CLANG_FLAG=OFF
    else
        output+="Clang: Not installed\n"; CLANG_FLAG=ON
    fi

    # CMake
    if command -v cmake &> /dev/null; then
        output+="CMake: Installed\n"; CMAKE_FLAG=OFF
    else
        output+="CMake: Not installed\n"; CMAKE_FLAG=ON
    fi

    # Eigen3
    if pkg-config --exists eigen3 &> /dev/null; then
        output+="Eigen3: Installed\n"; EIGEN3_FLAG=OFF
    else
        output+="Eigen3: Not installed\n"; EIGEN3_FLAG=ON
    fi

    # Irrlicht
    if command -v dpkg &> /dev/null && dpkg -l | grep -q irrlicht; then
        output+="Irrlicht: Installed (via dpkg)\n"; IRRLICHT_FLAG=OFF
    elif pkg-config --exists irrlicht &> /dev/null; then
        output+="Irrlicht: Installed\n"; IRRLICHT_FLAG=OFF
    else
        output+="Irrlicht: Not installed\n"; IRRLICHT_FLAG=ON
    fi

    # Blaze
    if ls /usr/local/include/ | grep -q blaze; then
        output+="Blaze: Installed\n"; BLAZE_FLAG=OFF
    else
        output+="Blaze: Not installed\n"; BLAZE_FLAG=ON
    fi

    # Boost
    if pkg-config --exists boost &> /dev/null || [ -f "/usr/include/boost/version.hpp" ] || [ -f "/usr/local/include/boost/version.hpp" ]; then
        output+="Boost: Installed\n"; BOOST_FLAG=OFF
    else
        output+="Boost: Not installed\n"; BOOST_FLAG=ON
    fi

    # GLFW
    if pkg-config --exists glfw3 &> /dev/null; then
        output+="GLFW: Installed\n"; GLFW_FLAG=OFF
    else
        output+="GLFW: Not installed\n"; GLFW_FLAG=ON
    fi

    # GLEW
    if pkg-config --exists glew &> /dev/null; then
        output+="GLEW: Installed\n"; GLEW_FLAG=OFF
    else
        output+="GLEW: Not installed\n"; GLEW_FLAG=ON
    fi

    # OpenGL
    if pkg-config --exists gl &> /dev/null; then
        output+="OpenGL: Installed\n"; OPENGL_FLAG=OFF
    else
        output+="OpenGL: Not installed\n"; OPENGL_FLAG=ON
    fi

    # OpenCASCADE
    if pkg-config --exists occt &> /dev/null; then
        output+="OpenCASCADE: Installed (version: $(pkg-config --modversion occt))\n"; OPENCASCADE_FLAG=OFF
    elif [ -f "/usr/lib/libTKernel.so" ] || [ -f "/usr/local/lib/libTKernel.so" ]; then
        output+="OpenCASCADE: Installed\n"; OPENCASCADE_FLAG=OFF
    else
        output+="OpenCASCADE: Not installed\n"; OPENCASCADE_FLAG=ON
    fi

    # Librealsense
    if [ -d "/usr/local/include/librealsense2" ] || [ -f "/usr/local/lib/librealsense2.so" ]; then
        output+="Librealsense: Installed\n"; LIBREALSENSE_FLAG=OFF
    else
        output+="Librealsense: Not installed\n"; LIBREALSENSE_FLAG=ON
    fi

    cd "$ORIG_DIR" || exit 1

    if ! whiptail --title "Package Presence Check" --yes-button "Proceed" --no-button "Cancel" \
        --yesno "$output" 25 80; then
        whiptail --title "Cancelled" --msgbox "Installation cancelled." 10 50
        exit 1
    fi
}

run_installer() {
    selected=$(whiptail --title "Advanced Control Systems Lab Physics Simulator" \
        --checklist "Configure pre-requisites for compiling Project Chrono\n\nSelect packages to install:" 22 70 14 \
        "Python3" " " $PYTHON3_FLAG \
        "pip3" " " $PIP3_FLAG \
        "GCC" " " $GCC_FLAG \
        "Clang" " " $CLANG_FLAG \
        "CMake" " " $CMAKE_FLAG \
        "Eigen3" " " $EIGEN3_FLAG \
        "Irrlicht" " " $IRRLICHT_FLAG \
        "Blaze" " " $BLAZE_FLAG \
        "Boost" " " $BOOST_FLAG \
        "GLFW" " " $GLFW_FLAG \
        "GLEW" " " $GLEW_FLAG \
        "OpenGL" " " $OPENGL_FLAG \
        "OpenCASCADE" " " $OPENCASCADE_FLAG \
        "Librealsense" " " $LIBREALSENSE_FLAG \
        --ok-button "Proceed" \
        3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        whiptail --title "Cancelled" --msgbox "Installation cancelled." 10 50
        exit 1
    fi

    ORIG_DIR=$(pwd)
    for choice in $selected; do
        choice="${choice//\"}"
        case "$choice" in
            "Python3")
                echo "Installing Python3..."
                sudo apt install -y python3 | tee /dev/tty
                ;;
            "pip3")
                echo "Installing pip3..."
                sudo apt install -y python3-pip | tee /dev/tty
                ;;
            "GCC")
                echo "Installing GCC (build-essential)..."
                sudo apt install -y build-essential | tee /dev/tty
                ;;
            "Clang")
                echo "Installing Clang..."
                sudo apt install -y clang | tee /dev/tty
                ;;
            "CMake")
                echo "Installing CMake from source..."
                cd ../libraries/third-party/CMake || exit 1
                ./bootstrap | tee /dev/tty
                make -j$(nproc) | tee /dev/tty
                sudo make install | tee /dev/tty
                cd "$ORIG_DIR" || exit 1
                ;;
            "Eigen3")
                echo "Installing Eigen3..."
                sudo apt install -y libeigen3-dev | tee /dev/tty
                ;;
            "Irrlicht")
                echo "Installing Irrlicht..."
                sudo apt install -y libirrlicht1.8 libirrlicht-dev libirrlicht-doc | tee /dev/tty
                ;;
            "Blaze")
                echo "Installing Blaze headers..."
                sudo cp -r ../libraries/third-party/blaze/ /usr/local/include/ | tee /dev/tty
                ;;
            "Boost")
                echo "Installing Boost..."
                BOOST_DIR=../libraries/third-party/boost/
                cd "$BOOST_DIR" || exit 1
                ./bootstrap.sh | tee /dev/tty
                ./b2 --build-type=complete -j$(nproc) | tee /dev/tty
                sudo ./b2 install -j$(nproc) | tee /dev/tty
                cd "$ORIG_DIR" || exit 1
                ;;
            "GLFW")
                echo "Installing GLFW..."
                sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev | tee /dev/tty
                if [ "$XDG_SESSION_TYPE" = "x11" ]; then
                    sudo apt install -y xorg-dev | tee /dev/tty
                fi
                ;;
            "GLEW")
                echo "Installing GLEW..."
                sudo apt install -y libglew-dev libglu1-mesa-dev | tee /dev/tty
                if [ "$XDG_SESSION_TYPE" = "x11" ]; then
                    sudo apt install -y xorg-dev | tee /dev/tty
                fi
                ;;
            "OpenGL")
                echo "Installing OpenGL..."
                sudo apt install -y libglu1-mesa-dev freeglut3-dev mesa-common-dev | tee /dev/tty
                ;;
            "OpenCASCADE")
                echo "Installing OpenCASCADE dependencies..."
                sudo apt-get update && sudo apt-get install -y \
                    libtool autoconf automake gfortran gdebi \
                    gcc-multilib libxi-dev libxmu-dev libxmu-headers \
                    libx11-dev mesa-common-dev libglu1-mesa-dev \
                    libfontconfig1-dev \
                    libfreetype6 libfreetype6-dev \
                    tcl tcl-dev tk tk-dev | tee /dev/tty

                OCCT_DIR="../libraries/third-party/opencascade-7.4.0/"
                if [ ! -d "$OCCT_DIR" ]; then
                    echo "OpenCASCADE source directory not found: $OCCT_DIR"
                    exit 1
                fi
                TMP_BUILD_DIR="$OCCT_DIR/build"
                mkdir -p "$TMP_BUILD_DIR"
                cd "$TMP_BUILD_DIR" || { echo "Failed to change directory to $TMP_BUILD_DIR"; exit 1; }
                cmake .. | tee /dev/tty
                make -j$(nproc) | tee /dev/tty
                sudo make install | tee /dev/tty
                cd "$ORIG_DIR" || { echo "Failed to return to original directory"; exit 1; }
                rm -rf "$TMP_BUILD_DIR" | tee /dev/tty
                ;;
            "Librealsense")
                echo "Installing Librealsense and dependencies..."
                sudo apt install -y libusb-1.0-0-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libssl-dev | tee /dev/tty
                LIBREALSENSE_DIR="../libraries/third-party/librealsense-2.53.1/"
                if [ ! -d "$LIBREALSENSE_DIR" ]; then
                    echo "Librealsense source directory not found: $LIBREALSENSE_DIR"
                    exit 1
                fi
                cd "$LIBREALSENSE_DIR" || exit 1
                mkdir -p build && cd build
                cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release | tee /dev/tty
                make -j$(nproc) | tee /dev/tty
                sudo make install | tee /dev/tty
                cd "$ORIG_DIR" || exit 1
                rm -rf "$LIBREALSENSE_DIR/build" | tee /dev/tty
                sudo cp "$LIBREALSENSE_DIR/config/99-realsense-libusb.rules" /etc/udev/rules.d/ | tee /dev/tty
                sudo udevadm control --reload-rules && sudo udevadm trigger | tee /dev/tty
                ;;
        esac
    done
}

# -------------------- Main Script Execution --------------------
update_system
ensure_whiptail
show_license
check_packages
run_installer

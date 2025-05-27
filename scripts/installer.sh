#!/bin/bash

set -x  # Enable debug mode

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo"
    exit 1
fi

# Function to update and upgrade the system
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

# Function to install whiptail
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

# Function to check and install whiptail
ensure_whiptail() {
    if ! command -v whiptail &> /dev/null; then
        install_whiptail
        if ! command -v whiptail &> /dev/null; then
            echo "Failed to install whiptail. Exiting."
        exit 1
        fi
    fi
}

# Function to display license agreement
show_license() {
    # Get absolute path to license file
    LICENSE_PATH="$(realpath $(dirname $0)/../LICENSE)"
    if [ ! -f "$LICENSE_PATH" ]; then
        whiptail --title "Error" --msgbox "License file not found at: $LICENSE_PATH" 10 60
        exit 1
    fi

    # Read license content and remove carriage returns
    LICENSE_CONTENT=$(cat "$LICENSE_PATH" | tr -d '\r')

    # Display license agreement
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

# Function to check for package presence and show recommendations
check_packages() {
    # Save current directory
    ORIG_DIR=$(pwd)
    # Initialize output
    output="Package presence check results:\n\n"

    # GCC
    if command -v gcc &> /dev/null && command -v g++ &> /dev/null; then
        output+="GCC: Installed\n"
    else
        output+="GCC: Not installed (recommend installing build-essential)\n"
    fi

    # Clang
    if command -v clang &> /dev/null; then
        output+="Clang: Installed\n"
    else
        output+="Clang: Not installed\n"
    fi

    # CMake
    if command -v cmake &> /dev/null; then
        output+="CMake: Installed\n"
    else
        output+="CMake: Not installed\n"
    fi

    # Eigen3
    if pkg-config --exists eigen3 &> /dev/null; then
        output+="Eigen3: Installed\n"
    else
        output+="Eigen3: Not installed\n"
    fi

    # Irrlicht
    if command -v dpkg &> /dev/null && dpkg -l | grep -q irrlicht; then
        output+="Irrlicht: Installed (via dpkg)\n"
    elif pkg-config --exists irrlicht &> /dev/null; then
        output+="Irrlicht: Installed\n"
    else
        output+="Irrlicht: Not installed\n"
    fi

    # Blaze (check if present in /usr/local/include)
    if ls /usr/local/include/ | grep -q blaze; then
        output+="Blaze: Installed\n"
    else
        output+="Blaze: Not installed\n"
    fi

    # Boost (check system installation, do not output version)
    if pkg-config --exists boost &> /dev/null || \
       [ -f "/usr/include/boost/version.hpp" ] || \
       [ -f "/usr/local/include/boost/version.hpp" ]; then
        output+="Boost: Installed\n"
    else
        output+="Boost: Not installed\n"
    fi

    # GLFW
    if pkg-config --exists glfw3 &> / dev/null; then
        output+="GLFW: Installed\n"
    else
        output+="GLFW: Not installed\n"
    fi

    # GLEW
    if pkg-config --exists glew &> /dev/null; then
        output+="GLEW: Installed\n"
    else
        output+="GLEW: Not installed\n"
    fi

    # OpenGL
    if pkg-config --exists gl &> /dev/null; then
        output+="OpenGL: Installed\n"
    else
        output+="OpenGL: Not installed\n"
    fi

    # OpenCASCADE (check system libraries)
    if pkg-config --exists occt &> /dev/null; then
        output+="OpenCASCADE: Installed (version: $(pkg-config --modversion occt))\n"
    elif [ -f "/usr/lib/libTKernel.so" ] || [ -f "/usr/local/lib/libTKernel.so" ]; then
        output+="OpenCASCADE: Installed\n"
    else
        output+="OpenCASCADE: Not installed\n"
    fi

    # Return to original directory
    cd "$ORIG_DIR" || { echo "Failed to return to original directory"; exit 1; }

    # Display results with "Proceed" button
    if whiptail --title "Package Presence Check" --yes-button "Proceed" --no-button "Cancel" \
        --yesno "$output" 25 80; then
        :
    else
        whiptail --title "Cancelled" --msgbox "Installation cancelled." 10 50
        exit 1
    fi
}

# Main installer function
run_installer() {
    selected=$(whiptail --title "Advanced Control Systems Lab Physics Simulator" \
        --checklist "Configure pre-requisites for compiling Project Chrono\n\nSelect packages to install:" 20 70 11 \
        "GCC" " " OFF \
        "Clang" " " OFF \
        "CMake" " " OFF \
        "Eigen3" " " OFF \
        "Irrlicht" " " OFF \
        "Blaze" " " OFF \
        "Boost" " " OFF \
        "GLFW" " " OFF \
        "GLEW" " " OFF \
        "OpenGL" " " OFF \
        "OpenCASCADE" " " OFF \
        --ok-button "Proceed" \
        3>&1 1>&2 2>&3)

    if [ $? -ne 0 ]; then
        whiptail --title "Cancelled" --msgbox "Installation cancelled." 10 50
        exit 1
    fi

    # Save current directory
    ORIG_DIR=$(pwd)

    # Process selections
    for choice in $selected; do
        choice="${choice//\"}"
        case "$choice" in
            "GCC")
                echo "Installing GCC (build-essential)..."
                sudo apt install build-essential -y | tee /dev/tty
                ;;
            "Clang")
                echo "Installing Clang..."
                sudo apt-get install clang -y | tee /dev/tty
                ;;
            "CMake")
                echo "Installing CMake from source..."
                cd ../libraries/third-party/CMake || { echo "Failed to change directory"; exit 1; }
                ./bootstrap | tee /dev/tty
                make -j$(nproc) | tee /dev/tty
                sudo make install | tee /dev/tty
                cd "$ORIG_DIR" || { echo "Failed to return to original directory"; exit 1; }
                ;;
            "Eigen3")
                echo "Installing Eigen3..."
                sudo apt install libeigen3-dev -y | tee /dev/tty
                ;;
            "Irrlicht")
                echo "Installing Irrlicht..."
                sudo apt-get install libirrlicht1.8 libirrlicht-dev libirrlicht-doc -y | tee /dev/tty
                ;;
            "Blaze")
                echo "Installing Blaze headers..."
                sudo cp -r ../libraries/third-party/blaze/ /usr/local/include/ | tee /dev/tty
                ;;
            "Boost")
                echo "Installing Boost..."
                BOOST_DIR=../libraries/third-party/boost/
                if [ ! -d "$BOOST_DIR" ]; then
                    echo "Boost directory not found: $BOOST_DIR"
                    exit 1
                fi
                cd "$BOOST_DIR" || { echo "Failed to change directory to $BOOST_DIR"; exit 1; }
                ./bootstrap.sh | tee /dev/tty
                ./b2 --build-type=complete -j$(nproc) | tee /dev/tty
                sudo ./b2 install -j$(nproc) | tee /dev/tty
                cd "$ORIG_DIR" || { echo "Failed to return to original directory"; exit 1; }
                ;;
            "GLFW")
                echo "Installing GLFW and supporting packages..."
                sudo apt install libglfw3-dev libglu1-mesa-dev -y | tee /dev/tty
                if [ "$XDG_SESSION_TYPE" = "x11" ]; then
                    echo "X11 detected. Installing xorg-dev..."
                    sudo apt install xorg-dev -y | tee /dev/tty
                else
                    echo "Not running X11. Skipping xorg-dev installation."
                fi
                ;;
            "GLEW")
                echo "Installing GLEW and supporting packages..."
                sudo apt install libglew-dev libglu1-mesa-dev -y | tee /dev/tty
                if [ "$XDG_SESSION_TYPE" = "x11" ]; then
                    echo "X11 detected. Installing xorg-dev..."
                    sudo apt install xorg-dev -y | tee /dev/tty
                else
                    echo "Not running X11. Skipping xorg-dev installation."
                fi
                ;;
            "OpenGL")
                echo "Installing OpenGL and supporting packages..."
                sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev -y | tee /dev/tty
                if [ "$XDG_SESSION_TYPE" = "x11" ]; then
                    echo "X11 detected. Installing xorg-dev..."
                    sudo apt install xorg-dev -y | tee /dev/tty
                else
                    echo "Not running X11. Skipping xorg-dev installation."
                fi
                ;;
            "OpenCASCADE")
                echo "Installing OpenCASCADE dependencies..."
                sudo apt-get update && sudo apt-get install -y \
                    libtool autoconf automake gfortran gdebi \
                    gcc-multilib libxi-dev libxmu-dev libx11-dev \
                    mesa-common-dev libglu1-mesa-dev \
                    libfontconfig1_dev \
                    libfreetype6 libfreetype6-dev \
                    tcl tcl-dev tk tk-dev | tee /dev/tty

                echo "Building OpenCASCADE from source..."
                OCCT_DIR="../libraries/third-party/opencascade-7.4.0/"
                if [ ! -d "$OCCT_DIR" ]; then
                    echo "OpenCASCADE source directory not found: $OCCT_DIR"
                    exit 1
                fi
                cd "$OCCT_DIR" || { echo "Failed to change directory to $OCCT_DIR"; exit 1; }
                mkdir -p build && cd build
                cmake .. | tee /dev/tty
                sudo make -j$(nproc) | tee /dev/tty
                sudo make install | tee /dev/tty
                cd "$ORIG_DIR" || { echo "Failed to return to original directory"; exit 1; }

                # Cleanup build directory
                echo "Removing build folder..."
                rm -rf "$OCCT_DIR/build" | tee /dev/tty
                ;;
        esac
    done
}

# -------------------- Main Script Execution --------------------
# Update system first
update_system

# Ensure whiptail is installed
ensure_whiptail

# Show license agreement after system updates and whiptail checks
show_license

# Check for package presence and show recommendations
check_packages

# Start the installer
run_installer

#!/bin/bash

set -x # Enable debug mode

# Dynamically set whiptail size to fit terminal
read WHIPTAIL_ROWS WHIPTAIL_COLS < <(stty size)

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
        whiptail --title "Error" --msgbox "License file not found at: $LICENSE_PATH" $WHIPTAIL_ROWS $WHIPTAIL_COLS
        exit 1
    fi
    LICENSE_CONTENT=$(tr -d '\r' < "$LICENSE_PATH")
    whiptail --title "Advanced Control Systems Lab Physics Simulator" \
        --scrolltext \
        --yesno "$LICENSE_CONTENT\n\nDo you accept the license agreement?" \
        $WHIPTAIL_ROWS $WHIPTAIL_COLS \
        --yes-button "I Accept" \
        --no-button "I Decline"
    if [ $? -ne 0 ]; then
        whiptail --title "License Declined" --msgbox "Installation cannot continue without accepting the license agreement." $WHIPTAIL_ROWS $WHIPTAIL_COLS
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
        ROS2_RECOMMENDED=""
        ROS2_INSTALLED="Not installed"
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

    export ROS2_INSTALLED
    export ROS2_RECOMMENDED

    if ! whiptail --title "Package Presence Check" --yes-button "Proceed" --no-button "Cancel" \
        --yesno "$output" $WHIPTAIL_ROWS $WHIPTAIL_COLS; then
        whiptail --title "Cancelled" --msgbox "Installation cancelled." $WHIPTAIL_ROWS $WHIPTAIL_COLS
        exit 1
    fi
}

run_installer() {
    selected=$(whiptail --title "Advanced Control Systems Lab Physics Simulator" \
        --checklist "Configure pre-requisites for compiling Project Chrono\n\nSelect packages to install:" $WHIPTAIL_ROWS $WHIPTAIL_COLS 14 \
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
        whiptail --title "Cancelled" --msgbox "Installation cancelled." $WHIPTAIL_ROWS $WHIPTAIL_COLS
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

install_ros2() {
    UBUNTU_VERSION=$(lsb_release -rs)
    case $UBUNTU_VERSION in
        "20.04") ROS2_VERSION="galactic" ;;
        "22.04") ROS2_VERSION="humble" ;;
        "24.04") ROS2_VERSION="jazzy" ;;
        *)
            whiptail --title "ROS2 Install" --msgbox "No recommended ROS2 version for Ubuntu $UBUNTU_VERSION. Skipping ROS2 install." $WHIPTAIL_ROWS $WHIPTAIL_COLS
            return
            ;;
    esac

    sudo apt update
    sudo apt install -y curl gnupg2 lsb-release | tee /dev/tty
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update && sudo apt upgrade -y | tee /dev/tty

    if [ "$ROS2_VERSION" = "galactic" ]; then
        sudo apt install -y ros-galactic-desktop python3-argcomplete ros-dev-tools | tee /dev/tty
    elif [ "$ROS2_VERSION" = "humble" ]; then
        sudo apt install -y ros-humble-desktop python3-argcomplete ros-dev-tools | tee /dev/tty
    elif [ "$ROS2_VERSION" = "jazzy" ]; then
        sudo apt install -y ros-jazzy-desktop python3-argcomplete ros-dev-tools | tee /dev/tty
    fi

    ROS2_VERSION_INSTALLED="$ROS2_VERSION"
    whiptail --title "ROS2 Install" --msgbox "ROS2 $ROS2_VERSION installation complete." $WHIPTAIL_ROWS $WHIPTAIL_COLS
}

ros2_bashrc_prompt() {
    if [ -n "$ROS2_VERSION_INSTALLED" ]; then
        local bashrc_line="\n# Source ROS2 $ROS2_VERSION_INSTALLED for every terminal instance\nsource /opt/ros/$ROS2_VERSION_INSTALLED/setup.bash"
        local warning="You can automatically source ROS2 $ROS2_VERSION_INSTALLED for every terminal by adding the following to your ~/.bashrc:\n\n$bashrc_line\n\n\
WARNING:\nIf you have multiple ROS2 versions installed, sourcing more than one in your ~/.bashrc can break your environment and cause unexpected issues. \
Only add this if you do not plan to use multiple ROS2 versions on this machine.\n\n\
If you are unsure, it is safer to source ROS2 manually in each terminal session."

        if whiptail --title "Add ROS2 to .bashrc?" --yesno "$warning" $WHIPTAIL_ROWS $WHIPTAIL_COLS; then
            if [ "$SUDO_USER" ]; then
                USER_BASHRC="/home/$SUDO_USER/.bashrc"
                sudo -u "$SUDO_USER" bash -c "grep -Fxq 'source /opt/ros/$ROS2_VERSION_INSTALLED/setup.bash' \"$USER_BASHRC\" || echo -e '$bashrc_line' >> \"$USER_BASHRC\""
            else
                USER_BASHRC="$HOME/.bashrc"
                grep -Fxq "source /opt/ros/$ROS2_VERSION_INSTALLED/setup.bash" "$USER_BASHRC" || echo -e "$bashrc_line" >> "$USER_BASHRC"
            fi
            whiptail --title "Done" --msgbox "Added to $USER_BASHRC:\n\n$bashrc_line" 12 80
        fi
    fi
}

ros2_prompt_and_install() {
    if [ "$ROS2_INSTALLED" = "Not installed" ] && [[ "$ROS2_RECOMMENDED" =~ ^(galactic|humble|jazzy)$ ]]; then
        if whiptail --title "ROS2 Installation" \
            --yesno "ROS2 ($ROS2_RECOMMENDED) is not installed. Would you like to install the recommended ROS2 version for your system now?\n\nThis will install the full desktop version and development tools." $WHIPTAIL_ROWS $WHIPTAIL_COLS; then
            install_ros2
            ros2_bashrc_prompt
        fi
    fi
}

# -------------------- Main Script Execution --------------------
update_system
ensure_whiptail
show_license
check_packages
run_installer
ros2_prompt_and_install

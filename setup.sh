#!/bin/bash

# Function to display usage
usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  --create-conda <env_name>  Create a new Conda environment with the specified name."
    echo "  --install                  Install dependencies and the project in the current environment."
    echo "  --help                     Display this help message."
}

# Check OS
check_os() {
    OS_NAME=$(uname -s)
    OS_VERSION=""

    if [[ "$OS_NAME" == "Linux" ]]; then
        if command -v lsb_release &>/dev/null; then
            OS_VERSION=$(lsb_release -rs)
        elif [[ -f /etc/os-release ]]; then
            . /etc/os-release
            OS_VERSION=$VERSION_ID
        fi
        # Supporting both 22.04 and 24.04
        if [[ "$OS_VERSION" != "22.04" && "$OS_VERSION" != "24.04" ]]; then
            echo "Warning: This script has only been tested on Ubuntu 22.04 and 24.04"
            echo "Your system is running Ubuntu $OS_VERSION."
            read -p "Do you want to continue anyway? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "Installation cancelled."
                exit 1
            fi
        fi
    else
        echo "Unsupported operating system: $OS_NAME"
        exit 1
    fi
    echo "Operating system check passed: $OS_NAME $OS_VERSION"
}

create_conda_env() {
    ENV_NAME=$1
    
    # Detect Python version
    if command -v python3 &>/dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1)
    elif command -v python &>/dev/null; then
        PYTHON_VERSION=$(python --version 2>&1)
    else
        echo "Python is not installed on this system."
        exit 1
    fi
    
    PYTHON_MAJOR_MINOR=$(echo $PYTHON_VERSION | grep -oP '\d+\.\d+')
    
    # Source conda
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/miniconda3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        . "$HOME/anaconda3/etc/profile.d/conda.sh"
    else
        # Try to rely on PATH
        if ! command -v conda &>/dev/null; then
             echo "Conda command not found. Please ensure Conda is installed and in your PATH."
             exit 1
        fi
    fi

    echo "Creating Conda environment '$ENV_NAME' with Python $PYTHON_MAJOR_MINOR..."
    # Deactivate any current env just in case
    source $(conda info --base)/etc/profile.d/conda.sh 2>/dev/null || true
    conda deactivate
    
    conda create -n "$ENV_NAME" python=$PYTHON_MAJOR_MINOR -y
    
    echo -e "\n[INFO] Created conda environment '$ENV_NAME'."
    echo "To proceed with installation:"
    echo "  1. conda activate $ENV_NAME"
    echo "  2. bash $0 --install"
}

install_dependencies() {
    echo "Starting installation..."

    # Check if running in Conda
    if [[ -n "$CONDA_PREFIX" ]]; then
        echo "Conda environment detected: $CONDA_DEFAULT_ENV"
        # Conda-specific fix for Linux
        if [[ "$(uname -s)" == "Linux" ]]; then
            echo "Installing libstdcxx-ng (Conda specific)..."
            conda install -c conda-forge libstdcxx-ng -y
        fi
    else
        echo "No Conda environment detected. Installing in current system/venv..."
    fi

    # Install uv for faster builds
    echo "Installing/Updating 'uv'..."
    pip install uv
    uv pip install --upgrade pip

    # Clean and recreate dependencies folder
    if [ -d "dependencies" ]; then
        echo "Cleaning existing dependencies folder..."
        rm -rf dependencies
    fi
    mkdir dependencies
    cd dependencies

    # XRoboToolkit
    echo "Cloning XRoboToolkit-PC-Service-Pybind..."
    git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
    cd XRoboToolkit-PC-Service-Pybind
    bash setup_ubuntu.sh
    cd ..

    # R5 SDK
    echo "Cloning R5 SDK..."
    git clone https://github.com/zhigenzhao/R5.git
    cd R5
    git checkout dev/python_pkg
    cd py/ARX_R5_python/
    echo "Installing R5 SDK..."
    uv pip install .
    cd ../../../..

    # Main Package
    echo "Installing xrobotoolkit_teleop..."
    uv pip install -e . || { echo "Failed to install xrobotoolkit_teleop"; exit 1; }

    echo -e "\n[SUCCESS] Installation complete!\n"
}

# Main logic
if [[ $# -eq 0 ]]; then
    usage
    exit 0
fi

check_os

while [[ $# -gt 0 ]]; do
    case $1 in
        --create-conda)
            if [[ -n "$2" ]]; then
                create_conda_env "$2"
                shift 2
            else
                echo "Error: --create-conda requires an environment name."
                exit 1
            fi
            ;;
        --install)
            install_dependencies
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

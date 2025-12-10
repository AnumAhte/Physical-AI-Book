---
sidebar_position: 2
title: Development Environment Setup
description: Complete guide to setting up your Physical AI development environment with Ubuntu, ROS 2, and essential tools
keywords: [ros2, ubuntu, setup, installation, development environment, gazebo, python]
---

# Development Environment Setup

<div className="chapter-meta">
  <div className="chapter-meta-item">
    <strong>Week:</strong> 1-2
  </div>
  <div className="chapter-meta-item">
    <strong>Duration:</strong> ~2 hours
  </div>
  <div className="chapter-meta-item">
    <strong>Level:</strong> Beginner
  </div>
</div>

## Prerequisites

<div className="prerequisites">

Before starting this setup, ensure you have:

- A computer with at least 16GB RAM (32GB recommended for simulation)
- 50GB+ free disk space
- A discrete GPU (NVIDIA recommended for Isaac Sim)
- Basic familiarity with command-line interfaces
- Internet connection for downloading packages

</div>

## Learning Objectives

<div className="learning-objectives">

By the end of this chapter, you will have:

- Installed Ubuntu 22.04 (native or WSL2)
- Set up ROS 2 Humble development environment
- Installed Python 3.10+ with essential libraries
- Configured VS Code for robotics development
- Verified your installation with a simple ROS 2 program

</div>

## System Requirements

### Minimum Requirements

| Component | Specification |
|-----------|---------------|
| OS | Ubuntu 22.04 LTS |
| CPU | 4+ cores, x86_64 |
| RAM | 16 GB |
| Storage | 50 GB SSD |
| GPU | Any (integrated OK for ROS 2) |

### Recommended for Isaac Sim

| Component | Specification |
|-----------|---------------|
| OS | Ubuntu 22.04 LTS |
| CPU | 8+ cores, recent Intel/AMD |
| RAM | 32 GB |
| Storage | 100 GB NVMe SSD |
| GPU | NVIDIA RTX 3080+ (16GB+ VRAM) |

## Step 1: Operating System Setup

### Option A: Native Ubuntu Installation

For the best performance, install Ubuntu 22.04 directly:

1. Download Ubuntu 22.04 LTS from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create a bootable USB using Balena Etcher
3. Boot from USB and follow installation wizard
4. Choose "Normal installation" with third-party drivers

### Option B: Windows Subsystem for Linux (WSL2)

If you're on Windows 11, WSL2 provides excellent compatibility:

```bash
# Open PowerShell as Administrator
wsl --install -d Ubuntu-22.04

# After restart, set Ubuntu as default
wsl --set-default Ubuntu-22.04

# Update WSL to latest version
wsl --update
```

For GUI applications (Gazebo, RViz), install WSLg support:

```bash
# In Windows PowerShell
wsl --update

# In Ubuntu WSL
sudo apt update && sudo apt upgrade -y
```

### Option C: Virtual Machine

Use VMware Workstation Pro or VirtualBox:

- Allocate at least 8 CPU cores
- Assign 16GB+ RAM
- Enable 3D acceleration
- Use VirtIO drivers for better performance

:::warning
Virtual machines have significantly reduced performance for simulation. Native installation or WSL2 is strongly recommended.
:::

## Step 2: System Updates and Dependencies

Open a terminal and run:

```bash
# Update package lists
sudo apt update

# Upgrade existing packages
sudo apt upgrade -y

# Install essential build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common

# Install Python dependencies
sudo apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev
```

## Step 3: Install ROS 2 Humble

ROS 2 Humble is the LTS release compatible with Ubuntu 22.04.

### Add ROS 2 Repository

```bash
# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Desktop

```bash
# Update package index
sudo apt update

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y ros-dev-tools
```

### Configure Shell Environment

Add ROS 2 to your shell configuration:

```bash
# For bash users
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# For zsh users
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

### Verify ROS 2 Installation

```bash
# Check ROS 2 version
ros2 --version
# Expected: ros2 0.x.x (Humble)

# Test with talker/listener demo
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2 (new terminal):
ros2 run demo_nodes_cpp listener
```

You should see messages being published and received.

## Step 4: Install Gazebo Simulation

Gazebo is the standard robotics simulator:

```bash
# Install Gazebo Fortress (recommended for Humble)
sudo apt install -y ros-humble-gazebo-ros-pkgs

# Install additional Gazebo tools
sudo apt install -y \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins
```

### Test Gazebo

```bash
# Launch empty Gazebo world
gazebo
```

If Gazebo opens with a 3D viewport, installation is successful.

## Step 5: Python Environment Setup

Create a dedicated Python environment for Physical AI development:

```bash
# Create project directory
mkdir -p ~/physical_ai_ws
cd ~/physical_ai_ws

# Create virtual environment
python3 -m venv .venv

# Activate environment
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install core packages
pip install \
    numpy \
    scipy \
    matplotlib \
    pandas \
    jupyter \
    ipykernel

# Install ML/AI packages
pip install \
    torch \
    torchvision \
    transformers \
    opencv-python \
    pillow

# Install robotics packages
pip install \
    pybullet \
    gymnasium \
    stable-baselines3
```

### Register Jupyter Kernel

```bash
# Make virtualenv available in Jupyter
python -m ipykernel install --user --name=physical_ai --display-name="Physical AI"
```

## Step 6: VS Code Setup

Visual Studio Code is the recommended IDE for robotics development.

### Install VS Code

```bash
# Download and install VS Code
sudo snap install code --classic
```

### Recommended Extensions

Install these extensions for the best experience:

```bash
# Install via command line
code --install-extension ms-python.python
code --install-extension ms-python.vscode-pylance
code --install-extension ms-vscode.cpptools
code --install-extension ms-vscode.cmake-tools
code --install-extension ms-iot.vscode-ros
code --install-extension redhat.vscode-xml
code --install-extension redhat.vscode-yaml
```

### Configure VS Code for ROS 2

Create workspace settings `.vscode/settings.json`:

```json
{
    "python.defaultInterpreterPath": "${workspaceFolder}/.venv/bin/python",
    "python.analysis.typeCheckingMode": "basic",
    "ros.distro": "humble",
    "files.associations": {
        "*.launch.py": "python",
        "*.urdf": "xml",
        "*.xacro": "xml",
        "*.sdf": "xml"
    },
    "editor.formatOnSave": true,
    "[python]": {
        "editor.defaultFormatter": "ms-python.black-formatter"
    }
}
```

## Step 7: Create Your First ROS 2 Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize workspace
colcon build

# Source the workspace
source install/setup.bash

# Add to shell configuration
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Create a Test Package

```bash
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_first_package

# Build
cd ~/ros2_ws
colcon build --packages-select my_first_package

# Source updated workspace
source install/setup.bash
```

## Step 8: Verify Complete Installation

Run this verification script to check all components:

```python
#!/usr/bin/env python3
"""Verify Physical AI development environment setup."""

import subprocess
import sys

def check_command(cmd, name):
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ {name}: OK")
            return True
        else:
            print(f"❌ {name}: Failed")
            return False
    except Exception as e:
        print(f"❌ {name}: Error - {e}")
        return False

def check_python_package(package):
    try:
        __import__(package)
        print(f"✅ Python {package}: OK")
        return True
    except ImportError:
        print(f"❌ Python {package}: Not installed")
        return False

print("=" * 50)
print("Physical AI Environment Verification")
print("=" * 50)

# System checks
check_command("ros2 --version", "ROS 2")
check_command("gazebo --version", "Gazebo")
check_command("python3 --version", "Python 3")
check_command("git --version", "Git")
check_command("cmake --version", "CMake")

print("\n" + "-" * 50)
print("Python Packages")
print("-" * 50)

# Python package checks
packages = ["numpy", "torch", "cv2", "transformers", "matplotlib"]
for pkg in packages:
    check_python_package(pkg)

print("\n" + "=" * 50)
print("Verification Complete!")
print("=" * 50)
```

Save as `verify_setup.py` and run:

```bash
python3 verify_setup.py
```

## Troubleshooting

### Common Issues

#### ROS 2 Commands Not Found

```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Add permanently
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Gazebo Display Issues (WSL2)

```bash
# Install X server dependencies
sudo apt install -y x11-apps mesa-utils

# Test display
glxgears
```

#### GPU Not Detected

```bash
# Check NVIDIA driver
nvidia-smi

# If not installed, add NVIDIA repository
sudo ubuntu-drivers autoinstall
sudo reboot
```

#### Permission Denied for USB Devices

```bash
# Add user to dialout group (for serial devices)
sudo usermod -aG dialout $USER

# Log out and back in for changes to take effect
```

## Summary

You now have a complete Physical AI development environment with:

- ✅ Ubuntu 22.04 operating system
- ✅ ROS 2 Humble with Gazebo simulation
- ✅ Python 3 with AI/ML libraries
- ✅ VS Code configured for robotics
- ✅ A ready-to-use ROS 2 workspace

In the next chapter, we will dive into ROS 2 fundamentals and build our first robot applications.

## Exercises

1. **Verification**: Run the verification script and resolve any issues.

2. **Exploration**: Launch `ros2 run turtlesim turtlesim_node` and control it with `ros2 run turtlesim turtle_teleop_key`.

3. **Documentation**: Document your installation process, noting any issues you encountered and how you resolved them.

# NVIDIA Isaac Sim Installation Prerequisites for Ubuntu 22.04

## System Requirements

### Hardware Requirements
- **CPU**: Intel Core i7 or AMD Ryzen 7 (8+ cores recommended)
- **RAM**: 32GB or more (64GB recommended for complex scenes)
- **GPU**: NVIDIA RTX 3070/4070 or better (12GB+ VRAM recommended)
  - For optimal performance: RTX 4090 or RTX 6000 Ada Generation
  - Compute Capability: 6.0 or higher
- **Storage**: 50GB+ free space for Isaac Sim installation
- **Display**: 4K display recommended for optimal workflow

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS (64-bit)
- **Kernel Version**: 5.15 or higher recommended
- **NVIDIA GPU Driver**: Version 535 or higher
- **CUDA**: 11.8 or 12.x (compatible with GPU driver)
- **Docker**: 20.10 or higher (optional but recommended)

## Prerequisites Installation Guide

### 1. Update System
```bash
sudo apt update && sudo apt upgrade -y
```

### 2. Install NVIDIA GPU Drivers
```bash
# Add graphics drivers PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install recommended driver
sudo ubuntu-drivers autoinstall

# Or install specific version
sudo apt install nvidia-driver-535 nvidia-utils-535
```

### 3. Verify GPU Installation
```bash
# Check GPU detection
nvidia-smi

# Verify CUDA installation
nvcc --version
```

### 4. Install Additional Dependencies
```bash
# Essential build tools
sudo apt install build-essential cmake pkg-config -y

# Graphics and display libraries
sudo apt install libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1 -y

# Audio libraries
sudo apt install pulseaudio alsa-utils -y

# Python dependencies
sudo apt install python3-dev python3-pip python3-venv -y
```

### 5. Install Docker (Optional but Recommended)
```bash
# Install Docker
sudo apt install docker.io docker-compose-v2 -y

# Add user to docker group
sudo usermod -aG docker $USER

# Start and enable Docker
sudo systemctl start docker
sudo systemctl enable docker
```

## Isaac Sim-Specific Prerequisites

### 1. Install Isaac Sim Dependencies
```bash
# Install Isaac Sim-specific packages
sudo apt install libavcodec-dev libavformat-dev libavutil-dev libswscale-dev -y
sudo apt install libglu1-mesa-dev freeglut3-dev mesa-common-dev -y
sudo apt install libusb-1.0-0-dev libudev-dev -y
```

### 2. Configure System for Isaac Sim
```bash
# Increase shared memory limit (important for Isaac Sim)
echo "tmpfs /dev/shm tmpfs defaults,size=8G 0 0" | sudo tee -a /etc/fstab
sudo mount -o remount,size=8G /dev/shm

# Verify shared memory
df -h /dev/shm
```

### 3. Install Python Dependencies
```bash
# Create virtual environment for Isaac Sim
python3 -m venv ~/isaac_env
source ~/isaac_env/bin/activate
pip install --upgrade pip

# Install required Python packages
pip install numpy scipy matplotlib opencv-python
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Troubleshooting Common Issues

### GPU Not Detected
- Ensure GPU is properly seated and powered
- Check that correct drivers are installed
- Verify kernel module is loaded: `lsmod | grep nvidia`

### CUDA Installation Issues
- Verify compatibility between driver and CUDA version
- Check for conflicting CUDA installations
- Use `nvidia-ml-py3` for Python NVIDIA management

### Display Issues
- Ensure proper display drivers are installed
- Check X11 forwarding if using remote desktop
- Verify OpenGL support: `glxinfo | grep "OpenGL renderer"`

## Network Configuration (If Using Remote Access)

### SSH with X11 Forwarding
```bash
# Install X11 forwarding support
sudo apt install xorg openbox -y

# Connect with X11 forwarding
ssh -X username@hostname
```

### Firewall Configuration
```bash
# Open necessary ports for Isaac Sim
sudo ufw allow 55555:55565/tcp
sudo ufw allow 33333:33343/tcp
```

## Verification Steps

### 1. System Verification
```bash
# Check system specifications
lscpu
free -h
nvidia-smi
df -h
```

### 2. Dependency Verification
```bash
# Verify critical dependencies
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

### 3. Isaac Sim Readiness Check
```bash
# Create test script to verify readiness
cat > test_isaac_readiness.py << 'EOF'
import sys
try:
    import cv2
    print("✓ OpenCV available")
except ImportError:
    print("✗ OpenCV not available")

try:
    import torch
    if torch.cuda.is_available():
        print("✓ CUDA available")
    else:
        print("⚠ CUDA not available")
except ImportError:
    print("✗ PyTorch not available")

try:
    import numpy as np
    print("✓ NumPy available")
except ImportError:
    print("✗ NumPy not available")

print("Prerequisites check complete")
EOF

python3 test_isaac_readiness.py
```

## Performance Optimization

### 1. System Tuning
```bash
# Add performance tuning to /etc/sysctl.conf
echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
echo "kernel.shmmax=2147483648" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 2. GPU Power Management
```bash
# Disable GPU power management for consistent performance
sudo nvidia-smi -pm 1
```

## Next Steps

After completing these prerequisites, proceed to:

1. Download and install the Omniverse Launcher from NVIDIA Developer website
2. Sign in with your NVIDIA Developer account
3. Install Isaac Sim LTS through the Omniverse Launcher
4. Verify installation with basic scene loading

This prerequisite setup ensures optimal performance and compatibility for Isaac Sim operation on Ubuntu 22.04 systems.
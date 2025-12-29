# System Setup Documentation

This document covers all system-level changes made to the Raspberry Pi.

## Operating System

- **OS:** Ubuntu 22.04 Server (64-bit)
- **Hardware:** Raspberry Pi 3 Model B
- **Hostname:** redbot

## Initial Setup

### Locale Configuration
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### System Updates
```bash
sudo apt update
sudo apt upgrade -y
```

## Serial Port Configuration

### Add User to dialout Group
```bash
sudo usermod -a -G dialout skylar
```

Logout and login for group change to take effect.

### Temporary Permission Fix
```bash
sudo chmod 666 /dev/ttyUSB0
```

## ROS2 Installation

See main README.md for complete ROS2 Humble installation steps.

## Systemd Service

### Service File Location

`/etc/systemd/system/redbot.service`

### Service Configuration

See README.md for complete service file content.

### Service Commands
```bash
# Enable auto-start on boot
sudo systemctl enable redbot.service

# Start service
sudo systemctl start redbot.service

# Stop service
sudo systemctl stop redbot.service

# Restart service
sudo systemctl restart redbot.service

# Check status
sudo systemctl status redbot.service

# View logs
journalctl -u redbot.service -f
```

## Network Configuration

WiFi configured during OS installation via Raspberry Pi Imager.

## Power Management

- Raspberry Pi: Powered by USB battery pack (5V, 2.5A minimum)
- RedBot: Powered by 4x AA batteries (6V)
- Systems electrically isolated (no shared power)

## SSH Configuration

SSH enabled by default on Ubuntu Server.

Access via:
```bash
ssh skylar@redbot.local
```

## Installed Packages
```bash
# System tools
sudo apt install git tmux htop -y

# Python packages
pip install pyserial
```

## Auto-start Configuration

The redbot.service starts automatically on boot and launches both:
- RedBot serial bridge node
- Odometry node

## Backup Recommendations

Regular backups recommended:
- Clone SD card image
- Git repository for code
- Document any manual system changes

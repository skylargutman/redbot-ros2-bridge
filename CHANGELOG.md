# Changelog

All notable changes to this project will be documented in this file.

## [1.1.0] - 2024-12-18

### Added
- RViz visualization support with saved configuration
- Motor command publisher for odometry direction correction
- CycloneDDS configuration for WSL2 network discovery
- Documentation for remote visualization setup

### Fixed
- Odometry now correctly tracks backward movement
- Encoder direction now properly signed based on motor commands

### Changed
- Odometry node now subscribes to motor commands for directional information
- Bridge node publishes motor commands on `/motor_commands` topic

## [1.0.0] - 2024-12-18

### Added
- Initial release
- Arduino firmware with serial protocol
- ROS2 bridge node for sensor communication  
- Odometry node for position tracking
- Systemd service for auto-start on boot
- Keyboard teleop control
- Complete documentation

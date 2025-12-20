# RedBot ROS2 Bridge

ROS2 Humble package for SparkFun RedBot with shadow chassis running on Raspberry Pi 3B.

## Hardware

- **Robot:** SparkFun RedBot Mainboard (ATmega328P)
- **Computer:** Raspberry Pi 3 Model B
- **Sensors:**
  - 2x Line follower sensors (center, right)
  - MMA8452Q 3-axis accelerometer
  - 2x Wheel encoders (192 ticks/rev)
  - 2x Bumper switches
  - Buzzer

## Pin Configuration

### Arduino Pins
- A0: Left bumper
- A1: Right bumper
- A2: Left encoder
- A3: Right encoder
- A6: Center line sensor
- A7: Right line sensor
- A4/A5: Accelerometer (I2C)
- 9: Buzzer

## Software Architecture

### Nodes

1. **redbot_bridge** - Serial communication with Arduino
   - Publishes: `/line_sensors`, `/accelerometer`, `/encoders`, `/bumper/left`, `/bumper/right`
   - Subscribes: `/cmd_vel`

2. **odometry_node** - Calculates robot pose from encoders
   - Publishes: `/odom`, TF transform (odom → base_link)
   - Subscribes: `/encoders`

## Installation

### Prerequisites
- Ubuntu 22.04 Server (64-bit)
- ROS2 Humble

### Install ROS2 Humble
```bash
sudo apt update
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions python3-pip -y
pip install pyserial
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Build the Package
```bash
mkdir -p ~/redbot_ws/src
cd ~/redbot_ws/src
git clone <your-repo-url> redbot_bridge
cd ~/redbot_ws
colcon build
source install/setup.bash
```

### Install Teleop Keyboard
```bash
cd ~/redbot_ws/src
git clone https://github.com/ros2/teleop_twist_keyboard.git
cd ~/redbot_ws
colcon build --packages-select teleop_twist_keyboard
source install/setup.bash
```

## Arduino Firmware

The Arduino sketch is located at `arduino/RedBot_ROS2_Bridge/RedBot_ROS2_Bridge.ino`

### Upload to RedBot

1. Open Arduino IDE
2. Set board to **Arduino Uno**
3. Install **SparkFun RedBot Library**
4. Open the sketch
5. Select correct COM port
6. Upload

### Serial Protocol

**Arduino → Raspberry Pi (20Hz):**
```
SENSORS,line_c,line_r,accel_x,accel_y,accel_z,enc_left,enc_right,bump_left,bump_right\n
```

**Raspberry Pi → Arduino:**
```
MOTOR,left_speed,right_speed\n        # -255 to 255
RESET_ENCODERS\n
BUZZ,frequency,duration\n
```

## Usage

### Manual Start

Terminal 1 - Launch nodes:
```bash
ros2 launch redbot_bridge redbot_launch.py
```

Terminal 2 - Keyboard control:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Auto-Start on Boot

See [System Service Setup](#system-service-setup) below.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Motor velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot pose and velocity |
| `/encoders` | `std_msgs/Int32MultiArray` | Raw encoder ticks [left, right] |
| `/line_sensors` | `std_msgs/Int32MultiArray` | Line sensor values [center, right] |
| `/accelerometer` | `std_msgs/Int32MultiArray` | Acceleration [x, y, z] |
| `/bumper/left` | `std_msgs/Bool` | Left bumper state |
| `/bumper/right` | `std_msgs/Bool` | Right bumper state |

## Parameters

### redbot_bridge

- `serial_port` (string, default: "/dev/ttyUSB0")
- `baud_rate` (int, default: 115200)

### odometry_node

- `wheel_diameter` (double, default: 0.065) - meters
- `wheel_base` (double, default: 0.15) - meters
- `ticks_per_revolution` (int, default: 192)

## System Service Setup

To start automatically on boot:
```bash
sudo nano /etc/systemd/system/redbot.service
```

Add:
```ini
[Unit]
Description=RedBot ROS2 Bridge
After=network.target

[Service]
Type=simple
User=skylar
Group=skylar
WorkingDirectory=/home/skylar
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/skylar/redbot_ws/install/setup.bash && ros2 launch redbot_bridge redbot_launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:
```bash
sudo systemctl daemon-reload
sudo systemctl enable redbot.service
sudo systemctl start redbot.service
```

Check status:
```bash
sudo systemctl status redbot.service
```

View logs:
```bash
journalctl -u redbot.service -f
```

## Calibration

### Measuring Robot Parameters

**Wheel Diameter:**
1. Mark a point on the wheel
2. Roll the wheel one complete rotation
3. Measure the distance traveled
4. Diameter = distance / π

**Wheel Base:**
Measure the distance between the center of the left and right wheels.

**Encoder Ticks:**
RedBot encoders: 192 ticks per revolution (verified)

### Testing Odometry Accuracy

Drive a known pattern and check final position:
```bash
# Watch position while driving
ros2 topic echo /odom --field pose.pose.position
```

Drive a 1m square, return to start. Check drift.

## Troubleshooting

### Serial Port Issues
```bash
# Find the port
ls /dev/ttyUSB*

# Set permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
```

### Upload Blocked by Serial Data

Press reset button on RedBot when Arduino IDE shows "Uploading..."

### Service Won't Start
```bash
# Check logs
journalctl -u redbot.service -n 50

# Check serial permissions
groups $USER  # should show 'dialout'
```

## Contributing

Feel free to submit issues and pull requests!

## License

MIT License

## Author

Built for integrating Arduino-based robots with ROS2.

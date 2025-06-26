# Micro-ROS ESP32 Ultrasonic Distance Sensor with Kalman Filter

A professional-grade distance measurement system using ESP32, HC-SR04 ultrasonic sensor, and advanced Kalman filtering, seamlessly integrated with ROS2 through micro-ROS.

## 🚀 Features

- **Real-time distance measurement** using HC-SR04 ultrasonic sensor (20Hz)
- **Advanced Kalman filtering** for noise reduction and signal stability
- **Voltage divider protection** for safe 3.3V operation on ESP32
- **Dual-topic publishing**: raw and filtered sensor data
- **Professional PlatformIO integration** with micro-ROS
- **Comprehensive error handling** and diagnostic output
- **RViz2 visualization support** with proper frame configuration

## Hardware Requirements

- ESP32 Development Board
- HC-SR04 Ultrasonic Distance Sensor
- 2x Resistors: 1kΩ (R1) and 2kΩ (R2) for voltage divider
- Jumper wires
- Breadboard

## Circuit Connections & Voltage Divider

### Why Voltage Divider is Needed

The HC-SR04 operates at 5V logic levels, but the ESP32 GPIO pins are designed for 3.3V. The ECHO pin outputs 5V when HIGH, which could potentially damage the ESP32. A voltage divider reduces this to a safe 3.3V level.

### Voltage Divider Calculation

Using resistors R1 = 1kΩ and R2 = 2kΩ:

```
Vout = Vin × (R2 / (R1 + R2))
Vout = 5V × (2kΩ / (1kΩ + 2kΩ))
Vout = 5V × (2/3) = 3.33V
```

This safely converts the 5V ECHO signal to 3.33V for the ESP32.

### Circuit Schematic

```
HC-SR04 Ultrasonic Sensor        ESP32
    ┌─────────────┐              ┌─────────────┐
    │             │              │             │
    │        VCC──┼──────────────┼──5V (VIN)   │
    │        GND──┼──────────────┼──GND        │
    │       TRIG──┼──────────────┼──GPIO2      │
    │             │              │             │
    │       ECHO──┼────┐         │             │
    └─────────────┘    │         │             │
                       │         │             │
                    R1 │ 1kΩ     │             │
                       │         │             │
                       ├─────────┼──GPIO4      │
                       │         │             │
                    R2 │ 2kΩ     │             │
                       │         │             │
                      GND────────┼──GND        │
                                 └─────────────┘

Legend:
- R1: 1kΩ resistor (Brown-Black-Red)
- R2: 2kΩ resistor (Red-Black-Red)
- Voltage divider output = 3.33V (safe for ESP32)
```

### Pin Connections

| HC-SR04 | ESP32 Pin | Connection Type | Notes |
|---------|-----------|-----------------|-------|
| VCC | 5V (VIN) | Direct | Power supply (5V required) |
| GND | GND | Direct | Ground connection |
| TRIG | GPIO2 | Direct | Trigger pin (3.3V compatible) |
| ECHO | GPIO4 | Via Voltage Divider | Through R1(1kΩ) + R2(2kΩ) |

## Software Setup

### Prerequisites

1. **PlatformIO** (VS Code extension or CLI)
2. **ROS2** (Humble/Iron/Jazzy recommended)
3. **micro-ROS agent**

### PlatformIO Configuration

Create a `platformio.ini` file in your project root:

```ini
[env:upesy_wroom]
platform = espressif32
board = upesy_wroom
framework = arduino
board_microros_distro = humble
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio

```

### Required Libraries (Auto-installed via PlatformIO)

```cpp
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/range.h>
```

### Installation Steps

1. **Install PlatformIO**:
   - VS Code: Install PlatformIO IDE extension
   - CLI: `pip install platformio`

2. **Create new project**:
   ```bash
   pio project init --board esp32dev
   ```

3. **Build and upload**:
   ```bash
   pio run --target upload
   pio device monitor
   ```

4. **Set up micro-ROS agent**:
   ```bash
   # Install micro-ROS agent (one-time setup)
   sudo apt install ros-humble-micro-ros-agent
   
   # Run the agent (replace /dev/ttyUSB0 with your ESP32 port)
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   ```

## ROS2 Integration

### Published Topics

The system publishes distance data on the following ROS2 topics:

- **Filtered Topic**: `/ultrasonic_sensor/range`
  - **Message Type**: `sensor_msgs/Range`
  - **Frame ID**: `ultrasonic_filtered`
  - **Frequency**: ~20 Hz

- **Raw Topic**: `/ultrasonic_sensor/range_raw`
  - **Message Type**: `sensor_msgs/Range`
  - **Frame ID**: `ultrasonic_raw`
  - **Frequency**: ~20 Hz

### ROS2 Commands

#### View Available Topics
```bash
ros2 topic list
```

#### Echo Ultrasonic Data (Filtered)
```bash
ros2 topic echo /ultrasonic_sensor/range
```

#### Echo Raw Ultrasonic Data
```bash
ros2 topic echo /ultrasonic_sensor/range_raw
```

#### Compare Data Rates
```bash
ros2 topic hz /ultrasonic_sensor/range
ros2 topic hz /ultrasonic_sensor/range_raw
```

#### Show Topic Information
```bash
ros2 topic info /ultrasonic_sensor/range
ros2 interface show sensor_msgs/Range
```

### Range Message Structure

```yaml
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type       # ULTRASOUND = 0
float32 field_of_view      # ~0.26 radians (15 degrees)
float32 min_range          # 0.02 meters (2cm)
float32 max_range          # 4.0 meters (400cm)
float32 range              # Measured distance [m]
```

## Kalman Filter Implementation

The system implements a 1D Kalman filter to smooth ultrasonic distance measurements and reduce sensor noise.

### Filter Algorithm

```cpp
// Prediction Step
P_predicted = P_previous + Q

// Update Step  
K = P_predicted / (P_predicted + R)
x_estimated = x_previous + K * (measurement - x_previous)
P_updated = (1 - K) * P_predicted
```

### Default Parameters

```cpp
struct KalmanFilter {
  float Q = 0.01;    // Process noise - how much distance expected to change
  float R = 0.1;     // Measurement noise - sensor uncertainty
  float P = 1.0;     // Initial error covariance
};
```

### Parameter Tuning Guide

| Parameter | Low Value (0.005-0.01) | Medium Value (0.01-0.05) | High Value (0.05-0.2) |
|-----------|-------------------------|---------------------------|------------------------|
| **Q** (Process) | Static environment | Normal indoor use | Moving robot/dynamic |
| **R** (Measurement) | Perfect mounting | Standard HC-SR04 | Noisy environment |

### Performance Benefits

- **Noise Reduction**: Filters out electrical and acoustic noise
- **Outlier Rejection**: Smooths sudden spikes in readings  
- **Stability**: Provides consistent distance measurements
- **Adaptability**: Automatically adjusts to measurement confidence

## Visualization and Analysis

### RViz2 Setup

1. Launch RViz2:
   ```bash
   rviz2
   ```

2. Add Range displays for both topics:
   - **Filtered Data**: Add → By topic → `/ultrasonic_sensor/range` → Range
   - **Raw Data**: Add → By topic → `/ultrasonic_sensor/range_raw` → Range

3. **Important**: Change the Fixed Frame:
   - In Global Options, change "Fixed Frame" from "map" to "ultrasonic_filtered"
   - This matches the frame_id used in the sensor messages

4. Configure display colors:
   - Set different colors for filtered vs raw data for easy comparison
   - Filtered data should appear smoother and more stable

### Data Analysis with PlotJuggler

Monitor and compare raw vs filtered sensor data:

```bash
# Install PlotJuggler (if not already installed)
sudo apt install ros-humble-plotjuggler-ros

# Launch PlotJuggler with ROS2 support
ros2 run plotjuggler plotjuggler

# In PlotJuggler:
# 1. Start ROS2 Topic Subscriber
# 2. Select both /ultrasonic_sensor/range and /ultrasonic_sensor/range_raw
# 3. Plot range values to visualize Kalman filter performance
```

### Serial Monitor (PlatformIO)

Monitor debug output and sensor readings:

```bash
# Using PlatformIO CLI
pio device monitor

# Using PlatformIO IDE
# Click on "Monitor" button in PlatformIO toolbar
```

Expected serial output:
```
Ultrasonic sensor with Kalman filter initialized!
Publishing filtered data to: /ultrasonic_sensor/range
Publishing raw data to: /ultrasonic_sensor/range_raw
Kalman Filter Parameters:
  Process Noise (Q): 0.01
  Measurement Noise (R): 0.1
Raw: 0.156m, Filtered: 0.158m, Gain: 0.091
Raw: 0.159m, Filtered: 0.159m, Gain: 0.083
```

## Troubleshooting

## Troubleshooting

### PlatformIO Issues

1. **Build errors with micro-ROS**:
   ```bash
   # Clean and rebuild
   pio run --target clean
   pio run
   
   # Check library installation
   pio lib list
   ```

2. **Upload fails**:
   ```bash
   # Check available ports
   pio device list
   
   # Specify port manually
   pio run --target upload --upload-port /dev/ttyUSB0
   ```

3. **Serial monitor not working**:
   ```bash
   # Set correct baud rate
   pio device monitor --baud 115200
   ```

### Hardware Issues

1. **No distance readings**:
   - Verify 5V power supply to HC-SR04
   - Check voltage divider output (should read 3.3V when ECHO is HIGH)
   - Test connections with multimeter
   - Ensure proper grounding

2. **Erratic readings**:
   - Check for loose connections
   - Verify voltage divider resistor values (1kΩ and 2kΩ)
   - Move away from acoustic interference sources
   - Check sensor mounting (should be stable)

3. **ESP32 not responding**:
   - ESP32 may be damaged by 5V on GPIO pin
   - Always use voltage divider on ECHO pin
   - Check if built-in LED blinks (error pattern)

### ROS2 Integration Issues

1. **micro-ROS agent not connecting**:
   ```bash
   # Check serial permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Try different baud rates
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
   
   # Reset ESP32 and restart agent
   ```

2. **No topic data**:
   ```bash
   # Verify agent connection
   ros2 topic list | grep ultrasonic
   
   # Check message frequency
   ros2 topic hz /ultrasonic_sensor/range
   
   # Monitor for errors
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
   ```

3. **RViz2 display issues**:
   - Ensure Fixed Frame matches sensor frame_id (`ultrasonic_filtered`)
   - Check Range display configuration
   - Verify topic names and message types
   - Try resetting RViz2 configuration

### Debugging Tools

```bash
# Monitor serial output
pio device monitor

# Check ROS2 topics and types
ros2 topic list -t
ros2 topic info /ultrasonic_sensor/range

# Test with command line
ros2 topic echo /ultrasonic_sensor/range --once

# Check node status
ros2 node list
ros2 node info /ultrasonic_sensor_node
```

### Project Structure

```
micro-ros-esp32-ultrasonic-kalman/
├── src/
│   └── main.cpp                 # Main application code
├── platformio.ini              # PlatformIO configuration
├── README.md                   # This documentation
└── docs/                       # Additional documentation
    ├── kalman_filter.md        # Kalman filter theory
    ├── circuit_design.md       # Hardware design details
    └── ros2_integration.md     # ROS2 setup guide
```

### Development Workflow

```bash
# 1. Clone the repository
git clone git@github.com:Adem-Aoun/micro-ros-esp32-ultrasonic-kalman.git
cd micro-ros-esp32-ultrasonic-kalman

# 2. Build the project
pio run

# 3. Upload to ESP32
pio run --target upload

# 4. Monitor serial output
pio device monitor

# 5. In another terminal, start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

## Configuration Notes

## Configuration Notes

### Important RViz2 Configuration

⚠️ **Critical**: Change the Fixed Frame in RViz2 from "map" to **"ultrasonic_filtered"** to properly visualize the sensor data.

### Sensor Specifications (HC-SR04)

| Parameter | Value | Unit |
|-----------|-------|------|
| **Operating Range** | 2 - 400 | cm |
| **Accuracy** | ±3 | mm |
| **Resolution** | 0.3 | cm |
| **Measuring Angle** | 15 | degrees |
| **Operating Frequency** | 40 | kHz |
| **Supply Voltage** | 5 | V DC |
| **Operating Current** | 15 | mA |
| **Standby Current** | <2 | mA |

### Performance Characteristics

- **Update Rate**: 20 Hz (50ms intervals)
- **Timeout Protection**: 50ms pulse timeout
- **Range Validation**: Automatic out-of-range detection
- **Filter Latency**: <1ms (real-time processing)
- **Memory Usage**: <2KB RAM for Kalman filter state

## License

MIT License - Feel free to use and modify for your projects.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch  
5. Create a Pull Request

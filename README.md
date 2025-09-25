# DCD Time Reference (dcd-timeref)

A ROS package for publishing time reference messages from PPS (Pulse Per Second) devices, designed for time synchronization across ROS systems.

## Overview

This package provides ROS nodes that read PPS signals from hardware devices and publish `sensor_msgs/TimeReference` messages. It's particularly useful for time synchronization in robotics applications where precise timing is critical.

## Features

- **PPS Device Support**: Reads PPS signals from hardware devices (e.g., GPS receivers)
- **Configurable Edge Detection**: Supports assert, clear, or both edge detection
- **Flexible Publishing**: Configurable publishing rates and message parameters
- **Two Implementations**: Full PPS version and simplified simulation version
- **Docker Support**: Containerized deployment with Docker and docker-compose
- **Launch Files**: Ready-to-use ROS launch configurations

## Package Structure

```
dcd-timeref/
├── src/
│   ├── dcd-timeref.cpp          # Full PPS implementation
│   └── dcd-timeref-simple.cpp   # Simplified simulation version
├── launch/
│   ├── dcd-timeref.launch       # Basic launch configuration
│   └── dcd-timeref_advanced.launch  # Advanced configuration
├── CMakeLists.txt               # Build configuration
├── package.xml                  # ROS package metadata
├── Dockerfile                   # Container configuration
├── docker-compose.yml          # Multi-container setup
└── build.sh                    # Build script
```

## Dependencies

### ROS Dependencies
- `roscpp`
- `sensor_msgs`
- `std_msgs`

### System Dependencies
- **Full version**: `libpps-dev` (for PPS device access)
- **Simple version**: No additional dependencies (simulation only)

## Installation

### 1. Clone and Build

```bash
# Navigate to your catkin workspace
cd /path/to/catkin_ws/src

# Clone the repository
git clone <repository-url> dcd-timeref

# Build the package
cd /path/to/catkin_ws
catkin_make
# or
catkin build dcd-timeref
```

### 2. Install System Dependencies (Full Version)

```bash
# Ubuntu/Debian
sudo apt-get install libpps-dev

# CentOS/RHEL
sudo yum install pps-tools-devel
```

## Usage

### Basic Usage

```bash
# Launch the simple simulation version
roslaunch dcd-timeref dcd-timeref.launch

# Launch with custom parameters
roslaunch dcd-timeref dcd-timeref_advanced.launch
```

### Running the Node Directly

```bash
# Simple simulation version
rosrun dcd-timeref dcd-timeref_simple

# Full PPS version (requires PPS device)
rosrun dcd-timeref dcd-timeref
```

### Docker Usage

```bash
# Build and run with docker-compose
docker-compose up --build

# Or build manually
docker build -t dcd-timeref .
docker run --rm -it dcd-timeref
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pps_device` | string | `/dev/pps0` | Path to PPS device |
| `edge` | string | `assert` | Edge detection: `assert`, `clear`, or `both` |
| `source` | string | `PPS` | Source identifier for messages |
| `frame_id` | string | `""` | Frame ID for message headers |
| `publish_both_edges` | bool | `false` | Publish on both edges |
| `use_header_stamp_now` | bool | `true` | Use current time for header stamp |
| `queue_size` | int | `10` | Publisher queue size |
| `rate` | double | `100.0` | Publishing rate (Hz) |
| `timeout_sec` | double | `1.0` | Timeout for PPS operations |

## Published Topics

- `/time_reference` (`sensor_msgs/TimeReference`): Time reference messages from PPS device

### Listening to Time Reference Messages

To monitor the published time reference messages:

```bash
# Listen to all time reference messages
rostopic echo /dcd_timeref/time_reference

# Listen with limited messages (useful for testing)
rostopic echo /dcd_timeref/time_reference -n 5

# Check topic information
rostopic info /dcd_timeref/time_reference

# Monitor topic rate
rostopic hz /dcd_timeref/time_reference
```

Example output:
```
header: 
  seq: 0
  stamp: 
    secs: 1758765292
    nsecs: 770291743
  frame_id: "gps_time"
time_ref: 
  secs: 1758765292
  nsecs: 770293092
source: "GPS_PPS"
---
```

## Message Format

The node publishes `sensor_msgs/TimeReference` messages with:
- `header.stamp`: ROS timestamp
- `header.frame_id`: Configurable frame ID
- `time_ref`: PPS timestamp
- `source`: Source identifier

## Launch File Examples

### Basic Configuration
```xml
<launch>
  <node pkg="dcd-timeref" type="dcd-timeref" name="dcd-timeref" output="screen">
    <param name="pps_device" value="/dev/pps0" />
    <param name="edge" value="assert" />
    <param name="source" value="GPS_PPS" />
    <param name="frame_id" value="gps_time" />
  </node>
</launch>
```

### Advanced Configuration
```xml
<launch>
  <node pkg="dcd-timeref" type="dcd-timeref" name="dcd-timeref" output="screen">
    <param name="pps_device" value="/dev/pps0" />
    <param name="edge" value="both" />
    <param name="publish_both_edges" value="true" />
    <param name="rate" value="100.0" />
    <param name="timeout_sec" value="2.0" />
  </node>
</launch>
```

## Hardware Setup

### PPS Device Configuration

1. **GPS Receiver Setup**: Ensure your GPS receiver provides PPS output
2. **Device Path**: Configure the correct device path (typically `/dev/pps0`)
3. **Permissions**: Ensure the user has access to the PPS device:
   ```bash
   sudo chmod 666 /dev/pps0
   # or add user to appropriate group
   ```

### Testing PPS Device

```bash
# Check if PPS device exists
ls -la /dev/pps*

# Test PPS device with pps-tools
sudo ppstest /dev/pps0
```

## Troubleshooting

### Common Issues

1. **Permission Denied**: Ensure proper permissions on PPS device
2. **Device Not Found**: Check if PPS device path is correct
3. **No PPS Signal**: Verify GPS receiver is providing PPS output
4. **Build Errors**: Install required dependencies (`libpps-dev`)

### Debug Mode

```bash
# Enable debug logging
export ROSCONSOLE_CONFIG_FILE=/path/to/debug.conf
roslaunch dcd-timeref dcd-timeref.launch
```

## Development

### Building

```bash
# Build with catkin
catkin_make

# Or use the provided build script
./build.sh
```

### Testing

```bash
# Test the simple version (no hardware required)
rosrun dcd-timeref dcd_timeref_simple

# Monitor published messages
rostopic echo /time_reference
```

## License

MIT License - see LICENSE file for details.

## Author

**clausqr** - hola@claus.com.ar

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Changelog

- **v1.0.0**: Initial release with PPS support and simulation mode

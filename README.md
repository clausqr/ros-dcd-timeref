# DCD Time Reference (dcd_timeref)

A ROS package for publishing time reference messages from PPS (Pulse Per Second) devices, designed for time synchronization across ROS systems. Specifically optimized for IMU → DCD → PPS → TimeReference workflows.

## Overview

This package provides ROS nodes that read PPS signals from hardware devices and publish `sensor_msgs/TimeReference` messages. It's particularly useful for time synchronization in robotics applications where precise timing is critical.

### Event-Driven Architecture

The node uses **event-driven PPS monitoring** rather than polling, ensuring:
- **Zero CPU waste**: No continuous polling loops
- **Real-time responsiveness**: Immediate event detection
- **High-frequency compatibility**: Supports IMU rates up to 400Hz without performance degradation
- **Kernel-level efficiency**: Leverages Linux PPS subsystem for optimal performance

### Performance Characteristics

- **CPU Usage**: Minimal - event-driven architecture eliminates polling overhead
- **Latency**: Sub-millisecond - direct kernel event handling
- **Throughput**: Supports high-frequency IMU data (400Hz+) without performance degradation
- **Memory**: Low footprint - no buffering or queuing of events

## Features

- **PPS Device Support**: Reads PPS signals from hardware devices (e.g., GPS receivers)
- **Configurable Edge Detection**: Supports assert, clear, or both edge detection
- **Flexible Publishing**: Configurable publishing rates and message parameters
- **Two Implementations**: Full PPS version and simplified simulation version
- **Docker Support**: Containerized deployment with Docker and docker-compose
- **Launch Files**: Ready-to-use ROS launch configurations

## Package Structure

```
dcd_timeref/
├── src/
│   ├── dcd_timeref.cpp          # Full PPS implementation
│   └── dcd_timeref_simple.cpp   # Simplified simulation version
├── launch/
│   ├── dcd_timeref.launch       # Basic launch configuration
│   └── dcd_timeref_advanced.launch  # Advanced configuration
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
- **Full version**: `libpps-dev` and `pps-tools` (for PPS device access)
- **Simple version**: No additional dependencies (simulation only)

```bash
# Ubuntu/Debian
sudo apt-get install pps-tools libpps-dev

# CentOS/RHEL/Fedora
sudo yum install pps-tools libpps-devel
```

## Installation

### 1. Clone and Build

```bash
# Navigate to your catkin workspace
cd /path/to/catkin_ws/src

# Clone the repository
git clone <repository-url> dcd_timeref

# Build the package
cd /path/to/catkin_ws
catkin_make
# or
catkin build dcd_timeref
```

### 2. Install System Dependencies (Full Version)

```bash
# Ubuntu/Debian
sudo apt-get install pps-tools libpps-dev

# CentOS/RHEL/Fedora
sudo yum install pps-tools libpps-devel
```

## Usage

### Basic Usage

```bash
# Launch the full PPS version (for DCD→PPS)
roslaunch dcd_timeref dcd_timeref.launch

# Launch with custom parameters
roslaunch dcd_timeref dcd_timeref_advanced.launch

# Launch simulation version (no hardware required)
roslaunch dcd_timeref dcd_timeref.launch pps_device:=/dev/null
```

### Running the Node Directly

```bash
# Full PPS version (requires PPS device)
rosrun dcd_timeref dcd_timeref

# Simple simulation version
rosrun dcd_timeref dcd_timeref_simple
```

### Docker Usage

For hardware access (DCD → PPS), you need to mount devices and configure permissions:

```bash
# Build and run with docker-compose (includes device mounting)
docker-compose up --build

# Manual build with device access
docker build -t dcd_timeref .
docker run --rm -it \
  --device /dev/pps0 \
  --device /dev/ttyUSB0 \
  --group-add dialout \
  dcd_timeref
```

**Docker Compose Configuration** (for hardware access):
```yaml
version: '3.8'
services:
  dcd_timeref:
    build: .
    devices:
      - "/dev/pps0:/dev/pps0"
      - "/dev/ttyUSB0:/dev/ttyUSB0"
    group_add:
      - "dialout"
    environment:
      - ROS_DISTRO=noetic
```

## Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pps_device` | string | `/dev/pps0` | Path to PPS device |
| `edge` | string | `assert` | Edge detection: `assert`, `clear`, or `both` |
| `source` | string | `DCD_PPS` | Source identifier for messages |
| `frame_id` | string | `gps_time` | Frame ID for message headers |
| `publish_both_edges` | bool | `false` | Publish on both edges (redundant if edge="both") |
| `use_header_stamp_now` | bool | `true` | Use current time for header stamp |
| `queue_size` | int | `10` | Publisher queue size |
| `rate` | double | `100.0` | Loop throttling rate (Hz) - controls loop frequency after event processing |
| `timeout_sec` | double | `1.0` | Timeout for PPS operations |
| `min_interval_ns` | int64 | `0` | Minimum interval between events (anti-rebounce) |
| `topic_name` | string | `time_reference` | Custom topic name |
| `latched` | bool | `false` | Use latched publishing |

### Rate Parameter Technical Details

The `rate` parameter controls the **loop throttling frequency**, not the PPS event detection frequency:

**How it works:**
1. **Event Detection**: `time_pps_fetch()` is **blocking** - waits for actual PPS events
2. **Event Processing**: When event occurs, message is published immediately
3. **Loop Throttling**: `rate.sleep()` limits how fast the main loop can iterate
4. **CPU Protection**: Prevents excessive loop iterations when no events are pending

**Execution Flow:**
```
PPS Event → time_pps_fetch() returns → Publish message → ros::spinOnce() → rate.sleep() → Loop
```

**Rate Values:**
- **100 Hz**: Maximum 100 loop iterations per second (10ms between loops)
- **200 Hz**: Maximum 200 loop iterations per second (5ms between loops)

**Important**: This is **NOT polling** - the node waits for real PPS events and responds immediately when they occur.

## Published Topics

- `/dcd_timeref/time_reference` (`sensor_msgs/TimeReference`): Time reference messages from PPS device

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

### Basic Configuration (Recommended for DCD→PPS)
```xml
<launch>
  <node pkg="dcd_timeref" type="dcd_timeref" name="dcd_timeref" output="screen">
    <param name="pps_device" value="/dev/pps0" />
    <param name="edge" value="assert" />
    <param name="source" value="DCD_PPS" />
    <param name="frame_id" value="gps_time" />
  </node>
</launch>
```

**Basic Configuration Rationale:**
- **`edge="assert"`**: Single-edge detection on rising edge (assert) of DCD trigger signal
- **`source="DCD_PPS"`**: Identifies PPS source as DCD device, distinguishing from GPS sources
- **Minimal configuration**: Essential parameters only for standard operation
- **Use case**: Standard DCD→PPS applications with moderate timing requirements

### Advanced Configuration (High-Precision Applications)
```xml
<launch>
  <node pkg="dcd_timeref" type="dcd_timeref" name="dcd_timeref" output="screen">
    <param name="pps_device" value="/dev/pps0" />
    <param name="edge" value="both" />
    <param name="publish_both_edges" value="true" />
    <param name="rate" value="200.0" />
    <param name="timeout_sec" value="0.5" />
    <param name="min_interval_ns" value="1000000" />
  </node>
</launch>
```

**Advanced Configuration Rationale:**
- **`edge="both"`**: Dual-edge detection capturing both rising (assert) and falling (clear) edges
- **`publish_both_edges="true"`**: Publishes messages on both edges for maximum temporal resolution
- **`rate="200.0"`**: Event monitoring frequency (200 Hz) for system health checks, not polling
- **`timeout_sec="0.5"`**: Aggressive timeout for rapid failure detection and system monitoring
- **`min_interval_ns="1000000"`**: Debounce protection (1ms) to prevent double-trigger artifacts
- **Use case**: High-precision timing applications, critical timing systems, maximum temporal accuracy requirements

### Configuration Comparison

| Parameter | Basic | Advanced | Technical Rationale |
|-----------|-------|----------|-------------------|
| **Edge Detection** | `assert` only | `both` edges | Basic: Single-edge trigger detection<br>Advanced: Dual-edge for maximum temporal precision |
| **Publishing** | Single edge | Both edges | Basic: One message per trigger event<br>Advanced: Two messages per trigger cycle |
| **Event Monitoring** | 100 Hz | 200 Hz | Basic: Standard system health checks<br>Advanced: Enhanced monitoring frequency |
| **Timeout** | 1.0 sec | 0.5 sec | Basic: Standard error detection interval<br>Advanced: Aggressive failure detection for critical systems |
| **Debounce** | None | 1ms | Basic: No signal conditioning<br>Advanced: Hardware-level debounce protection |

### Configuration Selection Criteria

**Basic Configuration is recommended for:**
- Standard DCD→PPS integration scenarios
- Applications with moderate timing precision requirements
- Systems with resource constraints
- Single-edge trigger detection sufficient for use case
- Standard error detection and monitoring requirements

**Advanced Configuration is required for:**
- High-precision timing applications (microsecond-level accuracy)
- Critical timing systems with strict temporal requirements
- Applications requiring maximum temporal resolution
- Dual-edge trigger detection for enhanced precision
- Hardware-level debounce protection for noisy signals
- Aggressive failure detection for mission-critical systems

## Hardware Setup

### DCD → PPS Configuration (Recommended Setup)

This setup is optimized for IMU → DCD → PPS → TimeReference workflows:

1. **Load PPS Line Discipline**:
   ```bash
   sudo modprobe pps_ldisc
   ```

2. **Attach PPS to Serial Device**:
   ```bash
   # For USB serial (most common)
   sudo ldattach PPS /dev/ttyUSB0
   
   # For built-in serial
   sudo ldattach PPS /dev/ttyS0
   ```

3. **Verify PPS Device**:
   ```bash
   # Check if PPS device was created
   ls -la /dev/pps*
   
   # Test PPS device
   sudo ppstest /dev/pps0
   ```

4. **Configure Permissions** (create `/etc/udev/rules.d/99-pps.rules`):
   ```bash
   # /etc/udev/rules.d/99-pps.rules
   KERNEL=="pps[0-9]*", MODE="0660", GROUP="dialout"
   KERNEL=="ttyUSB[0-9]*", MODE="0660", GROUP="dialout"
   
   # Reload rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   
   # Add user to dialout group
   sudo usermod -a -G dialout $USER
   ```

### Alternative: GPS PPS Setup

1. **GPS Receiver Setup**: Ensure your GPS receiver provides PPS output
2. **Device Path**: Configure the correct device path (typically `/dev/pps0`)
3. **Permissions**: Ensure the user has access to the PPS device

### Testing PPS Device

```bash
# Check if PPS device exists
ls -la /dev/pps*

# Test PPS device with pps-tools
sudo ppstest /dev/pps0

# Monitor PPS events in real-time
sudo ppstest /dev/pps0 -p
```

## Runtime Checklist (IMU → DCD → PPS)

For your specific use case, follow this checklist:

1. **Connect Hardware**:
   - Connect trigger IMU → DCD (via RS-232 interface)
   - Connect DCD → Computer (via USB serial or built-in serial)

2. **Setup PPS Line Discipline**:
   ```bash
   sudo modprobe pps_ldisc
   sudo ldattach PPS /dev/ttyUSB0  # or /dev/ttyS0
   ```

3. **Verify PPS Device**:
   ```bash
   ls -l /dev/pps*
   sudo ppstest /dev/pps0
   ```

4. **Launch Node**:
   ```bash
   roslaunch dcd_timeref dcd_timeref.launch \
     pps_device:=/dev/pps0 \
     edge:=assert \
     source:=DCD_PPS \
     frame_id:=gps_time
   ```

5. **Verify Output**:
   ```bash
   rostopic echo /dcd_timeref/time_reference
   # Check for incrementing timestamps
   ```

## Troubleshooting

### Common Issues

1. **Permission Denied**: Ensure proper permissions on PPS device
2. **Device Not Found**: Check if PPS device path is correct
3. **No PPS Signal**: Verify DCD is providing trigger signals
4. **Build Errors**: Install required dependencies (`libpps-dev`)
5. **ldattach fails**: Check if serial device is available and not in use

### Debug Mode

```bash
# Enable debug logging
export ROSCONSOLE_CONFIG_FILE=/path/to/debug.conf
roslaunch dcd_timeref dcd_timeref.launch
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
rosrun dcd_timeref dcd_timeref_simple

# Monitor published messages
rostopic echo /dcd_timeref/time_reference
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

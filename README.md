# ROS Noetic Time Reference from PPS/DCD (IMU Trigger) — dcd_timeref

Publish precise `sensor_msgs/TimeReference` from **kernel-level PPS on DCD** (IMU trigger, GPS-disciplined clock) on **Ubuntu 20.04 / ROS Noetic**.  
**Keywords:** ROS Noetic, PPS, DCD, IMU, GPS, timestamp, time sync, `sensor_msgs/TimeReference`, Ubuntu 20.04, PPSAPI, chrony.


![ROS](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange)
![License](https://img.shields.io/badge/License-MIT-green)

> **TL;DR**: Convert **IMU → DCD** into **/dev/ppsX** (`pps_ldisc`), and publish exact trigger timestamps as `sensor_msgs/TimeReference`. **Corrects USB-serial jitter and lag** by using direct RS-232/RS-485 hardware sync.

## Table of Contents
- [Overview](#overview)
- [Event-Driven Architecture](#event-driven-architecture)
- [Performance](#performance-characteristics)
- [Features](#features)
- [Quickstart](#quickstart)
- [Usage](#usage)
- [Parameters](#configuration-parameters)
- [Hardware Setup (DCD → PPS)](#hardware-setup)
- [Docker](#docker-usage)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview
This package provides ROS nodes that read **PPS events from /dev/ppsX** and publish `sensor_msgs/TimeReference`. Optimized for **IMU trigger on DCD** (RS-232) → **PPSAPI** → **TimeReference**.

### Event-Driven Architecture
No polling. Uses **blocking `time_pps_fetch()`** (kernel timestamps). Sub-ms latency, µs-ns jitter (hardware/driver dependent).

### Performance Characteristics
- **CPU**: minimal (event-driven)
- **Latency**: sub-ms (kernel timestamp)
- **Throughput**: 400 Hz+
- **Memory**: tiny (no queues)
- **Jitter**: µs-ns precision (hardware dependent)
- **USB-Serial**: Correct USB-serial jitter and lag by using direct RS-232/RS-485 connections

### System Clock Synchronization

**Critical**: This package assumes your system clock is synchronized with GPS time. The PPS events provide precise timing, but the system clock must be accurate for proper timestamp correlation.

**Recommended setup with chrony**:
```bash
# Install chrony for GPS time synchronization
sudo apt-get install chrony

# Configure chrony to use GPS (example configuration)
echo "refclock SHM 0 offset 0.0 delay 0.1" | sudo tee -a /etc/chrony/chrony.conf
echo "refclock SHM 1 offset 0.0 delay 0.1" | sudo tee -a /etc/chrony/chrony.conf

# Restart chrony
sudo systemctl restart chrony
```

**Without proper clock synchronization**, the `time_ref` timestamps will be inaccurate even with precise PPS events.

### Alternative Use Cases (No GPS Required)

Even without GPS time synchronization, this package is valuable for:

**USB Jitter Correction**: Correct for USB-serial jitter and lag in sensor data by aligning trigger timestamps to computer clock
**Sensor Data Alignment**: Synchronize multiple sensors using precise trigger timestamps
**Relative Timing**: Maintain precise relative timing between sensor events and computer processing

**Example use case**: IMU data via USB with trigger pulse → PPS timestamp → align IMU data to precise trigger time

## Quickstart
```bash
sudo apt update && sudo apt install -y pps-tools libpps-dev
sudo modprobe pps_ldisc
sudo ldattach PPS /dev/ttyUSB0   # or /dev/ttyS0
ls -l /dev/pps*

# Build
cd ~/catkin_ws/src && git clone https://github.com/clausqr/ros-noetic-dcd-timeref.git dcd_timeref && cd ..
catkin_make

# Run
roslaunch dcd_timeref dcd_timeref.launch pps_device:=/dev/pps0 edge:=assert source:=DCD_PPS frame_id:=gps_time
```

## Features

- **PPS Device Support**: Reads PPS signals from hardware devices (e.g., GPS receivers)
- **Configurable Edge Detection**: Supports assert, clear, or both edge detection
- **Flexible Publishing**: Configurable publishing rates and message parameters
- **Two Implementations**: Full PPS version and simplified simulation version
- **Docker Support**: Containerized deployment with Docker and docker-compose
- **Launch Files**: Ready-to-use ROS launch configurations

## Usage

### Basic Usage

```bash
# Launch the simple simulation version
roslaunch dcd_timeref dcd_timeref.launch

# Launch with custom parameters
roslaunch dcd_timeref dcd_timeref_advanced.launch
```

### Running the Node Directly

```bash
# Simple simulation version
rosrun dcd_timeref dcd_timeref_simple

# Full PPS version (requires PPS device)
rosrun dcd_timeref dcd_timeref
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

## Hardware Setup (DCD → PPS)

### DCD Pin Connection

**Critical**: You must connect your **trigger pulse source (IMU, sensor, etc.)** of your serial port:

1. **DCD Pin Connection**: Connect your trigger signal to the **DCD (Data Carrier Detect) pin** of your RS-232 serial port
2. **Signal Requirements**: The trigger pulse should be a clean digital signal for DCD pin detection:
   - **DCD Pin Logic**: 
     - **Active (High)**: +3V to +15V (DCD pin asserted)
     - **Inactive (Low)**: -3V to -15V (DCD pin deasserted)
   - **Edge Detection**: Rising edge (assert) or falling edge (clear) triggers PPS event
   - **Transition Time**: < 1μs for reliable edge detection
   - **Signal Integrity**: Clean transitions without ringing or overshoot
   - **Note**: DCD pin is a **control signal**, not a data signal, so timing requirements are less strict than data pins
3. **Connection Types**:
   - **RS-232**: Direct connection to DCD pin (pin 1 on DB-9)
   - **RS-485**: Use appropriate level converter to RS-232 DCD pin
   - **Avoid USB-serial**: USB-serial converters don't provide DCD pin capability - use direct RS-232/RS-485 to eliminate jitter

### PPS Device Configuration

1. **Kernel-level setup** (required first):
   ```bash
   # Load PPS line discipline
   sudo modprobe pps_ldisc
   
   # Attach PPS to serial device (creates /dev/pps0)
   sudo ldattach PPS /dev/ttyS0
   
   # Verify PPS device
   ls -la /dev/pps*
   sudo ppstest /dev/pps0
   ```

2. **Edge Detection Configuration**: Configure which edge triggers the PPS event via ROS parameter `edge`:
   - **Rising Edge (assert)**: `edge: "assert"` - Trigger on positive transition (0 → 1)
   - **Falling Edge (clear)**: `edge: "clear"` - Trigger on negative transition (1 → 0)  
   - **Both Edges**: `edge: "both"` - Trigger on both transitions if relevant
   
   **Configuration example**:
   ```bash
   roslaunch dcd_timeref dcd_timeref.launch edge:=assert
   # or in launch file: <param name="edge" value="assert" />
   ```
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

## Docker Usage

```bash
# Build and run with docker-compose
docker-compose up --build

# Or build manually
docker build -t dcd-timeref .
docker run --rm -it dcd-timeref
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
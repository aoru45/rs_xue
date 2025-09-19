# RS_XUE - RoboSense LiDAR Python Interface

A Python interface library for RoboSense LiDAR that provides real-time point cloud data acquisition and PCAP file conversion functionality.

## Features

- **Real-time Point Cloud Acquisition**: Support real-time point cloud data acquisition from RoboSense LiDAR
- **PCAP File Conversion**: Convert PCAP format LiDAR data to NumPy arrays
- **Simplified Interface**: Provides easy-to-use Python API
- **Multiple LiDAR Model Support**: Supports RSEM4 and other RoboSense LiDAR models
- **Calibration and Filtering**: Supports point cloud calibration transformation and distance filtering

## System Requirements

- Python >= 3.8
- NumPy
- RoboSense LiDAR SDK (rs_driver)
- CMake >= 3.16.0
- C++11 or higher compiler

## Dependencies

This project includes the following submodules:
- `pybind11`: Python-C++ binding library
- `cnpy`: NumPy file I/O library

## Build and Installation
First, build and install rs_driver.

### 1. Clone the project and initialize submodules

```bash
git clone <repository-url>
cd rs_xue
git submodule update --init --recursive
```

### 2. Build the project

```bash
mkdir build
cd build
cmake ..
make -j$(nproc)
```

### 3. Install Python package

```bash
# Run installation script
./install.sh

# Or install manually
cp build/*.so rs_xue_package/rs_xue/
cd rs_xue_package
pip install .
```

## Usage

### Real-time Point Cloud Acquisition

```python
import rs_xue
import numpy as np
import time

# Create client
client = rs_xue.Client()

# Initialize connection (using LiDAR IP address)
if client.initialize("192.168.1.200"):
    print("Connection successful")
    
    try:
        while True:
            # Get point cloud data
            points = client.get()  # Returns NumPy array with shape (N, 3) containing [x, y, z] coordinates
            
            if points is not None:
                print(f"Received {len(points)} points")
                print(f"Point cloud range: X[{points[:, 0].min():.2f}, {points[:, 0].max():.2f}], "
                      f"Y[{points[:, 1].min():.2f}, {points[:, 1].max():.2f}], "
                      f"Z[{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
            
            time.sleep(0.1)  # 100ms interval
            
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Stop client
        client.stop()
else:
    print("Connection failed")
```

### PCAP File Conversion

```python
import rs_xue

# Basic PCAP conversion
result = rs_xue.convert_pcap(
    from_name="input.pcap",    # Input PCAP file path
    to_name="output",          # Output directory
    num_frames=100             # Number of frames to convert
)

# PCAP conversion with calibration
import numpy as np

# Rotation matrix (3x3)
R = np.eye(3, dtype=np.float32)

# Translation vector (3,)
t = np.array([0.0, 0.0, 0.0], dtype=np.float32)

# Distance range [x_min_range, x_max_range, y...]， 
ranges = np.array([0.5, 100.0， 0.5, 100.0， 0.5, 100.0], dtype=np.float32)

result = rs_xue.convert_pcap_with_calib(
    from_name="input.pcap",
    to_name="output",
    R=R,
    t=t,
    ranges=ranges,
    num_frames=100
)
```

## API Reference

### Client Class

#### Methods

- `__init__()`: Create client instance
- `initialize(lidar_ip: str) -> bool`: Initialize connection using default port 6699 and RSEM4 type
- `get() -> numpy.ndarray`: Get point cloud data, returns array with shape (N, 3) containing [x, y, z] coordinates
- `stop()`: Stop client

### Conversion Functions

- `convert_pcap(from_name, to_name, num_frames)`: Basic PCAP conversion
- `convert_pcap_with_calib(from_name, to_name, R, t, ranges, num_frames)`: PCAP conversion with calibration

## Example Programs

The project includes the following examples:

- `test_simple_lidar.py`: Complete example of real-time point cloud acquisition
- `demo_online.cpp`: C++ version of real-time data acquisition example

Run Python example:

```bash
python test_simple_lidar.py
```

## Configuration

### Default Parameters

- MSOP Port: 6699
- DIFOP Port: 7788  
- LiDAR Type: RSEM4
- Host IP: "0.0.0.0" (auto-detect)

### Supported LiDAR Models

- RSEM4
- Other RoboSense models (need to specify during initialization)

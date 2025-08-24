# RS_XUE - RealSense LiDAR Python Interface

This package provides Python bindings for RealSense LiDAR devices.

## Installation

```bash
pip install .
```

## Usage

```python
import rs_xue
import numpy as np

# Create LiDAR client
client = rs_xue.Client()

# Initialize with LiDAR IP
if client.initialize("0.0.0.0"):
    print("LiDAR initialized successfully")
    
    # Get point cloud data
    point_cloud = client.get()
    if point_cloud is not None:
        print(f"Got {point_cloud.shape[0]} points")
        print(f"Point cloud shape: {point_cloud.shape}")
    
    # Stop the client
    client.stop()
else:
    print("Failed to initialize LiDAR")
```

## Requirements

- Python >= 3.8
- NumPy
- RealSense LiDAR device
# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a ROS2 Python package called `vision_node` that implements computer vision functionality using Intel RealSense cameras and Ultralytics YOLO for tool detection and analysis. The main functionality is in `tool_detect.py` which provides real-time detection, segmentation, and 2D orientation analysis of tools with depth information, publishing results via ROS2 topics and accepting tool selection via services.

## Core Architecture

### Main Module: `tool_detect.py`
- **Purpose**: Real-time tool detection using RealSense cameras + YOLO models
- **Key Features**:
  - Supports both segmentation and detection YOLO models with automatic fallback
  - 2D OBB (Oriented Bounding Box) calculation with handle/tip differentiation
  - Depth estimation using aligned depth frames
  - Tool orientation (roll angle) with smoothing
  - Multi-criteria handle/tip classification using width, distance transform radius, and pointiness metrics

### Key Components:
1. **Vision Pipeline**: RealSense camera → YOLO inference → OBB calculation → visualization
2. **Tool Analysis**: PCA-based orientation analysis with handle/tip detection logic
3. **Depth Processing**: Spatial and temporal filtering with median-based depth estimation
4. **Visualization**: Real-time display with segmentation masks, OBBs, and handle/tip markers

### Configuration Constants:
- Target tool classes: `nipper`, `vernier_calipers`, `wire_cutter`, `wire_stripper`
- Model path: `/home/temp_id/ros2_ws/src/vision_node/vision_node/best.pt`
- Optimized for small tool detection (imgsz=960, relaxed conf/iou thresholds)

## Commands

### Testing
```bash
# Run all tests (from ROS2 workspace root)
colcon test --packages-select vision_node

# Run specific test types
pytest test/test_flake8.py    # Code style (flake8)
pytest test/test_pep257.py    # Docstring style (pep257)
pytest test/test_copyright.py # Copyright headers (currently skipped)
```

### Building
```bash
# Build the package (from ROS2 workspace root)
colcon build --packages-select vision_node

# Source the built package
source install/setup.bash
```

### Running
```bash
# Run the ROS2 vision node
ros2 run vision_node vision_node

# With parameters
ros2 run vision_node vision_node --ros-args -p target_tool:="nipper" -p publish_rate:=30.0
```

### ROS2 Interface

**Published Topics:**
- `/tool_detections` (vision_msgs/Detection3DArray): Tool detection results with 3D position and orientation

**Services:**
- `/set_target_tool` (vision_node/SetTargetTool): Set specific tool to detect or empty string for all tools

**Parameters:**
- `target_tool` (string): Specific tool name to detect, empty for all tools
- `publish_rate` (double): Detection publishing rate in Hz (default: 30.0)

### Service Usage Examples
```bash
# Detect specific tool
ros2 service call /set_target_tool vision_node/srv/SetTargetTool "{tool_name: 'nipper'}"

# Detect all tools
ros2 service call /set_target_tool vision_node/srv/SetTargetTool "{tool_name: ''}"

# Monitor detections
ros2 topic echo /tool_detections
```

### Linting
```bash
# Code style checking
flake8 vision_node/

# Docstring style checking  
pep257 vision_node/
```

## Development Notes

- This is a ROS2 humble package using ament_python build system with custom service definitions
- Main dependencies: OpenCV, NumPy, pyrealsense2, ultralytics (YOLO), tf_transformations
- The code includes Korean comments indicating it may be part of an international project  
- Uses defensive coding practices for camera failures with headless fallback mode
- Model file (`best.pt`) is a trained YOLO model specific to tool detection
- Implements sophisticated tool orientation analysis beyond basic bounding boxes
- Publishes 3D position and orientation in camera coordinate frame
- Supports dynamic tool selection via ROS2 services

## File Structure

```
vision_node/
├── vision_node/
│   ├── __init__.py
│   ├── tool_detect.py     # Main ROS2 vision node
│   └── best.pt           # YOLO model weights
├── srv/
│   └── SetTargetTool.srv # Custom service definition
├── test/                 # Standard ROS2 ament tests
├── CMakeLists.txt       # CMake build configuration
├── package.xml          # ROS2 package manifest with dependencies
└── setup.py            # Python package setup with entry points
```
# Hornet: Quadruped ROS2 Robot

This package contains the ROS2 implementation of the Hornet quadruped robot with face recognition and Xbox controller support.

## Architecture

The system consists of three main nodes:

1. **servo_controller_node** - Controls PCA9685 servo hardware
2. **kinematics_node** - Handles inverse kinematics and movement interpolation  
3. **behavior_controller_node** - High-level behaviors, face recognition, and controller input

## Setup

### Dependencies

Install required Python packages:
```bash
pip install Adafruit-PCA9685 pygame opencv-python face-recognition numpy
```

## Running

### Start All Nodes

```bash
# Launch all nodes
ros2 launch behavior_controller hornet_robot.launch.py
```

### Individual Nodes

Run nodes individually for testing:

```bash
# Servo controller
ros2 run servo_controller servo_controller_node

# Kinematics
ros2 run kinematics kinematics_node  

# Behavior controller
ros2 run behavior_controller behavior_controller_node
```

## Controls

### Xbox Controller
- **Left Stick Y**: Forward/Backward movement
- **Left Stick X**: Lean left/right
- **Right Stick X**: Turn left/right
- **A Button**: Hand wave gesture
- **B Button**: Hand shake gesture  
- **X Button**: Sit down
- **Y Button**: Stand up

### Face Recognition
The robot automatically greets known faces (Souri, Vishnu, Arjun) with:
- Hand wave (3 times)
- Hand shake (3 times)
- Step forward (6 steps)
- Step back (6 steps)

## Topics

### Published
- `/servo_angles` - Servo angle commands (Float64MultiArray)
- `/leg_positions` - Leg position commands (Float64MultiArray)
- `/behavior_status` - Current behavior status (String)
- `/servo_feedback` - Current servo positions (Float64MultiArray)
- `/leg_feedback` - Current leg positions (Float64MultiArray)

### Subscribed
- `/joy` - Joystick input (Joy)

## Hardware Requirements
- Raspberry Pi with I2C enabled
- PCA9685 servo driver board
- 12 servo motors (3 per leg)
- USB webcam for face recognition
- Xbox controller (optional)

## Notes
- The system gracefully handles missing hardware dependencies
- Face recognition requires the `encodings.pickle` file from the original hornet directory
- All movement logic is preserved from the original implementation

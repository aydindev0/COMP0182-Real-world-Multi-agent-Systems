# PhaseSpace Messages Package

This package contains ROS2 message definitions for PhaseSpace motion capture system integration.

**Package Creator:** Valerio Modugno (jianwei.liu.21@ucl.ac.uk)

## Message Types

### Camera Messages
- **`Camera.msg`**: Individual camera pose information
- **`Cameras.msg`**: Collection of camera poses

### Marker Messages  
- **`Marker.msg`**: Individual LED marker pose
- **`Markers.msg`**: Collection of marker poses for a tracked body

### Rigid Body Messages
- **`Rigid.msg`**: Rigid body pose (position + orientation)
- **`Rigids.msg`**: Collection of rigid body poses

## Message Structure

### Rigid Body Message (`Rigid.msg`)
```
# Position (x, y, z in meters)
float64 x
float64 y  
float64 z

# Orientation quaternion (w, x, y, z)
float64 qw
float64 qx
float64 qy
float64 qz
```

### Marker Message (`Marker.msg`)
```
# Marker ID
int32 id

# Position (x, y, z in meters)
float64 x
float64 y
float64 z
```

## Usage

These messages are typically published by PhaseSpace ROS nodes that interface with PhaseSpace motion capture hardware. Common topics include:

- `/phasespace/rigid_body_<body_name>` - Rigid body poses
- `/phasespace/markers_<body_name>` - Individual marker positions
- `/phasespace_cameras` - Camera poses

## Dependencies

- `std_msgs` - Standard ROS2 message types
- `rosidl_default_generators` - Message generation tools

## Building

```bash
colcon build --symlink-install --packages-select phasespace_msgs_2
source install/setup.bash
```

## License

BSD License
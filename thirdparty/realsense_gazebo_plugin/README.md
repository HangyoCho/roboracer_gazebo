# Intel RealSense Gazebo ROS plugin

Gazebo Classic ROS plugin for Intel RealSense cameras. Supports both dual-IR (D435/D435i) and single-IR (L515) configurations.

## Features

- Publishes depth, color, and IR images via `image_transport`
- Publishes camera_info with intrinsics derived from HFOV
- Optional XYZRGB point cloud generation
- **Distance-dependent depth noise** for LiDAR cameras (L515)

## Distance-dependent depth noise

For LiDAR-based cameras like the L515, depth noise increases with distance. This plugin supports a linear noise model applied per-pixel in `OnNewDepthFrame()`:

```
stddev(d) = depthNoiseBase + depthNoiseSlope * d
```

Configure via SDF parameters:

```xml
<depthNoiseBase>0.004</depthNoiseBase>   <!-- base noise in meters -->
<depthNoiseSlope>0.00111</depthNoiseSlope> <!-- noise increase per meter -->
```

When these parameters are absent or zero, distance noise is disabled and the plugin falls back to Gazebo's built-in sensor noise (used by D435/D435i).

## Supported configurations

| Camera | IR channels | Distance noise | Notes |
|--------|------------|----------------|-------|
| D435 | 2 (stereo) | No | Uses Gazebo sensor-level constant noise |
| D435i | 2 (stereo) | No | Same as D435 + IMU |
| L515 | 1 (single) | Yes | LiDAR, per-pixel distance-dependent noise |

## Acknowledgement

Based on work by [SyrianSpock](https://github.com/SyrianSpock) and Intel Corporation's [D435 ROS model](https://github.com/intel-ros/realsense).

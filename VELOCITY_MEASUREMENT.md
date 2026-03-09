# Velocity Measurement Enhancement for FASTLIO2 SLAM

## Overview

This enhancement extends FASTLIO2 to integrate robot odometry velocity measurements into the IESKF (Invariant EKF) state estimator. This improves SLAM accuracy by providing additional velocity constraints, especially useful in scenarios where LiDAR registration quality is degraded.

## Architecture

### Components Added

1. **VelocityProcessor** (`velocity_processor.h/cpp`)
   - Manages velocity measurement buffering and processing
   - Handles asynchronous velocity updates
   - Manages measurement covariance

2. **IESKF Enhancement** (`ieskf.h/cpp`)
   - New `updateVelocity()` method for direct velocity updates
   - Uses Kalman gain for state correction
   - Implements Joseph form for numerical stability

3. **LIONode Integration** (`lio_node.cpp`)
   - Odometry subscriber for velocity topic
   - Synchronization of velocity measurements with LiDAR frames
   - Configuration parameter loading

## Configuration Parameters

Add these parameters to `lio.yaml`:

```yaml
# Odometry topic from robot
odometry_topic: /eduard/odom

# Enable velocity measurement integration
enable_velocity_measurement: true

# Velocity measurement covariance (standard deviation in m/s)
# Lower values = higher trust in odometry
velocity_cov_x: 0.01
velocity_cov_y: 0.01
velocity_cov_z: 0.01
```

## How It Works

### Prediction Step (Unchanged)
- IMU data drives forward prediction
- Gyroscope and accelerometer update state

### Update Step (Enhanced)
1. **LiDAR Update** (existing)
   - Point cloud registration provides position constraints

2. **Velocity Update** (new)
   - Odometry velocity measurements update velocity state
   - Measurement model: `z = v` (direct velocity measurement)
   - Update only affects velocity estimate (indices 12-14 of 21D state)

### Synchronization
- Odometry measurements are collected asynchronously
- When a LiDAR frame is processed, all odometry measurements up to that timestamp are applied
- This allows odometry and LiDAR to operate at different frequencies

## State Vector Structure (21D)

The IESKF state includes:
- **[0-2]**: Rotation (r_wi)
- **[3-5]**: Position (t_wi)
- **[6-8]**: IMU-LiDAR rotation (r_il)
- **[9-11]**: IMU-LiDAR position (t_il)
- **[12-14]**: **Velocity (v)** ← Updated by odometry
- **[15-17]**: Gyroscope bias (bg)
- **[18-20]**: Accelerometer bias (ba)

## Usage Example

```bash
# Start FASTLIO2 with velocity measurements enabled
ros2 launch fastlio2 lio_launch.py

# Publish odometry from your robot (e.g., encoder-based odometry)
# Expected message type: nav_msgs/Odometry
# Velocity is extracted from: msg.twist.twist.linear.{x,y,z}
```

## Tuning Covariance Parameters

- **Low covariance (0.001 - 0.01)**: Trust odometry measurements highly
  - Use if odometry is very accurate
  - Better convergence but can diverge if odometry is wrong

- **Medium covariance (0.01 - 0.1)**: Balanced trust
  - Recommended for most scenarios
  - Good compromise between correction power and stability

- **High covariance (0.1 - 1.0)**: Low trust in odometry
  - Use if odometry is unreliable or noisy
  - LiDAR will dominate, velocity measurements provide gentle guidance

## Optional Integration

The velocity measurement is optional and can be:
1. Disabled entirely by setting `enable_velocity_measurement: false`
2. Dynamically tuned via the `velocity_cov_*` parameters
3. Toggled without recompiling - just update `lio.yaml`

## Mathematical Details

### Velocity Update Equation

Measurement model:
$$z_v = v + \eta_v$$

where $\eta_v \sim \mathcal{N}(0, R_v)$ with $R_v = \text{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2)$

Measurement Jacobian:
$$H = [0_{3 \times 12} \, I_3 \, 0_{3 \times 6}]$$

Kalman gain:
$$K = P H^T (H P H^T + R_v)^{-1}$$

State update:
$$\Delta x = K (z_v - v)$$

Covariance update (Joseph form):
$$P \gets (I - KH) P (I - KH)^T + K R_v K^T$$

## Performance Considerations

1. **Computational Cost**: Minimal - only adds 3x21 matrix operations
2. **Memory**: Negligible additional overhead
3. **Update Frequency**: Independent of LiDAR frequency
4. **Latency**: Negligible (velocity updates processed immediately)

## Troubleshooting

### Issue: Poor convergence
- Check that `enable_velocity_measurement: true`
- Verify odometry topic is being published: `ros2 topic echo /eduard/odom`
- Increase `velocity_cov_*` values to reduce trust in odometry

### Issue: Drift increases
- Odometry measurements may be unreliable
- Try increasing `velocity_cov_*` values
- Or disable velocity measurement entirely

### Issue: Position estimates lag
- Odometry may have systematic bias
- Check for proper frame alignment
- Consider that velocity integration happens after LiDAR update

## References

- IESKF Paper: Brossard et al., "Invariant EKF for Visual Inertial Odometry" (2022)
- Kalman Filter Update: Simon, "Optimal State Estimation" (2006)

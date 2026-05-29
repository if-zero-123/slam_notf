---
name: mid360-px4imu-calibration
description: Calibrate Livox MID360 LiDAR with PX4/MAVROS IMU for FAST-LIO using LI-Init. Use when deploying LI-Init, recording calibration bags, interpreting LiDAR-to-IMU extrinsics, configuring FAST-LIO mid360.yaml for /mavros/imu/data_raw, or troubleshooting IMU acceleration scale, time offset, and extrinsic_R/T signs.
---

# MID360 PX4 IMU Calibration

Use this skill for the MID360 + PX4 IMU + FAST-LIO stack in `~/ros_ws`, especially when replacing `/livox/imu` with `/mavros/imu/data_raw`.

## Core Facts

- PX4/MAVROS raw IMU topic: `/mavros/imu/data_raw`.
- PX4 IMU acceleration is already `m/s^2`; static norm is about `9.8`.
- FAST-LIO in this workspace supports `common/imu_acc_scale`.
  - Livox built-in IMU: `imu_acc_scale: 9.81`.
  - PX4/MAVROS IMU: `imu_acc_scale: 1.0`.
- FAST-LIO extrinsics are LiDAR to IMU:
  - `p_imu = extrinsic_R * p_lidar + extrinsic_T`
  - `extrinsic_T` is the LiDAR origin expressed in the IMU frame.
  - `extrinsic_R` rotates LiDAR-frame points into the IMU frame.
- LI-Init result line `Homogeneous Transformation Matrix from LiDAR to IMU` can be copied directly into FAST-LIO `extrinsic_R/T`.
- In this FAST-LIO code, IMU timestamps are adjusted as:
  - `imu_stamp = raw_imu_stamp - time_offset_lidar_to_imu`
  - Therefore copy LI-Init `Time Lag IMU to LiDAR` into `time_offset_lidar_to_imu` with the same sign unless testing proves otherwise.

## Preconditions

Before calibration:

1. Remove propellers; do not fly while recording calibration data.
2. Keep PX4 IMU and MID360 rigidly fixed relative to each other. If either mount changes, recalibrate.
3. Start MID360 driver and MAVROS.
4. Verify topics:

```bash
rostopic hz /livox/lidar
rostopic hz /mavros/imu/data_raw
rostopic echo -n 1 /mavros/imu/data_raw
```

Expected PX4 IMU:

- Frequency near 100-250 Hz.
- `orientation_covariance[0] == -1` is OK.
- Static `linear_acceleration` norm about `9.8`.

## Deploy LI-Init

If LI-Init is absent:

```bash
cd ~/ros_ws/src/slam_notf
git clone https://github.com/hku-mars/LiDAR_IMU_Init.git
```

For this workspace, adapt original `livox_ros_driver` references to `livox_ros_driver2` in:

- `LiDAR_IMU_Init/CMakeLists.txt`
- `LiDAR_IMU_Init/package.xml`
- `LiDAR_IMU_Init/src/preprocess.h`
- `LiDAR_IMU_Init/src/preprocess.cpp`
- `LiDAR_IMU_Init/src/laserMapping.cpp`

If system Ceres is missing and sudo is unavailable, build local Ceres:

```bash
cd ~/ros_ws
mkdir -p third_party
git clone --branch 2.0.0 --depth 1 https://github.com/ceres-solver/ceres-solver.git third_party/ceres-solver
cmake -S third_party/ceres-solver -B third_party/ceres-build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/ros_ws/third_party/ceres_install \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
  -DMINIGLOG=ON -DSUITESPARSE=OFF -DCXSPARSE=OFF -DLAPACK=OFF
cmake --build third_party/ceres-build --target install -j4
```

In LI-Init `CMakeLists.txt`, add a local Ceres fallback before `find_package(Ceres REQUIRED)`:

```cmake
if(NOT Ceres_DIR)
  set(LOCAL_CERES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/ceres_install/lib/cmake/Ceres")
  if(EXISTS "${LOCAL_CERES_DIR}/CeresConfig.cmake")
    set(Ceres_DIR "${LOCAL_CERES_DIR}" CACHE PATH "Local Ceres config directory" FORCE)
  endif()
endif()
```

Build:

```bash
cd ~/ros_ws
catkin_make --pkg lidar_imu_init
source devel/setup.bash
```

## LI-Init Config

Use `LiDAR_IMU_Init/config/mid360.yaml`:

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"

preprocess:
    lidar_type: 1
    feature_extract_en: false
    scan_line: 6
    blind: 1

initialization:
    cut_frame_num: 5
    orig_odom_freq: 10
    mean_acc_norm: 9.805
    online_refine_time: 20
    data_accum_length: 500
```

Recommended launch:

```xml
<launch>
    <arg name="rviz" default="false" />
    <rosparam command="load" file="$(find lidar_imu_init)/config/mid360.yaml" />
    <node pkg="lidar_imu_init" type="li_init" name="lidar_imu_init" output="screen" />
    <param name="point_filter_num" type="int" value="2"/>
    <param name="max_iteration" type="int" value="5" />
    <param name="cube_side_length" type="double" value="2000" />
</launch>
```

## Record Calibration Bag

Record:

```bash
cd ~/ros_ws
source devel/setup.bash
rosbag record -O ~/mid360_px4imu_calib.bag /livox/lidar /mavros/imu/data_raw
```

Motion:

- First keep the vehicle static for 5-10 seconds.
- Move the whole vehicle, not the LiDAR alone.
- Excite roll, pitch, yaw, forward/back, left/right, and up/down.
- Record 60-120 seconds.
- Use a structured environment with walls, desks, pillars, or similar geometry.

## Run Calibration

Terminal 1:

```bash
cd ~/ros_ws
source devel/setup.bash
roslaunch lidar_imu_init mid360_px4imu_calib.launch
```

Terminal 2:

```bash
rosbag play ~/mid360_px4imu_calib.bag
```

Results are written to:

```text
~/ros_ws/src/slam_notf/LiDAR_IMU_Init/result/Initialization_result.txt
```

Prefer `Refinement result` over initial result when refinement reaches 100%.

## Write FAST-LIO Config

Fill `FAST_LIO/config/mid360.yaml`:

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"
    imu_acc_scale: 1.0
    time_sync_en: false
    time_offset_lidar_to_imu: <LI-Init Time Lag IMU to LiDAR>

mapping:
    extrinsic_est_en: false
    extrinsic_T: [ tx, ty, tz ]
    extrinsic_R: [ r11, r12, r13,
                   r21, r22, r23,
                   r31, r32, r33 ]
```

Example from a good calibration:

```yaml
common:
    imu_topic:  "/mavros/imu/data_raw"
    imu_acc_scale: 1.0
    time_offset_lidar_to_imu: -0.020980

mapping:
    extrinsic_T: [ -0.008110, 0.006676, 0.155897 ]
    extrinsic_R: [ 0.858658, -0.008207,  0.512484,
                   0.014996,  0.999846, -0.009115,
                  -0.512330,  0.015511,  0.858649 ]
```

## Validate

1. Run FAST-LIO on the recorded bag first.
2. Then run live while hand-moving the entire vehicle.
3. Then test motors without taking off.
4. Only fly after `/Odometry` is stable in the above tests.

If the platform drifts only when motors spin, suspect vibration, self-points, or non-rigid relative motion between the flight controller and LiDAR.

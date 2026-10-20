# PX4 IMU + Livox MID360s + FAST-LIO2/LI-Init (ROS 1 Noetic)

## 1. What the measured 50 ms means

A 10 Hz Livox frame contains about 100 ms of points. `header.stamp` is the first
point time and `offset_time` gives each point's relative acquisition time. The
measured `end_age` is therefore the delivery/assembly latency of the final point,
not automatically a clock offset. Do **not** add 50 ms to the point cloud merely
to make `end_age` zero; FAST-LIO consumes measurement timestamps and can tolerate
messages arriving later as long as IMU buffering covers the scan and both streams
share one clock domain.

## 2. Time architecture

Recommended chain:

1. Jetson system clock is the ROS clock.
2. MID360s synchronizes to Jetson over PTP on `eth0` (`192.168.1.5`).
3. MAVROS `TIMESYNC` maps PX4 boot-time IMU samples into the Jetson/ROS epoch.
4. FAST-LIO keeps `time_sync_en: false`; only the calibrated residual constant
   is written to `time_offset_lidar_to_imu`.

The Jetson `eth0` adapter reports software timestamp support but no PTP Hardware
Clock. Run linuxptp in software timestamp mode:

```bash
sudo apt-get update
sudo apt-get install -y linuxptp
sudo ptp4l -i eth0 -S -m -f \
  ~/gcs_ws/src/livox_ros_driver2/config/time_sync/ptp4l-mid360s.conf
```

Keep this terminal open before powering/starting the LiDAR. In the Livox driver
console, verify that the timestamp mode changes to `PTP/gPTP`, not `NoSync`.

> Do not run `phc2sys` for `eth0`: this NIC has no PHC. Keep Jetson wall time
> synchronized separately (NTP/chrony) when internet/GNSS time is required.

## 3. MAVROS/PX4 timestamp check

MAVROS Noetic's PX4 defaults use MAVLink TIMESYNC at 10 Hz. Start MAVROS as usual,
then check:

```bash
rostopic echo /mavros/timesync_status
rostopic hz /mavros/imu/data_raw
```

The `estimated_offset_ns` absolute value is normally large because PX4 and Unix
clocks have different epochs. Judge synchronization by a stable offset and low,
stable `round_trip_time_ms`.

Start the Livox driver and run the combined check:

```bash
roslaunch livox_ros_driver2 msg_MID360s.launch
rosrun livox_ros_driver2 check_px4_livox_time_sync.py _duration:=30
```

A healthy run has monotonic stamps, a scan span near 100 ms, no one-hour epoch
warnings, stable MAVROS offset, and a positive IMU lead at LiDAR callback time.

## 4. LI-Init extrinsic and residual time calibration

Install Ceres once:

```bash
sudo apt-get install -y libceres-dev
cd ~/gcs_ws
catkin_make -DROS_EDITION=ROS1
source devel/setup.bash
```

Run all three data sources/algorithms in separate terminals:

```bash
roslaunch mavros px4.launch fcu_url:=<your-fcu-url>
roslaunch livox_ros_driver2 msg_MID360s.launch
roslaunch lidar_imu_init livox_mid360s_px4.launch
```

Calibration motion:

- Hold still for 5-10 seconds first.
- Then rotate around all three axes with sufficiently rich angular motion.
- Add translations in several directions; avoid only yawing on a table.
- Continue through the 20-second online refinement stage.
- Keep the airframe disarmed and remove propellers during hand motion.

Result file:

```text
~/gcs_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

Use the **Refinement result** homogeneous matrix. FAST-LIO expects exactly the
same convention used by LI-Init:

```text
p_IMU = R_LI * p_LiDAR + T_LI
```

Copy the top-left 3x3 block to `mapping/extrinsic_R`, the top-right 3x1 block to
`mapping/extrinsic_T`, and copy `Time Lag IMU to LiDAR (second)` to
`common/time_offset_lidar_to_imu` in:

```text
~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

Repeat the calibration at least three times. Accept it only if rotation,
translation, and residual time offset repeat closely and agree with tape-measure
mount geometry.

## 5. Run FAST-LIO2

After replacing the identity/zero placeholders with LI-Init results:

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360_px4.launch
```

Leave `time_sync_en: false` when PTP + MAVROS TIMESYNC is working. Enabling
FAST-LIO's simple software synchronization should be a fallback only, not the
normal configuration.

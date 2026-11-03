# MID360 passage detector (ROS 2 Humble)

No `multiple_object_tracking_lidar` dependency. Standalone Python proof of concept.

## Purpose

Receive MID360 `/livox/lidar` as **sensor_msgs/msg/PointCloud2**, transform to `base_link`, filter/downsample, cluster, assign temporary track IDs, and detect relative crossings alongside the robot.

- `REAR_TO_FRONT`: target passes from x < -threshold to x > +threshold in base_link.
- `FRONT_TO_REAR`: target passes from x > +threshold to x < -threshold.

**IMPORTANT:** This detects the direction of a *relative crossing*, not whether the target is a robot or whether it autonomously overtook you. With a moving/rotating observer, stationary obstacles can produce the same signal. Test with the observer stationary first. A LiDAR-only target-classifier / fixed-frame velocity test is a later enhancement.

## 0. Check sensor message type

```bash
ros2 topic type /livox/lidar
```

Must say `sensor_msgs/msg/PointCloud2`. If it says `livox_ros_driver2/msg/CustomMsg`, configure Livox driver for PointCloud2 instead or add a conversion node.

## 1. Install (inside Docker, if using Docker)

```bash
sudo apt update
sudo apt install -y python3-numpy python3-scipy ros-humble-sensor-msgs-py ros-humble-tf2-ros ros-humble-visualization-msgs
cd ~/ros2_ws/src
# Copy this directory to ~/ros2_ws/src/mid360_passage_detector
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mid360_passage_detector
source install/setup.bash
```

If the previously failing tracking package still resides in `src`, `--packages-select mid360_passage_detector` is intentional; this avoids rebuilding unrelated packages.

## 2. Required TF

`ros2 run tf2_ros tf2_echo base_link livox_frame`

This TF must exist at each LiDAR point cloud timestamp. During rosbag playback with `/clock`, set `use_sim_time:=true` and publish `/tf_static` for the LiDAR mounting frame. Adjust `base_frame` if needed.

## 3. Run

```bash
ros2 launch mid360_passage_detector passage_detector.launch.py
# for simulated time / rosbag with /clock:
ros2 launch mid360_passage_detector passage_detector.launch.py use_sim_time:=true
```

Inspect:

```bash
ros2 topic hz /livox/lidar
ros2 topic echo /tracked_objects
ros2 topic echo /passing_direction
ros2 topic hz /tracked_objects
```

In RViz2 set Fixed Frame=`base_link` and add MarkerArray `/tracked_objects_markers`; IDs are drawn as text. To see the raw cloud add PointCloud2 `/livox/lidar` (with appropriate TF).

## 4. First trial

1. Stop the observer robot. Place just one moving target beside it.
2. Pass target from behind to in front, then repeat front to rear.
3. Confirm `/tracked_objects` emits persistent IDs, then `/passing_direction` reports `REAR_TO_FRONT` or `FRONT_TO_REAR`.
4. Inspect empty scene for false clusters, and tune `config/params.yaml`.

Key settings:
- `min_z_m`, `max_z_m` measured in **base_link** (not laser frame); must match mounting and floor. Do not assume `base_link` is exactly floor height.
- `self_radius_m` approximates the robot circular footprint; rectangular robots benefit from a polygon filter later.
- `cluster_tolerance_m`, `min_cluster_points` tune the cluster formation after voxel downsampling; too large merges walls and robots, too small fragments objects.
- `association_distance_m`, `max_missed_s` determine ID stability.
- `x_threshold_m`, `side_min_m`, `side_max_m` set the crossing detection corridor.

## 5. Limits and development notes

- This is a minimal proof of concept, **not a safety function**.
- It clusters every visible 3D object with a simple height cut, not semantic robots. If a wall/plant enters the corridor, it may be tracked as a target.
- No ground-plane estimation; slopes, low or tall robots and dense sensor returns may require adjustment.
- Tracking is nearest predicted cluster center using Hungarian assignment, not a full Kalman filter; IDs can change during occlusion, splitting or merging.
- Each cloud produces one tracking update; no multi-frame point accumulation.
- To suppress stationary obstacles while the observer moves, use odom-frame displacement/velocity (TF at each timestamp), plus gating for target motion. Relative crossing events are a different question from actual overtaking.
- A timestamped `PointCloud2` is required; point-by-point motion distortion within a scan is not compensated.
- `tracked_objects` is JSON encoded `std_msgs/String` to simplify building; for production migrate to custom ROS messages.

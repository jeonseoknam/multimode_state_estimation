# Project Notice

This repository (`multimode_state_estimation`) is a **standalone modified
copy** of [`jeonseoknam/3d_lidar_state_estimation`](https://github.com/jeonseoknam/3d_lidar_state_estimation)
adapted for the fault-tolerant LiDAR-Visual-GNSS multimodal localization
pipeline ([`fault-tolerant-localization-pipeline`](https://github.com/jeonseoknam/fault-tolerant-localization-pipeline)).

## Authorship

The author of both repos is the same (`jeonseoknam`). This repo is split from
the original `3d_lidar_state_estimation` so the multimode-localization
configuration evolves independently from any other 3D state-estimation work.

## Bundled third-party packages (under `deps/`)

The pose-graph + EKF stack pulls in several Autoware Universe packages,
vendored here for build self-containedness:

- `autoware_common/autoware_cmake`, `autoware_lint_common`,
  `autoware_common/tmp/lanelet2_extension`
- `common/kalman_filter`, `common/tier4_autoware_utils`
- `tier4_autoware_msgs/tier4_debug_msgs`
- `autoware_auto_msgs/autoware_auto_{vehicle,planning,perception,mapping,geometry}_msgs`

All retain their original Apache-2.0 licenses and copyright notices. The
LiDAR driver `Livox-SDK2` retains its upstream MIT license.

## Modifications in this fork

- Removed unused `carstate_3d` and `livox_ros_driver2` subdirectories
- Added `odom_to_twist_converter`, `pose_to_pose_with_cov` packages used by
  the multimode pipeline
- EKF localizer / gyro odometer launch and core tweaks for the MORAI
  3-modality fusion experiments

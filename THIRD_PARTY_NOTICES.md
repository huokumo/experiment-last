# 第三方组件与许可

本仓库自身为 BSD-3-Clause（见 [LICENSE](LICENSE)）。下列组件按各自许可使用，**均未修改源码**，只通过路径引用或运行时依赖：

## 1. experiment5（R1 模型与深度相机）

* 来源：<https://github.com/huokumo/experiment5>
* 固定提交：`807de9e2454d0c6bd7ab3067e6743acf429e19f9`
* 许可：BSD-3-Clause
* 使用内容：`models/unitree_r1/model_depth.sdf`（含 `depth_camera` 传感器定义）、`description/r1_depth.urdf`、`models/` 资源目录
* 说明：本实验只**读取**该模型并在自己的世界文件里追加两个 Gazebo 插件，不改动模型文件本身。R1 本体模型另有 Unitree 的 BSD-3-Clause 声明（见该仓库 `LICENSE.unitree`）。

## 2. experiment7（Gazebo 系统插件与场景约定）

* 来源：<https://github.com/huokumo/experiment7>
* 固定提交：`9ed58e7330504337377b973c1da227c8bd2a9715`
* 许可：BSD-3-Clause
* 使用内容：
  * `libmid360_teaching_system.so`（CPU 光线投射，扮演 Mid-360）
  * `libr1_planar_motion_system.so`（平面运动与 2D 保护判据）
  * `scripts/state_node.py`（发布 `base_link → lidar_frame`、`/livox/imu`）
  * `vendor/livox/mid360.csv`（扫描图案）
* 说明：两个 `.so` 由 `tools/prepare_workspace.sh` 拉取固定提交后编译，本仓库**不复制、不改写**插件源码。实验 7 的文档里已声明该插件是教学用简化模型，不代表真实设备特性。

## 3. Livox 扫描图案

* 来源：<https://github.com/Livox-SDK/livox_laser_simulation>
* 提交：`1cce1073633a062b92e30243a4c2920e45551bb5`，文件 `scan_mode/mid360.csv`
* 许可：MIT
* 说明：只使用方向图案。CSV 第一列不被当作时间使用（详见 experiment7 的 `vendor/livox/SOURCE.json`）。

## 4. 运行时依赖（由 apt 安装，不随仓库分发）

| 组件 | 许可 |
| --- | --- |
| ROS 2 Humble（rclpy、message_filters、tf2、nav2、rviz2） | Apache-2.0 / BSD-3-Clause |
| Gazebo Fortress（ignition-gazebo6） | Apache-2.0 |
| ros_gz（ros_gz_sim / ros_gz_bridge） | Apache-2.0 |
| NumPy、PyYAML | BSD-3-Clause / MIT |

## 5. 本仓库生成的内容

`config/map/warehouse.pgm` 与 `config/map/warehouse.yaml` 由 `tools/make_map.py` 从 `experiment9_core/scene9.py` 的 `STATIC_BOXES` 光栅化生成；`config/fusion.yaml` 的外参数值由 `tools/derive_extrinsics.py` 从 SDF 的关节链计算得到。两者都可以随时重新生成，不需要手工维护。

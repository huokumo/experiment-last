# 接口与坐标约定

本文件是"这条链上到底传了什么"的唯一依据。改任何一项，都要同步 `config/`、`tests/` 与本文件。

## 1. TF 链

```
map ──静态恒等──> odom ──50 Hz──> base_link ──静态──> lidar_frame
                                        └──────> depth_camera_link ──> depth_camera_optical_frame
```

| 变换 | 发布者 | 说明 |
| --- | --- | --- |
| `map → odom` | `nav_demo.launch.py` 里的 `static_transform_publisher` | 恒等。仿真里程计无漂移，没有需要校正的东西 |
| `odom → base_link` | `scripts/odom_tf.py` | 由 `/odom`（仿真真值）转成 TF，50 Hz |
| `base_link → lidar_frame` | 实验 7 的 `state_node.py` | 纯平移 `(0,0,1.3)` |
| `base_link → …` 机器人各连杆 | `robot_state_publisher` | 来自实验 5 的 `r1_depth.urdf` |

`state_node.py` 的 `publish_truth_tf` 必须保持 `false`：否则它会再发一条 `world → base_link`，`base_link` 就有两个父节点，TF 树直接报错。

## 2. 坐标系定义

| 系 | 轴向 | 备注 |
| --- | --- | --- |
| `map` / `odom` / `base_link` / `lidar_frame` | x 前、y 左、z 上 | 右手系 |
| `depth_camera_optical_frame` | x 右、y 下、z 前 | 光学惯例，不是 ROS 惯例 |

`base_link` 在**地面高度**（R1 的 `pelvis` 在其上方 0.7436 m），不是质心。这一点决定了 1.3 m 的雷达安装高度和 1.154 m 的相机高度都是相对地面的。

## 3. 外参（由 `tools/derive_extrinsics.py` 从 SDF 计算）

```yaml
# base_link -> lidar_frame
lidar_translation: [0.0, 0.0, 1.3]
lidar_rotation_rowmajor: [1,0,0, 0,1,0, 0,0,1]

# base_link -> depth_camera_optical_frame
depth_translation: [0.185795, 0.0, 1.154059]
depth_rotation_rowmajor: [ 0.0, -0.333487,  0.942755,
                          -1.0,  0.0,       0.0,
                           0.0, -0.942755, -0.333487]
```

光轴在 `base_link` 下为 `(0.943, 0, -0.333)`：朝前、下俯 19.48°。

ROS 2 参数不支持二维数组，所以矩阵按行摊平成 9 个数；`cloud_io.extrinsic()` 负责校验正交性与 `det=+1`。

## 4. 话题

### 输入

| 话题 | 类型 | 频率 | 坐标系 | 来源 |
| --- | --- | --- | --- | --- |
| `/livox/points` | `sensor_msgs/PointCloud2` | 5 Hz | `lidar_frame` | 实验 7 的 CPU 光线投射插件（字段含 `offset_time`） |
| `/r1/depth/image` | `sensor_msgs/Image` | 2 Hz | — | Gazebo `depth_camera`，`32FC1`，单位米 |
| `/r1/depth/camera_info` | `sensor_msgs/CameraInfo` | 2 Hz | — | fx=fy=138.56, cx=80, cy=60；话题无数据时节点按 `depth_hfov` 现算，结果相同 |
| `/odom` | `nav_msgs/Odometry` | 50 Hz | `odom` | 平面运动系统的真实位姿（即真值，不另设 `/r1/ground_truth`） |
| `/r1/collision_blocked` | `std_msgs/Bool` | 事件 | — | 2D 圆 vs 矩形保护判据 |
| `/clock` | `rosgraph_msgs/Clock` | — | — | 必须 `use_sim_time:=true` |

### 输出

| 话题 | 类型 | 频率 | 坐标系 | 说明 |
| --- | --- | --- | --- | --- |
| `/r1/fusion/points` | `sensor_msgs/PointCloud2` | ≈2 Hz（受深度帧率限制） | `base_link` | 融合结果，Nav2 两个代价地图都订阅它 |
| `/r1/fusion/points_lidar` | `sensor_msgs/PointCloud2` | ≈2 Hz | `base_link` | 只有雷达那一路：TODO 1 写完即出现，供分步验证 |
| `/r1/fusion/points_depth` | `sensor_msgs/PointCloud2` | ≈2 Hz | `base_link` | 只有深度那一路：TODO 2 写完即出现 |
| `/r1/fusion/status` | `std_msgs/String` | ≈2 Hz | — | JSON：`stage`（`lidar`/`lidar+depth`/`lidar+depth+merged`）、三路点数、`pair_dt`、`z_min/z_max` |
| `/r1/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | — | Nav2 → 仿真，限速 0.25 m/s、0.5 rad/s |

**融合结果发布在 `base_link`，不是 `map`。** 代价地图已经知道怎么从 `base_link` 走到 `map`；发布在 `map` 会把机器人位姿重复叠加一次。

## 5. 参数

`config/fusion.yaml`（融合节点）

| 参数 | 值 | 依据 |
| --- | --- | --- |
| `mode` | `student` / `solution` / `lidar` / `depth` | 学生版 / 参考实现 / 单路负向验证（丢一路） |
| `z_min` | 0.10 | 去掉地面；低于此值的点会让车被地面困死 |
| `z_max` | 1.20 | 车高 0.5 m；K4 横梁 1.6 m 必须落在带外 |
| `voxel_lidar` | 0.10 | 雷达 10 m 处点距约 5 cm |
| `voxel_depth` | 0.04 | 深度 1 m 处点距约 6 mm |
| `sync_slop` | 0.60 | 配对新鲜度窗口取 `max(2.0, 3×sync_slop)`；深度 2 Hz，窗口必须大于一帧 |
| `depth_min_range` / `depth_max_range` | 0.10 / 10.0 | 与 SDF 的 `clip` 一致 |

`config/nav2_params.yaml`（关键三项）

| 参数 | 值 | 与融合的关系 |
| --- | --- | --- |
| `obstacle_layer.cloud.topic` | `/r1/fusion/points` | 融合结果进代价地图的唯一入口 |
| `max_obstacle_height` | 2.0 | 比融合的 `z_max` 宽，防止二次误杀 |
| `robot_radius` | 0.45 | 与仿真 2D 保护判据一致，否则"感知说能过、保护说撞" |

## 6. 时间配对

* Gazebo 用 `/clock`，所有节点必须 `use_sim_time: true`。
* **不用消息头里的 `stamp` 做配对**：教学镜像上实测 `/livox/points` 的 `stamp` 为 `3650 s`、
  `/r1/depth/image` 为 `1696 s`，两路相差 1955 s（来自不同时钟）。`ApproximateTimeSynchronizer`
  因此一帧都配不上，节点不报错也不输出，`/r1/fusion/status` 一条都没有。
  现在按"最新的雷达帧 + 最新的深度图"配对，新鲜度看**到达时刻**，窗口
  `max(2.0, 3×sync_slop)`。
* 输出的三路点云统一带**深度图的时间戳**（仿真时钟）：`/global_costmap` 要拿这个 `stamp`
  查 `base_link→map`，沿用雷达那个时钟会让 TF 外推出界、整帧被丢掉。
* 节点把两路 `stamp` 的真实差值打进 `/r1/fusion/status` 的 `pair_dt`，首次超过 5 s 时打一条
  warn；报告里要引用这个数，并说明"配对按到达时刻、输出用深度图时间戳"。
* 雷达点云带逐点 `offset_time`（相对该帧起始，纳秒）。本实验不做运动补偿：0.22 m/s × 0.2 s 扫一帧 ≈ 4.4 cm，小于 K1 的余量，写进报告的"局限"里。

## 7. 场景几何

`experiment9_core/scene9.py` 是两个列表的唯一来源：

* `STATIC_BOXES` —— 房间、蓝墙、两根柱子、木箱、内墙与门洞。**已烘焙进 `config/map/warehouse.pgm`。**
* `FUSION_OBSTACLES` —— K1–K5。**只在实时场景里，不在静态地图里。**

同一个列表驱动三个消费者：Gazebo 世界里的可视/碰撞几何、CPU 光线投射的 `<obstacle>` 列表、2D 保护判据的 `<obstacle>` 列表。K4 横梁**只进光线投射、不进保护判据**——1.6 m 高的横梁拦不住 0.5 m 高的机器人，而 2D 判据没有高度概念，给它就会把路堵死。

| 关卡 | 中心 (x, y, z) | 尺寸 | 阻断运动 |
| --- | --- | --- | --- |
| K1 低矮托盘 | (-1.70, -3.25, 0.175) | 1.20×0.80×0.35 | 是 |
| K2 细立柱 | (0.30, -0.85, 1.00) | 0.10×0.10×2.00 | 是 |
| K3 薄板 | (1.55, 0.35, 1.00) | 0.04×1.20×2.00 | 是 |
| K4 悬空横梁 | (3.875, 0.30, 1.60) | 1.45×0.15×0.15 | 否 |
| K5 侧后方货架 | (2.60, 2.15, 0.75) | 0.60×0.60×1.50 | 是 |

起点 `(-4.4, -3.4)`，目标 `(4.6, 2.6)`。

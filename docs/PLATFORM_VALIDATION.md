# 平台运行验证记录

**状态：待执行。** 本文件是空表格，用来在教学平台上跑一遍后逐项填写。
不要在没有实际运行的情况下把它改成"通过"——这张表是这本实验能不能发出去的唯一依据。

填写方式：每项写清「命令 / 预期 / 实际 / 结论」，实测数据放在 `docs/evidence/` 下（JSON 或 txt，不要只截图）。

## 0. 环境

| 项 | 值 |
| --- | --- |
| 日期 | |
| 镜像 / 系统版本 | Ubuntu 22.04，ROS 2 Humble，Gazebo Fortress |
| 机器配置（CPU/内存/是否 GPU） | |
| 工作区路径 | |
| 是否用教学 ZIP 资源包 | |

## 1. 编译

```bash
bash tools/install_dependencies.sh
bash tools/prepare_workspace.sh
bash tools/build.sh
```

| 检查 | 预期 | 实际 | 结论 |
| --- | --- | --- | --- |
| 三个包全部编译 | `experiment5_depth` / `experiment7_lidar` / `experiment9_fusion` 均成功 | | |
| `experiment7_lidar/lib` 下有 `libmid360_teaching_system.so` 与 `libr1_planar_motion_system.so` | 两个文件存在 | | |
| `experiment9_fusion` 的脚本可执行 | `ros2 run experiment9_fusion check_ready.py` 能找到 | | |

## 2. 起场景

```bash
ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution
ros2 run experiment9_fusion check_ready.py          # 另一终端
```

| 检查 | 预期 | 实际 | 结论 |
| --- | --- | --- | --- |
| Gazebo 中能看到房间、蓝墙、五个关卡 | 目视确认 | | |
| 机器人出生在 (-4.4, -3.4) | 西南角 | | |
| `check_ready.py` | 4/4 通过 | | |
| `/livox/points` 频率 | 约 5 Hz | | |
| `/r1/depth/image` 频率 | 约 2 Hz | | |
| `/r1/fusion/points` 频率 | 约 8 Hz（跟雷达同频，深度图按 2 Hz 刷新） | | |
| `/odom` 频率 | 约 50 Hz | | |

> 若 `/r1/fusion/points` 只有 2 Hz 左右，说明深度图断流、配对窗口以外的帧被丢掉；若为 0 Hz 且 `/r1/fusion/status` 一条都没有，先查两路 `stamp` 是否来自不同时钟（见附录 4）。

## 3. 融合质量

```bash
ros2 run experiment9_fusion cloud_check.py --wall
```

| 检查 | 预期（`tools/simulate_pipeline.py` 沿路径 37 个位姿的预测） | 实际 | 结论 |
| --- | --- | --- | --- |
| K1 深度列 | 起步就可见，峰值 > 1000 点 | | |
| K1 雷达列 | **= 0**（7.52 m 内 0.35 m 高在最低射线之下） | | |
| K2 雷达列 | 全程都有数，峰值约 100 点 | | |
| K2 深度列 | 4 m 外接近 0（不足 2 像素），进到 3 m 内才涨到数百（峰值约 960） | | |
| K3 雷达列 / 深度列 | 两路都有数（峰值约 180 / 2300），形态不同：稀疏竖线 vs 一整片 | | |
| K4 深度列 | **= 0**（全程，横梁在相机光轴上界之外） | | |
| K4 雷达列 | 有数，峰值 > 1000 点 | | |
| K5 雷达列 / 深度列 | 两路都有数（峰值约 100 / 300），雷达在车还没转向时就报到 | | |
| 蓝墙墙距偏差（两路差） | < 5 cm | | |

> 判据是「**全程至少有一次看到**」，不是「每一帧都看得到」。`cloud_check` 只测当前位姿：在起点量时 K2、K5 的深度列可能是 0，属正常。想复现上表，把车沿路径开一遍，每个位姿各量一次取峰值。

## 4. 代价地图

```bash
ros2 run experiment9_fusion costmap_dump.py --save docs/evidence/costmap_start.txt
```

| 检查 | 预期 | 实际 | 结论 |
| --- | --- | --- | --- |
| K1 位置出现 `@` | 有 | | |
| K5 位置出现 `@` | 有 | | |
| K4 横梁处**没有**致命格 | 无 | | |
| `A`（机器人）位置与 Gazebo 一致 | 一致 | | |

## 5. 端到端导航

```bash
ros2 run experiment9_fusion record_run.py --out docs/evidence/run_reference.csv
ros2 run experiment9_fusion send_goal.sh          # 另终端
```

| 检查 | 预期 | 实际 | 结论 |
| --- | --- | --- | --- |
| `send_goal` 结果 | `status: SUCCEEDED` | | |
| 到达耗时 | < 120 s（13.95 m 路径，0.22 m/s） | | |
| 碰撞帧 | 0 | | |
| 终点距离 | < 0.30 m | | |
| 最小安全距离 | > 0 | | |
| 恢复行为次数 | ≤ 2 | | |
| 路径是否绕开 K1、K2、K3 | 目视 + `/plan` | | |

## 6. 负向验证（必须做）

这两项决定实验有没有教学价值：能跑通不算，**能按预期跑坏才算**。

| 操作 | 预期现象 | 实际 | 结论 |
| --- | --- | --- | --- |
| `z_max` 改 2.0 | 车在 K4 横梁前停死；`costmap_dump` 该处出现 `@` | | |
| `z_min` 改 0.0 | 车起步就被地面困死；`cloud_check` 总点数暴涨 | | |
| `mode:=lidar` | 车撞 K1 后被拦住（`collision_blocked` 持续 true） | | |
| `mode:=depth` | 车在 K2 附近反复修正 / 被拦 | | |
| TODO 1 的 `t.z` 写成 -1.3 | 点云整体低 2.6 m；`cloud_check --wall` 墙距偏差约 260 cm | | |

## 7. 性能

| 项 | 实测 |
| --- | --- |
| 实时率（`gui:=true`） | |
| 实时率（`gui:=false rviz:=false`） | |
| 融合节点单帧耗时 | |
| 虚拟内存占用峰值 | |

## 8. 结论

| 项 | 结论 |
| --- | --- |
| 是否可以在课堂上使用 | |
| 需要修的问题 | |
| 建议的课时调整 | |

## 附录：2026-11-12 首次平台实测发现

教学镜像（Ubuntu 22.04，无 GPU，软件渲染）上按手册第 1～5 步实跑，发现两个静态检查
抓不到、只在运行时暴露的问题，均已修复。

### 1. 教学包解压后脚本没有执行位

`src/experiment7/scripts/*.py` 全部是 `644`。`--symlink-install` 装进 `install/` 的是
指向源码的符号链接，权限跟着源码走，于是启动时报
`executable 'state_node.py' not found on the libexec directory ...`。压缩包里记录的
权限位是 755，是资源库转存时丢掉了 Unix 权限位。

处理：手册第 2 步新增一条 `chmod`；`tools/build.sh` 编译前统一补 `+x`，安装自检
从 `-e` 改为 `-x`；`experiment9.launch.py` 区分"缺失"与"存在但不可执行"。

### 2. 生成的 SDF 缺 Sensors 系统插件，深度相机不出帧

`scene9.generate` 只写了 Physics、UserCommands、SceneBroadcaster 三个 world 插件。
实验五的两个 world 里都显式写了 `libignition-gazebo-sensors-system.so`，本实验的 SDF
是代码现拼的，漏了这一个。表现极易误判：

| 话题 | 实测 |
| --- | --- |
| `/livox/points` | 8.0 Hz（正常，雷达是实验七自写的 CPU 光线投射插件，不走渲染） |
| `/r1/depth/image` | **0.0 Hz** |
| `/r1/fusion/points` | 0.0 Hz（`mode:=student` 未实现 TODO 时的正常表现） |
| `/odom`、TF 链、`/map`、代价地图、`navigate_to_pose` | 全部正常 |

处理：`scene9.generate` 补第四项插件，`render_engine` 与实验五一致取 `ogre2`；
`tools/preflight.py` 增加第 15 项检查（世界含 Sensors 插件且实验五模型里确有
`depth_camera` 传感器），这类问题下次在静态阶段就能拦住。

### 3. 深度相机不出 CameraInfo 时整条链路静默

补上 Sensors 插件后 `/r1/depth/image` 恢复到 1.5 Hz，但 `/r1/fusion/points` 仍是 0 Hz。
原因不在融合算法：`fusion_node._on_pair` 开头就要求 `self.camera_info` 非空，否则
直接 `return`，于是连 `/r1/fusion/status` 都不发。

Gazebo 的深度相机只有在渲染循环真的在出图、且 info 话题有订阅者时才发 CameraInfo
（gz-sensors#309）。软件渲染的虚拟机上这个话题经常长时间没有数据，症状是"雷达有、
深度图有、融合没有"，很容易被误判成 TODO 写错。

处理：

* `fusion_node` 在 CameraInfo 缺席时按图像尺寸与 `depth_hfov` 现算内参
  （`fx = (W/2)/tan(hfov/2)`，`cx = W/2`、`cy = H/2`），与相机自报的值一致
  （138.56 / 80 / 60），只在第一次打一条 warn；
* `/r1/fusion/status` 增加 `intrinsics` 字段，标明用的是 `camera_info` 还是 `geometry`；
* `check_ready.py` 订阅 `/r1/fusion/status`，`/r1/fusion/points` 为 0 Hz 时能区分
  "一帧都没处理"（配对/话题问题）与"处理了但没产出"（TODO 问题）。

### 4. 两路时间戳来自不同时钟，近似同步一帧都配不上

补上 Sensors 插件、`/r1/depth/image` 恢复到 1.8 Hz 之后，`/r1/fusion/points` 仍是 0 Hz，
而且 `/r1/fusion/status` **一条都没有**。取两路的时间戳：

| 话题 | `header.stamp` |
| --- | --- |
| `/livox/points` | `3650.001 s` |
| `/r1/depth/image` | `1696.000 s` |

相差 1955 s。`ApproximateTimeSynchronizer(slop=0.6)` 按 `stamp` 配对，因此一帧都没配上，
节点既不报错也不输出——症状和"TODO 没写"完全一样，是最容易误判的一种。

处理：

* `fusion_node` 改为按**到达时刻**配对（最新的雷达帧 + 最新的深度图，窗口
  `max(2.0, 3×sync_slop)`），不再用 `message_filters`；
* 三路输出点云统一改带**深度图的时间戳**（仿真时钟）。沿用雷达那个时钟会让
  `/global_costmap` 查 `base_link→map` 时外推出界，点云被整帧丢掉；
* `pair_dt` 继续上报两路 `stamp` 的真实差值，首次超过 5 s 时打一条 warn 说明原因；
* 增加 1 Hz 看门狗：任一路断流 5 s 就报 `no /livox/points for 5 s`，不再"静默"。

### 5. 深度图时间戳本身也可能不是 /clock 那个时钟

第 4 条按"深度图的时间戳来自仿真时钟"做的前提，在换机器、换 Gazebo 版本时不总成立：
`ros_gz_bridge` 对图像类消息的 `header.stamp` 偶尔会带上系统时钟。这种情形下融合点云
仍然发得出来，但代价地图拿这个 stamp 去查 `odom→base_link` 会外推出界，整帧被丢，
`/global_costmap` 里看不到障碍，表现是"融合正常、导航撞墙"。

处理：`fusion_node` 每帧把深度图的 `stamp` 与节点自己的时钟比一次，差超过 2 s 就改用
节点时钟（`get_clock().now()`，与 `/clock`、与 TF 的 `odom→base_link` 同源），
只在第一次打一条 warn，并在 `/r1/fusion/status` 的 `stamp_source` 字段里标成 `clock`
（正常情况是 `image`）。这一层是兜底，不是常规路径：正常运行时不会触发。

### 6. 规划路径在拐角处被 2D 保护判据判撞，车在空地上原地打转

现象：融合 4/4 通过、`/r1/fusion/points` 8 Hz、代价地图里 K1 的 `@` 也标出来了，
但 `send_goal.sh` 发出去之后车只走了 1.5 m 左右就停住，位置在十几位小数上
纹丝不动、yaw 却以 0.31 rad/s 匀速转，`number_of_recoveries` 一路涨到 30+，
`distance_remaining` 一步不降，最后 `Result: result: {}` 直接 abort。
Gazebo 里看就是"停在一片空地上"，四周根本没有任何东西。

原因：两个模型在拐角处不一致。

* Nav2 把车当**圆**，`robot_radius = 0.45`，规划器在"离障碍 0.45 m"处就算自由；
* 实验七的 `planar_motion_system` 把每个障碍箱按**每个轴各扩 0.45 m** 来判
  （`segmentHitsBox` 里 `extent = half + radius`，是切比雪夫距离，不是欧氏）。

直墙上两者一样，拐角处差 0.19 m（`0.45×(√2−1)`）。而原来的膨胀层只有
`inflation_radius = 0.55 / cost_scaling_factor = 3.0`，软圈只有 0.10 m 宽，
几乎没有把路径往通道中间推的力，NavFn 于是贴着障碍边缘 0.55 m 走——
在 45° 拐角上切比雪夫距离只剩 0.39 m，低于保护判据要求的 0.45 m。
车开到拐角前 0.5 m 处，RPP 的 `use_collision_detection` 先看到前方的
253 格、判 Collision Ahead 退出，BT 进恢复、原地 spin，重新规划还是同一条路，
于是无限循环。

处理：把膨胀圈放大到 `inflation_radius = 0.90`、`cost_scaling_factor = 1.5`
（local 与 global 两处）。软圈变成 0.45 m 宽、衰减更平缓，NavFn 为省代价
自然走到通道中间。离线核算：路径 15.38 m → 16.58 m，离障碍最小欧氏距离
0.55 → 0.73 m，最小切比雪夫距离 0.39 → 0.64 m，两个模型都接受。

同时新增 `tools/check_route.py` 并接进 `tools/preflight.py`（第 16 项）：
它先用 Nav2 的圆形+膨胀代价图规划一遍，再用保护判据的方形膨胀逐点复核，
任何一步不合法就 FAIL。这类"两个模型各对一半"的问题，只有把两边放在一起
才能离线复现，RViz 里是看不出来的。

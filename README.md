# 实验10：仿真环境下的多传感器融合感知

> **编号约定**：本仓库为 `experiment-last`（`https://github.com/huokumo/experiment-last`）、包名 `experiment9_fusion`、工作区 `/root/exp9`（沿用系列 `exp(N−1)` 的偏移），对应学生手册的**实验10**。
> 下文"实验 N"沿用**仓库编号**：实验 5 = `experiment5_depth`（R1 模型与深度相机）、实验 7 = `experiment7_lidar`（Gazebo 插件）、实验 8 = `experiment8_slam`（FAST-LIO）。换算成手册编号各 +1。

承接实验 5、实验 7 的同一台 R1 机器人和同一间教室场景：**把深度相机的点云和激光雷达的点云对齐到同一个坐标系、合并成一片，交给 Nav2，然后发一个目标点，让机器人自己开过去。**

4 课时。学生只改一个文件里的三个函数。

## 一句话说清这次实验做了什么

```
/livox/points (lidar_frame) ──TODO 1──┐
                                      ├──TODO 3──> /r1/fusion/points ──> Nav2 ──> 到达
/r1/depth/image + camera_info ──TODO 2─┘
```

链路的每一段都必须真的用到下一段：

* 环节 1 的坐标错了 → 环节 2 的点云是歪的；
* 环节 2 的融合漏了一块 → 车就在对应的关卡过不去；
* 车开不到终点，说明这条链上有环节错了，不需要另外做"验证实验"。

**判据只有一条：发一个目标点，车开到终点，0 碰撞。**

## 为什么不是"三种方案对比"

单路在多数场景也能到达，成功率必然饱和成 3/3 : 3/3 : 3/3；要把差异逼出来就得把场景做成"单路必撞"，而那种场景往往两路都撞。成本高、结论弱、还多占 2.5 学时。

所以本实验**不做 A/B/C 对照**，互补性由第 5 章理论课承担，本实验只负责把链路建起来跑通。

`mode` 有四个取值，前两个给学生用，后两个用于负向验证（教师必做、学生选做加分，见 `docs/PLATFORM_VALIDATION.md`）：

| `mode` | 含义 |
| --- | --- |
| `student`（默认） | 调用 `experiment9_student/fusion.py` 里的三个 TODO |
| `solution` | 调用 `experiment9_core/reference.py` 的参考实现（演示、对照、学生卡住时切换） |
| `lidar` / `depth` | 参考实现但**故意丢掉另一路**：车必须因此被拦住，否则说明关卡没放在必经路径上 |

## 五道关卡：全是"只有感知才能过"的

地形沿用实验 7 的 14 m × 12 m 房间（`STATIC_BOXES`，已烘焙进静态地图），另加五个**只存在于实时场景、不在静态地图里**的目标。它们是否真的在必经路径上，由 `tools/check_layout.py` 用栅格 A* 验证，不靠手感。

| 关卡 | 尺寸/高度 | 只有雷达会怎样 | 只有相机会怎样 |
| --- | --- | --- | --- |
| K1 低矮托盘 | 1.2×0.8×**0.35 m** | 雷达装 1.3 m 高、下视 7.2°，**7.52 m 内看不见 0.35 m** → 直接撞 | 相机俯视可见，能过 |
| K2 细立柱 | Ø**0.10 m**，2 m 高 | 稳定回波 | 5 m 外不足 2 像素，时有时无 |
| K3 薄板（玻璃替代物） | **0.04 m** 厚，1.2 m 宽 | 只有掠射线，点极稀 | 一整片深度面，两路形态不同 |
| K4 悬空横梁 | 1.6 m 高，跨 1.45 m | 能测到，**高度过滤写错就把车拦死** | 在相机光轴上方，看不见 |
| K5 侧向货架 | 0.6×0.6×1.5 m | 远处就报到（360°） | 只在转到附近才看见 |

布局验证结果（`python3 tools/check_layout.py`）：

```
纯静态地图路径: 9.75 m   含关卡路径: 13.95 m
K1 阻断型  0.050m 在必经路径上
K2 阻断型  0.000m 在必经路径上
K3 阻断型  0.000m 在必经路径上
K4 上方    1.185m 车从下方通过(水平 0.27 m)，检验高度过滤
K5 阻断型  0.000m 在必经路径上
```

**静态地图里不能有关卡。** 地图里只要画进去一个 K，学生不写一行融合代码也能跑通，整个实验失效。这条由 `tests/test_algorithms.py::test_map_is_free_at_every_checkpoint` 每次跑测试时守住。

## 学生要写的三个函数（约 60 行）

`src/experiment9_fusion/experiment9_student/fusion.py`

| 函数 | 做什么 | 典型错误的现象 |
| --- | --- | --- |
| `transform_cloud` | `lidar_frame` → `base_link`（R=I，t=(0,0,1.3)） | 符号写反 → 整片点云低 2.6 m，车从托盘上碾过去 |
| `depth_to_points` | 用内参反投影，再用外参转到 `base_link` | 少乘旋转 → 点云绕光轴转 90°；u/v 写反 → 左右镜像 |
| `merge_clouds` | 高度过滤 + **两路分别**体素降采样 + 合并 | `z_min` 低 → 地面进代价地图，起步就卡死；`z_max` ≥1.6 → K4 横梁变落地墙 |

两路必须**分别**降采样：雷达 10 m 处点距约 5 cm，深度 1 m 处约 6 mm。统一用 0.10 m 体素等于按密度惩罚近处，恰好把 K1 最需要的那部分压掉。

## 部署

初始环境与实验 5—8 一致：Ubuntu 22.04 + ROS 2 Humble。RViz2 随 ROS 2 桌面版已装好，点云、栅格地图、机器人模型的显示插件都在其中，**不需要另外安装**；Gazebo Fortress 与 Nav2 不在初始环境内，由 `tools/install_dependencies.sh` 一并装齐。下列命令以 root、工作区 `/root/exp9` 为例。

**方式一：教学包（推荐，含实验 5 / 实验 7 源码，不联网）**

```bash
apt install -y wget unzip
mkdir -p /root/exp9 && cd /root/exp9
wget -O experiment9_teaching_20261112_v2.zip "http://study.zhonghui.vip:10188/api/web/resource/embodied-1/EnvironmentalPerception/10/expPackages/experiment9_teaching_20261112_v2.zip"
unzip -o experiment9_teaching_20261112_v2.zip -d /root/exp9
bash tools/install_dependencies.sh
bash tools/prepare_workspace.sh      # 包内已带依赖，只校验文件哈希与提交号
bash tools/build.sh
```

若 `/root/exp9` 里已有做过实验的旧工程，先从第 2 行之前删掉整个目录（`rm -rf /root/exp9`）再重建：`-o` 只保证同名文件被覆盖，不会清掉上一版留下的 `build/`。

教学包把 `src/experiment5`、`src/experiment7` 按固定提交一起打进来，`prepare_workspace.sh` 检测到 `BUNDLED_DEPENDENCIES.json` 就只做校验、**不访问网络**。

**方式二：从代码仓库克隆（需要网络）**

```bash
git clone https://github.com/huokumo/experiment-last.git /root/exp9
cd /root/exp9
bash tools/install_dependencies.sh
bash tools/prepare_workspace.sh      # 克隆实验5（R1 模型）与实验7（Gazebo 插件），版本固定
bash tools/build.sh
```

没有随包依赖时，`prepare_workspace.sh` 会把实验 5 / 实验 7 克隆到 `src/` 下并校验到固定提交。Gazebo 的雷达与运动插件来自实验 7，**不要**再单独装同名包。

## 运行

```bash
source /opt/ros/humble/setup.bash
source /root/exp9/install/setup.bash
export PYTHONNOUSERSITE=1

# 1) 只看融合（还没有导航）
ros2 launch experiment9_fusion experiment9.launch.py mode:=solution

# 2) 完整闭环：场景 + 融合 + 地图 + Nav2
ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution

# 3) 另一个终端：自检
ros2 run experiment9_fusion check_ready.py

# 4) 发目标点（命令行，不用 RViz 点）
ros2 run experiment9_fusion send_goal.sh          # 或 scripts/send_goal.sh

# 5) 边跑边录，结束时自动出指标
ros2 run experiment9_fusion record_run.py --out runs/run1.csv
```

无图形界面：`gui:=false rviz:=false`。

### 用命令行看结果

| 想知道的 | 命令 |
| --- | --- |
| 数据在不在、频率对不对 | `check_ready.py`（4 组：频率 / TF 链 / 地图与代价地图 / 动作服务） |
| 障碍进没进代价地图 | `costmap_dump.py`（字符画，`@` 就是融合新标出来的） |
| 哪一路贡献了多少点 | `cloud_check.py`（逐关卡点数表） |
| 两片点云对齐没有 | `cloud_check.py --wall`（蓝墙 x=2.9 的距离偏差，两路应 < 5 cm） |
| 开得怎么样 | `send_goal.sh` 的 `distance_remaining` / `number_of_recoveries` / `status` |
| 硬指标 | `record_run.py` 的 CSV 摘要 |

RViz 照常可用（看点云、代价地图、路径），只是**发目标点用命令行**：手点每次落点不同、无法复现、无法存档，而命令本身就是证据。

## 验收（两条）

1. **端到端导航**：`send_goal.sh` 返回 `status: SUCCEEDED`，`record_run.py` 摘要里 `碰撞帧 = 0`、`终点距离 < 0.3 m`、`最小安全距离 > 0`。附 `runs/*.csv`。
2. **融合链路贯通**：`/r1/fusion/points` 确实由 `/livox/points` + `/r1/depth/image` 算出。证据是 `cloud_check.py` 的逐关卡点数表——**该为 0 的必须为 0**（K1 的雷达列、K4 的深度列），该有数的必须有数（K1/K3 的深度列、K2/K4 的雷达列）。直接转发某一路、或读真值造点云，这张表立刻露馅。

## 已知限制（写进报告，不要当成 bug）

* **深度相机只有 2 Hz、160×120、远裁 10 m**。规划速度 0.22 m/s 下帧间位移 0.11 m，绕 K1 托盘（0.8 m 长）够用，但车在障碍前会"确认一下再走"。想更顺可以改模型里的 `update_rate`，但那就不是实验 5 的相机了。
* **"玻璃"用 0.04 m 薄板代替**。Fortress 的深度相机渲染和教学用 CPU 光线投射都不会做透明穿透，真玻璃在这个仿真里会"两路都正常"，反而没有教学价值。薄板保留了"两路形态差异大、正好检验对齐"的作用。
* **`/odom` 是仿真真值**（运动系统直接报告真实位姿），不是轮式里程计积分。这样差异只来自障碍覆盖，不来自定位漂移。换成 FAST-LIO 只需把 `map→odom` 那条静态变换替换掉，代码一行不改。
* **K5 不是"视场外"关卡**。原设计想让车看不见它，实测不成立：车会转向，深度相机在靠近时也能看到 299 个点。它的真实作用是抓"只保留车头前方点云"这类过滤错误——货架在路径侧方 1.2 m，`x > 0` 之类的过滤会把它整块丢掉，车随即被拦。

## 坐标与接口约定

| 项 | 值 |
| --- | --- |
| 世界/地图系 | `map`（原点在地图左下，覆盖 x∈[-7,7]、y∈[-6,6]，0.05 m/格） |
| TF 链 | `map →(静态恒等)→ odom →(50 Hz)→ base_link →(静态)→ lidar_frame` |
| 雷达 | 仿真 Mid-360，`lidar_frame`，装在 `base_link` 上方 **1.3 m**，无旋转，5 Hz，逐点带 `offset_time` |
| 深度相机 | 160×120、HFOV 60°、**2 Hz**、near 0.1 / far 10、`depth_camera_optical_frame`，高 **1.154 m**、下俯 **19.48°** |
| 融合输出 | `/r1/fusion/points`，`base_link`，PointCloud2，z∈[0.10, 1.20] |
| 控制 | Nav2 → `/r1/cmd_vel`（`geometry_msgs/Twist`）→ 仿真平面运动系统，限速 0.25 m/s、0.5 rad/s |
| 碰撞判据 | `/r1/collision_blocked`（2D 圆 vs 矩形），由仿真运动系统发布 |

外参由 `tools/derive_extrinsics.py` 从 R1 的 SDF 里算出来，不要手抄：

```bash
python3 tools/derive_extrinsics.py --model src/experiment5/models/unitree_r1/model_depth.sdf
```

## 目录

```
experiment9/
├── src/experiment9_fusion/          本实验包
│   ├── experiment9_core/            框架：消息编解码、参考实现、场景几何
│   ├── experiment9_student/         ★ 学生唯一要改的 fusion.py
│   ├── launch/                      experiment9.launch.py / nav_demo.launch.py
│   ├── config/                      fusion.yaml / bridge9.yaml / nav2_params.yaml / map/
│   ├── scripts/                     fusion_node / odom_tf / check_ready / costmap_dump / cloud_check / record_run / send_goal
│   └── tests/                       20 项纯 NumPy 单元测试
├── tools/                           install_dependencies / prepare_workspace / build / preflight / derive_extrinsics / make_map / check_layout
└── docs/                            STUDENT_GUIDE / INTERFACES / PLATFORM_VALIDATION
```

## 开课前自检

```bash
python3 tools/preflight.py \
  --model src/experiment5/models/unitree_r1/model_depth.sdf \
  --pattern src/experiment7/vendor/livox/mid360.csv
```

14 项：语法、XML/YAML、桥接话题、代价地图订阅、地图尺寸与原点、外参正交性与 SDF 一致性、关卡布局、离线光线投射的观测结构、20 项算法测试、学生文件待完成状态。

## 许可

本仓库 BSD-3-Clause。实验 5 的 R1 模型与实验 7 的 Gazebo 插件按各自仓库的许可使用，详见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

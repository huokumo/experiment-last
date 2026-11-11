# 实验10 学生指导手册：多传感器融合与自主导航

> 教学包 `experiment9_teaching_20261111.zip` 解压到 `/root/exp9` 即可（包内已带实验五、实验七源码，不联网也能编译）；也可用仓库 `experiment-last`（`git clone https://github.com/huokumo/experiment-last.git /root/exp9`，需要网络）。文中"实验 N"是仓库编号（实验5 = `experiment5_depth`，实验7 = `experiment7_lidar`，实验8 = `experiment8_slam`），换算成手册编号各 +1。

> 4 课时，2 人一组。产出：一份能开车的工程 + 一张逐关卡点数表 + 一段 CSV。

## 1. 你要做成一件事

在 Gazebo 里看到一间有障碍物的房间，**发一个目标点，机器人自己规划路径、避开障碍、开到终点**。

不是"分别验证传感器"，也不是"比较三种方案"。链路跑通即通过，跑不通才逐段排查。

```
四路数据 → 时空对齐 → 融合 → 障碍点云 → Nav2 → 目标点
```

## 2. 你已经有的

| 来自 | 内容 |
| --- | --- |
| 实验 5 | R1 模型与深度相机（160×120、2 Hz、60° HFOV） |
| 实验 7 | 同一间 14×12 m 房间、仿真 Mid-360（5 Hz、逐点时间）、平面运动系统 |
| 实验 8 | FAST-LIO 的位姿输出。**本实验不用它**，用仿真里程计，理由见第 8 节 |

本实验只新增一件事：**把两路点云对齐、合并、喂给 Nav2**。

## 3. 环境

```bash
cd /root/exp9
bash tools/install_dependencies.sh
bash tools/prepare_workspace.sh
bash tools/build.sh

source /opt/ros/humble/setup.bash
source /root/exp9/install/setup.bash
export PYTHONNOUSERSITE=1
```

先跑参考实现，确认环境没问题：

```bash
ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution
ros2 run experiment9_fusion check_ready.py      # 另一个终端，应 4/4 通过
```

看到车开到终点，再回来写自己的代码。

## 4. 三个 TODO

文件：`src/experiment9_fusion/experiment9_student/fusion.py`

### TODO 1 `transform_cloud` —— 雷达点云搬到车身系

```
p_base = R @ p_lidar + t
```

本仿真里 `R = I`，`t = (0, 0, 1.3)`：雷达装在机身上方 1.3 m，没有旋转。

**这个 TODO 只有一行，但符号错了整片点云会低 2.6 m**。注意：`cloud_check.py --wall` 量的是水平（x）方向，z 方向错了它看不出来。符号反的表现是——RViz 里点云钻到地面以下、经过高度带过滤后合并点数骤降到几十个、车被拦在 K1 托盘前过不去。写完先看 `ros2 topic echo /r1/fusion/status --once` 里 `lidar_base` 与 `lidar_in` 是否接近。

### TODO 2 `depth_to_points` —— 深度图反投影

先在光学系里反投影（x 右、y 下、z 前）：

```
z = depth[v, u]
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```

再转到车身系：`p_base = R @ p_cam + t`。

`fx = fy = 138.56`、`cx = 80`、`cy = 60`，运行时从 `/r1/depth/camera_info` 读，不要写死。

两个坑：

* **`u` 是列号，`v` 是行号**。写反了左右镜像，肉眼看点云"好像也对"，但两片叠不到一起。
* **本仿真的相机不是 90° 整数倍安装**：高 1.154 m、下俯 19.48°，所以旋转矩阵必须用配置里给的那个。

### TODO 3 `merge_clouds` —— 过滤、降采样、合并

顺序不能换：**先按高度过滤 → 每路各自降采样 → 再合并**。

* `z_min = 0.10`：低于 0.10 m 的是地面。设成 0 或负数，地面进代价地图，车起步就被自己脚下的"墙"困死。
* `z_max = 1.20`：车高约 0.5 m，K4 横梁在 1.6 m。设到 1.6 以上，横梁变成落地墙，车在能过的通道里停死。
* 两路体素不同：雷达 0.10 m、深度 0.04 m。合并后统一降采样会把近处细节压掉，正好压掉 K1 托盘。

## 5. 五个环节（180 分钟）

**先记住这条**：三个 TODO 是**分开生效**的，每写完一个就能立刻看到东西，不必等三个都写完。

| 你写到哪 | 该看到什么 | 验证命令 |
| --- | --- | --- |
| 什么都没写 | 终端报 `TODO 1 transform_cloud not implemented yet`，没有任何点云 | `check_ready.py` 会提示"先确认 TODO 1" |
| 只写完 TODO 1 | `/r1/fusion/points_lidar` 每秒约 2 帧；RViz（Fixed Frame = `map`）里雷达点云贴在墙面和柱子上 | `ros2 topic hz /r1/fusion/points_lidar`、`cloud_check.py --wall`（雷达那行有偏差数） |
| 写完 TODO 1+2 | 多出 `/r1/fusion/points_depth`；两片点云在 RViz 里叠在同一面墙上 | `cloud_check.py --wall` 两路偏差都 < 5 cm |
| 写完 TODO 3 | `/r1/fusion/points` 出现，`check_ready.py` 第 1 组全绿，代价地图里 K1/K5 出现 `@` | `costmap_dump.py`、`cloud_check.py` |

> 发布节奏：`/r1/fusion/points_lidar` 与 `..._depth` 是排障用的单路点云，`/r1/fusion/points` 才是喂给 Nav2 的融合结果。前者有了、后者没有，说明卡在 TODO 3，不用去查 TF 和代价地图。
> RViz 里勾选左侧 `Displays` 已有但默认关闭的"雷达（base_link）"/"深度（base_link）"两项，Color Transformer 用 `AxisColor`、Size 调 0.02，就能看清点落在哪。

| 环节 | 时间 | 干什么 | 通过判据 |
| --- | --- | --- | --- |
| 0 起场景 | 25 min | `nav_demo.launch.py mode:=solution` + `check_ready.py` | 4/4 通过 |
| 1 对坐标（TODO 1、2） | 55 min | 先只做雷达（看 `points_lidar` 贴不贴墙），再加深部（看 `points_depth` 能否与雷达重合） | 两路墙距偏差 < 5 cm |
| 2 合并（TODO 3） | 40 min | 发布 `/r1/fusion/points`，看 `costmap_dump` 里 `@` 有没有出现在 K1、K5 处 | `cloud_check` 表里该有数的关卡都有数 |
| 3 发目标点 | 45 min | `send_goal.sh`，盯 `distance_remaining` 是否单调下降 | `status: SUCCEEDED`、碰撞帧 0 |
| 4 收尾 | 15 min | 填表、写三行结论 | 交 `runs/*.csv` + 点数表 |

**改完代码怎么生效**：`tools/build.sh` 用了 `colcon build --symlink-install`，所以改 `fusion.py` **不用重新编译**——`Ctrl-C` 停掉融合节点、重新 `launch` 即可。如果你自己用不带 `--symlink-install` 的方式编过，就必须重新 `colcon build`，否则改的是源码、跑的是旧副本（症状：改了半天毫无变化）。

第 0 环节有个 5 分钟演示，务必做一次：

```bash
ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0 --z 0 \
    --frame-id map --child-frame-id odom
```

**点云没变，但代价地图里所有障碍整体平移 0.3 m**，车开始贴着墙走。看完立刻关掉。这一步把"定位错了 = 障碍投错位置"直接演示出来，比讲十遍 TF 有用。

## 6. 出问题了怎么查（按现象查，不要瞎改）

| 现象 | 最可能的原因 | 一条命令定位 |
| --- | --- | --- |
| 什么点云都没有 | TODO 1 还没写完 | 终端会报 `TODO 1 transform_cloud not implemented yet` |
| 有 `points_lidar`、没有 `points_depth` | TODO 2 还没写完 | 终端报 `TODO 2 depth_to_points not implemented yet` |
| 两路单点云都有、没有 `points` | TODO 3 还没写完 | `ros2 topic echo /r1/fusion/status`，看 `stage` 字段 |
| `/r1/fusion/points` 有数，代价地图全空 | TF 链断（缺 `odom→base_link`） | `ros2 run tf2_tools view_frames` |
| 车被地面困死、原地不动 | `z_min` 太低，地面进了代价地图 | `cloud_check.py` 看总点数是否异常大 |
| K1 托盘没进地图，车撞上去 | `z_min` 太高，把 0.35 m 的托盘切掉了 | 临时把 `z_min` 调到 0.0 再看 |
| 车在 K4 前停死（明明能过） | `z_max` ≥ 1.6，横梁成了落地墙 | 看 `costmap_dump` 那一条横线 |
| 车全程贴墙、路径不居中 | 外参差几厘米，点云整体偏 | `cloud_check.py --wall` |
| 深度那一路墙距偏 90° | `depth_to_points` 少乘旋转或 u/v 写反 | 同上 |
| 车在 RViz 里瞬移、原地画圈 | 里程计侧：`use_sim_time` 没开，或第 0 环节的 0.3 m 演示没关 | `ros2 topic hz /odom` 应 50 Hz |
| 发了目标点没反应 | `orientation` 全 0（零四元数），行为树直接拒绝 | 用 `send_goal.sh`，它算四元数 |

## 7. 验收与报告

**必交**

1. `runs/*.csv` + `record_run.py` 结束时打印的摘要（里程、碰撞帧、最小安全距离、终点距离）；
2. `cloud_check.py` 的逐关卡点数表（文本，可直接贴）；
3. 一页结论：三个函数各解决什么、哪一关最险、如果重做会改哪个参数。

**加分（选做）**

* 把 `update_rate` 改成 5 Hz 再跑一遍，比较 K1 前的"犹豫"次数；
* 用 `mode:=lidar` 和 `mode:=depth` 各跑一次，指出车分别死在哪个关卡，并对应到 `cloud_check` 表里为 0 的那一格。

**报告四个板块**：任务与部署 → 坐标统一（外参/反投影）→ 合并与代价地图 → 导航结果与结论。

## 8. 三个必须知道的前提（写进报告，不算 bug）

1. **深度相机 2 Hz**，0.22 m/s 下帧间位移 0.11 m。K1 托盘长 0.8 m，够用，但车会在障碍前"停一下再走"，这是数据率导致的，不是算法慢。
2. **`/odom` 是仿真真值**，不是轮式积分。本实验不考定位：三路数据共用同一个位姿源，差异只来自障碍覆盖。换成 FAST-LIO 只需替换 `map→odom` 那条静态变换，融合代码一行不改。
3. **"玻璃"是 0.04 m 薄板**。真透明材质在 Fortress 里深度和激光都"正常返回"，看不出问题；薄板保留了两路形态差异（稀疏掠射线 vs 一整片深度面），正好用来检验对齐。

## 9. 话题速查

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/livox/points` | PointCloud2 | 5 Hz | `lidar_frame`，逐点 `offset_time` |
| `/r1/depth/image` | Image | 2 Hz | 32FC1，米 |
| `/r1/depth/camera_info` | CameraInfo | 2 Hz | fx/fy/cx/cy |
| `/odom` | Odometry | 50 Hz | `odom → base_link` |
| `/r1/cmd_vel` | Twist | — | Nav2 输出，限速 0.25 m/s |
| `/r1/collision_blocked` | Bool | 事件 | 2D 保护判据拦截时为 true |
| `/r1/fusion/points` | PointCloud2 | ≈2 Hz | **你的输出**，`base_link`，Nav2 订阅的就是它 |
| `/r1/fusion/points_lidar` | PointCloud2 | ≈2 Hz | 只有雷达那一路（TODO 1 写完就有） |
| `/r1/fusion/points_depth` | PointCloud2 | ≈2 Hz | 只有深度那一路（TODO 2 写完就有） |
| `/r1/fusion/status` | String(JSON) | ≈2 Hz | `stage`（走到哪一步）、三路点数、配对 Δt、当前参数 |
| `/map` | OccupancyGrid | 一次 | 预置地图，只有墙和地面 |
| `/global_costmap/costmap` | OccupancyGrid | 1 Hz | 代价地图 |
| `/plan` | Path | 每次规划 | 全局路径 |

完整接口约定见 [INTERFACES.md](INTERFACES.md)。

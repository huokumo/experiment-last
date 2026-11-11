# experiment9_fusion

实验10（仓库 `experiment9`）的 ROS 2 包。学生只改 `experiment9_student/fusion.py` 里的三个函数。

```bash
ros2 launch experiment9_fusion experiment9.launch.py mode:=solution   # 只看融合
ros2 launch experiment9_fusion nav_demo.launch.py mode:=solution      # 完整闭环
ros2 run experiment9_fusion check_ready.py                            # 自检
```

| 目录 | 用途 |
| --- | --- |
| `experiment9_core/` | 框架：PointCloud2/Image 编解码（`cloud_io.py`）、参考实现（`reference.py`）、场景几何（`scene9.py`） |
| `experiment9_student/` | ★ `fusion.py`，三个 TODO |
| `launch/` | `experiment9.launch.py`（场景+传感器+融合）、`nav_demo.launch.py`（+地图+Nav2） |
| `config/` | `fusion.yaml`、`bridge9.yaml`、`nav2_params.yaml`、`map/`、`experiment9.rviz` |
| `scripts/` | `fusion_node.py`、`odom_tf.py`、`check_ready.py`、`costmap_dump.py`、`cloud_check.py`、`record_run.py`、`send_goal.sh` |
| `tests/` | 20 项纯 NumPy 测试，不需要仿真 |

`experiment9_core/reference.py` 是参考实现，`mode:=solution` 时加载它。它和 `experiment9_student/fusion.py` 的函数签名完全一致，所以两条路径可以互相对照。

依赖说明：Gazebo 插件（`libmid360_teaching_system.so`、`libr1_planar_motion_system.so`）和 R1 模型分别在 `experiment7_lidar`、`experiment5_depth` 两个包里，**本包不重新编译它们**，只通过 launch 把它们的 `share`/`lib` 路径拼进环境变量。

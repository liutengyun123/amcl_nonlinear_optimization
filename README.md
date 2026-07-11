# AMCL 非线性优化定位

这是一个 ROS 1（catkin）二维定位功能包，在 AMCL、NDT 和 CSM 定位能力的基础上，加入了基于扫描匹配的非线性优化定位。

`csm_localizer_node` 使用 Levenberg–Marquardt（LM）方法迭代优化机器人二维位姿 `(x, y, yaw)`：将激光扫描投影到地图中，与地图边缘特征建立对应关系，并最小化点到边缘法线的鲁棒残差。优化过程包含 Huber 损失、LM 阻尼自适应调整、位姿步长限制和收敛判定，以提高复杂环境下的匹配稳定性。

## 功能

- **非线性 CSM 优化定位**：LM 非线性最小二乘优化二维位姿。
- **鲁棒匹配**：支持 Huber 损失、距离加权和对应点有效性筛选。
- **稳定性保护**：支持 LM 阻尼调节、步长限制、退化检测和弱方向融合。
- **多种定位节点**：提供 `csm_localizer_node`、`ndt_localizer_node` 与 `amcl_localizer_node`。
- **重定位与动态点处理**：支持启动/丢失重定位及动态点过滤。

## 依赖

- ROS 1 / catkin（推荐 ROS Noetic）
- PCL
- OpenCV
- yaml-cpp
- `ndt_omp`
- `localization_msgs`

## 构建

将本仓库放入 catkin 工作空间的 `src` 目录后执行：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行非线性优化定位

使用 CSM 非线性优化定位节点：

```bash
roslaunch localization_ndt csm_localization.launch
```

启动文件中的 `map_yaml_path`、激光、里程计、IMU 和坐标系话题均可按实际机器人配置覆盖。例如：

```bash
roslaunch localization_ndt csm_localization.launch \
  map_yaml_path:=/path/to/map.yaml \
  scan_topic:=/scan \
  odom_topic:=/odom \
  imu_topic:=/imu \
  base_frame_id:=base_footprint
```

## 关键非线性优化参数

CSM 节点提供以下私有参数用于调整 LM 优化器：

- `optimizer_max_iterations`：每帧最大优化迭代次数。
- `optimizer_corr_max_dist_m`：扫描点与地图特征的最大对应距离。
- `optimizer_huber_delta_m`：Huber 鲁棒损失阈值。
- `optimizer_lm_lambda_init`：LM 初始阻尼系数。
- `optimizer_lm_lambda_up` / `optimizer_lm_lambda_down`：拒绝/接受步长后的阻尼调整倍率。
- `optimizer_lm_max_inner_trials`：单次迭代允许的 LM 内部重试次数。

完整的默认配置与参数说明可参考 [launch/csm_localization.launch](launch/csm_localization.launch) 及 `src/csm_localizer_node.cpp`。

## 许可证

本项目包含不同来源的代码，请参阅 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 了解第三方许可信息。

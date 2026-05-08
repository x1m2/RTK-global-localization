# RTK Global Localization 面试讲解文档

这份文档的目标不是背概念，而是帮你把项目讲成一个真实工程问题：我不是简单调用 `robot_localization`，而是围绕 RTK、FAST-LIO、IMU 航向、Nav2 地图坐标系做了一套可部署、可调试、可离线验证的全局定位链路。

## 一句话介绍

这个项目解决的是：FAST-LIO 局部里程计每次启动都会从新的 `odom` 原点开始，RTK 有绝对位置但频率低、跳变明显，Nav2 又需要稳定的 `map -> odom -> body` TF。我的工作是把 RTK 的 WGS84 绝对坐标、双天线/IMU 航向和 FAST-LIO 的高频局部运动融合起来，让车在真实 2D 地图上开机后自动落到正确位置，不需要手动在 RViz 点 `2D Pose Estimate`。

可以这样开场：

> 这个项目不是只把几个 ROS 包接起来。我主要做的是全局定位工程链路：RTK 数据清洗和协方差处理、FAST-LIO 里程计重封装、WGS84 到地图坐标的固定原点绑定、双天线航向转 map-relative yaw、EKF 融合配置，以及离线 bag 和实车 launch 的验证。最终输出稳定的 `map -> odom`，供 Nav2 使用。

## 系统架构

核心 TF：

```text
map -> odom -> body
```

数据流：

```text
FAST-LIO /Odometry
  -> odometry_sanitizer.py
  -> /odometry/local

RTK /ap/navsat
  -> navsat_fix_sanitizer.py
  -> /rtk/fix
  -> navsat_transform_node
  -> /odometry/gps
  -> odometry_low_pass_filter.py
  -> /odometry/gps/smoothed

IMU /ap/imu/experimental/data
  -> imu_ned_to_enu.py
  -> /rtk/imu/enu
  -> map_relative_heading_imu.py
  -> /rtk/navsat_heading/map_relative

/odometry/local + /odometry/gps/smoothed + /rtk/navsat_heading/map_relative
  -> robot_localization EKF
  -> /odometry/global + map -> odom
```

面试时要强调：

- FAST-LIO 负责高频、连续、平滑的局部运动。
- RTK 负责低频、绝对、地理参考的位置约束。
- 双天线/IMU 航向负责解决 yaw 不可观或收敛慢的问题。
- EKF 不是让 RTK 替代 FAST-LIO，而是让 RTK 把 FAST-LIO 的局部 `odom` 重新放回固定 `map`。

## 我的工作量怎么讲

可以按这 6 类讲，听起来会比“调用了包”扎实很多。

1. 传感器数据适配

我写了 `navsat_fix_sanitizer.py` 清洗原始 RTK：过滤无效 fix、过滤 0 经纬度、统一 `frame_id`，并修补或限制 `NavSatFix` 的协方差。因为很多 RTK 驱动给的 covariance 可能是 0、未知或不稳定，如果直接喂给 EKF，滤波器会过度相信错误测量。

2. FAST-LIO 里程计工程化处理

我写了 `odometry_sanitizer.py`，把 FAST-LIO 的 `/Odometry` 统一成融合链路需要的 `/odometry/local`，补齐 pose/twist covariance，并且支持从 pose 差分估计 twist，还能按车辆运动学阈值过滤明显跳变。

3. WGS84 到 map 的坐标绑定

我设计了 `map_origin.yaml`，记录地图原点对应的经纬度、高度和 `yaw_enu`。启动时把这个原点作为 `navsat_transform_node` 的固定 datum，这样同一张 2D 地图下，不管 bag 从哪里开始播放，或者实车在哪里开机，RTK 都能落回同一个 `map` 坐标系。

4. 地理坐标转换验证工具

我写了 `map_point_to_navsat.py` 和 `navsat_point_to_map.py`，实现 WGS84、ECEF、ENU、map 之间的正反转换，用于验证“地图上某个点对应的经纬度”和“某个经纬度落在地图哪里”。这不是只依赖黑盒，而是可以独立检查坐标转换是否正确。

5. 航向处理

我写了 `imu_ned_to_enu.py` 把设备/NED 风格 IMU 转成 ROS ENU 风格；再用 `map_relative_heading_imu.py` 把绝对 ENU yaw 转成相对地图的 yaw：

```text
yaw_map = yaw_enu_current - yaw_enu_origin
```

这样 EKF 融合的是 map 下的航向约束，不是随便把一个 IMU yaw 塞进去。

6. 融合调参和验证入口

我整理了实车入口 `rtk_vehicle.launch.py` 和离线入口 `rtk_bag_test.launch.py`。实车里配置了 RTK 天线杆臂，比如 `gps_link` 相对 `body` 是后 5 cm、左 5 cm、上 3 cm；bag 测试里可以同时启动 FAST-LIO、地图服务、RTK pipeline 和 rosbag 播放。

## 深挖问题 1：时钟同步和频率差异

面试官可能问：

> RTK 1-10 Hz，FAST-LIO 可能 100 Hz 以上，你怎么处理时间戳对齐？

推荐回答：

> 我没有把 RTK 和 FAST-LIO 强行重采样到同一个频率，而是让 EKF 按消息时间戳做异步融合。FAST-LIO 作为高频连续运动源，RTK 作为低频绝对位置更新。配置上 global EKF 运行在 30 Hz，`sensor_timeout` 是 0.3 s，`predict_to_current_time` 打开，RTK、FAST-LIO、IMU 都有独立 queue。这样高频 FAST-LIO 持续预测，低频 RTK 到来时修正 `map -> odom`。

继续补充工程细节：

- 实车运行时 `restamp_to_now=false`，保留传感器原始时间戳，避免把延迟伪装成当前测量。
- 离线 bag 验证时可以 `restamp_to_now=true`，主要是为了 bag 播放、TF 缓存和仿真时钟更容易对齐。
- `navsat_transform_node` 订阅 RTK、IMU 和 filtered odometry，它会根据时间戳和 TF 查询把 WGS84 转成 odometry。
- 我对 RTK 输出做低通平滑，而不是让每个低频 RTK sample 都硬拉 `map -> odom`。

可以主动说局限性：

> 当前项目属于松耦合融合，没有做硬件级 PPS/PTP 同步，也没有写自定义 message_filters 的精确同步器。我的处理重点是保持时间戳语义正确、让 EKF 异步融合、通过 covariance 和低通避免低频 RTK 对高频里程计造成跳变。如果速度更高或者传感器延迟更大，下一步会标定 RTK 延迟，配置 `transform_time_offset` 或输入时间偏置，必要时做硬件同步。

这段话很重要，因为它既说明你知道工程同步问题，也不会过度吹项目已经做了没做的事。

## 深挖问题 2：坐标系和地理转换

面试官可能问：

> RTK 是 WGS84，经纬度怎么转到 Nav2 的 map/odom？

推荐回答：

> 我用固定地图原点把 WGS84 和 2D 栅格地图绑定起来。`map_origin.yaml` 里保存地图原点的 latitude、longitude、altitude 和 `yaw_enu`。启动时 launch 文件读取这个文件，把 `[latitude, longitude, yaw_enu]` 作为 `navsat_transform_node` 的 datum。RTK 的 `/rtk/fix` 经过 navsat_transform 后变成 map/odom 语义下的 `/odometry/gps`。然后 EKF 用 GPS 的 X/Y 约束和 FAST-LIO 的局部运动生成 `map -> odom`。

可以画这个转换链：

```text
WGS84 latitude/longitude/altitude
  -> fixed datum from map_origin.yaml
  -> ENU/local Cartesian
  -> rotate by yaw_enu
  -> map frame
```

你项目里能展示的细节：

- `map_origin.yaml` 不是随便写的，它由 `map_origin_recorder.py` 记录 RTK fix 和航向生成。
- `map_relative_heading_imu.py` 明确把绝对 ENU yaw 转成相对地图 yaw。
- `map_point_to_navsat.py` 和 `navsat_point_to_map.py` 手写了 WGS84/ECEF/ENU/map 的正反转换，用于验证 navsat_transform 的结果。
- `rtk_vehicle.launch.py` 里配置了 `body -> gps_link` 静态杆臂，避免 RTK 天线位置和车体中心混用。

可以主动解释 TF：

```text
map: 2D 栅格地图坐标系
odom: FAST-LIO 每次启动产生的连续局部坐标系
body: 车体坐标系，当前约定为雷达中心
gps_link: 主 RTK 天线坐标系
```

关键句：

> 我最终不是直接发布 `map -> body`，而是让 EKF 发布 `map -> odom`，FAST-LIO 或局部里程计提供 `odom -> body`。这样 Nav2 看到的是标准的连续 TF 树，局部运动不会被 RTK 低频跳变直接破坏。

## 深挖问题 3：协方差、RTK 遮挡和 EKF 局限

面试官可能问：

> RTK 进厂房被遮挡时怎么办？协方差怎么处理？

推荐回答：

> 我对 RTK 做了两层处理。第一层在 `navsat_fix_sanitizer.py`，过滤无效 fix，修复 unknown/zero covariance，并设置最小和最大方差，避免 EKF 因为错误 covariance 过度信任 RTK。第二层在 GPS odom 上做低通和 covariance floor，实车入口里 `gps_smoothing_alpha=0.15`，`gps_smoothed_position_variance_floor=25.0`，意思是 RTK 只作为温和的全局修正，不让它每次跳变都强行拉动 `map -> odom`。

继续说明遮挡场景：

> 当 RTK 质量变差或丢失时，合理做法是让 RTK covariance 变大，或者直接丢弃无效 fix，让 FAST-LIO 接管短时间连续定位。EKF 在没有 RTK 时会继续依靠 FAST-LIO 预测，但长期会漂移。我的代码已经有 fix 状态过滤、协方差修补和 covariance floor；更进一步可以接入 RTK quality、fix type、卫星数或 HDOP，根据质量动态膨胀 covariance。

面试官可能继续问：

> 为什么不用 GTSAM 因子图，做 LiDAR+RTK+IMU 紧耦合？

推荐回答：

> 我当前选择 EKF 是因为项目目标是接入 Nav2 实车部署，要求实时、可解释、容易和 ROS2 TF 体系集成。`robot_localization` 能直接输出 `map -> odom`，对 Nav2 友好，调参和诊断成本低。这个项目的主要矛盾不是要做后端全局最优 SLAM，而是要让已有 FAST-LIO 局部里程计在固定地图里获得可靠全局锚定。

然后承认 EKF 局限：

> EKF 是单状态递推，不能像因子图那样在 RTK 恢复后回溯优化过去轨迹；对非高斯跳变、多时延、多传感器外参误差也不如因子图灵活。如果需求变成高精度建图或长时间遮挡后的全局轨迹优化，我会考虑 GTSAM/iSAM2，把 FAST-LIO 位姿增量、IMU 预积分、RTK 位置因子和外参作为因子统一优化。但对当前 Nav2 在线定位任务，EKF 的实时性和 TF 接口更合适。

## 面试时的 2 分钟讲法

可以直接这样说：

> 我这个项目做的是 RTK 辅助的全局定位。FAST-LIO 提供高频局部里程计，但它每次启动的 odom 都是局部坐标，不知道自己在 2D 地图里的绝对位置。RTK 提供绝对经纬度，但频率低、会跳，而且在遮挡场景下不稳定。所以我没有让 RTK 直接控制车体，而是把 RTK 作为低频绝对位置约束，把 FAST-LIO 作为高频连续运动约束，再加上双天线/IMU 航向约束，通过 EKF 输出 `map -> odom`，让 Nav2 使用标准 TF 树。

> 工程上我做了几件事：第一，清洗 RTK 和 FAST-LIO 消息，统一 frame、时间戳和 covariance；第二，用 `map_origin.yaml` 把 WGS84 经纬度和 2D map 坐标系绑定，处理 ENU 到 map 的 yaw；第三，写了 WGS84/ECEF/ENU/map 双向转换工具验证坐标正确性；第四，对 GPS odom 做低通和平滑协方差处理，让 RTK 作为温和的全局修正；第五，整理了实车和离线 bag 的 launch，能复现实验链路。

> 所以这个项目的关键不是“我用了 robot_localization”，而是我把不同频率、不同坐标系、不同可靠性的传感器数据，整理成 Nav2 可以稳定消费的全局定位输出。

## 常见追问和回答

Q：为什么 global EKF 里 FAST-LIO 要 differential？

A：FAST-LIO 的 pose 是相对它自己的 `odom` 原点，不是 map 下的绝对 pose。如果把它当绝对 map pose，启动原点变化会污染全局定位。用 differential 的思路是让它贡献局部位移增量和平滑运动，绝对位置由 RTK 约束。

Q：为什么只融合 GPS 的 X/Y，不融合 Z？

A：当前 Nav2 是 2D 导航，`two_d_mode=true`，地图也是 2D 栅格。RTK 高度和地面车导航关系不大，而且高度噪声可能更大，所以主要用 X/Y 和 yaw 来约束平面定位。

Q：为什么需要航向，只有 RTK 位置不行吗？

A：只有 RTK 位置可以约束平移，但 yaw 收敛依赖运动轨迹，对开机静止或低速场景不可靠。双天线/IMU 航向能直接约束 map 下 yaw，让车开机后朝向也对。

Q：RTK 跳变时会不会把车拉飞？

A：我没有把 RTK 直接发布成车体 TF，而是先经过 covariance、低通和平滑，再进 EKF。RTK 的作用是修正 `map -> odom`，FAST-LIO 仍然保持 `odom -> body` 的局部连续性。极端跳变还可以通过 fix 质量、速度门限、Mahalanobis rejection 或动态 covariance 膨胀继续增强。

Q：这个项目怎么验证？

A：我做了离线 bag 测试入口，可以同时启动 FAST-LIO、2D map server、RTK pipeline 和 bag 播放；同时有坐标正反转换工具，可以在 RViz 点 map 点转经纬度，也可以输入经纬度看它落到地图哪里。实车入口则配置真实 topic、天线杆臂和 yaw correction。

## 可以主动展示的文件

- `src/rtk_global_localization/README.md`：项目整体设计和数据流。
- `src/rtk_global_localization/launch/rtk_vehicle.launch.py`：实车入口、topic、天线杆臂、GPS 平滑参数。
- `src/rtk_global_localization/launch/rtk_bag_test.launch.py`：离线 bag 验证入口。
- `src/rtk_global_localization/config/ekf_map_origin.yaml`：global EKF 融合配置。
- `src/rtk_global_localization/scripts/navsat_fix_sanitizer.py`：RTK 清洗和协方差修补。
- `src/rtk_global_localization/scripts/odometry_sanitizer.py`：FAST-LIO 里程计清洗、协方差、twist 估计、异常过滤。
- `src/rtk_global_localization/scripts/odometry_low_pass_filter.py`：GPS odom 低通和 covariance floor。
- `src/rtk_global_localization/scripts/map_point_to_navsat.py`：map 到 WGS84 的转换验证。
- `src/rtk_global_localization/scripts/navsat_point_to_map.py`：WGS84 到 map 的转换验证。
- `src/rtk_global_localization/scripts/map_relative_heading_imu.py`：绝对 ENU yaw 到 map-relative yaw。

## 诚实边界

面试时不要说：

- 我做了紧耦合 LiDAR-RTK-IMU 后端优化。
- 我做了硬件级时间同步。
- 我完全解决了 RTK 遮挡。

可以说：

- 当前是面向 Nav2 在线定位的松耦合 EKF 方案。
- 我处理了时间戳语义、异步融合、低频 RTK 对高频里程计的修正方式。
- 我处理了 WGS84/map 坐标绑定和 TF 树输出。
- 我处理了 RTK covariance、无效 fix、GPS 平滑和遮挡时的短期鲁棒性。
- 如果需求升级到高精度全局优化，可以扩展到 GTSAM/iSAM2 因子图。

最后一句收束：

> 这个项目体现的工作量不在“用了哪个包”，而在我把真实传感器数据的不确定性、坐标系、时间戳、TF、协方差和 Nav2 接口都打通了，并且能用 bag 和实车 launch 复现验证。

# RTK Global Localization

这个包把 FAST-LIO 的局部里程计、RTK/GPS 的全局位置、IMU/双天线航向融合起来，
输出 Nav2 需要的全局定位结果：

```text
map -> odom -> body
```

目标效果是：小车开到已经建过 2D 地图的真实场景中，开机后不需要在 RViz 里手动点
`2D Pose Estimate`。只要 RTK、航向、FAST-LIO 都正常，小车会自动落到 `map` 坐标系
下的实际位置，RViz 中可以直接看到车在 2D 栅格地图上的位置。

## 坐标系约定

本工程当前使用：

```text
map   全局地图坐标系，对应 2D 栅格地图和 map_origin.yaml
odom  FAST-LIO 每次启动产生的局部连续坐标系
body  车体坐标系，当前约定为雷达中心
gps_link 主 RTK 天线坐标系
camera_init FAST-LIO/点云相关的初始化坐标系，不代表车体
```

ROS 车体坐标方向：

```text
x 正方向 = 车头前方
y 正方向 = 车体左方
z 正方向 = 向上
```

当前实车测量值：

```text
body = 雷达中心
主 RTK 天线 = 雷达中心后方 5 cm、左方 5 cm、上方 3 cm

body -> gps_link:
  x = -0.05
  y =  0.05
  z =  0.03
```

这个杆臂已经写进 `launch/rtk_vehicle.launch.py`。如果之后重新量了天线位置，需要同步修改
`gps_x/gps_y/gps_z`。

## 算法原理

FAST-LIO 的优点是局部运动连续、平滑，适合提供短时间的相对位移。但它的 `odom` 坐标系
每次启动都会从新的局部原点开始，不知道自己在整张地图里的绝对位置。

RTK/GPS 的优点是能给出地球坐标系下的绝对位置，但会有跳变、延迟和协方差不稳定。如果
直接用 RTK 驱动车体，轨迹会不够平滑。

IMU/双天线航向提供全局 yaw。只有 RTK 位置而没有全局航向时，小车可能落在正确位置附近，
但朝向不一定可靠。

所以当前流程是：

```text
FAST-LIO /Odometry
  -> 清洗、补协方差、统一 frame
  -> /odometry/local

RTK /ap/navsat
  -> 清洗、补协方差、统一 frame
  -> /rtk/fix
  -> navsat_transform_node
  -> /odometry/gps
  -> 低通平滑
  -> /odometry/gps/smoothed

IMU /ap/imu/experimental/data
  -> NED 转 ENU
  -> /rtk/imu/enu
  -> 减去地图原点 yaw_enu
  -> /rtk/navsat_heading/map_relative

/odometry/local
/odometry/gps/smoothed
/rtk/navsat_heading/map_relative
  -> global EKF
  -> /odometry/global
  -> map -> odom
```

最终 Nav2 使用 TF：

```text
map -> odom -> body
```

其中 `odom -> body` 来自 FAST-LIO 或局部里程计，`map -> odom` 来自 global EKF。RTK 的
作用不是替代 FAST-LIO，而是把 FAST-LIO 每次启动产生的局部 `odom` 坐标系重新放回固定
的 `map` 坐标系里。

## 地图原点

`config/map_origin.yaml` 把真实经纬度和 2D 地图的 `map` 坐标系绑定：

```yaml
frame_id: map
latitude: 29.478019714355
longitude: 106.634170532227
altitude: 269.259979
yaw_enu: 2.443460953000
```

含义：

- `latitude/longitude/altitude` 是地图原点对应的 WGS84 坐标。
- `yaw_enu` 是地图 `map` 的 x 轴相对 ENU 坐标系的 yaw。
- 换地图、重新建图、改变地图原点，都必须重新生成这个文件。

在当前纬度附近，粗略量级是：

```text
纬度 +0.000001 deg 约等于北向 +0.111 m
经度 +0.000001 deg 约等于东向 +0.097 m
```

因为 `map` 坐标系相对 ENU 旋转了 `yaw_enu`，所以经纬度变化不会只体现在 map 的单一
轴上，而是会同时影响 `map_x/map_y`。

## 文件作用

`launch/rtk_global_localization.launch.py`

核心启动文件。负责启动 RTK 清洗、IMU 转换、FAST-LIO odom 清洗、GPS 平滑、两个 EKF、
`navsat_transform_node` 和可选调试节点。它是通用底层入口。

`launch/rtk_vehicle.launch.py`

实车部署推荐入口。它包含当前车的参数：真实时间、主 RTK 天线杆臂、IMU yaw 修正、GPS
平滑和 global EKF 配置。上车时优先启动这个文件。

`launch/rtk_bag_test.launch.py`

离线 bag 验证入口。会启动 FAST-LIO、全局定位 pipeline、2D map server，并播放 bag。它
默认使用 `maps/menkou.yaml` 做 2D 地图验证。

`launch/simulated_start_pose.launch.py`

没有实车时的模拟开机点工具。输入一个 map 坐标，发布对应的 `/simulated_start/odom`、
`/simulated_start/point`、`/simulated_start/navsat` 和 `map -> sim_body` TF。

`launch/navsat_point_to_map.launch.py`

经纬度转 map 点的验证工具。订阅 `/manual_navsat/fix`，输出 `/manual_navsat/point`、
`/manual_navsat/odom` 和 `map -> manual_navsat` TF。它只用于坐标转换验证，不会自动喂给
global EKF。

`scripts/navsat_fix_sanitizer.py`

清洗原始 `NavSatFix`。负责统一 frame、按需要重打时间戳、修补协方差、过滤无效 RTK fix，
输出 `/rtk/fix`。

`scripts/imu_ned_to_enu.py`

把设备/NED 风格 IMU 转成 ROS ENU 风格 IMU，输出 `/rtk/imu/enu`。其中
`imu_yaw_correction` 用于修正转换后的固定 yaw 偏差。

`scripts/map_relative_heading_imu.py`

读取 `map_origin.yaml` 的 `yaw_enu`，把绝对 ENU 航向转换成相对地图的航向：

```text
yaw_map = yaw_enu_current - yaw_enu_origin
```

输出 `/rtk/navsat_heading/map_relative`，供 global EKF 约束小车在 map 下的 yaw。

`scripts/odometry_sanitizer.py`

清洗 FAST-LIO `/Odometry`，输出 `/odometry/local`。负责 frame、child frame、时间戳、
协方差，并可过滤明显不符合车辆运动学的跳变。

`scripts/odometry_low_pass_filter.py`

对 `/odometry/gps` 做低通平滑，输出 `/odometry/gps/smoothed`。这样 RTK 成为温和的全局
修正源，不会每个 GPS 更新都硬拉 `map -> odom`。

`scripts/map_origin_recorder.py`

建图或标定时记录当前 RTK 经纬度和航向，生成新的 `map_origin.yaml`。正常导航时不需要
运行。

`scripts/georeferenced_rtk_odometry.py`

固定地图原点下的 RTK odom 调试输出。只用于检查 map origin 和 RTK 坐标关系，主链路
默认不用它融合。

`scripts/map_point_to_navsat.py`

RViz `Publish Point` 点选 map 坐标后，输出对应经纬度 `/clicked_point/navsat`。用于验证
某个地图点对应的 WGS84 坐标。

`scripts/navsat_point_to_map.py`

输入一个 `NavSatFix`，输出对应 map 点和 TF。用于验证“经纬度变化后，地图位置是否按预期
变化”。

`scripts/simulated_start_pose.py`

输入一个模拟 map 位姿，输出 map 点、NavSatFix、Odometry 和 TF。用于没有实车时演示
“开机后车出现在某个地图位置”的效果。

`scripts/pose_heading_to_imu.py`

把位姿消息里的 yaw 转成 IMU 风格航向。当前主链路默认不用，保留给使用 `/ap/pose/filtered`
航向的方案。

`scripts/static_heading_imu.py`

发布一个固定 yaw 的 IMU 航向。用于没有真实航向输入时的调试，不适合作为实车长期方案。

`config/ekf_odom.yaml`

local EKF 配置。订阅 `/odometry/local`，输出 `/odometry/local_ekf`。它主要用于调试和
平滑显示，不发布 TF。

`config/ekf_map_origin.yaml`

global EKF 配置。融合 `/odometry/local`、`/odometry/gps/smoothed` 和
`/rtk/navsat_heading/map_relative`，输出 `/odometry/global`，并发布 `map -> odom`。

`config/ekf_map.yaml`

通用 global EKF 配置。保留给不使用固定 map origin 的基础实验；实车和当前 bag 验证优先
使用 `ekf_map_origin.yaml`。

`config/navsat_transform.yaml`

`robot_localization/navsat_transform_node` 参数。负责把 WGS84 经纬度转换成 map 下的
`/odometry/gps`。

`config/map_origin.yaml`

当前地图的经纬度锚点和地图朝向。它必须和当前 2D 地图一一对应。

## 实车部署步骤

### 1. 准备输入数据

实车至少需要：

```text
/ap/navsat                    RTK/GPS NavSatFix，主天线位置
/ap/imu/experimental/data      IMU 或双天线航向数据
/Odometry                      FAST-LIO 局部里程计
body -> gps_link               主 RTK 天线相对车体的 TF
odom -> body                   FAST-LIO 或局部里程计 TF
```

如果实际 topic 名不同，可以在 launch 时覆盖。

### 2. 确认 2D 地图和 map_origin.yaml 匹配

地图文件和 `map_origin.yaml` 必须来自同一次建图/标定。当前 bag 对应的 2D 地图是：

```text
maps/menkou.yaml
maps/menkou.pgm
```

如果换了地图，只改 `maps/*.yaml` 不够，必须重新记录对应的 `map_origin.yaml`。

### 3. 编译并 source

```bash
colcon build --packages-select rtk_global_localization
source install/setup.bash
```

### 4. 启动传感器和 FAST-LIO

先确保这些话题有数据：

```bash
ros2 topic echo /ap/navsat --once
ros2 topic echo /ap/imu/experimental/data --once
ros2 topic echo /Odometry --once
```

并确认 TF：

```bash
ros2 run tf2_ros tf2_echo odom body
```

### 5. 启动全局定位

推荐实车入口：

```bash
ros2 launch rtk_global_localization rtk_vehicle.launch.py
```

如果实车 topic 名不同：

```bash
ros2 launch rtk_global_localization rtk_vehicle.launch.py \
  navsat_topic:=/your/navsat \
  imu_ned_topic:=/your/imu \
  fastlio_odom_topic:=/your/fastlio/odom
```

当前 `rtk_vehicle.launch.py` 已经设置：

```text
use_sim_time=false
restamp_to_now=false
fix_frame_id=gps_link
publish_gps_static_tf=true
gps_x=-0.05
gps_y=0.05
gps_z=0.03
imu_yaw_correction=0.8776404167
gps_smoothing_alpha=0.15
gps_smoothed_position_variance_floor=25.0
```

其中 `imu_yaw_correction` 是当前 bag/车辆数据调出来的固定 yaw 修正。正式上车前需要确认：
如果车头在 RViz 中整体偏一个固定角度，优先重新校准这个参数。

### 6. 启动 Nav2

Nav2 参数要满足：

```text
global_frame: map
robot_base_frame: body
odom_topic: /odometry/local 或 /odometry/local_ekf
```

注意：

- 本包发布 `map -> odom`。
- AMCL 不能同时发布 `map -> odom`。
- 如果必须启动 AMCL，设置 AMCL `tf_broadcast:=false`。

### 7. RViz 设置

RViz 建议：

```text
Fixed Frame: map

Displays:
  Map: /map
  TF
  Odometry: /odometry/global
  Odometry: /odometry/local
  Odometry: /odometry/local_ekf
  Odometry: /odometry/gps/smoothed
```

预期现象：

- `/odometry/local` 和 `/odometry/local_ekf` 的 `frame_id` 是 `odom`。
- `/odometry/global` 的 `frame_id` 是 `map`。
- 在 RViz 的 `map` 视角下，`local/local_ekf` 会经过 `map -> odom` 变换后显示，所以它们
  往往和 `global` 很接近，可能被重叠盖住。
- 如果想单独看局部轨迹，可以临时把 RViz `Fixed Frame` 改成 `odom`。

### 8. 实车定位是否成功的检查

启动后检查：

```bash
ros2 lifecycle get /ekf_filter_node_odom
ros2 lifecycle get /ekf_filter_node_map
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/gps/smoothed --once
ros2 topic echo /odometry/global --once
ros2 run tf2_ros tf2_echo map odom
```

成功时应看到：

- 两个 EKF 都是 `active`。
- `/odometry/local` 连续输出，`frame_id=odom`。
- `/odometry/gps/smoothed` 连续输出，`frame_id=map`。
- `/odometry/global` 连续输出，`frame_id=map`。
- `tf2_echo map odom` 能持续输出动态变换。

## Bag 验证步骤

播放完整 bag：

```bash
ros2 launch rtk_global_localization rtk_bag_test.launch.py
```

从 70 秒开始，模拟“车在场景中途开机”：

```bash
ros2 launch rtk_global_localization rtk_bag_test.launch.py bag_start_offset:=70.0
```

这个 launch 默认会启动：

```text
FAST-LIO
2D map server: maps/menkou.yaml
RTK/IMU/odom 清洗
navsat_transform_node
local EKF
global EKF
map_point_to_navsat.py
rosbag play --clock
```

RViz 中加载 `/map`，Fixed Frame 设为 `map`，再看这些话题：

```text
/odometry/global
/odometry/local
/odometry/local_ekf
/odometry/gps/smoothed
```

### 验证 GPS 是否真的融进 global EKF

正常融合 GPS：

```bash
ros2 launch rtk_global_localization rtk_bag_test.launch.py bag_start_offset:=70.0
```

GPS A/B 对照，不让 global EKF 订阅 GPS：

```bash
ros2 launch rtk_global_localization rtk_bag_test.launch.py \
  bag_start_offset:=70.0 \
  gps_odom_topic:=/no_gps
```

判断方式：

- 有 GPS 时，`/odometry/global` 应该被拉到 RTK 对应的地图位置，`map -> odom` 有明显全局
  偏移。
- 无 GPS 时，`/odometry/global` 更接近 FAST-LIO 本次启动的局部原点，不能自动落到真实
  地图位置。
- 如果两次 `/odometry/global` 表现完全一样，说明 GPS 没有被 global EKF 实际融合。

检查订阅关系：

```bash
ros2 param get /ekf_filter_node_map odom1
ros2 topic info /odometry/gps/smoothed -v
```

正常融合时，`odom1` 应该是 `/odometry/gps/smoothed`，并且
`ekf_filter_node_map` 是 `/odometry/gps/smoothed` 的订阅者。

## 坐标转换验证工具

### 地图点转经纬度

启动 bag 测试后，RViz 用 `Publish Point` 点地图，节点会输出 `/clicked_point/navsat`。

也可以手动发布：

```bash
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
"{header: {frame_id: 'map'}, point: {x: 10.0, y: -1.0, z: 0.0}}"

ros2 topic echo /clicked_point/navsat --once
```

这个工具用于回答：“地图上的某个点对应什么经纬度？”

### 经纬度转地图点

启动转换工具：

```bash
ros2 launch rtk_global_localization navsat_point_to_map.launch.py \
  start_map_server:=false \
  latitude:=nan \
  longitude:=nan
```

手动发经纬度：

```bash
ros2 topic pub --once /manual_navsat/fix sensor_msgs/msg/NavSatFix \
"{header: {frame_id: 'gps_link'}, latitude: 29.478029714355, longitude: 106.634170532227, altitude: 269.26}"
```

查看输出：

```bash
ros2 topic echo /manual_navsat/point --once
ros2 topic echo /manual_navsat/odom --once
```

这个工具只证明经纬度和 map 坐标的数学转换正确，不等于证明 RTK 已经融进 FAST-LIO。要
证明融合，仍然要看 `/odometry/global` 和 GPS A/B 对照。

### 模拟开机点

没有实车时，可以模拟一辆车出现在地图某个位置：

```bash
ros2 launch rtk_global_localization simulated_start_pose.launch.py \
  map_x:=10.0 \
  map_y:=-1.0 \
  yaw:=0.0
```

输出：

```text
/simulated_start/point
/simulated_start/navsat
/simulated_start/odom
map -> sim_body
```

运行中可以改点：

```bash
ros2 param set /simulated_start_pose map_x 15.0
ros2 param set /simulated_start_pose map_y -2.0
ros2 param set /simulated_start_pose yaw 1.57
```

这个工具用于演示和检查坐标关系，不参与真实 global EKF 融合。

## 常见问题

### RViz 显示 No map received

先确认 map server 已激活：

```bash
ros2 topic echo /map --once
ros2 lifecycle get /map_server
```

RViz 的 Map display 话题要选 `/map`，Fixed Frame 要设为 `map`。bag 模式下必须等
`rosbag play --clock` 开始后，使用 `use_sim_time` 的节点才会正常更新时间。

### 看不见 /odometry/local 或 /odometry/local_ekf

先看话题是否有数据：

```bash
ros2 topic echo /odometry/local --once
ros2 topic echo /odometry/local_ekf --once
ros2 topic info /odometry/local_ekf -v
```

如果话题有数据且 RViz 已订阅，通常是显示问题：

- `/odometry/local` 和 `/odometry/local_ekf` 在 `odom` 下。
- RViz Fixed Frame 是 `map` 时，会用 `map -> odom` 把它们变换到 map 下。
- 它们可能和 `/odometry/global` 重合，被颜色或点云盖住。
- 临时把 Fixed Frame 改成 `odom`，可以单独看局部轨迹。

### camera_init 坐标轴为什么会动

`camera_init` 挂在 `odom` 下面，不是车体。RViz Fixed Frame 设为 `map` 时，global EKF
持续修正 `map -> odom`，因此你会看到 `camera_init` 在 map 视角下移动。这不是车倒退，
也不是定位一定错。判断车位置请看 `body`、`/odometry/global` 和 Nav2 robot pose。

### navsat_transform 报 Latitude more than 20d from N pole

日志类似：

```text
Latitude 29.478d more than 20d from N pole
```

这是 `robot_localization` 在 UTM/UPS 转换时打印的错误级别日志。当前纬度 29.478 度本身
是正常纬度，后续如果看到：

```text
Datum UTM coordinate is (...)
/odometry/gps 有输出
```

通常可以继续观察，不要只因为这条日志就判断 RTK 不可用。

### RTK 融合后轨迹不够贴合地图

优先检查：

```text
1. map_origin.yaml 是否对应当前 2D 地图
2. body -> gps_link 杆臂是否正确
3. imu_yaw_correction 是否导致车头整体偏转
4. /odometry/gps/smoothed 协方差是否过大或过小
5. RTK fix 是否有跳变或 invalid fix
6. AMCL 是否也在发布 map -> odom
```

`gps_smoothed_position_variance_floor` 越大，RTK 拉 global EKF 越弱；越小，RTK 拉得越硬。
bag 测试里默认是 `25.0`，目的是让 RTK 温和修正，避免 GPS 台阶把轨迹拉抖。

### 改经纬度后六位，地图位置变化多少

在当前纬度附近：

```text
latitude +0.000001 约 0.111 m
longitude +0.000001 约 0.097 m
```

例如你之前测试：

```text
latitude +0.000010 -> ENU 北向约 1.108 m -> map 约 (0.713, -0.849)
latitude +0.000100 -> ENU 北向约 11.085 m -> map 约 (7.125, -8.491)
```

成倍增长是正常的，说明坐标转换工具工作正常。

## 当前工程的验证结论

目前已经能验证三件事：

1. `map_origin.yaml` 能把经纬度和 `map` 坐标互相转换。
2. bag 模式下 `/odometry/gps/smoothed` 能被 `ekf_filter_node_map` 订阅。
3. 有 GPS 和 `gps_odom_topic:=/no_gps` 的 A/B 对照能观察 global 轨迹是否被 RTK 拉到地图
   坐标。

需要注意的是：`map_point_to_navsat.py`、`navsat_point_to_map.py`、
`simulated_start_pose.py` 是验证工具，不是主融合链路。真正代表全局定位输出的是：

```text
/odometry/global
map -> odom
body 在 map 下的 TF
```

如果这些输出稳定，并且车在真实场景中的位置能和 2D 地图对应，就说明这套 RTK 辅助
FAST-LIO 全局定位链路已经起作用。

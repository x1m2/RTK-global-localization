import sys
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_typestore, Stores  # <-- 修改了这里

# 定义两个包的路径 (请确保路径正确)
OFFICIAL_BAG = Path('robot_localization/test/test3.bag')
USER_BAG = Path('ap_lidar_all_20260324_175502/ap_lidar_all_20260324_175502_0.db3')

def inspect_bag(path, label):
    print(f"\n{'='*20} {label} {'='*20}")
    print(f"路径: {path}")
    
    if not path.exists():
        print(f"❌ 错误: 找不到文件 {path}")
        return

    # 使用枚举常量 Stores.ROS2_HUMBLE 替代字符串 'humble'
    typestore = get_typestore(Stores.ROS2_HUMBLE)

    try:
        with AnyReader([path], default_typestore=typestore) as reader:
            print(f"\n[1. 话题与类型列表]")
            for connection in reader.connections:
                print(f"话题: {connection.topic:35} | 类型: {connection.msgtype}")

            print(f"\n[2. 数据结构透视 (关键对比)]")
            found_data = False
            # 遍历消息，寻找位姿/里程计/IMU
            for connection, timestamp, rawdata in reader.messages():
                msg = reader.deserialize(rawdata, connection.msgtype)
                
                # 对比位姿/里程计/IMU
                is_pose = 'Odometry' in connection.msgtype or 'Pose' in connection.msgtype
                is_imu = 'Imu' in connection.msgtype

                if is_pose or is_imu:
                    print(f">>> 发现传感器话题: {connection.topic}")
                    print(f"    - 消息完整类型: {connection.msgtype}")
                    
                    # 检查是否有 covariance 字段 (位姿和里程计)
                    if hasattr(msg, 'pose') and hasattr(msg.pose, 'covariance'):
                        print(f"    - 是否包含协方差(Covariance)字段: ✅ 是 (EKF认得)")
                        print(f"    - 协方差数据示例: {msg.pose.covariance[:3]} ...")
                    # 检查 IMU 的协方差
                    elif hasattr(msg, 'orientation_covariance'):
                        print(f"    - 是否包含 IMU 协方差字段: ✅ 是")
                        print(f"    - IMU 姿态协方差示例: {msg.orientation_covariance[:3]} ...")
                    else:
                        print(f"    - 是否包含协方差字段: ❌ 否 (EKF不认)")
                        print(f"    - 提示: 该格式不含置信度矩阵，robot_localization 将无法利用此数据。")
                    
                    found_data = True
                    break # 找到第一个有意义的话题就停止对比
            
            if not found_data:
                print("! 未在该包中找到位姿或IMU相关话题。")
    except Exception as e:
        print(f"❌ 读取包时发生错误: {e}")

# 执行比对
inspect_bag(OFFICIAL_BAG, "官方测试包 (ROS 1 格式)")
inspect_bag(USER_BAG, "你的数据包 (ROS 2 格式)")

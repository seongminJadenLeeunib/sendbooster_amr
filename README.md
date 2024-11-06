sendbooster_amr_bringup
개요
sendbooster_amr_bringup 프로젝트는 ROS 2 Humble 환경에서 운영되는 자율 이동 로봇(AMR)을 위한 ROS 패키지입니다. 이 패키지는 로봇의 엔코더, IMU, 오도메트리 데이터를 처리하여 로봇의 위치와 자세를 추정하고, 이를 /odom 토픽을 통해 다른 ROS 2 노드와 공유합니다. 이 패키지 내에는 각각 모터 드라이버(엔코더), IMU, 오도메트리 기능을 수행하는 세 개의 노드가 포함되어 있습니다.

주요 기능
motordriver: 엔코더 값을 주기적으로 발행하여 바퀴 회전 수를 제공합니다.
imu: IMU 센서의 yaw 데이터를 읽어 발행합니다.
odometry: 엔코더와 IMU 데이터를 구독하여 로봇의 위치와 자세를 계산한 후, /odom 토픽에 오도메트리 데이터를 발행합니다.
사용법
1. ROS 2 패키지 빌드
bash
코드 복사
# 워크스페이스에서 빌드
colcon build --packages-select sendbooster_amr_bringup
2. 노드 실행
bash
코드 복사
# 모든 노드 실행
ros2 launch sendbooster_amr_bringup bringup.launch.py
3. Rviz에서 시각화 확인
bash
코드 복사
# Rviz 실행
ros2 run rviz2 rviz2
Rviz에서 odom 프레임을 고정하여 오도메트리 데이터를 시각화할 수 있습니다. TF 트리와 Odometry 메시지를 확인해 로봇의 위치와 자세를 실시간으로 모니터링할 수 있습니다.

파일 트리
plaintext
코드 복사
sendbooster_amr_bringup/
├── launch/
│   └── bringup.launch.py           # 전체 노드를 실행하는 launch 파일
└── src/
    ├── motordriver.py              # 엔코더 노드, 주기적으로 바퀴 회전 수 발행
    ├── imu.py                      # IMU 노드, 주기적으로 yaw 값 발행
    └── odometry.py                 # 오도메트리 노드, 엔코더와 IMU 값을 구독해 위치 및 자세 계산
코드 해석
1. motordriver.py
motordriver.py는 모터 드라이버 노드로서 엔코더 데이터를 발행합니다. 이 데이터는 바퀴가 일정한 속도로 회전하는 것으로 가정해, 엔코더 값이 매번 10씩 증가하도록 설정되어 있습니다.

python
코드 복사
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motordriver')
        self.encoder_left_pub = self.create_publisher(Float64, 'encoder_left', 10)
        self.encoder_right_pub = self.create_publisher(Float64, 'encoder_right', 10)
        self.encoder_ticks_left = 0
        self.encoder_ticks_right = 0
        self.timer = self.create_timer(0.1, self.publish_encoder_data)

    def publish_encoder_data(self):
        self.encoder_ticks_left += 10
        self.encoder_ticks_right += 10
        self.encoder_left_pub.publish(Float64(data=self.encoder_ticks_left))
        self.encoder_right_pub.publish(Float64(data=self.encoder_ticks_right))

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
2. imu.py
imu.py는 IMU 노드로서 주기적으로 yaw 값을 발행합니다. (여기서는 시리얼 통신을 통해 yaw 값을 읽어오는 대신 시뮬레이션 값으로 구현합니다.)

python
코드 복사
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class IMU(Node):
    def __init__(self):
        super().__init__('imu')
        self.yaw_pub = self.create_publisher(Float64, 'yaw', 10)
        self.yaw_value = 0.0
        self.timer = self.create_timer(0.1, self.publish_yaw)

    def publish_yaw(self):
        self.yaw_pub.publish(Float64(data=self.yaw_value))
        self.yaw_value += 1.0

def main(args=None):
    rclpy.init(args=args)
    node = IMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
3. odometry.py
odometry.py는 오도메트리 노드로서 엔코더와 IMU 데이터를 구독하여 로봇의 위치(x, y)와 자세(theta)를 계산하고 /odom 토픽에 발행합니다.

python
코드 복사
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf2_ros
import geometry_msgs.msg
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_radius = 0.1
        self.wheel_base = 0.5
        self.create_subscription(Float64, 'encoder_left', self.encoder_left_callback, 10)
        self.create_subscription(Float64, 'encoder_right', self.encoder_right_callback, 10)
        self.create_subscription(Float64, 'yaw', self.imu_callback, 10)
        self.encoder_left = 0.0
        self.encoder_right = 0.0
        self.yaw = 0.0
        self.create_timer(0.2, self.update_odometry)

    def encoder_left_callback(self, msg):
        self.encoder_left = msg.data

    def encoder_right_callback(self, msg):
        self.encoder_right = msg.data

    def imu_callback(self, msg):
        self.yaw = msg.data
        self.theta = math.radians(self.yaw)

    def update_odometry(self):
        left_distance = self.encoder_left * self.wheel_radius
        right_distance = self.encoder_right * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        current_time = self.get_clock().now()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        quat = self.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(transform)
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.odom_pub.publish(odom_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
주의 사항
TF 프레임 설정: odometry.py에서 odom 프레임과 base_link 프레임을 설정하여 TF 트리를 구성해야 RViz에서 정상적으로 시각화됩니다.
IMU 데이터 및 엔코더 데이터 정확도: 현재 코드는 시뮬레이션용으로 임의의 데이터를 사용하고 있습니다. 실제 센서 데이터를 사용하는 경우 데이터 주기와 필터링 방법을 추가로 고려해야 합니다.

###USB포트 설정
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.4.1", SYMLINK+="sendbooster_motor_driver"
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", ATTRS{devpath}=="1.4.2", SYMLINK+="sendbooster_bat_state"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", ATTRS{devpath}=="1.4.4", SYMLINK+="sendbooster_imu"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.4", SYMLINK+="sendbooster_front_lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.1", SYMLINK+="sendbooster_center_lidar"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.2", SYMLINK+="sendbooster_mini_lidar_1"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.3.4", SYMLINK+="sendbooster_mini_lidar_2"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.3.1", SYMLINK+="sendbooster_mini_lidar_3"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{devpath}=="1.1.3.2", SYMLINK+="sendbooster_mini_lidar_4"

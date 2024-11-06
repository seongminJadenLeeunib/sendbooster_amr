import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf2_ros
import geometry_msgs.msg
import math
from std_msgs.msg import Float32

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        
        # Odometry Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        
        # IMU Subscriber
        self.imu_sub = self.create_subscription(Float32, 'imu_data', self.imu_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Wheel parameters
        self.wheel_radius = 0.1  # Wheel radius in meters
        self.wheel_base = 0.5     # Distance between the wheels in meters

        # Odometry variables
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation (yaw)

        # Timer to update odometry
        self.timer = self.create_timer(0.2, self.update_odometry)  # Update every 0.2 seconds (5 Hz)

        # Constant velocities for movement (instead of random)
        self.linear_velocity = 0.1  # Linear velocity in meters per second
        self.angular_velocity = 0.1  # Angular velocity in radians per second

    def imu_callback(self, msg):
        """IMU 데이터에서 yaw 값을 받아옴 (도에서 라디안으로 변환)"""
        degrees_yaw = msg.data  # IMU로 받은 yaw 값이 도 단위라고 가정
        self.theta = degrees_yaw * (math.pi / 180.0)  # 도를 라디안으로 변환

    def update_odometry(self):
        # 선속도와 각속도를 일정하게 설정
        left_linear_velocity = self.linear_velocity
        right_linear_velocity = self.linear_velocity

        # 평균 선속도 계산
        linear_velocity = (left_linear_velocity + right_linear_velocity) / 2.0
        angular_velocity = (right_linear_velocity - left_linear_velocity) / self.wheel_base

        # 현재 위치와 각도 업데이트 (0.2초 동안의 이동 거리)
        self.x += linear_velocity * math.cos(self.theta) * 0.2
        self.y += linear_velocity * math.sin(self.theta) * 0.2
        self.theta += angular_velocity * 0.2  # 0.2초 동안의 각도 변화

        # 트랜스폼 메시지 생성
        current_time = self.get_clock().now()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = current_time.to_msg()  # 항상 새로운 시간으로 갱신
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # 로봇의 오리엔테이션을 Quaternion으로 변환
        quat = self.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # 트랜스폼 브로드캐스터로 송출
        self.tf_broadcaster.sendTransform(transform)

        # 오도메트리 메시지 송출
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()  # 항상 새로운 시간으로 갱신
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=quat[0],
            y=quat[1],
            z=quat[2],
            w=quat[3]
        )

        self.odom_pub.publish(odom_msg)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        ]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

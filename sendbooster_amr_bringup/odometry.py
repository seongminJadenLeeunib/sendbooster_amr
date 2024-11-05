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
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Encoder and IMU data initialization
        self.encoder_ticks_left = 0
        self.encoder_ticks_right = 0
        self.wheel_radius = 0.1  # Wheel radius in meters
        self.wheel_base = 0.5     # Distance between the wheels in meters
        
        # Odometry variables
        self.x = 0.0  # Robot's x position
        self.y = 0.0  # Robot's y position
        self.theta = 0.0  # Robot's orientation (yaw)

        # Subscribe to encoder and IMU topics
        self.create_subscription(Float32, 'encoder_left', self.encoder_callback_left, 10)
        self.create_subscription(Float32, 'encoder_right', self.encoder_callback_right, 10)
        self.create_subscription(Float32, 'imu_data', self.imu_callback, 10)
        
        # Timer to update odometry
        self.timer = self.create_timer(0.2, self.update_odometry)  # Update every 0.2 seconds (5 Hz)

    def encoder_callback_left(self, msg):
        """Callback for left encoder data."""
        self.encoder_ticks_left = msg.data

    def encoder_callback_right(self, msg):
        """Callback for right encoder data."""
        self.encoder_ticks_right = msg.data

    def imu_callback(self, msg):
        """Callback for IMU yaw data."""
        self.theta = math.radians(msg.data)  # Yaw in radians

    def update_odometry(self):
        # Calculate distance and delta_theta
        left_distance = self.encoder_ticks_left * self.wheel_radius
        right_distance = self.encoder_ticks_right * self.wheel_radius
        distance = (left_distance + right_distance) / 2.0

        # Update x and y based on distance and theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Publish transform
        current_time = self.get_clock().now()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Convert theta to Quaternion
        quat = self.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Publish transform
        self.tf_broadcaster.sendTransform(transform)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
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

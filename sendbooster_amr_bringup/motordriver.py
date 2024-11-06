import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# 시리얼 포트 설정
ser = serial.Serial('/dev/ttyUSB8', baudrate=19200, timeout=1)

def calculate_checksum(packet):
    checksum_value = sum(packet) & 0xFF
    checksum = (~checksum_value + 1) & 0xFF
    return checksum

def send_packet(rpm1, rpm2):
    rpm1_low = rpm1 & 0xFF
    rpm1_high = (rpm1 >> 8) & 0xFF
    rpm2_low = rpm2 & 0xFF
    rpm2_high = (rpm2 >> 8) & 0xFF
    packet = [183, 184, 1, 207, 7, 1, rpm1_low, rpm1_high, 1, rpm2_low, rpm2_high, 0]
    checksum = calculate_checksum(packet)
    packet.append(checksum)
    ser.write(bytearray(packet))

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        wheel_base = 0.5
        wheel_radius = 0.1

        rpm1 = int(((linear_x - (angular_z * wheel_base / 2)) / (2 * 3.1416 * wheel_radius)) * -60)
        rpm2 = int(((linear_x + (angular_z * wheel_base / 2)) / (2 * 3.1416 * wheel_radius)) * 60)

        send_packet(rpm1, rpm2)

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

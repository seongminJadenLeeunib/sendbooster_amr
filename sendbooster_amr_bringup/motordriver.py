import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial

# 시리얼 포트 설정
motor_ser = serial.Serial('/dev/ttyUSB8', baudrate=19200, timeout=1)
distance_sensors = [
    serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1),
    serial.Serial('/dev/ttyUSB2', baudrate=115200, timeout=1),
    serial.Serial('/dev/ttyUSB3', baudrate=115200, timeout=1),
    serial.Serial('/dev/ttyUSB4', baudrate=115200, timeout=1)
]

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
    motor_ser.write(bytearray(packet))

def calculate_distance_checksum(data):
    """TFmini Plus 거리 체크섬 계산 함수"""
    return sum(data[:8]) & 0xFF

def parse_distance_data(data_frame):
    """TFmini Plus 거리 데이터 파싱 함수"""
    if len(data_frame) == 9 and data_frame[0] == 0x59 and data_frame[1] == 0x59:
        distance = data_frame[2] | (data_frame[3] << 8)
        strength = data_frame[4] | (data_frame[5] << 8)
        temp = data_frame[6] | (data_frame[7] << 8)
        checksum = data_frame[8]
        
        if calculate_distance_checksum(data_frame) == checksum:
            return distance
    return None

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # cmd_vel 토픽 구독
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        
        # motor_rpm 토픽에 왼쪽과 오른쪽 RPM 값을 발행하는 퍼블리셔 생성
        self.motor_rpm_pub = self.create_publisher(Float32MultiArray, 'motor_rpm', 10)

    def listener_callback(self, msg):
        # 모든 거리 센서에서 데이터를 읽고 30cm 이하인 센서가 있는지 확인
        stop_motor = False
        for sensor in distance_sensors:
            data_frame = sensor.read(9)
            if len(data_frame) == 9:
                distance = parse_distance_data(data_frame)
                if distance is not None and distance <= 30:
                    stop_motor = True
                    break

        if stop_motor:
            # 거리 30cm 이하인 경우 RPM을 0으로 설정
            rpm1 = rpm2 = 0
            self.get_logger().info("거리 30cm 이하 감지: 모터 정지")
        else:
            # cmd_vel 메시지를 통해 모터 RPM 계산
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            wheel_base = 0.745
            wheel_radius = 0.095
            gear_ratio = 10  # 기어비 1:10

            # 기어비를 고려하여 RPM 계산
            rpm1 = int(((linear_x - (angular_z * wheel_base / 2)) / (2 * 3.1416 * wheel_radius)) * -60 * gear_ratio)
            rpm2 = int(((linear_x + (angular_z * wheel_base / 2)) / (2 * 3.1416 * wheel_radius)) * 60 * gear_ratio)

        # RPM 값 시리얼 포트로 전송
        send_packet(rpm1, rpm2)

        # motor_rpm 토픽에 왼쪽과 오른쪽 RPM 값 발행
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [rpm1, rpm2]
        self.motor_rpm_pub.publish(rpm_msg)

        self.get_logger().info(f"Published motor RPMs - Left: {rpm1}, Right: {rpm2}")

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

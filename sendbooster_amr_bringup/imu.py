import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import threading

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        # IMU 데이터 발행자 생성
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        self.angle_pub = self.create_publisher(Vector3, 'imu_angles', 10)

        # 시리얼 포트 설정 (수정 필요)
        self.ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
        self.ser.write(b'ss=7\n')  # IMU 모드 설정

        # 별도 스레드로 IMU 데이터를 주기적으로 읽기
        self.imu_thread = threading.Thread(target=self.publish_imu_data)
        self.imu_thread.daemon = True
        self.imu_thread.start()

    def publish_imu_data(self):
        while True:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f"Raw IMU Data: {line}")  # 원시 IMU 데이터 출력

                try:
                    # IMU 데이터를 파싱하여 가속도, 각속도, 각도 값으로 구분
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, angle_x, angle_y, angle_z = m>

                    # Imu 메시지 생성
                    imu_msg = Imu()
                    imu_msg.linear_acceleration.x = accel_x
                    imu_msg.linear_acceleration.y = accel_y
                    imu_msg.linear_acceleration.z = accel_z
                    imu_msg.angular_velocity.x = gyro_x
                    imu_msg.angular_velocity.y = gyro_y
                    imu_msg.angular_velocity.z = gyro_z

                    # 각도 데이터는 Vector3 메시지로 별도 발행
                    angle_msg = Vector3()
                    angle_msg.x = angle_x
                    angle_msg.y = angle_y
                    angle_msg.z = angle_z  # yaw
                    
                    # 토픽에 데이터 게시
                    self.imu_pub.publish(imu_msg)
                    self.angle_pub.publish(angle_msg)

                    # 각도 데이터 출력
                    self.get_logger().info(f"Published IMU Data - Accel: ({accel_x:.2f}, {accel_y:.2>
                                           f"Gyro: ({gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}), "
                                           f"Angle: ({angle_x:.2f}, {angle_y:.2f}, {angle_z:.2f})")
                    except ValueError:
                        self.get_logger().warn(f"Invalid data format: {line}")  # 잘못된 데이터 형식 경고

    def main(args=None):
        rclpy.init(args=args)
        node = IMUNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

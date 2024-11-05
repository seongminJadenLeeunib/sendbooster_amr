import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import threading

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # IMU 데이터 발행자 생성
        self.imu_pub = self.create_publisher(Float32, 'imu_data', 10)
        
        # 시리얼 포트 설정 (수정 필요)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser.write(b'ss=4\n')  # IMU 모드 설정

        # 별도 스레드로 IMU 데이터를 주기적으로 읽기
        self.imu_thread = threading.Thread(target=self.publish_imu_data)
        self.imu_thread.daemon = True
        self.imu_thread.start()

    def publish_imu_data(self):
        while True:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                try:
                    _, _, z = map(float, line.split())
                    yaw = z  # z축 값을 yaw로 사용
                    msg = Float32()
                    msg.data = yaw
                    self.imu_pub.publish(msg)
                    print(f"Published IMU Yaw: {yaw:.2f}")
                except ValueError:
                    print(f"Invalid data format: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

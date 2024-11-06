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
        while rclpy.ok():
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                try:
                    # IMU 데이터 형식이 "x y z" 형태로 오는 것으로 가정
                    _, _, z = map(float, line.split())
                    yaw = z  # z축 값을 yaw로 사용
                    msg = Float32()
                    msg.data = yaw
                    self.imu_pub.publish(msg)
                    self.get_logger().info(f"Published IMU Yaw: {yaw:.2f}")
                except ValueError:
                    self.get_logger().warn(f"Invalid data format: {line}")
                except Exception as e:
                    self.get_logger().error(f"Error reading IMU data: {e}")

    def destroy(self):
        """Gracefully stop the IMU thread when shutting down the node"""
        self.imu_thread.join()  # Ensure the thread is properly joined before shutdown
        self.ser.close()  # Close the serial connection

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder')
        
        # Publishers for left and right encoder values
        self.encoder_left_pub = self.create_publisher(Float32, 'encoder_left', 10)
        self.encoder_right_pub = self.create_publisher(Float32, 'encoder_right', 10)
        
        # Initialize encoder values
        self.encoder_left_value = 0.0
        self.encoder_right_value = 0.0
        
        # Timer to update encoder values
        self.timer = self.create_timer(0.2, self.update_encoder_values)  # Update every 0.2 seconds

    def update_encoder_values(self):
        # Increment encoder values by 10 each update
        self.encoder_left_value += 0.01
        self.encoder_right_value += 0.01

        # Publish encoder values
        left_msg = Float32()
        right_msg = Float32()
        left_msg.data = self.encoder_left_value
        right_msg.data = self.encoder_right_value
        
        self.encoder_left_pub.publish(left_msg)
        self.encoder_right_pub.publish(right_msg)
        
        self.get_logger().info(f"Published Left Encoder: {self.encoder_left_value}, Right Encoder: {self.encoder_right_value}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

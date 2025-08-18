import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')
        
        # Publisher vers /vention
        self.publisher_ = self.create_publisher(JointState, '/vention', 10)
        
        # Subscriber sur /Fr3_labo/joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/Fr3_labo/joint_states',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        # Republie le message reçu directement
        self.publisher_.publish(msg)
        self.get_logger().info('Republished joint states on /vention')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class SafetyStopNode(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        qos_profile=QoSProfile(depth=10,reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile) #,
            #10)
        self.publisher = self.create_publisher(Bool, 'stop_signal', 10)

    def scan_callback(self, msg):
        stop_signal = Bool()
        stop_signal.data = any(distance < 0.3 for distance in msg.ranges)

        self.publisher.publish(stop_signal)
        if stop_signal.data:
            self.get_logger().info('Stop signal sent: Obstacle detected')

def main(args=None):
    rclpy.init(args=args)
    safety_stop_node = SafetyStopNode()
    rclpy.spin(safety_stop_node)
    safety_stop_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

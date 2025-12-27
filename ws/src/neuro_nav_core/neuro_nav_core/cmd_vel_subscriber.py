import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("cmd_vel_subscriber")
        self.subscription = self.create_subscription(
            Twist,
            "cmd_vel",
            self.listener_callback,
            10
        )
        self.get_logger().info("CmdVel Subscriber")

    
    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received -> linear.x: {msg.linear.x}, angular.z: {msg.angular.z}'
        )
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
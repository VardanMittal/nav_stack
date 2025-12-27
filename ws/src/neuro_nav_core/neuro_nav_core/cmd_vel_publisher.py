import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_publisher")
        self.publisher_ = self.create_publisher(Twist,"cmd_vel",10)
        self.timer = self.create_timer(1.0,self.publish_velocity)
        self.get_logger().info("CmdVel Publisher")

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 0.3
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published cmd_vel: linear.x={msg.linear.x:.2f} angular.z={msg.angular.z:.2f}")



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
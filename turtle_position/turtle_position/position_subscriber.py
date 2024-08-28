import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePositionSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_position_subscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # Impede o aviso de variável não utilizada

    def pose_callback(self, msg):
        self.get_logger().info(f'Posição da Tartaruga - x: {msg.x}, y: {msg.y}, theta: {msg.theta}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePositionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_mover')
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_pose = None
        self.is_moving_forward = True

    def pose_callback(self, msg):
        self.current_pose = msg
        self.check_wall_collision()

    def check_wall_collision(self):
        if self.current_pose:
            # Definir os limites do ambiente turtlesim
            threshold = 0.5  # Distância mínima da parede para parar
            x_limit = 11.0 - threshold
            y_limit = 11.0 - threshold

            # Verificar se a tartaruga está próxima de qualquer borda
            if self.current_pose.x < threshold or self.current_pose.x > x_limit or \
               self.current_pose.y < threshold or self.current_pose.y > y_limit:
                self.change_direction()
            else:
                self.move_forward()

    def move_forward(self):
        if self.is_moving_forward:
            velocity_msg = Twist()
            velocity_msg.linear.x = 1.0  # Velocidade linear para frente
            velocity_msg.angular.z = 0.0  # Sem rotação
            self.velocity_publisher.publish(velocity_msg)
            self.get_logger().info(f'Movendo para frente - Posição atual: x={self.current_pose.x}, y={self.current_pose.y}')

    def change_direction(self):
        if self.is_moving_forward:
            self.is_moving_forward = False
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.0  # Parar o movimento linear
            velocity_msg.angular.z = 1.0  # Girar em torno do eixo z
            self.velocity_publisher.publish(velocity_msg)
            self.get_logger().info('Mudando de direção...')

            # Pausar o movimento para permitir a rotação
            self.create_timer(1.0, self.resume_movement)

    def resume_movement(self):
        self.is_moving_forward = True
        self.move_forward()

def main(args=None):
    rclpy.init(args=args)
    node = TurtleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

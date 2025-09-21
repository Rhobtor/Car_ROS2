import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, '/wheel_torque_command', 10)
        self.get_logger().info('Node started. Use W/S for forward/backward, A/D for turning.')

    def run(self):
        try:
            while True:
                key = input("Enter command (W/S/A/D): ").lower()
                msg = Twist()

                if key == 'w':
                    msg.linear.x = 9.0  # Avanzar
                elif key == 's':
                    msg.linear.x = -9.0  # Retroceder
                elif key == 'a':
                    msg.angular.z = 2.0  # Girar a la izquierda
                elif key == 'd':
                    msg.angular.z = -2.0  # Girar a la derecha
                else:
                    self.get_logger().info("Invalid key. Use W/S/A/D.")
                    continue

                self.publisher.publish(msg)  # Publica el mensaje
                self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')
        except KeyboardInterrupt:
            self.get_logger().info("Shutting down keyboard control node.")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
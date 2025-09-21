import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control')
        self.joy_subscriber = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        self.command_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.get_logger().info("Joystick control node started.")

    def joy_callback(self, msg: Joy):
        # Crear el mensaje Twist basado en el input del joystick
        twist_msg = Twist()

        # Asignar ejes del joystick a linear.x y angular.z
        twist_msg.linear.x = msg.axes[1] * 5.0  # Eje izquierdo vertical (escala al torque)
        twist_msg.angular.z = msg.axes[3] * 5.0    # Eje izquierdo horizontal (dirección)

        # Publicar el mensaje Twist
        self.command_publisher.publish(twist_msg)
        self.get_logger().info(f"Published: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    joystick_control_node = JoystickControlNode()
    rclpy.spin(joystick_control_node)
    joystick_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

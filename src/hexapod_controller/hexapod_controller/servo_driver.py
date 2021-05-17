# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

# PCA9685 imports
import board
import busio
import adafruit_motor.servo
import adafruit_pca9685
from adafruit_servokit import ServoKit


class AngleSubscriber(Node):

    def __init__(self, servo):
        super().__init__('angle_subscriber')
        self.subscription = self.create_subscription(
            Int16,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.servo = servo

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.servo.angle = msg.data


def main(args=None):

    # Initialize the I2C connection
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = adafruit_pca9685.PCA9685(i2c)

    # PCA configurations
    kit = ServoKit(channels=16)

    # PWM parameters
    pca.frequency = 50
    kit.servo[0].set_pulse_width_range(500, 2500)

    rclpy.init(args=args)
    angle_subscriber = AngleSubscriber(kit.servo[0])
    rclpy.spin(angle_subscriber)

    # Destroy the node explicitly
    angle_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

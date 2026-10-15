import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import RPi.GPIO as GPIO
from time import sleep

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Define the GPIO pins for the two servos
        self.SERVO_PIN_1 = 33
        self.SERVO_PIN_2 = 15

        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.SERVO_PIN_1, GPIO.OUT)
        GPIO.setup(self.SERVO_PIN_2, GPIO.OUT)

        # Subscriber to control servos
        self.subscription = self.create_subscription(
            Float64MultiArray,  # Data type (2 angles)
            'servo_angles',  # Topic name
            self.listener_callback,  # Callback function
            10  # Queue size
        )

    def set_angle(self, angle1, angle2):
        # Calculate duty cycle from angle


        duty1 = angle1 / 18 + 2
        duty2 = angle2 / 18 + 2

        self.pwm1=0
        self.pwm1 = GPIO.PWM(self.SERVO_PIN_1, 50)  # 50Hz frequency
        self.pwm1.start(0)
        self.pwm1.ChangeDutyCycle(duty1)

        self.pwm2=0
        self.pwm2 = GPIO.PWM(self.SERVO_PIN_2, 50)  # 50Hz frequency
        self.pwm2.start(0)
        self.pwm2.ChangeDutyCycle(duty2)
        sleep(1)  # Small delay to allow servos to reach position
        
        self.pwm1=0
        self.pwm1 = GPIO.PWM(self.SERVO_PIN_1, 50)  # 50Hz frequency
        self.pwm1.ChangeDutyCycle(0)  # Stop sending signals to avoid jitter

        self.pwm2=0
        self.pwm2 = GPIO.PWM(self.SERVO_PIN_2, 50)  # 50Hz frequency
        self.pwm2.ChangeDutyCycle(0)  # Stop sending signals to avoid jitter

    def listener_callback(self, msg):
        # msg.data should be an array of two values (angle1, angle2)
        if len(msg.data) == 2:
            angle1, angle2 = msg.data
            self.get_logger().info(f'Received angles: {angle1}, {angle2}')
            self.set_angle(angle1, angle2)
        else:
            self.get_logger().error('Received incorrect number of angles.')

    # def destroy_node(self):
    #     self.pwm1.stop()
    #     self.pwm2.stop()
    #     GPIO.cleanup()
    #     super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)

    # # Cleanup on shutdown
    # servo_controller.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()


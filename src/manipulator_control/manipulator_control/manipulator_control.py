import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Float64MultiArray
from dynamixel_sdk_custom_interfaces.msg import SetMultiplePositions
import time




class manipulator_control(Node):
    def __init__(self):
        super().__init__('manipulator_control')
        
        # Subscriptions
        self.manipulator_start_subscription = self.create_subscription(
            Int32,
            '/manipulator_start_flag',
            self.transition_callback,
            10)
        
        # Publishers
        self.collection_unit_publisher = self.create_publisher(
            Int32,
            '/cu_cmd',
             10)
        
        self.go_home_publisher = self.create_publisher(
            Int32,
            '/bin_full_id',
             10)
        
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
             10)
        
        self.dynamixel_publisher = self.create_publisher(
            SetMultiplePositions,
            '/set_position',
             10)
        
        self.servo_publisher = self.create_publisher(
            Float64MultiArray,
            '/servo_angles',
             10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
             10)

        # Variables
        self.sleep_time = 5.0
        self.grip_servo = [170.0,20.0]
        self.ungrip_servo = [10.0,190.0]
        self.flag = 0

        # Timers
        self.cmd_vel_timer = self.create_timer(0.1, self.publish_cmd_vel)

    def transition_callback(self, msg):
        if msg.data == 1:

            # make sure the gripper is open and at home position
            self.flag = 1
            self.publish_servo_angles(self.ungrip_servo)
            DynamixelPublishMsg = SetMultiplePositions()
            DynamixelPublishMsg.ids = [1,2]
            DynamixelPublishMsg.position = 180.0
            self.dynamixel_publisher.publish(DynamixelPublishMsg)
            self.get_logger().info("started manipulation control please wait ..")
            time.sleep(self.sleep_time)
            
            # manipulator down (wait duration = 10.0 sec , position = 8 degrees)
            DynamixelPublishMsg = SetMultiplePositions()
            DynamixelPublishMsg.ids = [1,2]
            DynamixelPublishMsg.position = 8.0
            self.dynamixel_publisher.publish(DynamixelPublishMsg)
            self.get_logger().info("manipulator going down please wait ..")
            time.sleep(self.sleep_time)
            
            # grip the bin (wait duration = 5.0 sec , position = servos angles)
            self.publish_servo_angles(self.grip_servo)
            self.get_logger().info("closing gripper please wait ..")
            time.sleep(10.0)
            
            # # open CU an wait 10.0 seconds for it to open
            # cu_cmd = Int32()
            # cu_cmd.data = 1
            # self.collection_unit_publisher.publish(cu_cmd)
            # self.get_logger().info("opening collection unit please wait ..")
            # time.sleep(self.sleep_time)
            
            # manipulator to drain position (wait duration = 10.0 sec , position = 95 degrees)
            DynamixelPublishMsg.position = 85.0
            self.dynamixel_publisher.publish(DynamixelPublishMsg)
            self.get_logger().info("drainning bin please wait ..")
            time.sleep(self.sleep_time)

            # drain time untill bin is empty
            time.sleep(15.0)
            self.get_logger().info("drainning bin please wait ..")

            self.get_logger().info("draining done ..")


            # manipulator to puts down the bin (wait duration = 10.0 sec , position = 8 degrees)
            DynamixelPublishMsg.position = 8.0
            self.dynamixel_publisher.publish(DynamixelPublishMsg)
            self.get_logger().info("Please wait for the manipulator to close ..")
            time.sleep(self.sleep_time)

            # # close CU an wait 10.0 seconds for it to close
            # cu_cmd.data = 0
            # self.collection_unit_publisher.publish(cu_cmd)
            # time.sleep(self.sleep_time)
            
            # grip the bin (wait duration = 10.0 sec , position = servos angles)
            self.publish_servo_angles(self.ungrip_servo)
            self.publish_servo_angles(self.ungrip_servo)
            self.publish_servo_angles(self.ungrip_servo)
            self.publish_servo_angles(self.ungrip_servo)
            time.sleep(10.0)

            # manipulator return to home position (wait duration = 10.0 sec , position = 180 degrees)
            DynamixelPublishMsg.position = 180.0
            self.dynamixel_publisher.publish(DynamixelPublishMsg)
            time.sleep(self.sleep_time)
            self.get_logger().info("draining done ..")

            self.publish_hold()
            time.sleep(5.0)

            self.publish_cmd_vel()
            go_home = Int32()
            go_home.data = 51
            self.go_home_publisher.publish(go_home)

            time.sleep(self.sleep_time)
            
        else:
            pass

    def publish_servo_angles(self, angles):
        servo_msg = Float64MultiArray()
        servo_msg.data = angles
        self.servo_publisher.publish(servo_msg)


    def publish_hold(self):
        if self.flag == 1:
            msg = Twist()
            msg.linear.x = -0.1
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)
    

    def publish_cmd_vel(self):
        if self.flag == 1:
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)
    


def main(args=None):
    rclpy.init(args=args)
    manipulator = manipulator_control()
    rclpy.spin(manipulator)
    manipulator_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

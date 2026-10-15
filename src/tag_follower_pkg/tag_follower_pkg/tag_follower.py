import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import cv2
from cv_bridge import CvBridge
import apriltag
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile
from action_msgs.msg import GoalStatusArray
import time





class TagDistanceCalculator(Node):
    def __init__(self):
        super().__init__('tag_follower')

        self.get_logger().info('Tag Follower On')
        # self._action_client = ActionClient(self, NavigateToPose,'navigate_to_pose')

        # Parameters for control
        self.declare_parameter('linear_velocity', 0.1)  # Base forward velocity
        self.declare_parameter('angular_velocity', 0.2)  # Base angular velocity
        self.declare_parameter('size_threshold', 187.0)  # Stop threshold based on tag size
        self.declare_parameter('proportional_gain', 0.005)  # Gain for angular control
        
        # Publishers
        self.publisher = self.create_publisher(Twist, '/cmd_vel_tracker', 10)
        self.manipulator_start_publisher = self.create_publisher(Int32, '/manipulator_start_flag', 10)
        
        # Subscriber for image_raw
        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # CV Bridge for converting ROS Image to OpenCV image
        self.bridge = CvBridge()

        # AprilTag detector
        self.detector = apriltag.Detector()

        # State variables
        self.goal_reached = False
        self.centered = False


        # Camera frame dimensions
        self.image_width = 640  # Initialize with default, updated dynamically
        self.image_height = 480






    def image_callback(self, msg):
        """Process the incoming image, detect AprilTags, and control the robot."""

        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

            # Update image dimensions
            self.image_height, self.image_width = cv_image.shape[:2]

            # Detect AprilTags
            detections = self.detector.detect(cv_image)

            if not detections:
                self.get_logger().info("No tags detected. Stopping robot.")
                self.stop_robot()
                return

            # Process the first detected tag
            detection = detections[0]  # Assuming the first tag is the one to follow
            self.align_and_approach(detection)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            self.stop_robot()

    def align_and_approach(self, detection):
        """Control logic for centering and stopping at the tag."""
        size_threshold = self.get_parameter('size_threshold').value
        proportional_gain = self.get_parameter('proportional_gain').value
        linear_velocity = self.get_parameter('linear_velocity').value
        angular_velocity = self.get_parameter('angular_velocity').value

        corners = detection.corners
        center_x = detection.center[0]
        tag_size = self.calculate_tag_size(detection)

        # Calculate horizontal error based on corners
        left_edge = corners[0][0]  # X-coordinate of the leftmost corner
        right_edge = corners[1][0]  # X-coordinate of the rightmost corner
        center_error = center_x - (self.image_width / 2)

        # Calculate cornering radius (distance between edges and frame edges)
        left_error = left_edge
        right_error = self.image_width - right_edge

        self.get_logger().info(
            f"Error X: {center_error}, Left Error: {left_error}, Right Error: {right_error}, Tag Size: {tag_size}"
        )

        twist = Twist()

        # Adjust linear velocity to approach the tag if below size threshold
        if tag_size < size_threshold:
            twist.linear.x = linear_velocity
            self.centered = False
        else:
            twist.linear.x = 0.0
            self.goal_reached = True

        # Adjust angular velocity based on cornering errors
        if abs(center_error) > 5:  # Center alignment
            twist.angular.z = -proportional_gain * center_error
        elif abs(left_error - right_error) > 10:  # Corner alignment
            twist.angular.z = proportional_gain * (left_error - right_error)
        else:
            twist.angular.z = 0.0
            self.centered = True

        self.publisher.publish(twist)

        if self.goal_reached and self.centered:
            self.signal_manipulator_start()

    def calculate_tag_size(self, detection):
        """Calculate the average size of the detected tag."""
        corners = detection.corners
        tag_width = np.linalg.norm(corners[0] - corners[1])
        tag_height = np.linalg.norm(corners[1] - corners[2])
        return (tag_width + tag_height) / 2

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def signal_manipulator_start(self):
        """Publish a signal to start the manipulator."""
        flag = Int32()
        flag.data = 1
        self.manipulator_start_publisher.publish(flag)
        self.get_logger().info("Goal reached. Starting manipulator...")
        time.sleep(120.0)






        

def main(args=None):
    rclpy.init(args=args)
    tag_distance_calculator = TagDistanceCalculator()
    rclpy.spin(tag_distance_calculator)
    tag_distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

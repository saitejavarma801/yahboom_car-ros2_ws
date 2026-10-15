# # Copyright 2023 Josh Newans
# #
# # Licensed under the Apache License, Version 2.0 (the "License");
# # you may not use this file except in compliance with the License.
# # You may obtain a copy of the License at
# #
# #     http://www.apache.org/licenses/LICENSE-2.0
# #
# # Unless required by applicable law or agreed to in writing, software
# # distributed under the License is distributed on an "AS IS" BASIS,
# # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# # See the License for the specific language governing permissions and
# # limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
import ball_tracker.process_image as proc

class DetectBall(Node):

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Looking for the ball...')
        self.image_sub = self.create_subscription(Image,"/image_in",self.callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_pub  = self.create_publisher(Point,"/detected_ball",1)

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",0)
        self.declare_parameter("h_max",180)
        self.declare_parameter("s_min",0)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",100)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_pub.publish(point_out) 
        except CvBridgeError as e:
            print(e)  


def main(args=None):

    rclpy.init(args=args)

    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)
        proc.wait_on_gui()

    detect_ball.destroy_node()
    rclpy.shutdown()

# Copyright 2023 Josh Newans
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
# from cv_bridge import CvBridge, CvBridgeError
# import ball_tracker.process_image as proc  # Assuming you have renamed this package to something else for AprilTag

# class DetectAprilTag(Node):

#     def __init__(self):
#         super().__init__('detect_ball')

#         self.get_logger().info('Looking for the AprilTag...')
#         self.image_sub = self.create_subscription(Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
#         self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
#         self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
#         self.apriltag_pub = self.create_publisher(Point, "/detected_apriltag", 1)

#         self.declare_parameter('tuning_mode', False)

#         # These parameters can be adjusted as needed or removed if irrelevant for AprilTag detection
#         self.declare_parameter("x_min", 0)
#         self.declare_parameter("x_max", 100)
#         self.declare_parameter("y_min", 0)
#         self.declare_parameter("y_max", 100)

#         self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
#         self.tuning_params = {
#             'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
#             'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
#             'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
#             'y_max': self.get_parameter('y_max').get_parameter_value().integer_value
#         }

#         self.bridge = CvBridge()

#         if self.tuning_mode:
#             proc.create_tuning_window(self.tuning_params)

#     def callback(self, data):  # <-- Indented inside the class
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         try:
#             if self.tuning_mode:
#                 self.tuning_params = proc.get_tuning_params()

#             # Replace find_circles with the function to detect AprilTags
#             keypoints_norm, out_image, tuning_image = proc.find_apriltags(cv_image, self.tuning_params)

#             img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
#             img_to_pub.header = data.header
#             self.image_out_pub.publish(img_to_pub)

#             img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
#             img_to_pub.header = data.header
#             self.image_tuning_pub.publish(img_to_pub)

#             point_out = Point()

#             # Process the detected tags, assuming keypoints_norm contains tag centers
#             for i, kp in enumerate(keypoints_norm):
#                 x = kp[0]
#                 y = kp[1]
#                 s = 1.0  # Ensure s is a float

#                 self.get_logger().info(f"Tag {i}: ({x},{y},{s})")

#                 # Publish the tag data
#                 if s > point_out.z:                    
#                     point_out.x = x
#                     point_out.y = y
#                     point_out.z = s  # Ensure this is a float

#             if point_out.z > 0:
#                 self.apriltag_pub.publish(point_out)

#         except CvBridgeError as e:
#             print(e)

# def main(args=None):
#     rclpy.init(args=args)
#     detect_apriltag = DetectAprilTag()

#     while rclpy.ok():
#         rclpy.spin_once(detect_apriltag)
#         proc.wait_on_gui()

#     detect_apriltag.destroy_node()
#     rclpy.shutdown()

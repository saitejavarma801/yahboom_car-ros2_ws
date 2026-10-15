import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
from math import pi , cos , sin
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy


class robot_navigation(Node):
    def __init__(self):
        super().__init__('robot_nav')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,depth=10)

        self.bins_list = [[0.0,0.0],
                          [2.0,-0.004],
                          [3.5,0.004],
                          [5.9,-0.004],
                          [4.7,-0.5],
                          [9.0,0.8],
                          [9.0,-0.8],
                          [12.0,0.8],
                          [12.0,-0.3],
                          [15.0,0.3],
                          [15.0,-0.3]
                          ]

        self.current_bin_full_id = Int32()
        self.goal_pose = PoseStamped()
        
        self.bin_full_subscription = self.create_subscription(
            Int32,
            '/bin_full_id',
            self.bin_full_id_callback,
            qos_profile
        )
        
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        


    def bin_full_id_callback(self,bin_full_id_msg):

        
        self.current_bin_full_id = bin_full_id_msg
        print("Cuurent Bin Goal Set To : "+ str(bin_full_id_msg.data - 51))
        
        idx = bin_full_id_msg.data - 51
        print('ID: ' , idx)
        
	# go to dock
        if(idx==0):
            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = self.bins_list[idx][0]
            self.goal_pose.pose.position.y = self.bins_list[idx][1]

            self.goal_pose.pose.orientation.x = 0.0
            self.goal_pose.pose.orientation.y = 0.0
            self.goal_pose.pose.orientation.z = 0.0
            self.goal_pose.pose.orientation.w = 1.0

            #self.goal_pose_publisher.publish(self.goal_pose)
        # go to bin
        elif(idx>0):

            self.goal_pose.header.frame_id = "map"
            self.goal_pose.pose.position.x = self.bins_list[idx][0]
            self.goal_pose.pose.position.y = self.bins_list[idx][1]

            if(self.bins_list[idx][1]>0):
                q = self.euler_to_quaternion(0,0,-1.5707)
                self.goal_pose.pose.orientation.x = 0.0
                self.goal_pose.pose.orientation.y = 0.0
                self.goal_pose.pose.orientation.z = -0.707
                self.goal_pose.pose.orientation.w = 0.707
                print(q)
            else:
                q = self.euler_to_quaternion(0,0,1.5707)
                self.goal_pose.pose.orientation.x = 0.0
                self.goal_pose.pose.orientation.y = 0.0
                self.goal_pose.pose.orientation.z = 0.707
                self.goal_pose.pose.orientation.w = 0.7071
                print(q)
        
            self.goal_pose_publisher.publish(self.goal_pose)
            #print("X : " + str(self.goal_pose.pose.position.x) )
            #print("Y : " + str(self.goal_pose.pose.position.y) )
            #print(self.goal_pose)
        else:
            print("Invalid Goal Index : Index "+ str(bin_full_id_msg.data - 51))



    def euler_to_quaternion(self,yaw, pitch, roll):

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            return [qx, qy, qz, qw]



def main(args=None):
    rclpy.init(args=args)
    navigate = robot_navigation()
    rclpy.spin(navigate)
    robot_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

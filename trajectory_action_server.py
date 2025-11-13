'''
Author : Bineesha Jabi
Date : 5 / 11 / 2025

Description : Publishes
'''
import rclpy

from rclpy.node import Node

from rclpy.node import Node
from builtin_interfaces.msg import Duration 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header

class TrajectoryPublisher(Node):
    
    def __init__(self):
        super().__init__('trajectory_publisher_node')
        self.time_period = 5.0
        self.duration_sec = 2
        self.timer = self.create_timer(self.time_period, self.timer_callback)
        self.trajectory_publisher =self.create_publisher(JointTrajectory,"/joint_trajectory_controller/follow_joint_trajectory ", 10)

        #Set the poses for the arm
        self.arm_positions=[]
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Home location
        self.arm_positions.append([-1.345, -1.23, 0.264, -0.296, 0.389, -1.5]) # Goal location
        self.arm_positions.append([-1.345, -1.23, 0.264, -0.296, 0.389, -1.5]) 
        self.arm_positions.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # Home location

        #To track trajectory
        self.index =0

    def timer_callback(self):

        #Declaring the joints
        joints = ['link1_to_link2','link2_to_link3','link3_to_link4','link4_to_link5','link5_to_link6','link6_to_link6_flange']

        #Create new JointTrajectory messages
        my_trajectory_msg=JointTrajectory()
        my_trajectory_msg.header = Header()
        my_trajectory_msg.joint_names=joints

        #Create JointTrajectoryPoints
        point_msg = JointTrajectoryPoint()
        point_msg.positions = self.arm_positions[self.index]
        point_msg.time_from_start = Duration(sec=int(self.duration_sec))

        my_trajectory_msg.points.append(point_msg)
        
        self.trajectory_publisher.publish(my_trajectory_msg)

        #Reset the index
        if self.index == len(self.arm_positions) - 1:
              self.index = 0
        else :
              self.index = self.index + 1
def main(args=None):
        

        #Initialize the rclpy library
        rclpy.init(args=args)

        #Declare the node
        arm_traj_path=TrajectoryPublisher()

        #Spin the node so the callback function is called
        rclpy.spin(arm_traj_path)

        #Destroy the node
        arm_traj_path.destroy_node()

        #Shutdown the python library
        rclpy.shutdown()

if __name__== "__main__" :
        main()




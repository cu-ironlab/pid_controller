#!/usr/bin/python

import PID
import time
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import *
from gazebo_msgs.msg import ModelStates
import sys
#Subscribe to current_robot_pose
#Subscribe to goal_pose
#Create 2 PID controller objects, 1 - Rotation/Orientation 2 - Position
#Order orient along target location -> move to target position -> orient along final orientation
#Create turning and straight methods (standard sampling time)
class pid_node:

    def __init__(self):        

		######PID Gains

		self.pid_orient = PID.PID(1,0,0)
		self.pid_pos = PID.PID(1,0,0)

		################Initializations
		self.output_pub = rospy.Publisher('/pid_output',Float32, queue_size = 10)
		rospy.Subscriber("/vicon/Fetch/Fetch", TransformStamped ,self.current_pose_callback)
		# rospy.Subscriber("/gazebo/model_states", ModelStates ,self.current_pose_callback)
		rospy.Subscriber("/goal", TransformStamped ,self.goal_callback)
		self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		self.pid_orient.clear()
		self.pid_pos.clear()
		# self.current_pose = ModelStates()
		# print("ModelStates():",self.current_pose.pose)
		# self.current_pose = TransformStamped()
		# self.current_pose.transform.rotation.w = 0.866
		# self.current_pose.transform.rotation.z = -0.5
		# self.current_pose.pose.orientation.w = 0.866
		# self.current_pose.pose.orientation.z = -0.5
		self.goal = TransformStamped()
		# print('Start Goal: ',self.goal)
		self.goal_flag = False
		self.delta = 0.1
		self.last_time = 0

		################Flags

		self.orient_flag_initial = True
		self.position_flag = False
		self.orient_flag_final = False

    def quat_to_angle_Transform(self,pose):
        #Returns angle of pose wrt z axis in [0,2pi)
        angle =  2 * np.arccos(pose.transform.rotation.w*np.sign(pose.transform.rotation.z))
        if angle > 3.14159265:
        	angle = angle - 2*np.pi
        # print("Angle = ",angle)
        return angle

    def current_pose_callback(self,data):
        self.current_pose = data
        

    def goal_callback(self,data):
    	self.goal = data
    	print("Goal Recieved: ",self.goal)
    	print("Starting Manoeuvre...........")
    	self.orient_flag_initial = True
    	self.goal_flag = True
    	self.main()

    def move(self,direction):
		print("Moving in direction: ",direction)
		speed = 0.5
		turn = 1
		moveBindings = {
		        'straight':(0.25,0,0,0),
		        'o':(1,0,0,-0.25),
		        'left':(0,0,0,0.25),
		        'right':(0,0,0,-0.25),
		        'u':(1,0,0,1),
		        'reverse':(-0.25,0,0,0),
		        '.':(-1,0,0,1),
		        'm':(-1,0,0,-1),
		        'O':(1,-1,0,0),
		        'I':(1,0,0,0),
		        'J':(0,1,0,0),
		        'L':(0,-1,0,0),
		        'U':(1,1,0,0),
		        '<':(-1,0,0,0),
		        '>':(-1,-1,0,0),
		        'M':(-1,1,0,0),
		        't':(0,0,1,0),
		        'b':(0,0,-1,0),
			}
		if direction in moveBindings.keys():
			x = moveBindings[direction][0]
			y = moveBindings[direction][1]
			z = moveBindings[direction][2]
			th = moveBindings[direction][3]
		else:
			x = 0
			y = 0
			z = 0
			th = 0
			if (key == '\x03'):
			    return
		twist = Twist()
		twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
		self.pub.publish(twist)


    def main(self):
		# print("Current Pose : ",self.current_pose)
		# print("Current Goal : ",self.goal)
		if not self.goal_flag:
			return
		if self.orient_flag_initial:
		    #for real Fetch
		    required_angle = np.arctan2((-self.current_pose.transform.translation.y + self.goal.transform.translation.y),(-self.current_pose.transform.translation.x + self.goal.transform.translation.x))
		    #for Gazebo
		    # required_angle = np.arctan2((-self.current_pose.pose[1].position.y + self.goal.transform.translation.y),(-self.current_pose.pose[1].position.x + self.goal.transform.translation.x))
		    #########################
		    current_angle = self.quat_to_angle_Transform(self.current_pose)
		    # print(required_angle)
		    # print(current_angle)
		    print("Orienting Initial")
		    error = current_angle - required_angle
		    # print("error: ",error)
		    if abs(error) < 0.05:
		    	self.orient_flag_initial = False
		    	self.position_flag = True
		    if error < 0:
		    	self.move('left')
		    else:
		    	self.move('right')
		    # output = self.pid_orient.update(error)
		    # print("PID output for orient_initial", output)

		    ##############Turn based on output##############

		elif self.position_flag:
			print("Orienting Position")
			#for Fetch
			distance = np.sqrt(np.square(self.goal.transform.translation.y - self.current_pose.transform.translation.y) + np.square(self.goal.transform.translation.x - self.current_pose.transform.translation.x))
			#For gazebo
			# distance = np.sqrt(np.square(self.goal.transform.translation.y - self.current_pose.pose[1].position.y) + np.square(self.goal.transform.translation.x - self.current_pose.pose[1].position.x))
			###############################
			if distance < 0.25:
				self.position_flag = False
				self.orient_flag_final = True
			#output = self.pid_pos.update(distance)
			else:
				self.move('straight')
		    ##############Go straight based on output##############
		elif self.orient_flag_final:
			print("Orienting Final")
			required_angle = self.quat_to_angle_Transform(self.goal)
			current_angle = self.quat_to_angle_Transform(self.current_pose)
			error = current_angle - required_angle
			if abs(error) < 0.05:
				self.orient_flag_final = False
				print("Reached Goal")
				return
			#output = self.pid_orient.update(error)
			if error < 0:
				self.move('left')
			else:
				self.move('right')
		    ##############Turn based on output##############

if __name__ == "__main__":
	print("Initializing Node and waiting 3 secs")
	rospy.init_node('master_node')
	pid_node = pid_node()
	time.sleep(3)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pid_node.main()
		rate.sleep()

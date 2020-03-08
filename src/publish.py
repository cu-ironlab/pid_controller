#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, TransformStamped, Transform
import roslaunch
import time

#Talk
def talker():
	
	goal_pub = rospy.Publisher('/goal', TransformStamped, queue_size=3)
	rospy.init_node('publish_test', anonymous=True)

	goal = TransformStamped()
	goal.header.seq = 0
	p = Transform()
	p.translation.x = 0
	p.translation.y = 0
	p.translation.z = 0
	p.rotation.w = 1
	p.rotation.x = 0
	p.rotation.y = 0
	p.rotation.z = 0
	goal.transform = p
	print("Publishing: ",goal)
	goal_pub.publish(goal)

if __name__ == '__main__':
    try:
    	
        talker()
    except rospy.ROSInterruptException:
        pass


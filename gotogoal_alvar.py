#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg import Pose
from math import pow,atan2,sqrt
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
import time
import numpy as np

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/ar_pose_marker_delayed', AlvarMarkers, self.callback)
        self.pose = Pose().position
        self.rate = rospy.Rate(10)

    def wrap_angle(self,val):
        return( ( val + np.pi) % (2.0 * np.pi ) - np.pi )

    #Callback function implementing the pose value received
    def callback(self, data):
	for marker in data.markers:
		if marker.id == 4:
			self.pose = marker.pose.pose.position
			#self.pose.y = -self.pose.y
			#self.pose.x = -self.pose.x
			quaternion = (
    				marker.pose.pose.orientation.x,
    				marker.pose.pose.orientation.y,
    				marker.pose.pose.orientation.z,
    				marker.pose.pose.orientation.w)

        #self.pose.x = round(self.pose.x, 4)
        #self.pose.y = round(self.pose.y, 4)
	
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	self.theta = -self.wrap_angle(yaw + 3.14)#- 1.57)
	
	#print self.theta


    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distancegoal_pose.y 

    def move2goal(self,xpose,ypose,tol):
        goal_pose = Pose().position
        goal_pose.x = xpose
        goal_pose.y = ypose
        distance_tolerance = tol
        vel_msg = Twist()
	
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:
	    #print self.pose.x,self.pose.y, self.pose.z, self.theta
            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = .2 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            #vel_msg.angular.z = 0.5 * self.wrap_angle((atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.theta))
            vel_msg.angular.z = 0.5 * self.wrap_angle((atan2(goal_pose.x - self.pose.x, goal_pose.y - self.pose.y) - self.theta))
            #Publishing our vel_msg
            print self.pose.x, self.pose.y,self.pose.z,vel_msg.angular.z, self.theta
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        #Stopping our robot after the movement is over
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
	time.sleep(3)

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
	time.sleep(2)
	for i in range(20):
		x.move2goal(0.0,-0.8,0.1)
		x.move2goal(-.95,0.0,0.1)
		#x.move2goal(1.0,0.0,0.1)
		#x.move2goal(0.0,0.0,0.1)


    except rospy.ROSInterruptException: pass

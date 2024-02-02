#!/usr/bin/env python3
import rospy
import tf
from std_msgs.msg import String, Header, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import sqrt, cos, sin, pi, atan2
import numpy
import sys

class PID:
    def __init__(self, Kp, Td, Ti, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt
        
    def update_control(self, current_error, reset_prev=False):
        # todo: implement this
        if reset_prev:
            self.prev_error = 0
            self.prev_error_deriv = 0
            self.sum_error = 0
        prev_error = self.prev_error
        self.prev_error = self.curr_error
        self.prev_error_deriv = self.curr_error_deriv
        self.curr_error = current_error
        self.curr_error_deriv = (current_error - prev_error) / self.dt
        self.sum_error = self.sum_error + current_error * self.dt
        self.control = self.Kp * current_error + self.Td * self.curr_error_deriv + self.Ti * self.sum_error
        


    def get_control(self):
        return self.control
        
class WallFollowerHusky:
    def __init__(self):
        rospy.init_node('wall_follower_husky', anonymous=True)

        self.forward_speed = rospy.get_param("~forward_speed")
        self.desired_distance_from_wall = rospy.get_param("~desired_distance_from_wall")
        self.hz = 50 

        self.controller = PID(-1, -1.24, 0.0002, 0.02)
        # todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
        # using geometry_msgs.Twist messages
        self.cmd_pub = rospy.Publisher('/husky_1/cmd_vel', Twist, queue_size=20)

        self.pub = rospy.Publisher('/husky_1/cte', Float32, queue_size=20)
        
        # todo: set up the laser scan subscriber
        # this will set up a callback function that gets executed
        # upon each spinOnce() call, as long as a laser scan
        # message has been published in the meantime by another node
        self.laser_sub = rospy.Subscriber('/husky_1/scan', LaserScan, self.laser_scan_callback)
        

    def laser_scan_callback(self, msg):
        # todo: implement this
        # Populate this command based on the distance to the closest
        # object in laser scan. I.e. compute the cross-track error
        # as mentioned in the PID slides.

        # You can populate the command based on either of the following two methods:
        # (1) using only the distance to the closest wall
        # (2) using the distance to the closest wall and the orientation of the wall
        #
        # If you select option 2, you might want to use cascading PID control. 
        cmd = Twist()
        error = Float32()

        cross_track_error = self.desired_distance_from_wall
        distance = msg.ranges[0]
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < distance:
                distance = msg.ranges[i]
        cross_track_error = cross_track_error - distance
        
        self.controller.update_control(cross_track_error)
        cmd.angular.z = self.controller.get_control()
        cmd.linear.x = self.forward_speed
        error.data = cross_track_error
        self.cmd_pub.publish(cmd)
        self.pub.publish(error)


    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            rate.sleep()

    
if __name__ == '__main__':
    wfh = WallFollowerHusky()
    wfh.run()



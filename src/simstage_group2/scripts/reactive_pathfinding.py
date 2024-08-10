#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ReactiveNavigation:
    def __init__(self):  
        rospy.init_node('reactive_navigation', anonymous=True) 
        self.cmd_vel = Twist()
        self.robot_stopped = False
        self.obstacle_distance = 100.0
        self.laser_msg = None  
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laser_cb, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(5)  # 5 Hz
        self.map = np.zeros((100, 100))  # Initialize a 100x100 map

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if self.laser_msg is None:
            return 
        
        # Check if the laser scan data is valid
        if len(self.laser_msg.ranges) == 0:
            return
        
        # Find the minimum distance to an obstacle
        self.obstacle_distance = min(self.laser_msg.ranges)
        
        # If the obstacle is far away, move forward
        if self.obstacle_distance > 1.0:
            self.cmd_vel.linear.x = 1.0
            self.cmd_vel.angular.z = 0.0
        # If the obstacle is close, turn around
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 1.0

        # Update the map
        self.update_map()

        self.cmd_vel_pub.publish(self.cmd_vel)
        # Reset the obstacle distance to a large value
        self.obstacle_distance = 100.0  

    def update_map(self):
        # Convert laser scan data to a 2D point cloud
        points = self.laser_to_points(self.laser_msg)
        
        # Update the map with the new points
        for point in points:
            x, y = point
            x = int(x)
            y = int(y)
            
            if 0 <= x < self.map.shape[0] and 0 <= y <self.map.shape[1]:
            	self.map[x, y] = 1

    def laser_to_points(self, laser_msg):
        points = []
        for i, range in enumerate(laser_msg.ranges):
            angle = laser_msg.angle_min + i * laser_msg.angle_increment
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            points.append((x, y))
        return points

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ReactiveNavigation()
        controller.run()
    except rospy.ROSInterruptException:
        pass

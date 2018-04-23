#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size = 1)        
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb, queue_size = 1)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        
        self.loop()

    def loop(self):
        #publish at constant frequency of 50Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
	        if self.pose and self.base_waypoints:
                    # Get the closest waypoint
                    closest_waypoint_idx = self.get_closest_waypoint_idx()
                    self.publish_waypoints(closest_waypoint_idx)
	        rate.sleep()

    def get_closest_waypoint_idx(self):
        # Find closest waypoint given current car position
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y

        closest_len = 1e+10 #large number
        closest_idx = 0
        for idx in len(self.waypoints_2d):
            dist = math.sqrt((self.waypoints_2d[idx][0]-x)**2 + (self.waypoints_2d[idx][1]-y)**2)
            if  dist < closest_len:
                closest_len = dist
                closest_idx = idx
        
        # Check if waypoint is ahead or behind current car position
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val>0:
            # Take next waypoint if this one is behind the car
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header

        base_waypoints = self.base_waypoints.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]

        #check if we need to slow down because of obstacles ahead
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= closest_idx+LOOKAHEAD_WPS):
            lane.waypoints = base_waypoints # no need to stop
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_idx) #plan stop here

        self.final_waypoints_pub.publish(lane)


    def decelerate_waypoints(self, base_waypoints,closest_idx):
        temp = []
        for i,wp in enumerate(base_waypoints):

            p = Waypoint()
            p.pose = wp.pose

            stop_idx = max(self.stopline_wp_idx - closest_idx - 2,0) #-2 meters is to stop before center of car
            dist = self.distance(waypoints,i,stop_idx)
            vel = 2*MAX_DECEL*dist #velocity decreases linearly approaching the stop target waypoint
            if vel < 1:
                vel = 0

            p.twist.twist.linear.x = min(vel,wp.twist.twist.linear.x) #keep velocity always under speed limit
            temp.append(p)

        return temp
                

    def pose_cb(self, msg):
        self.cur_pose = msg.pose 

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        self.obstacle_waypoint = msg.data

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

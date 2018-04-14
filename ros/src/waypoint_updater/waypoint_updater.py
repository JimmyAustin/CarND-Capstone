#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from toolbox.profiler import time_function, Timer


import math
from scipy.spatial import KDTree

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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number

pi = 3.142

debug = False

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.target_velocity = rospy.get_param('~velocity', 10)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        rospy.Subscriber('/traffic_waypoint', Int32, self.next_redlight_waypoint_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.next_redlight_waypoint = -1
        self.all_waypoints = []
        self.waypoint_tree = None
        # TODO: Add other member variables you need below

        rospy.spin()

    def current_velocity_cb(self, message):
        self.last_velocity_msg = message
    
    # def pose_cb(self, msg):
    #     timer = Timer()
    #     with Timer:
    #         self.run_pose_cb(msg)
        # rospy.logerr("Pose CB Time: " + str(timer.length))
    
    #@time_function(print_func=rospy.logerr)
    def pose_cb(self, msg):
        print("PoseCB: + " + str(msg))
        next_waypoint_id = self.next_waypoint(msg.pose)
        if next_waypoint_id is None:
            return
        else:
            if False:
                rospy.logerr("Next Waypoint ID: " + str(next_waypoint_id))
        next_waypoints = self.get_next_waypoints(next_waypoint_id)
        velocities_set = False
        if (self.next_redlight_waypoint is not None and self.next_redlight_waypoint != -1):
            current_linear = self.last_velocity_msg.twist.linear.x

            if self.distance(self.all_waypoints, next_waypoint_id, self.next_redlight_waypoint) < 100:
                wp_count = self.count_between_waypoints(next_waypoint_id, self.next_redlight_waypoint)
                step_down = current_linear / wp_count
                #rospy.logerr("Dropping speed from {0} at {1}".format(current_linear, step_down))
                def calculate_velocity_for_step(steps_remaining):
                    deadzone_count = 5
                    if steps_remaining < deadzone_count:
                        return -1
                    deceleration_curve = 0.2
                    return min(deceleration_curve * (steps_remaining - deadzone_count), current_linear)

                    target_velocity = current_linear - (i * step_down)
                    if target_velocity < 1:
                        return -1
                    return target_velocity
                [self.set_waypoint_velocity(wp, calculate_velocity_for_step(wp_count - i)) for i, wp in enumerate(next_waypoints)]
                velocities_set = True
        if velocities_set is False:
            #rospy.logerr("Standard acceleration: {0}".format(self.target_velocity))
            [self.set_waypoint_velocity(wp, self.target_velocity) for wp in next_waypoints]

        lane = Lane(waypoints = next_waypoints)

        self.final_waypoints_pub.publish(lane)
        # TODO: Implement
        pass

    def next_redlight_waypoint_cb(self, msg):
        self.next_redlight_waypoint = msg.data

    def waypoints_cb(self, waypoints):
        points = [(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in waypoints.waypoints]
        self.waypoint_tree = KDTree(points)
        self.all_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        #print("traffic_cb: + " + msg)

        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        #print("obstacle_cb: + " + msg)
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def closest_waypoint(self, car_pose):
        if self.waypoint_tree is None:
            return None
        return self.waypoint_tree.query((car_pose.position.x, car_pose.position.y))[1]

    def next_waypoint(self, car_pose):
        closest = self.closest_waypoint(car_pose)
        if closest is None:
            return None
        waypoint = self.all_waypoints[closest]
        #RemotePdb('127.0.0.1', 4444).set_trace()

        angle = self.angle_offset(car_pose, waypoint)

        if(angle > pi/4):
            closest = closest + 1
            if closest == len(self.all_waypoints):
                closest = 0

        return closest

    def angle_offset(self, car_pose, waypoint):
        x = car_pose.position.x
        y = car_pose.position.y
        theta = car_pose.orientation.z
        
        map_x =  waypoint.pose.pose.position.x
        map_y =  waypoint.pose.pose.position.y
        
        heading = math.atan2((map_y-y),(map_x-x));

        angle = abs(theta-heading);
        
        angle = min(2*pi - angle, angle);

        return angle

    def get_next_waypoints(self, starting, n=LOOKAHEAD_WPS):
        return [self.all_waypoints[i % len(self.all_waypoints)] for i in range(starting, starting+n)]

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def count_between_waypoints(self, waypoint1_id, waypoint2_id):
        if waypoint2_id < waypoint1_id:
            return waypoint2_id + len(self.all_waypoints) - waypoint1_id
        return waypoint2_id - waypoint1_id

def distance_between_poses(pose1, pose2):
    position1 = pose1.position
    position2 = pose2.position
    return math.sqrt((position1.x-position2.x)**2 + (position1.y-position2.y)**2  + (position1.z-position2.z)**2)

def dump_waypoint(waypoint):
    pose = waypoint.pose.pose
    twist = waypoint.twist.twist
    return {
        'pose': {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z
            }
        },
        'twist': {
            'linear': {
                'x': twist.linear.x,
                'y': twist.linear.y,
                'z': twist.linear.z
            },
            'angular': {
                'x': twist.angular.x,
                'y': twist.angular.y,
                'z': twist.angular.z
            }
        }
    }

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

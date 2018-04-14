#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
from scipy.spatial import KDTree
from toolbox.profiler import Profiler, time_function
from multiprocessing import Process, Pipe

STATE_COUNT_THRESHOLD = 2

length = lambda x, y, x2, y2:  math.sqrt((x-x2)**2 + (y-y2)**2)


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None


        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.proper_traffic_lights = None

        self.find_traffic_light_cache = {}
        self.find_next_stoplight_cache = {}
        self.waypoints = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.image_sub = rospy.Subscriber('/image_color', Image, self.image_cb # buff_size=565536,
            )
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_callback, queue_size=1)#self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        points = [(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in waypoints.waypoints]
        self.waypoint_tree = KDTree(points)
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg, *cb_args, **kwargs):
        rospy.logerr(msg.header.seq)
        return
#        print(msg)
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        self.update_traffic_light_state(light_wp, state)

    #@time_function(print_func=rospy.logerr)
    def update_traffic_light_state(self, light_wp, state):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        print("old state: {0}, new_state: {1}, count: {2}".format(self.state, state, self.state_count))
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self):
        return self.find_waypoint_nearest_point((self.pose.position.x, self.pose.position.y))
        pose2.position.x
        closest_point = None
        closest_point_distance = 10001000
        if self.waypoints is None: 
            return None 
        for i, wp in enumerate(self.waypoints):
            distance = distance_between_poses(self.pose, wp.pose.pose)
            if distance < closest_point_distance:
                closest_point_distance = distance
                closest_point = i;
        return closest_point, closest_point_distance;
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """


        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        #Get classification
        #with Profiler():
        classification = self.light_classifier.get_classification(cv_image)
        print("Found classification" + str(classification))
        return classification

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (self.camera_image is None or self.pose is None or self.lights is None):
            return -1, TrafficLight.UNKNOWN

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        stop_point = self.find_next_stop_point(stop_line_positions)
        if stop_point is None:
            return -1, TrafficLight.UNKNOWN

        closest_traffic_light = self.find_traffic_light(stop_point)
        if closest_traffic_light is None:
            return -1, TrafficLight.UNKNOWN
        waypoint_id, distance = self.find_waypoint_nearest_point(stop_point)
        rospy.logerr('Distance to next traffic light: ' + str(distance))

        if distance > 0:
            return waypoint_id, TrafficLight.UNKNOWN
        state = self.get_light_state(closest_traffic_light)
        return waypoint_id, state

    def find_waypoint_nearest_point(self, point):
        if (self.waypoint_tree is None):
            return None
        return self.waypoint_tree.query(point)[1]
        closest_waypoint = None
        closest_waypoint_distance = 1000000000

        for i, waypoint in enumerate(self.waypoints):
            position = waypoint.pose.pose.position
            wp_distance = length(point[0], point[1], position.x, position.y)
            if wp_distance < closest_waypoint_distance:
                closest_waypoint_distance = wp_distance
                closest_waypoint = i

        return closest_waypoint, closest_waypoint_distance

    def find_next_stop_point(self, stop_points):
        closest_waypoint_id = self.get_closest_waypoint()
        if closest_waypoint_id is None:
            return None
        for waypoint in self.waypoints[closest_waypoint_id:] + self.waypoints[:closest_waypoint_id]:
            position = waypoint.pose.pose.position
            for point in stop_points:
                wp_distance = length(point[0], point[1], position.x, position.y)
                if wp_distance < 10:
                    return point
        return None

    def find_traffic_light(self, stop_point):
        closest_light = None
        closest_light_distance = 1000000
        for light in self.lights:
            position = light.pose.pose.position
            light_distance = length(stop_point[0], stop_point[1], position.x, position.y)
            if light_distance < closest_light_distance:
                closest_light = light
                closest_light_distance = light_distance
        return closest_light

def distance_between_poses(pose1, pose2):
    position1 = pose1.position
    position2 = pose2.position
    return math.sqrt((position1.x-position2.x)**2 + (position1.y-position2.y)**2  + (position1.z-position2.z)**2)

if __name__ == '__main__':
    try:
        print("TLDetector")
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

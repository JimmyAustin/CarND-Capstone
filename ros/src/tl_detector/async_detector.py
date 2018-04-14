#!/usr/bin/env python2.7
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
from toolbox.profiler import Profiler
from multiprocessing import Process, Queue, Pipe

STATE_COUNT_THRESHOLD = 2

length = lambda x, y, x2, y2:  math.sqrt((x-x2)**2 + (y-y2)**2)


def f(instruction_conn, result_conn):
    from time import sleep
    bridge = CvBridge()
    light_classifier = TLClassifier()

    print('subprocess started')
    while instruction_conn.poll() is False :
        print('waiting on first')
        sleep(0.5)
    current_image = instruction_conn.recv()
    while True:
        print("Processing image: {0}", len(str(current_image.header.seq)))
        sleep(2)
        print("Finished Processing image: {0}", len(str(current_image.header.seq)))
        current_image = None
        current_best = None
        while instruction_conn.poll():
            if current_image is not None:
                print("Skipping: {0}", len(str(current_image.header.seq)))
            current_image = instruction_conn.recv()
        classification = get_classification(light_classifier, bridge, current_image)
        result_conn.send(classification)

def get_classification(light_classifier, bridge, image):
    cv_image = bridge.imgmsg_to_cv2(image, "rgb8")

    #Get classification
    with Profiler():
        classification = light_classifier.get_classification(cv_image)
    print("Found classification" + str(classification))
    return classification


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

        self.instruction_conn, self.result_conn = Pipe()

        p = Process(target=f, args=(self.result_conn,self.instruction_conn))
        p.start()
        
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        
        self.image_sub = rospy.Subscriber('/image_color', Image, self.image_cb, # buff_size=565536,
            queue_size=1)
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_callback, queue_size=1)#self.image_cb)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose


        self.get_updated_image_classification()
        #self.check_if_new_image()

    def get_updated_image_classification(self):
        latest_state = None
        while self.result_conn.poll():
            if latest_state is not None:
                print("Skipping: {0}", len(str(latest_state)))
            latest_state = self.result_conn.recv()
        if latest_state is not None:
            print("Found latest classification: {0}".format(len(str(latest_state))))
        else:
            print("no new classification")
#        get_classification(light_classifier, bridge, image)

    # def check_if_new_image(self):
    #     print("Check if new image")
    #     current_image = None
    #     while self.result_conn.empty() is False:
    #         if current_image is not None:
    #             print("Skipping {0}".format(current_image.header.seq))
    #         current_image = self.result_conn.get_nowait()
    #     if current_image is not None:
    #         self.image_cb(current_image)

    def waypoints_cb(self, waypoints):
        points = [(wp.pose.pose.position.x, wp.pose.pose.position.y) for wp in waypoints.waypoints]
        self.waypoint_tree = KDTree(points)
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_callback(self, msg):
        print("IMAGE CALLBACK")
        print(msg.header)
        self.instruction_conn.send({'object': msg.header.seq})

    def image_cb(self, msg, *cb_args, **kwargs):
        print(msg.header.seq)
        #self.instruction_conn.send({'object': msg.header.seq})
        self.instruction_conn.send(msg)

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
        return closest_point;
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
        with Profiler():
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
        waypoint_id = self.find_waypoint_nearest_point(stop_point)
        return -1, TrafficLight.UNKNOWN
        state = self.get_light_state(closest_traffic_light)
        return waypoint_id, state


        # if (self.proper_traffic_lights is not None and self.pose is not None):
        #     #RemotePdb('127.0.0.1', 4444).set_trace()
        #     stop_point = self.find_next_stop_point(stop_line_positions)
        #     if stop_point is None:
        #         return -1, TrafficLight.UNKNOWN

        #     #rospy.logerr("Next stop point" + str(length(stop_point[0], stop_point[1], self.pose.position.x, self.pose.position.y)) + ' -' + str(stop_point))
        #     closest_traffic_light = self.find_traffic_light(stop_point)
        #     if closest_traffic_light is None:
        #         return -1, TrafficLight.UNKNOWN
        #     #rospy.logerr("ClosestTraffic Light" + str(closest_traffic_light))
        #     waypoint_id = self.find_waypoint_nearest_point(stop_point)
        #     #rospy.logerr("Waypoint ID" + str(waypoint_id))
        #     return waypoint_id, closest_traffic_light.state

        # #TODO find the closest visible traffic light (if one exists)

        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state
        # return -1, TrafficLight.UNKNOWN

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

        return closest_waypoint

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
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

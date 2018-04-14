from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy
from styx_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped
import cte_calculator

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):

        # self.vehicle_mass    = rospy.get_param('~vehicle_mass')
        # self.fuel_capacity   = rospy.get_param('~fuel_capacity')
        # self.brake_deadband = rospy.get_param('~brake_deadband')
        # self.decel_limit     = rospy.get_param('~decel_limit')
        # self.accel_limit     = rospy.get_param('~accel_limit')
        # self.wheel_radius    = rospy.get_param('~wheel_radius')
        # self.wheel_base      = rospy.get_param('~wheel_base')
        # self.steer_ratio     = rospy.get_param('~steer_ratio')
        # self.max_lat_accel   = rospy.get_param('~max_lat_accel')
        # self.max_steer_angle = rospy.get_param('~max_steer_angle')

        self.vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        self.fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)
        self.decel_limit = rospy.get_param('~decel_limit', -5)
        self.accel_limit = rospy.get_param('~accel_limit', 1.)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        self.wheel_base = rospy.get_param('~wheel_base', 2.8498)
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.final_waypoints = None

        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)

        self.final_wp_sub = rospy.Subscriber('final_waypoints', Lane, self.final_waypoints_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.current_pose_cb, queue_size=1)

        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Minimum throttle value
        mx = 0.5 # Maximum throttle value
        self.throttle_pid = PID(kp, ki, kd, mn, mx)

        self.steering_pid = PID(kp=0.15, ki=0.003, kd=0.1, 
                                mn=-self.max_steer_angle, mx=self.max_steer_angle)


        tau = 0.5 # 1/(2pi*tau) = cutoff frequency
        ts = 0.02 # Sample time
        self.vel_lpf = LowPassFilter(tau, ts)

        self.last_time = rospy.get_time()

        self.dbw_enabled = True
        # TODO: Implement
    

    def current_pose_cb(self, message):
        self.current_pose = message

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def final_waypoints_cb(self, message):
        self.final_waypoints = message.waypoints

    def reset_pids(self):
        self.throttle_pid.reset()
        self.steering_pid.reset()
    
    def ready(self):
        if self.final_waypoints is None:
            return False
        if self.dbw_enabled is None:
            return False
        if self.current_pose is None:
            return False
        if self.last_time is None:
            self.last_time = rospy.get_time()
        return True


    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if self.ready() is False:# or dbw_enabled is False:
            self.reset_pids()
            return 0,0,0

        if not dbw_enabled:
            self.reset_pids()
            return 0., 0., 0.

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        cross_track_error = cte_calculator.get_cross_track_error(self.final_waypoints, self.current_pose)
        if (abs(cross_track_error) > 1):
            linear_vel = linear_vel * 0.95
        rospy.logerr("Cross Track ERror: {0}".format(cross_track_error))
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        steering_shift = self.steering_pid.step(cross_track_error, sample_time) * 0.3
        rospy.logerr("CTE: {2}, YC: {0}, SPID: {1}".format(steering, steering_shift, cross_track_error))
        steering = steering + steering_shift
        
        vel_error = linear_vel - current_vel
        self.last_vel = current_vel


        throttle = self.throttle_pid.step(vel_error, sample_time)

        brake = 0
        if linear_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 400
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return throttle, brake, steering

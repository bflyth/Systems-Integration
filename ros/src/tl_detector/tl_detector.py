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
import numpy as np

STATE_COUNT_THRESHOLD = 2


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoints_2d = None
        self.waypoints_tree = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''/vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights',
                                TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_sim = self.config["is_sim"]
        print("Is Sim %d" % self.is_sim)
        
        self.upcoming_red_light_pub = rospy.Publisher(
            '/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.light_classifier = TLClassifier(self.is_sim)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        #self.image_list = np.array

        #rospy.spin()
        self.ros_spin()

    def ros_spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            '''Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.pose is not None and self.waypoints is not None and self.camera_image is not None:
                light_wp, state = self.process_traffic_lights()
                #print("Light waypoint Index: ",light_wp, "Traffic Light: ",TrafficLight.RED, state)
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    self.last_wp = light_wp
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                    #print(light_wp,self.state_count)
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1
        rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    #Extracts the x and y coordinates of a waypoint
    def waypoint_xy(self, waypoint):
        position = waypoint.pose.pose.position
        return [position.x, position.y]

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [self.waypoint_xy(waypoint) for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        #Comment out to test for lag
        light_wp, state = self.process_traffic_lights()

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #Use KDTree to search through waypoints
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """
        Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        #Get classification
        return self.light_classifier.get_classification(cv_image) 
        #return light.state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        closest_light = None
        light_wp_idx = None
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            print(car_wp_idx)
        
        diff = len(self.waypoints.waypoints)
        #print(self.pose.pose.position.x,self.pose.pose.position.y, car_wp_idx)
   
        for i, light in enumerate(self.lights):
            #Get stop line waypoint index
            line = stop_line_positions[i]
            #print(line[0], line[1])
            temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
            # Find Closest stop line waypoint index
            d = temp_wp_idx - car_wp_idx
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                light_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            #print("Light waypoint Index: ",light_wp_idx, "Traffic Light: ",TrafficLight.RED, state)
            return light_wp_idx, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

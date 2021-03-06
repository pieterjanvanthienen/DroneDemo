#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


### PJ 2e Semester ###

from sensor_msgs.msg import Image
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import tf2_ros
import rospy

from fabulous.color import highlight_red


class Perception(object):
    '''
    Read out position of the drone, pose of the drone and position of the
    obstacles
    '''

    def __init__(self):
        """
        Initialization of Perception object.
        """
        self.pose_vive = TwistStamped()
        self.pose_bebop = Twist()
        self.twist_bebop = Twist()
        self.tf_t_in_w_prev = TransformStamped()
        self.init = True
        self.vive_calibrating = False
        self.ids  = []

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Name of topic can change depending on name used in the code for
        # reading out the vive.
        rospy.Subscriber('/bebop/odom', Odometry, self.get_bebop_data)
        rospy.Subscriber(
            'vive_localization/calibrate', Empty, self.vive_calibrate)

        # Uncomment the following if using vive in combination with drone camera
        # rospy.Subscriber(            '/bebop/image_raw', Image, self.read_image)
        

        # Create aruco markers for reading out from drone camera
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        fig = plt.figure()
        nx = 4
        ny = 3
        for i in range(1, nx*ny+1):
            ax = fig.add_subplot(ny,nx, i)
            img = aruco.drawMarker(aruco_dict,i, 700)
            plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
            ax.axis("off")
        self.bridge = CvBridge()
        plt.savefig("markers.pdf")
       


    def read_image(self, msg):
        """
        Updates the ids of markers the camera of the drone sees.
        Only used with vive. Otherwise it is performed in code
        """
        try:
            # Convert your ROS Image message to OpenCV2
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            self.ids=ids


    def get_bebop_data(self, data):
        """
        Updates pose and twist data by using measurements on the bebop itself.
        """
        self.pose_bebop = data.pose.pose
        self.twist_bebop = data.twist.twist

    def measurement_check(self):
        '''Monitor function: checks measurement.
        If the measurement equals the vive frame origin, this means that
        vibrations cause a false measurement.

        Should be moved to monitor later.
        '''
        tf_v_in_w = self.get_transform("vive", "world")
        tf_t_in_w = self.get_transform("tracker", "world")

        meas_distance = np.linalg.norm(
            np.array([tf_t_in_w.transform.translation.x,
                      tf_t_in_w.transform.translation.y,
                      tf_t_in_w.transform.translation.z])
            - np.array([self.tf_t_in_w_prev.transform.translation.x,
                        self.tf_t_in_w_prev.transform.translation.y,
                        self.tf_t_in_w_prev.transform.translation.z]))
        self.tf_t_in_w_prev = tf_t_in_w

        measurement_valid = not (
            (tf_v_in_w.transform == tf_t_in_w.transform) or
            (meas_distance > 0.25))
        if not measurement_valid:
            if not (self.init or self.vive_calibrating):
                print highlight_red(' Warning: invalid measurement!')
            else:
                self.init = False
                self.vive_calibrating = False

        return measurement_valid

    def vive_calibrate(self, *_):
        '''When Vive is calibrating, an invalid measurement is perceived once,
        but then we don't want to trigger emergency.
        '''
        self.vive_calibrating = True

    def get_transform(self, _from, _to):
        '''
        '''
        tf_f_in_t = self.tfBuffer.lookup_transform(
            _to, _from, rospy.Time(0), rospy.Duration(0.1))

        return tf_f_in_t


if __name__ == '__main__':
    perception = Perception()

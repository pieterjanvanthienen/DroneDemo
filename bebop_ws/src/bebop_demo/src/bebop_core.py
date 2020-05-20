#!/usr/bin/env python

from geometry_msgs.msg import (
    Twist, PoseStamped, Point, PointStamped, TwistStamped)
from vive_localization.msg import PoseMeas
from std_msgs.msg import String, Empty, Bool
from bebop_msgs.msg import (Ardrone3PilotingStateFlyingStateChanged,
                            CommonCommonStateBatteryStateChanged)

from bebop_demo.srv import GetPoseEst, GetPoseEstResponse, GetPoseEstRequest

import numpy as np
import math as m
import rospy
import tf2_ros
import tf2_geometry_msgs as tf2_geom
import random

from fabulous.color import (highlight_red, highlight_green, highlight_blue,
                            highlight_yellow, cyan, green, red, blue, highlight_magenta,magenta)

from perception import *
from world_model import *
from kalman import *


class Demo(object):
    '''
    Acts as hub for the Perception and WorldModel.
    '''

    def __init__(self):
        '''
        Initialization of Demo object.
        '''

        rospy.init_node('bebop_demo')

        self.state = "initialization"
        self.state_sequence = []
        self.change_state = False
        self.new_task = False
        self.state_finish = False
        self.omg_standby = False
        self.airborne = False
        self.trackpad_held = False
        self.menu_button_held = False
        self.gripButtonPressed_held = False
        self.obstacle_1 = False
        self.obstacle_2 = False
        self.obstacle_3 = False

        self.task_dict = {
            "standby": [],
            "invalid measurement": ["emergency"],
            "take-off": ["take-off"],
            "land": ["land"],
            "point to point": ["omg standby", "omg fly"],
            "place cyl obstacles": ["land",
                                    "place cyl obstacles",
                                    "configure motionplanner",
                                    "take-off"],
            "place slalom obstacles": ["land",
                                       "place slalom obstacles",
                                       "configure motionplanner",
                                       "take-off"],
            "place plate obstacles": ["land",
                                      "place plate obstacles",
                                      "configure motionplanner",
                                      "take-off"],
            "place hex obstacles": ["land",
                                    "place hex obstacles",
                                    "configure motionplanner",
                                    "take-off"],
            "place window obstacles": ["land",
                                       "place window obstacles",
                                       "configure motionplanner",
                                       "take-off"],
            "track drawn trajectory slow": ["land", "draw path slow",
                                            "take-off", "fly to start",
                                            "follow path"],
            "track drawn trajectory fast": ["land", "draw path fast",
                                            "take-off", "fly to start",
                                            "follow path"],
            "drag drone": ["drag drone"],
            "undamped spring": ["undamped spring", "reset PID"],
            "viscous fluid": ["viscous fluid", "reset PID"],
            "gamepad flying": ["gamepad flying"],
            "dodge dynamic obstacle": ["dodge dyn obst"],
            "summon":["summon"],
            "point to point A": ["omg standby A", "omg fly A"],
            "inspection tour":["standby tour","omg fly inspection"],
            "stoplight procedure":["stoplight","stoplight","stoplight checking","stoplight","stoplight","stoplight","stoplight"],
            "guide":["omg standby", "omg fly", "check target", "omg guide"],
            "take picture of":["omg standby", "omg fly", "check target", "take picture of","save picture"]}

        self.pose_pub = rospy.Publisher(
            'world_model/yhat', PointStamped, queue_size=1)
        self.pose_r_pub = rospy.Publisher(
            'world_model/yhat_r', PointStamped, queue_size=1)
        self.fsm_state = rospy.Publisher(
            'fsm/state', String, queue_size=1)
        # Initialization finishes when pushing controller buttons
        self.fsm_state.publish("initialization")
        

        # self.request = rospy.Publisher('controller/request',int,queue_size=1)
        # self.request.pusblish(0)

        rospy.Subscriber(
            'vive_localization/ready', Empty, self.vive_ready)
        rospy.Subscriber(
            'vive_localization/pose', PoseMeas, self.new_measurement)
        rospy.Subscriber(
            'fsm/task', String, self.switch_task)
        rospy.Subscriber(
            'ctrl_keypress/rmenu_button', Bool, self.take_off_land)
        rospy.Subscriber(
            'controller/state_finish', Empty, self.ctrl_state_finish)
        rospy.Subscriber(
            'ctrl_keypress/rtrackpad', Bool, self.switch_state)
        rospy.Subscriber(
            'ctrl_keypress/rtrigger', Bool, self.r_trigger)
        rospy.Subscriber(
            'ctrl_keypress/gripButtonPressed', Bool, self.switch_p2p)

        rospy.Subscriber(
            '/bebop/states/ardrone3/PilotingState/FlyingStateChanged',
            Ardrone3PilotingStateFlyingStateChanged, self.flying_state)
        rospy.Subscriber(
            '/bebop/states/common/CommonState/BatteryStateChanged',
            CommonCommonStateBatteryStateChanged, self.battery_state)

        """extra init when working from home without vive

        """
        cv2.namedWindow("preview")
        

        # Create aruco markers for reading out from drone camera
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        fig = plt.figure()
        nx = 10
        ny = 10
        for i in range(1, nx*ny+1):
            ax = fig.add_subplot(ny,nx, i)
            img = aruco.drawMarker(aruco_dict,i, 700)
            plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
            ax.axis("off")
        self.bridge = CvBridge()
        plt.savefig("markers.pdf")


        self._get_pose_service = None
        self.ids=[]
        rospy.Subscriber(
            '/bebop/image_raw', Image, self.read_image)
        self.task_str=""

    def start(self):
        '''
        Starts running of bebop_demo node. Runs along the state sequence, sends
        out the current state and returns to the standby state when task is
        completed.
        '''
        print green('----    Bebop core running     ----')

        while not rospy.is_shutdown():

            if self.task_str == "home inspection":
                self.home_inspection()
            elif self.task_str == "home inventory":
                self.inventory()
            elif self.task_str == "home count":
                self.count()
            elif self.task_str == "home picture":
                self.picture()
            elif self.task_str == "home guide":
                self.guide()
            elif self.task_str == "home events":
                self.events()
            elif self.task_str == "home smart inspection":
                self.home_inspection_smart()


            if self.new_task:
                self.new_task = False
                self.change_state = False
                self.state_finish = False

                # Run over sequence of states corresponding to current task.
                for state in self.state_sequence:
                    self.state = state
                    print cyan(' Bebop_core state changed to: ', self.state)
                    self.fsm_state.publish(state)

                    # Omg tools should return to its own standby status unless
                    # the controller trackpad has been pressed.
                    if self.state in {"omg standby", "omg fly", "omg fly A", "omg standby A","omg fly inspection"}:
                        self.omg_standby = True
                    else:
                        self.omg_standby = False

                    if self.state == "land":
                        self.change_state = True

                    task_final_state = (self.state == self.state_sequence[-1])
                    # Check if previous state is finished and if allowed to
                    # switch state based on controller input.
                    while not ((self.state_finish and (
                                self.change_state or task_final_state)) or
                               self.new_task or rospy.is_shutdown()):
                        # Remaining in state. Allow state action to continue.
                        if self.state not in {"standby", "initialization"}:
                            self.camera_actions()

                        rospy.sleep(0.1)

                    self.change_state = False
                    self.state_finish = False

                    leave_omg = (
                        self.state == "omg standby" and not self.omg_standby)
                    # User forces leaving omg with trackpad or other new task
                    # received --> leave the for loop for the current task.
                    if (leave_omg or self.new_task):
                        # print cyan('---- Broke for loop ----')
                        break

                # Make sure that omg-tools task is repeated until force quit.
                if self.omg_standby:
                    self.new_task = True
                # Only publish standby state when task is finished.
                # Except for repetitive tasks (back to first state in task).
                if not self.new_task:
                    self.state = "standby"
                    self.fsm_state.publish("standby")
                    print cyan(' Bebop_core state changed to: ', "standby")

            rospy.sleep(0.1)
            
    

    def read_image(self, msg):
        """
        Updates the ids of markers the camera of the drone sees.
        """
        try:
            # Convert your ROS Image message to OpenCV2
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.frame = image
        except CvBridgeError, e:
            print(e)
        else:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            if ids is None:
                self.ids = []
            else:
                self.ids = ids
            


    def camera_actions(self):
        ''' In this function the camera actions are stated based  on scenarios executed in the dronelab with the vive.
        Some are tested, some are not. Highly undependable
        '''
        if self.state == "standby tour":
            if [1] in self.ids:
                print("inspection completed")
                self.change_state = True
                self.omg_standby = False
                self.ids = []
            elif [2] in self.ids:
                print("inspection completed: error detected")
                self.change_state = True
                self.omg_standby = False
                self.ids = []
            else:
                print("inspection failed")
        if self.state == "stoplight checking":
            if [1] in self.ids:
                print("light is green! let's go")
                self.change_state = True
                # self.omg_standby = False
                self.ids = []
            elif [2] in self.ids:
                print("light is red. keep hovering")
            else:
                print("no stoplight detected")

        if self.state == "check target":
            if self.ids != []:
                if [1] in self.ids:
                    # self.request.pusblish(1)
                    print("fly to target 1")
                elif [1] in self.ids:
                    # self.request.pusblish(1)
                    print("fly to target 2")
                elif [1] in self.ids:
                    # self.request.pusblish(1)
                    print("fly to target 3")
                elif [1] in self.ids:
                    # self.request.pusblish(1)
                    print("fly to target 4")
                elif [1] in self.ids:
                    # self.request.pusblish(1)
                    print("fly to target 5")
                else:
                    print("not a valid target")
            else:
                print("no request")




    def vive_ready(self, *_):
        '''
        Provides get_pose service when vive is calibrated.
        '''
        self._get_pose_service = rospy.Service(
            "/world_model/get_pose", GetPoseEst, self.get_kalman_pos_est)

    def get_kalman_pos_est(self, req_vel):
        '''
        Receives a time-stamped twist and returns an estimate of the position
        Argument:
            - req_vel = TwistStamped
        '''
        # Don't do prediction and transformation calculations if the
        # measurement is invalid.
        if (not self.measurement_valid) and (
             not self.state_sequence == ["emergency"]):
            req_vel.input_cmd.twist = Twist()
            inv_meas_task = String(data="invalid measurement")
            self.switch_task(inv_meas_task)

        self.kalman.input_cmd_list.append(req_vel.input_cmd)
        self.kalman.latest_input_cmd = req_vel.input_cmd

        self.wm.yhat_r, self.wm.vhat_r = self.kalman.kalman_pos_predict(
                                self.kalman.latest_input_cmd, self.wm.yhat_r)

        # Transform the rotated yhat and vhat to world frame.
        self.wm.yhat = self.transform_point(
            self.wm.yhat_r, "world_rot", "world")
        self.wm.vhat = self.transform_point(
            self.wm.vhat_r, "world_rot", "world")
        self.wm.yaw = self.pc.yaw

        self.pose_r_pub.publish(self.wm.yhat_r)
        self.pose_pub.publish(self.wm.yhat)

        return GetPoseEstResponse(
            self.wm.yhat, self.wm.vhat, self.wm.yaw, self.measurement_valid)

    def new_measurement(self, data):
        '''Processes incoming measurement from Vive localization.
        data:
            meas_world: PoseStamped
            yaw: float32
        '''
        self.pc.pose_vive = data.meas_world
        self.pc.yaw = data.yaw
        self.measurement_valid = self.pc.measurement_check()

        self.ids= self.pc.ids

        measurement = self.kalman.transform_pose(
                                self.pc.pose_vive, "world", "world_rot")
        if self.measurement_valid:
            if self.kalman.init:
                self.wm.yhat_r_t0.header = measurement.header
                zero_input_cmd = TwistStamped()
                zero_input_cmd.header = measurement.header
                self.kalman.input_cmd_list = [zero_input_cmd]
                self.kalman.init = False

            # Apply correction step.
            self.wm.yhat_r, self.wm.yhat_r_t0 = self.kalman.kalman_pos_correct(
                                                measurement, self.wm.yhat_r_t0)
            self.wm.yhat = self.transform_point(
                self.wm.yhat_r, "world_rot", "world")
            self.pose_pub.publish(self.wm.yhat)

####################
# Task functions #
####################

    def switch_task(self, task):
        '''Reads out the task topic and switches to the desired task.
        '''
        if task.data in ["home inspection","home count","home inventory", "home guide", "home picture","home events", "home smart inspection"]:
            self.task_str =task.data
        else:    
            if task.data not in self.task_dict:
                print highlight_red(
                        ' Not a valid task, drone will remain in standby state.')
    
            self.state_sequence = self.task_dict.get(task.data, [])
            self.new_task = True
            print cyan(' Bebop_core received a new task: ', task.data)

    def take_off_land(self, pressed):
        '''Check if menu button is pressed and switch to take-off or land
        sequence depending on last task that was executed.
        '''
        if pressed.data and not self.menu_button_held:
            if self.airborne:
                self.state_sequence = self.task_dict.get("land", [])
            else:
                self.state_sequence = self.task_dict.get("take-off", [])
            self.new_task = True
            print cyan(
                ' Bebop_core received a new task: ',
                self.state_sequence[0])
            self.menu_button_held = True
        elif not pressed.data and self.menu_button_held:
            self.menu_button_held = False

    def switch_p2p(self, pressed):
        ''' Function to switch easily to p2p
        '''

        if pressed.data and not self.gripButtonPressed_held:
            self.state_sequence = self.task_dict.get("point to point", [])
            self.new_task = True
            print cyan(
                ' Bebop_core received a new task: ',
                self.state_sequence[0])
            self.gripButtonPressed_held = True
        elif not pressed.data and self.gripButtonPressed_held:
            self.gripButtonPressed_held = False

####################
# Home Functions   #
####################


    def home_inspection(self):
        ''' this scenario lets the drone inspect several objects if failing or functional
        '''
        inspection_list = [1,3,5]
        print highlight_magenta('            INSPECTION STARTED                   ')
        print magenta('inspecting machines'+str(inspection_list))
        report=[0,0,0,0,0,0]
        inspection_list.append('home')
        print magenta(' fly to machine '+str(inspection_list[0]))
        while len(inspection_list)> 1:
            self.events()
            if [21] in self.ids and [22] in self.ids and 1 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 1 ok   <<<   ")
                        inspection_list.remove(1)
                        report[1]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 1 not ok   <<<   ")
                        inspection_list.remove(1)
                        report[1]=2
                    else:
                        print magenta("   >>>    Machine 1 inconclusive   <<<   ")
                        inspection_list.remove(1)
                        report[1]=3
                    print magenta(' fly to machine '+ str(inspection_list[0]))
                else:
                    print magenta("other marker or not enclosed")
            if [23] in self.ids and [24] in self.ids and 2 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 2 ok   <<<   ")
                        inspection_list.remove(2)
                        report[2]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 2 not ok   <<<   ")
                        inspection_list.remove(2)
                        report[2]=2
                    else:
                        print magenta("   >>>   Machine 2 inconclusive    <<<   ")
                        inspection_list.remove(2)
                        report[2]=3
                    print magenta(' fly to machine '+ str(inspection_list[0]))

                else:
                    print magenta("other marker or not enclosed")
            if [25] in self.ids and [26] in self.ids and 3 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 3 ok    <<<   ")
                        inspection_list.remove(3)
                        report[3]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 3 not ok    <<<   ")
                        inspection_list.remove(3)
                        report[3]=2
                    else:
                        print magenta("   >>>   Machine 3 inconclusive   <<<   ")
                        inspection_list.remove(3)
                        report[3]=3
                    print magenta(' fly to machine '+ str(inspection_list[0]))
                else:
                    print magenta("other marker or not enclosed")
            if [27] in self.ids and [28] in self.ids and 4 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 4 ok   <<<   ")
                        inspection_list.remove(4)   
                        report[4]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 4 not ok   <<<   ")
                        inspection_list.remove(4)
                        report[4]=2
                    else:
                        print magenta("   >>>   Machine 4 inconclusive   <<<   ")
                        inspection_list.remove(4)
                        report[4]=3
                    print magenta(' fly to machine '+ str(inspection_list[0]))
                else:
                    print magenta("other marker or not enclosed")
            if [29] in self.ids and [30] in self.ids and 5 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 5 ok   <<<   ")
                        inspection_list.remove(5)
                        report[5]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 5 not ok   <<<   ")
                        inspection_list.remove(5)
                        report[5]=2
                    else:
                        print magenta("   >>>   Machine 5 inconclusive   <<<   ")
                        inspection_list.remove(5)
                        report[5]=3
                    print magenta(' fly to machine '+ str(inspection_list[0]))
                else:
                    print magenta("other marker or not enclosed")
                
            rospy.sleep(0.5)
        print highlight_magenta('                    REPORT                       ')
        for i in range(len(report)):
            if report[i]==1:
                print green('            MACHINE '+str(i)+ ' OK')
            elif report[i]==2:
                print red('            MACHINE '+str(i)+ ' NOT OK')
            elif report[i]==3:
                print red('            MACHINE '+str(i)+ ' INCONCLUSIVE')
        print highlight_magenta('            INSPECTION COMPLETED                 ')
        self.task_str=""




    def home_inspection_smart(self):
        ''' this scenario lets the drone inspect several objects if failing or functional
        However, each item influences the objects that will have to be inspected later on.
        '''

        inspection_list = {1}
        dependency_dict = {
        "M1":{2,3},
        "M2":{5},
        "M3":{},
        "M4":{},
        "M5":{},
        "M6":{},
        "M7":{},
        "M8":{},
        "M9":{},
        }
        print highlight_magenta('            INSPECTION STARTED                   ')
        print magenta('inspecting machines'+str(tuple(inspection_list)))
        report=[0,0,0,0,0,0]
        print magenta(' fly to machine '+str(random.choice(tuple(inspection_list))))
        while len(inspection_list)> 0:
            self.events()
            if [21] in self.ids and [22] in self.ids and 1 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 1 ok   <<<   ")
                        inspection_list.discard(1)
                        report[1]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 1 not ok   <<<   ")
                        inspection_list.discard(1)
                        inspection_list = inspection_list.union(dependency_dict["M1"])
                        report[1]=2
                    else:
                        print magenta("   >>>    Machine 1 inconclusive   <<<   ")
                        inspection_list.discard(1)
                        inspection_list = inspection_list.union(dependency_dict["M1"])

                        report[1]=3
                    if len(inspection_list)>0:
                        print magenta(' machines to inspect' + str(tuple(inspection_list)))
                        print magenta(' fly to machine '+ str(random.choice(tuple(inspection_list))))
                    else:
                        print magenta(' fly home')
                else:
                    print magenta("other marker or not enclosed")

            if [23] in self.ids and [24] in self.ids and 2 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 2 ok   <<<   ")
                        inspection_list.discard(2)
                        report[2]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 2 not ok   <<<   ")
                        inspection_list.discard(2)
                        inspection_list = inspection_list.union(dependency_dict["M2"])
                        report[2]=2
                    else:
                        print magenta("   >>>    Machine 2 inconclusive   <<<   ")
                        inspection_list.discard(2)
                        inspection_list = inspection_list.union(dependency_dict["M2"])
                        report[2]=3
                    if len(inspection_list)>0:
                        print magenta(' machines to inspect' + str(tuple(inspection_list)))
                        print magenta(' fly to machine '+ str(random.choice(tuple(inspection_list))))
                    else:
                        print magenta(' fly home')

            if [25] in self.ids and [26] in self.ids and 3 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 3 ok   <<<   ")
                        inspection_list.discard(3)
                        report[3]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 3 not ok   <<<   ")
                        inspection_list.discard(3)
                        inspection_list = inspection_list.union(dependency_dict["M3"])
                        report[3]=2
                    else:
                        print magenta("   >>>    Machine 3 inconclusive   <<<   ")
                        inspection_list.discard(3)
                        inspection_list = inspection_list.union(dependency_dict["M3"])
                        report[3]=3
                    if len(inspection_list)>0:
                        print magenta(' machines to inspect' + str(tuple(inspection_list)))
                        print magenta(' fly to machine '+ str(random.choice(tuple(inspection_list))))
                    else:
                        print magenta(' fly home')
            if [27] in self.ids and [28] in self.ids and 4 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 4 ok   <<<   ")
                        inspection_list.discard(4)
                        report[4]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 4 not ok   <<<   ")
                        inspection_list.discard(4)
                        inspection_list = inspection_list.union(dependency_dict["M4"])
                        report[4]=2
                    else:
                        print magenta("   >>>    Machine 4 inconclusive   <<<   ")
                        inspection_list.discard(4)
                        inspection_list = inspection_list.union(dependency_dict["M4"])
                        report[4]=3
                    if len(inspection_list)>0:
                        print magenta(' machines to inspect' + str(tuple(inspection_list)))
                        print magenta(' fly to machine '+ str(random.choice(tuple(inspection_list))))
                    else:
                        print magenta(' fly home')
            if [29] in self.ids and [30] in self.ids and 5 in inspection_list:
                if len(self.ids) == 3:
                    if [1] in self.ids:
                        print magenta("   >>>   Machine 5 ok   <<<   ")
                        inspection_list.discard(5)
                        report[5]=1
                    elif [2] in self.ids:
                        print magenta("   >>>   Machine 5 not ok   <<<   ")
                        inspection_list.discard(5)
                        inspection_list = inspection_list.union(dependency_dict["M5"])
                        report[5]=2
                    else:
                        print magenta("   >>>    Machine 5 inconclusive   <<<   ")
                        inspection_list.discard(5)
                        inspection_list = inspection_list.union(dependency_dict["M5"])
                        report[5]=3
                    if len(inspection_list)>0:
                        print magenta(' machines to inspect' + str(tuple(inspection_list)))
                        print magenta(' fly to machine '+ str(random.choice(tuple(inspection_list))))
                    else:
                        print magenta(' fly home')
                
            rospy.sleep(0.5)
        print highlight_magenta('                    REPORT                       ')
        for i in range(len(report)):
            if report[i]==1:
                print green('            MACHINE '+str(i)+ ' OK')
            elif report[i]==2:
                print red('            MACHINE '+str(i)+ ' NOT OK')
            elif report[i]==3:
                print red('            MACHINE '+str(i)+ ' INCONCLUSIVE')
        print highlight_magenta('            INSPECTION COMPLETED                 ')
        self.task_str=""


    def count(self):
        ''' This scenario lets the drone count all the objects on a predefined trajectory
        '''
        print highlight_magenta('              COUNTING STARTED                   ')
        print magenta('fly to start point!')
        counting_list=[[50],[51],[52],[53],[54],[55],[56],[57],[58],[59],[60],[61],[62],[63],[64],[65],[66],[67],[68],[69],[70],[71],[72],[73],[74],[75],[76],[77],[78],[79]]
        red_c = 0
        blue_c = 0
        green_c = 0
        counted_list = []
        while [11] not in self.ids:
            rospy.sleep(0.5)
        print magenta('starting to count')  
        while [12] not in self.ids:
            for i in self.ids:
                if i in counting_list:
                    counting_list.remove(i)
                    counted_list.append(i)
                    print magenta('   >>>   found one   <<<   ')
            rospy.sleep(0.5)
        for i in counted_list:
            if 50 <= i[0] <= 59:
                red_c += 1
            elif 60 <= i[0] <= 69:
                blue_c += 1
            elif 70 <= i[0] <= 79:
                green_c += 1
        print highlight_magenta('                    REPORT                       ')
        print red('         red:'+str(red_c))
        print blue('         blue:'+str(blue_c))
        print green('         green:'+str(green_c))
        print highlight_magenta('              COUNTING COMPLETED                 ')
        self.task_str=""

    def inventory(self):
        ''' In this scenario an inventory is made of several objects
        '''
        inventory_list = [1,2,3,4,5]
        print highlight_magenta('            INVENTORY STARTED                   ')
        print magenta('making inventory of items:'+str(inventory_list))
        report=["","","","","","",""]
        inventory_list.append('home')

        while len(inventory_list)> 1:
            if [21] in self.ids and [22] in self.ids and 1 in inventory_list:
                print magenta("   >>>   target 1   <<<   ")
                report[1] = "     "+str(np.count_nonzero(self.ids==[31]))+" of element 1"
                inventory_list.remove(1)
                print magenta(' fly to machine '+ str(inventory_list[0]))

            if [23] in self.ids and [24] in self.ids and 2 in inventory_list:
                print magenta("   >>>   target 2   <<<   ")
                report[2] = "     "+str(np.count_nonzero(self.ids==[32]))+" of element 2"
                inventory_list.remove(2)
                print magenta(' fly to machine '+ str(inventory_list[0]))
            if [25] in self.ids and [26] in self.ids and 3 in inventory_list:
                print magenta("   >>>   target 3   <<<   ")
                report[3] = "     "+str(np.count_nonzero(self.ids==[33]))+" of element 3"
                inventory_list.remove(3)
                print magenta(' fly to machine '+ str(inventory_list[0]))
            if [27] in self.ids and [28] in self.ids and 4 in inventory_list:
                print magenta("   >>>   target 4   <<<   ")
                report[4] = "     "+str(np.count_nonzero(self.ids==[34]))+" of element 4"
                inventory_list.remove(4)
                print magenta(' fly to machine '+ str(inventory_list[0]))
            if [29] in self.ids and [30] in self.ids and 5 in inventory_list:
                print magenta("   >>>   target 5   <<<   ")
                report[5] = "     "+str(np.count_nonzero(self.ids==[35]))+" of element 5"
                inventory_list.remove(5)
                print magenta(' fly to machine '+ str(inventory_list[0]))
            rospy.sleep(0.5)
        print highlight_magenta('                    REPORT                       ')
        for i in range(len(report)):
            if report[i] != "":
                print magenta(report[i])
        self.task_str=""
        print highlight_magenta('            INVENTORY COMPLETED                 ')


    def guide(self):
        ''' In this scenario a person is guided to a certain object
        '''
        print highlight_magenta('            GUIDE STARTED                   ')
        print magenta("flying to start point")
        target = 0
        while target == 0:
            view_list = self.ids
            if [21] in view_list and [22] in view_list and len(view_list) == 3:
                view_list = view_list[view_list != [21]]
                view_list = view_list[view_list != [22]]
                view_list = view_list[0]
                target = view_list-35
                print magenta("start point found")
                print magenta("   >>>   fly to target " + str(target) + "   <<<   ")
            rospy.sleep(0.5)
        if target == 1:
            tag_left = [23]
            tag_right = [24]
        elif target == 2:
            tag_left = [25]
            tag_right = [26]
        elif target == 3:
            tag_left = [27]
            tag_right = [28]
        elif target == 4:
            tag_left = [29]
            tag_right = [30]
        while target != 0:
            if tag_left in self.ids and tag_right in self.ids:
                print magenta("target "+ str(target)+ " found")
                target = 0
            rospy.sleep(0.5)
        self.task_str = ""
        print highlight_magenta('            GUIDE COMPLETED                 ')


    def picture(self):
        '''In this scenario the drone captures a picture of a requested object
        '''
        print highlight_magenta('            PICTURE STARTED                   ')
        print magenta("flying to start point")
        target = 0
        while target == 0:
            view_list = self.ids
            if [21] in view_list and [22] in view_list and len(view_list) == 3:
                view_list = view_list[view_list != [21]]
                view_list = view_list[view_list != [22]]
                view_list = view_list[0]
                target = view_list-35
                print magenta("start point found")
                print magenta("   >>>   fly to target " + str(target) + "   <<<   ")
            rospy.sleep(0.5)
        if target == 1:
            tag_left = [23]
            tag_right = [24]
        elif target == 2:
            tag_left = [25]
            tag_right = [26]
        elif target == 3:
            tag_left = [27]
            tag_right = [28]
        elif target == 4:
            tag_left = [29]
            tag_right = [30]
        while target != 0:
            if tag_left in self.ids and tag_right in self.ids: # voor een goeie foto
                print magenta("target "+ str(target)+ " found")
                target = 0
                cv2.imshow("preview", self.frame)
                cv2.waitKey()
            rospy.sleep(0.5)
        self.task_str = ""
        print highlight_magenta('            PICTURE COMPLETED                 ')

    def events(self):
        '''This function enables the actions on obstacles and or stoplight
        '''
        if [15] in self.ids and not self.obstacle_1:
                print magenta("Watch out! Unknown obstacle ahead!")
                self.obstacle_1 = True
        elif self.obstacle_1 and [15] not in self.ids:
            self.obstacle_1 = False
        if [18] in self.ids:
            print magenta("Watch out! Dynamic obstacle ahead! Please land and wait for 10 seconds")
            rospy.sleep(10)
            print magenta("You are allowed to start again")

        while [14] in self.ids and [13] not in self.ids:
            print highlight_red('The light is red! You can not pass.')
            rospy.sleep(2)
        while [13] in self.ids:
            print highlight_green('The light is green! you can proceed.')
            rospy.sleep(2)




####################
# Helper functions #
####################

    def switch_state(self, trackpad_pressed):
        '''When controller trackpad is pressed changes change_state variable
        to true to allow fsm to switch states in state sequence.
        '''
        if (trackpad_pressed.data and not self.trackpad_held) and (
                self.state not in {"standby", "initialization"}):
            self.trackpad_held = True
            if self.state == "omg standby":
                self.omg_standby = False
                self.new_task = False          
            self.change_state = True
            print highlight_blue(' Switching to next state ')

        elif not trackpad_pressed.data and self.trackpad_held:
            self.trackpad_held = False

    def flying_state(self, flying_state):
        '''Checks whether the drone is standing on the ground or flying and
        changes the self.airborne variable accordingly.
        '''
        if flying_state.state == 2:
            self.airborne = True
        elif flying_state.state == 0:
            self.airborne = False

    def battery_state(self, battery):
        '''Checks the discharge state of the battery and gives a warning
        when the battery voltage gets low.
        '''
        if ((battery.percent <= 20) and ((battery.percent % 5) == 0)):
            print 'battery.percent', battery.percent, (battery.percent % 5)
            print highlight_yellow(
                        ' Battery voltage low -- ', battery.percent,
                        '% left, switch to a freshly charged battery! ')

    def ctrl_state_finish(self, empty):
        '''Checks whether controller has finished the current state.
        '''
        self.state_finish = True

    def r_trigger(self, pressed):
        if pressed.data and (self.state == "omg standby"):
            self.change_state = True

    def transform_point(self, point, _from, _to):
        '''Transforms point (geometry_msgs/PointStamped) from frame "_from" to
        frame "_to".
        Arguments:
            - _from, _to = string, name of frame
        '''
        transform = self.kalman.get_transform(_from, _to)
        point_transformed = tf2_geom.do_transform_point(point, transform)

        return point_transformed


if __name__ == '__main__':
    demo = Demo()
    demo.pc = Perception()
    demo.wm = WorldModel()
    demo.kalman = Kalman(demo.wm.model)
    demo.start()

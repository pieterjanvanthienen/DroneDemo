#!/usr/bin/env python

from geometry_msgs.msg import Twist, TwistStamped, Point, Pose2D, PoseStamped
from std_msgs.msg import Bool, Empty
from visualization_msgs.msg import Marker

from bebop_demo.msg import Trigger, Trajectories, Obstacle

from bebop_demo.srv import GetPoseEst, ConfigMotionplanner

import rospy
import numpy as np


class VelCommander(object):

    def __init__(self):
        """Initialization of Controller object.

        Args:
            sample_time : Period between computed velocity samples.
            update_time : Period of problem solving.
        """
        rospy.init_node("vel_commander_node")

        self.calc_succeeded = False
        self.progress = True
        self.startup = False
        self._index = 0

        self.feedback_gain = rospy.get_param('vel_cmd/feedback_gain', 0.3)
        self.safety_treshold = rospy.get_param('vel_cmd/safety_treshold', 0.5)
        self.pos_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_pos_tol', 0.05)
        self.angle_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_angle_tol', 0.05)
        self.input_nrm_tol = rospy.get_param(
                                        'vel_cmd/goal_reached_input_tol', 0.03)

        self._sample_time = rospy.get_param('vel_cmd/sample_time', 0.01)
        self._update_time = rospy.get_param('vel_cmd/update_time', 0.5)
        self.rate = rospy.Rate(1./self._sample_time)

        self.pos_nrm = np.inf

        self.cmd_twist_convert = TwistStamped()
        self.cmd_twist_convert.header.frame_id = "world_rot"
        self._trigger = Trigger()

        # Marker setup
        self.marker_setup()

        # Coefficients for inverted model of velocity to input angle
        self.initialize_vel_model()

        self._robot_est_pose = Pose2D()
        self._robot_est_pose.x = 0.
        self._robot_est_pose.y = 0.

        self._traj = {'v': [0.0], 'w': [0.0], 'x': [0.0], 'y': [0.0]}
        self._traj_strg = {'v': [0.0], 'w': [0.0], 'x': [0.0], 'y': [0.0]}
        self._inputs_applied = {'jx': [], 'jy': []}

        self.cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)
        self._mp_trigger_topic = rospy.Publisher(
            'mp_trigger', Trigger, queue_size=1)
        self.trajectory_desired = rospy.Publisher(
            'motionplanner/desired_path', Marker, queue_size=1)
        self.trajectory_real = rospy.Publisher(
            'motionplanner/real_path', Marker, queue_size=1)
        self.trajectory_marker = rospy.Publisher(
            'motionplanner_traj_marker', Marker, queue_size=1)

        rospy.Subscriber('motionplanner/result', Trajectories,
                         self.get_mp_result)

    def initialize_vel_model(self):
        '''Initializes model parameters for conversion of desired velocities to
        angle inputs.
        State space model x[k+1] = A*x[k] + B*u[k] in observable canonical
        form, corresponding to discete time transfer function

                   b1*z + b0
        G(z) = -----------------
                z^2 + a1*z + a0

        with sampling time equal to vel_cmd_Ts (0.01s).
        '''
        # X-direction
        self.input_old_x = 0.

        b0 = -0.0007172
        b1 = 0.002182
        a0 = 0.9592
        a1 = -1.959

        self.coeffs_x = np.array([-b0, a0, a1, 1])/b1

        # Y-direction
        self.input_old_y = 0.

        b0 = -0.0007388
        b1 = 0.002249
        a0 = 0.9576
        a1 = -1.957

        self.coeffs_y = np.array([-b0, a0, a1, 1])/b1

        # # Z-direction
        # b0 = 0.05301
        # a0 = -0.946
        #
        # self.coeffs_x = np.array([a0, 1])/b0

    def start(self):
        '''Configures,
        Starts the controller's periodical loop.
        '''
        rate = self.rate

        configured = self.configure()
        print '-----------------------------------------'
        print '- Controller & Motionplanner Configured -'
        print '-        Velocity Control Started       -'
        print '-----------------------------------------'

        while not rospy.is_shutdown():
            while (self.progress):
                if (self.startup):
                    self.update()
                    self.progress = self.proceed()
                rate.sleep()
            print '-------------------'
            print '- Target Reached! -'
            print '-------------------'
            rate.sleep()

    def configure(self):
        '''Configures the controller by loading in the room and static
        obstacles.
        Sends Settings to Motionplanner.
        Settings constists of
            - environment
        Waits for Motionplanner to set mp_status to configured.

        NOTE: This would be better as a service!
        '''

        # List containing obstacles of type Obstacle()
        self.obstacles = []
        rospy.wait_for_service("/motionplanner/config_motionplanner")
        config_success = False
        try:
            config_mp = rospy.ServiceProxy(
                "/motionplanner/config_motionplanner", ConfigMotionplanner)
            config_success = config_mp(self.obstacles)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            config_success = False
        rospy.Subscriber('motionplanner/goal', Pose2D, self.set_goal)

        return config_success

    def set_goal(self, goal):
        '''Sets the goal and fires motionplanner.
        Args:
            goal: Pose2D
        '''
        self._goal = goal

        self.progress = True
        self._time = 0.
        self._new_trajectories = False

        self._robot_est_pose = self.get_pose_est()
        self._inputs_applied = {'jx': [], 'jy': []}

        self.fire_motionplanner()

        self._init = True
        self.startup = True

        print '-----------------------'
        print 'Motionplanner goal set!'
        print '-----------------------'

    def fire_motionplanner(self):
        '''Publishes inputs to motionplanner via Trigger topic.
        '''
        self._trigger.goal = self._goal
        self._trigger.state = self._robot_est_pose
        self._trigger.current_time = self._time
        self._mp_trigger_topic.publish(self._trigger)

    def get_mp_result(self, data):
        '''Store results of motionplanner calculations.

        Args:
            data : calculated trajectories received from 'mp_result' topic,
                   published by motionplanner.
        '''
        v_traj = data.v_traj
        w_traj = data.w_traj
        x_traj = data.x_traj
        y_traj = data.y_traj
        self.store_trajectories(v_traj, w_traj, x_traj, y_traj)

    def update(self):
        '''
        - Updates the controller with newly calculated trajectories and
        velocity commands.
        - Sends out new velocity command.
        - Retrieves new pose estimate.
        '''
        # Send velocity sample.
        self.cmd_twist_convert.header.stamp = rospy.Time.now()
        self.cmd_vel.publish(self.cmd_twist_convert)

        # Store applied commands.
        self._inputs_applied['jx'].append(
                                        self.cmd_twist_convert.twist.linear.x)
        self._inputs_applied['jy'].append(
                                        self.cmd_twist_convert.twist.linear.y)

        # Retrieve new pose estimate from World Model.
        # This is a pose estimate for the first following time instance [k+1]
        # if the velocity command sent above corresponds to time instance [k].
        self._robot_est_pose = self.get_pose_est()

        # Check for new trajectories. Trigger Motionplanner or raise
        # 'overtime'
        pose0 = [self._robot_est_pose.x, self._robot_est_pose.y]

        if self._init:
            if not self._new_trajectories:
                return
            self._index = int(self._update_time/self._sample_time)
            self._init = False

        if ((self._index >= int(self._update_time/self._sample_time))
                or (self._index >= len(self._traj['v'])-2)):
            if self._new_trajectories:
                # Load fresh trajectories.
                self.load_trajectories()
                self._new_trajectories = False
                self._time += self._index*self._sample_time
                self.pos_index = self._index
                self._index = 1
                # Trigger motion planner.
                self.fire_motionplanner()

            else:
                self.calc_succeeded = False
                print '-- !! -------- !! --'
                print '-- !! Overtime !! --'
                # Brake as emergency measure: Bebop brakes automatically when
                # /bebop/cmd_vel topic receives all zeros.
                self.cmd_twist_convert.twist.linear.x = 0.
                self.cmd_twist_convert.twist.linear.y = 0.
                self.cmd_twist_convert.twist.linear.z = 0.
                self.cmd_vel.publish(self.cmd_twist_convert.twist)
                return

        # Convert feedforward velocity command to angle input.
        self.convert_vel_cmd(self._index)

        # Combine feedback and feedforward commands.
        self.calc_vel_cmd()

        self._index += 1

    def convert_vel_cmd(self, index):
        '''Converts a velocity command to a desired input angle according to
        the difference equations:

        - for second order velocity/input relation (x and y):
            j[k+1] = 1/b1*{ -b0*j[k] + a0*v[k] + a1*v[k+1] + v[k+2] }
                   = 1/b1*(-b0, a0, a1, 1)*(j[k], v[k], v[k+1], v[k+2])'

        (- for first order velocity/input relation (z):
            j[k+1] = 1/b0*{ a0*v[k+1] + v[k+2] } (later, 3d flight))

        where j is the input signal applied to the bebop and v the desired
        velocity.
        '''

        self._cmd_twist.twist
        self.cmd_twist_convert

        # Convert velocity command in x-direction
        phi = np.array([[self._inputs_applied['jx'][-1]],
                        [self._traj['v'][index]],
                        [self._traj['v'][index + 1]],
                        [self._traj['v'][index + 2]]])
        self.cmd_twist_convert.twist.linear.x = np.matmul(self.coeffs_x, phi)

        # Convert velocity command in y-direction
        phi = np.array([[self._inputs_applied['jy'][-1]],
                        [self._traj['w'][index]],
                        [self._traj['w'][index + 1]],
                        [self._traj['w'][index + 2]]])
        self.cmd_twist_convert.twist.linear.y = np.matmul(self.coeffs_y, phi)
        # print ('twist convert',
        #        'jx', self.cmd_twist_convert.twist.linear.x,
        #        'jy', self.cmd_twist_convert.twist.linear.y)

    def calc_vel_cmd(self):
        '''Combines the feedforward and feedback commands to generate a
        velocity command and publishes this command.
        '''

        # Summing feedforward and feedback part of the controller.
        x_error = self._traj['x'][self._index] - self._robot_est_pose.x
        y_error = self._traj['y'][self._index] - self._robot_est_pose.y

        # Safety feature, if position measurement stops working, set velocity
        # command equal to zero
        if (x_error > self.safety_treshold) or (
                y_error > self.safety_treshold):
            self.cmd_twist_convert.twist = Twist()
            return

        # Combine feedforward and feedback part
        # SHOULD BE ADAPTED! OR FEEDBACK WILL BE TAKEN AS PART OF LAST INPUT J
        # BOS: Geeft dat niet iets accurater dan het niet te doen?
        self.cmd_twist_convert.twist.linear.x = (
            x_error * self.feedback_gain +
            self.cmd_twist_convert.twist.linear.x)

        self.cmd_twist_convert.twist.linear.y = (
            y_error * self.feedback_gain +
            self.cmd_twist_convert.twist.linear.y)

    def get_pose_est(self):
        '''Retrieves a new pose estimate from world model.
        '''

        rospy.wait_for_service("/world_model/get_pose")
        try:
            pose_est = rospy.ServiceProxy(
                "/world_model/get_pose", GetPoseEst)
            resp = pose_est(self._cmd_twist)
            xhat = resp.pose_est.point
            self.__publish_real(xhat.x, xhat.y)
            pose = Pose2D(x=xhat.x, y=xhat.y)
            return pose
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return

    def load_trajectories(self):
        self._traj['v'] = self._traj_strg['v'][:]
        self._traj['w'] = self._traj_strg['w'][:]
        self._traj['x'] = self._traj_strg['x'][:]
        self._traj['y'] = self._traj_strg['y'][:]

    def store_trajectories(self, v_traj, w_traj, x_traj, y_traj):
        '''Stores the trajectories and indicate that new trajectories have
        been calculated.

        Args:
            v_traj : trajectory speed in x-direction
            w_traj : trajectory speed in y-direction
            x_traj : trajectory position in x-direction
            y_traj : trajectory position in y-direction
        '''
        self._traj_strg = {}
        self._traj_strg = {
            'v': v_traj, 'w': w_traj, 'x': x_traj, 'y': y_traj}
        self._new_trajectories = True
        print '--------------- NEW TRAJECTORIES AVAILABLE ---------------'

        x_traj = self._traj_strg['x'][:]
        y_traj = self._traj_strg['y'][:]
        self.__publish_desired(x_traj, y_traj)

    def proceed(self):
        '''Determines whether goal is reached.
        Returns:
            not stop: boolean whether goal is reached. If not, controller
                      proceeds to goal.
        '''
        if len(self._inputs_applied['jx']) == 0:
            return True

        stop_linear = True
        stop = True

        pos_nrm = np.linalg.norm(np.array(
            [self._robot_est_pose.x, self._robot_est_pose.y])
            - np.array([self._goal.x, self._goal.y]))
        input_nrm = np.linalg.norm(
            [self._inputs_applied['jx'][-1],
                self._inputs_applied['jy'][-1]])
        angle_nrm = np.abs(self.desired_angle - self.current_angle)

        stop_linear = (pos_nrm < self.pos_nrm_tol) and (
                            input_nrm < self.input_nrm_tol)

        if (stop_linear):
            self.cmd_twist_convert.twist.linear.x = 0.
            self.cmd_twist_convert.twist.linear.y = 0.
        stop *= (self.stop_linear and angle_nrm < self.angle_nrm_tol)

        return not stop

    def marker_setup(self):
        '''Setup markers to display the desired and real path of the drone in
        rviz.
        '''
        # Desired path
        self._desired_path = Marker()
        self._desired_path.header.frame_id = 'world'
        self._desired_path.ns = "trajectory_desired"
        self._desired_path.id = 0
        self._desired_path.type = 4  # Line List.
        self._desired_path.action = 0
        self._desired_path.pose.position.z = 0.
        self._desired_path.pose.orientation.x = 0
        self._desired_path.pose.orientation.y = 0
        self._desired_path.pose.orientation.z = 0
        self._desired_path.scale.x = 0.05
        self._desired_path.scale.y = 0.05
        self._desired_path.scale.z = 0.0
        self._desired_path.color.r = 1.0
        self._desired_path.color.g = 0.0
        self._desired_path.color.b = 0.0
        self._desired_path.color.a = 1.0
        self._desired_path.lifetime = rospy.Duration(0)

        self.pos_index = 0
        self.old_len = 0

        # Real path
        self._real_path = Marker()
        self._real_path.header.frame_id = 'world'
        self._real_path.ns = "trajectory_real"
        self._real_path.id = 1
        self._real_path.type = 4  # Line List.
        self._real_path.action = 0
        self._real_path.pose.position.z = 0.
        self._real_path.pose.orientation.x = 0
        self._real_path.pose.orientation.y = 0
        self._real_path.pose.orientation.z = 0
        self._real_path.scale.x = 0.2
        self._real_path.scale.y = 0.2
        self._real_path.scale.z = 1.0
        self._real_path.color.r = 0.0
        self._real_path.color.g = 1.0
        self._real_path.color.b = 0.0
        self._real_path.color.a = 1.0
        self._real_path.lifetime = rospy.Duration(0)

    def __publish_desired(self, x_traj, y_traj):
        '''Publish planned x and y trajectory to topic for visualisation in
        rviz.
        '''
        # Still has to be adapted to remove old path when new goal has been
        # set.
        self._desired_path.header.stamp = rospy.get_rostime()

        # Delete points in path that have not been used before new list was
        # calculated.
        self._desired_path.points = self._desired_path.points[
                                            0:(self.old_len + self.pos_index)]
        self.old_len = len(self._desired_path.points)

        # Add new calculated pos list to old one.
        new_pos = [0]*len(x_traj)
        for k in range(len(x_traj)):
            new_pos[k] = Point(x=x_traj[k], y=y_traj[k])
        self._desired_path.points += new_pos

        self.trajectory_desired.publish(self._desired_path)

    def __publish_real(self, x_pos, y_pos):
        '''Publish real x and y trajectory to topic for visualisation in
        rviz.
        '''
        self._real_path.header.stamp = rospy.get_rostime()

        point = Point(x=x_pos, y=y_pos)
        self._real_path.points.append(point)

        self.trajectory_real.publish(self._real_path)


if __name__ == '__main__':
    vel_command = VelCommander()
    vel_command.start()

#!/usr/bin/env python
import math, sys
import rospy
import actionlib
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Twist, Pose, TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray, Float64, Bool
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, SwitchControllerResponse
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
from robotiq_ft_sensor.srv import sensor_accessor
import tf
import numpy as np
# import matplotlib.pyplot as plt

class UR3CartROS(object):
    def __init__(self):

        self.ft_data = [0.0, 0.0, 0.0]
        self.orien_goal = [0.0, 0.0, 1.0]
        self.contacted = 0
        self.flag_docking = 0
        self.twist = Twist()
        self.vel_lst = 0
        self.init_pos = Bool()
        self.init_pos.data = False
        self.docked = Bool()
        self.docked.data = False

        self.base_link = 'base_link'
        self.ee_link = 'ee_link'

        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints=self.tree.getNrOfJoints()

        self.vel_ik_solver = kdl.ChainIkSolverVel_pinv(self.chain)
        self.pos_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)

        self.joint_state = kdl.JntArrayVel(self.num_joints)
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        rospy.init_node('ur_ee_vel_controller')
        rospy.Subscriber('/joint_state', JointState, self.arm_joint_state_cb)
        rospy.Subscriber('ft_data', Twist, self.ft_cb, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        self.joint_vel_cmd_pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.joint_pos_cmd_pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10)
        self.speed_scaling_pub = rospy.Publisher('/speed_scaling_factor', Float64, queue_size=1)
        self.init_pub = rospy.Publisher('init_flg', Bool, queue_size=1 )
        self.docked_pub = rospy.Publisher('docked_flg', Bool, queue_size=1 )

        self.twist_pub = rospy.Publisher('jog_arm_server/delta_jog_cmds', TwistStamped, queue_size=1)
        # self.surf_orien_pub = rospy.Publisher('surf_orien', Twist, queue_size=1)
        self.speed_scaling_pub.publish(Float64(0.3))

        self.switch_controller_cli = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        # rospy.Subscriber('robotiq_ft_wrench', WrenchStamped, self.ft_cb, queue_size=10)
        # self.ft_sensor_cli = rospy.ServiceProxy('robotiq_ft_sensor_acc', sensor_accessor)
        self.joint_pos_cli = actionlib.SimpleActionClient('scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.tf_listener = tf.TransformListener()
        # self.reset_ft_sensor()
        self.init_pub.publish(self.init_pos)
        self.docked_pub.publish(self.docked)
        # rospy.sleep(0.5)


    # def reset_ft_sensor(self):
    #     rospy.wait_for_service('robotiq_ft_sensor_acc')
    #     try:
    #         print('Service is ready, reset force sensor...')
    #         self.ft_sensor_cli(8,"")
    #         print('Done')
    #
    #     except rospy.ServiceException as e:
    #         print("Service call fialed: %s" %e)


    def switch_controller(self, mode = None):
        req = SwitchControllerRequest()
        res = SwitchControllerResponse()

        req.start_asap = False
        req.timeout = 0.0

        if mode == 'velocity':
            req.start_controllers = ['joint_group_vel_controller']
            req.stop_controllers = ['scaled_pos_traj_controller']
            req.strictness = req.STRICT

        elif mode == 'position':
            req.start_controllers = ['scaled_pos_traj_controller']
            req.stop_controllers = ['joint_group_vel_controller']
            req.strictness = req.STRICT
        else:
            rospy.logwarn('Unkown mode for controller!')

        res = self.switch_controller_cli.call(req)

    def arm_joint_state_cb(self, joint_msg):
        self.joint_state.q[0] = joint_msg.position[2]
        self.joint_state.q[1] = joint_msg.position[1]
        self.joint_state.q[2] = joint_msg.position[0]
        self.joint_state.q[3] = joint_msg.position[3]
        self.joint_state.q[4] = joint_msg.position[4]
        self.joint_state.q[5] = joint_msg.position[5]

        self.joint_state.qdot[0] = joint_msg.velocity[2]
        self.joint_state.qdot[1] = joint_msg.velocity[1]
        self.joint_state.qdot[2] = joint_msg.velocity[0]
        self.joint_state.qdot[3] = joint_msg.velocity[3]
        self.joint_state.qdot[4] = joint_msg.velocity[4]
        self.joint_state.qdot[5] = joint_msg.velocity[5]

    def pub_joint_vel(self, q_dot):
        joint_vel = Float64MultiArray()
        qdot = [0.0]*self.num_joints

        for i in range(self.num_joints):
            qdot[i] = q_dot[i]
        rospy.loginfo(qdot)
        joint_vel.data = qdot
        self.joint_vel_cmd_pub.publish(joint_vel)

    def pub_joint_pos(self, q):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj_point = JointTrajectoryPoint()
        traj_point.time_from_start = rospy.Duration(3.0)

        for i in range(self.num_joints):
            traj_point.positions.append(q[i])
            traj_point.velocities.append(0.0)

        traj.points.append(traj_point)
        traj_goal = FollowJointTrajectoryGoal()
        traj_goal.trajectory = traj
        self.joint_pos_cli.send_goal(traj_goal)
        self.joint_pos_cli.wait_for_result()

    def get_eef_pose(self):
        eef_pose = kdl.Frame()
        self.pos_fk_solver.JntToCart(self.joint_state.q, eef_pose)

        return eef_pose

    def ik(self, eef_pose):
        q_init = self.joint_state.q  # respect to the robot base
        q_sol = kdl.JntArray(self.num_joints)
        self.pos_ik_solver.CartToJnt(q_init, eef_pose, q_sol) # q_sol is respect to the robot base
        return q_sol

    def go_to_eef_pose(self, eef_pose):
        q_sol = self.ik(eef_pose)
        self.pub_joint_pos(q_sol)

    def _calculate_pose(self, eef_vel):
        cur_pos = self.get_eef_pose()
        new_pos = kdl.Frame()
        rospy.loginfo('cur_pos %s', cur_pos)
        new_pos.p = cur_pos.p +  kdl.Vector(eef_vel[0]/100,eef_vel[1]/100,eef_vel[2]/100)
        angular_R = kdl.Rotation.EulerZYX(eef_vel[5]/(6*math.pi), eef_vel[4]/(6*math.pi), eef_vel[3]/(6*math.pi))
        new_pos.M = cur_pos.M * angular_R
        rospy.loginfo('new_pos %s', new_pos)
        return new_pos

    def _calculate_qdot(self, eef_vel):
        eef_vel_kdl = kdl.Twist.Zero()
        eef_vel_kdl.vel[0] = eef_vel[0]
        eef_vel_kdl.vel[1] = eef_vel[1]
        eef_vel_kdl.vel[2] = eef_vel[2]
        eef_vel_kdl.rot[0] = eef_vel[3]
        eef_vel_kdl.rot[1] = eef_vel[4]
        eef_vel_kdl.rot[2] = eef_vel[5]
        # Transform EEF Twist to base frame
        angular_R = kdl.Rotation.EulerZYX(eef_vel[5]/(6*math.pi), eef_vel[4]/(6*math.pi), eef_vel[3]/(6*math.pi))
        eef_pose = kdl.Frame()
        qdot_sol = kdl.JntArray(self.num_joints)
        self.pos_fk_solver.JntToCart(self.joint_state.q, eef_pose)
        eev_vel_base = eef_pose.M * angular_R * eef_vel_kdl
        self.vel_ik_solver.CartToJnt(self.joint_state.q, eef_vel_kdl, qdot_sol)
        return qdot_sol

    def rotation_matrix_from_vectors(self, vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return rot: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (np.around(vec1, decimals=1) / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) / (1+c)
        return rotation_matrix

    def cal_RPY(self, rot):
        r21 = rot[1][0]
        r11 = rot[0][0]
        r31 = rot[2][0]
        r32 = rot[2][1]
        r33 = rot[2][2]

        r = np.arctan2(r32, r33)
        p = np.arctan2(-r31, np.sqrt(r32**2+r33**2))
        y = np.arctan2(r21, r11)

        return r, p, y

    def orien_align(self, vec1, vec2):
        rot = self.rotation_matrix_from_vectors(vec1, vec2)
        roll, pitch, yaw = self.cal_RPY(rot)
        return roll, pitch, yaw

    def vel_smooth(self,vel_lst, vel_in):
        step = 0.001
        dif = vel_in - vel_lst

        if np.abs(dif) > step:
            vel_out = vel_lst + step * np.sign(vel_in - vel_lst)
        else:
            vel_out = vel_in

        return vel_out

    def ft_cb(self, ft_msg):
        # self.ft_data[0] = ft_msg.wrench.force.x
        # self.ft_data[1] = ft_msg.wrench.force.y
        # self.ft_data[2] = -ft_msg.wrench.force.z
        self.ft_data[0] = math.floor(ft_msg.linear.x) if ft_msg.linear.x >= 0 else math.ceil(ft_msg.linear.x)
        self.ft_data[1] = math.floor(ft_msg.linear.y) if ft_msg.linear.y >= 0 else math.ceil(ft_msg.linear.y)
        self.ft_data[2] = -ft_msg.linear.z
        ft_norm = np.around(np.abs(self.ft_data[2]), decimals=1)
        # print('contacted force: {}'.format(np.linalg.norm(self.ft_data)))
        if self.flag_docking ==1 and (self.ft_data[0] > 1 or self.ft_data[1] > 1 or self.ft_data[2] > 1):
            self.contacted =1
            if ft_norm > 6:
                rospy.loginfo('ft_z:{}'.format(ft_norm))
                vel = 0.3 if (ft_norm-5) >0.3 else max(np.abs(ft_norm-5), 0.1)

            elif ft_norm < 4:
                vel = -0.3 if (3-ft_norm) >0.3 else -max((3-ft_norm), 0.1)
            else:
                vel = 0.0

            self.twist.linear.x = vel
            self.twist.linear.y = 0
            self.twist.linear.z = 0

            # roll angular_z pitch angular_y yaw angular_x
            cur_orien = [0.0, 0.0, 0.0]
            cur_orien[0] = self.ft_data[0] if self.ft_data[0] >=1 else 0.0
            cur_orien[1] = self.ft_data[1] if self.ft_data[1] >=1 else 0.0
            cur_orien[2] = self.ft_data[2]

            roll, pitch, yaw = self.orien_align(cur_orien, self.orien_goal)
            roll = roll*180/math.pi
            pitch = pitch*180/math.pi
            yaw = yaw*180/math.pi
            self.twist.angular.x = 0
            self.twist.angular.y = pitch
            self.twist.angular.z = roll

        elif self.contacted == 0 or (self.contacted == 1 and ft_norm <1):
            if self.twist.linear.x == 0:
                self.twist.linear.x = -0.2
            else:
                self.twist.linear.x = -0.3 if (1-np.abs(self.ft_data[2])*0.7) > 0.3 else -max((1-np.abs(self.ft_data[2])*0.7),0.1)

            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0

        else:
            self.twist.linear.x = 0
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = 0


    def joy_cb(self, joy_msg):
        #    left axis 0 1  right axis 3 4  LB buttons 4 RB buttons 5 X buttons 2 Y buttons 3 A buttons 1 B buttons 2
        self.flag_docking = joy_msg.buttons[2]
        self.flag_init_joint = joy_msg.buttons[3]
        self.init_pos.data = False
        self.init_pub.publish(self.init_pos)
        if self.flag_init_joint==1:
            rospy.loginfo("start initialize joint...")
            self.init_pos.data = True
            self.init_pub.publish(self.init_pos)
            self.docked.data = False
            self.docked_pub.publish(self.docked)
            # eef_home_joint = [0, -0.60*math.pi, -0.511*math.pi, 0.1074*math.pi, 0.5038*math.pi, math.pi*1.5]
            # eef_home_joint = [-math.pi, -1.211, 1.92553, -0.71161395, math.pi/2, math.pi/2]
            # eef_home_joint = [0.007588, -1.211, 1.92553, -0.71161395, math.pi/2, math.pi/2]
            eef_home_joint = [0, -0.60*math.pi, -0.477*math.pi, 0.0806*math.pi, 0.5*math.pi, 1.5*math.pi]
            self.pub_joint_pos(eef_home_joint)
            rospy.loginfo("ee_f_cart_pos %s", self.get_eef_pose())
            # rospy.sleep(1)
            self.flag_init_joint=0
            self.contacted = 0
            # self.reset_ft_sensor()


        elif self.flag_docking ==1:

            rospy.loginfo('start docking...')
            self.init_pos.data = False
            f_hist = []
            self.init_pub.publish(self.init_pos)
            while True:
                twiststamped  = TwistStamped()
                twiststamped.header.stamp = rospy.Time.now()
                # print ('roll:{},pitch:{},yaw:{}'.format(self.twist.angular.x, self.twist.angular.y, self.twist.angular.y))
                vel_sm = self.vel_smooth(self.vel_lst, self.twist.linear.x)
                # rospy.loginfo('linear.x_in: {}, linear_x_sm: {}'.format(self.twist.linear.x, vel_sm))
                if self.contacted==1 and (np.abs(self.twist.angular.y) > 3 or np.abs(self.twist.angular.z) > 3):
                    twiststamped.twist.linear.x = vel_sm

                    twiststamped.twist.angular.x = 0.0
                    angular_y = np.sign(self.twist.angular.y)*0.5 if (np.abs(self.twist.angular.y) - 3)*0.05 >0.5 else np.sign(self.twist.angular.y) * max(np.around((np.abs(self.twist.angular.y) - 3)*0.05,decimals=1), 0.1)
                    twiststamped.twist.angular.y = angular_y
                    angular_z = np.sign(self.twist.angular.z)*0.5 if (np.abs(self.twist.angular.z) - 3)*0.05 >0.5 else np.sign(self.twist.angular.z) * max(np.around((np.abs(self.twist.angular.z) - 3)*0.05,decimals=1), 0.1)
                    twiststamped.twist.angular.z = angular_z

                    twiststamped.twist.linear.y = np.round(angular_z, decimals=1)
                    twiststamped.twist.linear.z = -np.round(angular_y, decimals=1)
                    # rospy.loginfo('contacted:{},angular.x: {}, angular.y:{}, angular.z:{}'.format(self.contacted, twiststamped.twist.angular.x, angular_y, angular_z))
                    # print(twiststamped.twist.linear.x, twiststamped.twist.angular.y, twiststamped.twist.angular.z)
                else:
                    twiststamped.twist.linear.x = vel_sm
                    twiststamped.twist.linear.y = 0.0
                    twiststamped.twist.linear.z = 0.0
                    twiststamped.twist.angular.x = 0.0
                    twiststamped.twist.angular.y = 0.0
                    twiststamped.twist.angular.z = 0.0
                # rospy.loginfo('contacted:{},linear.x: {}, linear.y:{}, linear.z:{}'.format(self.contacted, twiststamped.twist.linear.x, twiststamped.twist.linear.y, twiststamped.twist.linear.z))
                self.twist_pub.publish(twiststamped)
                self.vel_lst = twiststamped.twist.linear.x

                if self.contacted==1 and np.abs(self.twist.angular.y) <= 5 and np.abs(self.twist.angular.z) <= 5 and 3 <= np.abs(self.ft_data[2]) and np.abs(self.ft_data[2]) <=5:
                    f_hist.append(np.abs(self.ft_data[2]))
                    rospy.loginfo("f_hist:{}".format(f_hist))
                    if len(f_hist) ==5:
                        if all(f>=3 for f in f_hist):
                            break
                        else:
                            f_hist=[]
            twiststamped.twist.linear.x = 0.0
            twiststamped.twist.linear.y = 0.0
            twiststamped.twist.linear.z = 0.0
            twiststamped.twist.angular.x = 0.0
            twiststamped.twist.angular.y = 0.0
            twiststamped.twist.angular.z = 0.0
            self.twist_pub.publish(twiststamped)
            self.vel_lst = 0

            # print ('pitch:{},yaw:{}, force_Z:{}'.format(self.twist.angular.y, self.twist.angular.y, np.abs(self.ft_data[2])))
            rospy.loginfo("docking completed, contacted:{}".format(self.contacted))
            self.docked.data = True
            self.docked_pub.publish(self.docked)

        # else:
        #     twist_ft = Twist()
        #     if np.abs(self.twist.angular.y) > 5 or np.abs(self.twist.angular.z) > 5:
        #         twist_ft.linear.x = 0.0
        #         twist_ft.linear.y = 0.0
        #         twist_ft.linear.z = 0.0
        #         twist_ft.angular.x = 0.0
        #         twist_ft.angular.y = np.sign(self.twist.angular.y)*0.5
        #         twist_ft.angular.z = np.sign(self.twist.angular.z)*0.5
        #         # print(twist_ft.angular.y, twist_ft.angular.z)
        #     else:
        #         twist_ft.linear.x = 0.0
        #         twist_ft.linear.y = 0.0
        #         twist_ft.linear.z = 0.0
        #         twist_ft.angular.x = 0.0
        #         twist_ft.angular.y = 0.0
        #         twist_ft.angular.z = 0.0
        #     self.ft_orien_pub.publish(twist_ft)


        # elif self.flag_init_pos==1:
            # aa = ur3_ultrasound.get_eef_pose().M
            # bb = kdl.Rotation(1,0,0,0,0,-1,0,1,0)
            # print(bb.GetQuaternion())
        #     rospy.loginfo("start initialize pos....")
        #     eef_home_pos = kdl.Frame(kdl.Rotation.Quaternion(1, 0, 0, 0), kdl.Vector(0.335, 0.112, 0.125))
        #     self.go_to_eef_pose(eef_home_pos)
        #     rospy.loginfo("ee_f_cart_pos %s", self.get_eef_pose())
        #     rospy.sleep(1)
        #     self.flag_init_pos=0
        # else:
        #
        #     linear_x = joy_msg.axes[1]
        #     linear_y = joy_msg.axes[0]
        #     linear_z = -joy_msg.buttons[4]+joy_msg.buttons[5]
        #     angular_x = joy_msg.axes[3]
        #     angular_y = joy_msg.axes[4]
        #     angular_z = -joy_msg.buttons[0] + joy_msg.buttons[1]
        #     eef_vel = [linear_x, linear_y, linear_z,angular_x,angular_y,angular_z]
        #     if linear_x!=0 or linear_y!=0 or linear_z!=0 or angular_x!=0 or angular_y!=0 or angular_z!=0:
        #         rospy.loginfo("eef_vel %s", eef_vel)
        #         pos_new = self._calculate_pose(eef_vel)
        #         self.go_to_eef_pose(pos_new)


if __name__=='__main__':
    try:
        ur3_ultrasound = UR3CartROS()
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down initial_pos node')

#! /usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from math import pi
import geometry_msgs.msg
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from time import sleep
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class initial_pub_pos:
    def __init__(self):
        rospy.init_node('initial_pub_pos', anonymous = True)
        rospy.loginfo("Strating node initial_pub_pos")
        self.joy_sub = rospy.Subscriber('joy', Joy, self.call_back, queue_size = 1)
        self.init_pub = rospy.Publisher('init_flg', Bool, queue_size=1 )

        self.init_pos = Bool()
        self.init_pos.data = False

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur3_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur3_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.05)
        self.arm.set_max_velocity_scaling_factor(.2)

        self.default_joint_states = self.arm.get_current_joint_values()
        # self.default_joint_states[0] = -1.57691
        # self.default_joint_states[1] = -1.71667
        # self.default_joint_states[2] = 1.79266
        # self.default_joint_states[3] = -1.67721
        # self.default_joint_states[4] = -1.5705
        # self.default_joint_states[5] = 0.0
        self.default_joint_states[0] = 0  # shoulder_pan_joint
        self.default_joint_states[1] = -1.0996  # shoulder_lift_joint
        self.default_joint_states[2] = 1.9199    # elbow_joint
        self.default_joint_states[3] = -0.7858    # wrist_1_joint
        self.default_joint_states[4] = 1.57079     # wrist_2_joint
        self.default_joint_states[5] = 1.7639         # wrist_3s_joint

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        #move to target joint_states
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()
        self.arm.execute(plan)
        self.init_pub.publish(self.init_pos)

    def cartesian_execut(self, waypoints):
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)
        # If we have a complete plan, execute the trajectory
        if 1-fraction < 0.2:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed")

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


    def call_back(self, msg):
        self.init_position = msg.buttons[2]
        self.init_joint = msg.buttons[3]

        self.execute()

        if self.init_position or self.init_joint:
            self.initial = True
        else:
            self.initial = False
        self.init_pos.data = self.initial
        self.init_pub.publish(self.init_pos)

        self.init_position = False
        self.init_joint = False


    def execute(self):
        if self.init_joint:
            self.default_joint_states = self.arm.get_current_joint_values()
            # self.default_joint_states[0] = -1.57691
            # self.default_joint_states[1] = -1.71667
            # self.default_joint_states[2] = 1.79266
            # self.default_joint_states[3] = -1.67721
            # self.default_joint_states[4] = -1.5705
            # self.default_joint_states[5] = 0.0
            self.default_joint_states[0] = 0  # shoulder_pan_joint
            self.default_joint_states[1] = -1.0996  # shoulder_lift_joint
            self.default_joint_states[2] = 1.9199    # elbow_joint
            self.default_joint_states[3] = -0.7858    # wrist_1_joint
            self.default_joint_states[4] = 1.57079     # wrist_2_joint
            self.default_joint_states[5] = 1.7639         # wrist_3s_joint

            self.arm.set_joint_value_target(self.default_joint_states)

            # Set the internal state to the current state
            #move to target joint_states
            self.arm.set_start_state_to_current_state()
            plan = self.arm.plan()
            self.arm.execute(plan)

        elif self.init_position:
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # print(start_pose)
            # Initialize the waypoints list
            self.waypoints = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            wpose.position.x = 0.335
            wpose.position.y = 0.112
            wpose.position.z = 0.125

            # print(wpose)
            self.waypoints.append(deepcopy(wpose))
            self.arm.set_start_state_to_current_state()
            position_offset = np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.y-start_pose.position.y)**2 \
                +(wpose.position.z-start_pose.position.z)**2)
            rospy.loginfo(position_offset)
            if position_offset<0.001:
                rospy.loginfo("Warnig: target position overlaps with the initial position!")
            else:
                self.cartesian_execut(self.waypoints)

if __name__ == "__main__":
    try:
        initial_pub_pos()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down initial_pub_pos node."

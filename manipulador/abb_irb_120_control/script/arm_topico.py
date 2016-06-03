#!/usr/bin/env python

"""
    trajectory_demo.py - Version 0.1 2014-01-14

    Send a trajectory to the FollowJointTrajectoryAction server

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

    Versie voor onze smart_arm
"""
import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL
from trajectory_msgs.msg import JointTrajectory

class TrajectoryDemo():
    def __init__(self):
        #filename = '/home/kaiser/WS_ROS/catkin_ws/src/abb_experimental-indigo-devel/abb_irb120_gazebo/urdf/modelo_exportado.urdf'
        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/IRB120_URDF_v2/robots/IRB120_URDF_v2.URDF'
        robot = urdf_parser_py.urdf.URDF.from_xml_file(filename)

        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        chain = tree.getChain("base_link", "link_6")
        print chain.getNrOfSegments()
        solverCinematicaDirecta = PyKDL.ChainFkSolverPos_recursive(chain)
        nj_izq = chain.getNrOfJoints()
        posicionInicial = PyKDL.JntArray(nj_izq)
        posicionInicial[0] = 0
        posicionInicial[1] = 0
        posicionInicial[2] = 0
        posicionInicial[3] = 0
        posicionInicial[4] = 0
        posicionInicial[5] = 0
        print "Forward kinematics"
        finalFrame = PyKDL.Frame()
        solverCinematicaDirecta.JntToCart(posicionInicial, finalFrame)
        print "Rotational Matrix of the final Frame: "
        print  finalFrame.M
        print "End-effector position: ", finalFrame.p

        rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        arm_joints = ['joint_1',
                      'joint_2',
                      'joint_3',
                      'joint_4',
                      'joint_5',
                      'joint_6']


        if reset:
            # Set the arm back to the resting position
            arm_goal  = [0, 0, 0, 0, 0, 0]

        else:
            # Set a goal configuration for the arm
            arm_goal  = [-0.3, 0.5, -1.0, 1.8, 0.3, 0.6]

        print "Inverse Kinematics"
        q_init = posicionInicial  # initial angles
        vik = PyKDL.ChainIkSolverVel_pinv(chain)
        ik = PyKDL.ChainIkSolverPos_NR(chain, solverCinematicaDirecta, vik)
        #desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame = finalFrame
        desiredFrame.p[0] = finalFrame.p[0]
        desiredFrame.p[1] = finalFrame.p[1]
        desiredFrame.p[2] = finalFrame.p[2]-0.25
        print "Desired Position: ", desiredFrame.p
        q_out = PyKDL.JntArray(6)
        ik.CartToJnt(q_init, desiredFrame, q_out)
        print "Output angles in rads: ", q_out

        arm_goal[0] = q_out[0]
        arm_goal[1] = q_out[1]
        arm_goal[2] = q_out[2]
        arm_goal[3] = q_out[3]
        arm_goal[4] = q_out[4]
        arm_goal[5] = q_out[5]


        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        pub = rospy.Publisher('joint_path_command',JointTrajectory,queue_size=10)

        #arm_client = actionlib.SimpleActionClient('joint_trajectory_action', FollowJointTrajectoryAction)

        #arm_client.wait_for_server()

        rospy.loginfo('...connected.')


        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(2.0)

        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')

        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()

        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)

        # Send the goal to the action server
        #arm_client.send_goal(arm_goal)

        pub.publish(arm_trajectory)

        while not rospy.is_shutdown():
            pub.publish(arm_trajectory)
            rospy.sleep(rospy.Duration(0.5))

        #if not sync:
            # Wait for up to 5 seconds for the motion to complete
            #arm_client.wait_for_result(rospy.Duration(5.0))


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import actionlib
import rospy
from scipy.io import loadmat

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL
import copy
from angles import normalize

import pickle

class TrajectoryDemo():
    def __init__(self):
        porcentaje = 0.7#0.5
        porcentaje1 = 0.7 #1
        porcentaje16 = 0.8 # 1
        des6 = 0.1 # 0.15

        giro = 0
        x = loadmat('estruc_piernas_fase3.mat')
        # print x
        piernas_der = x['estruc_piernaDer_fase']
        art1_der = piernas_der[0, 0]
        art2_der = piernas_der[0, 1]
        art3_der = piernas_der[0, 2]
        art4_der = piernas_der[0, 3]
        art5_der = piernas_der[0, 4]
        art6_der = piernas_der[0, 5]

        piernas_izq = x['estruc_piernaIzq_fase']
        art1_izq = piernas_izq[0, 0]
        art2_izq = piernas_izq[0, 1]
        art3_izq = piernas_izq[0, 2]
        art4_izq = piernas_izq[0, 3]
        art5_izq = piernas_izq[0, 4]
        art6_izq = piernas_izq[0, 5]

        rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        piernas_joints = ['cilinder_blue_box1_der_joint', 'cilinder_blue1_box2_der_joint','cilinder_blue_box2_der_joint',
                          'cilinder_blue_box4_der_joint', 'cilinder_blue1_box6_der_joint','cilinder_blue_box6_der_joint',
                          'cilinder_blue_box1_izq_joint', 'cilinder_blue1_box2_izq_joint','cilinder_blue_box2_izq_joint',
                          'cilinder_blue_box4_izq_joint', 'cilinder_blue1_box6_izq_joint','cilinder_blue_box6_izq_joint']



        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        arm_client = actionlib.SimpleActionClient('/piernas/piernas_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        #/piernas/piernas_controller/follow_joint_trajectory
        arm_client.wait_for_server()

        rospy.loginfo('...connected.')

        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = piernas_joints

        # piernas_goalF0 = [0, -0.3, -0.4, 0.8, -0.3, -0.4,
        #                   0, -0.4, -0.3, 0.6, +0.4, -0.3]
        # arm_trajectory.points.append(JointTrajectoryPoint())
        # arm_trajectory.points[0].positions = piernas_goalF0
        # arm_trajectory.points[0].velocities = [0.0 for k in piernas_goalF0]
        # arm_trajectory.points[0].accelerations = [0.0 for k in piernas_goalF0]
        # arm_trajectory.points[0].time_from_start = rospy.Duration(2)

        piernas_goalF01 = [giro* porcentaje*art1_der['senialCompleta'][0, 50], 0.8*porcentaje*art2_der['senialCompleta'][0, 50],porcentaje1*art3_der['senialCompleta'][0, 50] ,
                           porcentaje1 *art4_der['senialCompleta'][0, 50] , 0.8*porcentaje*art5_der['senialCompleta'][0, 50],porcentaje16*art6_der['senialCompleta'][0, 50] +des6,
                           giro* porcentaje*art1_izq['senialCompleta'][0, 50], 0.8*porcentaje*art2_izq['senialCompleta'][0, 50],porcentaje1*art3_izq['senialCompleta'][0, 50] ,
                           porcentaje1 *art4_izq['senialCompleta'][0, 50] , 0.8*porcentaje*art5_izq['senialCompleta'][0, 50],porcentaje16*art6_izq['senialCompleta'][0, 50] +des6]
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = piernas_goalF01
        arm_trajectory.points[0].velocities = [0.0 for k in piernas_goalF01]
        arm_trajectory.points[0].accelerations = [0.0 for k in piernas_goalF01]
        arm_trajectory.points[0].time_from_start = rospy.Duration(2)

        ind = 1
        salto =4
        for i in range(50, int(art1_der['senialCompleta'].shape[1]*0.25), salto):
            # print ind
            # print int(art1_der['senialCompleta'].shape[1]*0.2)
            piernas_goalF = [giro*porcentaje*art1_der['senialCompleta'][0, i], 0.8*porcentaje*art2_der['senialCompleta'][0, i], porcentaje1*art3_der['senialCompleta'][0, i],
                             porcentaje1 *art4_der['senialCompleta'][0, i], 0.8*porcentaje*art5_der['senialCompleta'][0, i], porcentaje16*art6_der['senialCompleta'][0, i]+des6,
                             giro*porcentaje*art1_izq['senialCompleta'][0, i], 0.8*porcentaje*art2_izq['senialCompleta'][0, i], porcentaje1*art3_izq['senialCompleta'][0, i],
                             porcentaje1 *art4_izq['senialCompleta'][0, i], 0.8*porcentaje*art5_izq['senialCompleta'][0, i], porcentaje16*art6_izq['senialCompleta'][0, i]+des6]
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[ind].positions = piernas_goalF
            arm_trajectory.points[ind].velocities = [0.0 for j in piernas_goalF]
            arm_trajectory.points[ind].accelerations = [0.0 for j in piernas_goalF]
            # art1_der['tiempo_completo'][0, i]
            # 0.01 * (ind + 1)
            arm_trajectory.points[ind].time_from_start = rospy.Duration( 2+(porcentaje1*art1_der['tiempo_completo'][0, i])*1.5)
            # print 1+art1_der['tiempo_completo'][0, i]
            ind+=1
            # print art1_der['senialCompleta'][0, i]



        # Create an empty trajectory goal
        piernas_goal = FollowJointTrajectoryGoal()

        # Set the trajectory component to the goal trajectory created above
        piernas_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        piernas_goal.goal_time_tolerance = rospy.Duration(0.001)

        # Send the goal to the action server
        arm_client.send_goal(piernas_goal)

        if not sync:
            # Wait for up to 5 seconds for the motion to complete
            arm_client.wait_for_result(rospy.Duration(5.0))

        arm_client.wait_for_result()
        # arm_client.send_goal(piernas_goal)
        print arm_client.get_result()


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

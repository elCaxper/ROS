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
        porcentaje = 0.7
        porcentaje1 = 0.7
        porcentaje16 = 0.8
        des6 = 0.1

        # porcentaje = 0.5497  # 0.5
        # porcentaje1 = 0.5497  # 1
        # porcentaje16 = 0.6497  # 1
        # des6 = 0.1  # 0.15

        giro = 1
        girod = 0#1
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

        hacia_delante = JointTrajectory()
        hacia_delante.joint_names = piernas_joints

        # piernas_goalF0 = [0, -0.3, -0.4, 0.8, -0.3, -0.4,
        #                   0, -0.4, -0.3, 0.6, +0.4, -0.3]
        # hacia_delante.points.append(JointTrajectoryPoint())
        # hacia_delante.points[0].positions = piernas_goalF0
        # hacia_delante.points[0].velocities = [0.0 for k in piernas_goalF0]
        # hacia_delante.points[0].accelerations = [0.0 for k in piernas_goalF0]
        # hacia_delante.points[0].time_from_start = rospy.Duration(2)
        ind_de_inicio = 100
        piernas_goalF01 = [giro* porcentaje*art1_der['senialCompleta'][0, ind_de_inicio], 0.4*porcentaje*art2_der['senialCompleta'][0, ind_de_inicio],porcentaje1*art3_der['senialCompleta'][0, ind_de_inicio] ,
                           porcentaje1 *art4_der['senialCompleta'][0, ind_de_inicio] , 0.4*porcentaje*art5_der['senialCompleta'][0, ind_de_inicio],porcentaje16*art6_der['senialCompleta'][0, ind_de_inicio] +des6,
                           giro* porcentaje*art1_izq['senialCompleta'][0, ind_de_inicio], 0.4*porcentaje*art2_izq['senialCompleta'][0, ind_de_inicio],porcentaje1*art3_izq['senialCompleta'][0, ind_de_inicio] ,
                           porcentaje1 *art4_izq['senialCompleta'][0, ind_de_inicio] , 0.4*porcentaje*art5_izq['senialCompleta'][0, ind_de_inicio],porcentaje16*art6_izq['senialCompleta'][0, ind_de_inicio] +des6]
        hacia_delante.points.append(JointTrajectoryPoint())
        hacia_delante.points[0].positions = piernas_goalF01
        hacia_delante.points[0].velocities = [0.0 for k in piernas_goalF01]
        hacia_delante.points[0].accelerations = [0.0 for k in piernas_goalF01]
        hacia_delante.points[0].time_from_start = rospy.Duration(1.5)

        ind = 1
        salto =4
        for i in range(ind_de_inicio, int(art1_der['senialCompleta'].shape[1]*0.08), salto):
            # print ind
            # print int(art1_der['senialCompleta'].shape[1]*0.2)
            piernas_goalF = [giro*porcentaje*art1_der['senialCompleta'][0, i], 0.4*porcentaje*art2_der['senialCompleta'][0, i], porcentaje1*art3_der['senialCompleta'][0, i],
                             porcentaje1 *art4_der['senialCompleta'][0, i], 0.4*porcentaje*art5_der['senialCompleta'][0, i], porcentaje16*art6_der['senialCompleta'][0, i]+des6,
                             giro*porcentaje*art1_izq['senialCompleta'][0, i], 0.4*porcentaje*art2_izq['senialCompleta'][0, i], porcentaje1*art3_izq['senialCompleta'][0, i],
                             porcentaje1 *art4_izq['senialCompleta'][0, i], 0.4*porcentaje*art5_izq['senialCompleta'][0, i], porcentaje16*art6_izq['senialCompleta'][0, i]+des6]
            hacia_delante.points.append(JointTrajectoryPoint())
            hacia_delante.points[ind].positions = piernas_goalF
            hacia_delante.points[ind].velocities = [0.0 for j in piernas_goalF]
            hacia_delante.points[ind].accelerations = [0.0 for j in piernas_goalF]
            # art1_der['tiempo_completo'][0, i]
            # 0.01 * (ind + 1)
            hacia_delante.points[ind].time_from_start = rospy.Duration( 1.5+porcentaje1*art1_der['tiempo_completo'][0, i]*0.4)
            # print 1+art1_der['tiempo_completo'][0, i]
            ind+=1
            # print art1_der['senialCompleta'][0, i]



        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = piernas_joints

        porcentajeg = porcentaje
        girod = 1  # 1
        long_atras = 0.9
        piernas_goalF01 = [-girod*porcentajeg * art1_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           porcentajeg*0.6 * art2_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           long_atras*porcentajeg*art3_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           long_atras*porcentajeg*art4_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           porcentajeg*0.6 * art5_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)] + 0.015,
                           long_atras*porcentajeg*art6_der['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)] - 0.03,
                           -girod*porcentajeg * art1_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           porcentajeg*0.6 * art2_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           long_atras*porcentajeg*art3_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           long_atras*porcentajeg*art4_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)],
                           porcentajeg*0.6 * art5_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)] - 0.03,
                           long_atras*porcentajeg*art6_izq['senialCompleta'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93)] - 0.03]
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = piernas_goalF01
        # arm_trajectory.points[0].velocities = [0.0 for k in piernas_goalF01]
        # arm_trajectory.points[0].accelerations = [0.0 for k in piernas_goalF01]
        arm_trajectory.points[0].time_from_start = rospy.Duration(1)

        ind = 1;
        salto = 5;
        for i in range(int((art1_der['senialCompleta'].shape[1] - 1)*0.93), int(art1_der['senialCompleta'].shape[1] * 0.89), #0.93
                       -salto):
            # print i
            # print int(art1_der['senialCompleta'].shape[1]*0.2)
            piernas_goalF = [-girod*porcentajeg * art1_der['senialCompleta'][0, i], 0.6 *porcentajeg* art2_der['senialCompleta'][0, i],long_atras*porcentajeg*art3_der['senialCompleta'][0, i],
                             long_atras*porcentajeg*art4_der['senialCompleta'][0, i], 0.6 *porcentajeg* art5_der['senialCompleta'][0, i] + 0.015,long_atras*porcentajeg*art6_der['senialCompleta'][0, i] - 0.03,
                             -girod*porcentajeg * art1_izq['senialCompleta'][0, i], 0.6 *porcentajeg* art2_izq['senialCompleta'][0, i],long_atras*porcentajeg*art3_izq['senialCompleta'][0, i],
                             long_atras*porcentajeg*art4_izq['senialCompleta'][0, i], 0.6 *porcentajeg* art5_izq['senialCompleta'][0, i] - 0.03,long_atras*porcentajeg*art6_izq['senialCompleta'][0, i] - 0.03]
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[ind].positions = piernas_goalF
            arm_trajectory.points[ind].velocities = [0.0 for j in piernas_goalF]
            arm_trajectory.points[ind].accelerations = [0.0 for j in piernas_goalF]
            # art1_der['tiempo_completo'][0, i]
            # 0.01 * (ind + 1)
            arm_trajectory.points[ind].time_from_start = rospy.Duration(
                1.1 + (art1_der['tiempo_completo'][0, int((art1_der['senialCompleta'].shape[1] - 1)*0.93) - i])*0.4)
            # print 1+art1_der['tiempo_completo'][0, i]
            ind += 1
            # print art1_der['senialCompleta'][0, i]


        reposo = JointTrajectory()
        reposo.joint_names = piernas_joints
        piernas_goalF01 = [0,0,0,0,0,0,0,0,0,0,0,0]
        reposo.points.append(JointTrajectoryPoint())
        reposo.points[0].positions = piernas_goalF01
        reposo.points[0].velocities = [0.0 for k in piernas_goalF01]
        reposo.points[0].accelerations = [0.0 for k in piernas_goalF01]
        reposo.points[0].time_from_start = rospy.Duration(2)


        # Create an empty trajectory goal
        piernas_goal = FollowJointTrajectoryGoal()
        piernas_goal_atras = FollowJointTrajectoryGoal()
        piernas_reposo = FollowJointTrajectoryGoal()
        # Set the trajectory component to the goal trajectory created above
        piernas_goal.trajectory = hacia_delante
        piernas_goal_atras.trajectory = arm_trajectory
        piernas_reposo.trajectory = reposo
        # piernas_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        piernas_goal.goal_time_tolerance = rospy.Duration(0.001)
        piernas_goal_atras.goal_time_tolerance = rospy.Duration(0.001)
        piernas_reposo.goal_time_tolerance = rospy.Duration(0.001)

        # Send the goal to the action server
        arm_client.send_goal(piernas_goal)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal_atras)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal_atras)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal_atras)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal)
        arm_client.wait_for_result()
        arm_client.send_goal(piernas_goal_atras)
        arm_client.wait_for_result()

        # arm_client.send_goal(piernas_goal)
        print arm_client.get_result()
        arm_client.send_goal(piernas_reposo)

        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

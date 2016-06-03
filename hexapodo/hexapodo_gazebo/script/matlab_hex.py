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

        x = loadmat('estruc_patas_rec2.mat')
        # print x
        piernas_der = x['estruc_patas_rec']
        piernaLF = piernas_der[0, 0]
        piernaLM = piernas_der[0, 1]
        piernaLR = piernas_der[0, 2]
        piernaRF = piernas_der[0, 3]
        piernaRM = piernas_der[0, 4]
        piernaRR = piernas_der[0, 5]

        rospy.init_node('trajectory_hex')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        piernas_joints_RR = ['coxa_joint_RR', 'femur_joint_RR', 'tibia_joint_RR', 'tarsus_joint_RR']
        piernas_joints_LM = ['coxa_joint_LM', 'femur_joint_LM', 'tibia_joint_LM', 'tarsus_joint_LM']
        piernas_joints_RF = ['coxa_joint_RF', 'femur_joint_RF', 'tibia_joint_RF', 'tarsus_joint_RF']

        piernas_joints_LR = ['coxa_joint_LR', 'femur_joint_LR', 'tibia_joint_LR', 'tarsus_joint_LR']
        piernas_joints_RM = ['coxa_joint_RM', 'femur_joint_RM', 'tibia_joint_RM', 'tarsus_joint_RM']
        piernas_joints_LF = ['coxa_joint_LF', 'femur_joint_LF', 'tibia_joint_LF', 'tarsus_joint_LF']

        rospy.loginfo('Waiting for right arm trajectory controller...')

        rr_client = actionlib.SimpleActionClient('/hexapodo/pata_rr/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        rr_client.wait_for_server()

        lm_client = actionlib.SimpleActionClient('/hexapodo/pata_lm/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        lm_client.wait_for_server()

        rf_client = actionlib.SimpleActionClient('/hexapodo/pata_rf/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        rf_client.wait_for_server()

        lr_client = actionlib.SimpleActionClient('/hexapodo/pata_lr/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        lr_client.wait_for_server()

        rm_client = actionlib.SimpleActionClient('/hexapodo/pata_rm/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        rm_client.wait_for_server()

        lf_client = actionlib.SimpleActionClient('/hexapodo/pata_lf/follow_joint_trajectory',
                                                 FollowJointTrajectoryAction)
        lf_client.wait_for_server()

        rospy.loginfo('...connected.')

        rr_trajectory1 = JointTrajectory()
        rr_trajectory1.joint_names = piernas_joints_RR
        lm_trajectory1 = JointTrajectory()
        lm_trajectory1.joint_names = piernas_joints_LM
        rf_trajectory1 = JointTrajectory()
        rf_trajectory1.joint_names = piernas_joints_RF
        lr_trajectory1 = JointTrajectory()
        lr_trajectory1.joint_names = piernas_joints_LR
        rm_trajectory1 = JointTrajectory()
        rm_trajectory1.joint_names = piernas_joints_RM
        lf_trajectory1 = JointTrajectory()
        lf_trajectory1.joint_names = piernas_joints_LF

        # piernas_goalF0 = [0, -0.3, -0.4, 0.8, -0.3, -0.4,
        #                   0, -0.4, -0.3, 0.6, +0.4, -0.3]
        # arm_trajectory.points.append(JointTrajectoryPoint())
        # arm_trajectory.points[0].positions = piernas_goalF0
        # arm_trajectory.points[0].velocities = [0.0 for k in piernas_goalF0]
        # arm_trajectory.points[0].accelerations = [0.0 for k in piernas_goalF0]
        # arm_trajectory.points[0].time_from_start = rospy.Duration(2)

        piernasLF_goalF01 = [ piernaLF['nuevo_coxa2'][0, 0], piernaLF['nuevo_femur'][0, 0],piernaLF['nuevo_tibia'][0, 0] ,
                              piernaLF['nuevo_tarsus'][0, 0] ]
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[0].positions = piernasLF_goalF01
        lf_trajectory1.points[0].time_from_start = rospy.Duration(1)

        piernasLM_goalF01 = [piernaLM['nuevo_coxa2'][0, 0], piernaLM['nuevo_femur'][0, 0],
                             piernaLM['nuevo_tibia'][0, 0],piernaLM['nuevo_tarsus'][0, 0]]
        lm_trajectory1.points.append(JointTrajectoryPoint())
        lm_trajectory1.points[0].positions = piernasLM_goalF01
        lm_trajectory1.points[0].time_from_start = rospy.Duration(1)

        piernasLR_goalF01 = [piernaLR['nuevo_coxa2'][0, 0], piernaLR['nuevo_femur'][0, 0], piernaLR['nuevo_tibia'][0, 0],
                             piernaLR['nuevo_tarsus'][0, 0]]
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[0].positions = piernasLR_goalF01
        lr_trajectory1.points[0].time_from_start = rospy.Duration(1)

        piernasRF_goalF01 = [piernaRF['nuevo_coxa2'][0, 0], piernaRF['nuevo_femur'][0, 0], piernaRF['nuevo_tibia'][0, 0],
                             piernaRF['nuevo_tarsus'][0, 0]]
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[0].positions = piernasRF_goalF01
        rf_trajectory1.points[0].time_from_start = rospy.Duration(1)

        piernasRM_goalF01 = [piernaRM['nuevo_coxa2'][0, 0], piernaRM['nuevo_femur'][0, 0], piernaRM['nuevo_tibia'][0, 0],
                             piernaRM['nuevo_tarsus'][0, 0]]
        rm_trajectory1.points.append(JointTrajectoryPoint())
        rm_trajectory1.points[0].positions = piernasRM_goalF01
        rm_trajectory1.points[0].time_from_start = rospy.Duration(1)

        piernasRR_goalF01 = [piernaRR['nuevo_coxa2'][0, 0], piernaRR['nuevo_femur'][0, 0], piernaRR['nuevo_tibia'][0, 0],
                             piernaRR['nuevo_tarsus'][0, 0]]
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[0].positions = piernasRR_goalF01
        rr_trajectory1.points[0].time_from_start = rospy.Duration(1)




        ind = 1
        salto =5
        for i in range(1, int(piernaRR['nuevo_tarsus'].shape[0]*0.3), salto):
            piernasLF_goalF01 = [piernaLF['nuevo_coxa2'][i,0], piernaLF['nuevo_femur'][i,0],
                                 piernaLF['nuevo_tibia'][i,0],piernaLF['nuevo_tarsus'][i,0]]
            lf_trajectory1.points.append(JointTrajectoryPoint())
            lf_trajectory1.points[ind].positions = piernasLF_goalF01
            lf_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            lf_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]

            piernasLM_goalF01 = [piernaLM['nuevo_coxa2'][i,0], piernaLM['nuevo_femur'][i,0],
                                 piernaLM['nuevo_tibia'][i,0], piernaLM['nuevo_tarsus'][i,0]]
            lm_trajectory1.points.append(JointTrajectoryPoint())
            lm_trajectory1.points[ind].positions = piernasLM_goalF01
            lm_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            lm_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]

            piernasLR_goalF01 = [piernaLR['nuevo_coxa2'][i,0], piernaLR['nuevo_femur'][i,0],
                                 piernaLR['nuevo_tibia'][i,0],piernaLR['nuevo_tarsus'][i,0]]
            lr_trajectory1.points.append(JointTrajectoryPoint())
            lr_trajectory1.points[ind].positions = piernasLR_goalF01
            lr_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            lr_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]

            piernasRF_goalF01 = [piernaRF['nuevo_coxa2'][i,0], piernaRF['nuevo_femur'][i,0],
                                 piernaRF['nuevo_tibia'][i,0], piernaRF['nuevo_tarsus'][i,0]]
            rf_trajectory1.points.append(JointTrajectoryPoint())
            rf_trajectory1.points[ind].positions = piernasRF_goalF01
            rf_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            rf_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]

            piernasRM_goalF01 = [piernaRM['nuevo_coxa2'][i,0], piernaRM['nuevo_femur'][i,0],
                                 piernaRM['nuevo_tibia'][i,0],piernaRM['nuevo_tarsus'][i,0]]
            rm_trajectory1.points.append(JointTrajectoryPoint())
            rm_trajectory1.points[ind].positions = piernasRM_goalF01
            rm_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            rm_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]

            piernasRR_goalF01 = [piernaRR['nuevo_coxa2'][i,0], piernaRR['nuevo_femur'][i,0],
                                 piernaRR['nuevo_tibia'][i,0],piernaRR['nuevo_tarsus'][i,0]]
            rr_trajectory1.points.append(JointTrajectoryPoint())
            rr_trajectory1.points[ind].positions = piernasRR_goalF01
            rr_trajectory1.points[ind].velocities = [0.0 for k in piernasLF_goalF01]
            rr_trajectory1.points[ind].accelerations = [0.0 for k in piernasLF_goalF01]
            # print ind
            # print int(art1_der['senialCompleta'].shape[1]*0.2)
            # arm_trajectory.points[ind].velocities = [0.0 for j in piernas_goalF]
            # arm_trajectory.points[ind].accelerations = [0.0 for j in piernas_goalF]
            # art1_der['tiempo_completo'][0, i]
            # 0.01 * (ind + 1)
            lf_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaLF['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            lm_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaLM['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            lr_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaLR['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            rf_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaRF['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            rm_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaRM['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            rr_trajectory1.points[ind].time_from_start = rospy.Duration(1 + piernaRR['tiempo'][i, 0]-piernaLF['tiempo'][0, 0])
            # print 1+art1_der['tiempo_completo'][0, i]
            ind+=1
            # print art1_der['senialCompleta'][0, i]

        print int(piernaRR['nuevo_tarsus'].shape[0])

        # Create an empty trajectory goal
        rospy.loginfo('Moving the arm to goal position...')

        # Create an empty trajectory goal
        rr_goal = FollowJointTrajectoryGoal()
        lm_goal = FollowJointTrajectoryGoal()
        rf_goal = FollowJointTrajectoryGoal()

        lr_goal = FollowJointTrajectoryGoal()
        rm_goal = FollowJointTrajectoryGoal()
        lf_goal = FollowJointTrajectoryGoal()
        # Set the trajectory component to the goal trajectory created above
        rr_goal.trajectory = rr_trajectory1
        lm_goal.trajectory = lm_trajectory1
        rf_goal.trajectory = rf_trajectory1
        lr_goal.trajectory = lr_trajectory1
        rm_goal.trajectory = rm_trajectory1
        lf_goal.trajectory = lf_trajectory1
        # Specify zero tolerance for the execution time
        rr_goal.goal_time_tolerance = rospy.Duration(0.01)
        lm_goal.goal_time_tolerance = rospy.Duration(0.01)
        rf_goal.goal_time_tolerance = rospy.Duration(0.01)
        lr_goal.goal_time_tolerance = rospy.Duration(0.01)
        rm_goal.goal_time_tolerance = rospy.Duration(0.01)
        lf_goal.goal_time_tolerance = rospy.Duration(0.01)

        # Send the goal to the action server
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)


        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        rr_client.wait_for_result()

        '''rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)

        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        rr_client.wait_for_result()

        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        rr_client.wait_for_result()

        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)

        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        rr_client.wait_for_result()
        #'''

        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

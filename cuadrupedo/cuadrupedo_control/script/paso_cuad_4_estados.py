#!/usr/bin/env python

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL
import hrpsys.ModelLoader_idl
import copy
from angles import normalize
import pickle

class TrajectoryDemo():
    def __init__(self):

        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/mi_cuadrupedo_exp.urdf'

        altura_pata = 0.05 #cm
        avance_x = 0.04
        # altura_pata = 0  # -0.04 #cm
        # avance_x = 0
        robot = urdf_parser_py.urdf.URDF.from_xml_file(filename)

        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        cadena_RR = tree.getChain("base_link", "tarsus_RR")
        cadena_RF = tree.getChain("base_link", "tarsus_RF")
        cadena_LR = tree.getChain("base_link", "tarsus_LR")
        cadena_LF = tree.getChain("base_link", "tarsus_LF")

        print cadena_RR.getNrOfSegments()
        fksolver_RR = PyKDL.ChainFkSolverPos_recursive(cadena_RR)
        fksolver_RF = PyKDL.ChainFkSolverPos_recursive(cadena_RF)
        fksolver_LR = PyKDL.ChainFkSolverPos_recursive(cadena_LR)
        fksolver_LF = PyKDL.ChainFkSolverPos_recursive(cadena_LF)

        vik_RR = PyKDL.ChainIkSolverVel_pinv(cadena_RR)
        ik_RR = PyKDL.ChainIkSolverPos_NR(cadena_RR, fksolver_RR, vik_RR)

        vik_RF = PyKDL.ChainIkSolverVel_pinv(cadena_RF)
        ik_RF = PyKDL.ChainIkSolverPos_NR(cadena_RF, fksolver_RF, vik_RF)

        vik_LR = PyKDL.ChainIkSolverVel_pinv(cadena_LR)
        ik_LR = PyKDL.ChainIkSolverPos_NR(cadena_LR, fksolver_LR, vik_LR)

        vik_LF = PyKDL.ChainIkSolverVel_pinv(cadena_LF)
        ik_LF = PyKDL.ChainIkSolverPos_NR(cadena_LF, fksolver_LF, vik_LF)

        nj_izq = cadena_RR.getNrOfJoints()
        posicionInicial_R = PyKDL.JntArray(nj_izq)
        posicionInicial_R[0] = 0
        posicionInicial_R[1] = 0
        posicionInicial_R[2] = 0
        posicionInicial_R[3] = 0

        nj_izq = cadena_LR.getNrOfJoints()
        posicionInicial_L = PyKDL.JntArray(nj_izq)
        posicionInicial_L[0] = 0
        posicionInicial_L[1] = 0
        posicionInicial_L[2] = 0
        posicionInicial_L[3] = 0

        print "Forward kinematics"
        finalFrame_R = PyKDL.Frame()
        finalFrame_L = PyKDL.Frame()

        rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        piernas_joints_RR = ['coxa_joint_RR', 'femur_joint_RR','tibia_joint_RR', 'tarsus_joint_RR']
        piernas_joints_RF = ['coxa_joint_RF', 'femur_joint_RF','tibia_joint_RF', 'tarsus_joint_RF']

        piernas_joints_LR = ['coxa_joint_LR', 'femur_joint_LR', 'tibia_joint_LR', 'tarsus_joint_LR']
        piernas_joints_LF = ['coxa_joint_LF', 'femur_joint_LF', 'tibia_joint_LF', 'tarsus_joint_LF']

        rr_goal0 = [0.0, 0.0, 0.0, 0.0]
        rf_goal0 = [0.0, 0.0, 0.0, 0.0]
        rr_goal1 = [0.0, 0.0, 0.0, 0.0]
        rf_goal1 = [0.0, 0.0, 0.0, 0.0]
        lr_goal0 = [0.0, 0.0, 0.0, 0.0]
        lf_goal0 = [0.0, 0.0, 0.0, 0.0]
        lr_goal1 = [0.0, 0.0, 0.0, 0.0]
        lf_goal1 = [0.0, 0.0, 0.0, 0.0]
        rr_goal2 = [0.0, 0.0, 0.0, 0.0]
        rr_goal3 = [0.0, 0.0, 0.0, 0.0]
        rr_goal4 = [0.0, 0.0, 0.0, 0.0]
        rr_goal5 = [0.0, 0.0, 0.0, 0.0]
        rr_goal6 = [0.0, 0.0, 0.0, 0.0]
        rr_goal7 = [0.0, 0.0, 0.0, 0.0]
        lf_goal2 = [0.0, 0.0, 0.0, 0.0]
        lf_goal3 = [0.0, 0.0, 0.0, 0.0]
        lf_goal4 = [0.0, 0.0, 0.0, 0.0]
        lf_goal5 = [0.0, 0.0, 0.0, 0.0]
        lf_goal6 = [0.0, 0.0, 0.0, 0.0]
        lf_goal7 = [0.0, 0.0, 0.0, 0.0]

        #11111111111111111111111111111111111111111111
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(posicionInicial_R, finalFrame_R)
        q_init_RR = posicionInicial_R  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] +avance_x
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]+altura_pata
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal0[0] = q_out_RR[0]
        rr_goal0[1] = q_out_RR[1] #+angulo_avance/2
        rr_goal0[2] = q_out_RR[2]
        rr_goal0[3] = q_out_RR[3] #- angulo_avance/2

        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal0[0]
        RR_actual[1] = rr_goal0[1]
        RR_actual[2] = rr_goal0[2]
        RR_actual[3] = rr_goal0[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] #- avance_x
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2] - altura_pata
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal1[0] = q_out_RR[0]
        rr_goal1[1] = q_out_RR[1] #+ angulo_avance / 2
        rr_goal1[2] = q_out_RR[2]
        rr_goal1[3] = q_out_RR[3] #- angulo_avance/2


        print "Inverse Kinematics"
        pos_ini_izq = PyKDL.JntArray(nj_izq)
        pos_ini_izq[0] = +rr_goal1[0]
        pos_ini_izq[1] = -rr_goal1[1]
        pos_ini_izq[2] = -rr_goal1[2]
        pos_ini_izq[3] = rr_goal1[3]
        fksolver_LF.JntToCart(pos_ini_izq, finalFrame_L)
        q_init_LF = pos_ini_izq  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal0[0] = q_out_LF[0]
        lf_goal0[1] = q_out_LF[1] #+ angulo_avance/6
        lf_goal0[2] = q_out_LF[2]
        lf_goal0[3] = q_out_LF[3] #- angulo_avance / 6

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal0[0]
        LF_actual[1] = lf_goal0[1]
        LF_actual[2] = lf_goal0[2]
        LF_actual[3] = lf_goal0[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal1[0] = q_out_LF[0]
        lf_goal1[1] = q_out_LF[1] #+ angulo_avance/6
        lf_goal1[2] = q_out_LF[2]
        lf_goal1[3] = q_out_LF[3] #- angulo_avance / 6

        # 22222222222222222222222222222222222222222222

        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal1[0]
        RR_actual[1] = rr_goal1[1]
        RR_actual[2] = rr_goal1[2]
        RR_actual[3] = rr_goal1[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal2[0] = q_out_RR[0]
        rr_goal2[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal2[2] = q_out_RR[2]
        rr_goal2[3] = q_out_RR[3] #+ angulo_avance / 6

        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal2[0]
        RR_actual[1] = rr_goal2[1]
        RR_actual[2] = rr_goal2[2]
        RR_actual[3] = rr_goal2[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal3[0] = q_out_RR[0]
        rr_goal3[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal3[2] = q_out_RR[2]
        rr_goal3[3] = q_out_RR[3] #+ angulo_avance / 6

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal1[0]
        LF_actual[1] = lf_goal1[1]
        LF_actual[2] = lf_goal1[2]
        LF_actual[3] = lf_goal1[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal2[0] = q_out_LF[0]
        lf_goal2[1] = q_out_LF[1] #+ angulo_avance/6
        lf_goal2[2] = q_out_LF[2]
        lf_goal2[3] = q_out_LF[3] #- angulo_avance / 6

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal2[0]
        LF_actual[1] = lf_goal2[1]
        LF_actual[2] = lf_goal2[2]
        LF_actual[3] = lf_goal2[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal3[0] = q_out_LF[0]
        lf_goal3[1] = q_out_LF[1] #+ angulo_avance / 6
        lf_goal3[2] = q_out_LF[2]
        lf_goal3[3] = q_out_LF[3] #- angulo_avance / 6


        #3333333333333333333333333333333333333333333333333



        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal3[0]
        RR_actual[1] = rr_goal3[1]
        RR_actual[2] = rr_goal3[2]
        RR_actual[3] = rr_goal3[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal4[0] = q_out_RR[0]
        rr_goal4[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal4[2] = q_out_RR[2]
        rr_goal4[3] = q_out_RR[3] #+ angulo_avance / 6



        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal4[0]
        RR_actual[1] = rr_goal4[1]
        RR_actual[2] = rr_goal4[2]
        RR_actual[3] = rr_goal4[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal5[0] = q_out_RR[0]
        rr_goal5[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal5[2] = q_out_RR[2]
        rr_goal5[3] = q_out_RR[3] #+ angulo_avance / 6


        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal3[0]
        LF_actual[1] = lf_goal3[1]
        LF_actual[2] = lf_goal3[2]
        LF_actual[3] = lf_goal3[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal4[0] = q_out_LF[0]
        lf_goal4[1] = q_out_LF[1] #+ angulo_avance / 6
        lf_goal4[2] = q_out_LF[2]
        lf_goal4[3] = q_out_LF[3] #- angulo_avance / 6

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal4[0]
        LF_actual[1] = lf_goal4[1]
        LF_actual[2] = lf_goal4[2]
        LF_actual[3] = lf_goal4[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] - avance_x/3
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal5[0] = q_out_LF[0]
        lf_goal5[1] = q_out_LF[1] #+ angulo_avance / 6
        lf_goal5[2] = q_out_LF[2]
        lf_goal5[3] = q_out_LF[3] #- angulo_avance / 6

        # 4444444444444444444444444444444444444444444444444

        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal5[0]
        RR_actual[1] = rr_goal5[1]
        RR_actual[2] = rr_goal5[2]
        RR_actual[3] = rr_goal5[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal6[0] = q_out_RR[0]
        rr_goal6[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal6[2] = q_out_RR[2]
        rr_goal6[3] = q_out_RR[3] #+ angulo_avance / 6


        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal6[0]
        RR_actual[1] = rr_goal6[1]
        RR_actual[2] = rr_goal6[2]
        RR_actual[3] = rr_goal6[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0] - avance_x/3
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal7[0] = q_out_RR[0]
        rr_goal7[1] = q_out_RR[1] #- angulo_avance / 6
        rr_goal7[2] = q_out_RR[2]
        rr_goal7[3] = q_out_RR[3] #+ angulo_avance / 6



        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal5[0]
        LF_actual[1] = lf_goal5[1]
        LF_actual[2] = lf_goal5[2]
        LF_actual[3] = lf_goal5[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0] + 2*avance_x
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2] + altura_pata
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal6[0] = q_out_LF[0]
        lf_goal6[1] = q_out_LF[1] #- angulo_avance / 2
        lf_goal6[2] = q_out_LF[2]
        lf_goal6[3] = q_out_LF[3] #+angulo_avance/2

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal6[0]
        LF_actual[1] = lf_goal6[1]
        LF_actual[2] = lf_goal6[2]
        LF_actual[3] = lf_goal6[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0]
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2] - altura_pata
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal7[0] = q_out_LF[0]
        lf_goal7[1] = q_out_LF[1] #- angulo_avance / 2
        lf_goal7[2] = q_out_LF[2]
        lf_goal7[3] = q_out_LF[3] #+angulo_avance/2


        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        rr_client = actionlib.SimpleActionClient('/cuadrupedo/pata_rr/follow_joint_trajectory', FollowJointTrajectoryAction)
        rr_client.wait_for_server()


        rf_client = actionlib.SimpleActionClient('/cuadrupedo/pata_rf/follow_joint_trajectory',FollowJointTrajectoryAction)
        rf_client.wait_for_server()

        lr_client = actionlib.SimpleActionClient('/cuadrupedo/pata_lr/follow_joint_trajectory',FollowJointTrajectoryAction)
        lr_client.wait_for_server()


        lf_client = actionlib.SimpleActionClient('/cuadrupedo/pata_lf/follow_joint_trajectory', FollowJointTrajectoryAction)
        lf_client.wait_for_server()

        rospy.loginfo('...connected.')

        # Create a single-point arm trajectory with the piernas_goal as the end-point
        rr_trajectory1 = JointTrajectory()
        rr_trajectory1.joint_names = piernas_joints_RR
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[0].positions = rr_goal0
        rr_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[0].time_from_start = rospy.Duration(0.25)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[1].positions = rr_goal1
        rr_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[1].time_from_start = rospy.Duration(0.5)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[2].positions = rr_goal3
        # rr_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RR]
        # rr_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[2].time_from_start = rospy.Duration(0.75)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[3].positions = rr_goal3
        # rr_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RR]
        # rr_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[3].time_from_start = rospy.Duration(1)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[4].positions = rr_goal4
        # rr_trajectory1.points[4].velocities = [0.0 for i in piernas_joints_RR]
        # rr_trajectory1.points[4].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[4].time_from_start = rospy.Duration(1.25)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[5].positions = rr_goal5
        # rr_trajectory1.points[5].velocities = [0.0 for i in piernas_joints_RR]
        # rr_trajectory1.points[5].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[5].time_from_start = rospy.Duration(1.5)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[6].positions = rr_goal6
        # rr_trajectory1.points[6].velocities = [0.0 for i in piernas_joints_RR]
        # rr_trajectory1.points[6].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[6].time_from_start = rospy.Duration(1.75)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[7].positions = rr_goal7
        rr_trajectory1.points[7].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[7].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[7].time_from_start = rospy.Duration(2)
        #'''

        rf_trajectory1 = JointTrajectory()
        rf_trajectory1.joint_names = piernas_joints_RF
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[0].positions = rr_goal6  # [0,0,0,0]
        # rf_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RF]
        # rf_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[0].time_from_start = rospy.Duration(0.25)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[1].positions = rr_goal7  # [0,0,0,0]
        rf_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[1].time_from_start = rospy.Duration(0.5)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[2].positions = rr_goal0  # [0,0,0,0]
        rf_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[2].time_from_start = rospy.Duration(0.75)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[3].positions = rr_goal1  # [0,0,0,0]
        rf_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[3].time_from_start = rospy.Duration(1)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[4].positions = rr_goal2  # [0,0,0,0]
        # rf_trajectory1.points[4].velocities = [0.0 for i in piernas_joints_RF]
        # rf_trajectory1.points[4].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[4].time_from_start = rospy.Duration(1.25)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[5].positions = rr_goal3  # [0,0,0,0]
        # rf_trajectory1.points[5].velocities = [0.0 for i in piernas_joints_RF]
        # rf_trajectory1.points[5].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[5].time_from_start = rospy.Duration(1.5)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[6].positions = rr_goal4  # [0,0,0,0]
        # rf_trajectory1.points[6].velocities = [0.0 for i in piernas_joints_RF]
        # rf_trajectory1.points[6].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[6].time_from_start = rospy.Duration(1.75)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[7].positions = rr_goal5  # [0,0,0,0]
        # rf_trajectory1.points[7].velocities = [0.0 for i in piernas_joints_RF]
        # rf_trajectory1.points[7].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[7].time_from_start = rospy.Duration(2)
        # '''
        lr_trajectory1 = JointTrajectory()
        lr_trajectory1.joint_names = piernas_joints_LR
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[0].positions = lf_goal2#lr_goal0
        # lr_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RR]
        # lr_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[0].time_from_start = rospy.Duration(0.25)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[1].positions = lf_goal3
        # lr_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RR]
        # lr_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[1].time_from_start = rospy.Duration(0.5)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[2].positions = lf_goal4
        # lr_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RR]
        # lr_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[2].time_from_start = rospy.Duration(0.75)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[3].positions = lf_goal5  # lr_goal0
        lr_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[3].time_from_start = rospy.Duration(1)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[4].positions = lf_goal6  # lr_goal0
        lr_trajectory1.points[4].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[4].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[4].time_from_start = rospy.Duration(1.25)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[5].positions = lf_goal7  # lr_goal0
        lr_trajectory1.points[5].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[5].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[5].time_from_start = rospy.Duration(1.5)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[6].positions = lf_goal0  # lr_goal0
        # lr_trajectory1.points[6].velocities = [0.0 for i in piernas_joints_RR]
        # lr_trajectory1.points[6].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[6].time_from_start = rospy.Duration(1.75)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[7].positions = lf_goal1  # lr_goal0
        # lr_trajectory1.points[7].velocities = [0.0 for i in piernas_joints_RR]
        # lr_trajectory1.points[7].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[7].time_from_start = rospy.Duration(2)
        # '''

        lf_trajectory1 = JointTrajectory()
        lf_trajectory1.joint_names = piernas_joints_LF
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[0].positions = lf_goal0#lf_goal0  # [0,0,0,0]
        # lf_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RF]
        # lf_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[0].time_from_start = rospy.Duration(0.25)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[1].positions = lf_goal1  # [0,0,0,0]
        # lf_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RF]
        # lf_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[1].time_from_start = rospy.Duration(0.5)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[2].positions = lf_goal2  # [0,0,0,0]
        # lf_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RF]
        # lf_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[2].time_from_start = rospy.Duration(0.75)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[3].positions = lf_goal3  # lf_goal0  # [0,0,0,0]
        # lf_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RF]
        # lf_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[3].time_from_start = rospy.Duration(1)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[4].positions = lf_goal4  # lf_goal0  # [0,0,0,0]
        # lf_trajectory1.points[4].velocities = [0.0 for i in piernas_joints_RF]
        # lf_trajectory1.points[4].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[4].time_from_start = rospy.Duration(1.25)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[5].positions = lf_goal5  # lf_goal0  # [0,0,0,0]
        lf_trajectory1.points[5].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[5].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[5].time_from_start = rospy.Duration(1.5)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[6].positions = lf_goal6  # lf_goal0  # [0,0,0,0]
        lf_trajectory1.points[6].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[6].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[6].time_from_start = rospy.Duration(1.75)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[7].positions = lf_goal7  # lf_goal0  # [0,0,0,0]
        lf_trajectory1.points[7].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[7].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[7].time_from_start = rospy.Duration(2)
        # '''
        # Send the trajectory to the arm action server
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
        rf_goal.trajectory = rf_trajectory1
        lr_goal.trajectory = lr_trajectory1
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
        rf_client.send_goal(rf_goal)
        #rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        lf_client.send_goal(lf_goal)
        #lr_client.wait_for_result(rospy.Duration(5.0))
        #rr_client.send_goal(rr_goal)
        #lm_client.send_goal(lm_goal)
        #rf_client.send_goal(rf_goal)'''

        if not sync:
            # Wait for up to 5 seconds for the motion to complete
            rr_client.wait_for_result(rospy.Duration(5.0))

        rr_client.wait_for_result()
        print rr_client.get_result()


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

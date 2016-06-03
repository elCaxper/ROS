#!/usr/bin/env python

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL
from sensor_msgs.msg import Joy
import hrpsys.ModelLoader_idl
import copy
from angles import normalize
import pickle
from rotacion_hex import TrajectoryDemo

class TrajectoryDemo2():
    def __init__(self):

        rospy.init_node('ps3')
        self.joy_topic = "joy"
        rospy.Subscriber(self.joy_topic, Joy, self.on_joy_message)



        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/mi_hexapodo_exp.urdf'

        angulo_avance = -0.40 #rad
        altura_pata = +0.04 #cm
        # angulo_avance = 0  # +0.40 #rad
        # altura_pata = 0  # -0.04 #cm

        robot = urdf_parser_py.urdf.URDF.from_xml_file(filename)

        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        cadena_RR = tree.getChain("base_link", "tarsus_RR")
        cadena_RM = tree.getChain("base_link", "tarsus_RM")
        cadena_RF = tree.getChain("base_link", "tarsus_RF")
        cadena_LR = tree.getChain("base_link", "tarsus_LR")
        cadena_LM = tree.getChain("base_link", "tarsus_LM")
        cadena_LF = tree.getChain("base_link", "tarsus_LF")

        print cadena_RR.getNrOfSegments()
        fksolver_RR = PyKDL.ChainFkSolverPos_recursive(cadena_RR)
        fksolver_RM= PyKDL.ChainFkSolverPos_recursive(cadena_RM)
        fksolver_RF = PyKDL.ChainFkSolverPos_recursive(cadena_RF)
        fksolver_LR = PyKDL.ChainFkSolverPos_recursive(cadena_LR)
        fksolver_LM = PyKDL.ChainFkSolverPos_recursive(cadena_LM)
        fksolver_LF = PyKDL.ChainFkSolverPos_recursive(cadena_LF)

        vik_RR = PyKDL.ChainIkSolverVel_pinv(cadena_RR)
        ik_RR = PyKDL.ChainIkSolverPos_NR(cadena_RR, fksolver_RR, vik_RR)

        vik_RM = PyKDL.ChainIkSolverVel_pinv(cadena_RM)
        ik_RM = PyKDL.ChainIkSolverPos_NR(cadena_RM, fksolver_RM, vik_RM)

        vik_RF = PyKDL.ChainIkSolverVel_pinv(cadena_RF)
        ik_RF = PyKDL.ChainIkSolverPos_NR(cadena_RF, fksolver_RF, vik_RF)

        vik_LR = PyKDL.ChainIkSolverVel_pinv(cadena_LR)
        ik_LR = PyKDL.ChainIkSolverPos_NR(cadena_LR, fksolver_LR, vik_LR)

        vik_LM = PyKDL.ChainIkSolverVel_pinv(cadena_LM)
        ik_LM = PyKDL.ChainIkSolverPos_NR(cadena_LM, fksolver_LM, vik_LM)

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

        # rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        piernas_joints_RR = ['coxa_joint_RR', 'femur_joint_RR','tibia_joint_RR', 'tarsus_joint_RR']
        piernas_joints_LM = ['coxa_joint_LM', 'femur_joint_LM','tibia_joint_LM', 'tarsus_joint_LM']
        piernas_joints_RF = ['coxa_joint_RF', 'femur_joint_RF','tibia_joint_RF', 'tarsus_joint_RF']

        piernas_joints_LR = ['coxa_joint_LR', 'femur_joint_LR', 'tibia_joint_LR', 'tarsus_joint_LR']
        piernas_joints_RM = ['coxa_joint_RM', 'femur_joint_RM', 'tibia_joint_RM', 'tarsus_joint_RM']
        piernas_joints_LF = ['coxa_joint_LF', 'femur_joint_LF', 'tibia_joint_LF', 'tarsus_joint_LF']

        rr_goal0 = [0.0, 0.0, 0.0, 0.0]
        lm_goal0 = [0.0, 0.0, 0.0, 0.0]
        rf_goal0 = [0.0, 0.0, 0.0, 0.0]
        rr_goal1 = [0.0, 0.0, 0.0, 0.0]
        lm_goal1 = [0.0, 0.0, 0.0, 0.0]
        rf_goal1 = [0.0, 0.0, 0.0, 0.0]
        lr_goal0 = [0.0, 0.0, 0.0, 0.0]
        rm_goal0 = [0.0, 0.0, 0.0, 0.0]
        lf_goal0 = [0.0, 0.0, 0.0, 0.0]
        lr_goal1 = [0.0, 0.0, 0.0, 0.0]
        rm_goal1 = [0.0, 0.0, 0.0, 0.0]
        lf_goal1 = [0.0, 0.0, 0.0, 0.0]

        #11111111111111111111111111111111111111111111
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(posicionInicial_R, finalFrame_R)
        q_init_RR = posicionInicial_R  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0]
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2]+altura_pata
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal0[0] = q_out_RR[0]+angulo_avance
        rr_goal0[1] = q_out_RR[1]
        rr_goal0[2] = q_out_RR[2]
        rr_goal0[3] = q_out_RR[3]

        print "Inverse Kinematics"
        fksolver_LM.JntToCart(posicionInicial_L, finalFrame_L)
        q_init_LM = posicionInicial_L  # initial angles
        desiredFrameLM = finalFrame_L
        desiredFrameLM.p[0] = finalFrame_L.p[0]
        desiredFrameLM.p[1] = finalFrame_L.p[1]
        desiredFrameLM.p[2] = finalFrame_L.p[2] +altura_pata
        print "Desired Position: ", desiredFrameLM.p
        q_out_LM = PyKDL.JntArray(cadena_LM.getNrOfJoints())
        ik_LM.CartToJnt(q_init_LM, desiredFrameLM, q_out_LM)
        print "Output angles in rads: ", q_out_LM

        lm_goal0[0] = q_out_LM[0]+angulo_avance
        lm_goal0[1] = q_out_LM[1]
        lm_goal0[2] = q_out_LM[2]
        lm_goal0[3] = q_out_LM[3]

        print "Inverse Kinematics"
        fksolver_RF.JntToCart(posicionInicial_R, finalFrame_R)
        q_init_RF = posicionInicial_R  # initial angles
        desiredFrameRF = finalFrame_R
        desiredFrameRF.p[0] = finalFrame_R.p[0]
        desiredFrameRF.p[1] = finalFrame_R.p[1]
        desiredFrameRF.p[2] = finalFrame_R.p[2] + altura_pata
        print "Desired Position: ", desiredFrameRF.p
        q_out_RF = PyKDL.JntArray(cadena_RF.getNrOfJoints())
        ik_RF.CartToJnt(q_init_RF, desiredFrameRF, q_out_RF)
        print "Output angles in rads: ", q_out_RF

        rf_goal0[0] = q_out_RF[0]+angulo_avance
        rf_goal0[1] = q_out_RF[1]
        rf_goal0[2] = q_out_RF[2]
        rf_goal0[3] = q_out_RF[3]

        # 22222222222222222222222222222222222222222222
        RR_actual = PyKDL.JntArray(nj_izq)
        RR_actual[0] = rr_goal0[0]
        RR_actual[1] = rr_goal0[1]
        RR_actual[2] = rr_goal0[2]
        RR_actual[3] = rr_goal0[3]
        print "Inverse Kinematics"
        fksolver_RR.JntToCart(RR_actual, finalFrame_R)
        q_init_RR = RR_actual  # initial angles
        desiredFrameRR = finalFrame_R
        desiredFrameRR.p[0] = finalFrame_R.p[0]
        desiredFrameRR.p[1] = finalFrame_R.p[1]
        desiredFrameRR.p[2] = finalFrame_R.p[2] - altura_pata
        print "Desired Position: ", desiredFrameRR.p
        q_out_RR = PyKDL.JntArray(cadena_RR.getNrOfJoints())
        ik_RR.CartToJnt(q_init_RR, desiredFrameRR, q_out_RR)
        print "Output angles in rads: ", q_out_RR

        rr_goal1[0] = q_out_RR[0]
        rr_goal1[1] = q_out_RR[1]
        rr_goal1[2] = q_out_RR[2]
        rr_goal1[3] = q_out_RR[3]

        LM_actual = PyKDL.JntArray(nj_izq)
        LM_actual[0] = lm_goal0[0]
        LM_actual[1] = lm_goal0[1]
        LM_actual[2] = lm_goal0[2]
        LM_actual[3] = lm_goal0[3]
        print "Inverse Kinematics"
        fksolver_LM.JntToCart(LM_actual, finalFrame_L)
        q_init_LM = LM_actual  # initial angles
        desiredFrameLM = finalFrame_L
        desiredFrameLM.p[0] = finalFrame_L.p[0]
        desiredFrameLM.p[1] = finalFrame_L.p[1]
        desiredFrameLM.p[2] = finalFrame_L.p[2] - altura_pata
        print "Desired Position: ", desiredFrameLM.p
        q_out_LM = PyKDL.JntArray(cadena_LM.getNrOfJoints())
        ik_LM.CartToJnt(q_init_LM, desiredFrameLM, q_out_LM)
        print "Output angles in rads: ", q_out_LM

        lm_goal1[0] = q_out_LM[0]
        lm_goal1[1] = q_out_LM[1]
        lm_goal1[2] = q_out_LM[2]
        lm_goal1[3] = q_out_LM[3]

        RF_actual = PyKDL.JntArray(nj_izq)
        RF_actual[0] = rf_goal0[0]
        RF_actual[1] = rf_goal0[1]
        RF_actual[2] = rf_goal0[2]
        RF_actual[3] = rf_goal0[3]
        print "Inverse Kinematics"
        fksolver_RF.JntToCart(RF_actual, finalFrame_R)
        q_init_RF = RF_actual  # initial angles
        desiredFrameRF = finalFrame_R
        desiredFrameRF.p[0] = finalFrame_R.p[0]
        desiredFrameRF.p[1] = finalFrame_R.p[1]
        desiredFrameRF.p[2] = finalFrame_R.p[2]- altura_pata
        print "Desired Position: ", desiredFrameRF.p
        q_out_RF = PyKDL.JntArray(cadena_RF.getNrOfJoints())
        ik_RF.CartToJnt(q_init_RF, desiredFrameRF, q_out_RF)
        print "Output angles in rads: ", q_out_RF

        rf_goal1[0] = q_out_RF[0]
        rf_goal1[1] = q_out_RF[1]
        rf_goal1[2] = q_out_RF[2]
        rf_goal1[3] = q_out_RF[3]


        # 11111111111111111111111111111111111111111111
        print "Inverse Kinematics"
        fksolver_LR.JntToCart(posicionInicial_L, finalFrame_L)
        q_init_LR = posicionInicial_L  # initial angles
        desiredFrameLR = finalFrame_L
        desiredFrameLR.p[0] = finalFrame_L.p[0]
        desiredFrameLR.p[1] = finalFrame_L.p[1]
        desiredFrameLR.p[2] = finalFrame_L.p[2]  # + altura_pata
        print "Desired Position: ", desiredFrameLR.p
        q_out_LR = PyKDL.JntArray(cadena_LR.getNrOfJoints())
        ik_LR.CartToJnt(q_init_LR, desiredFrameLR, q_out_LR)
        print "Output angles in rads: ", q_out_LR

        lr_goal0[0] = q_out_LR[0] - angulo_avance
        lr_goal0[1] = q_out_LR[1]
        lr_goal0[2] = q_out_LR[2]
        lr_goal0[3] = q_out_LR[3]

        print "Inverse Kinematics"
        fksolver_RM.JntToCart(posicionInicial_R, finalFrame_R)
        q_init_RM = posicionInicial_R  # initial angles
        desiredFrameRM = finalFrame_R
        desiredFrameRM.p[0] = finalFrame_R.p[0]
        desiredFrameRM.p[1] = finalFrame_R.p[1]
        desiredFrameRM.p[2] = finalFrame_R.p[2]  # + altura_pata
        print "Desired Position: ", desiredFrameRM.p
        q_out_RM = PyKDL.JntArray(cadena_RM.getNrOfJoints())
        ik_RM.CartToJnt(q_init_RM, desiredFrameRM, q_out_RM)
        print "Output angles in rads: ", q_out_RM

        rm_goal0[0] = q_out_RM[0] - angulo_avance
        rm_goal0[1] = q_out_RM[1]
        rm_goal0[2] = q_out_RM[2]
        rm_goal0[3] = q_out_RM[3]

        print "Inverse Kinematics"
        fksolver_LF.JntToCart(posicionInicial_L, finalFrame_L)
        q_init_LF = posicionInicial_L  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0]
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]  # + altura_pata
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal0[0] = q_out_LF[0] - angulo_avance
        lf_goal0[1] = q_out_LF[1]
        lf_goal0[2] = q_out_LF[2]
        lf_goal0[3] = q_out_LF[3]

        # 2222222222222222222222222222222222222222222222

        LR_actual = PyKDL.JntArray(nj_izq)
        LR_actual[0] = lr_goal0[0]
        LR_actual[1] = lr_goal0[1]
        LR_actual[2] = lr_goal0[2]
        LR_actual[3] = lr_goal0[3]
        print "Inverse Kinematics"
        fksolver_LR.JntToCart(LR_actual, finalFrame_L)
        q_init_LR = LR_actual  # initial angles
        print "Inverse Kinematics"
        desiredFrameLR = finalFrame_L
        desiredFrameLR.p[0] = finalFrame_L.p[0]
        desiredFrameLR.p[1] = finalFrame_L.p[1]
        desiredFrameLR.p[2] = finalFrame_L.p[2]   + altura_pata
        print "Desired Position: ", desiredFrameLR.p
        q_out_LR = PyKDL.JntArray(cadena_LR.getNrOfJoints())
        ik_LR.CartToJnt(q_init_LR, desiredFrameLR, q_out_LR)
        print "Output angles in rads: ", q_out_LR

        lr_goal1[0] = q_out_LR[0] #- angulo_avance
        lr_goal1[1] = q_out_LR[1]
        lr_goal1[2] = q_out_LR[2]
        lr_goal1[3] = q_out_LR[3]

        RM_actual = PyKDL.JntArray(nj_izq)
        RM_actual[0] = rm_goal0[0]
        RM_actual[1] = rm_goal0[1]
        RM_actual[2] = rm_goal0[2]
        RM_actual[3] = rm_goal0[3]
        print "Inverse Kinematics"
        fksolver_RM.JntToCart(RM_actual, finalFrame_R)
        q_init_RM = RM_actual  # initial angles
        desiredFrameRM = finalFrame_R
        desiredFrameRM.p[0] = finalFrame_R.p[0]
        desiredFrameRM.p[1] = finalFrame_R.p[1]
        desiredFrameRM.p[2] = finalFrame_R.p[2]   + altura_pata
        print "Desired Position: ", desiredFrameRM.p
        q_out_RM = PyKDL.JntArray(cadena_RM.getNrOfJoints())
        ik_RM.CartToJnt(q_init_RM, desiredFrameRM, q_out_RM)
        print "Output angles in rads: ", q_out_RM

        rm_goal1[0] = q_out_RM[0] #- angulo_avance
        rm_goal1[1] = q_out_RM[1]
        rm_goal1[2] = q_out_RM[2]
        rm_goal1[3] = q_out_RM[3]

        LF_actual = PyKDL.JntArray(nj_izq)
        LF_actual[0] = lf_goal0[0]
        LF_actual[1] = lf_goal0[1]
        LF_actual[2] = lf_goal0[2]
        LF_actual[3] = lf_goal0[3]
        print "Inverse Kinematics"
        fksolver_LF.JntToCart(LF_actual, finalFrame_L)
        q_init_LF = LF_actual  # initial angles
        desiredFrameLF = finalFrame_L
        desiredFrameLF.p[0] = finalFrame_L.p[0]
        desiredFrameLF.p[1] = finalFrame_L.p[1]
        desiredFrameLF.p[2] = finalFrame_L.p[2]   + altura_pata
        print "Desired Position: ", desiredFrameLF.p
        q_out_LF = PyKDL.JntArray(cadena_LF.getNrOfJoints())
        ik_LF.CartToJnt(q_init_LF, desiredFrameLF, q_out_LF)
        print "Output angles in rads: ", q_out_LF

        lf_goal1[0] = q_out_LF[0] #- angulo_avance
        lf_goal1[1] = q_out_LF[1]
        lf_goal1[2] = q_out_LF[2]
        lf_goal1[3] = q_out_LF[3]

        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        global rr_client,lm_client,rf_client,lr_client,rm_client,lf_client

        rr_client = actionlib.SimpleActionClient('/hexapodo/pata_rr/follow_joint_trajectory', FollowJointTrajectoryAction)
        rr_client.wait_for_server()

        lm_client = actionlib.SimpleActionClient('/hexapodo/pata_lm/follow_joint_trajectory', FollowJointTrajectoryAction)
        lm_client.wait_for_server()

        rf_client = actionlib.SimpleActionClient('/hexapodo/pata_rf/follow_joint_trajectory',FollowJointTrajectoryAction)
        rf_client.wait_for_server()

        lr_client = actionlib.SimpleActionClient('/hexapodo/pata_lr/follow_joint_trajectory',FollowJointTrajectoryAction)
        lr_client.wait_for_server()

        rm_client = actionlib.SimpleActionClient('/hexapodo/pata_rm/follow_joint_trajectory',FollowJointTrajectoryAction)
        rm_client.wait_for_server()

        lf_client = actionlib.SimpleActionClient('/hexapodo/pata_lf/follow_joint_trajectory', FollowJointTrajectoryAction)
        lf_client.wait_for_server()

        rospy.loginfo('...connected.')

        # Create a single-point arm trajectory with the piernas_goal as the end-point
        rr_trajectory1 = JointTrajectory()
        rr_trajectory1.joint_names = piernas_joints_RR
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[0].positions = rr_goal0
        rr_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[1].positions = rr_goal1
        rr_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[2].positions = lr_goal1
        rr_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        rr_trajectory1.points.append(JointTrajectoryPoint())
        rr_trajectory1.points[3].positions = lr_goal0
        rr_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RR]
        rr_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        lm_trajectory1 = JointTrajectory()
        lm_trajectory1.joint_names = piernas_joints_LM
        lm_trajectory1.points.append(JointTrajectoryPoint())
        lm_trajectory1.points[0].positions = lm_goal0#[0,0,0,0]
        lm_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        lm_trajectory1.points.append(JointTrajectoryPoint())
        lm_trajectory1.points[1].positions = lm_goal1  # [0,0,0,0]
        lm_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        lm_trajectory1.points.append(JointTrajectoryPoint())
        lm_trajectory1.points[2].positions = rm_goal1  # [0,0,0,0]
        lm_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        lm_trajectory1.points.append(JointTrajectoryPoint())
        lm_trajectory1.points[3].positions = rm_goal0  # [0,0,0,0]
        lm_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_LM]
        lm_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        rf_trajectory1 = JointTrajectory()
        rf_trajectory1.joint_names = piernas_joints_RF
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[0].positions = rf_goal0  # [0,0,0,0]
        rf_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[1].positions = rf_goal1  # [0,0,0,0]
        rf_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[2].positions = lf_goal1  # [0,0,0,0]
        rf_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        rf_trajectory1.points.append(JointTrajectoryPoint())
        rf_trajectory1.points[3].positions = lf_goal0  # [0,0,0,0]
        rf_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RF]
        rf_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        lr_trajectory1 = JointTrajectory()
        lr_trajectory1.joint_names = piernas_joints_LR
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[0].positions = lr_goal0#lr_goal0
        lr_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[1].positions = lr_goal1
        lr_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[2].positions = rr_goal1
        lr_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        lr_trajectory1.points.append(JointTrajectoryPoint())
        lr_trajectory1.points[3].positions = rr_goal0  # lr_goal0
        lr_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RR]
        lr_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        rm_trajectory1 = JointTrajectory()
        rm_trajectory1.joint_names = piernas_joints_RM
        rm_trajectory1.points.append(JointTrajectoryPoint())
        rm_trajectory1.points[0].positions = rm_goal0#rm_goal0  # [0,0,0,0]
        rm_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        rm_trajectory1.points.append(JointTrajectoryPoint())
        rm_trajectory1.points[1].positions = rm_goal1  # [0,0,0,0]
        rm_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        rm_trajectory1.points.append(JointTrajectoryPoint())
        rm_trajectory1.points[2].positions = lm_goal1  # [0,0,0,0]
        rm_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        rm_trajectory1.points.append(JointTrajectoryPoint())
        rm_trajectory1.points[3].positions = lm_goal0  # rm_goal0  # [0,0,0,0]
        rm_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_LM]
        rm_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        lf_trajectory1 = JointTrajectory()
        lf_trajectory1.joint_names = piernas_joints_LF
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[0].positions = lf_goal0#lf_goal0  # [0,0,0,0]
        lf_trajectory1.points[0].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[0].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[0].time_from_start = rospy.Duration(1.0)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[1].positions = lf_goal1  # [0,0,0,0]
        lf_trajectory1.points[1].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[1].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[1].time_from_start = rospy.Duration(2.0)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[2].positions = rf_goal1  # [0,0,0,0]
        lf_trajectory1.points[2].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[2].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[2].time_from_start = rospy.Duration(3.0)
        lf_trajectory1.points.append(JointTrajectoryPoint())
        lf_trajectory1.points[3].positions = rf_goal0  # lf_goal0  # [0,0,0,0]
        lf_trajectory1.points[3].velocities = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[3].accelerations = [0.0 for i in piernas_joints_RF]
        lf_trajectory1.points[3].time_from_start = rospy.Duration(4.0)

        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')



        # Create an empty trajectory goal
        global rr_goal, lm_goal, rf_goal, lr_goal, rm_goal, lf_goal
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



        '''rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        #rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)

        rr_client.wait_for_result()
        rr_client.send_goal(rr_goal)
        lm_client.send_goal(lm_goal)
        rf_client.send_goal(rf_goal)
        # rr_client.wait_for_result(rospy.Duration(5.0))
        print 'Resultado recibido'
        lr_client.send_goal(lr_goal)
        rm_client.send_goal(rm_goal)
        lf_client.send_goal(lf_goal)
        #lr_client.wait_for_result(rospy.Duration(5.0))
        #rr_client.send_goal(rr_goal)
        #lm_client.send_goal(lm_goal)
        #rf_client.send_goal(rf_goal)'''

        # if not sync:
        #     # Wait for up to 5 seconds for the motion to complete
        #     rr_client.wait_for_result(rospy.Duration(5.0))
        #
        # rr_client.wait_for_result()
        # print rr_client.get_result()


        rospy.loginfo('...done')

    def on_joy_message(self, msg):
        if int(msg.buttons[15]) == 1:
            print "Pulsado cuadrado"
        if int(msg.buttons[13]) == 1:
            print "Pulsado circulo"
        if msg.axes[1] > 0.5:
            print "Avanza"
            print msg.axes[1]
            rr_client.send_goal(rr_goal)
            lm_client.send_goal(lm_goal)
            rf_client.send_goal(rf_goal)
            # rr_client.wait_for_result(rospy.Duration(5.0))
            print 'Resultado recibido'
            lr_client.send_goal(lr_goal)
            rm_client.send_goal(rm_goal)
            lf_client.send_goal(lf_goal)
            rr_client.wait_for_result()
        if msg.axes[1] < -0.5:
            print "Retrocede"
            print msg.axes[1]
        if msg.axes[0] > 0.5:
            print "Giro Izq"
            print msg.axes[1]
            TrajectoryDemo()
        if msg.axes[0] < -0.5:
            print "Giro Der"
            print msg.axes[1]

if __name__ == '__main__':
    try:
        TrajectoryDemo2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

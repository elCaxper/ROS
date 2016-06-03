#!/usr/bin/env python

import actionlib
import rospy

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

        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/urdf_exportado4.urdf'
        datos_articulaciones = open('datos_articulaciones.pkl', 'wb')

        robot = urdf_parser_py.urdf.URDF.from_xml_file(filename)

        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filename)
        cadena_der_up_down = tree.getChain("base_link", "pie_der_link")
        cadena_der_down_up = tree.getChain("pie_der_link", "base_link")
        cadena_izq_up_down = tree.getChain("base_link", "pie_izq_link")
        cadena_izq_down_up = tree.getChain("pie_izq_link", "base_link")

        print cadena_der_up_down.getNrOfSegments()
        fksolver_der_up_down = PyKDL.ChainFkSolverPos_recursive(cadena_der_up_down)
        fksolver_der_down_up = PyKDL.ChainFkSolverPos_recursive(cadena_der_down_up)
        fksolver_izq_up_down = PyKDL.ChainFkSolverPos_recursive(cadena_izq_up_down)
        fksolver_izq_down_up = PyKDL.ChainFkSolverPos_recursive(cadena_izq_down_up)

        vik_der_up_down = PyKDL.ChainIkSolverVel_pinv(cadena_der_up_down)
        ik_der_up_down = PyKDL.ChainIkSolverPos_NR(cadena_der_up_down, fksolver_der_up_down, vik_der_up_down)

        vik_der_down_up = PyKDL.ChainIkSolverVel_pinv(cadena_der_down_up)
        ik_der_down_up = PyKDL.ChainIkSolverPos_NR(cadena_der_down_up, fksolver_der_down_up, vik_der_down_up)

        vik_izq_up_down = PyKDL.ChainIkSolverVel_pinv(cadena_izq_up_down)
        ik_izq_up_down = PyKDL.ChainIkSolverPos_NR(cadena_izq_up_down, fksolver_izq_up_down, vik_izq_up_down)

        vik_izq_down_up = PyKDL.ChainIkSolverVel_pinv(cadena_izq_down_up)
        ik_izq_down_up = PyKDL.ChainIkSolverPos_NR(cadena_izq_down_up, fksolver_izq_down_up, vik_izq_down_up)

        nj_izq = cadena_der_up_down.getNrOfJoints()
        posicionInicial_der_up_down = PyKDL.JntArray(nj_izq)
        posicionInicial_der_up_down[0] = 0.3
        posicionInicial_der_up_down[1] = -0.3
        posicionInicial_der_up_down[2] = 0
        posicionInicial_der_up_down[3] = 0.6
        posicionInicial_der_up_down[4] = -0.3
        posicionInicial_der_up_down[5] = -0.3

        nj_izq = cadena_izq_up_down.getNrOfJoints()
        posicionInicial_izq_up_down = PyKDL.JntArray(nj_izq)
        posicionInicial_izq_up_down[0] = 0.3
        posicionInicial_izq_up_down[1] = -0.3
        posicionInicial_izq_up_down[2] = 0.0
        posicionInicial_izq_up_down[3] = 0.6
        posicionInicial_izq_up_down[4] = -0.3
        posicionInicial_izq_up_down[5] = -0.3

        nj_izq = cadena_der_down_up.getNrOfJoints()
        posicionInicial_der_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_der_down_up[5] = 0.3
        posicionInicial_der_down_up[4] = -0.3
        posicionInicial_der_down_up[3] = 0.0
        posicionInicial_der_down_up[2] = 0.6
        posicionInicial_der_down_up[1] = -0.3
        posicionInicial_der_down_up[0] = -0.3

        nj_izq = cadena_izq_down_up.getNrOfJoints()
        posicionInicial_izq_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_izq_down_up[5] = 0.3
        posicionInicial_izq_down_up[4] = -0.3
        posicionInicial_izq_down_up[3] = 0.0
        posicionInicial_izq_down_up[2] = 0.6
        posicionInicial_izq_down_up[1] = -0.3
        posicionInicial_izq_down_up[0] = -0.3
        print "Forward kinematics"
        finalFrame_izq_up_down = PyKDL.Frame()
        finalFrame_izq_down_up = PyKDL.Frame()
        finalFrame_der_up_down = PyKDL.Frame()
        finalFrame_der_down_up = PyKDL.Frame()


        print "Rotational Matrix of the final Frame: "
        print  finalFrame_izq_up_down.M
        print "End-effector position: ", finalFrame_izq_up_down.p

        rospy.init_node('trajectory_demo')

        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)

        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)

        # Which joints define the arm?
        piernas_joints = ['cilinder_blue1_box1_der_joint', 'cilinder_blue_box1_der_joint',  'cilinder_blue_box2_der_joint',
                          'cilinder_blue_box4_der_joint',  'cilinder_blue1_box6_der_joint', 'cilinder_blue_box6_der_joint',
                          'cilinder_blue1_box1_izq_joint', 'cilinder_blue_box1_izq_joint',  'cilinder_blue_box2_izq_joint',
                          'cilinder_blue_box4_izq_joint',  'cilinder_blue1_box6_izq_joint', 'cilinder_blue_box6_izq_joint' ]


        piernas_goal  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #11111111111111111111111111111111111111111111
        print "Inverse Kinematics"
        fksolver_izq_up_down.JntToCart(posicionInicial_izq_up_down, finalFrame_izq_up_down)
        fksolver_izq_down_up.JntToCart(posicionInicial_izq_down_up, finalFrame_izq_down_up)
        q_init_izq_up_down = posicionInicial_izq_up_down  # initial angles
        desiredFrame = finalFrame_izq_up_down
        desiredFrame.p[0] = finalFrame_izq_up_down.p[0]
        desiredFrame.p[1] = finalFrame_izq_up_down.p[1]
        desiredFrame.p[2] = finalFrame_izq_up_down.p[2]
        print "Desired Position: ", desiredFrame.p
        q_out_izq_up_down = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame, q_out_izq_up_down)
        print "Output angles in rads: ", q_out_izq_up_down

        piernas_goal[6] = q_out_izq_up_down[0]
        piernas_goal[7] = q_out_izq_up_down[1]
        piernas_goal[8] = q_out_izq_up_down[2]
        piernas_goal[9] = q_out_izq_up_down[3]
        piernas_goal[10] = q_out_izq_up_down[4]
        piernas_goal[11] = q_out_izq_up_down[5]

        print "Inverse Kinematics"
        fksolver_der_up_down.JntToCart(posicionInicial_der_up_down, finalFrame_der_up_down)
        fksolver_der_down_up.JntToCart(posicionInicial_der_down_up, finalFrame_der_down_up)
        q_init_der_up_down = posicionInicial_der_up_down  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame = finalFrame_der_up_down
        desiredFrame.p[0] = finalFrame_der_up_down.p[0]
        desiredFrame.p[1] = finalFrame_der_up_down.p[1]
        desiredFrame.p[2] = finalFrame_der_up_down.p[2]+0.02 #0.02
        print "Desired Position: ", desiredFrame.p
        q_out_der_up_down = PyKDL.JntArray(cadena_der_up_down.getNrOfJoints())
        ik_der_up_down.CartToJnt(q_init_der_up_down, desiredFrame, q_out_der_up_down)
        print "Output angles in rads: ", q_out_der_up_down

        piernas_goal[0] = q_out_der_up_down[0]
        piernas_goal[1] = q_out_der_up_down[1]
        piernas_goal[2] = q_out_der_up_down[2]
        piernas_goal[3] = q_out_der_up_down[3]
        piernas_goal[4] = q_out_der_up_down[4]
        piernas_goal[5] = q_out_der_up_down[5]

        #121212122121212121212121212121212121212121212
        piernas_goal12 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print "Inverse Kinematics"
        desiredFrame = PyKDL.Frame()
        fksolver_izq_up_down.JntToCart(q_out_izq_up_down, desiredFrame)
        fksolver_izq_down_up.JntToCart(posicionInicial_izq_down_up, finalFrame_izq_down_up)
        q_init_izq_up_down = posicionInicial_izq_up_down  # initial angles
        desiredFrame.p[0] = desiredFrame.p[0]
        desiredFrame.p[1] = desiredFrame.p[1]
        desiredFrame.p[2] = desiredFrame.p[2]
        print "Desired Position: ", desiredFrame.p
        q_out_izq_up_down = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame, q_out_izq_up_down)
        print "Output angles in rads: ", q_out_izq_up_down

        piernas_goal12[6] = q_out_izq_up_down[0]
        piernas_goal12[7] = q_out_izq_up_down[1]
        piernas_goal12[8] = q_out_izq_up_down[2]
        piernas_goal12[9] = q_out_izq_up_down[3]
        piernas_goal12[10] = q_out_izq_up_down[4]
        piernas_goal12[11] = q_out_izq_up_down[5]

        print "Inverse Kinematics"
        desiredFrame0 = PyKDL.Frame()
        fksolver_der_up_down.JntToCart(q_out_der_up_down, desiredFrame0)
        fksolver_der_down_up.JntToCart(posicionInicial_der_down_up, finalFrame_der_down_up)
        q_init_der_up_down = posicionInicial_der_up_down  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame0.p[0] = desiredFrame0.p[0]
        desiredFrame0.p[1] = desiredFrame0.p[1] -0.07
        desiredFrame0.p[2] = desiredFrame0.p[2]
        print "Desired Position: ", desiredFrame0.p
        q_out_der_up_down = PyKDL.JntArray(cadena_der_up_down.getNrOfJoints())
        ik_der_up_down.CartToJnt(q_init_der_up_down, desiredFrame0, q_out_der_up_down)
        print "Output angles in rads: ", q_out_der_up_down

        piernas_goal12[0] = q_out_der_up_down[0]
        piernas_goal12[1] = q_out_der_up_down[1]
        piernas_goal12[2] = q_out_der_up_down[2]
        piernas_goal12[3] = q_out_der_up_down[3]
        piernas_goal12[4] = q_out_der_up_down[4]
        piernas_goal12[5] = q_out_der_up_down[5]

        # 222222222222222222222222222222222222222222
        piernas_goal2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print "Inverse Kinematics"
        desiredFrame2 = PyKDL.Frame()
        fksolver_izq_down_up.JntToCart(posicionInicial_izq_down_up, desiredFrame2)
        q_init_izq_down_up = posicionInicial_izq_down_up  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame2.p[0] = desiredFrame2.p[0] -0.06 #0.05
        desiredFrame2.p[1] = desiredFrame2.p[1] -0.06#0.06
        desiredFrame2.p[2] = desiredFrame2.p[2] -0.01  #0.02
        print "Desired Position2222aaa: ", desiredFrame2.p
        #print desiredFrame2.M
        q_out_izq_down_up = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_down_up.CartToJnt(q_init_izq_down_up, desiredFrame2, q_out_izq_down_up)
        print "Output angles in rads2222: ", q_out_izq_down_up

        piernas_goal2[6] = q_out_izq_down_up[5]#+0.1
        piernas_goal2[7] = q_out_izq_down_up[4]#-0.05
        piernas_goal2[8] = q_out_izq_down_up[3]
        piernas_goal2[9] = q_out_izq_down_up[2]
        piernas_goal2[10] = q_out_izq_down_up[1]
        piernas_goal2[11] = q_out_izq_down_up[0]

        #q_out_izq_down_up[4] -=0.1

        print "Inverse Kinematics"
        q_init_der_down_up = PyKDL.JntArray(6)
        q_init_der_down_up[0] = q_out_der_up_down[5] # PROBLEMAS CON LA ASIGNACION
        q_init_der_down_up[1] = q_out_der_up_down[4]
        q_init_der_down_up[2] = q_out_der_up_down[3]
        q_init_der_down_up[3] = q_out_der_up_down[2]
        q_init_der_down_up[4] = q_out_der_up_down[1]
        q_init_der_down_up[5] = q_out_der_up_down[0]+0.08
        fksolver_der_down_up.JntToCart(q_init_der_down_up,finalFrame_der_down_up)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame3 = finalFrame_der_down_up
        desiredFrame3.p[0] = finalFrame_der_down_up.p[0]- 0.05
        desiredFrame3.p[1] = finalFrame_der_down_up.p[1]- 0.04
        desiredFrame3.p[2] = finalFrame_der_down_up.p[2]-0.02
        print "Desired Position2222der: ", desiredFrame3.p
        q_out_der_down_up = PyKDL.JntArray(cadena_der_down_up.getNrOfJoints())
        ik_der_down_up.CartToJnt(q_init_der_down_up, desiredFrame3, q_out_der_down_up)
        print "Output angles in rads22222der: ", q_out_der_down_up
        print "VALOR", q_out_der_up_down[5]
        piernas_goal2[0] = -0.3
        piernas_goal2[1] = -0.3
        piernas_goal2[2] = 0
        piernas_goal2[3] = 0.6
        piernas_goal2[4] = 0.3
        piernas_goal2[5] = -0.3


        # 333333333333333333333333333333333333333333333333
        piernas_goal3 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        print "Inverse Kinematics"
        desiredFrame4 = PyKDL.Frame()
        fksolver_izq_down_up.JntToCart(q_out_izq_down_up,desiredFrame4)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame4.p[0] = desiredFrame4.p[0]
        desiredFrame4.p[1] = desiredFrame4.p[1] - 0.02
        desiredFrame4.p[2] = desiredFrame4.p[2]
        ik_izq_down_up.CartToJnt(q_out_izq_down_up, desiredFrame4, q_out_izq_down_up)

        q_init_izq_up_down[0] = q_out_izq_down_up[5]
        q_init_izq_up_down[1] = q_out_izq_down_up[4]
        q_init_izq_up_down[2] = q_out_izq_down_up[3]
        q_init_izq_up_down[3] = q_out_izq_down_up[2]
        q_init_izq_up_down[4] = q_out_izq_down_up[1]
        q_init_izq_up_down[5] = q_out_izq_down_up[0]

        desiredFrame5 = PyKDL.Frame()
        fksolver_izq_up_down.JntToCart(q_init_izq_up_down,desiredFrame5)
        desiredFrame5.p[0] = desiredFrame5.p[0]
        desiredFrame5.p[1] = desiredFrame5.p[1] - 0.1
        desiredFrame5.p[2] = desiredFrame5.p[2]+0.01
        print "Desired Position: ", desiredFrame5.p
        q_out_izq_up_down2 = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame5, q_out_izq_up_down2)
        print "Output angles in rads: ", q_out_izq_up_down2
        piernas_goal3[6] = q_out_izq_up_down2[0]
        piernas_goal3[7] = q_out_izq_up_down2[1]
        piernas_goal3[8] = q_out_izq_up_down2[2]
        piernas_goal3[9] = q_out_izq_up_down2[3]
        piernas_goal3[10] = q_out_izq_up_down2[4]#+0.15
        piernas_goal3[11] = q_out_izq_up_down2[5]-0.08

        print "Inverse Kinematics"

        piernas_goal3[0] = -0.3
        piernas_goal3[1] = -0.3
        piernas_goal3[2] = 0
        piernas_goal3[3] = 0.6
        piernas_goal3[4] = 0.3
        piernas_goal3[5] = -0.3

        # 121212122121212121212121212121212121212121212
        piernas_goal121 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print "Inverse Kinematics"
        desiredFrame6 = PyKDL.Frame()
        fksolver_izq_up_down.JntToCart(q_out_izq_up_down2, desiredFrame6)
        fksolver_izq_down_up.JntToCart(posicionInicial_izq_down_up, finalFrame_izq_down_up)
        q_init_izq_up_down = q_out_izq_up_down2  # initial angles  #CUIDADO
        desiredFrame6.p[0] = desiredFrame6.p[0]
        desiredFrame6.p[1] = desiredFrame6.p[1]-0.05
        desiredFrame6.p[2] = desiredFrame6.p[2]#+0.01
        print "Desired Position: ", desiredFrame6.p
        q_out_izq_up_down = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame6, q_out_izq_up_down)
        print "Output angles in rads_izq_121: ", q_out_izq_up_down

        piernas_goal121[6] = q_out_izq_up_down[0]
        piernas_goal121[7] = q_out_izq_up_down[1]
        piernas_goal121[8] = q_out_izq_up_down[2]
        piernas_goal121[9] = q_out_izq_up_down[3]
        piernas_goal121[10] = q_out_izq_up_down[4]
        piernas_goal121[11] = q_out_izq_up_down[5]

        print "Inverse Kinematics"
        desiredFrame06 = PyKDL.Frame()
        fksolver_der_up_down.JntToCart(q_out_der_up_down, desiredFrame06)
        fksolver_der_down_up.JntToCart(posicionInicial_der_down_up, finalFrame_der_down_up)
        q_init_der_up_down = q_out_der_up_down  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame06.p[0] = desiredFrame06.p[0]
        desiredFrame06.p[1] = desiredFrame06.p[1]
        desiredFrame06.p[2] = desiredFrame06.p[2]
        print "Desired Position: ", desiredFrame06.p
        q_out_der_up_down = PyKDL.JntArray(cadena_der_up_down.getNrOfJoints())
        ik_der_up_down.CartToJnt(q_init_der_up_down, desiredFrame06, q_out_der_up_down)
        print "Output angles in rads: ", q_out_der_up_down

        q_out_der_up_down21 = PyKDL.JntArray(6)
        q_out_der_up_down21 = [-.3, -.3, 0, .6, .3, -.3 ]
        piernas_goal121[0] = q_out_der_up_down21[0]
        piernas_goal121[1] = q_out_der_up_down21[1]
        piernas_goal121[2] = q_out_der_up_down21[2]
        piernas_goal121[3] = q_out_der_up_down21[3]
        piernas_goal121[4] = q_out_der_up_down21[4]
        piernas_goal121[5] = q_out_der_up_down21[5]

        # 55555555555555555555555555555555555555555
        piernas_goal25 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print "Inverse Kinematics"
        desiredFrame7= PyKDL.Frame()
        q_init_izq_down_up3 = PyKDL.JntArray(6)
        q_init_izq_down_up3[0] = q_out_izq_up_down[5] * 1
        q_init_izq_down_up3[1] = q_out_izq_up_down[4] * 1
        q_init_izq_down_up3[2] = q_out_izq_up_down[3] * 1
        q_init_izq_down_up3[3] = q_out_izq_up_down[2] * 1
        q_init_izq_down_up3[4] = q_out_izq_up_down[1] * 1
        q_init_izq_down_up3[5] = q_out_izq_up_down[0] * 1
        fksolver_izq_down_up.JntToCart(q_init_izq_down_up3, desiredFrame7)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame7.p[0] = desiredFrame7.p[0] + 0.05#0.03  # PROBAR A PONERLO MAYOR
        desiredFrame7.p[1] = desiredFrame7.p[1] - 0.06#0.04
        desiredFrame7.p[2] = desiredFrame7.p[2] + 0.005
        print "Desired Position2222: ", desiredFrame7.p
        q_out_izq_down_up5 = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_down_up.CartToJnt(q_init_izq_down_up3, desiredFrame7, q_out_izq_down_up5)
        print "Output angles in rads2222AAAAA: ", q_out_izq_down_up5

        piernas_goal25[6] = q_out_izq_down_up5[5]
        piernas_goal25[7] = q_out_izq_down_up5[4]
        piernas_goal25[8] = q_out_izq_down_up5[3]
        piernas_goal25[9] = q_out_izq_down_up5[2]
        piernas_goal25[10] = q_out_izq_down_up5[1]-0.05
        piernas_goal25[11] = q_out_izq_down_up5[0]+0.05

        print "Inverse Kinematics"
        q_init_der_down_up31 = PyKDL.JntArray(6)
        q_init_der_down_up31[0] = q_out_der_up_down21[5] *1 # PROBLEMAS CON LA ASIGNACION
        q_init_der_down_up31[1] = q_out_der_up_down21[4] *1
        q_init_der_down_up31[2] = q_out_der_up_down21[3] *1
        q_init_der_down_up31[3] = q_out_der_up_down21[2] *1
        q_init_der_down_up31[4] = q_out_der_up_down21[1] *1
        q_init_der_down_up31[5] = q_out_der_up_down21[0] *1
        desiredFrame7 = PyKDL.Frame()
        fksolver_der_down_up.JntToCart(q_init_der_down_up31, desiredFrame7)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame7.p[0] = desiredFrame7.p[0] + 0.05
        desiredFrame7.p[1] = desiredFrame7.p[1] - 0.06
        desiredFrame7.p[2] = desiredFrame7.p[2] - 0.02
        print "Desired Position2222der: ", desiredFrame7.p
        q_out_der_down_up71 = PyKDL.JntArray(cadena_der_down_up.getNrOfJoints())
        ik_der_down_up.CartToJnt(q_init_der_down_up31, desiredFrame7, q_out_der_down_up71)
        print "Output angles in rads22222der: ", q_out_der_down_up71
        piernas_goal25[0] = q_out_der_down_up71[5]
        piernas_goal25[1] = q_out_der_down_up71[4]
        piernas_goal25[2] = q_out_der_down_up71[3]
        piernas_goal25[3] = q_out_der_down_up71[2]
        piernas_goal25[4] = q_out_der_down_up71[1]
        piernas_goal25[5] = q_out_der_down_up71[0]

        # 333333333333333333333333333333333333333333333333
        piernas_goal341 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        print "Inverse Kinematics"
        desiredFrame441 = PyKDL.Frame()
        fksolver_der_down_up.JntToCart(q_out_der_down_up71, desiredFrame441)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame441.p[0] = desiredFrame441.p[0]
        desiredFrame441.p[1] = desiredFrame441.p[1] - 0.02
        desiredFrame441.p[2] = desiredFrame441.p[2] - 0.015
        ik_der_down_up.CartToJnt(q_out_der_down_up71, desiredFrame441, q_out_der_down_up71)

        q_init_der_up_down[0] = q_out_der_down_up71[5]
        q_init_der_up_down[1] = q_out_der_down_up71[4]
        q_init_der_up_down[2] = q_out_der_down_up71[3]
        q_init_der_up_down[3] = q_out_der_down_up71[2]
        q_init_der_up_down[4] = q_out_der_down_up71[1]
        q_init_der_up_down[5] = q_out_der_down_up71[0]

        desiredFrame541 = PyKDL.Frame()
        fksolver_der_up_down.JntToCart(q_init_der_up_down, desiredFrame541)
        desiredFrame541.p[0] = desiredFrame541.p[0] - 0.03
        desiredFrame541.p[1] = desiredFrame541.p[1] - 0.1
        desiredFrame541.p[2] = desiredFrame541.p[2] - 0.01 #nada
        print "Desired Position: ", desiredFrame541.p
        q_out_der_up_down241 = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_der_up_down.CartToJnt(q_init_der_up_down, desiredFrame541, q_out_der_up_down241)# con desiredFrame5 va
        print "Output angles in radsaaaaa: ", q_out_der_up_down241

        print "Inverse Kinematics"

        piernas_goal341[6] = 0.3
        piernas_goal341[7] = -0.3
        piernas_goal341[8] = 0
        piernas_goal341[9] = 0.6
        piernas_goal341[10] = -0.3
        piernas_goal341[11] = -0.3

        piernas_goal341[0] = q_out_der_up_down241[0]#+0.1
        piernas_goal341[1] = q_out_der_up_down241[1]
        piernas_goal341[2] = q_out_der_up_down241[2]
        piernas_goal341[3] = q_out_der_up_down241[3]
        piernas_goal341[4] = q_out_der_up_down241[4]#-0.1
        piernas_goal341[5] = q_out_der_up_down241[5]#-0.01

        pickle.dump(piernas_goal, datos_articulaciones, -1)
        pickle.dump(piernas_goal12, datos_articulaciones, -1)
        pickle.dump(piernas_goal2, datos_articulaciones, -1)
        pickle.dump(piernas_goal3, datos_articulaciones, -1)
        pickle.dump(piernas_goal121, datos_articulaciones, -1)
        pickle.dump(piernas_goal25, datos_articulaciones, -1)
        pickle.dump(piernas_goal341, datos_articulaciones, -1)

        datos_articulaciones.close()

        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        arm_client = actionlib.SimpleActionClient('/piernas/piernas_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        #/piernas/piernas_controller/follow_joint_trajectory
        arm_client.wait_for_server()

        rospy.loginfo('...connected.')



        # Create a single-point arm trajectory with the piernas_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = piernas_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = piernas_goal
        arm_trajectory.points[0].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(2.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[1].positions = piernas_goal12
        arm_trajectory.points[1].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].time_from_start = rospy.Duration(4.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[2].positions = piernas_goal2
        arm_trajectory.points[2].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].time_from_start = rospy.Duration(6.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[3].positions = piernas_goal3
        arm_trajectory.points[3].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].time_from_start = rospy.Duration(8.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[4].positions = piernas_goal121
        arm_trajectory.points[4].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].time_from_start = rospy.Duration(10.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[5].positions = piernas_goal25
        arm_trajectory.points[5].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].time_from_start = rospy.Duration(12.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[6].positions = piernas_goal341
        arm_trajectory.points[6].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].time_from_start = rospy.Duration(14.0)

        '''arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[7].positions = piernas_goal12
        arm_trajectory.points[7].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].time_from_start = rospy.Duration(17.5)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[8].positions = piernas_goal2
        arm_trajectory.points[8].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].time_from_start = rospy.Duration(19.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[9].positions = piernas_goal3
        arm_trajectory.points[9].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].time_from_start = rospy.Duration(21.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[10].positions = piernas_goal121
        arm_trajectory.points[10].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].time_from_start = rospy.Duration(23.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[11].positions = piernas_goal25
        arm_trajectory.points[11].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].time_from_start = rospy.Duration(25.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[12].positions = piernas_goal341
        arm_trajectory.points[12].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].time_from_start = rospy.Duration(28.0)'''
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')



        # Create an empty trajectory goal
        piernas_goal = FollowJointTrajectoryGoal()

        # Set the trajectory component to the goal trajectory created above
        piernas_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        piernas_goal.goal_time_tolerance = rospy.Duration(0.01)

        # Send the goal to the action server
        arm_client.send_goal(piernas_goal)

        if not sync:
            # Wait for up to 5 seconds for the motion to complete
            arm_client.wait_for_result(rospy.Duration(5.0))

        arm_client.wait_for_result()
        print arm_client.get_result()


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

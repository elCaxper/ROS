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

        pi4 = 0.7853

        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/mi_robot_xacro4_cambio.urdf'

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
        posicionInicial_der_up_down[0] = 0
        posicionInicial_der_up_down[1] = 0.3
        posicionInicial_der_up_down[2] = -0.3
        posicionInicial_der_up_down[3] = 0.6
        posicionInicial_der_up_down[4] = -0.3
        posicionInicial_der_up_down[5] = -0.3

        nj_izq = cadena_izq_up_down.getNrOfJoints()
        posicionInicial_izq_up_down = PyKDL.JntArray(nj_izq)
        posicionInicial_izq_up_down[0] = 0
        posicionInicial_izq_up_down[1] = 0.3
        posicionInicial_izq_up_down[2] = -0.3
        posicionInicial_izq_up_down[3] = 0.6
        posicionInicial_izq_up_down[4] = -0.3
        posicionInicial_izq_up_down[5] = -0.3

        nj_izq = cadena_der_down_up.getNrOfJoints()
        posicionInicial_der_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_der_down_up[5] = 0
        posicionInicial_der_down_up[4] = 0.3
        posicionInicial_der_down_up[3] = -0.3
        posicionInicial_der_down_up[2] = 0.6
        posicionInicial_der_down_up[1] = -0.3
        posicionInicial_der_down_up[0] = -0.3

        nj_izq = cadena_izq_down_up.getNrOfJoints()
        posicionInicial_izq_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_izq_down_up[5] = 0
        posicionInicial_izq_down_up[4] = 0.3
        posicionInicial_izq_down_up[3] = -0.3
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
        piernas_joints = ['cilinder_blue_box1_der_joint', 'cilinder_blue1_box2_der_joint','cilinder_blue_box2_der_joint',
                          'cilinder_blue_box4_der_joint', 'cilinder_blue1_box6_der_joint','cilinder_blue_box6_der_joint',
                          'cilinder_blue_box1_izq_joint', 'cilinder_blue1_box2_izq_joint','cilinder_blue_box2_izq_joint',
                          'cilinder_blue_box4_izq_joint', 'cilinder_blue1_box6_izq_joint','cilinder_blue_box6_izq_joint']

        piernas_goal0  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

        piernas_goal0[6] = q_out_izq_up_down[0]
        piernas_goal0[7] = q_out_izq_up_down[1]
        piernas_goal0[8] = q_out_izq_up_down[2]
        piernas_goal0[9] = q_out_izq_up_down[3]
        piernas_goal0[10] = q_out_izq_up_down[4]
        piernas_goal0[11] = q_out_izq_up_down[5]

        piernas_goal0[0] = q_out_der_up_down[0]
        piernas_goal0[1] = q_out_der_up_down[1]
        piernas_goal0[2] = q_out_der_up_down[2]
        piernas_goal0[3] = q_out_der_up_down[3]
        piernas_goal0[4] = q_out_der_up_down[4]
        piernas_goal0[5] = q_out_der_up_down[5]

        #121212122121212121212121212121212121212121212
        piernas_goal12 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #q_out_izq_up_down[0] -= pi4
        piernas_goal12[6] = q_out_izq_up_down[0]-2*pi4
        piernas_goal12[7] = q_out_izq_up_down[1]
        piernas_goal12[8] = q_out_izq_up_down[2]
        piernas_goal12[9] = q_out_izq_up_down[3]
        piernas_goal12[10] = q_out_izq_up_down[4]
        piernas_goal12[11] = q_out_izq_up_down[5]

        piernas_goal12[0] = 0
        piernas_goal12[1] = 0
        piernas_goal12[2] = -0.7
        piernas_goal12[3] = 1.4
        piernas_goal12[4] = 0
        piernas_goal12[5] = -0.7

        #########################################################

        piernas_goal2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        piernas_goal2[6] = q_out_izq_up_down[0]-2*pi4
        piernas_goal2[7] = q_out_izq_up_down[1]-0.3-0.3
        piernas_goal2[8] = q_out_izq_up_down[2]-0.3
        piernas_goal2[9] = q_out_izq_up_down[3]+0.6
        piernas_goal2[10] = q_out_izq_up_down[4]+0.3+0.3
        piernas_goal2[11] = q_out_izq_up_down[5] -0.3

        piernas_goal2[0] = 0#-pi4
        piernas_goal2[1] = -0.25
        piernas_goal2[2] = -0.3
        piernas_goal2[3] = 0.6
        piernas_goal2[4] = 0.25
        piernas_goal2[5] = -0.3

        ##################################################
        piernas_goal3 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        piernas_goal3[6] = q_out_izq_up_down[0]   +0.0
        piernas_goal3[7] = q_out_izq_up_down[1] - 0.3
        piernas_goal3[8] = q_out_izq_up_down[2] - 0.3
        piernas_goal3[9] = q_out_izq_up_down[3] + 0.6
        piernas_goal3[10] = q_out_izq_up_down[4] + 0.3
        piernas_goal3[11] = q_out_izq_up_down[5] - 0.3

        piernas_goal3[0] = -2*pi4
        piernas_goal3[1] = -0.25
        piernas_goal3[2] = -0.3
        piernas_goal3[3] = 0.6
        piernas_goal3[4] = 0.25
        piernas_goal3[5] = -0.3

        ##################################################
        piernas_goal4 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        piernas_goal4[6] = q_out_izq_up_down[0] +0.0
        piernas_goal4[7] = q_out_izq_up_down[1] - 0.3  +0.3
        piernas_goal4[8] = q_out_izq_up_down[2] - 0.3   +0.3
        piernas_goal4[9] = q_out_izq_up_down[3] + 0.6  -0.6
        piernas_goal4[10] = q_out_izq_up_down[4] + 0.3 -0.3
        piernas_goal4[11] = q_out_izq_up_down[5] - 0.3 +0.3

        piernas_goal4[0] = -2*pi4
        piernas_goal4[1] = -0.25
        piernas_goal4[2] = -0.7
        piernas_goal4[3] = 1.4
        piernas_goal4[4] = 0.25
        piernas_goal4[5] = -0.7



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
        arm_trajectory.points[0].positions = piernas_goal0
        arm_trajectory.points[0].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[1].positions = piernas_goal12
        arm_trajectory.points[1].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].time_from_start = rospy.Duration(6.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[2].positions = piernas_goal2
        arm_trajectory.points[2].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].time_from_start = rospy.Duration(9.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[3].positions = piernas_goal3
        arm_trajectory.points[3].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].time_from_start = rospy.Duration(12.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[4].positions = piernas_goal4
        arm_trajectory.points[4].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].time_from_start = rospy.Duration(15.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[5].positions = piernas_goal12
        arm_trajectory.points[5].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].time_from_start = rospy.Duration(18.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[6].positions = piernas_goal2
        arm_trajectory.points[6].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].time_from_start = rospy.Duration(21.0)

        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[7].positions = piernas_goal3
        arm_trajectory.points[7].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].time_from_start = rospy.Duration(24)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[8].positions = piernas_goal4
        arm_trajectory.points[8].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].time_from_start = rospy.Duration(27)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[9].positions = piernas_goal12
        arm_trajectory.points[9].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].time_from_start = rospy.Duration(30.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[10].positions = piernas_goal2
        arm_trajectory.points[10].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].time_from_start = rospy.Duration(33.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[11].positions = piernas_goal3
        arm_trajectory.points[11].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].time_from_start = rospy.Duration(36.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[12].positions = piernas_goal4
        arm_trajectory.points[12].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].time_from_start = rospy.Duration(39.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[13].positions = piernas_goal12
        arm_trajectory.points[13].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[13].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[13].time_from_start = rospy.Duration(42.0)
        '''arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[14].positions = piernas_goal4
        arm_trajectory.points[14].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[14].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[14].time_from_start = rospy.Duration(32.0)'''
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

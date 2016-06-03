#!/usr/bin/env python

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL
from angles import normalize

class TrajectoryDemo():
    def __init__(self):

        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/urdf_exportado4.urdf'
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
        piernas_joints = ['cilinder_blue1_box1_der_joint', 'cilinder_blue_box1_der_joint', 'cilinder_blue_box2_der_joint',
                        'cilinder_blue_box4_der_joint', 'cilinder_blue1_box6_der_joint', 'cilinder_blue_box6_der_joint',
                          'cilinder_blue1_box1_izq_joint', 'cilinder_blue_box1_izq_joint', 'cilinder_blue_box2_izq_joint',
                          'cilinder_blue_box4_izq_joint', 'cilinder_blue1_box6_izq_joint', 'cilinder_blue_box6_izq_joint' ]


        if reset:
            # Set the arm back to the resting position
            arm_goal  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        else:
            # Set a goal configuration for the arm
            arm_goal  =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #11111111111111111111111111111111111111111111
        print "Inverse Kinematics"
        fksolver_izq_up_down.JntToCart(posicionInicial_izq_up_down, finalFrame_izq_up_down)
        fksolver_izq_down_up.JntToCart(posicionInicial_izq_down_up, finalFrame_izq_down_up)
        q_init_izq_up_down = posicionInicial_izq_up_down  # initial angles
        #desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame = finalFrame_izq_up_down
        desiredFrame.p[0] = finalFrame_izq_up_down.p[0]
        desiredFrame.p[1] = finalFrame_izq_up_down.p[1]
        desiredFrame.p[2] = finalFrame_izq_up_down.p[2]
        print "Desired Position: ", desiredFrame.p
        q_out_izq_up_down = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame, q_out_izq_up_down)
        print "Output angles in rads: ", q_out_izq_up_down

        arm_goal[6] = q_out_izq_up_down[0]
        arm_goal[7] = q_out_izq_up_down[1]
        arm_goal[8] = q_out_izq_up_down[2]
        arm_goal[9] = q_out_izq_up_down[3]
        arm_goal[10] = q_out_izq_up_down[4]
        arm_goal[11] = q_out_izq_up_down[5]

        print "Inverse Kinematics"
        fksolver_der_up_down.JntToCart(posicionInicial_der_up_down, finalFrame_der_up_down)
        fksolver_der_down_up.JntToCart(posicionInicial_der_down_up, finalFrame_der_down_up)
        q_init_der_up_down = posicionInicial_der_up_down  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame = finalFrame_der_up_down
        desiredFrame.p[0] = finalFrame_der_up_down.p[0]
        desiredFrame.p[1] = finalFrame_der_up_down.p[1]-0.06
        desiredFrame.p[2] = finalFrame_der_up_down.p[2]+0.02
        print "Desired Position: ", desiredFrame.p
        q_out_der_up_down = PyKDL.JntArray(cadena_der_up_down.getNrOfJoints())
        ik_der_up_down.CartToJnt(q_init_der_up_down, desiredFrame, q_out_der_up_down)
        print "Output angles in rads: ", q_out_der_up_down

        arm_goal[0] = q_out_der_up_down[0]
        arm_goal[1] = q_out_der_up_down[1]
        arm_goal[2] = q_out_der_up_down[2]
        arm_goal[3] = q_out_der_up_down[3]
        arm_goal[4] = q_out_der_up_down[4]
        arm_goal[5] = q_out_der_up_down[5]

        # 222222222222222222222222222222222222222222
        arm_goal2 =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print "Inverse Kinematics"
        q_init_izq_down_up = posicionInicial_izq_down_up  # initial angles
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame2 = finalFrame_izq_down_up
        desiredFrame2.p[0] = finalFrame_izq_down_up.p[0] - 0.05
        desiredFrame2.p[1] = finalFrame_izq_down_up.p[1] - 0.06
        desiredFrame2.p[2] = finalFrame_izq_down_up.p[2] - 0.02
        print "Desired Position: ", desiredFrame2.p
        q_out_izq_down_up = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_down_up.CartToJnt(q_init_izq_down_up, desiredFrame2, q_out_izq_down_up)
        print "Output angles in rads: ", q_out_izq_down_up

        arm_goal2[6] = q_out_izq_down_up[5]
        arm_goal2[7] = q_out_izq_down_up[4]
        arm_goal2[8] = q_out_izq_down_up[3]
        arm_goal2[9] = q_out_izq_down_up[2]
        arm_goal2[10] = q_out_izq_down_up[1]
        arm_goal2[11] = q_out_izq_down_up[0]

        print "Inverse Kinematics"
        q_init_der_down_up = q_out_der_up_down  # initial angles
        q_init_der_down_up[0] = q_out_der_up_down[5]
        q_init_der_down_up[1] = q_out_der_up_down[4]
        q_init_der_down_up[2] = q_out_der_up_down[3]
        q_init_der_down_up[3] = q_out_der_up_down[2]
        q_init_der_down_up[4] = q_out_der_up_down[1]
        q_init_der_down_up[5] = q_out_der_up_down[0]
        fksolver_der_down_up.JntToCart(q_init_der_down_up,finalFrame_der_down_up)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame3 = finalFrame_der_down_up
        desiredFrame3.p[0] = finalFrame_der_down_up.p[0] - 0.05
        desiredFrame3.p[1] = finalFrame_der_down_up.p[1] - 0.06
        desiredFrame3.p[2] = finalFrame_der_down_up.p[2] - 0.02
        print "Desired Position: ", desiredFrame3.p
        q_out_der_down_up = PyKDL.JntArray(cadena_der_up_down.getNrOfJoints())
        ik_der_down_up.CartToJnt(q_init_der_down_up, desiredFrame3, q_out_der_down_up)
        print "Output angles in rads: ", q_out_der_down_up

        arm_goal2[0] = q_out_der_down_up[5]
        arm_goal2[1] = q_out_der_down_up[4]
        arm_goal2[2] = q_out_der_down_up[3]
        arm_goal2[3] = q_out_der_down_up[2]
        arm_goal2[4] = q_out_der_down_up[1]
        arm_goal2[5] = q_out_der_down_up[0]


        # 333333333333333333333333333333333333333333333333
        arm_goal3 = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        print "Inverse Kinematics"
        fksolver_izq_down_up.JntToCart(q_out_izq_down_up,finalFrame_izq_down_up)
        # desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame3 = finalFrame_izq_down_up
        desiredFrame3.p[0] = finalFrame_izq_down_up.p[0]
        desiredFrame3.p[1] = finalFrame_izq_down_up.p[1] - 0.02
        desiredFrame3.p[2] = finalFrame_izq_down_up.p[2]
        ik_izq_down_up.CartToJnt(q_out_izq_down_up, desiredFrame3, q_out_izq_down_up)

        q_init_izq_up_down[0] = q_out_izq_down_up[5]
        q_init_izq_up_down[1] = q_out_izq_down_up[4]
        q_init_izq_up_down[2] = q_out_izq_down_up[3]
        q_init_izq_up_down[3] = q_out_izq_down_up[2]
        q_init_izq_up_down[4] = q_out_izq_down_up[1]
        q_init_izq_up_down[5] = q_out_izq_down_up[0]
        fksolver_izq_up_down.JntToCart(q_init_izq_up_down,finalFrame_izq_up_down)

        desiredFrame4 = finalFrame_izq_up_down
        desiredFrame4.p[0] = finalFrame_izq_up_down.p[0]
        desiredFrame4.p[1] = finalFrame_izq_up_down.p[1] - 0.13
        desiredFrame4.p[2] = finalFrame_izq_up_down.p[2] + 0.012
        print "Desired Position: ", desiredFrame4.p
        q_out_izq_up_down2 = PyKDL.JntArray(cadena_izq_up_down.getNrOfJoints())
        ik_izq_up_down.CartToJnt(q_init_izq_up_down, desiredFrame4, q_out_izq_up_down2)
        print "Output angles in rads: ", q_out_izq_up_down2
        arm_goal3[6] = q_out_izq_up_down2[0]
        arm_goal3[7] = q_out_izq_up_down2[1]
        arm_goal3[8] = q_out_izq_up_down2[2]
        arm_goal3[9] = q_out_izq_up_down2[3]
        arm_goal3[10] = q_out_izq_up_down2[4]
        arm_goal3[11] = q_out_izq_up_down2[5]

        print "Inverse Kinematics"

        arm_goal3[0] = -0.4
        arm_goal3[1] = -0.25
        arm_goal3[2] = 0
        arm_goal3[3] = 0.5
        arm_goal3[4] = 0.4
        arm_goal3[5] = -0.25

        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')

        arm_client = actionlib.SimpleActionClient('/piernas/piernas_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        arm_client.wait_for_server()

        rospy.loginfo('...connected.')


        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = piernas_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(1.0)
        #arm_trajectory.points.append(JointTrajectoryPoint())
        #arm_trajectory.points[1].positions = arm_goal2
        #arm_trajectory.points[1].velocities = [0.0 for i in piernas_joints]
        #arm_trajectory.points[1].accelerations = [0.0 for i in piernas_joints]
        #arm_trajectory.points[1].time_from_start = rospy.Duration(3.0)
        #arm_trajectory.points.append(JointTrajectoryPoint())
        #arm_trajectory.points[2].positions = arm_goal3
        #arm_trajectory.points[2].velocities = [0.0 for i in piernas_joints]
        #arm_trajectory.points[2].accelerations = [0.0 for i in piernas_joints]
        #arm_trajectory.points[2].time_from_start = rospy.Duration(4.0)
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')

        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()

        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory

        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.01)

        # Send the goal to the action server
        arm_client.send_goal(arm_goal)

        if not sync:
            # Wait for up to 5 seconds for the motion to complete
            arm_client.wait_for_result(rospy.Duration(5.0))


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

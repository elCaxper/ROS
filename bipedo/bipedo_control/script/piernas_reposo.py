#!/usr/bin/env python

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import urdf_parser_py.urdf
import urdf_parser_py.xml_reflection
import kdl_parser_py.urdf
import PyKDL

class TrajectoryDemo():
    def __init__(self):
        filename = '/home/kaiser/WS_ROS/catkin_ws/src/urdf_serp/urdf/urdf_exportado3.urdf'
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
        posicionInicial_izq_up_down[2] = 0
        posicionInicial_izq_up_down[3] = 0.6
        posicionInicial_izq_up_down[4] = -0.3
        posicionInicial_izq_up_down[5] = -0.3

        nj_izq = cadena_der_down_up.getNrOfJoints()
        posicionInicial_der_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_der_down_up[5] = 0.3
        posicionInicial_der_down_up[4] = -0.3
        posicionInicial_der_down_up[3] = 0
        posicionInicial_der_down_up[2] = 0.6
        posicionInicial_der_down_up[1] = -0.3
        posicionInicial_der_down_up[0] = -0.3

        nj_izq = cadena_izq_down_up.getNrOfJoints()
        posicionInicial_izq_down_up = PyKDL.JntArray(nj_izq)
        posicionInicial_izq_down_up[5] = 0.3
        posicionInicial_izq_down_up[4] = -0.3
        posicionInicial_izq_down_up[3] = 0
        posicionInicial_izq_down_up[2] = 0.6
        posicionInicial_izq_down_up[1] = -0.3
        posicionInicial_izq_down_up[0] = -0.3
        print "Forward kinematics"
        finalFrame_izq_up_down = PyKDL.Frame()
        finalFrame_izq_down_up = PyKDL.Frame()
        finalFrame_der_up_down = PyKDL.Frame()
        finalFrame_der_down_up = PyKDL.Frame()

        fksolver_izq_up_down.JntToCart(posicionInicial_izq_up_down, finalFrame_izq_up_down)
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
            arm_goal  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        else:
            # Set a goal configuration for the arm
            arm_goal  = [0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        print "Inverse Kinematics"
        q_init = posicionInicial_izq_up_down  # initial angles
        #desiredFrame = PyKDL.Frame(PyKDL.Vector(0.5, 0.4, 1))
        desiredFrame = finalFrame_izq_up_down
        desiredFrame.p[0] = finalFrame_izq_up_down.p[0]
        desiredFrame.p[1] = finalFrame_izq_up_down.p[1]
        desiredFrame.p[2] = finalFrame_izq_up_down.p[2]
        print "Desired Position: ", desiredFrame.p
        q_out = PyKDL.JntArray(6)
        ik_izq_up_down.CartToJnt(q_init, desiredFrame, q_out)
        print "Output angles in rads: ", q_out

        #arm_goal[0] = q_out[0]
        #arm_goal[1] = q_out[1]
        #arm_goal[2] = q_out[2]
        #arm_goal[3] = q_out[3]
        #arm_goal[4] = q_out[4]
        #arm_goal[5] = q_out[5]


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

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

        pkl_file = open('datos_articulacionespkl.pkl', 'rb')

        # data1 = pickle.load(pkl_file)
        # pprint.pprint(data1)

        # data2 = pickle.load(pkl_file)
        # pprint.pprint(data2)

        # print data1['b'][0]
        # for i in data2:
        #    print i

        datos = []
        try:
            while True:  # loop indefinitely
                print "LEYENDO"
                # print pickle.load(pkl_file)
                datos.append(pickle.load(pkl_file))  # add each item from the file to a list
        except EOFError:  # the exception is used to break the loop
            pass  # we don't need to do anything special

        pkl_file.close()
        print len(datos)

        # print datos_trans
        for i in datos:
            pass
            # print i


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

        # Create a single-point arm trajectory with the piernas_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = piernas_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = datos[0]
        arm_trajectory.points[0].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(1.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[1].positions = datos[1]
        arm_trajectory.points[1].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[1].time_from_start = rospy.Duration(4.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[2].positions = datos[2]
        arm_trajectory.points[2].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[2].time_from_start = rospy.Duration(6.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[3].positions = datos[3]
        arm_trajectory.points[3].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[3].time_from_start = rospy.Duration(8.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[4].positions = datos[4]
        arm_trajectory.points[4].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[4].time_from_start = rospy.Duration(10.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[5].positions = datos[5]
        arm_trajectory.points[5].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[5].time_from_start = rospy.Duration(12.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[6].positions = datos[6]
        arm_trajectory.points[6].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[6].time_from_start = rospy.Duration(14.0)

        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[7].positions = datos[1]
        arm_trajectory.points[7].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[7].time_from_start = rospy.Duration(17.5)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[8].positions = datos[2]
        arm_trajectory.points[8].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[8].time_from_start = rospy.Duration(19.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[9].positions = datos[3]
        arm_trajectory.points[9].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[9].time_from_start = rospy.Duration(21.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[10].positions = datos[4]
        arm_trajectory.points[10].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[10].time_from_start = rospy.Duration(23.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[11].positions = datos[5]
        arm_trajectory.points[11].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[11].time_from_start = rospy.Duration(25.0)
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[12].positions = datos[6]
        arm_trajectory.points[12].velocities = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].accelerations = [0.0 for i in piernas_joints]
        arm_trajectory.points[12].time_from_start = rospy.Duration(28.0)
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
        arm_client.send_goal(piernas_goal)
        print arm_client.get_result()


        rospy.loginfo('...done')

if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass

#! /usr/bin/env python

import roslib
import rospy
import sys
sys.path.append('/root/mraa/build/src/python')
# import mraa
import math
#from ctypes import ctypes.c_ubyte
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult
from control_msgs.msg import FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
import actionlib
from control_msgs.msg._FollowJointTrajectoryActionFeedback import FollowJointTrajectoryActionFeedback
from std_msgs.msg import Empty
import ctypes

# x = mraa.I2c(0)

class TrajectoryAction():
  # create messages that are used to publish feedback/result
  _feedback = FollowJointTrajectoryFeedback()
  _result   = FollowJointTrajectoryResult()

  def __init__(self):
    self._action_name = 'Prueba Accion'
    self._as = actionlib.SimpleActionServer('joint_trajectory_action', FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

  '''x.address(0x07)
  x.writeByte(183)
  rospy.sleep(rospy.Duration(0, 5000000))
  x.writeByte(ctypes.c_uint8(5).value)'''


  def execute_cb(self, goal):
    self._result.SUCCESSFUL
    self._result.error_code = 10
    self._result.error_string = 'Leido y enviado'
    self._as.set_succeeded(self._result)
    # helper variables
    r = rospy.Rate(50)
    success = True
    print goal.trajectory.points

    # start executing the action
    for i in xrange(0, len(goal.trajectory.points)):
      # check that preempt has not been requested by the client
      print goal.trajectory.points[i].positions
      # goal.trajectory.points[i].velocities

      x.address(0x05)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0,5000000))
      j_1v = goal.trajectory.points[i].velocities[0]
      #x.writeByte(ctypes.c_uint8(int(round(j_1v))).value)
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_1 = goal.trajectory.points[i].positions[0]
      j_1b = ctypes.c_uint8(int(round(j_1*180/math.pi)+90)).value
      x.writeByte(j_1b)
      rospy.sleep(rospy.Duration(0, 30000000))


      x.address(0x07)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_2v = goal.trajectory.points[i].velocities[1]
      #x.writeByte(ctypes.c_uint8(int(round(j_2v))).value)
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_2 = goal.trajectory.points[i].positions[1]
      j_2b = ctypes.c_uint8(int(round(j_2*180/math.pi)+90)).value
      x.writeByte(j_2b)

      x.address(0x08)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_3v = goal.trajectory.points[i].velocities[2]
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      #x.writeByte(ctypes.c_uint8(int(round(j_3v))).value)
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_3 = goal.trajectory.points[i].positions[2]
      j_3b = ctypes.c_uint8(int(round(j_3*180/math.pi)+90)).value
      x.writeByte(j_3b)
      rospy.sleep(rospy.Duration(0, 30000000))

      x.address(0x03)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_4v = goal.trajectory.points[i].velocities[3]
      #x.writeByte(ctypes.c_uint8(int(round(j_4v))).value)
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_4 = goal.trajectory.points[i].positions[3]
      j_4b = ctypes.c_ushort(int(round(j_4*180/math.pi)+90)).value
      x.writeByte(j_4b)
      rospy.sleep(rospy.Duration(0, 30000000))

      x.address(0x04)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_5v = goal.trajectory.points[i].velocities[4]
      #x.writeByte(ctypes.c_uint8(int(round(j_5v))).value)
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_5 = goal.trajectory.points[i].positions[4]
      j_5b = ctypes.c_uint8(int(round(j_5*180/math.pi)+90)).value
      x.writeByte(j_5b)
      rospy.sleep(rospy.Duration(0, 30000000))

      x.address(0x06)
      x.writeByte(ctypes.c_uint8(181).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_6v = goal.trajectory.points[i].velocities[5]
      #x.writeByte(ctypes.c_uint8(int(round(j_6v))).value)
      x.writeByte(ctypes.c_uint8(1).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      x.writeByte(ctypes.c_uint8(182).value)
      rospy.sleep(rospy.Duration(0, 30000000))
      j_6 = goal.trajectory.points[i].positions[5]
      j_6b = ctypes.c_uint8(int(round(j_6*180/math.pi)+90)).value
      x.writeByte(j_6b)
      rospy.sleep(rospy.Duration(0, 30000000))

      
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        #self._as.set_preempted()
        #success = False
        break

      r.sleep()

    '''if success:
      #self._result.sequence = self._feedback.sequence
      rospy.loginfo('%s: Succeeded' % self._action_name)
      self._result.SUCCESSFUL
      self._result.error_code = 10
      self._result.error_string = 'Leido y enviado'
      self._as.set_succeeded(self._result)'''

if __name__ == '__main__':
  rospy.init_node('servidor')
  TrajectoryAction()
  rospy.spin()

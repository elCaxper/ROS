import rospy
from std_msgs.msg import String
from sensor_msgs.msg import  JointState
import mraa
sys.path.append('/root/mraa/build/src/python')
# import mraa
import math
import ctypes

x = mraa.I2c(0)
conversion_vel = 90.0/6.25

def callback(data):
    # print data.velocity
    mi_lista=  [a * b for a, b in zip(data.velocity, [x+conversion_vel for x in [0] * len(data.velocity)])]
    mi_lista = [round(x+90) for x in mi_lista]

    # RF
    x.address(0x05)
    x.writeByte(ctypes.c_uint8(183).value)
    rospy.sleep(rospy.Duration(0, 5000000))
    x.writeByte(ctypes.c_uint8(mi_lista[2]).value)
    rospy.sleep(rospy.Duration(0, 5000000))

    # RR
    x.address(0x04)
    x.writeByte(ctypes.c_uint8(183).value)
    rospy.sleep(rospy.Duration(0, 5000000))
    x.writeByte(ctypes.c_uint8(mi_lista[4]).value)
    rospy.sleep(rospy.Duration(0, 5000000))

    # LF
    x.address(0x07)
    x.writeByte(ctypes.c_uint8(183).value)
    rospy.sleep(rospy.Duration(0, 5000000))
    x.writeByte(ctypes.c_uint8(mi_lista[1]).value)
    rospy.sleep(rospy.Duration(0, 5000000))

    # LR
    x.address(0x08)
    x.writeByte(ctypes.c_uint8(183).value)
    rospy.sleep(rospy.Duration(0, 5000000))
    x.writeByte(ctypes.c_uint8(mi_lista[3]).value)
    rospy.sleep(rospy.Duration(0, 5000000))



def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
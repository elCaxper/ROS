import rospy
from std_msgs.msg import String
from sensor_msgs.msg import  JointState

conversion_vel = 90.0/6.25

def callback(data):
    # print data.velocity
    mi_lista=  [a * b for a, b in zip(data.velocity, [x+conversion_vel for x in [0] * len(data.velocity)])]
    print [round(x+90) for x in mi_lista]
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
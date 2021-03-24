
import rospy
from sensor_msgs.msg import JointState
import numpy as np

def talker(pos, lbl, freq, skipnr):
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('rplayer', anonymous=True)
    rate = rospy.Rate(freq) # 10hz
    idx  = 0
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        JointState js;
        js.header.stamp = rospy.get_time()
        js.name = lbl
        js.position = pos[idx,:].tolist()
        pub.publish(js)
        rate.sleep()
        idx = idx + skipnr
        if idx > pos.shape[0]:
            idx = 0
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

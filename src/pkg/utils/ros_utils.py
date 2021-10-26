
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

class Listener:

    def __init__(self, topic_name, topic_type):
        self.topic_name, self.topic_type = topic_name, topic_type
        self.data_stack = []

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.data_stack.append(data)

    def get_data(self, timeout=5):
        return rospy.wait_for_message(self.topic_name, self.topic_type,
                                      timeout=timeout)

    ##
    # @brief spin() simply keeps python from exiting until this node is stopped
    def spin(self):
        rospy.Subscriber(self.topic_name, self.topic_type, self.callback)
        rospy.spin()
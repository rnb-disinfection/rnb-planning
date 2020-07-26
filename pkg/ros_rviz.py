from __future__ import print_function
import time as timer
import numpy as np
import uuid
import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation

from .geometry import *

def get_publisher(joint_names, robot_name=""):
    pub = rospy.Publisher(robot_name+'/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    joints = JointState()
    joints.header.seq += 1
    joints.header.stamp = rospy.Time.now()
    joints.name = joint_names
    joints.position = [0]*len(joint_names)
    # Publish the marker
    while pub.get_num_connections() < 1:
        print("Please create a subscriber to the marker");
        timer.sleep(1)
    print('publication OK')
    pub.publish(joints);
    print('published: {}'.format(joints.position))
    return pub, joints, rate

def show_motion(pose_list, marker_list, gframevec_list, pub, joints, error_skip=1e-6, period=1e-6):
    pvec_last = np.array(pose_list)+1
    for pvec, gframevec in zip(pose_list, gframevec_list):
        if np.linalg.norm(pvec-pvec_last)<error_skip:
            break
        pvec_last = pvec
        joints.header.seq += 1
        joints.header.stamp = rospy.Time.now()
        joints.position = pvec.tolist()
        pub.publish(joints);
        for marker, gframe in zip(marker_list, gframevec):
            marker.move_marker(gframe)
        timer.sleep(period)

def show_motion_dict(pose_list_dict, marker_list_dict, pub_dict, joints_dict, joint_names_dict, error_skip=1e-6, period=1e-6):
    for robot, pose_list in pose_list_dict:
        marker_list, pub, joints, joint_names =\
            marker_list_dict[robot], pub_dict[robot], joints_dict[robot], joint_names_dict[robot]
        pvec_last = np.array(pose_list)+1
        for pvec in pose_list:
            if np.linalg.norm(pvec-pvec_last)<error_skip:
                break
            pvec_last = pvec
            joints.header.seq += 1
            joints.header.stamp = rospy.Time.now()
            joints.position = pvec.tolist()
            pub.publish(joints);
            for marker in marker_list:
                marker.move_marker({joints.name[i]: joints.position[i] for i in range(len(joint_names))})
            timer.sleep(period)
    #         print('published: {}'.format(joints.position), end="\r")
        
def set_markers(gitem_list, gframe_dict, urdf_content):
    marker_list = []
    for gitem in gitem_list:
        if gitem.display:
            marker_list += [GeoMarker(geometry=gitem,urdf_content=urdf_content)]
            marker_list[-1].set_marker(gframe_dict[gitem.name])
    return marker_list

class GeoMarker:
    ID_COUNT = 0
    def __init__(self, geometry, urdf_content, mark_name='visualization_marker', node_name=None):
        if node_name is None:
            node_name = mark_name + "_pub"
        self.geometry = geometry
        self.urdf_content = urdf_content
        self.pub = rospy.Publisher(mark_name, Marker, queue_size=10)
#         rospy.init_node(node_name, anonymous=True)
        rate = rospy.Rate(1) # 10hz
        # Publish the marker
        while self.pub.get_num_connections() < 1:
            print("Please create a subscriber to the marker");
            timer.sleep(1)
        print('publication OK - {}'.format(geometry.name))
    
    def set_marker(self, gframe):
        GeoMarker.ID_COUNT += 1
        self.marker = Marker()
#         self.marker.header.frame_id = self.geometry.link_name # let rviz transform link - buggy
        self.marker.header.frame_id = "/{}".format(gframe.link_name)
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "basic_shapes"
        self.marker.id = GeoMarker.ID_COUNT
        self.marker.type = self.get_type()
        if hasattr(self.geometry, 'uri') and self.geometry.uri:
            self.marker.mesh_resource = self.geometry.uri;
        self.marker.action = Marker.ADD
        self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = self.geometry.get_rviz_scale()
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a  = self.geometry.color
        self.marker.lifetime = rospy.Duration()
        self.__set_position(gframe.Toff)
        self.pub.publish(self.marker);
        
    def __set_position(self, Toff):
        self.marker.pose.orientation.x, self.marker.pose.orientation.y, \
            self.marker.pose.orientation.z, self.marker.pose.orientation.w = \
            Rotation.from_dcm(Toff[:3,:3]).as_quat()
        self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z = \
            Toff[:3,3]
        
    def move_marker(self, gframe):
        self.marker.action = Marker.MODIFY
        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = "/{}".format(gframe.link_name)
        self.__set_position(gframe.Toff)
        self.pub.publish(self.marker);
        
    def get_type(self):
        if self.geometry.gtype == GeoType.BOX:
            return Marker.CUBE
        elif self.geometry.gtype == GeoType.PLANE:
            return Marker.CUBE
        elif self.geometry.gtype == GeoType.LINE:
            return Marker.CYLINDER
        elif self.geometry.gtype == GeoType.SPHERE:
            return Marker.SPHERE
        elif self.geometry.gtype == GeoType.CYLINDER:
            return Marker.CYLINDER
        elif self.geometry.gtype == GeoType.MESH:
            return Marker.MESH_RESOURCE
        
    def delete(self):
        self.marker.action = Marker.DELETE
        self.pub.publish(self.marker);
        
    def __del__(self):
        self.marker.action = Marker.DELETE
        self.pub.publish(self.marker);
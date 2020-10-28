from __future__ import print_function
import time as timer
import numpy as np
import uuid
import rospy
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from .geometry import *

def get_publisher(joint_names, robot_name="", control_freq=100):
    pub = rospy.Publisher(robot_name+'/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(control_freq) # 10hz
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

def show_motion(pose_list, marker_list, pub, joints, joint_names, error_skip=1e-6, period=1e-6):
    pvec_last = np.array(pose_list)+1
    for pvec in pose_list:
        if np.linalg.norm(pvec-pvec_last)<error_skip:
            break
        pvec_last = pvec
        joints.header.seq += 1
        joints.header.stamp = rospy.Time.now()
        joints.position = pvec.tolist() if isinstance(pvec, np.ndarray) else pvec
        pub.publish(joints);
        for marker in marker_list:
            marker.move_marker({joints.name[i]: joints.position[i] for i in range(len(joint_names))})
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
        
def set_markers(geometry_items, joints, joint_names, link_names, urdf_content):
    marker_list = []
    joint_dict = {joints.name[i]: joints.position[i] for i in range(len(joint_names))}
    for link_name in link_names:
        ctems = geometry_items[link_name]
        for ctem in ctems:
            if ctem.display:
                marker_list += [GeoMarker(geometry=ctem,urdf_content=urdf_content)]
                marker_list[-1].set_marker(joint_dict)
    return marker_list

class GeoMarker:
    ID_COUNT = 0
    def __init__(self, geometry, urdf_content, mark_name='visualization_marker'):
        self.geometry = geometry
        self.urdf_content = urdf_content
        self.pub = rospy.Publisher(mark_name, Marker, queue_size=10)
        # Publish the marker
        while self.pub.get_num_connections() < 1:
            print("Please create a subscriber to the marker");
            timer.sleep(1)
#         print('publication OK')

    @classmethod
    def create_marker_template(cls, mtype, scale, color):
        GeoMarker.ID_COUNT += 1
        marker = Marker()
        marker.header.frame_id = "/world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = GeoMarker.ID_COUNT
        marker.type = mtype
        marker.action = Marker.ADD
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a  = color
        marker.lifetime = rospy.Duration()
        return marker
    
    def set_marker(self, joint_dict):
        self.marker = GeoMarker.create_marker_template(self.get_type(), self.geometry.get_dims(), self.geometry.color)
#         self.marker.header.frame_id = self.geometry.link_name # let rviz transform link - buggy
        if hasattr(self.geometry, 'uri'):
            self.marker.mesh_resource = self.geometry.uri;
        if self.geometry.gtype == GEOTYPE.MESH:
            self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = self.geometry.scale
        self.submarkers = []
        self.subTs = []
        if self.geometry.gtype == GEOTYPE.SEGMENT:
            self.submarkers.append(GeoMarker.create_marker_template(Marker.SPHERE, [self.geometry.radius*2]*3, self.geometry.color))
            self.subTs.append(SE3(np.identity(3), [0,0,self.geometry.length/2]))
            self.submarkers.append(GeoMarker.create_marker_template(Marker.SPHERE, [self.geometry.radius*2]*3, self.geometry.color))
            self.subTs.append(SE3(np.identity(3), [0,0,-self.geometry.length/2]))
        self.__set_position(joint_dict)
        self.publish_marker()

#     def __set_position(self, joint_dict): # let rviz transform link - buggy
#         T = self.geometry.get_offset_tf()
#         self.marker.pose.orientation.x, self.marker.pose.orientation.y, \
#             self.marker.pose.orientation.z, self.marker.pose.orientation.w = \
#             Rotation.from_dcm(T[:3,:3]).as_quat()
#         self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z = \
#             T[:3,3]
        
    def __set_position(self, joint_dict):
        T = self.geometry.get_tf(joint_dict)
        self.marker.pose.orientation.x, self.marker.pose.orientation.y, \
            self.marker.pose.orientation.z, self.marker.pose.orientation.w = Rotation.from_dcm(T[:3,:3]).as_quat()
        self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z = T[:3,3]
        for smk, sT in zip(self.submarkers, self.subTs):
            Tsub = np.matmul(T,sT)
            smk.pose.orientation.x, smk.pose.orientation.y, \
                smk.pose.orientation.z, smk.pose.orientation.w = Rotation.from_dcm(Tsub[:3,:3]).as_quat()
            smk.pose.position.x, smk.pose.position.y, smk.pose.position.z = Tsub[:3,3]
        
    def move_marker(self, joint_dict = [0]*6):
        self.marker.action = Marker.MODIFY
        self.marker.header.stamp = rospy.Time.now()
        self.__set_position(joint_dict)
        self.publish_marker()

    def publish_marker(self):
        self.pub.publish(self.marker)
        for smk in self.submarkers:
            self.pub.publish(smk)
        
    def get_type(self):
        if self.geometry.gtype == GEOTYPE.BOX:
            return Marker.CUBE
        elif self.geometry.gtype == GEOTYPE.SEGMENT:
            return Marker.CYLINDER
        elif self.geometry.gtype == GEOTYPE.SPHERE:
            return Marker.SPHERE
        elif self.geometry.gtype == GEOTYPE.MESH:
            return Marker.MESH_RESOURCE
        
    def delete(self):
        self.marker.action = Marker.DELETE
        self.pub.publish(self.marker)
        for smk in self.submarkers:
            smk.action = Marker.DELETE
            self.pub.publish(smk)
        
    def __del__(self):
        self.delete()
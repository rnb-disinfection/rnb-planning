#!/usr/bin/env python3

import struct
import socket
import struct
import threading
import time
# from pynput import keyboard

SEND_UDP_IP = "192.168.0.102" # Robot IP
RECV_UDP_IP = "192.168.0.10"
UDP_PORT_SEND = 50305 # SEND PORT
UDP_PORT_RECV = 50306 # RECV PORT

gnGoalreached = 0
on_motion = 0
gnGoalreached_edge_up = 0

cur_pose_x = 0.0
cur_pose_y = 0.0
cur_pose_heading_q_z = 0.0
cur_pose_heading_q_w = 0.0

#test pose
#target_pose_x_s = 3.85
#target_pose_y_s = -1.49
#target_pose_z_s = 0.803
#target_pose_w_s = 0.5953

target_pose_x_s = 2.245
target_pose_y_s = 0.479
target_pose_z_s = 0.698482554952
target_pose_w_s = 0.715627081955

target_pose_x_1 = 2.75
target_pose_y_1 = 3.64
target_pose_z_1 = 1.0
target_pose_w_1 = 0.0

target_pose_x_2 = 2.5
target_pose_y_2 = 3.64
target_pose_z_2 = 1.0
target_pose_w_2 = 0.0

target_pose_x_3 = 2.5
target_pose_y_3 = 4.06
target_pose_z_3 = 1.0
target_pose_w_3 = 0.0

target_pose_x_4 = 2.48
target_pose_y_4 = 4.51
target_pose_z_4 = 1.0
target_pose_w_4 = 0.0

target_pose_x_5 = 0.55
target_pose_y_5 = 4.42
target_pose_z_5 = 0.0
target_pose_w_5 = 1.55

target_pose_x_6 = 0.65
target_pose_y_6 = 4.42
target_pose_z_6 = 0.0
target_pose_w_6 = 1.0

target_pose_x_7 = 0.6527
target_pose_y_7 = 4.0096
target_pose_z_7 = 0.0
target_pose_w_7 = 1.0

target_pose_x_8 = 0.633
target_pose_y_8 = 3.439
target_pose_z_8 = 0.0
target_pose_w_8 = 1.0

# keyboard input function
def on_press(key):
    if str(key) == "'0'":
        Packet_data=(target_pose_x_s, target_pose_y_s, target_pose_z_s, target_pose_w_s)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'1'":
        Packet_data=(target_pose_x_1, target_pose_y_1, target_pose_z_1, target_pose_w_1)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'2'":
        Packet_data=(target_pose_x_2, target_pose_y_2, target_pose_z_2, target_pose_w_2)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'3'":
        Packet_data=(target_pose_x_3, target_pose_y_3, target_pose_z_3, target_pose_w_3)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'4'":
        Packet_data=(target_pose_x_4, target_pose_y_4, target_pose_z_4, target_pose_w_4)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'5'":
        Packet_data=(target_pose_x_5, target_pose_y_5, target_pose_z_5, target_pose_w_5)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'6'":
        Packet_data=(target_pose_x_6, target_pose_y_6, target_pose_z_6, target_pose_w_6)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'7'":
        Packet_data=(target_pose_x_7, target_pose_y_7, target_pose_z_7, target_pose_w_7)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))
    elif str(key) == "'8'":
        Packet_data=(target_pose_x_8, target_pose_y_8, target_pose_z_8, target_pose_w_8)
        udp_send_data = struct.pack('>ffff',*Packet_data)
        print(Packet_data)
        sock.sendto(udp_send_data,(SEND_UDP_IP, UDP_PORT_SEND))

def udp_server_thread_func(sock):    
    global cur_pose_x, cur_pose_y, cur_pose_heading_q_z, cur_pose_heading_q_w, \
        gnGoalreached, gnGoalreached_edge_up
    print("[MOBILE ROBOT] Start UDP THREAD")
    # sock.settimeout(1)
    try:
        while True:
            try:
                data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
                recive_udp_data = struct.unpack('>ffffi',data)
                # print(addr, recive_udp_data)
                cur_pose_x = recive_udp_data[0]
                cur_pose_y = recive_udp_data[1]
                cur_pose_heading_q_z = recive_udp_data[2]
                cur_pose_heading_q_w = recive_udp_data[3]
                if gnGoalreached != recive_udp_data[4]:
                    print("goal reach: {} -> {} ({})".format(gnGoalreached,
                                                             recive_udp_data[4],
                                                             time.time()))
                    if (not gnGoalreached) and recive_udp_data[4]:
                        print("goal reach signal edge up")
                        gnGoalreached_edge_up = 1
                    gnGoalreached = recive_udp_data[4]
                time.sleep(0.05)
    #             print("send_addr:{}, x:{}, y:{}, quaternion_z:{}, quaternion_w:{}, Goal reached:{}".format(addr, round(cur_pose_x,4), round(cur_pose_y,4), round(cur_pose_heading_q_z,4), round(cur_pose_heading_q_w,4), gnGoalreached))
            except socket.timeout:
                print("[MOBILE ROBOT] socket timeout")
                continue
            except Exception as e:
                print("[MOBILE ROBOT] UDP SERVER LOOP ERROR: {}".format(e))
                continue
    except Exception as e:
        print("[MOBILE ROBOT] UDP SERVER ERROR: {}".format(e))
    finally:
        print("[MOBILE ROBOT] QUIT UDP THREAD")

##
# @brief start udp server thread for updating KIRO mobile robot state
# @remark usage: sock, server_thread = start_mobile_udp_thread(recv_ip=ip_cur)
def start_mobile_udp_thread(recv_ip=RECV_UDP_IP):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((recv_ip, UDP_PORT_RECV))
    print("[MOBILE ROBOT] bind: {}".format((recv_ip, UDP_PORT_RECV)))
    udp_server_thread = threading.Thread(target=udp_server_thread_func,args=(sock,))
    udp_server_thread.daemon = True
    udp_server_thread.start()
    return sock, udp_server_thread

import time

def get_xyzw_cur():
    return cur_pose_x, cur_pose_y, cur_pose_heading_q_z, cur_pose_heading_q_w

def get_reach_state():
    return gnGoalreached

##
# @brief send target pose and wait until motion ends
# @remark usage: cur_xyzw = send_pose_wait(tar_xyzw, send_ip=MOBILE_IP)
# @param tar_xyzw tuple of 4 numbers to represent 2D location (x,y/qz,qw in xyzquat)
# @param tool_angle tool angle
def send_pose_wait(sock, tar_xyzw, tool_angle=0, send_ip=SEND_UDP_IP, recv_delay=2, fix_delay=False):
    global gnGoalreached_edge_up
    assert len(tar_xyzw)==4, "tar_xyzw should be tuple of 4 floats"
    Packet_data=tuple(tar_xyzw)+tuple((tool_angle,))
    udp_send_data = struct.pack('>ffffi',*Packet_data)
    print(Packet_data)
    gnGoalreached_edge_up = 0
    sock.sendto(udp_send_data,(send_ip, UDP_PORT_SEND))
    time.sleep(recv_delay)
    if not fix_delay:
        while not gnGoalreached_edge_up:
            time.sleep(0.5)
    gnGoalreached_edge_up = 0
    return cur_pose_x, cur_pose_y, cur_pose_heading_q_z, cur_pose_heading_q_w

def wait_goal_reached():
    while not gnGoalreached:
        time.sleep(0.5)

    
    
if __name__ == '__main__':
    #udp config & udp thread config
    sock, udp_server_thread = start_mobile_udp_thread()

#     with keyboard.Listener(on_press=on_press) as listener:
#         listener.join()

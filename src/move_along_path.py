#!/usr/bin/env python

import A_star_v2
import numpy as np
import time
import serial
import rospy
from uwb_pos import UWBListener  # Import the UWBListener class
import pure_pursuit_target
import threading
import path_process


# Initialize the UWBListener globally
listener = UWBListener()

# 将速度变量转为串口指令
def encode_vel_to_bytes(vel, type):
    if not (0 <= vel <=4095):
        raise ValueError("Velocity must be in the range 0 to 4095")
    bin_value = format(vel, '012b')
    if type == 'l':
        bin_value='0011' + bin_value
    elif type == 'r':
        bin_value='0100' + bin_value
    int_value = int(bin_value, 2)
    bytes_value = int_value.to_bytes(2, byteorder='big')
    return bytes_value

# 串口通信，告知stm32小车左轮和右轮的速度
def set_vel(vl, vr):
    ser.write(b'\xaa\xaa')  # 起始校验位
    ser.write(encode_vel_to_bytes(vl, 'l'))
    ser.write(encode_vel_to_bytes(vr, 'r'))
    ser.write(b'\x50\x00')  # 命令小车前行
    ser.write(b'\xff\xff')  # 结束校验位
    print(f"Setting velocities: Left = {vl}, Right = {vr}")
    return

# 将原地转向变量转换为串口指令
def encode_angle_to_bytes(angle_abs, type):
    if not (0 <= angle_abs <=255):
        raise ValueError("Angle must be in the range 0 to 255")
    bin_value = format(angle_abs, '08b')
    # 在此处设置了速度模式，例如00010101的后四位0101代表5,表示转向速度设为50
    if type == 'l':
        bin_value='00010101' + bin_value
    elif type == 'r':
        bin_value='00100101' + bin_value
    int_value = int(bin_value, 2)
    bytes_value = int_value.to_bytes(2, byteorder='big')
    return bytes_value

# 原地转向，angle为角度，有正负
def turn(angle):
    angle_abs = int(abs(angle))
    speed_mode = 56
    # 假设速度模式为1

    if angle > 0:  # 左转
        command = encode_angle_to_bytes(angle_abs, 'l')
    elif angle < 0:  # 右转
        command = encode_angle_to_bytes(angle_abs, 'r')
    ser.write(b'\xaa\xaa')  # 起始校验位
    ser.write(command)
    ser.write(b'\xff\xff') # 结束校验位
    print(f"Turning by {angle} degrees")
    return



# 计算向量夹角，单位为度
def angle_between_vectors(vector_a, vector_b):
    dot_product = np.dot(vector_a, vector_b)
    norm_a = np.linalg.norm(vector_a)
    norm_b = np.linalg.norm(vector_b)
    cos_theta = dot_product / (norm_a * norm_b)
    angle_radians = np.arccos(cos_theta)
    angle_degrees = np.degrees(angle_radians)
    cross_product = vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]
    if cross_product < 0:
        angle_degrees = -angle_degrees
    return angle_degrees

# ros Subscriber：使用uwb获得当前坐标，频率为50hz，使用kalman滤波以获得更精确的位置
# 根据基站与地图的相对位置，调整坐标
# uwb返回的坐标值以米为单位，在本程序中使用厘米为单位，因此要乘100，单位：cm
def get_current_pos():
    # print(listener.current_pos)
    pos=np.zeros(2)
    pos[0]=listener.current_pos[0]*100
    pos[1]=listener.current_pos[2]*100
    # print("pos:", pos)
    return pos

# 同样使用uwb获取坐标，利用kalman滤波可以实现获得当前速度，单位：cm/s
def get_curent_vel():
    vel=np.zeros(2)
    vel[0]=listener.current_pos[1]*100
    vel[1]=listener.current_pos[3]*100
    # print("vel:", vel)
    return vel

# 在这里实现pure pursuit纯跟踪算法，返回转向半径r，负的r右转，正的r左转
def pure_pursuit(current_vel, current_pos, target_pos):
    current_vector = np.subtract(target_pos, current_pos)
    alpha = angle_between_vectors(current_vel, current_vector)/180*np.pi
    return np.linalg.norm(current_vector)/(2*np.sin(alpha)), alpha

def pure_pursuit_vel(v_mid, r, l):
    if -l <= r <= l:
        if r >= 0:
            r = l
        else:
            r = -l
    set_vel(int(v_mid*(1-l/(2*r))), int(v_mid*(1+l/(2*r))))



# # 从一个节点走到另一个节点的运动控制，带有偏移矫正，使用pure pursuit纯跟踪算法
# def move_to(p1, p2):
#     target_vector = np.subtract(p2, p1)
#     # sub_goals=generate_sub_goals(p1,p2,1)
#     for sub_goal in sub_goals:
#         while True:
#             current_pos = get_current_pos()
#             current_vector = np.subtract(sub_goal, current_pos)
#             sub_target_vector = np.subtract(sub_goal, p1)
#             distance_to_target = np.linalg.norm(current_vector)
#             r=pure_pursuit(get_curent_vel(), get_current_pos(), sub_goal)
#             print("pos:", current_pos)
#             pure_pursuit_vel(v_mid=50, r=r, l=18)
#             if np.dot(current_vector, sub_target_vector) <= 0:  # Adjust the threshold as needed
#                 break
#             time.sleep(0.1)
        
    # # angle = angle_between_vectors(target_vector, current_vector)
    # set_vel(0, 0)  # Stop the vehicle
    # # if np.dot(np.subtract(p2, current_pos), target_vector) <= 0:  # Adjust the threshold as needed
    # #         break

def path_track(path):
    led = 30
    interval = 2
    path_interpolated = path_process.interpolate_path(path, interval)
    while True:
        current_pos = get_current_pos()
        target_idx = path_process.find_nearest_point_idx(path_interpolated, current_pos)+int(led/interval)
        if target_idx >= len(path_interpolated):
            target_idx = len(path_interpolated)-1
        target = path_interpolated[target_idx]
        current_vector = np.subtract(path[-2], current_pos)
        r, alpha=pure_pursuit(get_curent_vel(), get_current_pos(), target)
        print("pos:", current_pos, "target:", target)
        print("r:",r,"  alpha:",alpha)
        v_mid = 80-(np.abs(alpha)*60)**1.5
        if v_mid>100:
            v_mid=100
        elif v_mid<20:
            v_mid=20
        distance_to_target = np.linalg.norm(current_vector)
        if distance_to_target <= 30:
            v_mid=20
        pure_pursuit_vel(v_mid, r=r, l=18)
        if distance_to_target <= 30 and np.dot(current_vector, np.subtract(path[-2],path[-3])) <= 0:
            break
        time.sleep(0.1)
    set_vel(0,0)
        
    

if __name__ == '__main__':
    rospy.init_node('move_along_path', anonymous=True)
    
    listener_thread = threading.Thread(target=listener.start_listener)
    listener_thread.start()
    
    ser = serial.Serial('/dev/ttyAMA2', 115200)  # 打开串口设备
    if not ser.isOpen:
        ser.open()
    print('serial opened')

    # 建立地图，找到path
    points = [(180, 60), (340, 80), (35, 90),
              (35, 225), (180, 225), (210, 235), (340, 230),
              (30, 385), (220, 375), (340, 375), (420, 375)]
    obstacles = [((180, 50), (310, 200)),
                 ((60, 110), (170, 200)),
                 ((370, 50), (450, 200)),
                 ((60, 260), (190, 350)),
                 ((230, 260), (310, 350)),
                 ((370, 260), (450, 350))]
    start = (135, 25)
    goal = (210, 360)
    points.append(start)
    points.append(goal)
    graph = A_star_v2.create_graph(points, obstacles)
    # A_star_v2.visualize_graph(graph, obstacles)
    path = A_star_v2.a_star(graph, start, goal)
    # path = [(220, 235), (220, 372)]
    print("Path:", path)

    # # 从path中的起点开始，走过每一个坐标点，最终到达终点
    # # 假设一开始小车朝向y轴放置
    # for i in range(0, len(path) - 1):
    #     if i == 0:
    #         v0 = (0, 1)
    #     else:
    #         v0 = np.subtract(path[i], path[i - 1])
    #     v1 = np.subtract(path[i+1], path[i])
    #     angle = angle_between_vectors(v0, v1)

    #     # 原地转向
    #     turn(angle)

    #     time.sleep(3)
    #     # 走到下一个节点
    #     move_to(path[i], path[i+1])
    #     time.sleep(1)
    time.sleep(1)
    set_vel(20,20)
    time.sleep(2)
    path = pure_pursuit_target.append_point(path)
    path_track(path)
    path = [(200, 375), (350,375), (350,25), (135,25)]
    print(path)
    path = pure_pursuit_target.append_point(path)
    path_track(path)
    
    set_vel(0, 0)
    time.sleep(0.1)
    
    print('process finished')
    set_vel(1,1)
    set_vel(0,0)
    for i in range(100):
        set_vel(0,0)
        time.sleep(0.01)
    time.sleep(0.01)
    ser.close()

    

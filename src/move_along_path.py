#!/usr/bin/env python

import A_star_v2
import numpy as np
import time
import serial
import rospy
from uwb_pos import UWBListener  # Import the UWBListener class
import threading


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
    print(listener.current_pos)
    pos=np.zeros(2)
    pos[0]=listener.current_pos[0]*100
    pos[1]=listener.current_pos[2]*100
    print(pos)
    return pos

# 同样使用uwb获取坐标，利用kalman滤波可以实现获得当前速度，单位：cm/s
def get_curent_vel():
    vel=np.zeros(2)
    vel[0]=listener.current_pos[1]*100
    vel[1]=listener.current_pos[3]*100
    print(vel)
    return vel

# 从一个节点走到另一个节点的运动控制，带有偏移矫正
def move_to(p1, p2):
    target_vector = np.subtract(p2, p1)
    while True:
        current_pos = get_current_pos()
        current_vector = np.subtract(p2, current_pos)
        distance_to_target = np.linalg.norm(current_vector)
        if distance_to_target <1:  # Adjust the threshold as needed
            break
        angle = angle_between_vectors(target_vector, current_vector)
        if abs(angle) > 5:  # Adjust the threshold as needed
            if angle > 0:
                set_vel(50, 60)  # Turn left
            else:
                set_vel(60, 50)  # Turn right
        else:
            set_vel(50, 50)  # Move forward
        time.sleep(0.1)  # Adjust the delay as needed
    set_vel(0, 0)  # Stop the vehicle

if __name__ == '__main__':
    rospy.init_node('move_along_path', anonymous=True)
    
    listener_thread = threading.Thread(target=listener.start_listener)
    listener_thread.start()
    
    ser = serial.Serial('/dev/ttyAMA2', 115200)  # 打开串口设备
    if not ser.isOpen:
        ser.open()
    print('serial opened')

    # 建立地图，找到path
    points = [(170, 80), (340, 80), (30, 90),
              (30, 230), (170, 225), (200, 235), (340, 230),
              (30, 375), (200, 375), (340, 375), (420, 375)]
    obstacles = [((180, 50), (310, 200)),
                 ((60, 110), (150, 200)),
                 ((370, 50), (450, 200)),
                 ((60, 260), (190, 350)),
                 ((230, 260), (310, 350)),
                 ((370, 260), (450, 350))]
    start = (135, 25)
    goal = (100, 352)
    points.append(start)
    points.append(goal)
    graph = A_star_v2.create_graph(points, obstacles)
    # A_star_v2.visualize_graph(graph, obstacles)
    path = A_star_v2.a_star(graph, start, goal)
    print("Path:", path)

    # 从path中的起点开始，走过每一个坐标点，最终到达终点
    # 假设一开始小车朝向y轴放置
    for i in range(0, len(path) - 1):
        if i == 0:
            v0 = (0, 1)
        else:
            v0 = np.subtract(path[i], path[i - 1])
        v1 = np.subtract(path[i+1], path[i])
        angle = angle_between_vectors(v0, v1)

        # 原地转向
        turn(angle)

        time.sleep(3)
        # 走到下一个节点
        move_to(path[i], path[i+1])
        time.sleep(1)

    print('process finished')

    ser.close()

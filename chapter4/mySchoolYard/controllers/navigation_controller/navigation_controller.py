"""navigation_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
 
from controller import GPS
from math import atan2
import cv2
import numpy as np
import argparse
import os
import sys
import datetime
from paho.mqtt import client as mqtt_client
import random
import time
import threading
import json
import math
import pandas as pd
import struct


#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
camera_topic = "/camera/collect" #保留
roads_topic = "/roads/collect" #保留
xy_topic = "/map/xy" #发布建图后的定位
client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#控制指令topic 
cmd_vel_topic = "/cmd/vel"
#接收A*规划航点
pathPlanning_topic = "/path/planning"
navigation_topic  ="/navigation/value"
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
front_left_wheel = robot.getDevice('front left wheel')
front_right_wheel = robot.getDevice('front right wheel')
back_left_wheel = robot.getDevice('back left wheel')
back_right_wheel = robot.getDevice('back right wheel')

gps =  robot.getDevice("gps")
if gps.getCoordinateSystem() == GPS.WGS84:
    print("GPS is using 'WGS84' coordinates system")
gps.enable(timestep)
compass =  robot.getDevice('compass')
compass.enable(timestep)
#设置初始位置
front_left_wheel.setPosition(float('inf'))
front_right_wheel.setPosition(float('inf'))
back_left_wheel.setPosition(float('inf'))
back_right_wheel.setPosition(float('inf'))
front_left_wheel.setVelocity(0.0)
front_right_wheel.setVelocity(0.0)
back_left_wheel.setVelocity(0.0)
back_right_wheel.setVelocity(0.0)

w = 0
v = 2
start_x =0
start_y =0
start_navigation_flg =0
def getxy(x,y):
    data = {"valid":"1","x":"%s"%(x),"y":"%s" %(y)
           }
    return  data
def loop():
    global vel,ang,start_flg,start_navigation_flg
    global start_x 
    global start_y 
    last_time =0
    last_x =0
    last_y =0
    init_x =800/2
    init_y =800/2
    pix_di =0
    global v,w
    first_flg =0
    '''当设置 header=None 时，则认为csv文件没有列索引，为其添加相应范围的索引，
    range(1,1200)指建立索引号从1开始最大到1199的列索引，
    当数据长度超过范围时，索引沿列数据的右侧对齐。'''
    obj=pd.read_csv('index1.csv',header=None,nrows=1)
    data =obj.values[0:3]
    index =data[0]
    last_x =index[0]
    last_y =index[1]
    init_x =index[2]
    init_y = index[3]
    print(index[2] )
 
    cnt = 0
    while robot.step(timestep) != -1:
        cnt =cnt + 1
        x=gps.getValues()[0]
        y=gps.getValues()[1]

        # 获取速度，单位m/s
        # 以下调试用
        #gps.getSpeed()
        #north = compass.getValues()
        #angle = atan2(north[1], north[0])
        
        #判断是否行驶超过3m
        dis = math.pow(x-last_x,2) + math.pow(y-last_y,2)
        #1m上报一次 
        if(dis >= (1*1) or cnt>=10):
            cnt = 0
            #print(str(x) + ' '+ str(y))
            rad = atan2(y-last_y, x-last_x)
            
            pix_di = math.sqrt(dis)/0.5;
            start_x = (init_x +  (pix_di*math.cos(rad)/1.4142135))*100/100
            start_y = (init_y +  (pix_di*math.sin(rad)/1.4142135))*100/100
            #print("compass:"+str(angle*180/3.14))
            #print(str(init_x)+" "+str(init_y)+" "+str(start_x)+" "+str(start_y))
            xy =getxy(start_x,start_y)
            json_data = json.dumps(xy)
            result = client.publish(xy_topic,json_data,0)
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send  to topic `{xy_topic}`")
            else:
                print(f"Failed to send message to topic {xy_topic}")
            
            last_x = x
            last_y = y
            init_x = start_x
            init_y = start_y
        if(int(start_navigation_flg) != 1):
            front_left_wheel.setVelocity(0)
            front_right_wheel.setVelocity(0)
            back_left_wheel.setVelocity(0)
            back_right_wheel.setVelocity(0)
            continue
        
        front_left_wheel.setVelocity(-w+v)
        front_right_wheel.setVelocity(w+v)
        back_left_wheel.setVelocity(-w+v)
        back_right_wheel.setVelocity(w+v)
        #time.sleep(0.01)
        pass

#作为子线程开启
th = threading.Thread(target=loop)
th.setDaemon(True)
th.start()

#时间周期
dt = 0.3
#最小接近距离
limit_dis =0.3
limit_ang =10
#来源于《轮式自主移动机器人编程实战》图书
#借助参考 本函数未被使用
def pp_control(x_start, y_start, theta_start, x_goal, y_goal):
    """
    d 为机器人和目标之间的距离
    alpha  为目标相对于本身的方向方位角
    beta 为最终角度和方位角的差
    """
    x = x_start
    y = y_start
    theta = theta_start

    x_diff = x_goal - x
    y_diff = y_goal - y

    x_traj, y_traj = [], []

    d = np.hypot(x_diff, y_diff)
    dis = d
    
    #满足到达目标的最小距离和角度最小值
    while dis > limit_dis:
        x_traj.append(x)
        y_traj.append(y)

        x_diff = x_goal - x
        y_diff = y_goal - y
        # 角度差
        # 范围为[-pi, pi] 
        # 从 0 到 2*pi 
        #求两点距离
        dis = np.hypot(x_diff, y_diff)
        #求两点角度
        gama =np.arctan2(y_diff, x_diff)
        alpha = gama - theta   
        #角度归一化
        if  (alpha < -np.pi):                    
            alpha = alpha + 2 * np.pi 
        elif(alpha > np.pi):
            alpha = alpha - 2 * np.pi 
        
        #使用匀速行驶
        v = 0.50
        if(dis < limit_dis):
            v = 0
        #根据匀速计算角速度w
        w = 2*v*math.sin(alpha)/dis

        print("dis,v,w,theta,oritation,goal")
        print(dis,v,w,theta*180/3.14,gama*180/3.14)
        theta = theta + w * dt
        if  (theta < -np.pi):                     
            theta = theta + 2 * np.pi 
        elif(theta > np.pi):
            theta = theta - 2 * np.pi 
        #x = x + v * np.cos(theta) * dt
        #y = y + v * np.sin(theta) * dt
 
#在黑白地图上取两个点来获取机器人的初始方向设置，
#经标记在黑白地图上向右行驶时为+90度 顺时针增加
#所以机器人可以i设置为初始角度为+90度 顺时针增加
def navigation():
    global vel,ang,start_flg
    last_time =0
    waypoint_x =0
    waypoint_y =0
    init_bias =0
    fact_heading =90
    pix_di =0
    global start_x 
    global start_y  
    global start_navigation_flg
    global v,w
    first_flg =0

    cnt = 0
    while True:
        time.sleep(0.05)
        #是否启动导航
        if(int(start_navigation_flg) != 1):
            continue
        print("start navigation \n")
        print(len(path_points))
        i = 0
        while i < len(path_points):
            time.sleep(0.005)
            #print(path_points[len(path_points)-1-i])
            xy = path_points[len(path_points)-1-i]
            print("number wpt:"+str(i))
            #获取导航函数 注意a*算法规划路径后的反相
            waypoint_y = xy[0]
            waypoint_x = xy[1]
            cnt =cnt + 1
            
            # 获取速度，单位m/s
            gps_speed = gps.getSpeed()
            
            north = compass.getValues()
            angle = atan2(north[1], north[0])
            if(first_flg ==0):
                first_flg =1
                init_bias = angle
            #和图像直角坐标系保持一致
            fact_heading = 0 - (angle - init_bias)
            #0 - (angle - init_bias)
            if  (fact_heading < -np.pi):                     
                fact_heading = fact_heading + 2 * np.pi 
            elif(fact_heading > np.pi):
                fact_heading = fact_heading - 2 * np.pi
            #计算距离  和航点之间的角度
            dis = math.pow(int(start_y)-waypoint_x,2) + math.pow(int(start_x)-waypoint_y,2)
            rad = atan2( waypoint_x-start_y,waypoint_y-start_x)#atan2(y,x) =y/x
            print("cpt:"+str(int(start_x))+","+str(int(start_y))+"->wpt:"+str(waypoint_y)+","+str(waypoint_x)+
            "[deg:"+str(int(rad*180/3.14))+",dis:"+str(int(math.sqrt(dis)))+']')
            
            print("fact_heading:"+str(int(fact_heading*180/3.14)))
            #print("wpt rad:"+str(rad*180/3.14))
            alpha = rad - fact_heading
            if  (alpha < -np.pi):                     
                alpha = alpha + 2 * np.pi 
            elif(alpha > np.pi):
                alpha = alpha - 2 * np.pi
            print("alpha:"+str(alpha*180/3.14))
            #判断距离航点是否超过4m
            if(math.sqrt(dis) <= (4)):
                cnt = 0
                #读取下一个航点
                i += 1 
                continue 
            w = 2*v*math.sin(alpha)/math.sqrt(dis)
            print("turn w:"+str(w))
            #结束导航
            if(i == (len(path_points)-1)):
                w=0
                v=0
                start_navigation_flg =0
                break

#作为子线程开启
th2 = threading.Thread(target=navigation)
th2.setDaemon(True)
th2.start()

#定义mqtt的client
client = mqtt_client.Client(client_id)
last_time =0
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    #client.subscribe("test")
path_points = []
def on_message(client, userdata, message):
    global start_navigation_flg
    #print(message.topic +":"+message.payload.decode("utf-8"))
    if (message.topic == pathPlanning_topic):
        #bydat = message.payload.decode("utf-8")
        A =struct.unpack(str(len(message.payload))+'B', message.payload)
        n =len(A)/8
        print(n)
        x_low =0
        x_high=0
        y_high=0
        y_low=0
        #接收来自网页规划后A*计算出的航点  
        #A*算法在 easymqOS_aStar_planning 中
        path_points.clear()
        for index, value in enumerate(A):
            if(index%8 ==0):
                y_low = value
            elif(index%8 ==1):
                y_high = value*256
            elif(index%8 ==4):
                x_low = value 
            elif(index%8 ==5):
                x_high = value*256
            elif(index%8 ==7):
                y =y_low+y_high
                x =x_low+x_high
                path_points.append([x,y])
        print(len(path_points))
    #接收导航指令
    if (message.topic == navigation_topic):
        country_dict = json.loads(message.payload)
        print(country_dict["value"])
        print(message.payload)
        start_navigation_flg = country_dict["value"]
#暂时保留
def publish(client):
    global last_time
    global detect_ok

    global resultImg
    msg_count = 0


if __name__ == '__main__':
    print("connected!")        
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    #publish(client)
    # 订阅主题
    client.subscribe(pathPlanning_topic)
    client.subscribe(navigation_topic)
    client.loop_forever()

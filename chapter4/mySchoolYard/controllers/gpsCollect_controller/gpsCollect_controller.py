"""gpsCollect_controller controller."""

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
from pynput import keyboard

#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
camera_topic = "/camera/collect" #保留
roads_topic = "/roads/collect" #保留

client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#控制指令topic 
cmd_vel_topic = "/cmd/vel"

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
#gps.disable()
#设置初始位置
front_left_wheel.setPosition(float('inf'))
front_right_wheel.setPosition(float('inf'))
back_left_wheel.setPosition(float('inf'))
back_right_wheel.setPosition(float('inf'))
front_left_wheel.setVelocity(0.0)
front_right_wheel.setVelocity(0.0)
back_left_wheel.setVelocity(0.0)
back_right_wheel.setVelocity(0.0)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

vr =0 
vl =0
vel =0.8
ang =0.1
start_flg =0
'''
    ↑
←   ↓   →
K—停止
↑←↓→—前、左、后、右
q/z : 最大速度增加/减少10%
w/x : 仅线性速度增加/减少10%
e/c : 仅角速度增加/减少10%
'''
def conver2wheel(v,w):
    global vr,vl
    vr = (2*v + w*10)/2
    vl = (2*v - w*10)/2
     
    
def on_press(key):
    print(str(key))
    global vel,ang,start_flg
    if(str(key) =="Key.up"):
        #前进
        start_flg =1
        ang = 0.0
        conver2wheel(vel,0)
    elif (str(key) =="Key.right"):
        ang = -0.1 
        conver2wheel(vel/2,ang)
    elif (str(key) =="Key.left"):
        ang = 0.1
        conver2wheel(vel/2,ang)
    elif (str(key) =="Key.down"):
        conver2wheel(0,0)
        start_flg =0
    elif (str(key) =="'w'"):
        if vel <10.0:
            vel = vel +0.1 
        conver2wheel(vel,0)
    elif (str(key) =="'x'"):
        if(vel>0.1):
            vel = vel -0.1
        conver2wheel(vel,0)
    elif (str(key) =="'e'"):
        if ang <1.0:
            ang = ang+0.1 
        conver2wheel(vel,ang)
    elif (str(key) =="'c'"):
        if ang < 1.0:
            ang = ang + 0.1
        conver2wheel(vel,ang)
    print("vel:"+str(vel))
    front_left_wheel.setVelocity(vl)
    front_right_wheel.setVelocity(vr)
    back_left_wheel.setVelocity(vl)
    back_right_wheel.setVelocity(vr)
    
def loop():
    global vel,ang,start_flg
    last_time =0
    last_x =0
    last_y =0
    init_x =800/2
    init_y =800/2
    pix_di =0
    start_x =0
    start_y =0
    first_flg =0
    g_img =None
    g_img = np.ones((800,800), np.uint8)
    g_img.fill(0)
    while robot.step(timestep) != -1:
        
        x=gps.getValues()[0]
        y=gps.getValues()[1]
        if(first_flg ==0):
            first_flg =1
            last_x = x
            last_y = y
            continue
        # 获取速度，单位m/s
        gps.getSpeed()
        
        north = compass.getValues()
        angle = atan2(north[1], north[0])
        #print(str(angle) + ' ')
        #判断是否行驶超过3m
        dis = math.pow(x-last_x,2) + math.pow(y-last_y,2)
        if(dis >= (3*3)):
            #print(str(x) + ' '+ str(y))
            rad = atan2(y-last_y, x-last_x)
            img =None
            img = np.ones((800,800), np.uint8)
            img.fill(0)
            pix_di = math.sqrt(dis)/0.5;
            start_x = (init_x +  (pix_di*math.cos(rad)/1.4142135))*100/100
            start_y = (init_y +  (pix_di*math.sin(rad)/1.4142135))*100/100
            print(str(init_x)+" "+str(init_y)+" "+str(start_x)+" "+str(start_y))
            cv2.line(img,(int(init_x),int(init_y)), (int(start_x),int(start_y)),(255),15)
            cv2.line(g_img,(int(init_x),int(init_y)), (int(start_x),int(start_y)),(255),15)
            #找到临时的道路为白色点位 
            idx = cv2.findNonZero( img )
            print(idx.shape)
            print(type(idx))
            print(idx[0])
            #cv2.imshow('img', img)
            #cv2.waitKey(500)
            byteArr = bytearray(idx)#1xN维的数据
            print(len(byteArr))
            print(type(byteArr))
            print(byteArr[0])
            result = client.publish(roads_topic,byteArr,0)
            
            # result: [0, 1]
            status = result[0]
            if status == 0:
                print(f"Send  to topic `{roads_topic}`")
            else:
                print(f"Failed to send message to topic {roads_topic}")
            cv2.imwrite('img.png', g_img)
            last_x = x
            last_y = y
            init_x = start_x
            init_y = start_y
        pass
#作为子线程开启
th = threading.Thread(target=loop)
th.setDaemon(True)
th.start()




#定义mqtt的client
client = mqtt_client.Client(client_id)
last_time =0
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    #client.subscribe("test")

def on_message(client, userdata, msg):
    
    print(msg.topic+" "+msg.payload.decode("utf-8"))
     
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
    client.subscribe(cmd_vel_topic)
    # Enter here exit cleanup code.
    with keyboard.Listener(on_press=on_press) as lsn:
        lsn.join()
    client.loop_forever()
    
 
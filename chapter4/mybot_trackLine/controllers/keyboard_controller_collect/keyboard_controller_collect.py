"""test_controller controller."""

# 导入需要的类和模块. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from paho.mqtt import client as mqtt_client
import time
import datetime
import random
import sys
import json
import threading
import cv2
import numpy as np
import math
from pynput import keyboard
#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
distance_topic = "/sensors/vl53"
odom_topic = "/sensors/odom"
cmd_vel_topic = "/cmd/vel"

client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#定义mqtt的client
client = mqtt_client.Client(client_id)


Range = 1000
# 创建机器人实例.
robot = Robot()

# 获得仿真环境的时间步.
timestep = int(robot.getBasicTimeStep())
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

camera = robot.getDevice('camera')
camera.enable(timestep)

print(camera.getWidth()) #获取图像宽度
print(camera.getHeight()) #获取图像高度
#设置初始位置
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


vr =0 
vl =0
vel =0.1
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
        if vel <1.0:
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
    
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)
def loop():
    global vel,ang,start_flg
    last_time =0
    while robot.step(timestep) != -1:
        img = camera.getImageArray()
        img = np.asarray(img, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        
        curr_time = datetime.datetime.now()
        t=time.time()
        time_str = datetime.datetime.strftime(curr_time,'%Y%m%d%H%M%S%f')
        if (t*1000 - last_time*1000 >= 500):
            last_time = t
            if (start_flg==1):
                with open("dataset/driving_log.csv","a") as file:
                    file.write(str(vel)+","+ str(ang) +","+time_str+".jpg"+"\n")        
                cv2.imwrite("dataset/%s.jpg"%time_str, img) 
        pass
#作为子线程开启
th = threading.Thread(target=loop)
th.setDaemon(True)
th.start()
with keyboard.Listener(on_press=on_press) as lsn:
            lsn.join()
'''       
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    #client.subscribe("test")
#MQTT接收信息的回调函数
def on_message(client, userdata, message):
   
    global left_motor,right_motor
    print(message.topic+":"+message.payload.decode("utf-8"))
   
    if (message.topic == cmd_vel_topic):
       
        #解析指令 {"control": "1", "vel": "0.1", "ang": "0.0"}
        country_dict = json.loads(message.payload)
        v = country_dict["vel"]
        w = country_dict["ang"]
        

        #差速轮换算成每个的速度
        vr = (2*float(v) + float(w)*15)/2
        vl = (2*float(v) - float(w)*15)/2
        
        print(vr)
        print(vl)
        #测试向前运动
        left_motor.setVelocity(vl)
        right_motor.setVelocity(vr)
        
        
    if (message.topic == odom_topic):
        #save_payload(message.payload, vid_filename)
        print(message.payload)
         
if __name__ == "__main__":
   
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    
    # 订阅主题
    client.subscribe(cmd_vel_topic)
    #publish(client)
    client.loop_forever()

'''
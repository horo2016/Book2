"""test_controller controller."""

# 导入需要的类和模块. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from paho.mqtt import client as mqtt_client
import time
import random
import sys
import json
import threading
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
ds0 = robot.getDevice('ds0')
ds1 = robot.getDevice('ds1')
ds2 = robot.getDevice('ds2')
ds3 = robot.getDevice('ds3')
ds4 = robot.getDevice('ds4')
#使能时间脚步
ds0.enable(timestep)
ds1.enable(timestep)
ds2.enable(timestep)
ds3.enable(timestep)
ds4.enable(timestep)

#设置初始位置
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0 )
right_motor.setVelocity(0)

dis =None
def get_distance(ds0,ds1,ds2,ds3,ds4):
    data = {"left_far":"%s"%(ds0),"left_near":"%s"%(ds1),"center":"%s" %(ds2),"right_near":"%s"%(ds3),"right_far":"%s"%(ds4)
           }
    return  data
    
    

def run_get_distance():
    global client
    while robot.step(timestep) != -1:
        # 读取5路传感器，并打印出来    

        ds0_m = (1000-ds0.getValue())/10
        ds1_m = (1000-ds1.getValue())/10
        ds2_m = (1000-ds2.getValue())/10
        ds3_m = (1000-ds3.getValue())/10
        ds4_m = (1000-ds4.getValue())/10

        #获取所有的距离和json   round 保留小数点1位
        dis = get_distance(round(ds0_m,1),round(ds1_m,1),round(ds2_m,1),round(ds3_m,1),round(ds4_m,1))
        json_data = json.dumps(dis)
        result = client.publish(distance_topic,json_data,0)
        status = result[0]
        if status == 0:
            print(f"Send  to topic `{distance_topic}`")
        else:
            print(f"Failed to send message to topic {distance_topic}")
        #time.sleep(0.05)
        
         
        pass

# Enter here exit cleanup code.
     
#作为子线程开启
th = threading.Thread(target=run_get_distance)
th.setDaemon(True)
th.start()

        
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("test")
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


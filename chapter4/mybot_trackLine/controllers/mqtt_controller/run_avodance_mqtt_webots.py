import pdb
import random
import math
import numpy as np
from makeLearn_aarch import *
import sys
import json
from paho.mqtt import client as mqtt_client
import time
import struct 
import threading

torch.set_num_threads(2)

SummarySensorData = []
StepSizeValue = 1/10 # Step Size For Simulation
ClockTickValue = 25  # Clock Tick
BotSpeed = 20 # Speed Of The Bot
model = Net(InputSize, NumClasses)
model.load_state_dict(torch.load('./NNBot.pth'))# my ANN


#MQTT相关
broker = '192.168.3.2'
#broker = 'www.woyilian.com'
port = 1883
lidar_topic = "/sensors/vl53"
odom_topic = "/sensors/odom"
cmd_vel_topic = "/cmd/vel"

client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#定义mqtt的client
client = mqtt_client.Client(client_id)

SensorsData = []
NumberOfSensors = 5
inference_flg =0

BotStartLocation =  2
DetectCrashResult =0


def getcmd(v,a):
    data = {"control":"1","vel":"%s"%(v),"ang":"%s" %(a)
           }
    return  data
    

def Inference():
    global  inference_flg
    global last_time
    global DetectCrashResult
    while True:
        if(inference_flg ==1):
            print("***********22222222222*******8")
            _SensorsDatas = np.append(SensorsData, 30)#小车角度
            print(_SensorsDatas)
            _SensorsDatas = np.append(_SensorsDatas, [0])
            cnt =0
            cnt100=0
            print(_SensorsDatas[:-2])  ## Print The Sensor Data有时候会是空
            for c in _SensorsDatas[:-2]:
                if(c<50):
                    cnt =cnt+1
                if(c<100):
                    cnt100 =cnt100+1
            if(cnt == 5):
                DetectCrashResult=-1
                inference_flg =2
                time.sleep(0.1)
                #SensorsData.clear()
                continue
            print("*************cnt100****"+str(cnt100))
            DataTensor = torch.Tensor(_SensorsDatas[:-1]).view(1,-1)
            if (model != None):
                ## 从神经网络中获得决策 
                DetectCrash = model(Variable(DataTensor))
                print("inference:")
                #print(DetectCrash.data)
                DetectCrashResult = abs(np.round(DetectCrash.data[0][0]))
                print(DetectCrashResult)
                if(DetectCrashResult > 0 or cnt100==5):#为1时
                    SignalData = _SensorsDatas[:-2]
                    if(sum(SignalData[:2]) > sum(SignalData[-2:])):#左边的空间比右边的空间大
                        DetectCrashResult = 3
                    else:#右转
                        DetectCrashResult = 4
                
                print(DetectCrashResult)
                SignalData = _SensorsDatas[:-2]
                time.sleep(0.01)
            #SensorsData.clear()
            inference_flg = 2
        else:
            time.sleep(0.1)#100ms
        #t=time.time()
        #print (int(round(t * 1000)))
     
#作为子线程开启
th = threading.Thread(target=Inference)
th.setDaemon(True)
th.start()

def Move():
    global  inference_flg
    cmd =getcmd("0.0","0.0") 
    while True:
        print("move pthread:"+ str(DetectCrashResult)+"+"+str(inference_flg))
        if inference_flg == 2:
            if (DetectCrashResult > 0):
                
                for i in range(5):
                    if (DetectCrashResult==3):
                        cmd =getcmd("0.0","-0.2")
                    else:
                        cmd =getcmd("0.0","0.2")
                    json_data = json.dumps(cmd)
                    result = client.publish(cmd_vel_topic,json_data,0)
                    # result: [0, 1]
                    status = result[0]
                    if status == 0:
                        print(f"Send `{cmd}` to topic `{cmd_vel_topic}`")
                    else:
                        print(f"Failed to send message to topic {cmd_vel_topic}")
                    time.sleep(0.1)
            elif(DetectCrashResult == -1):#前进
                cmd =getcmd("-0.1","0")
                json_data = json.dumps(cmd)
                result = client.publish(cmd_vel_topic,json_data,0)
                # result: [0, 1]
                status = result[0]
                if status == 0:
                    print(f"Send `{cmd}` to topic `{cmd_vel_topic}`")
                else:
                    print(f"Failed to send message to topic {cmd_vel_topic}")
                    time.sleep(0.1)
            else:
               #前进
                cmd =getcmd("0.2","0")
                json_data = json.dumps(cmd)
                result = client.publish(cmd_vel_topic,json_data,0)
                # result: [0, 1]
                status = result[0]
                if status == 0:
                    print(f"Send `{cmd}` to topic `{cmd_vel_topic}`")
                else:
                    print(f"Failed to send message to topic {cmd_vel_topic}")
            inference_flg =0
        time.sleep(0.2)
        
#作为子线程开启
th1 = threading.Thread(target=Move)
th1.setDaemon(True)
th1.start()

        
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("test")

def on_message(client, userdata, message):
    global _switch
    global  inference_flg
    print(message.topic+":"+message.payload.decode("utf-8"))
    #_switch = ord(message.payload.decode("utf-8"))-48
    #print("%d"%_switch)
    #print("Receiving message")
    #print(message.topic)
    if (message.topic == lidar_topic):
        #bydat = message.payload.decode("utf-8")
        #A =struct.unpack(str(len(message.payload))+'B', message.payload)
        #save_payload(message.payload, pic_filename)
        '''
        {"left_far": "-2.4", "left_near": "44.16", "center": "75.2", "right_near": "100.0", "right_far": "100.0"}
        '''
        country_dict = json.loads(message.payload)
        left_far = country_dict["left_far"]
        left_near = country_dict["left_near"]
        center = country_dict["center"]
        right_near = country_dict["right_near"]
        right_far = country_dict["right_far"]
        
        
        if(inference_flg == 0):
            SensorsData.clear()
            
            SensorsData.append(round(float(left_far),1))
            SensorsData.append(round(float(left_near),1))
            SensorsData.append(round(float(center),1))
            SensorsData.append(round(float(right_near),1))
            SensorsData.append(round(float(right_far),1))
            inference_flg =1
        
    if (message.topic == odom_topic):
        #save_payload(message.payload, vid_filename)
        print(message.payload)
         
if __name__ == "__main__":

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    
    # 订阅主题
    client.subscribe(lidar_topic)
    #publish(client)
    client.loop_forever()


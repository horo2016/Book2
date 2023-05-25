import pdb
import random
import math
import numpy as np

import sys
import json
from paho.mqtt import client as mqtt_client
import time
import threading
from pynput import keyboard

#MQTT相关
broker = '127.0.0.1'
#broker = 'www.woyilian.com'
port = 1883
lidar_topic = "/sensors/vl53"
odom_topic = "/sensors/odom"
cmd_vel_topic = "/cmd/vel"

client_id = 'python-mqtt-{}'.format(random.randint(0, 1000))
#定义mqtt的client
client = mqtt_client.Client(client_id)



def getcmd(v,a):
    data = {"control":"1","vel":"%s"%(v),"ang":"%s" %(a)
           }
    return  data
    

def on_press(key):
    cmd =getcmd("0.0","0.0")
    if(str(key) =="Key.up"):
        #前进
        cmd =getcmd("0.2","0")
        
    elif (str(key) =="Key.right"):
        cmd =getcmd("0.0","-0.2")
    elif (str(key) =="Key.left"):
        cmd =getcmd("0.0","0.2")
    
    json_data = json.dumps(cmd)
    result = client.publish(cmd_vel_topic,json_data,0)
    # result: [0, 1]
    status = result[0]
    if status == 0:
        print(f"Send `{cmd}` to topic `{cmd_vel_topic}`")
    else:
        print(f"Failed to send message to topic {cmd_vel_topic}")
def Move():
    global  inference_flg
     
    while True:
        with keyboard.Listener(on_press=on_press) as lsn:
            lsn.join()

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

         
if __name__ == "__main__":

    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port, 60)
    
    # 订阅主题
    client.subscribe(lidar_topic)
    #publish(client)
    client.loop_forever()


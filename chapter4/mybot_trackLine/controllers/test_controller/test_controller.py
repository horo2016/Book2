"""test_controller controller."""

# 导入需要的类和模块. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from paho.mqtt import client as mqtt_client
import time
import cv2
import numpy as np
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
camera = robot.getDevice('camera')
#使能时间脚步
ds0.enable(timestep)
ds1.enable(timestep)
ds2.enable(timestep)
ds3.enable(timestep)
ds4.enable(timestep)
camera.enable(timestep)
#camera.setFocalDistance(0.01) #设置焦距
print(camera.getWidth()) #获取图像宽度
print(camera.getHeight()) #获取图像高度
#设置初始位置
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)
# Main loop:
# - 演示仿真步行
while robot.step(timestep) != -1:
    # 读取5路传感器，并打印出来    
    '''
    print("ds0")
    print(1000-ds0.getValue())
    print("ds1")
    print(1000-ds1.getValue())
    print("ds2")
    print(1000-ds2.getValue())
    print("ds3")
    print(1000-ds3.getValue())
    print("ds4")
    print(1000-ds4.getValue())
    time.sleep(0.1)'''
    #测试向前运动
    #left_motor.setVelocity(0.5 )
    #right_motor.setVelocity(0.5)
    img = camera.getImageArray()
    
    img = np.asarray(img, dtype=np.uint8)
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
    
    #cv2.imshow("lena", img)
    #cv2.waitKey(100)

    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

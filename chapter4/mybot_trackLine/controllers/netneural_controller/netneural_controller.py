"""netneural_controller controller."""
from controller import Robot
import cv2
import numpy as np
import argparse
import os
import math
path = "./test"
y_bias = 190
gray_min = np.array([0, 0, 46])
gray_max = np.array([180, 43, 255])

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

def goStraight():
    left_motor.setVelocity(0.4)
    right_motor.setVelocity(0.4)
def turnRight():
    left_motor.setVelocity(0.1)
    right_motor.setVelocity(-0.1)
def turnLeft():
    left_motor.setVelocity(-0.1)
    right_motor.setVelocity(0.1)
if __name__ == '__main__':
    
    net = cv2.dnn.readNetFromONNX('nvidia.onnx')
    while(True):
        while robot.step(timestep) != -1:
            img = camera.getImageArray()
            img = np.asarray(img, dtype=np.uint8)
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask_gray = cv2.inRange(hsv, gray_min, gray_max)
    
            # 二值化
            retval, dst = cv2.threshold(mask_gray, 30, 255, cv2.THRESH_BINARY)
            # 膨胀，白区域变大
            dst = cv2.dilate(dst, None, iterations=2)
            # # 腐蚀，白区域变小
            # dst = cv2.erode(dst, None, iterations=6)
            #矩阵切片，把需要的东西提取出来
            hawk = dst[y_bias:256,28:228]
            #cv2.imshow("hawk.jpg",hawk)
    
            #cv2.waitKey(50)
            #continue 
    
            #srcimg = cv2.normalize(srcimg, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            t1 = cv2.getTickCount()
            blob = cv2.dnn.blobFromImage(hawk, scalefactor=1,size= (66, 200),mean=[0.485])
            net.setInput(blob)
            layer = net.getUnconnectedOutLayersNames()#获取最后一层 
            #layeralls= net.getLayerNames()#获取所有的输出层名称
            #print(layer)
            #前向传播获得信息
            pred = net.forward(layer) 
            
            #print( (pred))
            #print( type(pred))#查看类型 class tuple
            #print( len(pred))#查看元组长度 1
            #print( type(pred[0]))#查看元组0的类型
            
            array_output= pred[0]
           
            #print( array_output)   
            #print( array_output.ndim) #查看维度
            #print(array_output.shape)#输出行数和列数 1x10 与netron 查看结果一样
            #print(array_output.size)#输出总共有多少元素  上边相乘的结果 
          
            
            t3 = cv2.getTickCount()
            sec = (t3 - t1)
            label = 'Inference time: %.2f ms' % (sec * 1000.0 /  cv2.getTickFrequency())
            print(label)
            
            feature_map = array_output[0]
            if(feature_map <= -0.10):
                print("turn right")
                turnRight()
            elif(feature_map  > -0.10):
                print("run go Straight ")
                goStraight()
            if(feature_map  > 0.00):
                print("turn left ")
                turnLeft()
            print( feature_map)
        
 

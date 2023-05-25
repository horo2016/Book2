import torch
import torch.nn as nn
from torch.autograd import Variable
import pdb
import numpy as np

def PreprocessData():
    
    SensorData = np.loadtxt('./SensorData/SensorData.txt')
    # Mark how many steps beforehand the collision needs to be predicted  标记向前追溯到预测碰撞的步数 20步
    SensorDataRows = []
    for i in range(15):
        #print(np.roll(SensorData[:,-1],-i-1))
        SensorDataRows.append(np.roll(SensorData[:,-1],-i-1))#先将所有行的最后一列数据滚动到 -i-1的位置
    for i in range(15):
        
        SensorData[:,-1] += SensorDataRows[i]    #取所有行，但是取列时仅取最后一列 ，仅对最后一列代替
    
    #print((SensorData))    
    np.savetxt('./SensorData/LabeledSensorData.txt',SensorData,fmt='%d')
    CollisionFullData = SensorData[ SensorData[:,-1] > 0 ]  #最后一列为碰撞数据，将最后一列大于0的全部取出来并复制
     
    # Duplicating collision Data for faster learning    复制碰撞数据加速学习
    for i in range(10):
        SensorData = np.append(SensorData,CollisionFullData,axis=0)
    # Shuffle the sensor data
    np.random.shuffle(SensorData)
    SensorNNData = torch.Tensor(SensorData[:,:-1])#1x6维  取所有行，但是取列时最后一列不取
    CollisionData = torch.Tensor(CollisionFullData[:,:-1])#取所有行，但是取列时最后一列不取
    SensorNNLabels = torch.Tensor(SensorData[:,-1]).view(-1,1)#1x6维  取所有行，但是取列时仅取最后一列
    CollisionSensorNNLabels = torch.Tensor(CollisionFullData[:,-1]).view(-1,1)
    total = SensorNNData.shape[0]#获取长度
    
    TrainSize = int(0.70*total)#70%用于训练
    TestSize = total - TrainSize#30%用于测试
    TrainSensorNNData = SensorNNData[:TrainSize]
    TrainSensorNNLabels = SensorNNLabels[:TrainSize]
    
    return TrainSize,SensorNNData,SensorNNLabels

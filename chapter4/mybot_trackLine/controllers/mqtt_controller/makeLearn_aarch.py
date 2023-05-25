import torch
import torch.nn as nn
from torch.autograd import Variable
import pdb
import numpy as np
from PreProcessing import PreprocessData
import torch.nn.functional as F



# Set Seeds For Randomness
torch.manual_seed(10)
np.random.seed(10)    
InputSize = 6  # 张量输入大小 6个特征值
batch_size = 1 # 神经网络批处理大小
NumClasses = 1 # 输出类别  碰撞类

############################################# FOR STUDENTS #####################################

NumEpochs = 25
HiddenSize = 10
device =   "cpu"  #"cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
print(f"Using {device} device")
# 创建神经网络
class Net(nn.Module):
    def __init__(self, InputSize,NumClasses):
        super(Net, self).__init__()
		###### Define The Feed Forward Layers Here! ######
        self.fc1 = nn.Linear(InputSize, HiddenSize)#线性
        self.relu1 =nn.ReLU()#转为非线性
        
        self.fc2 = nn.Linear(HiddenSize, NumClasses)#线性回归到1类
    def forward(self, x):
		###### Write Steps For Forward Pass Here! ######
        out=self.fc1(x)  #将上述层构建联合起来
        out=self.relu1(out)
        
        out=self.fc2(out)
        return out

net = Net(InputSize, NumClasses)     
print(net)


import torch.optim as optim
#criterion = nn.CrossEntropyLoss()   ###### Define The Loss Function Here! ######
criterion = nn.MSELoss()   ###### Define The Loss Function Here! ######
#optimizer =  optim.Adam(net.parameters(), lr=0.01)###### Define The Optimizer Here! ######
optimizer =  optim.SGD(net.parameters(), lr=0.00001)
optimizer.zero_grad()
##################################################################################################

def test(dataloader, model, loss_fn):
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    model.eval()
    test_loss, correct = 0, 0
    with torch.no_grad():
        for X, y in dataloader:
            X, y = X.to(device), y.to(device)
            pred = model(X)
            test_loss += loss_fn(pred, y).item()
            correct += (pred.argmax(1) == y).type(torch.float).sum().item()
    test_loss /= num_batches
    correct /= size
    print(f"Test Error: \n Accuracy: {(100*correct):>0.1f}%, Avg loss: {test_loss:>8f} \n")



if __name__ == "__main__":
        
    TrainSize,SensorNNData,SensorNNLabels = PreprocessData()
    print ('Total Epoch %d,epoch TrainSize %d' %(NumEpochs,TrainSize)) 
    loss_vals=  []
    for j in range(NumEpochs):
        losses = 0
        
        for i in range(TrainSize):  
            input_values = Variable(SensorNNData[i])
            labels = Variable(SensorNNLabels[i])
            #print(input_values)
            #print(labels)
            # Forward + Backward + Optimize
            optimizer.zero_grad()
            outputs = net(input_values)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            losses += loss.item()
            if i % 2000 == 0:#样本较大 适当往大取值
                loss, current = loss.item(), i * len(input_values)
                print(f"    loss: {loss:>7f}  [{current:>5d}/{TrainSize:>5d}]")
        loss_vals.append(round(losses/SensorNNData.shape[0],2))
        
        print ('Epoch %d, Loss: %.4f' %(j+1, losses/SensorNNData.shape[0])) 
        print(f"\n-------------------------------")
          
        torch.save(net.state_dict(), './SavedNets/NNBot.pth')
    print(len(loss_vals))
    print((loss_vals))
 
           
    



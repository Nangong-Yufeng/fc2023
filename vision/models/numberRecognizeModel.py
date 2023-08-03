import torch
import torch.nn as nn
import torch.nn.functional as F

# class Net(nn.Module):

#     def __init__(self):
#         super(Net, self).__init__()
#         # 卷积层
#         self.conv1 = nn.Conv2d(1, 6, 5, padding=2)
#         self.conv2 = nn.Conv2d(6, 16, 5)
#         # 全连接层
#         self.fc1 = nn.Linear(16 * 5 * 5, 120)
#         self.fc2 = nn.Linear(120, 84)
#         self.fc3 = nn.Linear(84, 10)

#     def forward(self, x):
#         x = self.conv1(x)           # 卷积层
#         x = F.relu(x)               # 激活函数
#         x = F.avg_pool2d(x, 2)      # 池化层

#         x = self.conv2(x)           # 卷积层
#         x = F.relu(x)               # 激活函 
#         x = F.avg_pool2d(x, 2)      # 池化层

#         x = torch.flatten(x, 1)     # 降维

#         x = self.fc1(x)             # 全连接层
#         x = F.relu(x)               # 激活函数

#         x = self.fc2(x)             # 全连接层
#         x = F.relu(x)               # 激活函数

#         x = self.fc3(x)             # 全连接层

#         return max(x)

class Net(torch.nn.Module):
    def __init__(self):
        super(Net,self).__init__()
        self.conv1 = torch.nn.Conv2d(1,10,kernel_size=5)
        self.conv2 = torch.nn.Conv2d(10,20,kernel_size=5)
        self.pooling = torch.nn.MaxPool2d(2)
        self.fc = torch.nn.Linear(320,10)
    def forward(self,x):
        batch_size = x.size(0)
        x = F.relu((self.pooling(self.conv1(x))))
        x = F.relu((self.pooling(self.conv2(x))))
        x = x.view(batch_size,-1)
        x = self.fc(x)
        return x
    
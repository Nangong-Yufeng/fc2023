import torch
import torch.nn as nn

class Net(torch.nn.Module):
    def __init__(self) -> None:
        super(Net,self).__init__()
        self.conv1 = nn.Sequential(
            nn.Conv2d(
                in_channels=1, #gray image(1,28,28),the size which can not be changed depends on EMNIST
                out_channels=16,
                kernel_size=5,
                stride=1,
                padding=int((5 - 1) / 2)
            ),#the size of images is (16,28,28)
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),#the size of image is (16,14,14)
        )
        self.conv2 = nn.Sequential(
            nn.Conv2d(
                in_channels=16,
                out_channels=32,
                kernel_size=5,
                stride=1,
                padding=int((5-1)/2) #padding = (kernel_size-1)/2,keep the height and width unchanged
            ),#the size of images is (32,14,14)
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2),#the size of images is (32,7,7)
        )
        self.linear_layer = nn.Linear(32*7*7,10)#the outputs are 10 numbers
    
    def forword(self,input):
        x = input
        x = self.conv1(x)#convolution-relu-pooling
        x = self.conv2(x)
        x = self.flatten(x)#be flattened and put it into the linear_layer
        output = self.linear_layer(x)
        return output

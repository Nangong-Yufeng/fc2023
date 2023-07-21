import os

import torch
from torchvision import transforms

from my_dataset import MyDataSet
from utils import read_split_data, plot_data_loader_image
import torch.nn.functional as F
import torch.optim as optim
import RGB2GRAY

# https://storage.googleapis.com/download.tensorflow.org/example_images/flower_photos.tgz
root = "D:\Python code\shuzishibie\dataset\shuzi"  # 数据集所在根目录

#创建一个train_loader,先要用RGB2GRAY将图片放到shuzi的对应文件中
#test_loader因为暂时没有数据集，先没写，后续补充
def main():
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print("using {} device.".format(device))

    train_images_path, train_images_label, val_images_path, val_images_label = read_split_data(root)

    data_transform = {
        "train": transforms.Compose([transforms.RandomResizedCrop(28),
                                     transforms.RandomHorizontalFlip(),
                                     transforms.ToTensor(),
                                     transforms.Normalize((0.1307,),(0.3081,))]),
        "val": transforms.Compose([transforms.ToTensor(),
                                   transforms.Normalize((0.1307,),(0.3081,))])}

    train_data_set = MyDataSet(images_path=train_images_path,
                               images_class=train_images_label,
                               transform=data_transform["train"])

    batch_size = 2
    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers
    print('Using {} dataloader workers'.format(nw))
    global train_loader
    train_loader = torch.utils.data.DataLoader(train_data_set,
                                               batch_size=batch_size,
                                               shuffle=True,
                                               num_workers=0,
                                               collate_fn=train_data_set.collate_fn)

    # plot_data_loader_image(train_loader)
    #for step, data in enumerate(train_loader):
        #images, labels = data

#降维
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

#创建损失和优化器
model = Net()
criterion = torch.nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=0.01, momentum=0.5)

#训练集训练
def train(epoch):
    running_loss = 0.0
    for batch_idx,data in enumerate(train_loader,0):
        inputs,target = data
        #inputs,target = inputs.to(device),target.to(device)
        optimizer.zero_grad()

        outputs = model(inputs)
        loss = criterion(outputs,target)
        loss.backward()
        optimizer.step()
        running_loss+=loss.item()
        if batch_idx %300 == 299:
            print("[%d, %5d] loss:%.3f" %(epoch+1,batch_idx +1 ,running_loss/300))
            running_loss = 0.0

#测试集测试
def test():
    correct = 0
    total = 0
    with torch.no_grad():
        for data in test_loader:
            images,labels  = data
            #images,labels = images.to(device), labels.to(device)
            outputs = model(images)
            _,predicted = torch.max(outputs.data,dim = 1)
            total +=labels.size(0)
            correct +=(predicted ==labels).sum().item()
            print("Accuracy on test set: %d %% " %(100 *correct/total))
            print("-------",predicted)

if __name__ == '__main__':
    main()
    #开始训练和测试
    for epoch in range(100):
        train(epoch)
        #test()

    #未完待续：yza传入的图片，先使用RGB2GRAY转为灰度图片，创建好testloader，再传入test中，输出predicted即可
    #这部分需要对接的时候再进行写入


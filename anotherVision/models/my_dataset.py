import os
import torch
from torch.utils.data import Dataset
from PIL import Image
from torchvision import transforms

# The custom dataset is necessary.
class CustomPrintedDigitsDataset(Dataset):
    def __init__(self, images_path: list, images_class: list, transform: object = None):
        self.images_path = images_path
        self.images_class = images_class
        self.transform = transform

    def __len__(self):
        return len(self.images_path)

    def __getitem__(self, item):
        img = Image.open(self.images_path[item])
        # RGB为彩色图片，L为灰度图片
        if img.mode != 'L':
            raise ValueError("image: {} isn't RGB mode.".format(self.images_path[item]))
        label = self.images_class[item]

        if self.transform is not None:
            img = self.transform(img)

        return img, label

    @staticmethod
    def collate_fn(batch):
        images, labels = tuple(zip(*batch))
        images = torch.stack(images, dim=0)
        labels = torch.as_tensor(labels)
        return images, labels


data_transform = {
    "train": transforms.Compose([transforms.RandomResizedCrop(28),
                                transforms.RandomHorizontalFlip(),
                                transforms.ToTensor(),
                                transforms.Normalize((0.1307,), (0.3081,))]),
    "test": transforms.Compose([transforms.RandomResizedCrop(28),
                               transforms.ToTensor(),
                               transforms.Normalize((0.1307,), (0.3081,))])}
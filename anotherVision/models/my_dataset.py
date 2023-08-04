import os
import torch
from torch.utils.data import Dataset
from PIL import Image

# MINST数据集的精度不够时再考虑自己标数据集，问过学长，说是没必要，作为方案二
class CustomPrintedDigitsDataset(Dataset):
    def __init__(self, root_dir, transform=None):
        self.root_dir = root_dir
        self.transform = transform
        # List to store image paths and labels
        self.samples = []

        # Load samples from the root directory
        for label in os.listdir(self.root_dir):
            label_dir = os.path.join(self.root_dir, label)
            if os.path.isdir(label_dir):
                for img_name in os.listdir(label_dir):
                    img_path = os.path.join(label_dir, img_name)
                    self.samples.append((img_path, int(label)))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        img_path, label = self.samples[index]
        # convert to grayscale
        image = Image.open(img_path).convert('L')
        if self.transform:
            image = self.transform(image)

        return image, label

    @staticmethod
    def collate_fn(self, batch):
        images, labels = tuple(zip(*batch))
        images = torch.stack(images, dim=0)
        labels = torch.as_tensor(labels)
        return images, labels

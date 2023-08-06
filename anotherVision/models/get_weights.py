import train
import test
import os
from my_dataset import CustomPrintedDigitsDataset
from my_dataset import data_transform
import neural_network as Model
import split_data
import torch
from torch.utils.data import DataLoader
from torch import optim

device = (
    "cuda"
    if torch.cuda.is_available()
    else "mps"
    if torch.backends.mps.is_available()
    else "cpu"
)
print("using {} device.".format(device))

count_device = {
    "cuda": torch.cuda.device_count(),
    "mps": 0,
    "cpu": os.cpu_count()
}

root = {
    "data": "class",
    "weights": "weights"
}

model = Model.Model().to(device)
loss = torch.nn.CrossEntropyLoss()
optimizer = optim.SGD(model.parameters(), lr=1e-3, momentum=0.5)
epochs = 1

images_path_list_dir, images_label_list_dir = split_data.split_data(root["data"])

dataset = {
    "train": CustomPrintedDigitsDataset(images_path=images_path_list_dir["train"],
                                        images_class=images_label_list_dir["train"],
                                        transform=data_transform["train"]),
    "test": CustomPrintedDigitsDataset(images_path=images_path_list_dir["test"],
                                       images_class=images_label_list_dir["test"],
                                       transform=data_transform["test"])
}

batch_size = {
    "train": 2,
    "test": 1
}

num_worker = min([count_device[device], batch_size["train"] if batch_size["train"] > 1 else 0, 8])

dataloader = {
    "train": DataLoader(dataset["train"],
                        batch_size=batch_size["train"],
                        shuffle=True,
                        num_workers=0,
                        collate_fn=dataset["train"].collate_fn
                        ),
    "test": DataLoader(dataset["test"],
                       batch_size=batch_size["test"],
                       shuffle=True,
                       num_workers=0,
                       collate_fn=dataset["test"].collate_fn
                       )
}


def get_weights(epochs):
    for i in range(epochs):
        train.train_loop(dataloader["train"], model, loss, optimizer, device)
        test.test_loop(dataloader["test"], model, loss, device)
    torch.save(model.state_dict(), root["weights"] + 'model_weights.pth')


if __name__ == "__main__":
    get_weights(epochs)

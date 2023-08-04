import torch
from torch import nn as nn
from torch import optim as op
import neural_network as model
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision.transforms import ToTensor

my_learning_rate = 1e-3
my_batch_size = 64
my_epochs = 10
my_model = model.Model()
my_loss_fn = nn.CrossEntropyLoss()
my_optimizer = op.SGD(my_model.parameters(), lr=my_learning_rate)

my_training_data = datasets.MNIST(
    root="",  # need to replenish
    train=True,
    download=True,
    transform=ToTensor()
)

my_test_data = datasets.MNIST(
    root="",
    train=False,
    download=True,
    transform=ToTensor()
)

my_train_dataloader = DataLoader(my_training_data, batch_size=my_batch_size)
my_test_dataloader = DataLoader(my_test_data, batch_size=my_batch_size)

def train_loop(dataloader, model, loss_fn, optimizer):
    size = len(dataloader.dataset)
    # training mode
    model.train()
    for batch_index, (input_data, real_label) in enumerate(dataloader):
        prediction = model(input_data)
        loss = loss_fn(prediction, real_label)
        # use the optimizer with respect to the gradient
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
        # calculate the loss every hundred times
        if batch_index % 100 == 0:
            loss, current = loss.item(), (batch_index + 1) * len(input_data)
            print(f"loss: {loss:>7f} [{current:>5d}/{size:>5d}]")

def test_loop(dataloader, model, loss_fn):
    # evaluation mode,keep the parameters unchanged
    model.eval()
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    test_loss, correct_num = 0, 0
    with torch.no_grad():
        for input_data, real_label in dataloader:
            prediction = model(input_data)
            test_loss += loss_fn(prediction, real_label).item()
            correct_num += (prediction.argmax(1) == real_label).type(torch.float).sum().item()

        average_test_loss = test_loss / num_batches
        correct_rate = correct_num / size
        print(f"Test :\n Accuracy: {(100 * correct_rate):>0.1f}%, Average loss:{average_test_loss:>8f}\n")

def main(epochs, train_dataloader, test_dataloader, model, loss_fn, optimizer):
    for i in range(epochs):
        print(f"Epoch = {i + 1}\n")
        train_loop(train_dataloader, model, loss_fn, optimizer)
        test_loop(test_dataloader, model, loss_fn)
    print("Done!")


if __name__ == "__main__":
    main(my_epochs, my_train_dataloader, my_test_dataloader, my_model, my_loss_fn, my_optimizer)

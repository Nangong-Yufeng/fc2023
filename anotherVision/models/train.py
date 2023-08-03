import torch


def train_loop(dataloader, net, loss_fn, optimizer):
    size = len(dataloader.dataset)
    # training mode
    net.train()
    for batch, (input_data, real_label) in enumerate(dataloader):
        prediction = net(input_data)
        loss = loss_fn(prediction, real_label)
        # use the optimizer with respect to the gradient
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        if batch % 100 == 0:
            loss, current = loss.item(), (batch + 1) * len(input_data)
            print(f"loss: {loss:>7f} [{current:>5d}/{size:>5d}]")

def test_loop(dataloader, net, loss_fn):
    #evaluation mode,keep the paraments unchanged
    model.eval()
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    test_loss, correct_num = 0, 0
    with torch.no_grad():
        for input_data , real_label in dataloader:
            prediction = net(input_data)
            test_loss += loss_fn(pred, y).item()
            correct_num += (prediction.argmax(1) == real_label).type(torch.float).sum().item()

        average_test_loss = test_loss / num_batches
        correct_rate = correct_num / size
        print(f"Test :\n Accuracy: {(100*correct_rate):>0.1f}%, Average loss:{average_test_loss:>8f}\n")
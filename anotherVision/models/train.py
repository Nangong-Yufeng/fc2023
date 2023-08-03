
def train_loop(dataloader, net, loss_fn, optimizer):
    size = len(dataloader.dataset)
    # training pattern
    net.train()
    for batch, (input_data, target) in enumerate(dataloader):
        prediction = net(input_data)
        loss = loss_fn(prediction, target)
        # use the optimizer with respect to the gradient
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()

        if batch % 100 == 0:
            loss, current = loss.item(), (batch + 1) * len(input_data)
            print(f"loss: {loss:>7f} [{current:>5d}/{size:>5d}]")

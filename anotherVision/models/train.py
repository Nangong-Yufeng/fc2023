
def train_loop(dataloader, model, loss_fn, optimizer, device):
    size = len(dataloader.dataset)
    # training mode
    model.train()
    for batch_index, (input_data, real_label) in enumerate(dataloader):
        # print(input_data)
        input_data, real_label = input_data.to(device), real_label.to(device)
        output_data = model(input_data)
        loss = loss_fn(output_data, real_label)
        # use the optimizer with respect to the gradient
        loss.backward()
        optimizer.step()
        optimizer.zero_grad()
        # calculate the loss every hundred times
        if batch_index % 100 == 0:
            loss, current = loss.item(), (batch_index + 1) * len(input_data)
            print(f"loss: {loss:>7f} [{current:>5d}/{size:>5d}]")






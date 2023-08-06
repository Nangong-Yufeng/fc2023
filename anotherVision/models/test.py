import torch


def test_loop(dataloader, model, loss_fn, device):
    # evaluation mode,keep the parameters unchanged
    model.eval()
    size = len(dataloader.dataset)
    num_batches = len(dataloader)
    test_loss, correct_num = 0, 0
    with torch.no_grad():
        for input_data, real_label in dataloader:
            input_data, real_label = input_data.to(device), real_label.to(device)
            output_data = model(input_data)
            test_loss += loss_fn(output_data, real_label).item()
            correct_num += (output_data.argmax(1) == real_label).type(torch.float).sum().item()

        average_test_loss = test_loss / num_batches
        correct_rate = correct_num / size
        print(f"Test :\n Accuracy: {(100 * correct_rate):>0.1f}%, Average loss:{average_test_loss:>8f}\n")

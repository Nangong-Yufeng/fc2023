from time import sleep
import torch
from utils import getWeightPath
from models.neural_network import Model

if __name__ == "__main__":
    net = Model()
    print("i am training...")
    sleep(1)
    print("finish!") 
    torch.save(net, getWeightPath('a_fake_number_recognizer'))


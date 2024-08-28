
import os
import sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

import torch
from torch.utils.data import DataLoader
from torchvision import transforms

from model import SiameseNetwork
from dataset import Dataset

class Siamese:
    def __init__(self) -> None:
        checkpoint = "/home/choiyoonji/catkin_ws/src/soomac/src/vision/siamese_network/best.pth"
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        checkpoint = torch.load(checkpoint)
        self.model = SiameseNetwork(backbone=checkpoint['backbone'])
        self.model.to(device)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.model.eval()

        self.transform = transforms.Compose([
                                                transforms.ToTensor(),
                                                transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
                                            ])
        
    def eval(self, img1, img2):
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        img1 = Image.fromarray(img1)
        img2 = Image.fromarray(img2)

        img1 = self.transform(img1).float().unsqueeze(0)
        img2 = self.transform(img2).float().unsqueeze(0)

        prob = self.model(img1.to(device), img2.to(device))

        return prob[0][0].item()


if __name__ == "__main__":
    val_path = "/home/choiyoonji/catkin_ws/src/soomac/src/vision/siamese_network/data/val"
    out_path = "/home/choiyoonji/catkin_ws/src/soomac/src/vision/siamese_network/data"
    checkpoint = "/home/choiyoonji/catkin_ws/src/soomac/src/vision/siamese_network/best.pth"

    os.makedirs(out_path, exist_ok=True)

    # Set device to CUDA if a CUDA device is available, else CPU
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    val_dataset     = Dataset(val_path, shuffle_pairs=False, augment=False)
    val_dataloader   = DataLoader(val_dataset, batch_size=1)

    criterion = torch.nn.BCELoss()

    checkpoint = torch.load(checkpoint)
    model = SiameseNetwork(backbone=checkpoint['backbone'])
    model.to(device)
    model.load_state_dict(checkpoint['model_state_dict'])
    model.eval()

    losses = []
    correct = 0
    total = 0

    inv_transform = transforms.Compose([ transforms.Normalize(mean = [ 0., 0., 0. ],
                                                         std = [ 1/0.229, 1/0.224, 1/0.225 ]),
                                    transforms.Normalize(mean = [ -0.485, -0.456, -0.406 ],
                                                         std = [ 1., 1., 1. ]),
                                   ])
    
    for i, ((img1, img2), y, (class1, class2)) in enumerate(val_dataloader):
        print("[{} / {}]".format(i, len(val_dataloader)))

        img1, img2, y = map(lambda x: x.to(device), [img1, img2, y])
        print(img1.shape)
        class1 = class1[0]
        class2 = class2[0]

        prob = model(img1, img2)
        loss = criterion(prob, y)

        losses.append(loss.item())
        correct += torch.count_nonzero(y == (prob > 0.5)).item()
        total += len(y)

        fig = plt.figure("class1={}\tclass2={}".format(class1, class2), figsize=(4, 2))
        plt.suptitle("cls1={}  conf={:.2f}  cls2={}".format(class1, prob[0][0].item(), class2))

        # Apply inverse transform (denormalization) on the images to retrieve original images.
        img1 = inv_transform(img1).cpu().numpy()[0]
        img2 = inv_transform(img2).cpu().numpy()[0]
        # show first image
        ax = fig.add_subplot(1, 2, 1)
        plt.imshow(img1[0], cmap=plt.cm.gray)
        plt.axis("off")

        # show the second image
        ax = fig.add_subplot(1, 2, 2)
        plt.imshow(img2[0], cmap=plt.cm.gray)
        plt.axis("off")

        # show the plot
        plt.savefig(os.path.join(out_path, '{}.png').format(i))

    print("Validation: Loss={:.2f}\t Accuracy={:.2f}\t".format(sum(losses)/len(losses), correct / total))
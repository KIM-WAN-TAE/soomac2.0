import torch
import torch.nn as nn
import torch.nn.functional as F

from torchvision import models

class SiameseNetwork(nn.Module):
    def __init__(self, backbone="resnet18"):
        super().__init__()
        # Create a backbone network.
        self.backbone = models.__dict__[backbone](pretrained=True, progress=True)

        # Get the number of features that are outputted by the last layer of backbone network.
        out_features = list(self.backbone.modules())[-1].out_features

        # Create an MLP as the classification head. 
        # Classifies if provided combined feature vector of the 2 images represent same object or different.
        self.cls_head = nn.Sequential(
            nn.Dropout(p=0.5),
            nn.Linear(out_features, 512),
            nn.BatchNorm1d(512),
            nn.ReLU(),

            nn.Dropout(p=0.5),
            nn.Linear(512, 64),
            nn.BatchNorm1d(64),
            nn.Sigmoid(),
            nn.Dropout(p=0.5),

            nn.Linear(64, 1),
            nn.Sigmoid(),
        )

    def forward(self, img1, img2):
        feat1 = self.backbone(img1)
        feat2 = self.backbone(img2)
        
        combined_features = feat1 * feat2

        output = self.cls_head(combined_features)
        return output
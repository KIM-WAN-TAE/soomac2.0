import torch
import torch.nn as nn
import torch.nn.functional as F

import numpy as np

from model import Siamese


def main():
    Net = Siamese()
    loss_fn = torch.nn.BCEWithLogitsLoss(size_average=True)
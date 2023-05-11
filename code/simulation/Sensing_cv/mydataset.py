
from torch.utils.data import Dataset
import torch
import os
import cv2
import random
import numpy as np
from PIL import Image

cls_dict = {
    'Blisterpack': 0,
    'Bottle': 1,
    'Cook': 2,
    'Metal': 3,
    'Paper': 4,
    'Plastic': 5,
    'Shoe': 6,
    'Styrofoampiece': 7}

class ImageClassifyDataset(Dataset):
    def __init__(self, imagedir, classify_num=8, train=True, mode='train', transform=None):

        self.imagedir = imagedir
        self.classify_num = classify_num
        self.transform = transform
        self.mode = mode
        self.img_list = []
        img_files = os.listdir(imagedir)
        for img in img_files:
            cls = img.split('_')[0]
            label = cls_dict[cls]
            p = os.path.join(imagedir, img)
            self.img_list.append([p, label])
        img_len = len(self.img_list)
        if mode == 'val':
            self.img_list = random.sample(self.img_list, int(0.3*img_len))

    def __len__(self):
        return len(self.img_list)
        
    def __getitem__(self, index):
        image = Image.open(self.img_list[index][0])
        if self.transform:
            image = self.transform(image)
        _int_label = int(self.img_list[index][1])	
        label = torch.tensor(_int_label,dtype=torch.long)
        return image, label



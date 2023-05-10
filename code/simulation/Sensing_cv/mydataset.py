
from torch.utils.data import Dataset
import torch
import os
import cv2
import random
import numpy as np
from PIL import Image
cls_dict ={'Shoe': 0,
 'Paper': 1,
 'Styrofoampiece': 2,
 'Cook': 3,
 'Bottle': 4,
 'Metal': 5,
 'Blisterpack': 6,
 'Plastic': 7}
class ImageClassifyDataset(Dataset):
    def __init__(self, imagedir, classify_num=8, train=True, transform=None):

        self.imagedir = imagedir
        self.classify_num = classify_num
        self.transform = transform
        self.img_list = []
        img_files = os.listdir(imagedir)
        for img in img_files:
            cls = img.split('_')[0]
            label = cls_dict[cls]
            p = os.path.join(imagedir, img)
            self.img_list.append([p, label])


    def __len__(self):
        return len(self.img_list)
        
    def __getitem__(self, index):
        image = Image.open(self.img_list[index][0])
        if self.transform:
            image = self.transform(image)
        _int_label = int(self.img_list[index][1])	
        label = torch.tensor(_int_label,dtype=torch.long)
        return image, label



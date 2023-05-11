import os
import json
import mydataset
import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import transforms, datasets
from tqdm import tqdm
from torch.utils.data import DataLoader, random_split
from models import resnet50
from collections import OrderedDict
from torch.utils.tensorboard import SummaryWriter
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import numpy as np


def plot_confusion_matrix(cm, labels_name, title):
    cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]  
    plt.imshow(cm, interpolation='nearest',cmap=plt.cm.Blues)    
    plt.title(title)    
    plt.colorbar()
    num_local = np.array(range(len(labels_name)))    
    plt.xticks(num_local, labels_name, rotation=45)    
    plt.yticks(num_local, labels_name)    
    plt.ylabel('True label')    
    plt.xlabel('Predicted label')


def main():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("using {} device.".format(device))
    data_transforms = transforms.Compose([transforms.Resize(256),  
                                   transforms.CenterCrop(224),  
                                   transforms.ToTensor(),
                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])
    data_root = './cropped_result/'  # get data root path
    val_dataset = mydataset.ImageClassifyDataset(data_root, transform=data_transforms, mode='val')
    batch_size = 32

    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers
    print('Using {} dataloader workers every process'.format(nw))

    val_loader = DataLoader(val_dataset,
                                batch_size=batch_size, shuffle=False,
                                num_workers=nw)

    net = resnet50(8)
    net.load_state_dict(torch.load('./resNet50_pre.pth', map_location=device))
    all_labels = []
    all_pre = []
    net.to(device)
    acc = 0.0
    with torch.no_grad():
        val_bar = tqdm(val_loader)
        for val_data in val_bar:
            val_images, val_labels = val_data
            outputs = net(val_images.to(device))
            predict_y = torch.max(outputs, dim=1)[1]
            acc += torch.eq(predict_y, val_labels.to(device)).sum().item()
            for item in predict_y.tolist():
                all_pre.append(item)
            for item in val_labels.tolist():
                all_labels.append(item)
            
    val_accurate = acc / len(val_dataset)
    print(val_accurate)
    cls_names = ['Blisterpack','Bottle','Cook','Metal','Paper','Plastic','Shoe','Styrofoampiece']
    print(classification_report(all_labels, all_pre, target_names=cls_names))
    cm= confusion_matrix(all_labels, all_pre, labels=[0,1,2,3,4,5,6,7])
    print(cm)

    labels = cls_names
    np.set_printoptions(precision=2)
    cm_normalized = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
    plt.figure(figsize=(10, 8), dpi=120)
    tick_marks = np.array(range(len(labels))) + 0.5

    ind_array = np.arange(len(labels))
    x, y = np.meshgrid(ind_array, ind_array)

    for x_val, y_val in zip(x.flatten(), y.flatten()):
        c = cm_normalized[y_val][x_val]
        if c > 0.01:
            plt.text(x_val, y_val, "%0.2f" % (c,), color='black', fontsize=7, va='center', ha='center')
    # offset the tick
    plt.gca().set_xticks(tick_marks, minor=True)
    plt.gca().set_yticks(tick_marks, minor=True)
    plt.gca().xaxis.set_ticks_position('none')
    plt.gca().yaxis.set_ticks_position('none')
    plt.grid(True, which='minor', linestyle='-')
    plt.gcf().subplots_adjust(bottom=0.15)

    plot_confusion_matrix(cm_normalized,cls_names, title='confusion matrix')
    # show confusion matrix
    plt.savefig('./Confusion_Matrix.png', format='png')
    plt.show()

if __name__ == '__main__':
    main()

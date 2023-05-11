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

def main():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    print("using {} device.".format(device))

    data_transform = {
        "train": transforms.Compose([transforms.RandomResizedCrop(256),
                                     transforms.RandomHorizontalFlip(),
                                     transforms.ToTensor(),
                                     transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])]),  
        "val": transforms.Compose([transforms.Resize(256),  
                                   transforms.CenterCrop(224),  
                                   transforms.ToTensor(),
                                   transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])])}

    data_root = './cropped_result/'  # get data root path
    dataset = mydataset.ImageClassifyDataset(data_root, transform=data_transform["train"])
    total_len = len(dataset)
    train_len = int(0.7 * total_len)
    val_len = total_len - train_len
    train_dataset, val_dataset = random_split(dataset, [train_len, val_len])

    batch_size = 16
    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])  # number of workers
    print('Using {} dataloader workers every process'.format(nw))

    train_loader = DataLoader(train_dataset,
                            batch_size=batch_size, shuffle=True,
                            num_workers=nw)

    validate_loader = DataLoader(val_dataset,
                                batch_size=batch_size, shuffle=False,
                                num_workers=nw)

    print("using {} images for training, {} images for validation.".format(train_len,
                                                                           val_len))

    net = resnet50(8)
    # load pretrain weights
    # model_weight_path = "./resnet50-19c8e357.pth"  # 保存权重的路径
    # assert os.path.exists(model_weight_path), "file {} does not exist.".format(model_weight_path)
    net.load_state_dict(torch.load('./resNet50.pth', map_location=device))

    # state_dict = torch.load(model_weight_path, map_location='cpu')
    # new_state_dict = OrderedDict()
    # for k, v in state_dict.items():
    #     name = k
    #     if name.startswith('module.'):
    #         name = name[7:]  
    #     new_state_dict[name] = v.to(device)
    # net.load_state_dict(new_state_dict)
    # for param in net.parameters():
    #     param.requires_grad = False

    # change fc layer structure
    writer = SummaryWriter(comment='train')

    # in_channel = net.fc.in_features  # net.fc即model.py中定义的网络的全连接层,in_features是输入特征矩阵的深度
    # net.fc = nn.Linear(in_channel, 8)  
    net.to(device)

    # define loss function
    loss_function = nn.CrossEntropyLoss()

    # construct an optimizer
    params = [p for p in net.parameters() if p.requires_grad]
    optimizer = optim.SGD(params, lr=0.01)

    epochs = 200
    best_acc = 0.0
    save_path = './resNet50_pre.pth'
    train_steps = len(train_loader)
    for epoch in range(epochs):
        # train
        net.train()  
        running_loss = 0.0
        train_bar = tqdm(train_loader)
        for step, data in enumerate(train_bar):
            images, labels = data
            optimizer.zero_grad()
            logits = net(images.to(device))
            loss = loss_function(logits, labels.to(device))  
            loss.backward()  
            optimizer.step()  

            # print statistics
            running_loss += loss.item()

            train_bar.desc = "train epoch[{}/{}] loss:{:.3f}".format(epoch + 1,
                                                                     epochs,
                                                                     loss)
        writer.add_scalar('loss/train_loss', running_loss, epoch)
        # validate
        if epoch % 10 == 9:
            net.eval()  
            acc = 0.0  # accumulate accurate number / epoch
            with torch.no_grad():  
                val_bar = tqdm(validate_loader)
                for val_data in val_bar:
                    val_images, val_labels = val_data
                    outputs = net(val_images.to(device))
                    # loss = loss_function(outputs, test_labels)
                    predict_y = torch.max(outputs, dim=1)[1]
                    acc += torch.eq(predict_y, val_labels.to(device)).sum().item()

                    val_bar.desc = "valid epoch[{}/{}]".format(epoch + 1,
                                                            epochs)

            val_accurate = acc / val_len
            print('[epoch %d] train_loss: %.3f  val_accuracy: %.3f' %
                (epoch + 1, running_loss / train_steps, val_accurate))
            writer.add_scalar('loss/val_loss',running_loss / train_steps, epoch)
            writer.add_scalar('loss/accuracy', val_accurate, epoch)
            if val_accurate > best_acc:
                best_acc = val_accurate
                torch.save(net.state_dict(), save_path)
    print('Best accuracy is %.2f.'%(best_acc))
    print('Finished Training')


if __name__ == '__main__':
    main()


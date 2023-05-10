import os
import json
import argparse
import torch
from PIL import Image
from torchvision import transforms
import matplotlib.pyplot as plt
import json
import os
import random
from Sensing_cv.models import resnet50

class_dict = {
    0:'Blisterpack',
    1:'Bottle',
    2:'Cook',
    3:'Metal',
    4:'Paper',
    5:'Plastic',
    6:'Shoe',
    7:'Styrofoampiece'
}

def predict(path):
    device = torch.device("cuda:"+ str(opt.device) if torch.cuda.is_available() else "cpu")

    data_transform = transforms.Compose(
        [transforms.Resize(256),
         transforms.CenterCrop(224),
         transforms.ToTensor(),
         transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])]) 
    # load image

    img_path = path
    

    assert os.path.exists(img_path), "file: '{}' dose not exist.".format(img_path)
    img = Image.open(img_path)
    # [N, C, H, W]
    img = data_transform(img)
    # expand batch dimension
    img = torch.unsqueeze(img, dim=0)


    # create model
    model = resnet50(num_classes=8).to(device)  

    # load model weights
    weights_path = "Sensing_cv/resNet50.pth"
    model.load_state_dict(torch.load(weights_path, map_location=device)) 

    # prediction
    model.eval()  
    with torch.no_grad():  
        # predict class
        output = torch.squeeze(model(img.to(device))).cpu()
        predict = torch.softmax(output, dim=0)
        predict_cla = torch.argmax(predict).numpy()

    print_res = "class: {}   prob: {:.3}".format(class_dict[int(predict_cla)],
                                                 predict[predict_cla].numpy())
    return print_res

def parse_opt():
    parser = argparse.ArgumentParser()
    
    parser.add_argument('--source', required=True, type=str, help='img to detect')
    parser.add_argument('--device', default=1, help='cuda device, i.e. 0 or 0,1,2,3 or cpu')

    opt = parser.parse_args()
    return opt




def getRandomImgs(num):
    folder_path = os.getcwd() + "/Sensing_cv/cropped_result/"
    file_names = os.listdir(folder_path)
    image_names = [f for f in file_names if f.endswith((".jpg", ".jpeg", ".png", ".bmp", ".gif"))]
    randomArr = random.sample(range(1, 7000), num)
    pathArr = []
    for i in range(num):
        id = randomArr[i]
        imgName = image_names[id]
        path = os.getcwd() + "/Sensing_cv/cropped_result/" + imgName
        pathArr.append(path)
    return pathArr  
    

def convert2Type(res):
    type = []
    
    for i in res:
        item = dict()
        words = i.split()
        className = ""
        for i, word in enumerate(words):
            if word == 'class:':
                className = words[i+1]
        if className == "Cook":
            typeId = 0
        elif className == "Styrofoampiece":
            typeId = 2
        else:
            typeId = 1
        item["obj_id"] = typeId
        item["name"] = className
        type.append(item)
    return type


def run(num):
    pictureNum = num
    pathArr = getRandomImgs(pictureNum)
    res = []
    res_json = []
    for img in range(pictureNum):
        res.append(predict(pathArr[img]))

    res_json = convert2Type(res)
    res_json.insert(0,{"num_obj":len(res_json)})
    stored_json = json.dumps(res_json)

    path = os.getcwd().replace("/Sensing_cv","")
    path = path + "/data/data.json"

    with open(path, "w") as f:
        f.write(stored_json)
    
    
    print("======data preparation done======")
    
    
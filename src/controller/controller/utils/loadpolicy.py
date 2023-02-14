import sys
sys.path.insert(0,'/home/orin/Documents/isaac_rover_physical_2.0/src/controller/controller/utils')

from student_model import Student
from teacher_model import Teacher

student = False
teacher = False

import torch


class student_loader():

    def __init__(self,info,model_path="") -> None:
        self.cfg = self.cfg_fn()
        self.info = info
        self.model = self.load_model('/home/orin/Documents/isaac_rover_physical_2.0/src/controller/controller/utils/models/' + model_path + '/student/best.pt')
        self.h = self.model.belief_encoder.init_hidden(1).to('cuda:0')

    def load_model(self, model_name):
        model = Student(self.info, self.cfg)
        checkpoint = torch.load(model_name)
        model.load_state_dict(checkpoint['state_dict'])
        model.eval()
        model.cuda()

        return model

    def act(self,observations):
        with torch.no_grad():
            #print(observations.shape)
            actions, predictions, self.h = self.model(observations.unsqueeze(1),self.h)
            return actions

    def cfg_fn(self):
        cfg = {
            "info":{
                "reset":            0,
                "actions":          0,
                "proprioceptive":   0,
                "exteroceptive":    0,
            },
            "learning":{
                "learning_rate": 1e-4,
                "epochs": 500,
                "batch_size": 1,
            },
            "encoder":{
                "activation_function": "leakyrelu",
                "encoder_features": [80,60]},

            "belief_encoder": {
                "hidden_dim":       300,
                "n_layers":         2,
                "activation_function":  "leakyrelu",
                "gb_features": [64,64,120],
                "ga_features": [64,64,120]},

            "belief_decoder": {
                "activation_function": "leakyrelu",
                "gate_features":    [128,256,512],
                "decoder_features": [128,256,512]
            },
            "mlp":{"activation_function": "leakyrelu",
                "network_features": [256,160,128]},
                }

        return cfg   

class teacher_loader():
    def __init__(self,info,model_path="") -> None:
        self.cfg = self.cfg_fn()
        self.info = info
        self.model = self.load_model(model_path)
        #self.h = self.model.belief_encoder.init_hidden(1).to('cuda:0')

    def load_model(self, model_path):
        model = Teacher(self.info, self.cfg, '/home/orin/Documents/isaac_rover_physical_2.0/src/controller/controller/utils/models/' + model_path + '/teacher/best.pt')
        #checkpoint = torch.load(model_name)
        #model.load_state_dict(checkpoint['state_dict'])
        model.eval()
        model.cuda()

        return model

    def act(self,observations):
        with torch.no_grad():
            #print(observations.shape)
            actions= self.model(observations.unsqueeze(1))
            return actions

    def cfg_fn(self):
        cfg = {
            "info":{
                "reset":            0,
                "actions":          0,
                "proprioceptive":   0,
                "exteroceptive":    0,
            },
            "encoder":{
                "activation_function": "leakyrelu",
                "encoder_features": [80,60]},

            "mlp":{"activation_function": "leakyrelu",
                "network_features": [256,160,128]},
                }

        return cfg   

info = {
    "reset": 0,
    "actions": 2,
    "proprioceptive": 4,
    "sparse": 634,
    "dense": 1112}



teacher = teacher_loader(info, "model1")
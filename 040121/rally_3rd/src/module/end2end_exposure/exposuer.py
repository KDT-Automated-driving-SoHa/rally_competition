#!/usr/bin/env python
# -*- coding: utf-8 -*-

import torch


def load_model(epoch_cnt, batch_count):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    script_dir = os.path.dirname(__file__)
    rospack = rospkg.RosPack()
    LoadPath_main = rospack.get_path('white2')+"/src/save/main_model_"+str(episode).zfill(6)+"_"+str(batch_cnt).zfill(6)+ ".pth"
    with open(LoadPath_main, 'rb') as f:
        LoadBuffer = io.BytesIO(f.read())
    model.load_state_dict(torch.load(LoadBuffer, map_location=device))
    return model




net = end2end().to(device)
net = study_model_load(1000, 77, net, device)

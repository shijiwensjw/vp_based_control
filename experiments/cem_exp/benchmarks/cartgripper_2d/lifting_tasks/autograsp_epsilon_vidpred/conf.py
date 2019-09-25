import os
from python_visual_mpc.video_prediction.setup_predictor_towers import setup_predictor
current_dir = os.path.dirname(os.path.realpath(__file__))

# local output directory
OUT_DIR = current_dir + '/modeldata'

from python_visual_mpc.video_prediction.dynamic_rnn_model.alex_model_interface import Alex_Interface_Model
from video_prediction.models.savp_model import SAVPVideoPredictionModel
import video_prediction


base_dir = video_prediction.__file__
base_dir = '/'.join(str.split(base_dir, '/')[:-2])
modeldir = base_dir + '/pretrained_models/cartgripper_xz_grasp/vanilla_env/'
configuration = {
'pred_model': Alex_Interface_Model,
'setup_predictor':setup_predictor,
'json_dir':  modeldir + '/view0/model.savp.None',
'pretrained_model': modeldir + '/view0/model.savp.None/model-300000',   # 'filepath of a pretrained model to resume training from.' ,
'sequence_length': 15,      # 'sequence length to load, including context frames.' ,
'context_frames': 2,        # of frames before predictions.' ,
'model': 'appflow',            #'model architecture to use - CDNA, DNA, or STP' ,
'batch_size': 200,
'sdim':3,
'adim':3,
'orig_size':[48,64],
'ndesig':1,
'ncam':1,
}
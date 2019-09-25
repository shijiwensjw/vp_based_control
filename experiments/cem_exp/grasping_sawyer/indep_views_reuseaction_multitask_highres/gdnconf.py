import python_visual_mpc
base_dir = python_visual_mpc.__file__

base_dir = '/'.join(str.split(base_dir, '/')[:-2])

# tf record data location:
import os
current_dir = os.path.dirname(os.path.realpath(__file__))

# local output directory
OUT_DIR = current_dir + '/modeldata'
# local output directory

from python_visual_mpc.goaldistancenet.multiview_testgdn import MulltiviewTestGDN

configuration = {
'experiment_name': 'correction',
'pred_model':MulltiviewTestGDN,
'pretrained_model': [base_dir + '/tensorflow_data/gdn/weiss/multiview_new_env_96x128_len8/view0/modeldata/model56002',
                     base_dir + '/tensorflow_data/gdn/weiss/multiview_new_env_96x128_len8/view1/modeldata/model56002'],
'output_dir': OUT_DIR,      #'directory for model checkpoints.' ,
'current_dir': base_dir,   #'directory for writing summary.' ,
'num_iterations':100000,
'sequence_length':8,
'train_val_split':.95,
'visualize':'',
'skip_frame':1,
'batch_size': 1,           #'batch size for training' ,
'learning_rate': 0.001,     #'the base learning rate of the generator' ,
'normalization':'None',
'image_only':'',
'orig_size': [96,128],
'norm':'charbonnier',
'smoothcost':1e-7,
'smoothmode':'2nd',
'fwd_bwd':'',
'flow_diff_cost':1e-7,
'hard_occ_thresh':'',
'occlusion_handling':1e-7,
'occ_thres_mult':0.5,
'occ_thres_offset':1.,
'flow_penal':1e-7,
'ch_mult':4,
'decay_lr':'',
'view':0,
'new_loader': True
}



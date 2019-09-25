import python_visual_mpc
base_dir = python_visual_mpc.__file__

base_dir = '/'.join(str.split(base_dir, '/')[:-2])

# tf record data location:
import os
DATA_DIR = {os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/sudri_ag/good': 8,
         os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/sudri_ag/bad': 12,
         os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/vestri_ag/good': 8,
         os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/vestri_ag/bad': 12,
         os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/sudri_ag_long/good': 12,
         os.environ['VMPC_DATA_DIR'] + '/sawyer_grasping/sawyer_data_hres/sudri_ag_long/bad': 12,
            }
import os
current_dir = os.path.dirname(os.path.realpath(__file__))

# local output directory
OUT_DIR = current_dir + '/modeldata'


configuration = {
'experiment_name': 'correction',
'data_dir': DATA_DIR,       # 'directory containing data.' ,
'output_dir': OUT_DIR,      #'directory for model checkpoints.' ,
'current_dir': base_dir,   #'directory for writing summary.' ,
'num_iterations':100000,
'sequence_length':14,
'train_val_split':.95,
'visualize':'',
'skip_frame':1,
'batch_size': 64,           #'batch size for training' ,
'learning_rate': 0.001,     #'the base learning rate of the generator' ,
'normalization':'None',
'sdim' :5,
'adim' :4,
'orig_size': [96,128],
'norm':'charbonnier',
'smoothcost':1e-7,
'smoothmode':'2nd',
'fwd_bwd':'',
'flow_diff_cost':1e-7,
'hard_occ_thresh':'',
'occlusion_handling':1e-4,
'occ_thres_mult':0.5,
'occ_thres_offset':1.,
'flow_penal':1e-7,
'ch_mult':4,
'decay_lr':'',
'view':1,
}
import pickle as pkl
import os

file_name = "checkpoint.pkl"
if os.path.exists(file_name):
    with open(file_name,'wb') as f:
        dic = {'ntraj' : 210, 'broken_traj': []}
        pkl.dump(dic, f)

with open(file_name, 'rb') as f:
    a = pkl.load(f)
    print('traj th: ',a['ntraj'])
# ck = {'ntraj' : 0, 'broken_traj' : []}

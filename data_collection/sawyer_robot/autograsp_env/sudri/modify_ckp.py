import pickle as pkl
import os
import argparse

file_name = "checkpoint.pkl"

with open(file_name, 'rb') as f:
    a = pkl.load(f)
    print('traj th(before): ',a['ntraj'])

parser = argparse.ArgumentParser()
parser.add_argument('ntraj', type=int)
args = parser.parse_args()


if os.path.exists(file_name):
    with open(file_name,'wb') as f:
        dic = {'ntraj' : args.ntraj, 'broken_traj': []}
        pkl.dump(dic, f)

with open(file_name, 'rb') as f:
    a = pkl.load(f)
    print('traj th(now): ',a['ntraj'])

# ck = {'ntraj' : 0, 'broken_traj' : []}

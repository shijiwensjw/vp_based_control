import glob
import imageio
from PIL import Image
import numpy as np
import os

def save_video_mp4(filename, frames):
    writer = imageio.get_writer(filename + '.mp4', fps=10)
    for frame in frames:
        writer.append_data(frame)
    writer.close()

def save_video_gif(filename, frames):
    imageio.mimsave(filename+'.gif', frames, 'GIF', duration = 0.01)

def save_video(grp, num):
    imlist = []
    for t in range(31):
        image_folder = '/home/steven/Project/graduate_design/vp_based_control/data_collection/sawyer_robot/autograsp_env/sudri/train/traj_group{}/traj{}'.format(str(grp), str(num)) + '/images0'
        # print('image_folder: ',image_folder)
        try:
            [imfile] = glob.glob(image_folder + "/im_{}.jpg".format(str(t)))
        except Exsception:
            print('No such file')
        # imfile = image_folder + "/im_{}.jpg".format(str(t))
        # print('imfile:', imfile)
        image = np.asarray(Image.open(imfile))

        imlist.append(image)

    save_dir = '/home/steven/Project/graduate_design/vp_based_control/data_collection/sawyer_robot/autograsp_env/sudri/rawdata_video/traj_group{}'.format(str(grp))
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    save_video_gif(save_dir+'/traj{}'.format(num), imlist)

def main():
    for grp in range(10):
        for num in range(10):
            save_video(grp, num)
if __name__ == '__main__':
    main()

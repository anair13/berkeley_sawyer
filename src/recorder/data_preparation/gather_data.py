import os
import glob
import numpy as np
from PIL import Image
import pdb

import create_gif

class Trajectory(object):
    def __init__(self, cameranames):
        self.cameranames = cameranames
        self.T = 15
        self.n_cam = len(cameranames)
        self.images = np.zeros((self.T, self.n_cam, 64, 64, 3), dtype = np.uint8)
        self.dimages = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.uint8)
        self.dvalues = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.float32)

def gather(sourcedirs):


    # go to first source and get group names:
    groupnames = glob.glob(os.path.join(sourcedirs[0], '*'))
    groupnames = [str.split(n, '/')[-1] for n in groupnames]

    src_names = [str.split(n, '/')[-1] for n in sourcedirs]


    traj_list = []
    ntraj_max = 7
    itraj = 0

    for gr in groupnames:
        gr_dir_main = sourcedirs[0] + '/' + gr

        trajnames = glob.glob(os.path.join(gr_dir_main, '*'))
        trajnames = [str.split(n, '/')[-1] for n in trajnames]

        for trajname in trajnames:

            traj = Trajectory(src_names)
            traj_dir_main = gr_dir_main + '/' + trajname

            traj_subpath = '/'.join(str.split(traj_dir_main, '/')[-2:])

            for i_src, src in enumerate(sourcedirs):

                sourcedir = os.path.join(src, traj_subpath)
                # sourcedir = src + '/' + traj_subpath

                for t in range(traj.T):
                    file = glob.glob(sourcedir + '/'+
                         'images/{0}_cropped_im{1}_*.png'.format(src_names[i_src], t))
                    if len(file)> 1:
                        print 'warning: more than 1 image one per time step for {}'.format(sourcedir + '/'+
                         'images/{0}_cropped_im{1}_*.png'.format(src_names[i_src], t))
                    file = file[0]

                    im = Image.open(file)
                    im.load()
                    if src_names[i_src] == 'aux1':
                        im = im.rotate(180)
                    im = np.asarray(im)
                    traj.images[t, i_src] = im

                    file = glob.glob(sourcedir + '/'+ 'depth_images/{0}_cropped_depth_im{1}_*.png'
                                       .format(src_names[i_src], t))

                    if len(file)> 1:
                        print 'warning: more than 1 depthimage one per time step for {}'.format(sourcedir + '/'+
                         'images/{0}_cropped_im{1}_*.png'.format(src_names[i_src], t))
                    file = file[0]

                    im = Image.open(file)
                    im.load()
                    if src_names[i_src] == 'aux1':
                        im = im.rotate(180)
                    im = np.asarray(im)
                    traj.dimages[t, i_src] = im

            traj_list.append(traj)
            itraj += 1

            if itraj == ntraj_max:
                create_gif.comp_video(traj_list, 'test')
                return


    # '/'.join(str.split(current_dir, '/')[:-3]) + '/pushing_data/random_action_var10/train'


if __name__ == "__main__":

    sourcedirs =['/home/guser/sawyer_data/main',
                 '/home/guser/sawyer_data/aux1']

    gather(sourcedirs)
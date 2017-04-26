import os
import glob
import numpy as np
from PIL import Image
import tensorflow as tf
import cPickle

def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))
def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

import pdb
import re

import create_gif

class Trajectory(object):
    def __init__(self, cameranames):
        self.cameranames = cameranames
        self.T = 29
        self.n_cam = len(cameranames)
        self.images = np.zeros((self.T, self.n_cam, 64, 64, 3), dtype = np.uint8)  # n_cam=0: main, n_cam=1: aux1
        self.dimages = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.uint8)
        self.dvalues = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.float32)
        self.actions = None
        self.endeffector_pos = None
        self.joint_angles = np.zeros((self.T, 7))

class TF_rec_converter:
    def __init__(self):
        pass

    def gather(self, sourcedirs, gif_dir= None, tf_rec_dir = None):

        nopkl_file = 0

        self.tfrec_dir = tf_rec_dir
        for dirs in sourcedirs:
            if not os.path.exists(dirs):
                raise ValueError('path {} does not exist!'.format(dirs))
        if not os.path.exists(tf_rec_dir):
            raise ValueError('path {} does not exist!'.format(tf_rec_dir))

        # go to first source and get group names:
        groupnames = glob.glob(os.path.join(sourcedirs[0], '*'))
        groupnames = [str.split(n, '/')[-1] for n in groupnames]

        self.src_names = [str.split(n, '/')[-1] for n in sourcedirs]

        traj_list = []
        ntraj_max = 7
        itraj = 0
        traj_start_ind = 0
        for gr in groupnames:  # loop over groups
            gr_dir_main = sourcedirs[0] + '/' + gr

            trajnames = glob.glob(os.path.join(gr_dir_main, '*'))
            trajnames = [str.split(n, '/')[-1] for n in trajnames]


            for trajname in trajnames:  # loop of traj0, traj1,..
                print 'processing', trajname

                traj_index = re.match('.*?([0-9]+)$', trajname).group(1)

                traj = Trajectory(self.src_names)
                traj_dir_main = gr_dir_main + '/' + trajname

                traj_subpath = '/'.join(str.split(traj_dir_main, '/')[-2:])

                #load actions:
                pkl_file = traj_dir_main + '/joint_angles_traj{}.pkl'.format(traj_index)
                if not os.path.isfile(pkl_file):
                    nopkl_file += 1
                    print 'no pkl file found, file no: ', nopkl_file
                    continue

                actions = cPickle.load(open(pkl_file, "rb"))
                traj.actions = actions['actions']
                traj.joint_angles = actions['jointangles']
                traj.endeffector_pos = actions['endeffector_pos']

                for i_src, src in enumerate(sourcedirs):  # loop over main, aux1, ..

                    traj_dir_src = os.path.join(src, traj_subpath)

                    for t in range(traj.T):

                        # getting color image:
                        file = glob.glob(traj_dir_src + '/'+
                             'images/{0}_cropped_im{1}_*.png'.format(self.src_names[i_src], t))
                        if len(file)> 1:
                            print 'warning: more than 1 image one per time step for {}'.format(traj_dir_src + '/'+
                             'images/{0}_cropped_im{1}_*.png'.format(self.src_names[i_src], t))
                        file = file[0]

                        im = Image.open(file)
                        im.load()
                        if self.src_names[i_src] == 'aux1':
                            im = im.rotate(180)
                        im = np.asarray(im)
                        traj.images[t, i_src] = im

                        # getting depth image:
                        file = glob.glob(traj_dir_src + '/'+ 'depth_images/{0}_cropped_depth_im{1}_*.png'
                                           .format(self.src_names[i_src], t))
                        if len(file)> 1:
                            print 'warning: more than 1 depthimage one per time step for {}'.format(traj_dir_src + '/'+
                             'images/{0}_cropped_im{1}_*.png'.format(self.src_names[i_src], t))
                        file = file[0]

                        im = Image.open(file)
                        im.load()
                        if self.src_names[i_src] == 'aux1':
                            im = im.rotate(180)
                        im = np.asarray(im)
                        traj.dimages[t, i_src] = im

                        file = glob.glob(traj_dir_src +'/depth_images/{0}_depth_im{1}_*.pkl'.format(self.src_names[i_src], t))
                        file = file[0]
                        traj.dvalues[t, i_src] = cPickle.load(open(file, "rb"))

                traj_list.append(traj)
                itraj += 1

                if tf_rec_dir != None:
                    if len(traj_list) == 256:

                        filename = 'traj_{0}_to_{1}' \
                            .format(traj_start_ind, itraj-1)
                        self.save_tf_record(filename, traj_list)
                        traj_start_ind = itraj
                        traj_list = []

                if gif_dir != None:
                    if itraj == ntraj_max:
                        create_gif.comp_video(traj_list, gif_dir)
                        print 'created gif, exiting'
                        return

    def save_tf_record(self, filename, trajectory_list):
        """
        saves data_files from one sample trajectory into one tf-record file
        """
        train_dir = os.path.join(tf_rec_dir, 'train')
        if not os.path.exists(self.tfrec_dir):
            os.makedirs(train_dir)

        filename = os.path.join(train_dir, filename + '.tfrecords')
        print('Writing', filename)
        writer = tf.python_io.TFRecordWriter(filename)

        feature = {}

        for tr in range(len(trajectory_list)):

            traj = trajectory_list[tr]
            sequence_length = traj.T

            for tstep in range(sequence_length):

                feature[str(tstep) + '/action']= _float_feature(traj.actions[tstep].tolist())
                feature[str(tstep) + '/endeffector_pos'] = _float_feature(traj.endeffector_pos[tstep].tolist())
                # feature[str(tstep) + '/jointangles'] = _float_feature(traj.joint_angles.tolist())

                feature[str(tstep) + '/depth_main'] = _float_feature(traj.dvalues[tstep, 0].flatten().tolist())
                feature[str(tstep) + '/depth_main'] = _float_feature(traj.dvalues[tstep, 1].flatten().tolist())

                image_raw = traj.images[tstep, 0].tostring()  # for camera 0, i.e. main
                feature[str(tstep) + '/image_main/encoded'] = _bytes_feature(image_raw)
                image_raw = traj.images[tstep, 1].tostring()  # for camera 1, i.e. aux1
                feature[str(tstep) + '/image_aux1/encoded'] = _bytes_feature(image_raw)

                image_raw = traj.images[tstep, 0].tostring()  # for camera 0, i.e. main
                feature[str(tstep) + '/image_main/encoded'] = _bytes_feature(image_raw)
                image_raw = traj.images[tstep, 1].tostring()  # for camera 1, i.e. aux1
                feature[str(tstep) + '/image_aux1/encoded'] = _bytes_feature(image_raw)

            example = tf.train.Example(features=tf.train.Features(feature=feature))
            writer.write(example.SerializeToString())

        writer.close()


if __name__ == "__main__":

    # sourcedirs =['/home/guser/sawyer_data/main',
    #              '/home/guser/sawyer_data/aux1']

    sourcedirs =["/media/frederik/harddrive/sawyerdata/noup_29/main",
                 "/media/frederik/harddrive/sawyerdata/noup_29/aux1"]

    gif_dir = '/home/frederik/Documents/sawyer_data/gathered_data/'
    tf_rec_dir = '/home/frederik/Documents/lsdc/pushing_data/sawyer_noup_29/'

    tfrec_converter = TF_rec_converter()
    tfrec_converter.gather(sourcedirs, tf_rec_dir = tf_rec_dir)


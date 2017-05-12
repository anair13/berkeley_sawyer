import os
import glob
import numpy as np
from PIL import Image
import tensorflow as tf
import cPickle
import imutils   #pip install imutils
import random

def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))
def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

import pdb
import re

import cv2

import create_gif

class Trajectory(object):
    def __init__(self, cameranames, split_seq_by = 1):
        self.cameranames = cameranames
        self.T = 30/split_seq_by
        self.n_cam = len(cameranames)
        self.images = np.zeros((self.T, self.n_cam, 64, 64, 3), dtype = np.uint8)  # n_cam=0: main, n_cam=1: aux1
        self.dimages = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.uint8)
        self.dvalues = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.float32)
        self.actions = None
        self.endeffector_pos = None
        self.joint_angles = np.zeros((self.T, 7))

class TF_rec_converter:
    def __init__(self,sourcedirs,
                   tf_rec_dir = None,
                   gif_dir= None,
                   crop_from_highres = False,
                   split_seq_by = 1):

        self.tfrec_dir = tf_rec_dir
        self.gif_dir = gif_dir
        self.crop_from_highres = crop_from_highres
        self.split_seq_by = split_seq_by

    def gather(self):
        donegif = False

        nopkl_file = 0
        if self.tfrec_dir != None:
            for dirs in sourcedirs:
                if not os.path.exists(dirs):
                    raise ValueError('path {} does not exist!'.format(dirs))
            if not os.path.exists(self.tfrec_dir):
                raise ValueError('path {} does not exist!'.format(self.tfrec_dir))

        # go to first source and get group names:
        groupnames = glob.glob(os.path.join(sourcedirs[0], '*'))
        groupnames = [str.split(n, '/')[-1] for n in groupnames]

        self.src_names = [str.split(n, '/')[-1] for n in sourcedirs]

        traj_list = []
        ntraj_max = 7
        itraj = 0
        tf_start_ind = 0
        for gr in groupnames:  # loop over groups
            gr_dir_main = sourcedirs[0] + '/' + gr

            trajname_l = glob.glob(os.path.join(gr_dir_main, '*'))
            trajname_l = [str.split(n, '/')[-1] for n in trajname_l]

            trajname_ind_l = []  # list of tuples (trajname, ind) where ind is 0,1,2 in range(self.split_seq_by)
            for name in trajname_l:
                for i in range(self.split_seq_by):
                    trajname_ind_l.append((name,i))

            random.shuffle(trajname_ind_l)

            for traj_tuple in trajname_ind_l:  # loop of traj0, traj1,..
                trajname = traj_tuple[0]
                print 'processing {}, seq-part {}'.format(trajname, traj_tuple[1] )

                traj_index = re.match('.*?([0-9]+)$', trajname).group(1)

                self.traj = Trajectory(self.src_names, self.split_seq_by)
                traj_dir_main = gr_dir_main + '/' + trajname

                traj_subpath = '/'.join(str.split(traj_dir_main, '/')[-2:])

                #load actions:
                pkl_file = traj_dir_main + '/joint_angles_traj{}.pkl'.format(traj_index)
                if not os.path.isfile(pkl_file):
                    nopkl_file += 1
                    print 'no pkl file found, file no: ', nopkl_file
                    continue

                actions = cPickle.load(open(pkl_file, "rb"))
                self.traj.actions = actions['actions']
                self.traj.joint_angles = actions['jointangles']
                self.traj.endeffector_pos = actions['endeffector_pos']

                traj_start_ind = traj_tuple[1] * self.traj.T
                traj_end_ind = (traj_tuple[1] + 1) * self.traj.T
                for i_src, src in enumerate(sourcedirs):  # loop over cameras: main, aux1, ..
                    self.traj_dir_src = os.path.join(src, traj_subpath)
                    self.step_from_to(traj_start_ind, traj_end_ind, i_src)

                traj_list.append(self.traj)
                itraj += 1

                if self.tfrec_dir != None:
                    if len(traj_list) == 256:

                        filename = 'traj_{0}_to_{1}' \
                            .format(tf_start_ind, itraj-1)
                        self.save_tf_record(filename, traj_list)
                        tf_start_ind = itraj
                        traj_list = []

                if self.gif_dir != None and not donegif:
                    if itraj == ntraj_max:
                        create_gif.comp_video(traj_list, self.gif_dir)
                        print 'created gif, exiting'
                        done = True

    def step_from_to(self, start, end, i_src):
        ttraj = 0
        for dataind in range(start, end):

            # getting color image:
            if self.crop_from_highres:
                im_filename = self.traj_dir_src + '/images/{0}_full_cropped_im{1}_*'.format(self.src_names[i_src], dataind)
            else:
                im_filename = self.traj_dir_src + '/images/{0}_cropped_im{1}_*.png'.format(self.src_names[i_src], dataind)

            if dataind == start:
                print 'processed from file {}'.format(im_filename)
            if dataind == end - 1:
                print 'processed to file {}'.format(im_filename)

            file = glob.glob(im_filename)
            if len(file) > 1:
                print 'warning: more than 1 image one per time step for {}'.format(im_filename)
            file = file[0]

            if not self.crop_from_highres:
                im = Image.open(file)
                im.load()
                if self.src_names[i_src] == 'aux1':
                    im = im.rotate(180)
                im = np.asarray(im)
            else:
                im = self.crop_and_rot(file, i_src)

            self.traj.images[ttraj, i_src] = im
            # getting depth image:
            file = glob.glob(self.traj_dir_src + '/' + 'depth_images/{0}_cropped_depth_im{1}_*.png'
                             .format(self.src_names[i_src], dataind))
            if len(file) > 1:
                print 'warning: more than 1 depthimage one per time step for {}'.format(self.traj_dir_src + '/' +
                                                                                        'images/{0}_cropped_im{1}_*.png'.format(
                                                                                            self.src_names[i_src], dataind))
            file = file[0]
            im = Image.open(file)
            im.load()
            if self.src_names[i_src] == 'aux1':
                im = im.rotate(180)
            im = np.asarray(im)
            self.traj.dimages[ttraj, i_src] = im

            file = glob.glob(self.traj_dir_src + '/depth_images/{0}_depth_im{1}_*.pkl'.format(self.src_names[i_src], dataind))
            file = file[0]
            self.traj.dvalues[ttraj, i_src] = cPickle.load(open(file, "rb"))
            ttraj += 1


    def crop_and_rot(self, file, i_src):
        img = cv2.imread(file)
        imheight = 64
        imwidht = 64
        if self.src_names[i_src] == 'aux1':
            rowstart = 0
            colstart = 17
            img = cv2.resize(img, (0, 0), fx=1 / 12., fy=1 / 12., interpolation=cv2.INTER_AREA)
        else:
            rowstart = 10
            colstart = 28
            img = cv2.resize(img, (0, 0), fx=1 / 9., fy=1 / 9., interpolation=cv2.INTER_AREA)
        img = img[rowstart:rowstart+imheight, colstart:colstart+imwidht]
        # assert img.shape == (64,64,3)
        img = img[...,::-1]

        if self.src_names[i_src] == 'aux1':
            img = imutils.rotate_bound(img, 180)
            # img = Image.fromarray(img)
            # img.show()
            # pdb.set_trace()

        return img

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

    sourcedirs =["/media/frederik/harddrive/sawyerdata/softmotion/main",
                 "/media/frederik/harddrive/sawyerdata/softmotion/aux1"]

    gif_dir = '/media/frederik/harddrive/sawyerdata/softmotion/exampletraj'
    tf_rec_dir = '/home/frederik/Documents/lsdc/pushing_data/softmotion15/'

    tfrec_converter = TF_rec_converter(sourcedirs,tf_rec_dir, gif_dir,crop_from_highres= True, split_seq_by=2)
    tfrec_converter.gather()


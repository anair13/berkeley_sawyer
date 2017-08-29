import os
import glob
import numpy as np
from PIL import Image
import tensorflow as tf
import cPickle
import imutils   #pip install imutils
import random
import time
from multiprocessing import Pool

def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))
def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))
def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

import pdb
import re

import cv2
import ray
import create_gif

class Trajectory(object):
    def __init__(self, cameranames, split_seq_by = 1):
        self.cameranames = cameranames
        self.T = 30/split_seq_by
        self.n_cam = len(cameranames)
        self.images = np.zeros((self.T, self.n_cam, 64, 64, 3), dtype = np.uint8)  # n_cam=0: main, n_cam=1: aux1
        self.dimages = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.uint8)
        self.dvalues = np.zeros((self.T, self.n_cam, 64, 64), dtype = np.float32)
        self.actions = np.zeros((self.T, 4), dtype = np.float32)
        self.endeffector_pos = np.zeros((self.T, 3), dtype = np.float32)
        self.joint_angles = np.zeros((self.T, 7), dtype = np.float32)

@ray.actor
class TF_rec_converter(object):
    def __init__(self,sourcedirs,
                   tf_rec_dir = None,
                   gif_dir= None,
                   crop_from_highres = False,
                   split_seq_by = 1,
                   startidx = None,
                   endidx = None):

        self.startidx = startidx
        self.endidx = endidx
        self.sourcedirs = sourcedirs
        self.tfrec_dir = tf_rec_dir
        self.gif_dir = gif_dir
        self.crop_from_highres = crop_from_highres
        self.split_seq_by = split_seq_by

        print 'started process with PID:', os.getpid()


    def gather(self):
        print 'PID: {0} started gather going from {1} to {2}'.format(os.getpid(), self.startidx, self.endidx)

        donegif = False
        ierror = 0

        nopkl_file = 0
        if self.tfrec_dir != None:
            for dirs in self.sourcedirs:
                if not os.path.exists(dirs):
                    raise ValueError('path {} does not exist!'.format(dirs))
            if not os.path.exists(self.tfrec_dir):
                raise ValueError('path {} does not exist!'.format(self.tfrec_dir))


        self.src_names = [str.split(n, '/')[-1] for n in sourcedirs]

        traj_list = []
        ntraj_max = 8
        startgrp = self.startidx / 1000
        endgrp = self.endidx / 1000

        trajname_ind_l = []  # list of tuples (trajname, ind) where ind is 0,1,2 in range(self.split_seq_by)
        for gr in range(startgrp, endgrp+1):  # loop over groups
            gr_dir_main = self.sourcedirs[0] + '/traj_group' + str(gr)

            if gr == startgrp:
                trajstart = self.startidx
            else:
                trajstart = gr*1000
            if gr == endgrp:
                trajend = self.endidx
            else:
                trajend = (gr+1)*1000-1

            for i_tra in range(trajstart, trajend+1):
                trajdir = gr_dir_main + "/traj{}".format(i_tra)
                if not os.path.exists(trajdir):
                    print 'file {} not found!'.format(trajdir)
                    continue
                for i in range(self.split_seq_by):
                    trajname_ind_l.append((trajdir, i))

        print 'length trajname_ind_l', len(trajname_ind_l)

        # ###
        # print 'startind', self.startidx
        # for i in range(10):
        #     print trajname_ind_l[i]
        #
        # for i in range(10):
        #     print trajname_ind_l[-i]
        # print 'trajend, ', trajend
        # print 'endind', self.endidx
        #
        # if len(trajname_ind_l) != len(set(trajname_ind_l)):
        #     print 'list has duplicates'
        # ###
        # pdb.set_trace()

        random.shuffle(trajname_ind_l)

        tf_start_ind = self.startidx
        for traj_tuple in trajname_ind_l:  # loop of traj0, traj1,..

            try:
                trajname = traj_tuple[0]
                # print 'processing {}, seq-part {}'.format(trajname, traj_tuple[1] )

                traj_index = re.match('.*?([0-9]+)$', trajname).group(1)
                self.traj = Trajectory(self.src_names, self.split_seq_by)

                traj_subpath = '/'.join(str.split(trajname, '/')[-2:])   #string with only the group and trajectory

                #load actions:
                pkl_file = trajname + '/joint_angles_traj{}.pkl'.format(traj_index)
                if not os.path.isfile(pkl_file):
                    nopkl_file += 1
                    print 'no pkl file found, file no: ', nopkl_file
                    continue

                pkldata = cPickle.load(open(pkl_file, "rb"))
                self.all_actions = pkldata['actions']
                self.all_joint_angles = pkldata['jointangles']
                self.all_endeffector_pos = pkldata['endeffector_pos']

                traj_start_ind = traj_tuple[1] * self.traj.T
                traj_end_ind = (traj_tuple[1] + 1) * self.traj.T
                for i_src, src in enumerate(self.sourcedirs):  # loop over cameras: main, aux1, ..
                    self.traj_dir_src = os.path.join(src, traj_subpath)
                    self.step_from_to(traj_start_ind, traj_end_ind, i_src)

                traj_list.append(self.traj)
                maxlistlen = 256

                if self.tfrec_dir != None:
                    if maxlistlen == len(traj_list):

                        filename = 'traj_{0}_to_{1}' \
                            .format(tf_start_ind,tf_start_ind + maxlistlen-1)
                        self.save_tf_record(filename, traj_list)
                        tf_start_ind += maxlistlen
                        traj_list = []

                if self.gif_dir != None and not donegif:
                    if len(traj_list) == ntraj_max:
                        create_gif.comp_video(traj_list, self.gif_dir)
                        print 'created gif, exiting'
                        donegif = True

                print 'processed {} trajectories'.format(len(traj_list))

            except KeyboardInterrupt:
                print 'exiting after Keyboard interrupt!'
                return
            except:
                print '##############################################3'
                print 'error occured!'
                ierror += 1
                print 'number of errors:', ierror
                continue

        print 'done, {} errors occurred:'.format(ierror)

        return 'done'


    def step_from_to(self, start, end, i_src):
        trajind = 0  # trajind is the index in the target trajectory
        for dataind in range(start, end):  # dataind is the index in the source trajetory

            # get low dimensional data
            self.traj.actions[trajind] = self.all_actions[dataind]
            self.traj.joint_angles[trajind] = self.all_joint_angles[dataind]
            self.traj.endeffector_pos[trajind] = self.all_endeffector_pos[dataind]

            # getting color image:
            if self.crop_from_highres:
                im_filename = self.traj_dir_src + '/images/{0}_full_cropped_im{1}_*'\
                    .format(self.src_names[i_src], dataind)
            else:
                im_filename = self.traj_dir_src + '/images/{0}_cropped_im{1}_*.png'\
                    .format(self.src_names[i_src], dataind)

            # if dataind == start:
            #     print 'processed from file {}'.format(im_filename)
            # if dataind == end - 1:
            #     print 'processed to file {}'.format(im_filename)

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

            self.traj.images[trajind, i_src] = im

            # getting depth image:
            # file = glob.glob(self.traj_dir_src + '/' + '/depth_images/{0}_cropped_depth_im{1}_*.png'
            #                  .format(self.src_names[i_src], dataind))
            # if len(file) > 1:
            #     print 'warning: more than 1 depthimage one per time step for {}'.format(self.traj_dir_src + '/' +
            #                                                                             'images/{0}_cropped_im{1}_*.png'.format(
            #                                                                                 self.src_names[i_src], dataind))
            # file = file[0]
            # im = Image.open(file)
            # im.load()
            # if self.src_names[i_src] == 'aux1':
            #     im = im.rotate(180)
            # im = np.asarray(im)
            # self.traj.dimages[trajind, i_src] = im
            #
            # file = glob.glob(self.traj_dir_src + '/depth_images/{0}_depth_im{1}_*.pkl'.format(self.src_names[i_src], dataind))
            # file = file[0]
            # self.traj.dvalues[trajind, i_src] = cPickle.load(open(file, "rb"))

            trajind += 1

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

                # feature[str(tstep) + '/depth_main'] = _float_feature(traj.dvalues[tstep, 0].flatten().tolist())
                # feature[str(tstep) + '/depth_main'] = _float_feature(traj.dvalues[tstep, 1].flatten().tolist())

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


def get_maxtraj(sourcedirs):
    for dirs in sourcedirs:
        if not os.path.exists(dirs):
            raise ValueError('path {} does not exist!'.format(dirs))

    # go to first source and get group names:
    groupnames = glob.glob(os.path.join(sourcedirs[0], '*'))
    groupnames = [str.split(n, '/')[-1] for n in groupnames]
    gr_ind = []
    for grname in groupnames:
        gr_ind.append(int(re.match('.*?([0-9]+)$', grname).group(1)))
    max_gr = np.max(np.array(gr_ind))

    trajdir = sourcedirs[0] + "/traj_group{}".format(max_gr)
    trajname_l = glob.glob(trajdir +'/*')
    trajname_l = [str.split(n, '/')[-1] for n in trajname_l]
    traj_num = []
    for trname in trajname_l:
        traj_num.append(int(re.match('.*?([0-9]+)$', trname).group(1)))

    max_traj = np.max(np.array(traj_num))
    return max_traj


def start_parallel(source_dirs,tf_rec_dir, gif_dir, crop_from_highres= True, split_seq_by=2):
    ray.init()

    n_traj = get_maxtraj(sourcedirs)
    n_worker = 10
    traj_per_worker = int(n_traj / np.float32(n_worker))
    start_idx = [traj_per_worker * i for i in range(n_worker)]
    end_idx = [traj_per_worker * (i + 1) - 1 for i in range(n_worker)]

    workers = []
    for i in range(n_worker):
        workers.append(TF_rec_converter(sourcedirs,tf_rec_dir, gif_dir, crop_from_highres,split_seq_by,start_idx[i], end_idx[i]))

    id_list = []
    for worker in workers:
        # time.sleep(2)
        id_list.append(worker.gather())

    # blocking call
    res = [ray.get(id) for id in id_list]


def start_parallel_multiprocessing(source_dirs,tf_rec_dir, gif_dir, crop_from_highres= True, split_seq_by=2):
    n_traj = get_maxtraj(source_dirs)

    n_worker = 12
    traj_per_worker = int(n_traj / np.float32(n_worker))
    start_idx = [traj_per_worker * i for i in range(n_worker)]
    end_idx = [traj_per_worker * (i + 1) - 1 for i in range(n_worker)]

    conflist = []
    for i in range(n_worker):
        conflist.append([sourcedirs,tf_rec_dir, gif_dir, crop_from_highres,split_seq_by,start_idx[i], end_idx[i]])

    def worker(conf):
        converter = TF_rec_converter(conf[0], conf[1], conf[2], conf[3], conf[4], conf[5], conf[6])
        converter.gather()

    p = Pool(n_worker)
    p.map(worker, conflist)


if __name__ == "__main__":

    sourcedirs =["/media/frederik/harddrive/sawyerdata/softmotion_0511/main",
                 "/media/frederik/harddrive/sawyerdata/softmotion_0511/aux1"]

    gif_dir = '/media/frederik/harddrive/sawyerdata/softmotion_0511/exampletraj'
    tf_rec_dir = '/media/frederik/harddrive/pushingdata/softmotion30_44k'

    parallel = True

    if parallel:
        start_parallel(sourcedirs,tf_rec_dir, gif_dir,crop_from_highres= True, split_seq_by=1)
    else:

        tfrec_converter = TF_rec_converter(sourcedirs,
                                           tf_rec_dir,
                                           gif_dir,
                                           crop_from_highres= True,
                                           split_seq_by=1, startidx=0,
                                           endidx=get_maxtraj(sourcedirs))
        tfrec_converter.gather()
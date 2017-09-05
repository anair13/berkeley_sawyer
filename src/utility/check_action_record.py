import cPickle as pkl


file = '/home/febert/Documents/sawyer_data/newrecording/traj_group0/traj0/joint_angles_traj0.pkl'

pkldata = pkl.load(open(file, "rb"))

print 'pkldata'



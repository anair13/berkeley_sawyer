import numpy as np
import matplotlib.pyplot as plt
import cPickle


main_file = '/home/guser/sawyer_data/newrecording/main_snapshot_timing.pkl'
main_timing = cPickle.load(open(main_file, "rb"))

aux1_file = '/home/guser/sawyer_data/newrecording/aux1_snapshot_timing.pkl'
aux1_timing = cPickle.load(open(aux1_file, "rb"))


tsave_main = np.array(main_timing['t_finish_save'])
t0 = tsave_main[0]
tsave_main = tsave_main - t0

tsave_aux1 = np.array(aux1_timing['t_finish_save'])
tsave_aux1 = tsave_aux1 - t0
treq_aux1 = np.array(aux1_timing['t_get_request'])
treq_aux1 = treq_aux1 - t0


y = .001* np.ones(tsave_main.shape[0])
plt.scatter(tsave_main, y, c= 'r', alpha=0.5)

y = .002* np.ones(tsave_aux1.shape[0])
plt.scatter(tsave_aux1, y, c= 'b' ,alpha=0.5)

y = .003* np.ones(treq_aux1.shape[0])
plt.scatter(treq_aux1, y, c= 'b', alpha=0.5)

plt.show()
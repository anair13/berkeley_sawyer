import numpy as np


def quat_to_zangle(quat):
    """
    :param quat: quaternion with only
    :return: zangle in rad
    """
    phi = np.arctan2( 2 *(quat[0 ] *quat[1] + quat[2 ] *quat[3]), 1 - 2 *(quat[1] ** 2 + quat[2] ** 2))
    return np.array([phi])


def zangle_to_quat(zangle):
    quat = np.array((  # downward and turn a little
        np.cos(zangle / 2),
        np.sin(zangle / 2),
        0.0,
        0.0))

    return quat


alpha = np.pi*2.

quat = zangle_to_quat(alpha)
print 'quat', quat

alpha1 = quat_to_zangle(quat)
print 'alpha1', alpha1



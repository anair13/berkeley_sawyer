import os
import pdb
import socket


def delete_local(path, remote_machine= None):

    if remote_machine ==None:
        os.system("rm -r *")
        cmd = "cd {} && rm -r *"

    print 'performing cmd: {}'.format(cmd)
    pdb.set_trace()
    os.system("rm -r *")

def delete_recording(path):

    delete_local(path)
    pdb.set_trace()
    delete_local(path, remote_machine='k1')

if __name__ == '__main__':
    delete_recording("/home/guser/sawyer_data/testrecording")
from NED import NED
import numpy as np


PATH = '/media/eric-uib/HardDisk/uib/map_estimation/datasets/HR_pn2'
FILE_IN = '/eks_path.csv'
FILE_OUT = '/eks_path_2.csv'
HEADER = ['stamp', 'seq', 'north', 'east', 'depth', 'q1' ,'q2' ,'q3' ,'q4']
NED_ORIGIN = [39.5244816667, 2.55069]


if __name__ == '__main__':
    ned = NED(NED_ORIGIN[0], NED_ORIGIN[1], 0.0)

    mat = np.loadtxt(PATH + FILE_IN, delimiter=',')

    for i in range(mat.shape[0]):
        mat[i,2], mat[i,3], _ = ned.ned2geodetic([mat[i,2], mat[i,3], 0.0])

    np.savetxt(PATH + FILE_OUT, mat, delimiter=',')


import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse


def pose2tum(folder_name):
    
    if folder_name[-1] == "/": folder_name[:-1]
    
    filenmale = f"{folder_name}/pose.txt"

    data = np.loadtxt(filenmale)
    data = data[data[:, 0].argsort()]
    
    start_frame = 1
    start_trans = None
    start_rot = None

    with open(f"{folder_name}/pose_tum.txt", "w", encoding="utf-8") as fil:
        for i, l in enumerate(data, start=1):
            t = l[4:]
            a = R.from_rotvec(l[1:4]).as_quat()
        
            mat = R.from_quat(a).as_matrix().T

            if i == start_frame:
                start_rot = mat
                start_trans = t

            rot = start_rot.T @ mat
            trans = start_rot.T @ (t - start_trans)
            trans = trans.T
            rot = R.from_matrix(rot).as_quat()
            fil.write(f"{i} {trans[0]} {trans[1]} {trans[2]} {rot[0]} {rot[1]} {rot[2]} {rot[3]}\n")

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=str, default="", help="dir to find pose.txt", required=True)
    args = parser.parse_args()
    
    pose2tum(args.folder)
import os
import argparse
import json

def senitize(path):
    if path[-1] == "/": path = path[:-1]
    return path

def create_calibarion_file_buss(dir_name):
    
    camera = "PINHOLE_RADIAL_TANGENTIAL"
    w = 1920
    h = 1080

    focal_length = 1811.0354
    aspect_ratio = 1807.77084/1811.0354
    principal_point = [962.108902, 531.754439]
    k1, k2, p1, p2, k3 = 2.57006987e-01, -1.68927129e+00, 1.50087054e-03, 1.07188929e-04, 3.47164291e+00

    new_cal = {'priors': []}
    arr = []
    files = os.listdir(f"{dir_name}/images")
    for img in files:
        new_cam = {"CameraIntrinsicsPrior": {}}
        new_cam["CameraIntrinsicsPrior"]["image_name"] = img
        new_cam["CameraIntrinsicsPrior"]["focal_length"] = focal_length
        new_cam["CameraIntrinsicsPrior"]["principal_point"] = principal_point 
        new_cam["CameraIntrinsicsPrior"]["aspect_ratio"] = aspect_ratio 

        new_cam["CameraIntrinsicsPrior"]["radial_distortion_1"] = k1
        new_cam["CameraIntrinsicsPrior"]["radial_distortion_2"] = k2
        new_cam["CameraIntrinsicsPrior"]["radial_distortion_3"] = k3
        new_cam["CameraIntrinsicsPrior"]["tangential_distortion_1"] = p1
        new_cam["CameraIntrinsicsPrior"]["tangential_distortion_2"] = p2
        new_cam["CameraIntrinsicsPrior"]["camera_intrinsics_type"] = camera 

        new_cam["CameraIntrinsicsPrior"]["width"] = w 
        new_cam["CameraIntrinsicsPrior"]["height"] = h
        arr.append(new_cam) 

    new_cal['priors'] = arr

    with open(f"{dir_name}/calibration.txt", "w") as outfile:
        json.dump(new_cal, outfile)

def create_calibarion_file_kitti(dir_name):
    
    camera = "PINHOLE"
    w = 1241
    h = 376

    focal_length = 718.856
    principal_point = [607.1928, 185.2157]

    new_cal = {'priors': []}
    arr = []
    files = os.listdir(f"{dir_name}/images")
    for img in files:
        new_cam = {"CameraIntrinsicsPrior": {}}
        new_cam["CameraIntrinsicsPrior"]["image_name"] = img
        new_cam["CameraIntrinsicsPrior"]["focal_length"] = focal_length
        new_cam["CameraIntrinsicsPrior"]["principal_point"] = principal_point 
        new_cam["CameraIntrinsicsPrior"]["camera_intrinsics_type"] = camera 

        new_cam["CameraIntrinsicsPrior"]["width"] = w 
        new_cam["CameraIntrinsicsPrior"]["height"] = h
        arr.append(new_cam) 

    new_cal['priors'] = arr

    with open(f"{dir_name}/calibration.txt", "w") as outfile:
        json.dump(new_cal, outfile)
    
     
def create_calibarion_file(dir_name, dataset):
    
    if dataset == "buss":
        create_calibarion_file_buss(dir_name)
    elif dataset == "kitti":
        create_calibarion_file_kitti(dir_name)
    else:
        print(f"not implemented {dataset}")
        exit(1)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-r', type=str, required=True, help="root of all dataset")
    parser.add_argument('-d', type=str, required=True, help="dataset name")
    parser.add_argument('-s', type=str, required=True, help="name of sequence")
    parser.add_argument('-t', type=str, required=True, help="root of experiment")
    parser.add_argument('-e', type=str, required=True, help="experiment name")
    args = parser.parse_args()
    
    r = senitize(args.r)
    d = senitize(args.d)
    s = senitize(args.s)
    t = senitize(args.t)
    e = senitize(args.e)
    
    print(args)
    
    data_dir = f"{r}/{d}/{s}"
    imgaes = os.listdir(os.path.join(data_dir, "images"))
    img_count = len(imgaes)
    imgext = os.path.splitext(imgaes[0])[1]
    
    with open('generic_flag.txt', 'r') as file:
        file_content = file.read()
    
    file_content = file_content.replace("{{ROOT}}",     r)
    file_content = file_content.replace("{{DATASET}}",  d)
    file_content = file_content.replace("{{SEQUENCE}}", s)
    file_content = file_content.replace("{{ROOT_EXP}}", t)
    file_content = file_content.replace("{{EXP_NAME}}", e)
    file_content = file_content.replace("{{EXT}}", imgext)
    
    
    #save file to backup the exp
    expriment_dir = f"{t}/{d}/{e}/{s}"
    os.makedirs(expriment_dir, exist_ok=True)
    with open(f'{expriment_dir}/flag.txt', 'w') as file:
        file.write(file_content)
        
        
    # write flag file to loction to run theia
    # this will be overwritten
    with open(f'{data_dir}/flag.txt', 'w') as file:
        file.write(file_content)
        
    if not os.path.isfile(f'{data_dir}/calibration.txt'):
        create_calibarion_file(data_dir, d)
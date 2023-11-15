import os
import json

def create_sintel_calibration(i_frames):

    camera = "PINHOLE_RADIAL_TANGENTIAL"
    width = 1920
    height = 1080
    focal_length = 1811.0354
    principal_point = [962.108902,
                       531.754439
                       ]

    new_cal = {'priors': []}
    arr = []

    for i in i_frames:
        new_cam = {"CameraIntrinsicsPrior": {}}
        new_cam["CameraIntrinsicsPrior"]["image_name"] = f"{i:04d}.jpg"
        new_cam["CameraIntrinsicsPrior"]["focal_length"] = focal_length
        new_cam["CameraIntrinsicsPrior"]["aspect_ratio"] = 0.9981974068535601
        new_cam["CameraIntrinsicsPrior"]["principal_point"] = principal_point
        new_cam["CameraIntrinsicsPrior"]["camera_intrinsics_type"] = camera
        new_cam["CameraIntrinsicsPrior"]["width"] = width
        new_cam["CameraIntrinsicsPrior"]["height"] = height
        new_cam["CameraIntrinsicsPrior"]["radial_distortion_1"] = 0.257006987
        new_cam["CameraIntrinsicsPrior"]["radial_distortion_2"] = -1.68927129
        new_cam["CameraIntrinsicsPrior"]["radial_distortion_3"] = 3.47164291
        new_cam["CameraIntrinsicsPrior"]["tangential_distortion_1"] = 0.00150087054
        new_cam["CameraIntrinsicsPrior"]["tangential_distortion_2"] = 0.000107188929
        arr.append(new_cam)

    new_cal['priors'] = arr

    with open(f"cam_cal.txt", "w") as outfile:
        json.dump(new_cal, outfile)


if __name__ == "__main__":
    i_frames = range(1, 301 + 1) # range(1, 50 + 1)  # [1, 10]
    create_sintel_calibration(i_frames)

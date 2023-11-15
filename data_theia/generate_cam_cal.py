import os
import json

def create_sintel_calibration(i_frames):

    camera = "PINHOLE"
    width = 1024
    height = 436
    focal_length = 896
    principal_point = [511.5, 217.5]

    new_cal = {'priors': []}
    arr = []

    for i in i_frames:
        new_cam = {"CameraIntrinsicsPrior": {}}
        new_cam["CameraIntrinsicsPrior"]["image_name"] = f"frame_{i:04d}.png"
        new_cam["CameraIntrinsicsPrior"]["focal_length"] = focal_length
        new_cam["CameraIntrinsicsPrior"]["principal_point"] = principal_point
        new_cam["CameraIntrinsicsPrior"]["camera_intrinsics_type"] = camera
        new_cam["CameraIntrinsicsPrior"]["width"] = width
        new_cam["CameraIntrinsicsPrior"]["height"] = height
        arr.append(new_cam)

    new_cal['priors'] = arr

    with open(f"cam_cal.txt", "w") as outfile:
        json.dump(new_cal, outfile)


if __name__ == "__main__":
    i_frames = range(1, 50 + 1)  # [1, 10]
    create_sintel_calibration(i_frames)

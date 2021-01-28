import numpy as np
import cv2
import os
from matplotlib import pyplot as pyplot
from perception import (ColorImage, RgbdImage)
from perception import DepthImage
from perception import BinaryImage
from perception import CameraIntrinsics
import json
from autolab_core import YamlConfig, Logger
from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
import klampt.model.sensing

def create_segmentaton_Mask(depth_array, camera_intr):
    depth_im = DepthImage(depth_array, frame=camera_intr.frame)
    maximum = np.max(depth_array)
    minimum = np.min(depth_array)
    mean = np.mean(depth_array)
    for i in range(len(depth_array)):
        if depth_array[i] < minimum + (mean - minimum)/50:
            depth_array[i] = 0
        else:
            depth_array[i] = 255
    return BinaryImage(depth_array, frame = camera_intr.frame)
    segmentation_mask = depth_im.invalid_pixel_mask().inverse()
    return segmentation_mask.mask_binary(segmentation_mask)

def create_camera_intrinsic(frame, fx, fy=None, cx=0.0, cy=0.0, skew=0.0, height=None, width=None):
    camera_intrinsics = CameraIntrinsics(frame, fx, fy, cx, cy, skew, height, width)
    camera_intrinsics.save("cintrinsics.intr")
    return camera_intrinsics

def grasping_wrapper(depth_array, segmentation_mask, camera_intrinsic, config, model_path):
    # get the rgdb_im
    depth_im = DepthImage(depth_array, frame=camera_intrinsic.frame)
    color_im = ColorImage(np.zeros([depth_im.height, depth_im.width,
                                    3]).astype(np.uint8),
                          frame=camera_intrinsic.frame)
    inpaint_rescale_factor = config["inpaint_rescale_factor"]
    depth_im = depth_im.inpaint(rescale_factor=inpaint_rescale_factor)
    rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
    state = RgbdImageState(rgbd_im, camera_intrinsic, segmask= segmentation_mask)


    policy_config = config["policy"]
    if "gqcnn_model" in policy_config["metric"]:
        policy_config["metric"]["gqcnn_model"] = model_path
        if not os.path.isabs(policy_config["metric"]["gqcnn_model"]):
            policy_config["metric"]["gqcnn_model"] = os.path.join(
                os.path.dirname(os.path.realpath(__file__)),
                policy_config["metric"]["gqcnn_model"])

    policy_config["metric"]["fully_conv_gqcnn_config"]["im_height"] = depth_image.shape[0]
    policy_config["metric"]["fully_conv_gqcnn_config"]["im_width"] = depth_image.shape[1]
    policy = FullyConvolutionalGraspingPolicyParallelJaw(policy_config)

    action = policy(state)
    return action.grasp # this contains center ang depth width

if __name__ == "__main__":
    depth_image = np.load("data/depth_0.npy")
    # segmentation_mask = BinaryImage.open("data/segmask_0.png")
    camera_intrinsic = CameraIntrinsics.load("data/phoxi.intr")
    # config = YamlConfig("data/fc_gqcnn_pj.yaml")
    # model_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
    #                              "data")
    # model_path = os.path.join(model_dir, "FC-GQCNN-4.0-PJ")
    # grasp = grasping_wrapper(depth_image, segmentation_mask, camera_intrinsic, config, model_path)
    # print
    # print(grasp.center, grasp.angle, grasp.width)
    segmentation_mask = create_segmentaton_Mask(depth_image, camera_intrinsic)
    segmentation_mask.save("a.png")

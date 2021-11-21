# ---------------------------------------------------------------------
# Exercises from lesson 2 (object detection)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import open3d as o3d
import math
import numpy as np
import zlib
from easydict import EasyDict as edict

import matplotlib
#matplotlib.use('wxagg') # change backend so that figure maximizing works on Mac as well     
import matplotlib.pyplot as plt

# Exercise C2-4-6 : Plotting the precision-recall curve
def plot_precision_recall(): 

    # Please note: this function assumes that you have pre-computed the precions/recall value pairs from the test sequence
    #              by subsequently setting the variable configs.conf_thresh to the values 0.1 ... 0.9 and noted down the results.
    
    # Please create a 2d scatter plot of all precision/recall pairs 
    pass


# Exercise C2-3-4 : Compute precision and recall
def compute_precision_recall(det_performance_all, conf_thresh=0.5):

    if len(det_performance_all)==0 :
        print("no detections for conf_thresh = " + str(conf_thresh))
        return
    
    # extract the total number of positives, true positives, false negatives and false positives
    # format of det_performance_all is [ious, center_devs, pos_negs]

    #print("TP = " + str(true_positives) + ", FP = " + str(false_positives) + ", FN = " + str(false_negatives))
    
    # compute precision
    
    # compute recall 

    #print("precision = " + str(precision) + ", recall = " + str(recall) + ", conf_thres = " + str(conf_thresh) + "\n")    
    



# Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(lidar_pcl, configs, vis=True):

    # compute bev-map discretization by dividing x-range by the bev-image height
    # positive-x pointing up and configs.bev_height is the number of pixels in height direction
    # bev_discret is in meters per pixel
    bev_discret = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height

    # create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates
    # i = np.floor(x); such that i <= x
    # ex: (1) floor(-1.6) = -2.0,  (2) floor(1.5) = 1.0
    lidar_pcl_copy = np.copy(lidar_pcl)
    lidar_pcl_copy[:, 0] = np.int_(np.floor(lidar_pcl_copy[:, 0] / bev_discret))

    # transform all metrix y-coordinates as well but center the foward-facing x-axis on the middle of the image
    lidar_pcl_copy[:, 1] = np.int_(np.floor(lidar_pcl_copy[:, 1] / bev_discret) + ((configs.bev_width + 1) / 2))

    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl_copy[:, 2] = lidar_pcl_copy[:, 2] - configs.lim_z[0]

    # re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then by decreasing height
    height_mask_idx = np.lexsort((-lidar_pcl_copy[:, 2], lidar_pcl_copy[:, 1], lidar_pcl_copy[:, 0]))
    lidar_pcl_height = lidar_pcl_copy[height_mask_idx]

    # extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    _ , height_mask_idx_unique = np.unique(lidar_pcl_height[:, 0:2], return_index=True, axis=0)
    lidar_pcl_height = lidar_pcl_height[height_mask_idx_unique]

    # assign the height value of each unique entry in lidar_top_pcl to the height map and 
    # make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    height_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    x_idx = np.int_(lidar_pcl_height[:, 0])
    y_idx = np.int_(lidar_pcl_height[:, 1])
    normalize_flt = float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    height_map[x_idx, y_idx] = lidar_pcl_height[:, 2] / normalize_flt

    # sort points such that in case of identical BEV grid coordinates, the points in each grid cell are arranged based on their intensity
    lidar_pcl_copy[lidar_pcl_copy[:,3] > 1.0, 3] = 1.0 # limit the intensity to 1.0
    intensity_mask_idx = np.lexsort((-lidar_pcl_copy[:, 3], lidar_pcl_copy[:, 1], lidar_pcl_copy[:, 0]))
    lidar_pcl_intensity = lidar_pcl_copy[intensity_mask_idx]

    # only keep one point per grid cell
    _ , intensity_mask_idx_unique = np.unique(lidar_pcl_intensity[:, 0:2], return_index=True, axis=0)
    lidar_pcl_intensity = lidar_pcl_intensity[intensity_mask_idx_unique]

    # create the intensity map
    intensity_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    x_idx = np.int_(lidar_pcl_intensity[:, 0])
    y_idx = np.int_(lidar_pcl_intensity[:, 1])
    normalize_flt = (np.amax(lidar_pcl_intensity[:, 3]) - np.amin(lidar_pcl_intensity[:, 3]))
    intensity_map[x_idx, y_idx] = lidar_pcl_intensity[:, 3] / normalize_flt

    #visualize intensity map
    # if vis:
    #    img_intensity = intensity_map * 256
    #    img_intensity = img_intensity.astype(np.uint8)
    #    while (1):
    #        cv2.imshow('img_intensity', img_intensity)
    #        if cv2.waitKey(10) & 0xFF == 27:
    #            break
    #    cv2.destroyAllWindows()

    #visualize height map
    # if vis:
    #    img_height = height_map * 256
    #    img_height = img_height.astype(np.uint8)
    #    while (1):
    #        cv2.imshow('img_height', img_height)
    #        if cv2.waitKey(10) & 0xFF == 27:
    #            break
    #    cv2.destroyAllWindows()

    pass

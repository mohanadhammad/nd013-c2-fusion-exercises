# ---------------------------------------------------------------------
# Exercises from lesson 1 (lidar)
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
import numpy as np
import zlib

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2

def load_range_image(frame, lidar_name):
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0] # get laser data structure from frame
    ri = []
    if len(lidar.ri_return1.range_image_compressed) > 0: # use first response
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    return ri

# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):

    print("Exercise C1-5-5")
    # extract range image from frame
    ri = load_range_image(frame, lidar_name)
    ri[ri<0] = 0.0

    # map value range to 8bit
    ri_intentsity = ri[:,:,1]
    ri_intentsity_min = np.amin(ri_intentsity)
    ri_intentsity_max = np.amax(ri_intentsity)
    contrast_adjust = ri_intentsity_max / 2.

    ri_intentsity *= (contrast_adjust * 255) / (ri_intentsity_max - ri_intentsity_min)
    img_intensity = ri_intentsity.astype(np.uint8)

    # focus on +/- 45° around the image center
    deg45 = img_intensity.shape[1] // 8
    img_center = img_intensity.shape[1] // 2
    img_intensity = img_intensity[:,img_center-deg45:img_center+deg45]

    print('max. val = ' + str(round(np.amax(img_intensity[:,:]),2)))
    print('min. val = ' + str(round(np.amin(img_intensity[:,:]),2)))

    cv2.imshow('img_intensity', img_intensity)
    cv2.waitKey(0)


# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame, lidar_name):

    print("Exercise C1-5-2")
    # load range image
    lidar = [obj for obj in frame.lasers if obj.name == lidar_name][0]
    if (len(lidar.ri_return1.range_image_compressed) > 0):
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(lidar.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
        print(ri.shape)
        
    # compute vertical field-of-view from lidar calibration
    lidar_calib = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]
    vfov_rad = lidar_calib.beam_inclination_max - lidar_calib.beam_inclination_min
    vfov_deg = vfov_rad * 180. / np.pi
    print("vfov_rad = %f" % vfov_rad)
    print("vfov_deg = %f" % vfov_deg)

    # compute pitch resolution and convert it to angular minutes
    pitch_res_rad = vfov_rad / ri.shape[0]
    pitch_res_deg = pitch_res_rad * 180. / np.pi
    pitch_res_min = pitch_res_deg * 60.
    print("pitch_res_deg = %f" % pitch_res_deg)
    print("pitch_res_min = %f" % pitch_res_min)


# Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(frame):

    print("Exercise C1-3-1")    

    # find out the number of labeled vehicles in the given frame
    # Hint: inspect the data structure frame.laser_labels
    num_vehicles = 0
    for laser_label in frame.laser_labels:
        if laser_label.type == laser_label.TYPE_VEHICLE:
            num_vehicles += 1
            
    print("number of labeled vehicles in current frame = " + str(num_vehicles))


def  print_no_of_laser_leds(frame, lidar_name):
    counts = 0
    for laser_calibration in frame.context.laser_calibrations:
        if laser_calibration.name == lidar_name:
            print(len(laser_calibration.beam_inclinations))
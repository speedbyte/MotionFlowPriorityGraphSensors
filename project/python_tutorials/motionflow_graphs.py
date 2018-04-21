#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import yaml
import io
import math

import pandas as pd
import cv2
import numpy
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy
from scipy.interpolate import griddata
import matplotlib as mp
from math import pi

import random

dataset = "vires"
scenario = "two"
file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" +scenario + "/values.yml"

#file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

environment_list = ["none","light_snow_", "mild_snow_", "heavy_snow_"]#night
environment_list = ["none",]

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

OUTLIER = 100000
SCALE = 1

def histogramm():

    with open("hist") as test:
        hist = []
        for line in test:
            hist.append(line.rstrip())

    xbuf = []
    ybuf = []
    y = []

    for line in hist:
        type = line.split(" ")
        xbuf.append(float(type[0]))
        l = type[2]
        for i in range(0,int(l[:-4])):
            ybuf.append(float(type[0]))

    print(ybuf)

    fig1 = plt.figure()
    plt.xlabel("Displacement")
    plt.ylabel("Counter", )

    y = np.asarray(ybuf)
    bins = xbuf # use for small bars
    bins = range(-10,10)
    print bins

    x,y,_ = plt.hist(y.astype('float'),bins=bins, align='left', rwidth=0.5,)

    maxIndex = np.argmax(x)
    print maxIndex

    plt.bar(bins[0]+maxIndex,max(x),color='red',width=0.5)

    plt.show()
    fig1.savefig(output_folder + 'histogramm.png', dpi= 200)
    plt.close('all')


def motionflow_pixelgraphs_no_noise(): ##done

    offset=1
    offset_index=0

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig2 = plt.figure()
    #plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig2.add_subplot(111)

    #shapeplot0.set_title('shape_pointsframe_skip1_dataprocessing_0')

    shapeplot0.set_xlabel('frame_count')
    shapeplot0.set_ylabel('no_noise_good_pixels/no_noise_total_pixels')

    shapeplot0.set_ylim([0, 1])
    legend = shapeplot0.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))


    list_of_shape_metrics = [
        "shape_pointsframe_skip1_dataprocessing_0_generated",
        "shape_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_none_",
    ]

    color_of_shape_metrics = ["red", "green", "yellow", "black"]

    #assert(len(list_of_shape_metrics)/1 == len(color_of_shape_metrics))

    num=0

    for no_of_metrics in range(1): #generated

        for x in range(1):

            shape_points = yaml_load[list_of_shape_metrics[offset_index+x]]
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0_gt, y0_gt = data.T
                y0_gt = x0_gt/y0_gt

            x0 = np.arange(0.0, 4.0, 1)

    y0_mean_list = list()
    y1_mean_list = list()
    y2_mean_list = list()
    y3_mean_list = list()

    for no_of_metrics in range(1): # none

        y0_mean = 0
        y1_mean = 0
        y2_mean = 0
        y3_mean = 0

        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics[offset_index+1+x]]
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0, y0 = data.T
                y0 = x0/y0
            if ( x == 1):
                x1, y1 = data.T
                y1 = x1/y1
            if ( x == 2):
                x2, y2 = data.T
                y2 = x2/y2
            if ( x == 3):
                x3, y3 = data.T
                y3 = x3/y3

            x0 = np.arange(0.0, len(shape_points)-1, 1)

        for n,i in enumerate(y0):
            y0_mean=y0_mean+i
        for n,i in enumerate(y1):
            y1_mean=y1_mean+i
        for n,i in enumerate(y2):
            y2_mean=y2_mean+i
        for n,i in enumerate(y3):
            y3_mean=y3_mean+i
        y0_mean = y0_mean/(n+1)
        y1_mean = y1_mean/(n+1)
        y2_mean = y2_mean/(n+1)
        y3_mean = y3_mean/(n+1)
        y0_mean_list.append(y0_mean)
        y1_mean_list.append(y1_mean)
        y2_mean_list.append(y2_mean)
        y3_mean_list.append(y3_mean)

        shapeplot0.plot(x0, y0, 'ko-', lw=1, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot0.plot(x0, y1, 'ko-', lw=1, color=color_of_shape_metrics[1+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot0.plot(x0, y2, 'ko-', lw=1, color=color_of_shape_metrics[2+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot0.plot(x0, y3, 'ko-', lw=1, color=color_of_shape_metrics[3+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

        print "Table 1 " + environment_list[no_of_metrics]
        print y0_mean_list[no_of_metrics]
        print y1_mean_list[no_of_metrics]
        print y2_mean_list[no_of_metrics]
        print y3_mean_list[no_of_metrics]

    #fig2.set_size_inches(18.5, 10.5)
    fig2.savefig(output_folder + 'pixel_robustness_optical_flow.png', bbox_inches='tight',dpi=200)
#    plt.close('all')


def motionflow_pixelgraphs_noise():

    offset=1
    offset_index=0

    print "Start Noise "
    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig0 = plt.figure()
    #plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig1 = plt.figure()
    #plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig2 = plt.figure()
    #plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig3 = plt.figure()
    #plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig0.add_subplot(111)
    shapeplot1 = fig1.add_subplot(111)
    shapeplot2 = fig2.add_subplot(111)
    shapeplot3 = fig3.add_subplot(111)

    #shapeplot0.set_title('shape_pointsframe_skip1_dataprocessing_0')
    #shapeplot1.set_title('shape_pointsframe_skip1_dataprocessing_1')
    #shapeplot2.set_title('shape_pointsframe_skip1_dataprocessing_2')
    #shapeplot3.set_title('shape_pointsframe_skip1_dataprocessing_3')

    shapeplot0.set_xlabel('frame_count')
    shapeplot0.set_ylabel('noise_good_pixels/no_noise_total_pixels')
    shapeplot1.set_xlabel('frame_count')
    shapeplot1.set_ylabel('noise_good_pixels/no_noise_total_pixels')
    shapeplot2.set_xlabel('frame_count')
    shapeplot2.set_ylabel('noise_good_pixels/no_noise_total_pixels')
    shapeplot3.set_xlabel('frame_count')
    shapeplot3.set_ylabel('noise_good_pixels/no_noise_total_pixels')

    shapeplot0.set_ylim([0, 1])
    shapeplot1.set_ylim([0, 1])
    shapeplot2.set_ylim([0, 1])
    shapeplot3.set_ylim([0, 1])
    legend = shapeplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_shape_metrics = [
        "shape_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_none_",

        "shape_pointsframe_skip1_dataprocessing_0results_FB_light_snow_",
        "shape_pointsframe_skip1_dataprocessing_1results_FB_light_snow_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_light_snow_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_light_snow_",

        "shape_pointsframe_skip1_dataprocessing_0results_FB_mild_snow_",
        "shape_pointsframe_skip1_dataprocessing_1results_FB_mild_snow_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_mild_snow_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_mild_snow_",

        "shape_pointsframe_skip1_dataprocessing_0results_FB_heavy_snow_",
        "shape_pointsframe_skip1_dataprocessing_1results_FB_heavy_snow_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_heavy_snow_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_heavy_snow_",

    ]

    color_of_shape_metrics = ["red", "green", "yellow", "black"]

    assert(len(list_of_shape_metrics)/4 == len(color_of_shape_metrics))
    print "Länge", len(list_of_shape_metrics)/4

    num=0
    y0_mean_list = list()
    y1_mean_list = list()
    y2_mean_list = list()
    y3_mean_list = list()

    for no_of_metrics in range(len(environment_list)): #none, night

        y0_mean = 0
        y1_mean = 0
        y2_mean = 0
        y3_mean = 0

        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics[offset_index+x]]
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0, y0 = data.T
                y0 = x0/y0
            if ( x == 1):
                x1, y1 = data.T
                y1 = x1/y1
            if ( x == 2):
                x2, y2 = data.T
                y2 = x2/y2
            if ( x == 3):
                x3, y3 = data.T
                y3 = x3/y3

            x0 = np.arange(0.0, len(shape_points)-1, 1)

        for n,i in enumerate(y0):
            y0_mean=y0_mean+i
        for n,i in enumerate(y1):
            y1_mean=y1_mean+i
        for n,i in enumerate(y2):
            y2_mean=y2_mean+i
        for n,i in enumerate(y3):
            y3_mean=y3_mean+i
        y0_mean = y0_mean/(n+1)
        y1_mean = y1_mean/(n+1)
        y2_mean = y2_mean/(n+1)
        y3_mean = y3_mean/(n+1)
        y0_mean_list.append(y0_mean)
        y1_mean_list.append(y1_mean)
        y2_mean_list.append(y2_mean)
        y3_mean_list.append(y3_mean)

        shapeplot0.plot(x0, y0, 'ko-', lw=1, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))


        shapeplot1.plot(x0, y1, 'ko-', lw=1, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot2.plot(x0, y2, 'ko-', lw=1, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot3.plot(x0, y3, 'ko-', lw=1, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

        print "Table 2 " + environment_list[no_of_metrics]
        print y0_mean_list[no_of_metrics]
        print y1_mean_list[no_of_metrics]
        print y2_mean_list[no_of_metrics]
        print y3_mean_list[no_of_metrics]

# fig0.set_size_inches(18.5, 10.5)
    fig0.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_0',bbox_inches='tight', dpi=200)
   # fig1.set_size_inches(18.5, 10.5)
    fig1.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
   # fig2.set_size_inches(18.5, 10.5)
    fig2.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
   # fig3.set_size_inches(18.5, 10.5)
    fig3.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_3', bbox_inches='tight',dpi=200)

 #   plt.close("all")

def motionflow_vectorgraphs_no_noise():


    offset=1
    offset_index=0

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig1 = plt.figure()
    fig2 = plt.figure()
    #plt.suptitle("Deviation of collision points between ground truth and scenes without noise")

    deviationplot0 = fig1.add_subplot(111)
    collisionplot0 = fig2.add_subplot(111)

    #deviationplot0.set_title('collision_pointsframe_skip1_dataprocessing_0')

    deviationplot0.set_xlabel('frame_count')
    deviationplot0.set_ylabel('deviation [no_noise_points-groundtruth_points]')

    collisionplot0.set_xlabel('X')
    collisionplot0.set_ylabel('Y')

    legend = deviationplot0.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [
        "collision_pointsframe_skip1_dataprocessing_0_generated",
        "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",
    ]

    color_of_collision_metrics = ["blue", "red", "green", "yellow", "black"]

    assert(len(list_of_collision_metrics) == len(color_of_collision_metrics))

    num=0

    delete_point_array = np.array([-65535])
    for no_of_metrics in range(1): # generated

        dev0_gt_mean = 0

        for x in range(1):

            collision_points = yaml_load[list_of_collision_metrics[offset_index+x]]
            print "collision index no noise" , offset_index+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0_gt, y0_gt = data.T
                x0_gt = np.setdiff1d(x0_gt, delete_point_array)
                y0_gt = np.setdiff1d(y0_gt, delete_point_array)
                dev0_gt = numpy.sqrt((x0_gt - x0_gt) ** 2 + (y0_gt - y0_gt) ** 2)

        dev0_gt_mean_list = list()

        for n,i in enumerate(dev0_gt):
            dev0_gt_mean=(dev0_gt_mean+dev0_gt[n])

        dev0_gt_mean = dev0_gt_mean/(n+1)
        dev0_gt_mean_list.append(dev0_gt_mean)

    dev0_mean_list = list()
    dev1_mean_list = list()
    dev2_mean_list = list()
    dev3_mean_list = list()

    for no_of_metrics in range(1): # none

        dev0_mean = 0
        dev1_mean = 0
        dev2_mean = 0
        dev3_mean = 0

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index+1+x]]
            print "collision index no noise dataprocessing" , offset_index+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                x0 = np.setdiff1d(x0, delete_point_array)
                y0 = np.setdiff1d(y0, delete_point_array)
                dev0 = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                x1 = np.setdiff1d(x1, delete_point_array)
                y1 = np.setdiff1d(y1, delete_point_array)
                dev1 = numpy.sqrt((x0_gt - x1) ** 2 + (y0_gt - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                x2 = np.setdiff1d(x2, delete_point_array)
                y2 = np.setdiff1d(y2, delete_point_array)
                dev2 = numpy.sqrt((x0_gt - x2) ** 2 + (y0_gt - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                x3 = np.setdiff1d(x3, delete_point_array)
                y3 = np.setdiff1d(y3, delete_point_array)
                dev3 = numpy.sqrt((x0_gt - x3) ** 2 + (y0_gt - y3) ** 2)

            frame_number = np.arange(0.0, len(x0_gt), 1)

        for n,i in enumerate(dev0):
            if ( abs(i) > OUTLIER):
                dev0[n] = dev0[n-1]
                if ( n == 0 ):
                    dev0[n] = 0
            dev0_mean=(dev0_mean+dev0[n])
            #dev0_mean=(dev0_mean+dev0[n])/2
            #dev0[n] = dev0_mean
        for n,i in enumerate(dev1):
            if ( i > OUTLIER):
                dev1[n] = dev1[n-1]
            dev1_mean=(dev1_mean+dev1[n])
            #dev1_mean=(dev1_mean+dev1[n])/2
            #dev1[n] = dev1_mean
        for n,i in enumerate(dev2):
            if ( i > OUTLIER):
                dev2[n] = dev2[n-1]
            dev2_mean=(dev2_mean+dev2[n])
            #dev2_mean=(dev2_mean+dev2[n])/2
            #dev2[n] = dev2_mean
        for n,i in enumerate(dev3):
            if ( i > OUTLIER):
                dev3[n] = dev3[n-1]
            dev3_mean=(dev3_mean+dev3[n])
            #dev3_mean=(dev3_mean+dev3[n])/2
            #dev3[n] = dev3_mean


        deviationplot0.plot(frame_number, dev0_gt/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot0.plot(frame_number, dev0/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[1+no_of_metrics], label=list_of_collision_metrics[1+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot0.plot(frame_number, dev1/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[2+no_of_metrics], label=list_of_collision_metrics[2+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot0.plot(frame_number, dev2/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[3+no_of_metrics], label=list_of_collision_metrics[3+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot0.plot(frame_number, dev3/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[4+no_of_metrics], label=list_of_collision_metrics[4+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))


        index_x0_gt_sorted = np.argsort(x0_gt)
        index_x0_sorted = np.argsort(x0)
        index_x1_sorted = np.argsort(x1)
        index_x2_sorted = np.argsort(x2)
        index_x3_sorted = np.argsort(x3)
        print x0
        print index_x0_gt_sorted

        collisionplot0.plot(x0_gt, y0_gt/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x0, y0/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[1+no_of_metrics], label=list_of_collision_metrics[1+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x1, y1/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[2+no_of_metrics], label=list_of_collision_metrics[2+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x2, y2/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[3+no_of_metrics], label=list_of_collision_metrics[3+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x3, y3/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[4+no_of_metrics], label=list_of_collision_metrics[4+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))


        offset_index=offset_index+4

        dev0_mean = dev0_mean/(n+1)
        dev1_mean = dev1_mean/(n+1)
        dev2_mean = dev2_mean/(n+1)
        dev3_mean = dev3_mean/(n+1)
        dev0_mean_list.append(dev0_mean)
        dev1_mean_list.append(dev1_mean)
        dev2_mean_list.append(dev2_mean)
        dev3_mean_list.append(dev3_mean)

        print "Table 3 " + environment_list[no_of_metrics]
        print dev0_gt_mean_list[no_of_metrics]/SCALE
        print dev0_mean_list[no_of_metrics]/SCALE
        print dev1_mean_list[no_of_metrics]/SCALE
        print dev2_mean_list[no_of_metrics]/SCALE
        print dev3_mean_list[no_of_metrics]/SCALE

    #fig2.set_size_inches(18.5, 10.5)
    DEV_MAX = max(numpy.amax(dev0_gt/SCALE), numpy.amax(dev0/SCALE), numpy.amax(dev1/SCALE), numpy.amax(dev2/SCALE), numpy.amax(dev3/SCALE)) + 100
    Y_MAX = max(numpy.amax(y0_gt/SCALE), numpy.amax(y0/SCALE), numpy.amax(y1/SCALE), numpy.amax(y2/SCALE), numpy.amax(y3/SCALE)) + 100
    Y_MIN = max(numpy.amin(y0_gt/SCALE), numpy.amin(y0/SCALE), numpy.amin(y1/SCALE), numpy.amin(y2/SCALE), numpy.amin(y3/SCALE)) - 100
    deviationplot0.set_ylim([0, DEV_MAX])
    collisionplot0.set_ylim([Y_MIN, Y_MAX])
    #plt.show()
    fig1.savefig(output_folder + 'deviation_plot', bbox_inches='tight',dpi=200)
    fig2.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


#    plt.close("all")


def motionflow_vectorgraphs_noise():

    offset=1
    offset_index=0

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig0 = plt.figure()
    #plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig1 = plt.figure()
    #plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig2 = plt.figure()
    #plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig3 = plt.figure()
    #plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    deviationplot0 = fig0.add_subplot(111)
    deviationplot1 = fig1.add_subplot(111)
    deviationplot2 = fig2.add_subplot(111)
    deviationplot3 = fig3.add_subplot(111)

    #deviationplot0.set_title('collision_pointsframe_skip1_dataprocessing_0')
    #deviationplot1.set_title('collision_pointsframe_skip1_dataprocessing_1')
    #deviationplot2.set_title('collision_pointsframe_skip1_dataprocessing_2')
    #deviationplot3.set_title('collision_pointsframe_skip1_dataprocessing_3')

    deviationplot0.set_xlabel('frame_count')
    deviationplot0.set_ylabel('deviation noise_points-no_noise_points')
    deviationplot1.set_xlabel('frame_count')
    deviationplot1.set_ylabel('deviation noise_points-no_noise_points')
    deviationplot2.set_xlabel('frame_count')
    deviationplot2.set_ylabel('deviation noise_points-no_noise_points')
    deviationplot3.set_xlabel('frame_count')
    deviationplot3.set_ylabel('deviation noise_points-no_noise_points')

    deviationplot0.set_ylim([0, OUTLIER])
    deviationplot1.set_ylim([0, OUTLIER])
    deviationplot2.set_ylim([0, OUTLIER])
    deviationplot3.set_ylim([0, OUTLIER])
    legend = deviationplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [

        "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",

        "collision_pointsframe_skip1_dataprocessing_0results_FB_light_snow_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_light_snow_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_light_snow_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_light_snow_",

        "collision_pointsframe_skip1_dataprocessing_0results_FB_mild_snow_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_mild_snow_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_mild_snow_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_mild_snow_",

        "collision_pointsframe_skip1_dataprocessing_0results_FB_heavy_snow_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_heavy_snow_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_heavy_snow_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_heavy_snow_",

    ]

    color_of_collision_metrics  = ["red", "green", "yellow", "black"]

    print len(list_of_collision_metrics)

    assert(len(list_of_collision_metrics)/4 == len(color_of_collision_metrics))

    num=0

    for no_of_metrics in range(1): # generated

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index+x]]
            print offset_index+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0_base, y0_base = data.T
            if ( x == 1):
                x1_base, y1_base = data.T
            if ( x == 2):
                x2_base, y2_base = data.T
            if ( x == 3):
                x3_base, y3_base = data.T

            x0 = np.arange(0.0, 5.0, 1)

    dev0_mean_list = list()
    dev1_mean_list = list()
    dev2_mean_list = list()
    dev3_mean_list = list()

    for no_of_metrics in range(len(environment_list)): # none

        dev0_mean = 0
        dev1_mean = 0
        dev2_mean = 0
        dev3_mean = 0

        for x in range(4):


            collision_points = yaml_load[list_of_collision_metrics[offset_index+x]]
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                dev0 = numpy.sqrt((x0_base - x0) ** 2 + (y0_base - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                dev1 = numpy.sqrt((x1_base - x1) ** 2 + (y1_base - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                dev2 = numpy.sqrt((x2_base - x2) ** 2 + (y2_base - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                dev3 = numpy.sqrt((x3_base - x3) ** 2 + (y3_base - y3) ** 2)

            x0 = np.arange(0.0, len(collision_points)-1, 1)

        for n,i in enumerate(dev0):
            if ( i > OUTLIER):
                dev0[n] = dev0[n-1]
                if ( n == 0 ):
                    dev0[n] = 0
            dev0_mean=(dev0_mean+dev0[n])
            #dev0_mean=(dev0_mean+dev0[n])/2
            #dev0[n] = dev0_mean
        for n,i in enumerate(dev1):
            if ( i > OUTLIER):
                dev1[n] = dev1[n-1]
            dev1_mean=(dev1_mean+dev1[n])
            #dev1_mean=(dev1_mean+dev1[n])/2
            #dev1[n] = dev1_mean
        for n,i in enumerate(dev2):
            if ( i > OUTLIER):
                dev2[n] = dev2[n-1]
            dev2_mean=(dev2_mean+dev2[n])
            #dev2_mean=(dev2_mean+dev2[n])/2
            #dev2[n] = dev2_mean
        for n,i in enumerate(dev3):
            if ( i > OUTLIER):
                dev3[n] = dev3[n-1]
            dev3_mean=(dev3_mean+dev3[n])
            #dev3_mean=(dev3_mean+dev3[n])/2
            #dev3[n] = dev3_mean

        dev0_mean = dev0_mean/(n+1)
        dev1_mean = dev1_mean/(n+1)
        dev2_mean = dev2_mean/(n+1)
        dev3_mean = dev3_mean/(n+1)
        dev0_mean_list.append(dev0_mean)
        dev1_mean_list.append(dev1_mean)
        dev2_mean_list.append(dev2_mean)
        dev3_mean_list.append(dev3_mean)

        deviationplot0.plot(x0, dev0/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #deviationplot1.legend()
        deviationplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot1.plot(x0, dev1/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #deviationplot1.legend()
        deviationplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot2.plot(x0, dev2/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #deviationplot1.legend()
        deviationplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        deviationplot3.plot(x0, dev3/SCALE, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #deviationplot1.legend()
        deviationplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

        print "Table 4 " + environment_list[no_of_metrics]
        print dev0_mean_list[no_of_metrics]/SCALE
        print dev1_mean_list[no_of_metrics]/SCALE
        print dev2_mean_list[no_of_metrics]/SCALE
        print dev3_mean_list[no_of_metrics]/SCALE

    Y_MAX = max(numpy.amax(dev0/SCALE), numpy.amax(dev1/SCALE), numpy.amax(dev2/SCALE), numpy.amax(dev3/SCALE)) + 100
    deviationplot0.set_ylim([0, Y_MAX])
   # fig0.set_size_inches(18.5, 10.5)
    fig0.savefig(output_folder + 'vector_robustness_data_processing_algorithm_0',bbox_inches='tight', dpi=200)
    #fig1.set_size_inches(18.5, 10.5)
    fig1.savefig(output_folder + 'vector_robustness_data_processing_algorithm_1',bbox_inches='tight', dpi=200)
   # fig2.set_size_inches(18.5, 10.5)
    fig2.savefig(output_folder + 'vector_robustness_data_processing_algorithm_2',bbox_inches='tight', dpi=200)
   # fig3.set_size_inches(18.5, 10.5)
    fig3.savefig(output_folder + 'vector_robustness_data_processing_algorithm_3',bbox_inches='tight', dpi=200)

    plt.close('all')



if __name__ == '__main__':
    #histogram()
    #cam()
    #histogram_image()
    #threedplot()
    #sphere()
    #criticalpoint()
    #lemniscate()
    #simple()

    motionflow_pixelgraphs_no_noise()
    #motionflow_pixelgraphs_noise()
    motionflow_vectorgraphs_no_noise()
    #motionflow_vectorgraphs_noise()

    #histogramm()

    #motionflow_vectorgraphs()
    #check()
    #vkitti()

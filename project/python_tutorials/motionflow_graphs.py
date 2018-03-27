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

environment_list = ["none", "night"]

def motionflow_pixelgraphs_no_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

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
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig2.add_subplot(111)

    shapeplot0.set_title('shape_pointsframe_skip1_dataprocessing_0')

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

    color_of_shape_metrics = ["red", "green", "blue", "black"]

    #assert(len(list_of_shape_metrics)/1 == len(color_of_shape_metrics))

    offset=1
    offset_index=0
    num=0

    for no_of_metrics in range(1): #generated

        for x in range(1):

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
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

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+1+x]]
            print offset_index*no_of_metrics+1+x
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

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/pixel_robustness_optical_flow.png', dpi=200)


def motionflow_pixelgraphs_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

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
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig1 = plt.figure()
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig2 = plt.figure()
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")
    fig3 = plt.figure()
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig0.add_subplot(111)
    shapeplot1 = fig1.add_subplot(111)
    shapeplot2 = fig2.add_subplot(111)
    shapeplot3 = fig3.add_subplot(111)

    shapeplot0.set_title('shape_pointsframe_skip1_dataprocessing_0')
    shapeplot1.set_title('shape_pointsframe_skip1_dataprocessing_1')
    shapeplot2.set_title('shape_pointsframe_skip1_dataprocessing_2')
    shapeplot3.set_title('shape_pointsframe_skip1_dataprocessing_3')

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
        "shape_pointsframe_skip1_dataprocessing_1results_FB_night_",
        "shape_pointsframe_skip1_dataprocessing_0results_FB_night_",
        "shape_pointsframe_skip1_dataprocessing_2results_FB_night_",
        "shape_pointsframe_skip1_dataprocessing_3results_FB_night_",
    ]

    color_of_shape_metrics = ["red", "black"]

    assert(len(list_of_shape_metrics)/4 == len(color_of_shape_metrics))

    offset=1
    offset_index=0
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

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
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


    fig0.set_size_inches(18.5, 10.5)
    fig0.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/pixel_robustness_data_processing_algorithm_0', dpi=200)
    fig1.set_size_inches(18.5, 10.5)
    fig1.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/pixel_robustness_data_processing_algorithm_1', dpi=200)
    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/pixel_robustness_data_processing_algorithm_2', dpi=200)
    fig3.set_size_inches(18.5, 10.5)
    fig3.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/pixel_robustness_data_processing_algorithm_3', dpi=200)




def motionflow_vectorgraphs_no_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

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
    plt.suptitle("Deviation of collision points between ground truth and scenes without noise")

    collisionplot0 = fig2.add_subplot(111)

    collisionplot0.set_title('collision_pointsframe_skip1_dataprocessing_0')

    collisionplot0.set_xlabel('frame_count')
    collisionplot0.set_ylabel('deviation [noise_pixels-no_noise_total_pixels]')

    #collisionplot0.set_ylim([0, 100])
    legend = collisionplot0.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [
        "collision_pointsframe_skip1_dataprocessing_0_generated",
        "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",
    ]

    color_of_collision_metrics = ["red", "green", "blue", "black"]

    #assert(len(list_of_collision_metrics)/4 == len(color_of_collision_metrics))

    offset=0
    offset_index=0
    num=0

    for no_of_metrics in range(1): # generated


        for x in range(1):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0_gt, y0_gt = data.T

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

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+1+x]]
            print offset_index*no_of_metrics+1+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                y0 = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                y1 = numpy.sqrt((x0_gt - x1) ** 2 + (y0_gt - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                y2 = numpy.sqrt((x0_gt - x2) ** 2 + (y0_gt - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                y3 = numpy.sqrt((x0_gt - x3) ** 2 + (y0_gt - y3) ** 2)

            x0 = np.arange(0.0, len(collision_points), 1)

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

        collisionplot0.plot(x0, y0, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x0, y1, 'ko-', lw=1, color=color_of_collision_metrics[1+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x0, y2, 'ko-', lw=1, color=color_of_collision_metrics[2+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot0.plot(x0, y3, 'ko-', lw=1, color=color_of_collision_metrics[3+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

        print "Table 3 " + environment_list[no_of_metrics]
        print y0_mean_list[no_of_metrics]
        print y1_mean_list[no_of_metrics]
        print y2_mean_list[no_of_metrics]
        print y3_mean_list[no_of_metrics]

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/vector_robustness_optical_flow', dpi=200)



def motionflow_vectorgraphs_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

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
    plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig1 = plt.figure()
    plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig2 = plt.figure()
    plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    fig3 = plt.figure()
    plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    collisionplot0 = fig0.add_subplot(111)
    collisionplot1 = fig1.add_subplot(111)
    collisionplot2 = fig2.add_subplot(111)
    collisionplot3 = fig3.add_subplot(111)

    collisionplot0.set_title('collision_pointsframe_skip1_dataprocessing_0')
    collisionplot1.set_title('collision_pointsframe_skip1_dataprocessing_1')
    collisionplot2.set_title('collision_pointsframe_skip1_dataprocessing_2')
    collisionplot3.set_title('collision_pointsframe_skip1_dataprocessing_3')

    collisionplot0.set_xlabel('frame_count')
    collisionplot0.set_ylabel('deviation noise_pixels-no_noise_total_pixels')
    collisionplot1.set_xlabel('frame_count')
    collisionplot1.set_ylabel('deviation noise_pixels-no_noise_total_pixels')
    collisionplot2.set_xlabel('frame_count')
    collisionplot2.set_ylabel('deviation noise_pixels-no_noise_total_pixels')
    collisionplot3.set_xlabel('frame_count')
    collisionplot3.set_ylabel('deviation noise_pixels-no_noise_total_pixels')

    collisionplot0.set_ylim([0, 100])
    collisionplot1.set_ylim([0, 100])
    collisionplot2.set_ylim([0, 100])
    collisionplot3.set_ylim([0, 100])
    legend = collisionplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [
        "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",
        "collision_pointsframe_skip1_dataprocessing_1results_FB_night_",
        "collision_pointsframe_skip1_dataprocessing_0results_FB_night_",
        "collision_pointsframe_skip1_dataprocessing_2results_FB_night_",
        "collision_pointsframe_skip1_dataprocessing_3results_FB_night_",
    ]

    color_of_collision_metrics = ["red", "black"]

    assert(len(list_of_collision_metrics)/4 == len(color_of_collision_metrics))

    offset=0
    offset_index=0
    num=0

    for no_of_metrics in range(1): # generated

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
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

    y0_mean_list = list()
    y1_mean_list = list()
    y2_mean_list = list()
    y3_mean_list = list()

    for no_of_metrics in range(len(environment_list)): # none

        y0_mean = 0
        y1_mean = 0
        y2_mean = 0
        y3_mean = 0

        for x in range(4):


            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                y0 = numpy.sqrt((x0_base - x0) ** 2 + (y0_base - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                y1 = numpy.sqrt((x1_base - x1) ** 2 + (y1_base - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                y2 = numpy.sqrt((x2_base - x2) ** 2 + (y2_base - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                y3 = numpy.sqrt((x3_base - x3) ** 2 + (y3_base - y3) ** 2)

            x0 = np.arange(0.0, len(collision_points), 1)

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

        collisionplot0.plot(x0, y0, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot1.plot(x0, y1, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot2.plot(x0, y2, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot3.plot(x0, y3, 'ko-', lw=1, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

        print "Table 4 " + environment_list[no_of_metrics]
        print y0_mean_list[no_of_metrics]
        print y1_mean_list[no_of_metrics]
        print y2_mean_list[no_of_metrics]
        print y3_mean_list[no_of_metrics]

    fig0.set_size_inches(18.5, 10.5)
    fig0.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/vector_robustness_data_processing_algorithm_0', dpi=200)
    fig1.set_size_inches(18.5, 10.5)
    fig1.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/vector_robustness_data_processing_algorithm_1', dpi=200)
    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/vector_robustness_data_processing_algorithm_2', dpi=200)
    fig3.set_size_inches(18.5, 10.5)
    fig3.savefig('/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/vector_robustness_data_processing_algorithm_3', dpi=200)






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
    motionflow_pixelgraphs_noise()
    motionflow_vectorgraphs_no_noise()
    motionflow_vectorgraphs_noise()

    #motionflow_vectorgraphs()
    #check()
    #vkitti()

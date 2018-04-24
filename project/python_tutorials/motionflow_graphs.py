#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import yaml
import io
import math
import pandas as pd
import cv2
import numpy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import scipy
from scipy.interpolate import griddata
from math import pi

import random

from motionflow_graphs_common import Figures, YAMLParser

from motionflow_graphs_data import *

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "vires"
scenario = "two"
file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" +scenario + "/values.yml"

#file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

environment_list = ["none","light_snow_", "mild_snow_", "heavy_snow_"]#night
environment_list = ["none",]

data_processing_list = ["0", "1", "2", "3"]

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

    y = numpy.asarray(ybuf)
    bins = xbuf # use for small bars
    bins = range(-10,10)
    print bins

    x,y,_ = plt.hist(y.astype('float'),bins=bins, align='left', rwidth=0.5,)

    maxIndex = numpy.argmax(x)
    print maxIndex

    plt.bar(bins[0]+maxIndex,max(x),color='red',width=0.5)

    plt.show()
    fig1.savefig(output_folder + 'histogramm.png', dpi= 200)
    plt.close('all')


def motionflow_pixelgraphs_no_noise(): ##done

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    figures = Figures(1)

    x0_dev_list = list()
    y0_dev_list = list()
    y0_mean = 0

    shape_points = yaml_load[list_of_shape_metrics_no_noise[yaml_list_index_offset]]
    shape = list()
    frame_count = numpy.arange(0.0, len(shape_points), 1)

    print "Table 1 OpticalFlowNoNoise " + environment_list[0]

    for count in range(len(shape_points)):
        xy = list()
        xy.append(shape_points[count]["good_pixels"])
        xy.append(shape_points[count]["total_pixels"])
        shape.append(xy)
    data = numpy.array(shape)
    x0_gt, y0_gt = data.T
    y0_gt = x0_gt/y0_gt

    x0_dev_list.append(frame_count)
    y0_dev_list.append(y0_gt)

    y0_mean_list = list()

    for n,i in enumerate(y0_gt):
        y0_mean=y0_mean+i
    y0_mean = y0_mean/(n+1)
    y0_mean_list.append(y0_mean)

    for x in range(len(data_processing_list)):

        shape_points = yaml_load[list_of_shape_metrics_no_noise[yaml_list_index_offset+1+x]]
        shape = list()
        y0_mean = 0
        for count in range(len(shape_points)):
            xy = list()
            xy.append(shape_points[count]["good_pixels"])
            xy.append(shape_points[count]["total_pixels"])
            shape.append(xy)
        data = numpy.array(shape)
        x0, y0 = data.T
        y0 = x0/y0
        for n,i in enumerate(y0):
            y0_mean=y0_mean+i
        y0_mean = y0_mean/(n+1)
        y0_mean_list.append(y0_mean)

        x0_dev_list.append(frame_count)
        y0_dev_list.append(y0)

    print y0_mean_list

    plot1 = ('frame_count',
             'no_noise_good_pixels/no_noise_total_pixels',
             x0_dev_list,
             y0_dev_list,
             color_of_shape_metrics_no_noise,
             list_of_shape_metrics_no_noise,
             "optical flow pixel robustness",
             0,
             1,
             )

    figures.plot_all([plot1], 0)
    figures.plot_show_shape_no_noise()



def motionflow_pixelgraphs_noise():

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    print "Start Noise "

    figures = Figures(4)

    for env_index in range(len(environment_list)):

        print "Table 2 " + environment_list[env_index]
        y0_mean_list = list()
        list_of_plots = list()

        for x in range(len(data_processing_list)):

            y0_mean = 0
            shape_points = yaml_load[list_of_shape_metrics_noise[yaml_list_index_offset+x]]
            shape = list()
            frame_count = numpy.arange(0.0, len(shape_points), 1)
            for count in range(len(shape_points)):
                xy = list()
                xy.append(shape_points[count]["good_pixels"])
                xy.append(shape_points[count]["total_pixels"])
                shape.append(xy)
            data = numpy.array(shape)
            x0, y0 = data.T
            y0 = x0/y0
            for n,i in enumerate(y0):
                y0_mean=y0_mean+i
            y0_mean = y0_mean/(n+1)
            y0_mean_list.append(y0_mean)
            plot1 = ('frame_count',
                     'noise_good_pixels/no_noise_total_pixels',
                     [frame_count],
                     [y0],
                     color_of_shape_metrics_noise,
                     list_of_shape_metrics_noise,
                     "shape_pointsframe_skip1_dataprocessing_" + str(x),
                     0,
                     1,
                     )
            list_of_plots.append(plot1)

        print y0_mean_list

        yaml_list_index_offset=yaml_list_index_offset+4
        figures.plot_all(list_of_plots, env_index)

    figures.plot_show_shape_noise()





def motionflow_vectorgraphs_no_noise():


    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    delete_point_array = numpy.array([-65535])

    dev0_gt_mean = 0
    dev0_mean = 0
    dev0_gt_mean_list = list()
    dev0_mean_list = list()

    max_coll = 0; min_coll = 0; min_dev = 0; max_dev = 0;

    x0_dev_list = list()
    y0_dev_list = list()
    x0_coll_list = list()
    y0_coll_list = list()

    collision_points = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset]]
    collision = list()
    frame_count = numpy.arange(0.0, len(collision_points), 1)
    for count in range(len(collision_points)):
        xy = list()
        xy.append(collision_points[count]["x"])
        xy.append(collision_points[count]["y"])
        collision.append(xy)

    data = numpy.array(collision)
    x0_gt, y0_gt = data.T

    #x0_gt = numpy.setdiff1d(x0_gt, delete_point_array)
    #y0_gt = numpy.setdiff1d(y0_gt, delete_point_array)


    dev0_gt = numpy.sqrt((x0_gt - x0_gt) ** 2 + (y0_gt - y0_gt) ** 2)

    for n,i in enumerate(dev0_gt):
        dev0_gt_mean=(dev0_gt_mean+dev0_gt[n])

    dev0_gt_mean = dev0_gt_mean/(n+1)
    dev0_gt_mean_list.append(dev0_gt_mean)


    x0_dev_list.append(frame_count)
    y0_dev_list.append(dev0_gt)

    x0_coll_list.append(x0_gt)
    y0_coll_list.append(y0_gt)

    yaml_list_index_offset=0

    for x in range(len(data_processing_list)):

        collision_points = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset+1+x]]
        collision = list()
        for count in range(len(collision_points)):
            xy = list()
            xy.append(collision_points[count]["x"])
            xy.append(collision_points[count]["y"])
            collision.append(xy)
        data = numpy.array(collision)
        x0, y0 = data.T
        #x0 = numpy.setdiff1d(x0, delete_point_array)
        #y0 = numpy.setdiff1d(y0, delete_point_array)
        dev0 = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2)
        for n,i in enumerate(dev0):
            if ( abs(i) > OUTLIER):
                dev0[n] = dev0[n-1]
                if ( n == 0 ):
                    dev0[n] = 0
            dev0_mean=(dev0_mean+dev0[n])
            #dev0_mean=(dev0_mean+dev0[n])/2
            #dev0[n] = dev0_mean

        dev0_mean = dev0_mean/(n+1)
        dev0_mean_list.append(dev0_mean)

        x0_dev_list.append(frame_count)
        y0_dev_list.append(dev0)

        x0_coll_list.append(x0)
        y0_coll_list.append(y0)

        max_coll = max(numpy.amax(dev0), max_coll)
        min_coll = min(numpy.amin(dev0), min_coll)

        max_dev = max(numpy.amax(dev0), max_dev)
        min_dev = min(numpy.amin(dev0), min_dev)




    plot1 = ('frame_count',
             'deviation [no_noise_points-groundtruth_points]',
             x0_dev_list,
             y0_dev_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "deviation_pointsframe_skip1_dataprocessing_0",
             min_dev,
             max_dev,
             )

    plot2 = ('X', 'Y',
             x0_coll_list,
             y0_coll_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "collision_pointsframe_skip1_dataprocessing_0",
             min_coll,
             max_coll,
             )

    index_x0_gt_sorted = numpy.argsort(x0_gt)

    print "Table 3 " + environment_list[0]
    print dev0_gt_mean_list
    print dev0_mean_list

    figures = Figures(1)
    figures.plot_all([plot1], 0)
    figures.plot_show_vector_dev_no_noise()

    figures = Figures(1)
    figures.plot_all([plot2], 0)
    figures.plot_show_vector_coll_no_noise()



def motionflow_vectorgraphs_noise():

    offset=1
    yaml_list_index_offset=0


    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    min_dev = 0; max_dev = 0;


    x0_base_list = list(); y0_base_list = list();

    for x in range(len(data_processing_list)):

        collision_points = yaml_load[list_of_collision_metrics_noise[yaml_list_index_offset+x]]
        collision = list()
        for count in range(len(collision_points)):
            xy = list()
            xy.append(collision_points[count]["x"])
            xy.append(collision_points[count]["y"])
            collision.append(xy)
        data = numpy.array(collision)
        if ( x == 0):
            x0_base, y0_base = data.T
            x0_base_list.append(x0_base)
            y0_base_list.append(y0_base)
        if ( x == 1):
            x1_base, y1_base = data.T
            x0_base_list.append(x1_base)
            y0_base_list.append(y1_base)
        if ( x == 2):
            x2_base, y2_base = data.T
            x0_base_list.append(x2_base)
            y0_base_list.append(y2_base)
        if ( x == 3):
            x3_base, y3_base = data.T
            x0_base_list.append(x3_base)
            y0_base_list.append(y3_base)

    figures = Figures( 4 )

    for env_index in range(len(environment_list)):

        print "Table 4 " + environment_list[env_index]
        dev0_mean_list = list()
        list_of_plots = list()

        for x in range(len(data_processing_list)):

            dev0_mean = 0
            collision_points = yaml_load[list_of_collision_metrics_noise[yaml_list_index_offset+x]]
            collision = list()
            frame_count = numpy.arange(0.0, len(collision_points), 1)
            for count in range(len(collision_points)):
                xy = list()
                xy.append(collision_points[count]["x"])
                xy.append(collision_points[count]["y"])
                collision.append(xy)
            data = numpy.array(collision)
            x0, y0 = data.T
            dev0 = numpy.sqrt((x0_base_list[x] - x0) ** 2 + (y0_base_list[x] - y0) ** 2)

            for n,i in enumerate(dev0):
                if ( i > OUTLIER):
                    dev0[n] = dev0[n-1]
                    if ( n == 0 ):
                        dev0[n] = 0
                dev0_mean=(dev0_mean+i)

            dev0_mean = dev0_mean/(n+1)
            dev0_mean_list.append(dev0_mean)

            max_dev = max(numpy.amax(dev0), max_dev)
            min_dev = min(numpy.amin(dev0), min_dev)

            plot1 = ('frame_count',
                     'deviation [noise_points-nonoise_points]',
                     [frame_count],
                     [dev0],
                     color_of_collision_metrics_noise,
                     list_of_collision_metrics_noise,
                     "deviation_pointsframe_skip1_dataprocessing_" + str(x),
                     min_dev,
                     max_dev,
                     )
            list_of_plots.append(plot1)


        yaml_list_index_offset=yaml_list_index_offset+4
        figures.plot_all(list_of_plots, env_index)

        print dev0_mean_list

    figures.plot_show_vector_noise()



def scenario_displacement_occurence():


    offset=1
    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #ax = fig.add_subplot(111, projection='3d')
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')

    X, Y, Z = axes3d.get_test_data(0.1)
    #ax.plot_wireframe(X, Y, Z, rstride=5, cstride=5)


    for env_index in range(1): # generated

        scenario_displacement_occurence = yaml_load[list_of_displacement_occurence_metrics[env_index]]
        occurences = list()
        for count in range(len(scenario_displacement_occurence)):
            xyz = list()
            xyz.append(scenario_displacement_occurence[count]["x"])
            xyz.append(scenario_displacement_occurence[count]["y"])
            xyz.append(scenario_displacement_occurence[count]["occurence"])
            occurences.append(xyz)

        data = numpy.array(occurences)

        x_gt, y_gt, occurence_gt = data.T
        print data

        scenario_displacement_occurence = yaml_load[list_of_displacement_occurence_metrics[env_index+1]]
        occurences = list()
        for count in range(len(scenario_displacement_occurence)):
            xyz = list()
            xyz.append(scenario_displacement_occurence[count]["x"])
            xyz.append(scenario_displacement_occurence[count]["y"])
            xyz.append(scenario_displacement_occurence[count]["occurence"])
            occurences.append(xyz)

        data = numpy.array(occurences)

        for x in range(1): # generated
            if ( x == 0 ):
                x, y, occurence = data.T


    dx = numpy.empty(numpy.size(x_gt))
    dx.fill(0.1)
    dy = numpy.empty(numpy.size(x_gt))
    dy.fill(0.1)

    #ax1.set_xlim([-6,6])
    #ax1.set_ylim([-2,2])
    ax1.set_xlim([min(numpy.amin(x_gt), numpy.amin(x)),  max(numpy.amax(x_gt), numpy.amax(x))])
    ax1.set_ylim([min(numpy.amin(y_gt), numpy.amin(y)),  max(numpy.amax(y_gt), numpy.amax(y))])
    ax1.set_zlim([0, numpy.amax(occurence_gt)])

    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')

    z_gt = numpy.zeros(numpy.size(x_gt))

    ax1.bar3d(x_gt, y_gt, z_gt, dx, dy, occurence_gt, "red" )
    #ax1.bar(x_gt, y_gt, occurence_gt, zdir='z', color='red', alpha=0.8 )

    dx = numpy.empty(numpy.size(x))
    dx.fill(0.5)
    dy = numpy.empty(numpy.size(x))
    dy.fill(0.5)

    z = numpy.zeros(numpy.size(x))

    ax2.set_xlabel('X Label')
    ax2.set_ylabel('Y Label')
    ax2.set_zlabel('Z Label')

    ax2.set_xlim([min(numpy.amin(x_gt), numpy.amin(x)),  max(numpy.amax(x_gt), numpy.amax(x))])
    ax2.set_ylim([min(numpy.amin(y_gt), numpy.amin(y)),  max(numpy.amax(y_gt), numpy.amax(y))])
    ax2.set_zlim([0,numpy.amax(occurence)])

    ax2.bar3d(x, y, z, dx, dy, occurence, "red" )

    #ax2.set_xlim([-10,10])
    #ax2.set_ylim([-10,10])

    #ax2.bar3d(x, y, occurence, numpy.ones(numpy.size(x)), numpy.ones(numpy.size(x)), numpy.ones(numpy.size(x)), "red" )
    #ax2.plot_wireframe(x, y, occurence)


    figures = Figures(1)
    fig1.savefig(output_folder + '3d_plot_gt.png', dpi= 200)
    fig2.savefig(output_folder + '3d_plot_algo.png', dpi= 200)




#    plt.close("all")



if __name__ == '__main__':


    #motionflow_pixelgraphs_no_noise()
    #motionflow_pixelgraphs_noise()
    #motionflow_vectorgraphs_no_noise()
    #motionflow_vectorgraphs_noise()
    scenario_displacement_occurence()
    #histogramm()


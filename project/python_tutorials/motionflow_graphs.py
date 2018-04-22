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
from mpl_toolkits.mplot3d import axes3d
import scipy
from scipy.interpolate import griddata
import matplotlib as mp
from math import pi

import random

from motionflow_graphs_data import *

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

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    graph = Graph(1)

    x0_list = list()
    y0_list = list()

    for no_of_metrics in range(1): #generated

        for x in range(1):

            shape_points = yaml_load[list_of_shape_metrics_no_noise[offset_index+x]]
            shape = list()
            frame_count = np.arange(0.0, len(shape_points)-1, 1)
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            x0_gt, y0_gt = data.T
            y0_gt = x0_gt/y0_gt

            x0_list.append(frame_count)
            y0_list.append(y0_gt)

    y0_mean_list = list()

    for no_of_metrics in range(1): # none

        print "Table 1 " + environment_list[no_of_metrics]

        y0_mean = 0
        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics_no_noise[offset_index+1+x]]
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            x0, y0 = data.T
            y0 = x0/y0
            for n,i in enumerate(y0):
                y0_mean=y0_mean+i
            y0_mean = y0_mean/(n+1)
            y0_mean_list.append(y0_mean)
            frame_count = np.arange(0.0, len(x0), 1)

            x0_list.append(frame_count)
            y0_list.append(y0)

            print y0_mean_list[x]

        offset_index=offset_index+4


    plot1 = ('frame_count',
             'no_noise_good_pixels/no_noise_total_pixels',
             x0_list,
             y0_list,
             color_of_shape_metrics_no_noise,
             list_of_shape_metrics_no_noise,
             "shape_pointsframe_skip1_dataprocessing_0"
             )

    graph.plot_all([plot1])
    graph.plot_show_shape_no_noise()



def motionflow_pixelgraphs_noise():

    offset=1
    offset_index=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    print "Start Noise "

    y0_mean_list = list()

    graph = Graph(4)

    for no_of_metrics in range(len(environment_list)):

        y0_mean = 0
        print "Table 2 " + environment_list[no_of_metrics]

        list_of_plots = list()

        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics_noise[offset_index+x]]
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            x0, y0 = data.T
            y0 = x0/y0
            for n,i in enumerate(y0):
                y0_mean=y0_mean+i
            y0_mean = y0_mean/(n+1)
            y0_mean_list.append(y0_mean)
            frame_count = np.arange(0.0, len(x0), 1)
            plot1 = ('frame_count',
                     'noise_good_pixels/no_noise_total_pixels',
                     [frame_count],
                     [y0],
                     color_of_shape_metrics_noise,
                     list_of_shape_metrics_noise,
                     "shape_pointsframe_skip1_dataprocessing_0"
                     )
            list_of_plots.append(plot1)

            print y0_mean_list[x]

        offset_index=offset_index+4
        graph.plot_all(list_of_plots)


    graph.plot_show_shape_noise()





def motionflow_vectorgraphs_no_noise():


    offset=1
    offset_index=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    delete_point_array = np.array([-65535])

    dev0_gt_mean = 0
    dev0_mean = 0
    dev0_gt_mean_list = list()
    dev0_mean_list = list()

    x0_list = list()
    y0_list = list()
    x0_coll_list = list()
    y0_coll_list = list()

    collision_points = yaml_load[list_of_collision_metrics_no_noise[offset_index]]
    collision = list()
    for count in range(len(collision_points)-offset):
        xy = list()
        xy.append(collision_points[offset + count]["x"])
        xy.append(collision_points[offset + count]["y"])
        collision.append(xy)

    data = np.array(collision)
    x0_gt, y0_gt = data.T
    x0_gt = np.setdiff1d(x0_gt, delete_point_array)
    y0_gt = np.setdiff1d(y0_gt, delete_point_array)
    dev0_gt = numpy.sqrt((x0_gt - x0_gt) ** 2 + (y0_gt - y0_gt) ** 2)
    for n,i in enumerate(dev0_gt):
        dev0_gt_mean=(dev0_gt_mean+dev0_gt[n])

    dev0_gt_mean = dev0_gt_mean/(n+1)
    dev0_gt_mean_list.append(dev0_gt_mean)

    frame_count = np.arange(0.0, len(x0_gt), 1)

    x0_list.append(frame_count)
    y0_list.append(dev0_gt)

    x0_coll_list.append(x0_gt)
    y0_coll_list.append(y0_gt)


    offset_index=0

    for x in range(4):

        collision_points = yaml_load[list_of_collision_metrics_no_noise[offset_index+1+x]]
        collision = list()
        for count in range(len(collision_points)-offset):
            xy = list()
            xy.append(collision_points[offset + count]["x"])
            xy.append(collision_points[offset + count]["y"])
            collision.append(xy)
        data = np.array(collision)
        x0, y0 = data.T
        x0 = np.setdiff1d(x0, delete_point_array)
        y0 = np.setdiff1d(y0, delete_point_array)
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

        x0_list.append(frame_count)
        y0_list.append(dev0)

        x0_coll_list.append(x0)
        y0_coll_list.append(y0)


    offset_index=offset_index+4


    plot1 = ('frame_count',
             'deviation [no_noise_points-groundtruth_points]',
             x0_list,
             y0_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "deviation_pointsframe_skip1_dataprocessing_0"
             )

    plot2 = ('X', 'Y',
             x0_coll_list,
             y0_coll_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "collision_pointsframe_skip1_dataprocessing_0"
             )

    index_x0_gt_sorted = np.argsort(x0_gt)

    print "Table 3 " + environment_list[0]
    print dev0_gt_mean_list[0]/SCALE
    print dev0_mean_list[0   ]/SCALE
    print dev0_mean_list[1]/SCALE
    print dev0_mean_list[2]/SCALE
    print dev0_mean_list[3]/SCALE

    graph = Graph(2)
    graph.plot_all([plot1, plot2])
    graph.plot_show_vector_no_noise()




def motionflow_vectorgraphs_noise():

    offset=1
    offset_index=0


    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()


    for no_of_metrics in range(1): # generated

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics_noise[offset_index+x]]
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

    dev0_mean_list = list()
    dev1_mean_list = list()
    dev2_mean_list = list()
    dev3_mean_list = list()

    graph = Graph( 4 )

    for no_of_metrics in range(len(environment_list)):

        dev0_mean = 0
        dev1_mean = 0
        dev2_mean = 0
        dev3_mean = 0

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics_noise[offset_index+x]]
            collision = list()
            x0 = np.arange(0.0, len(collision_points)-1, 1)
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


        frame_count = np.arange(0.0, len(x0), 1)

        plot1 = ('frame_count',
                 'deviation [noise_points-nonoise_points]',
                 [frame_count],
                 [dev0],
                 color_of_collision_metrics_noise,
                 list_of_collision_metrics_noise,
                 "vector_robustness_data_processing_algorithm_0"
                 )
        plot2 = ('frame_count',
                 'deviation [noise_points-nonoise_points]',
                 [frame_count],
                 [dev1],
                 color_of_collision_metrics_noise,
                 list_of_collision_metrics_noise,
                 "vector_robustness_data_processing_algorithm_1"
                 )
        plot3 = ('frame_count',
                 'deviation [noise_points-nonoise_points]',
                 [frame_count],
                 [dev2],
                 color_of_collision_metrics_noise,
                 list_of_collision_metrics_noise,
                 "vector_robustness_data_processing_algorithm_2"
                 )
        plot4 = ('frame_count',
                 'deviation [noise_points-nonoise_points]',
                 [frame_count],
                 [dev3],
                 color_of_collision_metrics_noise,
                 list_of_collision_metrics_noise,
                 "vector_robustness_data_processing_algorithm_3"
                 )

        graph.plot_all([plot1, plot2, plot3, plot4])



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

        offset_index=offset_index+4

        print "Table 4 " + environment_list[no_of_metrics]
        print dev0_mean_list[no_of_metrics]/SCALE
        print dev1_mean_list[no_of_metrics]/SCALE
        print dev2_mean_list[no_of_metrics]/SCALE
        print dev3_mean_list[no_of_metrics]/SCALE

    graph.plot_show_vector_noise()



class Graph(object):

    def __init__(self, number_of_plots):

        self.number_of_plots = number_of_plots
        self.list_of_plots = []
        self.color_index = 0

        if ( self.number_of_plots >= 1 ):
            self.fig1 = plt.figure(figsize=plt.figaspect(0.5))
            #self.ax = axes3d(self.fig1)
            plot1 = self.fig1.add_subplot(111)
            self.list_of_plots.append(plot1)

        if ( self.number_of_plots >= 2 ):
            self.fig2 = plt.figure()
            self.plot2 = self.fig2.add_subplot(111)
            self.list_of_plots.append(self.plot2)

        if ( self.number_of_plots >= 3 ):
            self.fig3 = plt.figure()
            self.plot3 = self.fig3.add_subplot(111)
            self.list_of_plots.append(self.plot3)

        if ( self.number_of_plots >= 4 ):
            self.fig4 = plt.figure()
            self.plot4 = self.fig4.add_subplot(111)
            self.list_of_plots.append(self.plot4)

        #plt.suptitle("Deviation of collision points between ground truth and scenes without noise")


    def plot_3d(self, plot_configuration_list):

        self.ax.plot(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)


    def plot_all(self, plot_configuration_list):

        number_of_plots = 0
        self.plot_configuration_list = plot_configuration_list

        while(number_of_plots < self.number_of_plots ):

            no_of_metrics = 0
            self.boundary = []
            self.list_of_plots[number_of_plots].set_xlabel(self.plot_configuration_list[number_of_plots][0])
            self.list_of_plots[number_of_plots].set_ylabel(self.plot_configuration_list[number_of_plots][1])
            legend = self.list_of_plots[number_of_plots].legend(loc='center right', shadow=True, fontsize='x-small')

            MAX=0
            MIN=0
            for x in range(len(self.plot_configuration_list[number_of_plots][3])):

                self.list_of_plots[number_of_plots].plot(self.plot_configuration_list[number_of_plots][2][x], self.plot_configuration_list[number_of_plots][3][x]/SCALE, 'ko-', lw=1, color=self.plot_configuration_list[number_of_plots][4][x], label=self.plot_configuration_list[number_of_plots][5][x])
                #pltplot1.legend()
                self.list_of_plots[number_of_plots].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                self.list_of_plots[number_of_plots].set_title(self.plot_configuration_list[number_of_plots][6])
                MAX = max(numpy.amax(self.plot_configuration_list[number_of_plots][3][x]/SCALE), MAX) + 100
                MIN = min(numpy.amin(self.plot_configuration_list[number_of_plots][3][x]/SCALE), MIN) + 100
                self.boundary.append((MAX, MIN))

            number_of_plots += 1
        self.color_index += 1


    def plot_show_vector_no_noise(self):

        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        self.fig1.savefig(output_folder + 'deviation_plot', bbox_inches='tight',dpi=200)

        #self.list_of_plots[0].set_ylim([0, self.boundary[1][1]])
        self.fig2.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)

        #plt.close("all")
        #fig2.set_size_inches(18.5, 10.5)

    def plot_show_shape_no_noise(self):

        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        self.fig1.savefig(output_folder + 'pixel_robustness_optical_flow.png', bbox_inches='tight',dpi=200)
        #plt.close("all")
        #fig2.set_size_inches(18.5, 10.5)

    def plot_show_vector_noise(self):

        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        self.fig1.savefig(output_folder + 'vector_robustness_data_processing_algorithm_0', bbox_inches='tight',dpi=200)
        #self.list_of_plots[0].set_ylim([0, self.boundary[1][1]])
        self.fig2.savefig(output_folder + 'vector_robustness_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        self.fig3.savefig(output_folder + 'vector_robustness_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
        #self.list_of_plots[0].set_ylim([0, self.boundary[1][1]])
        self.fig4.savefig(output_folder + 'vector_robustness_data_processing_algorithm_3', bbox_inches='tight',dpi=200)


    def plot_show_shape_noise(self):
        self.fig1.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_0', bbox_inches='tight', dpi=200)
        self.fig2.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
        self.fig3.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
        self.fig4.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_3', bbox_inches='tight',dpi=200)




class YAMLParser(object):

    def __init__(self, filename):
        self.file = filename
        self.yaml_file = open(file, "r")
        check = self.yaml_file.readline()
        print "yaml check" , check
        if ("YAML:1.0" in check ):
            read_yaml_file = self.yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
            read_yaml_file = self.read_yaml_file.replace(':', ': ')
            self.yaml_file.close()
            self.yaml_file = open(file, "w")
            self.yaml_file.write(read_yaml_file)
            self.yaml_file.close()


    def close(self):
        if ( self.yaml_file != None ):
            self.yaml_file.close()

    def load(self):
        yaml_load = yaml.load(open(self.file))
        return yaml_load


def scenario_displacement_occurence():


    offset=1
    offset_index=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #ax = fig.add_subplot(111, projection='3d')
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection='3d')

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection='3d')

    X, Y, Z = axes3d.get_test_data(0.1)
    #ax.plot_wireframe(X, Y, Z, rstride=5, cstride=5)


    for x in range(2): # generated

        scenario_displacement_occurence = yaml_load[list_of_displacement_occurence_metrics[x]]
        occurences = list()
        for count in range(len(scenario_displacement_occurence)):
            xyz = list()
            xyz.append(scenario_displacement_occurence[count]["x"])
            xyz.append(scenario_displacement_occurence[count]["y"])
            xyz.append(scenario_displacement_occurence[count]["occurence"])
            occurences.append(xyz)

        data = np.array(occurences)

        if ( x == 0 ):
            x_gt, y_gt, occurence_gt = data.T
            print x_gt
        if ( x == 1 ):
            x, y, occurence = data.T




    ax1.set_xlabel('X Label')
    ax1.set_ylabel('Y Label')
    ax1.set_zlabel('Z Label')

    ax1.set_xlim([numpy.amin(x_gt), numpy.amax(x_gt)])
    ax1.set_ylim([numpy.amin(y_gt), numpy.amax(y_gt)])
    ax1.set_zlim([0, numpy.amax(occurence_gt)])

    ax2.set_xlim([numpy.amin(x), numpy.amax(x)])
    ax2.set_ylim([numpy.amin(y), numpy.amax(y)])

    print numpy.amin(x_gt)
    print numpy.amax(x_gt)

    #ax1.set_ylim([-10,10])

    ax1.bar(x_gt, y_gt, occurence_gt)#, zdir='y', color='r', alpha=0.8 )
    ax2.plot_wireframe(x, y, occurence)


    graph = Graph(1)
    fig1.savefig(output_folder + '3d_plot_gt.png', dpi= 200)
    fig2.savefig(output_folder + '3d_plot_algo.png', dpi= 200)




#    plt.close("all")



if __name__ == '__main__':


    scenario_displacement_occurence()
    #motionflow_pixelgraphs_no_noise()
    #motionflow_pixelgraphs_noise()
    #motionflow_vectorgraphs_no_noise()
    #motionflow_vectorgraphs_noise()

    #histogramm()


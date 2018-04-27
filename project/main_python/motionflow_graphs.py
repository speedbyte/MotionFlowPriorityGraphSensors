#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import numpy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d

from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

import  robustness

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "vires"
scenario = "two"
#file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" +scenario + "/values.yml"
file = "/local/git/MotionFlowPriorityGraphSensors/project/main/values.yml"


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
            if ( scenario_displacement_occurence[count]["occurence"] > 200 ):
                xyz.append(scenario_displacement_occurence[count]["x"])
                xyz.append(scenario_displacement_occurence[count]["y"])
                xyz.append(scenario_displacement_occurence[count]["occurence"])
                occurences.append(xyz)

        data = numpy.array(occurences)

        x_gt, y_gt, occurence_gt = data.T

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


    data_list_deviation = list_of_collision_metrics_no_noise
    data_list_pixel = list_of_shape_metrics_no_noise
    data_list_deviation = list_of_collision_metrics_noise
    data_list_pixel = list_of_shape_metrics_noise

    robustness.robustness_(file, "pixel", "no_noise", list_of_shape_metrics_no_noise, color_of_shape_metrics_no_noise, "jaccard index no noise all algorithm")
    robustness.robustness_(file, "deviation", "no_noise", list_of_collision_metrics_no_noise, color_of_collision_metrics_no_noise, "deviation no noise all algorithm ")

    robustness.robustness_(file, "pixel", "noise", list_of_shape_metrics_noise, color_of_shape_metrics_noise, "jaccard index environment algorithm ")
    robustness.robustness_(file, "deviation", "noise" , list_of_collision_metrics_noise, color_of_collision_metrics_noise, "deviation environment algorithm ")

    robustness.collisiongraphs_no_noise(file, "collision", "no_noise", list_of_collision_metrics_no_noise, color_of_collision_metrics_no_noise, "collision points no noise all algorithm")
    #scenario_displacement_occurence()
    #histogramm()


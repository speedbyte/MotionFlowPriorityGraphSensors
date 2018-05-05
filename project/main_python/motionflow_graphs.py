#!/usr/bin/env python
# _*_ encoding=utf-8 _*_


import numpy
import matplotlib
matplotlib.use('Agg')

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d

from motionflow_graphs_common import Figures, YAMLParser
from robustness import SensorDataPlot
from motionflow_graphs_data import *

import  robustness
import threading
import time

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "vires"
scenario = "two"
#file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" +scenario + "/values.yml"
file = "/local/git/MotionFlowPriorityGraphSensors/project/main/values.yml"


SCALE = 1


def plot_at_once(figures_plot_array, sensor_index):

    lower_x = 0; upper_x = 0; lower_y = 0; upper_y = 0;

    for figures_plot in figures_plot_array:
        for figures_plot_index in range(len(figures_plot)):
            lower_x = min(figures_plot[figures_plot_index][5], lower_x)
            upper_x = max(figures_plot[figures_plot_index][6], upper_x)
            lower_y = min(figures_plot[figures_plot_index][7], lower_y)
            upper_y = max(figures_plot[figures_plot_index][8], upper_y)

    print lower_x, lower_y, upper_x, upper_y

    for figures_plot in figures_plot_array:

        figures = Figures(len(figures_plot[0][0]))

        for figures_plot_index in range(len(figures_plot)):

            figures_plot[figures_plot_index][0][0][7][0] = lower_x
            figures_plot[figures_plot_index][0][0][7][1] = upper_x
            figures_plot[figures_plot_index][0][0][8][0] = lower_y
            figures_plot[figures_plot_index][0][0][8][1] = upper_y

            print figures_plot[figures_plot_index][0][0][8][1]

            figures.plot_all(figures_plot[figures_plot_index][0], env_index=figures_plot[figures_plot_index][1], measuring_parameter=figures_plot[figures_plot_index][2])

        figures.save_figure(figures_plot[figures_plot_index][2], figures_plot[figures_plot_index][3], figures_plot[figures_plot_index][4], sensor_index)



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


    for env_index in range(1): # ground_truth

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

        for x in range(1): # ground_truth
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



class thread1(threading.Thread):

    def __init__(self, yaml_load, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_load = yaml_load
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread pixel "

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "environment"):
                current_list = environment_list

            # ---------------------------------
            for weather in environment_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("pixel", yaml_load, weather, step_size))

        self.threadRun = False


    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures



class thread2(threading.Thread):

    def __init__(self, yaml_load, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_load = yaml_load
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread deviation "

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "environment"):
                current_list = environment_list

            for weather in environment_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("deviation", yaml_load, weather, step_size))

        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


class thread3(threading.Thread):

    def __init__(self, yaml_load, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_load = yaml_load
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread collision "

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "environment"):
                current_list = environment_list

            for weather in environment_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("collision", yaml_load, weather, step_size))

        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


class thread4(threading.Thread):

    def __init__(self, yaml_load, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_load = yaml_load
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread obj displacement"

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "environment"):
                current_list = environment_list

            for weather in environment_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("obj_displacement", yaml_load, weather, step_size))
        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


#    plt.close("all")

import subprocess

if __name__ == '__main__':

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    command = "rm " + output_folder + "*.png"
    try:
        subprocess.check_output([command], shell='/usr/bin/bash')
    except subprocess.CalledProcessError as e:
        print e.returncode
        if e.returncode != 1:
            exit(0)


    for x in [1,2]:

        thread_pixel = None
        thread_deviation = None
        thread_collision = None
        thread_obj_displacement = None

        sensor_plot = SensorDataPlot(x)

        if ( 1 ):

            thread_pixel = thread1(yaml_load, sensor_plot)
            thread_pixel.start()

        if ( 1 ):

            thread_deviation = thread2(yaml_load, sensor_plot)
            thread_deviation.start()

        if ( 1 ):

            thread_collision = thread3(yaml_load, sensor_plot)
            thread_collision.start()

        if ( 1 ):

            thread_obj_displacement = thread4(yaml_load, sensor_plot)
            thread_obj_displacement.start()

        if thread_pixel != None:

            while ( True ):
                time.sleep(1)
                if ( thread_pixel.getThreadState() == False ):

                        plot_at_once_figures = thread_pixel.getPlotList()
                        plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                        # summary
                        summary = sensor_plot.get_summary()
                        figures = Figures(1)
                        figures.evaluate_pixel(summary, step_list)
                        figures.save_figure("pixel", "summary")

                        break


        if ( thread_deviation != None ):

            while ( True ):
                time.sleep(1)
                if ( thread_deviation.getThreadState() == False ):

                    plot_at_once_figures = thread_deviation.getPlotList()
                    plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                    # summary
                    summary = sensor_plot.get_summary()
                    figures = Figures(1)
                    figures.evaluate_deviation(summary, step_list)
                    figures.save_figure("deviation", "summary")

                    break


        if ( thread_collision != None ):

            while ( True ):
                time.sleep(1)
                if ( thread_collision.getThreadState() == False ):

                    plot_at_once_figures = thread_collision.getPlotList()
                    plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                    # summary
                    #summary = sensor_plot.get_summary()
                    #figures = Figures(1)
                    #figures.evaluate_collision(summary, step_list)
                    #figures.save_figure("collision", "summary")

                    break


        # ---------------------------------
        if ( thread_obj_displacement != None ):

            while ( True ):
                time.sleep(1)
                if ( thread_obj_displacement.getThreadState() == False ):

                    plot_at_once_figures = thread_obj_displacement.getPlotList()
                    plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                    # summary
                    summary = sensor_plot.get_summary()
                    figures = Figures(1)
                    figures.evaluate_obj_displacement(summary, step_list)
                    figures.save_figure("obj_displacement", "summary")

                    break

            #scenario_displacement_occurence()
            #histogramm()



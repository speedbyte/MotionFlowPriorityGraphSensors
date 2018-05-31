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

    print lower_x, upper_x, lower_y, upper_y

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


class thread1(threading.Thread):

    def __init__(self, yaml_file_data, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_file_data = yaml_file_data
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

            custom_data_list_name = list()
            plot_mapping = self.sensor_plot.templateToYamlMapping_GT("pixel")
            custom_data_list_name.append(plot_mapping)
            plot_data = self.sensor_plot.extract_plot_data_from_data_list(yaml_file_data, custom_data_list_name, "pixel", algorithm_list[0], "ground_truth", str(step_size), color_list_algorithms, label_list_algorithm, 0, "jaccard index " + algorithm_list[0] )
            self.plot_at_once_figures.append(plot_data)
            custom_data_list_name.append(plot_mapping)

            for n,i in enumerate(environment_list):
                plot_mapping = self.sensor_plot.templateToYamlMapping("pixel",i, step_size)
                custom_data_list_name[1] = plot_mapping
                print custom_data_list_name
                plot_data = self.sensor_plot.extract_plot_data_from_data_list(yaml_file_data, custom_data_list_name, "pixel", algorithm_list[0], i, str(step_size), color_list_algorithms, label_list_algorithm, 0, "jaccard index " + algorithm_list[0] )
                self.plot_at_once_figures.append(plot_data)


        self.threadRun = False


    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures

#    plt.close("all")

import subprocess

if __name__ == '__main__':

    yaml_file_handle = YAMLParser(file)
    yaml_file_data = yaml_file_handle.load()

    command = "rm " + output_folder + "*.png"
    try:
        subprocess.check_output([command], shell='/usr/bin/bash')
    except subprocess.CalledProcessError as e:
        print e.returncode
        if e.returncode != 1:
            exit(0)

    collectPlots = list()

    #for x in [0,1]:
    for x in [0]:

        thread_pixel = None
        thread_deviation = None
        thread_collision = None
        thread_obj_displacement = None

        sensor_plot = SensorDataPlot(x)

        thread_pixel = thread1(yaml_file_data, sensor_plot)
        thread_pixel.start()

        if thread_pixel != None:

            while ( True ):
                time.sleep(1)
                if ( thread_pixel.getThreadState() == False ):

                        plot_at_once_figures = thread_pixel.getPlotList()
                        collectPlots.append(plot_at_once_figures)
                        plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                        # summary
                        summary = sensor_plot.get_summary()
                        figures = Figures(1)
                        figures.evaluate_pixel(summary, step_list)
                        figures.save_figure("pixel", "summary")

                        break





    #for x in [1,2]:
    #    plot_at_once(collectPlots[x-1], x)


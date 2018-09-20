#!/usr/bin/env python
# _*_ encoding=utf-8 _*_


import numpy as np
import os
import matplotlib
matplotlib.use('Agg')

import cv2

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d

from motionflow_graphs_common import Figures, YAMLParser
from SensorDataPlot import SensorDataPlot
from motionflow_graphs_data import *

import threading
import time

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "vires"
scenario = "two"
#file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" +scenario + "/values.yml"
file = "/local/git/MotionFlowPriorityGraphSensors/project/main/values.yml"


SCALE = 1


def plot_at_once(figures_plot_array_all):

    print "plot_at_once---------------------------"
    print figures_plot_array_all
    print "plot_at_once---------------------------"

    lower_x = 0; upper_x = 0; lower_y = 0; upper_y = 0;

    for figures_plot_index_all in range(len(figures_plot_array_all)):
        figures_plot_array = figures_plot_array_all[figures_plot_index_all]
        for figures_plot_index in figures_plot_array:
                lower_x = min(figures_plot_index.get_x_axis_limits()[0], lower_x)
                upper_x = max(figures_plot_index.get_x_axis_limits()[1], upper_x)
                lower_y = min(figures_plot_index.get_y_axis_limits()[0], lower_y)
                upper_y = max(figures_plot_index.get_y_axis_limits()[1], upper_y)

    for figures_plot_index_all in range(len(figures_plot_array_all)):
        figures_plot_array = figures_plot_array_all[figures_plot_index_all]
        for figures_plot_index in figures_plot_array:
            figures_plot_index.set_x_axis_limits([lower_x, upper_x])
            figures_plot_index.set_y_axis_limits([lower_y, upper_y])

    print lower_x, upper_x, lower_y, upper_y

    # plot the figure in numpy cache.
    for figures_plot_index_all in range(len(figures_plot_array_all)):
        figures = Figures(1)
        figures_plot_array = figures_plot_array_all[figures_plot_index_all]
        figures.plot_all(figures_plot_array)

        # and lastly save the figure
        figures.save_figure(figures_plot_array[0].get_measuring_parameter(), figures_plot_array[0].get_algorithm(), figures_plot_array[0].get_step_size(), figures_plot_array_all[0][0].get_sensor_index())


#    plt.close("all")

import subprocess

# first for ground truth and then for ground truth and the each noise condition. This is because in each plot
# both ground truth and noise is depicted.

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

    for n, parameter in enumerate(parameter_list):
    # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        print "PARAMETER ----- ", parameter

        bargraph_summary_list_each_parameter_collect = list()
        plotgraph_list_each_parameter_collect = list()

        for sensor_index in sensor_list:

            parameter_plots_with_details_each_sensor = list()

            for algorithm in algorithm_list:

                sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm)
                sensor_data_plot_object.set_measuring_parameter(parameter)

                plot_mapping_gt = sensor_data_plot_object.templateToYamlMapping_GT()

                parameter_plot_all_noise_each_parameter = list()

                for step_size in step_list:

                    assert(noise_list[0] != "ground_truth")
                    print "---------------------------"

                    if just_ground_truth is True:
                        environment = ["ground_truth"]
                        plot_mapping = [plot_mapping_gt]

                    else:
                        environment = ["ground_truth"]
                        plot_mapping = [plot_mapping_gt]
                        for noise in noise_list:
                            environment.append(noise)
                            plot_mapping_noise = sensor_data_plot_object.templateToYamlMapping(noise, step_size)
                            plot_mapping.append(plot_mapping_noise)

                    print plot_mapping

                    for index,env in enumerate(environment):
                        plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, plot_mapping[index], parameter, sensor_index, env, str(step_size), 0, x_label="frame_number", y_label="dummy" ) #y_axis_label_dict[parameter]

                        parameter_plot_all_noise_each_parameter.append(plot_data)
                    
                    if just_ground_truth is True:
                        break
                        
                print "---------------------------"

                # store the summary for future use:
                plotgraph_list_each_parameter_collect.append(parameter_plot_all_noise_each_parameter)
                bargraph_summary_list_each_parameter_collect.append(sensor_data_plot_object.get_summary())

        # we are still in each parameter

        # plot_at_once plots and saves the figure for each sensor separately
        plot_at_once(plotgraph_list_each_parameter_collect)

        #parameter_plot_all_noise_each_parameter = getPlotList(sensor_data_plot_object, measuring_parameter="good_pixels", x_label="frame_number", y_label="good pixels / visible pixels")

        figures_bargraph_each_parameter_all_data = Figures(1) # only 1 figure for bar graph consisting of all details including multiple sensors

        flatten_bargraph_summary_list_each_parameter_collect = dict()
        for summary in bargraph_summary_list_each_parameter_collect:
            print "bar graph summary ", summary
            flatten_bargraph_summary_list_each_parameter_collect.update(summary)
        print len(flatten_bargraph_summary_list_each_parameter_collect)

        figures_bargraph_each_parameter_all_data.bargraph_pixel(flatten_bargraph_summary_list_each_parameter_collect, parameter )
        figures_bargraph_each_parameter_all_data.save_figure(parameter , "summary")

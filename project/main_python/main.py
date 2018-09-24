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



SCALE = 1


def plot_at_once(figures_plot_array_all):

    print "plot_at_once---------------------------"
    print figures_plot_array_all
    print "---------------------------"

    lower_x = 0; upper_x = 0; lower_y = 0; upper_y = 0;
    # set x and y_limits.
    for figures_plot_array in figures_plot_array_all:
            lower_x = min(figures_plot_array.get_x_axis_limits()[0], lower_x)
            upper_x = max(figures_plot_array.get_x_axis_limits()[1], upper_x)
            lower_y = min(figures_plot_array.get_y_axis_limits()[0], lower_y)
            upper_y = max(figures_plot_array.get_y_axis_limits()[1], upper_y)
            figures_plot_array.set_x_axis_limits([lower_x, upper_x])
            figures_plot_array.set_y_axis_limits([lower_y, upper_y])
    print "plot limits ", lower_x, upper_x, lower_y, upper_y

    # plot the figure in numpy cache.
    figures = Figures(1)
    figures.plot_all(figures_plot_array_all)

    # and lastly save the figure
    figures.save_figure(figures_plot_array_all[0].get_measuring_parameter(), figures_plot_array_all[0].get_algorithm(), figures_plot_array_all[0].get_step_size(), figures_plot_array_all[0].get_sensor_index())


def plot_at_once_summary(parameter, bargraph_summary_list_each_parameter_collect, extended=False):

    figures_bargraph_each_parameter_all_data = Figures(1) # only 1 figure for bar graph consisting of all details including multiple sensors

    flatten_bargraph_summary_list_each_parameter_collect = dict()
    for summary in bargraph_summary_list_each_parameter_collect:
        flatten_bargraph_summary_list_each_parameter_collect.update(summary)
    print flatten_bargraph_summary_list_each_parameter_collect

    figures_bargraph_each_parameter_all_data.bargraph_pixel(flatten_bargraph_summary_list_each_parameter_collect, parameter, extended )
    figures_bargraph_each_parameter_all_data.save_figure(parameter , "summary")

#    plt.close("all")

import subprocess

# first for ground truth and then for ground truth and the each noise condition. This is because in each plot
# both ground truth and noise is depicted.

if __name__ == '__main__':


    #concatenate files
    fd_concatenated = open(file_final, "w")
    fd_ground_truth = open(file_ground_truth, "r")
    fd_concatenated.write(fd_ground_truth.read())
    fd_concatenated.close()
    fd_ground_truth.close()

    for index, algorithm in enumerate(algorithm_list):
        fd_concatenated = open(file_final, "a")
        fd_algo = open(file_list[index], "r")
        fd_algo.readline() # remove header
        fd_algo.readline() # remove header

        fd_concatenated.write(fd_algo.read())
        fd_algo.close()

    fd_concatenated.close()

    yaml_file_handle = YAMLParser(file_final)
    yaml_file_data = yaml_file_handle.load()

    command = "rm " + output_folder + "*.png"
    try:
        subprocess.check_output([command], shell='/usr/bin/bash')
    except subprocess.CalledProcessError as e:
        print e.returncode
        if e.returncode != 1:
            exit(0)

    plotgraph_list_all_parameter_collect = list()

    for n, parameter in enumerate(parameter_list):
    # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        bargraph_summary_list_all_parameter_collect = list()
        print "PARAMETER ----- ", parameter

        for sensor_index in sensor_list:

            for algorithm in algorithm_list:

                for step_size in step_list:

                    sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm, parameter)

                    assert(noise_list[0] == "ground_truth")
                    print "---------------------------"

                    if just_ground_truth is True:
                        noise_list = ["ground_truth"]

                    for noise in noise_list:
                        if noise is "ground_truth":
                            plot_mapping = sensor_data_plot_object.templateToYamlMapping_GT()
                        else:
                            plot_mapping = sensor_data_plot_object.templateToYamlMapping(noise, step_size)

                        plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, plot_mapping, noise, str(step_size), datafilter_index=0, x_label="frame_number", y_label="dummy" )
                        plotgraph_list_all_parameter_collect.append(plot_data)

                        val = plot_data.get_summary()
                        bargraph_summary_list_all_parameter_collect.append(val)

                    if just_ground_truth is True:
                        break
                        
                print "---------------------------"

        # store the summary for future use:
        plot_at_once_summary(parameter, bargraph_summary_list_all_parameter_collect)

        # we are still in each parameter

    # plot_at_once plots and saves one figure for a single parameter per sensor
    # now make pairs.. whatever you want to get printed in one figure.
    # i will print total_pixel_gt and blue_sky and sroi gt and blue sky in one graph

    for configuration in configuration_list:
        reshape_plotgraph_list = list()
        for individual_plots in plotgraph_list_all_parameter_collect:
            if ( individual_plots.get_map_to_data() in configuration ):
                reshape_plotgraph_list.append(individual_plots)
        if (len(reshape_plotgraph_list) > 0 ):
            plot_at_once(reshape_plotgraph_list)

    plotgraph_list_all_parameter_collect_extended = list()

    for n, parameter in enumerate(parameter_list_extended):
        # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        bargraph_summary_list_all_parameter_collect = list()

        print "PARAMETER EXTENDED ----- ", parameter

        for algorithm in algorithm_list:

            for each_plot in plotgraph_list_all_parameter_collect:

                if ( each_plot.get_algorithm() is algorithm and  each_plot.get_sensor_index() == 0 ):

                    algorithm = each_plot.get_algorithm()
                    sensor_index = each_plot.get_sensor_index()
                    step_size = 1
                    env = each_plot.get_env_index()

                    if ( parameter[0] is each_plot.get_measuring_parameter()):
                        first_parameter = each_plot
                        print "parameter ", each_plot.get_measuring_parameter()
                        print "step size ", each_plot.get_step_size()
                        x_axis_1 = each_plot.get_x_axis()
                        y_axis_1 = each_plot.get_y_axis()
                    elif ( parameter[1] is each_plot.get_measuring_parameter()):
                        second_parameter = each_plot
                        print "parameter ", each_plot.get_measuring_parameter()
                        print "step size ", each_plot.get_step_size()
                        x_axis_2 = each_plot.get_x_axis()
                        y_axis_2 = each_plot.get_y_axis()

                if ( first_parameter is not None and second_parameter is not None ):
                    print "hurrah found data"
                    sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm, parameter)
                    print y_axis_1
                    print y_axis_2
                    new_y_axis = y_axis_1*100.0/y_axis_2
                    yaml_file_data = [x_axis_1, new_y_axis]

                    parameter_extended = "extended_" + parameter[0] + "_" + parameter[1]
                    plot_mapping = "extended"


                    plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, plot_mapping, parameter_extended, sensor_index, env, str(step_size), datafilter_index=0, x_label="frame_number", y_label="dummy" )
                    plotgraph_list_all_parameter_collect.append(plot_data)

                    val = plot_data.get_summary()
                    bargraph_summary_list_all_parameter_collect.append(val)

        plot_at_once_summary(parameter, bargraph_summary_list_all_parameter_collect)


    for configuration in configuration_list_extended:
        reshape_plotgraph_list = list()
        for individual_plots in plotgraph_list_all_parameter_collect:
            if ( individual_plots.get_map_to_data() in configuration ):
                reshape_plotgraph_list.append(individual_plots)
        if (len(reshape_plotgraph_list) > 0 ):
            plot_at_once(reshape_plotgraph_list)


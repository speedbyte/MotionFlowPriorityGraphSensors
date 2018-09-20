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
file_final = "/local/git/MotionFlowPriorityGraphSensors/project/main/values.yml"
file_ground_truth = "/local/git/MotionFlowPriorityGraphSensors/project/main/values_ground_truth.yml"
file_LK = "/local/git/MotionFlowPriorityGraphSensors/project/main/values_LK.yml"


SCALE = 1


def plot_at_once(figures_plot_array_all):

    print "plot_at_once---------------------------"
    print figures_plot_array_all
    print "plot_at_once---------------------------"

    lower_x = 0; upper_x = 0; lower_y = 0; upper_y = 0;
    # set x and y_limits.
    for figures_plot_array in figures_plot_array_all:
        for figures_plot_index in figures_plot_array:
                lower_x = min(figures_plot_index.get_x_axis_limits()[0], lower_x)
                upper_x = max(figures_plot_index.get_x_axis_limits()[1], upper_x)
                lower_y = min(figures_plot_index.get_y_axis_limits()[0], lower_y)
                upper_y = max(figures_plot_index.get_y_axis_limits()[1], upper_y)
        for figures_plot_index in figures_plot_array:
            figures_plot_index.set_x_axis_limits([lower_x, upper_x])
            figures_plot_index.set_y_axis_limits([lower_y, upper_y])
    print lower_x, upper_x, lower_y, upper_y

    # plot the figure in numpy cache.
    for figures_plot_array in figures_plot_array_all:
        figures = Figures(1)
        figures.plot_all(figures_plot_array)

        # and lastly save the figure
        figures.save_figure(figures_plot_array[0].get_measuring_parameter(), figures_plot_array[0].get_algorithm(), figures_plot_array[0].get_step_size(), figures_plot_array_all[0][0].get_sensor_index())


def plot_at_once_summary(bargraph_summary_list_each_parameter_collect, parameter, extended=False):
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

    fd_concatenated = open(file_final, "a")
    fd_lk = open(file_LK, "r")
    fd_lk.readline() # remove header
    fd_lk.readline() # remove header

    fd_concatenated.write(fd_lk.read())
    fd_lk.close()
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
    bargraph_summary_list_all_parameter_collect = list()

    for n, parameter in enumerate(parameter_list):
    # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        print "PARAMETER ----- ", parameter

        plotgraph_list_each_parameter_collect = list()
        bargraph_summary_list_each_parameter_collect = list()

        for sensor_index in sensor_list:

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
                        plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, plot_mapping[index], parameter, sensor_index, env, str(step_size), 0, x_label="frame_number", y_label="dummy" )

                        parameter_plot_all_noise_each_parameter.append(plot_data)

                    if just_ground_truth is True:
                        break
                        
                print "---------------------------"

                # store the summary for future use:
                plotgraph_list_each_parameter_collect.append(parameter_plot_all_noise_each_parameter)
                bargraph_summary_list_each_parameter_collect.append(sensor_data_plot_object.get_summary())

        # we are still in each parameter
        plotgraph_list_all_parameter_collect.append(plotgraph_list_each_parameter_collect)
        bargraph_summary_list_all_parameter_collect.append([bargraph_summary_list_each_parameter_collect, parameter])

    # plot_at_once plots and saves one figure for a single parameter per sensor
    if ( 1 ):
        for each_plot in plotgraph_list_all_parameter_collect:
            plot_at_once(each_plot)
        # plot_at_once_summary plots and saves one figure for a single parameter per sensor
        for each_plot_bar in bargraph_summary_list_all_parameter_collect:
            parameter = each_plot_bar[1]
            plot_at_once_summary(each_plot_bar[0], parameter)




    plotgraph_list_all_parameter_collect_extended = list()
    bargraph_summary_list_all_parameter_collect_extended = list()


    for n, parameter in enumerate(parameter_list_extended):
        # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        parameter_plot_all_noise_each_parameter = list()
        plotgraph_list_each_parameter_collect = list()

        bargraph_summary_list_each_parameter_collect = list()

        print "PARAMETER EXTENDED ----- ", parameter

        for algoirhtm in algorithm_list:
            for each_plot in plotgraph_list_all_parameter_collect:
                for figures_plot_array in each_plot:

                    for final_data in figures_plot_array:

                        if ( final_data.get_algorithm() is algorithm and  final_data.get_sensor_index() == 0 ):

                            algorithm = final_data.get_algorithm()
                            sensor_index = final_data.get_sensor_index()
                            step_size = 1
                            env = final_data.get_env_index()

                            if ( parameter[0] is final_data.get_measuring_parameter()):
                                first_parameter = figures_plot_array
                                print "parameter ", final_data.get_measuring_parameter()
                                print "step size ", final_data.get_step_size()
                                x_axis_1 = final_data.get_x_axis()
                                y_axis_1 = final_data.get_y_axis()
                            elif ( parameter[1] is final_data.get_measuring_parameter()):
                                second_parameter = figures_plot_array
                                print "parameter ", final_data.get_measuring_parameter()
                                print "step size ", final_data.get_step_size()
                                x_axis_2 = final_data.get_x_axis()
                                y_axis_2 = final_data.get_y_axis()

            if ( first_parameter is not None and second_parameter is not None ):
                print "hurrah found data"
                sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm)
                sensor_data_plot_object.set_measuring_parameter(parameter)
                print y_axis_1
                print y_axis_2
                new_y_axis = y_axis_1*100.0/y_axis_2
                yaml_file_data = [x_axis_1, new_y_axis]

                parameter_extended = "extended_" + parameter[0] + "_" + parameter[1]
                plot_mapping[0] = "extended"

                plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, plot_mapping[0], parameter_extended, sensor_index, env, str(step_size), 0, x_label="frame_number", y_label="dummy" )

                parameter_plot_all_noise_each_parameter.append(plot_data)
                plotgraph_list_each_parameter_collect.append(parameter_plot_all_noise_each_parameter)
                plotgraph_list_all_parameter_collect_extended.append(plotgraph_list_each_parameter_collect)

                bargraph_summary_list_each_parameter_collect.append(sensor_data_plot_object.get_summary())
                bargraph_summary_list_all_parameter_collect_extended.append([bargraph_summary_list_each_parameter_collect, parameter_extended])


    if ( 1 ):
        for each_plot in plotgraph_list_all_parameter_collect_extended:
            plot_at_once(each_plot)
        for each_plot_bar in bargraph_summary_list_all_parameter_collect_extended:
            parameter = each_plot_bar[1]
            plot_at_once_summary(each_plot_bar[0], parameter, True)

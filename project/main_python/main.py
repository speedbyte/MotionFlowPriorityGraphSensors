#!/usr/bin/env python
# _*_ encoding=utf-8 _*_


import numpy
import matplotlib
matplotlib.use('Agg')

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


def plot_at_once(figures_plot_array_all, sensor_index):

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

    for figures_plot_index_all in range(len(figures_plot_array_all)):
        figures = Figures(1)
        figures_plot_array = figures_plot_array_all[figures_plot_index_all]
        #for figures_plot_index in figures_plot_array:
        figures.plot_all(figures_plot_array)

        figures.save_figure(figures_plot_array[0].get_measuring_parameter(), figures_plot_array[0].get_algorithm(), figures_plot_array[0].get_step_size(), sensor_index)



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
        summary_list = list()

        for sensor_index in sensor_list:

            parameter_plots_with_details = list()

            for algorithm in algorithm_list:

                sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm)
                sensor_data_plot_object.set_measuring_parameter(parameter)
                plot_mapping_gt = sensor_data_plot_object.templateToYamlMapping_GT()

                parameter_plot_at_once_figures = list()

                for step_size in step_list:

                    for n, noise in enumerate(noise_list):

                        print "---------------------------"
                        custom_data_list_name = list()

                        plot_mapping_noise = sensor_data_plot_object.templateToYamlMapping(noise, step_size)

                        plot_mapping = [plot_mapping_gt, plot_mapping_noise]

                        if noise == "ground_truth":
                            environment = ["ground_truth"]
                        else:
                            environment = ["ground_truth", noise]

                        for index,env in enumerate(environment):

                            custom_data_list_name.append(plot_mapping[index])
                            plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, custom_data_list_name, parameter, env, str(step_size), 0, x_label="current_frame_index", y_label=y_axis_label_dict[parameter] )
                            parameter_plot_at_once_figures.append(plot_data)

                        print custom_data_list_name
                    print "---------------------------"

                parameter_plots_with_details.append(parameter_plot_at_once_figures)

                # summary
                summary_list.append(sensor_data_plot_object.get_summary())
                print len(sensor_data_plot_object.get_summary())

            # plotting the figure for each sensor separately
            plot_at_once(parameter_plots_with_details, 0) #plot_at_once(parameter_plot_at_once_figures, sensor_data_plot_object.getSensorIndex())
            #parameter_plot_at_once_figures = getPlotList(sensor_data_plot_object, measuring_parameter="good_pixels", x_label="current_frame_index", y_label="good pixels / visible pixels")
            

        figures = Figures(1) # only 1 figure for bar graph consisting of all details including multiple sensors

        flatten_summary_list = dict()
        for summary in summary_list:
            flatten_summary_list.update(summary)
        print len(flatten_summary_list)

        figures.bargraph_pixel(flatten_summary_list, parameter )
        figures.save_figure(parameter , "summary")


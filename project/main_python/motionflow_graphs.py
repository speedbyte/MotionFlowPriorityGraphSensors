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


def plot_at_once(figures_plot_array, sensor_index):

    print "---------------------------"

    lower_x = 0; upper_x = 0; lower_y = 0; upper_y = 0;
    print figures_plot_array

    figures = Figures(1)

    for figures_plot_index in figures_plot_array:
            lower_x = min(figures_plot_index.get_x_axis_limits()[0], lower_x)
            upper_x = max(figures_plot_index.get_x_axis_limits()[1], upper_x)
            lower_y = min(figures_plot_index.get_y_axis_limits()[0], lower_y)
            upper_y = max(figures_plot_index.get_y_axis_limits()[1], upper_y)

    for figures_plot_index in figures_plot_array:
        figures_plot_index.set_x_axis_limits([lower_x, upper_x])
        figures_plot_index.set_y_axis_limits([lower_y, upper_y])

    print lower_x, upper_x, lower_y, upper_y

    figures.plot_all(figures_plot_array)

    figures.save_figure(figures_plot_array[0].get_measuring_parameter(), figures_plot_array[0].get_algorithm(), figures_plot_array[0].get_step_size(), sensor_index)


def getPlotList(sensor_plot, measuring_parameter, x_label, y_label):


    plot_at_once_figures = list()

    for step_size in step_list:

        for n, weather in enumerate(weather_list):

            print "---------------------------"
            custom_data_list_name = list()

            plot_mapping = sensor_plot.templateToYamlMapping_GT(measuring_parameter)
            custom_data_list_name.append(plot_mapping)
            plot_data = sensor_plot.extract_plot_data_from_data_list(yaml_file_data, custom_data_list_name, measuring_parameter, algorithm_list[0], "ground_truth", str(step_size), 0, x_label, y_label )

            if ( weather != "ground_truth"):
                plot_mapping = sensor_plot.templateToYamlMapping(measuring_parameter, weather, step_size)
                custom_data_list_name.append(plot_mapping)
                plot_data = sensor_plot.extract_plot_data_from_data_list(yaml_file_data, custom_data_list_name, measuring_parameter, algorithm_list[0], weather, str(step_size), 0, x_label, y_label)
                #print plot_data.get_x_axis()

            print custom_data_list_name
            plot_at_once_figures.append(plot_data)

        print "---------------------------"

    return plot_at_once_figures

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

    for x in sensor_list:

        sensor_plot = SensorDataPlot(x)

        plot_at_once_figures = getPlotList(sensor_plot, "visible_pixels", "frame_count", "visible pixels / ground truth pixels")
        collectPlots.append(plot_at_once_figures)
        plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

        # summary
        if ( 0 ):
            summary = sensor_plot.get_summary()
            figures = Figures(1)
            figures.evaluate_pixel(summary)
            figures.save_figure("visible_pixels", "summary", step_list[0], x)

        plot_at_once_figures = getPlotList(sensor_plot, "good_pixels", "frame_count", "good pixels / visible pixels")
        collectPlots.append(plot_at_once_figures)
        plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())



#!/usr/bin/env python
# _*_ encoding=utf-8 _*_


from Figures import Figures
from YAMLParser import YAMLParser
from SensorDataPlot import SensorDataPlot
from motionflow_graphs_data import *
import math
import numpy as np

SCALE = 1

def plot_at_once_pointgraph(figures_plot_array_all, display_list_index, dual = False, multiple = False):

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
    figures.pointgraph_pixel(figures_plot_array_all)

    # and lastly save the figure
    if ( dual ):
        suffix = "point_graph_dual"
    elif ( multiple ):
        suffix = "point_graph_multiple"
    else:
        suffix = "point_graph"

    figures.save_figure(suffix, str(display_list_index), figures_plot_array_all[0].get_step_size(), figures_plot_array_all[0].get_sensor_index())


def plot_at_once_bargraph(parameter, bargraph_list_each_parameter_collect, dual=False, multiple=False):

    figures_bargraph_each_parameter_all_data = Figures(1) # only 1 figure for bar graph consisting of all details including multiple sensors

    flatten_bargraph_list_each_parameter_collect = dict()
    for summary in bargraph_list_each_parameter_collect:
        flatten_bargraph_list_each_parameter_collect.update(summary)
    print flatten_bargraph_list_each_parameter_collect

    figures_bargraph_each_parameter_all_data.bargraph_pixel(parameter, flatten_bargraph_list_each_parameter_collect, dual )
    figures_bargraph_each_parameter_all_data.save_figure(parameter, "bar_graph")

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

    for file_to_append in file_list:
        fd_concatenated = open(file_final, "a")
        fd_algo = open(file_to_append, "r")
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

        bargraph_list_all_parameter_collect = list()
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
                        bargraph_list_all_parameter_collect.append(val)

                    if just_ground_truth is True:
                        break

                print "---------------------------"

        # store the summary for future use:
        for configuration in display_list_bargraph:
            if ( parameter in configuration ):
                plot_at_once_bargraph(parameter, bargraph_list_all_parameter_collect, False)
                break
            else:
                print parameter + " is not found in the configuration list"

        # we are still in each parameter

    # plot_at_once plots and saves one figure for a single parameter per sensor
    # now make pairs.. whatever you want to get printed in one figure.
    # i will print total_pixel_gt and blue_sky and sroi gt and blue sky in one graph

    plotgraph_list_all_parameter_dual_collect = list()

    for n, parameter in enumerate(parameter_list_dual):
        # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

        bargraph_list_all_parameter_dual_collect = list()
        parameter_dual = "dual_" + parameter[0] + "_" + parameter[1]

        print "PARAMETER dual ----- ", parameter_dual

        for sensor_index in sensor_list:

            for algorithm in algorithm_list:

                for noise in noise_list:

                    for each_plot in plotgraph_list_all_parameter_collect:

                        # set algorithm wise
                        if ( each_plot.get_algorithm() is algorithm and  each_plot.get_sensor_index() == 0 and each_plot.get_noise() is noise ):

                            algorithm = each_plot.get_algorithm()
                            sensor_index = each_plot.get_sensor_index()
                            step_size = 1
                            noise = each_plot.get_noise()

                            if ( parameter[0] is each_plot.get_measuring_parameter()):
                                first_parameter = True
                                print "map_data ", each_plot.get_map_to_data()
                                x_axis_1 = each_plot.get_x_axis()
                                y_axis_1 = each_plot.get_y_axis()

                            elif ( parameter[1] is each_plot.get_measuring_parameter()):
                                second_parameter = True
                                print "map_data ", each_plot.get_map_to_data()
                                x_axis_2 = each_plot.get_x_axis()
                                y_axis_2 = each_plot.get_y_axis()

                    if ( first_parameter is True and second_parameter is True ):
                        print "hurrah found data"

                        sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm, parameter_dual)
                        print y_axis_1
                        print y_axis_2
                        ### formula #######
                        new_y_axis = y_axis_1*100.0/y_axis_2
                        yaml_file_data = [x_axis_1, new_y_axis]

                        plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, parameter_dual, noise, str(step_size), datafilter_index=0, x_label="frame_number", y_label="dummy" )
                        plotgraph_list_all_parameter_dual_collect.append(plot_data)

                        val = plot_data.get_summary()
                        bargraph_list_all_parameter_dual_collect.append(val)


        # store the summary for future use:
        for configuration in display_list_bargraph_dual:
            if ( parameter_dual in configuration ):
                plot_at_once_bargraph(parameter_dual, bargraph_list_all_parameter_dual_collect, True)
                break
            else:
                print parameter_dual + " is not found in the configuration list"


    # calculate consistency

    plotgraph_list_all_parameter_multiple_collect = list()

    for sensor_index in sensor_list:

        for algorithm in algorithm_list:

            for noise in noise_list:

                for each_plot in plotgraph_list_all_parameter_collect:

                    # set algorithm wise
                    if ( each_plot.get_algorithm() is algorithm and  each_plot.get_sensor_index() == 0 and each_plot.get_noise() is noise ):

                        algorithm = each_plot.get_algorithm()
                        sensor_index = each_plot.get_sensor_index()
                        step_size = 1
                        noise = each_plot.get_noise()

                        if ( each_plot.get_measuring_parameter() == "eroi_l2_good_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            eroi_l2_good_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "eroi_ma_good_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            eroi_ma_good_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "eroi_all_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            eroi_all_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "sroi_all_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            sroi_all_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "sroi_l2_good_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            sroi_l2_good_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "distribution_matrix"):
                            x_axis_1 = each_plot.get_x_axis()
                            distribution_matrix = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "sroi_l2_cumulative_error_all_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            sroi_l2_cumulative_error_all_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "eroi_l2_cumulative_error_all_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            eroi_l2_cumulative_error_all_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "sroi_l2_cumulative_error_good_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            sroi_l2_cumulative_error_good_pixels = each_plot.get_y_axis()
                        elif ( each_plot.get_measuring_parameter() == "eroi_l2_cumulative_error_good_pixels"):
                            x_axis_1 = each_plot.get_x_axis()
                            eroi_l2_cumulative_error_good_pixels = each_plot.get_y_axis()




                for n, parameter in enumerate(parameter_list_multiple):
                    # do for each parameter one by one. each parameter takes ground truth and all other factors such as noise, type of algorithm etc.

                    parameter_multiple = "multiple_" + parameter[0]

                    print "PARAMETER multiple ----- ", parameter_multiple

                    if ( parameter_multiple == "multiple_nucleus_consistency" ):
                        nucleus_consistency = (( eroi_l2_good_pixels - eroi_ma_good_pixels ) * 1.0 / eroi_all_pixels )
                        ### formula #######
                        new_y_axis = 1/np.log(nucleus_consistency)
                        print "nucleus",

                    elif ( parameter_multiple == "multiple_occlusion_consistency" ):
                        occlusion_consistency = 1/np.exp( (sroi_l2_cumulative_error_all_pixels * 1.0 / eroi_l2_cumulative_error_all_pixels) - (sroi_all_pixels * 1.0 / eroi_all_pixels) )
                        ### formula #######
                        new_y_axis = (occlusion_consistency)
                        print "occlusion",  occlusion_consistency

                    sensor_data_plot_object = SensorDataPlot(sensor_index, algorithm, parameter_multiple)
                    yaml_file_data = [x_axis_1, new_y_axis]

                    plot_data = sensor_data_plot_object.extract_plot_data_from_data_list(yaml_file_data, parameter_multiple, noise, str(step_size), datafilter_index=0, x_label="frame_number", y_label="dummy" )
                    plotgraph_list_all_parameter_multiple_collect.append(plot_data)



    for n, parameter in enumerate(parameter_list_multiple):
        bargraph_list_all_parameter_multiple_collect = list()
        parameter_multiple = "multiple_" + parameter[0]

        for sensor_index in sensor_list:

            for algorithm in algorithm_list:

                for noise in noise_list:

                    for each_plot in plotgraph_list_all_parameter_multiple_collect:

                        if ( each_plot.get_measuring_parameter() == parameter_multiple):
                            val = each_plot.get_summary()
                            bargraph_list_all_parameter_multiple_collect.append(val)

        # store the summary for future use:
        for configuration in display_list_bargraph_multiple:
            if ( parameter_multiple in configuration ):
                plot_at_once_bargraph(parameter_multiple, bargraph_list_all_parameter_multiple_collect, True)
                break
            else:
                print parameter_multiple + " is not found in the configuration list"



    for index, configuration in enumerate(display_list):
        reshape_plotgraph_list = list()
        for individual_plots in plotgraph_list_all_parameter_collect:
            if ( individual_plots.get_map_to_data() in configuration ):
                reshape_plotgraph_list.append(individual_plots)
        if (len(reshape_plotgraph_list) > 0 ):
            plot_at_once_pointgraph(reshape_plotgraph_list, index)
        else:
            print str(configuration) + " is not found !!!!!! "


    for index, configuration in enumerate(display_list_dual):
        reshape_plotgraph_list = list()
        for individual_plots in plotgraph_list_all_parameter_dual_collect:
            if ( individual_plots.get_map_to_data() in configuration ):
                reshape_plotgraph_list.append(individual_plots)
        if (len(reshape_plotgraph_list) > 0 ):
            plot_at_once_pointgraph(reshape_plotgraph_list, index, True)
        else:
            print configuration + " is not found !!!!!! "


    for index, configuration in enumerate(display_list_multiple):
        reshape_plotgraph_list = list()
        for individual_plots in plotgraph_list_all_parameter_multiple_collect:
            if ( individual_plots.get_map_to_data() in configuration ):
                reshape_plotgraph_list.append(individual_plots)
        if (len(reshape_plotgraph_list) > 0 ):
            plot_at_once_pointgraph(reshape_plotgraph_list, index, False, True)
        else:
            print configuration + " is not found !!!!!! "

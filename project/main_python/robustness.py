

import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

from robustness_common import *


summary_mean = dict()

def robustness_(file, measuring_parameter, noise, stepSize, data_list, color_list, label=""):

    #summary_mean.clear()

    figures_plot = list()

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #axes limits
    lower_x = 0; upper_x = 0;
    lower_y = 0; upper_y = 0;

    # Ground Truth
    if ( measuring_parameter == "deviation"):
        data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
        print "getting " , list_of_collision_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getDeviationPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "pixel"):
        data_points_gt = yaml_load[list_of_pixel_density_ground_truth[0]]
        print "getting " , list_of_pixel_density_ground_truth[yaml_list_index_offset]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getShape(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "collision"):
        data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
        print "getting " , list_of_collision_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getCollisionPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "obj_displacement"):
        data_points_gt = yaml_load[list_of_obj_displacement_ground_truth[0]]
        print "getting " , list_of_obj_displacement_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getObjectDisplacement(data_points_gt, data_points_gt)

    for env_name in environment_list:

        print "Table " + measuring_parameter + " robustness with " + env_name + " " + noise

        mean_list = list()
        mean_list.append(y_axis_gt_mean)

        list_of_plots = list()

        x_axis_list = list()
        y_axis_list = list()
        x_axis_list.append(x_axis_gt)
        y_axis_list.append(y_axis_gt)

        lower_x = min(numpy.nanmin(x_axis_gt), lower_x)
        upper_x = max(numpy.nanmax(x_axis_gt), upper_x)

        lower_y = min(numpy.nanmin(y_axis_gt), lower_y)
        upper_y = max(numpy.nanmax(y_axis_gt), upper_y)


        for datafilter_index in range(len(datafilter_list)):

            if ( noise == "no_noise"):
                data_points = yaml_load[data_list[yaml_list_index_offset+1+datafilter_index]]
                print "getting ", data_list[yaml_list_index_offset+1+datafilter_index]
                if ( measuring_parameter == "deviation"):
                    x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "pixel"):
                    x_axis, y_axis, y_axis_mean = getShape(data_points_gt, data_points)
                elif ( measuring_parameter == "collision"):
                    x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "obj_displacement"):
                    x_axis, y_axis, y_axis_mean = getObjectDisplacement(data_points_gt, data_points)

                x_axis_list.append(x_axis)
                y_axis_list.append(y_axis)

            else:

                data_points = yaml_load[data_list[yaml_list_index_offset+datafilter_index]]
                print "getting " , data_list[yaml_list_index_offset+datafilter_index]
                if ( measuring_parameter == "deviation"):
                    x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "pixel"):
                    x_axis, y_axis, y_axis_mean = getShape(data_points_gt, data_points)
                elif ( measuring_parameter == "collision"):
                    x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "obj_displacement"):
                    x_axis, y_axis, y_axis_mean = getObjectDisplacement(data_points_gt, data_points)

                x_axis_list = [x_axis]
                y_axis_list = [y_axis]
                plot1 = ['x_axis',
                         'y_axis',
                         x_axis_list,
                         y_axis_list,
                         color_list,
                         data_list,
                         measuring_parameter + " " + dict_datafilters["datafilter_" + str(datafilter_index)] + " step size" + " " + str(stepSize),
                         [lower_x, upper_x],
                         [lower_y, upper_y],
                         ]

            if ( noise == "noise"):
                list_of_plots.append(plot1)

            mean_list.append(y_axis_mean)

            lower_x = min(numpy.nanmin(x_axis), lower_x)
            upper_x = max(numpy.nanmax(x_axis), upper_x)

            lower_y = min(numpy.nanmin(y_axis), lower_y)
            upper_y = max(numpy.nanmax(y_axis), upper_y)

        if ( noise == "no_noise"):
            plot1 = ['x_axis',
                     'y_axis',
                     x_axis_list,
                     y_axis_list,
                     color_list,
                     data_list,
                     measuring_parameter + " all datafilters " + " step size" + " " + str(stepSize),
                     [lower_x, upper_x],
                     [lower_y, upper_y],
                     ]

            list_of_plots.append(plot1)


        if noise == "noise":
            # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
            summary_mean[measuring_parameter + '_' + env_name + '_' + str(stepSize) ] = mean_list

        yaml_list_index_offset = yaml_list_index_offset + 4

        figures_plot.append((list_of_plots, dict_environment[env_name], measuring_parameter, noise, stepSize, lower_x, upper_x, lower_y, upper_y))
        if (noise == "no_noise" ):
            break

    return figures_plot


def get_summary():
    return summary_mean








import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

from robustness_common import *

summary_mean = dict()



def robustness_(file, measuring_parameter, noise, data_list, color_list, label):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #axes limits
    lower = 0; upper = 0;

    # Ground Truth
    if ( measuring_parameter == "deviation"):
        data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
        print "getting " , list_of_collision_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getDeviationPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "pixel"):
        data_points_gt = yaml_load[list_of_pixel_density_ground_truth[0]]
        print "getting " , list_of_pixel_density_ground_truth[yaml_list_index_offset]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getNewShape(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "collision"):
        data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
        print "getting " , list_of_collision_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getCollisionPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "obj_displacement"):
        data_points_gt = yaml_load[list_of_obj_displacement_ground_truth[0]]
        print "getting " , list_of_obj_displacement_ground_truth[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getObjectDisplacement(data_points_gt, data_points_gt)

    if ( noise == "no_noise" ):
        figures = Figures(1)
    else:
        figures = Figures(len(data_processing_list))

    for env_index in range(len(environment_list)):

        print "Table " + measuring_parameter + " robustness with " + environment_list[env_index] + " " + noise

        mean_list = list()
        mean_list.append(y_axis_gt_mean)

        list_of_plots = list()

        x_axis_list = list()
        y_axis_list = list()
        x_axis_list.append(x_axis_gt)
        y_axis_list.append(y_axis_gt)

        for x in range(len(data_processing_list)):

            if ( noise == "no_noise"):
                data_points = yaml_load[data_list[yaml_list_index_offset+1+x]]
                print "getting ", data_list[yaml_list_index_offset+1+x]
                if ( measuring_parameter == "deviation"):
                    x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "pixel"):
                    x_axis, y_axis, y_axis_mean = getNewShape(data_points_gt, data_points)
                else:
                    x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points_gt, data_points)

                x_axis_list.append(x_axis)
                y_axis_list.append(y_axis)

            else:

                data_points = yaml_load[data_list[yaml_list_index_offset+x]]
                print "getting " , data_list[yaml_list_index_offset+x]
                if ( measuring_parameter == "deviation"):
                    x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)
                elif ( measuring_parameter == "pixel"):
                    x_axis, y_axis, y_axis_mean = getNewShape(data_points_gt, data_points)
                else:
                    x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points_gt, data_points)

                x_axis_list = [x_axis]
                y_axis_list = [y_axis]

                plot1 = ['x_axis',
                         'y_axis',
                         x_axis_list,
                         y_axis_list,
                         color_list,
                         data_list,
                         label + str(x),
                         lower,
                         upper,
                         ]

            if ( noise == "noise"):
                list_of_plots.append(plot1)

            mean_list.append(y_axis_mean)


            lower = min(numpy.nanmin(y_axis), lower)
            upper = max(numpy.nanmax(y_axis), upper)

        if ( noise == "no_noise"):
            plot1 = ['x_axis',
                     'y_axis',
                     x_axis_list,
                     y_axis_list,
                     color_list,
                     data_list,
                     label,
                     lower,
                     upper,
                     ]

            list_of_plots.append(plot1)

        if measuring_parameter == "pixel":
            lower=0; upper=1;

        for x in range(len(list_of_plots)):
            list_of_plots[x][7] = lower
            list_of_plots[x][8] = upper

        summary_mean[measuring_parameter + '_' + str(env_index)] = mean_list

        yaml_list_index_offset=yaml_list_index_offset+4

        print "env ", env_index
        figures.plot_all(list_of_plots, env_index)
        if (noise == "no_noise" ):
            break

    figures.save_figure(measuring_parameter, noise)


def get_summary():
    return summary_mean






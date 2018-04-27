

import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

OUTLIER = 100000
summary_mean = dict()

def getDeviationPoints(data_points_gt, data_points ):

    data = list()

    for count in range(len(data_points_gt)):
        xy = list()
        xy.append(data_points_gt[count]["x"])
        xy.append(data_points_gt[count]["y"])
        data.append(xy)

    data = numpy.array(data)
    x0_gt, y0_gt = data.T

    frame_count = numpy.arange(0.0, len(data_points), 1)

    data = list()

    for count in range(len(data_points)):
        xy = list()
        xy.append(data_points[count]["x"])
        xy.append(data_points[count]["y"])
        data.append(xy)

    y_axis_mean = 0
    data = numpy.array(data)
    x0, y0 = data.T
    x_axis = frame_count
    y_axis = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2)

    count = 0

    for n,i in enumerate(y_axis):
        if ( abs(i) > OUTLIER):
            y0[n] = y0[n-1]
            if ( n == 0 ):
                y0[n] = 0
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    for n,i in enumerate(x_axis):
        if ( abs(i) > OUTLIER):
            x0[n] = x0[n-1]
            if ( n == 0 ):
                x0[n] = 0

    y_axis_mean = y_axis_mean/(count)
    return x_axis, y_axis, y_axis_mean


def getCollisionPoints(data_points_gt, data_points):

    data = list()

    for count in range(len(data_points)):
        xy = list()
        xy.append(data_points[count]["x"])
        xy.append(data_points[count]["y"])
        data.append(xy)

    y_axis_mean = 0
    data = numpy.array(data)
    x0, y0 = data.T
    x_axis = x0
    y_axis = y0

    count = 0

    for n,i in enumerate(y_axis):
        if ( abs(i) > OUTLIER):
            y0[n] = y0[n-1]
            if ( n == 0 ):
                y0[n] = 0
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    for n,i in enumerate(x_axis):
        if ( abs(i) > OUTLIER):
            x0[n] = x0[n-1]
            if ( n == 0 ):
                x0[n] = 0

    y_axis_mean = y_axis_mean/(count)
    return x_axis, y_axis, y_axis_mean



def getNewShape(data_points_gt, data_points):

    data = list()

    for count in range(len(data_points_gt)):
        xy = list()
        xy.append(data_points[count]["good_pixels"][0])
        xy.append(data_points[count]["total_pixels"][0])
        xy.append(data_points[count]["total_pixels"][1])
        data.append(xy)

    newshape = list()
    previous_x_axis = 0
    frame_good_pixels = 0
    frame_total_pixels = 0
    total_count = 0
    for count in range(len(data)):

        if ( data[count][0] != previous_x_axis ):

            previous_x_axis = data[count][0]
            xy = list()
            xy.append(frame_good_pixels/total_count)
            xy.append(frame_total_pixels/total_count)
            newshape.append(xy)
            total_count = 0
            frame_good_pixels = 0
            frame_total_pixels = 0

        if ( data[count][0] == previous_x_axis ):
            total_count = total_count+1
            frame_good_pixels = frame_good_pixels + data[count][1]
            frame_total_pixels = frame_total_pixels + data[count][2]

        if ( count == len(data)-1 ):

            xy = list()
            xy.append(frame_good_pixels/total_count)
            xy.append(frame_total_pixels/total_count)
            newshape.append(xy)

    y_axis_mean = 0
    data = numpy.array(newshape)
    x0_gt, y0_gt = data.T

    data = list()

    for count in range(len(data_points)):
        xy = list()
        xy.append(data_points[count]["good_pixels"][0])
        xy.append(data_points[count]["total_pixels"][0])
        xy.append(data_points[count]["total_pixels"][1])
        data.append(xy)

    newshape = list()
    previous_x_axis = 0
    frame_good_pixels = 0
    frame_total_pixels = 0
    total_count = 0
    for count in range(len(data)):

        if ( data[count][0] != previous_x_axis ):

            previous_x_axis = data[count][0]
            xy = list()
            xy.append(frame_good_pixels/total_count)
            xy.append(frame_total_pixels/total_count)
            newshape.append(xy)
            total_count = 0
            frame_good_pixels = 0
            frame_total_pixels = 0

        if ( data[count][0] == previous_x_axis ):
            total_count = total_count+1
            frame_good_pixels = frame_good_pixels + data[count][1]
            frame_total_pixels = frame_total_pixels + data[count][2]

        if ( count == len(data)-1 ):

            xy = list()
            xy.append(frame_good_pixels/total_count)
            xy.append(frame_total_pixels/total_count)
            newshape.append(xy)

    y_axis_mean = 0
    data = numpy.array(newshape)
    x0, y0 = data.T
    y_axis = x0/y0_gt

    count = 0
    for n,i in enumerate(y_axis):
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    y_axis_mean = y_axis_mean/(count)
    x_axis = numpy.arange(0.0, len(newshape), 1)
    return x_axis, y_axis, y_axis_mean



def robustness_(file, measuring_parameter, noise, data_list, color_list, label):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #axes limits
    lower = 0; upper = 0;


    # Ground Truth
    if ( measuring_parameter == "deviation"):
        data_points_gt = yaml_load[list_of_collision_no_noise[0]]
        print "getting " , list_of_collision_no_noise[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getDeviationPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "pixel"):
        data_points_gt = yaml_load[list_of_shape_no_noise[0]]
        print "getting " , list_of_shape_no_noise[yaml_list_index_offset]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getNewShape(data_points_gt, data_points_gt)

    else:
        data_points_gt = yaml_load[list_of_collision_no_noise[0]]
        print "getting " , list_of_collision_no_noise[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = getCollisionPoints(data_points_gt, data_points_gt)

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






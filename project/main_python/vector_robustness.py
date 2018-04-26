

import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

OUTLIER = 10000


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
    print y_axis
    return x_axis, y_axis, y_axis_mean


def getCollisionPoints(data_points):

    data = list()

    frame_count = numpy.arange(0.0, len(data_points), 1)
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
    print y_axis
    return x_axis, y_axis, y_axis_mean


def deviationgraphs_no_noise(file):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #plotters
    x_axis_list = list()
    y_axis_list = list()

    #means
    mean_list = list()

    lower = 0; upper = 0;

    print "Table 3 Vector Robustness with " + environment_list[0] + " noise "
    data_points_gt = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset]]

    for x in range(len(data_processing_list)):

        data_points = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset+1+x]]

        x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)

        mean_list.append(y_axis_mean)

        x_axis_list.append(x_axis)
        y_axis_list.append(y_axis)

        lower = min(numpy.nanmin(y_axis), lower)
        upper = max(numpy.nanmax(y_axis), upper)


    plot1 = ('x_axis',
             'deviation [no_noise_points-groundtruth_points]',
             x_axis_list,
             y_axis_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "deviation_pointsframe_skip1_dataprocessing_0",
             min,
             upper,
             )

    print mean_list

    figures = Figures(1)
    figures.plot_all([plot1], 0)
    figures.plot_show_vector_dev_no_noise()


def collisiongraphs_no_noise(file):


    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #plotters
    x_axis_list = list()
    y_axis_list = list()

    #means
    mean_list = list()

    lower = 0; upper = 0;

    print "Table 3 Collision with " + environment_list[0] + " noise "
    data_points = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset]]
    x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points)

    mean_list.append(y_axis_mean)

    x_axis_list.append(x_axis)
    y_axis_list.append(y_axis)

    yaml_list_index_offset=0

    for x in range(len(data_processing_list)):

        data_points = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset+1+x]]
        x_axis, y_axis, y_axis_mean = getCollisionPoints(data_points)

        mean_list.append(y_axis_mean)

        x_axis_list.append(x_axis)
        y_axis_list.append(y_axis)

        lower = min(numpy.nanmin(y_axis), lower)
        upper = max(numpy.nanmax(y_axis), upper)

    plot2 = ('X', 'Y',
             x_axis_list,
             y_axis_list,
             color_of_collision_metrics_no_noise,
             list_of_collision_metrics_no_noise,
             "collision_pointsframe_skip1_dataprocessing_0",
             lower,
             upper,
             )

    print mean_list

    figures = Figures(1)
    figures.plot_all([plot2], 0)
    figures.plot_show_vector_coll_no_noise()



def vectorgraphs_noise():

    offset=1
    yaml_list_index_offset=0


    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    lower = 0; upper = 0;

    x0_base_list = list(); y0_base_list = list();

    for x in range(len(data_processing_list)):

        data_points = yaml_load[list_of_collision_metrics_noise[yaml_list_index_offset+x]]
        data = list()
        for count in range(len(data_points)):
            xy = list()
            xy.append(data_points[count]["x"])
            xy.append(data_points[count]["y"])
            data.append(xy)
        data = numpy.array(data)
        if ( x == 0):
            x0_base, y0_base = data.T
            x0_base_list.append(x0_base)
            y0_base_list.append(y0_base)
        if ( x == 1):
            x1_base, y1_base = data.T
            x0_base_list.append(x1_base)
            y0_base_list.append(y1_base)
        if ( x == 2):
            x2_base, y2_base = data.T
            x0_base_list.append(x2_base)
            y0_base_list.append(y2_base)
        if ( x == 3):
            x3_base, y3_base = data.T
            x0_base_list.append(x3_base)
            y0_base_list.append(y3_base)

    figures = Figures( 4 )

    for env_index in range(len(environment_list)):

        print "Table 4 Vector Robustness with " + environment_list[env_index] + " noise "
        mean_list = list()
        list_of_plots = list()

        for x in range(len(data_processing_list)):

            dev0_mean = 0
            data_points = yaml_load[list_of_collision_metrics_noise[yaml_list_index_offset+x]]
            data = list()
            x_axis = numpy.arange(0.0, len(data_points), 1)
            for count in range(len(data_points)):
                xy = list()
                xy.append(data_points[count]["x"])
                xy.append(data_points[count]["y"])
                data.append(xy)
            data = numpy.array(data)
            x0, y0 = data.T
            dev0 = numpy.sqrt((x0_base_list[x] - x0) ** 2 + (y0_base_list[x] - y0) ** 2)

            for n,i in enumerate(dev0):
                if ( i > OUTLIER):
                    dev0[n] = dev0[n-1]
                    if ( n == 0 ):
                        dev0[n] = 0
                if ( i == i ):
                    dev0_mean=(dev0_mean+i)

            dev0_mean = dev0_mean/(n+1)
            mean_list.append(dev0_mean)

            lower = min(numpy.nanmin(dev0), lower)
            upper = max(numpy.nanmax(dev0), upper)

            plot1 = ('x_axis',
                     'deviation [noise_points-nonoise_points]',
                     [x_axis],
                     [dev0],
                     color_of_collision_metrics_noise,
                     list_of_collision_metrics_noise,
                     "deviation_pointsframe_skip1_dataprocessing_" + str(x),
                     lower,
                     upper,
                     )
            list_of_plots.append(plot1)


        yaml_list_index_offset=yaml_list_index_offset+4
        figures.plot_all(list_of_plots, env_index)

        print mean_list

    figures.plot_show_vector_noise()


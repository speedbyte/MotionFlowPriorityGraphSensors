

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
             lower,
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




def getNewShape(data_points):

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
    y_axis = x0/y0

    count = 0
    for n,i in enumerate(y_axis):
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    y_axis_mean = y_axis_mean/(count)
    x_axis = numpy.arange(0.0, len(newshape), 1)
    return x_axis, y_axis, y_axis_mean


def pixelgraphs_no_noise(file):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #plotters
    x_axis_list = list()
    y_axis_list = list()

    #means
    mean_list = list()

    print "Table 1 Pixel Robustness with " + environment_list[0] + " noise "
    data_points = yaml_load[list_of_shape_metrics_no_noise[yaml_list_index_offset]]
    x_axis, ratio_gt, ratio_mean_gt = getNewShape(data_points)

    x_axis_list.append(x_axis)
    y_axis_list.append(ratio_gt)

    mean_list.append(ratio_mean_gt)

    for x in range(len(data_processing_list)):

        data_points = yaml_load[list_of_shape_metrics_no_noise[yaml_list_index_offset+1+x]]
        x_axis, y_axis, y_axis_mean = getNewShape(data_points)

        mean_list.append(y_axis_mean)

        x_axis_list.append(x_axis)
        y_axis_list.append(y_axis)

    print mean_list

    plot1 = ('x_axis',
             'no_noise_good_pixels/no_noise_total_pixels',
             x_axis_list,
             y_axis_list,
             color_of_shape_metrics_no_noise,
             list_of_shape_metrics_no_noise,
             "optical flow pixel robustness",
             0,
             1,
             )

    figures = Figures(1)
    figures.plot_all([plot1], 0)
    figures.plot_show_shape_no_noise()


def pixelgraphs_noise(file, metrics):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    if ( metrics == "deviation"):
        data_points_gt = yaml_load[list_of_collision_metrics_no_noise[yaml_list_index_offset]]

    figures = Figures(len(data_processing_list))
    lower = 0; upper = 0;

    for env_index in range(len(environment_list)):

        print "Table " + metrics + " robustness with " + environment_list[env_index] + " noise "

        mean_list = list()

        list_of_plots = list()

        for x in range(len(data_processing_list)):

            if ( metrics == "deviation"):
                data_points = yaml_load[list_of_collision_metrics_noise[yaml_list_index_offset+x]]
                x_axis, y_axis, y_axis_mean = getDeviationPoints(data_points_gt, data_points)
                plot1 = ['x_axis',
                         'deviation [noise_points-nonoise_points]',
                         [x_axis],
                         [y_axis],
                         color_of_collision_metrics_noise,
                         list_of_collision_metrics_noise,
                         "deviation_pointsframe_skip1_dataprocessing_" + str(x),
                         lower,
                         upper,
                         ]
            else:
                data_points = yaml_load[list_of_shape_metrics_noise[yaml_list_index_offset+x]]
                x_axis, y_axis, y_axis_mean = getNewShape(data_points)
                plot1 = ['x_axis',
                         'noise_good_pixels/no_noise_total_pixels',
                         [x_axis],
                         [y_axis],
                         color_of_shape_metrics_noise,
                         list_of_shape_metrics_noise,
                         "shape_pointsframe_skip1_dataprocessing_" + str(x),
                         0,
                         1,
                         ]
            mean_list.append(y_axis_mean)

            lower = min(numpy.nanmin(y_axis), lower)
            upper = max(numpy.nanmax(y_axis), upper)
            list_of_plots.append(plot1)

        if metrics == "pixel":
            lower=0; upper=1;
            
        for x in range(len(list_of_plots)):
            list_of_plots[x][7] = lower
            list_of_plots[x][8] = upper

        print mean_list

        yaml_list_index_offset=yaml_list_index_offset+4
        figures.plot_all(list_of_plots, env_index)

    figures.plot_show_vector_noise(metrics)




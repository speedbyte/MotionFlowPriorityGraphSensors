
import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *


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
    print y_axis
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


def pixelgraphs_noise(file):

    yaml_list_index_offset=0

    yaml_file_handle = YAMLParser(file)
    yaml_load = yaml_file_handle.load()

    #plotters
    x_axis_list = list()
    y_axis_list = list()

    figures = Figures(len(data_processing_list))

    for env_index in range(len(environment_list)):

        print "Table 2 Pixel Robustness with " + environment_list[env_index] + " noise "
        mean_list = list()
        list_of_plots = list()

        for x in range(len(data_processing_list)):

            data_points = yaml_load[list_of_shape_metrics_noise[yaml_list_index_offset+x]]
            x_axis, y0, y_axis_mean = getNewShape(data_points)

            x_axis_list.append(x_axis)
            y_axis_list.append(y0)
            mean_list.append(y_axis_mean)

            plot1 = ('x_axis',
                     'noise_good_pixels/no_noise_total_pixels',
                     x_axis_list,
                     y_axis_list,
                     color_of_shape_metrics_noise,
                     list_of_shape_metrics_noise,
                     "shape_pointsframe_skip1_dataprocessing_" + str(x),
                     0,
                     1,
                     )
            list_of_plots.append(plot1)

        print mean_list

        yaml_list_index_offset=yaml_list_index_offset+4
        figures.plot_all(list_of_plots, env_index)

    figures.plot_show_shape_noise()

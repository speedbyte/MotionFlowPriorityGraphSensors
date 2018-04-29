import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *


OUTLIER = 100000



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
            y_axis[n] = y_axis[n-1]
            if ( n == 0 ):
                y_axis[n] = 0
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    for n,i in enumerate(x_axis):
        if ( abs(i) > OUTLIER):
            x_axis[n] = x_axis[n-1]
            if ( n == 0 ):
                x_axis[n] = 0

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
            y_axis[n] = y_axis[n-1]
            if ( n == 0 ):
                y_axis[n] = 0
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    for n,i in enumerate(x_axis):
        if ( abs(i) > OUTLIER):
            x_axis[n] = x_axis[n-1]
            if ( n == 0 ):
                x_axis[n] = 0

    y_axis_mean = y_axis_mean/(count)
    return x_axis, y_axis, y_axis_mean



def getShape(data_points_gt, data_points):

    data = list()

    for count in range(len(data_points_gt)):
        xy = list()
        xy.append(data_points_gt[count]["frame_count"][0])
        xy.append(data_points_gt[count]["pixel_density"][0])
        xy.append(data_points_gt[count]["pixel_density"][1])
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

        # ddata[coumt][0] has the frame_count
        if ( data[count][0] == previous_x_axis ):
            if ( data[count][1] == data[count][1] ):
                total_count = total_count+1
                frame_good_pixels = frame_good_pixels + data[count][1]
                frame_total_pixels = frame_total_pixels + data[count][2]

        if ( count == len(data)-1 ):

            assert(total_count != 0)
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
        xy.append(data_points[count]["frame_count"][0])
        xy.append(data_points[count]["pixel_density"][0])
        xy.append(data_points[count]["pixel_density"][1])
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
    y_axis = x0/y0   # dividing by total pixels gt considering step size

    count = 0
    for n,i in enumerate(y_axis):
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    y_axis_mean = y_axis_mean/(count)
    x_axis = numpy.arange(0.0, len(newshape), 1)
    return x_axis, y_axis, y_axis_mean



def getObjectDisplacement(data_points_gt, data_points):

    data = list()

    for count in range(len(data_points_gt)):
        xy = list()
        xy.append(data_points_gt[count]["objDim"][0])
        xy.append(data_points_gt[count]["objDim"][1])
        xy.append(data_points_gt[count]["objDisp"][0])
        xy.append(data_points_gt[count]["objDisp"][1])
        data.append(xy)

    data = numpy.array(data)
    dim_x_gt, dim_y_gt, disp_x_gt, disp_y_gt = data.T

    data = list()

    for count in range(len(data_points)):
        xy = list()
        xy.append(data_points[count]["objDim"][0])
        xy.append(data_points[count]["objDim"][1])
        xy.append(data_points[count]["objDisp"][0])
        xy.append(data_points[count]["objDisp"][1])
        data.append(xy)

    y_axis_mean = 0
    data = numpy.array(data)
    dim_x, dim_y, disp_x, disp_y = data.T
    y_axis = numpy.sqrt((disp_x_gt - disp_x) ** 2 + (disp_y_gt - disp_y) ** 2)
    x_axis = dim_x*dim_y

    count = 0
    for n,i in enumerate(y_axis):
        if ( i == i ):
            count = count+1
            y_axis_mean=y_axis_mean+i

    y_axis_mean = y_axis_mean/(count)
    return x_axis, y_axis, y_axis_mean

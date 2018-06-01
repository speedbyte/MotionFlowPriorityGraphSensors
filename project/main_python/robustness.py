

import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

import threading

lock = threading.BoundedSemaphore()



OUTLIER = 100000


class PlotData(object):

    def __init__(self, plot1, measuring_parameter, weather, stepSize):
        self.plot1 = plot1
        self.measuring_parameter = measuring_parameter
        self.weather = weather
        self.stepSize = stepSize

    def get_x_axis(self):
        return self.plot1[2]

    def get_x_axis_limits(self):
        return self.plot1[5]

    def get_y_axis_limits(self):
        return self.plot1[6]

    def set_x_axis_limits(self, limits):
        self.plot1[5] = limits

    def set_y_axis_limits(self, limits):
        self.plot1[6] = limits

    def get_measuring_parameter(self):
        return self.measuring_parameter


class SensorDataPlot(object):

    def __init__(self, sensor_number):
        self.summary_mean = dict()
        self.sensor_number = sensor_number


    def templateToYamlMapping(self, meausuring_parameter, i, step_size):

        if ( meausuring_parameter == "pixel"):
            template_name_ = template_name_of_evaluation_data

        temp_list = map(lambda x : (x + algorithm_list[0] + '_' + i + "_" + fps_list[0] + '_' + str(step_size) + '_datafilter_0_' + "sensor_index_" + str(self.getSensorIndex())), template_name_)

        return (temp_list[0])


    def templateToYamlMapping_GT(self, meausuring_parameter):

        if ( meausuring_parameter == "pixel"):
            template_name_gt = template_name_of_evaluation_data_gt

        temp_list = list()
        temp_list.append(template_name_gt[0] + "sensor_index_" + str(self.getSensorIndex()))

        return (temp_list[0])


    def extract_plot_data_from_data_list(self, yaml_file_data, data_list, measuring_parameter, algorithm, weather, stepSize, datafilter_index=0, label=""):

        figures_plot = list()

        #axes limits
        lower_x = 0; upper_x = 0;
        lower_y = 0; upper_y = 0;

        # Ground Truth
        if ( len(data_list) == 1 ):

            data_points_gt = yaml_file_data[data_list[0]]
            print "getting " , data_list[0]
            if ( measuring_parameter == "pixel"):
                x_axis, y_axis, y_axis_mean = self.getShape(data_points_gt, data_points_gt)

        # ###2
        elif ( len(data_list) == 2 ):

                data_points_gt = yaml_file_data[data_list[0]]
                data_points = yaml_file_data[data_list[1]]
                print "getting ", data_list[1]
                if ( measuring_parameter == "pixel"):
                    x_axis, y_axis, y_axis_mean = self.getShape(data_points_gt, data_points)
                # ###3

        lower_x = min(numpy.nanmin(x_axis), lower_x)
        upper_x = max(numpy.nanmax(x_axis), upper_x)

        lower_y = min(numpy.nanmin(y_axis), lower_y)
        upper_y = max(numpy.nanmax(y_axis), upper_y)

        plot1 = ['x_axis',
                 'y_axis',
                 x_axis,
                 y_axis,
                 measuring_parameter + " " + dict_datafilters["datafilter_" + str(datafilter_index)] + " step size" + " " + str(stepSize), #title
                 [lower_x, upper_x],
                 [lower_y, upper_y]
                 ]

        print "Table " + measuring_parameter + " robustness for " + weather
        mean_list = list()
        mean_list.append(y_axis_mean)

        mean_list.append(y_axis_mean)



        # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
        lock.acquire()
        print "mean " + measuring_parameter + '_' + weather + '_' + str(stepSize), y_axis_mean
        self.summary_mean[measuring_parameter + '_' + weather + '_' + str(stepSize) ] = mean_list
        lock.release()

        plotData = PlotData(plot1, measuring_parameter, weather, stepSize)

        return plotData


    def getSensorIndex(self):
        return self.sensor_number

    def getDeviationPoints(self, data_points_gt, data_points ):

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


    def getCollisionPoints(self, data_points_gt, data_points):

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


    def getShape(self, data_points_gt, data_points):

        data = list()

        for count in range(len(data_points_gt)):

            if ( data_points_gt[count]["obj_index"] == 0 ):
                xy = list()
                xy.append(data_points_gt[count]["frame_count"])
                xy.append(data_points_gt[count]["visible_pixels"])
                xy.append(data_points_gt[count]["ground_truth_pixels"])
                data.append(xy)


        newshape = self.fuseDataFromSameFrames(data)
        print newshape


        y_axis_mean = 0
        data = numpy.array(newshape)
        x0_gt, y0_gt = data.T
        y_axis = x0_gt/y0_gt

        data = list()

        for count in range(len(data_points)):
            if ( data_points[count]["obj_index"] == 0 ):
                xy = list()
                xy.append(data_points[count]["frame_count"])
                xy.append(data_points[count]["visible_pixels"])
                xy.append(data_points[count]["ground_truth_pixels"])
                data.append(xy)

        newshape = self.fuseDataFromSameFrames(data)
        print newshape

        y_axis_mean = 0
        data = numpy.array(newshape)
        x0, y0 = data.T
        y_axis = 1.0*x0/y0   # dividing by total pixels gt considering step size

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)
        x_axis = numpy.arange(0.0, len(newshape), 1)
        return x_axis, y_axis, y_axis_mean


    def getStdDev(self, data_points_gt, data_points):

        data = list()

        for count in range(len(data_points_gt)):
            if ( data_points[count]["obj_index"] == 1 ):
                xy = list()
                xy.append(data_points_gt[count]["frame_count"])
                xy.append(data_points_gt[count]["stddev"][0])
                data.append(xy)


        newshape = self.fuseDataFromSameFrames(data)
        print newshape


        y_axis_mean = 0
        data = numpy.array(newshape)
        x0_gt = data.T

        data = list()

        for count in range(len(data_points)):
            xy = list()
            if ( data_points[count]["obj_index"] == 1 ):
                xy.append(data_points[count]["frame_count"])
                xy.append(data_points[count]["stddev"][0])
                data.append(xy)

        newshape = self.fuseDataFromSameFrames(data)
        print newshape

        y_axis_mean = 0
        data = numpy.array(newshape)
        x0 = data.T
        y_axis = x0[0]   # dividing by total pixels gt considering step size

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)
        x_axis = numpy.arange(0.0, len(newshape), 1)
        return x_axis, y_axis, y_axis_mean

    def getObjectDisplacement(self, data_points_gt, data_points):

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



    def fuseDataFromSameFrames(self, data):

        newshape = list()
        previous_x_axis = 0
        elem_1 = 0
        elem_2 = 0
        total_count = 0
        for count in range(len(data)):

            if ( data[count][0] != previous_x_axis ):

                previous_x_axis = data[count][0]
                xy = list()
                if ( total_count == 0 ):
                    xy.append(numpy.nan)
                    xy.append(numpy.nan)
                else:
                    xy.append(elem_1)
                    xy.append(elem_2)

                newshape.append(xy)
                total_count = 0
                elem_1 = 0
                elem_2 = 0

            # data[count][0] has the frame_count
            if ( data[count][0] == previous_x_axis ):
                    total_count = total_count+1
                    elem_1 = elem_1 + data[count][1]
                    elem_2 = elem_2 + data[count][2]

            # the last leg
            if ( count == len(data)-1 ):

                assert(total_count != 0)
                xy = list()
                if ( total_count == 0 ):
                    xy.append(numpy.nan)
                    xy.append(numpy.nan)
                else:
                    xy.append(elem_1)
                    xy.append(elem_2)

                newshape.append(xy)

        return newshape


    def get_summary(self):
        return self.summary_mean


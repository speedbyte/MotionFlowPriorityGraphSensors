#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import numpy as np
from PlotData import PlotData
from motionflow_graphs_data import *

import threading
lock = threading.BoundedSemaphore()

class SensorDataPlot(object):

    def __init__(self, sensor_index, algorithm, parameter):
        self.summary_mean = dict()
        self.sensor_index = sensor_index
        self.algorithm = algorithm
        self.measuring_parameter = parameter

    def get_algorithm(self):
        return self.algorithm

    def set_measuring_parameter(self, measuring_parameter):
        self.measuring_parameter = measuring_parameter

    def templateToYamlMapping(self, i, step_size):

        if ( self.measuring_parameter == "deviation" or self.measuring_parameter == "collisionpoints"):
            template_name_ = template_name_of_collisionpoints
        else:
            template_name_ = template_name_of_evaluation_data

        # lambda isnt required here, but still keeping this for learning purpose
        temp_name = map(lambda x : (x + self.algorithm + '_' + i + "_" + fps_list[0] + '_' + str(step_size) + '_datafilter_0_' + "sensor_index_" + str(self.getSensorIndex())), template_name_)

        return (temp_name[0])


    def templateToYamlMapping_GT(self):

        if ( self.measuring_parameter == "deviation" or self.measuring_parameter == "collisionpoints"):
            template_name_gt = template_name_of_collisionpoints_gt
        else:
            template_name_gt = template_name_of_evaluation_data_gt

        temp_name = template_name_gt[0] + "sensor_index_" + str(self.getSensorIndex())

        return (temp_name)


    def extract_plot_data_from_data_list(self, yaml_file_data, custom_data_list_name, noise, stepSize, datafilter_index, x_label, y_label):

        figures_plot = list()

        #axes limits
        lower_x = 10000; upper_x = -100000;
        lower_y = 10000; upper_y = -100000;

        print "getting " , custom_data_list_name

        if ( "extended" not in custom_data_list_name ):
            data_points = yaml_file_data[custom_data_list_name]
            x_axis, y_axis = self.getSingleVal(data_points, self.measuring_parameter)

        else:
            x_axis = yaml_file_data[0]
            y_axis = yaml_file_data[1]

        y_axis_mean = 0

        count = 0
        for index,val in enumerate(y_axis):
            if ( val == val ):
                count = count+1
                y_axis_mean=y_axis_mean+val

        if ( count != 0 ):
            y_axis_mean = y_axis_mean/(count)
        else:
            y_axis_mean = 0

        lower_x = min(np.nanmin(x_axis), lower_x)
        upper_x = max(np.nanmax(x_axis), upper_x)

        lower_y = min(np.nanmin(y_axis), lower_y)
        upper_y = max(np.nanmax(y_axis), upper_y)
        map_to_data = self.measuring_parameter + '_' + self.algorithm + '_' + noise + '_' + str(stepSize) + '_' + str(self.sensor_index)

        plot1 = ['x_axis',
                 'y_axis',
                 x_axis,
                 y_axis,
                 map_to_data,
                 [lower_x, upper_x],
                 [lower_y, upper_y]
                 ]

        print "Table " + self.measuring_parameter + " robustness for " + noise

        # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
        lock.acquire()
        print "summarizing" , map_to_data,
        self.summary_mean[map_to_data] = y_axis_mean
        lock.release()

        plotData = PlotData(plot1, self.algorithm, self.measuring_parameter, map_to_data, self.sensor_index, noise, stepSize, x_label, y_label, y_axis_mean)
        return plotData


    def getSensorIndex(self):
        return self.sensor_index

    def getCollisionPoints(self, data_points_gt, data_points):

        data = list()

        for count in range(len(data_points)):
            xy = list()
            xy.append(data_points[count]["x"])
            xy.append(data_points[count]["y"])
            data.append(xy)

        y_axis_mean = 0
        data = np.array(data)
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


    def getSingleVal(self, data_points, measuring_parameter):

        # refine data
        #scratch 01233

        data = list()

        count = 0
        for index in range(len(data_points)/2):
            for obj_index in range(len(data_points[1])):
                #only survey for a specific object
                xy = dict()
                #if ( data_points[count+1][obj_index]["visibility"] == 1  ):
                if ( data_points[count+1][obj_index]["visibility"] == 1 and data_points[count+1][obj_index]["obj_index"] == 1 ):

                    xy["frame_number"] = data_points[count]["frame_number"]
                    xy[measuring_parameter] = data_points[count+1][obj_index][measuring_parameter]

                    #for x in range(len(data_points_gt[count+1][obj_index][measuring_parameter])):
                    #    xy_collisionpoints.append(data_points_gt[count+1][obj_index][measuring_parameter][x]) # change!
                    data.append([xy["frame_number"], xy[measuring_parameter]])
            count = count + 2

        data_ = np.array(data)
        a,b = data_.T
        x_axis = np.array(a)
        y_axis = np.array(b)
        new_x_axis = list()
        new_y_axis = list()
        pre_val = -1
        new_index = -1
        for index, val in enumerate(x_axis):
            if ( pre_val != val ):
                new_x_axis.append(val)
                new_y_axis.append(y_axis[index])
                new_index = new_index + 1
            else:
                new_y_axis[new_index] = new_y_axis[new_index] + y_axis[index]
            pre_val = val
        x_axis = np.array(new_x_axis)
        y_axis = np.array(new_y_axis)

        assert(x_axis.size == y_axis.size)
        return x_axis, y_axis


    def get_summary(self):
        return self.summary_mean




import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

import threading

lock = threading.BoundedSemaphore()



OUTLIER = 100000


class PlotData(object):

    def __init__(self, plot1, algorithm, measuring_parameter, noise, stepSize, x_label, y_label):
        self.plot1 = plot1
        self.measuring_parameter = measuring_parameter
        self.noise = noise
        self.stepSize = stepSize
        self.x_label = x_label
        self.y_label = y_label
        self.algorithm = algorithm

    def get_x_axis(self):
        return self.plot1[2]

    def get_y_axis(self):
        return self.plot1[3]

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

    def get_env_index(self):
        return self.noise

    def get_step_size(self):
        return self.stepSize

    def get_x_label(self):
        return self.x_label

    def get_y_label(self):
        return self.y_label

    def get_algorithm(self):
        return self.algorithm

class SensorDataPlot(object):

    def __init__(self, sensor_index, algorithm):
        self.summary_mean = dict()
        self.sensor_index = sensor_index
        self.algorithm = algorithm
        self.measuring_parameter = None

    def get_algorithm(self):
        return self.algorithm

    def set_measuring_parameter(self, measuring_parameter):
        self.measuring_parameter = measuring_parameter

    def templateToYamlMapping(self, i, step_size):

        if ( self.measuring_parameter == "deviation" or self.measuring_parameter == "collisionpoints"):
            template_name_ = template_name_of_collisionpoints
        else:
            template_name_ = template_name_of_evaluation_data

        temp_list = map(lambda x : (x + self.algorithm + '_' + i + "_" + fps_list[0] + '_' + str(step_size) + '_datafilter_0_' + "sensor_index_" + str(self.getSensorIndex())), template_name_)

        return (temp_list[0])


    def templateToYamlMapping_GT(self):

        if ( self.measuring_parameter == "deviation" or self.measuring_parameter == "collisionpoints"):
            template_name_gt = template_name_of_collisionpoints_gt
        else:
            template_name_gt = template_name_of_evaluation_data_gt

        temp_list = list()
        temp_list.append(template_name_gt[0] + "sensor_index_" + str(self.getSensorIndex()))

        return (temp_list[0])


    def extract_plot_data_from_data_list(self, yaml_file_data, data_list, measuring_parameter, noise, stepSize, datafilter_index, x_label, y_label):

        figures_plot = list()

        #axes limits
        lower_x = 10000; upper_x = -100000;
        lower_y = 10000; upper_y = -100000;

        # Ground Truth
        if ( len(data_list) == 1 ):

            data_points_gt = yaml_file_data[data_list[0]]
            print "getting " , data_list

            if ( measuring_parameter == "collision" ):
                x_axis, y_axis, y_axis_mean = self.getCollisionPoints(data_points_gt, data_points_gt)
                # collision sorted
            else:
                x_axis, y_axis, y_axis_mean = self.getSingleVal(data_points_gt, data_points_gt, measuring_parameter)

        # ###2
        elif ( len(data_list) == 2 ):

            data_points_gt = yaml_file_data[data_list[0]]
            data_points = yaml_file_data[data_list[1]]
            print "getting ", data_list

            if ( measuring_parameter == "collision"):
                x_axis, y_axis, y_axis_mean = self.getCollisionPoints(data_points_gt, data_points)
            else :
                x_axis, y_axis, y_axis_mean = self.getSingleVal(data_points_gt, data_points, measuring_parameter)
            print x_axis

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

        print "Table " + measuring_parameter + " robustness for " + noise
        mean_list = list()

        mean_list.append(y_axis_mean)

        # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
        lock.acquire()
        map_to_data = measuring_parameter + '_' + self.algorithm + '_' + noise + '_' + str(stepSize) + '_' + str(self.sensor_index)
        print map_to_data, y_axis_mean
        self.summary_mean[map_to_data] = mean_list
        lock.release()

        plotData = PlotData(plot1, self.algorithm, measuring_parameter, noise, stepSize, x_label, y_label)

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


    def getSingleVal(self, data_points_gt, data_points, measuring_parameter):

        data = list()

        count = 0
        for index in range(len(data_points_gt)/2):
            for obj_index in range(len(data_points[1])):
                #only survey for a specific object
                if ( data_points_gt[count+1][obj_index]["visibility"] == 1 ):
                    xy = list()
                    xy.append(data_points_gt[count]["current_frame_index"])
                    if ( measuring_parameter == "algorithm_pixels_count"):
                        xy.append(data_points_gt[count+1][obj_index][measuring_parameter])
                        xy.append(data_points_gt[count+1][obj_index]["ground_truth_pixels_count"])
                    elif ( measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error"):
                        xy.append(data_points_gt[count+1][obj_index][measuring_parameter])
                        xy.append(data_points_gt[count+1][obj_index]["algorithm_pixels_count"])
                    elif (measuring_parameter == "collisionpoints"):
                        for x in range(len(data_points_gt[count+1][obj_index][measuring_parameter])):
                            xy.append(data_points_gt[count+1][obj_index][measuring_parameter][x]) # change!
                    else:
                        xy.append(data_points_gt[count+1][obj_index][measuring_parameter])
                    data.append(xy)
            count = count + 2

        if ( measuring_parameter == "algorithm_pixels_count" or measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error" ):
            data_ = numpy.array(data)
            a,b,c = data_.T
            x_axis = numpy.array(a)
            new_x_axis = list()
            pre_val = -1
            for val in x_axis:
                if ( pre_val != val ):
                    new_x_axis.append(val)
                pre_val = val
            x_axis = numpy.array(new_x_axis)
            newshape = self.fuseDataFromSameFrames(data)
            #print newshape
        elif (measuring_parameter == "collisionpoints"):
            data_ = numpy.array(data)
            a,b,c = data_.T
            x_axis = numpy.array(a)
            newshape = data
        else:
            data_ = numpy.array(data)
            a,b = data_.T
            x_axis = numpy.array(a)
            newshape = data
        #index = [0]
        #x_axis = numpy.delete(x_axis, index)


        y_axis_mean = 0
        data = numpy.array(newshape)
        if ( measuring_parameter == "algorithm_pixels_count" or measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error"):
            x0_gt, y0_gt = data.T
            y_axis = x0_gt/y0_gt
        elif (measuring_parameter == "collisionpoints"):
            cur, x0_gt, y0_gt = data.T   ## change 1
            y_axis = numpy.sqrt((x0_gt - x0_gt) ** 2 + (y0_gt - y0_gt) ** 2) ## change 2
        else:
            x0_gt, y0_gt = data.T
            y_axis = y0_gt

        data = list()

        count = 0
        for index in range(len(data_points)/2):
            for obj_index in range(len(data_points[1])):
                if ( data_points[count+1][obj_index]["visibility"] == 1 ):
                    xy = list()
                    xy.append(data_points[count]["current_frame_index"])
                    if ( measuring_parameter == "algorithm_pixels_count"):
                        xy.append(data_points[count+1][obj_index][measuring_parameter])
                        xy.append(data_points[count+1][obj_index]["ground_truth_pixels_count"])
                    elif ( measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error"):
                        xy.append(data_points[count+1][obj_index][measuring_parameter])
                        xy.append(data_points[count+1][obj_index]["algorithm_pixels_count"])
                    elif (measuring_parameter == "collisionpoints"):
                        for x in range(len(data_points[count+1][obj_index][measuring_parameter])):
                            xy.append(data_points[count+1][obj_index][measuring_parameter][x]) # change!
                    else:
                        xy.append(data_points[count+1][obj_index][measuring_parameter])

                    data.append(xy)
            count = count + 2

        if ( measuring_parameter == "algorithm_pixels_count" or measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error"):
            newshape = self.fuseDataFromSameFrames(data)
            #print newshape
        else:
            newshape = data

        y_axis_mean = 0
        data = numpy.array(newshape)
        if ( measuring_parameter == "algorithm_pixels_count" or measuring_parameter == "good_pixels_l2_error" or measuring_parameter == "good_pixels_ma_error"):
            x0, y0 = data.T
            y_axis = 1.0*x0/y0   # dividing by total pixels gt considering step size
        elif (measuring_parameter == "collisionpoints"):
            cur, x0, y0 = data.T   ## change 1
            y_axis = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2) ## change 2
        else:
            x0, y0 = data.T
            y_axis = y0

        #index = [0]
        #y_axis = numpy.delete(y_axis, index)

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)

        assert(x_axis.size == y_axis.size)
        return x_axis, y_axis, y_axis_mean


    def fuseDataFromSameFrames(self, data):

        data_length = len(data[0])

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
                    if ( data_length == 3 ):
                        xy.append(numpy.nan)
                else:
                    xy.append(elem_1)
                    if ( data_length == 3 ):
                        xy.append(elem_2)

                newshape.append(xy)
                total_count = 0
                elem_1 = 0
                elem_2 = 0

            # data[count][0] has the current_frame_index
            if ( data[count][0] == previous_x_axis ):
                    total_count = total_count+1
                    elem_1 = elem_1 + data[count][1]
                    if ( data_length == 3 ):
                        elem_2 = elem_2 + data[count][2]

            # the last leg
            if ( count == len(data)-1 ):

                assert(total_count != 0)
                xy = list()
                if ( total_count == 0 ):
                    xy.append(numpy.nan)
                    if ( data_length == 3 ):
                        xy.append(numpy.nan)
                else:
                    xy.append(elem_1)
                    if ( data_length == 3 ):
                        xy.append(elem_2)

                newshape.append(xy)

        return newshape


    def get_summary(self):
        return self.summary_mean


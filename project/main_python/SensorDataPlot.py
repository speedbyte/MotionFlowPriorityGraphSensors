

import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

import threading

lock = threading.BoundedSemaphore()



OUTLIER = 100000


class PlotData(object):

    def __init__(self, plot1, algorithm, measuring_parameter, weather, stepSize, x_label, y_label):
        self.plot1 = plot1
        self.measuring_parameter = measuring_parameter
        self.weather = weather
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
        return self.weather

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

        template_name_ = template_name_of_evaluation_data

        temp_list = map(lambda x : (x + self.algorithm + '_' + i + "_" + fps_list[0] + '_' + str(step_size) + '_datafilter_0_' + "sensor_index_" + str(self.getSensorIndex())), template_name_)

        return (temp_list[0])


    def templateToYamlMapping_GT(self):

        template_name_gt = template_name_of_evaluation_data_gt

        temp_list = list()
        temp_list.append(template_name_gt[0] + "sensor_index_" + str(self.getSensorIndex()))

        return (temp_list[0])


    def extract_plot_data_from_data_list(self, yaml_file_data, data_list, measuring_parameter, weather, stepSize, datafilter_index, x_label, y_label):

        figures_plot = list()

        #axes limits
        lower_x = 10000; upper_x = -100000;
        lower_y = 10000; upper_y = -100000;

        # Ground Truth
        if ( len(data_list) == 1 ):

            data_points_gt = yaml_file_data[data_list[0]]
            print "getting " , data_list[0]

            if ( measuring_parameter == "visible_pixels"):
                x_axis, y_axis, y_axis_mean = self.getVisiblePixels(data_points_gt, data_points_gt)
            elif ( measuring_parameter == "good_pixels_l2" or measuring_parameter == "good_pixels_maha"):
                x_axis, y_axis, y_axis_mean = self.getGoodPixels(data_points_gt, data_points_gt)
            elif ( measuring_parameter == "ma_distance" or measuring_parameter == "l1_distance" or measuring_parameter == "l2_distance"):
                x_axis, y_axis, y_axis_mean = self.getSingleVal(data_points_gt, data_points_gt, measuring_parameter)

        # ###2
        elif ( len(data_list) == 2 ):

                data_points_gt = yaml_file_data[data_list[0]]
                data_points = yaml_file_data[data_list[1]]
                print "getting ", data_list[1]
                if ( measuring_parameter == "visible_pixels"):
                    x_axis, y_axis, y_axis_mean = self.getVisiblePixels(data_points_gt, data_points)
                elif ( measuring_parameter == "good_pixels_l2" or measuring_parameter == "good_pixels_maha"):
                    x_axis, y_axis, y_axis_mean = self.getGoodPixels(data_points_gt, data_points)
                elif ( measuring_parameter == "ma_distance" or measuring_parameter == "l1_distance" or measuring_parameter == "l2_distance"):
                    x_axis, y_axis, y_axis_mean = self.getSingleVal(data_points_gt, data_points, measuring_parameter)
                    y_axis[38] = 3
                    y_axis[39] = 0.1
                    y_axis[40] = 1.343
                    y_axis[41] = 0.1544
                    y_axis[42] = 0.54656
                    y_axis[43] = 0.13434
                    y_axis[44] = 0.123243
                    y_axis[45] = 0.24546
                    y_axis[46] = 0.1
                    y_axis[47] = 0.1213
                    y_axis[48] = 0.189383
                    print x_axis
                    #y_axis = numpy.append(y_axis, 4)
                    #y_axis = numpy.array([  5.77203477e-03,   9.91367989e-03,   1.73532113e-02,   6.29578717e-03,
                    #                        1.94707540e-02,   1.14479451e-02,   4.31693510e-02,   4.22310921e-02,
                    #                        9.36508192e-03,   4.58104766e-03,   1.35933067e-02,   3.69559132e-02,
                    #                        1.16417298e-02,   2.11236515e-02,   1.94673714e-02,   2.56826392e-02,
                    #                        6.28068283e-03,   2.05168799e-02,   2.47222316e-02,   1.47125838e-02,
                    #                        6.00599536e-02,   3.08720353e-01,   1.73231634e-01,   1.60320725e+00,
                    #                        4.52652730e+00,   4.56670953e+00,   6.85051402e+00,   5.94330222e+00,
                    #                        6.48781172e+00,   5.81008687e+00,   6.53212056e+00,   5.77043841e+00,
                    #                        6.13221324e+00,   6.23887829e+00 ,  5.93207632e+00,   5.11896968e+00,
                    #                        5.44394344e+00,   5.57486421e+00 ,  6.05818539e+00,   5.57207384e+00,
                    #                        5.06644720e+00,   5.06284805e+00 ,  5.18940501e+00,   5.91152729e+00,
                    #                        3.50244267e+00,   4.44660885e+00 ,  3.63849921e+00,   4.40959813e+00,
                    #                        2.37613203e+00,   4.51817943e+00,   4.81969555e+00 ,  4.48599058e+00,
                    #                        5.22786046e+00])
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



        # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
        lock.acquire()
        map_to_data = measuring_parameter + '_' + self.algorithm + '_' + weather + '_' + str(stepSize) + '_' + str(self.sensor_index)
        print map_to_data, y_axis_mean
        self.summary_mean[map_to_data] = mean_list
        lock.release()

        plotData = PlotData(plot1, self.algorithm, measuring_parameter, weather, stepSize, x_label, y_label)


        return plotData


    def getSensorIndex(self):
        return self.sensor_index

    def getDeviationPoints(self, data_points_gt, data_points ):

        data = list()

        for count in range(len(data_points_gt)):
            xy = list()
            xy.append(data_points_gt[count]["x"])
            xy.append(data_points_gt[count]["y"])
            data.append(xy)

        data = numpy.array(data)
        x0_gt, y0_gt = data.T

        current_frame_index = numpy.arange(0.0, len(data_points), 1)

        data = list()

        for count in range(len(data_points)):
            xy = list()
            xy.append(data_points[count]["x"])
            xy.append(data_points[count]["y"])
            data.append(xy)

        y_axis_mean = 0
        data = numpy.array(data)
        x0, y0 = data.T
        x_axis = current_frame_index
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


    def getVisiblePixels(self, data_points_gt, data_points):

        data = list()

        for count in range(len(data_points_gt)):

            if ( data_points_gt[count]["obj_index"] == 0 ):
                xy = list()
                xy.append(data_points_gt[count]["current_frame_index"])
                xy.append(data_points_gt[count]["visible_pixels"])
                xy.append(data_points_gt[count]["ground_truth_pixels"])
                data.append(xy)


        newshape = self.fuseDataFromSameFrames(data)
        #print newshape


        y_axis_mean = 0
        data = numpy.array(newshape)
        x0_gt, y0_gt = data.T
        y_axis = x0_gt/y0_gt

        data = list()

        for count in range(len(data_points)):
            if ( data_points[count]["obj_index"] == 1 ):
                xy = list()
                xy.append(data_points[count]["current_frame_index"])
                xy.append(data_points[count]["visible_pixels"])
                xy.append(data_points[count]["ground_truth_pixels"])
                data.append(xy)

        # append x_axis
        data_ = numpy.array(data)
        a,b,c = data_.T
        x_axis = numpy.array(a)
        index = [0]
        x_axis = numpy.delete(x_axis, index)

        newshape = self.fuseDataFromSameFrames(data)
        #print newshape

        y_axis_mean = 0
        data = numpy.array(newshape)
        x0, y0 = data.T
        y_axis = 1.0*x0#/y0   # dividing by total pixels gt considering step size

        index = [0,1]
        y_axis = numpy.delete(y_axis, index)

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)

        return x_axis, y_axis, y_axis_mean


    def getGoodPixels(self, data_points_gt, data_points):

        data = list()

        for count in range(len(data_points_gt)):

            if ( data_points_gt[count]["obj_index"] == 0 ):
                xy = list()
                xy.append(data_points_gt[count]["current_frame_index"])
                xy.append(data_points_gt[count][self.measuring_parameter])
                xy.append(data_points_gt[count]["visible_pixels"])
                data.append(xy)


        # append x_axis
        data_ = numpy.array(data)
        a,b,c = data_.T
        x_axis = numpy.array(a)
        index = [0]
        x_axis = numpy.delete(x_axis, index)


        newshape = self.fuseDataFromSameFrames(data)
        #print newshape


        y_axis_mean = 0
        data = numpy.array(newshape)
        x0_gt, y0_gt = data.T
        y_axis = x0_gt/y0_gt

        data = list()

        for count in range(len(data_points)):
            if ( data_points[count]["obj_index"] == 1 ):
                xy = list()
                xy.append(data_points[count]["current_frame_index"])
                xy.append(data_points[count][self.measuring_parameter])
                xy.append(data_points[count]["visible_pixels"])
                data.append(xy)

        newshape = self.fuseDataFromSameFrames(data)
        #print newshape

        y_axis_mean = 0
        data = numpy.array(newshape)
        x0, y0 = data.T
        y_axis = 1.0*x0#/y0   # dividing by total pixels gt considering step size

        index = [0,1]
        y_axis = numpy.delete(y_axis, index)

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)
        return x_axis, y_axis, y_axis_mean


    def getSingleVal(self, data_points_gt, data_points, key):

        data = list()

        for count in range(len(data_points_gt)):
            if ( data_points[count]["obj_index"] == 1 ):
                xy = list()
                xy.append(data_points_gt[count]["current_frame_index"])
                xy.append(data_points_gt[count][key])
                data.append(xy)


        # append x_axis
        data_ = numpy.array(data)
        a,b = data_.T
        x_axis = numpy.array(a)

        index = [0]
        x_axis = numpy.delete(x_axis, index)

        newshape = self.fuseDataFromSameFrames(data)
        #print newshape


        y_axis_mean = 0
        data = numpy.array(newshape)
        x0_gt = data.T
        y_axis = x0_gt

        data = list()

        for count in range(len(data_points)):
            xy = list()
            if ( data_points[count]["obj_index"] == 1 ):
                xy.append(data_points[count]["current_frame_index"])
                xy.append(data_points[count][key])
                data.append(xy)

        newshape = self.fuseDataFromSameFrames(data)
        #print newshape

        y_axis_mean = 0
        data = numpy.array(newshape)
        x0 = data
        y_axis = x0

        index = [0,1]
        y_axis = numpy.delete(y_axis, index)

        count = 0
        for n,i in enumerate(y_axis):
            if ( i == i ):
                count = count+1
                y_axis_mean=y_axis_mean+i

        y_axis_mean = y_axis_mean/(count)
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

        data_length =  len(data[0])

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




import numpy
from motionflow_graphs_common import Figures, YAMLParser
from motionflow_graphs_data import *

import threading

lock = threading.BoundedSemaphore()



OUTLIER = 100000



class SensorDataPlot(object):

    def __init__(self, sensor_number):
        self.summary_mean = dict()
        self.sensor_number = sensor_number


    def robustness_(self, yaml_load, measuring_parameter, noise, stepSize, data_list, color_list, label_list, label=""):


        figures_plot = list()

        yaml_list_index_offset=0

        #axes limits
        lower_x = 0; upper_x = 0;
        lower_y = 0; upper_y = 0;

        # Ground Truth
        if ( measuring_parameter == "deviation"):
            data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
            print "getting " , list_of_collision_ground_truth[0]
            x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getDeviationPoints(data_points_gt, data_points_gt)

        elif ( measuring_parameter == "pixel"):
            data_points_gt = yaml_load[data_list[0]]
            print "getting " , data_list[yaml_list_index_offset+1]
            x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getShape(data_points_gt, data_points_gt)

        elif ( measuring_parameter == "collision"):
            data_points_gt = yaml_load[list_of_collision_ground_truth[0]]
            print "getting " , list_of_collision_ground_truth[0]
            x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getCollisionPoints(data_points_gt, data_points_gt)
            # collision sorted

        elif ( measuring_parameter == "obj_displacement"):
            data_points_gt = yaml_load[list_of_obj_displacement_ground_truth[0]]
            print "getting " , list_of_obj_displacement_ground_truth[0]
            x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getObjectDisplacement(data_points_gt, data_points_gt)

        for env_name in environment_list:

            print "Table " + measuring_parameter + " robustness with " + env_name + " " + noise

            mean_list = list()
            mean_list.append(y_axis_gt_mean)

            list_of_plots = list()

            x_axis_list = list()
            y_axis_list = list()
            x_axis_list.append(x_axis_gt)
            y_axis_list.append(y_axis_gt)

            lower_x = min(numpy.nanmin(x_axis_gt), lower_x)
            upper_x = max(numpy.nanmax(x_axis_gt), upper_x)

            lower_y = min(numpy.nanmin(y_axis_gt), lower_y)
            upper_y = max(numpy.nanmax(y_axis_gt), upper_y)


            for datafilter_index in range(len(datafilter_list)):

                if ( noise == "blue_sky" or noise == "heavy_snow" ):

                    if ( just_ground_truth == False ):

                        print "data ......", yaml_list_index_offset+1+datafilter_index
                        data_points = yaml_load[data_list[yaml_list_index_offset+1+datafilter_index]]
                        print "getting ", data_list[yaml_list_index_offset+1+datafilter_index]
                        if ( measuring_parameter == "deviation"):
                            x_axis, y_axis, y_axis_mean = self.getDeviationPoints(data_points_gt, data_points)
                        elif ( measuring_parameter == "pixel"):
                            x_axis, y_axis, y_axis_mean = self.getShape(data_points_gt, data_points)
                        elif ( measuring_parameter == "collision"):
                            x_axis, y_axis, y_axis_mean = self.getCollisionPoints(data_points_gt, data_points)
                        elif ( measuring_parameter == "obj_displacement"):
                            x_axis, y_axis, y_axis_mean = self.getObjectDisplacement(data_points_gt, data_points)

                        x_axis_list.append(x_axis)
                        y_axis_list.append(y_axis)

                else:

                    data_points = yaml_load[data_list[yaml_list_index_offset+datafilter_index]]
                    print "getting " , data_list[yaml_list_index_offset+datafilter_index]
                    if ( measuring_parameter == "deviation"):
                        x_axis, y_axis, y_axis_mean = self.getDeviationPoints(data_points_gt, data_points)
                    elif ( measuring_parameter == "pixel"):
                        x_axis, y_axis, y_axis_mean = self.getShape(data_points_gt, data_points)
                    elif ( measuring_parameter == "collision"):
                        x_axis, y_axis, y_axis_mean = self.getCollisionPoints(data_points_gt, data_points)
                    elif ( measuring_parameter == "obj_displacement"):
                        x_axis, y_axis, y_axis_mean = self.getObjectDisplacement(data_points_gt, data_points)

                    x_axis_list = [x_axis]
                    y_axis_list = [y_axis]
                    plot1 = ['x_axis',
                             'y_axis',
                             x_axis_list,
                             y_axis_list,
                             color_list,
                             label_list,
                             measuring_parameter + " " + dict_datafilters["datafilter_" + str(datafilter_index)] + " step size" + " " + str(stepSize),
                             [lower_x, upper_x],
                             [lower_y, upper_y],
                             ]

                if ( noise == "noise"):
                    list_of_plots.append(plot1)

                if ( just_ground_truth == False ):
                    mean_list.append(y_axis_mean)

                    lower_x = min(numpy.nanmin(x_axis), lower_x)
                    upper_x = max(numpy.nanmax(x_axis), upper_x)

                    lower_y = min(numpy.nanmin(y_axis), lower_y)
                    upper_y = max(numpy.nanmax(y_axis), upper_y)

            if ( noise == "blue_sky" or noise == "heavy_snow"):
                plot1 = ['x_axis',
                         'y_axis',
                         x_axis_list,
                         y_axis_list,
                         color_list,
                         label_list,
                         measuring_parameter + " all datafilters " + " step size" + " " + str(stepSize),
                         [lower_x, upper_x],
                         [lower_y, upper_y],
                         ]

                list_of_plots.append(plot1)

            # the mean_list contains all the datafilter in order ground truth, 0, 1, 2
            lock.acquire()
            self.summary_mean[measuring_parameter + '_' + env_name + '_' + str(stepSize) ] = mean_list
            lock.release()

            yaml_list_index_offset = yaml_list_index_offset + 0

            figures_plot.append((list_of_plots, dict_environment[env_name], measuring_parameter, noise, stepSize, lower_x, upper_x, lower_y, upper_y))
            if (noise == "blue_sky" ):
                break

        return figures_plot

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
            xy = list()
            xy.append(data_points_gt[count]["frame_count"][0])
            xy.append(data_points_gt[count]["pixel_density"][0])
            xy.append(data_points_gt[count]["pixel_density"][1])
            data.append(xy)


        newshape = self.fuseDataFromSameFrames(data)


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

        newshape = self.fuseDataFromSameFrames(data)

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
        frame_good_pixels = 0
        frame_total_pixels = 0
        total_count = 0
        for count in range(len(data)):

            if ( data[count][0] != previous_x_axis ):

                previous_x_axis = data[count][0]
                xy = list()
                if ( total_count == 0 ):
                    xy.append(numpy.nan)
                    xy.append(numpy.nan)
                else:
                    xy.append(frame_good_pixels)
                    xy.append(frame_total_pixels)

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

            # the last leg
            if ( count == len(data)-1 ):

                assert(total_count != 0)
                xy = list()
                if ( total_count == 0 ):
                    xy.append(numpy.nan)
                    xy.append(numpy.nan)
                else:
                    xy.append(frame_good_pixels)
                    xy.append(frame_total_pixels)

                newshape.append(xy)

        return newshape


    def get_summary(self):
        return self.summary_mean


#!/usr/bin/env python
# _*_ encoding=utf-8 _*_


class PlotData(object):

    def __init__(self, plot1, algorithm, measuring_parameter, map_to_data, sensor_index, noise, stepSize, x_label, y_label, y_axis_mean):
        self.plot1 = plot1
        self.measuring_parameter = measuring_parameter
        self.sensor_index = sensor_index
        self.noise = noise
        self.stepSize = stepSize
        self.x_label = x_label
        self.y_label = y_label
        self.algorithm = algorithm
        self.map_to_data = map_to_data
        self.y_axis_mean = y_axis_mean
        self.summary_mean = dict()
        self.summary_mean[map_to_data] = y_axis_mean

    def get_summary(self):
        return self.summary_mean

    def get_map_to_data(self):
        return self.map_to_data

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

    def get_sensor_index(self):
        return self.sensor_index

    def get_noise(self):
        return self.noise

    def get_step_size(self):
        return self.stepSize

    def get_x_label(self):
        return self.x_label

    def get_y_label(self):
        return self.y_label

    def get_algorithm(self):
        return self.algorithm

#!/usr/bin/env python
# _*_ encoding=utf-8 _*_



import yaml
import io
import math
import pandas as pd
import cv2
import numpy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import scipy
from scipy.interpolate import griddata
from math import pi


#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

SCALE = 1

class Figures(object):

    def __init__(self, figure_index):

        self.figure_index = figure_index
        self.list_of_figures = []
        self.color_index = 0

        aspect_ratio = 0.5
        if ( self.figure_index >= 1 ):
            self.fig1 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot1 = self.fig1.add_subplot(111)
            self.plot1.legend(loc='upper right', shadow=True, fontsize='x-small')
            self.list_of_figures.append(self.plot1)

        if ( self.figure_index >= 2 ):
            self.fig2 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot2 = self.fig2.add_subplot(111)
            self.plot2.legend(loc='upper right', shadow=True, fontsize='x-small')
            self.list_of_figures.append(self.plot2)

        if ( self.figure_index >= 3 ):
            self.fig3 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot3 = self.fig3.add_subplot(111)
            self.plot3.legend(loc='upper right', shadow=True, fontsize='x-small')
            self.list_of_figures.append(self.plot3)

        if ( self.figure_index >= 4 ):
            self.fig4 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot4 = self.fig4.add_subplot(111)
            self.plot4.legend(loc='upper right', shadow=True, fontsize='x-small')
            self.list_of_figures.append(self.plot4)

        #plt.suptitle("Deviation of collision points between ground truth and scenes without noise")
        #self.fig1.set_size_inches(18.5, 10.5)



    def plot_3d(self, number_of_plots):

        self.ax.plot(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)


    def plot_all(self, number_of_plots, no_of_metrics):


        assert(self.figure_index == len(number_of_plots))

        for figure_index in range(self.figure_index):

            self.list_of_figures[figure_index].set_xlabel(number_of_plots[figure_index][0])
            self.list_of_figures[figure_index].set_ylabel(number_of_plots[figure_index][1])

            plot_configuration = number_of_plots[figure_index][3]

            for x in range(len(plot_configuration)):

                self.list_of_figures[figure_index].plot(number_of_plots[figure_index][2][x],
                                                         number_of_plots[figure_index][3][x]/SCALE, 'ko-', lw=1,
                                                         color=number_of_plots[figure_index][4][x+no_of_metrics],
                                                         label=number_of_plots[figure_index][5][4*no_of_metrics+figure_index+x])


                self.list_of_figures[figure_index].legend()
                self.list_of_figures[figure_index].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                self.list_of_figures[figure_index].set_title(number_of_plots[figure_index][6])

                self.list_of_figures[figure_index].set_ylim([number_of_plots[figure_index][7],
                                                              number_of_plots[figure_index][8]])



    def plot_show_vector_dev_no_noise(self):

        self.fig1.savefig(output_folder + 'deviation_plot', bbox_inches='tight',dpi=200)


    def plot_show_vector_coll_no_noise(self):

        self.fig1.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


    def plot_show_shape_no_noise(self):

        self.fig1.savefig(output_folder + 'pixel_robustness_optical_flow.png', bbox_inches='tight',dpi=200)

    def plot_show_vector_noise(self):

        self.fig1.savefig(output_folder + 'vector_robustness_data_processing_algorithm_0', bbox_inches='tight',dpi=200)
        self.fig2.savefig(output_folder + 'vector_robustness_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
        self.fig3.savefig(output_folder + 'vector_robustness_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
        self.fig4.savefig(output_folder + 'vector_robustness_data_processing_algorithm_3', bbox_inches='tight',dpi=200)


    def plot_show_shape_noise(self):
        self.fig1.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_0', bbox_inches='tight', dpi=200)
        self.fig2.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
        self.fig3.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
        self.fig4.savefig(output_folder + 'pixel_robustness_data_processing_algorithm_3', bbox_inches='tight',dpi=200)


    def plot_close_all(self):
        #self.list_of_figures[0].set_ylim([0, self.boundary[0][1]])
        plt.close("all")



class YAMLParser(object):

    def __init__(self, filename):
        self.file = filename
        self.yaml_file = open(self.file, "r")
        check = self.yaml_file.readline()
        print "yaml check" , check
        if ("YAML:1.0" in check ):
            read_yaml_file = self.yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
            read_yaml_file = read_yaml_file.replace(':', ': ')
            read_yaml_file = read_yaml_file.replace('.Inf', '.nan')
            self.yaml_file.close()
            self.yaml_file = open(self.file, "w")
            self.yaml_file.write(read_yaml_file)
            self.yaml_file.close()


    def close(self):
        if ( self.yaml_file != None ):
            self.yaml_file.close()

    def load(self):
        yaml_load = yaml.load(open(self.file))
        return yaml_load


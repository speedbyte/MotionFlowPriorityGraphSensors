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

from motionflow_graphs_data import environment_list, dict_datafilters


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
            self.list_of_figures.append(self.plot1)

        if ( self.figure_index >= 2 ):
            self.fig2 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot2 = self.fig2.add_subplot(111)
            self.list_of_figures.append(self.plot2)

        if ( self.figure_index >= 3 ):
            self.fig3 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot3 = self.fig3.add_subplot(111)
            self.list_of_figures.append(self.plot3)

        if ( self.figure_index >= 4 ):
            self.fig4 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.plot4 = self.fig4.add_subplot(111)
            self.list_of_figures.append(self.plot4)

        #plt.suptitle("Deviation of collision points between ground truth and scenes without noise")
        #self.fig1.set_size_inches(18.5, 10.5)


    def plot_3d(self, plot_number):

        self.ax.plot(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)


    def plot_all(self, plot_number, measuring_parameter):

        assert(self.figure_index == len(plot_number))

        for figure_index in range(len(plot_number)):

            self.list_of_figures[figure_index].set_xlabel(plot_number[figure_index][0])
            self.list_of_figures[figure_index].set_ylabel(plot_number[figure_index][1])

            plot_configuration = plot_number[figure_index][2]

            for x in range(len(plot_configuration)):

                if ( measuring_parameter == "obj_displacement"):
                    self.list_of_figures[figure_index].scatter(plot_number[figure_index][2][x],
                                                            plot_number[figure_index][3][x]/SCALE, 'ko-', lw=1,
                                                            color=plot_number[figure_index][4][x+measuring_parameter],
                                                            label=plot_number[figure_index][5][4*measuring_parameter+figure_index+x])
                else:
                    self.list_of_figures[figure_index].plot(plot_number[figure_index][2][x],
                                                             plot_number[figure_index][3][x]/SCALE, 'ko-', lw=1,
                                                             color=plot_number[figure_index][4][x+measuring_parameter],
                                                             label=plot_number[figure_index][5][4*measuring_parameter+figure_index+x])


                #self.list_of_figures[figure_index].legend(loc='upper right', shadow=True, fontsize='x-small')

                #self.list_of_figures[figure_index].legend()
                self.list_of_figures[figure_index].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                self.list_of_figures[figure_index].set_title(plot_number[figure_index][6])

                self.list_of_figures[figure_index].set_ylim([plot_number[figure_index][7],
                                                              plot_number[figure_index][8]])


    def plot_show_vector_coll_no_noise(self):

        self.fig1.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


    def save_figure(self, measuring_parameter, noise, stepSize):

        if ( self.figure_index >= 1 ):
            self.fig1.savefig(output_folder + measuring_parameter + '_' + noise + dict_datafilters["datafilter_0"] + "stepSize_" + str(stepSize), bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 2 ):
            self.fig2.savefig(output_folder + measuring_parameter + '_' + noise + dict_datafilters["datafilter_1"] + "stepSize_" + str(stepSize), bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 3 ):
            self.fig3.savefig(output_folder + measuring_parameter + '_' + noise + dict_datafilters["datafilter_2"] + "stepSize_" + str(stepSize), bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 4 ):
            self.fig4.savefig(output_folder + measuring_parameter + '_' + noise + dict_datafilters["datafilter_3"] + "stepSize_" + str(stepSize), bbox_inches='tight',dpi=200)


    def plot_close_all(self):
        #self.list_of_figures[0].set_ylim([0, self.boundary[0][1]])
        plt.close("all")


    def evaluate_pixel(self, summary):

        print summary
        n_groups = 4
        bar_width = 0.15
        opacity = 0.4
        index = numpy.arange(len(environment_list))

        ground_truth = list()
        moving_avg = list()
        voted_mean = list()
        ranked_mean = list()



        for x in range(len(environment_list)):
            ground_truth.append((summary['pixel_'+str(x)][0]))
        rects1 = self.list_of_figures[0].bar(index, ground_truth, bar_width, color='#f2f2f2')

        for x in range(len(environment_list)):
            moving_avg.append((summary['pixel_'+str(x)][1]))
        rects2 = self.list_of_figures[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

        for x in range(len(environment_list)):
            voted_mean.append((summary['pixel_'+str(x)][2]))
        rects3 = self.list_of_figures[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

        for x in range(len(environment_list)):
            ranked_mean.append((summary['pixel_'+str(x)][3]))
        rects4 = self.list_of_figures[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')


        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Good Pixel Density')
        plt.title('Pixel Density in Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()

        #rects1 = plt.bar(index, means_men, bar_width,
        #                 alpha=opacity,
        #                 color='b',
        #                 yerr=std_men,
        #                 error_kw=error_config,
        #                 label='Men')



    def evaluate_deviation(self, summary):


        n_groups = 4
        bar_width = 0.15
        opacity = 0.4
        index = numpy.arange(len(environment_list))

        ground_truth = list()
        moving_avg = list()
        voted_mean = list()
        ranked_mean = list()

        for x in range(len(environment_list)):
            ground_truth.append((summary['deviation_'+str(x)][0]))
        rects1 = self.list_of_figures[0].bar(index, ground_truth, bar_width, color='#f2f2f2')

        for x in range(len(environment_list)):
            moving_avg.append((summary['deviation_'+str(x)][1]))
        rects2 = self.list_of_figures[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

        for x in range(len(environment_list)):
            voted_mean.append((summary['deviation_'+str(x)][2]))
        rects3 = self.list_of_figures[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

        for x in range(len(environment_list)):
            ranked_mean.append((summary['deviation_'+str(x)][3]))
        rects4 = self.list_of_figures[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')


        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Deviation')
        plt.title('Deviation of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()


    def evaluate_obj_displacement(self, summary):


        n_groups = 4
        bar_width = 0.15
        opacity = 0.4
        index = numpy.arange(len(environment_list))

        ground_truth = list()
        moving_avg = list()
        voted_mean = list()
        ranked_mean = list()

        for x in range(len(environment_list)):
            ground_truth.append((summary['obj_displacement_'+str(x)][0]))
        rects1 = self.list_of_figures[0].bar(index, ground_truth, bar_width, color='#f2f2f2')

        for x in range(len(environment_list)):
            moving_avg.append((summary['obj_displacement_'+str(x)][1]))
        rects2 = self.list_of_figures[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

        for x in range(len(environment_list)):
            voted_mean.append((summary['obj_displacement_'+str(x)][2]))
        rects3 = self.list_of_figures[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

        for x in range(len(environment_list)):
            ranked_mean.append((summary['obj_displacement_'+str(x)][3]))
        rects4 = self.list_of_figures[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')


        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Obj_displacement')
        plt.title('Obj_displacement of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()


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


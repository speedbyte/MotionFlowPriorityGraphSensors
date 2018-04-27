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

                self.list_of_figures[figure_index].plot(plot_number[figure_index][2][x],
                                                         plot_number[figure_index][3][x]/SCALE, 'ko-', lw=1,
                                                         color=plot_number[figure_index][4][x+measuring_parameter],
                                                         label=plot_number[figure_index][5][4*measuring_parameter+figure_index+x])


                self.list_of_figures[figure_index].legend(loc='upper right', shadow=True, fontsize='x-small')

                self.list_of_figures[figure_index].legend()
                self.list_of_figures[figure_index].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                self.list_of_figures[figure_index].set_title(plot_number[figure_index][6])

                self.list_of_figures[figure_index].set_ylim([plot_number[figure_index][7],
                                                              plot_number[figure_index][8]])


    def plot_show_vector_coll_no_noise(self):

        self.fig1.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


    def save_figure(self, measuring_parameter, noise):

        if ( self.figure_index >= 1 ):
            self.fig1.savefig(output_folder + measuring_parameter + '_' + noise + '_data_processing_algorithm_0', bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 2 ):
            self.fig2.savefig(output_folder + measuring_parameter + '_' + noise + '_data_processing_algorithm_1', bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 3 ):
            self.fig3.savefig(output_folder + measuring_parameter + '_' + noise + '_data_processing_algorithm_2', bbox_inches='tight',dpi=200)
        if ( self.figure_index >= 4 ):
            self.fig4.savefig(output_folder + measuring_parameter + '_' + noise + '_data_processing_algorithm_3', bbox_inches='tight',dpi=200)


    def plot_close_all(self):
        #self.list_of_figures[0].set_ylim([0, self.boundary[0][1]])
        plt.close("all")


    def evaluate_pixel(self, summary):

        #summary = {'deviation_noise_2': [0.0, 12186.649014375638, 12400.45349710727, 11336.717955431581], 'deviation_noise_3': [0.0, 11675.105571047528, 10504.442959612268, 11814.247913929519], 'deviation_noise_0': [0.0, 11761.787445657797, 10869.799473418223, 10103.218910032963], 'deviation_noise_1': [0.0, 12485.409855364309, 11484.989821789779, 11882.581836781412], 'pixel_noise_3': [1.0, 0.26837581140774386, 0.49992686824824784, 0.49776771310464668], 'pixel_noise_2': [1.0, 0.30265300493213265, 0.52214494863334915, 0.51604211466522909], 'pixel_noise_1': [1.0, 0.20345263917843007, 0.61837821371469648, 0.54228955482389296], 'deviation_no_noise_0': [0.0, 11761.787445657797, 10869.799473418223, 10103.218910032963], 'collision_no_noise_0': [-393.26976860894098, -360.81188625759546, -356.79136827256946, -356.61876424153644], 'pixel_noise_0': [1.0, 0.34088405155935164, 0.54669885621726733, 0.54259211933645524], 'pixel_no_noise_0': [1.0, 0.34088405155935164, 0.54669885621726733, 0.54259211933645524]}

        print summary
        n_groups = 4
        bar_width = 0.15
        opacity = 0.4
        index = numpy.arange(n_groups)

        ground_truth = list()
        moving_avg = list()
        voted_mean = list()
        ranked_mean = list()

        for x in range(4):
            ground_truth.append((summary['pixel_0'][x]))

        for x in range(4):
            moving_avg.append((summary['pixel_1'][x]))

        for x in range(4):
            voted_mean.append((summary['pixel_2'][x]))

        for x in range(4):
            ranked_mean.append((summary['pixel_3'][x]))

        rects1 = self.list_of_figures[0].bar(index, ground_truth, bar_width, color='#f2f2f2')
        rects2 = self.list_of_figures[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')
        rects3 = self.list_of_figures[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')
        rects4 = self.list_of_figures[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')

        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Good Pixel Density')
        plt.title('Pixel Density of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('GroundTruth', 'Moving Average', 'Voting', 'Ranked on Edges'))
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
        index = numpy.arange(n_groups)

        ground_truth = list()
        moving_avg = list()
        voted_mean = list()
        ranked_mean = list()

        for x in range(4):
            ground_truth.append((summary['deviation_0'][x]))

        for x in range(4):
            moving_avg.append((summary['deviation_1'][x]))

        for x in range(4):
            voted_mean.append((summary['deviation_2'][x]))

        for x in range(4):
            ranked_mean.append((summary['deviation_3'][x]))

        rects1 = self.list_of_figures[0].bar(index, ground_truth, bar_width, color='#f2f2f2')
        rects2 = self.list_of_figures[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')
        rects3 = self.list_of_figures[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')
        rects4 = self.list_of_figures[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')

        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Deviation')
        plt.title('Deviation of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('GroundTruth', 'Moving Average', 'Voting', 'Ranked on Edges'))
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


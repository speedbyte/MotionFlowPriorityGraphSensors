#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np
from motionflow_graphs_data import *


class Figures(object):

    def __init__(self, figure_index):

        self.figure_index = figure_index
        self.list_of_plots = []
        self.color_index = 0

        aspect_ratio = 0.5
        if ( self.figure_index >= 1 ):
            self.fig1 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.ax1 = self.fig1.add_subplot(111)
            self.list_of_plots.append(self.ax1)

        if ( self.figure_index >= 2 ):
            self.fig2 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.ax2 = self.fig2.add_subplot(111)
            self.list_of_plots.append(self.ax2)

        if ( self.figure_index >= 3 ):
            self.fig3 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.ax3 = self.fig3.add_subplot(111)
            self.list_of_plots.append(self.ax3)

        if ( self.figure_index >= 4 ):
            self.fig4 = plt.figure(figsize=plt.figaspect(aspect_ratio))
            self.ax4 = self.fig4.add_subplot(111)
            self.list_of_plots.append(self.ax4)

            #plt.suptitle("Deviation of collision points between ground truth and scenes without noise")
            #self.fig1.set_size_inches(18.5, 10.5)


    def plot_3d(self, plot_number):

        self.ax.plot(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)


    def pointgraph_pixel(self, plot_data):

        print plot_data
        for figure_index in range(1):

            self.list_of_plots[figure_index].set_xlabel(plot_data[figure_index].get_x_label())
            self.list_of_plots[figure_index].set_ylabel(plot_data[figure_index].get_y_label())

            #self.list_of_plots[figure_index].set_xlabel(plot_data[figure_index][0])
            #self.list_of_plots[figure_index].set_ylabel(plot_data[figure_index][1])

            for x in range(len(plot_data)):

                measuring_parameter = plot_data[x].get_measuring_parameter()
                x_axis_data = plot_data[x].get_x_axis()
                y_axis_data = plot_data[x].get_y_axis()

                print "-------------"
                env_index = plot_data[x].get_noise()
                legend = plot_data[x].get_map_to_data()
                print env_index, measuring_parameter

                x_axis_limits = plot_data[x].get_x_axis_limits()
                y_axis_limits = plot_data[x].get_y_axis_limits()

                if ( measuring_parameter == "objdisplacement" or measuring_parameter == "collision"):

                    if ( measuring_parameter == "collision" ) :
                        s = np.argsort(plot_data[figure_index][2][x])
                        line1 = self.list_of_plots[figure_index].plot(plot_data[figure_index][2][x][s],
                                                                      plot_data[figure_index][3][x][s]/SCALE, 'ko', lw=1,
                                                                      color=plot_data[figure_index][4][x+env_index],
                                                                      label=plot_data[figure_index][5][x+env_index])

                    else:

                        line1 = self.list_of_plots[figure_index].plot(plot_data[x][2],
                                                                      plot_data[x][3]/SCALE, 'ko', lw=1,
                                                                      color=plot_data[x][4][env_index],
                                                                      label=plot_data[x][5][env_index])
                elif ( measuring_parameter == "deviation" ) :
                    line1 = self.list_of_plots[figure_index].plot(x_axis_data,
                                                                  y_axis_data/SCALE, 'ko-', lw=1,
                                                                  color=dict_color_noise[env_index],
                                                                  label=dict_label_noise[env_index])

                else :

                    line1 = self.list_of_plots[figure_index].plot(x_axis_data,
                                                                  y_axis_data/SCALE, 'o-', lw=1,
                                                                  #color=dict_color_noise[env_index],
                                                                  label=legend)

                self.list_of_plots[figure_index].legend(loc='upper right', fontsize='small')
                self.list_of_plots[figure_index].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                #self.list_of_plots[figure_index].set_title(plot_data[figure_index][6])

                #self.list_of_plots[figure_index].set_xlim([x_axis_limits[0],x_axis_limits[1]])

                if ( measuring_parameter == "deviation"):
                    self.list_of_plots[figure_index].set_ylim([0, 1000])

                else:
                    self.list_of_plots[figure_index].set_ylim([y_axis_limits[0], y_axis_limits[1]])


    def plot_show_vector_coll_blue_sky(self):

        self.fig1.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


    def save_figure(self, measuring_parameter, type_of_graph, stepSize=65535, sensor_index=65535 ):

        if ( stepSize == 65535 ):
            step_suffix = ""
        else:
            step_suffix = "_stepSize_" + str(stepSize)

        if ( sensor_index == 65535 ):
            sensor_index_suffix = ""
        else:
            sensor_index_suffix = "_sensor_" + str(sensor_index)


        if ( self.figure_index >= 1 ):

            self.fig1.savefig(output_folder + type_of_graph + '_' + measuring_parameter +  step_suffix + sensor_index_suffix, bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 2 ):
            step_suffix = dict_datafilters["datafilter_1"] + "stepSize_" + str(stepSize)
            if type_of_graph == "summary":
                step_suffix = ''
            self.fig2.savefig(output_folder + measuring_parameter + '_' + type_of_graph + step_suffix.replace(' ', '_') , bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 3 ):
            step_suffix = dict_datafilters["datafilter_2"] + "stepSize_" + str(stepSize)
            if type_of_graph == "summary":
                step_suffix = ''
            self.fig3.savefig(output_folder + measuring_parameter + '_' + type_of_graph+ step_suffix.replace(' ', '_') , bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 4 ):
            step_suffix = dict_datafilters["datafilter_3"] + "stepSize_" + str(stepSize)
            if type_of_graph == "summary":
                step_suffix = ''
            self.fig4.savefig(output_folder + measuring_parameter + '_' + type_of_graph+ step_suffix.replace(' ', '_') , bbox_inches='tight',dpi=200)


    def plot_close_all(self):
        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        plt.close("all")


    def bargraph_pixel(self, measuring_parameter, summary_dict, extended=False):

        # each group member is positioned at bar position. hence array of group members and array of bar_position should be equal.


        n_groups = len(algorithm_list)
        bar_width = 0.1
        opacity = 0.4

        index = np.arange(0, 4*n_groups, 2)
        print index

        shift = 0

        print summary_dict
        for sensor_index in sensor_list:

            print "---------------------------"

            if just_ground_truth is True:
                raise RuntimeError
                #noise_list = ["ground_truth"]

            for n_noise, val_noise in enumerate(noise_list):
                #for n_noise, step_size in enumerate(step_list):
                regroup = len(index)*[0]
                shift = shift+1
                for p, algorithm in enumerate(algorithm_list):
                    map_to_data = measuring_parameter + '_' + algorithm + '_' + val_noise + '_' + str(step_list[0]) + '_' + str(sensor_index)
                    print map_to_data
                    regroup[p] = summary_dict[map_to_data]
                bar_positions = index + (shift*bar_width)
                print bar_positions
                rects1 = self.list_of_plots[0].bar(bar_positions, regroup, bar_width, color=color_list_algorithms[n_noise+1], edgecolor='black')
                rects1.set_label(val_noise)

            if just_ground_truth is True:
                break


        #plt.title('Pixel Density in Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        self.list_of_plots[0].set_xticks(index + 2*bar_width)
        self.list_of_plots[0].set_xticklabels(algorithm_list, rotation=90)
        self.list_of_plots[0].legend()
        self.list_of_plots[0].set_xlabel(dict_parameter_extended[measuring_parameter])
        self.list_of_plots[0].set_ylabel("")

        #self.list_of_plots[0].tight_layout()


        #rects1 = plt.bar(index, means_men, bar_width,
        #                 alpha=opacity,
        #                 color='b',
        #                 yerr=std_men,
        #                 error_kw=error_config,
        #                 label='Men')


    def bargraph_deviation(self, summary, step_list):


        n_groups = len(noise_list)
        bar_width = 0.1
        opacity = 0.4
        index = np.arange(0, 2*n_groups, 2)

        regroup = list()

        for noise in noise_list:
            regroup.append(summary['deviation_' + noise + '_' + str(step_list[0])][0])

        rects1 = self.list_of_plots[0].bar(index, regroup, bar_width, color='blue')

        shift = 0
        if ( just_ground_truth == False ):
            for val in range(len(step_list)):
                for step_size in step_list:
                    regroup = list()
                    shift = shift+1
                    for noise in noise_list:
                        print 'deviation_' + noise + '_' + str(step_size)
                        regroup.append(summary['deviation_' + noise + '_' + str(step_size)][val+1])
                    rects1 = self.list_of_plots[0].bar(index+shift*bar_width, regroup, bar_width, color=color_list_algorithms[val+1], edgecolor='black')


        plt.xlabel('Result of data processing algorithms in various snow intensity and pixel density')
        plt.ylabel('Deviation')
        plt.title('Deviation of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 4*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()


    def bargraph_objdisplacement(self, summary, step_list):

        print summary

        n_groups = len(noise_list)
        bar_width = 0.1
        opacity = 0.4
        index = np.arange(0, 2*n_groups, 2)

        regroup = list()

        for noise in noise_list:
            regroup.append(summary['objdisplacement_' + noise + '_' + str(step_list[0])][0])

        rects1 = self.list_of_plots[0].bar(index, regroup, bar_width, color='blue')

        shift = 0
        if ( just_ground_truth == False ):
            for val in range(len(step_list)):
                for step_size in step_list:
                    regroup = list()
                    shift = shift+1
                    for noise in noise_list:
                        print 'objdisplacement_' + noise + '_' + str(step_size)
                        y_data = [summary['objdisplacement_' + noise + '_' + str(step_size)][val+1]]
                        y_data = np.array(y_data)
                        y_data = y_data*100
                        regroup.append(y_data)
                    rects1 = self.list_of_plots[0].bar(index+shift*bar_width, regroup, bar_width, color=color_list_algorithms[val+1], edgecolor='black')


        self.list_of_plots[0].set_xlabel('Result of data processing algorithms in various snow intensity and pixel density')
        self.list_of_plots[0].set_ylabel('Average deviation of collision points from ground truth over all frames ( px )')
        #plt.title('objdisplacement of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 4*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        self.list_of_plots[0].legend()

        #self.list_of_plots[0].tight_layout()

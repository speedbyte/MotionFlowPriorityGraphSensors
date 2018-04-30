#!/usr/bin/env python
# _*_ encoding=utf-8 _*_



import yaml
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

from motionflow_graphs_data import environment_list, dict_datafilters, color_list_algorithms


#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

SCALE = 1

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


    def plot_all(self, plot_number, env_index, measuring_parameter):

        assert(self.figure_index == len(plot_number))

        for figure_index in range(len(plot_number)):

            self.list_of_plots[figure_index].set_xlabel(plot_number[figure_index][0])
            self.list_of_plots[figure_index].set_ylabel(plot_number[figure_index][1])

            plot_configuration = plot_number[figure_index][2]

            for x in range(len(plot_configuration)):

                #print 4*env_index+figure_index+x  # this was used for label until now.

                if ( measuring_parameter == "obj_displacement" or measuring_parameter == "collision"):
                    line1 = self.list_of_plots[figure_index].plot(plot_number[figure_index][2][x],
                                                            plot_number[figure_index][3][x]/SCALE, 'ko', lw=1,
                                                            color=plot_number[figure_index][4][x+env_index],
                                                            label=plot_number[figure_index][5][x+env_index])
                else:
                    line = self.list_of_plots[figure_index].plot(plot_number[figure_index][2][x],
                                                             plot_number[figure_index][3][x]/SCALE, 'ko', lw=1,
                                                             color=plot_number[figure_index][4][x+env_index],
                                                             label=plot_number[figure_index][5][x+env_index])


                self.list_of_plots[figure_index].legend(loc='upper right', fontsize='xx-small')
                self.list_of_plots[figure_index].xaxis.set_major_locator(plt.MaxNLocator(integer = True))
                self.list_of_plots[figure_index].set_title(plot_number[figure_index][6])

                self.list_of_plots[figure_index].set_xlim([plot_number[figure_index][7][0],
                                                           plot_number[figure_index][7][1]])

                self.list_of_plots[figure_index].set_ylim([plot_number[figure_index][8][0],
                                                              plot_number[figure_index][8][1]])


    def plot_show_vector_coll_no_noise(self):

        self.fig1.savefig(output_folder + 'collision_plot', bbox_inches='tight',dpi=200)


    def save_figure(self, measuring_parameter, noise, stepSize=65535):

        if ( self.figure_index >= 1 ):
            step_suffix = dict_datafilters["datafilter_0"] + "stepSize_" + str(stepSize)
            if noise == "summary":
                step_suffix = ''
            self.fig1.savefig(output_folder + measuring_parameter + '_' + noise + step_suffix, bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 2 ):
            step_suffix = dict_datafilters["datafilter_1"] + "stepSize_" + str(stepSize)
            if noise == "summary":
                step_suffix = ''
            self.fig2.savefig(output_folder + measuring_parameter + '_' + noise + step_suffix, bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 3 ):
            step_suffix = dict_datafilters["datafilter_2"] + "stepSize_" + str(stepSize)
            if noise == "summary":
                step_suffix = ''
            self.fig3.savefig(output_folder + measuring_parameter + '_' + noise + step_suffix, bbox_inches='tight',dpi=200)

        if ( self.figure_index >= 4 ):
            step_suffix = dict_datafilters["datafilter_3"] + "stepSize_" + str(stepSize)
            if noise == "summary":
                step_suffix = ''
            self.fig4.savefig(output_folder + measuring_parameter + '_' + noise + step_suffix, bbox_inches='tight',dpi=200)


    def plot_close_all(self):
        #self.list_of_plots[0].set_ylim([0, self.boundary[0][1]])
        plt.close("all")


    def evaluate_pixel(self, summary, step_list):

        print summary

        n_groups = 4
        bar_width = 0.1
        opacity = 0.4
        index = np.arange(0, 2*n_groups, 2)

        regroup = list()

        regroup = [summary['pixel_none_' + str(step_list[0])][0], summary['pixel_light_snow_' + str(step_list[0])][0], summary['pixel_mild_snow_' + str(step_list[0])][0], summary['pixel_heavy_snow_' + str(step_list[0])][0]]
        rects1 = self.list_of_plots[0].bar(index, regroup, bar_width, color='blue')

        shift = 0
        for val in range(len(step_list)):
            for step_size in step_list:
                regroup = list()
                shift = shift+1
                for env_name in environment_list:
                    #print 'pixel_' + env_name + '_' + str(step_size)
                    regroup.append(summary['pixel_' + env_name + '_' + str(step_size)][val+1])
                rects1 = self.list_of_plots[0].bar(index+shift*bar_width, regroup, bar_width, color=color_list_algorithms[val+1])

        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Good Pixel Density')
        plt.title('Pixel Density in Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()


        #rects1 = plt.bar(index, means_men, bar_width,
        #                 alpha=opacity,
        #                 color='b',
        #                 yerr=std_men,
        #                 error_kw=error_config,
        #                 label='Men')


    def evaluate_deviation(self, summary, step_list):


        print summary

        n_groups = 4
        bar_width = 0.1
        opacity = 0.4
        index = np.arange(0, 2*n_groups, 2)

        regroup = list()

        regroup = [summary['deviation_none_' + str(step_list[0])][0], summary['deviation_light_snow_' + str(step_list[0])][0], summary['deviation_mild_snow_' + str(step_list[0])][0], summary['deviation_heavy_snow_' + str(step_list[0])][0]]
        rects1 = self.list_of_plots[0].bar(index, regroup, bar_width, color='blue')

        shift = 0
        for val in range(len(step_list)):
            for step_size in step_list:
                regroup = list()
                shift = shift+1
                for env_name in environment_list:
                    print 'deviation_' + env_name + '_' + str(step_size)
                    regroup.append(summary['deviation_' + env_name + '_' + str(step_size)][val+1])
                rects1 = self.list_of_plots[0].bar(index+shift*bar_width, regroup, bar_width, color=color_list_algorithms[val+1])


        plt.xlabel('Data Processing Algorithm')
        plt.ylabel('Deviation')
        plt.title('Deviation of Blue Sky, Light Snow, Mild Snow and Heavy Snow')
        plt.xticks(index + 2*bar_width, ('Blue Sky', 'Light Snow', 'Mild Snow', 'Heavy Snow'))
        plt.legend()

        plt.tight_layout()


    def evaluate_obj_displacement(self, summary, step_list):

        print summary

        n_groups = 4
        bar_width = 0.1
        opacity = 0.4
        index = np.arange(0, 2*n_groups, 2)

        regroup = list()

        regroup = [summary['obj_displacement_none_' + str(step_list[0])][0], summary['obj_displacement_light_snow_' + str(step_list[0])][0], summary['obj_displacement_mild_snow_' + str(step_list[0])][0], summary['obj_displacement_heavy_snow_' + str(step_list[0])][0]]
        rects1 = self.list_of_plots[0].bar(index, regroup, bar_width, color='blue')

        shift = 0
        for val in range(len(step_list)):
            for step_size in step_list:
                regroup = list()
                shift = shift+1
                for env_name in environment_list:
                    print 'obj_displacement_' + env_name + '_' + str(step_size)
                    regroup.append(summary['obj_displacement_' + env_name + '_' + str(step_size)][val+1])
                rects1 = self.list_of_plots[0].bar(index+shift*bar_width, regroup, bar_width, color=color_list_algorithms[val+1], edgecolor='black')


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


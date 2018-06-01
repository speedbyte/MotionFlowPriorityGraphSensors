
delete_point_array = numpy.array([-65535])
index_x0_gt_sorted = numpy.argsort(x0_gt)



#x0_gt = numpy.setdiff1d(x0_gt, delete_point_array)
#y0_gt = numpy.setdiff1d(y0_gt, delete_point_array)

#x0 = numpy.setdiff1d(x0, delete_point_array)
#y0 = numpy.setdiff1d(y0, delete_point_array)


for env_name in weather_list:
    for stepSize in step_list:
        ground_truth.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][0]))
    rects1 = self.list_of_plots[0].bar(index+bar_width, ground_truth, bar_width, color='#f2f2f2')

for env_name in weather_list:
    for stepSize in step_list:
        moving_avg.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][1]))
    rects2 = self.list_of_plots[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

for env_name in weather_list:
    for stepSize in step_list:
        voted_mean.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][2]))
    rects3 = self.list_of_plots[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

for env_name in weather_list:
    for stepSize in step_list:
        ranked_mean.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][3]))
    rects4 = self.list_of_plots[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')





    for n,i in enumerate(weather_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][0]))
    print "length = " , (len(ground_truth))

    ground_truth = list()
    for n,i in enumerate(weather_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][1]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+12*bar_width, ground_truth, bar_width, color='#000000')

    ground_truth = list()
    for n,i in enumerate(weather_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][2]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+24*bar_width, ground_truth, bar_width, color='#f2f2f2')

    ground_truth = list()
    for n,i in enumerate(weather_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][3]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+36*bar_width, ground_truth, bar_width, color='#f2f2f2')

    summary = {
        'pixel_none_1':       [1.0, 0.336421821668766, 0.54807821181116871, 0.5401514593321959],
        'pixel_light_snow_1': [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_mild_snow_1':  [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_heavy_snow_1': [1.0, 0.264877294075811, 0.49535830145631016, 0.4947636068633854],
        'pixel_none_4':       [1.0, 0.336421821668766, 0.54807821181116871, 0.5401514593321959],
        'pixel_light_snow_4': [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_mild_snow_4':  [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_heavy_snow_4': [1.0, 0.264877294075811, 0.49535830145631016, 0.4947636068633854],
        'pixel_none_7':       [1.0, 0.336421821668766, 0.54807821181116871, 0.5401514593321959],
        'pixel_light_snow_7': [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_mild_snow_7':  [1.0, 0.300394977273299, 0.52059040724647876, 0.5145554052260507],
        'pixel_heavy_snow_7': [1.0, 0.264877294075811, 0.49535830145631016, 0.4947636068633854],
    }








class thread2(threading.Thread):

    def __init__(self, yaml_file_data, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_file_data = yaml_file_data
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread deviation "

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "weather"):
                current_list = weather_list

            for weather in weather_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("deviation", yaml_file_data, weather, step_size))

        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


class thread3(threading.Thread):

    def __init__(self, yaml_file_data, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_file_data = yaml_file_data
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread collision "

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "weather"):
                current_list = weather_list

            for weather in weather_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("collision", yaml_file_data, weather, step_size))

        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


class thread4(threading.Thread):

    def __init__(self, yaml_file_data, sensor_plot):
        threading.Thread.__init__(self)
        self.threadRun = False
        self.yaml_file_data = yaml_file_data
        self.sensor_plot = sensor_plot


    def stop(self):
        self.threadRun = False

    def run(self):
        self.threadRun = True
        #while ( self.threadRun ):
        print "i am in thread obj displacement"

        self.plot_at_once_figures = list()
        for step_size in step_list:

            if ( evaluation == "weather"):
                current_list = weather_list

            for weather in weather_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("obj_displacement", yaml_file_data, weather, step_size))
        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures




    #---------------        if ( 0 ):

    thread_deviation = thread2(yaml_file_data, sensor_plot)
    thread_deviation.start()

if ( 0 ):

    thread_collision = thread3(yaml_file_data, sensor_plot)
    thread_collision.start()

if ( 0 ):

    thread_obj_displacement = thread4(yaml_file_data, sensor_plot)
    thread_obj_displacement.start()





if ( thread_deviation != None ):

        while ( True ):
            time.sleep(1)
            if ( thread_deviation.getThreadState() == False ):

                plot_at_once_figures = thread_deviation.getPlotList()
                collectPlots.append(plot_at_once_figures)
                plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                # summary
                summary = sensor_plot.get_summary()
                figures = Figures(1)
                figures.evaluate_deviation(summary, step_list)
                figures.save_figure("deviation", "summary")

                break


    if ( thread_collision != None ):

        while ( True ):
            time.sleep(1)
            if ( thread_collision.getThreadState() == False ):

                plot_at_once_figures = thread_collision.getPlotList()
                plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                # summary
                #summary = sensor_plot.get_summary()
                #figures = Figures(1)
                #figures.evaluate_collision(summary, step_list)
                #figures.save_figure("collision", "summary")

                break


    # ---------------------------------
    if ( thread_obj_displacement != None ):

        while ( True ):
            time.sleep(1)
            if ( thread_obj_displacement.getThreadState() == False ):

                plot_at_once_figures = thread_obj_displacement.getPlotList()
                plot_at_once(plot_at_once_figures, sensor_plot.getSensorIndex())

                # summary
                summary = sensor_plot.get_summary()
                figures = Figures(1)
                figures.evaluate_obj_displacement(summary, step_list)
                figures.save_figure("obj_displacement", "summary")

                break

                #scenario_displacement_occurence()
                #histogramm()




###2

        elif ( measuring_parameter == "deviation"):
        data_points_gt = yaml_file_data[data_list[0]]
        print "getting " , data_list[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getDeviationPoints(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "stddev"):
        data_points_gt = yaml_file_data[data_list[0]]
        print "getting " , data_list[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getStdDev(data_points_gt, data_points_gt)

    elif ( measuring_parameter == "collision"):
        data_points_gt = yaml_file_data[data_list[0]]
        print "getting " , data_list[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getCollisionPoints(data_points_gt, data_points_gt)
        # collision sorted

    elif ( measuring_parameter == "obj_displacement"):
        data_points_gt = yaml_file_data[data_list[0]]
        print "getting " , data_list[0]
        x_axis_gt, y_axis_gt, y_axis_gt_mean = self.getObjectDisplacement(data_points_gt, data_points_gt)


###3
    elif ( measuring_parameter == "deviation"):
        x_axis, y_axis, y_axis_mean = self.getDeviationPoints(data_points_gt, data_points)
    elif ( measuring_parameter == "stddev"):
        x_axis, y_axis, y_axis_mean = self.getStdDev(data_points_gt, data_points)
    elif ( measuring_parameter == "collision"):
        x_axis, y_axis, y_axis_mean = self.getCollisionPoints(data_points_gt, data_points)
    elif ( measuring_parameter == "obj_displacement"):
        x_axis, y_axis, y_axis_mean = self.getObjectDisplacement(data_points_gt, data_points)

## tmeplate_to_yaml_mapping

def templateToYamlMapping(self, meausuring_parameter, yaml_file_data, weather, step_size):

    if ( meausuring_parameter == "visible_pixels"):
        template_name_ = template_name_of_evaluation_data
        template_name_gt = template_name_of_evaluation_data_gt
    elif ( meausuring_parameter == "deviation"):
        template_name_ = template_name_of_collision
        template_name_gt = template_name_of_collision_points_gt
    elif ( meausuring_parameter == "collision"):
        template_name_ = template_name_of_collision
        template_name_gt = template_name_of_collision_points_gt
    elif ( meausuring_parameter == "obj_displacement"):
        template_name_ = template_name_of_obj_displacement
        template_name_gt = template_name_of_obj_displacement_gt

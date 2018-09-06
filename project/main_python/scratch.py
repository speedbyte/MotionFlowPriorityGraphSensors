
delete_point_array = numpy.array([-65535])
index_x0_gt_sorted = numpy.argsort(x0_gt)



#x0_gt = numpy.setdiff1d(x0_gt, delete_point_array)
#y0_gt = numpy.setdiff1d(y0_gt, delete_point_array)

#x0 = numpy.setdiff1d(x0, delete_point_array)
#y0 = numpy.setdiff1d(y0, delete_point_array)


for env_name in noise_list:
    for stepSize in step_list:
        ground_truth.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][0]))
    rects1 = self.list_of_plots[0].bar(index+bar_width, ground_truth, bar_width, color='#f2f2f2')

for env_name in noise_list:
    for stepSize in step_list:
        moving_avg.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][1]))
    rects2 = self.list_of_plots[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

for env_name in noise_list:
    for stepSize in step_list:
        voted_mean.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][2]))
    rects3 = self.list_of_plots[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

for env_name in noise_list:
    for stepSize in step_list:
        ranked_mean.append((summary["visible_pixels_" + env_name + '_' + str(stepSize)][3]))
    rects4 = self.list_of_plots[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')





    for n,i in enumerate(noise_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][0]))
    print "length = " , (len(ground_truth))

    ground_truth = list()
    for n,i in enumerate(noise_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][1]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+12*bar_width, ground_truth, bar_width, color='#000000')

    ground_truth = list()
    for n,i in enumerate(noise_list):
        for stepSize in step_list:
            ground_truth.append((summary["visible_pixels_" + i + '_' + str(stepSize)][2]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+24*bar_width, ground_truth, bar_width, color='#f2f2f2')

    ground_truth = list()
    for n,i in enumerate(noise_list):
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









class thread_example(threading.Thread):

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

            if ( evaluation == "noise"):
                current_list = noise_list

            for noise in noise_list:
                self.plot_at_once_figures.append(self.sensor_plot.templateToYamlMapping("collision", yaml_file_data, noise, step_size))

        self.threadRun = False

    def getThreadState(self):
        return self.threadRun

    def getPlotList(self):
        return self.plot_at_once_figures


    thread_deviation = thread2(yaml_file_data, sensor_plot)
    thread_deviation.start()



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
                figures.bargraph_deviation(summary, step_list)
                figures.save_figure("deviation", "summary")

                break



def templateToYamlMapping(self, meausuring_parameter, yaml_file_data, noise, step_size):

    elif ( meausuring_parameter == "objdisplacement"):
        template_name_ = template_name_of_objdisplacement
        template_name_gt = template_name_of_objdisplacement_gt


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


    #scratch 01233
    data = numpy.array(newshape)
    if ( measuring_parameter == "algorithm_pixels_count" or measuring_parameter == "l2_cumulative_distance_good_pixels" or measuring_parameter == "ma_cumulative_distance_good_pixels"):
        x0_gt, y0_gt = data.T
        y_axis = x0_gt/y0_gt
    elif (measuring_parameter == "collisionpoints"):
        cur, x0_gt, y0_gt = data.T   ## change 1
        y_axis = numpy.sqrt((x0_gt - x0_gt) ** 2 + (y0_gt - y0_gt) ** 2) ## change 2
    else:
        x0_gt, y0_gt = data.T
        y_axis = y0_gt

        #index = [0]
        #y_axis = numpy.delete(y_axis, index)



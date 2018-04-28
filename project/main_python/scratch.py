
delete_point_array = numpy.array([-65535])
index_x0_gt_sorted = numpy.argsort(x0_gt)



#x0_gt = numpy.setdiff1d(x0_gt, delete_point_array)
#y0_gt = numpy.setdiff1d(y0_gt, delete_point_array)

#x0 = numpy.setdiff1d(x0, delete_point_array)
#y0 = numpy.setdiff1d(y0, delete_point_array)


for env_name in environment_list:
    for stepSize in step_list:
        ground_truth.append((summary['pixel_' + env_name + '_' + str(stepSize)][0]))
    rects1 = self.list_of_plots[0].bar(index+bar_width, ground_truth, bar_width, color='#f2f2f2')

for env_name in environment_list:
    for stepSize in step_list:
        moving_avg.append((summary['pixel_' + env_name + '_' + str(stepSize)][1]))
    rects2 = self.list_of_plots[0].bar(index+bar_width, moving_avg, bar_width, color='#cccccc')

for env_name in environment_list:
    for stepSize in step_list:
        voted_mean.append((summary['pixel_' + env_name + '_' + str(stepSize)][2]))
    rects3 = self.list_of_plots[0].bar(index+2*bar_width, voted_mean, bar_width, color='#808080')

for env_name in environment_list:
    for stepSize in step_list:
        ranked_mean.append((summary['pixel_' + env_name + '_' + str(stepSize)][3]))
    rects4 = self.list_of_plots[0].bar(index+3*bar_width, ranked_mean, bar_width, color='#000000')





    for n,i in enumerate(environment_list):
        for stepSize in step_list:
            ground_truth.append((summary['pixel_' + i + '_' + str(stepSize)][0]))
    print "length = " , (len(ground_truth))

    ground_truth = list()
    for n,i in enumerate(environment_list):
        for stepSize in step_list:
            ground_truth.append((summary['pixel_' + i + '_' + str(stepSize)][1]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+12*bar_width, ground_truth, bar_width, color='#000000')

    ground_truth = list()
    for n,i in enumerate(environment_list):
        for stepSize in step_list:
            ground_truth.append((summary['pixel_' + i + '_' + str(stepSize)][2]))
    print "length = " , (len(ground_truth))
    rects1 = self.list_of_plots[0].bar(index+24*bar_width, ground_truth, bar_width, color='#f2f2f2')

    ground_truth = list()
    for n,i in enumerate(environment_list):
        for stepSize in step_list:
            ground_truth.append((summary['pixel_' + i + '_' + str(stepSize)][3]))
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

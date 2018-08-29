#!/usr/bin/python
# _*_ encoding:utf-8 _*_

just_ground_truth = False

dict_datafilters = {
    "ground_truth": "ground truth",
    "datafilter_0": "datafilter_0",
    "datafilter_1": "datafilter_1",
    "datafilter_2": "datafilter_2",
    "datafilter_3": "datafilter_3",
}

dict_label_weather = {
    "ground_truth" : "ground_truth",
    "blue_sky"     : "blue_sky",
    "light_snow"   : "light_snow",
    "mild_snow"    : "mild_snow",
    "heavy_snow"   : "heavy_snow",
}

dict_color_weather = {
    "ground_truth" : "green",
    "blue_sky"     : "blue",
    "light_snow"   : "red",
    "mild_snow"    : "brown",
    "heavy_snow"   : "black",
}

y_axis_label_dict = {
    "ground_truth_pixels_count" : "ground_truth_pixels_count",
    "algorithm_pixels_count" : "algorithm_pixels_count/ground_truth_pixels_count",
    "good_pixels_l2_error": "good_pixels/algorithm_pixels_count",
    "good_pixels_ma_error": "good_pixels/algorithm_pixels_count",
     "ma_error_all" : "ma_error_all",
    "l1_error_all" : "l1_error_all",
    "l2_error_all" : "l2_error_all",
    "collisionpoints" : "deviation",
}

parameter_list = [  ]
parameter_list = [ "algorithm_pixels_count", "good_pixels_l2_error", "good_pixels_ma_error", "ma_error_all", "l1_error_all", "l2_error_all"]
parameter_list = [ "algorithm_pixels_count", "good_pixels_l2_error", "good_pixels_ma_error", "ma_error_all", "l2_error_all", "collisionpoints"]
parameter_list = [ "ground_truth_pixels_count", "algorithm_pixels_count", "l1_method_good_pixel_count", "l2_method_good_pixel_count", "ma_method_good_pixel_count"]
parameter_list = [ "ground_truth_pixels_count"]

datafilter_list  = [ "0", ]

algorithm_list   = ["LK", "FB",]
algorithm_list   = ["FB", ]

weather_list = ["blue_sky", "light_snow", "mild_snow", "heavy_snow"] #night
weather_list = ["ground_truth", "blue_sky", "heavy_snow",]
weather_list = ["ground_truth", "blue_sky",]

sensor_list      = [0, 1, 2]
sensor_list      = [0, ]
step_list        = [1]
fps_list         = ["30", "15", "10", "7.5"]#night
fps_list         = ["30",]

color_list_algorithms = ["blue", "green", "blue", "black", "brown"]
color_list_weather = ["blue", "gray", "brown", "black", "brown"]

#color_list_algorithms = ['#f2f2f2', '#cccccc', '#808080', '#000000']
color_list_bar = ['#f2f2f2', '#cccccc', '#808080', '#000000']
label_list_bar = ['ground truth', 'moving average', 'voted mean', 'ranked mean']

template_name_of_evaluation_data_gt = [
    "evaluation_data_ground_truth_",
]
template_name_of_evaluation_data = [
    "evaluation_data_results_",
]

template_name_of_collisionpoints_gt = [
    "collisionpoints_data_ground_truth_",
]

template_name_of_collisionpoints = [
    "collisionpoints_data_results_",
]

template_name_of_objdisplacement_gt = [
    "objdisplacement_ground_truth_",
]

template_name_of_objdisplacement = [
    "objdisplacement_data_results_",
]

template_name_of_displacement_occurence = [
    "scenario_displacement_occurence_sensor_index_1_datafilter_0_ground_truth",
    "scenario_displacement_occurence_sensor_index_1_datafilter_0results_none_",
]

color_of_displacement_occurence = ["blue", "red"]
assert(len(template_name_of_displacement_occurence) == len(color_of_displacement_occurence))

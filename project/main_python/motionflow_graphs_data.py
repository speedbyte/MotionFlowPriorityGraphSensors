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
    "ground_truth" : "blue",
    "blue_sky"     : "red",
    "light_snow"   : "yellow",
    "mild_snow"    : "green",
    "heavy_snow"   : "brown",
}

y_axis_label_dict = {
    "visible_pixels" : "visible_pixel/ground_truth_pixels",
    "good_pixels": "good_pixels/visible_pixels",
     "ma_distance" : "ma_distance",
    "l1_distance" : "l1_distance",
    "l2_distance" : "l2_distance",
}

parameter_list = [  ]
parameter_list = [ "visible_pixels", "good_pixels", "ma_distance", "l1_distance", "l2_distance"]

datafilter_list  = [ "0", ]
algorithm_list   = ["LK", "FB",]
#algorithm_list   = ["LK", ]
weather_list = ["blue_sky", "light_snow", "mild_snow", "heavy_snow"] #night
weather_list = ["ground_truth", "blue_sky", "heavy_snow",]
#weather_list = ["blue_sky",]
sensor_list      = [0, 1, 2]
step_list        = [5]
fps_list         = ["30", "15", "10", "7.5"]#night
fps_list         = ["30",]

color_list_algorithms = ["blue", "red", "yellow", "green", "brown"]
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

template_name_of_collision_points_gt = [
    "collision_points_ground_truth_",
]

template_name_of_obj_displacement = [
    "obj_displacement_datafilter_0results_",
    "obj_displacement_datafilter_1results_",
    "obj_displacement_datafilter_2results_",
    "obj_displacement_datafilter_3results_",

]

template_name_of_obj_displacement_gt = [
    "obj_displacement_ground_truth_",
]

template_name_of_collision = [

    "collision_points_datafilter_0results_",
    "collision_points_datafilter_1results_",
    "collision_points_datafilter_2results_",
    "collision_points_datafilter_3results_",

]


template_name_of_displacement_occurence = [
    "scenario_displacement_occurence_sensor_index_1_datafilter_0_ground_truth",
    "scenario_displacement_occurence_sensor_index_1_datafilter_0results_none_",
]

color_of_displacement_occurence = ["blue", "red"]
assert(len(template_name_of_displacement_occurence) == len(color_of_displacement_occurence))

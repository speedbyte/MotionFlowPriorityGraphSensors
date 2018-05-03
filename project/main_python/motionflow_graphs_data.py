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

datafilter_list = [ "1", "2"]

dict_environment = {
    "none" : 0,
    "light_snow"   : 1,
    "mild_snow"    : 2,
    "heavy_snow"   : 3,
}

dict_environment_label = {
    "none":  "blue_sky",
    "env_0": "light snow",
    "env_1": "mild snow",
    "env_2": "heavy snow",
}

environment_list = ["none", "light_snow", "mild_snow", "heavy_snow"] #night
environment_list = ["none",]

number_of_sensors = 1

step_list = [5]

fps_list = ["30", "15", "10", "7.5"]#night
fps_list = ["30",]

evaluation = "environment"

color_list_algorithms = ["blue", "red", "yellow", "green", "dummy"]

color_list_environment = ["blue", "gray", "brown", "black", "dummy"]

label_list_algorithm = [dict_datafilters["ground_truth"], dict_datafilters["datafilter_0"], dict_datafilters["datafilter_1"], dict_datafilters["datafilter_2"]]

label_list_enironment = [dict_environment_label["none"], dict_environment_label["env_0"], dict_environment_label["env_1"], dict_environment_label["env_2"]]

#color_list_algorithms = ['#f2f2f2', '#cccccc', '#808080', '#000000']
color_list_bar = ['#f2f2f2', '#cccccc', '#808080', '#000000']
label_list_bar = ['ground truth', 'moving average', 'voted mean', 'ranked mean']


list_of_pixel_density_ground_truth = [
    "pixel_density_sensor_index_1_ground_truth",
    "pixel_density_sensor_index_2_ground_truth",
]

list_of_collision_ground_truth = [
    "collision_points_sensor_index_1_ground_truth",
    "collision_points_sensor_index_2_ground_truth",
]

list_of_obj_displacement_ground_truth = [
    "obj_displacement_sensor_index_1_ground_truth",
    "obj_displacement_sensor_index_2_ground_truth",
]


template_of_pixel_density = [
    "pixel_density_sensor_index_1_datafilter_0results_FB_",
    "pixel_density_sensor_index_1_datafilter_1results_FB_",
    "pixel_density_sensor_index_1_datafilter_2results_FB_",
    "pixel_density_sensor_index_1_datafilter_3results_FB_",

    "pixel_density_sensor_index_1_datafilter_0results_FB_",
    "pixel_density_sensor_index_1_datafilter_1results_FB_",
    "pixel_density_sensor_index_1_datafilter_2results_FB_",
    "pixel_density_sensor_index_1_datafilter_3results_FB_",

    "pixel_density_sensor_index_1_datafilter_0results_FB_",
    "pixel_density_sensor_index_1_datafilter_1results_FB_",
    "pixel_density_sensor_index_1_datafilter_2results_FB_",
    "pixel_density_sensor_index_1_datafilter_3results_FB_",

    "pixel_density_sensor_index_1_datafilter_0results_FB_",
    "pixel_density_sensor_index_1_datafilter_1results_FB_",
    "pixel_density_sensor_index_1_datafilter_2results_FB_",
    "pixel_density_sensor_index_1_datafilter_3results_FB_",

]


template_of_obj_displacement = [
    "obj_displacement_sensor_index_1_datafilter_0results_FB_",
    "obj_displacement_sensor_index_1_datafilter_1results_FB_",
    "obj_displacement_sensor_index_1_datafilter_2results_FB_",
    "obj_displacement_sensor_index_1_datafilter_3results_FB_",

    "obj_displacement_sensor_index_1_datafilter_0results_FB_",
    "obj_displacement_sensor_index_1_datafilter_1results_FB_",
    "obj_displacement_sensor_index_1_datafilter_2results_FB_",
    "obj_displacement_sensor_index_1_datafilter_3results_FB_",

    "obj_displacement_sensor_index_1_datafilter_0results_FB_",
    "obj_displacement_sensor_index_1_datafilter_1results_FB_",
    "obj_displacement_sensor_index_1_datafilter_2results_FB_",
    "obj_displacement_sensor_index_1_datafilter_3results_FB_",

    "obj_displacement_sensor_index_1_datafilter_0results_FB_",
    "obj_displacement_sensor_index_1_datafilter_1results_FB_",
    "obj_displacement_sensor_index_1_datafilter_2results_FB_",
    "obj_displacement_sensor_index_1_datafilter_3results_FB_",

]


template_of_collision = [

    "collision_points_sensor_index_1_datafilter_0results_FB_",
    "collision_points_sensor_index_1_datafilter_1results_FB_",
    "collision_points_sensor_index_1_datafilter_2results_FB_",
    "collision_points_sensor_index_1_datafilter_3results_FB_",

    "collision_points_sensor_index_1_datafilter_0results_FB_",
    "collision_points_sensor_index_1_datafilter_1results_FB_",
    "collision_points_sensor_index_1_datafilter_2results_FB_",
    "collision_points_sensor_index_1_datafilter_3results_FB_",

    "collision_points_sensor_index_1_datafilter_0results_FB_",
    "collision_points_sensor_index_1_datafilter_1results_FB_",
    "collision_points_sensor_index_1_datafilter_2results_FB_",
    "collision_points_sensor_index_1_datafilter_3results_FB_",

    "collision_points_sensor_index_1_datafilter_0results_FB_",
    "collision_points_sensor_index_1_datafilter_1results_FB_",
    "collision_points_sensor_index_1_datafilter_2results_FB_",
    "collision_points_sensor_index_1_datafilter_3results_FB_",

]


template_of_displacement_occurence = [
    "scenario_displacement_occurence_sensor_index_1_datafilter_0_ground_truth",
    "scenario_displacement_occurence_sensor_index_1_datafilter_0results_FB_none_",
]

color_of_displacement_occurence = ["blue", "red"]
assert(len(template_of_displacement_occurence) == len(color_of_displacement_occurence))

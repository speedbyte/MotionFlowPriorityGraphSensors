#!/usr/bin/python
# _*_ encoding:utf-8 _*_


dict_datafilters = {
    "ground_truth": "ground truth",
    "datafilter_0": "moving average",
    "datafilter_1": "voted mean",
    "datafilter_2": "ranked mean",
    "datafilter_3": "something",
}

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
#environment_list = ["none",]

step_list = [2, 6, 10]

fps_list = ["30", "15", "10", "7.5"]#night
fps_list = ["30",]

evaluation = "environment"

datafilter_list = ["0", "1", "2"]

color_list_algorithms = ["blue", "red", "yellow", "green", "dummy"]

color_list_environment = ["blue", "gray", "brown", "black", "dummy"]

label_list_algorithm = [dict_datafilters["ground_truth"], dict_datafilters["datafilter_0"], dict_datafilters["datafilter_1"], dict_datafilters["datafilter_2"]]

label_list_enironment = [dict_environment_label["none"], dict_environment_label["env_0"], dict_environment_label["env_1"], dict_environment_label["env_2"]]

color_list_bar = ['#f2f2f2', '#cccccc', '#808080', '#000000']


list_of_pixel_density_ground_truth = [
    "pixel_density_ground_truth",
]

list_of_collision_ground_truth = [
    "collision_points_ground_truth",
]

list_of_obj_displacement_ground_truth = [
    "obj_displacement_ground_truth",
]


template_of_pixel_density = [
    "pixel_density_frame_skip_1_datafilter_0results_FB_",
    "pixel_density_frame_skip_1_datafilter_1results_FB_",
    "pixel_density_frame_skip_1_datafilter_2results_FB_",
    "pixel_density_frame_skip_1_datafilter_3results_FB_",

    "pixel_density_frame_skip_1_datafilter_0results_FB_",
    "pixel_density_frame_skip_1_datafilter_1results_FB_",
    "pixel_density_frame_skip_1_datafilter_2results_FB_",
    "pixel_density_frame_skip_1_datafilter_3results_FB_",

    "pixel_density_frame_skip_1_datafilter_0results_FB_",
    "pixel_density_frame_skip_1_datafilter_1results_FB_",
    "pixel_density_frame_skip_1_datafilter_2results_FB_",
    "pixel_density_frame_skip_1_datafilter_3results_FB_",

    "pixel_density_frame_skip_1_datafilter_0results_FB_",
    "pixel_density_frame_skip_1_datafilter_1results_FB_",
    "pixel_density_frame_skip_1_datafilter_2results_FB_",
    "pixel_density_frame_skip_1_datafilter_3results_FB_",

]


template_of_obj_displacement = [
    "obj_displacement_frame_skip_1_datafilter_0results_FB_",
    "obj_displacement_frame_skip_1_datafilter_1results_FB_",
    "obj_displacement_frame_skip_1_datafilter_2results_FB_",
    "obj_displacement_frame_skip_1_datafilter_3results_FB_",

    "obj_displacement_frame_skip_1_datafilter_0results_FB_",
    "obj_displacement_frame_skip_1_datafilter_1results_FB_",
    "obj_displacement_frame_skip_1_datafilter_2results_FB_",
    "obj_displacement_frame_skip_1_datafilter_3results_FB_",

    "obj_displacement_frame_skip_1_datafilter_0results_FB_",
    "obj_displacement_frame_skip_1_datafilter_1results_FB_",
    "obj_displacement_frame_skip_1_datafilter_2results_FB_",
    "obj_displacement_frame_skip_1_datafilter_3results_FB_",

    "obj_displacement_frame_skip_1_datafilter_0results_FB_",
    "obj_displacement_frame_skip_1_datafilter_1results_FB_",
    "obj_displacement_frame_skip_1_datafilter_2results_FB_",
    "obj_displacement_frame_skip_1_datafilter_3results_FB_",

]


template_of_collision = [

    "collision_points_frame_skip_1_datafilter_0results_FB_",
    "collision_points_frame_skip_1_datafilter_1results_FB_",
    "collision_points_frame_skip_1_datafilter_2results_FB_",
    "collision_points_frame_skip_1_datafilter_3results_FB_",

    "collision_points_frame_skip_1_datafilter_0results_FB_",
    "collision_points_frame_skip_1_datafilter_1results_FB_",
    "collision_points_frame_skip_1_datafilter_2results_FB_",
    "collision_points_frame_skip_1_datafilter_3results_FB_",

    "collision_points_frame_skip_1_datafilter_0results_FB_",
    "collision_points_frame_skip_1_datafilter_1results_FB_",
    "collision_points_frame_skip_1_datafilter_2results_FB_",
    "collision_points_frame_skip_1_datafilter_3results_FB_",

    "collision_points_frame_skip_1_datafilter_0results_FB_",
    "collision_points_frame_skip_1_datafilter_1results_FB_",
    "collision_points_frame_skip_1_datafilter_2results_FB_",
    "collision_points_frame_skip_1_datafilter_3results_FB_",

]


template_of_displacement_occurence = [
    "scenario_displacement_occurence_frame_skip_1_datafilter_0_ground_truth",
    "scenario_displacement_occurence_frame_skip_1_datafilter_0results_FB_none_",
]

color_of_displacement_occurence = ["blue", "red"]
assert(len(template_of_displacement_occurence) == len(color_of_displacement_occurence))

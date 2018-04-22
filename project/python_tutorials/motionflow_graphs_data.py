#!/usr/bin/python
# _*_ encoding:utf-8 _*_


list_of_shape_metrics_no_noise = [
    "shape_pointsframe_skip1_dataprocessing_0_generated",
    "shape_pointsframe_skip1_dataprocessing_0results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_1results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_2results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_3results_FB_none_",
]
color_of_shape_metrics_no_noise = ["blue", "red", "green", "yellow", "black"]
assert(len(list_of_shape_metrics_no_noise) == len(color_of_shape_metrics_no_noise))


list_of_shape_metrics_noise = [
    "shape_pointsframe_skip1_dataprocessing_0results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_1results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_2results_FB_none_",
    "shape_pointsframe_skip1_dataprocessing_3results_FB_none_",

    "shape_pointsframe_skip1_dataprocessing_0results_FB_light_snow_",
    "shape_pointsframe_skip1_dataprocessing_1results_FB_light_snow_",
    "shape_pointsframe_skip1_dataprocessing_2results_FB_light_snow_",
    "shape_pointsframe_skip1_dataprocessing_3results_FB_light_snow_",

    "shape_pointsframe_skip1_dataprocessing_0results_FB_mild_snow_",
    "shape_pointsframe_skip1_dataprocessing_1results_FB_mild_snow_",
    "shape_pointsframe_skip1_dataprocessing_2results_FB_mild_snow_",
    "shape_pointsframe_skip1_dataprocessing_3results_FB_mild_snow_",

    "shape_pointsframe_skip1_dataprocessing_0results_FB_heavy_snow_",
    "shape_pointsframe_skip1_dataprocessing_1results_FB_heavy_snow_",
    "shape_pointsframe_skip1_dataprocessing_2results_FB_heavy_snow_",
    "shape_pointsframe_skip1_dataprocessing_3results_FB_heavy_snow_",

]
color_of_shape_metrics_noise = ["red", "green", "yellow", "black"]
assert(len(list_of_shape_metrics_noise)/4 == len(color_of_shape_metrics_noise))



list_of_collision_metrics_no_noise = [
    "collision_pointsframe_skip1_dataprocessing_0_generated",
    "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",
]
color_of_collision_metrics_no_noise = ["blue", "red", "green", "yellow", "black"]
assert(len(list_of_collision_metrics_no_noise) == len(color_of_collision_metrics_no_noise))



list_of_collision_metrics_noise = [

    "collision_pointsframe_skip1_dataprocessing_0results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_none_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_none_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_light_snow_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_light_snow_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_light_snow_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_light_snow_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_mild_snow_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_mild_snow_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_mild_snow_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_mild_snow_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_heavy_snow_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_heavy_snow_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_heavy_snow_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_heavy_snow_",

]
color_of_collision_metrics_noise  = ["red", "green", "yellow", "black"]
assert(len(list_of_collision_metrics_noise)/4 == len(color_of_collision_metrics_noise))



list_of_displacement_occurence_metrics = [
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0_generated",
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0results_FB_none_",
]
color_of_displacement_occurence_metrics = ["blue", "red"]
assert(len(list_of_displacement_occurence_metrics) == len(color_of_displacement_occurence_metrics))

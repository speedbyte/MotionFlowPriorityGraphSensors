#!/usr/bin/python
# _*_ encoding:utf-8 _*_


environment_list = ["none", "light_snow_", "mild_snow_", "heavy_snow_"]#night
#environment_list = ["none",]

data_processing_list = ["0", "1", "2"]

color_list = ["blue", "red", "green", "yellow", "black"]

list_of_shape_metrics_no_noise = [
    "pixel_density_ground_truth",
]

list_of_collision_metrics_no_noise = [
    "collision_points_ground_truth",
]

list_of_shape_metrics_noise = [
    "pixel_densityframe_skip1_dataprocessing_0results_FB_none_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_none_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_none_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_none_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_light_snow_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_light_snow_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_light_snow_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_light_snow_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_mild_snow_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_mild_snow_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_mild_snow_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_mild_snow_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_heavy_snow_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_heavy_snow_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_heavy_snow_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_heavy_snow_",

]

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


list_of_displacement_occurence_metrics = [
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0_ground_truth",
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0results_FB_none_",
]

color_of_displacement_occurence_metrics = ["blue", "red"]
assert(len(list_of_displacement_occurence_metrics) == len(color_of_displacement_occurence_metrics))

#!/usr/bin/python
# _*_ encoding:utf-8 _*_


dict_datafilters = {
    "datafilter_0" : "moving_average",
    "datafilter_1" : "voted_mean",
    "datafilter_2" : "ranked_mean",
    "datafilter_3" : "something",
}

environment_list = ["none", "light_snow", "mild_snow", "heavy_snow"]#night
#environment_list = ["none",]

step_list = ["1", "2", "3", "4"]#night
step_list = ["1",]

fps_list = ["30", "15", "10", "7.5"]#night
fps_list = ["30",]

evaluation = "environment"



data_processing_list = ["0", "1", "2"]

color_list = ["blue", "red", "green", "yellow", "black"]

color_list_bar = ['#f2f2f2', '#cccccc', '#808080', '#000000']


list_of_pixel_density_ground_truth = [
    "pixel_density_ground_truth",
]

list_of_collision_ground_truth = [
    "collision_points_ground_truth",
]

list_of_obj_displacement_ground_truth = [
    "collision_points_ground_truth",
]


template_of_pixel_density = [
    "pixel_densityframe_skip1_dataprocessing_0results_FB_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_",

    "pixel_densityframe_skip1_dataprocessing_0results_FB_",
    "pixel_densityframe_skip1_dataprocessing_1results_FB_",
    "pixel_densityframe_skip1_dataprocessing_2results_FB_",
    "pixel_densityframe_skip1_dataprocessing_3results_FB_",

]


template_of_obj_displacement = [
    "obj_displacementframe_skip1_dataprocessing_0results_FB_",
    "obj_displacementframe_skip1_dataprocessing_1results_FB_",
    "obj_displacementframe_skip1_dataprocessing_2results_FB_",
    "obj_displacementframe_skip1_dataprocessing_3results_FB_",

    "obj_displacementframe_skip1_dataprocessing_0results_FB_",
    "obj_displacementframe_skip1_dataprocessing_1results_FB_",
    "obj_displacementframe_skip1_dataprocessing_2results_FB_",
    "obj_displacementframe_skip1_dataprocessing_3results_FB_",

    "obj_displacementframe_skip1_dataprocessing_0results_FB_",
    "obj_displacementframe_skip1_dataprocessing_1results_FB_",
    "obj_displacementframe_skip1_dataprocessing_2results_FB_",
    "obj_displacementframe_skip1_dataprocessing_3results_FB_",

    "obj_displacementframe_skip1_dataprocessing_0results_FB_",
    "obj_displacementframe_skip1_dataprocessing_1results_FB_",
    "obj_displacementframe_skip1_dataprocessing_2results_FB_",
    "obj_displacementframe_skip1_dataprocessing_3results_FB_",

]


template_of_collision = [

    "collision_pointsframe_skip1_dataprocessing_0results_FB_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_",

    "collision_pointsframe_skip1_dataprocessing_0results_FB_",
    "collision_pointsframe_skip1_dataprocessing_1results_FB_",
    "collision_pointsframe_skip1_dataprocessing_2results_FB_",
    "collision_pointsframe_skip1_dataprocessing_3results_FB_",

]


template_of_displacement_occurence = [
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0_ground_truth",
    "scenario_displacement_occurenceframe_skip1_dataprocessing_0results_FB_none_",
]

color_of_displacement_occurence = ["blue", "red"]
assert(len(template_of_displacement_occurence) == len(color_of_displacement_occurence))

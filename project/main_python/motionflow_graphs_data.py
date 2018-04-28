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

step_list = [1, 4, 7]

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

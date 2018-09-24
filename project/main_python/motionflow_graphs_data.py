#!/usr/bin/python
# _*_ encoding:utf-8 _*_

just_ground_truth = False
SCALE = 1
OUTLIER = 100000

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "cpp"
scenario = "two"

file_list = list()

file_final = "/local/git/MotionFlowPriorityGraphSensors/project/main_python/values.yml"


dict_parameter_extended = {
    "extended_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels" : "percentage of sroi pixels error to eroi pixels error" ,
    "extended_sroi_total_pixel_eroi_total_pixel" : "percentage of sroi pixels to eroi pixels",
    "sroi_total_pixel" : "number of pixels in the special roi",
    "eroi_total_pixel" : "number of pixels in the entire roi",

}

dict_datafilters = {
    "ground_truth": "ground truth",
    "datafilter_0": "datafilter_0",
    "datafilter_1": "datafilter_1",
    "datafilter_2": "datafilter_2",
    "datafilter_3": "datafilter_3",
}

dict_label_noise = {
    "ground_truth" : "ground_truth",
    "blue_sky"     : "blue_sky",
    "light_snow"   : "light_snow",
    "mild_snow"    : "mild_snow",
    "heavy_snow"   : "heavy_snow",
}

dict_color_noise = {
    "ground_truth" : "green",
    "blue_sky"     : "blue",
    "light_snow"   : "red",
    "mild_snow"    : "brown",
    "heavy_snow"   : "black",
}

y_axis_label_dict = {

}

configuration_list = [["eroi_total_pixel_FB_ground_truth_1_0", "eroi_total_pixel_FB_blue_sky_1_0", "sroi_total_pixel_FB_blue_sky_1_0"], ["eroi_total_pixel_LK_ground_truth_1_0", "eroi_total_pixel_LK_blue_sky_1_0", "sroi_total_pixel_LK_blue_sky_1_0"]]
configuration_list_extended  = [["extended_sroi_total_pixel_eroi_total_pixel_FB_ground_truth_1_0"]]

parameter_list = [ ]
#parameter_list = [ "algorithm_pixels", "ground_truth_sroi_pixels", "l1_total_good_pixels", "l2_total_good_pixels","ma_total_good_pixels", "algorithm_sroi_pixels"]
parameter_list = ["eroi_total_pixel", "eroi_l2_cumulative_error_all_pixels", "sroi_total_pixel", "eroi_ma_cumulative_error_all_pixels", "sroi_l2_cumulative_error_all_pixels",]
#parameter_list = ["eroi_total_pixel", "sroi_total_pixel", ]
parameter_list_extended = [["sroi_total_pixel", "eroi_total_pixel" ], ["sroi_l2_cumulative_error_all_pixels", "eroi_l2_cumulative_error_all_pixels" ],] #"sroi_l2_cumulative_error_all_pixels/l2_cumulative_error_all_pixels", "collision", ]
#parameter_list_extended = [ ]


datafilter_list  = [ "0", ]

#algorithm_list   = ["LK", "FB",]
#algorithm_list   = ["FB",]
algorithm_list   = ["FB", "TVL", "LK"]

file_ground_truth = "/local/git/MotionFlowPriorityGraphSensors/datasets/" + dataset + "_dataset/data/stereo_flow/two/ground_truth/values_ground_truth.yml"

for algorithm in algorithm_list:
    file_list.append("/local/git/MotionFlowPriorityGraphSensors/datasets/" + dataset + "_dataset/results/stereo_flow/two/results_" + algorithm + "_blue_sky_30_1/values_" + algorithm + ".yml")

noise_list = ["ground_truth", "blue_sky", "light_snow", "mild_snow", "heavy_snow"] #night
noise_list = ["ground_truth", "blue_sky", "heavy_snow",]
noise_list = ["ground_truth", "blue_sky",]


sensor_list      = [0, 1, 2]
sensor_list      = [0, ]
step_list        = [1]
fps_list         = ["30", "15", "10", "7.5"]#night
fps_list         = ["30",]

color_list_algorithms = ["blue", "green", "blue", "black", "brown"]
color_list_noise = ["blue", "gray", "brown", "black", "brown"]
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

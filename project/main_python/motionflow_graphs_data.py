#!/usr/bin/python
# _*_ encoding:utf-8 _*_

just_ground_truth = False
SCALE = 1
OUTLIER = 100000

#figure = Figures(1)
#figure.plot_single(x_axis_1, nucleus, [-1,1])
#def save_figure(self, type_of_graph, measuring_index, stepSize=65535, sensor_index=65535 ):
#figure.save_figure("point_graph_multiple", algorithm, 1, 0)

#output_folder = '/local/git/MotionFlowPriorityGraphSensors/overleaf/paper_1/'
output_folder = '/local/tmp/eaes/'

dataset = "vires"
scenario = "two"

noise_list = ["ground_truth", "blue_sky", "light_snow", "mild_snow", "heavy_snow"] #night
noise_list = ["ground_truth", "blue_sky", "heavy_snow",]
#noise_list = ["ground_truth", "blue_sky",]

#algorithm_list   = ["TVL", "FB",]
#algorithm_list   = ["LK",]
#algorithm_list   = ["FB",]
#algorithm_list   = ["FB", "LK"]
algorithm_list   = ["FB", "LK", "TVL"]

file_list = list()

file_final = "/local/git/MotionFlowPriorityGraphSensors/project/main_python/values.yml"

for noise in noise_list:
    if (  noise is not "ground_truth"):
        for algorithm in algorithm_list:
            file_list.append("/local/git/MotionFlowPriorityGraphSensors/datasets/" + dataset + "_dataset/results/stereo_flow/two/results_" + algorithm + "_" + noise + "_30_1/values_" + algorithm + ".yml")

dict_parameter_dual = {
    "dual_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels" : "percentage of sroi pixels error to eroi pixels error" ,
    "dual_sroi_all_pixels_eroi_all_pixels" : "percentage of sroi pixels to eroi pixels",
    "sroi_all_pixels" : "number of pixels in the special roi",
    "eroi_all_pixels" : "number of pixels in the entire roi",
    "eroi_l2_cumulative_error_all_pixels" : "add up all the l2 errors",
    "eroi_ma_cumulative_error_all_pixels" : "add up all the mahalanobis distance errors",
    "multiple_nucleus_reliability" : "reliability due to the nucleus",
    "multiple_occlusion_reliability" : "reliability at the occlusion boundaries",

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

display_list = [

    # the following graph shows the importance of mahalabnobis distance
    # the conclusion is that if eigen vector is not equal to the l2 radius && l2 good pixels tends to ma good pixels,
    # we have a beautiful nucleus. show 37, 46 for sroi and 39 for eroi.
    [
        "eroi_l2_good_pixels_FB_blue_sky_1_0",
        "eroi_ma_good_pixels_FB_blue_sky_1_0",
    ],

    [
        "eroi_l2_good_pixels_FB_heavy_snow_1_0",
        "eroi_ma_good_pixels_FB_heavy_snow_1_0",
    ],


    # the following graph shows the importance of mahalabnobis distance
    [
        "sroi_l2_good_pixels_FB_blue_sky_1_0",
        "sroi_ma_good_pixels_FB_blue_sky_1_0",
    ],

    # this graph shows if the object is fully covered or just a part of
    [
        "distribution_matrix_FB_blue_sky_1_0",
        "distribution_matrix_LK_blue_sky_1_0",
        "distribution_matrix_TVL_blue_sky_1_0",
    ],

    # this graph shows if the covariance matches. If yes, then it helps in the prediction phase

    [
        "intersection_angle_deg_FB_blue_sky_1_0",
        "intersection_angle_deg_LK_blue_sky_1_0",
        "intersection_angle_deg_TVL_blue_sky_1_0"],
]



display_list_dual  = [

###############################
# what is the percentage of error due to the special region of interest.
    [
        "dual_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels_FB_blue_sky_1_0",
        "dual_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels_LK_blue_sky_1_0",
        "dual_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels_TVL_blue_sky_1_0",
    ],

    # what is the percentage of good pixels in the entire area
    [
        "dual_sroi_all_pixels_eroi_all_pixels_FB_ground_truth_1_0",
        "dual_sroi_all_pixels_eroi_all_pixels_FB_blue_sky_1_0",
        "dual_sroi_all_pixels_eroi_all_pixels_LK_blue_sky_1_0",
        "dual_sroi_all_pixels_eroi_all_pixels_TVL_blue_sky_1_0",
    ],

###################################

# what is the percentage of good pixels in the special region of interest
    [
        "dual_sroi_l2_good_pixels_sroi_all_pixels_FB_blue_sky_1_0",
        "dual_sroi_l2_good_pixels_sroi_all_pixels_LK_blue_sky_1_0",
        "dual_sroi_l2_good_pixels_sroi_all_pixels_TVL_blue_sky_1_0",
    ],

# what is the percentage of good pixels in the entire area
    [
        "dual_eroi_all_pixels_eroi_all_pixels_FB_blue_sky_1_0",
        "dual_eroi_l2_good_pixels_eroi_all_pixels_FB_blue_sky_1_0",
        "dual_eroi_l2_good_pixels_eroi_all_pixels_LK_blue_sky_1_0",
        "dual_eroi_l2_good_pixels_eroi_all_pixels_TVL_blue_sky_1_0",
    ],


# how much error does each pixel contribute to in the entire area
    [
        "dual_eroi_l2_cumulative_error_all_pixels_eroi_all_pixels_FB_blue_sky_1_0",
        "dual_eroi_l2_cumulative_error_all_pixels_eroi_all_pixels_LK_blue_sky_1_0",
        "dual_eroi_l2_cumulative_error_all_pixels_eroi_all_pixels_TVL_blue_sky_1_0",
    ],

# how much error does each pixel contribute to in the special area
    [
        "dual_sroi_l2_cumulative_error_all_pixels_sroi_all_pixels_FB_blue_sky_1_0",
        "dual_sroi_l2_cumulative_error_all_pixels_sroi_all_pixels_LK_blue_sky_1_0",
        "dual_sroi_l2_cumulative_error_all_pixels_sroi_all_pixels_TVL_blue_sky_1_0",
    ],

]

display_list_multiple  = [

    ###############################
    # what is the percentage of error due to the special region of interest.
    [
        "multiple_nucleus_reliability_FB_blue_sky_1_0",
        "multiple_nucleus_reliability_FB_heavy_snow_1_0",
        #"multiple_nucleus_reliability_LK_blue_sky_1_0",
        #"multiple_nucleus_reliability_TVL_blue_sky_1_0",
    ],
]


display_list_bargraph = []

display_list_bargraph_dual = [

    [
        "dual_sroi_l2_cumulative_error_all_pixels_eroi_l2_cumulative_error_all_pixels"
    ],
    [
        "dual_sroi_all_pixels_eroi_all_pixels"
    ],
]

display_list_bargraph_multiple = [

    [
        "multiple_nucleus_reliability"
    ],
]


parameter_list = [

    "eroi_all_pixels",
    "eroi_l1_good_pixels",
    "eroi_l2_good_pixels",
    "eroi_ma_good_pixels",
    "eroi_l1_cumulative_error_all_pixels",
    "eroi_l2_cumulative_error_all_pixels",
    "eroi_ma_cumulative_error_all_pixels",
    "eroi_l1_cumulative_error_good_pixels",
    "eroi_l2_cumulative_error_good_pixels",
    "eroi_ma_cumulative_error_good_pixels",
    "eroi_interpolated_all_pixels",
    "eroi_interpolated_l1_good_pixels",
    "eroi_interpolated_l2_good_pixels",
    "eroi_interpolated_ma_good_pixels",
    "eroi_interpolated_l1_cumulative_error_all_pixels",
    "eroi_interpolated_l2_cumulative_error_all_pixels",
    "eroi_interpolated_ma_cumulative_error_all_pixels",
    "eroi_interpolated_l1_cumulative_error_good_pixels",
    "eroi_interpolated_l2_cumulative_error_good_pixels",
    "eroi_interpolated_ma_cumulative_error_good_pixels",
    "sroi_all_pixels",
    "sroi_l1_good_pixels",
    "sroi_l2_good_pixels",
    "sroi_ma_good_pixels",
    "sroi_l1_cumulative_error_all_pixels",
    "sroi_l2_cumulative_error_all_pixels",
    "sroi_ma_cumulative_error_all_pixels",
    "sroi_l1_cumulative_error_good_pixels",
    "sroi_l2_cumulative_error_good_pixels",
    "sroi_ma_cumulative_error_good_pixels",
    "sroi_interpolated_all_pixels",
    "sroi_interpolated_l1_good_pixels",
    "sroi_interpolated_l2_good_pixels",
    "sroi_interpolated_ma_good_pixels",
    "sroi_interpolated_l1_cumulative_error_all_pixels",
    "sroi_interpolated_l2_cumulative_error_all_pixels",
    "sroi_interpolated_ma_cumulative_error_all_pixels",
    "sroi_interpolated_l1_cumulative_error_good_pixels",
    "sroi_interpolated_l2_cumulative_error_good_pixels",
    "sroi_interpolated_ma_cumulative_error_good_pixels",
    "distribution_matrix",
    "intersection_angle_deg",
    #"correlation",

]

parameter_list_dual = [

    [ "sroi_all_pixels", "eroi_all_pixels" ],
    [ "sroi_l2_good_pixels", "sroi_all_pixels" ],
    [ "sroi_l2_good_pixels", "eroi_l2_good_pixels" ],
    [ "sroi_l2_cumulative_error_all_pixels", "eroi_l2_cumulative_error_all_pixels" ],
    [ "sroi_ma_cumulative_error_all_pixels", "eroi_ma_cumulative_error_all_pixels" ],
    [ "sroi_l2_cumulative_error_good_pixels", "eroi_l2_cumulative_error_good_pixels" ],
    [ "sroi_ma_cumulative_error_good_pixels", "eroi_ma_cumulative_error_good_pixels" ],
    [ "eroi_l2_good_pixels", "eroi_all_pixels" ],
    [ "sroi_l2_good_pixels", "eroi_all_pixels" ],
    [ "eroi_ma_good_pixels", "eroi_all_pixels" ],
    [ "sroi_ma_good_pixels", "eroi_all_pixels" ],
    [ "eroi_l2_cumulative_error_all_pixels", "eroi_all_pixels" ],
    [ "sroi_l2_cumulative_error_all_pixels", "sroi_all_pixels" ],

]


parameter_list_multiple = [

    [ "nucleus_reliability" ],

]


datafilter_list  = [ "0", ]

file_ground_truth = "/local/git/MotionFlowPriorityGraphSensors/datasets/" + dataset + "_dataset/data/stereo_flow/two/ground_truth/values_ground_truth.yml"

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

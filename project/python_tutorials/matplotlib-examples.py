#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import yaml
import io
import math

import pandas as pd
import cv2
import numpy
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import scipy
from scipy.interpolate import griddata
import matplotlib as mp
from math import pi

import random


def histogram():
    # Build a vector of 10000 normal deviates with variance 0.5^2 and mean 2
    mu, sigma = 2, 0.5
    v = numpy.random.normal(mu,sigma,10000)
    # Plot a normalized histogram with 50 bins
    plt.hist(v, bins=50, normed=1)       # matplotlib version (plot)
    plt.show()
    # Compute the histogram with numpy and then plot it
    (n, bins) = numpy.histogram(v, bins=50, normed=True)  # NumPy version (no plot)
    plt.plot(.5*(bins[1:]+bins[:-1]), n)
    plt.show()
    
def cam():
    cap = cv2.VideoCapture(0)
    
    while(1):
    
        # Take each frame
        _, frame = cap.read()
    
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
        # define range of blue color in HSV
        lower_blue = numpy.array([110,50,50])
        upper_blue = numpy.array([130,255,255])
        # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= mask)
    
        cv2.imshow('frame',frame)
        cv2.imshow('mask',mask)
        cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
        
    
    cv2.destroyAllWindows()

def histogram_image():
    img = cv2.imread('/local/git/everynote/dias/tunnel-reflection.jpeg',0)
    plt.hist(img.ravel(),256,[0,256]);    
    #hist = cv2.calcHist([img],[0],None,[256],[0,256])
#     color = ('b','g','r')
#     for i,col in enumerate(color):
#         histr = cv2.calcHist([img],[i],None,[256],[0,256])
#         plt.plot(histr,color = col)
#         plt.xlim([0,256])
    plt.show()

def threedplot():
    
    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    # note this: you can skip rows!
    my_data = np.genfromtxt('file1.csv', delimiter=',', skiprows=1)
    X = my_data[:,0]
    Y = my_data[:,1]
    Z = my_data[:,2]
    
    xi = np.linspace(X.min(),X.max(),100)
    yi = np.linspace(Y.min(),Y.max(),100)
    # VERY IMPORTANT, to tell matplotlib how is your data organized
    zi = griddata((X, Y), Z, (xi[None,:], yi[:,None]), method='cubic')
    
    CS = plt.contour(xi,yi,zi,15,linewidths=0.5,color='k')
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    
    xig, yig = np.meshgrid(xi, yi)
    
    surf = ax.plot_surface(xig, yig, zi,
            linewidth=0)
    
    plt.show()
    
def sphere():
    npoints=100
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    u = np.linspace(0, 2 * np.pi, npoints)
    v = np.linspace(0, np.pi, npoints)
    x = 10 * np.outer(np.cos(u), np.sin(v))
    y = 10 * np.outer(np.sin(u), np.sin(v))
    z = 10 * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x,y,z,  rstride=4, cstride=4, color='b')
    plt.show()
    
def criticalpoint():
    # evenly sampled time at 200ms intervals
    t = np.arange(0., 5., 0.2)
    # red dashes, blue squares and green triangles
    plt.plot(t, t, 'r', t, t**2, 'b', t, t**3, 'g')
    plt.xlabel('time')
    plt.ylabel('progress')
    plt.show()


def lemniscate():
    style = {
        "axes.linewidth": 3,
        "grid.linewidth": 3,
        "grid.linestyle": "solid",
        "grid.color": "#dadad0",
        "lines.linewidth": 3,
        "lines.markersize": 10,
        "xtick.labelsize": 24,
        "ytick.labelsize": 24,
    }

    for key in style:
        mp.rcParams[key]=style[key]

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_aspect('equal')
    color1 = "#505050"
    ax.spines['bottom'].set_color(color1)
    ax.spines['top'].set_color(color1)
    ax.spines['right'].set_color(color1)
    ax.spines['left'].set_color(color1)

    ax.axhline(y=0, color=color1,zorder=4)
    ax.axvline(x=0, color=color1,zorder=4)

    ax.xaxis.grid()
    ax.yaxis.grid()
    ax.set_axisbelow(True)
    ax.xaxis.set_tick_params(width=3,length=6,color=color1)
    ax.yaxis.set_tick_params(width=3,length=6,color=color1)

    t = np.arange(0,2*pi,0.02)
    plt.axis([-1.5,1.5,-1,1])
    plt.plot(np.cos(t),np.sin(2*t)/2,color='#0040b0',alpha=0.6,zorder=6,linewidth=6)

    ax.text(1.3,-0.94,"x", fontsize=32, style="italic", color="#202020")
    ax.text(-1.44,0.8,"y", fontsize=32, style="italic", color="#202020")

    plt.show()
    #plt.savefig("Gerono.svg",bbox_inches='tight')

def simple():
    data1 = []
    data2 = []
    data3 = []
    data4 = []
    for val in range(100):
        data1.append(random.uniform(100-val,100-val+1))
        data2.append(random.uniform(90-val,90-val+1))
        data3.append(random.uniform(80-val,80-val+1))
        data4.append(random.uniform(70-val,70-val+1))
    x1 = len(data1)
    t = x1
    t1 = np.linspace(0,x1,x1)

    #plt.subplot(211)
    #ax.get_legend_handles_labels()
    line1,= plt.plot(t1,data1,'-r',label='no fusion')
    line2,= plt.plot(t1,data2,'-g',label='with sensor fusion')
    line3,= plt.plot(t1,data3, 'b',label='with fitness landscape')
    line4,= plt.plot(t1,data4, 'y',label='with actuator feedback')

    #red_patch = mpatches(color='red', label='The red data')
    plt.legend([line1,line2,line3,line4])

    plt.xlim(0,t)
    plt.xlabel("time")
    plt.ylim(0,100)
    plt.ylabel("Speed")
    plt.grid(True)
    plt.show()

def yaml_try():

    # Define data
    data = {'a list': [1, 42, 3.141, 1337, 'help', u'€'],
            'a string': 'bla',
            'another dict': {'foo': 'bar',
                             'key': 'value',
                             'the answer': 42}}

    # Write YAML file
    with io.open('data.yaml', 'w', encoding='utf8') as outfile:
        yaml.dump(data, outfile, default_flow_style=False, allow_unicode=True)

    # Read YAML file
    with open("data.yaml", 'r') as stream:
        data_loaded = yaml.load(stream)

    print(data == data_loaded)
    print data


def motionflow_vectorgraphs_no_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig2 = plt.figure()
    plt.suptitle("Deviation of collision points between ground truth and scenes without noise")

    collisionplot0 = fig2.add_subplot(221)
    collisionplot1 = fig2.add_subplot(222)
    collisionplot2 = fig2.add_subplot(223)
    collisionplot3 = fig2.add_subplot(224)

    collisionplot0.set_title('collision_pointsframe_skip1_postprocessing_0')
    collisionplot1.set_title('collision_pointsframe_skip1_postprocessing_1')
    collisionplot2.set_title('collision_pointsframe_skip1_postprocessing_2')
    collisionplot3.set_title('collision_pointsframe_skip1_postprocessing_3')

    collisionplot0.set_xlabel('frame_count')
    collisionplot0.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot1.set_xlabel('frame_count')
    collisionplot1.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot2.set_xlabel('frame_count')
    collisionplot2.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot3.set_xlabel('frame_count')
    collisionplot3.set_ylabel('deviation noise_pixels-no_noise_pixels')

    #collisionplot0.set_ylim([0, 100])
    #collisionplot1.set_ylim([0, 100])
    #collisionplot2.set_ylim([0, 100])
    #collisionplot3.set_ylim([0, 100])
    legend = collisionplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [
        "collision_pointsframe_skip1_postprocessing_0_generated",
        "collision_pointsframe_skip1_postprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_3results_FB_none_",
    ]

    color_of_collision_metrics = ["red", "black"]

    #assert(len(list_of_collision_metrics)/4 == len(color_of_collision_metrics))

    offset=0
    offset_index=0
    num=0

    for no_of_metrics in range(1): # none

        for x in range(1):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0_gt, y0_gt = data.T

            x0 = np.arange(0.0, 4.0, 1)

    for no_of_metrics in range(1): # none

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+1+x]]
            print offset_index*no_of_metrics+1+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                y0 = numpy.sqrt((x0_gt - x0) ** 2 + (y0_gt - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                y1 = numpy.sqrt((x0_gt - x1) ** 2 + (y0_gt - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                y2 = numpy.sqrt((x0_gt - x2) ** 2 + (y0_gt - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                y3 = numpy.sqrt((x0_gt - x3) ** 2 + (y0_gt - y3) ** 2)

            x0 = np.arange(0.0, len(collision_points), 1)

        collisionplot0.plot(x0, y0, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot1.plot(x0, y1, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot2.plot(x0, y2, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot3.plot(x0, y3, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/tmp/eaes/vector_robustness_optical_flow', dpi=200)



def motionflow_vectorgraphs_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig2 = plt.figure()
    plt.suptitle("Deviation of collision points between scenes without noise and scenes with noise")

    collisionplot0 = fig2.add_subplot(221)
    collisionplot1 = fig2.add_subplot(222)
    collisionplot2 = fig2.add_subplot(223)
    collisionplot3 = fig2.add_subplot(224)

    collisionplot0.set_title('collision_pointsframe_skip1_postprocessing_0')
    collisionplot1.set_title('collision_pointsframe_skip1_postprocessing_1')
    collisionplot2.set_title('collision_pointsframe_skip1_postprocessing_2')
    collisionplot3.set_title('collision_pointsframe_skip1_postprocessing_3')

    collisionplot0.set_xlabel('frame_count')
    collisionplot0.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot1.set_xlabel('frame_count')
    collisionplot1.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot2.set_xlabel('frame_count')
    collisionplot2.set_ylabel('deviation noise_pixels-no_noise_pixels')
    collisionplot3.set_xlabel('frame_count')
    collisionplot3.set_ylabel('deviation noise_pixels-no_noise_pixels')

    collisionplot0.set_ylim([0, 100])
    collisionplot1.set_ylim([0, 100])
    collisionplot2.set_ylim([0, 100])
    collisionplot3.set_ylim([0, 100])
    legend = collisionplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_collision_metrics = [
        "collision_pointsframe_skip1_postprocessing_0results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_1results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_2results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_3results_FB_none_",
        "collision_pointsframe_skip1_postprocessing_1results_FB_night_",
        "collision_pointsframe_skip1_postprocessing_0results_FB_night_",
        "collision_pointsframe_skip1_postprocessing_2results_FB_night_",
        "collision_pointsframe_skip1_postprocessing_3results_FB_night_",
    ]

    color_of_collision_metrics = ["red", "black"]

    assert(len(list_of_collision_metrics)/4 == len(color_of_collision_metrics))

    offset=0
    offset_index=0
    num=0

    for no_of_metrics in range(1): # none

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0_base, y0_base = data.T
            if ( x == 1):
                x1_base, y1_base = data.T
            if ( x == 2):
                x2_base, y2_base = data.T
            if ( x == 3):
                x3_base, y3_base = data.T

            x0 = np.arange(0.0, 5.0, 1)

    for no_of_metrics in range(2): # none, night, ...

        for x in range(4):

            collision_points = yaml_load[list_of_collision_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            collision = list()
            for count in range(len(collision_points)-offset):
                xy = list()
                xy.append(collision_points[offset + count]["x"])
                xy.append(collision_points[offset + count]["y"])
                collision.append(xy)
            data = np.array(collision)
            if ( x == 0):
                x0, y0 = data.T
                y0 = numpy.sqrt((x0_base - x0) ** 2 + (y0_base - y0) ** 2)
            if ( x == 1):
                x1, y1 = data.T
                y1 = numpy.sqrt((x1_base - x1) ** 2 + (y1_base - y1) ** 2)
            if ( x == 2):
                x2, y2 = data.T
                y2 = numpy.sqrt((x2_base - x2) ** 2 + (y2_base - y2) ** 2)
            if ( x == 3):
                x3, y3 = data.T
                y3 = numpy.sqrt((x3_base - x3) ** 2 + (y3_base - y3) ** 2)

            x0 = np.arange(0.0, len(collision_points), 1)

        collisionplot0.plot(x0, y0, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot1.plot(x0, y1, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot2.plot(x0, y2, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        collisionplot3.plot(x0, y3, 'ko-', lw=2, color=color_of_collision_metrics[0+no_of_metrics], label=list_of_collision_metrics[0+no_of_metrics])
        #collisionplot1.legend()
        collisionplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/tmp/eaes/vector_robustness_post_processing_algorithm', dpi=200)



def motionflow_pixelgraphs_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig2 = plt.figure()
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig2.add_subplot(221)
    shapeplot1 = fig2.add_subplot(222)
    shapeplot2 = fig2.add_subplot(223)
    shapeplot3 = fig2.add_subplot(224)

    shapeplot0.set_title('shape_pointsframe_skip1_postprocessing_0')
    shapeplot1.set_title('shape_pointsframe_skip1_postprocessing_1')
    shapeplot2.set_title('shape_pointsframe_skip1_postprocessing_2')
    shapeplot3.set_title('shape_pointsframe_skip1_postprocessing_3')

    shapeplot0.set_xlabel('frame_count')
    shapeplot0.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot1.set_xlabel('frame_count')
    shapeplot1.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot2.set_xlabel('frame_count')
    shapeplot2.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot3.set_xlabel('frame_count')
    shapeplot3.set_ylabel('noise_pixels/no_noise_pixels')

    shapeplot0.set_ylim([0, 1])
    shapeplot1.set_ylim([0, 1])
    shapeplot2.set_ylim([0, 1])
    shapeplot3.set_ylim([0, 1])
    legend = shapeplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))

    list_of_shape_metrics = [
        "shape_pointsframe_skip1_postprocessing_0results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_1results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_2results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_3results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_1results_FB_night_",
        "shape_pointsframe_skip1_postprocessing_0results_FB_night_",
        "shape_pointsframe_skip1_postprocessing_2results_FB_night_",
        "shape_pointsframe_skip1_postprocessing_3results_FB_night_",
    ]

    color_of_shape_metrics = ["red", "black"]

    assert(len(list_of_shape_metrics)/4 == len(color_of_shape_metrics))

    offset=1
    offset_index=0
    num=0

    for no_of_metrics in range(2): #none, night

        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0, y0 = data.T
                y0 = x0/y0
            if ( x == 1):
                x1, y1 = data.T
                y1 = x1/y1
            if ( x == 2):
                x2, y2 = data.T
                y2 = x2/y2
            if ( x == 3):
                x3, y3 = data.T
                y3 = x3/y3

            x0 = np.arange(0.0, len(shape_points)-1, 1)

        shapeplot0.plot(x0, y0, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot1.plot(x0, y1, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot2.plot(x0, y2, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot3.plot(x0, y3, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/tmp/eaes/pixel_robustness_post_processing_algorithm', dpi=200)


def motionflow_pixelgraphs_no_noise():

    dataset = "cpp"
    scenario = "two"
    file = "/local/git/MotionFlowPriorityGraphSensors/datasets/"+dataset+"_dataset/data/stereo_flow/" + scenario + "/values.yml"
    #file = "/home/veikas/seafile_base/seafile_sync_work/tuebingen_phd/presentations/eaes/pics_20_02/values_all.yml"

    yaml_file = open(file, "r")
    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):
        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')
        yaml_file.close()
        yaml_file = open(file, "w")
        yaml_file.write(read_yaml_file)

    yaml_file.close()

    fig2 = plt.figure()
    plt.suptitle("Ratio of good pixels to total found no noise stencil pixels")

    shapeplot0 = fig2.add_subplot(221)
    shapeplot1 = fig2.add_subplot(222)
    shapeplot2 = fig2.add_subplot(223)
    shapeplot3 = fig2.add_subplot(224)

    shapeplot0.set_title('shape_pointsframe_skip1_postprocessing_0')
    shapeplot1.set_title('shape_pointsframe_skip1_postprocessing_1')
    shapeplot2.set_title('shape_pointsframe_skip1_postprocessing_2')
    shapeplot3.set_title('shape_pointsframe_skip1_postprocessing_3')

    shapeplot0.set_xlabel('frame_count')
    shapeplot0.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot1.set_xlabel('frame_count')
    shapeplot1.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot2.set_xlabel('frame_count')
    shapeplot2.set_ylabel('noise_pixels/no_noise_pixels')
    shapeplot3.set_xlabel('frame_count')
    shapeplot3.set_ylabel('noise_pixels/no_noise_pixels')

    shapeplot0.set_ylim([0, 1])
    shapeplot1.set_ylim([0, 1])
    shapeplot2.set_ylim([0, 1])
    shapeplot3.set_ylim([0, 1])
    legend = shapeplot1.legend(loc='center right', shadow=True, fontsize='x-small')

    yaml_load = yaml.load(open(file))


    list_of_shape_metrics = [
        "shape_pointsframe_skip1_postprocessing_0_generated",
        "shape_pointsframe_skip1_postprocessing_0results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_1results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_2results_FB_none_",
        "shape_pointsframe_skip1_postprocessing_3results_FB_none_",
    ]

    color_of_shape_metrics = ["red", "black"]

    #assert(len(list_of_shape_metrics)/1 == len(color_of_shape_metrics))

    offset=1
    offset_index=0
    num=0

    for no_of_metrics in range(1): #none, night

        for x in range(1):

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+x]]
            print offset_index*no_of_metrics+x
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0_gt, y0_gt = data.T
                y0_gt = x0_gt/y0_gt

            x0 = np.arange(0.0, 4.0, 1)

    for no_of_metrics in range(1): # none

        for x in range(4):

            shape_points = yaml_load[list_of_shape_metrics[offset_index*no_of_metrics+1+x]]
            print offset_index*no_of_metrics+1+x
            shape = list()
            for count in range(len(shape_points)-offset):
                xy = list()
                xy.append(shape_points[offset + count]["good_pixels"])
                xy.append(shape_points[offset + count]["total_pixels"])
                shape.append(xy)
            data = np.array(shape)
            if ( x == 0):
                x0, y0 = data.T
                y0 = x0/y0
            if ( x == 1):
                x1, y1 = data.T
                y1 = x1/y1
            if ( x == 2):
                x2, y2 = data.T
                y2 = x2/y2
            if ( x == 3):
                x3, y3 = data.T
                y3 = x3/y3

            x0 = np.arange(0.0, len(shape_points)-1, 1)

        shapeplot0.plot(x0, y0, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot0.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot1.plot(x0, y1, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot1.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot2.plot(x0, y2, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot2.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        shapeplot3.plot(x0, y3, 'ko-', lw=2, color=color_of_shape_metrics[0+no_of_metrics], label=list_of_shape_metrics[0+no_of_metrics])
        #shapeplot1.legend()
        shapeplot3.xaxis.set_major_locator(plt.MaxNLocator(integer = True))

        offset_index=offset_index+4

    fig2.set_size_inches(18.5, 10.5)
    fig2.savefig('/local/tmp/eaes/pixel_robustness_optical_flow.png', dpi=200)


def check():
    fig2 = plt.figure()
    shapeplot1 = fig2.add_subplot(111)

    coordinates = [[1,2], [10,11], [12,12], [14,15]]

    data = np.array(coordinates)
    x1, y1 = data.T
    shapeplot1.scatter(x1,y1, color='red')
    shapeplot1.plot([coordinates[0][0], coordinates[1][0]], [coordinates[0][1], coordinates[1][1]])
    shapeplot1.plot([coordinates[2][0], coordinates[3][0]], [coordinates[2][1], coordinates[3][1]])

    plt.show()
    ###

    #a list:
    #- 1
    #- 42
    #- 3.141
    #- 1337
    #- help
    #- €
    #a string: bla
    #another dict:
    #foo: bar
    #key: value
    #the answer: 42



def read_vkitti_png_flow(flow_fn):
    "Convert from .png to (h, w, 2) (flow_x, flow_y) float32 array"
    # read png to bgr in 16 bit unsigned short
    bgr = cv2.imread(flow_fn, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    h, w, _c = bgr.shape
    assert bgr.dtype == np.uint16 and _c == 3
    # b == invalid flow flag == 0 for sky or other invalid flow
    invalid = bgr[..., 0] == 0
    # g,r == flow_y,x normalized by height,width and scaled to [0;2**16 - 1]
    out_flow = 2.0 / (2**16 - 1.0) * bgr[..., 2:0:-1].astype('f4') - 1
    out_flow[..., 0] *= w - 1
    out_flow[..., 1] *= h - 1
    out_flow[invalid] = 0  # or another value (e.g., np.nan)
    return out_flow


def readradardata():
    filename = "/local/git/MotionFlowPriorityGraphSensors/datasets/radar_dataset/"
    radardata = pd.read_csv("<filename>", sep=" ", index_col=False)

def vkitti():
    filename = "/local/git/MotionFlowPriorityGraphSensors/project/main/non_code/0001_fog.txt"
    mot_data = pd.read_csv(filename, sep=" ", index_col=False)
    print mot_data


if __name__ == '__main__':
    #histogram()
    #cam()
    #histogram_image()
    #threedplot()
    #sphere()
    #criticalpoint()
    #lemniscate()
    #simple()
    motionflow_pixelgraphs_no_noise()
    motionflow_pixelgraphs_noise()
    motionflow_vectorgraphs_no_noise()
    motionflow_vectorgraphs_noise()

    #motionflow_vectorgraphs()
    #check()
    #vkitti()

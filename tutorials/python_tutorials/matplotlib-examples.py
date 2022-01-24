#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

from mpl_toolkits.mplot3d import axes3d

import yaml
import io
import math

import pandas as pd
import cv2
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import scipy
from scipy.interpolate import griddata
import matplotlib as mpl
import matplotlib.pyplot as plt
from math import pi

import random
from mpl_toolkits.mplot3d import Axes3D
import random

environment_list = ["none", "night"]

def histogram():
    # Build a vector of 10000 normal deviates with variance 0.5^2 and mean 2
    mu, sigma = 2, 0.5
    v = np.random.normal(mu,sigma,10000)
    # Plot a normalized histogram with 50 bins
    plt.hist(v, bins=50, normed=1)       # matplotlib version (plot)
    plt.show()
    # Compute the histogram with numpy and then plot it
    (n, bins) = np.histogram(v, bins=50, normed=True)  # NumPy version (no plot)
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
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
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

def boundingbox():
    fig = plt.figure()
    ax = Axes3D(fig)

    list_box_points = list(
        [-59.574017, 2470.1152, 9.5,
         -61.094013, 2470.1152, 9.5,
         -59.574017, 2467.6011, 9.5,
         -61.094013, 2467.6011, 9.5,
         -59.574017, 2470.1152, 10.98,
         -61.094013, 2470.1152, 10.98,
         -59.574017, 2467.6011, 10.98,
         -61.094013, 2467.6011, 10.98])
    sequence_containing_x_vals = list()
    sequence_containing_y_vals = list()
    sequence_containing_z_vals = list()

    for n in range(len(list_box_points)/3):

        print n
        sequence_containing_x_vals.append(list_box_points[3*n]) #list(range(0, 100))
        sequence_containing_y_vals.append(list_box_points[3*n+1])
        sequence_containing_z_vals.append(list_box_points[3*n+2])
        n = n+3

    print sequence_containing_x_vals



    #random.shuffle(sequence_containing_x_vals)
    #random.shuffle(sequence_containing_y_vals)
    #random.shuffle(sequence_containing_z_vals)

    ax.plot(sequence_containing_x_vals, sequence_containing_y_vals, sequence_containing_z_vals)
    plt.show()


def threedplot():


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # load some test data for demonstration and plot a wireframe
    X, Y, Z = axes3d.get_test_data(0.1)
    ax.plot_wireframe(X, Y, Z, rstride=5, cstride=5)

    # rotate the axes and update
    for angle in range(0, 360):
        ax.view_init(30, angle)
        plt.draw()
        plt.pause(.001)


def sphere():
    npoints=100
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    u = np.linspace(0, 2 * np.pi, npoints)
    v = np.linspace(0, np.pi, npoints)
    x = 10 * np.outer(np.cos(u), np.sin(v))
    y = 10 * np.outer(np.sin(u), np.sin(v))
    z = 10 * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surace(x,y,z,  rstride=4, cstride=4, color='b')
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
        mpl.rcParams[key]=style[key]

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


def check():
    fig2 = plt.figure()
    shapeplot1 = fig2.add_subplot(111)

    coordinates = [[1,2], [10,11], [12,12], [14,15]]

    data = np.array(coordinates)
    x1, y1 = data.T
    shapeplot1.scatter(x1,y1, color='red')
    #shapeplot1.plot([coordinates[0][0], coordinates[1][0]], [coordinates[2][0], coordinates[3][0]])
    #shapeplot1.plot([coordinates[2][0], coordinates[3][0]], [coordinates[2][1], coordinates[3][1]])

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


def bar_chart_2d():

    fig = plt.figure()
    ax = fig.add_subplot(111)

    n_groups = 4
    bar_width = 0.1
    opacity = 0.4
    index = np.arange(0, 2*n_groups, 2)

    data = [1,2,3,4]
    data2 = [5,6,7,8]

    data2 = np.array(data2)
    data2 = data2*4

    chart = plt.bar(index, data, bar_width, color='blue', edgecolor='black', label='plot1')
    chart = plt.bar(index+bar_width, data2, bar_width, color='red', edgecolor='black')
    chart = plt.bar(index+2*bar_width, data, bar_width, color='green', edgecolor='black')


    ax.set_xlabel('y_axis')
    ax.set_ylabel('x_axis')
    ax.set_title('Title')

    plt.xticks(index+bar_width*0.5, ('A', 'B', 'C', 'D'))
    plt.legend()
    plt.tight_layout()

    plt.show()


if __name__ == '__main__':
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)

    objp = np.zeros((6*7,3), np.float32)
    objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
    print objp

    #histogram()
    #cam()
    #histogram_image()
    #threedplot()
    #boundingbox()
    #sphere()
    #criticalpoint()
    #lemniscate()
    bar_chart_2d()
    #simple()

    #motionflow_vectorgraphs()
    #check()
    #vkitti()

#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import yaml
import io

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


def yaml_():

    yaml_file = open("/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/two/values.yml", "r")

    check = yaml_file.readline()
    print check
    if ("YAML:1.0" in check ):

        read_yaml_file = yaml_file.read().replace('YAML:1.0', 'YAML 1.0')
        read_yaml_file = read_yaml_file.replace(':', ': ')

        yaml_file.close()

        yaml_file = open("/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/two/values.yml", "w")

        yaml_file.write(read_yaml_file)

    yaml_file.close()


    yaml_load = yaml.load(open('/local/git/MotionFlowPriorityGraphSensors/datasets/cpp_dataset/data/stereo_flow/two/values.yml'))
    #print yaml_load
    collision_points = yaml_load["collision_pointsframe_skip1_generated"]
    coordinates = list()
    for count in range(len(collision_points)):
        xy = list()
        xy.append(collision_points[count]["x"])
        xy.append(collision_points[count]["y"])
        coordinates.append(xy)

    print coordinates
    data = np.array(coordinates)
    x, y = data.T
    plt.scatter(x,y)
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

if __name__ == '__main__':
    #histogram()
    #cam()
    #histogram_image()
    #threedplot()
    #sphere()
    #criticalpoint()
    #lemniscate()
    #simple()
    yaml_()
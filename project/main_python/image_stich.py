#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import os
import numpy as np
from matplotlib import pyplot as plt
import cv2
from os.path import basename

def stich_bargraphs():

    list_dir = os.listdir("/local/tmp/eaes")
    list_bargraph_dir = list()
    list_bargraph_names = list()
    list_graph_dir = list()
    for index, val in enumerate(list_dir):
        if ( "summary" in val ):
            list_bargraph_dir.append(val)
            list_bargraph_names.append(basename(val))
        else:
            list_graph_dir.append(val)

    print list_bargraph_dir
    print list_graph_dir

    image_bar = list()
    for index, image in enumerate(list_bargraph_dir):
        image_bar.append(cv2.imread("/local/tmp/eaes/" + list_bargraph_dir[index]))

    image_shape = image_bar[0].shape
    image_type = image_bar[0].dtype

    # the problem with tis method is that the image size needs to be properly taken.
    # for the method with numpy, the image size is irrelevant

    final_image = np.zeros((image_shape[0]*2, image_shape[1]*2, image_shape[2]), image_type)
    #final_image[0:image_shape[0], 0:image_shape[1]] = image_bar[0]
    #final_image[0:image_shape[0], image_shape[1]:image_shape[1]*2] = image_bar[1]

    #cv2.namedWindow("stiched", WINDOW_NORMAL)
    #cv2.imshow("stiched", final_image)
    #cv2.waitKey(0)


    plt.subplot(221),plt.imshow(image_bar[0],'gray'),plt.title(list_bargraph_names[0])
    plt.subplot(222),plt.imshow(image_bar[1],'gray'),plt.title(list_bargraph_names[1])
    plt.subplot(223),plt.imshow(image_bar[2],'gray'),plt.title(list_bargraph_names[2])
    plt.subplot(224),plt.imshow(image_bar[3],'gray'),plt.title(list_bargraph_names[3])

    plt.show()


if __name__=='__main__':
    stich_bargraphs()

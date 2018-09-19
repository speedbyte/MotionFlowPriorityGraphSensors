

import cv2
import os
import numpy


list_dir = os.listdir("/local/tmp/eaes")
list_bargraph_dir = list()
list_graph_dir = list()
for index, val in enumerate(list_dir):
    if ( "summary" in val ):
        list_bargraph_dir.append(val)
    else:
        list_graph_dir.append(val)

print list_bargraph_dir
print list_graph_dir

image = cv2.imread("/local/tmp/eaes/" + list_bargraph_dir[0])
print type(image)
final_image = numpy.ndarray(2)
print final_image

cv2.imshow("stiched", image)
#cv2.waitKey(0)
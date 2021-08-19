#!/usr/bin/env python
# _*_ encoding=utf-8 _*_

import cv2
import numpy as np
import sys

def change_color():
    img = cv2.imread("/local/mega_personal/personal_folder/jobs/aktuell/e-signature.jpg", 0)
    height, width = img.shape
    print height, width

    for i in range(height):
        for j in range(width):
            img[ht,wh] = [0,0,0]

    cv2.imshow("orig", img)
    cv2.waitKey(0)


def numpy_check():
    python_list = [1,2,3,4]
    check_numpy = np.array(python_list)

if __name__ == '__main__':
    change_color()
    print sys.version_info
    numpy_check()
#!/bin/python

import numpy as np
import cv2
import threading
import time
from matplotlib import pyplot as plt

# user k = 27 for ESC

class VideoCaptureClass(threading.Thread):
    def __init__(self):
        #threading.Thread.__init__(self)
        self.main_thread_stop = threading.Event()
        self.video_capture_stop = threading.Event()
        self.video_analysis_stop = threading.Event()

        self.mainThread = threading.Thread(target=self.mainThreadRun, args=(1, self.main_thread_stop))
        self.videoCaptureThread = threading.Thread(target=self.videoCaptureThreadRun, args=(1,self.video_capture_stop))
        self.videoAnalysisThread = threading.Thread(target=self.videoAnalysisThreadRun, args=(1,self.video_analysis_stop))
        
        self.mainThread.start()
        self.videoCaptureThread.start()
        #self.videoAnalysisThread.start()
        

    def videoCaptureThreadRun(self, args, stop_event):
        self.cap = cv2.VideoCapture(0)
        x='0'
        while(not stop_event.is_set()):
            # Capture frame-by-frame
            stop_event.wait(0.1)
            print "video capture thread " ,self.videoCaptureThread.getName()
            ret, frame = self.cap.read()
            x=str(int(x,10)+1)
            image='/local/pics/file'+x+'.png'
            ret = cv2.imwrite(image,frame)
            ret = False
            if ( ret is False ):
                print "No Camera is found" 
                break #This thread will automatically kill when the main thread closes
            # Our operations on the frame come here
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Display the resulting frame
            cv2.imshow('frame',gray)
           #similar to time.sleep()

    def videoAnalysisThreadRun(self, args, stop_event):
        x='0'
        while(not stop_event.is_set()):
            stop_event.wait(1)
            print "video analysis thread ",self.videoAnalysisThread.getName()
            x=str(int(x,10)+1)
            img = cv2.imread('/local/pics/file'+x+'.png',0)
            plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
            plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
            #plt.show()

    def videoSavingThreadRun(self, arg1, stop_event):
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter('/local/pics/output.avi',fourcc, 20.0, (640,480))

    def mainThreadRun(self, args, stop_event):
            print "video capture and analyis thread started successfully"
            while (not stop_event.is_set()):
                stop_event.wait(1)
                print "Main Thread ",self.mainThread.getName()
                if ( cv2.waitKey(0) & 0xFF == ord('q')):
                    if ( self.videoCaptureThread.getName() is not None ):
                        self.video_capture_stop.set()
                        self.cap.release()
                        self.out.release()
                    if ( self.videoAnalysisThread.getName() is not None ):
                        self.video_analysis_stop.set()
                    cv2.destroyAllWindows()
                    break


if __name__ == '__main__':
    video_capture_thread = VideoCaptureClass()
    #video_capture_thread.daemon = True
    #video_capture_thread.start()




#img = cv2.imread('messi5.jpg')
#>>> px = img[100,100]
#>>> print px
#[157 166 200]
#
## accessing only blue pixel
#>>> blue = img[100,100,0]
#>>> print blue
#157
#>>> img[100,100] = [255,255,255]
#>>> print img[100,100]
#[255 255 255]
#
## modifying RED value
#>>> img.itemset((10,10,2),100)
#>>> img.item(10,10,2)
#100
#
#>>> print img.shape
#(342, 548, 3)
#
#
#>>> b,g,r = cv2.split(img)
#>>> img = cv2.merge((b,g,r))
#
#>>> b = img[:,:,0]
#
#Suppose, you want to make all the red pixels to zero, you need not split like this and put it equal to zero. You can simply use Numpy indexing, and that is more faster.
#
#>>> img[:,:,2] = 0
#
#import cv2
#import cv2.cv as cv
#import sys
#import numpy as np
#
#def getDisparity(imgLeft, imgRight, method="BM"):
#
#    gray_left = cv2.cvtColor(imgLeft, cv.CV_BGR2GRAY)
#    gray_right = cv2.cvtColor(imgRight, cv.CV_BGR2GRAY)
#    print gray_left.shape
#    c, r = gray_left.shape
#    if method == "BM":
#        sbm = cv.CreateStereoBMState()
#        disparity = cv.CreateMat(c, r, cv.CV_32F)
#        sbm.SADWindowSize = 9
#        sbm.preFilterType = 1
#        sbm.preFilterSize = 5
#        sbm.preFilterCap = 61
#        sbm.minDisparity = -39
#        sbm.numberOfDisparities = 112
#        sbm.textureThreshold = 507
#        sbm.uniquenessRatio= 0
#        sbm.speckleRange = 8
#        sbm.speckleWindowSize = 0
#
#        gray_left = cv.fromarray(gray_left)
#        gray_right = cv.fromarray(gray_right)
#
#        cv.FindStereoCorrespondenceBM(gray_left, gray_right, disparity, sbm)
#        disparity_visual = cv.CreateMat(c, r, cv.CV_8U)
#        cv.Normalize(disparity, disparity_visual, 0, 255, cv.CV_MINMAX)
#        disparity_visual = np.array(disparity_visual)
#
#    elif method == "SGBM":
#        sbm = cv2.StereoSGBM()
#        sbm.SADWindowSize = 9;
#        sbm.numberOfDisparities = 96;
#        sbm.preFilterCap = 63;
#        sbm.minDisparity = -21;
#        sbm.uniquenessRatio = 7;
#        sbm.speckleWindowSize = 0;
#        sbm.speckleRange = 8;
#        sbm.disp12MaxDiff = 1;
#        sbm.fullDP = False;
#
#        disparity = sbm.compute(gray_left, gray_right)
#        disparity_visual = cv2.normalize(disparity, alpha=0, beta=255, norm_type=cv2.cv.CV_MINMAX, dtype=cv2.cv.CV_8U)
#
#    return disparity_visual
#
#imgLeft = cv2.imread(sys.argv[1])
#imgRight = cv2.imread(sys.argv[2])
#try:
#    method = sys.argv[3]
#except IndexError:
#    method = "BM"
#
#disparity = getDisparity(imgLeft, imgRight, method)
#cv2.imshow("disparity", disparity)
#cv2.imshow("left", imgLeft)
#cv2.imshow("right", imgRight)
#cv2.waitKey(0)#

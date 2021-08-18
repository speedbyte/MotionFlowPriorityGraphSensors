Virtual KITTI 1.3.1
===================

http://www.xrce.xerox.com/Research-Development/Computer-Vision/Proxy-Virtual-Worlds

COPYRIGHT
Copyrights in The Virtual KITTI Dataset are owned by Xerox.

PLEASE READ THESE TERMS CAREFULLY BEFORE DOWNLOADING THE VIRTUAL KITTI DATASET. DOWNLOADING OR USING THE DATASET MEANS YOU ACCEPT THESE TERMS.
The Virtual KITTI Dataset is provided by Xerox and may be used for non-commercial purposes only and is subject to the Creative Commons Attribution-NonCommercial-ShareAlike 3.0.

ATTRIBUTION
The Virtual KITTI Dataset is an Adaptation of the KITTI Vision Benchmark Suite. See also the publication by Andreas Geiger and Philip Lenz and Raquel Urtasun, entitled "Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite", in Computer Vision and Pattern Recognition (CVPR), 2012.

CITATION
When using or referring to this dataset in your research, please cite Xerox as the originator of the Virtual KITTI Dataset and cite our CVPR 2016 paper (reference below).

Virtual Worlds as Proxy for Multi-Object Tracking Analysis
Adrien Gaidon, Qiao Wang, Yohann Cabon, Eleonora Vig
In IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2016


Optical Flow Ground
-------------------

Link: http://download.xrce.xerox.com/virtual-kitti-1.3.1/vkitti_1.3.1_flowgt.tar
 
The optical flow ground truth from the current frame to the next frame for each video is a folder in the format:

    vkitti_<version>_flowgt/<world>/<variation>/%05d.png

where each 3-channel PNG16 flow image (index starting from 00000) corresponds to the normalized, quantized, and masked (cf. below) flow from the current frame to the next frame (flow at t = frame t -> frame t+1). The flow values in pixels can be decoded from the RGB values of each pixel (16 bits per channel) as:

	R = flow along x-axis normalized by image width and quantized to [0;2^16 - 1]
	G = flow along y-axis normalized by image height and quantized to [0;2^16 - 1]
	B = 0 for invalid flow (e.g., sky pixels)

Some example decoding code in Python using OpenCV and numpy:

import numpy as np
import cv2

def read_vkitti_png_flow(flow_fn):
    "Convert from .png to (h, w, 2) (flow_x, flow_y) float32 array"
    # read png to bgr in 16 bit unsigned short
    bgr = cv2.imread(flow_fn, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    h, w, _c = bgr.shape
    assert bgr.dtype == np.uint16 and _c == 3
    # b == invalid flow flag: == 0 for sky or other invalid flow
    invalid = bgr[..., 0] == 0
    # g,r == flow_y,x normalized by height,width and scaled to [0;2**16 - 1]
    out_flow = 2.0 / (2**16 - 1.0) * bgr[..., 2:0:-1].astype('f4') - 1
    out_flow[..., 0] *= w - 1
    out_flow[..., 1] *= h - 1
    out_flow[invalid] = 0   # or another value (e.g., np.nan)
    return out_flow

Note that our ground truth normalized/quantized/masked flow .png files look often like a purple haze in contrast to the visualization at the beginning of this section, which uses the common Sintel color wheel to display the flow in pixels. This is normal, and the consequence of our 16-bit encoding designed to keep high precision, including for large displacements, in a standard loss-less compressed format (PNG16).

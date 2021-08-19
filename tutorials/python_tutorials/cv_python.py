# coding=utf-8

import numpy as np
import cv2
import math
from numpy.matlib import repmat

data = [
    [ 3.47358069,  3.44533315,  3.41713449,  -0.74376439, -0.74551423,
      -0.74726348],
    [ 6.31560125,  6.31206073,  6.30855598, -0.56062642, -0.56053701,
      -0.56044761],
    [10.52600208, 10.52010122, 10.51425997,  0.74750189,  0.74738269,
     0.74726348]
]



if __name__ == '__main__':

    image_raw = cv2.imread("Depth.png", cv2.IMREAD_UNCHANGED)
    # row, column
    image_raw_height = image_raw.shape[0]
    image_raw_width = image_raw.shape[1]
    image_type = image_raw.dtype
    image_raw_fov = 90

    #single_channel = image.reshape(1, image_shape)

    array  = np.array(data)
    array = array[2]
    print array

    arr = [1,2,3,4]
    print arr[3::-1]

    file = "/local/git/MotionFlowPriorityGraphSensors/project/carla_tutorials/Depth.png"

    """Convert a CARLA raw image to a BGRA numpy array."""
    far = 1000.0  # max depth in meters.
    max_depth=0.9

    array = np.frombuffer(image_raw.data, dtype=np.dtype("uint8"))

    array = np.reshape(array, (image_raw_height, image_raw_width, 4))
    # reshaping changes the row and column. So each element is a 4 channel vector.
    array = array.astype(np.float32)

    # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    normalized_depth = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
    normalized_depth /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

    shape = np.shape(normalized_depth)
    dims = np.ndim(normalized_depth)

    # (Intrinsic) K Matrix
    k = np.identity(3)
    k[0, 2] = image_raw_width / 2.0
    k[1, 2] = image_raw_height / 2.0
    k[0, 0] = k[1, 1] = image_raw_width / \
                        (2.0 * math.tan(image_raw_fov * math.pi / 360.0))

    # 2d pixel coordinates
    pixel_length = image_raw_width * image_raw_height
    u_coord = repmat(np.r_[image_raw_width-1:-1:-1],
                     image_raw_height, 1).reshape(pixel_length)
    v_coord = repmat(np.c_[image_raw_height-1:-1:-1],
                     1, image_raw_width).reshape(pixel_length)
    normalized_depth = np.reshape(normalized_depth, pixel_length)

    # Search for pixels where the depth is greater than max_depth to
    # delete them
    max_depth_indexes = np.where(normalized_depth > max_depth)
    normalized_depth = np.delete(normalized_depth, max_depth_indexes)
    u_coord = np.delete(u_coord, max_depth_indexes)
    v_coord = np.delete(v_coord, max_depth_indexes)

    # pd2 = [u,v,1]
    p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])

    # P = [X,Y,Z]
    p3d = np.dot(np.linalg.inv(k), p2d)
    p3d *= normalized_depth * far

    #print(p3d)
    final_depth = p3d[2]
    print np.shape(final_depth)
    depth_image = np.zeros((image_raw_height, image_raw_width), np.float)
    print "finished"
    #cv2.imshow("aaa", final_depth)
    #cv2.waitKey(0)

    #self._surface = pygame.surfarray.make_surface(final_depth)
    #self._surface_depth = pygame.surfarray.make_surface(array.swapaxes(0, 1))

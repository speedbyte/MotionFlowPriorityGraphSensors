/**
Ground truth in KITTI Dataset is acquired by accumulating 3D point clouds from a 360 degree Velodyne HDL-64
Laserscanner and fitting 3D CAD models to individually moving cars.
Outdoor scenes are decomposed
- Into a small number of independently moving objects.
- Each object in the scene is represented by its rigid motion parameters
- The objects are a part of a superpixel.
- The superpixel is represented by a 3D plane
- The super pixel consists of the index of all the objects that are the part of the superpixel.

There are 200 test and training
data image pairs.
The
 */

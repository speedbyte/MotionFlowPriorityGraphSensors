/**
 Ground truth in KITTI Dataset is acquired by accumulating 3D point clouds from a 360 degree Velodyne HDL-64
 Laserscanner and fitting 3D CAD models to individually moving cars.
 Outdoor scenes are decomposed
 - Into a small number of independently moving objects.
 - Each object in the scene is represented by its rigid motion parameters
 - The objects are a part of a superpixel.
 - The superpixel is represented by a 3D plane
 - The super pixel consists of the index of all the objects that are the part of the superpixel.

 There are 200 test and training data image pairs.
  The training data set contains image datapairs and various ground truth information such as flow, disparity etc. They
  are colour coded and hence are also lossless images. That means a normal character buffer is encoded in a lossless
  PNG format.

  The testing data set contains images over which new algorithms can be tested. The kitti development kit then
  converts into a format that can be used to compare it with the ground truth. Ofcourse the ground truth information
  has not been published for the testing image datapairs, because otherwise one can optimise their algorithm to fit
  the specifications of the ground truth.

  The testing image pairs and training image pairs have been randomly chosen from the kitti raw data set.

 */

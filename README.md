In this project a priority graph will be generated. Right sensors will be 
used at the right condition after reading the priority graph.
There will be an offline and an online priority graph. In the beginning, 
only the offline priority graph will be generated.

The start of the project is with Optical depth map using two optical sensors.
The next step would be to include the laser point of cloud.
After this the radar point cloud will be included.

In all the three cases, radar, lidar and opticals, real world data and virtual
world data would be used and compared for validation purposes.

In the real world, optical cameras such as raspberry pi is used to generate the
specific images. The radar and lidar is not yet available.

In the virtual world, The image is generated using the image generation 
in the vitual environment.
The lidar and radar point cloud will be generated using the ray tracing
technique.

Priority graphs are generated for ideal cases. However, in a real driving 
environment the conditions are not ideal. Therefore, distortions are induced
in the virtual environment, thus making the driving simulation more 
realistic.

A perfect ray tracing technique will hence simulate many false positives and
false negatives. These false positives and negatives due to echo, reflections,
and other interferences would be an input to existing ADAS algorithms and hence
make the safety critical algorithms more robust.

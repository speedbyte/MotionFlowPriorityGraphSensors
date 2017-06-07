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


Advantages and disadvantages of the Virtual world.

What is VIRES

From VIRES we get the sensor data such as LIDAR Point Cloud and Images or RADAR Pedesterian detection.
But the major hurdle is to get the sensor data that also matches the sensor data in the real world?
That means the generated image, lidar and radar data should be as close ( validated ) to the real world.

All the three sensors create sensor data by capturing the rays in the real world as well as the simulated world.
The only difference is that in the real world, the traversing of electromagnetic waves in the real world is automatic 
and in the simulated world, the path of the electromagnetic waves are simulated and hence is not perfect.

This phonomena is called raytracing, that means the virtual sensors send rays and they are refelected by objects.
The reflections are done using the physics. However, it is not optimum because the rays fails to simulate the reality.

----

The optical or scene flow needs to be focussed on specific algorithms and in this case, 
we choose scene flow as a possible algorithms for example pedesterian movements.

First evaluation is done on the images with optical flow.
We downloaded the Kitti evaulation kit on real data set and ran the evaluation on 3 algorithms to understand the system.
Thereafter we ran the kitti evaluation kiti on the VIRES generated dataset and again ran the evaluaiton on the 3 algorithms.

Next we took another algorithm ( http://ais.informatik.uni-freiburg.de/publications/papers/dewan-16iros.pdf ) 
that runs with LIDAR and again ran the Kitti evaluation with real data set.
Correspondingly, as in the first method we ran the Kitti evaluation with generated LIDAR Dataset.

Finally we wrote a wrapper to extract the best solution in different scenarios.
Basically, we combined the two algorithms for the final evaluation.
This should be able to improve the detection of scene flow in comparision to just the image or just the lidar data set or just the radar data set.

----

Bachelor Arbeit:

Evaluate optical flow algorithms with Kitti data and Generated VIRES Data.
Capture pedesterian movements with Raspberry or any stereo camera and then run the above algorithms.
Generate scenes in VIRES with respect to pedesterian detection ( there should be similarity between Raspberry and the VIRES images )  and run the algorithms.
Compare the two methods ( 1st - real world values and 2nd - virtual values )  for validation purpose.


BAchelor Arbeit:

Evaluate lidar flow algorithms with Kitti data and Generated VIRES Data.
Capture pedesterian movements with Velodyne and then run the above algorithms.
Generate scenes in VIRES with respect to pedesterian detection ( there should be similarity between Velodyne and the VIRES sensor data )  and run the algorithms.
Compare the two methods ( 1st - real world values and 2nd - virtual values )  for validation purpose.


Bachelor ARbeit:





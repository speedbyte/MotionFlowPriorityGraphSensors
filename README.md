

Adding ffmpeg
Adding opencv
Adding boost


4 C's

Confusion
Consistency
Compromise
Confidence

Transformation Framework
Communication Framework
Decision Framework
Diagnostics Framework
Verification Framework


Ich schlage vor folgendes: ein Webcam nehmen und eine Software von
Internet kopieren die drei Stifte in deinem Hand detektieren. Eine
zweite Webcam die eine andere Position hat detektiert 2 Stifte plus eins
die der erste nicht sieht. Also das End ergebniss ist, dass 4 Stifte zu
sehen sind ( Fusion Bild von beiden Kamera ).
Aber es reicht noch nicht aus. Es muss genau dokumentiert werden, wie
die 4 Stifte gesehen worden sind. Die Gewichtung muss hier definiert
werden und abgespeichert werden.

Wenn du nächstes Mal wieder mit 4 Stiften vor den zwei Kameras stehst,
dann es soll Blind die Gewichtung genommen werden. Jetzt mach mal mit n
Kamera anstatt zwei. Stell dir mal vor, wie schnell die Entscheidung
sein wird, dass 4 Stiften zu sehen sind ! Hammer oder? 

Bin mir sicher, hast nicht kappiert aber trotz hammer.


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

The software WeatherNeffects creates false positives in a virtual simulation world. This helps in creating
a confusion sensor database. This database is used as an input for verification of ADAS algorithms, such as 
pedesterian recogntion.


The focus is on the optical or scene flow, we choose pedesterian detection as a possible detection algorithm.

First evaluation is done on the images with optical flow.
We downloaded the Kitti evaulation kit on real data set and ran the evaluation on 3 algorithms to understand the system.
Thereafter we ran the kitti evaluation kiti on the VIRES generated dataset and again ran the evaluaiton on the 3 algorithms.

Bachelor Arbeit:
Note: To design the complete Sand area in VIRES and the algorithms would be primarily used on the parking lot.

1. Evaluate optical flow algorithms with Kitti data as learning process. 
2. Generate scenes in VIRES with respect to pedesterian detection ( there should be similarity between Raspberry and the VIRES images )  and run the algorithms.
3. Capture pedesterian movements ( optical flow ) with Raspberry or any stereo camera and then run the above algorithms. - Validation von VIRES Dataset.
Compare the two methods ( 2nd - virtual values and 3rd - real data set )  for validation purpose.

Next we took another algorithm ( http://ais.informatik.uni-freiburg.de/publications/papers/dewan-16iros.pdf ) 
that runs with LIDAR and again ran the Kitti evaluation with real data set.
Correspondingly, as in the first method we ran the Kitti evaluation with generated LIDAR Dataset.

Bachelor Arbeit:

1. Evaluate lidar flow algorithms with Kitti data and Generated VIRES Data.
2. Generate scenes in VIRES with respect to pedesterian detection ( there should be similarity between Velodyne and the VIRES sensor data )  and run the algorithms.
3. Capture pedesterian movements with Velodyne and then run the above algorithms.
Compare the two methods ( 1st - real world values and 2nd - virtual values )  for validation purpose.


Bachelor Arbeit:
Note: How does optical shadowing work with VIRES

1. Create objects with reflectivity index in the virtual simulation.
2. Run raytracing to simulate multipath fading, cross-talks and reflection of passing objects.
3. Capture pedesterian movements with Camera and then run the above algorithm.

Example: Refelction of a car in a lake for example can lead to false positives.

Bachelor Arbeit:

1. Create objects with reflectivity index in the virtual simulation.
2. Run raytracing to simulate multipath fading, cross-talks and reflection of passing objects.
3. Capture pedesterian movements with Camera and then run the above algorithm.

Example: Deflection of a beam through another object and eventually reception from the sender, that leads to further false positives. 


Finall we wrote a wrapper to extract the best solution in different scenarios.
Basically, we combined the two algorithms for the final evaluation.
This should be able to improve the detection of scene flow in comparision to just the image or just the lidar data set or just the radar data set.



----






VIRES Installation:
Die Dateien liegen hier:

https://secure.vires.com/demo/vtd/vtd.2.0.3.Demo.Road.20170131.tgz
https://secure.vires.com/demo/vtd/vtd.2.0.3.addOns.ROD64b.Standard_Dongle_20170209.tgz

login: demo
OpenSesame: viresDemo

VTD.2.0/Runtime/Tools/Installation
./checklib.sh | grep "not found"
Install all the packages one by one.


In my case:
    library libGL.so.1 ...not found.
    library libGL.so.1 ...not found.
    library libGL.so.1 ...not found.
    library libGL.so.1 ...not found.
    library libEGL.so.1 ...not found.
    library liblibcli_lsa3.so.0 ...not found.
    library libcli_smb_common.so.0 ...not found.
    library libutil_cmdline.so.0 ...not found.
    library libGLdispatch.so.0 ...not found.
    library libcli_cldap.so.0 ...not found.
    library libsmb_transport.so.0 ...not found.
    library libutil_tdb.so.0 ...not found.
    library libutil_reg.so.0 ...not found.
    library libsmbd_shim.so.0 ...not found.
    library libutil_setid.so.0 ...not found.
    library libutil_ntdb.so.0 ...not found.
    library libauth_sam_reply.so.0 ...not found.
    library libflag_mapping.so.0 ...not found.
    

freeglut, libdvdre4a

The license file needs to be placed under VTD2.0/bin/
Please make sure the license file obtained has the same MAC Address as the Dongle.

Wireless Dongle:

14.04
The precondition is that the wifi dongle should be loaded in ifconfig.
sudo apt-add-repository ppa:thopiekar/mt7601
sudo apt-get update
sudo apt-get install mt7601-sta-dkms
Sempre Wireless Dongles use MediaTek drivers ( mt7601u ). The following will
load the driver mt7601Usta to be found under 
/lib/modules/3.13.0-65-generic/updates/dkms/mt7601Usta.ko
Please check using modinfo mt7601Usta


16.04 - automatic because the drivers is already built in the kernel
To check please invoke lsmod | grep mt761u

if the device is not present in ifconfig, then check for ifconfig -a
Then start the link ip link set dev <devicename> up

Open the file Data/Setups/Current/Config/SimServer/simServer.xml
Add an environment variable: <EnvVar name="VI_LIC_DEVICE" val="ra0" /> (replace ra0 with the name of your device)

in the bin directory start .vtdStart.sh

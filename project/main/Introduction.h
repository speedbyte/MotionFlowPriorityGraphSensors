
/**

 Related Work:

 http://vision.middlebury.edu/flow/floweval-ijcv2011.pdf,
    Seite Nr. 19 ist einen Überblick der verschiedene Algorithm mit deren Runtime.
    Colorcode steht in 13 und flow error in 25
 Laboratory

 Ford Campus Vision and Lidar Data Set

 Michigan University

 Kitti:
 because it was found that high ranked optical flow algorithms in lab, fail in the real world. Hence kitti was
 designed for the real world.


 Presentation:
 noise induction = { simple, rain noise }
 2 sensor data - light spectrum and radio spectrum
 4 kind of environments  - self developed, external dataset, vires dataset, real environment
 4 layers - ground truth generation, noise induction[2], store returned intensity value*, inspect the returned
 intensity values using a well establised algorithm, publish paper

 *1 The assumption is that the intensity value returned would be different in different noise environments.
 *2 The chose of well established algorithm is described below.

 Choice of algorithm -
  - Optical flow was chosen, because it works on the principles of intensity movement. The algorithm tracks the
  intensities in the sensor database.

 Ground truth with pedesterians: 118,119,150,167,169 ( 169 is the best )
 5 percent or 3px displacement error means that the differnce between the ground truth and the generated displacement
 vector should not be more than 5% of the ground truth displacement vector. For very small displacement vector,
 ofcourse, 5 % is extremely small, hence the error tolerance is increased to 3px.

 What are the methods to generate ground truth?
  VIRES is one of the method to generate ground truth.
  Is Crowdsourcing for Optical Flow Ground Truth Generation Feasible?
  https://link.springer.com/chapter/10.1007/978-3-642-39402-7_20

 What are the methods to test ADAS algorithms. Rain Simulation is one of the method to generate noise.

 Regression analysis to determine the robustness of optical flow algorithms.
    In this regression between two independent variables such as one image with noise and one image
    without noise should be analysed. The regression methods can be eigenvalues, covariance, least square method etc.
    Through the regression analysis, it would be helpful to understand how the typical value of the dependant
    variable changes when one of the independent variable is varied, while the other independent variables are kept
    fixed.
    Dependant variable - displacement vector every 100ms
    Independant variable - movement of the subject
    Other independant variable - other objects are kept fixed.
    The analysis is to see how the displacement vector varies when the subject moves. And this is the ground truth,
    regression analysis of the displacement vector against the independant variable ( movement of the pedesterian ).

    The statistical regression analysis as said before is done by both the least squqre method and the eigen value
    method. Ofcourse the underlying algorithm to solve the least square and the eigen value is the QR decomposition
    method. And the methods to calculate the Q and R is the Householder Reflections method. The Householder
    Reflection itself is a linear transformation that enables a vector to be reflected through a plane or through
    hyper plane. The result is the Q component and the R component. The R component can also be achieved by reducing
    the original matrix in Hessenberg form in finite number of steps also through Householders algorithms. The
    Hessenberg is yet not a triangular matrix, but almost a triangular matrix. Further reduction of the Hessenberg to
    a triangular matrix ( R ) can be achieved through iterative procedures such as shifted QR factorization,
    sometimes combined with deflation steps. It was also possible to reduce the general matrix directly to a
    triangular matrix, but doing it through Hessenberg matrix, often economizes the arithmetic involved in the QR
    algorithm for eigen value problems.

    It is required to find two images whose variance is an orthogonal matrix.

 Input classes for statistical regression:
 Salt and Pepper Noise

 Fingerprinting to determine the robustness of optical flow algorithms.
 Computational complexity is the next step to determine how many BigOh's are required to complete the algorithm. The
 time required to solve the problem ( or the space required, or any measure of complexity ) is calculated as a
 funciton of the size of instance. It is interesting to see how the algorithm to find the robustness scale with
 increase in the input size of images.

 Flowchart for testing algorithms with Image Sensors:
 Acquisiton step and noise induction
 Acquire image data -> Induce Noise by some forumula -> New noised image data

 Apply optical flow algorithm step and noise induction
 Acquire new noised image data -> Induce specific movements as noise > New specific movements image data
 Acquire new specific movements image data -> Apply Optical Flow algorithm -> Output displacement vector

 Apply validation step and noise induction
 Acquire displacement vector -> Apply collision algorithm -> Output collision graph
 Compare input collision graph with output collision graph -> Output Error

 --------------------------------------

 Random variable is displacements greater than 2px in a sample space of an image with 100 moving objects. There can be
 no objects moving and hence the random variable as a real valued function has a value 0, because
 none of the objects are moving.

 --------------------------

 A graph to show the dynamicity of the image. Is the image set very dynamic or relatively static?
 Compare the dynamicity of a scene. Intuitively one can achieve a lot. This dynamicity is the fingerprint of the image.
 How the dynamicity of the image can confirm that two dynamics are the same.

 Ofcourse, the experiments cannot be done on all the movements. A reference needs to be set up and anything apart
 from this reference are outliers. Outlying observations can cause problems because they may strongly influence the
 result. OUtliers are detected by searching for the model fitted by the majority of the data. So, if there are
 many models ( areas ) , the model ( area ) with the maximum number of data can be used as the observation sample,
 and other models can be neglected or called outliers.  There are many robust statistics methods that deals with dealing
 the outliers. Robust procedures are applied on data, to detect the outliers.
 Univariate, low dimensional and high dimensional data, such as
 1. estimation of location and scatter,
 2. linear regression,
 3. principal component analysis, and
 4. classification

 Outliers ( observations, that are different from majority ) can be
 1. Errors
 2. recorded under exceptional circumstances
 3. belong to other populations
 Consequently, they do not fit the model well.


 Fingerprinting - are the displacement vector linear?

 Read two manual image files without rain. The two image files are from Marcel
 Read two manual image files with rain. Converted via Dennis tool.

 Read two kitti image files without rain. The two image files are from kitti
 Read two kitti image files with rain. Converted via Dennis tool.

 Read two VIRES image files without rain. The two image files are from VIRES
 Read two VIRES image files with rain. Converted via Dennis tool.

 For the ground truth, you need an external source.

 Parallel Threads:
 Thread 1:  1. Calculate Optical Flow between the two image file
            2. Ground Truth displacement vector is generated at the time of generating the files using coordinate system.
            3. Compare with ground truth.
            4. Plot Displacement Vector
            5. Standard deviation of all the displacement vector magnitude.
            6. Standard deviation of all the displacement vector angle.
            7. Cross Correlation map between the two different function output in 5 and 6.
 Thread 2:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is directly from the testing data in kitti database. Kitti guys generated the ground truth
            using Velodyne.
            3. Follow Step 2 onwards from above.
 Thread 3:  1. Use the pics and run different rain noise ( angle, how strong etc. )
            2. Ground Truth is from VIRES coordinate system.
            2. Follow Step 2 onwards from above.

 */



/**
 Project goals:
 From VIRES we get the sensor data such as LIDAR Point Cloud and Images or RADAR Pedesterian detection. But the major
 hurdle is to get the sensor data that also matches the sensor data in the real world? That means the generated
 image, lidar and radar data should be as close ( validated ) to the real world. All the three sensors create sensor
 data by capturing the rays in the real world as well as the simulated world. The only difference is that in the real
 world, the traversing of electromagnetic waves in the real world is automatic  and in the simulated world, the path
 of the electromagnetic waves are simulated and hence is not perfect.

 This phonomena is called raytracing, that means the virtual sensors send rays and they are refelected by objects.
 The reflections are done using the physics. However, it is not optimum because the rays fails to simulate the reality.


 Design of the software
 has-a or is-a relationship? Depending on these two aspects ( Composition or heirarchial ), we would proceed with our
 design.

 This section consists of the design of the overall software. First of all, it is important to define the number of
 interfaces from the Sensor value acquisition till the Optical Flow. The clases which define the interface has
 specific attributes and hence has a concrete implementaton. For example, if Patter recognition interface just
 produces abstract methods, but when a class is inherited from the interface, it should clearly define what kind of
 pattern recognition method is being used here.

 The interfaces are:

 Hardware Sensor interface. This includes a method to calibrate the sensors, as all sensor need some kind of
 calibration before data can be acquired

 Write memory interface. The user does not need to know, how the memory is distributed. What kind of file system and
 where the sensor data would be placed.

 Read Memory interface. The user does not need to know the layout of the memory. The memory reader interface should
 have abstract methods which provides the data when asked for from a definite memory location. Or initialise the
 memory location so that the read memory member function can always grab the memory from a definite address.

 Noise Interface. In order to filter the data, we need an interface through which the noise would be either filtered
 or induced. That  means this interface (between Memory Read and Data usage ) provides a method Noise induction or
 the method Data correction ( Noise filtering ). Again Noise can be induced in many many ways. One of the method is
 to add Noise by adding noise through Rain distortion models.

 Patter recognition: The interface lies between the filtered data and the object list. How the patter recognition is
 done i.e which algorithms for example Lukas kanade are taken is not important for the object lists. The patter
 recognition interface class just provides a trigger method to activate a specific optical flow algorithm, such as
 Lukas Kanade in the opencv library. Hence the triggerAlgorithm in the Patterrecogntion interface is just a template,
 because it is unknown which optical flow algorithm would be triggered. The conversion of filtered sensor data into
 object lists can then be acquired with the getter method.

 Object list validation interface : The object list validation interface provides an interface between the
 acquistion of the object list and other data. How the validation is done . For example if validation is done using
 voting mechanism or the validation is done against the ground truth is unknown at this stage. The result is stored
 in the database along with all the other surrounding information.

 Data statistics : The fitness landscape interface provides an interface between the object list validation and the
 end user interface. Test data needs to be stored somewhere, so that they can be used later. The end user, needs
 this statistical information along with the object list to come up with a decision.


 Algorithms

 Disparity
 Disparity is a phenomena when objects that are closer to the eyes will appear to jump a significant distance while
 objects further away will move very little. The motion of jumping is called disparity. This can be seen by closing
 one eye and suddenly opening another eye while closing the first eye. Basically, we are simulating a stereo vision
 with two lenses.
 In image processing, its a pixel differnce between two stereo images. In a gray scale disparity map, the brighter
 pixel denotes a greater shift and the darker pixel represents a smaller shift. This way, the relative distance
 between the object can be shown, that means a 3D representation in a 2D plane through color coding\\


 Pose
 Feature based pose estimation and 3D pose estimation


 */
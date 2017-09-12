
/**

 Papers:

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

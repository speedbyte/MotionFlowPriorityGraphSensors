sudo apt install emacs


===================================================================================

Source code for the paper:

     "A Variational Method for Scene Flow Estimation from Stereo Sequences"

Authors: Frederic.Huguet@inria.fr and Frederic.Devernay@inria.fr

Reference: Research Report 6267, INRIA https://hal.inria.fr/inria-00166589

===================================================================================

*** Copyright and license

The source code is (c)INRIA.

Permission is granted to use this code for educational and research purposes.
Commercial use is strictly forbidden.
The source code should not be distributed without the explicit authorization from
the authors.

*** System requirements

- Linux with g++ installed, MacOSX with Xcode, or MS Windows with Visual C++ 2005
- OpenCV library (>= 0.9.7)
- At least 1Gb RAM for the stereo initialization using belief propagation
  (initialization will run slowly with less memory)


*** Compilation

* Unix, Linux, MacOSX
Type "make"

* MS Windows
Open "sceneflow.sln" with MS Visual Studio 2005, then compile the Release version.


*** Usage

Execute the resulting sceneflow binary from the commqnd line, with one qrgument which is the
INI file to use.

Example:

(on Linux)
$ ./sceneflow Ball_SF.ini

(on MS Windows)
> sceneflow Ball_SF.ini

*** Files description

The archive, once unzipped, contains 1 directory with 3 subdirectories:
 
The main directory . provides source code :
    -ImagesTest.cpp : contains the function main(). The other functions reflects the organization of the algorithm described in the paper.
    -Resolution.cpp : provides the low level functions : optical flow and scene flow estimation.
    -source for a parser under licence

The algorithm can be runs with two ways : the scene flow estimation step of the algorithm  can be realized using several steps (D optimization, U,V Optimization, U,V,D' Optimization, and one step (U,V,D,D' single optimization).
It also provides data files to initialize the algorithm for the two ways. Each data file provides most of the optimal parameters for each example.
    -Ball.ini for the rotating ball with discontinuity with scene flow by steps computation, and Ball_SF.ini for scene flow in 1 step computation  
    -Teddy.ini for teddy example with scene flow by steps computation, and Tedy_SF.ini otherwise
    -Venus.ini for Venus example with scene flow by steps computation, and Venus_SF.ini otherwise
    -Cones.ini for Cones example with scene flow by steps computation, and Cones_SF.ini otherwise
   
Each of these .ini files provides the algorithm parameters for each example, and all the paths of the directories where results images are created.

Subdirectories:

*bp-vision : provides source code and a makefile for the BP algorithm used to initialize the disparity at time t. To get the binary file on your machine, simply go in the bp-vision subdirectory and then enter the command "make".

*Images : provides PGM input images for each example , Ball, Venus, teddy, cones (one subdirectory for each example)
*Results : provides PNG output images . One subdirectory for each example. And in these directories, you'll find final results images and a subdirectory "Debug" provinding the multiresolution gaussian pyramids and preliminary images results.


*** Intermediate results

All the parameters, and the evolution of the algorithm is written during run-time to the file sf.log in the main directory. Parameters first, and we print the evolution of the energy for both the optical flow and the scene flow problem at the beginning and at the end of each level of resolution. In the case of the scene flow, we break the energy to see the relative weight of every term relative to the another terms. The number of iterations needed is also printed in sf.log.

-The Results images.

In the subdirectory debug of each example directory (./Results/Ball/Debug, ./Results/Venus/Debug ...) you will find images of the actual evolution of the main unknowns of the problem : optical flow (u,v) and disparity at time t and t+1 (D, Dprime). The images which namme contains "delta" show the evolution of the increments of the unknowns after each linear system has been solved. this allows to follow almost in real time the convergence.
Images with "u0" or "v0" are the initial values at the beginning of each resolution level. You can also look at the multiresolution gaussian images pyramides (images with "Multiresolution" in their name).

In the example directories (./Results/Ball, ./Results/Venus ...) you will find the ground truth in both disparities at time t and t+1 (groundTruth_DispInit.png, groundTruthDisp.png) and for Teddy , Venus and Cones, the ground truth for the Flow along X (the flow is 0 in Y in these cases).
You will find the BP results both in true and false colors (BP_output.png, BP_Initial_Disparity.png). and the optical flow left computed alone (file names which contains "only", see for example ConesFlotX_only1.png , ConesFlotY_only1.png) and optical flow after scene flow evaluation (for example ConesFlot_X1.png, ConesFlot_Y1.png) . Disparity after scene flow estimation is in the same directory (ConesDisp_Init1.png for the initial disparity on the cones example after scene flow estimation, (ConesDisp1.png for the disparity at time t+1 on the cones example after scene flow estimation).
 

*** Computation time

-If you have 2Go Ram on your computer, and a Pentium IV 2Ghz the belief propagation initialisation is very fast , between 1 or 2 minutes on the biggest images (the Ball example).
- We tested on a Dual Core 2Ghz with 1Go Ram, we need between 5 and 10 minutes. Our code has no  
- With less than 1 Go ram there is a lot of swap and the computer slows down, so it is better to get at least 

Optical flow computation is very quick, but our code in scene flow has not been yet optimized using parallel  methods. 

On a Pentium IV 2Ghz, the total time computation for the most difficult case and the greatest Images (Ball exemple, images 512*512) is 30 mns. 


*** Programming interface to the scene flow and optical flow functions

Here are the functions header:


void OpticFlow(FILE* logfile, IplImage *Image, IplImage *Image_Init, const double epsilon_init, CvMat* u0, CvMat* v0, CvMat* u_end, CvMat* v_end, const double h, const double epsilon1, const double epsilon2, const int flow_down_level, const int flow_up_level, const double eta, int *sizeOX, int *sizeOY, const int NbIter, const int SORIter, CvMat* u_i, CvMat *v_i, const int level_i, const double alpha, const double gamma)


/*This method is called two times in ImagesTest.cpp, both for left and right optical flows. Its last parameter control the smoothness of the solution.
Here are some values taken to generate results preserving discontinuities (the highest alpha is, the smoothest the solution is which can be undesirable).
Ball.ini : alpha 250 gamma 1.5
Teddy.ini : alpha 250 gamma 5.0 
Venus.ini:  alpha 250 gamma 5.0
Cones.ini:  alpha 250 gamma 5.0

You can control the optical flow solution regularity by tuning the parameter alpha.
 
For the scene flow  */:

void SceneFlow(FILE* logfile, IplImage * Left_Image, IplImage * Right_Image, IplImage * Left_Image_Init, IplImage * Right_Image_Init, const double epsilon_init, CvMat* D_init, CvMat* u0, CvMat* v0, CvMat* ur0, CvMat* vr0, CvMat * Dprime_end, CvMat * u_end, CvMat * v_end, const double h, const double epsilon1, const double epsilon2, const double lambda, const double mu, const int detail_level_down, const int detail_level_up, const int methode, const double eta, const int NbIter, const int SORIter, const double omegaSOR, const double alpha, const double gamma, const int level_up, CvMat* disparity_up)

/* The parameters which could be modified are here alpha, for the smoothness of the final solution, and gamma which controls the gradients weights. Alpha should keep the same value as in the case of optical flow alone. 

*/ 




#ifndef RESOLUTION_H
#define RESOLUTION_H
#include <math.h>
#include <stdlib.h>
#include <cv.h>
#include <cvaux.h>
#include <cvtypes.h>
#include <highgui.h>
#include <stdio.h>
#include <cxcore.h>

int QR_factor(CvMat &A, CvMat& C, CvMat &D);

void SaveImage(const char *filename, const CvArr* arr);
void SaveResults(const char *filename, const CvArr* Results, int scale CV_DEFAULT(1), double min CV_DEFAULT(0.), double max CV_DEFAULT(-1.));

void PyramidBuild(CvMat *I,
                  double eta,
                  int pyramid_levels,
                  CvMat **I_pyramid);

void PyramidSmooth(int pyramid_levels,
                   CvMat **I_pyramid);

void PyramidCopy(int pyramid_levels,
                 const CvMat * const * const In_pyramid,
                 CvMat ** Out_pyramid);

void DtEstimation(const CvMat *U, //Left optical flow
		  const CvMat *Ur, //Right optical flow
		  const CvMat *D0, //Disparity d at time t
		  CvMat*Dt);
void ScaleChange (FILE* logfile, //the log file
                  const double eta, //reduction factor
                  int pyramid_levels, // total number of pyramid levels
                  const CvMat * const * const Il0_pyramid, //time t Image
                  const CvMat * const * const Ir0_pyramid, //time t Image
                  const CvMat * const * const Ilt_pyramid, //time t+1 Image
                  const CvMat * const * const Irt_pyramid, //time t+1 Image
                  int pyramid_level_start, // starting level
                  const CvMat* const U_start, //horizontal left optical flow initialization
                  const CvMat* const V_start, //vertical optical flow initialization
                  const CvMat* const D0_start, // disparity initialization
                  const CvMat* const Dt_start,
                  int pyramid_level_end,   // end pyramid level
                  CvMat* U_end, //final U
                  CvMat* V_end, //final V
                  CvMat* D0_end, //t disparity 
                  CvMat* Dt_end);

void SceneFlow(FILE* logfile, //the log file
	       const double eta, //reduction factor
	       int pyramid_levels, // total number of pyramid levels
	       const CvMat * const * const Il0_pyramid, //time t Image
	       const CvMat * const * const Ir0_pyramid, //time t Image
	       const CvMat * const * const Ilt_pyramid, //time t+1 Image
	       const CvMat * const * const Irt_pyramid, //time t+1 Image
               int pyramid_level_start, // starting level
	       CvMat* U_start, //horizontal left optical flow initialization
	       CvMat* V_start, //vertical optical flow initialization
               CvMat* D0_start, // disparity initialization
               CvMat* Dt_start,
               int pyramid_level_end,   // end pyramid level
	       CvMat* U_end, //final U
	       CvMat* V_end, //final V
               CvMat* D0_end, //t disparity 
               CvMat* Dt_end,
               const double h, //grid step for finite elements
	       const double epsilon_outer, //stopping criteria for outer iterations 
	       const double epsilon_inner, //stopping criteria for inner iterations
	       const double epsilon_SOR, //stopping criteria for the linear system resolution
	       const int max_iter_outer, //maximal number of outer iterations
	       const int max_iter_inner, //maximal number of inner iterations
	       const int max_iter_SOR, //maximal number of linear system resolution iterations
	       const double omega_SOR, //SOR method weight
               const double alpha,      // smooth term weight
               const double gamma,      // data term: gradient weight
               const double lambda,     // smooth term: disparity flow weight
               const double mu,         // smooth term: disparity weight
               const int methode);       //the chosen method 

void OpticalFlow(FILE* logfile, //The log file
	       const double eta, //reduction factor
	       int pyramid_levels, // total number of pyramid levels
	       const CvMat * const * const Il0_pyramid, //time t Image
	       const CvMat * const * const Ilt_pyramid, //time t+1 Image
               int pyramid_level_start, // starting level
	       const CvMat* U_start, // in : X component of Optical Flow
	       const CvMat* V_start, // in : X component of Optical Flow
               int pyramid_level_end,   // end pyramid level
	       CvMat* U_end, //final U
	       CvMat* V_end, //final V
	       const double h, //grid step for finite elements
	       const double epsilon_outer, //stopping criteria for outer iterations 
	       const double epsilon_inner, //stopping criteria for inner iterations
	       const double epsilon_SOR, //stopping criteria for the linear system resolution
	       const int max_iter_outer, //maximal number of outer iterations
	       const int max_iter_inner, //maximal number of inner iterations
	       const int max_iter_SOR, //maximal number of linear system resolution iterations
	       const double omega_SOR, //SOR method weight
	       const double alpha, //smoothing parameter
	       const double gamma); //the gradients weight
#endif

#include <stdio.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include "ImagesTest.h"
#include "Resolution.h"
#include "groundtruth.h"
#include "iniconfig.h"
#include "resize.h"
#ifdef linux
#include <signal.h>
#include <fenv.h>
#endif
#ifdef WIN32
#include <stdlib.h>
#define snprintf _snprintf
#endif

#include "bp-vision/stereo.h"

// parameters for the Ball
#define SIZE 256
#define RADIUS 0.45

#define SIMPLE_TEST 0

#ifndef M_PI
# define M_PI               3.14159265358979323846
#endif

#ifdef linux
static struct sigaction old_sigfpe;


static void
sigfpe_handler(int sig, siginfo_t *sip, void *uap) {
    char *label;
    switch (sip->si_code) {
        case FPE_FLTINV: label = "invalid operand"; break;
        case FPE_FLTRES: label = "inexact"; break;
        case FPE_FLTDIV: label = "division-by-zero"; break;
        case FPE_FLTUND: label = "underflow"; break;
        case FPE_FLTOVF: label = "overflow"; break;
        default: label = "???"; break;
    }
    fprintf(stderr, "FP exception %s (0x%x) occurred at address %p.\n",
	    label, sip->si_code, (void *) sip->si_addr);
    if(old_sigfpe.sa_handler == SIG_IGN) {
        printf("   signal ignored.\n");
    }
    else if (old_sigfpe.sa_handler == SIG_DFL) {
	printf("   signal with default action.\n");
	signal(sig, SIG_DFL);
	raise(sig);
    }
    else {
	printf("   signal handler at 0x%p\n", old_sigfpe.sa_handler);
	old_sigfpe.sa_handler(sig);
    }
}
#endif

static inline double square(double x) {
    return x*x;
}


// //Disparity ground truth computation
void disp_groundtruth(CvMat* true_disparity, const int Nx, const int Ny){
    int x,y;
    double xf, yf;
    for (y=0; y<Ny; y++) {
        for (x=0; x<Nx; x++) {
            xf = (x-Nx/2)/(double)(Nx);
            yf = -(y-Ny/2+1)/(double)(Ny);
            cvSetReal2D(true_disparity, y, x, disp_sphere(xf,yf,RADIUS)+1);
        }
    }
}

//Optical flow ground truth computation for the Y-rotating ball case
void flow_groundtruth(CvMat* true_flow, const int Nx, const int Ny, const double t)
{
    int x,y;
    double xf, yf,t2;
        
    t2 = t*M_PI/180.;
    for (y=0; y<Ny; y++) {
        for (x=0; x<Nx; x++) {
            xf = (x-Nx/2)/(double)(Nx);
            yf = -(y-Ny/2+1)/(double)(Ny);
            cvSetReal2D(true_flow, y, x, flowx_sphere(xf,yf,RADIUS, t2));
        }
    }
}

//Optical flow ground truth computation 
void flow_groundtruth2(CvMat* true_flow, const int Nx, const int Ny, const double t)
{
    int x,y;
    double xf, yf,t2;
        
    t2 = t*M_PI/180.;
    for (y=0; y<Ny; y++) {
        for (x=0; x<Nx; x++) {
            /* POVRAY coordinates relatives to the image center */
            /* has been runned only for SIZE = 2^k*/
            xf = (x-Nx/2)/(double)(Nx);
            yf = -(y-Ny/2+1)/(double)(Ny);
            cvSetReal2D(true_flow, y, x, flowx_sphere(-yf,xf,RADIUS, t2));
        }
    }
}

//Optical flow ground truth computation
void flow_groundtruth_fracture(CvMat* tOX, CvMat* tOY, CvMat *Disparity, const int Nx, const double tetaX, const double tetaY, const double tetaZ)
{
    double flowX, flowY, disp;
        
    for (int y=0; y<Nx; y++) {
        for (int x=0; x<Nx; x++) {
            flow_sphere(x, y, Nx, RADIUS, tetaX, tetaY, tetaZ, &flowX, &flowY, &disp);
            cvSetReal2D(tOX, y, x, flowX);
            cvSetReal2D(tOY, y, x, flowY);
            cvSetReal2D(Disparity, y, x, disp);
        }
    }
}


void Yosemite_generate_data(const int nb_images, const int sizeX, const int sizeY){
  
    FILE* data;
    unsigned char* tmp;
    unsigned int *tmp2;
    int i, x,y;
    char DataFile[256];
    char DataFileCloud[256];
    char ImageFile[256];
    char ImageFileCloud[256];
    double  Yo_dx[14];

    Yo_dx[0] = 0.000018075078;
    Yo_dx[1] = 0.000018053659;
    Yo_dx[2] = 0.00001799506;
    Yo_dx[3] = 0.000017974295;
    Yo_dx[4] = 0.000017907842;
    Yo_dx[5] = 0.000017895192;
    Yo_dx[6] = 0.00001787095;
    Yo_dx[7] = 0.000017811042;
    Yo_dx[8] = 0.000017794662;
    Yo_dx[9] = 0.000017771406;
    Yo_dx[10] = 0.000017714598;
    Yo_dx[11] = 0.000017692271;
    Yo_dx[12] = 0.000017631128;
    Yo_dx[13] = 0.000017612967;
    IplImage* Yoimage = cvCreateImage(cvSize(sizeX, sizeY), IPL_DEPTH_8U, 1);
    CvMat* ZData =  cvCreateMat(sizeY, sizeX, CV_32FC1);
    tmp = (unsigned char *)malloc((size_t)sizeX*sizeY*sizeof(char));
    tmp2 = (unsigned int *)malloc((size_t)sizeX*sizeY*sizeof(unsigned int));
    for(i = 0; i < nb_images; i++){
      sprintf(DataFile, "%s/yos%d%s", YosImagesPath, i, "/data");
        data = fopen(DataFile, "rb");
        fread((char *)tmp, sizeof(char), sizeX * sizeY, data);
        for(y = 0; y < sizeY; y++)
            for(x = 0; x < sizeX; x++)
                cvSetReal2D(Yoimage, y, x, (unsigned char)tmp[y*sizeX + x]);
        if (YosOutputImagesPrefix) {
            sprintf(ImageFile, "%s%d.%s", YosOutputImagesPrefix, i, YosOutputSuffix);
            cvSaveImage(ImageFile, Yoimage);
        }
        fclose(data);

        sprintf(DataFileCloud, "%s/data%d", YosCloudsPath, i);
        data = fopen(DataFileCloud, "rb");
        fread((char *)tmp, sizeof(char), sizeX * sizeY, data);
        for(y = 0; y < sizeY; y++)
            for(x = 0; x < sizeX; x++)
                cvSetReal2D(Yoimage, y, x, (unsigned char)tmp[y*sizeX + x]);
        if (YosOutputCloudsPrefix) {
            sprintf(ImageFileCloud, "%s%d.%s", YosOutputCloudsPrefix, i, YosOutputSuffix);
            cvSaveImage(ImageFileCloud, Yoimage);
        }
        fclose(data);
     
    }
    free(tmp);
    free(tmp2);
    cvReleaseImage(&Yoimage);
    cvReleaseMat(&ZData);
}

void Yosemite_generate_groundtruth(const int id_image, const int sizeX, const int sizeY, const double scale, const double dx, CvMat* True_OX, CvMat* True_OY){
    FILE* data;
    unsigned char* tmp;
    int x,y;
    char Data0File[1024];
    char Data1File[1024];
    char ImageFile[1024];

    tmp = (unsigned char *)malloc((size_t)sizeX*sizeY*sizeof(char));

    // X flow
    sprintf(Data1File, "%s/actual-flow%d/data1", YosFlowsPath, id_image);
    data = fopen(Data1File, "rb");
    fread((char *)tmp, sizeX * sizeY * sizeof(char), 1, data);
    for(y = 0; y < sizeY; y++)
        for(x = 0; x < sizeX; x++)
            cvSetReal2D(True_OX, y, x, (float)tmp[y*sizeX + x]*scale + dx);

    sprintf(ImageFile, "%sOX%d.%s", YosOutputFlowsPrefix, id_image, YosOutputSuffix);
    SaveResults(ImageFile, True_OX);
    fclose(data);
    
    // Y flow
    sprintf(Data0File, "%s/actual-flow%d/data0", YosFlowsPath, id_image);
    data = fopen(Data0File, "rb");
    fread((char *)tmp, sizeX * sizeY * sizeof(char), 1, data);
    for(y = 0; y < sizeY; y++)
        for(x = 0; x < sizeX; x++)
            cvSetReal2D(True_OY, y, x, (float)tmp[y*sizeX + x]*scale + dx);
    if (YosOutputFlowsPrefix) {
        sprintf(ImageFile, "%sOY%d.%s", YosOutputFlowsPrefix, id_image, YosOutputSuffix);
        SaveResults(ImageFile, True_OY);
    }
    fclose(data);
    free(tmp);
}

void AngularErrorGeneration(char *filename,  char* filenameOcc, CvMat *U, CvMat *V, CvMat *TruthU, CvMat *TruthV, CvMat* AngularErrorOcc, CvMat* AngularError, const int Nx, const int Ny, double* AverageAngularErrorOcc, double *StandardDeviationOcc,  double* AverageAngularError, double *StandardDeviation, const double alpha, const int type_images){
  
   
    double teta = 0.;
    double scalar = 0.;
    double det = 0.;
    double xf = 0.;
    double yf = 0.;
    double r = 0.;
    double sqrty = 0.;
    double alpha0 = 0.;
    double sum = 0.;
    double sumOcc = 0.;						
    double sum2 = 0.;
    double sum2Occ = 0.;
    CvMat* NulTruth = cvCreateMat(Ny, Nx, CV_32FC1);
    CvMat* NulTruthOcc = cvCreateMat(Ny, Nx, CV_32FC1);
    int bol1 = 1;
    int bol2 = 1;
    cvSetZero(NulTruth);
    cvSetZero(NulTruthOcc);
    int compteurOcc = Nx*Ny;
    int compteur = Nx*Ny;


    for(int y = 0; y < Ny; y++){
        for(int x = 0; x < Nx; x++){
            if(type_images){
               
                xf = (x-Nx/2)/(double)(Nx);
                yf = -(y-Ny/2+1)/(double)(Ny);
                r = RADIUS;
      

                //Outside of the ball ?
                if ((yf<=-r) || (yf>=r)){
                    compteur --;
                    compteurOcc--;
                    teta = 0.;
                    cvSetReal2D(NulTruth, y, x, 1.);
                    cvSetReal2D(NulTruthOcc, y, x, 1.);
                    bol1 = 0;
                }
	
                if(bol1){
                    sqrty = sqrt(1-yf*yf/(r*r));
	
                    if ((xf<-sqrty*r) || (xf>sqrty*r)){
                        bol2 = 0;
                        teta = 0.;
                        cvSetReal2D(NulTruth, y, x, 1.);
                        cvSetReal2D(NulTruthOcc, y, x, 1.);
                        compteur--;
                        compteurOcc--;
                    }
                }
            }
            //if inside we compute the Average Angular error (AAE)
            if(bol1 && bol2){
                if(type_images)
                    alpha0 = acos(xf/(sqrty*r));
                if(fabs(cvGetReal2D(TruthU, y, x)) < 0.001 && fabs(cvGetReal2D(TruthV, y, x)) < 0.001){
                    compteurOcc--;
                    compteur--;
                    cvSetReal2D(NulTruth, y, x, 1.);
                    cvSetReal2D(NulTruthOcc, y, x, 1.);
                    //if(fabs(cvGetReal2D(U, y, x)) < 0.001 && fabs(cvGetReal2D(V, y, x)) < 0.001)
		    teta = 0.;
                    //else
		    //  teta = 180.;
                }
                else{
                    scalar = cvGetReal2D(U, y, x)*cvGetReal2D(TruthU, y, x) + cvGetReal2D(V, y, x)*cvGetReal2D(TruthV, y, x);
                    det =  cvGetReal2D(U, y, x)*cvGetReal2D(TruthV, y, x) - cvGetReal2D(V, y, x)*cvGetReal2D(TruthU, y, x);
		    
                    teta = (180./M_PI)*fabs(atan2(det,scalar));
                    sumOcc += teta;
                }
                cvSetReal2D(AngularErrorOcc, y, x, teta);
              
                if ( type_images && ((alpha0+alpha) < 0 || (alpha0+alpha) > M_PI)){
                    //Occlusion
                    compteur--;
                    cvSetReal2D(NulTruth, y, x, 1.);
                    teta = 0;
                }
                if (teta != 180.) {
                    sum += teta;
                }
                cvSetReal2D(AngularError, y, x, teta);

            }
            bol1 = 1;
            bol2 = 1;
        }
    }
    sum /= compteur;
    sumOcc /= compteurOcc;
    *AverageAngularError = sum;
    *AverageAngularErrorOcc = sumOcc;
  
    //Standard deviation (SD)
    for(int j = 0; j < Ny; j++){
        for(int i = 0; i < Nx; i++){
            if(cvGetReal2D(NulTruth, j, i) > 0)
                sum2 += 0.;
            else
                sum2 += (cvGetReal2D(AngularError, j, i) - sum)*(cvGetReal2D(AngularError, j, i) - sum);
            if(cvGetReal2D(NulTruthOcc, j, i) > 0)
                sum2Occ += 0.;
            else
                sum2Occ += (cvGetReal2D(AngularErrorOcc, j, i) - sumOcc)*(cvGetReal2D(AngularErrorOcc, j, i) - sumOcc);
      
        }
    }
    sum2 /= compteur;
    sum2Occ /= compteurOcc;

    *StandardDeviation = sqrt(sum2);
    *StandardDeviationOcc = sqrt(sum2Occ);
    SaveResults(filename, AngularError);
    SaveResults(filenameOcc, AngularError);
    cvReleaseMat(&NulTruth);
    cvReleaseMat(&NulTruthOcc);
}

/******************************************Stereo Statistics for the Ball with the fracture ***************/
void BallStereoStatistics(CvMat *GT_D, CvMat *D, const int Nx, const int Ny, const double tetaX, const double tetaY, const double tetaZ, double *RMS){
    
    //Occluded pixels map 
    int bol1 = 1;
    
    //Nb of measures used to compute the disparity rms
    int nb_pixels = 0;
    double rms_tmp = 0.;
    
    const double cosa = 2/sqrt(4.25);
    const double sina = 0.5/sqrt(4.25);

  
    double angle_X = tetaX*M_PI/180.;
    double angle_Y = tetaY*M_PI/180.;
    double angle_Z = tetaZ*M_PI/180.;

    for(int y = 0; y < Ny; y++){
        for(int x = 0; x < Nx; x++){
           
            double xf = (x-Nx/2)/(double)(Nx);
            double yf = -(y-Ny/2+1)/(double)(Ny);
            double r = RADIUS;

            double Xl0 = xf/r;
            double Yl0 = yf/r;
            double Zl0 = Xl0*Xl0+Yl0*Yl0 - 1; 

            if (Zl0 > 0) //out of the ball
	      bol1 = 0;
            
            if(bol1){	
               
                Zl0 = -sqrt(-Zl0);
                /* convert from left image view to world coordinates: rotate around y axis */
                double Xw = Zl0*sina+Xl0*cosa;
                double Yw = Yl0;
                double Zw = Zl0*cosa-Xl0*sina;

                if (Xw > 0) {                 /* in right hemisphere */
                    angle_X = -angle_X;
                    /* uncomment the following to disable right hemisphere: */
                    /*
                     *flow_x = 0;
                     *flow_y = 0;
                     *disp = 0;
                     return;
                    */
                }

                /* rotate around X axis */
                double X = Xw;
                double Y = Yw*cos(angle_X)-Zw*sin(angle_X);
                double Z = Yw*sin(angle_X)+Zw*cos(angle_X);

                Xw = X;
                Yw = Y;
                Zw = Z;

                /* rotate around Y axis */
                Y = Yw;
                Z = Zw*cos(angle_Y)-Xw*sin(angle_Y);
                X = Zw*sin(angle_Y)+Xw*cos(angle_Y);

                Xw = X;
                Yw = Y;
                Zw = Z;
    
                /* rotate around Z axis */
                Z = Zw;
                X = Xw*cos(angle_Z)-Yw*sin(angle_Z);
                Y = Xw*sin(angle_Z)+Yw*cos(angle_Z);

                Xw = X;
                Yw = Y;
                Zw = Z;
    
                /* convert from world coordinates to left image view: rotate around y axis */
                //double Xl = -Zw*sina+Xw*cosa;
                //double Yl = Yw;
                double Zl = Zw*cosa+Xw*sina;

		//Current ground truth and estimated disparity 
		double gt_d = cvGetReal2D(GT_D, y, x);
		double d = cvGetReal2D(D, y, x);
                if(gt_d && Zl <= 0.){ //non flow-occluded pixel
		  rms_tmp += square(gt_d - d);
		  nb_pixels++;
		}
	    }
            bol1 = 1;
        }
    }
    if(nb_pixels)
      *RMS = sqrt(rms_tmp/nb_pixels);
    else
      *RMS = 1000.; //if no measure, put an error value
  
}

/****************************************** Optical Flow statistics for the Ball with the fracture ******/
void AngularErrorGenerationFracture(char *filename,  char* filenameOcc, CvMat *U, CvMat *V, CvMat *TruthU, CvMat *TruthV,  CvMat* AngularErrorOcc, CvMat* AngularError, const int Nx, const int Ny, double* AverageAngularErrorOcc, double *StandardDeviationOcc,  double* AverageAngularError, double *StandardDeviation, double *RMS_Flow, double *RMS_FlowOcc,  const double tetaX, const double tetaY, const double tetaZ){
  
    double RMSFlow_tmp = 0.;
    double RMSFlowOcc_tmp = 0.;
    double teta = 0.;
    double scalar = 0.; 
    double det = 0.;
    double xf = 0.;
    double yf = 0.;
    double r = 0.;
    double X,Y,Z,Xw,Yw,Zw, Xl0, Yl0, Zl0, Xl, Yl, Zl;
    double angle_X, angle_Y, angle_Z;
    double sum = 0.;
    double sumOcc = 0.;						
    double sum2 = 0.;
    double sum2Occ = 0.;
    CvMat* NulTruth = cvCreateMat(Ny, Nx, CV_32FC1);
    CvMat* NulTruthOcc = cvCreateMat(Ny, Nx, CV_32FC1);
    int bol1 = 1;
    cvSetZero(NulTruth);
    cvSetZero(NulTruthOcc);
    int compteurOcc = Nx*Ny;
    int compteur = Nx*Ny;
    const double cosa = 2/sqrt(4.25);
    const double sina = 0.5/sqrt(4.25);

  
    angle_X = tetaX*M_PI/180.;
    angle_Y = tetaY*M_PI/180.;
    angle_Z = tetaZ*M_PI/180.;

    for(int y = 0; y < Ny; y++){
        for(int x = 0; x < Nx; x++){
           
	    double gt_u = cvGetReal2D(TruthU, y, x);
	    double gt_v = cvGetReal2D(TruthV, y, x);
	    double u = cvGetReal2D(U, y, x);
	    double v = cvGetReal2D(V, y, x);

	   //  if(dodisparity && GT_D && D){
// 		double gt_d = cvGetReal2D(GT_D,y,x);
// 		double d = cvGetReal2D(D,y,x);
// 		RMS_D_tmp += square(gt_d - d);
// 	    }

            xf = (x-Nx/2)/(double)(Nx);
            yf = -(y-Ny/2+1)/(double)(Ny);
            r = RADIUS;

            Xl0 = xf/r;
            Yl0 = yf/r;
            Zl0 = Xl0*Xl0+Yl0*Yl0 - 1; 

            if (Zl0 > 0){ //out of the ball
                compteur --;
                compteurOcc--;
                teta = 0.;
                cvSetReal2D(NulTruth, y, x, 1.);
                cvSetReal2D(NulTruthOcc, y, x, 1.);
                bol1 = 0;
            }
	
           
            if(bol1){	
               
                Zl0 = -sqrt(-Zl0);
                /* convert from left image view to world coordinates: rotate around y axis */
                Xw = Zl0*sina+Xl0*cosa;
                Yw = Yl0;
                Zw = Zl0*cosa-Xl0*sina;

                if (Xw > 0) {                 /* in right hemisphere */
                    angle_X = -angle_X;
                    /* uncomment the following to disable right hemisphere: */
                    /*
                     *flow_x = 0;
                     *flow_y = 0;
                     *disp = 0;
                     return;
                    */
                }

                /* rotate around X axis */
                X = Xw;
                Y = Yw*cos(angle_X)-Zw*sin(angle_X);
                Z = Yw*sin(angle_X)+Zw*cos(angle_X);

                Xw = X;
                Yw = Y;
                Zw = Z;

                /* rotate around Y axis */
                Y = Yw;
                Z = Zw*cos(angle_Y)-Xw*sin(angle_Y);
                X = Zw*sin(angle_Y)+Xw*cos(angle_Y);

                Xw = X;
                Yw = Y;
                Zw = Z;
    
                /* rotate around Z axis */
                Z = Zw;
                X = Xw*cos(angle_Z)-Yw*sin(angle_Z);
                Y = Xw*sin(angle_Z)+Yw*cos(angle_Z);

                Xw = X;
                Yw = Y;
                Zw = Z;
    
                /* convert from world coordinates to left image view: rotate around y axis */
                Xl = -Zw*sina+Xw*cosa;
                Yl = Yw;
                Zl = Zw*cosa+Xw*sina;

            
                if(fabs(cvGetReal2D(TruthU, y, x)) < 0.001 && fabs(cvGetReal2D(TruthV, y, x)) < 0.001){
                    compteurOcc--;
                    compteur--;
                    cvSetReal2D(NulTruth, y, x, 1.);
                    cvSetReal2D(NulTruthOcc, y, x, 1.);
                    //if(fabs(cvGetReal2D(U, y, x)) < 0.001 && fabs(cvGetReal2D(V, y, x)) < 0.001)
		    teta = 0.;
			//else
                        //teta = M_PI;
                }
                else{
                    scalar = cvGetReal2D(U, y, x)*cvGetReal2D(TruthU, y, x) + cvGetReal2D(V, y, x)*cvGetReal2D(TruthV, y, x);
                    det =  cvGetReal2D(U, y, x)*cvGetReal2D(TruthV, y, x) - cvGetReal2D(V, y, x)*cvGetReal2D(TruthU, y, x);
                    teta = (180./M_PI)*fabs(atan2(det,scalar));
                    //teta = acos(scalar/(norm_vect(cvGetReal2D(U, y, x),  cvGetReal2D(V, y, x)) * norm_vect(cvGetReal2D(TruthU, y, x),  cvGetReal2D(TruthV, y, x))));
                    sumOcc += teta;
		    RMSFlowOcc_tmp += square(gt_u - u) + square(gt_v - v);
                }
                cvSetReal2D(AngularErrorOcc, y, x, teta);
                if (Zl > 0) {				/* undefined (flow occlusion) */
                    compteur--;
                    cvSetReal2D(NulTruth, y, x, 1.);
                    teta = 0; 
                }
		else{ // non occluded
		    RMSFlow_tmp += square(gt_u - u) + square(gt_v - v);
		     if (teta != 180.)
			 sum += teta;
		     cvSetReal2D(AngularError, y, x, teta);
		}              
            }
            bol1 = 1;
        }
    }
    sum /= compteur;
    sumOcc /= compteurOcc;
    *RMS_Flow = sqrt(RMSFlow_tmp/compteur);
    *RMS_FlowOcc = sqrt(RMSFlowOcc_tmp/compteurOcc);

    *AverageAngularError = sum;
    *AverageAngularErrorOcc = sumOcc;
  
    //standard deviation computation
    for(int j = 0; j < Ny; j++){
        for(int i = 0; i < Nx; i++){
            if(cvGetReal2D(NulTruth, j, i) > 0)
                sum2 += 0.;
            else
                sum2 += (cvGetReal2D(AngularError, j, i) - sum)*(cvGetReal2D(AngularError, j, i) - sum);
            if(cvGetReal2D(NulTruthOcc, j, i) > 0)
                sum2Occ += 0.;
            else
                sum2Occ += (cvGetReal2D(AngularErrorOcc, j, i) - sumOcc)*(cvGetReal2D(AngularErrorOcc, j, i) - sumOcc);
      
        }
    }

   //  if(dodisparity && GT_D && D)
// 	*D_RMS = RMS_D_tmp/sqrt(Nx*Ny);

    sum2 /= compteur;
    sum2Occ /= compteurOcc;
    *StandardDeviation = sqrt(sum2);
    *StandardDeviationOcc = sqrt(sum2Occ);
    (void)filename; //SaveResults(filename, AngularError);
    (void)filenameOcc; //SaveResults(filenameOcc, AngularError);
    cvReleaseMat(&NulTruth);
    cvReleaseMat(&NulTruthOcc);
}



/*******************************************Optical flow statistics for general examples ***********/
void OF_Statistics(CvMat *U, CvMat *V, CvMat *GT_U, CvMat *GT_V, double *AAE, double *sigma, double *RMS, const int Nx, const int Ny, const int occultations_flag, const char *OcclusionsFile = 0){

  //Generate the pixels map where the error can be computed
  CvMat *Pixels_Map = cvCreateMat(Ny, Nx, CV_32FC1);
  cvSetZero(Pixels_Map);
  //No measure if the groundtruth is 0
  for(int i = 0; i < Ny; i++){
      for(int j = 0; j < Nx; j++){
	//Groundtruth = 0 => no measure
	double gt_u = cvGetReal2D(GT_U, i, j);
	double gt_v = cvGetReal2D(GT_V, i, j);
	if(!gt_u && !gt_v)
	  cvSetReal2D(Pixels_Map,i,j,1.);
      }
  }
  
  //If an occlusion/discontinuity file is provided
  if(OcclusionsFile){
    //Reading the occlusions + discontinuities file
    IplImage *OccDisc = cvLoadImage(OcclusionsFile, CV_LOAD_IMAGE_GRAYSCALE);
    if (!OccDisc) {
        fprintf(stderr,"Error: could not load occlusion map %s\n", OcclusionsFile);
        exit(1);
    }
    for(int i = 0; i < Ny; i++){
      for(int j = 0; j < Nx; j++){
          
          //Occlusions : black
          double occ_value = cvGetReal2D(OccDisc, i, j);
          if(!occ_value)//Occlusion
              cvSetReal2D(Pixels_Map,i,j,1.);
          if(occultations_flag){//For discontinuities
              double disc_value = cvGetReal2D(OccDisc, i, j);
              if(disc_value == 255.)//discontinuity
                  cvSetReal2D(Pixels_Map,i,j,1.);
	}
      }
    }
    cvReleaseImage(&OccDisc);  
  }
 
  //Optical Flow statistics computation
  //Average Angular Error and RMS computation
  double average_angular_error = 0.;
  double rms_tmp = 0.;
  int pixels_number = 0;
  CvMat *AngularError = cvCreateMat(Ny, Nx, CV_32FC1);
  cvSetZero(AngularError);
  for(int y = 0; y < Ny; y++){
    for(int x = 0; x < Nx; x++){
      double pixel_value = cvGetReal2D(Pixels_Map,y,x);
      if(!pixel_value){
	//Angular Error computation
	double u = cvGetReal2D(U, y, x);
	double v = cvGetReal2D(V, y, x);
	double gt_u = cvGetReal2D(GT_U, y, x);
	double gt_v = cvGetReal2D(GT_V, y, x);
	double scalar = u*gt_u + v*gt_v;
        double det =  u*gt_v - v*gt_u;
        double teta = (180./M_PI)*fabs(atan2(det,scalar));
	average_angular_error += teta;
	cvSetReal2D(AngularError, y, x, teta);	
	rms_tmp += square(u-gt_u)+square(v-gt_v);
	pixels_number++;
      }
	
    }
  }

  //AAE
  average_angular_error /= pixels_number;
  *AAE = average_angular_error;
  *RMS = sqrt(rms_tmp/pixels_number);

  //Standard deviation computation
  double sigma_tmp = 0.;
  for(int y = 0; y < Ny; y++){
    for(int x = 0; x < Nx; x++){
      double pixel_value = cvGetReal2D(Pixels_Map,y,x);
      if(!pixel_value){
	double current_value = cvGetReal2D(AngularError, y, x);
	sigma_tmp += square(current_value - average_angular_error);
      }
    }
  }
  *sigma = sqrt(sigma_tmp/pixels_number);
  
  cvReleaseMat(&Pixels_Map);
  cvReleaseMat(&AngularError);    
}

/*******************************************Stereo statistics for general examples ***********/
void Stereo_Statistics(CvMat *D, CvMat *GT_D, double *RMS, const int Nx, const int Ny, const int occultations_flag , const char *OcclusionsFile = 0){
    //Generate the pixels map where the error can be computed
    CvMat *Pixels_Map = cvCreateMat(Ny, Nx, CV_32FC1);
    cvSetZero(Pixels_Map);
   
    //If an occlusion/discontinuity file is provided
    if(OcclusionsFile){
	//Reading the occlusions + discontinuities file
	IplImage *OccDisc = cvLoadImage(OcclusionsFile, CV_LOAD_IMAGE_GRAYSCALE);
	for(int i = 0; i < Ny; i++){
	    for(int j = 0; j < Nx; j++){
		//Occlusions : black
	      double occ_value = (double)cvGetReal2D(OccDisc, i, j);
		if(!occ_value)//Occlusion
		    cvSetReal2D(Pixels_Map,i,j,1.);
		if(occultations_flag){//For discontinuities
		    double disc_value = cvGetReal2D(OccDisc, i, j);
		    if(disc_value == 255.)//discontinuity
			cvSetReal2D(Pixels_Map,i,j,1.);
		}
	    }
	}
	cvReleaseImage(&OccDisc);  
    }
    
    
    //RMS computation
    double rms_tmp = 0.;
    int pixels_number = 0;
    
    for(int y = 0; y < Ny; y++){
	for(int x = 0; x < Nx; x++){
	    double pixel_value = cvGetReal2D(Pixels_Map,y,x);
	    if(!pixel_value){
		double d = cvGetReal2D(D, y, x);
		double gt_d = cvGetReal2D(GT_D, y, x);
		rms_tmp += square(d-gt_d);
		pixels_number++;
	    }
	    
	}
    }

 
  *RMS = sqrt(rms_tmp/pixels_number);
  cvReleaseMat(&Pixels_Map);
}


void RectifyStereoPairs(CvMat *StereoL, CvMat *StereoR, CvMat *StereoLRectified, CvMat *StereoRRectified, CvMat *RectificationMatrix){
    cvWarpPerspective(StereoL, StereoLRectified, RectificationMatrix);
    cvWarpPerspective(StereoR, StereoRRectified, RectificationMatrix);
}

/*************************************Create a basic images example to check the diffusion symetry****/
void CreateSimpleImages(CvMat *Il0, CvMat *Ir0, CvMat *Ilt, CvMat *Irt, const int Nx, const int Ny, const int l){
  cvSetZero(Il0);
  cvSetZero(Ir0);
  cvSetZero(Ilt);
  cvSetZero(Irt);
  
  const int x_center = (int)(Nx/2);
  const int y_center = (int)(Ny/2);
  int color = 0;
  for(int y = y_center - l; y <= y_center + l; y++){
    for(int x = x_center - l; x < x_center + l; x++){
      cvSetReal2D(Il0,y,x,256 - color);
      cvSetReal2D(Ir0,y,x + l,256 - color);
      cvSetReal2D(Ilt,y+l,x+l,256 - color);
      cvSetReal2D(Irt,y+l,x+l,256 - color);
      color += 2;
    }
  }
}


int main( int argc, char **argv )
{
#ifdef linux
    struct sigaction sa;

    feenableexcept(FE_DIVBYZERO|FE_INVALID|FE_OVERFLOW);
  
    sa.sa_handler = NULL;
    sa.sa_sigaction = sigfpe_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGFPE, &sa, &old_sigfpe);
#endif
    
    double h = 1.;

    //The input images are turned into grey scale images
    CvMat* Il0 = 0;
    CvMat* Ir0 = 0;
    CvMat* Ilt = 0;
    CvMat* Irt = 0;
 
    //Ground Truth
    CvMat *True_D0 = 0;
    CvMat *True_Dt = 0;
    CvMat *True_U = 0;
    CvMat *True_V = 0;

    FILE *logfile;
    char image_filename[1024];

    double RMS_Flow = 0.;
    double RMS_Disp0 = 0.;
    double RMS_Dispt = 0.;
    double RMS_Flow_Occ = 0.;
    double RMS_Disp0_Occ = 0.;
    double RMS_Dispt_Occ = 0.;
    double RMS_Flow_OccDisc = 0.;
    double RMS_Disp0_OccDisc = 0.;
    double RMS_Dispt_OccDisc = 0.;
    

    if (argc == 1) {
        fprintf(stderr,"Usage: %s <primary.ini> <secondary.ini> ...\n", argv[0]);
        fprintf(stderr,
                "Each ini file may contain the following values (default values are indicated).\n"
                "The last file on the command-line overrides the previous ones, and you can put\n"
                "different settings in different files.\n");
        ReadConfig(NULL);
        PrintConfig(stderr);
        exit(3);
    }
    for (int i=1; i<argc; i++)
        ReadConfig(argv[i]);

    // handle default values
    if (AlphaOF == -1)
        AlphaOF = Alpha;
    if (AlphaST == -1)
        AlphaST = Alpha;
    if (AlphaSF == -1)
        AlphaSF = Alpha;

    if (PyramidLevelOF < PyramidLevelSF) {
        fprintf(stderr,"Error: PyramidLevelOF < PyramidLevelSF\n");
        exit(1);
    }
    if (PyramidLevelST < PyramidLevelSF) {
        fprintf(stderr,"Error: PyramidLevelST < PyramidLevelSF\n");
        exit(1);
    }
    if (PyramidLevelSF < PyramidLevelFinal) {
        fprintf(stderr,"Error: PyramidLevelSF=%d < PyramidLevelFinal=%d\n", PyramidLevelSF, PyramidLevelFinal);
        exit(1);
    }
    logfile = fopen(LogFile, "w");
    PrintConfig(logfile);
    
    int PyramidLevels = PyramidLevelOF > PyramidLevelST ? PyramidLevelOF : PyramidLevelST;

    
   
    //for yosemite
    if(GT_Special == 2){
	//Yosemite_generate_data(15, 316, 252);
    }
    
    //Taking the first pair within the stereo sequence
    if(Il0_FileName) {
        Il0 = cvLoadImageM(Il0_FileName, CV_LOAD_IMAGE_GRAYSCALE);
        if (!Il0) {
            fprintf(stderr,"Error: could not load Il0=%s\n", Il0_FileName);
            exit(1);
        }
    }
    if(Ir0_FileName) {
        Ir0 = cvLoadImageM(Ir0_FileName, CV_LOAD_IMAGE_GRAYSCALE);
        if (!Ir0) {
            fprintf(stderr,"Error: could not load Ir0=%s\n", Ir0_FileName);
            exit(1);
        }
    }
    if(Ilt_FileName) {
        Ilt = cvLoadImageM(Ilt_FileName, CV_LOAD_IMAGE_GRAYSCALE);
        if (!Ilt) {
            fprintf(stderr,"Error: could not load Ilt=%s\n", Ilt_FileName);
            exit(1);
        }
    }
    if(Irt_FileName) {
        Irt = cvLoadImageM(Irt_FileName, CV_LOAD_IMAGE_GRAYSCALE);
        if (!Irt) {
            fprintf(stderr,"Error: could not load Irt=%s\n", Irt_FileName);
            exit(1);
        }
    }
    CvSize final_size = cvGetSize(Il0);
    int N_x = final_size.width;
    int N_y = final_size.height;

    ///////////////////////////////
    //        Ground truth       //
    ///////////////////////////////


    //Ground truth at time t AND time t+1
    if (GT_Special == 1) {
        True_U = cvCreateMat(N_y, N_x, CV_32FC1);
        True_V = cvCreateMat(N_y, N_x, CV_32FC1);
        True_D0 = cvCreateMat(N_y, N_x, CV_32FC1);
        True_Dt = cvCreateMat(N_y, N_x, CV_32FC1);
        flow_groundtruth_fracture(True_U, True_V, True_D0, N_x, 0., 0., 0.);
        flow_groundtruth_fracture(True_U, True_V, True_Dt, N_x, 1., 1., 1.);
        GT_D0_Invalid = -1.;
        GT_Dt_Invalid = -1.;
        GT_U_Invalid = -10.;
        GT_V_Invalid = -10.;
    }
    else if (GT_Special == 2) {
       //  True_U = cvCreateMat(N_y, N_x, CV_32FC1);
//         True_V = cvCreateMat(N_y, N_x, CV_32FC1);
       
//         //Flow Groundtruth for the first images pair
//         Yosemite_generate_groundtruth(0, 316, 252, 0.02892589569091797, -3.702331066131592, True_U, True_V);
    }
    else {
        if (GT_D0_FileName) {
            CvMat *img = cvLoadImageM(GT_D0_FileName,0);
            if (!img) {
                fprintf(stderr,"Error: could not load D0 ground truth %s\n", GT_D0_FileName);
                exit(1);
            }
            True_D0 = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvertScale(img, True_D0, GT_D0_Scale);
            cvReleaseMat(&img);
        }
        if (GT_Dt_FileName) {
            CvMat *img = cvLoadImageM(GT_Dt_FileName,0);
            if (!img) {
                fprintf(stderr,"Error: could not load Dt ground truth %s\n", GT_Dt_FileName);
                exit(1);
            }
            True_Dt = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvertScale(img, True_Dt, GT_Dt_Scale);
            cvReleaseMat(&img);
        }
        if (GT_U_FileName) {
            CvMat *img = cvLoadImageM(GT_U_FileName,0);
            if (!img) {
                fprintf(stderr,"Error: could not load U ground truth %s\n", GT_U_FileName);
                exit(1);
            }
            True_U = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvertScale(img, True_U, GT_U_Scale);
            cvReleaseMat(&img);
        }

      
	if (GT_V_FileName) {
	  CvMat *img = cvLoadImageM(GT_V_FileName,0);
	  if (!img) {
	    fprintf(stderr,"Error: could not load V ground truth %s\n", GT_V_FileName);
	    exit(1);
	  }
	  True_V = cvCreateMat(N_y, N_x, CV_32FC1);
	  cvConvertScale(img, True_V, GT_V_Scale);
	  cvReleaseMat(&img);
	}
    }

    if(GT_Special == 3 && !True_V){
      //Horizontal ground truth
      True_V = cvCreateMat(N_y, N_x, CV_32FC1);
      cvSetZero(True_V);
    }
    if (True_D0 && GT_D0_View_FileName)
        SaveResults(GT_D0_View_FileName, True_D0);
    if (True_Dt && GT_Dt_View_FileName)
        SaveResults(GT_Dt_View_FileName, True_Dt);
    if (True_U && GT_U_View_FileName)
        SaveResults(GT_U_View_FileName, True_U);
    if (True_V && GT_V_View_FileName)
        SaveResults(GT_V_View_FileName, True_V);
    
    //Multiresolution images computation
    if (PyramidFloatingPoint) {
        CvMat *floatim;
        if (Il0) {
            floatim = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvert(Il0,floatim);
            cvReleaseMat(&Il0);
            Il0 = floatim;
        }
        if (Ilt) {
            floatim = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvert(Ilt,floatim);
            cvReleaseMat(&Ilt);
            Ilt = floatim;
        }
        if (Ir0) {
            floatim = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvert(Ir0,floatim);
            cvReleaseMat(&Ir0);
            Ir0 = floatim;
        }
        if (Irt) {
            floatim = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvert(Irt,floatim);
            cvReleaseMat(&Irt);
            Irt = floatim;
        }
    }
    double eta_total = 1;
    int level;
    for(level = 1; level < PyramidLevels; level++){
        eta_total *= PyramidEta;
    }
    if (cvCeil(N_x* eta_total) < 4 || cvCeil(N_y* eta_total) < 4) {
        fprintf(stderr, "Too many pyramid levels, resulting images are smaller than 4x4, please try with less pyramid levels\n");
        exit(1);
    }

    CvMat **Il0_pyramid = 0;
    if (Il0) {
        Il0_pyramid = new CvMat*[PyramidLevels];
        PyramidBuild(Il0, PyramidEta, PyramidLevels, Il0_pyramid);
        if (Il0PyramidFilePrefix && ImagePyramidFileSuffix) {
            for(level = 0; level < PyramidLevels; level++){
                snprintf(image_filename, sizeof(image_filename), "%s%02d.%s", Il0PyramidFilePrefix, level+1, ImagePyramidFileSuffix);
                SaveImage(image_filename, Il0_pyramid[level]);
            }
        }
        // Il0_pyramid is pre-smoothed
        PyramidSmooth(PyramidLevels, Il0_pyramid);
    }

    CvMat **Ilt_pyramid = 0;
    if (Ilt) {
        Ilt_pyramid = new CvMat*[PyramidLevels];
        PyramidBuild(Ilt, PyramidEta, PyramidLevels, Ilt_pyramid);
        if (IltPyramidFilePrefix && ImagePyramidFileSuffix) {
            for(level = 0; level < PyramidLevels; level++){
                snprintf(image_filename, sizeof(image_filename), "%s%02d.%s", IltPyramidFilePrefix, level+1, ImagePyramidFileSuffix); 
                SaveImage(image_filename, Ilt_pyramid[level]);
            }
        }
    }

    CvMat **Ir0_pyramid = 0;
    if (Ir0) {
        Ir0_pyramid = new CvMat*[PyramidLevels];
        PyramidBuild(Ir0, PyramidEta, PyramidLevels, Ir0_pyramid);
        if (Ir0PyramidFilePrefix && ImagePyramidFileSuffix) {
            for(level = 0; level < PyramidLevels; level++){
                snprintf(image_filename, sizeof(image_filename), "%s%02d.%s", Ir0PyramidFilePrefix, level, ImagePyramidFileSuffix);
                SaveImage(image_filename, Ir0_pyramid[level]);
            }
        }
    }

    CvMat **Irt_pyramid = 0;
    if (Irt) {
        Irt_pyramid = new CvMat*[PyramidLevels];
        PyramidBuild(Irt, PyramidEta, PyramidLevels, Irt_pyramid);
        if (IrtPyramidFilePrefix && ImagePyramidFileSuffix) {
            for(level = 0; level < PyramidLevels; level++){
                snprintf(image_filename, sizeof(image_filename), "%s%02d.%s", IrtPyramidFilePrefix, level, ImagePyramidFileSuffix);
                SaveImage(image_filename, Irt_pyramid[level]);
            }
        }
    }

   
    
   if (True_U && True_V && True_D0 && True_Dt) {
         if (logfile) {
             fprintf(logfile, "*** Step 0: Weights estimation\n");
            fprintf(logfile, "** Initialization: Ground Truth\n");
            fflush(logfile);
        }

        SceneFlow(logfile,
                  PyramidEta,
                  PyramidLevels,
                  Il0_pyramid,
                  Ir0_pyramid,
                  Ilt_pyramid,
                  Irt_pyramid,
                  1,
                  True_U,
                  True_V,
                  True_D0,
                  True_Dt,
                  1,
                  True_U,
                  True_V,
                  True_D0,
                  True_Dt,
                  h,
#if 0
                  EpsilonOuter,
                  EpsilonInner,
                  EpsilonSORSF,
                  MaxIterOuter,
                  MaxIterInner,
                  MaxIterSORSF,
                  OmegaSORSF,
#else
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
#endif
                  AlphaSF,
                  Gamma,
                  Lambda,
                  Mu,
                  0);
        //exit(0);
    }
    /////////////////////////////////////////////
    //      Left optical flow computation      //
    /////////////////////////////////////////////
    CvSize size_flow_start = cvGetSize(Il0_pyramid[PyramidLevelOF - 1]);
    CvSize size_flow_end = cvGetSize(Il0_pyramid[PyramidLevelSF - 1]);
    CvSize size_final = cvGetSize(Il0_pyramid[PyramidLevelFinal - 1]);
    
    //Computation and EDP solving loop
    CvMat *U_start = cvCreateMat(size_flow_start.height, size_flow_start.width, CV_32FC1);
    CvMat *V_start = cvCreateMat(size_flow_start.height, size_flow_start.width, CV_32FC1);
    CvMat *U_end = cvCreateMat(size_flow_end.height, size_flow_end.width, CV_32FC1);
    CvMat *V_end = cvCreateMat(size_flow_end.height, size_flow_end.width, CV_32FC1);

    if (logfile) {
        fprintf(logfile, "*** Step 1: Left Flow\n");
        fprintf(logfile, "** Min. resolution: %dx%d (level %d)\n",
                size_flow_start.width, size_flow_start.height, PyramidLevelOF);
        fprintf(logfile, "** Max. resolution: %dx%d (level %d)\n",
                size_flow_end.width, size_flow_end.height, PyramidLevelSF);
        fprintf(logfile, "** Initialization: null flow\n");
    }
    cvSetZero(U_start);
    cvSetZero(V_start);

    OpticalFlow(logfile,
              PyramidEta,
              PyramidLevels,
              Il0_pyramid, Ilt_pyramid,
              PyramidLevelOF,
              U_start, V_start,
              PyramidLevelSF,
              U_end, V_end,
              h,
              EpsilonOuter, EpsilonInner, EpsilonSOR,
              MaxIterOuter, MaxIterInner, MaxIterSOR, OmegaSOROF,
              AlphaOF, Gamma);
    cvReleaseMat(&U_start);
    cvReleaseMat(&V_start);

    if (logfile) {
        fprintf(logfile, "*** Step 1.5: Continuing Left Flow (for measurement purposes)\n");
        fprintf(logfile, "** Min. resolution: %dx%d (level %d)\n",
                size_flow_end.width, size_flow_end.height, PyramidLevelSF);
        fprintf(logfile, "** Max. resolution: %dx%d (level %d)\n",
                size_final.width, size_final.height, PyramidLevelFinal);
        fflush(logfile);
    }
    CvMat *U_final = cvCreateMat(size_final.height, size_final.width, CV_32FC1);
    CvMat *V_final = cvCreateMat(size_final.height, size_final.width, CV_32FC1);

    OpticalFlow(logfile,
              PyramidEta,
              PyramidLevels,
              Il0_pyramid, Ilt_pyramid,
              PyramidLevelSF,
              U_end, V_end,
              PyramidLevelFinal,
              U_final, V_final,
              h,
              EpsilonOuter, EpsilonInner, EpsilonSOR,
              MaxIterOuter, MaxIterInner, MaxIterSOR, OmegaSOROF,
              AlphaOF, Gamma);

    //Saving the optical flow images
    if (OF_U_Final_FileName)
        SaveResults(OF_U_Final_FileName, U_final);
    if (OF_V_Final_FileName)
        SaveResults(OF_V_Final_FileName, V_final);

    CvMat *U_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(U_final, U_total, CV_INTER_AREA);
    cvReleaseMat(&U_final);
    cvConvertScale(U_total, U_total, (double)N_x / size_final.width);
    CvMat *V_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(V_final, V_total, CV_INTER_AREA);
    cvReleaseMat(&V_final);
    cvConvertScale(V_total, V_total, (double)N_y / size_final.height);

    //Mean angular error and standard deviation computation
    //printf("exiting!\n");exit(1);
    //Error maps
    CvMat* AngularError = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat* AngularErrorOcc = cvCreateMat(N_y, N_x, CV_32FC1);
    cvSetZero(AngularError);
    cvSetZero(AngularErrorOcc);

    double AAE_alone, SD_alone,  AAE_aloneOcc, SD_aloneOcc, AAE_aloneOccDisc, SD_aloneOccDisc;
    if(GT_Special == 1 && True_U && True_V)             // Ball
        AngularErrorGenerationFracture(OF_FlowAngularError_FileName, OF_FlowAngularErrorOcclusions_FileName, U_total, V_total, True_U, True_V, AngularErrorOcc, AngularError, N_x, N_y, &AAE_aloneOcc, &SD_aloneOcc, &AAE_alone, &SD_alone, &RMS_Flow, &RMS_Flow_Occ, 1., 1., 1.);
    else if (True_U && True_V) {
	//Optical flow statistics computed with occultations and discontinuities
	OF_Statistics(U_total, V_total, True_U, True_V, &AAE_alone, &SD_alone, &RMS_Flow, N_x, N_y, 0);
	if(GT_OcclusionsDiscontinuities){
	    //with discontinuities and no occlusions 
	    OF_Statistics(U_total, V_total, True_U, True_V, &AAE_aloneOcc, &SD_aloneOcc, &RMS_Flow_Occ, N_x, N_y, 0, GT_OcclusionsDiscontinuities);
	    //computation without disc neither occlusions
	    OF_Statistics(U_total, V_total, True_U, True_V, &AAE_aloneOccDisc, &SD_aloneOccDisc, &RMS_Flow_OccDisc, N_x, N_y, 1, GT_OcclusionsDiscontinuities);
	}
    }
      

    cvReleaseMat(&AngularError);
    cvReleaseMat(&AngularErrorOcc);

		
    if (logfile && True_U && True_V) {
        fprintf(logfile, "Flow RMS with occlusion = %g\n", RMS_Flow);
        fprintf(logfile, "Flow RMS without occlusion = %g\n", RMS_Flow_Occ);
        fprintf(logfile, "Flow RMS without occlusion and discontinuities= %g\n", RMS_Flow_OccDisc);
        fprintf(logfile, "Angular error mean with occlusion = %g\n", AAE_alone);
        fprintf(logfile, "Angular error sdev with occlusion = %g\n", SD_alone);
        fprintf(logfile, "Angular error mean without occlusion = %g\n", AAE_aloneOcc);
        fprintf(logfile, "Angular error sdev without occlusion = %g\n", SD_aloneOcc);
        if(GT_OcclusionsDiscontinuities){
            fprintf(logfile, "Angular error mean without occlusion neither discontinuities= %g\n", AAE_aloneOccDisc);
            fprintf(logfile, "Angular error sdev without occlusion neither discontinuities = %g\n", SD_aloneOccDisc);
        }
        fflush(logfile);
    }


    cvReleaseMat(&U_total);
    cvReleaseMat(&V_total);

    //////////////////////////////////////////////
    //      Right optical flow computation      //
    //////////////////////////////////////////////
    CvMat *Ur_start = cvCreateMat(size_flow_start.height, size_flow_start.width, CV_32FC1);
    CvMat *Vr_start = cvCreateMat(size_flow_start.height, size_flow_start.width, CV_32FC1);
    CvMat *Ur_end = cvCreateMat(size_flow_end.height, size_flow_end.width, CV_32FC1);
    CvMat *Vr_end = cvCreateMat(size_flow_end.height, size_flow_end.width, CV_32FC1);
    if (logfile) {
        fprintf(logfile, "*** Step 2: Right Flow\n");
        fprintf(logfile, "** Min. resolution: %dx%d (level %d)\n",
                size_flow_start.width, size_flow_start.height, PyramidLevelOF);
        fprintf(logfile, "** Max. resolution: %dx%d (level %d)\n",
                size_flow_end.width, size_flow_end.height, PyramidLevelSF);
        fprintf(logfile, "** Initialization: null flow\n");
    }
    cvSetZero(Ur_start);
    cvSetZero(Vr_start);
    {
        CvMat **Ir0s_pyramid = 0;
        Ir0s_pyramid = new CvMat*[PyramidLevels];
        PyramidCopy(PyramidLevels, Ir0_pyramid, Ir0s_pyramid);
        PyramidSmooth(PyramidLevels, Ir0s_pyramid);

        OpticalFlow(logfile,
                    PyramidEta,
                    PyramidLevels,
                    Ir0s_pyramid, Irt_pyramid,
                    PyramidLevelOF,
                    Ur_start, Vr_start,
                    PyramidLevelSF,
                    Ur_end, Vr_end,
                    h,
                    EpsilonOuter, EpsilonInner, EpsilonSOR,
                    MaxIterOuter, MaxIterInner, MaxIterSOR, OmegaSOROF,
                    AlphaOF, Gamma);
        for(level = 0; level < PyramidLevels; level++)
            cvReleaseMat(&Ir0s_pyramid[level]);
        delete [] Ir0s_pyramid;
    }
    
    //Saving the optical flow images
    if (OF_Ur_Final_FileName)
        SaveResults(OF_Ur_Final_FileName, Ur_end);
    if (OF_Vr_Final_FileName)
        SaveResults(OF_Vr_Final_FileName, Vr_end);

    cvReleaseMat(&Ur_start);
    cvReleaseMat(&Vr_start);

    //////////////////////////////////
    //      Stereo computation      //
    //////////////////////////////////
    CvSize size_stereo_start = cvGetSize(Il0_pyramid[PyramidLevelST - 1]);
    CvSize size_stereo_end = cvGetSize(Il0_pyramid[PyramidLevelSF - 1]);
    CvMat *D0_start = 0;
    CvMat *D0_end = cvCreateMat(size_stereo_end.height, size_stereo_end.width, CV_32FC1);
    CvMat *Dt_start = 0;
    CvMat *Dt_end = 0;

    if(DoStereo){
        D0_start = cvCreateMat(size_stereo_start.height, size_stereo_start.width, CV_32FC1);
        if (logfile) {
            fprintf(logfile, "*** Step 3: Stereo\n");
            fprintf(logfile, "** Min. resolution: %dx%d (level %d)\n",
                    size_stereo_start.width, size_stereo_start.height, PyramidLevelOF);
            fprintf(logfile, "** Max. resolution: %dx%d (level %d)\n",
                    size_stereo_end.width, size_stereo_end.height, PyramidLevelSF);
            fflush(logfile);
        }
        if (!InitStereoBP) {
            if (logfile) {
                fprintf(logfile, "** Initialization: null flow\n");
                fflush(logfile);
            }
            cvSetZero(D0_start);
        }
        else {
            char Command[1024];
            char *ST_Il0 = Il0_FileName;
            char *ST_Ir0 = Ir0_FileName;
            if (ST_Il0_FileName) {
                unsigned char *data;
                cvGetRawData(Il0,&data);
                FILE *f = fopen(ST_Il0_FileName,"w");
                fprintf(f,"P5\n%d %d\n255\n", N_x, N_y);
                fwrite(data,1,N_x*N_y,f);
                fclose(f);
                ST_Il0 = ST_Il0_FileName;
            }
            if (ST_Ir0_FileName) {
                unsigned char *data;
                cvGetRawData(Ir0,&data);
                FILE *f = fopen(ST_Ir0_FileName,"w");
                fprintf(f,"P5\n%d %d\n255\n", N_x, N_y);
                fwrite(data,1,N_x*N_y,f);
                fclose(f);
                ST_Ir0 = ST_Ir0_FileName;
            }
            snprintf(Command, sizeof(Command), "%s %s %s %s", StereoExecutable, ST_Il0, ST_Ir0,  ST_D0_Init_Middlebury_FileName);
            //Belief Propagation for d
            if (logfile) {
                fprintf(logfile, "** Initialization: external command \"%s\"\n", Command);
                fflush(logfile);
            }
            int status = system(Command);
            if (status != 0) {
                fprintf(stderr,"Error: command \"%s\" failed\n", Command);
                exit(1);
            }
            //Taking the result
            CvMat *img = cvLoadImageM(ST_D0_Init_Middlebury_FileName,0);
            if (!img) {
                fprintf(stderr,"Error: could not load external stereo result %s\n", ST_D0_Init_Middlebury_FileName);
                exit(1);
            }
            CvMat *D0 = cvCreateMat(N_y, N_x, CV_32FC1);
            cvConvertScale(img, D0, -1/(double)BP_SCALE); //Rescaling
            cvReleaseMat(&img);

            // Adjust border values by replication
            int x, y;
            for(y = 1; y < N_y-1; y++) {
                cvSetReal2D(D0, y,0, cvGetReal2D(D0, y, 1));
                cvSetReal2D(D0, y,N_x-1, cvGetReal2D(D0, y, N_x-2));
            }
            for(x = 0; x < N_x; x++) {
                cvSetReal2D(D0, 0,x, cvGetReal2D(D0, 1, x));
            }
            for(x = 0; x < N_x; x++) {
                cvSetReal2D(D0, N_y-1,x, cvGetReal2D(D0, N_y-2,x));
            }
            cvResize(D0, D0_start, CV_INTER_AREA);
            cvConvertScale(D0_start, D0_start, (double)size_stereo_start.width / N_x);
            cvReleaseMat(&D0);
        }
        
        SceneFlow(logfile,
                  PyramidEta,
                  PyramidLevels,
                  Il0_pyramid,
                  Ir0_pyramid,
                  0,
                  0,
                  PyramidLevelST,
                  0,
                  0,
                  D0_start,
                  0,
                  PyramidLevelSF,
                  0,
                  0,
                  D0_end,
                  0,
                  h,
                  EpsilonOuter,
                  EpsilonInner,
                  EpsilonSOR,
                  MaxIterOuter,
                  MaxIterInner,
                  MaxIterSOR,
                  OmegaSORST,
                  AlphaST,
                  Gamma,
                  Lambda,
                  Mu,
                  5);
	
        if (ST_D0_FileName){
          double minD0, maxD0;
	  cvMinMaxLoc(D0_end, &minD0, &maxD0);
	  SaveResults(ST_D0_FileName, D0_end);
	}
        cvReleaseMat(&D0_start);
    }
    else
        cvSetZero(D0_end);
    
  

	

	//////////////////////////////////////
    //      Scene flow computation      //
    //////////////////////////////////////
    CvSize size_sf_start = cvGetSize(Il0_pyramid[PyramidLevelSF - 1]);

    if (logfile) {
        fprintf(logfile, "*** Step 4 : Scene Flow\n");
        fprintf(logfile, "** Min. resolution: %dx%d (level %d)\n",
                size_sf_start.width, size_sf_start.height, PyramidLevelSF);
        fprintf(logfile, "** Max. resolution: %dx%d (level %d)\n",
                size_final.width, size_final.height, PyramidLevelFinal);
        if(Lambda > Mu)
            fprintf(logfile, "** WARNING! Lamba=%g > Mu=%g, this may generate oscillations of the disparity flow!\n",
                    Lambda, Mu);
        fflush(logfile);
    }
    
    // Initialization from Optical Flow + Stereo
    U_start = U_end;
    U_end = 0;
    V_start = V_end;
    V_end = 0;
    D0_start = D0_end;
    D0_end = 0;
    Ur_start = Ur_end;
    Ur_end = 0;
    Vr_start = Vr_end;
    Vr_end = 0;
    Dt_start = cvCreateMat(size_sf_start.height, size_sf_start.width, CV_32FC1);
    //Disparity d' estimation using left and right optical flows
    DtEstimation(U_start, Ur_start, D0_start, Dt_start);
    cvReleaseMat(&Ur_start);
    cvReleaseMat(&Vr_start);
  

    if (SF_Steps == 0) {
        U_end = cvCreateMat(size_final.height, size_final.width, CV_32FC1);
        V_end = cvCreateMat(size_final.height, size_final.width, CV_32FC1);
        D0_end = cvCreateMat(size_final.height, size_final.width, CV_32FC1);
        Dt_end = cvCreateMat(size_final.height, size_final.width, CV_32FC1);
        SceneFlow(logfile,
                  PyramidEta,
                  PyramidLevels,
                  Il0_pyramid,
                  Ir0_pyramid,
                  Ilt_pyramid,
                  Irt_pyramid,
                  PyramidLevelSF,
                  U_start,
                  V_start,
                  D0_start,
                  Dt_start,
                  PyramidLevelFinal,
                  U_end,
                  V_end,
                  D0_end,
                  Dt_end,
                  h,
                  EpsilonOuter,
                  EpsilonInner,
                  EpsilonSORSF,
                  MaxIterOuter,
                  MaxIterInner,
                  MaxIterSORSF,
                  OmegaSORSF,
                  AlphaSF,
                  Gamma,
                  Lambda,
                  Mu,
                  0);
        
        cvReleaseMat(&U_start);
        cvReleaseMat(&V_start);
        cvReleaseMat(&D0_start);
        cvReleaseMat(&Dt_start);
    }
    else {
        // we must set these in case PyramidLevelSF == PyramidLevelFinal
        U_end = U_start;
        V_end = V_start;
        D0_end = D0_start;
        Dt_end = Dt_start;
        //Loop over the resolution levels to optimize separately D and U,V from U,V,D'
        for(int level = PyramidLevelSF; level >= PyramidLevelFinal; level--){
            if (logfile) {
                fprintf(logfile, "** Scene Flow : D Optimization\n");
                fflush(logfile);
            }

            //Optimizing D0 alone
            SceneFlow(logfile,
                      PyramidEta,
                      PyramidLevels,
                      Il0_pyramid,
                      Ir0_pyramid,
                      0,
                      0,
                      level,
                      0,
                      0,
                      D0_start,
                      0,
                      level,
                      0,
                      0,
                      D0_start,
                      0,
                      h,
                      EpsilonOuter,
                      EpsilonInner,
                      EpsilonSOR,
                      MaxIterOuter,
                      MaxIterInner,
                      MaxIterSOR,
                      OmegaSORST,
                      AlphaST,
                      Gamma,
                      Lambda,
                      Mu,
                      5);

            if (logfile) {
                fprintf(logfile, "** Scene Flow : U,V Optimization\n");
                fflush(logfile);
            }

	    SceneFlow(logfile,
                      PyramidEta,
                      PyramidLevels,
                      Il0_pyramid,
                      0,
                      Ilt_pyramid,
                      0,
                      level,
                      U_start,
                      V_start,
                      0,
                      Dt_start,
                      level,
                      U_start,
                      V_start,
                      0,
                      0,
                      h,
                      EpsilonOuter,
                      EpsilonInner,
                      EpsilonSOR,
                      MaxIterOuter,
                      MaxIterInner,
                      MaxIterSOR,
                      OmegaSOROF,
                      AlphaOF,
                      Gamma,
                      Lambda,
                      0.,
                      2);
	    
	  
            
            if (logfile) {
                fprintf(logfile, "** Scene Flow : U,V,D' Optimization\n");
                fflush(logfile);
            }
	
            SceneFlow(logfile,
                      PyramidEta,
                      PyramidLevels,
                      Il0_pyramid,
                      0,
                      Ilt_pyramid,
                      Irt_pyramid,
                      level,
                      U_start,
                      V_start,
                      D0_start,
                      Dt_start,
                      level,
                      U_start,
                      V_start,
                      0,
                      Dt_start,
                      h,
                      EpsilonOuter,
                      EpsilonInner,
                      EpsilonSORSF,
                      MaxIterOuter,
                      MaxIterInner,
                      MaxIterSORSF,
                      OmegaSORSF,
                      AlphaSF,
                      Gamma,
                      Lambda,
                      Mu,
                      1);
#if 0
#warning "double SF"
            SceneFlow(logfile,
                      PyramidEta,
                      PyramidLevels,
                      Il0_pyramid,
                      0,
                      Ilt_pyramid,
                      Irt_pyramid,
                      level,
                      U_start,
                      V_start,
                      D0_start,
                      Dt_start,
                      level,
                      U_start,
                      V_start,
                      0,
                      Dt_start,
                      h,
                      EpsilonOuter,
                      EpsilonInner,
                      EpsilonSORSF,
                      MaxIterOuter,
                      MaxIterInner,
                      MaxIterSORSF,
                      OmegaSORSF,
                      AlphaST,
                      Gamma,
                      Lambda,
                      Mu,
                      1);
#endif
            
            if (SF_Steps == 2) {
                if (logfile) {
                    fprintf(logfile, "** Scene Flow :  U,V,D,D' Optimization\n");
                    fflush(logfile);
                }

	
                SceneFlow(logfile,
                          PyramidEta,
                          PyramidLevels,
                          Il0_pyramid,
                          Ir0_pyramid,
                          Ilt_pyramid,
                          Irt_pyramid,
                          level,
                          U_start,
                          V_start,
                          D0_start,
                          Dt_start,
                          level,
                          U_start,
                          V_start,
                          D0_start,
                          Dt_start,
                          h,
                          EpsilonOuter,
                          EpsilonInner,
                          EpsilonSORSF,
                          MaxIterOuter,
                          MaxIterInner,
                          MaxIterSORSF,
                          OmegaSORSF,
                          AlphaSF,
                          Gamma,
                          Lambda,
                          Mu,
                          0);
            }

            // before going to the next level, scale the results if necessary
            if (level == PyramidLevelFinal) {
                assert(U_end == U_start);
                assert(V_end == V_start);
                assert(D0_end == D0_start);
                assert(Dt_end == Dt_start);
            }
            else {
                size_sf_start = cvGetSize(Il0_pyramid[level-1]);
                CvSize size_sf_next = cvGetSize(Il0_pyramid[level-2]);
                if (logfile) {
                    fprintf(logfile, "** Upscaling results from resolution: %dx%d (level %d)\n",
                            size_sf_start.width, size_sf_start.height, PyramidLevelSF);
                    fprintf(logfile, "** To resolution: %dx%d (level %d)\n",
                            size_sf_next.width, size_sf_next.height, PyramidLevelFinal);
                    fflush(logfile);
                }
                U_end = cvCreateMat(size_sf_next.height, size_sf_next.width, CV_32FC1);
                V_end = cvCreateMat(size_sf_next.height, size_sf_next.width, CV_32FC1);
                D0_end = cvCreateMat(size_sf_next.height, size_sf_next.width, CV_32FC1);
                Dt_end = cvCreateMat(size_sf_next.height, size_sf_next.width, CV_32FC1);

                ScaleChange (logfile, 
                             PyramidEta, 
                             PyramidLevels, 
                             Il0_pyramid, 
                             Ir0_pyramid,
                             Ilt_pyramid, 
                             Irt_pyramid, 
                             level,
                             U_start,
                             V_start,
                             D0_start,
                             Dt_start,
                             level-1,
                             U_end,
                             V_end,
                             D0_end,
                             Dt_end);

                cvReleaseMat(&U_start);
                cvReleaseMat(&V_start);
                cvReleaseMat(&D0_start);
                cvReleaseMat(&Dt_start);
                U_start = U_end;
                V_start = V_end;
                D0_start = D0_end;
                Dt_start = Dt_end;
            }
        }
    }

    if (logfile) {
        fprintf(logfile, "** Saving results\n");
        fflush(logfile);
    }
    if (SF_U_Final_FileName)
        SaveResults(SF_U_Final_FileName, U_end);
	     
    if (logfile) {
        fprintf(logfile, "** Saving results : V\n");
        fflush(logfile);
    }
    if (SF_V_Final_FileName)
        SaveResults(SF_V_Final_FileName, V_end);

    if (logfile) {
        fprintf(logfile, "** Saving results : D0\n");
        fflush(logfile);
    }
    if (SF_D0_Final_FileName)
        SaveResults(SF_D0_Final_FileName, D0_end);

    if (SF_D0_Final_Middlebury_FileName) {
        CvMat *img = cvCreateMat(N_y, N_x, CV_8UC1);
        CvMat *D0_final = cvCreateMat(N_y, N_x, CV_32FC1);
        cvResize(D0_end, D0_final, CV_INTER_AREA);
        cvConvertScale(D0_final, img, (double)N_x / (size_final.width * GT_D0_Scale));
        cvSaveImage(SF_D0_Final_Middlebury_FileName, img);
        cvReleaseMat(&D0_final);
        cvReleaseMat(&img);
    }

    if (logfile) {
        fprintf(logfile, "** Saving results : Dt\n");
        fflush(logfile);
    }
    if (SF_Dt_Final_FileName)
        SaveResults(SF_Dt_Final_FileName, Dt_end);
	
    //Optical Flow statistics
    U_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(U_end, U_total, CV_INTER_AREA);
    cvConvertScale(U_total, U_total, (double)N_x / size_final.width);
    V_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(V_end, V_total, CV_INTER_AREA);
    cvConvertScale(V_total, V_total, (double)N_y / size_final.height);

    CvMat* AngularErrorSceneFlow = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat* AngularErrorSceneFlowOcc = cvCreateMat(N_y, N_x, CV_32FC1);
    cvSetZero(AngularErrorSceneFlow);
    cvSetZero(AngularErrorSceneFlowOcc);
    
    if(GT_Special == 1 && True_U && True_V){             // Ball
        AngularErrorGenerationFracture(SF_FlowAngularError_FileName, SF_FlowAngularErrorOcclusions_FileName, U_total, V_total, True_U, True_V, AngularErrorSceneFlowOcc, AngularErrorSceneFlow, N_x, N_y, &AAE_aloneOcc, &SD_aloneOcc, &AAE_alone, &SD_alone, &RMS_Flow, &RMS_Flow_Occ, 1., 1., 1.);
	
    }
    else if (True_U && True_V) {
        OF_Statistics(U_total, V_total, True_U, True_V, &AAE_alone, &SD_alone, &RMS_Flow, N_x, N_y, 0);
        if(GT_OcclusionsDiscontinuities){
	    //with disc and no occlusions
	    OF_Statistics(U_total, V_total, True_U, True_V, &AAE_aloneOcc, &SD_aloneOcc, &RMS_Flow_Occ, N_x, N_y, 0, GT_OcclusionsDiscontinuities);
            //without disc-occlusions
	    OF_Statistics(U_total, V_total, True_U, True_V, &AAE_aloneOccDisc, &SD_aloneOccDisc, &RMS_Flow_OccDisc, N_x, N_y, 1, GT_OcclusionsDiscontinuities);
	}
    }

    //Statistics for the disparities
    CvMat *D0_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(D0_end, D0_total, CV_INTER_AREA);
    cvConvertScale(D0_total, D0_total, (double)N_x / size_final.width);
    CvMat *Dt_total = cvCreateMat(N_y, N_x, CV_32FC1);
    cvResize(Dt_end, Dt_total, CV_INTER_AREA);
    cvConvertScale(Dt_total, Dt_total, (double)N_x / size_final.width);
   
    //double AAE_aloneOcc_tmp, SD_aloneOcc_tmp, AAE_alone_tmp, SD_alone_tmp;
    
    if(GT_Special == 1 && True_D0){
	//AngularErrorGenerationFracture(SF_FlowAngularError_FileName, SF_FlowAngularErrorOcclusions_FileName, U_total, V_total, True_U, True_V, AngularErrorSceneFlowOcc, AngularErrorSceneFlow, N_x, N_y, &AAE_aloneOcc_tmp, &SD_aloneOcc_tmp, &AAE_alone_tmp, &SD_alone_tmp, 1., 0., 0., 1, True_D0, D0_total, &RMS_Disp0);
	RMS_Disp0 = cvNorm(True_D0, D0_total)/sqrt((double)N_x*N_y);
    }
    else if (True_D0) {
        //if(GT_Occlusions){
        //    Stereo_Statistics(D0_total, True_D0, &RMS_Disp0_Occ, N_x,  N_y,  0 , GT_Occlusions);
        //}
        Stereo_Statistics(D0_total, True_D0, &RMS_Disp0, N_x,  N_y,  0);
        if(GT_OcclusionsDiscontinuities){
	    //with disc without occlusions
            Stereo_Statistics(D0_total, True_D0, &RMS_Disp0_Occ, N_x, N_y,  0 , GT_OcclusionsDiscontinuities);
	    //without disc-occlusions
            Stereo_Statistics(D0_total, True_D0, &RMS_Disp0_OccDisc, N_x, N_y,  1 , GT_OcclusionsDiscontinuities);
        }
    }
  
    if(GT_Special == 1 && True_Dt){
	RMS_Dispt = cvNorm(True_Dt, Dt_total)/sqrt((double)N_x*N_y);
    //AngularErrorGenerationFracture(SF_FlowAngularError_FileName, SF_FlowAngularErrorOcclusions_FileName, U_total, V_total, True_U, True_V, AngularErrorSceneFlowOcc, AngularErrorSceneFlow, N_x, N_y, &AAE_aloneOcc_tmp, &SD_aloneOcc_tmp, &AAE_alone_tmp, &SD_alone_tmp, 1., 1., 1., 1, True_Dt, Dt_total, &RMS_Dispt);
    }
    else if (True_Dt) {
        //if(GT_Occlusions){
        //    Stereo_Statistics(Dt_total, True_Dt, &RMS_Disp0_Occ, N_x,  N_y,  0 , GT_Occlusions);
        //}
        Stereo_Statistics(Dt_total, True_Dt, &RMS_Dispt, N_x,  N_y,  0);
        if(GT_OcclusionsDiscontinuities){
	    //with disc without occlusions
            Stereo_Statistics(Dt_total, True_Dt, &RMS_Dispt_Occ, N_x, N_y,  0 , GT_OcclusionsDiscontinuities);
	    //without disc-occ
            Stereo_Statistics(Dt_total, True_Dt, &RMS_Dispt_OccDisc, N_x, N_y,  1 , GT_OcclusionsDiscontinuities);
        }
    }

  
    cvReleaseMat(&AngularErrorSceneFlow);
    cvReleaseMat(&AngularErrorSceneFlowOcc);

    if (logfile) {
        fprintf(logfile, "** Statistics on results\n");
	if (True_D0) {
            fprintf(logfile, "Post optimization D0 RMS with occlusion = %g\n", RMS_Disp0);
            fprintf(logfile, "Post optimization D0 RMS without occlusion = %g\n", RMS_Disp0_Occ);
            fprintf(logfile, "Post optimization D0 RMS without occlusion neither discontinuities= %g\n", RMS_Disp0_OccDisc);
        }
        if (True_Dt) {
            fprintf(logfile, "Post optimization Dt RMS with occlusion = %g\n", RMS_Dispt);
            fprintf(logfile, "Post optimization Dt RMS without occlusion = %g\n", RMS_Dispt_Occ);
            fprintf(logfile, "Post optimization Dt RMS without occlusion neither discontinuities= %g\n", RMS_Dispt_OccDisc);
        }
        if (True_U && True_V) {
            fprintf(logfile, "Angular error mean with occlusion = %g\n", AAE_alone);
            fprintf(logfile, "Angular error sdev with occlusion = %g\n", SD_alone);
            fprintf(logfile, "Angular error mean without occlusion = %g\n", AAE_aloneOcc);
            fprintf(logfile, "Angular error sdev without occlusion = %g\n", SD_aloneOcc);
            if(GT_OcclusionsDiscontinuities){
                fprintf(logfile, "Angular error mean without occlusion neither discontinuities= %g\n", AAE_aloneOccDisc);
                fprintf(logfile, "Angular error sdev without occlusion neither discontinuities = %g\n", SD_aloneOccDisc);
            }
            fprintf(logfile, "Flow RMS with occlusion = %g\n", RMS_Flow);
            fprintf(logfile, "Flow RMS without occlusion = %g\n", RMS_Flow_Occ);
            fprintf(logfile, "Flow RMS without occlusion neither discontinuities= %g\n", RMS_Flow_OccDisc);
        }
        fprintf(logfile, "********** FINISHED ************\n");
        fclose(logfile);
        logfile = 0;
    }

    cvReleaseMat(&U_end);
    cvReleaseMat(&V_end);
    cvReleaseMat(&D0_end);
    cvReleaseMat(&Dt_end);

    cvReleaseMat(&U_total);
    cvReleaseMat(&V_total);
    cvReleaseMat(&D0_total);
    cvReleaseMat(&Dt_total);
    
    if (True_U)
        cvReleaseMat(&True_U);
    if (True_V)
        cvReleaseMat(&True_V);
    if (True_D0)
        cvReleaseMat(&True_D0);
    if (True_Dt)
        cvReleaseMat(&True_Dt);

    if (Il0)
        cvReleaseMat(&Il0);
    if (Il0_pyramid) {
        for(level = 0; level < PyramidLevels; level++)
            cvReleaseMat(&Il0_pyramid[level]);
        delete [] Il0_pyramid;
    }
    if (Ir0)
        cvReleaseMat(&Ir0);
    if (Ir0_pyramid) {
        for(level = 0; level < PyramidLevels; level++)
            cvReleaseMat(&Ir0_pyramid[level]);
        delete [] Ir0_pyramid;
    }
    if (Ilt)
        cvReleaseMat(&Ilt);
    if (Ilt_pyramid) {
        for(level = 0; level < PyramidLevels; level++)
            cvReleaseMat(&Ilt_pyramid[level]);
        delete [] Ilt_pyramid;
    }
    if (Irt)
        cvReleaseMat(&Irt);        
    if (Irt_pyramid) {
        for(level = 0; level < PyramidLevels; level++)
            cvReleaseMat(&Irt_pyramid[level]);
        delete [] Irt_pyramid;
    }

    exit(0);

}



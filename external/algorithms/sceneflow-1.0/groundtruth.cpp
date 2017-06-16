/* 
 * generate a disp map for the synthetic sphere
 * 
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_STRING_H
#include <string.h>
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <math.h>

#define SIZE 512
#define RADIUS 0.45				/* synthetic sphere radius*/
#ifndef M_PI
# define M_PI               3.14159265358979323846
#endif

/* angle_X, angle_Y and angle_Z */
void flow_sphere(int x_pixels,          /* IN PIXELS */
                 int y_pixels,
                 int image_width,
                 double sphere_radius, /* from 0 to 0.5 */
                 double angle_X,        /* IN DEGREES */
                 double angle_Y,
                 double angle_Z,
                 double *flow_x,        /* IN PIXELS */
                 double *flow_y,        /* IN PIXELS */
                 double *disp)          /* IN PIXELS */
{
    double x, y;
    double Xl0,Yl0,Zl0;
    double Xw,Yw,Zw;
    double Xl,Yl,Zl;
    double Xr,Yr,Zr;
    double X,Y,Z;
#define VIEWPOINT_X 0.5
#define VIEWPOINT_Z 2
    const double sina = VIEWPOINT_X/sqrt(VIEWPOINT_Z*VIEWPOINT_Z+VIEWPOINT_X*VIEWPOINT_X);
    const double cosa = VIEWPOINT_Z/sqrt(VIEWPOINT_Z*VIEWPOINT_Z+VIEWPOINT_X*VIEWPOINT_X);
    
    /* normalize image coordinates */
    x = (x_pixels-image_width/2+0.5)/(double)(image_width); /* x positive to the right */
    y = -(y_pixels-image_width/2+0.5)/(double)(image_width); /* y positive to the top */

    /* convert angles to radians */
    angle_X *= M_PI/180.;
    angle_Y *= M_PI/180.;
    angle_Z *= M_PI/180.;
    
    Xl0 = x/sphere_radius;
    Yl0 = y/sphere_radius;
    Zl0 = Xl0*Xl0+Yl0*Yl0 - 1;                   /* Z is negative towards the viewer */
    if (Zl0 > 0) {                 /* out of the sphere */
        *flow_x = 0;
        *flow_y = 0;
        *disp = 0;
        return;
    }
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

    /* convert from world coordinates to right image view: rotate around y axis */
    Xr = Zw*sina+Xw*cosa;
    Yr = Yw;
    Zr = Zw*cosa-Xw*sina;

    if (Zl > 0) {				/* undefined (flow occlusion) */
        *flow_x = 0;
        *flow_y = 0;
        *disp = 0;
        return;
    }
    
    *flow_x = (Xl-Xl0) * sphere_radius * image_width;
    *flow_y = -(Yl-Yl0) * sphere_radius * image_width;
    /* compute disparity */
    if (Zr > 0) {                       /* disparity occlusion */
        *disp = 0;
    }
    else
        *disp = (Xr - Xl) * sphere_radius * image_width;

}


float disp_sphere(double x, double y, double r)
{
	double cosa = 2/sqrt(4.25);
	double sina = 0.5/sqrt(4.25);
	double sqrty;

	if ((y<-r) || (y>r))
		return 0.0;

	sqrty = sqrt(1-y*y/(r*r));
	
	if ((x<-sqrty*r) || (x>sqrty*r))
		return 0.0;
	
	if ( x<-cosa*sqrty*r )
		return 1.0;				/* undefined (occlusion) */

	return (float)(-2*sina*(sina*x+cosa*sqrt(r*r-y*y-x*x))*SIZE);

}

float dispx_sphere(double x, double y, double r)
{
	double cosa = 2/sqrt(4.25);
	double sina = 0.5/sqrt(4.25);
	double sqrty;

	if ((y<-r) || (y>r))
		return 0.0;

	sqrty = sqrt(1-y*y/(r*r));
	
	if ((x<=-sqrty*r) || (x>=sqrty*r))
		return 0.0;
	
	if ( x<-cosa*sqrty*r )
		return 0.0;				/* undefined (occlusion) */

	return (float)(-2*sina*(sina-cosa*x/sqrt(r*r-y*y-x*x)));

}

float dispy_sphere(double x, double y, double r)
{
	double cosa = 2/sqrt(4.25);
	double sina = 0.5/sqrt(4.25);
	double sqrty;
	double sqrtxy;

	if ((y<=-r) || (y>=r))
		return 0.0;

	sqrty = sqrt(1-y*y/(r*r));
	
	if ((x<=-sqrty*r) || (x>=sqrty*r))
		return 0.0;
	
	if ( x<-cosa*sqrty*r )
		return 0.0;				/* undefined (occlusion) */

	sqrtxy = sqrt(r*r-x*x-y*y);

	return (float)(-2*sina*cosa*y/sqrt(r*r-y*y-x*x));
}

float flowx_sphere(double x, double y, double r, double t)
{
	double sqrty;
        double t0;
        
	if ((y<=-r) || (y>=r))
		return 0.0;

	sqrty = sqrt(1-y*y/(r*r));
	
	if ((x<-sqrty*r) || (x>sqrty*r))
		return 0.0;
	t0 = acos(x/(sqrty*r));
        if ( (t0+t) < 0 || (t0+t) > M_PI)
            return 0;				/* undefined (occlusion) */
        
	return (float)(sqrty*r*(cos(t0+t)-x/(sqrty*r))*SIZE);
}

float flowy_sphere(double x, double y, double r, double t)
{
	(void)x;(void)y; (void)r; (void)t; 
    return 0.0;
}

/* reconstruction a partir de deux vues orthographiques */
void recons(double x, double y, double d,
			double *X, double *Y, double *Z)
{
	double cosa = 2/sqrt(4.25);
	double sina = 0.5/sqrt(4.25);

	*Y = -(y-SIZE/2+1)/(double)(SIZE);
	*X = (x-SIZE/2+d/2)/(double)(SIZE*cosa);
	*Z = d/(2*SIZE*sina);
}

/* projection */
void proj(double X, double Y, double Z,
		  double *x, double *y, double *d)			
{
	double cosa = 2/sqrt(4.25);
	double sina = 0.5/sqrt(4.25);

	*d = Z*(2*SIZE*sina);
	*x = X*SIZE*cosa + SIZE/2 - *d/2;
	*y = -Y*SIZE +SIZE/2 -1;
}

// /* compute disparity ground truth */
// (int argc, char **argv)
// {
// 	float *disp;
// 	int x,y;
// 	double xf, yf;
// 	double X, Y, Z;

// 	disp = (float*)malloc(SIZE*SIZE*sizeof(float));
// 	for (y=0; y<SIZE; y++) {
// 		for (x=0; x<SIZE; x++) {
// 			/* coordonnees dans le repere image de povray du CENTRE du
// 			   pixel */
// 			/* n'a ete teste que pour SIZE multiple de 2 */
// 			xf = (x-SIZE/2)/(double)(SIZE);
// 			yf = -(y-SIZE/2+1)/(double)(SIZE);
// 			disp[y*SIZE+x] = disp_sphere(xf,yf,RADIUS);
// 		}
// 	}
// 	disp[0] = 1;
// 	image1Save("-", (IMAGE1)disp, SIZE, SIZE,
// 			   IMAGE1_FLOAT|IMAGE1_INRIMAGE_FORMAT);
// 	free(disp);
// 	exit(0);
// }

// /* compute optical flow ground truth */
// int
// main1(int argc, char **argv)
// {
// 	float *flowx;
// 	int x,y;
// 	double xf, yf;
// 	double X, Y, Z;
//         double t;
        
//         if (argc != 2) {
//             fprintf(stderr,"Usage: %s <angle_degrees>\n", argv[0]);
//             exit(1);
//         }
//         t = atof(argv[1])*M_PI/180.;
// 	flowx = (float*)malloc(SIZE*SIZE*sizeof(float));
// 	for (y=0; y<SIZE; y++) {
// 		for (x=0; x<SIZE; x++) {
// 			/* coordonnees dans le repere image de povray du CENTRE du
// 			   pixel */
// 			/* n'a ete teste que pour SIZE multiple de 2 */
// 			xf = (x-SIZE/2)/(double)(SIZE);
// 			yf = -(y-SIZE/2+1)/(double)(SIZE);
// 			flowx[y*SIZE+x] = flowx_sphere(xf,yf,RADIUS, t);
// 		}
// 	}
// 	image1Save("-", (IMAGE1)flowx, SIZE, SIZE,
// 			   IMAGE1_FLOAT|IMAGE1_INRIMAGE_FORMAT);
// 	free(flowx);
// 	exit(0);
// }

// int
// main2(int argc, char **argv)
// {
// 	float *disp, *error;
// 	int x,y;
// 	double xf, yf, d;
// 	double X, Y, Z;
// 	int dimx, dimy, type;
// 	double dradius;

// 	disp = (float*)image1Load("-", &dimx, &dimy, &type);
// 	assert(dimx==SIZE);
// 	assert(dimy==SIZE);
// 	assert(type&IMAGE1_FLOAT);
// 	error = (float*)malloc(SIZE*SIZE*sizeof(float));

// 	for (y=0; y<SIZE; y++) {
// 		for (x=0; x<SIZE; x++) {
// #if 0
// 			/* disparity difference */
// 			xf = (x-SIZE/2)/(double)(SIZE);
// 			yf = -(y-SIZE/2+1)/(double)(SIZE);
// 			if (disp[y*SIZE+x] == disp[0]) {
// 				error[y*SIZE+x] = 0.0;
// 			} else {
// 				error[y*SIZE+x] = disp[y*SIZE+x] - disp_sphere(xf,yf,RADIUS);
// 			}
// #else
// 			/* disparity difference with the closest point */
// 			if (disp[y*SIZE+x] == disp[0]) {
// 				error[y*SIZE+x] = 0.0;
// 			} else {
// 				recons(x,y,disp[y*SIZE+x], &X, &Y, &Z);
// 				dradius = sqrt(X*X+Y*Y+Z*Z) - RADIUS;
// 				if (fabs(dradius) > fabs(Z)) {
// 					/* plus proche du plan */
// 					error[y*SIZE+x] = disp[y*SIZE+x];
// 				} else {
// 					/* plus proche de la sphere */
// 					dradius = RADIUS/sqrt(X*X+Y*Y+Z*Z);
// 					/* point le plus proche, sur la sphere */
// 					X*=dradius;
// 					Y*=dradius;
// 					Z*=dradius;
// 					proj(X,Y,Z,&xf, &yf, &d);
// 					error[y*SIZE+x] = disp[y*SIZE+x] - d;
// 				}
// 			}
// #endif
// 			if (error[y*SIZE+x]<-2)
// 				error[y*SIZE+x]=-2;
// 			if (error[y*SIZE+x]>2)
// 				error[y*SIZE+x]=2;
// 		}
// 	}
// 	image1Save("-", (IMAGE1)error, SIZE, SIZE,
// 			   IMAGE1_FLOAT|IMAGE1_INRIMAGE_FORMAT);
// 	free(disp);
// 	exit(0);
// }

// int
// main(int argc, char **argv)
// {
// 	float *disp = NULL, *dispx = NULL, *dispy = NULL;
//     float *dispxx = NULL, *dispxy = NULL, *dispyy = NULL;
// 	int x,y;
// 	double xf, yf, d;
// 	double X, Y, Z;
//     double xsampling = 1.0;
// 	int dimx, dimy, type;
// 	double dradius;
// 	int derivs = 0;
//     FILE *errorx_file =NULL, *errory_file =NULL, *errorr_file =NULL,
//         *error_file =NULL;
//     double totalerror = 0.0;
//     long int totalnb = 0.0;
//     extern char *optarg;
//     extern int optind;
//     int errflag = 0;
//     char *scorename = NULL;
//     float *score = NULL;
//     int c;
//     char *commandname = argv[0];
//     int xsize;

//     while((c = getopt(argc, argv, "x:S:")) != -1) {
//         switch (c) {
//         case 'S':
//             scorename = optarg;
//             break;
//         case 'x':
//             xsampling = atof(optarg);
//             break;
//         case '?':
//             errflag++;
//             break;
//         }
//     }
//     argv = &(argv[optind-1]);
//     argc -= (optind-1);

//     switch(argc) {
//     case 2:
//         derivs = 0;
//         break;
//     case 4:
//         derivs = 1;
//         break;
//     case 7:
//         derivs = 2;
//         break;
//     default:
//         errflag++;
//     }
//     if (errflag) {
// 		fprintf(stderr,"Usage: %s [-x x_sampling] [-S score] disp [dispx dispy [dispxx dispxy dispyy]]\n", commandname);
// 		exit(1);
//     }
//     xsize = (int)(SIZE*xsampling);

//     if (scorename != NULL) {
//         score = (float*)image1Load(scorename, &dimx, &dimy, &type);
//         assert(dimx==xsize);
//         assert(dimy==SIZE);
//         assert(type&IMAGE1_FLOAT);
//     }
// 	disp = (float*)image1Load(argv[1], &dimx, &dimy, &type);
// 	assert(dimx==xsize);
// 	assert(dimy==SIZE);
// 	assert(type&IMAGE1_FLOAT);
//     error_file = fopen("error","w");
// 	if (derivs >=1 ) {
// 		dispx = (float*)image1Load(argv[2], &dimx, &dimy, &type);
// 		assert(dimx==xsize);
// 		assert(dimy==SIZE);
// 		assert(type&IMAGE1_FLOAT);
// 		dispy = (float*)image1Load(argv[3], &dimx, &dimy, &type);
// 		assert(dimx==xsize);
// 		assert(dimy==SIZE);
// 		assert(type&IMAGE1_FLOAT);
//         errorx_file = fopen("errorx","w");
//         errory_file = fopen("errory","w");
//         errorr_file = fopen("errorr","w");
// 	}
//     if (derivs >=2) {
// 		dispxx = (float*)image1Load(argv[4], &dimx, &dimy, &type);
// 		assert(dimx==xsize);
// 		assert(dimy==SIZE);
// 		assert(type&IMAGE1_FLOAT);
// 		dispxy = (float*)image1Load(argv[5], &dimx, &dimy, &type);
// 		assert(dimx==xsize);
// 		assert(dimy==SIZE);
// 		assert(type&IMAGE1_FLOAT);
//  		dispyy = (float*)image1Load(argv[6], &dimx, &dimy, &type);
// 		assert(dimx==xsize);
// 		assert(dimy==SIZE);
// 		assert(type&IMAGE1_FLOAT);
//     }
// 	for (y=0; y<SIZE; y++) {
//         int linenb = 0;
//         double lineerror = 0.0;
// 		for (x=0; x<xsize; x++) {
// 			/* disparity difference with the closest point */
// 			if (disp[y*xsize+x] != disp[0]) {
// 				recons(x/xsampling,y,disp[y*xsize+x]/xsampling, &X, &Y, &Z);
// 				dradius = sqrt(X*X+Y*Y+Z*Z) - RADIUS;
// 				if (fabs(dradius) < fabs(Z)) {
//                     double dx, dy;
//                     double errorx, errory, error;
// 					/* plus proche de la sphere */
// 					dradius = RADIUS/sqrt(X*X+Y*Y+Z*Z);
// 					/* point le plus proche, sur la sphere */
// 					X*=dradius;
// 					Y*=dradius;
// 					Z*=dradius;
// 					proj(X,Y,Z,&xf, &yf, &d);
//                     xf = (xf-SIZE/2)/(double)(SIZE);
//                     yf = -(yf-SIZE/2+1)/(double)(SIZE);
//                     error = disp[y*xsize+x]/xsampling - d;
//                     dx = dispx_sphere(xf,yf,RADIUS);
//                     dy = dispy_sphere(xf,yf,RADIUS);
//                     fprintf(error_file,"%g %g",sqrt(dx*dx+dy*dy),error);
//                     if (score != NULL) {
//                         fprintf(error_file," %g",score[y*xsize+x]);
//                     }
//                     fputc('\n',error_file);
//                     lineerror += fabs(error);
//                     linenb++;
//                     if (derivs >=1 ) {
//                         errorx = dispx[y*xsize+x]*xsampling - dx;
//                         if (dispx[y*xsize+x] != 0. && dx != 0.) {
//                             fprintf(errorx_file,"%g %g\n",sqrt(dx*dx+dy*dy),errorx);
//                         }
//                         errory = dispy[y*xsize+x] - dy;
//                         if (dispy[y*xsize+x] != 0. && dy != 0.) {
//                             fprintf(errory_file,"%g %g\n",sqrt(dx*dx+dy*dy),errory);
//                         }
//                         if (dispy[y*xsize+x] != 0. && dy != 0.
//                             && dispx[y*xsize+x] != 0. && dx != 0.) {
//                             fprintf(errorr_file,"%g %g\n",sqrt(dx*dx+dy*dy),sqrt(errorx*errorx+errory*errory));
//                         }
//                     }
// 				}
// 			}
// 		}
//         totalerror += lineerror;
//         totalnb += linenb;
// 	}
// 	if (disp != NULL)
//         free(disp);
// 	if (dispx != NULL)
//         free(dispx);
// 	if (dispy != NULL)
//         free(dispy);
// 	if (dispxx != NULL)
//         free(dispxx);
// 	if (dispxy != NULL)
//         free(dispxy);
// 	if (dispyy != NULL)
//         free(dispyy);
//     fclose(error_file);
//     if (derivs >=1 ) {
//         fclose(errorx_file);
//         fclose(errory_file);
//         fclose(errorr_file);
//     }
//     printf("%ld %g\n",totalnb,totalerror/totalnb);
// 	exit(0);
// }

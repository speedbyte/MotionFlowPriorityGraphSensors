#include <math.h>
#include <assert.h>
#ifdef WIN32
#include <stdlib.h>
#define snprintf _snprintf
#endif
#include "Resolution.h"
#include "iniconfig.h"
#include "resize.h"

#define CONST_COLOR 256
#define MAX_HUE 150
//#define DIVERGENCE_8 0
//#define DISABLE_PIXEL_DISACTIVATION 0
#define GMRES 1
#define GMRES_KMAX 200
//#define OCCLUDED_PIXELS_LAPLACIAN 0
#define DISACTIVATION_THRESHOLD 1.5
//#define VERBOSE_ENERGY 1
#define WARP_DETERMINANT_THRESHOLD 0.5 // should be >= 0. and < 1.

#define EPSILON_PSI_SMOOTH 0.001               // epsilon for the psi function
#define EPSILON_PSI_DATA 1.               // epsilon for the psi function

#define MAX_INCR 0.5                    // maximum increment between two iterates in pixels

#define DEBUG_PIXEL 0
#define DEBUG_PIXEL_X 331
#define DEBUG_PIXEL_Y 87
#define DEBUG_PIXEL_COMPONENT 1

// Pixels that go beyond this border (in pixels) are disactivated. Must be at
// least 3, since gradient estimation for warped images is
// wrong 2 pixels from the border, and warping is wrong 1 pixel from the border
// but 4 (more than 2*sqrt(2)+1) is safer in
// case the transformed image border is at 45deg
#define DISACTIVATE_BORDER 3.9

#ifndef M_PI
# define M_PI               3.14159265358979323846
#endif

static void debug_me() {
}

static inline double square(double x) {
    return x*x;
}

static inline double fmax(double a, double b)
{
	return (a > b ? a : b);
}


CV_INLINE  unsigned char cvmGet8u( const CvMat* mat, int row, int col )
{
    assert( (unsigned)row < (unsigned)mat->rows &&
            (unsigned)col < (unsigned)mat->cols );
    assert( CV_MAT_TYPE(mat->type) == CV_8UC1 );
    return ((unsigned char*)(mat->data.ptr + (size_t)mat->step*row))[col];
}

CV_INLINE  static void  cvmSet8u( CvMat* mat, int row, int col, unsigned char value )
{
    assert( (unsigned)row < (unsigned)mat->rows &&
            (unsigned)col < (unsigned)mat->cols );
    assert( CV_MAT_TYPE(mat->type) == CV_8UC1 );

    ((unsigned char*)(mat->data.ptr + (size_t)mat->step*row))[col] = value;
}

// used to mark border pixels as disactivated (although they are not used anyway)
CV_INLINE  static void  cvmSet8u_borders( CvMat* mat, unsigned char value )
{
    int rows  = mat->rows;
    int cols = mat->cols;
    int i, j;

    // top border
    for(j = 0; j < cols; j++)
        cvmSet8u(mat,0,j,value);
    // bottom border
    for(j = 0; j < cols; j++)
        cvmSet8u(mat,rows-1,j,value);
    // left border
    for(i = 1; i < rows-1; i++)
        cvmSet8u(mat,i,0,value);
    // right border
    for(i = 1; i < rows-1; i++)
        cvmSet8u(mat,i,cols-1,value);
}


static inline double sign(double a)
{
    return (a > 0 ? 1.0 : -1.0);
}

static inline float sign(float a)
{
    return (a > 0.0 ? 1.0f : -1.0f);
}




/***************************************************Horizontal linear Interpolation**************************************************/
static inline void Interpolate_X(const CvMat* data, //data to be interpolated
				 const int x, //pixel x
				 const int y, //pixel y
				 const double alpha, //Interpolation coefficient
				 double *Interpolate, //Result
				 const int Nx){
    if(x < Nx - 1)
	*Interpolate = alpha * cvmGet(data, y, x + 1) + (1 - alpha) * cvmGet(data, y, x);
    else
	*Interpolate = alpha * cvmGet(data, y, x) + (1 - alpha) * cvmGet(data, y, x - 1); 
}

static inline void SmoothImage(CvArr *img) {
    cvSmooth(img,img, CV_GAUSSIAN, 3,3);
}

static inline void SmoothData(CvArr *img) {
#if 1
#ifndef _MSC_VER
#warning "not smoothing data between scale changes"
#endif
	(void)img;
#else
    cvSmooth(img,img, CV_GAUSSIAN, 3,3);
#endif
}

static inline double& Crop(double &val, double threshold) {
#if 1
    if (val > threshold)
        val = threshold;
    else if (val < -threshold)
        val = -threshold;
#endif
    return val;
}

static inline void CropArr(CvArr *arr, double threshold) {
#if 1
    cvMinS(arr, threshold, arr);
    cvMaxS(arr,-threshold, arr);
#endif
}


/***************************************************Initial Disparity d' estimation using left and right optical flows****************/
void DtEstimation(const CvMat *U, //Left optical flow
		  const CvMat *Ur, //Right optical flow
		  const CvMat *D0, //Disparity d at time t
		  CvMat*Dt)
{  
    double alpha;
    double xr0;
    double ur, u, d0;
    int xr0_int;
    int dimX = U->width;
    int dimY = U->height;
    //D' estimation
    for(int yl0 =0; yl0 < dimY; yl0++){
	for(int xl0 = 0; xl0 < dimX; xl0++){
	    //xr0 in the initial right image IR(x+d, yl0, t)
	    d0 =  cvmGet(D0,yl0,xl0);
	    xr0 = xl0 + d0;
	    //if D estimation is not correct, not changing Dt 
	    if(xr0 < 0 || xr0 > dimX)
		cvmSet(Dt,yl0,xl0, d0);
	    else{
		//Horizontal interpolation
		xr0_int = (int)cvFloor(xr0);
		alpha = xr0 - xr0_int;
		//Right flow estimation for the current point :U_r(x+d,yl0)
		Interpolate_X(Ur, xr0_int, yl0, alpha, &ur, dimX);
		//Left flow for the current point : U_l(x,yl0);
		u = cvmGet(U, yl0, xl0);
		//Disparity D' estimation  
		cvmSet(Dt, yl0, xl0, d0 + ur - u);
	    }
	}
    }
    //Results image save
    if (SF_Dt_Init_FileName)
        SaveResults(SF_Dt_Init_FileName, Dt);
}

/**************************************************Scene Flow Estimation*****************/
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
                  CvMat* Dt_end)
{
	(void)Il0_pyramid;
	(void)Ilt_pyramid;
	(void)Ir0_pyramid;
	(void)Irt_pyramid;
	(void)pyramid_levels;
	(void)eta;
	(void)logfile;
    if (U_start) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], U_start));
        assert(U_end);
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], U_end));
    }
    if (V_start) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], V_start));
        assert(V_end);
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], V_end));
    }
    if (D0_start) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], D0_start));
        assert(D0_end);
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], D0_end));
    }
    if (Dt_start) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], Dt_start));
        assert(Dt_end);
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], Dt_end));
    }
    if (pyramid_level_end == pyramid_level_start) {
        if (U_start)
            cvCopy(U_start, U_end);
        if (V_start)
            cvCopy(V_start, V_end);
        if (D0_start)
            cvCopy(D0_start, D0_end);
        if (Dt_start)
            cvCopy(Dt_start, Dt_end);
    }
   
    int interpolation = CV_INTER_LINEAR;
    double scale;
    if (pyramid_level_end > pyramid_level_start)
      interpolation = CV_INTER_AREA;  // downsizing is better with CV_INTER_AREA
    if (U_start) {
      cvResize(U_start, U_end, interpolation);
      scale = (double)U_end->width / U_start->width;
      cvConvertScale(U_end, U_end, scale);
    }
    if (V_start) {
      cvResize(V_start, V_end, interpolation);
      scale = (double)V_end->height / V_start->height;
      cvConvertScale(V_end, V_end, scale);
    }
    if (D0_start) {
      cvResize(D0_start, D0_end, interpolation);
      scale = (double)D0_end->width / D0_start->width;
      cvConvertScale(D0_end, D0_end, scale);
    }
   if (Dt_start) {
      cvResize(Dt_start, Dt_end, interpolation);
      scale = (double)Dt_end->width / Dt_start->width;
      cvConvertScale(Dt_end, Dt_end, scale);
    }
    
}

/*************************************** Gaussian density computation *******************/
static inline double NormalDensity(const double sigma, const double x){
    return (1./sqrt(2.*M_PI*fabs(sigma*sigma)))*exp((-1./(2.*sigma*sigma))*x*x);
}

/* lightweight convolution with 3x3 kernel */
static void GradientFast_32f( const CvMat* srcmat, CvMat* dstxmat, CvMat* dstymat )
{
    int  x, y;

    int src_type, dstx_type, dsty_type;
    src_type = CV_MAT_TYPE( srcmat->type );
    dstx_type = CV_MAT_TYPE( dstxmat->type );
    dsty_type = CV_MAT_TYPE( dstymat->type );
    assert(src_type == CV_32FC1 && dstx_type == CV_32FC1 && dsty_type == CV_32FC1);
    // assert(CV_ARE_SIZES_EQ( srcmat, dstxmat ));
//     assert(CV_ARE_SIZES_EQ( srcmat, dstymat ));
    int src_step = srcmat->step;
    int dst_step = dstxmat->step;
    assert(dst_step == dstymat->step);
    float* src = (float*)srcmat->data.ptr;
    float* dstx = (float*)dstxmat->data.ptr;
    float* dsty = (float*)dstymat->data.ptr;

    src_step /= sizeof(src[0]);
    dst_step /= sizeof(dstx[0]);

    for( y = 0; y < srcmat->height; y++, src += src_step, dstx += dst_step, dsty += dst_step)
    {
        float* srcprevline;
        float* srcnextline;
        float muly = 0.5;
        if (y == 0) {
            srcprevline = src;
            muly = 1;
        }
        else {
            srcprevline = src - src_step;
        }
        if (y == srcmat->height - 1) {
            srcnextline = src;
            muly = 1;
        }
        else {
            srcnextline = src + src_step;
        }
        for( x = 0; x < srcmat->width; x++ ) {
            float srcvalprevx;
            float srcvalnextx;
            float mulx = 0.5;
            if (x == 0) {
                srcvalprevx = src[x];
                mulx = 1;
            }
            else {
                srcvalprevx = src[x-1];
            }
            if (x == srcmat->width - 1) {
                srcvalnextx = src[x];
                mulx = 1;
            }
            else {
                srcvalnextx = src[x+1];
            }
            dstx[x] = (srcvalnextx-srcvalprevx)*mulx;
            dsty[x] = (srcnextline[x]-srcprevline[x])*muly;
        }
    }
}

/* lightweight convolution with 3x3 kernel */
static void ZBufMap_32f( const CvMat* src, CvMat* dst, const CvMat* mapx, const CvMat* mapy, float fillval )
{
    int  x, y;

    int src_type, dst_type, mapx_type, mapy_type;
    src_type = CV_MAT_TYPE( src->type );
    dst_type = CV_MAT_TYPE( dst->type );
    mapx_type = CV_MAT_TYPE( mapx->type );
    mapy_type = CV_MAT_TYPE( mapy->type );
    assert(src_type == CV_32FC1 && dst_type == CV_32FC1);
    assert(mapx_type == CV_32FC1 && mapy_type == CV_32FC1);
    assert(CV_ARE_SIZES_EQ( src, dst ));
    assert(CV_ARE_SIZES_EQ( src, mapx ));
    assert(CV_ARE_SIZES_EQ( src, mapy ));

    cvSet(dst, cvScalar(fillval));
    for( y = 0; y < src->height; y++)
    {
        for( x = 0; x < src->width; x++ ) {
            const double dstx = cvmGet(mapx,y,x);
            const double dsty = cvmGet(mapy,y,x);
            const double srcval = cvmGet(src,y,x);
            
            if (dstx < 0 || dstx > (dst->width-1) ||
                dsty < 0 || dsty > (dst->height-1))
                continue;
                
            for(int i=cvFloor(dsty); i <= cvCeil(dsty); i++) {
                for(int j=cvFloor(dstx); j <= cvCeil(dstx); j++) {
                    const double dstval = cvmGet(dst,i,j);
                    if (srcval < dstval)
                        cvmSet(dst,i,j,srcval);
                }
            }
        }
    }
}

/***************************Saving results in false color images********************/
void SaveImage(const char *filename, const CvArr* arr)
{
    int type = cvGetElemType(arr);
    if(type == CV_8UC1 || type == CV_8UC3) {
        cvSaveImage(filename, arr);
    }
    else {
        CvMat *charim = cvCreateMat(cvGetDimSize(arr,0), cvGetDimSize(arr,1), CV_8UC1);
        cvConvert(arr, charim);
        cvSaveImage(filename, charim);
        cvReleaseMat(&charim);
    }
}

/***************************Saving results in false color images********************/
void SaveResults(const char *filename, const CvArr* arr, int addscale, double min, double max)
{  
    double values[MAX_HUE+1];
    int i;
    char chaine[256];
    CvFont police;
    int ydiv = 1;
    CvSize size;

    if (CV_IS_SPARSE_MAT( arr ) && ((CvSparseMat*)arr)->dims == 2) {
        size.height = ((CvSparseMat*)arr)->size[0];
        size.width = ((CvSparseMat*)arr)->size[1];
    }
    else {
        size = cvGetSize(arr);
    }
    
    CvMat *Temp = cvCreateMat(size.height, size.width, CV_8UC1);

    if (min == 0. && max == -1.) {
        cvMinMaxLoc(arr, &min, &max);
    }
    //Data scaling
    if(max <= min) {
	cvSetZero(Temp);
    }
    else {
        double scale = (double)MAX_HUE/(max-min);
        double shift =  -(double)MAX_HUE*min/(max - min);
        
        if (!CV_IS_SPARSE_MAT( arr )) {
            cvConvertScale(arr, Temp, scale, shift);
        }
        else {
            cvSetZero(Temp);
            CvSparseMatIterator mat_iterator;
            const CvSparseMat *array = (CvSparseMat*) arr;
            CvSparseNode* node = cvInitSparseMatIterator( array, &mat_iterator );
            for( ; node != 0; node = cvGetNextSparseNode( &mat_iterator ))
            {
                int* idx = CV_NODE_IDX( array, node ); /* get pointer to the element indices */
                double val;
                switch(array->type) {
                    case CV_8UC1:
                        val = *(unsigned char*)CV_NODE_VAL( array, node );
                        break;
                    case CV_32FC1:
                        val = *(float*)CV_NODE_VAL( array, node );
                        break;
                    case CV_64FC1:
                        val = *(double*)CV_NODE_VAL( array, node );
                        break;
                    default:
                        val = 1.;
                }
                cvSetReal2D(Temp, idx[0], idx[1], val*scale+shift);
            }
        }
    }
  
    for(i = 0; i <= MAX_HUE; i++)
	values[i] = min + i*(max - min)/(double)MAX_HUE;
  
    //We use HSV images 
    IplImage *HSV_arr = cvCreateImage(size, IPL_DEPTH_8U, 3);
    //H channel update
    cvSetImageCOI(HSV_arr, 1);
    cvCopy(Temp, HSV_arr);
    //S and V channels update

    //HSV space use
    cvSet(Temp, cvScalar(255));
    cvSetImageCOI(HSV_arr, 2);
    cvCopy(Temp, HSV_arr);
    cvSetImageCOI(HSV_arr, 3);
    cvCopy(Temp, HSV_arr);
    cvSetImageCOI(HSV_arr, 0);
    cvReleaseMat(&Temp);
 
    //We put a value scale on the images
    if (size.width < 50 || size.height < 32)
        addscale = 0;
    if (size.height < MAX_HUE)
        ydiv = cvCeil((double)MAX_HUE/size.height);
    assert(ydiv > 0);
    if(addscale){
	for(i = 0; i <= MAX_HUE; i++){
	    cvLine(HSV_arr, cvPoint(0,i/ydiv), cvPoint(10,i/ydiv), cvScalar(i,255,255));
	}
    }

    //HSV => RGB
    IplImage *RGB_arr = cvCreateImage(size, IPL_DEPTH_8U, 3);
    cvCvtColor(HSV_arr, RGB_arr, 55);
    cvReleaseImage(&HSV_arr);

    if(addscale){
	cvInitFont(&police, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0.);
	snprintf(chaine, 256,"%g", values[0]);
	cvPutText(RGB_arr, chaine, cvPoint(11,10), &police, cvScalar(0,0,0));
	snprintf(chaine, 256,"%g", values[MAX_HUE/2]);
	cvPutText(RGB_arr, chaine, cvPoint(11,5+MAX_HUE/(2*ydiv)), &police, cvScalar(0,0,0));
	snprintf(chaine, 256,"%g", values[MAX_HUE]);
	cvPutText(RGB_arr, chaine, cvPoint(11,MAX_HUE/ydiv), &police, cvScalar(0,0,0));
    }
  
    //Writing the values scale 
    cvSaveImage(filename, RGB_arr);
    cvReleaseImage(&RGB_arr);
}

/************************************* Psi values estimation ********************/
static inline double PsiData(const double x){
    return sqrt(x + EPSILON_PSI_DATA*EPSILON_PSI_DATA);
}
static inline double PsiSmooth(const double x){
    return sqrt(x + EPSILON_PSI_SMOOTH*EPSILON_PSI_SMOOTH);
}

//**************************************Divergence operator coefficients computation***********************************************************
static inline double DPsiSmooth(const double grad2)
{
    //phi(s^2) = 2sqrt(1+s^2)-2
    return 1./sqrt((EPSILON_PSI_SMOOTH*EPSILON_PSI_SMOOTH + grad2));
}

static inline double DPsiData(const double intensity2)
{
    //phi(s^2) = 2sqrt(1+s^2)-2
    return 1./sqrt((EPSILON_PSI_DATA*EPSILON_PSI_DATA + intensity2));
}

//**************************************Half pixel interpolation for the divergence coefficients in the principal directions *****************
static inline void divergence_coefficients_interpolate(CvMat * coeffs, const int i, const int j, double *b1, double *b2, double *b3, double *b4)
{
    double valcentre = cvmGet(coeffs, i, j);
    //The function is called only on the inside points so no condition over the image side
    *b1 = (valcentre + cvmGet(coeffs, i, j + 1)) / 2;	
    *b2 = (valcentre + cvmGet(coeffs, i, j - 1)) / 2;
    *b3 = (valcentre + cvmGet(coeffs, i + 1, j)) / 2;	
    *b4 = (valcentre + cvmGet(coeffs, i - 1, j)) / 2;
}

//**************************************Half pixel interpolation for the divergence coefficients in the diagonal directions *****************
static inline void divergence_coefficients_interpolate_diag(CvMat * coeffs, const int i, const int j, double *b5, double *b6, double *b7, double *b8)
{
    double valcentre = cvmGet(coeffs, i, j);
    //The function is called only on the inside points so no condition over the image side
    *b5 = (valcentre + cvmGet(coeffs, i+1, j + 1)) / 2;	//en X
    *b6 = (valcentre + cvmGet(coeffs, i+1, j - 1)) / 2;
    *b7 = (valcentre + cvmGet(coeffs, i - 1, j+1)) / 2;	//en Y
    *b8 = (valcentre + cvmGet(coeffs, i - 1, j-1)) / 2;
}


//**************************************Images gradient computation********************************
static inline void Derivative(const CvArr * image,  int dx, int dy, CvArr * derivative_int, CvArr * derivative)
{
    if (derivative_int == 0)
        derivative_int = derivative;
    if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1)) {
        cvSobel(image, derivative_int, dx, dy, CV_SCHARR);
        cvConvertScale(derivative_int, derivative, 1.0/32.0);
    }
    else {
        assert(dx+dy == 2);
        cvSobel(image, derivative_int, dx, dy, 3);
        cvConvertScale(derivative_int, derivative, 1.0/4.0);
    }
}

/*************************************Matrix rescaling for linear system computation ******************/
void MatrixNormalize(CvMat *M, CvMat*B, const int Nbcol,  const int inf, const int sup){
    double line_norm;
    int i,j;
    for(i=inf; i < sup; i++){
        line_norm = 0.;
        for(j=0; j < Nbcol; j++)
            line_norm += square(cvGetReal2D(M, i, j));
        line_norm = sqrt(line_norm);//euclidean norm of each matrix line
        for(j=0; j < Nbcol; j++)
            cvSetReal2D(M,i,j,cvGetReal2D(M, i, j)/line_norm); 
        cvSetReal2D(B,i,0,cvGetReal2D(B, i, 0)/line_norm); 
    
    }
}


/***************************Energy computation for each multiscale level with each part of the energy******************************************/
/***************************Take as input the warped images******************************************************/

void EnergyComputation(const CvMat *Il0, //Left t image
		       const CvMat *Ir0w, //Right t image
		       const CvMat *Iltw, //Left t+1 image
		       const CvMat *Irtw, //Right t+1 image 
		       const CvMat* U    , //Horizontal optical flow
		       const CvMat* V    , //Vertical optical flow
		       const CvMat* Dt       , //disparity t+1
		       const CvMat* D0          , //Disparity t
                       const CvMat *disactivated_pixels_leftflow,
                       const CvMat *disactivated_pixels_rightflow,
                       const CvMat *disactivated_pixels_stereoinit,
                       const CvMat *disactivated_pixels_stereo,
		       const int N_x, //Horizontal pixels number 
		       const int N_y, //Vertical pixels number
		       double *data, //Data term
		       double *smooth, //Smoothing term
		       const double gamma , //Images gradient weight
		       const double lambda, //Smooth parameter for disparity variation
		       const double mu, //Smooth parameter for t disparity 
		       const int doleftflow, //Booleans
		       const int dorightflow, 
		       const int dodisparity, 
		       const int dodisparity0, 
		       double *Efl, //Left optical flow contribution
		       double *Efr, //Right optical flow contribution
		       double *Est, //t+1 stereo contribution
		       double *Es0, //t stereo contribution
		       double *Esflp, 
		       double *Esfrp, 
		       double *Esdp, 
		       double *Esd0p, 
		       double *Eintensityp, 
		       double *Egradientp){

    double Esmooth, Esmoothtmp;
    double El,Er,Elr0, Elr;
    double Es, Esfl, Esfr, Esd, Esd0;
    double Ei, Eg, Eintensity, Egradient;
    
    CvMat *Iltw_X       = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Iltw_Y       = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Il0_X  = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Il0_Y  = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Irtw_X      = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Irtw_Y      = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Ir0w_X = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Ir0w_Y = cvCreateMat(N_y, N_x, CV_32FC1);
   
    CvMat *U_X = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *U_Y = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *V_X = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *V_Y = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Dt_X   = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *Dt_Y   = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *D0_X  = cvCreateMat(N_y, N_x, CV_32FC1);
    CvMat *D0_Y  = cvCreateMat(N_y, N_x, CV_32FC1);

    if(doleftflow) {
        assert(Iltw && Il0 && U && V);
        GradientFast_32f(U, U_X, U_Y);
        GradientFast_32f(V, V_X, V_Y);
    }
    if (dorightflow) {
        assert(Irtw && Ir0w);
        fprintf(stderr,"cannot compute regularization term for right flow\n");
        return;
    }
    if (dodisparity) {
        assert(Iltw && Irtw && Dt);
	GradientFast_32f(Dt, Dt_X, Dt_Y);
    }
    if (dodisparity0) {
        assert(Il0 && Ir0w && D0);
    }
    if (dodisparity || dodisparity0) {
        GradientFast_32f(D0, D0_X, D0_Y);
    }
    
    // gradients
    CvMat *derivative_tmp = 0;
    if (CV_MAT_TYPE(Il0->type) == CV_8UC1)
        derivative_tmp = cvCreateMat(N_y, N_x, CV_16SC1);
    if (Iltw) {
        assert(Iltw && Iltw_X && Iltw_Y);
        Derivative(Iltw, 1, 0, derivative_tmp, Iltw_X);
        Derivative(Iltw, 0, 1, derivative_tmp, Iltw_Y);
    }
    if (Irtw) {
        assert(Irtw && Irtw_X && Irtw_Y);
        Derivative(Irtw, 1, 0, derivative_tmp, Irtw_X);
        Derivative(Irtw, 0, 1, derivative_tmp, Irtw_Y);
    }
    if (Il0) {
        assert(Il0 && Il0_X && Il0_Y);
        Derivative(Il0, 1, 0, derivative_tmp, Il0_X);
        Derivative(Il0, 0, 1, derivative_tmp, Il0_Y);
    }
    if (Ir0w) {
        assert(Ir0w && Ir0w_X && Ir0w_Y);
        Derivative(Ir0w, 1, 0, derivative_tmp, Ir0w_X);
        Derivative(Ir0w, 0, 1, derivative_tmp, Ir0w_Y);
    }
    cvReleaseMat(&derivative_tmp);

    // energy
    El = 0.;
    Er = 0.;
    Elr = 0.;
    Elr0 = 0.;
    Esmooth = 0.;
    Esfl = 0.;
    Esfr = 0.;
    Esd = 0;
    Esd0 = 0;
    Eintensity = 0.;
    Egradient = 0.;
    for(int yl0=0; yl0 < N_y; yl0++){
	for(int xl0=0; xl0 < N_x; xl0++){
            double ilt = 0., irt = 0.;
            double iltw_X = 0, iltw_Y = 0.;
            double irtw_X = 0, irtw_Y = 0.;
            double il0 = 0., ir0 = 0.;
            double il0_X = 0., il0_Y = 0.;
            double ir0w_X = 0., ir0w_Y = 0.;
            int pixel_doleftflow = 1;
            int pixel_dorightflow = 1;
            int pixel_dostereoinit = 1;
            int pixel_dostereo = 1;
            if (disactivated_pixels_leftflow)
                pixel_doleftflow = !cvmGet8u(disactivated_pixels_leftflow,yl0,xl0);
            if (disactivated_pixels_rightflow)
                pixel_dorightflow = !cvmGet8u(disactivated_pixels_rightflow,yl0,xl0);
            if (disactivated_pixels_stereoinit)
                pixel_dostereoinit = !cvmGet8u(disactivated_pixels_stereoinit,yl0,xl0);
            if (disactivated_pixels_stereo)
                pixel_dostereo = !cvmGet8u(disactivated_pixels_stereo,yl0,xl0);
            if (Il0) {
                il0 = cvGetReal2D(Il0,yl0,xl0);
                il0_X = cvmGet(Il0_X,yl0,xl0);
                il0_Y = cvmGet(Il0_Y,yl0,xl0);
            }
            if (Ir0w) {
                ir0 = cvGetReal2D(Ir0w,yl0,xl0);
                ir0w_X = cvmGet(Ir0w_X,yl0,xl0);
                ir0w_Y = cvmGet(Ir0w_Y,yl0,xl0);
            }
            if (Iltw) {
                ilt = cvGetReal2D(Iltw,yl0,xl0);
                iltw_X = cvmGet(Iltw_X,yl0,xl0);
                iltw_Y = cvmGet(Iltw_Y,yl0,xl0);
            }
	    if (Irtw) {
                irt = cvGetReal2D(Irtw,yl0,xl0);
                irtw_X = cvmGet(Irtw_X,yl0,xl0);
                irtw_Y = cvmGet(Irtw_Y,yl0,xl0);
            }

	    
            // data terms
	    if (pixel_doleftflow && (doleftflow || (dorightflow && dodisparity && dodisparity0))) {
                Ei = square(ilt - il0);
                Eg = square(iltw_X - il0_X)  + square(iltw_Y - il0_Y);
		El += PsiData(Ei  + gamma*Eg);
                Eintensity += Ei;
                Egradient += Eg;
	    }
            if (pixel_dorightflow && (dorightflow || (doleftflow && dodisparity && dodisparity0))) {
                Ei = square(irt - ir0);
                Eg = square(irtw_X - ir0w_X)  + square(irtw_Y - ir0w_Y);
		Er += PsiData(Ei + gamma*Eg);
                Eintensity += Ei;
                Egradient += Eg;
            }
            if (pixel_dostereoinit && (dodisparity0 || (doleftflow && dorightflow && dodisparity))) {
                Ei = square(ir0 - il0);
                Eg = square(ir0w_X - il0_X) + square(ir0w_Y - il0_Y);
		Elr0 += PsiData(Ei + gamma*Eg);
                Eintensity += Ei;
                Egradient += Eg;
            }
            if (pixel_dostereo && (dodisparity || (doleftflow && dorightflow && dodisparity0))) {
                Ei = square(irt - ilt);
                Eg = square(irtw_X - iltw_X)   + square(irtw_Y - iltw_Y);
		Elr += PsiData(Ei  + gamma*Eg);
                Eintensity += Ei;
                Egradient += Eg;
            }
            // regularization terms
            Esmoothtmp = 0.;
	    if(doleftflow) {
                double u_X = cvmGet(U_X,yl0,xl0);
                double u_Y = cvmGet(U_Y,yl0,xl0);
                double v_X = cvmGet(V_X,yl0,xl0);
                double v_Y = cvmGet(V_Y,yl0,xl0);
                Es = square(u_X) + square(u_Y) + square(v_X) + square(v_Y);
		Esmoothtmp += Es;
                Esfl += Es;
	    }
            if (dorightflow) {
            }
            if (dodisparity0) {
                double d0_X = cvmGet(D0_X,yl0,xl0);
                double d0_Y = cvmGet(D0_Y,yl0,xl0);
                Es = mu*(square(d0_X) + square(d0_Y));
                Esmoothtmp += Es;
                Esd0 += Es;
            }
            if (dodisparity) {
                double d0_X = cvmGet(D0_X,yl0,xl0);
                double d0_Y = cvmGet(D0_Y,yl0,xl0);
                double dt_X = cvmGet(Dt_X,yl0,xl0);
                double dt_Y = cvmGet(Dt_Y,yl0,xl0);
                Es = lambda*(square(dt_X - d0_X) + square(dt_Y - d0_Y));
                Esmoothtmp += Es;
                Esd += Es;
            }
	    Esmooth += PsiSmooth(Esmoothtmp);
	    
	}
    }
    *data = El + Er + Elr + Elr0;
    *smooth = Esmooth;
    if (Efl)
        *Efl = El;
    if (Efr)
        *Efr = Er;
    if (Est)
        *Est = Elr;
    if (Es0)
        *Es0 = Elr0;
    if (Esflp)
        *Esflp = Esfl;
    if (Esfrp)
        *Esfrp = Esfr;
    if (Esdp)
        *Esdp = Esd;
    if (Esd0p)
        *Esd0p = Esd0;
    if (Eintensityp)
        *Eintensityp = Eintensity;
    if (Egradientp)
        *Egradientp = Egradient;
    cvReleaseMat(&U_X);
    cvReleaseMat(&U_Y);
    cvReleaseMat(&V_X);
    cvReleaseMat(&V_Y);
    cvReleaseMat(&Dt_X);
    cvReleaseMat(&Dt_Y);
    cvReleaseMat(&D0_X);
    cvReleaseMat(&D0_Y);
    cvReleaseMat(&Iltw_X);
    cvReleaseMat(&Iltw_Y);
    cvReleaseMat(&Irtw_X);
    cvReleaseMat(&Irtw_Y);
    cvReleaseMat(&Il0_X);
    cvReleaseMat(&Il0_Y);
    cvReleaseMat(&Ir0w_X);
    cvReleaseMat(&Ir0w_Y);
}


//************************3 elements maxima computation*****************
static inline double max_3_arg(const double x, const double y, const double z)
{
    return fmax(fmax(x,y),z);
}

//**************************************Vector euclidean norm computation*******************************************************
static inline double norm_vect(const double x, const double y)
{
    return sqrt(x * x + y * y);
}

//**************************************Matrix euclidean norm computation*******************************************************
static inline double norm_mat(const double a, const double b, const double c, const double d)
{
    return sqrt(a * a + b * b + c * c + d * d);
}

//***************************************Gaussian data pyramid computation **********
static inline void GaussianPyramid(CvArr * Data, CvArr * Undersampled_Data)
{
    cvPyrDown(Data, Undersampled_Data);
}

//***************************************Angle of a vector computation with the X axis*********************
static inline double teta_vect(const double x, const double y){
    return acos(x/norm_vect(x,y));
}

//***********************************************Underpolate data to the k-ith resolution level with the reduction factor eta ***************
#if 1
static inline void Underpolate(const CvMat* data, //Data to be undersampled
			       CvMat* data_underpolated, //Undersampled data
			       const int k, //Resolution level
			       const double eta, //Reduction factor
			       const double total_scale)
{
	(void)eta;
	(void)k;
    cvResize(data, data_underpolated, CV_INTER_AREA);
    cvConvertScale(data_underpolated,data_underpolated, total_scale);
}
#else
static void Underpolate(CvMat* data, CvMat* data_unterpolated, const int k, const double eta, const double total_scale){
   
    int N_x = cvGetSize(data).width;
    int N_y = cvGetSize(data).height;
    CvMat* data_cur = cvCreateMat(N_y, N_x, CV_32FC1);
    cvCopy(data, data_cur);
    double eta_total = 1.;
    for(int i = 0; i < k; i++){
        eta_total *= eta;
        CvMat* data_next =  cvCreateMat(cvCeil(N_y*eta_total), cvCeil(N_x*eta_total) , CV_32FC1);
        cvResize(data_cur, data_next, CV_INTER_AREA);
        //SmoothData(data_next);
	cvReleaseMat(&data_cur);
	data_cur = data_next;
    }
    assert(CV_ARE_SIZES_EQ(data_cur,data_underpolated));
    cvConvertScale(data_cur,data_underpolated, total_scale);
    cvReleaseMat(&data_cur);
}
#endif

/***************************************************euclidean norm in R^n*****************************/
double n_norm(CvMat *x, const int Nx){
    double n = 0.;
    for(int k=0; k < Nx; k++)
        n += square(cvGetReal2D(x,k,0));
    return sqrt(n);
}

/***************************************************QR Factorization*********************************/
void QR(CvMat *M, CvMat *Q, CvMat *R, const int Nx, const int Ny){
    cvSetZero(Q);
    cvSetZero(R);
  
}

/**************************************************Matrix Transposition*****************************/
void Transpose(CvMat *Q1, CvMat *Q2, const int N){
    double tmp;
    cvSetZero(Q2);
    for(int i = 0; i < N; i++){
        for(int j = 0; j <= i; j++){
            if(i != j){
                tmp = cvGetReal2D(Q1,i,j);
                cvSetReal2D(Q2,j,i,tmp);
                tmp = cvGetReal2D(Q1,j,i);
                cvSetReal2D(Q2,i,j,tmp);
            }
            else{
                tmp = cvGetReal2D(Q1,i,j);
                cvSetReal2D(Q2,i,j,tmp);
            }
        }
    }
}

/********************************************Scene flow matrix with vector multiplication to spare memory******/
void SFMatrixProduct(CvMat *SFMatrix, CvMat *x, CvMat *x_end, const int Nx, const int Ny, const int dimension){
  
    int i;
    int N = dimension * Nx * Ny;
    for (i = 0; i < N; i++) {
		    
	if(i < dimension*Nx){
	    double x1 = cvGetReal2D(x,i,0);
	    double x2 = cvGetReal2D(x,i+dimension*Nx,0);
	    double m1 = cvGetReal2D(SFMatrix,i,0);
	    double m2 = cvGetReal2D(SFMatrix,i,1);
	    cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2);
	}
	else if(i >= dimension*Nx*(Ny-1)){
	    double x1 = cvGetReal2D(x,i,0);
	    double x2 = cvGetReal2D(x,i-dimension*Nx,0);
	    double m1 = cvGetReal2D(SFMatrix,i,0);
	    double m2 = cvGetReal2D(SFMatrix,i,1);
	    cvSetReal2D(x_end, i, 0,  m1*x1 + m2*x2);
	}
	else{
	    if(i%(dimension*Nx) == 0){
		double x1 = cvGetReal2D(x,i,0);
		double x2 = cvGetReal2D(x,i+dimension,0);
		double m1 = cvGetReal2D(SFMatrix,i,0);
		double m2 = cvGetReal2D(SFMatrix,i,1);
		cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2);
	    }
	    else if((i+dimension)%(dimension*Nx)==0){
		double x1 = cvGetReal2D(x,i,0);
		double x2 = cvGetReal2D(x,i-dimension,0);
		double m1 = cvGetReal2D(SFMatrix,i,0);
		double m2 = cvGetReal2D(SFMatrix,i,1);
		cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2);
	    }
	    else{

		int rest1 = i%(dimension*Nx);
		if(rest1 == 1 || rest1 == 2 || rest1 == 3){
                    double x1 = cvGetReal2D(x,i,0);
                    double x2 = cvGetReal2D(x,i+dimension,0);
                    double m1 = cvGetReal2D(SFMatrix,i,0);
                    double m2 = cvGetReal2D(SFMatrix,i,1);
                    cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2); 
		}
		else if(rest1 == 4*Nx - 3 || rest1 == 4*Nx - 2 || rest1 == 4*Nx - 1){
		    double x1 = cvGetReal2D(x,i,0);
		    double x2 = cvGetReal2D(x,i-dimension,0);
		    double m1 = cvGetReal2D(SFMatrix,i,0);
		    double m2 = cvGetReal2D(SFMatrix,i,1);
		    cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2);
		}
		else{
		    int rest2 = rest1 % dimension;
		    
		    if(rest2 == 0){

                        double x1 = cvGetReal2D(x,i,0);
                        double x2 = cvGetReal2D(x,i-dimension*Nx,0);
                        double x3 = cvGetReal2D(x,i-dimension,0);
                        double x4 = cvGetReal2D(x,i+1,0);
                        double x5 = cvGetReal2D(x,i+2,0);
                        double x6 = cvGetReal2D(x,i+3,0);
                        double x7 = cvGetReal2D(x,i+dimension*Nx,0);
                        double x8 = cvGetReal2D(x,i+dimension,0);
			
                        double m1 = cvGetReal2D(SFMatrix,i,0);
                        double m2 = cvGetReal2D(SFMatrix,i,4);
                        double m3 = cvGetReal2D(SFMatrix,i,6);
                        double m4 = cvGetReal2D(SFMatrix,i,1);
                        double m5 = cvGetReal2D(SFMatrix,i,2);
                        double m6 = cvGetReal2D(SFMatrix,i,3);
                        double m7 = cvGetReal2D(SFMatrix,i,5);
                        double m8 = cvGetReal2D(SFMatrix,i,7);
			 
                        cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2 + m3*x3 + m4*x4 + m5*x5 + m6*x6 + m7*x7 + m8*x8);
					
                    }
		    else if(rest2 == 1){
			//v equation
                        double x1 = cvGetReal2D(x,i,0);
                        double x2 = cvGetReal2D(x,i-dimension*Nx,0);
                        double x3 = cvGetReal2D(x,i-dimension,0);
                        double x4 = cvGetReal2D(x,i-1,0);
                        double x5 = cvGetReal2D(x,i+1,0);
                        double x6 = cvGetReal2D(x,i+2,0);
                        double x7 = cvGetReal2D(x,i+dimension*Nx,0);
                        double x8 = cvGetReal2D(x,i+dimension,0);
			
                        double m1 = cvGetReal2D(SFMatrix,i,1);
                        double m2 = cvGetReal2D(SFMatrix,i,4);
                        double m3 = cvGetReal2D(SFMatrix,i,6);
                        double m4 = cvGetReal2D(SFMatrix,i,0);
                        double m5 = cvGetReal2D(SFMatrix,i,2);
                        double m6 = cvGetReal2D(SFMatrix,i,3);
                        double m7 = cvGetReal2D(SFMatrix,i,5);
                        double m8 = cvGetReal2D(SFMatrix,i,7);
			 
                        cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2 + m3*x3 + m4*x4 + m5*x5 + m6*x6 + m7*x7 + m8*x8);

		    }

		    else if(rest2 == 2){
			//dprime equation
                        double x1 = cvGetReal2D(x,i,0);
                        double x2 = cvGetReal2D(x,i-dimension*Nx,0);
                        double x3 = cvGetReal2D(x,i-dimension,0);
                        double x4 = cvGetReal2D(x,i-2,0);
                        double x5 = cvGetReal2D(x,i-1,0);
                        double x6 = cvGetReal2D(x,i+1,0);
                        double x7 = cvGetReal2D(x,i+dimension*Nx,0);
                        double x8 = cvGetReal2D(x,i+dimension,0);
			
                        double m1 = cvGetReal2D(SFMatrix,i,2);
                        double m2 = cvGetReal2D(SFMatrix,i,4);
                        double m3 = cvGetReal2D(SFMatrix,i,6);
                        double m4 = cvGetReal2D(SFMatrix,i,0);
                        double m5 = cvGetReal2D(SFMatrix,i,1);
                        double m6 = cvGetReal2D(SFMatrix,i,3);
                        double m7 = cvGetReal2D(SFMatrix,i,5);
                        double m8 = cvGetReal2D(SFMatrix,i,7);
			 
                        cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2 + m3*x3 + m4*x4 + m5*x5 + m6*x6 + m7*x7 + m8*x8);
			  
					    
		    }
		    else{
			//d equation
			double x1 = cvGetReal2D(x,i,0);
			double x2 = cvGetReal2D(x,i-dimension*Nx,0);
			double x3 = cvGetReal2D(x,i-dimension,0);
			double x4 = cvGetReal2D(x,i-3,0);
			double x5 = cvGetReal2D(x,i-2,0);
			double x6 = cvGetReal2D(x,i-1,0);
			double x7 = cvGetReal2D(x,i+dimension*Nx,0);
			double x8 = cvGetReal2D(x,i+dimension,0);
			
			double m1 = cvGetReal2D(SFMatrix,i,3);
			double m2 = cvGetReal2D(SFMatrix,i,4);
			double m3 = cvGetReal2D(SFMatrix,i,6);
			double m4 = cvGetReal2D(SFMatrix,i,0);
			double m5 = cvGetReal2D(SFMatrix,i,1);
			double m6 = cvGetReal2D(SFMatrix,i,2);
			double m7 = cvGetReal2D(SFMatrix,i,5);
			double m8 = cvGetReal2D(SFMatrix,i,7);
			 
			cvSetReal2D(x_end, i, 0, m1*x1 + m2*x2 + m3*x3 + m4*x4 + m5*x5 + m6*x6 + m7*x7 + m8*x8);
					
		    }
		    
		}
	    }
	}
    }

}






/*******************************************Scalar Product in R^k****************************/
double scalar_prod(CvMat *x, CvMat *y, const int k){
    double scalar = 0.;
    for(int p = 0; p < k; p++)
        scalar += cvGetReal2D(x,p,0)*cvGetReal2D(y,p,0);
    return scalar;
}

/*************************************Extract a matrix column ****************/
void ColumnExtract(CvMat* Q, CvMat* q1, const int j, const int N){
    for(int p = 0; p < N; p++)
        cvSetReal2D(q1,p,0,cvGetReal2D(Q,p,j));
}

/**************************************************GMRES Method to solve the scene flow final linear system ***********/
void GmRes(CvMat *M, CvMat *b, CvMat *x0, CvMat *xk, CvMat *rk, const int Kmax, const double epsilon, const int Nx, const int Ny, const int dimension){
    int i,j,k, p;
    double h;
    const int N = Nx*Ny*dimension;
    CvMat *r0 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *q1 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *q2 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *Q =  cvCreateMat(N, Kmax, CV_32FC1);
    CvMat *x1 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *H = cvCreateMat(Kmax + 1, Kmax, CV_32FC1);
    CvMat *H_end = cvCreateMat(Kmax, Kmax, CV_32FC1);
    CvMat *c = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *s = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *g = cvCreateMat(Kmax+1, 1, CV_32FC1);
    CvMat *g_end = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *X = cvCreateMat(N, 1, CV_32FC1);
    CvMat *Y = cvCreateMat(Kmax, 1, CV_32FC1);
  
    cvSetZero(H);
    cvSetZero(Q);
    cvSetZero(H_end);
    cvSetZero(g);
    cvSetZero(g_end);
    //Initial residu r0 initialization
    SFMatrixProduct(M, x0, x1, Nx, Ny, dimension);
    cvSub(b, x1, r0);
    double beta = n_norm(r0,N);
    cvConvertScale(r0, q1, 1./beta); 
    //Q initialization
    for(p=0; p < N; p++)
        cvSetReal2D(Q, p, 0, cvGetReal2D(q1, p, 0));
  
    k = 1;
    double rho = beta;
    cvSetReal2D(g,0,0,beta);
    //Main loop
    while(k <= Kmax /*&& rho > epsilon * beta*/ ){
        SFMatrixProduct(M, q1, q2, Nx, Ny, dimension); //q2 = M*qk 
        //extracting vk
        ColumnExtract(Q, q1, k-1, N);
        for(j = 1; j <= k; j++){
            CvMat *gamma =  cvCreateMat(N, 1, CV_32FC1);
            ColumnExtract(Q,gamma, j-1, N); //gamma = qj
            h = scalar_prod(q2, gamma, N); //<M*qk,qj>
            cvSetReal2D(H, j-1, k-1,h); //Hjk = <M*qk,qj>
            cvConvertScale(gamma, gamma, h);
            cvSub(q2, gamma, q2);
            cvReleaseMat(&gamma);
        }
        h = n_norm(q2,N);
        cvSetReal2D(H, k, k-1,h);
        cvConvertScale(q2,q2,1./h);
        //Q update
        if(k < Kmax){
            for(p=0; p < N; p++)
                cvSetReal2D(Q, p, k, cvGetReal2D(q2, p, 0));
        }
     
        //Givens Rotations
        for(i = 1; i < k; i++){
            double t1 = cvGetReal2D(H,k,i-1);
            double t2 = cvGetReal2D(H,k,i);
            double ci = cvGetReal2D(c, i-1,0);
            double si = cvGetReal2D(s, i-1,0);
            cvSetReal2D(H,k,i-1,ci*t1+si*t2);
            cvSetReal2D(H,k,i,ci*t2-si*t1);
        }
        //Givens rotations computations
        double h1 = cvGetReal2D(H, k-1,k-1);
        double h2 = cvGetReal2D(H, k,k-1);
        double nu = sqrt(square(h1) + square(h2));
        double ck = h1/nu;
        double sk = h2/nu;
        cvSetReal2D(c,k-1,0,ck);
        cvSetReal2D(s,k-1,0,sk);
     
        //H update
        cvSetReal2D(H,k-1,k-1,ck*h1+sk*h2);
        cvSetReal2D(H,k,k-1,0.);
     
        double a1 = cvGetReal2D(g,k-1,0);
        double a2 = cvGetReal2D(g,k,0);
        cvSetReal2D(g,k-1,0,ck*a1+sk*a2);
        cvSetReal2D(g,k,0,ck*a2-sk*a1);

        rho = fabs(ck*a2-sk*a1);
        k++;
    }
    //Get out the Kmax*Kmax matrix from H
    for(i = 0; i < Kmax; i++){
        cvSetReal2D(g_end, i,0, cvGetReal2D(g,i,0));
        for(j = 0; j < Kmax; j++)
            cvSetReal2D(H_end, i,j, cvGetReal2D(H,i,j));
    }
    //Solving the linear system
    double det = cvDet(H_end);
    cvSolve(H_end, g_end, Y);
  
    //X=QY
    cvGEMM(Q, Y, 1, NULL, 0., X);
 
    //The actual solution
    cvAdd(x0, X, xk);

    //The residu
    SFMatrixProduct(M, X, rk, Nx, Ny, dimension);
    cvSub(r0, rk, rk);
		  
    cvReleaseMat(&r0);
    cvReleaseMat(&q1);
    cvReleaseMat(&q2);
    cvReleaseMat(&x1);
    cvReleaseMat(&H);
    cvReleaseMat(&H_end);
    cvReleaseMat(&Q);
    cvReleaseMat(&X);	  
    cvReleaseMat(&Y);	  
    cvReleaseMat(&c);
    cvReleaseMat(&s);
    cvReleaseMat(&g);
    cvReleaseMat(&g_end);
}

/**************************************************GMRES Method to solve the scene flow final linear system ***********/
void GmRes2(CvMat *M, CvMat *b, CvMat *x0, CvMat *xk, CvMat *rk, const int Kmax, const double epsilon, const int Nx, const int Ny, const int dimension){
    int i,j, p;
    double h;
    const int N = Nx*Ny*dimension;
    CvMat *r0 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *v1 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *v2 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *V =  cvCreateMat(N, Kmax, CV_32FC1);
    CvMat *x1 = cvCreateMat(N, 1, CV_32FC1);
    CvMat *H = cvCreateMat(Kmax + 1, Kmax, CV_32FC1);
    CvMat *H_end = cvCreateMat(Kmax, Kmax, CV_32FC1);
    CvMat *c = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *s = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *g = cvCreateMat(Kmax+1, 1, CV_32FC1);
    CvMat *g_end = cvCreateMat(Kmax, 1, CV_32FC1);
    CvMat *X = cvCreateMat(N, 1, CV_32FC1);
    CvMat *Y = cvCreateMat(Kmax, 1, CV_32FC1);
  
    cvSetZero(H);
    cvSetZero(V);
    cvSetZero(H_end);
    cvSetZero(g);
    cvSetZero(g_end);
    //Initial residu r0 initialization
    SFMatrixProduct(M, x0, x1, Nx, Ny, dimension);
    cvSub(b, x1, r0);
    double beta = n_norm(r0,N);
    cvConvertScale(r0, v1, 1./beta); 
    //V Krylov vectors matrix initialization
    for(p=0; p < N; p++)
        cvSetReal2D(V, p, 0, cvGetReal2D(v1, p, 0));
  
    double rho = beta;
    cvSetReal2D(g,0,0,beta);
    //Main loop
    for(j = 1; j <= Kmax; j++){
	//v1 = vj
	ColumnExtract(V,v1,j-1,N);
	CvMat *gamma =  cvCreateMat(N, 1, CV_32FC1);
	SFMatrixProduct(M, v1, gamma, Nx, Ny, dimension); //gamma = M*vj 
        for(i = 1; i <= j; i++){
	   ColumnExtract(V,v2,i-1,N); //v2 = Vi		
	   h = scalar_prod(gamma, v2, N); //h_ij = <M*vj,vi>	
	   cvSetReal2D(H, i-1, j-1,h); //Hij = <M*vj,vi>
	   cvConvertScale(v2, v2, h); //vi = h_ij * vi
           cvSub(gamma, v2,v1); //vj+1 = M*vj - h_ij*vi 	
        }
	h = n_norm(v1, N);
	cvSetReal2D(H, j, j-1,h);		
	//normalize the new krylov vector
	cvConvertScale(v1, v1, h);
	//V update
        if(j < Kmax){
            for(p=0; p < N; p++)
                cvSetReal2D(V, p, j, cvGetReal2D(v1, p, 0));
        }	

	cvReleaseMat(&gamma);		
     
        //Givens Rotations
       /* int k;
	   for(i = 1; i < k; i++){
            double t1 = cvGetReal2D(H,k,i-1);
            double t2 = cvGetReal2D(H,k,i);
            double ci = cvGetReal2D(c, i-1,0);
            double si = cvGetReal2D(s, i-1,0);
            cvSetReal2D(H,k,i-1,ci*t1+si*t2);
            cvSetReal2D(H,k,i,ci*t2-si*t1);
        }
        //Givens rotations computations
        double h1 = cvGetReal2D(H, k-1,k-1);
        double h2 = cvGetReal2D(H, k,k-1);
        double nu = sqrt(square(h1) + square(h2));
        double ck = h1/nu;
        double sk = h2/nu;
        cvSetReal2D(c,k-1,0,ck);
        cvSetReal2D(s,k-1,0,sk);
     
        //H update
        cvSetReal2D(H,k-1,k-1,ck*h1+sk*h2);
        cvSetReal2D(H,k,k-1,0.);
     
        double a1 = cvGetReal2D(g,k-1,0);
        double a2 = cvGetReal2D(g,k,0);
        cvSetReal2D(g,k-1,0,ck*a1+sk*a2);
        cvSetReal2D(g,k,0,ck*a2-sk*a1);

        rho = fabs(ck*a2-sk*a1);
        k++;*/
    }
    //Get out the Kmax*Kmax matrix from H
   /* for(i = 0; i < Kmax; i++){
        cvSetReal2D(g_end, i,0, cvGetReal2D(g,i,0));
        for(j = 0; j < Kmax; j++)
            cvSetReal2D(H_end, i,j, cvGetReal2D(H,i,j));
    }
    //Solving the linear system
    //double det = cvDet(H_end);
    cvSolve(H_end, g_end, Y);
  
    //X=QY
    cvGEMM(Q, Y, 1, NULL, 0., X);
 
    //The actual solution
    cvAdd(x0, X, xk);

    //The residu
    SFMatrixProduct(M, X, rk, Nx, Ny, dimension);
    cvSub(r0, rk, rk);*/
		  
    cvReleaseMat(&r0);
    cvReleaseMat(&v1);
    cvReleaseMat(&v2);
    cvReleaseMat(&x1);
    cvReleaseMat(&H);
    cvReleaseMat(&H_end);
    cvReleaseMat(&V);
    cvReleaseMat(&X);	  
    cvReleaseMat(&Y);	  
    cvReleaseMat(&c);
    cvReleaseMat(&s);
    cvReleaseMat(&g);
    cvReleaseMat(&g_end);
}



void PyramidBuild(CvMat *I,
                  double eta,
                  int pyramid_levels,
                  CvMat **I_pyramid)//in: array[pyramid_levels] of non-allocated pointers. out: pyramid
{
    //Images Gaussian Pyramid Computation
    double eta_total = 1.;
    int N_x = cvGetSize(I).width;
    int N_y = cvGetSize(I).height;

    I_pyramid[0] = cvCloneMat(I);
    
    for (int level = 1; level < pyramid_levels; level++) {
        eta_total *= eta;
        CvSize size_i = cvSize(cvCeil(N_x*eta_total),cvCeil(N_y*eta_total));
	//Images memory allocation
	I_pyramid[level] = cvCreateMat(size_i.height, size_i.width, CV_MAT_TYPE(I->type));       
	cvResize(I_pyramid[level-1], I_pyramid[level], CV_INTER_AREA);
    }
}

void PyramidSmooth(int pyramid_levels,
                   CvMat **I_pyramid)
{
    for (int level = 0; level < pyramid_levels; level++)
        SmoothImage(I_pyramid[level]);  // Il0_pyramid is pre-smoothed
}

void PyramidCopy(int pyramid_levels,
                 const CvMat * const * const In_pyramid,
                 CvMat ** Out_pyramid)
{
    for (int level = 0; level < pyramid_levels; level++) {
        Out_pyramid[level] = cvCloneMat(In_pyramid[level]);
    }
}

/**************************************************Optical Flow computation using Brox et al method*****************/
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
	       const double gamma) //the gradients weight
{
    int N_x,N_y;
    int pyramidtype;
    CvMat *U, *V;

    const int dimension = 2;

    assert(pyramid_level_start >= pyramid_level_end);
    assert(pyramid_level_end >= 1);
    assert(pyramid_level_start <= pyramid_levels);
    pyramidtype = CV_MAT_TYPE(Il0_pyramid[pyramid_level_start-1]->type);
    assert(pyramidtype == CV_8UC1 || pyramidtype == CV_32FC1);
    assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], U_start));
    assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], V_start));
    assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], U_end));
    assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], V_end));
    assert(CV_ARE_TYPES_EQ(Il0_pyramid[pyramid_level_start-1],Ilt_pyramid[pyramid_level_start-1]));
    
    //intial optical flow is set to the lowest resolution
    {
        CvSize size_start = cvGetSize(Il0_pyramid[pyramid_level_start-1]);
        
        U = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
        V = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
    }
    //Underpolate(U_start, U, pyramid_level_start - 1, eta, (double)U->width / U_start->width);
    cvCopy(U_start, U);
    //Underpolate(V_start, V, pyramid_level_start - 1, eta, (double)V->height / V_start->height);
    cvCopy(V_start, V);
    if(logfile){
        double max_Fx, min_Fx, max_Fy, min_Fy;
        cvMinMaxLoc(U, &min_Fx, &max_Fx);
        cvMinMaxLoc(V, &min_Fy, &max_Fy);
        fprintf(logfile, "initial optical flow (reduced resolution): %g<u<%g %g<v<%g\n", min_Fx, max_Fx, min_Fy, max_Fy);
        fflush(logfile);
    }
  
    //Loop over the resolution levels
    for(int level = pyramid_level_start - 1; level >= pyramid_level_end - 1; level--){
        assert(Il0_pyramid[level] && Ilt_pyramid[level]);
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], Ilt_pyramid[level]));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], U));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], V));

        CvSize level_size = cvGetSize(Il0_pyramid[level]);
        int xl0, yl0;

        if (OF_I0_FileName) {
            SaveImage(OF_I0_FileName,  Il0_pyramid[level]);
        }
        if (OF_It_FileName) {
            SaveImage(OF_It_FileName,  Ilt_pyramid[level]);
        }
	//Size of the current images
	N_x = level_size.width;
	N_y = level_size.height;


	//Maps used for the images warping : Initialization
	CvMat *MapIdentityX = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *MapIdentityY = cvCreateMat(N_y, N_x, CV_32FC1);
	for(yl0=0; yl0 < N_y; yl0++){
	    for(xl0=0; xl0 < N_x; xl0++){
		cvmSet(MapIdentityX, yl0, xl0, xl0);
		cvmSet(MapIdentityY, yl0, xl0, yl0);
	    }
	}
	//Creation and initialization of the warped image I_{l}(x+u, y+v) and the coordinates maps used to create it
	CvMat *Iltw = cvCreateMat(N_y, N_x, pyramidtype);
	CvMat *MapX_Ilt =  cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *MapY_Ilt =  cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *MapX_Ilt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *MapY_Ilt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
        
	//Images gradients computation
        CvMat *derivative_tmp = 0;
        if (pyramidtype == CV_8UC1)
            derivative_tmp = cvCreateMat(N_y, N_x, CV_16SC1);
	CvMat *Il0_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Il0_Y = cvCreateMat(N_y, N_x, CV_32FC1);
        CvMat *Iltw_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Iltw_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	// Second derivatives
	CvMat *Iltw_XX = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Iltw_XY = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Iltw_YY = cvCreateMat(N_y, N_x, CV_32FC1);
	    
        // compute gradients of the only non-warped image
        assert(Il0_X && Il0_Y);
        Derivative(Il0_pyramid[level], 1, 0, derivative_tmp, Il0_X);
        Derivative(Il0_pyramid[level], 0, 1, derivative_tmp, Il0_Y);


	//M is the matrix of the final linear system and B the second member of this system
        //int M_size[2] = {2 * N_x * N_y, 6};
        int M_dim;
#if DIVERGENCE_8
        M_dim = 10;
#else  // !DIVERGENCE_8
        M_dim = 6;
#endif  // !DIVERGENCE_8
	CvMat *M = cvCreateMat(dimension*N_x*N_y, M_dim, CV_32FC1); 
	CvMat *B = cvCreateMat(dimension*N_x*N_y,1,CV_32FC1);

        CvMat *disactivated_pixels = cvCreateMat(N_y, N_x, CV_8UC1);
        cvSetZero(disactivated_pixels);
        cvmSet8u_borders(disactivated_pixels, 127);
        
        //divergence coefficients
	CvMat *divergence_coefficients = cvCreateMat(N_y, N_x, CV_32FC1);

	//Saving the initial flow images at beginning of each resolution level
        if (OF_U_FileName) {
            SaveResults(OF_U_FileName, U);
        }	     
        if (OF_V_FileName) {
            SaveResults(OF_V_FileName, V);
        }
     
	CvMat *U_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *V_prec = cvCreateMat(N_y, N_x, CV_32FC1);

	//Flow increments and their derivatives	
	CvMat *dU = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dV = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dU_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dU_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dV_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dV_Y = cvCreateMat(N_y, N_x, CV_32FC1);

	 
	// flow gradients
	CvMat *U_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *U_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *V_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *V_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	 
	CvMat *dU_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *dV_prec = cvCreateMat(N_y, N_x, CV_32FC1);	     
       
	//For the final linear system and SOR iterations
	CvMat *SOR = cvCreateMat(dimension*N_x*N_y,1, CV_32FC1);		
	CvMat *SOR_prec = cvCreateMat(dimension*N_x*N_y,1,CV_32FC1);
        
	//Main loop over the optical flow values
        int iter_SOR = 0, iter_inner = 0, iter_outer = 0;
        int first_iteration = 1;
	double norm = 0., norm_inner = 0., norm_SOR = 0.;
	while((norm > epsilon_outer && iter_outer <= max_iter_outer) || iter_outer < 2 || first_iteration == 1){
	    first_iteration = 0;
	    
	    //Coordinate maps update
	    cvAdd(MapIdentityX, U, MapX_Ilt);
	    cvAdd(MapIdentityY, V, MapY_Ilt);
	
	    //Images warping
	    // Images warping + Warped images gradient computation
	    // (Warped images are smoothed)
	    cvRemap(Ilt_pyramid[level], Iltw, MapX_Ilt, MapY_Ilt);
	    SmoothImage(Iltw);
	    if (OF_It_Warped_FileName) {
		SaveImage(OF_It_Warped_FileName, Iltw);
	    }
	    //Warped images gradient computation
            assert(Iltw && Iltw_X && Iltw_Y && Iltw_XX && Iltw_XY && Iltw_YY);
	    Derivative(Iltw, 1, 0, derivative_tmp, Iltw_X);
	    Derivative(Iltw, 0, 1, derivative_tmp, Iltw_Y);
	    Derivative(Iltw, 2, 0, derivative_tmp, Iltw_XX);
	    Derivative(Iltw, 1, 1, derivative_tmp, Iltw_XY);
	    Derivative(Iltw, 0, 2, derivative_tmp, Iltw_YY);


	    //Flow gradients computation
	    GradientFast_32f(U, U_X, U_Y);
	    GradientFast_32f(V, V_X, V_Y);
	     
            //Increments initialization
	    cvSetZero(dU);
	    cvSetZero(dV);
	  
	    norm_inner = 0.;
	    first_iteration = 1;
	    iter_inner = 0;
	    //secondary fixed point iterations
	    while((norm_inner > epsilon_inner && iter_inner <= max_iter_inner) || first_iteration == 1){
                int i;
                
		first_iteration = 0;

		cvAdd(MapX_Ilt, dU, MapX_Ilt_inner);
		cvAdd(MapY_Ilt, dV, MapY_Ilt_inner);

		//flow increments gradients computation
		GradientFast_32f(dU, dU_X, dU_Y);
		GradientFast_32f(dV, dV_X, dV_Y);
		for(yl0 = 0; yl0 < N_y; yl0++){
		    for(xl0 = 0; xl0 < N_x; xl0++){
			double du_X = cvmGet(dU_X, yl0, xl0);
			double du_Y = cvmGet(dU_Y, yl0, xl0);
			double u_X = cvmGet(U_X, yl0, xl0);
			double u_Y = cvmGet(U_Y, yl0, xl0);
			double dv_X = cvmGet(dV_X, yl0, xl0);
			double dv_Y = cvmGet(dV_Y, yl0, xl0);
			double v_X = cvmGet(V_X, yl0, xl0);
			double v_Y = cvmGet(V_Y, yl0, xl0);
                        double s2 = (square(u_X + du_X) + square(u_Y + du_Y) +
                                     square(v_X + dv_X) + square(v_Y + dv_Y));
			double a = DPsiSmooth(s2) ;
			cvmSet(divergence_coefficients, yl0, xl0, a);
		    }
		}
		  
		//Computing M and B for the final linear system 
		cvSetZero(M);
		cvSetZero(B);

		//The dimension*N_x first lines in M correspond to the points on the y = 0 on the image
		int inf = 0;
                int sup = dimension*N_x;
		for (i = inf; i < sup; i++) {
		    cvmSet(M, i, 0, -1./h); 
		    cvmSet(M, i, 1,  1./h);			 
		}

		//Points on the lower horizontal side of the image      
		inf = dimension*N_x*(N_y-1);
		sup = dimension*N_x*N_y;
		for (i = inf; i < sup; i++) {
                    cvmSet(M, i, 0,  1./h); 
                    cvmSet(M, i, 1, -1./h);
		}

	
		inf = dimension*N_x;
		sup = dimension*N_x*(N_y-1) - 1;
		     
                //We fill in the lines of the matrix M corresponding to images pixels excepted points on the upper and lower horizontal side
		for(i = inf; i <= sup; i+=dimension){
                    yl0 = i/(dimension*N_x);
		    xl0 = (i % (inf))/dimension;
		    if(xl0 == 0){ //if point on the left side of image
			for(int d=0; d<dimension; d++) {
                            cvmSet(M,i+d,0,-1./h);
			    cvmSet(M,i+d,1, 1./h);
			}
		    }
		    else if(xl0 == (N_x-1)){//if point on the right side
                        for(int d=0; d<dimension; d++) {
			    cvmSet(M,i+d,0, 1./h);
			    cvmSet(M,i+d,1,-1./h);  
			}
		    }
		    else{
                        //Pixel inside the integration domain 
                        double ilt = 0., il0 = 0.;
                        double ilt_X = 0., ilt_Y = 0.;
                        double iltw_X = 0., iltw_Y = 0.;
                        double il0_X = 0., il0_Y = 0.;
                        double iltw_XX = 0., iltw_XY = 0., iltw_YY = 0.;
                        double iltw_X_X = 0., iltw_X_Y = 0.;
                        double iltw_Y_X = 0., iltw_Y_Y = 0.;
                        //Interpolated divergence coefficients
                        double b_f_1, b_f_2, b_f_3, b_f_4;
			

                        // gradients of the functions used for image warping
                        double uw_X = cvmGet(U_X, yl0, xl0);
                        double uw_Y = cvmGet(U_Y, yl0, xl0);
                        double vw_X = cvmGet(V_X, yl0, xl0);
                        double vw_Y = cvmGet(V_Y, yl0, xl0);
   
                        // Half pixel interpolation for the divergence coefficients 
                        divergence_coefficients_interpolate(divergence_coefficients, yl0, xl0, &b_f_1, &b_f_2, &b_f_3, &b_f_4);
			//If using the 8 points scheme for the divergence
#if DIVERGENCE_8
                        divergence_coefficients_interpolate_diag(divergence_coefficients, yl0, xl0, &b_f_5, &b_f_6, &b_f_7, &b_f_8);
#endif  // DIVERGENCE_8

			//Current coordinates of the followed point 
			double xlt = cvmGet(MapX_Ilt_inner,yl0,xl0);
			double ylt = cvmGet(MapY_Ilt_inner,yl0,xl0);

		
			
#if DIVERGENCE_8
			double b_f_5, b_f_6, b_f_7, b_f_8;
                        //For the 8-divergence
                        double alpha_p_u, alpha_p_v, alpha_d_u, alpha_d_v;
    
			//U and V Gradients angle with the X axis
			double u_X = cvmGet(U_X, yl0, xl0) + cvmGet(dU_X, yl0, xl0);
			double u_Y = cvmGet(U_Y, yl0, xl0) + cvmGet(dU_Y, yl0, xl0);
			double v_X = cvmGet(V_X, yl0, xl0) + cvmGet(dV_X, yl0, xl0);
			double v_Y = cvmGet(V_Y, yl0, xl0) + cvmGet(dV_Y, yl0, xl0);

			//Coefficients computation for 8-divergence in u and v
                        // must be 1 when the gradient is aligned with axes, 0 when diagonal
			//We check if gradients are 0
			if(u_X == 0. && u_Y == 0.) //We take constant coefficients
                            //if(1)
                            alpha_p_u = 0.5;
			else{
                            if (u_X == 0)
                                alpha_p_u = 1;
                            else
                                alpha_p_u = fabs(fabs(atan(u_Y/u_X))/M_PI_4-1);
			}

			if(v_X == 0. && v_Y == 0.)
                            //if(1) 
			    alpha_p_v = 0.5;
			else{
                            if (v_X == 0)
                                alpha_p_v = 1;
                            else
                                alpha_p_v = fabs(fabs(atan(v_Y/v_X))/M_PI_4-1);
			}
			

			//The validity constraint for the directional weights for the divergence 
                        alpha_d_u = (1.-alpha_p_u)/2.;
			alpha_d_v = (1.-alpha_p_v)/2.;
#endif  // DIVERGENCE_8
                        //Pixels out must not be taken into account in the energy data term
                        int pixel_is_disactivated = cvmGet8u(disactivated_pixels,yl0,xl0);

			//is the point visible at time t+1 ?
			if(!pixel_is_disactivated &&                          
                           (xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h  ||
                            ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h)){
                            //printf("disactivate %d,%d\n", x, yl0);
                            cvmSet8u(disactivated_pixels,yl0,xl0,127);
                            pixel_is_disactivated = 1;
			}
		        
			double a_v_1 = 0;
                        double a_u_2 = 0;
			//If using the 8 points scheme for the divergence
#if DIVERGENCE_8
                        double a_u_1 = (alpha/(4.*h*h))*(alpha_p_u*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_u*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
                        double a_v_2 = (alpha/(4.*h*h))*(alpha_p_v*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_v*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
#else  // !DIVERGENCE_8
                        double a_u_1 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                        double a_v_2 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);

#endif  // !DIVERGENCE_8
		        double psi_flowL = 0.;
                            //Data term estimation
                        if (!pixel_is_disactivated) {
                            il0 = cvGetReal2D(Il0_pyramid[level], yl0, xl0); 
                            il0_X = cvmGet(Il0_X,  yl0, xl0);
                            il0_Y = cvmGet(Il0_Y,  yl0, xl0);
			   
                            ilt = cvGetReal2D(Iltw,yl0,xl0);
                            iltw_X = cvmGet(Iltw_X, yl0, xl0);
                            iltw_Y = cvmGet(Iltw_Y, yl0, xl0);
			    iltw_XX = cvmGet(Iltw_XX,yl0,xl0);
			    iltw_XY = cvmGet(Iltw_XY,yl0,xl0);
			    iltw_YY = cvmGet(Iltw_YY,yl0,xl0);

                            // In Maple:
                            // Iltw(x,y):=Ilt(x+u,y+v);
                            // diff(Iltw(x,y),x);
                            // diff(Iltw(x,y),y);
                            // then, express D[1](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                            // and D[2](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                            double det = (1+uw_X)*(1+vw_Y) - uw_Y*vw_X;
                            if (det <= WARP_DETERMINANT_THRESHOLD) {
                                // image is reversed, set everything to 0
                                if (!pixel_is_disactivated) {
                                    cvmSet8u(disactivated_pixels,yl0,xl0,191);
                                    pixel_is_disactivated = 1;
                                }
                            }
                            else {              
                                ilt_X = ((1+vw_Y)*iltw_X - vw_X*iltw_Y)/det;
                                ilt_Y = ((1+uw_X)*iltw_Y - uw_Y*iltw_X)/det;
                            
                                // derivatives of warped image gradients with respect to non-warped coordinates
                                // 1-derivatives of the x coordinate of the gradient of the warped image
                                iltw_X_X = ((1+vw_Y)*iltw_XX - vw_X*iltw_XY)/det;
                                iltw_X_Y = ((1+uw_X)*iltw_XY - uw_Y*iltw_XX)/det;
                                // 2-derivatives of the y coordinate of the gradient of the warped image
                                iltw_Y_X = ((1+vw_Y)*iltw_XY - vw_X*iltw_YY)/det;
                                iltw_Y_Y = ((1+uw_X)*iltw_YY - uw_Y*iltw_XY)/det;

                                double du = cvmGet(dU, yl0, xl0);
                                double dv = cvmGet(dV, yl0, xl0);

                                // IN THE FOLLOWING:
                                // Il0 should match Ilt
                                // The first order Taylor expansion of ilt is:
                                // ilt(x+u+du,y+v+dv) = ilt + ilt_X*du + ilt_Y*dv
                                // iltw_X and iltw_Y should be interpolated similarly to ilt, since
                                // (Il0_X,Il0_Y) should match (Iltw_X,Iltw_Y), NOT (Ilt_X,Ilt_Y) !!!!
                                // see the expression of b below for an example

                                //psi' coefficients computation
                                double a = square(ilt + ilt_X*du + ilt_Y*dv - il0);
                                double b = gamma*(square(iltw_X + iltw_X_X*du + iltw_X_Y*dv - il0_X) +
                                                  square(iltw_Y + iltw_Y_X*du + iltw_Y_Y*dv - il0_Y));
                                //divergence coefficients
                                psi_flowL = DPsiData(a+b);
			       
                                //u equation
                                a_u_1 += psi_flowL*(ilt_X*ilt_X + gamma*(iltw_X_X*iltw_X_X + iltw_Y_X*iltw_Y_X));
                                a_v_1 += psi_flowL*(ilt_Y*ilt_X + gamma*(iltw_X_Y*iltw_X_X + iltw_Y_Y*iltw_Y_X));
                            
                                //v equation
                                a_u_2 += psi_flowL*(ilt_X*ilt_Y + gamma*(iltw_X_X*iltw_X_Y + iltw_Y_X*iltw_Y_Y));
                                a_v_2 += psi_flowL*(ilt_Y*ilt_Y + gamma*(iltw_X_Y*iltw_X_Y + iltw_Y_Y*iltw_Y_Y));
                            }
			}
			 
			//M estimation
			cvmSet(M,i,0,a_u_1);
			cvmSet(M,i+1,0,a_u_2);
			   
			cvmSet(M,i,1,a_v_1);
			cvmSet(M,i+1,1,a_v_2);
		
			 
			//If we use the 8 pts divergence scheme 
#if DIVERGENCE_8
                        cvmSet(M,i,2,(-alpha*alpha_p_u/(4*h*h))*b_f_4);
                        cvmSet(M,i+1,2,(-alpha*alpha_p_v/(4*h*h))*b_f_4);
			   
			   
                        cvmSet(M,i,3,(-alpha*alpha_p_u/(4*h*h))*b_f_3);
                        cvmSet(M,i+1,3,(-alpha*alpha_p_v/(4*h*h))*b_f_3);
				 

                        cvmSet(M,i,4,(-alpha*alpha_p_u/(4*h*h))*b_f_2);
                        cvmSet(M,i+1,4,(-alpha*alpha_p_v/(4*h*h))*b_f_2);
				

                        cvmSet(M,i,5,(-alpha*alpha_p_u/(4*h*h))*b_f_1);
                        cvmSet(M,i+1,5,(-alpha*alpha_p_v/(4*h*h))*b_f_1);
			
                        cvmSet(M,i,6,(-alpha*alpha_d_u/(4*h*h))*b_f_5);
                        cvmSet(M,i+1,6,(-alpha*alpha_d_v/(4*h*h))*b_f_5);
			   
			   
                        cvmSet(M,i,7,(-alpha*alpha_d_u/(4*h*h))*b_f_6);
                        cvmSet(M,i+1,7,(-alpha*alpha_d_v/(4*h*h))*b_f_6);
				 

                        cvmSet(M,i,8,(-alpha*alpha_d_u/(4*h*h))*b_f_7);
                        cvmSet(M,i+1,8,(-alpha*alpha_d_v/(4*h*h))*b_f_7);
				

                        cvmSet(M,i,9,(-alpha*alpha_d_u/(4*h*h))*b_f_8);
                        cvmSet(M,i+1,9,(-alpha*alpha_d_v/(4*h*h))*b_f_8);
#else  // !DIVERGENCE_8
                        cvmSet(M,i,2,(-alpha/(4*h*h))*b_f_4);
                        cvmSet(M,i+1,2,(-alpha/(4*h*h))*b_f_4);
			   
			   
                        cvmSet(M,i,3,(-alpha/(4*h*h))*b_f_3);
                        cvmSet(M,i+1,3,(-alpha/(4*h*h))*b_f_3);
				 

                        cvmSet(M,i,4,(-alpha/(4*h*h))*b_f_2);
                        cvmSet(M,i+1,4,(-alpha/(4*h*h))*b_f_2);
				

                        cvmSet(M,i,5,(-alpha/(4*h*h))*b_f_1);
                        cvmSet(M,i+1,5,(-alpha/(4*h*h))*b_f_1);   
#endif  // !DIVERGENCE_8
			   
                        //second member for the first equation
                        double bu0 = cvmGet(U, yl0, xl0);
                        double bu1 = cvmGet(U, yl0, xl0+1);
                        double bu2 = cvmGet(U, yl0, xl0-1);
                        double bu3 = cvmGet(U,yl0+1,xl0);
                        double bu4 = cvmGet(U,yl0-1,xl0);
                        //Crop(bu0, N_x);
#if DIVERGENCE_8
                        double bu5 = cvmGet(U, yl0+1, xl0+1);
                        double bu6 = cvmGet(U, yl0+1, xl0-1);
                        double bu7 = cvmGet(U,yl0-1,xl0+1);
                        double bu8 = cvmGet(U,yl0-1,xl0-1); 
                        double bu = (alpha/(4*h*h)) *
                            (alpha_p_u *
                             (b_f_1 * bu1 +
                              b_f_2 * bu2 +
                              b_f_3 * bu3 +
                              b_f_4 * bu4) +
                             alpha_d_u *
                             (b_f_5 * bu5 +
                              b_f_6 * bu6 +
                              b_f_7 * bu7 + 
                              b_f_8 * bu8)
                             - (alpha_p_u*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_u*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bu0);
#else  // !DIVERGENCE_8
                        double bu = (alpha/(4*h*h)) *
                            (b_f_1 * bu1 +
                             b_f_2 * bu2 +
                             b_f_3 * bu3 +
                             b_f_4 * bu4
                             - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bu0);
#endif  // !DIVERGENCE_8
			//second member for the second equation
                        double bv0 = cvmGet(V, yl0, xl0);
                        double bv1 = cvmGet(V, yl0, xl0+1);
                        double bv2 = cvmGet(V, yl0, xl0-1);
                        double bv3 = cvmGet(V,yl0+1,xl0);
                        double bv4 = cvmGet(V,yl0-1,xl0);
                        //Crop(bv0, N_y);
#if DIVERGENCE_8
                        double bv5 = cvmGet(V, yl0+1, xl0+1);
                        double bv6 = cvmGet(V, yl0+1, xl0-1);
                        double bv7 = cvmGet(V,yl0-1,xl0+1);
                        double bv8 = cvmGet(V,yl0-1,xl0-1); 
                        double bv = (alpha/(4*h*h)) *
                            (alpha_p_v * 
                             (b_f_1 * bv1 +
                              b_f_2 * bv2 +
                              b_f_3 * bv3 +
                              b_f_4 * bv4)+
                             alpha_d_v *
                             (b_f_5 * bv5 +
                              b_f_6 * bv6 +
                              b_f_7 * bv7 + 
                              b_f_8 * bv8)
                             - (alpha_p_v*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_v*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bv0);
#else  // !DIVERGENCE_8
                        double bv = (alpha/(4*h*h)) *
                            (b_f_1 * bv1 +
                             b_f_2 * bv2 +
                             b_f_3 * bv3 +
                             b_f_4 * bv4
                             - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bv0);
#endif  // !DIVERGENCE_8
                        if (!pixel_is_disactivated) {
                            //second member for the first equation
                            bu +=
                                - psi_flowL*(ilt_X*(ilt - il0) + gamma*(iltw_X_X*(iltw_X - il0_X) + iltw_Y_X*(iltw_Y - il0_Y)));
                            //second member for the second equation
                            bv +=
                                - psi_flowL*(ilt_Y*(ilt - il0) + gamma*(iltw_X_Y*(iltw_X - il0_X) + iltw_Y_Y*(iltw_Y - il0_Y)));
                        }
                        cvmSet(B,i,0,bu);
                        cvmSet(B,i+1,0,bv);
			   
                    }
		       
		}

		//Initial energy computation
		if(logfile && iter_inner == 0
#if VERBOSE_ENERGY
#else
                    && iter_outer == 0
#endif
                    )
                {
                    double Efl, Edata, Esmooth, Eintensity, Egradient;
                    EnergyComputation(Il0_pyramid[level], 0, Iltw, 0,
                                      U, V, 0, 0,
                                      disactivated_pixels, 0, 0, 0,
                                      N_x, N_y, &Edata, &Esmooth, gamma, 1.0, 1.0, 1, 0, 0, 0, &Efl, 0, 0, 0, 0, 0, 0, 0, &Eintensity, &Egradient);
		    fprintf(logfile, "** level %d resolution %d x %d\n", level+1, N_x, N_y);
		    fprintf(logfile, "* energy before outer iteration %d:\n", iter_outer+1);
		    fprintf(logfile, "Edata = %g Esmooth = %g Ed/Es = %g (alpha=%g)\n", Edata, Esmooth, Edata/Esmooth, alpha);
		    fprintf(logfile, "Edata breakdown (squared): Eintensity = %g, Egradient=%g Ei/Eg=%g (gamma=%g)\n", Eintensity, Egradient, Egradient != 0 ? Eintensity/Egradient : 0., gamma);
		    fflush(logfile);
		}


		//SOR iterations
		cvSetZero(SOR);
		cvSetZero(SOR_prec);
		first_iteration = 1;
		iter_SOR = 0;
		norm_SOR = 0.;
		while ((norm_SOR >= epsilon_SOR && iter_SOR < max_iter_SOR) || first_iteration == 1){
		    first_iteration = 0;
			 
			 
		    //Loop on the SOR vector
                    for (int sub_iter_SOR = 0; sub_iter_SOR <4; sub_iter_SOR++) {
                    int line, line_begin, line_end, line_incr;
                    int col, col_begin, col_end, col_incr;
                    switch (sub_iter_SOR) {
                        case 0:         // NE-SW
                            line_begin = 0;
                            line_end = N_y * dimension * N_x;
                            line_incr = dimension * N_x;
                            col_begin = 0;
                            col_end = dimension * N_x;
                            col_incr = 1;
                            break;
                        case 1:         // SW-NE
                            line_begin = (N_y-1) * dimension * N_x;
                            line_end = - dimension * N_x;
                            line_incr = - dimension * N_x;
                            col_begin = dimension * N_x - 1;
                            col_end = -1;
                            col_incr = -1;
                            break;
                        case 2:         // NW-SE
                            line_begin = 0;
                            line_end = N_y * dimension * N_x;
                            line_incr = dimension * N_x;
                            col_begin = dimension * N_x - 1;
                            col_end = -1;
                            col_incr = -1;
                            break;
                        default:         // SE-NW
                            line_begin = (N_y-1) * dimension * N_x;
                            line_end = - dimension * N_x;
                            line_incr = - dimension * N_x;
                            col_begin = 0;
                            col_end = dimension * N_x;
                            col_incr = 1;
                    }
                    for (line = line_begin; line != line_end; line+= line_incr) {
                    for (col = col_begin; col != col_end; col+= col_incr) {
                        i = line + col;
			double sum = 0;
                        double diag_term;
                        
			int rest = (i%(dimension*N_x))%dimension;
			   
			if(i < dimension*N_x){
			    diag_term = cvmGet(M,i,0);
			    sum = cvmGet(M,i,1)*cvmGet(SOR, i+dimension*N_x,0);
			}
			else if(i >= dimension*N_x*(N_y-1)){
			    diag_term = cvmGet(M, i,0);
			    sum = cvmGet(M, i,1)*cvmGet(SOR, i-dimension*N_x,0);
			}
			else{
			    if(i%(dimension*N_x) == 0){
				diag_term = cvmGet(M, i,0);
				sum = cvmGet(M, i,1)*cvmGet(SOR, i+dimension,0);
			    }
			    else if((i+dimension)%(dimension*N_x)==0){
				diag_term = cvmGet(M, i,0);
				sum = cvmGet(M, i,1)*cvmGet(SOR, i-dimension,0);
			    }
			    else{
				int rest1 = i%(dimension*N_x);
				if(rest1 == 1){
				    diag_term = cvmGet(M, i,0);
				    sum = cvmGet(M, i,1)*cvmGet(SOR, i+dimension,0);
				}
				else if(rest1 == dimension*N_x - 1){
				    diag_term = cvmGet(M, i,0);
				    sum = cvmGet(M, i,1)*cvmGet(SOR, i-dimension,0);
				}
				else{
				    diag_term = cvmGet(M, i,rest);
				    int rest2 = rest1 % 2;
				    if(rest2 == 0){//u equation
#if DIVERGENCE_8
                                        sum =
                                            cvmGet(M, i,2)*cvmGet(SOR, i-dimension*N_x,0) +
                                            cvmGet(M, i,4)*cvmGet(SOR, i-dimension,0) +
                                            cvmGet(M, i,8)*cvmGet(SOR, i-dimension*(N_x-1),0) +
                                            cvmGet(M, i,9)*cvmGet(SOR, i-dimension*(N_x+1),0) +
                                            cvmGet(M, i,5)*cvmGet(SOR, i+dimension,0) +
                                            cvmGet(M, i,3)*cvmGet(SOR, i+dimension*N_x,0) +
                                            cvmGet(M, i,1)*cvmGet(SOR, i+1,0) +
                                            cvmGet(M, i,7)*cvmGet(SOR, i+dimension*(N_x-1),0) +
                                            cvmGet(M, i,6)*cvmGet(SOR, i+dimension*(N_x+1),0) ;
#else  // !DIVERGENCE_8
                                        sum =
                                            cvmGet(M, i,2)*cvmGet(SOR, i-dimension*N_x,0) +
                                            cvmGet(M, i,4)*cvmGet(SOR, i-dimension,0) +
                                            cvmGet(M, i,5)*cvmGet(SOR, i+dimension,0) +
                                            cvmGet(M, i,3)*cvmGet(SOR, i+dimension*N_x,0) +
                                            cvmGet(M, i,1)*cvmGet(SOR, i+1,0);
#endif  // !DIVERGENCE_8
	                                diag_term = cvmGet(M, i,0);
				    }
				    else{//v equation
#if DIVERGENCE_8
                                        sum =
                                            cvmGet(M, i,2)*cvmGet(SOR, i-dimension*N_x,0) +
                                            cvmGet(M, i,4)*cvmGet(SOR, i-dimension,0) +
                                            cvmGet(M, i,0)*cvmGet(SOR, i-1,0) +
                                            cvmGet(M, i,8)*cvmGet(SOR, i-dimension*(N_x-1),0) +
                                            cvmGet(M, i,9)*cvmGet(SOR, i-dimension*(N_x+1),0) +
                                            cvmGet(M, i,5)*cvmGet(SOR, i+dimension,0) +
                                            cvmGet(M, i,3)*cvmGet(SOR, i+dimension*N_x,0) +
                                            cvmGet(M, i,7)*cvmGet(SOR, i+dimension*(N_x-1),0) +
                                            cvmGet(M, i,6)*cvmGet(SOR, i+dimension*(N_x+1),0);
#else  // !DIVERGENCE_8
                                        sum =
                                            cvmGet(M, i,2)*cvmGet(SOR, i-dimension*N_x,0) +
                                            cvmGet(M, i,4)*cvmGet(SOR, i-dimension,0) +
                                            cvmGet(M, i,0)*cvmGet(SOR, i-1,0) +
                                            cvmGet(M, i,5)*cvmGet(SOR, i+dimension,0) +
                                            cvmGet(M, i,3)*cvmGet(SOR, i+dimension*N_x,0);
#endif  // !DIVERGENCE_8
				        diag_term = cvmGet(M, i,1);
				    }
				}
			    }
			}
                        double rold = cvmGet(SOR, i,0);
                        double b = cvmGet(B, i,0);
                        double rnew = (1.-omega_SOR)*rold + (omega_SOR/diag_term) * (b - sum);
                        //assert(fabs(rnew)<(N_x+N_y));
			cvmSet(SOR, i,0,Crop(rnew,MAX_INCR));
		    } // for (col
		    } // for (line
		    } // for (sub_iter_SOR
			 
		    norm_SOR = cvNorm(SOR_prec, SOR, CV_RELATIVE_L2);
		    cvCopy(SOR, SOR_prec);
		    iter_SOR ++;
		} // while ((norm_SOR ...
		cvCopy(dU, dU_prec);
		cvCopy(dV, dV_prec);
		for(i = 0; i < dimension*N_x*N_y; i+=dimension){
                    xl0 = (i%(dimension*N_x))/dimension;
                    yl0 = i/(dimension*N_x);
                    double du = cvmGet(SOR, i,0);
                    double dv = cvmGet(SOR, i + 1,0);
		    cvmSet(dU, yl0, xl0, Crop(du,MAX_INCR));
		    cvmSet(dV, yl0, xl0, Crop(dv,MAX_INCR));
		}
                if (OF_ZeroTopBorder) {
                    for(xl0 = 0; xl0 < N_x; xl0++){
                        cvmSet(dU, 0, xl0, 0);
                        cvmSet(dV, 0, xl0, 0);
                        cvmSet(dU, 1, xl0, 0);
                        cvmSet(dV, 1, xl0, 0);
                    }
                    for(yl0 = 0; yl0 < N_y*7/8; yl0++){
                        cvmSet(dU, yl0, 0, 0);
                        cvmSet(dV, yl0, 0, 0);
                        cvmSet(dU, yl0, N_x-1, 0);
                        cvmSet(dV, yl0, N_x-1, 0);
                        cvmSet(dU, yl0, 1, 0);
                        cvmSet(dV, yl0, 1, 0);
                        cvmSet(dU, yl0, N_x-2, 0);
                        cvmSet(dV, yl0, N_x-2, 0);
                        cvmSet(dU, yl0, 2, 0);
                        cvmSet(dV, yl0, 2, 0);
                        cvmSet(dU, yl0, N_x-3, 0);
                        cvmSet(dV, yl0, N_x-3, 0);
                    }
                }
		norm_inner = fmax(cvNorm(dU_prec, dU, CV_RELATIVE_L2), cvNorm(dV_prec, dV, CV_RELATIVE_L2));
		
		iter_inner++;
	    }
	      
	   
            if (OF_U_increment_FileName) {
                SaveResults(OF_U_increment_FileName, dU);
            }
	     
            if (OF_V_increment_FileName) {
                SaveResults(OF_V_increment_FileName, dV);
            }

	    cvCopy(U, U_prec);
	    cvCopy(V, V_prec);
            cvAdd(U, dU, U);
	    cvAdd(V, dV, V);
	     
	    norm = fmax(cvNorm(U_prec, U, CV_RELATIVE_L2), cvNorm(V_prec, V, CV_RELATIVE_L2));
	     
	    if (OF_U_FileName) {
                SaveResults(OF_U_FileName, U);
            }	     
	    if (OF_V_FileName) {
                SaveResults(OF_V_FileName, V);
            }
	    if (OF_disactivated_FileName) {
                SaveResults(OF_disactivated_FileName, disactivated_pixels, 0, 0., 255.);
            }	     
	 
	    iter_outer++;
	}

	if (logfile) {
	    //Coordinate maps update
	    cvAdd(MapIdentityX, U, MapX_Ilt);
	    cvAdd(MapIdentityY, V, MapY_Ilt);
	
	    //Images warping
	    // Images warping + Warped images gradient computation
	    // (Warped images are smoothed)
	    cvRemap(Ilt_pyramid[level], Iltw, MapX_Ilt, MapY_Ilt);
	    SmoothImage(Iltw);
            fprintf(logfile, "* iterations: %d\n", iter_outer);
            fprintf(logfile, "iter_SOR=%d<%d norm_SOR=%g<%g\n", iter_SOR, max_iter_SOR, norm_SOR, epsilon_SOR);
            fprintf(logfile, "iter_inner=%d<=%d norm_inner=%g<%g\n",iter_inner,max_iter_inner,norm_inner,epsilon_inner);
            fprintf(logfile, "iter_outer=%d<=%d norm=%g<%g\n", iter_outer, max_iter_outer, norm, epsilon_outer);
            fflush(logfile);
            for(yl0 = 0; yl0<N_y; yl0++) {
                for(xl0 = 0; xl0<N_x; xl0++) {
                    //Current coordinates of the followed point 
                    double xlt = cvmGet(MapX_Ilt,yl0,xl0);
                    double ylt = cvmGet(MapY_Ilt,yl0,xl0);
                    //Pixels out must not be taken into account in the energy data term
                    int pixel_is_disactivated = cvmGet8u(disactivated_pixels,yl0,xl0);

                    //is the point visible at time t+1 ?
                    if(!pixel_is_disactivated &&                          
                       (xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h  ||
                        ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h)){
                        //printf("disactivate %d,%d\n", x, yl0);
                        cvmSet8u(disactivated_pixels,yl0,xl0,127);
                        pixel_is_disactivated = 1;
                    }
                }
            }
            double Efl, Edata, Esmooth, Eintensity, Egradient;
            EnergyComputation(Il0_pyramid[level], 0, Iltw, 0,
                              U, V, 0, 0,
                              disactivated_pixels, 0, 0, 0,
                              N_x, N_y, &Edata, &Esmooth, gamma, 1.0, 1.0, 1, 0, 0, 0, &Efl, 0, 0, 0, 0, 0, 0, 0, &Eintensity, &Egradient);
            fprintf(logfile, "* final energy:\n");
            fprintf(logfile, "Edata = %g Esmooth = %g Ed/Es = %g (alpha=%g)\n", Edata, Esmooth, Edata/Esmooth, alpha);
            fprintf(logfile, "Edata breakdown (squared): Eintensity = %g, Egradient=%g Ei/Eg=%g (gamma=%g)\n", Eintensity, Egradient,  Egradient != 0 ? Eintensity/Egradient : 0., gamma);
            fflush(logfile);
        }

	//Let's Clean
	cvReleaseMat(&Iltw_X);
	cvReleaseMat(&Iltw_Y);
	cvReleaseMat(&Il0_X);
	cvReleaseMat(&Il0_Y);
	cvReleaseMat(&Iltw_XX);
	cvReleaseMat(&Iltw_XY);
	cvReleaseMat(&Iltw_YY);
	cvReleaseMat(&derivative_tmp);

        cvReleaseMat(&divergence_coefficients);

	cvReleaseMat(&Iltw);
	cvReleaseMat(&MapX_Ilt);
	cvReleaseMat(&MapY_Ilt);
	cvReleaseMat(&MapX_Ilt_inner);
	cvReleaseMat(&MapY_Ilt_inner);
	cvReleaseMat(&MapIdentityX);
	cvReleaseMat(&MapIdentityY);
	cvReleaseMat(&M);
	cvReleaseMat(&B);
        cvReleaseMat(&disactivated_pixels);
	cvReleaseMat(&U_prec); 
	cvReleaseMat(&V_prec);
	     

	cvReleaseMat(&dU);
	cvReleaseMat(&dV);
	cvReleaseMat(&dU_X);
	cvReleaseMat(&dU_Y);
	cvReleaseMat(&dV_X);
	cvReleaseMat(&dV_Y);

	 

	cvReleaseMat(&U_X);
	cvReleaseMat(&U_Y);
	cvReleaseMat(&V_X);
	cvReleaseMat(&V_Y);
	 
	cvReleaseMat(&dU_prec);
	cvReleaseMat(&dV_prec);
	 
	cvReleaseMat(&SOR);
	cvReleaseMat(&SOR_prec);

	//if this is not the last level, images must be upscaled before iterations
	if (level != (pyramid_level_end-1)) {
            const CvSize next_level_size = cvGetSize(Il0_pyramid[level - 1]);
	    CvMat *Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
	    cvResize(U, Interpolated_Data,CV_INTER_LINEAR);
            double uscale = (double)Interpolated_Data->width / U->width;
	    cvReleaseMat(&U);
	    U = Interpolated_Data;
            SmoothData(U);
	    cvConvertScale(U, U, uscale);
	   
	    Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
	    cvResize(V, Interpolated_Data,CV_INTER_LINEAR);
            double vscale = (double)Interpolated_Data->height / V->height;
	    cvReleaseMat(&V);
	    V = Interpolated_Data;
            SmoothData(V);
	    cvConvertScale(V, V, vscale);
	}
    }//  for(int level = pyramid_level_start - 1; level >= pyramid_level_end - 1; level--){
     
 
    //cvResize(U, U_end, CV_INTER_AREA);
    //cvResize(V, V_end, CV_INTER_AREA);
    cvCopy(U, U_end);
    cvCopy(V, V_end);
    cvReleaseMat(&U);
    cvReleaseMat(&V);
}

/**************************************************Scene Flow Estimation*****************/
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
               const int methode)       //the chosen method 
{
    int N_x,N_y;
     
    CvMat *Dt=0, *D0=0, *U=0, *V=0;

    int dimension = 0;
    int doleftflow = 0;
    int dorightflow = 0;
    int dodisparity = 0;
    int dodisparity0 = 0;
    int xl0, yl0;
    
    if(methode == 0){
        // left flow + disparity at t0 and t
        doleftflow = 1;
        dodisparity = 1;
        dodisparity0 = 1;
        dimension = 4;
    }
    else if (methode == 1) {
        // left flow + disparity at time t
        doleftflow = 1;
	//dorightflow = 1;
        dodisparity = 1;
        dimension = 3;
    }
    else if(methode == 2){
        // left flow only
        doleftflow = 1;
        dimension = 2;
    }
    else if(methode == 3){
        doleftflow = 1;
        dorightflow = 1;
        dimension = 3;
    }
    else if (methode == 4) {
        //disparity at time t
        dodisparity = 1;
        dimension = 1;
    }
    else if (methode == 5) {
        //disparity at time t0
        dodisparity0 = 1;
        dimension = 1;
    }

    //assert(dimension == doleftflow*2+dorightflow+dodisparity+dodisparity0);
    
    assert(pyramid_level_start >= pyramid_level_end);
    assert(pyramid_level_end >= 1);
    assert(pyramid_level_start <= pyramid_levels);
    int pyramidtype = CV_MAT_TYPE(Il0_pyramid[pyramid_level_start-1]->type);
    assert(pyramidtype == CV_8UC1 || pyramidtype == CV_32FC1);
    if (doleftflow) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], U_start));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], V_start));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], U_end));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], V_end));
        assert(Ilt_pyramid);
        assert(CV_ARE_TYPES_EQ(Il0_pyramid[pyramid_level_start-1],Ilt_pyramid[pyramid_level_start-1]));
    }
    if (dodisparity0 || dodisparity) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], D0_start));
    }
    if (dodisparity0) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], D0_end));
        assert(Ir0_pyramid);
        assert(CV_ARE_TYPES_EQ(Il0_pyramid[pyramid_level_start-1],Ir0_pyramid[pyramid_level_start-1]));
    }
    if (dodisparity || doleftflow) {
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_start-1], Dt_start));
    }
    if (dodisparity) {
        assert(Irt_pyramid);
        assert(CV_ARE_TYPES_EQ(Il0_pyramid[pyramid_level_start-1],Irt_pyramid[pyramid_level_start-1]));
        assert(CV_ARE_SIZES_EQ(Il0_pyramid[pyramid_level_end-1], Dt_end));
    }

    //intial optical flow is set to the lowest resolution
    {
        CvSize size_start = cvGetSize(Il0_pyramid[pyramid_level_start-1]);
        
        if (doleftflow) {
            U = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
            V = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
            cvCopy(U_start, U);
            cvCopy(V_start, V);
        }
        if (dodisparity0 || dodisparity) {
            D0 = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
            cvCopy(D0_start, D0);
//// a single square, used for testing diffusion
            //cvSetZero(D0);
            //cvRectangle(D0,
            //            cvPoint(D0->width /2-50,D0->width /2-50),
            //            cvPoint(D0->width /2+50,D0->width /2+50),
            //            cvScalar(10), CV_FILLED);
        }
        if (dodisparity || doleftflow) {
            Dt = cvCreateMat(size_start.height, size_start.width, CV_32FC1);
            cvCopy(Dt_start, Dt);
        }
    }
    
    //Loop over the resolution levels
    for(int level = pyramid_level_start - 1; level >= pyramid_level_end - 1; level--){
        if (SF_Il0_FileName) {
            SaveImage(SF_Il0_FileName,  Il0_pyramid[level]);
        }
        if (SF_Ilt_FileName && Ilt_pyramid && Ilt_pyramid[level]) {
            SaveImage(SF_Ilt_FileName,  Ilt_pyramid[level]);
        }
        if (SF_Ir0_FileName && Ir0_pyramid && Ir0_pyramid[level]) {
            SaveImage(SF_Ir0_FileName,  Ir0_pyramid[level]);
        }
        if (SF_Irt_FileName && Irt_pyramid && Irt_pyramid[level]) {
            SaveImage(SF_Irt_FileName,  Irt_pyramid[level]);
        }
        if (doleftflow) {
            assert(Il0_pyramid[level] && Ilt_pyramid[level]);
            assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], Ilt_pyramid[level]));
            assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], U));
            assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], V));
        }
        if (dodisparity0 || dodisparity) {
            assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], D0));
        }
        if (dodisparity || doleftflow) {
            assert(CV_ARE_SIZES_EQ(Il0_pyramid[level], Dt));
        }

        CvSize level_size = cvGetSize(Il0_pyramid[level]);
	//Size of the current images
	N_x = level_size.width;
	N_y = level_size.height;

	//Maps used for the images warping : Initialization
	CvMat *MapIdentityX = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *MapIdentityY = cvCreateMat(N_y, N_x, CV_32FC1);
	for(yl0=0; yl0 < N_y; yl0++){
	    for(xl0=0; xl0 < N_x; xl0++){
		cvmSet(MapIdentityX, yl0, xl0, xl0);
		cvmSet(MapIdentityY, yl0, xl0, yl0);
	    }
	}

        //Warped Images
        CvMat *Iltw = 0;
        CvMat *Irtw = 0;
        CvMat *Ir0w = 0;
        //Maps for warping
        CvMat *MapX_Ilt = 0;
        CvMat *MapY_Ilt = 0;
	CvMat *MapX_Ilt_inner = 0;
	CvMat *MapY_Ilt_inner = 0;
	CvMat *MapX_Irt = 0;
	CvMat *MapY_Irt = 0;
	CvMat *MapX_Irt_inner = 0;
	CvMat *MapY_Irt_inner = 0;
	CvMat *MapX_Ir0 = 0;
	CvMat *MapY_Ir0 = 0;
	CvMat *MapX_Ir0_inner = 0;
	CvMat *MapY_Ir0_inner = 0;
        if (doleftflow || dodisparity) {
            Iltw = cvCreateMat(N_y, N_x, pyramidtype);
            MapX_Ilt =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Ilt =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapX_Ilt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Ilt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
        }
        if (dorightflow || dodisparity) {
            Irtw = cvCreateMat(N_y, N_x, pyramidtype);
            MapX_Irt =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Irt =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapX_Irt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Irt_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
        }
        if (dorightflow || dodisparity0) {
            Ir0w = cvCreateMat(N_y, N_x, pyramidtype);
            MapX_Ir0 =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Ir0 =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapX_Ir0_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
            MapY_Ir0_inner =  cvCreateMat(N_y, N_x, CV_32FC1);
        }
     

	
	//Images gradients computation
        CvMat *derivative_tmp = 0;
        if (pyramidtype == CV_8UC1)
            derivative_tmp = cvCreateMat(N_y, N_x, CV_16SC1);
	CvMat *Il0_X = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Il0_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	CvMat *Iltw_X = 0;
	CvMat *Iltw_Y = 0;
	CvMat *Irtw_X = 0;
	CvMat *Irtw_Y = 0;
	CvMat *Ir0w_X = 0;
	CvMat *Ir0w_Y = 0;
	// Second derivatives
	CvMat *Iltw_XX = 0;
	CvMat *Iltw_XY = 0;
	CvMat *Iltw_YY = 0;
	CvMat *Irtw_XX = 0;
	CvMat *Irtw_XY = 0;
	CvMat *Irtw_YY = 0;
	CvMat *Ir0w_XX = 0;
	CvMat *Ir0w_XY = 0;
	CvMat *Ir0w_YY = 0;

        if (doleftflow || dodisparity) {
            Iltw_X       = cvCreateMat(N_y, N_x, CV_32FC1);
            Iltw_Y       = cvCreateMat(N_y, N_x, CV_32FC1);
            Iltw_XX       = cvCreateMat(N_y, N_x, CV_32FC1);
            Iltw_XY       = cvCreateMat(N_y, N_x, CV_32FC1);
            Iltw_YY       = cvCreateMat(N_y, N_x, CV_32FC1);
        }
        if (dodisparity0 || dorightflow) {
            Ir0w_X = cvCreateMat(N_y, N_x, CV_32FC1);
            Ir0w_Y = cvCreateMat(N_y, N_x, CV_32FC1);
            Ir0w_XX = cvCreateMat(N_y, N_x, CV_32FC1);
            Ir0w_XY = cvCreateMat(N_y, N_x, CV_32FC1);
            Ir0w_YY = cvCreateMat(N_y, N_x, CV_32FC1);
        }
        if (dodisparity || dorightflow) {
            Irtw_X      = cvCreateMat(N_y, N_x, CV_32FC1);
            Irtw_Y      = cvCreateMat(N_y, N_x, CV_32FC1);
            Irtw_XX      = cvCreateMat(N_y, N_x, CV_32FC1);
            Irtw_XY      = cvCreateMat(N_y, N_x, CV_32FC1);
            Irtw_YY      = cvCreateMat(N_y, N_x, CV_32FC1);
        }
        // compute gradients of the only non-warped image
        assert(Il0_X && Il0_Y);
        Derivative(Il0_pyramid[level], 1, 0, derivative_tmp, Il0_X);
        Derivative(Il0_pyramid[level], 0, 1, derivative_tmp, Il0_Y);
	
	    
	//M is the matrix of the final linear system and B the second member of this system
#if DIVERGENCE_8
	CvMat *M = cvCreateMat(dimension * N_x * N_y, dimension + 8, CV_32FC1); 
	
#else	  // !DIVERGENCE_8
	CvMat *M = cvCreateMat(dimension * N_x * N_y, dimension + 4, CV_32FC1); 
	
#endif  // !DIVERGENCE_8

#ifdef M_IMAGE
	CvMat *M_image;
	if(doleftflow && dodisparity0 && dodisparity){
            M_image = cvCreateMat(dimension * N_x * N_y, dimension * N_x * N_y, CV_32FC1); 
            cvSetZero(M_image);
	}
	else
            M_image = cvCreateMat(dimension, dimension, CV_32FC1); 
#endif

        CvMat *B = cvCreateMat(dimension * N_x * N_y,1,CV_32FC1);

        CvMat *disactivated_pixels_leftflow = 0;
        CvMat *disactivated_pixels_rightflow = 0;
        CvMat *disactivated_pixels_stereoinit = 0;
        CvMat *disactivated_pixels_stereo = 0;
        if (doleftflow || (dorightflow && dodisparity && dodisparity0)) {
            disactivated_pixels_leftflow = cvCreateMat(N_y, N_x, CV_8UC1);
        }
        if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
            disactivated_pixels_rightflow = cvCreateMat(N_y, N_x, CV_8UC1);
        }
        if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
            disactivated_pixels_stereoinit = cvCreateMat(N_y, N_x, CV_8UC1);
        }
        if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
            disactivated_pixels_stereo = cvCreateMat(N_y, N_x, CV_8UC1);
        }
        
        //divergence coefficients
	CvMat *divergence_coefficients = cvCreateMat(N_y, N_x, CV_32FC1);

        CvMat *Dt_X,*Dt_Y, *D0_X,*D0_Y,*U_X, *U_Y, *V_X, *V_Y;
        CvMat *dDt, *dD0, *dU, *dV;
        CvMat* dDt_prec, *dD0_prec,  *dU_prec,  *dV_prec;
        CvMat *dDt_X,*dDt_Y, *dD0_X,*dD0_Y, *dU_X, *dU_Y, *dV_X, *dV_Y;
        CvMat *Dt_prec, *D0_prec, *U_prec, *V_prec;

	//left flow
	if(doleftflow){
            //saving the initial flow images
            if (SF_U_FileName) {
                SaveResults(SF_U_FileName, U);
            }
            if (SF_V_FileName) {
                SaveResults(SF_V_FileName, V);
            }
	
            U_prec = cvCreateMat(N_y, N_x, CV_32FC1);
            V_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	   
            dU = cvCreateMat(N_y, N_x, CV_32FC1);
            dV = cvCreateMat(N_y, N_x, CV_32FC1);
            dU_X = cvCreateMat(N_y, N_x, CV_32FC1);
            dU_Y = cvCreateMat(N_y, N_x, CV_32FC1);
            dV_X = cvCreateMat(N_y, N_x, CV_32FC1);
            dV_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	     
            U_X = cvCreateMat(N_y, N_x, CV_32FC1);
            U_Y = cvCreateMat(N_y, N_x, CV_32FC1);
            V_X = cvCreateMat(N_y, N_x, CV_32FC1);
            V_Y = cvCreateMat(N_y, N_x, CV_32FC1);

            dU_prec = cvCreateMat(N_y, N_x, CV_32FC1);
            dV_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	}
	
	if(dodisparity) {
	    if (SF_Dt_FileName) {
                SaveResults(SF_Dt_FileName, Dt);
            }
	     
	    Dt_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	    dDt = cvCreateMat(N_y, N_x, CV_32FC1);
	    dDt_X = cvCreateMat(N_y, N_x, CV_32FC1);
	    dDt_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	    Dt_X = cvCreateMat(N_y, N_x, CV_32FC1);
	    Dt_Y = cvCreateMat(N_y, N_x, CV_32FC1);
            dDt_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	}
	if(dodisparity0 || dodisparity) {
	    D0_X = cvCreateMat(N_y, N_x, CV_32FC1);
	    D0_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	    D0_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	}
	if(dodisparity0) {
            if (SF_D0_FileName) {
                SaveResults(SF_D0_FileName, D0);
            }	   
	    dD0 = cvCreateMat(N_y, N_x, CV_32FC1);
	    dD0_X = cvCreateMat(N_y, N_x, CV_32FC1);
	    dD0_Y = cvCreateMat(N_y, N_x, CV_32FC1);
	    dD0_prec = cvCreateMat(N_y, N_x, CV_32FC1);
	}


	//For the final linear system and SOR iterations
        CvMat *SOR = cvCreateMat(dimension*N_x*N_y,1, CV_32FC1);	
        CvMat *SOR_prec = cvCreateMat(dimension*N_x*N_y,1,CV_32FC1);

	//Main fixed point iterations, loop over optical flow/disparity values
        double norm = 0., norm_inner = 0., norm_SOR = 0.;
        int iter_SOR = 0, iter_inner = 0, iter_outer = 0;
        int first_iteration = 1;
	while((norm > epsilon_outer && iter_outer <= max_iter_outer) || iter_outer < 2 || first_iteration == 1){
	    first_iteration = 0;
	    
	    //Maps update
	    if (dorightflow || dodisparity0){
		cvAdd(MapIdentityX, D0, MapX_Ir0);
		cvCopy(MapIdentityY,  MapY_Ir0);
	    }
            if (doleftflow || dodisparity) {
		cvAdd(MapIdentityX, U, MapX_Ilt);
		cvAdd(MapIdentityY, V, MapY_Ilt);
            }
            if (dorightflow || dodisparity) {
		cvAdd(MapIdentityX, U, MapX_Irt);
		cvAdd(MapX_Irt, Dt, MapX_Irt);
		cvAdd(MapIdentityY, V, MapY_Irt);
            }

#if DISABLE_PIXEL_DISACTIVATION
#warning "no disactivation of occluded pixels"
            if (doleftflow || (dorightflow && dodisparity && dodisparity0)) {
                cvSetZero(disactivated_pixels_leftflow);
            }
            if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
                cvSetZero(disactivated_pixels_rightflow);
            }
            if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
                cvSetZero(disactivated_pixels_stereoinit);
            }
            if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
                cvSetZero(disactivated_pixels_stereo);
            }
#else //!DISABLE_PIXEL_DISACTIVATION
            // Initialize occlusion maps, for each of the data terms:
            // - first, compute stereo disparity the destination image, using Z-buffering
            // - then, remap those disparities to the left image at time t
            // - add disparity tolerance (e.g. 1.) to the remapped disparity
            // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
            
	    CvMat *D_warped = cvCreateMat(N_y, N_x, CV_32FC1);
            CvMat *D_backwarped = cvCreateMat(N_y, N_x, CV_32FC1);
            CvMat *D_rewarped = cvCreateMat(N_y, N_x, CV_32FC1);


	    if (Dt && (doleftflow || (dorightflow && dodisparity && dodisparity0))) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Ilt, MapY_Ilt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Ilt, MapY_Ilt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_leftflow, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_leftflow, 127);
            }
            if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(D0, D_warped, MapX_Ir0, MapY_Ir0, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Ir0, MapY_Ir0, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(D0, D_backwarped, disactivated_pixels_stereoinit, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_stereoinit, 127);
//#warning desactivation totale
                //cvSet(disactivated_pixels_stereoinit, cvScalar(127));
           }
            // not so sure about how to compute right flow occlusion...
            if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Irt, MapY_Irt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Irt, MapY_Irt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_rightflow, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_rightflow, 127);
            }
            // same as right flow
            if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Irt, MapY_Irt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Irt, MapY_Irt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_stereo, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_stereo, 127);
            }
            cvReleaseMat(&D_warped);
            cvReleaseMat(&D_backwarped);
            cvReleaseMat(&D_rewarped);
#endif //!DISABLE_PIXEL_DISACTIVATION
	    // Images warping + Warped images gradient computation
	    // (Warped images are smoothed)
	    if (doleftflow || dodisparity) {
		cvRemap(Ilt_pyramid[level], Iltw, MapX_Ilt, MapY_Ilt);
		SmoothImage(Iltw);
                if (SF_Ilt_Warped_FileName) {
                    SaveImage(SF_Ilt_Warped_FileName, Iltw);
                }
                assert(Iltw && Iltw_X && Iltw_Y && Iltw_XX && Iltw_XY && Iltw_YY);
		Derivative(Iltw, 1, 0, derivative_tmp, Iltw_X);
		Derivative(Iltw, 0, 1, derivative_tmp, Iltw_Y);
		Derivative(Iltw, 2, 0, derivative_tmp, Iltw_XX);
		Derivative(Iltw, 1, 1, derivative_tmp, Iltw_XY);
		Derivative(Iltw, 0, 2, derivative_tmp, Iltw_YY);
	    }
	    if (dodisparity0 || dorightflow) {
		cvRemap(Ir0_pyramid[level], Ir0w, MapX_Ir0, MapY_Ir0);
		SmoothImage(Ir0w);
                if (SF_Ir0_Warped_FileName) {
                    SaveImage(SF_Ir0_Warped_FileName, Ir0w);
                }
                assert(Ir0w && Ir0w_X && Ir0w_Y && Ir0w_XX && Ir0w_XY && Ir0w_YY);
		Derivative(Ir0w, 1, 0, derivative_tmp, Ir0w_X);
		Derivative(Ir0w, 0, 1, derivative_tmp, Ir0w_Y);
		Derivative(Ir0w, 2, 0, derivative_tmp, Ir0w_XX);
		Derivative(Ir0w, 1, 1, derivative_tmp, Ir0w_XY);
		Derivative(Ir0w, 0, 2, derivative_tmp, Ir0w_YY);
	    }
	    if (dodisparity || dorightflow) {
		cvRemap(Irt_pyramid[level], Irtw, MapX_Irt, MapY_Irt);
		SmoothImage(Irtw);
                if (SF_Irt_Warped_FileName) {
                    SaveImage(SF_Irt_Warped_FileName, Irtw);
                }
                assert(Irtw && Irtw_X && Irtw_Y && Irtw_XX && Irtw_XY && Irtw_YY);
		Derivative(Irtw, 1, 0, derivative_tmp, Irtw_X);
		Derivative(Irtw, 0, 1, derivative_tmp, Irtw_Y);
		Derivative(Irtw, 2, 0, derivative_tmp, Irtw_XX);
		Derivative(Irtw, 1, 1, derivative_tmp, Irtw_XY);
		Derivative(Irtw, 0, 2, derivative_tmp, Irtw_YY);
	    }
            
            //Flow gradients computation
	    if(doleftflow){
		GradientFast_32f(U, U_X, U_Y);
		GradientFast_32f(V, V_X, V_Y);
	    }
	    if(dodisparity){
		GradientFast_32f(Dt, Dt_X, Dt_Y);
	    }
	    if(dodisparity0 || dodisparity){
		GradientFast_32f(D0, D0_X, D0_Y);
	    }

	    //Increments initialization
	    if(doleftflow){
		cvSetZero(dU);
		cvSetZero(dV);
	    }
	   
	    if(dodisparity){
		cvSetZero(dDt);
	    }
	    if(dodisparity0){
		cvSetZero(dD0);
	    }
	  
            norm_inner = 0.;
	    first_iteration = 1;
	    iter_inner = 0;
	    //secondary fixed point iterations
	    while((norm_inner > epsilon_inner && iter_inner <= max_iter_inner) || first_iteration == 1){
		first_iteration = 0;
                
		//Coordinates maps update
                if (dorightflow || dodisparity0){
		    cvAdd(MapX_Ir0, dD0, MapX_Ir0_inner);
		    cvCopy(MapY_Ir0, MapY_Ir0_inner);
                }
                if (doleftflow || dodisparity) {
		    cvAdd(MapX_Ilt, dU, MapX_Ilt_inner);
		    cvAdd(MapY_Ilt, dV, MapY_Ilt_inner);
                }
                if (dorightflow || dodisparity) {
                    if (doleftflow)
                        cvAdd(MapX_Irt, dU, MapX_Irt_inner);
                    cvAdd(MapX_Irt_inner, dDt, MapX_Irt_inner);
                    if (doleftflow)
                        cvAdd(MapY_Irt, dV, MapY_Irt_inner);
                }
		
		//Flow increments gradients computation
		if(doleftflow){
		    GradientFast_32f(dU, dU_X, dU_Y);
		    GradientFast_32f(dV, dV_X, dV_Y);
		}
		if(dodisparity){
		    GradientFast_32f(dDt, dDt_X, dDt_Y);
		}
		if(dodisparity0){
		    GradientFast_32f(dD0, dD0_X, dD0_Y);
		}
		for(yl0 = 0; yl0 < N_y; yl0++){
		    for(xl0 = 0; xl0 < N_x; xl0++){
			double dd0_X = 0.;
			double dd0_Y = 0.;
			double d0_X = 0.;
			double d0_Y = 0.;
			double s2 = 0.;
			double s3 = 0.;
			if (doleftflow) {
			    double du_X = cvmGet(dU_X, yl0, xl0);
			    double du_Y = cvmGet(dU_Y, yl0, xl0);
			    double u_X = cvmGet(U_X, yl0, xl0);
			    double u_Y = cvmGet(U_Y, yl0, xl0);
			    double dv_X = cvmGet(dV_X, yl0, xl0);
			    double dv_Y = cvmGet(dV_Y, yl0, xl0);
			    double v_X = cvmGet(V_X, yl0, xl0);
			    double v_Y = cvmGet(V_Y, yl0, xl0);
                            s2 += (square(u_X + du_X) + square(u_Y + du_Y) +
                                   square(v_X + dv_X) + square(v_Y + dv_Y));
			}
		
                        if (dodisparity || dodisparity0) {
                            d0_X = cvmGet(D0_X, yl0, xl0);
			    d0_Y = cvmGet(D0_Y, yl0, xl0);
                        }
			if (dodisparity0) {
			    dd0_X = cvmGet(dD0_X, yl0, xl0);
			    dd0_Y = cvmGet(dD0_Y, yl0, xl0);
                            s2 += mu*(square(d0_X + dd0_X) + square(d0_Y + dd0_Y));
			    s3 = s2;
			}
                        if (dodisparity) {
			    double ddt_X = cvmGet(dDt_X, yl0, xl0);
			    double ddt_Y = cvmGet(dDt_Y, yl0, xl0);
			    double dt_X = cvmGet(Dt_X, yl0, xl0);
			    double dt_Y = cvmGet(Dt_Y, yl0, xl0);
                            s2 += lambda*(square(dt_X + ddt_X - d0_X - dd0_X) +
                                          square(dt_Y + ddt_Y - d0_Y - dd0_Y));
                        }

			double a = DPsiSmooth(s2);
			cvmSet(divergence_coefficients, yl0, xl0, a);
			
			
		    }
		}
		  
		//Computing M and B for the final linear system 
		cvSetZero(M);
		cvSetZero(B);
		
                int i;
		//The dimension*N_x first lines in M correspond to the points on the y = 0 on the image
		int inf = 0;
                int sup = dimension*N_x;
		for (i = inf; i < sup; i++) {
                    cvmSet(M, i, 0, -1./h); 
		    cvmSet(M, i, 1,  1./h);	

#ifdef M_IMAGE
		    if(doleftflow && dodisparity0 && dodisparity){
                        cvmSet(M_image, i, i, -1./h); 
                        cvmSet(M_image, i, i+dimension*N_x,  1./h);
		    }
#endif

		}
		//Points on the lower horizontal side of the image      
                inf = dimension*N_x*(N_y-1);
                sup = dimension*N_x*N_y;
		for (i = inf; i < sup; i++) {
		    cvmSet(M, i, 0,  1./h); 
		    cvmSet(M, i, 1, -1./h);

#ifdef M_IMAGE
                    if(doleftflow && dodisparity0 && dodisparity){
                        cvmSet(M_image, i, i, 1./h); 
                        cvmSet(M_image, i, i+dimension*N_x,  -1./h);
                    }
#endif

		}

                inf = dimension*N_x;
                sup = dimension*N_x*(N_y-1)-1;
		
                //We fill in the lines of the matrix M corresponding to images pixels except points on the upper and lower horizontal side
		for(i = inf; i <= sup; i+=dimension){
		    yl0 = i/(dimension*N_x);
		    xl0 = (i % (inf))/dimension;
           
		    if(xl0 == 0){ //if point on the left side of image
			for(int d=0; d<dimension; d++) {
                            cvmSet(M,i+d,0,-1./h);
			    cvmSet(M,i+d,1, 1./h);

#ifdef M_IMAGE
			    if(doleftflow && dodisparity0 && dodisparity){
                                cvmSet(M_image, i+d, i+d, -1./h); 
                                cvmSet(M_image, i+d, i+d + dimension,  1./h);
			    }
#endif

			}
		    }
		    else if(xl0 == (N_x-1)){//if point on the right side
                        for(int d=0; d<dimension; d++) {
			    cvmSet(M,i+d,0, 1./h);
			    cvmSet(M,i+d,1,-1./h);  

#ifdef M_IMAGE
			    if(doleftflow && dodisparity0 && dodisparity){
                                cvmSet(M_image, i+d, i+d, -1./h); 
                                cvmSet(M_image, i+d, i+d + dimension,  1./h);
			    }
#endif

			}
		    }
		    else{
                        //Pixel inside the integration domain 
                        if(methode == 0){
                            double il0 = 0.;
                            double il0_X = 0., il0_Y = 0.;
                            double ir0 = 0.;
                            double ir0_X = 0., ir0_Y = 0.;
                            double ir0w_X = 0., ir0w_Y = 0.;
                            double ir0w_XX = 0., ir0w_XY = 0., ir0w_YY = 0.;
                            double ir0w_X_X = 0., ir0w_X_Y = 0.;
                            double ir0w_Y_X = 0., ir0w_Y_Y = 0.;
                            double ilt = 0.;
                            double ilt_X = 0., ilt_Y = 0.;
                            double iltw_X = 0., iltw_Y = 0.;
                            double iltw_XX = 0., iltw_XY = 0., iltw_YY = 0.;
                            double iltw_X_X = 0., iltw_X_Y = 0.;
                            double iltw_Y_X = 0., iltw_Y_Y = 0.;
                            double irt = 0.;
                            double irt_X = 0., irt_Y = 0.;
                            double irtw_X = 0., irtw_Y = 0.;
                            double irtw_XX = 0., irtw_XY = 0., irtw_YY = 0.;
                            double irtw_X_X = 0., irtw_X_Y = 0.;
                            double irtw_Y_X = 0., irtw_Y_Y = 0.;
                            double du = 0, dv = 0, dd0 = 0, ddt = 0;
                            //interpolated divergence coefficients
                            double b_f_1, b_f_2, b_f_3, b_f_4;
			    
			    // Half pixel interpolation for the divergence coefficients 
                            divergence_coefficients_interpolate(divergence_coefficients, yl0, xl0, &b_f_1, &b_f_2, &b_f_3, &b_f_4);
			   

			    // gradients of the functions used for image warping
			    double uw_X = cvmGet(U_X, yl0, xl0);
			    double uw_Y = cvmGet(U_Y, yl0, xl0);
			    double vw_X = cvmGet(V_X, yl0, xl0);
			    double vw_Y = cvmGet(V_Y, yl0, xl0);
			    double dtw_X = cvmGet(Dt_X, yl0, xl0);
			    double dtw_Y = cvmGet(Dt_Y, yl0, xl0);
			    double d0w_X = cvmGet(D0_X, yl0, xl0);
			    double d0w_Y = cvmGet(D0_Y, yl0, xl0);

#ifdef DEBUG_PIXEL
                            if(i == ((DEBUG_PIXEL_X+DEBUG_PIXEL_Y*N_x)*dimension)) {
                                debug_me();
                            }
#endif
                            
			    //for the 8 divergence computation
#if DIVERGENCE_8
			    double b_f_5, b_f_6, b_f_7, b_f_8;
			    //For the 8-divergence
			    double alpha_p_u, alpha_p_v, alpha_d_u, alpha_d_v, alpha_p_dprime, alpha_d_dprime, alpha_p_d, alpha_d_d;
    
			    //U and V Gradients angle with the X axis
			    double u_X = uw_X;
			    double u_Y = uw_Y;
			    double v_X = vw_X;
			    double v_Y = vw_Y;
                            if (doleftflow) {
                                u_X += cvmGet(dU_X, yl0, xl0);
                                u_Y += cvmGet(dU_Y, yl0, xl0);
                                v_X += cvmGet(dV_X, yl0, xl0);
                                v_Y += cvmGet(dV_Y, yl0, xl0);
                            }
			    //D and D0 Gradients angle with the X axis
			    double d0_X = d0w_X;
			    double d0_Y = d0w_Y;
                            if (dodisparity0) {
                                d0_X += cvmGet(dD0_X, yl0, xl0);
                                d0_Y += cvmGet(dD0_Y, yl0, xl0);
                            }
			    double dt_X = dtw_X;
			    double dt_Y = dtw_Y;
                            if (dodisparity) {
                                dt_X += cvmGet(dDt_X, yl0, xl0);
                                dt_Y += cvmGet(dDt_Y, yl0, xl0);
                            }
			    //Coefficients computation for 8-divergence in u and v
			    // must be 1 when the gradient is aligned with axes, 0 when diagonal
			    //We check if gradients are 0
			    if(u_X == 0. && u_Y == 0.) //We take constant coefficients
                                //if(1)
                                alpha_p_u = 0.5;
			    else{
                                if (u_X == 0)
                                    alpha_p_u = 1;
                                else
                                    alpha_p_u = fabs(fabs(atan(u_Y/u_X))/M_PI_4-1);
			    }
			    if(v_X == 0. && v_Y == 0.) //We take constant coefficients
                                //if(1)
                                alpha_p_v = 0.5;
			    else{
                                if (v_X == 0)
                                    alpha_p_v = 1;
                                else
                                    alpha_p_v = fabs(fabs(atan(v_Y/v_X))/M_PI_4-1);
			    }
			    
			    if(dt_X == 0. && dt_Y == 0.) //We take constant coefficients
                                //if(1)
                                alpha_p_dprime = 0.5;
			    else{
                                if (dt_X == 0)
                                    alpha_p_dprime = 1;
                                else
                                    alpha_p_dprime = fabs(fabs(atan(dt_Y/dt_X))/M_PI_4-1);
			    }
			    if(d0_X == 0. && d0_Y == 0.) //We take constant coefficients
                                //if(1)
                                alpha_p_d = 0.5;
			    else{
                                if (d0_X == 0)
                                    alpha_p_d = 1;
                                else
                                    alpha_p_d = fabs(fabs(atan(d0_Y/d0_X))/M_PI_4-1);
			    }

			    //The validity constraint for the directional weights for the divergence 
			    alpha_d_u = (1.-alpha_p_u)/2.;
			    alpha_d_v = (1.-alpha_p_v)/2.;
			    alpha_d_dprime = (1.-alpha_p_dprime)/2.;
			    alpha_d_d = (1.-alpha_p_d)/2.;

			    // Half pixel interpolation for the divergence coefficients 
			    divergence_coefficients_interpolate_diag(divergence_coefficients, yl0, xl0, &b_f_5, &b_f_6, &b_f_7, &b_f_8);
#endif  // DIVERGENCE_8
			
			    //Pixels out must not be taken into account in the energy data term
			    int pixel_doleftflow = 0;
			    int pixel_dorightflow = 0;
			    int pixel_dostereoinit = 0;
			    int pixel_dostereo = 0;
                            if (disactivated_pixels_leftflow)
                                pixel_doleftflow = !cvmGet8u(disactivated_pixels_leftflow,yl0,xl0);
                            if (disactivated_pixels_rightflow)
                                pixel_dorightflow = !cvmGet8u(disactivated_pixels_rightflow,yl0,xl0);
                            if (disactivated_pixels_stereoinit)
                                pixel_dostereoinit = !cvmGet8u(disactivated_pixels_stereoinit,yl0,xl0);
                            if (disactivated_pixels_stereo)
                                pixel_dostereo = !cvmGet8u(disactivated_pixels_stereo,yl0,xl0);

			    //is the point visible at time t+1 ?
                            if (pixel_doleftflow) {
                                double xlt = cvmGet(MapX_Ilt_inner, yl0, xl0);
                                double ylt = cvmGet(MapY_Ilt_inner, yl0, xl0);
                                if(xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h ||
                                   ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h){
                                    cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,127);
                                    pixel_doleftflow = 0;
                                }
                            }

                            //On the right image at time t
                            if (pixel_dostereoinit || pixel_dorightflow) {
                                double xr0 = cvmGet(MapX_Ir0_inner, yl0, xl0);
                                double yr0 = yl0;
                                if(xr0 >= (N_x - 1 - DISACTIVATE_BORDER)*h || xr0 <= DISACTIVATE_BORDER*h ||
                                   yr0 >= (N_y - 1 - DISACTIVATE_BORDER)*h || yr0 <= DISACTIVATE_BORDER*h) {
                                    if (pixel_dostereoinit) {
                                        cvmSet8u(disactivated_pixels_stereoinit,yl0,xl0,127);
                                        pixel_dostereoinit = 0;
                                    }
                                    if (pixel_dorightflow) {
                                        cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,127);
                                        pixel_dorightflow = 0;
                                    }
                                }
                            }
                            
                            //On the right image at time t+1
                            if (pixel_dostereo || pixel_dorightflow) {
                                double xrt = cvmGet(MapX_Irt_inner, yl0, xl0);
                                double yrt = cvmGet(MapY_Irt_inner, yl0, xl0);
                                if(xrt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xrt < DISACTIVATE_BORDER*h ||
                                   yrt >= (N_y - 1 - DISACTIVATE_BORDER)*h || yrt < DISACTIVATE_BORDER*h) {
                                    if (pixel_dostereo) {
                                        cvmSet8u(disactivated_pixels_stereo,yl0,xl0,127);
                                        pixel_dostereo = 0;
                                    }
                                    if (pixel_dorightflow) {
                                        cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,127);
                                        pixel_dorightflow = 0;
                                    }
                                }
                            }


#if OCCLUDED_PIXELS_LAPLACIAN
                            //Occulted point : use of a laplacian instead of divergence

                            if(!pixel_doleftflow && !pixel_dorightflow && !pixel_dostereo && !pixel_dostereoinit){
				b_f_1 = 1.;
                                b_f_2 = 1.;
                                b_f_3 = 1.;
                                b_f_4 = 1.;
#if DIVERGENCE_8	   		  
                                b_f_5 = 1.;
                                b_f_6 = 1.;
                                b_f_7 = 1.;
                                b_f_8 = 1.;
#endif
                            }
#endif

                            //Matrix terms
                            double a_u_1, a_u_2, a_u_3, a_u_4, a_v_1, a_v_2, a_v_3, a_v_4, a_dt_1, a_dt_2, a_dt_3, a_dt_4, a_d0_1, a_d0_2, a_d0_3, a_d0_4;
#if DIVERGENCE_8
                            a_u_1 = (alpha/(4.*h*h))*(alpha_p_u*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_u*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
                            a_v_1 = 0;
                            a_dt_1 = 0;	  
                            a_d0_1 = 0;
		    
                            a_u_2 = 0;
                            a_v_2 = (alpha/(4.*h*h))*(alpha_p_v*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_v*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
                            a_dt_2 = 0;	  
                            a_d0_2 = 0;
		    
                            a_u_3 = 0;
                            a_v_3 = 0;
                            a_dt_3 = (alpha*lambda/(4.*h*h))*(alpha_p_dprime*(b_f_1 + b_f_2 + b_f_3 + b_f_4) + alpha_d_dprime*(b_f_5 + b_f_6 + b_f_7 + b_f_8));	  
                            a_d0_3 = -(alpha*lambda/(4*h*h))*(alpha_p_d*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_d*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
		    
                            a_u_4 = 0;
                            a_v_4 = 0;
                            a_dt_4 = -(alpha*lambda/(4*h*h))*(alpha_p_dprime*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+ alpha_d_dprime*(b_f_5 + b_f_6 + b_f_7 + b_f_8)); 
                            a_d0_4 = (alpha*(lambda + mu)/(4.*h*h))*(alpha_p_d*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+ alpha_d_d*(b_f_5 + b_f_6 + b_f_7 + b_f_8));
#else  // !DIVERGENCE_8
		    
                            a_u_1 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            a_v_1 = 0;
                            a_dt_1 = 0;	  
                            a_d0_1 = 0;
		    
                            a_u_2 = 0;
                            a_v_2 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            a_dt_2 = 0;	  
                            a_d0_2 = 0;
		    
                            a_u_3 = 0;
                            a_v_3 = 0;
                            a_dt_3 = (alpha*lambda/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);	  
                            a_d0_3 = -(alpha*lambda/(4*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4); 

                            a_u_4 = 0;
                            a_v_4 = 0;
                            a_dt_4 = -(alpha*lambda/(4*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4); 
                            a_d0_4 = (alpha*(lambda + mu)/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
#endif  // !DIVERGENCE_8
                            //second member for the first equation
                            double bu0 = cvmGet(U, yl0, xl0);
                            double bu1 = cvmGet(U, yl0, xl0+1);
                            double bu2 = cvmGet(U, yl0, xl0-1);
                            double bu3 = cvmGet(U,yl0+1,xl0);
                            double bu4 = cvmGet(U,yl0-1,xl0);
                            //Crop(bu0, N_x);
                            double bv0 = cvmGet(V, yl0, xl0);
                            double bv1 = cvmGet(V, yl0, xl0+1);
                            double bv2 = cvmGet(V, yl0, xl0-1);
                            double bv3 = cvmGet(V,yl0+1,xl0);
                            double bv4 = cvmGet(V,yl0-1,xl0);

                            double bdi0 = cvmGet(D0, yl0, xl0);
                            double bdi1 = cvmGet(D0, yl0, xl0+1);
                            double bdi2 = cvmGet(D0, yl0, xl0-1);
                            double bdi3 = cvmGet(D0,yl0+1,xl0);
                            double bdi4 = cvmGet(D0,yl0-1,xl0);
                            //Crop(bdi0, N_x);
                            double bd0 = cvmGet(Dt, yl0, xl0);
                            double bd1 = cvmGet(Dt, yl0, xl0+1);
                            double bd2 = cvmGet(Dt, yl0, xl0-1);
                            double bd3 = cvmGet(Dt,yl0+1,xl0);
                            double bd4 = cvmGet(Dt,yl0-1,xl0);

                            double bdd0 = bd0 - bdi0;
                            double bdd1 = bd1 - bdi1;
                            double bdd2 = bd2 - bdi2;
                            double bdd3 = bd3 - bdi3;
                            double bdd4 = bd4 - bdi4;
		    
#if DIVERGENCE_8
                            double bu5 = cvmGet(U, yl0+1, xl0+1);
                            double bu6 = cvmGet(U, yl0+1, xl0-1);
                            double bu7 = cvmGet(U,yl0-1,xl0+1);
                            double bu8 = cvmGet(U,yl0-1,xl0-1);
                            double bu = (alpha/(4*h*h)) *
                                (alpha_p_u*( 
                                     b_f_1 * bu1 +
                                     b_f_2 * bu2 +
                                     b_f_3 * bu3 +
                                     b_f_4 * bu4)
                                 +alpha_d_u*(
                                     b_f_5 * bu5 +
                                     b_f_6 * bu6 +
                                     b_f_7 * bu7 +
                                     b_f_8 * bu8)	     
                                 - (alpha_p_u*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_u*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bu0);
                            double bv5 = cvmGet(V, yl0+1, xl0+1);
                            double bv6 = cvmGet(V, yl0+1, xl0-1);
                            double bv7 = cvmGet(V,yl0-1,xl0+1);
                            double bv8 = cvmGet(V,yl0-1,xl0-1);
                            double bv = (alpha/(4*h*h)) *
                                (alpha_p_v*(
                                     b_f_1 * bv1 +
                                     b_f_2 * bv2 +
                                     b_f_3 * bv3 +
                                     b_f_4 * bv4)
                                 +alpha_d_v*(
                                     b_f_5 * bv5 +
                                     b_f_6 * bv6 +
                                     b_f_7 * bv7 +
                                     b_f_8 * bv8)		       
                                 - (alpha_p_v*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_v*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bv0);
		
                            double bdi5 = cvmGet(D0, yl0+1, xl0+1);
                            double bdi6 = cvmGet(D0, yl0+1, xl0-1);
                            double bdi7 = cvmGet(D0,yl0-1,xl0+1);
                            double bdi8 = cvmGet(D0,yl0-1,xl0-1);
	
                            double bd5 = cvmGet(Dt, yl0, xl0+1);
                            double bd6 = cvmGet(Dt, yl0, xl0-1);
                            double bd7 = cvmGet(Dt,yl0+1,xl0);
                            double bd8 = cvmGet(Dt,yl0-1,xl0);
		    
                            double bdd5 = bd5 - bdi5;
                            double bdd6 = bd6 - bdi6;
                            double bdd7 = bd7 - bdi7;
                            double bdd8 = bd8 - bdi8;
	
                            double bdi = (alpha*(lambda + mu)/(4*h*h)) *
                                (alpha_p_d*(
                                     b_f_1 * bdi1 +
                                     b_f_2 * bdi2 +
                                     b_f_3 * bdi3 +
                                     b_f_4 * bdi4)
                                 +alpha_d_d*(
                                     b_f_5 * bdi5 +
                                     b_f_6 * bdi6 +
                                     b_f_7 * bdi7 +
                                     b_f_8 * bdi8)	       
                                 - (alpha_p_d*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_d*(b_f_5 + b_f_6 + b_f_7 + b_f_8))* bdi0)
                                - (alpha*lambda/(4*h*h)) *
                                (alpha_p_dprime*(
                                     b_f_1 * bd1 +
                                     b_f_2 * bd2 +
                                     b_f_3 * bd3 +
                                     b_f_4 * bd4)
                                 +alpha_d_dprime*(
                                     b_f_5 * bd5 +
                                     b_f_6 * bd6 +
                                     b_f_7 * bd7 +
                                     b_f_8 * bd8)			
                                 - (alpha_p_dprime*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_dprime*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bd0);

	
		
                            double bd = (alpha*lambda/(4*h*h)) *
                                (alpha_p_dprime*(
                                     b_f_1 * bd1 +
                                     b_f_2 * bd2 +
                                     b_f_3 * bd3 +
                                     b_f_4 * bd4)
                                 -alpha_p_d*(
                                     b_f_1 * bdi1 +
                                     b_f_2 * bdi2 +
                                     b_f_3 * bdi3 +
                                     b_f_4 * bdi4)
                                 +alpha_d_dprime*(
                                     b_f_5 * bd5 +
                                     b_f_6 * bd6 +
                                     b_f_7 * bd7 +
                                     b_f_8 * bd8)
                                 -alpha_d_d*(
                                     b_f_5 * bdi5 +
                                     b_f_6 * bdi6 +
                                     b_f_7 * bdi7 +
                                     b_f_8 * bdi8)
                                 - ((alpha_p_dprime*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_dprime*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bd0 - (alpha_p_d*(b_f_1 + b_f_2 + b_f_3 + b_f_4)+alpha_d_d*(b_f_5 + b_f_6 + b_f_7 + b_f_8)) * bdi0));

#else  // !DIVERGENCE_8
                            double bu = (alpha/(4*h*h)) *
                                (b_f_1 * bu1 +
                                 b_f_2 * bu2 +
                                 b_f_3 * bu3 +
                                 b_f_4 * bu4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bu0);

                            //Crop(bv0, N_y);
                            double bv = (alpha/(4*h*h)) *
                                (b_f_1 * bv1 +
                                 b_f_2 * bv2 +
                                 b_f_3 * bv3 +
                                 b_f_4 * bv4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bv0);
                            double bdi = (alpha*(lambda + mu)/(4*h*h)) *
                                (b_f_1 * bdi1 +
                                 b_f_2 * bdi2 +
                                 b_f_3 * bdi3 +
                                 b_f_4 * bdi4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4)* bdi0)
                                - (alpha*lambda/(4*h*h)) *
                                (b_f_1 * bd1 +
                                 b_f_2 * bd2 +
                                 b_f_3 * bd3 +
                                 b_f_4 * bd4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bd0);

                            double bd = (alpha*lambda/(4*h*h)) *
                                (b_f_1 * bdd1 +
                                 b_f_2 * bdd2 +
                                 b_f_3 * bdd3 +
                                 b_f_4 * bdd4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bdd0);
#endif  // !DIVERGENCE_8

                            //Data term estimation
                            if (pixel_doleftflow || pixel_dostereoinit) {
                                il0 = cvGetReal2D(Il0_pyramid[level], yl0, xl0); 
                                il0_X = cvmGet(Il0_X,  yl0, xl0);
                                il0_Y = cvmGet(Il0_Y,  yl0, xl0);
                            }
                            if (pixel_dorightflow || pixel_dostereoinit) {
                                ir0 = cvGetReal2D(Ir0w,yl0,xl0);
				ir0w_X = cvmGet(Ir0w_X,  yl0, xl0);
				ir0w_Y = cvmGet(Ir0w_Y,  yl0, xl0);
				ir0w_XX = cvmGet(Ir0w_XX,  yl0, xl0);
				ir0w_XY = cvmGet(Ir0w_XY,  yl0, xl0);
				ir0w_YY = cvmGet(Ir0w_YY,  yl0, xl0);

                                // In Maple:
                                // Ir0w(x,y):=Irt(x+d,y);
                                // diff(Ir0w(x,y),x);
                                // diff(Ir0w(x,y),y);
                                // then, express D[1](Ir0) as a function of D[1](Ir0w) and D[2](Ir0w)
                                // and D[2](Ir0) as a function of D[1](Ir0w) and D[2](Ir0w)
                                double det = 1+d0w_X;
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_dostereoinit) {
                                        cvmSet8u(disactivated_pixels_stereoinit,yl0,xl0,191);
                                        pixel_dostereoinit = 0;
                                    }
                                    if (pixel_dorightflow) {
                                        cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,191);
                                        pixel_dorightflow = 0;
                                    }
                                }
                                else {
                                    ir0_X = ir0w_X/det;
                                    ir0_Y = ((1+d0w_X)*ir0w_Y - d0w_Y*ir0w_X)/det;

                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    ir0w_X_X = ir0w_XX/det;
                                    ir0w_X_Y = ((1+d0w_X)*ir0w_XY - d0w_Y*ir0w_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    ir0w_Y_X = ir0w_XY/det;
                                    ir0w_Y_Y = ((1+d0w_X)*ir0w_YY - d0w_Y*ir0w_XY)/det;
                                }
                            }
                            if (pixel_doleftflow || pixel_dostereo) {
                                ilt = cvGetReal2D(Iltw, yl0, xl0);
				iltw_X = cvmGet(Iltw_X,  yl0, xl0);
				iltw_Y = cvmGet(Iltw_Y,  yl0, xl0);
				iltw_XX = cvmGet(Iltw_XX,  yl0, xl0);
				iltw_XY = cvmGet(Iltw_XY,  yl0, xl0);
				iltw_YY = cvmGet(Iltw_YY,  yl0, xl0);

                                // In Maple:
                                // Iltw(x,y):=Ilt(x+u,y+v);
                                // diff(Iltw(x,y),x);
                                // diff(Iltw(x,y),y);
                                // then, express D[1](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                // and D[2](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                double det = ((1+uw_X)*(1+vw_Y) - uw_Y*vw_X);
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_doleftflow) {
                                        cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,191);
                                        pixel_doleftflow = 0;
                                    }
                                    if (pixel_dostereo) {
                                        cvmSet8u(disactivated_pixels_stereo,yl0,xl0,191);
                                        pixel_dostereo = 0;
                                    }
                                }
                                else {
                                    ilt_X = ((1+vw_Y)*iltw_X - vw_X*iltw_Y)/det;
                                    ilt_Y = ((1+uw_X)*iltw_Y - uw_Y*iltw_X)/det;

                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    iltw_X_X = ((1+vw_Y)*iltw_XX - vw_X*iltw_XY)/det;
                                    iltw_X_Y = ((1+uw_X)*iltw_XY - uw_Y*iltw_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    iltw_Y_X = ((1+vw_Y)*iltw_XY - vw_X*iltw_YY)/det;
                                    iltw_Y_Y = ((1+uw_X)*iltw_YY - uw_Y*iltw_XY)/det;
                                }
                            }
                            if (pixel_dorightflow || pixel_dostereo) {
                                irt = cvGetReal2D(Irtw,yl0,xl0);
				irtw_X = cvmGet(Irtw_X,  yl0, xl0);
				irtw_Y = cvmGet(Irtw_Y,  yl0, xl0);
				irtw_XX = cvmGet(Irtw_XX,  yl0, xl0);
				irtw_XY = cvmGet(Irtw_XY,  yl0, xl0);
				irtw_YY = cvmGet(Irtw_YY,  yl0, xl0);

                                // In Maple:
                                // Irtw(x,y):=Irt(x+u+dt,y+v);
                                // diff(Irtw(x,y),x);
                                // diff(Irtw(x,y),y);
                                // then, express D[1](Irt) as a function of D[1](Irtw) and D[2](Irtw)
                                // and D[2](Irt) as a function of D[1](Irtw) and D[2](Irtw)
                                double det = (1+uw_X+dtw_X)*(1+vw_Y) - (uw_Y+dtw_Y)*vw_X;
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_dostereo) {
                                        cvmSet8u(disactivated_pixels_stereo,yl0,xl0,191);
                                        pixel_dostereo = 0;
                                    }
                                    if (pixel_dorightflow) {
                                        cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,191);
                                        pixel_dorightflow = 0;
                                    }
                                }
                                else {
                                    irt_X = ((1+vw_Y)*irtw_X - vw_X*irtw_Y)/det;
                                    irt_Y = ((1+uw_X+dtw_X)*irtw_Y - (uw_Y+dtw_Y)*irtw_X)/det;
                            
                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    irtw_X_X = ((1+vw_Y)*irtw_XX - vw_X*irtw_XY)/det;
                                    irtw_X_Y = ((1+uw_X+dtw_X)*irtw_XY - (uw_Y+dtw_Y)*irtw_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    irtw_Y_X = ((1+vw_Y)*irtw_XY - vw_X*irtw_YY)/det;
                                    irtw_Y_Y = ((1+uw_X+dtw_X)*irtw_YY - (uw_Y+dtw_Y)*irtw_XY)/det;
                                }
                            }
                            if (pixel_doleftflow || pixel_dorightflow || pixel_dostereo) {
                                du = cvmGet(dU, yl0, xl0);
                                dv = cvmGet(dV, yl0, xl0);
                            }
                            if (pixel_dorightflow || pixel_dostereoinit) {
                                dd0 = cvmGet(dD0, yl0, xl0);
                            }
                            if (pixel_dorightflow || pixel_dostereo) {
                                ddt = cvmGet(dDt, yl0, xl0); 
                            }
                 
                            if (pixel_doleftflow) {
                                //psi' coefficients computation
                                double a = square(ilt + ilt_X*du + ilt_Y*dv - il0);
                                double b =
                                    gamma*(square(iltw_X + iltw_X_X*du + iltw_X_Y*dv - il0_X) +
                                           square(iltw_Y + iltw_Y_X*du + iltw_Y_Y*dv - il0_Y));			  
                                //divergence coefficients
                                double psi_flowL = DPsiData(a+b);
                                //u equation
                                a_u_1      +=
                                    psi_flowL*(ilt_X*ilt_X + gamma*(iltw_X_X*iltw_X_X + iltw_Y_X*iltw_Y_X));
                                a_v_1      +=
                                    psi_flowL*(ilt_Y*ilt_X + gamma*(iltw_X_Y*iltw_X_X + iltw_Y_Y*iltw_Y_X));
                                //v equation
                                a_u_2      +=
                                    psi_flowL*(ilt_X*ilt_Y + gamma*(iltw_X_X*iltw_X_Y + iltw_Y_X*iltw_Y_Y));
                                a_v_2      +=
                                    psi_flowL*(ilt_Y*ilt_Y + gamma*(iltw_X_Y*iltw_X_Y + iltw_Y_Y*iltw_Y_Y));
		   
                                // second term
                                bu +=
                                    - psi_flowL*(ilt_X*(ilt - il0) + gamma*(iltw_X_X*(iltw_X - il0_X) + iltw_Y_X*(iltw_Y - il0_Y)));		   
                                bv +=
                                    - psi_flowL*(ilt_Y*(ilt - il0) + gamma*(iltw_X_Y*(iltw_X - il0_X) + iltw_Y_Y*(iltw_Y - il0_Y)));
                            }
                            if (pixel_dorightflow) {
                                double a =
                                    square(irt + irt_X*(du+ddt) + irt_Y*dv - ir0 - ir0_X*dd0);
                                double b =
                                    gamma*(square(irtw_X + irtw_X_X*(du+ddt) + irtw_X_Y*dv - ir0w_X - ir0w_X_X*dd0) +
                                           square(irtw_Y + irtw_Y_X*(du+ddt) + irtw_Y_Y*dv - ir0w_Y - ir0w_Y_X*dd0));
                                double psi_flowR = DPsiData(a+b);
                                //u equation
                                a_u_1      +=
                                    psi_flowR*(irt_X*irt_X + gamma*(irtw_X_X*irtw_X_X + irtw_Y_X*irtw_Y_X));
                                a_v_1      +=
                                    psi_flowR*(irt_Y*irt_X + gamma*(irtw_X_Y*irtw_X_X + irtw_Y_Y*irtw_Y_X));
                                a_dt_1 +=
                                    psi_flowR*(irt_X*irt_X + gamma*(irtw_X_X*irtw_X_X + irtw_Y_X*irtw_Y_X));  
                                a_d0_1      +=
                                    -psi_flowR*(irt_X*ir0_X + gamma*(irtw_X_X*ir0w_X_X + irtw_Y_X*ir0w_Y_X));
		   
                                //v equation
                                a_u_2      +=
                                    psi_flowR*(irt_X*irt_Y + gamma*(irtw_X_X*irtw_X_Y + irtw_Y_X*irtw_Y_Y));
		   
                                a_v_2      +=
                                    psi_flowR*(irt_Y*irt_Y + gamma*(irtw_X_Y*irtw_X_Y + irtw_Y_Y*irtw_Y_Y));
                                a_dt_2 +=
                                    psi_flowR*(irt_X*irt_Y + gamma*(irtw_X_X*irtw_X_Y + irtw_Y_X*irtw_Y_Y));
                                a_d0_2      +=
                                    -psi_flowR*(irt_Y*ir0_X + gamma*(irtw_X_Y*ir0w_X_X + irtw_Y_Y*ir0w_Y_X));
		   
                                //d' equation
                                a_u_3      +=
                                    psi_flowR*(irt_X*irt_X + gamma*(irtw_X_X*irtw_X_X + irtw_Y_X*irtw_Y_X));
                                a_v_3      +=	// = a_dt_2
                                    psi_flowR*(irt_Y*irt_X + gamma*(irtw_X_Y*irtw_X_X + irtw_Y_Y*irtw_Y_X));
                                a_dt_3 +=
                                    psi_flowR*(irt_X*irt_X + gamma*(irtw_X_X*irtw_X_X + irtw_Y_X*irtw_Y_X));	  
                                a_d0_3      +=
                                    -psi_flowR*(irt_X*ir0_X + gamma*(irtw_X_X*ir0w_X_X + irtw_Y_X*ir0w_Y_X));
		   
                                //d equation
                                a_u_4      +=
                                    -psi_flowR*(irt_X*ir0_X + gamma*(irtw_X_X*ir0w_X_X + irtw_Y_X*ir0w_Y_X));
                                a_v_4      +=
                                    -psi_flowR*(irt_Y*ir0_X + gamma*(irtw_X_Y*ir0w_X_X + irtw_Y_Y*ir0w_Y_X)); 
                                a_dt_4 +=
                                    -psi_flowR*(irt_X*ir0_X + gamma*(irtw_X_X*ir0w_X_X + irtw_Y_X*ir0w_Y_X));  
                                a_d0_4      +=
                                    psi_flowR*(ir0_X*ir0_X + gamma*(ir0w_X_X*ir0w_X_X + ir0w_Y_X*ir0w_Y_X));
		   
                                // second term
                                bu +=
                                    - psi_flowR*(irt_X*(irt - ir0) + gamma*(irtw_X_X*(irtw_X - ir0w_X) + irtw_Y_X*(irtw_Y - ir0w_Y)));		   
                                bv +=
                                    - psi_flowR*(irt_Y*(irt - ir0) + gamma*(irtw_X_Y*(irtw_X - ir0w_X) + irtw_Y_Y*(irtw_Y - ir0w_Y)));
                                bd +=
                                    - psi_flowR*((irt - ir0)*irt_X  + gamma*(irtw_X_X *(irtw_X - ir0w_X) + irtw_Y_X *(irtw_Y - ir0w_Y)));
                                bdi +=
                                    - psi_flowR*((ir0 - irt)*ir0_X + gamma*(ir0w_X_X*(ir0w_X - irtw_X) + ir0w_Y_X*(ir0w_Y - irtw_Y)));
                            }
                            if (pixel_dostereoinit) {
                                double a = square(ir0 + ir0_X*dd0 - il0);
                                double b = gamma*(square(ir0w_X + ir0w_X_X*dd0 - il0_X) +
                                                  square(ir0w_Y + ir0w_X_Y*dd0 - il0_Y));
                                double psi_stereo_init = DPsiData(a+b);
		   
                                //d equation
                                a_d0_4      +=
                                    psi_stereo_init*(ir0_X*ir0_X + gamma*(ir0w_X_X*ir0w_X_X + ir0w_Y_X*ir0w_Y_X)); 

                                // second term
                                bdi +=
                                    -psi_stereo_init*((ir0 - il0)*ir0_X + gamma*(ir0w_X_X*(ir0w_X - il0_X) + ir0w_Y_X*(ir0w_Y - il0_Y)));
                            }
                            if (pixel_dostereo) {
                                double a = square(irt + irt_X*(du+ddt) + irt_Y*dv - ilt - ilt_X*du  - ilt_Y*dv);
                                double b = gamma*(square(irtw_X + irtw_X_X*(du+ddt) + irtw_X_Y*dv - iltw_X - iltw_X_X*du - iltw_X_Y*dv) +
                                                  square(irtw_Y + irtw_Y_X*(du+ddt) + irtw_Y_Y*dv - iltw_Y - iltw_Y_X*du - iltw_Y_Y*dv));
                                double psi_stereo = DPsiData(a+b);
                                //u equation
                                a_u_1  +=
                                    psi_stereo*(square(irt_X - ilt_X) +
                                                gamma*(square(irtw_X_X - iltw_X_X) + square(irtw_Y_X - iltw_Y_X)));
                                a_v_1  +=
                                    psi_stereo*((irt_X - ilt_X)*(irt_Y - ilt_Y)+ 
                                                gamma*((irtw_X_X - iltw_X_X)*(irtw_X_Y - iltw_X_Y) +
                                                       (irtw_Y_X - iltw_Y_X)*(irtw_Y_Y - iltw_Y_Y)));
                                a_dt_1 +=
                                    psi_stereo*(irt_X*(irt_X - ilt_X) +
                                                gamma*(irtw_X_X*(irtw_X_X-iltw_X_X) + irtw_Y_X*(irtw_Y_X-iltw_Y_X)));	  

                                //v equation
                                a_u_2  +=
                                    psi_stereo*((irt_X - ilt_X)*(irt_Y - ilt_Y) + 
                                                gamma*((irtw_X_X - iltw_X_X)*(irtw_X_Y - iltw_X_Y) +
                                                       (irtw_Y_X - iltw_Y_X)*(irtw_Y_Y - iltw_Y_Y)));

                                a_v_2  +=
                                    psi_stereo*(square(irt_Y - ilt_Y) + 
                                                gamma*(square(irtw_X_Y - iltw_X_Y) + square(irtw_Y_Y - iltw_Y_Y)));
                                a_dt_2 +=
                                    psi_stereo*(irt_X*(irt_Y - ilt_Y) +
                                                gamma*((irtw_X_Y-iltw_X_Y)*irtw_X_X + (irtw_Y_Y - iltw_Y_Y)*irtw_Y_X));

                                //d' equation
                                a_u_3  +=
                                    psi_stereo*((irt_X - ilt_X)*irt_X +
                                                gamma*((irtw_X_X - iltw_X_X)*irtw_X_X + (irtw_Y_X - iltw_Y_X)*irtw_Y_X));
                                a_v_3  +=	// = a_dt_2
                                    psi_stereo*(irt_X*(irt_Y - ilt_Y) + 
                                                gamma*((irtw_X_Y-iltw_X_Y)*irtw_X_X + (irtw_Y_Y - iltw_Y_Y)*irtw_Y_X));
                                a_dt_3 +=
                                    psi_stereo*(square(irt_X) + gamma*(square(irtw_X_X) + square(irtw_Y_X)));

                                // second term
                                bu +=
                                    - psi_stereo*((irt - ilt)*(irt_X - ilt_X) + gamma*((irtw_X_X - iltw_X_X)*(irtw_X - iltw_X) + (irtw_Y_X - iltw_Y_X)*(irtw_Y - iltw_Y)));		   
                                bv +=
                                    - psi_stereo*((irt - ilt)*(irt_Y - ilt_Y) + gamma*((irtw_X_Y - iltw_X_Y)*(irtw_X - iltw_X) + (irtw_Y_Y - iltw_Y_Y)*(irtw_Y - iltw_Y)));
                                bd +=
                                    - psi_stereo*((irt - ilt)*irt_X + gamma*(irtw_X_X*(irtw_X - iltw_X)  + irtw_Y_X*(irtw_Y - iltw_Y)));
                            }
		 
                            // 3 lines corresponding to the 3 equations at (yl0,xl0) 
                            cvmSet(M,i  ,0,a_u_1);
                            cvmSet(M,i+1,0,a_u_2);
                            cvmSet(M,i+2,0,a_u_3);
                            cvmSet(M,i+3,0,a_u_4);
			   
                            cvmSet(M,i  ,1,a_v_1);
                            cvmSet(M,i+1,1,a_v_2);
                            cvmSet(M,i+2,1,a_v_3);
                            cvmSet(M,i+3,1,a_v_4);
			
                            cvmSet(M,i  ,2,a_dt_1);
                            cvmSet(M,i+1,2,a_dt_2);
                            cvmSet(M,i+2,2,a_dt_3);
                            cvmSet(M,i+3,2,a_dt_4);
			   
                            cvmSet(M,i  ,3,a_d0_1);
                            cvmSet(M,i+1,3,a_d0_2);
                            cvmSet(M,i+2,3,a_d0_3);
                            cvmSet(M,i+3,3,a_d0_4);


#ifdef M_IMAGE
                            if(doleftflow && dodisparity0 && dodisparity){
                                //u equation
                                cvmSet(M_image, i, i, a_u_1); 
                                cvmSet(M_image, i, i+1,  a_v_1);
                                cvmSet(M_image, i, i+2, a_dt_1); 
                                cvmSet(M_image, i, i+3,  a_d0_1);

		 
                                //v equation
                                cvmSet(M_image, i+1, i, a_u_2); 
                                cvmSet(M_image, i+1, i+1,  a_v_2);
                                cvmSet(M_image, i+1, i+2, a_dt_2); 
                                cvmSet(M_image, i+1, i+3,  a_d0_2);
		 

                                //dprime equation
                                cvmSet(M_image, i+2, i, a_u_3); 
                                cvmSet(M_image, i+2, i+1,  a_v_3);
                                cvmSet(M_image, i+2, i+2, a_dt_3); 
                                cvmSet(M_image, i+2, i+3,  a_d0_3);
		 

                                //d equation
                                cvmSet(M_image, i+3, i, a_u_4); 
                                cvmSet(M_image, i+3, i+1,  a_v_4);
                                cvmSet(M_image, i+3, i+2, a_dt_4); 
                                cvmSet(M_image, i+3, i+3,  a_d0_4);
                            }
#endif // M_IMAGE

		 
#if DIVERGENCE_8
                            cvmSet(M,i  ,4,(-alpha*alpha_p_u/(4*h*h))*b_f_4);
                            cvmSet(M,i+1,4,(-alpha*alpha_p_v/(4*h*h))*b_f_4);
                            cvmSet(M,i+2,4,(-alpha*alpha_p_dprime*lambda/(4*h*h))*b_f_4);
                            cvmSet(M,i+3,4,(-alpha*alpha_p_d*(lambda + mu)/(4*h*h))*b_f_4);
		 
                            cvmSet(M,i  ,5,(-alpha*alpha_p_u/(4*h*h))*b_f_3);
                            cvmSet(M,i+1,5,(-alpha*alpha_p_v/(4*h*h))*b_f_3);
                            cvmSet(M,i+2,5,(-alpha*alpha_p_dprime*lambda/(4*h*h))*b_f_3);
                            cvmSet(M,i+3,5,(-alpha*alpha_p_d*(lambda + mu)/(4*h*h))*b_f_3);
		 
                            cvmSet(M,i  ,6,(-alpha*alpha_p_u/(4*h*h))*b_f_2);
                            cvmSet(M,i+1,6,(-alpha*alpha_p_v/(4*h*h))*b_f_2);
                            cvmSet(M,i+2,6,(-alpha*alpha_p_dprime*lambda/(4*h*h))*b_f_2);
                            cvmSet(M,i+3,6,(-alpha*alpha_p_d*(lambda + mu)/(4*h*h))*b_f_2);
		 
                            cvmSet(M,i  ,7,(-alpha*alpha_p_u/(4*h*h))*b_f_1);
                            cvmSet(M,i+1,7,(-alpha*alpha_p_v/(4*h*h))*b_f_1);
                            cvmSet(M,i+2,7,(-alpha*alpha_p_dprime*lambda/(4*h*h))*b_f_1);
                            cvmSet(M,i+3,7,(-alpha*alpha_p_d*(lambda + mu)/(4*h*h))*b_f_1);

                            cvmSet(M,i  ,8,(-alpha*alpha_d_u/(4*h*h))*b_f_5);
                            cvmSet(M,i+1,8,(-alpha*alpha_d_v/(4*h*h))*b_f_5);
                            cvmSet(M,i+2,8,(-alpha*alpha_d_dprime*lambda/(4*h*h))*b_f_5);
                            cvmSet(M,i+3,8,(-alpha*alpha_d_d*(lambda + mu)/(4*h*h))*b_f_5);
		 
                            cvmSet(M,i  ,9,(-alpha*alpha_d_u/(4*h*h))*b_f_6);
                            cvmSet(M,i+1,9,(-alpha*alpha_d_v/(4*h*h))*b_f_6);
                            cvmSet(M,i+2,9,(-alpha*alpha_d_dprime*lambda/(4*h*h))*b_f_6);
                            cvmSet(M,i+3,9,(-alpha*alpha_d_d*(lambda + mu)/(4*h*h))*b_f_6);
		 
                            cvmSet(M,i  ,10,(-alpha*alpha_d_u/(4*h*h))*b_f_7);
                            cvmSet(M,i+1,10,(-alpha*alpha_d_v/(4*h*h))*b_f_7);
                            cvmSet(M,i+2,10,(-alpha*alpha_d_dprime*lambda/(4*h*h))*b_f_7);
                            cvmSet(M,i+3,10,(-alpha*alpha_d_d*(lambda + mu)/(4*h*h))*b_f_7);
		 
                            cvmSet(M,i  ,11,(-alpha*alpha_d_u/(4*h*h))*b_f_8);
                            cvmSet(M,i+1,11,(-alpha*alpha_d_v/(4*h*h))*b_f_8);
                            cvmSet(M,i+2,11,(-alpha*alpha_d_dprime*lambda/(4*h*h))*b_f_8);
                            cvmSet(M,i+3,11,(-alpha*alpha_d_d*(lambda + mu)/(4*h*h))*b_f_8);
		 
#else  // !DIVERGENCE_8

                            cvmSet(M,i  ,4,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+1,4,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+2,4,(-alpha*lambda/(4*h*h))*b_f_4);
                            cvmSet(M,i+3,4,(-alpha*(lambda + mu)/(4*h*h))*b_f_4);
		 
                            cvmSet(M,i  ,5,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+1,5,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+2,5,(-alpha*lambda/(4*h*h))*b_f_3);
                            cvmSet(M,i+3,5,(-alpha*(lambda + mu)/(4*h*h))*b_f_3);
		 
                            cvmSet(M,i  ,6,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+1,6,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+2,6,(-alpha*lambda/(4*h*h))*b_f_2);
                            cvmSet(M,i+3,6,(-alpha*(lambda + mu)/(4*h*h))*b_f_2);
		 
                            cvmSet(M,i  ,7,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+1,7,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+2,7,(-alpha*lambda/(4*h*h))*b_f_1);
                            cvmSet(M,i+3,7,(-alpha*(lambda + mu)/(4*h*h))*b_f_1);


#ifdef M_IMAGE
                            if(doleftflow && dodisparity0 && dodisparity){
                                //u equation
                                cvmSet(M_image, i, i-dimension*N_x, (-alpha/(4*h*h))*b_f_4); 
                                cvmSet(M_image, i, i+dimension*N_x,  (-alpha/(4*h*h))*b_f_3);
                                cvmSet(M_image, i, i-dimension, (-alpha/(4*h*h))*b_f_2); 
                                cvmSet(M_image, i, i+dimension,  (-alpha/(4*h*h))*b_f_1);
		 
                                //v equation
                                cvmSet(M_image, i+1, i-dimension*N_x, (-alpha/(4*h*h))*b_f_4); 
                                cvmSet(M_image, i+1, i+dimension*N_x,  (-alpha/(4*h*h))*b_f_3);
                                cvmSet(M_image, i+1, i-dimension, (-alpha/(4*h*h))*b_f_2); 
                                cvmSet(M_image, i+1, i+dimension,  (-alpha/(4*h*h))*b_f_1);

		   
                                //dprime equation
                                cvmSet(M_image, i+2, i-dimension*N_x, (-alpha*lambda/(4*h*h))*b_f_4); 
                                cvmSet(M_image, i+2, i+dimension*N_x,  (-alpha*lambda/(4*h*h))*b_f_3);
                                cvmSet(M_image, i+2, i-dimension, (-alpha*lambda/(4*h*h))*b_f_2); 
                                cvmSet(M_image, i+2, i+dimension,  (-alpha*lambda/(4*h*h))*b_f_1);
		 

                                //d equation
                                cvmSet(M_image, i+3, i-dimension*N_x, (-alpha*(lambda+mu)/(4*h*h))*b_f_4); 
                                cvmSet(M_image, i+3, i+dimension*N_x,  (-alpha*(lambda+mu)/(4*h*h))*b_f_3);
                                cvmSet(M_image, i+3, i-dimension, (-alpha*(lambda+mu)/(4*h*h))*b_f_2); 
                                cvmSet(M_image, i+3, i+dimension,  (-alpha*(lambda+mu)/(4*h*h))*b_f_1);
		 


                            }
#endif //M_IMAGE

#endif  // !DIVERGENCE_8


                            cvmSet(B,i,0,bu);
                            cvmSet(B,i+1,0,bv);
                            cvmSet(B,i+2,0,bd);
                            cvmSet(B,i+3,0,bdi);

			}
			else if(methode == 1){
                            double il0 = 0.;
                            double il0_X = 0., il0_Y = 0.;
                            double ilt = 0.;
                            double ilt_X = 0., ilt_Y = 0.;
                            double iltw_X = 0., iltw_Y = 0.;
                            double iltw_XX = 0., iltw_XY = 0., iltw_YY = 0.;
                            double iltw_X_X = 0., iltw_X_Y = 0.;
                            double iltw_Y_X = 0., iltw_Y_Y = 0.;
                            double irt = 0.;
                            double irt_X = 0., irt_Y = 0.;
                            double irtw_X = 0., irtw_Y = 0.;
                            double irtw_XX = 0., irtw_XY = 0., irtw_YY = 0.;
                            double irtw_X_X = 0., irtw_X_Y = 0.;
                            double irtw_Y_X = 0., irtw_Y_Y = 0.;
                            double du = 0, dv = 0, ddt = 0;
                            //interpolated divergence coefficients
                            double b_f_1, b_f_2, b_f_3, b_f_4;
			    // Half pixel interpolation for the divergence coefficients 
                            divergence_coefficients_interpolate(divergence_coefficients, yl0, xl0, &b_f_1, &b_f_2, &b_f_3, &b_f_4);
			    
			    //U and V actual Gradients for the warp jacobian
			    double uw_X = cvmGet(U_X, yl0, xl0);
			    double uw_Y = cvmGet(U_Y, yl0, xl0);
			    double vw_X = cvmGet(V_X, yl0, xl0);
			    double vw_Y = cvmGet(V_Y, yl0, xl0);
			    double dtw_X = cvmGet(Dt_X, yl0, xl0);
			    double dtw_Y = cvmGet(Dt_Y, yl0, xl0);
			    
			    //Pixels out must not be taken into account in the energy data term
			    int pixel_doleftflow = 0;
			    int pixel_dostereo = 0;
			  
			    if (disactivated_pixels_leftflow)
			      pixel_doleftflow = !cvmGet8u(disactivated_pixels_leftflow,yl0,xl0);
                         
                            if (disactivated_pixels_stereo)
                                pixel_dostereo = !cvmGet8u(disactivated_pixels_stereo,yl0,xl0);

			   

// 			    //is the point visible at time t+1 ?

			    double xlt = cvmGet(MapX_Ilt_inner, yl0, xl0);
			    double ylt = cvmGet(MapY_Ilt_inner, yl0, xl0);

			    if(xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h ||
			       ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h){
                                if (pixel_doleftflow) {
                                    cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,127);
                                    pixel_doleftflow = 0;
                                }
                            }

                            //On the right image at time t+1
                            double xrt = cvmGet(MapX_Irt_inner, yl0, xl0);
                            double yrt = cvmGet(MapY_Irt_inner, yl0, xl0);
                            if(xrt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xrt < DISACTIVATE_BORDER*h ||
                               yrt >= (N_y - 1 - DISACTIVATE_BORDER)*h || yrt < DISACTIVATE_BORDER*h) {
                                if (pixel_dostereo) {
                                    cvmSet8u(disactivated_pixels_stereo,yl0,xl0,127);
                                    pixel_dostereo = 0;
                                }
                            }

	              

// 			    //Matrix terms
                            double a_u_1, a_u_2, a_u_3, a_v_1, a_v_2, a_v_3, a_dt_1, a_dt_2, a_dt_3;
			    a_u_1 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            a_v_1 = 0.;
                            a_dt_1 = 0.;	  
		    
                            a_u_2 = 0.;
                            a_v_2 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            a_dt_2 = 0.;	  
		    
                            a_u_3 = 0.;
                            a_v_3 = 0.;
                            a_dt_3 = (alpha*lambda/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);	  
                           
			    
			    double bu0 = cvmGet(U, yl0, xl0);
                            double bu1 = cvmGet(U, yl0, xl0+1);
                            double bu2 = cvmGet(U, yl0, xl0-1);
                            double bu3 = cvmGet(U,yl0+1,xl0);
                            double bu4 = cvmGet(U,yl0-1,xl0);
                            //Crop(bu0, N_x);
                            double bv0 = cvmGet(V, yl0, xl0);
                            double bv1 = cvmGet(V, yl0, xl0+1);
                            double bv2 = cvmGet(V, yl0, xl0-1);
                            double bv3 = cvmGet(V,yl0+1,xl0);
                            double bv4 = cvmGet(V,yl0-1,xl0);

                            double bdi0 = cvmGet(D0, yl0, xl0);
                            double bdi1 = cvmGet(D0, yl0, xl0+1);
                            double bdi2 = cvmGet(D0, yl0, xl0-1);
                            double bdi3 = cvmGet(D0,yl0+1,xl0);
                            double bdi4 = cvmGet(D0,yl0-1,xl0);
                            //Crop(bdi0, N_x);
                            double bd0 = cvmGet(Dt, yl0, xl0);
                            double bd1 = cvmGet(Dt, yl0, xl0+1);
                            double bd2 = cvmGet(Dt, yl0, xl0-1);
                            double bd3 = cvmGet(Dt,yl0+1,xl0);
                            double bd4 = cvmGet(Dt,yl0-1,xl0);

                            double bdd0 = bd0 - bdi0;
                            double bdd1 = bd1 - bdi1;
                            double bdd2 = bd2 - bdi2;
                            double bdd3 = bd3 - bdi3;
                            double bdd4 = bd4 - bdi4;
			    
			    double bu = (alpha/(4*h*h)) *
                                (b_f_1 * bu1 +
                                 b_f_2 * bu2 +
                                 b_f_3 * bu3 +
                                 b_f_4 * bu4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bu0);

                            //Crop(bv0, N_y);
                            double bv = (alpha/(4*h*h)) *
                                (b_f_1 * bv1 +
                                 b_f_2 * bv2 +
                                 b_f_3 * bv3 +
                                 b_f_4 * bv4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bv0);
                          
                            double bd = (alpha*lambda/(4*h*h)) *
                                (b_f_1 * bdd1 +
                                 b_f_2 * bdd2 +
                                 b_f_3 * bdd3 +
                                 b_f_4 * bdd4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bdd0);
			    
			    if (pixel_doleftflow) {
                                il0 = cvGetReal2D(Il0_pyramid[level], yl0, xl0); 
                                il0_X = cvmGet(Il0_X,  yl0, xl0);
                                il0_Y = cvmGet(Il0_Y,  yl0, xl0);
                            }

			    

			    //Data term estimation
                            if (pixel_doleftflow || pixel_dostereo) {
                                ilt = cvGetReal2D(Iltw, yl0, xl0);
                                iltw_X = cvmGet(Iltw_X,  yl0, xl0);
                                iltw_Y = cvmGet(Iltw_Y,  yl0, xl0);
                                iltw_XX = cvmGet(Iltw_XX, yl0, xl0);
                                iltw_XY = cvmGet(Iltw_XY, yl0, xl0);
                                iltw_YY = cvmGet(Iltw_YY, yl0, xl0);
			      
                                // In Maple:
                                // Iltw(x,y):=Ilt(x+u,y+v);
                                // diff(Iltw(x,y),x);
                                // diff(Iltw(x,y),y);
                                // then, express D[1](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                // and D[2](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                double det = ((1+uw_X)*(1+vw_Y) - uw_Y*vw_X);
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_doleftflow) {
                                        cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,191);
                                        pixel_doleftflow = 0;
                                    }
                                    if (pixel_dostereo) {
                                        cvmSet8u(disactivated_pixels_stereo,yl0,xl0,191);
                                        pixel_dostereo = 0;
                                    }
                                }
                                else {
                                    ilt_X = ((1+vw_Y)*iltw_X - vw_X*iltw_Y)/det;
                                    ilt_Y = ((1+uw_X)*iltw_Y - uw_Y*iltw_X)/det;
                            
                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    iltw_X_X = ((1+vw_Y)*iltw_XX - vw_X*iltw_XY)/det;
                                    iltw_X_Y = ((1+uw_X)*iltw_XY - uw_Y*iltw_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    iltw_Y_X = ((1+vw_Y)*iltw_XY - vw_X*iltw_YY)/det;
                                    iltw_Y_Y = ((1+uw_X)*iltw_YY - uw_Y*iltw_XY)/det;
                                }
                            }
                            if (pixel_dostereo) {
                                irt = cvGetReal2D(Irtw,yl0,xl0);
                                irtw_X = cvmGet(Irtw_X,  yl0, xl0);
                                irtw_Y = cvmGet(Irtw_Y,  yl0, xl0);
                                irtw_XX = cvmGet(Irtw_XX, yl0, xl0);
                                irtw_XY = cvmGet(Irtw_XY, yl0, xl0);
                                irtw_YY = cvmGet(Irtw_YY, yl0, xl0);

                                // In Maple:
                                // Irtw(x,y):=Irt(x+u+dt,y+v);
                                // diff(Irtw(x,y),x);
                                // diff(Irtw(x,y),y);
                                // then, express D[1](Irt) as a function of D[1](Irtw) and D[2](Irtw)
                                // and D[2](Irt) as a function of D[1](Irtw) and D[2](Irtw)
                                double det = (1+uw_X+dtw_X)*(1+vw_Y) - (uw_Y+dtw_Y)*vw_X;
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_dostereo) {
                                        cvmSet8u(disactivated_pixels_stereo,yl0,xl0,191);
                                        pixel_dostereo = 0;
                                    }
                                }
                                else {
                                    irt_X = ((1+vw_Y)*irtw_X - vw_X*irtw_Y)/det;
                                    irt_Y = ((1+uw_X+dtw_X)*irtw_Y - (uw_Y+dtw_Y)*irtw_X)/det;
                            
                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    irtw_X_X = ((1+vw_Y)*irtw_XX - vw_X*irtw_XY)/det;
                                    irtw_X_Y = ((1+uw_X+dtw_X)*irtw_XY - (uw_Y+dtw_Y)*irtw_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    irtw_Y_X = ((1+vw_Y)*irtw_XY - vw_X*irtw_YY)/det;
                                    irtw_Y_Y = ((1+uw_X+dtw_X)*irtw_YY - (uw_Y+dtw_Y)*irtw_XY)/det;
                                }
                            }
                            if (pixel_doleftflow || pixel_dostereo) {
                                du = cvmGet(dU, yl0, xl0);
                                dv = cvmGet(dV, yl0, xl0);
                            }
                            if (pixel_dostereo) {
                                ddt = cvmGet(dDt, yl0, xl0); 
                            }

			    if (pixel_doleftflow) {
                                //psi' coefficients computation
                                double a = square(ilt + ilt_X*du + ilt_Y*dv - il0);
                                double b =
                                    gamma*(square(iltw_X + iltw_X_X*du + iltw_X_Y*dv - il0_X) +
                                           square(iltw_Y + iltw_Y_X*du + iltw_Y_Y*dv - il0_Y));			  
                                //divergence coefficients
                                double psi_flowL = DPsiData(a+b);
                                //u equation
                                a_u_1      +=
                                    psi_flowL*(ilt_X*ilt_X + gamma*(iltw_X_X*iltw_X_X + iltw_Y_X*iltw_Y_X));
                                a_v_1      +=
                                    psi_flowL*(ilt_Y*ilt_X + gamma*(iltw_X_Y*iltw_X_X + iltw_Y_Y*iltw_Y_X));
                                //v equation
                                a_u_2      +=
                                    psi_flowL*(ilt_X*ilt_Y + gamma*(iltw_X_X*iltw_X_Y + iltw_Y_X*iltw_Y_Y));
                                a_v_2      +=
                                    psi_flowL*(ilt_Y*ilt_Y + gamma*(iltw_X_Y*iltw_X_Y + iltw_Y_Y*iltw_Y_Y));
		   
                                // second term
                                bu +=
                                    - psi_flowL*(ilt_X*(ilt - il0) + gamma*(iltw_X_X*(iltw_X - il0_X) + iltw_Y_X*(iltw_Y - il0_Y)));		   
                                bv +=
                                    - psi_flowL*(ilt_Y*(ilt - il0) + gamma*(iltw_X_Y*(iltw_X - il0_X) + iltw_Y_Y*(iltw_Y - il0_Y)));
                            }

			   
			    
			    if (pixel_dostereo) {
                                double a = square(irt + irt_X*(du+ddt) + irt_Y*dv - ilt - ilt_X*du - ilt_Y*dv);
                                double b = gamma*(square(irtw_X + irtw_X_X*(du+ddt) + irtw_X_Y*dv - iltw_X - iltw_X_X*du - iltw_X_Y*dv) +
                                                  square(irtw_Y + irtw_Y_X*(du+ddt) + irtw_Y_Y*dv - iltw_Y - iltw_Y_X*du - iltw_Y_Y*dv));
                                double psi_stereo = DPsiData(a+b);
                                //u equation
                                a_u_1  +=
                                    psi_stereo*(square(irt_X - ilt_X) +
                                                gamma*(square(irtw_X_X - iltw_X_X) + square(irtw_Y_X - iltw_Y_X)));
                                a_v_1  +=
                                    psi_stereo*((irt_X - ilt_X)*(irt_Y - ilt_Y)+ 
                                                gamma*((irtw_X_X - iltw_X_X)*(irtw_X_Y - iltw_X_Y) +
                                                       (irtw_Y_X - iltw_Y_X)*(irtw_Y_Y - iltw_Y_Y)));
                                a_dt_1 +=
                                    psi_stereo*(irt_X*(irt_X - ilt_X) +
                                                gamma*(irtw_X_X*(irtw_X_X-iltw_X_X) + irtw_Y_X*(irtw_Y_X-iltw_Y_X)));	  

                                //v equation
                                a_u_2  +=
                                    psi_stereo*((irt_X - ilt_X)*(irt_Y - ilt_Y) + 
                                                gamma*((irtw_X_X - iltw_X_X)*(irtw_X_Y - iltw_X_Y) +
                                                       (irtw_Y_X - iltw_Y_X)*(irtw_Y_Y - iltw_Y_Y)));

                                a_v_2  +=
                                    psi_stereo*(square(irt_Y - ilt_Y) + 
                                                gamma*(square(irtw_X_Y - iltw_X_Y) + square(irtw_Y_Y - iltw_Y_Y)));
                                a_dt_2 +=
                                    psi_stereo*(irt_X*(irt_Y - ilt_Y) +
                                                gamma*((irtw_X_Y-iltw_X_Y)*irtw_X_X + (irtw_Y_Y - iltw_Y_Y)*irtw_Y_X));

                                //d' equation
                                a_u_3  +=
                                    psi_stereo*((irt_X - ilt_X)*irt_X +
                                                gamma*((irtw_X_X - iltw_X_X)*irtw_X_X + (irtw_Y_X - iltw_Y_X)*irtw_Y_X));
                                a_v_3  +=	// = a_dt_2
                                    psi_stereo*(irt_X*(irt_Y - ilt_Y) + 
                                                gamma*((irtw_X_Y-iltw_X_Y)*irtw_X_X + (irtw_Y_Y - iltw_Y_Y)*irtw_Y_X));
                                a_dt_3 +=
                                    psi_stereo*(square(irt_X) + gamma*(square(irtw_X_X) + square(irtw_Y_X)));

                                // second term
                                bu +=
                                    - psi_stereo*((irt - ilt)*(irt_X - ilt_X) + gamma*((irtw_X_X - iltw_X_X)*(irtw_X - iltw_X) + (irtw_Y_X - iltw_Y_X)*(irtw_Y - iltw_Y)));		   
                                bv +=
                                    - psi_stereo*((irt - ilt)*(irt_Y - ilt_Y) + gamma*((irtw_X_Y - iltw_X_Y)*(irtw_X - iltw_X) + (irtw_Y_Y - iltw_Y_Y)*(irtw_Y - iltw_Y)));
                                bd +=
                                    - psi_stereo*((irt - ilt)*irt_X + gamma*(irtw_X_X*(irtw_X - iltw_X)  + irtw_Y_X*(irtw_Y - iltw_Y)));
                            }
			   
			    // 3 lines corresponding to the 3 equations at (yl0,xl0) 
                            cvmSet(M,i  ,0,a_u_1);
                            cvmSet(M,i+1,0,a_u_2);
                            cvmSet(M,i+2,0,a_u_3);
			   
                            cvmSet(M,i  ,1,a_v_1);
                            cvmSet(M,i+1,1,a_v_2);
                            cvmSet(M,i+2,1,a_v_3);
			
                            cvmSet(M,i  ,2,a_dt_1);
                            cvmSet(M,i+1,2,a_dt_2);
                            cvmSet(M,i+2,2,a_dt_3);
			   
                          
			    cvmSet(M,i  ,3,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+1,3,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+2,3,(-alpha*lambda/(4*h*h))*b_f_4);
                           
                            cvmSet(M,i  ,4,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+1,4,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+2,4,(-alpha*lambda/(4*h*h))*b_f_3);
                          
                            cvmSet(M,i  ,5,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+1,5,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+2,5,(-alpha*lambda/(4*h*h))*b_f_2);
                           
                            cvmSet(M,i  ,6,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+1,6,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+2,6,(-alpha*lambda/(4*h*h))*b_f_1);
                           
			    cvmSet(B,i,0,bu);
                            cvmSet(B,i+1,0,bv);
                            cvmSet(B,i+2,0,bd);

			    
			}
			else if(methode ==2){ //U,V only
                            double il0 = 0.;
                            double il0_X = 0., il0_Y = 0.;
                            double ilt = 0.;
                            double ilt_X = 0., ilt_Y = 0.;
                            double iltw_X = 0., iltw_Y = 0.;
                            double iltw_XX = 0., iltw_XY = 0., iltw_YY = 0.;
                            double iltw_X_X = 0., iltw_X_Y = 0.;
                            double iltw_Y_X = 0., iltw_Y_Y = 0.;
                            double du = 0, dv = 0;
                            //interpolated divergence coefficients
                            double b_f_1, b_f_2, b_f_3, b_f_4;
                            // Half pixel interpolation for the divergence coefficients 
                            divergence_coefficients_interpolate(divergence_coefficients, yl0, xl0, &b_f_1, &b_f_2, &b_f_3, &b_f_4);

                            //U and V actual Gradients for the warp jacobian
                            double uw_X = cvmGet(U_X, yl0, xl0);
                            double uw_Y = cvmGet(U_Y, yl0, xl0);
                            double vw_X = cvmGet(V_X, yl0, xl0);
                            double vw_Y = cvmGet(V_Y, yl0, xl0);
                            //Pixels out must not be taken into account in the energy data term
                            int pixel_doleftflow = 0;
                            if (disactivated_pixels_leftflow)
                                pixel_doleftflow = !cvmGet8u(disactivated_pixels_leftflow,yl0,xl0);

			    //is the point visible at time t+1 ?
			    double xlt = cvmGet(MapX_Ilt_inner, yl0, xl0)/h;
			    double ylt = cvmGet(MapY_Ilt_inner, yl0, xl0);
			    if(xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h ||
			       ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h){
                                if (pixel_doleftflow) {
                                    cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,127);
                                    pixel_doleftflow = 0;
                                }
                            }

			    //Matrix terms
                            double a_u_1, a_u_2, a_v_1, a_v_2;
			    a_u_1 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            a_v_1 = 0;
		    
                            a_u_2 = 0;
                            a_v_2 = (alpha/(4.*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                           
			    double bu0 = cvmGet(U, yl0, xl0);
                            double bu1 = cvmGet(U, yl0, xl0+1);
                            double bu2 = cvmGet(U, yl0, xl0-1);
                            double bu3 = cvmGet(U,yl0+1,xl0);
                            double bu4 = cvmGet(U,yl0-1,xl0);
                            //Crop(bu0, N_x);
                            double bv0 = cvmGet(V, yl0, xl0);
                            double bv1 = cvmGet(V, yl0, xl0+1);
                            double bv2 = cvmGet(V, yl0, xl0-1);
                            double bv3 = cvmGet(V,yl0+1,xl0);
                            double bv4 = cvmGet(V,yl0-1,xl0);

                           
			    double bu = (alpha/(4*h*h)) *
                                (b_f_1 * bu1 +
                                 b_f_2 * bu2 +
                                 b_f_3 * bu3 +
                                 b_f_4 * bu4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bu0);

                            //Crop(bv0, N_y);
                            double bv = (alpha/(4*h*h)) *
                                (b_f_1 * bv1 +
                                 b_f_2 * bv2 +
                                 b_f_3 * bv3 +
                                 b_f_4 * bv4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bv0);

			    if (pixel_doleftflow) {
                                il0 = cvGetReal2D(Il0_pyramid[level], yl0, xl0); 
                                il0_X = cvmGet(Il0_X,  yl0, xl0);
                                il0_Y = cvmGet(Il0_Y,  yl0, xl0);
                            }
                          
			    //Data term estimation
                            if (pixel_doleftflow ) {
                                ilt = cvGetReal2D(Iltw, yl0, xl0);
				iltw_X = cvmGet(Iltw_X,  yl0, xl0);
				iltw_Y = cvmGet(Iltw_Y,  yl0, xl0);
				iltw_XX = cvmGet(Iltw_XX, yl0, xl0);
				iltw_XY = cvmGet(Iltw_XY, yl0, xl0);
				iltw_YY = cvmGet(Iltw_YY, yl0, xl0);

                                // In Maple:
                                // Iltw(x,y):=Ilt(x+u,y+v);
                                // diff(Iltw(x,y),x);
                                // diff(Iltw(x,y),y);
                                // then, express D[1](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                // and D[2](Ilt) as a function of D[1](Iltw) and D[2](Iltw)
                                double det = ((1+uw_X)*(1+vw_Y) - uw_Y*vw_X);
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (pixel_doleftflow) {
                                        cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,191);
                                        pixel_doleftflow = 0;
                                    }
                                }
                                else {
                                    ilt_X = ((1+vw_Y)*iltw_X - vw_X*iltw_Y)/det;
                                    ilt_Y = ((1+uw_X)*iltw_Y - uw_Y*iltw_X)/det;

                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    iltw_X_X = ((1+vw_Y)*iltw_XX - vw_X*iltw_XY)/det;
                                    iltw_X_Y = ((1+uw_X)*iltw_XY - uw_Y*iltw_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    iltw_Y_X = ((1+vw_Y)*iltw_XY - vw_X*iltw_YY)/det;
                                    iltw_Y_Y = ((1+uw_X)*iltw_YY - uw_Y*iltw_XY)/det;
                                }
                            }
                         
                            if (pixel_doleftflow) {
                                du = cvmGet(dU, yl0, xl0);
                                dv = cvmGet(dV, yl0, xl0);
                            }

			    if (pixel_doleftflow) {
                                //psi' coefficients computation
                                double a = square(ilt + ilt_X*du + ilt_Y*dv - il0);
                                double b =
                                    gamma*(square(iltw_X + iltw_X_X*du + iltw_X_Y*dv - il0_X) +
                                           square(iltw_Y + iltw_Y_X*du + iltw_Y_Y*dv - il0_Y));			  
                                //divergence coefficients
                                double psi_flowL = DPsiData(a+b);
                                //u equation
                                a_u_1      +=
                                    psi_flowL*(ilt_X*ilt_X + gamma*(iltw_X_X*iltw_X_X + iltw_Y_X*iltw_Y_X));
                                a_v_1      +=
                                    psi_flowL*(ilt_Y*ilt_X + gamma*(iltw_X_Y*iltw_X_X + iltw_Y_Y*iltw_Y_X));
                                //v equation
                                a_u_2      +=
                                    psi_flowL*(ilt_X*ilt_Y + gamma*(iltw_X_X*iltw_X_Y + iltw_Y_X*iltw_Y_Y));
                                a_v_2      +=
                                    psi_flowL*(ilt_Y*ilt_Y + gamma*(iltw_X_Y*iltw_X_Y + iltw_Y_Y*iltw_Y_Y));
		   
                                // second term
                                bu +=
                                    - psi_flowL*(ilt_X*(ilt - il0) + gamma*(iltw_X_X*(iltw_X - il0_X) + iltw_Y_X*(iltw_Y - il0_Y)));		   
                                bv +=
                                    - psi_flowL*(ilt_Y*(ilt - il0) + gamma*(iltw_X_Y*(iltw_X - il0_X) + iltw_Y_Y*(iltw_Y - il0_Y)));
                            }
			   
			    // 2 lines corresponding to the 2 equations at (yl0,xl0) 
                            cvmSet(M,i  ,0,a_u_1);
                            cvmSet(M,i+1,0,a_u_2);
                           
			   
                            cvmSet(M,i  ,1,a_v_1);
                            cvmSet(M,i+1,1,a_v_2);
                            
                          
			    cvmSet(M,i  ,2,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+1,2,(-alpha/(4*h*h))*b_f_4);
                            cvmSet(M,i+2,2,(-alpha*lambda/(4*h*h))*b_f_4);
                           
                            cvmSet(M,i  ,3,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+1,3,(-alpha/(4*h*h))*b_f_3);
                            cvmSet(M,i+2,3,(-alpha*lambda/(4*h*h))*b_f_3);
                          
                            cvmSet(M,i  ,4,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+1,4,(-alpha/(4*h*h))*b_f_2);
                            cvmSet(M,i+2,4,(-alpha*lambda/(4*h*h))*b_f_2);
                           
                            cvmSet(M,i  ,5,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+1,5,(-alpha/(4*h*h))*b_f_1);
                            cvmSet(M,i+2,5,(-alpha*lambda/(4*h*h))*b_f_1);
                           
			    cvmSet(B,i,0,bu);
                            cvmSet(B,i+1,0,bv);
                          

			}
			else if(methode == 5){ //Dinit only
                            double ir0 = 0., il0 = 0.;
                            double il0_X = 0., il0_Y = 0.;
                            double ir0_X = 0., ir0_Y = 0.;
                            double ir0w_X = 0., ir0w_Y = 0.;
                            double ir0w_XX = 0., ir0w_XY = 0., ir0w_YY = 0.;
                            double ir0w_X_X = 0., ir0w_X_Y = 0.;
                            double ir0w_Y_X = 0., ir0w_Y_Y = 0.;
                            double psi_stereo_init = 0.;
			    double d0w_X = cvmGet(D0_X, yl0, xl0);
			    double d0w_Y = cvmGet(D0_Y, yl0, xl0);
			    double b_f_1, b_f_2, b_f_3, b_f_4;
                            //d only
			
                            // Half pixel interpolation for the divergence coefficients 
                            divergence_coefficients_interpolate(divergence_coefficients, yl0, xl0, &b_f_1, &b_f_2, &b_f_3, &b_f_4);
			     
                            //Pixels out must not be taken into account in the energy data term
                            int pixel_is_disactivated = cvmGet8u(disactivated_pixels_stereoinit,yl0,xl0);

                            //is the point visible at time t+1 ?
                            double xr0 = cvmGet(MapX_Ir0_inner, yl0, xl0);
                            double yr0 = yl0;
                            if(!pixel_is_disactivated &&
                               (xr0 >= (N_x - 1 - DISACTIVATE_BORDER)*h || xr0 < DISACTIVATE_BORDER*h ||
                                yr0 >= (N_y - 1 - DISACTIVATE_BORDER)*h || yr0 < DISACTIVATE_BORDER*h)) {
                                cvmSet8u(disactivated_pixels_stereoinit,yl0,xl0,127);
                                pixel_is_disactivated = 1;
                            }
				     
                            
                            double a_d0_4 = (alpha*mu/(4*h*h))*(b_f_1 + b_f_2 + b_f_3 + b_f_4);
                            if(!pixel_is_disactivated){
                                //Data term estimation
				il0 = cvGetReal2D(Il0_pyramid[level], yl0, xl0);
				il0_X = cvmGet(Il0_X, yl0, xl0);
				il0_Y = cvmGet(Il0_Y, yl0, xl0);	 
				ir0 = cvGetReal2D(Ir0w, yl0, xl0);
				ir0w_X = cvmGet(Ir0w_X, yl0, xl0);
				ir0w_Y = cvmGet(Ir0w_Y, yl0, xl0);
				ir0w_XX = cvmGet(Ir0w_XX, yl0, xl0);
				ir0w_XY = cvmGet(Ir0w_XY, yl0, xl0);

                                // In Maple:
                                // Ir0w(x,y):=Irt(x+d,y);
                                // diff(Ir0w(x,y),x);
                                // diff(Ir0w(x,y),y);
                                // then, express D[1](Ir0) as a function of D[1](Ir0w) and D[2](Ir0w)
                                // and D[2](Ir0) as a function of D[1](Ir0w) and D[2](Ir0w)
                                double det = 1+d0w_X;
                                if (det <= WARP_DETERMINANT_THRESHOLD) {
                                    // image is reversed, set everything to 0
                                    if (!pixel_is_disactivated) {
                                        cvmSet8u(disactivated_pixels_stereoinit,yl0,xl0,191);
                                        pixel_is_disactivated = 1;
                                    }
                                }
                                else {
                                    ir0_X = ir0w_X/det;
                                    ir0_Y = ((1+d0w_X)*ir0w_Y - d0w_Y*ir0w_X)/det;

                                    // derivatives of warped image gradients with respect to non-warped coordinates
                                    // 1-derivatives of the x coordinate of the gradient of the warped image
                                    ir0w_X_X = ir0w_XX/det;
                                    ir0w_X_Y = ((1+d0w_X)*ir0w_XY - d0w_Y*ir0w_XX)/det;
                                    // 2-derivatives of the y coordinate of the gradient of the warped image
                                    ir0w_Y_X = ir0w_XY/det;
                                    ir0w_Y_Y = ((1+d0w_X)*ir0w_YY - d0w_Y*ir0w_XY)/det;
                                }
                                double dd0 = cvmGet(dD0, yl0, xl0);
				
                                //psi' coefficients computation
				double a = square(ir0 + ir0_X*dd0 - il0);
				double b = gamma*(square(ir0w_X + ir0w_X_X*dd0 - il0_X) +
                                                  square(ir0w_Y + ir0w_Y_X*dd0 - il0_Y));
                                //divergence coefficients
				psi_stereo_init = DPsiData(a+b);
		   
				//d equation
                                a_d0_4      +=
                                    psi_stereo_init*(ir0_X*ir0_X + gamma*(ir0w_X_X*ir0w_X_X + ir0w_Y_X*ir0w_Y_X)); 
                            } // if(pixel_is_disactivated)
			      
			    cvmSet(M,i,0,a_d0_4);
				  
			    cvmSet(M,i,1,(-alpha*mu/(4*h*h))*b_f_4);
			     
			    cvmSet(M,i,2,(-alpha*mu/(4*h*h))*b_f_3);
			    
			    cvmSet(M,i,3,(-alpha*mu/(4*h*h))*b_f_2);
                            
			    cvmSet(M,i,4,(-alpha*mu/(4*h*h))*b_f_1);
                      
                            double bdi0 = cvmGet(D0, yl0, xl0);
                            double bdi1 = cvmGet(D0, yl0, xl0+1);
                            double bdi2 = cvmGet(D0, yl0, xl0-1);
                            double bdi3 = cvmGet(D0,yl0+1,xl0);
                            double bdi4 = cvmGet(D0,yl0-1,xl0);
                            double bdi = (alpha*mu/(4*h*h)) *
                                (b_f_1 * bdi1 +
                                 b_f_2 * bdi2 +
                                 b_f_3 * bdi3 +
                                 b_f_4 * bdi4
                                 - (b_f_1 + b_f_2 + b_f_3 + b_f_4) * bdi0);
                            if (!pixel_is_disactivated) {
                                // second term
                                bdi +=
                                    -psi_stereo_init*((ir0 - il0)*ir0_X + gamma*(ir0w_X_X*(ir0w_X - il0_X) + ir0w_Y_X*(ir0w_Y - il0_Y)));
                            }
                            //  if(pixel_is_disactivated)
// 				bdi = bdi*100000000.;
                            cvmSet(B,i,0,bdi);
                        } // else if(methode == 5)
		    } //Pixel inside the integration domain 
		} //for(i = inf; i <= sup; i+=dimension){

                
		//Initial energy computation
                if(logfile && iter_inner == 0 && iter_outer == 0){
                    double Efl, Efr, Est, Es0, Esfl, Esfr, Esd, Esd0;
                    double Edata, Esmooth, Eintensity, Egradient;
                    EnergyComputation(Il0_pyramid[level], Ir0w, Iltw, Irtw,
                                      U, V, Dt, D0,
                                      disactivated_pixels_leftflow, disactivated_pixels_rightflow,
                                      disactivated_pixels_stereoinit, disactivated_pixels_stereo,
                                      N_x, N_y, &Edata, &Esmooth, gamma, lambda, mu,
                                      doleftflow, dorightflow, dodisparity, dodisparity0,
                                      &Efl, &Efr, &Est, &Es0, &Esfl, &Esfr, &Esd, &Esd0, &Eintensity, &Egradient);
                    fprintf(logfile, "** level %d resolution %d x %d\n", level+1, N_x, N_y);
                    fprintf(logfile, "* initial energy:\n");
		    fprintf(logfile, "Edata = %g Esmooth = %g Ed/Es = %g (alpha=%g)\n", Edata, Esmooth, Edata/Esmooth, alpha);
                    fprintf(logfile, "Edata breakdown: Efl = %g Efr = %g Est = %g Es0 = %g\n", Efl, Efr, Est, Es0);
		    fprintf(logfile, "Edata breakdown (squared): Eintensity = %g, Egradient=%g Ei/Eg=%g (gamma=%g)\n", Eintensity, Egradient,  Egradient != 0 ? Eintensity/Egradient : 0., gamma);
                    fprintf(logfile, "Esmooth breakdown (squared): fl = %g fr = %g d-d0 = %g d0 = %g\n", Esfl, Esfr, Esd, Esd0);
                    fflush(logfile);
                }
                if (max_iter_outer == 0) {
                    level = pyramid_level_end-1;
                    goto CLEANUP;
                }



#if DISABLE_PIXEL_DISACTIVATION
#else
		//Preconditioning of M
		//MatrixNormalize(M, B, dimension+4, dimension*N_x, dimension*N_x*(N_y-1)-1);
#endif		     
		//SOR iterations
		cvSetZero(SOR);
		cvSetZero(SOR_prec);
		first_iteration = 1;
		iter_SOR = 0;
		norm_SOR = 0.;

#if DIVERGENCE_8
		CvMat *M2 = cvCreateMat(dimension * N_x * N_y, dimension + 8, CV_32FC1); 
#else	  // !DIVERGENCE_8
		CvMat *M2 = cvCreateMat(dimension * N_x * N_y, dimension + 4, CV_32FC1); 
#endif  // !DIVERGENCE_8
		CvMat *B2 = cvCreateMat(dimension * N_x * N_y,1,CV_32FC1);
		

		cvCopy(M,M2);
		cvCopy(B,B2);
		while ((norm_SOR >= epsilon_SOR && iter_SOR < max_iter_SOR) || first_iteration == 1){
		    first_iteration = 0;

                    for (int sub_iter_SOR = 0; sub_iter_SOR <4; sub_iter_SOR++) {
                    int line, line_begin, line_end, line_incr;
                    int col, col_begin, col_end, col_incr;
                    switch (sub_iter_SOR) {
                        case 0:         // NE-SW
                            line_begin = 0;
                            line_end = N_y * dimension * N_x;
                            line_incr = dimension * N_x;
                            col_begin = 0;
                            col_end = dimension * N_x;
                            col_incr = 1;
                            break;
                        case 1:         // SW-NE
                            line_begin = (N_y-1) * dimension * N_x;
                            line_end = - dimension * N_x;
                            line_incr = - dimension * N_x;
                            col_begin = dimension * N_x - 1;
                            col_end = -1;
                            col_incr = -1;
                            break;
                        case 2:         // NW-SE
                            line_begin = 0;
                            line_end = N_y * dimension * N_x;
                            line_incr = dimension * N_x;
                            col_begin = dimension * N_x - 1;
                            col_end = -1;
                            col_incr = -1;
                            break;
                        default:         // SE-NW
                            line_begin = (N_y-1) * dimension * N_x;
                            line_end = - dimension * N_x;
                            line_incr = - dimension * N_x;
                            col_begin = 0;
                            col_end = dimension * N_x;
                            col_incr = 1;
                    }
                    for (line = line_begin; line != line_end; line+= line_incr) {
                    for (col = col_begin; col != col_end; col+= col_incr) {
                        i = line + col;
			double sum = 0;
                        double diag_term = 0.;
                        
			//int rest = (i%(dimension*N_x))%dimension;
				  
		
			if(i < dimension*N_x){
			    diag_term = cvmGet(M2, i,0); 
			    sum = cvmGet(M2, i,1)*cvmGet(SOR, i+dimension*N_x,0);
			}
			else if(i >= dimension*N_x*(N_y-1)){
			    diag_term = cvmGet(M2, i,0); 
			    sum = cvmGet(M2, i, 1)*cvmGet(SOR, i-dimension*N_x,0);
			}
			else{
			    if(i%(dimension*N_x) == 0){
			
				diag_term = cvmGet(M2, i,0); 
				sum = cvmGet(M2, i,1)*cvmGet(SOR, i+dimension,0);
			    }
			    else if((i+dimension)%(dimension*N_x)==0){
				
				diag_term = cvmGet(M2, i,0); 
				sum = cvmGet(M2, i,1)*cvmGet(SOR, i-dimension,0);
			    }
			    else{

				if(dimension == 1){
				    sum = cvmGet(M2, i,1)*cvmGet(SOR, i-N_x,0) + cvmGet(M2, i,3)*cvmGet(SOR, i-1,0)
                                        + cvmGet(M2, i,2)*cvmGet(SOR, i+N_x,0) + cvmGet(M2, i,4)*cvmGet(SOR, i+1,0);
				    diag_term = cvmGet(M2, i,0);
				}
				else{
			       
				    int rest1 = i%(dimension*N_x);
				    
				    if((dimension == 2 && (rest1 == 1)) ||
                                       (dimension == 3 && (rest1 == 1 || rest1 == 2)) ||
                                       (dimension == 4 && (rest1 == 1 || rest1 == 2 || rest1 == 3))){
					diag_term = cvmGet(M2, i,0); 
					sum = cvmGet(M2, i,1)*cvmGet(SOR, i+dimension,0);
				    }
				    else if((dimension == 4 && (rest1 == 4*N_x - 3 || rest1 == 4*N_x - 2 || rest1 == 4*N_x - 1)) ||
                                            (dimension == 3 && (rest1 == 3*N_x - 2 || rest1 == 3*N_x - 1)) ||
                                            (dimension == 2 && (rest1 == 2*N_x - 1))){
					diag_term = cvmGet(M2, i,0); 
					sum = cvmGet(M2, i,1)*cvmGet(SOR, i-dimension,0);
				    }
				    else{
					int rest2 = rest1 % dimension;
					
					if(dimension == 2){
					    if(rest2 == 0){
					      //u equation
					      sum =
						cvmGet(M2, i,2)*cvmGet(SOR, i-2*N_x,0) +
						cvmGet(M2, i,4)*cvmGet(SOR, i-2    ,0) +
						cvmGet(M2, i,1)*cvmGet(SOR, i+1    ,0) +
						cvmGet(M2, i,3)*cvmGet(SOR, i+2*N_x,0) +
						cvmGet(M2, i,5)*cvmGet(SOR, i+2    ,0);
					      diag_term = cvmGet(M2, i,0);
					    }
					    else if(rest2 == 1){
					      //v equation
						sum =
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i-2*N_x,0) +
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-2    ,0) +
                                                    cvmGet(M2, i,0)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+2*N_x,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+2    ,0) ;
						diag_term = cvmGet(M2, i,1);
					    }
					}
					else if(dimension == 3){
				   
					    if(methode == 1){
						if(rest2 == 0){
						    //u equation
						    sum =
                                                        cvmGet(M2, i,3)*cvmGet(SOR, i-3*N_x,0) +
                                                        cvmGet(M2, i,5)*cvmGet(SOR, i-3    ,0) +
                                                        cvmGet(M2, i,1)*cvmGet(SOR, i+1    ,0) +
                                                        cvmGet(M2, i,2)*cvmGet(SOR, i+2    ,0) +
                                                        cvmGet(M2, i,4)*cvmGet(SOR, i+3*N_x,0) +
                                                        cvmGet(M2, i,6)*cvmGet(SOR, i+3    ,0);
						    diag_term = cvmGet(M2, i,0);
						}
						else if(rest2 == 1){
						    //v equation
						    sum =
                                                        cvmGet(M2, i,3)*cvmGet(SOR, i-3*N_x,0) +
                                                        cvmGet(M2, i,5)*cvmGet(SOR, i-3    ,0) +
                                                        cvmGet(M2, i,0)*cvmGet(SOR, i-1    ,0) +
                                                        cvmGet(M2, i,2)*cvmGet(SOR, i+1    ,0) +
                                                        cvmGet(M2, i,4)*cvmGet(SOR, i+3*N_x,0) +
                                                        cvmGet(M2, i,6)*cvmGet(SOR, i+3    ,0) ;
						    diag_term = cvmGet(M2, i,1);
						}
						else if(rest2 == 2){
						    //d equation
						    sum =
                                                        cvmGet(M2, i,3)*cvmGet(SOR, i-3*N_x,0) +
                                                        cvmGet(M2, i,5)*cvmGet(SOR, i-3    ,0) +
                                                        cvmGet(M2, i,0)*cvmGet(SOR, i-2    ,0) +
                                                        cvmGet(M2, i,1)*cvmGet(SOR, i-1    ,0) +
                                                        cvmGet(M2, i,3)*cvmGet(SOR, i+1    ,0) +
                                                        cvmGet(M2, i,4)*cvmGet(SOR, i+3*N_x,0) +
                                                        cvmGet(M2, i,6)*cvmGet(SOR, i+3    ,0);
						    diag_term = cvmGet(M2, i,2);
						}
					    }
					}
					else if(dimension == 4){
					    if(rest2 == 0){
					        //u equation
#if DIVERGENCE_8                              
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) +
                                                    cvmGet(M2, i,11)*cvmGet(SOR, i-4*(N_x+1),0) +
                                                    cvmGet(M2, i,10)*cvmGet(SOR, i-4*(N_x-1),0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i+2    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+3    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0) +
                                                    cvmGet(M2, i,9)*cvmGet(SOR, i+4*(N_x-1),0) +
                                                    cvmGet(M2, i,8)*cvmGet(SOR, i+4*(N_x+1),0);
					     
#else  // !DIVERGENCE_8
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i+2    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+3    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0);
					      
#endif  // !DIVERGENCE_8
                                                diag_term = cvmGet(M2, i,0);
					    }
					    else if(rest2 == 1){
					        //v equation
#if DIVERGENCE_8
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) +
                                                    cvmGet(M2, i,0)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,11)*cvmGet(SOR, i-4*(N_x+1),0)+
                                                    cvmGet(M2, i,10)*cvmGet(SOR, i-4*(N_x-1),0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+2    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0) +
                                                    cvmGet(M2, i,9)*cvmGet(SOR, i+4*(N_x-1),0) +
                                                    cvmGet(M2, i,8)*cvmGet(SOR, i+4*(N_x+1),0);
					      
#else  // !DIVERGENCE_8
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) +
                                                    cvmGet(M2, i,0)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+2    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0) ;
#endif  // !DIVERGENCE_8
                                                diag_term = cvmGet(M2, i,1);
					    }
					    else if(rest2 == 2){
					        //dprime equation
#if DIVERGENCE_8	
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) - cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x+1,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) - cvmGet(M2, i,6)*cvmGet(SOR, i-4    +1,0) +
                                                    cvmGet(M2, i,11)*cvmGet(SOR, i-4*(N_x+1),0) - cvmGet(M2, i,11)*cvmGet(SOR, i-4*N_x-3,0)+
                                                    cvmGet(M2, i,10)*cvmGet(SOR, i-4*(N_x-1),0) - cvmGet(M2, i,10)*cvmGet(SOR, i-4*(N_x-1)+1,0)+
                                                    cvmGet(M2, i,0)*cvmGet(SOR, i-2    ,0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) - cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x+1,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0) - cvmGet(M2, i,7)*cvmGet(SOR, i+4    +1,0) +
                                                    cvmGet(M2, i,9)*cvmGet(SOR, i+4*(N_x-1),0) - cvmGet(M2, i,9)*cvmGet(SOR, i+4*(N_x-1)+1,0) +
                                                    cvmGet(M2, i,8)*cvmGet(SOR, i+4*(N_x+1),0) - cvmGet(M2, i,8)*cvmGet(SOR, i+4*(N_x+1)+1,0);
#else  // !DIVERGENCE_8
                                                sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) - cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x+1,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) - cvmGet(M2, i,6)*cvmGet(SOR, i-4    +1,0) +
                                                    cvmGet(M2, i,0)*cvmGet(SOR, i-2    ,0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,3)*cvmGet(SOR, i+1    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0) - cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x+1,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0) - cvmGet(M2, i,7)*cvmGet(SOR, i+4    +1,0);
#endif  // !DIVERGENCE_8
                                                diag_term = cvmGet(M2, i,2);
					    }
					    else{
					        //d equation
                                                assert(rest2 == 3);
#if DIVERGENCE_8						
						sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) - cvmGet(M2, i-1 ,4)*cvmGet(SOR, i-4*N_x-1,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) - cvmGet(M2, i-1 ,6)*cvmGet(SOR, i-4    -1,0) +
                                                    cvmGet(M2, i,11)*cvmGet(SOR, i-4*(N_x+1),0) - cvmGet(M2, i-1,11)*cvmGet(SOR, i-4*N_x-5,0)+
						    cvmGet(M2, i,10)*cvmGet(SOR, i-4*(N_x-1),0) - cvmGet(M2, i-1,10)*cvmGet(SOR, i-4*(N_x-1)-1,0)+    
						    cvmGet(M2, i,0)*cvmGet(SOR, i-3    ,0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i-2    ,0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0)- cvmGet(M2, i-1,5)*cvmGet(SOR, i+4*N_x-1,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0)- cvmGet(M2, i-1,7)*cvmGet(SOR, i+4    -1,0) +
						    cvmGet(M2, i,9)*cvmGet(SOR, i+4*(N_x-1),0) - cvmGet(M2, i-1,9)*cvmGet(SOR, i+4*(N_x-1)-1,0) +
						    cvmGet(M2, i,8)*cvmGet(SOR, i+4*(N_x+1),0) - cvmGet(M2, i-1,8)*cvmGet(SOR, i+4*(N_x+1)-1,0);
						
#else  // !DIVERGENCE_8
						sum =
                                                    cvmGet(M2, i,4)*cvmGet(SOR, i-4*N_x,0) - cvmGet(M2, i-1 ,4)*cvmGet(SOR, i-4*N_x-1,0) +
                                                    cvmGet(M2, i,6)*cvmGet(SOR, i-4    ,0) - cvmGet(M2, i-1 ,6)*cvmGet(SOR, i-4    -1,0) +
						    cvmGet(M2, i,0)*cvmGet(SOR, i-3    ,0) +
                                                    cvmGet(M2, i,1)*cvmGet(SOR, i-2    ,0) +
                                                    cvmGet(M2, i,2)*cvmGet(SOR, i-1    ,0) +
                                                    cvmGet(M2, i,5)*cvmGet(SOR, i+4*N_x,0)- cvmGet(M2, i-1,5)*cvmGet(SOR, i+4*N_x-1,0) +
                                                    cvmGet(M2, i,7)*cvmGet(SOR, i+4    ,0)- cvmGet(M2, i-1,7)*cvmGet(SOR, i+4    -1,0);
#endif  // !DIVERGENCE_8
						diag_term = cvmGet(M2, i,3);
					    }
					}
				    }
				}
			    }
			}
			double rold = cvmGet(SOR,i,0);
                        double b = cvmGet(B2,i,0);
                        double rnew = (1.-omega_SOR)*rold + (omega_SOR/diag_term) * (b - sum);
#ifdef DEBUG_PIXEL
                        if(i == ((DEBUG_PIXEL_X+DEBUG_PIXEL_Y*N_x)*dimension+DEBUG_PIXEL_COMPONENT)) {
                            debug_me();
                        }
#endif
                        //assert(fabs(rnew)<(N_x+N_y));
			cvmSet(SOR,i,0,Crop(rnew,MAX_INCR));
		    } // for (col
		    } // for (line
		    } // for (sub_iter_SOR
		    norm_SOR = cvNorm(SOR_prec, SOR, CV_RELATIVE_L2);
		    cvCopy(SOR, SOR_prec);
		    iter_SOR ++;
		    
#if 1
#ifndef _MSC_VER
#warning  "preconditioning disactivated"
#endif
#else
		    //Condition the system by using the increments norm every 4 steps
                    if(iter_SOR < 10){
                        const double eps_sor = 0.001;
			CvMat *Precond =  cvCreateMat(dimension*N_y*N_x,1,CV_32FC1);
			SFMatrixProduct(M, SOR, Precond, N_x, N_y, dimension);
                        //printf("precond!\n");
                        for (i = 0; i < dimension * N_y * N_x; i++) {
                            //int xl0 = (i%(dimension*N_x))/dimension;
                            //int yl0 = i/(dimension*N_x);
                            double value = cvGetReal2D(B,i,0);
                            double norm_inc_sor = fabs(cvGetReal2D(Precond,i,0));
                            //if(norm_inc_sor > eps_sor)
                            //    printf("inc(%d,%d,%d)=%g\n",xl0,yl0,i%dimension,norm_inc_sor);
                            cvSetReal2D(B2,i,0,value/(norm_inc_sor+eps_sor));
                            for(int p = 0; p < dimension + 4; p++){
                                value = cvGetReal2D(M,i,p);
                                cvSetReal2D(M2,i,p,value/(norm_inc_sor+eps_sor));
                            }
                        }
			cvReleaseMat(&Precond);
		    }
		    else
#endif
                    {
                        cvCopy(M,M2);
                        cvCopy(B,B2);
		    }
                    //printf("iter_SOR=%d norm_SOR=%g\n",iter_SOR, norm_SOR);
		} // while ((norm_SOR >= epsilon_SOR && iter_SOR < max_iter_SOR) || first_iteration == 1){
                //printf("iter_SOR=%d norm_SOR=%g\n",iter_SOR, norm_SOR);
		cvReleaseMat(&M2);
		cvReleaseMat(&B2);
#if GMRES
		//Solving the system with a Krylov spaces methods, linear GRMS method
		if(dimension == 4){
		    int N = dimension*N_x*N_y;
		    CvMat *x0 = cvCreateMat(N,1,CV_32FC1);
		    CvMat *xk = cvCreateMat(N,1,CV_32FC1);
		    CvMat *rk = cvCreateMat(N,1,CV_32FC1);
		    cvSetZero(xk);
		    cvSetZero(rk);
		    
		    double norm_GMRES = n_norm(rk, N);
		    int flag = 1;
		    cvCopy(SOR, x0);
		    int iter_GMRES = 0;
		    while((norm_GMRES > epsilon_SOR || flag == 1) && iter_GMRES < 10){
			flag = 0;
			//GmRes Algorithm
			GmRes(M, B, x0, xk, rk, GMRES_KMAX, epsilon_SOR, N_x, N_y, dimension);
			norm_GMRES = n_norm(rk, N);
			cvCopy(xk, x0);  
			iter_GMRES++;
		    }
		    cvCopy(xk, SOR);
		    cvReleaseMat(&x0);
		    cvReleaseMat(&xk);
		    cvReleaseMat(&rk);
		}
#endif //GRMS
	
                if (doleftflow) {
                    cvCopy(dU, dU_prec); 
		    cvCopy(dV, dV_prec);
                }
                if (dodisparity) {
                    cvCopy(dDt, dDt_prec);
                }
                if (dodisparity0) {
                    cvCopy(dD0, dD0_prec);
                }
                
		for(i = 0; i < dimension*N_x*N_y; i+=dimension){
                    xl0 = (i%(dimension*N_x))/dimension;
                    yl0 = i/(dimension*N_x);
                    int d = 0;
                    if (doleftflow) {
                        double du = cvmGet(SOR, i + d,0); d++;
                        double dv = cvmGet(SOR, i + d,0); d++;
			cvmSet(dU, yl0, xl0, Crop(du,MAX_INCR));
			cvmSet(dV, yl0, xl0, Crop(dv,MAX_INCR));
                    }
                   
                    if (dodisparity) {
                        double ddt = cvmGet(SOR, i + d,0); d++;
			cvmSet(dDt, yl0, xl0, Crop(ddt,MAX_INCR));
                    }
                    if (dodisparity0) {
			double dd0 = cvmGet(SOR, i + d,0); d++;
                        cvmSet(dD0, yl0, xl0, Crop(dd0,MAX_INCR));
                        //cvmSet(dD0, yl0, xl0, dd0);
		    }
                    assert(d == dimension);
		}

                if (OF_ZeroTopBorder) {
                    if (doleftflow) {
                        for(xl0 = 0; xl0 < N_x; xl0++){
                            cvmSet(dU, 0, xl0, 0);
                            cvmSet(dV, 0, xl0, 0);
                            cvmSet(dU, 1, xl0, 0);
                            cvmSet(dV, 1, xl0, 0);
                        }
                        for(yl0 = 0; yl0 < N_y*7/8; yl0++){
                            cvmSet(dU, yl0, 0, 0);
                            cvmSet(dV, yl0, 0, 0);
                            cvmSet(dU, yl0, N_x-1, 0);
                            cvmSet(dV, yl0, N_x-1, 0);
                            cvmSet(dU, yl0, 1, 0);
                            cvmSet(dV, yl0, 1, 0);
                            cvmSet(dU, yl0, N_x-2, 0);
                            cvmSet(dV, yl0, N_x-2, 0);
                            cvmSet(dU, yl0, 2, 0);
                            cvmSet(dV, yl0, 2, 0);
                            cvmSet(dU, yl0, N_x-3, 0);
                            cvmSet(dV, yl0, N_x-3, 0);
                        }
                    }
                    if (dodisparity) {
                        for(xl0 = 0; xl0 < N_x; xl0++){
                            cvmSet(dDt, 0, xl0, -40.*N_x/854 - cvmGet(Dt,0,xl0));
                            cvmSet(dDt, 1, xl0, -40.*N_x/854 - cvmGet(Dt,1,xl0));
                        }
                        for(yl0 = 0; yl0 < N_y*7/8; yl0++){
                            cvmSet(dDt, yl0, 0, -40.*N_x/854 - cvmGet(Dt,yl0,0));
                            cvmSet(dDt, yl0, N_x-1, -40.*N_x/854 - cvmGet(Dt,yl0,N_x-1));
                            cvmSet(dDt, yl0, 1, -40.*N_x/854 - cvmGet(Dt,yl0,1));
                            cvmSet(dDt, yl0, N_x-2, -40.*N_x/854 - cvmGet(Dt,yl0,N_x-2));
                            cvmSet(dDt, yl0, 2, -40.*N_x/854 - cvmGet(Dt,yl0,2));
                            cvmSet(dDt, yl0, N_x-3, -40.*N_x/854 - cvmGet(Dt,yl0,N_x-3));
                        }
                    }
                    if (dodisparity0) {
                        for(xl0 = 0; xl0 < N_x; xl0++){
                            cvmSet(dD0, 0, xl0, -40.*N_x/854 - cvmGet(D0,0,xl0));
                            cvmSet(dD0, 1, xl0, -40.*N_x/854 - cvmGet(D0,1,xl0));
                        }
                        for(yl0 = 0; yl0 < N_y*7/8; yl0++){
                            cvmSet(dD0, yl0, 0, -40.*N_x/854 - cvmGet(D0,yl0,0));
                            cvmSet(dD0, yl0, N_x-1, -40.*N_x/854 - cvmGet(D0,yl0,N_x-1));
                            cvmSet(dD0, yl0, 1, -40.*N_x/854 - cvmGet(D0,yl0,1));
                            cvmSet(dD0, yl0, N_x-2, -40.*N_x/854 - cvmGet(D0,yl0,N_x-2));
                            cvmSet(dD0, yl0, 2, -40.*N_x/854 - cvmGet(D0,yl0,2));
                            cvmSet(dD0, yl0, N_x-3, -40.*N_x/854 - cvmGet(D0,yl0,N_x-3));
                        }
                    }
                }
                norm_inner = 0;
                if (doleftflow) {
                    norm_inner = fmax(norm_inner, cvNorm(dU_prec, dU, CV_RELATIVE_L2));
                    norm_inner = fmax(norm_inner, cvNorm(dV_prec, dV, CV_RELATIVE_L2));
                }
               
                if (dodisparity) {
                    norm_inner = fmax(norm_inner, cvNorm(dDt_prec, dDt, CV_RELATIVE_L2));
                }
                if (dodisparity0) {
                    norm_inner = fmax(norm_inner, cvNorm(dD0_prec, dD0, CV_RELATIVE_L2));
                   
               }
		     
	
		iter_inner++;
	    }

	    

            if (doleftflow) {
                if (SF_U_increment_FileName) {
                    SaveResults(SF_U_increment_FileName, dU);
                }
                if (SF_V_increment_FileName) {
                    SaveResults(SF_V_increment_FileName, dV);
                }
                cvCopy(U, U_prec);
		cvAdd(U, dU, U);
                CropArr(U, N_x);
                cvCopy(V, V_prec);
		cvAdd(V, dV, V);
                CropArr(V, N_y);
		    
            }

	     
            if (dodisparity0) {
                if (SF_D0_increment_FileName) {
                    SaveResults(SF_D0_increment_FileName, dD0);
                }
                cvCopy(D0, D0_prec);
		cvAdd(D0, dD0, D0);
                CropArr(D0, N_x);
            }

            if (dodisparity) {
                if (SF_Dt_increment_FileName) {
                    SaveResults(SF_Dt_increment_FileName, dDt);
                }
                cvCopy(Dt, Dt_prec);
		cvAdd(Dt, dDt, Dt);
                CropArr(Dt, N_x);
            }

            norm = 0;
	    if(doleftflow){
		norm = fmax(norm, cvNorm(U_prec, U, CV_RELATIVE_L2));
		norm = fmax(norm, cvNorm(V_prec, V, CV_RELATIVE_L2));
            }
           
            if (dodisparity) {
		norm = fmax(norm, cvNorm(Dt_prec, Dt, CV_RELATIVE_L2));
	    }
            if (dodisparity0) {
		norm = fmax(norm, cvNorm(D0_prec, D0, CV_RELATIVE_L2));
	    }

	    
	    if (doleftflow) {
                if (SF_U_FileName) {
                    SaveResults(SF_U_FileName, U);
                }
                if (SF_V_FileName) {	     
                    SaveResults(SF_V_FileName, V);
                }
	    }
	  
            if (dodisparity) {
                if (SF_Dt_FileName) {
                    SaveResults(SF_Dt_FileName, Dt);
                }
	    }
	    if (dodisparity0) {
                if (SF_D0_FileName) {
                    SaveResults(SF_D0_FileName, D0);
                }
	    }
            if (SF_disactivated_flowleft_FileName && disactivated_pixels_leftflow) {
                SaveResults(SF_disactivated_flowleft_FileName, disactivated_pixels_leftflow, 0, 0., 255.);
            }
            if (SF_disactivated_flowright_FileName && disactivated_pixels_rightflow) {
                SaveResults(SF_disactivated_flowright_FileName, disactivated_pixels_rightflow, 0, 0., 255.);
            }
            if (SF_disactivated_stereoinit_FileName && disactivated_pixels_stereoinit) {
                SaveResults(SF_disactivated_stereoinit_FileName, disactivated_pixels_stereoinit, 0, 0., 255.);
            }
            if (SF_disactivated_stereo_FileName && disactivated_pixels_stereo) {
                SaveResults(SF_disactivated_stereo_FileName, disactivated_pixels_stereo, 0, 0., 255.);
            }
            
	    iter_outer++;
	}
	

        if (logfile) {
	    //Maps update
	    if (dorightflow || dodisparity0){
		cvAdd(MapIdentityX, D0, MapX_Ir0);
		cvCopy(MapIdentityY,  MapY_Ir0);
	    }
            if (doleftflow || dodisparity) {
		cvAdd(MapIdentityX, U, MapX_Ilt);
		cvAdd(MapIdentityY, V, MapY_Ilt);
            }
            if (dorightflow || dodisparity) {
		cvAdd(MapIdentityX, U, MapX_Irt);
		cvAdd(MapX_Irt, Dt, MapX_Irt);
		cvAdd(MapIdentityY, V, MapY_Irt);
            }
            fprintf(logfile, "* iterations: %d\n", iter_outer);
            fprintf(logfile, "SOR: iter=%d<%d norm_SOR=%g<%g\n", iter_SOR, max_iter_SOR, norm_SOR, epsilon_SOR);
            fprintf(logfile, "secondary fixed point: iter_inner=%d<=%d norm_inner=%g<%g\n",iter_inner,max_iter_inner,norm_inner,epsilon_inner);
            fprintf(logfile, "primary fixed point: iter_outer=%d<=%d norm=%g<%g\n", iter_outer, max_iter_outer, norm, epsilon_outer);
            fflush(logfile);

#if DISABLE_PIXEL_DISACTIVATION
#warning "no disactivation of occluded pixels"
            if (doleftflow || (dorightflow && dodisparity && dodisparity0)) {
                cvSetZero(disactivated_pixels_leftflow);
            }
            if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
                cvSetZero(disactivated_pixels_rightflow);
            }
            if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
                cvSetZero(disactivated_pixels_stereoinit);
            }
            if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
                cvSetZero(disactivated_pixels_stereo);
            }
#else //!DISABLE_PIXEL_DISACTIVATION
            // Initialize occlusion maps, for each of the data terms:
            // - first, compute stereo disparity the destination image, using Z-buffering
            // - then, remap those disparities to the left image at time t
            // - add disparity tolerance (e.g. 1.) to the remapped disparity
            // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
            
	    CvMat *D_warped = cvCreateMat(N_y, N_x, CV_32FC1);
            CvMat *D_backwarped = cvCreateMat(N_y, N_x, CV_32FC1);
            CvMat *D_rewarped = cvCreateMat(N_y, N_x, CV_32FC1);


	    if (Dt && (doleftflow || (dorightflow && dodisparity && dodisparity0))) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Ilt, MapY_Ilt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Ilt, MapY_Ilt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_leftflow, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_leftflow, 127);
            }
            if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(D0, D_warped, MapX_Ir0, MapY_Ir0, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Ir0, MapY_Ir0, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(D0, D_backwarped, disactivated_pixels_stereoinit, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_stereoinit, 127);
//#warning desactivation totale
                //cvSet(disactivated_pixels_stereoinit, cvScalar(127));
           }
            // not so sure about how to compute right flow occlusion...
            if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Irt, MapY_Irt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Irt, MapY_Irt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_rightflow, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_rightflow, 127);
            }
            // same as right flow
            if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
                // - first, compute stereo disparity the destination image, using Z-buffering
                ZBufMap_32f(Dt, D_warped, MapX_Irt, MapY_Irt, (float)N_x);
                // - then, remap those disparities to the left image at time t
                cvRemap(D_warped, D_backwarped, MapX_Irt, MapY_Irt, CV_INTER_NN+CV_WARP_FILL_OUTLIERS, cvScalar(N_x));
                // - add disparity tolerance (e.g. 1.) to the remapped disparity
                cvAddS(D_backwarped, cvScalar(DISACTIVATION_THRESHOLD), D_backwarped);
                // comparison between the remapped disparity + tolerance and the disparity gives the occlusion map
                cvCmp(Dt, D_backwarped, disactivated_pixels_stereo, CV_CMP_GE);
                cvmSet8u_borders(disactivated_pixels_stereo, 127);
            }
            cvReleaseMat(&D_warped);
            cvReleaseMat(&D_backwarped);
            cvReleaseMat(&D_rewarped);
#endif //!DISABLE_PIXEL_DISACTIVATION
	    // Images warping + Warped images gradient computation
	    // (Warped images are smoothed)
	    if (doleftflow || dodisparity) {
		cvRemap(Ilt_pyramid[level], Iltw, MapX_Ilt, MapY_Ilt);
		SmoothImage(Iltw);
	    }
	    if (dodisparity0 || dorightflow) {
		cvRemap(Ir0_pyramid[level], Ir0w, MapX_Ir0, MapY_Ir0);
		SmoothImage(Ir0w);
	    }
	    if (dodisparity || dorightflow) {
		cvRemap(Irt_pyramid[level], Irtw, MapX_Irt, MapY_Irt);
		SmoothImage(Irtw);
	    }		    
            for(yl0 = 0; yl0 < N_y; yl0++){
                for(xl0 = 0; xl0 < N_x; xl0++){
                    //Pixels out must not be taken into account in the energy data term
                    int pixel_doleftflow = 0;
                    int pixel_dorightflow = 0;
                    int pixel_dostereoinit = 0;
                    int pixel_dostereo = 0;
                    if (disactivated_pixels_leftflow)
                        pixel_doleftflow = !cvmGet8u(disactivated_pixels_leftflow,yl0,xl0);
                    if (disactivated_pixels_rightflow)
                        pixel_dorightflow = !cvmGet8u(disactivated_pixels_rightflow,yl0,xl0);
                    if (disactivated_pixels_stereoinit)
                        pixel_dostereoinit = !cvmGet8u(disactivated_pixels_stereoinit,yl0,xl0);
                    if (disactivated_pixels_stereo)
                        pixel_dostereo = !cvmGet8u(disactivated_pixels_stereo,yl0,xl0);

                    //is the point visible at time t+1 ?
                    if (pixel_doleftflow) {
                        double xlt = cvmGet(MapX_Ilt, yl0, xl0);
                        double ylt = cvmGet(MapY_Ilt, yl0, xl0);
                        if(xlt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xlt < DISACTIVATE_BORDER*h ||
                           ylt >= (N_y - 1 - DISACTIVATE_BORDER)*h || ylt < DISACTIVATE_BORDER*h){
                            cvmSet8u(disactivated_pixels_leftflow,yl0,xl0,127);
                            pixel_doleftflow = 0;
                        }
                    }

                    //On the right image at time t
                    if (pixel_dostereoinit || pixel_dorightflow) {
                        double xr0 = cvmGet(MapX_Ir0, yl0, xl0);
                        double yr0 = yl0;
                        if(xr0 >= (N_x - 1 - DISACTIVATE_BORDER)*h || xr0 <= DISACTIVATE_BORDER*h ||
                           yr0 >= (N_y - 1 - DISACTIVATE_BORDER)*h || yr0 <= DISACTIVATE_BORDER*h) {
                            if (pixel_dostereoinit) {
                                cvmSet8u(disactivated_pixels_stereoinit,yl0,xl0,127);
                                pixel_dostereoinit = 0;
                            }
                            if (pixel_dorightflow) {
                                cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,127);
                                pixel_dorightflow = 0;
                            }
                        }
                    }
                            
                    //On the right image at time t+1
                    if (pixel_dostereo || pixel_dorightflow) {
                        double xrt = cvmGet(MapX_Irt, yl0, xl0);
                        double yrt = cvmGet(MapY_Irt, yl0, xl0);
                        if(xrt >= (N_x - 1 - DISACTIVATE_BORDER)*h || xrt < DISACTIVATE_BORDER*h ||
                           yrt >= (N_y - 1 - DISACTIVATE_BORDER)*h || yrt < DISACTIVATE_BORDER*h) {
                            if (pixel_dostereo) {
                                cvmSet8u(disactivated_pixels_stereo,yl0,xl0,127);
                                pixel_dostereo = 0;
                            }
                            if (pixel_dorightflow) {
                                cvmSet8u(disactivated_pixels_rightflow,yl0,xl0,127);
                                pixel_dorightflow = 0;
                            }
                        }
                    }
                }
            }

            double Efl, Efr, Est, Es0, Esfl, Esfr, Esd, Esd0;
            double Edata, Esmooth, Eintensity, Egradient;
            EnergyComputation(Il0_pyramid[level], Ir0w, Iltw, Irtw,
                              U, V, Dt, D0,
                              disactivated_pixels_leftflow, disactivated_pixels_rightflow,
                              disactivated_pixels_stereoinit, disactivated_pixels_stereo,
                              N_x, N_y, &Edata, &Esmooth, gamma, lambda, mu,
                              doleftflow, dorightflow, dodisparity, dodisparity0,
                              &Efl, &Efr, &Est, &Es0, &Esfl, &Esfr, &Esd, &Esd0, &Eintensity, &Egradient);
            fprintf(logfile, "* final energy:\n");
            fprintf(logfile, "Edata = %g Esmooth = %g Ed/Es = %g (alpha=%g)\n", Edata, Esmooth, Edata/Esmooth, alpha);
            fprintf(logfile, "Edata breakdown: Efl = %g Efr = %g Est = %g Es0 = %g\n", Efl, Efr, Est, Es0);
            fprintf(logfile, "Edata breakdown (squared): Eintensity = %g, Egradient=%g Ei/Eg=%g (gamma=%g)\n", Eintensity, Egradient, Egradient != 0 ? Eintensity/Egradient : 0., gamma);
            fprintf(logfile, "Esmooth breakdown (squared): fl = %g fr = %g d-d0 = %g d0 = %g\n", Esfl, Esfr, Esd, Esd0);
            fflush(logfile);
        }

      CLEANUP:
	//let's clean
        cvReleaseMat(&derivative_tmp);
        cvReleaseMat(&divergence_coefficients);
        cvReleaseMat(&Il0_X);
        cvReleaseMat(&Il0_Y);
        if (doleftflow || dodisparity) {
            cvReleaseMat(&Iltw_X);
            cvReleaseMat(&Iltw_Y);
            cvReleaseMat(&Iltw_XX);
            cvReleaseMat(&Iltw_XY);
            cvReleaseMat(&Iltw_YY);
        }
        if (dodisparity0 || dorightflow) {
            cvReleaseMat(&Ir0w_X);
            cvReleaseMat(&Ir0w_Y);
            cvReleaseMat(&Ir0w_XX);
            cvReleaseMat(&Ir0w_XY);
            cvReleaseMat(&Ir0w_YY);
        }
        if (dodisparity || dorightflow) {
            cvReleaseMat(&Irtw_X);
            cvReleaseMat(&Irtw_Y);
            cvReleaseMat(&Irtw_XX);
            cvReleaseMat(&Irtw_XY);
            cvReleaseMat(&Irtw_YY);
        }
	cvReleaseMat(&MapIdentityX);
	cvReleaseMat(&MapIdentityY);
        if (doleftflow || dodisparity) {
            cvReleaseMat(&Iltw);
            cvReleaseMat(&MapX_Ilt);
            cvReleaseMat(&MapY_Ilt);
            cvReleaseMat(&MapX_Ilt_inner);
            cvReleaseMat(&MapY_Ilt_inner);
        }
        if (dorightflow || dodisparity) {
            cvReleaseMat(&Irtw);
            cvReleaseMat(&MapX_Irt);
            cvReleaseMat(&MapY_Irt);
            cvReleaseMat(&MapX_Irt_inner);
            cvReleaseMat(&MapY_Irt_inner);
        }
        if (dorightflow || dodisparity0) {
            cvReleaseMat(&Ir0w);
            cvReleaseMat(&MapX_Ir0);
            cvReleaseMat(&MapY_Ir0);
            cvReleaseMat(&MapX_Ir0_inner);
            cvReleaseMat(&MapY_Ir0_inner);
        }
	
	cvReleaseMat(&M);
	cvReleaseMat(&B);

#ifdef M_IMAGE
	if(doleftflow && dodisparity0 && dodisparity)
            cvReleaseMat(&M_image);
#endif

        if (doleftflow || (dorightflow && dodisparity && dodisparity0)) {
            cvReleaseMat(&disactivated_pixels_leftflow);
        }
        if (dorightflow || (doleftflow && dodisparity && dodisparity0)) {
            cvReleaseMat(&disactivated_pixels_rightflow);
        }
        if (dodisparity0 || (doleftflow && dorightflow && dodisparity)) {
            cvReleaseMat(&disactivated_pixels_stereoinit);
        }
        if (dodisparity || (doleftflow && dorightflow && dodisparity0)) {
            cvReleaseMat(&disactivated_pixels_stereo);
        }
	
	if(doleftflow){
	    cvReleaseMat(&U_prec); 
	    cvReleaseMat(&V_prec);
	     
	   
	    cvReleaseMat(&dU);
	    cvReleaseMat(&dV);
	    cvReleaseMat(&dU_X);
	    cvReleaseMat(&dU_Y);
	    cvReleaseMat(&dV_X);
	    cvReleaseMat(&dV_Y);
	   

	 
	 
	    cvReleaseMat(&U_X);
	    cvReleaseMat(&U_Y);
	    cvReleaseMat(&V_X);
	    cvReleaseMat(&V_Y);
		
	    cvReleaseMat(&dU_prec);
	    cvReleaseMat(&dV_prec);
	}
	 
	if(dodisparity) {
	    cvReleaseMat(&Dt_prec);
	     
	    
	    cvReleaseMat(&dDt);
	    cvReleaseMat(&dDt_X);
	    cvReleaseMat(&dDt_Y);
	 
	 
	    
	    cvReleaseMat(&Dt_X);
	    cvReleaseMat(&Dt_Y);
	 
	    cvReleaseMat(&dDt_prec);
	   
	}
	 
	if(dodisparity0 || dodisparity){
	    cvReleaseMat(&D0_prec);
	    cvReleaseMat(&D0_X);
	    cvReleaseMat(&D0_Y);
	    
        }
        if (dodisparity0) {
	    cvReleaseMat(&dD0);
	    cvReleaseMat(&dD0_X);
	    cvReleaseMat(&dD0_Y);
	    cvReleaseMat(&dD0_prec);
	}
	cvReleaseMat(&SOR);
	cvReleaseMat(&SOR_prec);	

	//if this is not the last level, images must be upscaled before iterations
	if (level != (pyramid_level_end-1)) {
            const CvSize next_level_size = cvGetSize(Il0_pyramid[level - 1]);
            CvMat *Interpolated_Data;

            if(doleftflow){
                Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
                cvResize(U, Interpolated_Data,CV_INTER_LINEAR);
                double uscale = (double)Interpolated_Data->width / U->width;
                cvReleaseMat(&U);
                U = Interpolated_Data;
                SmoothData(U);
                cvConvertScale(U, U, uscale);
                
                Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
                cvResize(V, Interpolated_Data,CV_INTER_LINEAR);
                double vscale = (double)Interpolated_Data->height / V->height;
                cvReleaseMat(&V);
                V = Interpolated_Data;
                SmoothData(V);
                cvConvertScale(V, V, vscale);
	    }
            if(dodisparity || doleftflow) {
		Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
		cvResize(Dt, Interpolated_Data,CV_INTER_LINEAR);
                double dscale = (double)Interpolated_Data->width / Dt->width;
		cvReleaseMat(&Dt);
                Dt = Interpolated_Data;
		SmoothData(Dt);
		cvConvertScale(Dt, Dt, dscale);
	    }
            if(dodisparity0) {
		Interpolated_Data =  cvCreateMat(next_level_size.height, next_level_size.width, CV_32FC1);
		cvResize(D0, Interpolated_Data,CV_INTER_LINEAR);
                double dscale = (double)Interpolated_Data->width / D0->width;
		cvReleaseMat(&D0);
                D0 = Interpolated_Data;
		SmoothData(D0);
		cvConvertScale(D0, D0, dscale);
	    } 	
        }
    } // for(int level = pyramid_level_start - 1; level >= pyramid_level_end - 1; level--){
    
    if(dodisparity){
        if (Dt_end)
            cvCopy(Dt, Dt_end);
    }
    if (dodisparity || doleftflow) {
        if (Dt)
            cvReleaseMat(&Dt);
    }
     
    if(doleftflow){
        if (U_end)
            cvCopy(U, U_end);
        if (V_end)
            cvCopy(V, V_end);
        cvReleaseMat(&U);
        cvReleaseMat(&V);
    }
    
    if(dodisparity0){
        if (D0_end)
            cvCopy(D0, D0_end);
        cvReleaseMat(&D0);
    }
}




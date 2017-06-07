#define BP_ITER 5       // number of BP iterations at each scale
#define BP_LEVELS 5     // number of scales

//#define BP_DISC_K 1.7F         // truncation of discontinuity cost
#define BP_DISC_K 1700.F         // truncation of discontinuity cost
//#define BP_DATA_K 15.0F        // truncation of data cost
#define BP_DATA_K 10000.0F        // truncation of data cost
#define BP_LAMBDA 0.07F        // weighting of data cost

#define BP_INF 1E20F     // large cost
//#define BP_VALUES 16    // number of possible disparities
//#define BP_SCALE 16     // scaling from disparity to graylevel in output
#define BP_VALUES 128    // number of possible disparities
#define BP_SCALE 2     // scaling from disparity to graylevel in output

#define BP_SIGMA 0.7F    // amount to smooth the input images

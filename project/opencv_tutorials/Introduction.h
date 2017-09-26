

/** brief



 OpenCV

 create a directory release and then invoke
 cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=../build ..
 It is important to build with gtk support, otherwise, the opencv will built, but the programs wont show any images
 for example cvNamedWindow wont work.

 Motion Sensing Applications ( Optical Flow )

 Kinetic Sensors: 1 Monochrome CMOS Receptor ( in short a camera lens ),
 1 Infrared Camera to monitor depth and
 1 Microphone array.
 Additionally 3 axis accelerometers to find out if the kinetic is placed on a shelf which is
 tilted. The angle would be then compensated in the software.

 These sensors have a narrow imaging area of about 58 degrees horizontal and 43 degrees vertical. It can also not
 detect anything within the first 0.5 meters (~20 inches). Highly reflective surfaces, such as metals, glass, or
 mirrors cannot be detected by the 3D vision sensor.


 Depth cloud: Depth cloud is another name for the depth_image produced by the 3D sensor, such as the Kinect, ASUS,
 and PrimeSense depth cameras.
 Point cloud: A point cloud is a set of points with x, y, and z coordinates that represent the surface of an object.
 Registered DepthCloud and Registered PointCloud: These terms are used by ROS for special DepthCloud or PointCloud
 data colored by the rgb image data. These data streams are available when the depth_registration option is selected
 (set to true).

 Without BUILD_FFMPEG flag

 --   Checking for modules 'libavcodec;libavformat;libavutil;libswscale'
 --   Found libavcodec, version 56.60.100
 --   Found libavformat, version 56.40.101
 --   Found libavutil, version 54.31.100
 --   Found libswscale, version 3.1.101

 sudo apt-get install python-opencv

 -Wl,Blsymbolic ; --enable-pic ; --disable-static

 --------------------------------------------------------------------------------

 HighGUI - Functions, that allow to interact with the operating system, the file system and the the hardware such as
 cameras.
 - read and write graphics-related files
 - handle mouse and keyboard events
 - create sliders etc.
 - much of the code is redundant, because QT does it better.
 - still and video images.
 - provides a set of XML/YAML based functions for human readable, text based format.



 cv::cvtColor() - convert from one color space to another and also specify the number of destination channels.
 cv::mixChannels() - mix channels from one array to another

 cv::abs - element by element absolute of the matrix
 cv::absdiff() - element by element subtraction and saturation between two matrices
 cv::add() - elment by element addition and saturation between two matrices
 cv::addWeighted() - element by element weight (alpha,beta) mult and offset (gamma) addition between two matrices.
 cv::exp() - element by element exponential
 cv::convertScaleAbs() - scale, offset, absolut and saturate on single matrix.
 cv::log() - natural log element by element. For negative or zero, the destination element is a large neg value.
 cv::divide() - divide element by element matrix with a scalar or matrix and a matrix.
 cv::multiply() - multiply element by element btw. two matrix. scaling and saturation is also done.
 cv::eigen - calculautes all or few largest eigen value and eigen vector for a symmetric matrix with floating data pt.
 cv::bitwise_and(), cv::bitwise:or(), c::bitweise:not(), cv::bitwise_or(), cv::bitwise_xor() - element by element.
 cv::pow() - src^pow

 cv::calcCovarMatrix() - mean and covariance matrix of the 1 by n or n by 1 samples.
 cv::mean() - mean of all the value including the channels. the computation is done only on non masked elements.
 cv::meanStdDev() - mean and standard deviation of all the value including the channels.
 cv::Mahalanobis() - Calculates inverse covariance ( distance ) from a point x and the distributions mean.
 cv:cartToPolar() - two input arrays, each consists of x and the other consists of y component. Result is mag and angle.
 cv::magnitude() - two input arrays, each consists of x and the other consists of y component. Result is mag.
 cv::phase() - like magnitude, Result is angle
 cv::determinant() - determinant of a square matrix with floating data pt. and 1 channel. Large with Gaussian elimin.
 cv::mulTransposed() - multiple with transpose, optionally subtract delta and also scale.
 cv::polarToCart()

 cv::completeSymm() - copy the lower triangle to the upper or vice versa. Makes matrix symm. Diag remains unchanged.
 cv::flip() - flip around x axis or y axis or both
 cv::invert() - inverts a square matrix, but may be non square if pseudo inverse. Uses Gauss elim, SVD, and Cholesky.

 cv::countNonZero() - counts the number of nonzero pixels in the matrix.
 cv::compare() - element by element comparison and either 255 or 0, if the matrix elements ( =, >, <, <=, >=, != )
 cv::inRange() - Only for 1 D multi channel, element <, > lower and upper, then 255 otherwise 0.
 cv::checkRange() - returns true or false or exception if any element in the array is out of minVal or maxVal.
 cv::max(), cv::min()  - computes the max / min between two matrices.
 cv::merge() - merge different cv::Mat in one. The inputs are a array of std::vector<cv::Mat>
 cv::minMaxIdx() - for 1D, eturns minVal, maxVal and their corresponding indexes
 cv::minMaxLoc() - for 2D, returns minVal, maxVal and their corresponding cv::Points.

 cv::randu() - create random array from min number and max number. Uniformly distributed random values.
 Generated by Multiply with Carry Algorithm.
 cv::randn() - create random array from mean and standard deviation. Normally distributed random values.
 Gaussian-distribution random numbers are generated using Ziggurat algorithm.
 cv::randShuffle() - shuffles a 1D array.

 cv::solvePoly() : solve univariate polynomials
 cv::solve() : linear equation and least square problems ( uses SVD pr QR).
 cv::cubic() :
 cv::solveLP : linear programming problem using the Simplex Algorithm (Simplex Method).

 cv::imread() - reads an image, GRAY, BGR, BGRA etc.
     The first few bytes of the file ( signature or magic sequence ) determines the codec.
     All of them load 8 bits, except ANYDEPTH. Only if WITH_JPEG=ON, otherwise no jpeg extension can be read.
     cv::IMREAD_COLOR 	    Always load to three-channel array. 	default
     cv::IMREAD_GRAYSCALE 	Always load to single-channel array.
     cv::IMREAD_ANYCOLOR 	Channels as indicated by file (up to three).
     cv::IMREAD_ANYDEPTH 	Allow loading of more than 8-bit depth.
     cv::IMREAD_UNCHANGED 	Equivalent to combining: cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH
 cv::imwrite() - writes an image, compression algo should be given. jpg, png, tiff, bmp,
    PNG is not lossless. It depends on the compression. 0 is lossless. But the default is 3 and is lossy.
    imwrite will store mostly 8bit per channel, but 16 bit and float format is also allowed. Return value is true or
    false if the image was saved or not.
    cv::IMWRITE_JPG_QUALITY 	JPEG quality 	0–100 	95
    cv::IMWRITE_PNG_COMPRESSION 	PNG compression (higher values mean more compression) 	0-9 	3
    cv::IMWRITE_PXM_BINARY 	Use binary format for PPM, PGM, or PBM files 	0 or 1 	1
 cv::Mat::isempty() - check if its really an image?
 cv::imencode() - compresses / encodes a Mat to vector<uchar> in the memory. No file is written on the filesystem.
 cv::imdecode() - decompresses / decodes a vector<uchar>from the memory to Mat.
    returns an empty array if the buffer it is given is empty or invalid / unusable data.
 cv::VideoCapture() - either -1, filename or device. .mpg, avi. If device, then identifier+domain.
    In case of a file, OpenCV needs to know how the file should be decoded. In case of a device, an identifier (
    which camera should be acccessed, for single camera on the computer, it is 0 ) and a domain needs to be supplied.
    The domain is a predetermined constant, and determines how OpenCV would like to talk to the camera.
    When empty, then open() needs to be called.
 cv::VideoCapture()::isOpened() - check if the video can really be opened?
 cv::VideoCapture()::read()
 cv::VideoCapture& cv::VideoCapture::operator>>
 cv::VideoCapture::grab( void )
 cv::VideoCapture::retrieve()
 cv::VideoCapture::get()
 cv::VideoCapture::set()
    cv::CAP_PROP_POS_MSEC 	  	Current position in video file (milliseconds) or video capture timestamp
    cv::CAP_PROP_POS_FRAMES 	  	Zero-based index of next frame
    cv::CAP_PROP_POS_AVI_RATIO 	  	Relative position in the video (range is 0.0 to 1.0)
    cv::CAP_PROP_FRAME_WIDTH 	  	Width of frames in the video
    cv::CAP_PROP_FRAME_HEIGHT 	  	Height of frames in the video
    cv::CAP_PROP_FPS 	  	Frame rate at which the video was recorded
    cv::CAP_PROP_FOURCC 	  	Four character code indicating codec
    cv::CAP_PROP_FRAME_COUNT 	  	Total number of frames in a video file
    cv::CAP_PROP_FORMAT 	  	Format of the Mat objects returned (e.g., CV_8UC3)
    cv::CAP_PROP_MODE 	  	Indicates capture mode; values are specific to video backend being used (e.g., DC1394)
    cv::CAP_PROP_BRIGHTNESS 	✓ 	Brightness setting for camera (when supported)
    cv::CAP_PROP_CONTRAST 	✓ 	Contrast setting for camera (when supported)
    cv::CAP_PROP_SATURATION 	✓ 	Saturation setting for camera (when supported)
    cv::CAP_PROP_HUE 	✓ 	Hue setting for camera (when supported)
    cv::CAP_PROP_GAIN 	✓ 	Gain setting for camera (when supported)
    cv::CAP_PROP_EXPOSURE 	✓ 	Exposure setting for camera (when supported)
    cv::CAP_PROP_CONVERT_RGB 	✓ 	If nonzero, captured images will be converted to have three channels
    cv::CAP_PROP_WHITE_BALANCE 	✓ 	White balance setting for camera (when supported)
    cv::CAP_PROP_RECTIFICATION 	✓ 	Rectification flag for stereo cameras (DC1394-2.x only)
        cap.get( cv::CAP_PROP_FOURCC );

 cv::VideoWriter::VideoWriter(
    .jpg or .jpeg: baseline JPEG; 8-bit; one- or three-channel input
    .jp2: JPEG 2000; 8-bit or 16-bit; one- or three-channel input
    .tif or .tiff: TIFF; 8- or 16-bit; one-, three-, or four-channel input
    .png: PNG; 8- or 16-bit; one-, three-, or four-channel input
    .bmp: BMP; 8-bit; one-, three-, or four-channel input
    .ppm, .pgm: NetPBM; 8-bit; one-channel (PGM - portable gray map ) or three-channel (PPM - pixmap), (PBM - bitmap) .

 Sometimes, it is important to provide the matrix data as text data and not binary data. This is important for
 example to save calibration values of the camera. Ofcourse, the calibration values could be saved as bin files, but
 then it is not immediate comparable with different calibration files. Hence, OpenCV provides functions to save the
 data in either XML or YAML format. The YAML format comprises of mapping and sequences. Mapping is like a dictionary
 denoted by { and sequence is like a list with a series of numbers. List members are denoted by a leading hyphen (-)
 with one member per line or enclosed in square brackets ([ ]) and separated by comma space (, ). Apart from the
 conventional { and [, there are two more character sequences, {: and [:. They put the subequent lists in the same
 line. The {: and [: is simply ignored in XML formats, because the file layout is pretty constant, however not so
 comfortable to read. YAML wins when readability of the file needs to be maintained. The conversion to XML or YAML
 formats is called serialisation.
 cv::FileStorage::FileStorage( string fileName, int flag );
 cv::FileStorage::open( string fileName, int flag );
 cv::FileStorage::WRITE or cv::FileStorage::APPEND, cv::FileStorage::READ
 cv::FileStorage::release()
 << writes to the file. >> reads from a file

 The return value of this operator is not the desired object, however; it is an object of type cv::FileNode, which
 represents the value that goes with the given key in an abstract form.
 cv::Mat anArray;
 myFileStorage["calibrationMatrix"] >> anArray; // cv::FileNode >> cv::Mat
 cv::FileNodeIterator()
 cv::FileNode::name()
 cv::FileNode::begin() and end() returns a cv::FileNodeIterator that can be used to increment and decrement the
 operator.
 FileNode.begin() returns the FileNodeIterator  and FileNode.end returns the "after last" FileNodeIterator for either
 a mapping or a sequence. Once we have the FileNodeIterator, the FileNode can be obtained by dereferencing the
 FileNodeOperator.  The FileNode can be further dereferenced by either typecasting or redirection operator.
  - first -> this can have many mappings, listings etc and all of them are referenced using their names.
  - second
  - third


 cv::split() - splits into different channels
 cv::accumulate()
 cv::convertTo() - converts an image from one type to another. also see cv::cvtColor()
 cv::threshold() - src > threshold, then do this, otherwise do that.
    cv::THRESH_OTSU by comparing the varaince of pixels above the threshold and below the threshold. The algorithm
    hence sets an optimal threshold values, by searching for all possible thresholds in the space.
 cv::adaptiveThreshold()
 cv::copyMakeBorder() - makes borders based on specific algorithms
 cv::borderInterpolate() - returns the donor pixels that created the border


 cv::dct()
 cv::dft()
 cv::idct()
 cv::idft()
 cv::norm
 cv::normalize()
 cv::perspectiveTransform()
 gemm(), getElementScale...
 cv::mulSpectrum()
 cv::determinant() - single value decomposition.
 cv::GaussianBlur() - convolute the source image with a gaussian kernel with

 Humans see because of our eyes have sensors that is respondent to RGB. This doesnt mean that all creatures have the
 same potential. The bulls, lions, bees, snakes, respond differently to the same object because the sensors in their
 eyes stimulate to a different wavelength.
 RGB colors are additive and hence mixing with each other ( adding their components ) would give a different color.
 The addition is done on a completely black colour.
 CMY colors are subtractive colours. The subtraction is done on a completely white colour.
 In digital cameras, the light is focussed on an image plane. But the problem is that the lenses diffract differently
 for different wavelengths. And hence, some kind of correction needs to be done. The major wavelengths denoted by
 Fraunhofer D ( yellow ), Fraunhofer B ( blue ) and Fraunhofer C ( red ) lines are taken for correction and most of
 the time is enough for a good image. These wavelenths then must focus on a common point, otherwise, chromatic
 abberations will occur. A pair of convex lens and a concave lens makes it possible to focus the above lines onto a
 single plane.

 OpenCV was initially based on C and with OpenCV2.0 has been changed to C++. One of the main reasons are:
 1. Automatic memory allocation and deallocation ( constructors and destructors )
 2. Object oriented containers.
 3. The data pointer remains fixed for the images. Only headers are copied from one object to another.
 The cpmputation directly on the matrix irrespective of the functions saves some resources and computation time.
 There is no need to copy the matrix here and there, because the data is changed inline.
 There is only one single copy of the data, and only the headers are created. So,
     Mat A;
     Mat B(A);
     Mat C; C = A;
     Mat D = A;
  all have one data, but different headers. So the matrix operation on C will directly change the data of A ( but not
  the header ). The assigmnent operator and the copy constructor only copies the header.
  If you really want to copy the data also, then source.clone and source.copyTo(target) must be done.
     Mat F = A.clone();
     A = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Everytime a new matrix is created and is assigned or copy constructed ( XX reference ), then a reference counting
  mechanism counts the number of objects that the data is being referred to. When the last object is deleted, that is
  referring the real data ( image data ), then the data memory is also deallocated and correspondingly destructed (
  shredded ). Remember delete does not mean that the object is shredded. delete just takes away the life, but does
  not garbage dump the data yet.

  Color Coding:
  BGR is the OpenCV type. Other color codes are RGB, HSV, HSL, YCrCb, CIE L*a*b*
  The data used can be 8 bit signed or unsigned or 32 bit signed or unsigned float or even 64 bit double !!

  imread()
  The function determines the type of an image by the content, not by the file extension.
  In the case of color images, the decoded images will have the channels stored in B G R order.
  OPENCV_BUILD_3RDPARTY_LIBS flag in CMake would let you compile the relevant codecs.

  The image processing is just a game of numbers. It could be that the intensity has suddenly spiked up and this is
  known as a spike or a high frequency. If a statistics is taken to find out the difference in the pixel number from
  the neighbouring pixels, then any pixel which changes to a difference of for example 100 would be considered high
  frequency. These high frequencies are also introduced for example by scaling the images down. The
  downsampling is done by convolving the image with a series of delta functions. This convolution algorithm
  introduces high frequencies in the images, that needs to be filtered out later.

  Converting a RGB to Grayscale is as simple as taking the individual RGB values and averaging them. For example
  simply divide by 3, or there are other algorithms where weighted average is taken and green is given more weight:
  0.21 R + 0.72 G + 0.07 B. Also a max of RGB and min of RGB is added and is then divided by 2.

  cv::Vec3b -> typedef Vec<uchar, 3> Vec3b; -> 1 dimensional array with 3 char elements
  Each index is an array of 3 values i.e its a 2DArray and each element consists of 3 element 1D array.



  Basic Data Type ( cv::Matx<float,T,T>, cv::Vec<float,T>, cv::Size_<T>, cv::Point_<T>, cv::Point3_<T>,
  cv::RotatedRect<point, size, float>, cv::Rect<>

  Fixed Matrix classes:
      are classes whose dimensions are known at compile time. As a result, all memory for their data
      is allocated on the stack, which means that they allocate and clean up quickly. Operations on them are fast and
      there are even more specialized optimized implementations for small matrices.
      Representation: cv::Matx or correspnding aliases.
      Definition: cv::Matx<float,3,3>
      Access: m(3,1)
      Reshape: m.reshape<9,9>()
      Get Row / Column : m.col(1), m.row(1)
      Get diagnoal: m.diag()
      Transpose: m.t()
      Invert: m.inv(method) - default is cv::DECOMP_LU
      Per element mul: m.mul(m2)
      Normal multiplication: m1*m2, m1+m2, m1-m2
      solve linear system: m31f = m33f.solve(rhs31f, method)
      ::eye(), ::ones(), ::zeros(), ::all(x)
      Dotproduct: m1.dot(m2) - Dot product is essentially multiplying the corresponding elements and adding them all. It
      needs to be done on equal dimension matrix. Dot product -> a.b = |a||b|cos(þ),
      It is important to give the type of the object to .at when accessing a Mat object, because Mat can hold any
      kind of type at run time. It is the duty of the programmer to extract the type which he thinks that Mat would
      hold before accessing the elements. This can be made simply by explicitly decalaring Mat with the object type
      already at the declaration. This is done by Mat_. After this the programmer does not need to specify the type
      everytime a member function of Mat_ is accessed, simply because the type is already known to the compiler.

      Fixed Vector classes:
      are derived from the fixed matrix classes and other classes such as cv::Scalar derive from fixed vector classes.
      They are really just convinience functions for Matx. Fixed vector classes is a Fixed matrix class whose number of
      columns is one. Since it is inherited, the fixed vector class inherits everything from fixed matrix class.
      members access: v[i] # Please note the difference in parantheses from member access for single column fixed matrix
      class. m(i).
      Definition: cv::Vec<float,3>
      Access: v(3)
      Cross product: v3f.cross(u3f) - Cross product is always on between two 1 dimensional matrix. Hence this does not
      appear in cv::Matx. It needs to be done on equal dimension matrix. Cross product -> axb = |a||b|sin(þ)


  Helper Object ( cv::Ptr, cv::TermCriteria, cv::Range )

  Helper objects are helpful for controlling various algorithms such as termination criteria or for doing operations
  on the containers such as ranges or slices. There is also one very important object called a smart pointer object
  cv::Ptr.
  Termination criteria take the form of either some finite number of iterations or some kind of error parameters,
  that basically say if one is too close to this number ( epsilon ), then you can quit. The cv::TermCritera can be
  passed to an OpenCV algorithm. The member variables, type, maxCount and epsilon are public. If the termination
  criteria incudes Cv::TermCriteria::COUNT, then you are telling the algorithm to terminate after maxCount iterations.
  If the terminate criterion includes cv::TermCrtieria::EPS, then you are telling the algorithm to terminate after
  some metric associated with the algorithms convergence falls below epsilon.
  cv::Range class is used to specify a continous sequence of integers.
  cv::Ptr is a smart pointer, that allows us to create a reference to something and then pass it around. Everytime a
  reference is created, a reference count is incremented and subsequently decremented when the reference goes out of
  scope. When the reference ( instances of pointer ) count goes to 0, then the Ptr is destroyed.
  The cv::Ptr takes an object that needs to be wrapped. For example:
  cv::Ptr<cv::Matx<float,3,3> > p = makePtr<cv::Matx<float,3,3> >(). The constructor for the template object takes a
  pointer ot the object to be pointed to. This instantiates a smart pointer p, which is a sort of pointer like
  object, that you can pass around and use like a normal pointer ( uses . and -> ).
  addref() and release() increment and decrement the internal reference counter of the pointer. however should not be
  used, cause its the job of the class itself to inrement and dec the reference count.
  empty() determines, if the object still exists where the pointer is pointing to.

  cv::Exception has members
  code : numerical error code
  err : a string indicating the nature of error code that generated the exception
  func : name of the function where the occur occured
  file : the file in which the error occured
  line : the line on which the error occured.
  CV_Error(code, description)
  CV_Error_(code, printf_fmt_str, args)
  CV_Assert(condition) - throws exception if the condition is not met.

  cv::InputArray, cv::OutputArray:
  Any of the cv::Scalar, cv::Vec, cv::Matx, std::vector<>, cv::Mat and cv::SparseMat is defined by cv::InputArray and
  cv::OutputArray. The primary difference between the two is InputArray is const OutputArray is not.
  The InputArray and OutputArray is primarily to keep the interfaces from becoming complicated and reperetive. OpenCV
  defines the types InputArray and OutputArray that takes any of the array types as Input and Output. There is
  another class called InputOutputArray for in-place computation.

  Utility and system functions:
  cv::getTickCount(), cv::getThreadNum() and many more.......... Page 60


  Big Containers / Large Array type:

  Usage Mat:
  - Use the create(nrows, ncols, type) method or the similar Mat(nrows, ncols, type[, fillValue]) constructor.
  - CV_8UC1 means a 8-bit single-channel array, CV_32FC2 means a 2-channel (complex) floating-point array, and so on.
  - Mat M(7,7,CV_32FC2,Scalar(1,3));    // make a 7x7 complex matrix filled with 1+3j.
  - int sz[] = {100, 100, 100};
    Mat bigCube(3, sz, CV_8U, Scalar::all(0)); // create a 100x100x100 8-bit array
 - Use a copy constructor or assignment operator where there can be an array or expression on the right side (see below)
   As noted in the introduction, the array assignment is an O(1) operation because it only copies the header and
   increases the reference counter. The Mat::clone() method can be used to get a full (deep) copy of the array when
   you need it.
     // add the 5-th row, multiplied by 3 to the 3rd row
    M.row(3) = M.row(3) + M.row(5)*3;
    // now copy the 7-th column to the 1-st column
    // M.col(1) = M.col(7); // this will not work
    Mat M1 = M.col(1);
    M.col(7).copyTo(M1);
    // create a new 320x240 image
    Mat img(Size(320,240),CV_8UC3);
    // select a ROI
    Mat roi(img, Rect(10,10,100,100));
    // fill the ROI with (0,255,0) (which is green in RGB space);
    // the original 320x240 image will be modified
    roi = Scalar(0,255,0);
    Due to the additional datastart and dataend members, it is possible to compute a relative sub-array position in the
    main *container* array using locateROI():
    Mat A = Mat::eye(10, 10, CV_32S);
    // extracts A columns, 1 (inclusive) to 3 (exclusive).
    Mat B = A(Range::all(), Range(1, 3));
    // extracts B rows, 5 (inclusive) to 9 (exclusive).
    // that is, C \~ A(Range(5, 9), Range(1, 3))
    Mat C = B(Range(5, 9), Range::all());
    Size size; Point ofs;
    C.locateROI(size, ofs);
    As in case of whole matrices, if you need a deep copy, use the `clone()` method of the extracted sub-matrices.
    - Make a header for user-allocated data. It can be useful to do the following: -# Process "foreign" data using
    OpenCV (for example, when you implement a DirectShow\* filter or a processing module for gstreamer, and so on)
    . For example:
        void process_video_frame(const unsigned char* pixels,
                                 int width, int height, int step)
        {
            Mat img(height, width, CV_8UC3, pixels, step);
            GaussianBlur(img, img, Size(7,7), 1.5, 1.5);
        }
   -  Quickly initialize small matrices and/or get a super-fast element access.
        double m[3][3] = {{a, b, c}, {d, e, f}, {g, h, i}};
        Mat M = Mat(3, 3, CV_64F, m).inv(); Partial yet very common cases of this *user-allocated data* case are
        conversions from CvMat and IplImage to Mat. For this purpose, there is function cv::cvarrToMat taking pointers
        to CvMat or IplImage and the optional flag indicating whether to copy the data or not. size will be
        (width=10,height=10) and the ofs will be (x=1, y=5)

 Unlike Vec and Matx that are used only for max 3 dimensions, Mat can be used for any number of dimensions. The data
 is stored in an array, what can be thought of as an n-dimensional analog of raster scan order. In 1 D, the data is
 sequential, in 2 D, data is stored in rows, and in 3D, the data is stored in planes, where each plane is filled row
 by row. Unlike many other implementations, where the elements in the matrices are always primitive data, OpenCV Mat
 elements need not be single numbers, but can also be multiple numbers. In fact an n-dim array and an n-1 dimensional
 multichannel array are very similiar objects. So 1 3D array with single channel and 2D array with a multi channel are
 very  similiar.  Its similiar because the memory structure of both of them differ for example, the way padding
 occurs. The multi channel arrays are not completely sequential, as there is padding either after one full row or
 after one full channel. Multi channel arrays can be created by the "type" CV_32FC(7) - this creates a 7 channel for
 each element.
 An array simply by instantiating has no size and no datatype. But can be asked to be allocated with create().
 cv::Mat is a header for data. One can have many headers pointing to the same data.

 There are innumerous constructors:
 Constructors for 2D matrixes:
    row, col, type
    row, col, type, Scalar
    row, col, type, *data, step
    Size, type
    Size, type, Scalar
 The step is one of the way to initialise a matrix. Scalar gives constant values, but with step a pattern can be
 created. So, data can be initialised either by providing a Scalar ( in which the entire arry will be initialised to
 that value ) or by providing a pointer to an appropriate data block.
 Another way is to copy construct from existing arrays or copy construct regions from existing arrays.
 source, range_rows, range_cols
 source, rect
 source, *range

 The basic means to direct access is the member function at<>(). The way the function works, is that you specialize
 the at<>() template to the ata that the matrix contains, then access the element using the rows and column locations.
 Worth noting is the cv::DataType<>. This is the arguement type. The type in the cv::Mat constructor can also take
 cv::DataType<cv::Complexf>::type for example.
 Another way is the C Style ptr<>() member function. Unlike at, ptr returns a address. Once the address is there, you
 can manipulate with pointer operations. For example, given a three channel matrix mtx of type float, the
 construction mtx.ptr<Vec<float,3>>(3) would return the pointer to the first floating point channel of the first
 element in row 3 of the matrix.
 The third way is to use the member variable data and to use the member array step[] to compute addresses.
 Sometimes, the rows are not continous because of padding. Hence the member function isContinous() returns if its OK
 to iterate the whole array with a pointer.
 The last method is to use cv::MatIterator<>. The cv::Mat method begin() and end() return objects of this type. This
 method is smart, because it knows about the continuity and sequential of the array. Example:
 A 3D array of size 4*4
 cv::Mat m(3,sz,CV_32FC3)
 cv::randu(m,1,2);
 cv::MatIterator<cv::Vec<float,3>> it = m.begin()
 and then using m.begin() and m.end() the whole array can be iterated.

 The main idea behing cv::Mat is to separate the header and the workspace. Headers are created instantly, but the
 workspace remains fixed unless a copyTo is performed.

 Filling:
 cv::randu(m,1,2)
 cv::RNG::fill(m,cv::RNG::UNIFORM, 1,2)

 cv::saturate_case<>()

 Statistics:
 eigen values
 eigen vectors
 guassian distribution
 standard deviation : the more spread out the distribution is the greater the standard distribution. A standard
 deviation close to 0 means, that the data points are close to the mean. A higher standard deviation of the same
 mean, would mean that the data points are more spread out.
 Step 1: Add all the numbers and divide by the amount of numbers.
 Step 2: Find the difference between the number and the answer of step 1. Square it.
 Step 3: Add the squared numbers.
 Step 4: Divide by the amount of numbers. Beware that it is sample points - 1.
 Step 5: Square root the answer of step 4. Step 4 is called the Variance of the dataset.

 Correlation: If the correlation between the two numbers is close to 0, that means there is no corrleation at all. If
 it is tending to 1, then when one increases, the other increases too. If it is tending to -1, then when one
 increases, the other decreases. The result of Step 2 and Step 4 is called a z score.
 Step 1: Find the difference between the number and the mean of the first set
 Step 2: Divide the answer in Step 1 by the standard deviation of this set.
 Step 3: Find the difference between the number and the mean of the second set
 Step 4: Divide the answer in Step 3 by the standard deviation of this set.
 Step 5: Multiply the results of Step 2 and Step 4
 Step 6: Repeat this for each sample set and then add all of them.
 Step 7: Divide by the amount of numbers. Beware that it is sample points - 1.

 A linear model  has a correlation of 1 between the x and y points.

That's how you find the standard deviation.
 covariance
 inverse covariance
 determinant
 transpose




  Usage ROI:
    cv::Mat roiRange = img_rgb(cv::Range(100, 300), cv::Range(0, 512)); // start row, end row, start column, end column.
    cv::Mat roiRect = img_rgb(cv::Rect(0,100,512,200)); // start column, start row, width, height

  Point, Vec, Mat, Matx, Scalar, Size, Rect, RotatedRect

  Fixed matrix classes such as Point, Size, Vec, Matx have dimensions that is known at compile time. As a result, all
  memory for their data is allocated on the stack, which means that they allocate and clean up quickly. Operators on
  them are fast, and there are specially optimized implementations for small matrices.


  Scalar:
  Scalar is just for ease of use. As most of OpenCV operates on maximum 4 channel images, so Scalar is a simple class
  which is actually a cv::Vec of length 4, which can be used by OpenCV
  algorithms according to number of channels of the image. Instead of creating an array of different
  length each time, you just pass a scalar value to the algorithm.

  Rectangle and Range:
  If you want to pass both the row and columns, then way is to create a new matrix and pass the source image as the
  first parameter and the section ( range or rectangle ) as the second parameter. This is a kind of a value constructor.


  Summary ( please see Vec and Matx is without underscore )
  Static : Hence the templates consists of dimensions, where the dimensions can vary.
  Point_<T>, Point3_<T>
  Rect_<T>
  Scalar_<T>
  Vec_<T,int>      : dimensions can vary
  Matx<T,int,int>  : dimensions can vary

  Dynamic and hence the template does not consists of dimensions.
  Mat
  Mat_<T>


 Summary
 all(val),
 zero,
 ones,
 eye,
 diag,
 randu(min, max),
 nrandn(mean, variance) : create a matrix with normally distributed entities.

 Access:
 Dynamic
 Mat.at<>()                         : element access ( Mat type defined without template, dynamic dimension )
 Mat.at<cv::Vec>()[]                : channel access ( Mat type defined without template, dynamic dimension)
 Static
 Matx()                             : element access ( Matx type defined with template, static dimension )
 Mat_.at<>() or Mat_()              : element access ( Mat_ type defined with template, static dimension )
 Mat_.at()<cv::Vec>[]  or Mat_()[]  : channel access ( Mat type defined with template, static dimension )

 The purpose of using the template forms Mat_is that you dont have to use the template forms of their member functions.

 Access Subregion: Arrays corresponding to subregions. There is no copy, only headers are defined.
 Data of Mat is not copied to the new arrays unless copyTo() method is used.
 Mat.row()
 Mat.col()
 Mat.rowRange()
 Mat.colRange()
 Mat.rowRange(cv::Range)
 Mat.colRange(cv::Range)
 Mat.diag()
 Mat ( cv::Range, cv::Range )
 Mat ( cv::Rect )


 Ability to create algebraic expressions consisting of matrix arrays and singletons.  The result of the compuation is
 finally placed in the destination array by operator=(). Arrays are semantically referred to be as cv::Mat and matrix
 arrays means something which has a mathematical meaning ie on which matrix operations can be done. A cv::MatExpr is
 assigned to a cv::Mat. m1+m2 is not cv::Mat but a cv::MatExpr. The pointer to the result of MatExpr will be assigned
 to the destination array. The data of the array itself got created temporarily by the MatExpr, but isnt destroyed
 because the scope of the array still exists as the reference is incremented. Apart from simple algebra, there are
 many other operations, one such operation is cv::Mat::eye() or m1*m2 etc etc.. All of these are of the type MatExpr.
 Higher level operations such as transpose and inversions.
 inv(), t(), cross(), dot(), cv::abs(m), cv::norm(m), cv::mean(m), cv::sum(m)

 Underflowing and Overflowing is taken care from cv::saturation_cast<>()
 m0.clone() and m0.copyTo(m1) are the same.

 There are some general rules in OpenCV functions. Ofcourse there are special rules pertaining to individual
 functions, and they are reported by the exceptions. There are some general rules and they apply to all the functions:
 Saturation, Output array, Scalars, Masks, dtype, Inplace operations, Multichannel.

 1. Saturation: Outputs of calculations are saturation-casted to the type of the output array. cv::saturation_case<>()
 uchar& Vxy = m0.at<uchar>(y,x)
 Vxy = cv::saturate_cast<uchar>((Vxy-128)*2 + 128);}
 This explicit saturation_cast truncates the negative value into 0. Otherwise, the answer would have been the 2s
 complement.
 2. Output Array: The output array will be created with cv::Mat::create() if its type and size do not match the
 required type and size. Usually, the required output type and site are the same as inputs, but for some functions,
 xize may be different like for cv::transpose and cv::split
 3. Scalars: Addition of two arrays or an array and a scalar. The result of providing a scalar is the same as if a
 second array had been provided with the same scalar value in every element. Size as described above is taken from
 the input array.
 4: Mask: Whenever the mask argument is present for a function, the output wil be computed only for those elements
 where the value corresponding to the element in the output array is non zero. If the mask arguement is not
 present, then all the values would be calculated, even if its 0*0.
 5. dtype: This forces the output array to be of a specific type. For example, two CV_8UC1 input array will lead to
 the same output type. However, this can be overridden with the dtype=CV_32FC1.
 6. Inplace operation: One can specify the same arrays for both input and output. Its completely legal to do it, but
 ofcourse, it should be clear that the input array will completely loose its data.
 7. Multichannel: Each channel is processed separately, if multichannel arguements are set.



 In electronics and signal processing, a Gaussian filter is a filter whose impulse response is a Gaussian function (
 or an ( or an approximation to it ) . Gaussian filters have the properties of having no overshoot to a step function
 while minimizing the rise and fall time. This behaviour is closely connected to the fact that the Gaussian filter ha
 the minimum possible group delay. It is considered the ideal time domain filter, just as the sinc is the ideal
 frequency domain filter.  Mathematically, a Gaussian filter modfies the input signal by convolution with a Gaussian
 function. What are the different kinds of filters:

 1. Butterworth filter
 2. Chebyshev filter
 3. Elliptic filter
 4. Bessel filter
 5. Gaussian filter
 6. Optimum L ( Legendre ) filter
 7. Linkwitz-Riley filter

 Gaussian Blur:
 Mathematically applying Gaussian blur to an image is the same as convolving the image wih a Guassian function.
 A guassian function is a function of the form -
 Bokeh - a japanese word for blur.

 2i + 3j + 4k ( Unit Vectors )
 We can represent any vector by a linear combination of two vectors. This means the vectors can be spanned anywhere
 in that plane. If the vector has to be represented in another plane, then the linear combination of the two vectors
 wont work, because the span has changed. If two vectors cannot be represented  by scaling one of the vectors, then
 they are said to be linearly independent. An example for linearly independent vectors are the axes. The x axis can
 never be represented by scaling up or down the y axis and vice versa. Linear dependant vectors hence are vectors
 that cannot be represented by a combination of other vectors.
 Dot products are commutative and associative.

 For an arbitrary linear equation, there can be infinite number of solutions, for example, 2x + 4y + 7z + 9w = 1000.
 Hence x,y,z,w can consists of infinite number of solutions.

 The inverse of the matrix A can be calculated by many methods in linear algebra such as Gaussian elimination,
 Eigendecomposition, Cholesky decomposition and Carmer’s rule. A matrix can also be inverted by block inversion
 method and Neuman series. Every matrix can be transposed ( rearranging the columns and rows in a matrix ). The diagonal
 of the matrix remains unchanged. The matrix is symmetric, if the transpose of the matrix is equal to the matrix. Its
 skew symmetric, if the transpose of the matrix is the negative of the original matrix.Ofcourse, for the matrix to be
 symmetric or skew symmetric,  they need to be a square matrix. A(i,j) becomes A(j,i). Complex Conjugate matrix is
 the matrix where the complex number is negated. The real part remains the same.

 Pivot variables, Free variables, Echelon form.
 Imagining 4 vectors defined by 4 equations. The solution of the equation is the point where all the vectors meet
 each other and this point is given by (x1, x2, x3 and x4 )
 A Singular matrix is a matrix whose determinant is 0.

 C++ Libraries to solve linear algebra
 Flint, Singlular, Givaro, Giv ( Scientific Imaging ), Eigen

 Collections
 XML, YAML based functions is more universal in dealing with image coding and decoding.
 The produced YAML (and XML) consists of heterogeneous collections that can be nested. There are 2 types of collections: named collections (mappings) and unnamedcollections (sequences). In mappings each element has a name and is accessed by name. This is similar to structures and std::map in C/C++ and dictionaries in Python. In sequences elements do not have names, they are accessed by indices. This is similar to arrays and std::vector in C/C++ and lists, tuples in Python. “Heterogeneous” means that elements of each single collection can have different types.

 Commutative, Associative, Distributive.
 Cross Correlation and Convolution are the same except the kernel is time reversed. Hence cross correlation can be
 implemented efficiently using fast convolution algorithms like overlap-save.
 Overlap-save is an algorithm to evaluate the discrete convolution between a very long signal x [ n ] {\displaystyle
 x[n]}  x[n] and a finite impulse response (FIR) filter h [ n ] {\displaystyle h[n]} h[n]:
 Impulse response is a resonse to an impulse. The impulse response are FIR and IIR. FIR.
 The impulse response defines the response of a linear time-invariant system for all frequencies.
 Linearity means the scaling and adding of individual vectors have the same effect on the output vectors. Time
 invariant, means that a computation at time t will have the same result at time t+T. The only difference will be the
 time delay at the output. Or in other words, time doesnt play a role in the final value of the the computation.
 In other words, convolution in the time domain is equivalent to multiplication in the frequency domain.
 Any LTI system can be characterized entirely by a single function called the systems impulse response. The output
 of the system is simply a convolution of the input to the system with the systems impulse response. So, if we know
 the impulse response of the system through tests, then we can calculate the output of the system for any input
 signal. In the frequency domain, the laplace transform of the impulse response is called the transfer function.
 Impulse is denoted by the term ð ( delta ).

 The Laplace transform of the delta function is 1, so the impulse response is equivalent to the inverse Laplace
 transform of the system's transfer function. In control theory the impulse response is the response of a system to
 a Dirac delta input.

 An FIR filter is designed by finding the coefficients and filter order that meet certain specifications, which can be in the time-domain (e.g. a matched filter) and/or the frequency domain (most common). Matched filters perform a cross-correlation between the input signal and a known pulse-shape. The FIR convolution is a cross-correlation between the input signal and a time-reversed copy of the impulse-response. Therefore, the matched-filter's impulse response is "designed" by sampling the known pulse-shape and using those samples in reverse order as the coefficients of the filter.

 The only differnce between an  IIR and FIR filter is that the IIR filter never settles down to 0. This is because
 the cpaactiros retain a charge indefinetly for an impulse. On the discrete side, which is analogous to FIR filter,
 the memory comes back to its initial state after time t>T.

 For a FIR filter of order N, each value of the output sequence is a weighted sum of the most recent input values.
 The final value is when n has reached N, where beyond N, the value of the output sequence is equal to 0. The above
 compuation is also called discrete convolutoin.

 The convolution of a two dimensional image with a gaussian function can either be done in the spatial domain ( x, y
 ) or the frequency domain. If the convolution is done in the frequency domain, then the concolution is simply an
 inverser fourier transform of the multiplication  ( pixel by pixel ) between the fourier transform of the input
 image and the fourier transform of the gaussian funciton.
 If the spatial domain is used, then the convolution is the double integral of the image multiplied by the gaussian
 function from x and y tending from -infinity to +infinity.

 Convolution as associative, while correlation is not. The associative property is what allows you to convolve multiple filters into a single one, and then convolve that one filter with the image. That is the big advantage of convolution over correlation.

 A crystal can be said as the convolution of the unit cell with the lattice of the crystal. Here the lattice can be
 for example an image, and the unit cell can be thought of as a gaussiann kernel. This way, the gaussian kernel is
 going to be repeated in the entire lattice, forming a perfect crystal.

 Cross coorelation is used to find out the similarity between two different functions. In the convolution theorem,
 the kernel is 180 deg out of phase ( its flipped ) , whereas in cross correlation, there isnt any flip. The cross
 correlation uses the convolution theorem that the fourier of the output is a element by element multiplication of
 the fouriers of the two inputs.
 What is convolution?
 What is convoution theorem?
 What is a PSF?
 What does convolution has to do with the structure of the crystal?
 How might cross correlation be used in crypto FM?

 Auto correlation is between the same signals.


 Algorithms:

 Harris Method - computes derivative operators, analyses them and returns a list of the points that meet our definition.
 Shi and Tomasi method

 Inverse Matrix:
 Sometimes, the calculation needs to be aborted, if the matrix is singular. For example an inverse would make no
 sense for singular matrix. But the calculation is costly and hence, there are methods where a matrix can be
 decomposed into a multiplication of many matrices. One such is LU_DECOMPOSITION. In this algorithm, two matrix is
 decomposed into a lower triangle matrix and a upper triangle matrix. The matrix multiplication of both of them gives
 the original matrix. Its very easy to say if the original matrix is singular, by just multiplying the first element of
 the L with the first element of the upper. If its 0, then the matrix is singular, otherwise not.

 Matrix decomposition methods:
 DECOMP_LU Gaussian elimination with optimal pivot element chosen.
 DECOMP_CHOLESKY Cholesky LL^T factorization; the matrix src1 must be symmetrical and positively defined.
 DECOMP_EIG eigenvalue decomposition; the matrix src1 must be symmetrical.
 DECOMP_SVD singular value decomposition (SVD) method; the system can be over-defined and/or the matrix src1 can be
 singular.
 DECOMP_QR QR factorization; the system can be over-defined and/or the matrix src1 can be singular.
 DECOMP_NORMAL while all the previous flags are mutually exclusive, this flag can be used together with any of the
 previous; it means that the normal equations

 Robust statistics algorithms
 Outlier detection tools

 Matrix:
 Inverse / Reciprocal = fact{Transpose(Adjoint)}{Determinant}
 Orthogonal - Q.t()*Q = I. The inverse of an orthogonal matrix is its transpose.
 Hessenberg - below subdiagonal or above super diagnoal is 0. Important to find eigenvalues.
 Vandermonde Matrix - 1, roots, roots2. The determinant is easily found by products of x_j - x_i

 Eigen Values / Eigen Vectors:
 Companion Matrix -> Characteristicts equation or characteristicts polynomial -> Find roots which is also called
 eigen value -> Find eigen vectors.
 The companion matrix is called companion because its in a sense the companion of polyonomial p.
 In classical linear algebra, the eigenvalues of a matrix are sometimes defined as the roots of the characteristicts
 polynomial. An algorithm to compute the roots of the polynomial by computing the eigenvalues of the corresponding
 companion matrix turns the table on the usual definitions. The eigen value of the companion matrix coincide with the
 roots ( zeros ) of the associated polynomial because, p(z) = det(zI - C_p). M needs to be converted into the
 Hessenberg form.
 QR Algorithm for computing matrix eigen values. QR ( decompose original into orthogonal matrix and the upper
 triangle matrix ).
 Some methods to find the roots of the polynomial are
 Jenkins Traub - rpoly and cpoly
 Companion Matrix

 Linear Least square:
 The linear least square method is most widely solved by QR Decomposition method.


 cv::goodFeaturesToTrack():
    tempted to look for points that have some significant change in them - for example, a strong derivative.





 */







/** brief
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



  Basic Data Type ( cv::Matx<float,T,T>, cv::Vec<float,T>, cv::Size_<T>, cv::Point_<T>, cv::Point3_<T>,  cv::RotatedRect<point, size, float>, cv::Rect<>
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
  Scalar is just for ease of use. As most of OpenCV operates on maximum 4 channel images, so Scalar is a simple class which is actually a cv::Vec of length 4, which can be used by OpenCV
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


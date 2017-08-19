

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
  needs to be done on equal dimension matrix. Dot product -> a.b = |a||b|cos(þ)

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




  Point, Vec, Mat, Matx, Scalar, Size, Rect, RotatedRect



 */
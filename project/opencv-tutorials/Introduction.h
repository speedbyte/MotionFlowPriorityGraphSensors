

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

 */
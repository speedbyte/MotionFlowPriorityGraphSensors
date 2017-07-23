//
// Created by veikas on 23.07.17.
//

#ifndef OPENCV_TUTORIAL_CVMAT_H
#define OPENCV_TUTORIAL_CVMAT_H

/** brief

 Single channel.
    Mat M(100,100,CV_8U);  // create a 100*100 8 bit matrix.
    Mat_<char> M(100,100);
    M.row, M.col, M(i,j)

 Multi Channel.
    Mat_<Vec3b> img(240, 320, Vec3b(0,255,0)); // allocate a 320*240 color image and fill it with green. Not a row, column: but a column, row.
    img(i,i) = Vec3b(255,255,255); // Draw a diagonal white line.

 */

class CvMat {

};


#endif //OPENCV_TUTORIAL_CVMAT_H

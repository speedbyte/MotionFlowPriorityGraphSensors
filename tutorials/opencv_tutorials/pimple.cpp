
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;

Mat src,cloneimg;
bool mousedown;
vector<vector<Point> > contours;
vector<Point> pts;

bool findPimples(Mat img)
{
    Mat bw,bgr[3];
    split( img,bgr );
    bw = bgr[1];
    int pimplescount = 0;

    adaptiveThreshold(bw,bw,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,15,5);
    dilate(bw,bw, Mat(), Point(-1,-1),1);

    contours.clear();
    findContours( bw, contours, RETR_LIST, CHAIN_APPROX_SIMPLE );

    for( size_t i = 0; i< contours.size(); i++ )
    {

        if( contourArea(contours[i]) > 20 & contourArea(contours[i]) < 150 )
        {
            Rect minRect = boundingRect( Mat(contours[i]) );
            Mat imgroi(img,minRect);

            cvtColor(imgroi,imgroi,COLOR_BGR2HSV);
            Scalar color =mean(imgroi);
            cvtColor(imgroi,imgroi,COLOR_HSV2BGR);

            if(color[0] < 10 & color[1] > 70 & color[2] > 50)
            {
                Point2f center, vtx[4];
                float radius = 0;
                minEnclosingCircle(Mat(contours[i]), center, radius);

                if(radius < 20)
                {
                    rectangle(img,minRect,Scalar(0,255,0));
                    pimplescount++;
                }
            }
        }
    }
    putText(img, format("%d",pimplescount), Point(50, 30),FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255,255,0), 2);

    imshow( "pimples dedector", img );
}


void onMouse( int event, int x, int y, int flags, void* userdata )
{
    Mat img = *((Mat *)userdata);

    if( event == EVENT_LBUTTONDOWN )
    {
        mousedown = true;
        contours.clear();
        pts.clear();
    }

    if( event == EVENT_LBUTTONUP )
    {
        mousedown = false;
        if(pts.size() > 2 )
        {
            Mat mask(img.size(),CV_8UC1);
            mask = 0;
            contours.push_back(pts);
            drawContours(mask,contours,0,Scalar(255),-1);
            Mat masked(img.size(),CV_8UC3);
            masked = Scalar(255,255,255);
            img.copyTo(masked,mask);
            cloneimg = src.clone();
            findPimples(masked);
        }
    }

    if(mousedown)
    {
        if(pts.size() > 2 )
            line(img,Point(x,y),pts[pts.size()-1],Scalar(0,255,0));

        pts.push_back(Point(x,y));

        imshow( "pimples dedector", img );
    }
}


int main( int argc, const char** argv )
{
    src = imread(argv[1]);
    if(src.empty())
    {
        return -1;
    }


    src = cv::imread("../../../pics_dataset/lena.png", CV_LOAD_IMAGE_COLOR);
    namedWindow("pimples dedector", WINDOW_AUTOSIZE);
    cloneimg = src.clone();
    setMouseCallback( "pimples dedector", onMouse, &cloneimg );
    imshow( "pimples dedector", src );

    waitKey(0);
    return 0;
}
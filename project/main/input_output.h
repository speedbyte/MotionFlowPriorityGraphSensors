//
// Created by veikas on 08.09.17.
//

#ifndef MAIN_INPUT_OUTPUT_H
#define MAIN_INPUT_OUTPUT_H


#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv/cv.hpp>

void processFrame(cv::Mat& img, cv::Mat& out);


void canny(cv::Mat& img, cv::Mat& out) {
    // Convert to gray
    if (img.channels()==3)
        cv::cvtColor(img,out,CV_BGR2GRAY);
    // Compute Canny edges
    cv::Canny(out,out,100,200);
    // Invert the image
    cv::threshold(out,out,128,255,cv::THRESH_BINARY_INV);
}

class VideoProcessor {

private:
    // the OpenCV video writer object
    cv::VideoWriter writer;
    // output filename
    std::string outputFile;
    // current index for output images
    int currentIndex;
    // number of digits in output image filename
    int digits;
    // extension of output images
    std::string extension;

    // the OpenCV video capture object
    cv::VideoCapture capture;

    // the callback function to be called
    // for the processing of each frame
    void (*process)(cv::Mat &, cv::Mat &);

    // a bool to determine if the
    // process callback will be called
    bool callIt;
    // Input display window name
    std::string windowNameInput;
    // Output display window name
    std::string windowNameOutput;
    // delay between each frame processing
    int delay;
    // number of processed frames
    long fnumber;
    // stop at this frame number
    long frameToStop;
    // to stop the processing
    bool stop;
    // vector of image filename to be used as input
    std::vector <std::string> images;
    // image vector iterator
    std::vector<std::string>::const_iterator itImg;

    FrameProcessor *frameProcessor;

public:
    // get the codec of input video
    int getCodec(char codec[4]) {

        // undefined for vector of images
        if (images.size()!=0) return -1;

        union { // data structure for the 4-char code
            int value;
            char code[4]; } returned;

        // get the code
        returned.value= static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));

        // get the 4 characters
        codec[0]= returned.code[0];
        codec[1]= returned.code[1];
        codec[2]= returned.code[2];
        codec[3]= returned.code[3];

        // return the int value corresponding to the code
        return returned.value;
    }
    // set the output as a series of image files
    // extension must be ".jpg", ".bmp" ...
    bool setOutput(const std::string &filename, // prefix
                   const std::string &ext, // image file extension
                   int numberOfDigits=3,   // number of digits
                   int startIndex=0) {     // start index

        // number of digits must be positive
        if (numberOfDigits<0)
            return false;

        // filenames and their common extension
        outputFile= filename;
        extension= ext;

        // number of digits in the file numbering scheme
        digits= numberOfDigits;
        // start numbering at this index
        currentIndex= startIndex;

        return true;
    }
    // to write the output frame
    // could be: video file or images
    void writeNextFrame(cv::Mat& frame) {
        if (extension.length()) { // then we write images

            std::stringstream ss;
            // compose the output filename
            ss << outputFile << std::setfill('0') << std::setw(digits) << currentIndex++ << extension;
            cv::imwrite(ss.str(),frame);

        } else { // then write to video file 
            writer.write(frame);
        }
    }
    // set the output video file
    // by default the same parameters than
    // input video will be used
    bool setOutput(const std::string &filename, int codec=0, double framerate=0.0, bool isColor=true) {

        outputFile= filename;
        extension.clear();

        if (framerate==0.0)
            framerate= getFrameRate(); // same as input

        char c[4];
        // use same codec as input
        if (codec==0) {
            codec= getCodec(c);
        }

        // Open output video
        return writer.open(outputFile, // filename
                           codec,          // codec to be used
                           framerate,      // frame rate of the video
                           getFrameSize(), // frame size
                           isColor);       // color video?
    }
    // set the callback function that
    // will be called for each frame
    void setFrameProcessor(
            void (*frameProcessingCallback)(cv::Mat &, cv::Mat &)) {
        process = frameProcessingCallback;
    }

    // set the instance of the class that
    // implements the FrameProcessor interface
    void setFrameProcessor(FrameProcessor* frameProcessorPtr)
    {

        // invalidate callback function
        process= 0;
        // this is the frame processor instance
        // that will be called
        frameProcessor= frameProcessorPtr;
        callProcess();
    }

    // set the name of the video file
    bool setInput(std::string filename) {

        fnumber = 0;
        // In case a resource was already
        // associated with the VideoCapture instance
        capture.release();
        // Open the video file
        return capture.open(filename);
    }

    // set the vector of input images
    bool setInput(const std::vector <std::string> &imgs) {

        fnumber = 0;
        // In case a resource was already
        // associated with the VideoCapture instance
        capture.release();

        // the input will be this vector of images
        images = imgs;
        itImg = images.begin();

        return true;
    }

    // to display the input frames
    void displayInput(std::string wn) {

        windowNameInput = wn;
        cv::namedWindow(windowNameInput);
    }

    // to display the processed frames
    void displayOutput(std::string wn) {
        windowNameOutput = wn;
        cv::namedWindow(windowNameOutput);
    }

    // to grab (and process) the frames of the sequence
    void run() {
        // current frame
        cv::Mat frame;
        // output frame
        cv::Mat output;

        // if no capture device has been set
        if (!isOpened())
            return;

        stop= false;

        while (!isStopped()) {

            // read next frame if any
            if (!readNextFrame(frame))
                break;

            // display input frame
            if (windowNameInput.length()!=0)
                cv::imshow(windowNameInput,frame);

            // calling the process function or method
            if (callIt) {

                // process the frame
                if (process)
                    process(frame, output);
                else if (frameProcessor)
                    frameProcessor->process(frame,output);
                // increment frame number
                fnumber++;

            } else {

                output= frame;
            }

            // ** write output sequence **
            if (outputFile.length()!=0)
                writeNextFrame(output);

            // display output frame
            if (windowNameOutput.length()!=0)
                cv::imshow(windowNameOutput,output);

            // introduce a delay
            if (delay>=0 && cv::waitKey(delay)>=0)
                stopIt();

            // check if we should stop
            if (frameToStop>=0 && getFrameNumber()==frameToStop)
                stopIt();
        }
    }

    // Stop the processing
    void stopIt() {

        stop = true;
    }

    // Is the process stopped?
    bool isStopped() {

        return stop;
    }

    // Is a capture device opened?
    bool isOpened() {

        return capture.isOpened() || !images.empty();
    }

    // set a delay between each frame
    // 0 means wait at each frame
    // negative means no delay
    void setDelay(int d) {

        delay = d;
    }

    // to get the next frame
    // could be: video file; camera; vector of images
    bool readNextFrame(cv::Mat &frame) {

        if (images.size() == 0)
            return capture.read(frame);

        else {

            if (itImg != images.end()) {

                frame = cv::imread(*itImg);
                itImg++;
                return frame.data != 0;

            } else

                return false;
        }
    }

    // process callback to be called
    void callProcess() {

        callIt = true;
    }

    // do not call process callback
    void dontCallProcess() {

        callIt = false;
    }

    void stopAtFrameNo(long frame) {

        frameToStop = frame;
    }

    // return the frame number of the next frame
    long getFrameNumber() {

        // get info of from the capture device
        long fnumber = static_cast<long>(
                capture.get(CV_CAP_PROP_POS_FRAMES));
        return fnumber;
    }
};

// The frame processor interface
class FrameProcessor {

public:
    // processing method
    virtual void process(cv:: Mat &input, cv:: Mat &output)= 0;
};



class FeatureTracker : public FrameProcessor {

    cv::Mat gray;         // current gray-level image
    cv::Mat gray_prev;      // previous gray-level image
    // tracked features from 0->1
    std::vector<cv::Point2f> points[2];
    // initial position of tracked points
    std::vector<cv::Point2f> initial;
    std::vector<cv::Point2f> features;  // detected features
    int max_count;     // maximum number of features to detect
    double qlevel;    // quality level for feature detection
    double minDist;   // min distance between two points
    std::vector<uchar> status; // status of tracked features
    std::vector<float> err;    // error in tracking

public:

    FeatureTracker() : max_count(500), qlevel(0.01), minDist(10.) {}


    void process(cv:: Mat &frame, cv:: Mat &output) {

        // convert to gray-level image
        cv::cvtColor(frame, gray, CV_BGR2GRAY);
        frame.copyTo(output);

        // 1. if new feature points must be added
        if(addNewPoints())
        {
            // detect feature points
            detectFeaturePoints();
            // add the detected features to
            // the currently tracked features
            points[0].insert(points[0].end(),features.begin(),features.end());
            initial.insert(initial.end(),features.begin(),features.end());
        }

        // for first image of the sequence
        if(gray_prev.empty())
            gray.copyTo(gray_prev);

        // 2. track features
        cv::calcOpticalFlowPyrLK(gray_prev, gray, // 2 consecutive images
                                 points[0], // input point positions in first image
                                 points[1], // output point positions in the 2nd image
                                 status,    // tracking success
                                 err);      // tracking error

        // 3. loop over the tracked points to reject some
        int k=0;
        for( int i= 0; i < points[1].size(); i++ ) {

            // do we keep this point?
            if (acceptTrackedPoint(i)) {
                // keep this point in vector
                initial[k]= initial[i];
                points[1][k++] = points[1][i];
            }
        }

        // eliminate unsuccesful points
        points[1].resize(k);
        initial.resize(k);

        // 4. handle the accepted tracked points
        handleTrackedPoints(frame, output);

        // 5. current points and image become previous ones
        std::swap(points[1], points[0]);
        cv::swap(gray_prev, gray);
    }

    // feature point detection
    void detectFeaturePoints() {

        // detect the features
        cv::goodFeaturesToTrack(gray, // the image
                                features,   // the output detected features
                                max_count,  // the maximum number of features
                                qlevel,     // quality level
                                minDist);   // min distance between two features
    }

    // determine if new points should be added
    bool addNewPoints() {

        // if too few points
        return points[0].size() <= 10;
    }

    // determine which tracked point should be accepted
    bool acceptTrackedPoint(int i) {

        return status[i] &&
               // if point has moved
               (abs(points[0][i].x - points[1][i].x) + (abs(points[0][i].y - points[1][i].y)) > 2);
    }

    // handle the currently tracked points
    void handleTrackedPoints(cv::Mat &frame, cv::Mat &output) {

        // for all tracked points
        for (int i = 0; i < points[1].size(); i++) {

            // draw line and circle
            cv::line(output,
                     initial[i],  // initial position
                     points[1][i],// new position
                     cv::Scalar(255, 255, 255));
            cv::circle(output, points[1][i], 3, cv::Scalar(255, 255, 255), -1);
        }
    }
};



#endif //MAIN_INPUT_OUTPUT_H

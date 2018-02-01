//
// Created by veikas on 26.01.18.
//

#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>
#include "PlotFlow.h"
#include "datasets.h"
#include "Dataset.h"
#include "FlowImageExtended.h"


void PlotFlow::plot(const Dataset &dataset, const std::string &resultsordner) {

    char folder_name_flow[50], folder_name_plot[50];
    char file_name_image[50];
    cv::Mat showErrorImage;

    for ( int frame_skip = 1; frame_skip < MAX_SKIPS; frame_skip++ ){

        sprintf(folder_name_flow, "flow_occ_%02d", frame_skip);
        sprintf(folder_name_plot, "plots_%02d", frame_skip);
        cv::namedWindow(folder_name_flow, CV_WINDOW_AUTOSIZE);

        for (ushort frame_count=1; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            if ( frame_count%frame_skip != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", frame_count);
            std::string temp_gt_flow_path = dataset.getGroundTruthFlowPath().string() + "/" + folder_name_flow + "/"
                                            + file_name_image;
            std::string temp_result_flow_path = dataset.getResultPath().string() + "/" + resultsordner + "/" +
                                                folder_name_flow + "/" + file_name_image;
            std::string temp_plot_flow_path = dataset.getResultPath().string() + "/" + resultsordner + "/" +
                                              folder_name_plot + "/" + file_name_image;
            FlowImageExtended gt_flow_read(temp_gt_flow_path);
            FlowImageExtended result_flow_read(temp_result_flow_path);

            png::image<png::rgb_pixel> errorImage(dataset.getFrameSize().width, dataset.getFrameSize().height);

            // all black when the flow is identical. logcolor = false
            // all blue when the flow is identical. logcolor = true
            errorImage = gt_flow_read.errorImage(result_flow_read, result_flow_read, true);
            errorImage.write(temp_plot_flow_path);

            showErrorImage = cv::imread(temp_plot_flow_path, CV_LOAD_IMAGE_ANYCOLOR);
            cv::imshow(folder_name_flow, showErrorImage);
            cv::waitKey(10);
        }

        cv::destroyAllWindows();
    }
}

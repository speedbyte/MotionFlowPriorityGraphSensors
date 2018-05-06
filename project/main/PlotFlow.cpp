//
// Created by veikas on 26.01.18.
//

#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>
#include "PlotFlow.h"
#include "datasets.h"
#include "Dataset.h"
#include "FlowImageExtended.h"


void PlotFlow::plot(const std::string &resultsordner) {

    char frame_skip_folder_suffix[50], folder_name_plot[50];
    char file_name_image[50];
    cv::Mat showErrorImage;

    for ( int sensor_index = 1; sensor_index < MAX_SKIPS; sensor_index++ ){

        sprintf(frame_skip_folder_suffix, "flow_occ_%02d", sensor_index);
        sprintf(folder_name_plot, "plots_%02d", sensor_index);
        cv::namedWindow(frame_skip_folder_suffix, CV_WINDOW_AUTOSIZE);

        for (ushort frame_count=1; frame_count < MAX_ITERATION_RESULTS; frame_count++) {
            if ( frame_count%sensor_index != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", frame_count*sensor_index);
            std::string temp_gt_flow_path = Dataset::getGroundTruthPath().string() + frame_skip_folder_suffix + "/"
                                            + file_name_image;
            std::string temp_result_flow_path = Dataset::getResultPath().string() + "/" + resultsordner + "/" +
                                                frame_skip_folder_suffix + "/" + file_name_image;
            std::string temp_plot_flow_path = Dataset::getResultPath().string() + "/" + resultsordner + "/" +
                                              folder_name_plot + "/" + file_name_image;
            FlowImageExtended gt_flow_read(temp_gt_flow_path);
            FlowImageExtended result_flow_read(temp_result_flow_path);

            png::image<png::rgb_pixel> errorImage(Dataset::getFrameSize().width, Dataset::getFrameSize().height);

            // all black when the flow is identical. logcolor = false
            // all blue when the flow is identical. logcolor = true
            errorImage = gt_flow_read.errorImage(result_flow_read, result_flow_read, true);
            errorImage.write(temp_plot_flow_path);

            showErrorImage = cv::imread(temp_plot_flow_path, CV_LOAD_IMAGE_ANYCOLOR);
            cv::imshow(frame_skip_folder_suffix, showErrorImage);
            cv::waitKey(10);
        }

        cv::destroyAllWindows();
    }
}

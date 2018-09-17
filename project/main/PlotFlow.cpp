//
// Created by veikas on 26.01.18.
//

#include <opencv2/core/mat.hpp>
#include <opencv/cv.hpp>
#include "PlotFlow.h"
#include "Dataset.h"
#include "Dataset.h"
#include "FlowImageExtended.h"


void PlotFlow::plot(const std::string &resultsordner) {

    char sensor_index_folder_suffix[50], folder_name_plot[50];
    char file_name_image[50];
    cv::Mat showErrorImage;

    for ( int sensor_index = 0; sensor_index < Dataset::SENSOR_COUNT; sensor_index++ ){

        sprintf(sensor_index_folder_suffix, "flow_occ_%02d", sensor_index);
        sprintf(folder_name_plot, "plots_%02d", sensor_index);
        cv::namedWindow(sensor_index_folder_suffix, CV_WINDOW_AUTOSIZE);

        for (ushort current_frame_index=1; current_frame_index < Dataset::MAX_ITERATION_RESULTS; current_frame_index++) {
            if ( current_frame_index%sensor_index != 0 ) {
                continue;
            }
            sprintf(file_name_image, "000%03d_10.png", current_frame_index*sensor_index);
            std::string temp_gt_flow_path = Dataset::m_dataset_gtpath.string() + sensor_index_folder_suffix + "/"
                                            + file_name_image;
            std::string temp_result_flow_path = Dataset::m_dataset_resultpath.string() + "/" + resultsordner + "/" +
                                                sensor_index_folder_suffix + "/" + file_name_image;
            std::string temp_plot_flow_path = Dataset::m_dataset_resultpath.string() + "/" + resultsordner + "/" +
                                              folder_name_plot + "/" + file_name_image;
            FlowImageExtended gt_flow_read(temp_gt_flow_path);
            FlowImageExtended result_flow_read(temp_result_flow_path);

            png::image<png::rgb_pixel> errorImage(Dataset::m_frame_size.width, Dataset::m_frame_size.height);

            // all black when the flow is identical. logcolor = false
            // all blue when the flow is identical. logcolor = true
            errorImage = gt_flow_read.errorImage(result_flow_read, result_flow_read, true);
            errorImage.write(temp_plot_flow_path);

            showErrorImage = cv::imread(temp_plot_flow_path, CV_LOAD_IMAGE_ANYCOLOR);
            cv::imshow(sensor_index_folder_suffix, showErrorImage);
            cv::waitKey(10);
        }

        cv::destroyAllWindows();
    }
}

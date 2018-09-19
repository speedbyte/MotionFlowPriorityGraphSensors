//
// Created by veikas on 26.01.18.
//

#ifndef MAIN_DATASET_H
#define MAIN_DATASET_H

#include <opencv2/core/types.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <map>

#define KITTI_FLOW_DATASET_PATH "../../../datasets/kitti_flow_dataset/"
#define KITTI_RAW_DATASET_PATH "../../../datasets/kitti_raw_dataset/"

#define KITTI_RAW_CALIBRATION_PATH "../../../datasets/kitti_raw_dataset/data/"

#define MATLAB_DATASET_PATH "../../../datasets/matlab_dataset/"
#define CPP_DATASET_PATH "../../../datasets/cpp_dataset/"

#define VIRES_DATASET_PATH "../../../datasets/vires_dataset/"

#define DISPLACEMENT_ROUND_OFF 100

#define IMAGE_SKIP_FACTOR_DYNAMIC 1

#define MAX_SKIPS_REAL 1 // 1 means do not generate flow, 2 means generate base flow

#define IMAGE_WIDTH 1200
#define IMAGE_HEIGHT 400

#define ANGLE_ERROR_TOLERANCE 5

#define MAX_ALLOWED_OBJECTS 10
#define MAX_ALLOWED_SENSORS 10

//#define MAX_ALLOWED_SENSOR_GROUPS_EVALUATION 2
//#define Dataset::SENSOR_COUNT MAX_ALLOWED_SENSOR_GROUPS_EVALUATION + (1%MAX_ALLOWED_SENSOR_GROUPS_EVALUATION)

#define STENCIL_GRID_EXTENDER 15 // 10 pixels more than roi on each side

#define DO_STENCIL_GRID_EXTENSION 0

typedef enum {
    continous_frames = 0,
    pariwise_frames = 1,
    video_frames = 2
} FRAME_TYPES;

typedef enum {
    gt,
    lk,
    fb,
    sf,
    tvl
} ALGO_TYPES;

#define FOCAL_X 600
#define FOCAL_Y 600

#define START_BENCHMARK tic = std::chrono::steady_clock::now();
#define PRINT_BENCHMARK(prefix) printf ( #prefix" - %f\n", static_cast<float>(std::chrono::duration_cast<milliseconds>(std::chrono::steady_clock::now() - tic).count()) );


class Dataset {

private:
    Dataset() {}

public:

    static cv::Size_<unsigned> m_frame_size;
    static ushort m_depth;
    static ushort m_cn;

    static ushort SENSOR_COUNT;
    static ushort ITERATION_START_POINT;
    static ushort ITERATION_END_POINT;
    static bool GENERATE;
    static ushort MAX_ITERATION_RESULTS;
    static ushort MAX_GENERATION_DATASET;
    static ushort MAX_ITERATION_GT_SCENE_GENERATION_DATASET;

    static boost::filesystem::path m_dataset_basepath;
    static boost::filesystem::path  m_dataset_gtpath;
    static boost::filesystem::path  m_dataset_resultpath;

    static std::map<std::string, bool> m_dataprocessing_map;
    static std::map<std::string, ushort> m_algorithm_map;
    static std::map<std::string, ushort> m_algorithm_map_original;

    static bool m_execute_algorithm;

    static void fillDataset(cv::Size_<unsigned> frame_size, ushort depth, ushort cn, std::string dataset_path,
            std::string unterordner, std::string resultordner, bool generate, ushort start, ushort stop, ushort max_frames_dataset, std::map<std::string, bool> dataprocessing_map,
                            std::map<std::string, ushort> algorithm_map, std::vector<ushort> evaulation_list);

    static const ushort getMakeType();

};

typedef struct object_location_inertial_m { float location_x_m; float location_y_m; float location_z_m;} object_location_inertial_m_str;

//w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
typedef struct object_dimensions_px { float width_px; float height_px; float dim_length_m; } object_dimensions_px_str;

//w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
typedef struct object_realworld_dim_m { float dim_width_m; float dim_height_m; float dim_length_m; } object_realworld_dim_m_str;

typedef struct object_rotation_inertial_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad; } object_rotation_inertial_rad_str;

//offset. Shift of point from the center of mass
typedef struct object_offset_m { float offset_x; float offset_y; float offset_z; } object_offset_m_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_location_m { float location_x_m; float location_y_m; float location_z_m;} object_location_m_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct region_of_interest_px { float x; float y; float width_px; float height_px;} region_of_interest_px_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_location_px { float location_x_px; float location_y_px; float location_z_px; cv::Point2f cog_px;} object_location_px_str;

//x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
typedef struct object_occlusion { signed char occlusion_px; signed char occlusion_usk; signed char occlusion_inertial; } object_occlusion_str;

class STRUCT_GT_OBJECTS_ALL {

public:

    //Each line contains one object annotation with the following columns:
    //frame: frame index in the video (starts from 0)
    ushort frame_no;

    std::string object_name;

    //tid: track identification number (unique for each object instance)
    ushort tid;

    //label: KITTI-like name of the 'type' of the object (Car, Van, DontCare)
    std::string label;

    //truncated: (changed name in v1.3) KITTI-like truncation flag (0: not truncated, 1: truncated, 2: heavily truncated, marked as “DontCare”)
    bool truncated;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    ushort occluded;

    //occluded: (changed name in v1.3) KITTI-like occlusion flag  (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    ushort visMask;


    //alpha: KITTI-like observation angle of the object in [-pi..pi]
    float alpha_rad;

    //l, t, r, b: KITTI-like 2D 'bbox', respectively left, top, right, bottom bounding box in pixel coordinates (inclusive, (0,0) origin is on the upper left corner of the image)
    struct bounding_box_m {
        cv::Point2f bb_lower_bottom_px;
        cv::Point2f bb_lower_right_px;
        cv::Point2f bb_lower_top_px;
        cv::Point2f bb_lower_left_px;
        cv::Point2f bb_higher_bottom_px;
        cv::Point2f bb_higher_right_px;
        cv::Point2f bb_higher_top_px;
        cv::Point2f bb_higher_left_px;
    } m_bounding_box;

    object_dimensions_px_str m_object_dimension_camera_px;

    object_realworld_dim_m_str m_object_dimension_realworld_m;

    object_location_inertial_m_str m_object_location_inertial_m;

    object_location_m_str m_object_location_usk_m;

    object_location_px_str m_object_location_camera_px;

    object_occlusion_str m_object_occlusion;

    //(center of bottom face of 3D bounding box)
    //ry: KITTI-like 3D object 'rotation_y', rotation around Y-axis (yaw) in camera coordinates [-pi..pi]
    //(KITTI convention is ry == 0 iff object is aligned with x-axis and pointing right)
    //rx: rotation around X-axis (pitch) in camera coordinates [-pi..pi]
    //rz: rotation around Z-axis (roll) in camera coordinates [-pi..pi]
    struct object_rotation_rad { float rotation_rx_roll_rad; float rotation_ry_pitch_rad; float rotation_rz_yaw_rad;} m_object_rotation_rad;

    object_rotation_inertial_rad_str m_object_rotation_inertial_rad;

    region_of_interest_px_str m_region_of_interest_px;

    struct object_distances { float sensor_to_obj_px; float sensor_to_obj_usk; float total_distance_covered; } m_object_distances;

    //truncr: (changed in v1.3) object 2D truncation ratio in [0..1] (0: no truncation, 1: entirely truncated)
    bool truncr;

    //occupr: object 2D occupancy ratio (fraction of non-occluded pixels) in [0..1] (0: fully occluded, 1: fully visible, independent of truncation)
    bool occupr;

    //orig_label: original KITTI-like name of the 'type' of the object ignoring the 'DontCare' rules (allows to know original type of DontCare objects)
    std::string orig_label;

    //moving: 0/1 flag to indicate whether the object is really moving between this frame and the next one
    bool moving;

    //model: the name of the 3D model used to render the object (can be used for fine-grained recognition)
    std::string cad_3d_model;

    //color: the name of the color of the object
    std::string color;

    object_offset_m_str m_object_offset_m;

    //speed of the object
    struct object_speed_m { float x; float y; float z; } m_object_speed;
    struct object_speed_inertial_m { float x; float y; float z; } m_object_speed_inertial;

    /*
     Remarks about 3D information

    Internally, the 3D world is projected on the screen by using the Unity Engine rendering pipeline and various shaders. You can reproduce this by projecting points from the camera space (e.g., coordinates x3d,y3d,z3d) to the image pixels by using our camera intrinsic matrix (in pixels, constant, computed from our 1242x375 resolution and 29° fov):

          [[725,    0, 620.5],
    K =  [   0, 725, 187.0],
            [   0,     0,       1]]

    In our system of 3D camera coordinates x is going to the right, y is going down, and z is going forward (the origin is the optical center of the camera).*
     */

    /*
     * Camera pose (extrinsic parameters): link (1.1MB)

    The 3D camera pose (rotation and translation) for each frame of a video consists of one CSV-like text files named:

    vkitti_<version>_extrinsicsgt/<world>_<variation>.txt

    Each file can be loaded with the following one-liner in Python using the popular pandas library (assuming ‘import pandas as pd’):

    extgt = pd.read_csv("<filename>", sep=" ", index_col=False)

    Each line consists of the frame index in the video (starts from 0) followed by the row-wise flattened 4x4 extrinsic matrix at that frame:

    r1,1 r1,2 r1,3 t1
    M = r2,1 r2,2 r2,3 t2
    r3,1 r3,2 r3,3 t3
    0     0     0    1

    where ri,j are the coefficients of the camera rotation matrix R and ti are the coefficients of the camera translation coefficients t.

    This matrix can be used to convert points from world space to the camera space. For a point p = (x,y,z) in the world space, P = (x,y,z,1) in the homogeneous coordinates, you can get the coordinates in the camera space by doing the dot product MP.

    See section above for the camera intrinsic parameters and description of our camera coordinate system.
     */


    /*
    The MOT ground truth for each video consists of a CSV-like text file named:

    vkitti_<version>_motgt/<world>_<variation>.txt

    These files are in a KITTI-like format that can be loaded with the following one-liner in python using the popular pandas library (assuming ‘import pandas as pd’):

     motgt = pd.read_csv("<filename>", sep=" ", index_col=False)

    */

    // class for extracting meta data and supplying to Objects at runtime
    // Each dataset describes the way it extracts the metadata. I will try to keep it very generalized ie. follow the
    // kitti concept.

    // this also includes calibration data

    // for example[KITTI_RAW_CALIBRATION]
    // PATH = "../../../datasets/kitti_raw_dataset/data/"
    // EXECUTE = 0

};


#endif //MAIN_DATASET_H

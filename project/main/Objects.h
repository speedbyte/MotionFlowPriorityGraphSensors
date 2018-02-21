//
// Created by veikas on 08.02.18.
//

#ifndef MAIN_OBJECTS_H
#define MAIN_OBJECTS_H


#include <opencv2/core/types.hpp>

class Objects {

private:
/*
 The MOT ground truth for each video consists of a CSV-like text file named:

    vkitti_<version>_motgt/<world>_<variation>.txt

These files are in a KITTI-like format that can be loaded with the following one-liner in python using the popular pandas library (assuming ‘import pandas as pd’):

     motgt = pd.read_csv("<filename>", sep=" ", index_col=False)

Each line contains one object annotation with the following columns:
    frame: frame index in the video (starts from 0)
    tid: track identification number (unique for each object instance)
    label: KITTI-like name of the 'type' of the object (Car, Van, DontCare)
    truncated: (changed name in v1.3) KITTI-like truncation flag
    (0: not truncated, 1: truncated, 2: heavily truncated, marked as “DontCare”)
    occluded: (changed name in v1.3) KITTI-like occlusion flag
    (0: not occluded, 1; occluded, 2: heavily occluded, marked as “DontCare”)
    alpha: KITTI-like observation angle of the object in [-pi..pi]
    l, t, r, b: KITTI-like 2D 'bbox', respectively left, top, right, bottom bounding box in pixel coordinates
    (inclusive, (0,0) origin is on the upper left corner of the image)
    w3d, h3d, l3d: KITTI-like 3D object 'dimensions', respectively width, height, length in meters
    x3d, y3d, z3d: KITTI-like 3D object 'location', respectively x, y, z in camera coordinates in meters
    (center of bottom face of 3D bounding box)
    ry: KITTI-like 3D object 'rotation_y', rotation around Y-axis (yaw) in camera coordinates [-pi..pi]
    (KITTI convention is ry == 0 iff object is aligned with x-axis and pointing right)
    rx: rotation around X-axis (pitch) in camera coordinates [-pi..pi]
    rz: rotation around Z-axis (roll) in camera coordinates [-pi..pi]
    truncr: (changed in v1.3) object 2D truncation ratio in [0..1] (0: no truncation, 1: entirely truncated)
    occupr: object 2D occupancy ratio (fraction of non-occluded pixels) in [0..1]
    (0: fully occluded, 1: fully visible, independent of truncation)
    orig_label: original KITTI-like name of the 'type' of the object ignoring the 'DontCare' rules
    (allows to know original type of DontCare objects)
    moving: 0/1 flag to indicate whether the object is really moving between this frame and the next one
    model: the name of the 3D model used to render the object (can be used for fine-grained recognition)
    color: the name of the color of the object
  */
protected:

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > m_obj_extrapolated_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_shape_pixel_point_pixel_displacement;

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > >
            m_obj_extrapolated_stencil_pixel_point_pixel_displacement;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;

    std::vector<std::vector<bool> > m_obj_extrapolated_mean_visibility;

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >
            m_obj_line_parameters;

    int m_ObjectWidth;

    int m_ObjectHeight;

    unsigned m_objectId;

    std::vector<bool> m_obj_base_visibility;

    std::vector<std::vector<bool> >  m_obj_extrapolated_visibility;

    const std::string m_objectName;



public:

    Objects( std::string objectName) : m_objectName(objectName) {}

    Objects( std::string objectName, int width, int height, std::vector<std::vector<bool> >  extrapolated_visibility) : m_objectName(objectName) , m_ObjectWidth(width), m_ObjectHeight
            (height), m_obj_extrapolated_visibility(extrapolated_visibility) {}

    std::string getObjectName() const {
        return m_objectName;
    }

    void generate_obj_extrapolated_pixel_centroid_pixel_displacement_mean(const unsigned &max_skips, const std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > &obj_extrapolated_stencil_pixel_point_pixel_displacement);

    void generate_obj_line_parameters( const unsigned &max_skips);

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >    get_obj_extrapolated_pixel_centroid_pixel_displacement_mean()
    const {
        return m_obj_extrapolated_pixel_centroid_pixel_displacement_mean;
    }

    int getWidth() const {
        return m_ObjectWidth;
    }

    int getHeight() const {
        return m_ObjectHeight;
    }

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_line_parameters() const {
        return m_obj_line_parameters;
    }

    std::vector<std::vector<bool> >  get_obj_extrapolated_visibility()
    const {
        return m_obj_extrapolated_visibility;
    }

    std::vector<bool>  get_obj_base_visibility()
    const {
        return m_obj_base_visibility;
    }

    unsigned getObjectId() const {
        return m_objectId;
    }

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > get_obj_extrapolated_shape_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_shape_pixel_point_pixel_displacement;
    };

    std::vector<std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > > > get_obj_extrapolated_stencil_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_stencil_pixel_point_pixel_displacement;
    };

    std::vector<std::vector<std::pair<cv::Point2f, cv::Point2f> > >  get_obj_extrapolated_pixel_point_pixel_displacement() const {
        return m_obj_extrapolated_pixel_point_pixel_displacement;
    }


};


#endif //MAIN_OBJECTS_H

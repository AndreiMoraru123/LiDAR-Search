#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry>

// BoxQ is a struct that contains the information of a 3D bounding box
struct BoxQ
{
    Eigen::Vector3f bboxTransform;  // position of box center
    Eigen::Quaternionf bboxQuaternion;  // orientation of box
    float cube_length;  // length of box
    float cube_width;  // width of box
    float cube_height;  // height of box
};

struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

#endif /* BOX_H */
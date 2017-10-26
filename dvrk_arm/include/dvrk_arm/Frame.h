#ifndef CDVRK_FrameH
#define CDVRK_FrameH

#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"

struct Frame{
public:
    Frame(){
        pos.setZero();
        rot_quat.setRPY(0,0,0);
        rot_mat.setRotation(rot_quat);
        trans.setOrigin(pos);
        trans.setRotation(rot_quat);
    }
    ~Frame(){}
    tf::Transform trans;
    tf::Vector3 pos;
    tf::Quaternion rot_quat;
    tf::Matrix3x3 rot_mat;
};

#endif


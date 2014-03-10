// license
// Author: Zihan Chen

#ifndef CONVERSIONS_CISST_TF_H
#define CONVERSIONS_CISST_TF_H

#include <tf/transform_datatypes.h>
#include <cisstVector.h>

namespace cisst
{

/// Converts an cisst vct3 into a tf Vector3
void vectorCisstToTF(const vct3 &c, tf::Vector3 &t);

/// Converts a tf Vector3 into an Eigen Vector3d
void vectorTFToCisst(const tf::Vector3 &t, vct3 &c);

/// Converts a cisst matrix to tf matrix
void matrixCisstToTF(const vctRot3 &c, tf::Matrix3x3 &t);

/// Converts a tf matrix to cisst matrix
void matrixTFToCisst(const tf::Matrix3x3 &t, vctRot3 &c);

} // namespace tf

#endif

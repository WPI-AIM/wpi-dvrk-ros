
#include "cisst_conversions/cisst_tf.h"

namespace cisst
{

void vectorCisstToTF(const vct3 &c, tf::Vector3 &t)
{
  for (int i = 0; i < 3; i++)
    t[i] = c[i];
}

void vectorTFToCisst(const tf::Vector3 &t, vct3 &c)
{
  for (int i = 0; i < 3; i++)
    c[i] = t[i];
}

void matrixCisstToTF(const vctRot3 &c, tf::Matrix3x3 &t)
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      t[i][j] = c[i][j];
}

void matrixTFToCisst(const tf::Matrix3x3 &t, vctRot3 &c)
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      c[i][j] = t[i][j];
}

}

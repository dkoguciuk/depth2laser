#include <transform_matrix.h>
#include <stdio.h>

TransformMatrix::TransformMatrix()
{
    // hardMatrix (transformation between depth optical frame and depth frame)
    BareMatrix help23, help12, help01;

    help23.setZero();
    help12.setZero();
    help01.setZero();

    help23(0,0) = cos(M_PI/2);
    help23(0,2) = sin(M_PI/2);
    help23(1,1) = 1;
    help23(2,0) = -sin(M_PI/2);
    help23(2,2) = cos(M_PI/2);
    help23(3,3) = 1;

    help12(0,0) = 1;
    help12(1,1) = cos(0);
    help12(1,2) = -sin(0);
    help12(2,1) = sin(0);
    help12(2,2) = cos(0);
    help12(3,3) = 1;

    help01(0,0) = cos(-M_PI/2);
    help01(0,1) = -sin(-M_PI/2);
    help01(1,0) = sin(-M_PI/2);
    help01(1,1) = cos(-M_PI/2);
    help01(2,2) = 1;
    help01(3,3) = 1;

    hardTransformation(0,0) = 1;
    hardTransformation(1,1) = 1;
    hardTransformation(2,2) = 1;
    hardTransformation(3,3) = 1;

    hardTransformation = help23 * help12 * help01;
    finalTransformation = hardTransformation;
}

void TransformMatrix::setRotationYPR(float x, float y, float z,
                                     float yaw, float pitch, float roll)
{
    BareMatrix user23, user12, user01;
    user23.setZero(); user12.setZero(); user01.setZero();

    user23(0,0) = cos(-yaw);
    user23(1,1) = 1;
    user23(0,2) = sin(-yaw);
    user23(2,0) = -sin(-yaw);
    user23(2,2) = cos(-yaw);
    user23(3,3) = 1;

    user12(0,0) = 1;
    user12(1,1) = cos(-pitch);
    user12(1,2) = -sin(-pitch);
    user12(2,1) = sin(-pitch);
    user12(2,2) = cos(-pitch);
    user12(3,3) = 1;

    user01(0,0) = cos(-roll);
    user01(0,1) = -sin(-roll);
    user01(1,0) = sin(-roll);
    user01(1,1) = cos(-roll);
    user01(2,2) = 1;
    user01(3,3) = 1;

    finalTransformation = hardTransformation * user23 * user12 * user01;
    finalTransformation(0,3) = x;
    finalTransformation(1,3) = y;
    finalTransformation(2,3) = z;
}

float* TransformMatrix::getTransformationMatrix()
{
    return ((float*)finalTransformation.data());
}




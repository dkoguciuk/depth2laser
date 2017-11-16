#ifndef TRANSFROM_MATRIX_H
#define TRANSFROM_MATRIX_H


/*
 * @file    transform_matrix.h
 * @authors Daniel Koguciuk <daniel.koguciuk@gmail.com>
 *          Tomasz Gąsior <tomziomgasior@gmail.com>
 * @date    04.03.2015
 * @section DESCTIPTION
 *
 * This is cloud transformation container. It is composed of
 * two components: first is the base transformation (translation
 * and rotation about y axis) and the second one is only rotation
 * and it's still changing. Main rule is as follows:
 * T0->3 = T2->3 * T1->2 * T0->1
 * Cusomizing:
 * T = Tchanging * Tstill
 */

#include <math.h>
#include <Eigen/Eigen>


class TransformMatrix
{
public:

    typedef Eigen::Matrix<float,4,4,Eigen::RowMajor> BareMatrix;

    // ================================
    // ======== PUBLIC METHODS ========
    // ================================

    /** brief Constructor. */
		TransformMatrix();

    /** \brief Get final transformation matrix. */
    float* getTransformationMatrix();

    /**
     * @brief setRotationYPR            Angles in camera_link frame!
     * @param yaw
     * @param pitch
     * @param roll
     */
    void setRotationYPR(float x, float y, float z,
                        float yaw, float pitch, float roll);

private:

    // ================================
    // ====== PRIVATE VARIABLES =======
    // ================================

    BareMatrix hardTransformation;
    BareMatrix finalTransformation;
};
#endif //TRANSFORM_MATRIX_H

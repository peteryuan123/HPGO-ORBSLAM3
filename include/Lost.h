//
// Created by mpl on 22-11-30.
//

#ifndef ORB_SLAM3_LOST_H
#define ORB_SLAM3_LOST_H

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <ceres/rotation.h>


#include <vector>

struct PureRotationFunctor
{
    PureRotationFunctor(Eigen::Vector3f Bv1, Eigen::Vector3f Bv2): mBv1(Bv1), mBv2(Bv2){};

    template<class T>
    bool operator() (const T* const q, T* residual) const
    {
        Eigen::Matrix<T, 3, 1> Bv1;
        Bv1 << T(mBv1(0)), T(mBv1(1)), T(mBv1(2));

        Eigen::Matrix<T, 3, 1> Bv2;
        Bv2 << T(mBv2(0)), T(mBv2(1)), T(mBv2(2));

        Eigen::Matrix<T, 3, 1> Bv2_projected;
        T q_ceres[4] = {q[3], q[0], q[1], q[2]};
        ceres::QuaternionRotatePoint(q_ceres, Bv1.data(), Bv2_projected.data());

        residual[0] = T(1) - Bv2(0)*Bv2_projected(0) - Bv2(1)*Bv2_projected(1) - Bv2(2)*Bv2_projected(2);
//        residual[0] = Bv2(0) / Bv2(2) - Bv2_projected(0) / Bv2_projected(2);
//        residual[1] = Bv2(1) / Bv2(2) - Bv2_projected(1) / Bv2_projected(2);
        return true;
    }

    Eigen::Vector3f mBv1;
    Eigen::Vector3f mBv2;

};


#endif //ORB_SLAM3_LOST_H

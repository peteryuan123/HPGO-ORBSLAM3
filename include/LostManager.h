//
// Created by mpl on 23-1-4.
//

#ifndef ORB_SLAM3_LOSTMANAGER_H
#define ORB_SLAM3_LOSTMANAGER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ceres/rotation.h>

#include <vector>
#include "Frame.h"
#include "Lie.hpp"

namespace ORB_SLAM3
{
    typedef Eigen::Matrix<double, 7, 1> Vector7d;
    typedef Eigen::Matrix<double, 8, 1> Vector8d;

    struct GraphNode
    {
        int mId;
        bool mfixed;

        Sophus::SE3f mSIM3_Tcw;
        Vector7d msim3_Tcw_opt; // omega, upsilon, sigma (r, t, s)

        g2o::Sim3 mScw;

        GraphNode() = default;
        GraphNode(int id, bool fixed, Sophus::SE3f initValue):mId(id), mfixed(fixed), mSIM3_Tcw(initValue)
        {
            Vector8d SIM3_Tcw_opt;
            SIM3_Tcw_opt << initValue.so3().unit_quaternion().w(),
                            initValue.so3().unit_quaternion().x(),
                            initValue.so3().unit_quaternion().y(),
                            initValue.so3().unit_quaternion().z(),
                            initValue.translation().x(),
                            initValue.translation().y(),
                            initValue.translation().z(),
                            1;
            msim3_Tcw_opt = Sim3Log(SIM3_Tcw_opt);

        }

        Sophus::SE3f toSophus()
        {
            update();
            return mSIM3_Tcw;
        }

        void update()
        {
            Vector8d SIM3_Tcw_opt = Sim3Exp(msim3_Tcw_opt);
            mScw = g2o::Sim3(Eigen::Quaterniond(SIM3_Tcw_opt(0), SIM3_Tcw_opt(1), SIM3_Tcw_opt(2), SIM3_Tcw_opt(3)),
                             Eigen::Vector3d(SIM3_Tcw_opt(4), SIM3_Tcw_opt(5), SIM3_Tcw_opt(6)),
                             SIM3_Tcw_opt(7));

            mSIM3_Tcw = Sophus::SE3f(Eigen::Quaternionf(SIM3_Tcw_opt(0), SIM3_Tcw_opt(1), SIM3_Tcw_opt(2), SIM3_Tcw_opt(3)),
                                     Eigen::Vector3f(SIM3_Tcw_opt(4), SIM3_Tcw_opt(5), SIM3_Tcw_opt(6)) / SIM3_Tcw_opt(7));
        }
    };

    struct WeakEdge
    {
        int mId0; // from
        int mId1; // to
        Sophus::SE3f mSIM3_Tc1c0;

        WeakEdge(int id0, int id1, const Sophus::SE3f& SIM3_Tc1c0): mId0(id0), mId1(id1), mSIM3_Tc1c0(SIM3_Tc1c0)
        {};
    };

    struct NormalEdge
    {
        int mId0; // from
        int mId1; // to
        Sophus::SE3f mSIM3_Tc1c0;
        float mScale;
        NormalEdge(int id0, int id1, const Sophus::SE3f& SIM3_Tc1c0, float scale): mId0(id0), mId1(id1), mSIM3_Tc1c0(SIM3_Tc1c0), mScale(scale)
        {};
    };

    // Functor
    struct Sim3MagicFunctor
    {
        Sim3MagicFunctor(WeakEdge edge)
        {
            mSIM3_Tc1c0 << edge.mSIM3_Tc1c0.so3().unit_quaternion().w(),
                          edge.mSIM3_Tc1c0.so3().unit_quaternion().x(),
                          edge.mSIM3_Tc1c0.so3().unit_quaternion().y(),
                          edge.mSIM3_Tc1c0.so3().unit_quaternion().z(),
                          edge.mSIM3_Tc1c0.translation().x(),
                          edge.mSIM3_Tc1c0.translation().y(),
                          edge.mSIM3_Tc1c0.translation().z(),
                          1;
        };

        template<class T>
        bool operator() (const T* const node0, const T* const node1, T* residual) const
        {
            Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);
            Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

            Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
            SIM3_Tc1c0 << T(mSIM3_Tc1c0(0)), T(mSIM3_Tc1c0(1)), T(mSIM3_Tc1c0(2)),
                    T(mSIM3_Tc1c0(3)), T(mSIM3_Tc1c0(4)), T(mSIM3_Tc1c0(5)),
                    T(mSIM3_Tc1c0(6)), T(mSIM3_Tc1c0(7));


            Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
            Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

            Eigen::Matrix<T, 8, 1> SIM3_Tc0c1_1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));
            Eigen::Matrix<T, 8, 1> SIM3_error_1 = Tdot(SIM3_Tc0c1_1, SIM3_Tc1c0);
            Eigen::Matrix<T, 7, 1> sim3_error_1 = Sim3Log(SIM3_error_1);

//        Eigen::Matrix<T, 8, 1> SIM3_Tc1c0_2 = Tdot(SIM3_Tc1w, Sim3Inv(SIM3_Tc0w));
//        Eigen::Matrix<T, 8, 1> SIM3_error_2 = Tdot(SIM3_Tc1c0_2, Sim3Inv(SIM3_Tc1c0));


//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
//        Eigen::Matrix<T, 7, 1> sim3_error_2 = Sim3Log(SIM3_error_2);

//        sim3_error(6) = T(0);
//        std::cout << sim3_error_1 << std::endl;

            for (int i = 0; i < 6; i++)
                residual[i] = sim3_error_1(i);

            return true;
        }
        Vector8d mSIM3_Tc1c0; // qw, qx, qy, qz, x, y, z, s
    };

    struct Sim3Functor
    {
        Sim3Functor(NormalEdge edge)
        {
            mSIM3_Tc1c0 << edge.mSIM3_Tc1c0.so3().unit_quaternion().w(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().x(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().y(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().z(),
                    edge.mSIM3_Tc1c0.translation().x(),
                    edge.mSIM3_Tc1c0.translation().y(),
                    edge.mSIM3_Tc1c0.translation().z(),
                    edge.mScale;
        }

        template<class T>
        bool operator() (const T* const node0, const T* const node1, T* residual) const
        {
            Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);
            Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

            Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
            SIM3_Tc1c0 << T(mSIM3_Tc1c0(0)), T(mSIM3_Tc1c0(1)), T(mSIM3_Tc1c0(2)),
                    T(mSIM3_Tc1c0(3)), T(mSIM3_Tc1c0(4)), T(mSIM3_Tc1c0(5)),
                    T(mSIM3_Tc1c0(6)), T(mSIM3_Tc1c0(7));


            Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
            Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

            Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

            Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
            Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

            for (int i = 0; i < 7; i++)
                residual[i] = sim3_error(i);

            return true;
        }

        Vector8d mSIM3_Tc1c0; // qw, qx, qy, qz, x, y, z, s

    };

    struct Sim3FunctorFixNode0
    {
        Sim3FunctorFixNode0(GraphNode fixedNode, NormalEdge edge)
        {
            Vector8d temp_T;
            temp_T << fixedNode.mSIM3_Tcw.so3().unit_quaternion().w(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().x(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().y(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().z(),
                    fixedNode.mSIM3_Tcw.translation().x(),
                    fixedNode.mSIM3_Tcw.translation().y(),
                    fixedNode.mSIM3_Tcw.translation().z(),
                    1;
            msim3_Tc0w = Sim3Log(temp_T);

            mSIM3_Tc1c0 << edge.mSIM3_Tc1c0.so3().unit_quaternion().w(),
                        edge.mSIM3_Tc1c0.so3().unit_quaternion().x(),
                        edge.mSIM3_Tc1c0.so3().unit_quaternion().y(),
                        edge.mSIM3_Tc1c0.so3().unit_quaternion().z(),
                        edge.mSIM3_Tc1c0.translation().x(),
                        edge.mSIM3_Tc1c0.translation().y(),
                        edge.mSIM3_Tc1c0.translation().z(),
                        edge.mScale;

        };

        template<class T>
        bool operator() (const T* const node1, T* residual) const
        {
            Eigen::Matrix<T, 7, 1> sim3_Tc1w(node1);

            Eigen::Matrix<T, 7, 1> sim3_Tc0w;
            sim3_Tc0w << T(msim3_Tc0w(0)), T(msim3_Tc0w(1)), T(msim3_Tc0w(2)),
                    T(msim3_Tc0w(3)), T(msim3_Tc0w(4)), T(msim3_Tc0w(5)),
                    T(msim3_Tc0w(6));

            Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
            SIM3_Tc1c0 << T(mSIM3_Tc1c0(0)), T(mSIM3_Tc1c0(1)), T(mSIM3_Tc1c0(2)),
                    T(mSIM3_Tc1c0(3)), T(mSIM3_Tc1c0(4)), T(mSIM3_Tc1c0(5)),
                    T(mSIM3_Tc1c0(6)), T(mSIM3_Tc1c0(7));


            Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
            Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

            Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

            Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
            Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

            for (int i = 0; i < 7; i++)
                residual[i] = sim3_error(i);

            return true;
        }

        Vector8d mSIM3_Tc1c0; // qw, qx, qy, qz, x, y, z, s
        Vector7d msim3_Tc0w;
    };

    struct Sim3FunctorFixNode1
    {
        Sim3FunctorFixNode1(GraphNode fixedNode, NormalEdge edge)
        {
            Vector8d temp_T;
            temp_T << fixedNode.mSIM3_Tcw.so3().unit_quaternion().w(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().x(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().y(),
                    fixedNode.mSIM3_Tcw.so3().unit_quaternion().z(),
                    fixedNode.mSIM3_Tcw.translation().x(),
                    fixedNode.mSIM3_Tcw.translation().y(),
                    fixedNode.mSIM3_Tcw.translation().z(),
                    1;
            msim3_Tc1w = Sim3Log(temp_T);

            mSIM3_Tc1c0 << edge.mSIM3_Tc1c0.so3().unit_quaternion().w(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().x(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().y(),
                    edge.mSIM3_Tc1c0.so3().unit_quaternion().z(),
                    edge.mSIM3_Tc1c0.translation().x(),
                    edge.mSIM3_Tc1c0.translation().y(),
                    edge.mSIM3_Tc1c0.translation().z(),
                    edge.mScale;
        };

        template<class T>
        bool operator() (const T* const node0, T* residual) const
        {
            Eigen::Matrix<T, 7, 1> sim3_Tc0w(node0);

            Eigen::Matrix<T, 7, 1> sim3_Tc1w;
            sim3_Tc1w << T(msim3_Tc1w(0)), T(msim3_Tc1w(1)), T(msim3_Tc1w(2)),
                    T(msim3_Tc1w(3)), T(msim3_Tc1w(4)), T(msim3_Tc1w(5)),
                    T(msim3_Tc1w(6));

            Eigen::Matrix<T, 8, 1> SIM3_Tc1c0;
            SIM3_Tc1c0 << T(mSIM3_Tc1c0(0)), T(mSIM3_Tc1c0(1)), T(mSIM3_Tc1c0(2)),
                    T(mSIM3_Tc1c0(3)), T(mSIM3_Tc1c0(4)), T(mSIM3_Tc1c0(5)),
                    T(mSIM3_Tc1c0(6)), T(mSIM3_Tc1c0(7));


            Eigen::Matrix<T, 8, 1> SIM3_Tc0w = Sim3Exp(sim3_Tc0w);
            Eigen::Matrix<T, 8, 1> SIM3_Tc1w = Sim3Exp(sim3_Tc1w);

            Eigen::Matrix<T, 8, 1> SIM3_Tc0c1 = Tdot(SIM3_Tc0w, Sim3Inv(SIM3_Tc1w));

            Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc0c1, SIM3_Tc1c0);
//        Eigen::Matrix<T, 8, 1> SIM3_error = Tdot(SIM3_Tc1c0, SIM3_Tc0c1); // ? inverse ?
            Eigen::Matrix<T, 7, 1> sim3_error = Sim3Log(SIM3_error);

            for (int i = 0; i < 7; i++)
                residual[i] = sim3_error(i);

            return true;
        }

        Vector8d mSIM3_Tc1c0; // qw, qx, qy, qz, x, y, z, s
        Vector7d msim3_Tc1w;
    };
    // Functor



    class LostManager
    {
    protected:
        std::unordered_map<int, GraphNode> mGraphNodes;
        std::vector<WeakEdge> mWeakEdges;
        std::vector<NormalEdge> mNormalEdges;

    public:
        LostManager() = default;
        ~LostManager() = default;

        void addWeakEdges(int id0, int id1, const Sophus::SE3f& Tc1c0);

        void addNormalEdges(int id0, int id1, const Sophus::SE3f& Tc1c0, float scale);

        void addGraphNode(int id, const Sophus::SE3f& Tcw, bool fixed);

        void popLastWeakEdge(int& id0, int& id1, Sophus::SE3f& Tc1c0);

        void clearGraph();

        void optimize();

        void optimizeWeakAsNormal();

        void PrintGraphInfo();

        void PringGraphShortInfo();

        bool findNode(int id);

        bool setFixed(int id);

        void PrintNodes();

        g2o::Sim3 getSimPose(int id);

    };
}




//struct PureRotationFunctor
//{
//    PureRotationFunctor(Eigen::Vector3f Bv1, Eigen::Vector3f Bv2): mBv1(Bv1), mBv2(Bv2){};
//
//    template<class T>
//    bool operator() (const T* const q, T* residual) const
//    {
//        Eigen::Matrix<T, 3, 1> Bv1;
//        Bv1 << T(mBv1(0)), T(mBv1(1)), T(mBv1(2));
//
//        Eigen::Matrix<T, 3, 1> Bv2;
//        Bv2 << T(mBv2(0)), T(mBv2(1)), T(mBv2(2));
//
//        Eigen::Matrix<T, 3, 1> Bv2_projected;
//        T q_ceres[4] = {q[3], q[0], q[1], q[2]};
//        ceres::QuaternionRotatePoint(q_ceres, Bv1.data(), Bv2_projected.data());
//
//        residual[0] = T(1) - Bv2(0)*Bv2_projected(0) - Bv2(1)*Bv2_projected(1) - Bv2(2)*Bv2_projected(2);
////        residual[0] = Bv2(0) / Bv2(2) - Bv2_projected(0) / Bv2_projected(2);
////        residual[1] = Bv2(1) / Bv2(2) - Bv2_projected(1) / Bv2_projected(2);
//        return true;
//    }
//
//    Eigen::Vector3f mBv1;
//    Eigen::Vector3f mBv2;
//
//};
#endif //ORB_SLAM3_LOSTMANAGER_H

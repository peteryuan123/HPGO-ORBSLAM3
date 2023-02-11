//
// Created by mpl on 23-1-4.
//
#include "LostManager.h"
#include <algorithm>

namespace ORB_SLAM3
{


    void LostManager::addWeakEdges(int id0, int id1, const Sophus::SE3f& Tc1c0)
    {
        mWeakEdges.emplace_back(id0, id1, Tc1c0);
    }

    void LostManager::addNormalEdges(int id0, int id1, const Sophus::SE3f &Tc1c0, float scale)
    {
        mNormalEdges.emplace_back(id0, id1, Tc1c0, scale);
    }

    void LostManager::addGraphNode(int id, const Sophus::SE3f& Tcw, bool fixed)
    {
        mGraphNodes[id] = GraphNode(id, fixed, Tcw);
    }

    void LostManager::popLastWeakEdge(int& id0, int& id1, Sophus::SE3f& Tc1c0)
    {
        id0 = std::move(mWeakEdges.back().mId0);
        id1 = std::move(mWeakEdges.back().mId1);
        Tc1c0 = std::move(mWeakEdges.back().mSIM3_Tc1c0);
        mWeakEdges.pop_back();
    }

    void LostManager::clearGraph()
    {
        mGraphNodes.clear();
        mWeakEdges.clear();
        mNormalEdges.clear();
    }

    void LostManager::optimize()
    {
        ceres::Problem problem;
        ceres::Solver::Options options;

        // add Normal Edge
        for (const NormalEdge& nEdge: mNormalEdges)
        {
            if (mGraphNodes.find(nEdge.mId0) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << nEdge.mId0 << " in the graph" << std::endl;
                continue;
            }
            if (mGraphNodes.find(nEdge.mId1) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << nEdge.mId1<< " in the graph" << std::endl;
                continue;
            }

            if (mGraphNodes[nEdge.mId0].mfixed)
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3FunctorFixNode0, 7, 7>(
                                new Sim3FunctorFixNode0(mGraphNodes[nEdge.mId0], nEdge));

                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId1].msim3_Tcw_opt.data());
            }
            else if (mGraphNodes[nEdge.mId1].mfixed)
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3FunctorFixNode1, 7, 7>(
                                new Sim3FunctorFixNode1(mGraphNodes[nEdge.mId1], nEdge));

                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId0].msim3_Tcw_opt.data());
            }else
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3Functor, 7, 7, 7>(new Sim3Functor(nEdge));

                // consider huber loss
                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId0].msim3_Tcw_opt.data(),
                                         mGraphNodes[nEdge.mId1].msim3_Tcw_opt.data());
            }
        }

        // add Weak Edge
        for (const WeakEdge& wEdge: mWeakEdges)
        {
            if (mGraphNodes.find(wEdge.mId0) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << wEdge.mId0 << " in the graph" << std::endl;
                continue;
            }
            if (mGraphNodes.find(wEdge.mId1) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << wEdge.mId1<< " in the graph" << std::endl;
                continue;
            }
            assert(!mGraphNodes[wEdge.mId0].mfixed);
            assert(!mGraphNodes[wEdge.mId1].mfixed);

            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3MagicFunctor, 6, 7, 7>(new Sim3MagicFunctor(wEdge));

            // consider huber loss
            problem.AddResidualBlock(costFunction, nullptr,
                                     mGraphNodes[wEdge.mId0].msim3_Tcw_opt.data(),
                                     mGraphNodes[wEdge.mId1].msim3_Tcw_opt.data());
        }

        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        std::cout << "-----------------------\n";
        for (auto it = mGraphNodes.begin(); it != mGraphNodes.end(); it++)
        {
            it->second.update();
            std::cout << Sim3Exp(it->second.msim3_Tcw_opt).transpose() << std::endl;
        }
        std::cout << "-----------------------\n";

    }

    void LostManager::optimizeWeakAsNormal()
    {
        ceres::Problem problem;
        ceres::Solver::Options options;

        // add Normal Edge
        for (const NormalEdge& nEdge: mNormalEdges)
        {
            if (mGraphNodes.find(nEdge.mId0) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << nEdge.mId0 << " in the graph" << std::endl;
                continue;
            }
            if (mGraphNodes.find(nEdge.mId1) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << nEdge.mId1<< " in the graph" << std::endl;
                continue;
            }

            if (mGraphNodes[nEdge.mId0].mfixed)
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3FunctorFixNode0, 7, 7>(
                                new Sim3FunctorFixNode0(mGraphNodes[nEdge.mId0], nEdge));

                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId1].msim3_Tcw_opt.data());
            }
            else if (mGraphNodes[nEdge.mId1].mfixed)
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3FunctorFixNode1, 7, 7>(
                                new Sim3FunctorFixNode1(mGraphNodes[nEdge.mId1], nEdge));

                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId0].msim3_Tcw_opt.data());
            }else
            {
                ceres::CostFunction* costFunction =
                        new ceres::AutoDiffCostFunction<Sim3Functor, 7, 7, 7>(new Sim3Functor(nEdge));

                // consider huber loss
                problem.AddResidualBlock(costFunction, nullptr,
                                         mGraphNodes[nEdge.mId0].msim3_Tcw_opt.data(),
                                         mGraphNodes[nEdge.mId1].msim3_Tcw_opt.data());
            }
        }

        // add Weak Edge
        for (const WeakEdge& wEdge: mWeakEdges)
        {
            if (mGraphNodes.find(wEdge.mId0) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << wEdge.mId0 << " in the graph" << std::endl;
                continue;
            }
            if (mGraphNodes.find(wEdge.mId1) == mGraphNodes.end())
            {
                std::cout << "no Node id:" << wEdge.mId1<< " in the graph" << std::endl;
                continue;
            }
            assert(!mGraphNodes[wEdge.mId0].mfixed);
            assert(!mGraphNodes[wEdge.mId1].mfixed);

            NormalEdge nEdge(wEdge.mId0, wEdge.mId1, wEdge.mSIM3_Tc1c0, 1);

            ceres::CostFunction* costFunction =
                    new ceres::AutoDiffCostFunction<Sim3Functor, 7, 7, 7>(new Sim3Functor(nEdge));

            // consider huber loss
            problem.AddResidualBlock(costFunction, nullptr,
                                     mGraphNodes[nEdge.mId0].msim3_Tcw_opt.data(),
                                     mGraphNodes[nEdge.mId1].msim3_Tcw_opt.data());
        }

        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        options.minimizer_progress_to_stdout = true;
        options.max_num_iterations = 100;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << std::endl;
        std::cout << "-----------------------\n";
        for (auto it = mGraphNodes.begin(); it != mGraphNodes.end(); it++)
        {
            it->second.update();
            std::cout << Sim3Exp(it->second.msim3_Tcw_opt).transpose() << std::endl;
        }
        std::cout << "-----------------------\n";
    }

    void LostManager::PrintGraphInfo()
    {
        std::cout << "------------Graph nodes------------" << std::endl;
        if (mGraphNodes.empty())
            std::cout << "No Graph Nodes!" <<std::endl;
        else
        {
            std::vector<std::pair<int, GraphNode>> vec_nodes(mGraphNodes.begin(), mGraphNodes.end());
            std::sort(vec_nodes.begin(), vec_nodes.end(), [](auto prev, auto next) -> bool
            {
                return prev.first < next.first;
            });
            for (const auto &node: vec_nodes)
            {
                assert(node.first == node.second.mId);
                std::cout << "id:" << node.first
                          << ", Q(w, x, y, z):"
                          << node.second.mSIM3_Tcw.unit_quaternion().w() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().x() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().y() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().z() << " "
                          << ", t:" << node.second.mSIM3_Tcw.translation().transpose() << ", fixed:" << node.second.mfixed << std::endl;
            }
        }

        std::cout << "------------Weak Edges------------" << std::endl;
        if (mWeakEdges.empty())
            std::cout << "No weak edges!" <<std::endl;
        else
        {
            for (const auto &wEdge: mWeakEdges)
            {
                std::cout << "id0:" << wEdge.mId0 << ", id1:" << wEdge.mId1
                        << ", Q(w, x, y, z):"
                        << wEdge.mSIM3_Tc1c0.unit_quaternion().w() << " "
                        << wEdge.mSIM3_Tc1c0.unit_quaternion().x() << " "
                        << wEdge.mSIM3_Tc1c0.unit_quaternion().y() << " "
                        << wEdge.mSIM3_Tc1c0.unit_quaternion().z() << " "
                        << ", t:" << wEdge.mSIM3_Tc1c0.translation().transpose() << std::endl;
            }
        }

        std::cout << "------------Normal Edges------------" << std::endl;
        if (mNormalEdges.empty())
            std::cout << "No normal edges!" <<std::endl;
        else
        {
            for (const auto &nEdge: mNormalEdges)
            {
                std::cout << "id0:" << nEdge.mId0 << ", id1:" << nEdge.mId1
                          << ", Q(w, x, y, z):"
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().w() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().x() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().y() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().z() << " "
                          << ", t:" << nEdge.mSIM3_Tc1c0.translation().transpose()
                          << ", s:" << nEdge.mScale << std::endl;
            }
        }

    }

    void LostManager::PringGraphShortInfo()
    {
        std::cout << "------------Graph nodes------------" << std::endl;
        if (mGraphNodes.empty())
            std::cout << "No Graph Nodes!" <<std::endl;
        else
        {
            std::vector<std::pair<int, GraphNode>> vec_nodes(mGraphNodes.begin(), mGraphNodes.end());
            std::sort(vec_nodes.begin(), vec_nodes.end(), [](auto prev, auto next) -> bool
            {
                return prev.first < next.first;
            });
            for (const auto &node: vec_nodes)
            {
                assert(node.first == node.second.mId);
                std::cout << node.first << " "
                          << node.second.mSIM3_Tcw.unit_quaternion().w() << " "
                          << node.second.mSIM3_Tcw.unit_quaternion().x() << " "
                          << node.second.mSIM3_Tcw.unit_quaternion().y() << " "
                          << node.second.mSIM3_Tcw.unit_quaternion().z() << " "
                          << node.second.mSIM3_Tcw.translation().transpose()  << std::endl;
            }
        }

        std::cout << "------------Weak Edges------------" << std::endl;
        if (mWeakEdges.empty())
            std::cout << "No weak edges!" <<std::endl;
        else
        {
            for (const auto &wEdge: mWeakEdges)
            {
                std::cout << wEdge.mId0 << " " << wEdge.mId1 << " "
                          << wEdge.mSIM3_Tc1c0.unit_quaternion().w() << " "
                          << wEdge.mSIM3_Tc1c0.unit_quaternion().x() << " "
                          << wEdge.mSIM3_Tc1c0.unit_quaternion().y() << " "
                          << wEdge.mSIM3_Tc1c0.unit_quaternion().z() << " "
                          << wEdge.mSIM3_Tc1c0.translation().transpose()
                          << " " << -1 << std::endl;
            }
        }

        std::cout << "------------Normal Edges------------" << std::endl;
        if (mNormalEdges.empty())
            std::cout << "No normal edges!" <<std::endl;
        else
        {
            for (const auto &nEdge: mNormalEdges)
            {
                std::cout <<  nEdge.mId0 << " " << nEdge.mId1 << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().w() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().x() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().y() << " "
                          << nEdge.mSIM3_Tc1c0.unit_quaternion().z() << " "
                          << nEdge.mSIM3_Tc1c0.translation().transpose() << " "
                          << nEdge.mScale << std::endl;
            }
        }
    }

    bool LostManager::findNode(int id)
    {
        return mGraphNodes.find(id) != mGraphNodes.end();
    }

    bool LostManager::setFixed(int id)
    {
        if (mGraphNodes.find(id) == mGraphNodes.end())
            return false;
        else
        {
            mGraphNodes[id].mfixed = true;
            return true;
        }
    }

    void LostManager::PrintNodes()
    {
        if (mGraphNodes.empty())
            std::cout << "No Graph Nodes!" <<std::endl;
        else
        {
            std::vector<std::pair<int, GraphNode>> vec_nodes(mGraphNodes.begin(), mGraphNodes.end());
            std::sort(vec_nodes.begin(), vec_nodes.end(), [](auto prev, auto next) -> bool
            {
                return prev.first < next.first;
            });
            for (const auto &node: vec_nodes)
            {
                assert(node.first == node.second.mId);
                std::cout << node.first << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().w() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().x() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().y() << " "
                        << node.second.mSIM3_Tcw.unit_quaternion().z() << " "
                        << " " << node.second.mSIM3_Tcw.translation().transpose() << std::endl; // id, Q(w, x, y, z), t(x, y, z)
            }
        }
    }

    g2o::Sim3 LostManager::getSimPose(int id)
    {
        return mGraphNodes[id].mScw;
    }

}

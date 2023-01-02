//
// Created by mpl on 22-12-23.
//
#include "Utils.h"

namespace ORB_SLAM3
{
    void Utils::plotFlow(cv::Mat image,
                         const std::vector<cv::Point2f>& vStart,
                         const std::vector<cv::Point2f>& vEnd,
                         std::string windowName)
    {
        cv::Mat imageMerged;
        std::vector<cv::Mat> imgArray {image, image, image};
        cv::merge(imgArray, imageMerged);

        int N = vStart.size();
        for (int i = 0; i < N; i++)
        {
            cv::Point2f start = vStart[i];
            cv::Point2f end = vEnd[i];

            cv::circle(imageMerged, start, 1, cv::Scalar(255, 0, 0));  //begin
            cv::circle(imageMerged, end, 1, cv::Scalar(0, 0, 255)); //end
            cv::line(imageMerged, start, end, cv::Scalar(0, 255, 0));
        }
        cv::imshow(windowName, imageMerged);
    }

    void Utils::plotMatches(const Frame& f1, const Frame& f2, std::vector<int> matches)
    {
        cv::Mat image1 = f1.imgLeft.clone();
        cv::Mat image2 = f2.imgLeft.clone();
        std::vector<cv::Point2f> keys1;
        std::vector<cv::Point2f> keys2;
        std::vector<cv::DMatch> cvMatches;
        for (int i = 0; i < matches.size(); i++)
        {
            if (matches[i] >= 0)
            {
                cv::DMatch match;
                match.queryIdx = i;
                match.trainIdx = matches[i];
                cvMatches.push_back(match);
            }
            // std::cout << i << "," <<  matches[i] << std::endl;

            // keys1.push_back(f1.mvKeys[i].pt);
            // keys2.push_back(f1.mvKeys[matches[i]].pt);
        }
        cv::Mat outImage;
        cv::drawMatches(image1, f1.mvKeysUn, image2, f2.mvKeysUn, cvMatches, outImage);

        cv::imshow("matching", outImage);
        cv::waitKey(0);
    }


//    bool checkDisparity(const Frame& f1, const Frame& f2,
//                        const vector<std::pair<int,int>>& vMatches12,
//                        const vector<bool>& vbMatchesInlier)
//    {
//        std::vector<float> all_disparity;
//
//        for(size_t i=0; i < vMatches12.size();i++)
//        {
//            if (!vbMatchesInlier[i])
//                continue;
//
//            const cv::KeyPoint &kp1 = f1.mvKeysUn[vMatches12[i].first];
//            const cv::KeyPoint &kp2 = f2.mvKeysUn[vMatches12[i].second];
//
//            float disparity = sqrt((kp1.pt.x - kp2.pt.x) * (kp1.pt.x - kp2.pt.x) +
//                                   (kp1.pt.y - kp2.pt.y) * (kp1.pt.y - kp2.pt.y));
////        std::cout << disparity << std::endl;
//            all_disparity.push_back(disparity);
//        }
//
//        if (all_disparity.size() != 0)
//        {
//            sort(all_disparity.begin(), all_disparity.end());
//            size_t index = std::min(100, int(all_disparity.size() / 2));
//            std::cout << "median disparity:" << all_disparity[index] << std::endl;
//            if (all_disparity[index] > 3)
//                return true;
//        }
//        return false;
//    }
//
//    bool CheckPureRotation(const Eigen::Matrix3f& K, const Sophus::SE3f& T21,
//                           const Frame& f1, const Frame& f2,
//                           const vector<std::pair<int,int>>& vMatches12, const vector<bool>& vbMatchesInlier)
//    {
//
//        ceres::Problem problem;
//        ceres::Solver::Options options;
//        Eigen::Matrix3f K_inv = K.inverse();
//
//        std::vector<std::pair<float, float>> homography_ratio_and_normBefore;
//        std::vector<float> all_diffs;
//        std::vector<float> diff_optimized;
//        std::vector<cv::Point2f> vf2Feature;
//        std::vector<cv::Point2f> vf1Feature;
//        std::vector<cv::Point2f> vf1Feature_in_f2;
//        std::vector<cv::Point2f> vf1Feature_in_f2_optimized;
//
//        Eigen::Matrix3f H_inf = K * T21.rotationMatrix() * K.inverse();
//
//        float inlier_num = 0.0;
//        float total_dist = 0.0;
//
//        for(size_t i=0; i < vMatches12.size();i++)
//        {
//            if (!vbMatchesInlier[i])
//                continue;
//
//            const cv::KeyPoint& kp1 = f1.mvKeysUn[vMatches12[i].first];
//            const cv::KeyPoint& kp2 = f2.mvKeysUn[vMatches12[i].second];
//
//            Eigen::Vector3f p1(kp1.pt.x, kp1.pt.y, 1);
//            Eigen::Vector3f p2(kp2.pt.x, kp2.pt.y, 1);
//
//            Eigen::Vector3f p2_projected = H_inf * p1;
//            p2_projected = p2_projected / p2_projected(2);
//
//            float dist = sqrt( (p2_projected(0) - p2(0)) * (p2_projected(0) - p2(0)) +
//                               (p2_projected(1) - p2(1)) * (p2_projected(1) - p2(1)) );
//
//
//            Eigen::Vector3f flow_before = p2 - p1;
//            Eigen::Vector3f flow_after = p2 - p2_projected;
//
//            float ratio = flow_before.norm() / flow_after.norm();
//            homography_ratio_and_normBefore.push_back(std::make_pair(ratio, flow_before.norm()));
////        std::cout << flow_after.normalized().transpose() << std::endl;
////        flow_before.dot(flow)
////        std::cout << vMatches12[i].first << " " << vMatches12[i].second << std::endl;
//
//            all_diffs.push_back(dist);
//            total_dist += dist;
//            inlier_num++;
//
//            vf1Feature.emplace_back(p1(0), p1(1));
//            vf2Feature.emplace_back(p2(0), p2(1));
//            vf1Feature_in_f2.emplace_back(p2_projected(0), p2_projected(1));
//        }
//
//        // ----------------------optimize q---------------
//        Eigen::Quaterniond optimized_q(T21.unit_quaternion().w(),T21.unit_quaternion().x(), T21.unit_quaternion().y(), T21.unit_quaternion().z());
//        for(size_t i=0; i < vMatches12.size();i++) {
//            if (!vbMatchesInlier[i])
//                continue;
//
//            const cv::KeyPoint &kp1 = f1.mvKeysUn[vMatches12[i].first];
//            const cv::KeyPoint &kp2 = f2.mvKeysUn[vMatches12[i].second];
//
//            Eigen::Vector3f p1(kp1.pt.x, kp1.pt.y, 1);
//            Eigen::Vector3f p2(kp2.pt.x, kp2.pt.y, 1);
//
//            Eigen::Vector3f bv1 = K_inv * p1;
//            Eigen::Vector3f bv2 = K_inv * p2;
//            bv1.normalize();
//            bv2.normalize();
//
//            ceres::CostFunction* costFunction =
//                    new ceres::AutoDiffCostFunction<PureRotationFunctor, 1, 4>(
//                            new PureRotationFunctor(bv1, bv2));
//            problem.AddResidualBlock(costFunction, new ceres::HuberLoss(1 - cos(atan(sqrt(2.0)*3/522.0))),
//                                     optimized_q.coeffs().data());
//        }
//
//
//        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//        options.minimizer_progress_to_stdout = false;
//        options.max_num_iterations = 100;
//        ceres::Solver::Summary summary;
//        ceres::Solve(options, &problem, &summary);
//        std::cout << summary.BriefReport() <<"\n";
//
//        Eigen::Quaternionf optimized_qf(optimized_q.w(), optimized_q.x(),optimized_q.y(),optimized_q.z());
//        Eigen::Matrix3f H_inf_optimzied = K * optimized_qf.toRotationMatrix() * K.inverse();
//
//
//        for(size_t i=0; i < vMatches12.size();i++)
//        {
//            if (!vbMatchesInlier[i])
//                continue;
//
//            const cv::KeyPoint &kp1 = f1.mvKeysUn[vMatches12[i].first];
//            const cv::KeyPoint &kp2 = f2.mvKeysUn[vMatches12[i].second];
//
//            Eigen::Vector3f p1(kp1.pt.x, kp1.pt.y, 1);
//            Eigen::Vector3f p2(kp2.pt.x, kp2.pt.y, 1);
//            Eigen::Vector3f p2_projected = H_inf_optimzied * p1;
//            p2_projected = p2_projected / p2_projected(2);
//
//            Eigen::Vector3f flow_optimized = p2 - p2_projected;
//
//            float dist = sqrt( (p2_projected(0) - p2(0)) * (p2_projected(0) - p2(0)) +
//                               (p2_projected(1) - p2(1)) * (p2_projected(1) - p2(1)) );
//            diff_optimized.push_back(dist);
//
//            vf1Feature_in_f2_optimized.emplace_back(p2_projected(0), p2_projected(1));
//
//        }
//        //TODO: 1.finish optimization and evaluation 2. adjust huber loss
//
//        // ----------------------optimize q---------------
//
//
//        if (all_diffs.size() > 0)
//        {
//            sort(all_diffs.begin(), all_diffs.end());
//            sort(homography_ratio_and_normBefore.begin(), homography_ratio_and_normBefore.end());
//            sort(diff_optimized.begin(), diff_optimized.end());
//            std::cout << "ratio, before norm: ";
//            std::cout << homography_ratio_and_normBefore[homography_ratio_and_normBefore.size() / 2].first << ","
//                      << homography_ratio_and_normBefore[homography_ratio_and_normBefore.size() / 2].second << std::endl;
//
////        std::cout << "*********\n";
////        for (int i = 0; i < all_diffs.size(); i++)
////            std::cout << all_diffs[i] << std::endl;
////        std::cout << "*********\n";
//            std::cout << "all_diffs.size():" << all_diffs.size() << std::endl;
//            std::cout << "H_inf:" << H_inf << std::endl;
//            std::cout << "median cost:" << all_diffs[all_diffs.size() / 2] << std::endl;
//            std::cout << "optimized median cost:" << diff_optimized[diff_optimized.size() / 2] << std::endl;
//            std::cout << "average cost:" << total_dist / inlier_num << std::endl;
//            plotFlow(f2, vf2Feature, vf1Feature, "OriginFlow");
//            plotFlow(f2, vf2Feature, vf1Feature_in_f2, "Homography flow");
//            plotFlow(f2, vf2Feature, vf1Feature_in_f2_optimized, "optimized homography flow");
//            cv::waitKey(0);
//
//            if (homography_ratio_and_normBefore[homography_ratio_and_normBefore.size() / 2].first > 4)
//                return true;
//        }
//
//        return false;
//    }
}

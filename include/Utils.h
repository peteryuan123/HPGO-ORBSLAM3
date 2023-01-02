//
// Created by mpl on 22-12-23.
//

#ifndef ORB_SLAM3_UTILS_H
#define ORB_SLAM3_UTILS_H

#include <opencv2/opencv.hpp>
#include "Frame.h"

namespace ORB_SLAM3
{
    class Utils
    {
    public:
        static void plotFlow(cv::Mat image, const std::vector<cv::Point2f>& vStart,
                      const std::vector<cv::Point2f>& vEnd, std::string windowName);
        
        static void plotMatches(const Frame& f1, const Frame& f2, std::vector<int> matches);

    };
}
#endif //ORB_SLAM3_UTILS_H

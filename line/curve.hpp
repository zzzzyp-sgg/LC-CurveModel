/**
 * @file   camera.hpp
 * @brief  This file defines curve model.
 * @todo   Processing for realistic images can be more complex.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_H
#define CURVE_H

#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <glog/logging.h>

class Curve
{
public:
    Curve(bool visualize = false) : visualize_(visualize) {};

    ~Curve() = default;

    /// @brief  detection
    void curveDetection(const cv::Mat &image)
    {
        curve_lines_.clear();
        cv::Mat imgPre;
        // convert to gray
        cv::cvtColor(image, imgPre, cv::COLOR_BGR2GRAY);

        // Gaussian blur
        cv::GaussianBlur(imgPre, imgPre, cv::Size(5, 5), 0, 0);

        // Canny edge detection
        cv::Canny(imgPre, imgPre, 50, 150, 3);
        if (visualize_)
        {
            cv::imshow("canny", imgPre);
            cv::waitKey(0);
        }

        // find contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgPre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        LOG(INFO) << "contours size: " << contours.size() << std::endl;
        if (visualize_)
        {
            cv::drawContours(imgPre, contours, -1, cv::Scalar(222, 244, 255), 2);
            cv::imshow("contours", imgPre);
            cv::waitKey(0);
        }

        // whether the contour is a curve
        for (int i = 0; i < contours.size(); i++)
        {
            LOG(INFO) << "contours[" << i + 1 << "].size(): " << contours[i].size() << std::endl;
            if (contours[i].size() > 100)
            {
                curve_lines_.push_back(contours[i]);
            }
        }

        LOG(WARNING) << "detect " << curve_lines_.size() << " curves." << std::endl;
    }

    /// @brief  curve judgment
    /// TODO: need to be improved
    bool isCurve([[maybe_unused]] const std::vector<cv::Point> &points, [[maybe_unused]] const cv::Vec4i &hierarchy)
    {
        return true;
    }

    /// @brief  get curve lines
    std::vector<std::vector<cv::Point>> getCurveLines()
    {
        return curve_lines_;
    }


private:
    std::vector<std::vector<cv::Point>> curve_lines_;
    bool visualize_;
};

#endif  // CURVE_H
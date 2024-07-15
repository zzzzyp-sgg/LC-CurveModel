/**
 * @file   curve_modeling.h
 * @brief  This file contains 3D-curve-line modeling.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef CURVE_MODELING_H
#define CURVE_MODELING_H

#include "camera.hpp"
#include "curve.hpp"
#include "output.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <execution>
#include <thread>

class CurveModeling
{
public:
    CurveModeling(const std::string &yaml_file);

    ~CurveModeling();

    /// @brief  load lidar points from file
    void loadLidarPoints(const std::string &lidar_points_file);

    /// @brief  load camera from file
    void loadCamera(const YAML::Node &yaml, const std::string &yaml_file);

    /// @brief  load lidar2camera extrinsic Parameters
    void loadLidar2CameraExtrinsic(const YAML::Node &yaml);
    
    /// @brief  lidar preprocessing
    void lidarPreprocessing();
    
    /// @brief  fitting 3D-curve-line with input lidar points
    bool curveLidarFitting();

    /// @brief  residual testing
    bool lineResidualTesting();

    /// @brief  residual testing(image curve fitting)
    bool lineResidualTesting(const std::vector<cv::Point> &points, const Eigen::VectorXd &curve_param);

    /// @brief  generate points based on curve parameters
    void generateCurvePoints();

    /// @brief  image curve detection
    void curveImageDetection(bool visualize = false);

    /// @brief  fitting 2D-curve-line with image curve points
    Eigen::VectorXd curveImageFitting(const std::vector<cv::Point> &points);

    /// @brief  for debug visualization
    void visualization();

    /// @brief  project 3D points to image
    void project3DPointsToImage(const std::vector<Eigen::Vector3d> &points);

    /// @brief  optimization 3D-curve-points
    void optimization3DPoints();

private:
    std::vector<Eigen::Vector3d> lidar_points_;
    std::shared_ptr<Camera> cam_;
    Eigen::Vector3d end_point_;
    cv::Mat img_;
    Eigen::Matrix3d R_c_l_;
    Eigen::Vector3d t_c_l_;
    int degree_;

    double plane_param_[1][2];
    double mesh_param_[1][3];

    std::vector<Eigen::Vector3d> curve_points_;
    std::vector<std::vector<cv::Point>> curve_lines_;
    Eigen::VectorXd curve_param_;
};

#endif
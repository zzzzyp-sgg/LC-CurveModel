/**
 * @file   image_simulation.hpp
 * @brief  This file is used to generate image based.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#ifndef IMAGE_SIMULATION_H
#define IMAGE_SIMULATION_H

#include <Eigen/Dense>
#include <vector>

class ImageSimulation
{
public:
    ImageSimulation(const std::vector<Eigen::Vector3d> &points) : points_(points) 
    {

    };
    ~ImageSimulation();

private:
    int image_width_;
    int image_height_;
    std::vector<Eigen::Vector3d> points_;
};

# endif // IMAGE_SIMULATION_H
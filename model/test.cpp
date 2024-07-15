/**
 * @file   test.cpp
 * @brief  Test for model.
 * @author Yipeng Zhao
 * @date   2024-07
 */

#include "curve_modeling.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

DEFINE_string(yaml_file, "/home/zyp/Lidar/LC-CurveModel/config/model.yaml", "yaml文件路径");
DEFINE_bool(visualize, false, "是否可视化");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;

    std::string yaml_file = FLAGS_yaml_file;

    auto curve_modeling = std::make_shared<CurveModeling>(yaml_file);
    curve_modeling->curveImageDetection(FLAGS_visualize);

    curve_modeling->lidarPreprocessing();

    curve_modeling->optimization3DPoints();

    google::ShutdownGoogleLogging();
    return 0;
}
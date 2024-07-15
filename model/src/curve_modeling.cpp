// TODO: case of multiple curves
#include "curve_modeling.h"
#include "curve_factor.h"
#include "curve_factor_auto.h"

#include <fstream>
#include <glog/logging.h>
#include <ceres/ceres.h>
#include <chrono>

CurveModeling::CurveModeling(const std::string &yaml_file)
{
    YAML::Node yaml = YAML::LoadFile(yaml_file);

    // load lidar measurement
    std::string lidar_points_file = yaml["lidar_points_file"].as<std::string>();
    loadLidarPoints(lidar_points_file);

    // load camera
    loadCamera(yaml, yaml_file);

    // load end point
    if (yaml["end_point"])
    {
        end_point_ << yaml["end_point"][0].as<double>(), yaml["end_point"][1].as<double>(), yaml["end_point"][2].as<double>();
    }
    else
    {
        LOG(ERROR) << "No end point found in yaml file.\n";
        return;
    }

    // load image
    if (yaml["image_path"])
    {
        std::string image_file = yaml["image_path"].as<std::string>();
        img_ = cv::imread(image_file, cv::IMREAD_COLOR);
    }
    else
    {
        LOG(ERROR) << "No image file found in yaml file.\n";
        return;
    }

    // load image curve degree
    degree_ = yaml["curve_degree"].as<int>();

    // load lidar-camera extrinsic
    loadLidar2CameraExtrinsic(yaml);
}

CurveModeling::~CurveModeling()
{
    lidar_points_.clear();
    cam_.reset();
}

void CurveModeling::loadLidarPoints(const std::string &lidar_points_file)
{

    std::fstream input(lidar_points_file, std::ios::in);
    if (!input)
    {
        LOG(ERROR) << "Cannot open file: " << lidar_points_file << "\n";
        return;
    }

    std::string line;
    while (std::getline(input, line))
    {
        std::istringstream iss(line);
        Eigen::Vector3d point;
        iss >> point(0) >> point(1) >> point(2);
        lidar_points_.push_back(point);
    }
}

void CurveModeling::loadCamera(const YAML::Node &yaml, const std::string &yaml_file)
{
    if (yaml["cam_calib"])
    {
        std::string camera_file;
        int pn = yaml_file.find_last_of("/");
        camera_file = yaml_file.substr(0, pn + 1) + yaml["cam_calib"].as<std::string>();
        cam_ = std::make_shared<Camera>();
        cam_->readIntrinsicParameters(camera_file);
    }
    else
    {
        LOG(ERROR) << "No camera file found in yaml file.\n";
        return;
    }
}

void CurveModeling::loadLidar2CameraExtrinsic(const YAML::Node &yaml)
{
    if (!yaml["T_cam_lidar"])
    {
        LOG(ERROR) << "No extrinsic parameters found in yaml file.\n";
        return;
    }

    std::vector<double> vecT = yaml["T_cam_lidar"]["data"].as<std::vector<double>>();
    assert(vecT.size() == 16);

    Eigen::Matrix4d T = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(vecT.data());
    R_c_l_ = T.block<3, 3>(0, 0);
    t_c_l_ = T.block<3, 1>(0, 3);
}

void CurveModeling::lidarPreprocessing()
{
    // fit curve
    curveLidarFitting();

    // generate curve points
    generateCurvePoints();
}

bool CurveModeling::lineResidualTesting()
{
    auto point_error = [](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) -> double
    {
        return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
    };

    double sum_err = 0;
    int big_num = 0;
    std::vector<double> errors;
    for (const Eigen::Vector3d &p : lidar_points_)
    {
        Eigen::Vector3d line_point(p.x(), 0, 0);
        line_point.y() = plane_param_[0][0] * line_point.x() + plane_param_[0][1];
        line_point.z() = mesh_param_[0][0] * pow(line_point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];

        double error = point_error(p, line_point);
        if (error > 0.05)
        {
            big_num++;
        }
        errors.push_back(error);
    }

    double avg_err = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();

    Eigen::Vector3d line_end_point(end_point_.x(), 0, 0);
    line_end_point.y() = plane_param_[0][0] * line_end_point.x() + plane_param_[0][1];
    line_end_point.z() = mesh_param_[0][0] * pow(line_end_point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];
    double end_point_error = point_error(end_point_, line_end_point);

    if (big_num > 20 || avg_err > 0.05 || end_point_error > 0.03)
    {
        LOG(WARNING) << "Average error: " << avg_err << "\n";
        LOG(WARNING) << "Big error number: " << big_num << "\n";
        LOG(WARNING) << "End point error: " << end_point_error << "\n";
        return false;
    }

    return true;
}

bool CurveModeling::lineResidualTesting(const std::vector<cv::Point> &points, const Eigen::VectorXd &curve_param)
{    
    auto point_error = [](const cv::Point &p1, const cv::Point &p2) -> double
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    };

    double sum_err = 0;
    int big_error_num = 0;
    std::vector<double> errors;
    for (const cv::Point &p : points)
    {
        double y = 0;
        for (int i = 0; i <= degree_; i++)
        {
            y += curve_param(i) * pow(p.x, i);
        }

        double error = point_error(p, cv::Point(p.x, y));
        errors.push_back(error);

        if (error > 3)
        {
            big_error_num++;
        }
    }

    double avg_err = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
    LOG(WARNING) << "Average error: " << avg_err << "\n";
    LOG(WARNING) << "Big error number: " << big_error_num << "\n";

    return true;
}

bool CurveModeling::curveLidarFitting()
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    // initial value for better optimization
    Eigen::Vector3d start_point = lidar_points_[0];
    plane_param_[0][0] = (start_point.y() - end_point_.y()) / (start_point.x() - end_point_.x());
    plane_param_[0][1] = end_point_.y() - plane_param_[0][0] * end_point_.x();

    mesh_param_[0][1] = 0.5 * (start_point.x() + end_point_.x());
    mesh_param_[0][0] = (start_point.z() - lidar_points_.back().z()) /
                        (pow(start_point.x() - mesh_param_[0][1], 2) - pow(lidar_points_.back().x() - mesh_param_[0][1], 2));
    mesh_param_[0][2] = end_point_.z() - mesh_param_[0][0] * pow(end_point_.x() - mesh_param_[0][1], 2);

    problem.AddParameterBlock(plane_param_[0], 2);
    problem.AddParameterBlock(mesh_param_[0], 3);

#ifdef MAUNAL_DIFF   // manual diff
    for (const Eigen::Vector3d &p : lidar_points_)
    {
        CurveFactor *curve_factor = new CurveFactor(p);
        problem.AddResidualBlock(curve_factor, loss_function, plane_param_[0], mesh_param_[0]);
    }

    CurveFactor *curve_factor = new CurveFactor(end_point_, 1e2);
    problem.AddResidualBlock(curve_factor, loss_function, plane_param_[0], mesh_param_[0]);
#else                // auto diff
    for (const Eigen::Vector3d &p : lidar_points_)
    {
        ceres::CostFunction *cost_function = CurveFactorAuto::Create(p);
        problem.AddResidualBlock(cost_function, loss_function, plane_param_[0], mesh_param_[0]);
    }

    ceres::CostFunction *cost_function = CurveFactorAuto::Create(end_point_, 1e2);
    problem.AddResidualBlock(cost_function, loss_function, plane_param_[0], mesh_param_[0]);

#endif
    start = std::chrono::system_clock::now();
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    end = std::chrono::system_clock::now();
    auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    LOG(INFO) << "Optimization time: " << cost.count() << "ms\n";

    return lineResidualTesting();
}

void CurveModeling::generateCurvePoints()
{
    std::sort(lidar_points_.begin(), lidar_points_.end(), [](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) -> bool
              { return p1.x() < p2.x(); });

    int lidar_points_num = (int)lidar_points_.size();
    int num = lidar_points_num * (end_point_.x() - lidar_points_[lidar_points_num - 1].x())
            / (lidar_points_[lidar_points_num - 1].x() - lidar_points_[0].x());

    double step = (end_point_.x() - lidar_points_[lidar_points_num - 1].x()) / num;

    curve_points_.clear();
    for (int i = 1; i <= num; i++)
    {
        Eigen::Vector3d point;
        point.x() = lidar_points_[lidar_points_num - 1].x() + i * step;
        point.y() = plane_param_[0][0] * point.x() + plane_param_[0][1];
        point.z() = mesh_param_[0][0] * pow(point.x() - mesh_param_[0][1], 2) + mesh_param_[0][2];
        curve_points_.push_back(point);
    }

    outputPoints("curve_points.txt", curve_points_);
}

void CurveModeling::curveImageDetection(bool visualize)
{
    cv::Mat img = img_.clone();
    
    std::shared_ptr<Curve> curve = std::make_shared<Curve>();
    curve->curveDetection(img);

    curve_lines_ = curve->getCurveLines();

    curve_param_ = curveImageFitting(curve_lines_[0]);

    LOG(INFO) << "Curve parameters: " << curve_param_.transpose() << "\n";

    lineResidualTesting(curve_lines_[0], curve_param_);

    if (visualize)  visualization();
}

Eigen::VectorXd CurveModeling::curveImageFitting(const std::vector<cv::Point> &points)
{
    int n = (int)points.size();
    // degree 3 polynomial
    Eigen::MatrixXd A(n, degree_ + 1);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j <= degree_; j++)
        {
            A(i, j) = pow(points[i].x, j);
        }
    }

    Eigen::VectorXd y(n);
    for (int i = 0; i < n; i++)
    {
        y(i) = points[i].y;
    }

    Eigen::VectorXd result = A.householderQr().solve(y);

    return result;
}

void CurveModeling::visualization()
{
    cv::Mat img = img_.clone();

    for (const cv::Point &p : curve_lines_[0])
    {
        double y = 0;
        for (int i = 0; i <= degree_; i++)
        {
            y += curve_param_(i) * pow(p.x, i);
        }

        y = (int)y;
        if (y < 0 || y >= img.rows)
        {
            continue;
        }

        img.at<cv::Vec3b>(cv::Point(p.x, y)) = cv::Vec3b(0, 0, 255);
    }

    cv::imwrite(std::to_string(degree_) + "-degree.jpg", img);

    cv::imshow("curve points", img);
    cv::waitKey(0);
}

void CurveModeling::project3DPointsToImage(const std::vector<Eigen::Vector3d> &points)
{
    std::vector<cv::Point2f> image_points;
    for (const Eigen::Vector3d &p : points)
    {
        Eigen::Vector3d point = R_c_l_ * p + t_c_l_;
        Eigen::Vector2d image_point;
        cam_->spaceToPlane(point, image_point);
        if (image_point.y() < 0 || image_point.y() > cam_->img_h_ || image_point.x() < 0 || image_point.x() > cam_->img_w_)
        {
            continue;
        }
        image_points.push_back(cv::Point2f(image_point.x(), image_point.y()));
    }

    // error(distance to curve equation)
    double sum_err = 0;
    for (const cv::Point2f &p : image_points)
    {
        double y = 0;
        for (int i = 0; i <= degree_; i++)
        {
            y += curve_param_(i) * pow(p.x, i);
        }

        sum_err += sqrt(pow(p.y - y, 2));
    }

    double avg_err = sum_err / image_points.size();

    LOG(INFO) << "Average error: " << avg_err << "\n";
}

void CurveModeling::optimization3DPoints()
{
    std::for_each(std::execution::par_unseq, curve_points_.begin(), curve_points_.end(), [this](Eigen::Vector3d &p) {
        double min_err = 1e6;
        double dy = 0.0;
        double dz = 0.0;
        for (double delta_y = -0.03; delta_y <= 0.03; delta_y += 0.005)
        {
            for (double delta_z = -0.03; delta_z <= 0.03; delta_z += 0.005)
            {
                Eigen::Vector3d point_d(p.x(), p.y() + delta_y, p.z() + delta_z);
                Eigen::Vector3d point = R_c_l_ * point_d + t_c_l_;

                Eigen::Vector2d img_p;
                cam_->spaceToPlane(point, img_p);

                double y = 0;
                for (int i = 0; i <= degree_; i++)
                {
                    y += curve_param_(i) * pow(img_p.x(), i);
                }

                double err = sqrt(pow(img_p.y() - y, 2));

                if (err < min_err)
                {
                    min_err = err;
                    dy = delta_y;
                    dz = delta_z;
                }
            }
        }

        p.y() += dy;
        p.z() += dz;
    });

    project3DPointsToImage(curve_points_);

    outputPoints("curve_points_1.txt", curve_points_);
}
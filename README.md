# LC-CurveModel: Joint camera-lidar spatial curve modeling
<!-- markdownlint-disable MD047 -->
This is a simulation library for Lidar point cloud and image curve fitting, the code realizes the simulation of laser point cloud, the simulation of corresponding projected image and the curve fitting process in Lidar coordinate system and image coordinate system.  

## Dependencies

- Eigen(3.4.0 or other version)
- OpenCV(4.2.0 or other version)
- Ceres(2.1.0 or other version)
- GLOG/GFLAGS
- yaml-cpp

## Installation

1. Install the required dependencies.

2. Clone the repository:

    ```bash
    git clone https://github.com/zzzzyp-sgg/LC-CurveModel.git
    ```

3. Build the program:

    ```bash
    mkdir build && cd build
    cmake ..
    make -j8
    ```

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).

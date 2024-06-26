#include "lidar_camera_calibration.hpp"

LidarCameraCalibration::LidarCameraCalibration() : Node("lidar_camera_calibration")
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1563.52174,        0.0, 626.90356, 
                                                              0.0, 1568.90028, 488.93524,
                                                              0.0,        0.0,       1.0);

    cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.063200, -0.005061, -0.001755, 0.003472, 0.0);

    std::vector<cv::Point3d> lidar_points;
    std::vector<cv::Point2d> image_points;
    lidar_points = {cv::Point3d(3.263000, 0.113000, -0.666000), cv::Point3d(3.375000, 0.054000, -0.20200), cv::Point3d(3.136000, 0.015000, 0.154000),
                    cv::Point3d(3.089000, -0.357000, 0.124000), cv::Point3d(3.335000, -0.415000, -0.242000),cv::Point3d(3.225000, -0.433000, -0.695000)};
    /*image_points = {cv::Point2d(223,368), cv::Point2d(231,261), cv::Point2d(238,185),
                    cv::Point2d(334,187), cv::Point2d(340,269),cv::Point2d(342,371)};*/
    image_points = {cv::Point2d(223 * 2.0,368 * 2.133333333333), cv::Point2d(231 * 2.0 ,261 * 2.13333333333333333), cv::Point2d(238 * 2.0,185 * 2.1333333333333333),
                    cv::Point2d(334 * 2.0,187 * 2.133333333333), cv::Point2d(340 * 2.0,269 * 2.1333333333333333),cv::Point2d(342 * 2.0,371 * 2.13333333333333333)};

    for(int i =0;i<6;i++)
    {
        std::cout << "Lidar Point: " << lidar_points[i] << std::endl;
        std::cout << "Image Point: " << image_points[i] << std::endl;
    }

    cv::Mat rvec, tvec;

    cv::solvePnP(lidar_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    std::cout << "camera_matrix: " << camera_matrix << std::endl;
    std::cout << "dist_coeffs: " << dist_coeffs << std::endl;
    std::cout << "R: " << R << std::endl;
    std::cout << "t: " << tvec << std::endl;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarCameraCalibration>());
    rclcpp::shutdown();
    return 0;
}
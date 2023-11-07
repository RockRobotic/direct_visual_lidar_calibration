#include <vlcal/preprocess/generate_lidar_image.hpp>

#include <vlcal/common/estimate_fov.hpp>

namespace vlcal
{

std::pair<cv::Mat, cv::Mat> generate_lidar_image(const camera::GenericCameraBase::ConstPtr &proj,
                                                 const Eigen::Vector2i &image_size,
                                                 const Eigen::Isometry3d &T_camera_lidar, const Frame::ConstPtr &points)
{
    //
    const double camera_fov = estimate_camera_fov(proj, image_size);
    const double min_z = std::cos(camera_fov);
    std::cout << "fov: " << camera_fov << std::endl;
    std::cout << "minz: " << min_z << std::endl;

    cv::Mat sq_dist_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(std::numeric_limits<double>::max()));
    cv::Mat intensity_image(image_size[1], image_size[0], CV_64FC1, cv::Scalar::all(0));
    cv::Mat index_image(image_size[1], image_size[0], CV_32SC1, cv::Scalar::all(-1));

    for (int i = 0; i < points->size(); i++)
    {
        const auto &pt_lidar = points->points[i];
        const Eigen::Vector4d pt_camera = T_camera_lidar * pt_lidar;

        if (i == 0)
            std::cout << "pt: " << pt_camera.head<3>().normalized() << std::endl;
        // if (pt_camera.head<3>().normalized().z() < min_z) {
        //   // std::cout << "1" << std::endl;
        //   continue;
        // }

        const Eigen::Vector2i pt_2d = proj->project(pt_camera.head<3>()).cast<int>();
        if ((pt_2d.array() < Eigen::Array2i::Zero()).any() || (pt_2d.array() >= image_size.array()).any())
        {
            // std::cout << "2" << std::endl;
            continue;
        }

        const double sq_dist = pt_camera.head<3>().squaredNorm();
        if (sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) < sq_dist)
        {
            // std::cout << "3" << std::endl;
            continue;
        }

        sq_dist_image.at<double>(pt_2d.y(), pt_2d.x()) = sq_dist;
        intensity_image.at<double>(pt_2d.y(), pt_2d.x()) = points->intensities[i];
        index_image.at<std::int32_t>(pt_2d.y(), pt_2d.x()) = i;
    }

    return std::make_pair(intensity_image, index_image);
}

} // namespace vlcal

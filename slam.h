//
// Created by indemind on 2023/6/20.
//

#ifndef SHOWTOF_SLAM_H
#define SHOWTOF_SLAM_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace psl {
    using Time = std::uint64_t;  // us, microseconds
}

namespace psl
{
    struct SlamResult
    {
        psl::Time s_time;
        float s_rotation[4];  // w, x, y, z
        float s_position[3];
        float delt_rotation[4];  // w, x, y, z
        float delt_position[3];
        float home_rotation[4];  // w, x, y, z
        float home_position[3];
        float s_state;
        bool relocal_success;
        bool closure_success;
        bool update_home;
        bool normal;
    };
}

template<class T>
Eigen::Matrix<T, 3, 1> Quaternion2Angle(const Eigen::Quaternion<T> &quaternion);

Eigen::Quaternion<double> Pose2Quaternion(const psl::SlamResult &pose);

template<class T>
Eigen::Matrix<T, 3, 1> Quaternion2Angle(const Eigen::Quaternion<T> &quaternion)
{
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();

    if (normalized_quaternion.w() < 0.)
    {
        // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
        normalized_quaternion.w() = -1. * normalized_quaternion.w();
        normalized_quaternion.x() = -1. * normalized_quaternion.x();
        normalized_quaternion.y() = -1. * normalized_quaternion.y();
        normalized_quaternion.z() = -1. * normalized_quaternion.z();
    }

    const T angle = 2. * atan2(normalized_quaternion.vec().norm()
            , normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7;
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / 2.);
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(), scale * normalized_quaternion.z());
}

#endif //SHOWTOF_SLAM_H

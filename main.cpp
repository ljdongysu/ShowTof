#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <cmath>
#include "slam.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct TofDepthPoint {
    float X;
    float Y;
    float Z;
    float noise;  // HFF
    uint16_t grayValue;
    uint8_t depthConfidence;
};

struct TofDepthData {

    TofDepthPoint data[224*172];
};

void ReadArray(const YAML::Node &config, std::vector<float> &array)
{
    try
    {
        array = config.as<std::vector<float>>();
    }
    catch (...)
    {
        for (YAML::const_iterator it = config.begin(); it != config.end(); ++it)
        {
            array.push_back((*it).as<float>());
        }
    }
}

bool GetTof(std::string yamlFile, TofDepthData &tof)
{
    try
    {
        YAML::Node config;

        if (not access(yamlFile.c_str(), 0) == 0)
        {
            std::cout << "file not exist <" + yamlFile + ">" << std::endl;
        }
        config = YAML::LoadFile(yamlFile);

        static std::vector<float> data(172 * 224 * 6);

        ReadArray(config["data"], data);

        int p = 0;
        for (int i = 0; i < 224 * 172; i++)
        {

            tof.data[i].X = data[p++];
            if (tof.data[i].X > 15 or tof.data[i].X < -15) tof.data[i].X = 0;
            tof.data[i].Y = data[p++];
            if (tof.data[i].Y > 15 or tof.data[i].Y < -15) tof.data[i].Y = 0;
            tof.data[i].Z = data[p++];
            if (tof.data[i].Z > 15 or tof.data[i].Z < -15) tof.data[i].Z = 0;
            tof.data[i].noise = data[p++];
            tof.data[i].grayValue = data[p++];
            tof.data[i].depthConfidence = data[p++];
        }

        return true;
    }
    catch (...)
    {
        return false;
    }
}

void ConvertTof2PCL(TofDepthData &tof, PointCloud::Ptr cloud)
{
    // test rotation by Eigen
    std::vector<Eigen::Vector3d> points;
    float x = -1;

//    for (const auto &t : tof.data)
    for (int i = 0; i < sizeof(tof.data) / sizeof(tof.data[0]); ++i)
    {
        const auto t = tof.data[i];
        float thea = 0.279;
        PointT p;
        float tempX = -t.X;
        float tempY = -t.Y;
        float tempZ = t.Z;

        p.x = tempX;
//        p.y = tempY;
//        p.z = tempZ;



        p.y = (tempY * cos(thea) - t.Z * sin(thea));
        p.z = (t.Z * cos(thea) + tempY * sin(thea));
//
        if( p.y >0 && p.y < 1 )
        {
            p.r = 255;
            if(p.x < -0.13664 && p.x > -0.13665 )
            {
                p.r = 0;
                p.g = 255;
            }
            cloud->points.push_back(p);

            std::cout << "x: " << p.x << ", p.y: " << p.y << "p.z: " << p.z << std::endl;
//            std::cout << "Eiichi: " << i << std::endl;
            if (x < p.x)
            {
                x = p.x;
                std::cout << "===x: " << p.x << ", p.y: " << p.y << "p.z: " << p.z << std::endl;
            }
        }

    }
}
const float NNN = 100;
float Nan_Replace(const float value)
{
    if (std::isnan(value))
    {
        return NNN;
    }
    else
    {
        return value;
    }
}

bool GetPose(std::string yamlFile, psl::SlamResult &pose)
{
    try
    {
        YAML::Node config;

        if (not access(yamlFile.c_str(), 0) == 0)
        {
            std::cout << "file not exist <" + yamlFile + ">" << std::endl;
        }
        config = YAML::LoadFile(yamlFile);

        pose.s_time = config["time"].as<unsigned long>();
        pose.s_state = config["state"].as<float>();

        try
        {
            pose.normal = config["normal"].as<bool>();
        }
        catch (YAML::TypedBadConversion<bool> &e)
        {
            pose.normal = true;
        }

        size_t size = 0;
        std::vector<double> position = config["position"].as<std::vector<double>>();
        size = position.size();
        for (size_t i = 0; i < size; ++i)
        {
            pose.s_position[i] = position[i];
        }

        std::vector<double> rotation = config["rotation"].as<std::vector<double>>();
        size = rotation.size();
        for (size_t i = 0; i < size; ++i)
        {
            pose.s_rotation[i] = rotation[i];
        }

        return true;
    }
    catch (...)
    {
        return false;
    }
}

void GetRotationAngle(float &thea1, float &thea2, float &thea3, const psl::SlamResult &pose)
{
    float q0 = pose.s_rotation[0];
    float q1 = pose.s_rotation[1];
    float q2 = pose.s_rotation[2];
    float q3 = pose.s_rotation[2];
    thea1 = atan2(2*(q0*q1+q2*q3),(1-2*(q1*q1+q2*q2)));
    thea2 = asin(2*(q0*q2 - q1*q3));
    thea3 = atan2(2*(q0*q3+q1*q2),(1-2*(q2*q2+q3*q3)));
}

Eigen::Quaternion<double> Pose2Quaternion(const psl::SlamResult& pose)
{
    Eigen::Quaternion<double> quaternion;

    quaternion.w() = pose.s_rotation[0];
    quaternion.x() = pose.s_rotation[1];
    quaternion.y() = pose.s_rotation[2];
    quaternion.z() = pose.s_rotation[3];

    return quaternion;
}

void GetRotationAngle(float &thea1, float &thea2, float &thea3, const psl::SlamResult &poseLast, const psl::SlamResult &poseCurrent)
{
    Eigen::Quaternion<double> last = Pose2Quaternion(poseLast);
    Eigen::Quaternion<double> current = Pose2Quaternion(poseCurrent);
    Eigen::Quaternion<double> quaternion = last.inverse() * current;
    Eigen::Matrix<double, 3, 1> radian = Quaternion2Angle(quaternion);

//    auto angle = radian * RADIAN_2_ANGLE;
    thea1 = radian[0];
    thea2 = radian[1];
    thea3 = radian[2];
}

int main(int argc, char ** argv)
{

    std::string tofFile = argv[1];
    PointCloud::Ptr cloud(new PointCloud);
    TofDepthData tof;
    psl::SlamResult poseLast;
    psl::SlamResult poseCurrent;

    float a,b,c,d;
    if (not GetTof(tofFile, tof)) return 0;
    if (argc > 2)
    {
        std::string poseFile = argv[2];
        if (not GetPose(poseFile, poseLast)) return 0;

//        GetRotationAngle(a,b,c,poseLast);
    }

    if (argc > 3)
    {
        std::string poseFile = argv[3];
        if (not GetPose(poseFile, poseCurrent)) return 0;

//        GetRotationAngle(a,b,c,poseLast);
        GetRotationAngle(a,b,c,poseLast,poseCurrent);
    }
//    poseLast.s_rotation[0] = 1;
//    poseLast.s_rotation[1] = 0;
//
//    poseLast.s_rotation[2] = 0;
//
//    poseLast.s_rotation[3] = 0;



    ConvertTof2PCL(tof, cloud);

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << "point cloud size = " << cloud->points.size() << endl;
    cloud->is_dense = false;

    //显示点云图像
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()){ };

    std::cout << "Hello, World!" << std::endl;
    return 0;
}

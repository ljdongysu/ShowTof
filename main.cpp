#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>

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
            tof.data[i].Y = data[p++];
            tof.data[i].Z = data[p++];
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
    float maxZ = -1;
    float minZ = 1;
    float maxX = -1;
    float minX = 1;
    float maxY = -1;
    float minY = 1;
    for (const auto &t : tof.data)
    {
        PointT p;
        p.x = t.X;
        p.y = t.Y;
        p.z = t.Z;
        cloud->points.push_back(p);
//        if (t.X > maxX)
//        {
//            maxX = t.X;
//        }
//        if(t.X < minX)
//        {
//            minX = t.X;
//        }
//        if (t.Y > maxY)
//        {
//            maxY = t.Y;
//        }
//        if(t.Y < minY)
//        {
//            minY = t.Y;
//        }
//        if (t.Z > maxZ)
//        {
//            maxZ = t.Z;
//        }
//        if(t.Z < minZ)
//        {
//            minZ = t.Z;
//        }
    }
//
//    std::cout << "X: " << minX << "~" << maxX <<"  ,  Y: " << minY << "~" << maxY
//    << " , minZ:" << minZ << "~" << maxZ<<std::endl;
}

int main(int argc, char ** argv)
{

    std::string tofFile = argv[1];
    PointCloud::Ptr cloud(new PointCloud);
    TofDepthData tof;

    GetTof(tofFile, tof);

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

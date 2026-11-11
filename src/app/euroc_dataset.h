// euroc_dataset.h -- EuRoC MAV (ASL format) reader for the ROS-free VINS-Mono driver.
// Expects <root>/cam0/data.csv + cam0/data/*.png and <root>/imu0/data.csv (root = "mav0" folder or the folder that
// directly contains cam0/imu0).
#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Dense>

struct ImuSample
{
    int64_t t_ns;
    Eigen::Vector3d gyr; // rad/s
    Eigen::Vector3d acc; // m/s^2
};

struct ImageEntry
{
    int64_t t_ns;
    std::string file; // file name inside the image folder
};

class EurocDataset
{
  public:
    bool open(const std::string &root, std::string &err);
    bool loadImu(std::string &err);
    bool listImages(std::string &err); // from cam0/data.csv (macOS "._*" files in cam0/data are ignored)

    std::string root, imageDir, imuFile, camCsv, gtFile;
    std::vector<ImuSample> imu;
    std::vector<ImageEntry> images;
};

std::string joinPath(const std::string &a, const std::string &b);
bool fileExists(const std::string &p);
bool dirExists(const std::string &p);

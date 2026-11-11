#include "euroc_dataset.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

std::string joinPath(const std::string &a, const std::string &b) { return (fs::u8path(a) / fs::u8path(b)).u8string(); }
bool fileExists(const std::string &p)
{
    std::error_code ec;
    return fs::is_regular_file(fs::u8path(p), ec);
}
bool dirExists(const std::string &p)
{
    std::error_code ec;
    return fs::is_directory(fs::u8path(p), ec);
}

bool EurocDataset::open(const std::string &r, std::string &err)
{
    for (std::string c : {r, joinPath(r, "mav0")})
    {
        if (fileExists(joinPath(joinPath(c, "cam0"), "data.csv")) && fileExists(joinPath(joinPath(c, "imu0"), "data.csv")))
        {
            root = c;
            imageDir = joinPath(joinPath(c, "cam0"), "data");
            camCsv = joinPath(joinPath(c, "cam0"), "data.csv");
            imuFile = joinPath(joinPath(c, "imu0"), "data.csv");
            gtFile = joinPath(joinPath(c, "state_groundtruth_estimate0"), "data.csv");
            return true;
        }
    }
    err = "EuRoC folder " + r + " does not contain cam0/data.csv and imu0/data.csv (nor mav0/...)";
    return false;
}

bool EurocDataset::loadImu(std::string &err)
{
    std::ifstream in(fs::u8path(imuFile));
    if (!in)
    {
        err = "cannot open " + imuFile;
        return false;
    }
    std::string line;
    imu.clear();
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        for (char &ch : line)
            if (ch == ',')
                ch = ' ';
        std::istringstream ss(line);
        long long t;
        double w[3], a[3];
        if (!(ss >> t >> w[0] >> w[1] >> w[2] >> a[0] >> a[1] >> a[2]))
            continue;
        if (!imu.empty() && t <= imu.back().t_ns)
            continue;
        imu.push_back({(int64_t)t, Eigen::Vector3d(w[0], w[1], w[2]), Eigen::Vector3d(a[0], a[1], a[2])});
    }
    if (imu.size() < 100)
    {
        err = "too few IMU rows in " + imuFile;
        return false;
    }
    return true;
}

bool EurocDataset::listImages(std::string &err)
{
    std::ifstream in(fs::u8path(camCsv));
    if (!in)
    {
        err = "cannot open " + camCsv;
        return false;
    }
    std::string line;
    images.clear();
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        auto p = line.find(',');
        if (p == std::string::npos)
            continue;
        std::string name = line.substr(p + 1);
        while (!name.empty() && (name.back() == '\r' || name.back() == ' '))
            name.pop_back();
        while (!name.empty() && name.front() == ' ')
            name.erase(name.begin());
        images.push_back({(int64_t)std::strtoll(line.c_str(), nullptr, 10), name});
    }
    std::sort(images.begin(), images.end(), [](const ImageEntry &a, const ImageEntry &b) { return a.t_ns < b.t_ns; });
    if (images.empty())
    {
        err = "no images listed in " + camCsv;
        return false;
    }
    return true;
}

#pragma once
#include "ros_compat.h"

#include <vector>
#include <Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

// VINS-Mono uses a fixed "virtual" focal length of 460 px to express the image noise (sqrt_info = 460/1.5), the keyframe
// parallax (keyframe_parallax / 460) and the F-RANSAC threshold of the tracker in pixels. It is a config value here
// (key "virtual_focal_length", default 460 = original behaviour) because for a narrow-FOV camera
// (e.g. f ~ 3300 px) 460 would make the image noise 1.5 * 3300 / 460 = 11 real pixels. The two hard-coded angular thresholds of
// the initialisation (30 / 460 rad parallax in relativePose, 0.3 / 460 in solve_5pts) are kept unchanged.
extern double FOCAL_LENGTH;
// sliding-window size (10 in VINS-Mono). Compile-time constant (array sizes); CMake option VINS_WINDOW_SIZE.
#ifndef VINS_WINDOW_SIZE
#define VINS_WINDOW_SIZE 10
#endif
const int WINDOW_SIZE = VINS_WINDOW_SIZE;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

// failure detection thresholds (original hard-coded values are the defaults)
extern double FAILURE_MAX_ACC_BIAS, FAILURE_MAX_GYR_BIAS, FAILURE_MAX_TRANSLATION, FAILURE_MAX_Z, FAILURE_MAX_ANGLE_DEG;
extern int ENABLE_FAILURE_DETECTION;
extern int NUM_THREADS_SOLVER;
extern std::string OUTPUT_PATH;


void readParameters(const std::string &config_file);

// read an optional yaml key (keeps the default when the key is missing)
template <typename T>
void readOptional(const cv::FileStorage &fs, const char *key, T &value)
{
    cv::FileNode n = fs[key];
    if (!n.empty() && !n.isNone())
        n >> value;
}

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

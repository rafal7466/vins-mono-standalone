// ft_parameters.h -- parameters of the VINS-Mono feature tracker node (feature_tracker/src/parameters.h of the original).
// They live in namespace ft because the original tracker and estimator were separate ROS nodes that both define
// ROW, COL, FOCAL_LENGTH, WINDOW_SIZE ... with different types / values.
#pragma once
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "ros_compat.h"

namespace ft
{
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;

extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;

// reads the same yaml keys as the original node (max_cnt, min_dist, image_height, image_width, freq, F_threshold,
// show_track, equalize, fisheye). FOCAL_LENGTH is the "virtual" focal length (key virtual_focal_length, default 460).
void readParameters(const std::string &config_file, const std::string &vins_folder);
} // namespace ft

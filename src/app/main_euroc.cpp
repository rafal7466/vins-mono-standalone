// main_euroc.cpp -- ROS-free driver of VINS-Mono (HKUST-Aerial-Robotics, GPLv3) for the EuRoC MAV datasets (ASL format).
//
// The original system runs as two ROS nodes connected by topics:
//   feature_tracker_node : image -> (frequency control, KLT tracking, F-RANSAC, new corners) -> "feature" point cloud
//   estimator_node       : IMU + "feature" -> getMeasurements() -> processIMU() / processImage() (sliding-window BA)
// This driver replays the dataset and calls exactly the same code in the same order, but in one thread and without
// dropping data, so a run is deterministic and can be repeated (except that ceres' max_solver_time makes the number of
// iterations depend on the CPU speed; set max_solver_time large for strictly repeatable runs). The node logic
// (first-image skip, image-discontinuity restart, FREQ control, the IMU interpolation at the image time + td) is a
// line-by-line transcription of feature_tracker_node.cpp::img_callback and estimator_node.cpp::getMeasurements/process.
// Not ported: rviz visualisation, the loop-closure / pose_graph package and the AR demo.
//
// usage: vins_euroc [config.yaml] [--dataset DIR] [--out DIR] [--start IMG_IDX] [--duration SEC] [--log LEVEL]
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "euroc_dataset.h"
#include "estimator.h"
#include "feature_tracker.h"
#include "ft_parameters.h"
#include "parameters.h"
#include "ros_compat.h"

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

namespace
{
struct DriverConfig
{
    std::string configFile = "config/euroc_config.yaml";
    std::string datasetRoot = "dataset/MH_01_easy";
    std::string outDir;          // default: output_path of the yaml
    long long startImageIndex = 1; // 1-based
    double durationSec = 0;        // 0 = whole sequence
    int logLevel = 1;
    int progressEvery = 200;     // frames
    int saveTrackEvery = 0;      // write an image with the tracked features every N published frames (0 = off)
};

std::string nowStamp()
{
    std::time_t t = std::time(nullptr);
    char b[64];
    std::strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return b;
}

void usage()
{
    std::printf("usage: vins_euroc [config.yaml] [--dataset DIR] [--out DIR] [--start IMG_IDX(1-based)] [--duration SEC] [--log 0..4]\n");
}

bool parseArgs(int argc, char **argv, DriverConfig &c, std::map<std::string, std::string> &cli)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        auto next = [&](const char *name) -> std::string {
            if (i + 1 >= argc)
            {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--dataset") cli["dataset"] = next("--dataset");
        else if (a == "--out") cli["out"] = next("--out");
        else if (a == "--start") cli["start"] = next("--start");
        else if (a == "--duration") cli["duration"] = next("--duration");
        else if (a == "--log") cli["log"] = next("--log");
        else if (a == "-h" || a == "--help") { usage(); return false; }
        else if (!a.empty() && a[0] != '-') c.configFile = a;
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); usage(); return false; }
    }
    return true;
}

void readDriverConfig(DriverConfig &c, const std::map<std::string, std::string> &cli)
{
    cv::FileStorage fs(c.configFile, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::fprintf(stderr, "cannot open config %s\n", c.configFile.c_str());
        std::exit(2);
    }
    readOptional(fs, "dataset_root", c.datasetRoot);
    double v = (double)c.startImageIndex;
    readOptional(fs, "start_image_index", v);
    c.startImageIndex = (long long)v;
    readOptional(fs, "duration_s", c.durationSec);
    readOptional(fs, "log_level", c.logLevel);
    readOptional(fs, "progress_every", c.progressEvery);
    readOptional(fs, "save_track_every", c.saveTrackEvery);
    readOptional(fs, "output_path", c.outDir);
    if (cli.count("dataset")) c.datasetRoot = cli.at("dataset");
    if (cli.count("out")) c.outDir = cli.at("out");
    if (cli.count("start")) c.startImageIndex = std::stoll(cli.at("start"));
    if (cli.count("duration")) c.durationSec = std::stod(cli.at("duration"));
    if (cli.count("log")) c.logLevel = std::stoi(cli.at("log"));
}

// feature message of feature_tracker_node (sensor_msgs::PointCloud with channels id, u, v, vx, vy)
using FeatureFrame = std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>;

void drawTrack(const cv::Mat &gray, const ft::FeatureTracker &tr, const std::string &file)
{
    cv::Mat col;
    cv::cvtColor(gray, col, cv::COLOR_GRAY2BGR);
    for (size_t j = 0; j < tr.cur_pts.size(); ++j)
    {
        double len = std::min(1.0, 1.0 * tr.track_cnt[j] / ft::WINDOW_SIZE);
        cv::circle(col, tr.cur_pts[j], 3, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    cv::imwrite(file, col);
}
} // namespace

int main(int argc, char **argv)
{
    DriverConfig dc;
    std::map<std::string, std::string> cli;
    if (!parseArgs(argc, argv, dc, cli))
        return 1;
    readDriverConfig(dc, cli);
    vins_log::level = dc.logLevel;

    // ---------------- parameters of the two original nodes ----------------
    readParameters(dc.configFile);          // vins_estimator/parameters.cpp
    ft::readParameters(dc.configFile, "");  // feature_tracker/parameters.cpp
    if (dc.outDir.empty())
        dc.outDir = OUTPUT_PATH;
    if (dc.outDir.empty())
        dc.outDir = "output";
    {
        std::error_code ec;
        fs::create_directories(fs::u8path(dc.outDir), ec);
        if (dc.saveTrackEvery > 0)
            fs::create_directories(fs::u8path(dc.outDir) / "track", ec);
    }
    OUTPUT_PATH = dc.outDir;
    VINS_RESULT_PATH = joinPath(dc.outDir, "vins_result_no_loop.csv");
    if (ESTIMATE_EXTRINSIC)
        EX_CALIB_RESULT_PATH = joinPath(dc.outDir, "extrinsic_parameter.csv");
    std::string logFile = joinPath(dc.outDir, "run_log.txt");
    vins_log::file = std::fopen(logFile.c_str(), "w");

    ROS_INFO("VINS-Mono (ROS-free C++ port) on EuRoC -- started %s", nowStamp().c_str());
    ROS_INFO("config  : %s", dc.configFile.c_str());
    ROS_INFO("dataset : %s", dc.datasetRoot.c_str());
    ROS_INFO("output  : %s", dc.outDir.c_str());

    // ---------------- dataset ----------------
    EurocDataset ds;
    std::string err;
    auto tLoad = Clock::now();
    if (!ds.open(dc.datasetRoot, err) || !ds.loadImu(err) || !ds.listImages(err))
    {
        ROS_ERROR("%s", err.c_str());
        return 2;
    }
    ROS_INFO("IMU rows %zu, images %zu, loaded in %.1f s", ds.imu.size(),
             ds.images.size(), std::chrono::duration<double>(Clock::now() - tLoad).count());
    if (dc.startImageIndex < 1 || dc.startImageIndex > (long long)ds.images.size())
    {
        ROS_ERROR("start_image_index %lld out of range (1..%zu)", dc.startImageIndex, ds.images.size());
        return 2;
    }
    const size_t k0 = (size_t)(dc.startImageIndex - 1);
    const int64_t tStartNs = ds.images[k0].t_ns;
    const int64_t tEndNs = dc.durationSec > 0 ? tStartNs + (int64_t)std::llround(dc.durationSec * 1e9) : ds.images.back().t_ns;
    size_t k1 = k0;
    while (k1 + 1 < ds.images.size() && ds.images[k1 + 1].t_ns <= tEndNs)
        ++k1;
    ROS_INFO("run: images %zu..%zu (1-based %lld..%zu, %zu images), t0 = %lld ns, duration %.1f s", k0, k1, dc.startImageIndex, k1 + 1, k1 - k0 + 1,
             (long long)tStartNs, (ds.images[k1].t_ns - tStartNs) * 1e-9);
    ROS_INFO("camera %dx%d, max_cnt %d, min_dist %d, freq %d, F_threshold %.2f, equalize %d | window %d, virtual focal %.0f px, td %.4f s (estimate %d)",
             ft::COL, ft::ROW, ft::MAX_CNT, ft::MIN_DIST, ft::FREQ, ft::F_THRESHOLD, ft::EQUALIZE, WINDOW_SIZE, FOCAL_LENGTH, TD, ESTIMATE_TD);
    ROS_INFO("IMU noise acc_n %g gyr_n %g acc_w %g gyr_w %g, g_norm %.4f, solver %d it / %.3f s", ACC_N, GYR_N, ACC_W, GYR_W, G.z(), NUM_ITERATIONS, SOLVER_TIME);

    // ---------------- the two "nodes" ----------------
    std::unique_ptr<Estimator> estimator(new Estimator());
    estimator->setParameter();
    ft::FeatureTracker trackerData[ft::NUM_OF_CAM];
    for (int i = 0; i < ft::NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(ft::CAM_NAMES[i]);
    if (!trackerData[0].m_camera)
    {
        ROS_ERROR("cannot read the camera model from %s", ft::CAM_NAMES[0].c_str());
        return 2;
    }
    ROS_INFO("camera model: %s", trackerData[0].m_camera->parametersToString().c_str());

    // feature_tracker_node state
    double first_image_time = 0, last_image_time = 0;
    int pub_count = 1;
    bool first_image_flag = true, init_pub = false;
    // estimator_node state
    bool init_feature = false;
    double current_time = -1;
    // replay of the IMU topic: the IMU "buffer" is ds.imu[imuIdx ...]
    size_t imuIdx = 0;
    while (imuIdx < ds.imu.size() && ds.imu[imuIdx].t_ns < tStartNs - 100000000LL) // IMU subscriber starts 0.1 s before the first image
        ++imuIdx;

    // ---------------- outputs ----------------
    std::ofstream fRes(VINS_RESULT_PATH); // original VINS format (pubOdometry): t[ns],p,q(w,x,y,z),v
    std::ofstream fTraj(joinPath(dc.outDir, "vins_trajectory.csv"));
    fTraj << "timestamp_ns,t_rel_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz,td_s,segment,n_features_frame,n_tracked_window,solve_ms,track_ms\n";
    std::ofstream fFrames(joinPath(dc.outDir, "vins_frames.csv")); // every published frame (also before initialisation)
    fFrames << "timestamp_ns,t_rel_s,solver_flag,n_points,n_window_features,marg_flag,failures,segment,solve_ms,track_ms\n";

    long long nImgRead = 0, nPub = 0, nProcessed = 0, nOut = 0, nRestart = 0, nThrown = 0;
    int segment = 0;
    bool prevNonLinear = false;
    double firstInitTime = -1;
    double sumSolve = 0, sumTrack = 0, maxSolve = 0;
    auto tRun = Clock::now();
    int lastFailures = 0;

    for (size_t k = k0; k <= k1; ++k)
    {
        const double t = ds.images[k].t_ns * 1e-9;
        // ======================= feature_tracker_node.cpp :: img_callback =======================
        if (first_image_flag)
        {
            first_image_flag = false;
            first_image_time = t;
            last_image_time = t;
            continue;
        }
        if (t - last_image_time > 1.0 || t < last_image_time)
        {
            ROS_WARN("image discontinue! reset the feature tracker! (gap %.3f s at t_rel %.2f s)", t - last_image_time, t - tStartNs * 1e-9);
            first_image_flag = true;
            last_image_time = 0;
            pub_count = 1;
            // estimator_node.cpp :: restart_callback
            ROS_WARN("restart the estimator!");
            estimator->clearState();
            estimator->setParameter();
            current_time = -1;
            while (imuIdx < ds.imu.size() && ds.imu[imuIdx].t_ns <= ds.images[k].t_ns) // imu_buf cleared
                ++imuIdx;
            ++nRestart;
            continue;
        }
        last_image_time = t;
        if (std::round(1.0 * pub_count / (t - first_image_time)) <= ft::FREQ)
        {
            ft::PUB_THIS_FRAME = true;
            if (std::abs(1.0 * pub_count / (t - first_image_time) - ft::FREQ) < 0.01 * ft::FREQ)
            {
                first_image_time = t;
                pub_count = 0;
            }
        }
        else
            ft::PUB_THIS_FRAME = false;

        cv::Mat img = cv::imread(joinPath(ds.imageDir, ds.images[k].file), cv::IMREAD_GRAYSCALE);
        if (img.empty())
        {
            ROS_WARN("cannot read image %s -> skipped", ds.images[k].file.c_str());
            continue;
        }
        if (img.rows != ft::ROW || img.cols != ft::COL)
        {
            ROS_ERROR("image size %dx%d differs from image_width/image_height %dx%d of the config", img.cols, img.rows, ft::COL, ft::ROW);
            return 2;
        }
        ++nImgRead;
        TicToc t_r;
        for (int i = 0; i < ft::NUM_OF_CAM; i++)
            trackerData[i].readImage(img.rowRange(ft::ROW * i, ft::ROW * (i + 1)), t);
        for (unsigned int i = 0;; i++)
        {
            bool completed = false;
            for (int j = 0; j < ft::NUM_OF_CAM; j++)
                completed |= trackerData[j].updateID(i);
            if (!completed)
                break;
        }
        double trackMs = t_r.toc();
        sumTrack += trackMs;
        if (!ft::PUB_THIS_FRAME)
            continue;
        pub_count++;
        FeatureFrame image;
        for (int i = 0; i < ft::NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts;
            auto &cur_pts = trackerData[i].cur_pts;
            auto &ids = trackerData[i].ids;
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    // the ROS message carries float32 values (geometry_msgs::Point32 / ChannelFloat32)
                    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                    xyz_uv_velocity << (float)un_pts[j].x, (float)un_pts[j].y, 1.0, (float)cur_pts[j].x, (float)cur_pts[j].y, (float)pts_velocity[j].x,
                        (float)pts_velocity[j].y;
                    int v = p_id * ft::NUM_OF_CAM + i;
                    image[v / NUM_OF_CAM].emplace_back(v % NUM_OF_CAM, xyz_uv_velocity);
                }
            }
        }
        if (dc.saveTrackEvery > 0 && nPub % dc.saveTrackEvery == 0)
            drawTrack(img, trackerData[0], (fs::u8path(dc.outDir) / "track" / (std::to_string(ds.images[k].t_ns) + ".jpg")).u8string());
        ++nPub;
        // skip the first image; since no optical speed on first image
        if (!init_pub)
        {
            init_pub = true;
            continue;
        }
        // ======================= estimator_node.cpp :: feature_callback =======================
        if (!init_feature)
        {
            init_feature = true; // skip the first detected feature, which doesn't contain optical flow speed
            continue;
        }
        // ======================= estimator_node.cpp :: getMeasurements =======================
        double img_t = t + estimator->td;
        // the IMU buffer must contain a sample newer than the image (always true offline except at the end of the log)
        size_t jLast = imuIdx;
        while (jLast < ds.imu.size() && ds.imu[jLast].t_ns * 1e-9 <= img_t)
            ++jLast;
        if (jLast >= ds.imu.size())
        {
            ROS_WARN("no IMU after t = %.3f -> end of the run", img_t);
            break;
        }
        if (imuIdx >= ds.imu.size() || !(ds.imu[imuIdx].t_ns * 1e-9 < img_t))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            ++nThrown;
            continue;
        }
        std::vector<size_t> IMUs;
        while (ds.imu[imuIdx].t_ns * 1e-9 < img_t)
            IMUs.push_back(imuIdx++);
        IMUs.push_back(imuIdx); // first sample after the image: used for the interpolation, stays in the buffer

        // ======================= estimator_node.cpp :: process =======================
        double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
        for (size_t q : IMUs)
        {
            const ImuSample &m = ds.imu[q];
            double ti = m.t_ns * 1e-9;
            if (ti <= img_t)
            {
                if (current_time < 0)
                    current_time = ti;
                double dt = ti - current_time;
                ROS_ASSERT(dt >= 0);
                current_time = ti;
                dx = m.acc.x(); dy = m.acc.y(); dz = m.acc.z();
                rx = m.gyr.x(); ry = m.gyr.y(); rz = m.gyr.z();
                estimator->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            }
            else
            {
                double dt_1 = img_t - current_time;
                double dt_2 = ti - img_t;
                current_time = img_t;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                dx = w1 * dx + w2 * m.acc.x();
                dy = w1 * dy + w2 * m.acc.y();
                dz = w1 * dz + w2 * m.acc.z();
                rx = w1 * rx + w2 * m.gyr.x();
                ry = w1 * ry + w2 * m.gyr.y();
                rz = w1 * rz + w2 * m.gyr.z();
                estimator->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            }
        }
        std_msgs::Header header;
        header.stamp = std_msgs::Time(t);
        header.frame_id = "world";
        TicToc t_s;
        estimator->processImage(image, header);
        double solveMs = t_s.toc();
        ++nProcessed;
        sumSolve += solveMs;
        maxSolve = std::max(maxSolve, solveMs);

        bool nonLinear = estimator->solver_flag == Estimator::SolverFlag::NON_LINEAR;
        if (estimator->failure_count != lastFailures)
        {
            ROS_WARN("VINS failure #%d at t_rel = %.2f s (system reboot)", estimator->failure_count, t - tStartNs * 1e-9);
            lastFailures = estimator->failure_count;
        }
        if (nonLinear && !prevNonLinear)
        {
            ++segment;
            if (firstInitTime < 0)
                firstInitTime = t;
            ROS_WARN("VINS initialised (segment %d) at t_rel = %.2f s", segment, t - tStartNs * 1e-9);
        }
        prevNonLinear = nonLinear;
        int nWin = estimator->f_manager.getFeatureCount();
        fFrames << ds.images[k].t_ns << "," << std::fixed << std::setprecision(4) << (t - tStartNs * 1e-9) << "," << (int)estimator->solver_flag << ","
                << image.size() << "," << nWin << "," << (int)estimator->marginalization_flag << "," << estimator->failure_count << "," << segment << ","
                << std::setprecision(2) << solveMs << "," << trackMs << "\n";
        if (nonLinear)
        {
            // pubOdometry (utility/visualization.cpp)
            const Vector3d &P = estimator->Ps[WINDOW_SIZE];
            const Vector3d &V = estimator->Vs[WINDOW_SIZE];
            Quaterniond Q(estimator->Rs[WINDOW_SIZE]);
            fRes.setf(std::ios::fixed, std::ios::floatfield);
            fRes.precision(0);
            fRes << t * 1e9 << ",";
            fRes.precision(5);
            fRes << P.x() << "," << P.y() << "," << P.z() << "," << Q.w() << "," << Q.x() << "," << Q.y() << "," << Q.z() << "," << V.x() << "," << V.y() << ","
                 << V.z() << "," << std::endl;
            const Vector3d &Ba = estimator->Bas[WINDOW_SIZE], &Bg = estimator->Bgs[WINDOW_SIZE];
            fTraj << ds.images[k].t_ns << "," << std::setprecision(4) << (t - tStartNs * 1e-9) << "," << std::setprecision(4) << P.x() << "," << P.y() << ","
                  << P.z() << "," << std::setprecision(7) << Q.w() << "," << Q.x() << "," << Q.y() << "," << Q.z() << "," << std::setprecision(4) << V.x() << ","
                  << V.y() << "," << V.z() << "," << std::setprecision(6) << Ba.x() << "," << Ba.y() << "," << Ba.z() << "," << Bg.x() << "," << Bg.y() << ","
                  << Bg.z() << "," << estimator->td << "," << segment << "," << image.size() << "," << nWin << "," << std::setprecision(2) << solveMs << ","
                  << trackMs << "\n";
            ++nOut;
        }
        if (dc.progressEvery > 0 && nProcessed % dc.progressEvery == 0)
        {
            double el = std::chrono::duration<double>(Clock::now() - tRun).count();
            double tr = t - tStartNs * 1e-9;
            const Vector3d &P = estimator->Ps[WINDOW_SIZE];
            const Vector3d &V = estimator->Vs[WINDOW_SIZE];
            ROS_INFO("t_rel %7.1f s | %s | seg %d | fail %d | pts %3zu | |p| %8.1f m z %7.1f | |v| %5.1f m/s | ba %.3f bg %.4f td %+.4f | solve %.1f ms (avg %.1f) | x%.2f real time",
                     tr, nonLinear ? "VIO " : "INIT", segment, estimator->failure_count, image.size(), P.norm(), P.z(), V.norm(),
                     estimator->Bas[WINDOW_SIZE].norm(), estimator->Bgs[WINDOW_SIZE].norm(), estimator->td, solveMs, sumSolve / nProcessed, tr / std::max(el, 1e-6));
            fTraj.flush();
            fFrames.flush();
        }
    }
    double elapsed = std::chrono::duration<double>(Clock::now() - tRun).count();
    double runDur = (ds.images[k1].t_ns - tStartNs) * 1e-9;
    std::ostringstream sm;
    sm << std::fixed << std::setprecision(3);
    sm << "images_in_run=" << (k1 - k0 + 1) << "\nimages_read=" << nImgRead << "\nframes_published=" << nPub << "\nframes_processed=" << nProcessed
       << "\nposes_output=" << nOut << "\nfirst_init_t_rel_s=" << (firstInitTime < 0 ? -1.0 : firstInitTime - tStartNs * 1e-9)
       << "\nsegments=" << segment << "\nfailures_reboots=" << estimator->failure_count << "\nimage_discontinuity_restarts=" << nRestart
       << "\nthrown_images=" << nThrown << "\nrun_duration_s=" << runDur << "\nwall_time_s=" << elapsed
       << "\nrealtime_factor=" << runDur / std::max(elapsed, 1e-9) << "\nmean_solve_ms=" << (nProcessed ? sumSolve / nProcessed : 0)
       << "\nmax_solve_ms=" << maxSolve << "\nmean_track_ms=" << (nImgRead ? sumTrack / nImgRead : 0) << "\nfinal_td_s=" << estimator->td
       << "\nfinal_ba=" << estimator->Bas[WINDOW_SIZE].transpose() << "\nfinal_bg=" << estimator->Bgs[WINDOW_SIZE].transpose()
       << "\nfinal_tic=" << estimator->tic[0].transpose() << "\nconfig=" << dc.configFile << "\nstart_image_index=" << dc.startImageIndex
       << "\nt0_ns=" << tStartNs << "\n";
    {
        std::ofstream f(joinPath(dc.outDir, "summary.txt"));
        f << sm.str();
    }
    ROS_INFO("finished %s\n%s", nowStamp().c_str(), sm.str().c_str());
    if (vins_log::file)
        std::fclose(vins_log::file);
    return 0;
}

#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>

#include <Tracking.h>
#include <System.h>
#include "DenseMapper.h"

namespace {
    std::atomic<bool> running{true};

    void handleSignal(int) {
        running.store(false);
    }
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr
            << "Usage: ./mono_webcam vocabulary settings.yaml\n";
        return 1;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    cv::VideoCapture camera(0, cv::CAP_AVFOUNDATION);

    if (!camera.isOpened()) {
        std::cerr << "ERROR: Cannot open MacBook webcam.\n";
        return 1;
    }

    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    std::cout << "Webcam opened successfully.\n";
    std::cout << "Initializing ORB-SLAM3...\n";

    std::unique_ptr<DenseMapper> denseMapper;
    try {
        denseMapper.reset(new DenseMapper(argv[2]));
    } catch (const std::exception& error) {
        std::cerr << "ERROR: Dense mapper setup failed: "
                  << error.what() << '\n';
        camera.release();
        return 1;
    }

    ORB_SLAM3::System SLAM(
        argv[1],
        argv[2],
        ORB_SLAM3::System::MONOCULAR,
        true
    );

    std::thread cameraThread([&camera, &SLAM, &denseMapper]() {
        cv::Mat frame;

        while (running.load()) {
            if (!camera.read(frame) || frame.empty()) {
                std::cerr << "ERROR: Failed to read webcam frame.\n";
                running.store(false);
                break;
            }

            const double timestamp =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now()
                        .time_since_epoch()
                ).count();

            const Sophus::SE3f Tcw = SLAM.TrackMonocular(frame, timestamp);
            if (SLAM.GetTrackingState() == ORB_SLAM3::Tracking::OK) {
                std::vector<cv::Point3f> densePoints;
                std::vector<cv::Vec3b> denseColors;
                if (denseMapper->Process(frame, Tcw,
                                         densePoints, denseColors)) {
                    SLAM.AddDenseMapPoints(densePoints, denseColors);
                    std::cout << "[Dense map] Added " << densePoints.size()
                              << " points from a stereo pair." << std::endl;
                }
            } else {
                denseMapper->ResetReference();
            }

        }

        std::cout << "Camera thread stopped.\n";
    });

    // Pangolin viewer must run on the main thread in this fork.
    std::cout << "Starting Pangolin viewer...\n";
    SLAM.RunViewer();

    // The viewer has closed. Stop tracking and clean up.
    running.store(false);

    if (cameraThread.joinable()) {
        cameraThread.join();
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM(
        "KeyFrameTrajectory_webcam.txt"
    );
    if (!SLAM.SaveDenseMapPointsPLY("DensePointCloud_webcam.ply")) {
        std::cerr << "ERROR: No dense points were reconstructed; "
                     "DensePointCloud_webcam.ply was not written.\n";
    }

    camera.release();

    std::cout << "SLAM stopped. Trajectory saved.\n";
    return 0;
}

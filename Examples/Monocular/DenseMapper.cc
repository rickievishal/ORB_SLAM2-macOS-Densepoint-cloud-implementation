#include "DenseMapper.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

DenseMapper::DenseMapper(const std::string& settingsPath)
    : mDistortion(cv::Mat::zeros(1, 5, CV_64F)),
      mFramesSinceReference(0)
{
    cv::FileStorage settings(settingsPath, cv::FileStorage::READ);
    if (!settings.isOpened())
        throw std::runtime_error("Cannot read camera settings: " + settingsPath);

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    int width = 0;
    int height = 0;
    settings["Camera1.fx"] >> fx;
    settings["Camera1.fy"] >> fy;
    settings["Camera1.cx"] >> cx;
    settings["Camera1.cy"] >> cy;
    settings["Camera.width"] >> width;
    settings["Camera.height"] >> height;
    if (fx <= 0.0 || fy <= 0.0 || width <= 0 || height <= 0)
        throw std::runtime_error("Invalid camera intrinsics or image size in " +
                                 settingsPath);

    mCameraMatrix = (cv::Mat_<double>(3, 3) <<
        fx, 0.0, cx,
        0.0, fy, cy,
        0.0, 0.0, 1.0);
    const char* distortionKeys[] = {
        "Camera1.k1", "Camera1.k2", "Camera1.p1",
        "Camera1.p2", "Camera1.k3"
    };
    for (int i = 0; i < 5; ++i)
    {
        const cv::FileNode value = settings[distortionKeys[i]];
        if (!value.empty())
            mDistortion.at<double>(0, i) = static_cast<double>(value);
    }
    mConfiguredSize = cv::Size(width, height);
}

cv::Mat DenseMapper::CameraMatrixForSize(const cv::Size& imageSize) const
{
    const double scaleX =
        static_cast<double>(imageSize.width) / mConfiguredSize.width;
    const double scaleY =
        static_cast<double>(imageSize.height) / mConfiguredSize.height;
    cv::Mat cameraMatrix = mCameraMatrix.clone();
    cameraMatrix.at<double>(0, 0) *= scaleX;
    cameraMatrix.at<double>(0, 2) *= scaleX;
    cameraMatrix.at<double>(1, 1) *= scaleY;
    cameraMatrix.at<double>(1, 2) *= scaleY;
    return cameraMatrix;
}

bool DenseMapper::Process(const cv::Mat& image,
                          const Sophus::SE3f& Tcw,
                          std::vector<cv::Point3f>& worldPoints,
                          std::vector<cv::Vec3b>& colors)
{
    if (image.empty() || (image.channels() != 1 && image.channels() != 3))
        return false;

    worldPoints.clear();
    colors.clear();
    if (mReferenceImage.empty())
    {
        mReferenceImage = image.clone();
        mReferenceTcw = Tcw;
        return false;
    }

    ++mFramesSinceReference;
    if (mFramesSinceReference < 10)
        return false;

    const Sophus::SE3f TcurrentFromReference = Tcw * mReferenceTcw.inverse();
    const float baseline = TcurrentFromReference.translation().norm();
    if (baseline < 0.04f)
        return false;

    const cv::Size imageSize = image.size();
    const cv::Mat cameraMatrix = CameraMatrixForSize(imageSize);
    cv::Mat rotation(3, 3, CV_64F);
    cv::Mat translation(3, 1, CV_64F);
    const Eigen::Matrix3f rotationEigen =
        TcurrentFromReference.rotationMatrix();
    const Eigen::Vector3f translationEigen =
        TcurrentFromReference.translation();
    for (int row = 0; row < 3; ++row)
    {
        translation.at<double>(row, 0) = translationEigen[row];
        for (int column = 0; column < 3; ++column)
            rotation.at<double>(row, column) =
                rotationEigen(row, column);
    }

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(cameraMatrix, mDistortion,
                      cameraMatrix, mDistortion,
                      imageSize, rotation, translation,
                      R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0.0);

    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(cameraMatrix, mDistortion, R1, P1,
                                imageSize, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(cameraMatrix, mDistortion, R2, P2,
                                imageSize, CV_32FC1, map2x, map2y);

    cv::Mat referenceGray, currentGray;
    if (mReferenceImage.channels() == 3)
        cv::cvtColor(mReferenceImage, referenceGray, cv::COLOR_BGR2GRAY);
    else
        referenceGray = mReferenceImage;
    if (image.channels() == 3)
        cv::cvtColor(image, currentGray, cv::COLOR_BGR2GRAY);
    else
        currentGray = image;

    cv::Mat rectifiedReferenceGray, rectifiedCurrentGray;
    cv::remap(referenceGray, rectifiedReferenceGray, map1x, map1y,
              cv::INTER_LINEAR);
    cv::remap(currentGray, rectifiedCurrentGray, map2x, map2y,
              cv::INTER_LINEAR);

    cv::Mat rectifiedReferenceColor;
    if (mReferenceImage.channels() == 3)
        cv::remap(mReferenceImage, rectifiedReferenceColor,
                  map1x, map1y, cv::INTER_LINEAR);

    const int minimumDisparity = P2.at<double>(0, 3) > 0.0 ? -127 : 0;
    const int numberOfDisparities = 128;
    const int blockSize = 5;
    cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
        minimumDisparity, numberOfDisparities, blockSize,
        8 * blockSize * blockSize, 32 * blockSize * blockSize,
        1, 31, 10, 100, 2, cv::StereoSGBM::MODE_SGBM_3WAY);
    cv::Mat disparityFixed;
    stereo->compute(rectifiedReferenceGray, rectifiedCurrentGray,
                    disparityFixed);

    cv::Mat disparity;
    disparityFixed.convertTo(disparity, CV_32F, 1.0 / 16.0);
    cv::Mat pointsRectified;
    cv::reprojectImageTo3D(disparity, pointsRectified, Q, true, CV_32F);
    cv::Mat rectificationRotation64;
    cv::transpose(R1, rectificationRotation64);
    cv::Mat rectificationRotation;
    rectificationRotation64.convertTo(rectificationRotation, CV_32F);
    const Sophus::SE3f TwcReference = mReferenceTcw.inverse();

    for (int y = 0; y < imageSize.height; y += 2)
    {
        for (int x = 0; x < imageSize.width; x += 2)
        {
            const float d = disparity.at<float>(y, x);
            if (!std::isfinite(d) ||
                d <= minimumDisparity ||
                d >= minimumDisparity + numberOfDisparities - 1 ||
                std::abs(d) < 1.0f)
                continue;

            const cv::Vec3f rectified =
                pointsRectified.at<cv::Vec3f>(y, x);
            if (!std::isfinite(rectified[0]) ||
                !std::isfinite(rectified[1]) ||
                !std::isfinite(rectified[2]) ||
                rectified[2] <= 0.1f || rectified[2] > 30.0f)
                continue;

            const cv::Mat rectifiedPoint =
                (cv::Mat_<float>(3, 1) <<
                    rectified[0], rectified[1], rectified[2]);
            const cv::Mat cameraPoint =
                rectificationRotation * rectifiedPoint;
            const Eigen::Vector3f pointCamera(
                cameraPoint.at<float>(0),
                cameraPoint.at<float>(1),
                cameraPoint.at<float>(2));
            const Eigen::Vector3f pointWorld = TwcReference * pointCamera;
            worldPoints.emplace_back(pointWorld.x(), pointWorld.y(),
                                     pointWorld.z());
            if (mReferenceImage.channels() == 3)
                colors.push_back(rectifiedReferenceColor.at<cv::Vec3b>(y, x));
            else
            {
                const uchar gray = rectifiedReferenceGray.at<uchar>(y, x);
                colors.emplace_back(gray, gray, gray);
            }
        }
    }

    mReferenceImage = image.clone();
    mReferenceTcw = Tcw;
    mFramesSinceReference = 0;
    return !worldPoints.empty();
}

void DenseMapper::ResetReference()
{
    mReferenceImage.release();
    mFramesSinceReference = 0;
}

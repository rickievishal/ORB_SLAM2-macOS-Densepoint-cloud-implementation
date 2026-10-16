#ifndef ORB_SLAM3_DENSE_MAPPER_H
#define ORB_SLAM3_DENSE_MAPPER_H

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include <string>
#include <vector>

class DenseMapper
{
public:
    explicit DenseMapper(const std::string& settingsPath);

    bool Process(const cv::Mat& image,
                 const Sophus::SE3f& Tcw,
                 std::vector<cv::Point3f>& worldPoints,
                 std::vector<cv::Vec3b>& colors);
    void ResetReference();

private:
    cv::Mat CameraMatrixForSize(const cv::Size& imageSize) const;

    cv::Mat mCameraMatrix;
    cv::Mat mDistortion;
    cv::Size mConfiguredSize;
    cv::Mat mReferenceImage;
    Sophus::SE3f mReferenceTcw;
    int mFramesSinceReference;
};

#endif

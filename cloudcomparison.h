#ifndef CLOUDCOMPARISON_H
#define CLOUDCOMPARISON_H

#include <qprogressdialog.h>
#include <vector>
#include <QString>
#include <QVector3D>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <thread>
#include <atomic>
#include <QFuture>
#include <QtConcurrent>
#include <QMutex>

// Define Point Types and Constants
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudTColor = pcl::PointCloud<pcl::PointXYZRGB>;
using KdTreeT = pcl::KdTreeFLANN<PointT>;

// RGB colors represented as unsigned char for faster processing
constexpr unsigned char COLLISION_R = 255;
constexpr unsigned char COLLISION_G = 0;
constexpr unsigned char COLLISION_B = 0;
constexpr unsigned char ORIGINAL_COLOR_1_R = 100;
constexpr unsigned char ORIGINAL_COLOR_1_G = 100;
constexpr unsigned char ORIGINAL_COLOR_1_B = 100;
constexpr unsigned char ORIGINAL_COLOR_2_R = 150;
constexpr unsigned char ORIGINAL_COLOR_2_G = 150;
constexpr unsigned char ORIGINAL_COLOR_2_B = 150;

class CloudComparison
{
public:
    CloudComparison();

    // Structure to hold collision detection results
    struct CollisionResult {
        std::vector<QVector3D> points;
        std::vector<QVector3D> colors;
        int collisionCount;
        size_t totalPoints;
    };

    // Get optimal number of threads for this system
    inline int getOptimalThreadCount() const {
        return std::max(1, static_cast<int>(std::thread::hardware_concurrency() - 1));
    }

    // Perform collision detection between two point clouds
    bool performCollisionDetection(
        const std::vector<QVector3D>& points1,
        const std::vector<QVector3D>& colors1,
        const std::vector<QVector3D>& points2,
        const std::vector<QVector3D>& colors2,
        float threshold,
        CollisionResult& result1,
        CollisionResult& result2,
        QProgressDialog* progress = nullptr
        );

private:
    // Helper function to convert QVector3D points to PCL point cloud
    PointCloudT::Ptr convertToPCL(const std::vector<QVector3D>& points);

    // Process cloud points to check for collisions
    void processCloudPoints(
        const PointCloudT::Ptr& sourceCloud,
        const std::vector<QVector3D>& sourceColors,
        const KdTreeT& kdtree,
        float thresholdSquared,
        PointCloudTColor::Ptr resultCloud,
        const unsigned char defaultR,
        const unsigned char defaultG,
        const unsigned char defaultB,
        int startIdx,
        int endIdx,
        std::atomic<int>& collisionCounter
        );

    // Update progress safely from worker threads
    void updateProgress(QProgressDialog* progress, int value);

    // Mutex for thread-safe progress updates
    QMutex progressMutex;
};

#endif // CLOUDCOMPARISON_H

#include "cloudcomparison.h"
#include <QCoreApplication>
#include <algorithm>
#include <functional>
#include <QMutex>
#include <QMutexLocker>
#include <QMetaObject>
#include <QTimer>
#ifdef _OPENMP
#include <omp.h>
#endif

CloudComparison::CloudComparison() {}

// Helper function to check if all futures are finished
bool areFuturesFinished(const std::vector<QFuture<void>>& futures) {
    return std::all_of(futures.begin(), futures.end(),
                       [](const QFuture<void>& future) { return future.isFinished(); });
}

PointCloudT::Ptr CloudComparison::convertToPCL(const std::vector<QVector3D>& points) {
    PointCloudT::Ptr cloud(new PointCloudT);
    cloud->resize(points.size());

#pragma omp parallel for
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        cloud->points[i].x = points[i].x();
        cloud->points[i].y = points[i].y();
        cloud->points[i].z = points[i].z();
    }

    return cloud;
}

void CloudComparison::updateProgress(QProgressDialog* progress, int value) {
    if (!progress) return;

    // Use a mutex to prevent too many UI updates at once
    QMutexLocker locker(&progressMutex);

    // Use QMetaObject::invokeMethod to update UI from worker thread safely
    // This is safer than QtConcurrent::run which can cause issues
    QMetaObject::invokeMethod(progress, "setValue", Qt::QueuedConnection,
                              Q_ARG(int, value));

    // Don't process events from worker threads - this is a major cause of "not responding"
    // Let the main event loop handle events naturally
}

void CloudComparison::processCloudPoints(
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
    std::atomic<int>& collisionCounter) {

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    const bool hasColors = !sourceColors.empty();
    int localCollisionCount = 0;

    for (int i = startIdx; i < endIdx; ++i) {
        // Copy point coordinates
        resultCloud->points[i].x = sourceCloud->points[i].x;
        resultCloud->points[i].y = sourceCloud->points[i].y;
        resultCloud->points[i].z = sourceCloud->points[i].z;

        // Use direct search for better performance
        if (kdtree.nearestKSearch(sourceCloud->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            if (pointNKNSquaredDistance[0] <= thresholdSquared) {
                // Collision - mark red
                resultCloud->points[i].r = COLLISION_R;
                resultCloud->points[i].g = COLLISION_G;
                resultCloud->points[i].b = COLLISION_B;
                localCollisionCount++;
            } else {
                // No collision - use original color if available, otherwise default color
                if (hasColors && i < static_cast<int>(sourceColors.size())) {
                    resultCloud->points[i].r = static_cast<uint8_t>(sourceColors[i].x() * 255);
                    resultCloud->points[i].g = static_cast<uint8_t>(sourceColors[i].y() * 255);
                    resultCloud->points[i].b = static_cast<uint8_t>(sourceColors[i].z() * 255);
                } else {
                    resultCloud->points[i].r = defaultR;
                    resultCloud->points[i].g = defaultG;
                    resultCloud->points[i].b = defaultB;
                }
            }
        }
    }

    // Update collision counter atomically once at the end for better performance
    collisionCounter += localCollisionCount;
}

bool CloudComparison::performCollisionDetection(
    const std::vector<QVector3D>& points1,
    const std::vector<QVector3D>& colors1,
    const std::vector<QVector3D>& points2,
    const std::vector<QVector3D>& colors2,
    float threshold,
    CollisionResult& result1,
    CollisionResult& result2,
    QProgressDialog* progress) {

    if (points1.empty() || points2.empty()) {
        return false;
    }

    // Configure progress dialog for responsiveness
    if (progress) {
        progress->setMinimumDuration(0); // Show immediately
        progress->setValue(0);
        // Set a reasonable range
        progress->setRange(0, 100);
        progress->setCancelButton(nullptr); // Disable cancel button to avoid race conditions
        progress->setMinimumWidth(300); // Make it wide enough to see progress
    }

    // Pre-calculate squared threshold for faster comparison
    const float thresholdSquared = threshold * threshold;

    // Pre-allocate results to avoid reallocations later
    result1.points.clear();
    result1.colors.clear();
    result1.points.reserve(points1.size());
    result1.colors.reserve(points1.size());
    result1.collisionCount = 0;
    result1.totalPoints = points1.size();

    result2.points.clear();
    result2.colors.clear();
    result2.points.reserve(points2.size());
    result2.colors.reserve(points2.size());
    result2.collisionCount = 0;
    result2.totalPoints = points2.size();

    // Use delayed progress updates
    updateProgress(progress, 5);

    // Convert Qt point data to PCL format - run these sequentially to reduce memory pressure
    PointCloudT::Ptr cloud1 = convertToPCL(points1);
    updateProgress(progress, 10);

    PointCloudT::Ptr cloud2 = convertToPCL(points2);
    updateProgress(progress, 20);

    // Allow UI to refresh
    QCoreApplication::processEvents();

    // Create result point clouds
    PointCloudTColor::Ptr pcl_result1(new PointCloudTColor);
    PointCloudTColor::Ptr pcl_result2(new PointCloudTColor);
    pcl_result1->resize(cloud1->size());
    pcl_result2->resize(cloud2->size());

    // Build KD-trees using QtConcurrent to prevent UI blocking
    KdTreeT kdtree1, kdtree2;

    QFuture<void> futureKdtree1 = QtConcurrent::run([&kdtree1, &cloud1]() {
        kdtree1.setInputCloud(cloud1);
    });

    QFuture<void> futureKdtree2 = QtConcurrent::run([&kdtree2, &cloud2]() {
        kdtree2.setInputCloud(cloud2);
    });

    // Wait for both KD-trees to be built
    futureKdtree1.waitForFinished();
    futureKdtree2.waitForFinished();

    updateProgress(progress, 40);

    // Allow UI to refresh
    QCoreApplication::processEvents();

    // Get optimal thread count for parallelization, leave one core for UI
    int numThreads = getOptimalThreadCount();
#ifdef _OPENMP
    omp_set_num_threads(numThreads);
#endif

    // Atomic counters for collision counts
    std::atomic<int> collisionCount1(0);
    std::atomic<int> collisionCount2(0);

    // Process points from cloud1
    const int pointsPerThread = std::max(1, static_cast<int>(cloud1->size()) / numThreads);
    std::vector<QFuture<void>> futuresCloud1;

    for (int t = 0; t < numThreads; ++t) {
        int startIdx = t * pointsPerThread;
        int endIdx = (t == numThreads - 1) ? cloud1->size() : (t + 1) * pointsPerThread;

        futuresCloud1.push_back(QtConcurrent::run([this, &cloud1, &colors1, &kdtree2, thresholdSquared,
                                                   &pcl_result1, &collisionCount1, startIdx, endIdx]() {
            this->processCloudPoints(
                cloud1, colors1, kdtree2, thresholdSquared, pcl_result1,
                ORIGINAL_COLOR_1_R, ORIGINAL_COLOR_1_G, ORIGINAL_COLOR_1_B,
                startIdx, endIdx, collisionCount1
                );
        }));
    }

    // Use a timer to update progress and process events while waiting
    int progressValue = 40;
    while (!areFuturesFinished(futuresCloud1)) {
        // Increment progress
        if (progressValue < 60) {
            progressValue++;
            updateProgress(progress, progressValue);
        }

        // Sleep briefly to avoid CPU hogging
        QThread::msleep(50);

        // Process UI events on the main thread
        QCoreApplication::processEvents();
    }

    updateProgress(progress, 60);
    // Allow UI to refresh
    QCoreApplication::processEvents();

    // Process points from cloud2
    const int pointsPerThread2 = std::max(1, static_cast<int>(cloud2->size()) / numThreads);
    std::vector<QFuture<void>> futuresCloud2;

    for (int t = 0; t < numThreads; ++t) {
        int startIdx = t * pointsPerThread2;
        int endIdx = (t == numThreads - 1) ? cloud2->size() : (t + 1) * pointsPerThread2;

        futuresCloud2.push_back(QtConcurrent::run([this, &cloud2, &colors2, &kdtree1, thresholdSquared,
                                                   &pcl_result2, &collisionCount2, startIdx, endIdx]() {
            this->processCloudPoints(
                cloud2, colors2, kdtree1, thresholdSquared, pcl_result2,
                ORIGINAL_COLOR_2_R, ORIGINAL_COLOR_2_G, ORIGINAL_COLOR_2_B,
                startIdx, endIdx, collisionCount2
                );
        }));
    }

    // Use a timer to update progress and process events while waiting
    progressValue = 60;
    while (!areFuturesFinished(futuresCloud2)) {
        // Increment progress
        if (progressValue < 80) {
            progressValue++;
            updateProgress(progress, progressValue);
        }

        // Sleep briefly to avoid CPU hogging
        QThread::msleep(50);

        // Process UI events on the main thread
        QCoreApplication::processEvents();
    }

    updateProgress(progress, 80);
    // Allow UI to refresh
    QCoreApplication::processEvents();

    // Convert results back to Qt format
    result1.points.reserve(pcl_result1->size());
    result1.colors.reserve(pcl_result1->size());
    result2.points.reserve(pcl_result2->size());
    result2.colors.reserve(pcl_result2->size());

    // Use smaller batch sizes to avoid UI blocking
    const int batchSize = 10000; // Process points in smaller batches

    // Process first cloud results
    for (size_t i = 0; i < pcl_result1->size(); i += batchSize) {
        const size_t end = std::min(i + batchSize, pcl_result1->size());

        for (size_t j = i; j < end; ++j) {
            const auto& p = pcl_result1->points[j];
            result1.points.push_back(QVector3D(p.x, p.y, p.z));
            result1.colors.push_back(QVector3D(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f));
        }

        // Process events after each batch
        if (i % (batchSize * 5) == 0) {
            QCoreApplication::processEvents();
        }
    }

    // Process second cloud results
    for (size_t i = 0; i < pcl_result2->size(); i += batchSize) {
        const size_t end = std::min(i + batchSize, pcl_result2->size());

        for (size_t j = i; j < end; ++j) {
            const auto& p = pcl_result2->points[j];
            result2.points.push_back(QVector3D(p.x, p.y, p.z));
            result2.colors.push_back(QVector3D(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f));
        }

        // Process events after each batch
        if (i % (batchSize * 5) == 0) {
            QCoreApplication::processEvents();
        }
    }

    // Set collision counts
    result1.collisionCount = collisionCount1;
    result2.collisionCount = collisionCount2;

    updateProgress(progress, 100);

    return true;
}

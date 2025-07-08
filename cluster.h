#ifndef CLUSTER_H
#define CLUSTER_H

#include <QDialog>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QDialogButtonBox>
#include <QVector3D>
#include <QVector>
#include <Eigen/Dense>
#include <cilantro/clustering/kmeans.hpp>
#include <cilantro/utilities/point_cloud.hpp>
#include <cilantro/core/kd_tree.hpp>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <random>
#include <thread>
#include <atomic>

struct Vertex {
    QVector3D position;
    // Add other attributes if needed (e.g., color, normal)
};

// Enhanced struct to hold cluster information
struct ClusterInfo {
    int label;
    QVector3D centroid;
    size_t pointCount;
    Eigen::Vector3f extent; // Bounding box extent
    float density;          // Points per unit volume
};

class ClusteringParametersDialog : public QDialog {
    Q_OBJECT
public:
    explicit ClusteringParametersDialog(QWidget* parent = nullptr);

    float getVoxelSize() const { return voxelSizeSpinBox->value(); }
    float getSampleFraction() const { return sampleFractionSpinBox->value(); }
    float getPlaneDistanceThreshold() const { return planeDistanceThresholdSpinBox->value(); }
    float getClusterEps() const { return clusterEpsSpinBox->value(); }
    size_t getMaxPlaneIterations() const { return maxPlaneIterationsSpinBox->value(); }
    size_t getClusterMinPts() const { return clusterMinPtsSpinBox->value(); }
    float getMinPlaneInlierRatio() const { return minPlaneInlierRatioSpinBox->value(); }

signals:
    void clusteringParametersConfirmed(float voxelSize, float sampleFraction, float planeDistanceThreshold,
                                       float clusterEps, size_t maxPlaneIterations, size_t clusterMinPts,
                                       float minPlaneInlierRatio);

private slots:
    void on_okButton_clicked();
    void on_cancelButton_clicked();

private:
    QDoubleSpinBox* voxelSizeSpinBox;
    QDoubleSpinBox* sampleFractionSpinBox;
    QDoubleSpinBox* planeDistanceThresholdSpinBox;
    QDoubleSpinBox* clusterEpsSpinBox;
    QSpinBox* maxPlaneIterationsSpinBox;
    QSpinBox* clusterMinPtsSpinBox;
    QDoubleSpinBox* minPlaneInlierRatioSpinBox;
};

class ProgressDialog : public QDialog {
    Q_OBJECT
public:
    explicit ProgressDialog(QWidget* parent = nullptr);

public slots:
    void updateProgress(int value);

private:
    QProgressBar* progressBar;
};

class ClusterResultsDialog : public QDialog {
    Q_OBJECT
public:
    explicit ClusterResultsDialog(const std::vector<ClusterInfo>& clusters, QWidget* parent = nullptr);
};

QVector<Vertex> downsamplePointCloud(const QVector<Vertex>& inputCloud, float voxelSize, float sampleFraction);
std::pair<QVector<Vertex>, QVector<Vertex>> detectPlane(const QVector<Vertex>& cloud, float distanceThreshold,
                                                        size_t maxPlaneIterations, float minPlaneInlierRatio);
std::vector<int> performDensityClustering(const QVector<Vertex>& points, float eps, size_t minPts,
                                          std::atomic<int>& progress);
void processPointCloudClustering(QVector<Vertex>& combinedCloud, std::vector<int>& combinedLabels,
                                 float voxelSize, float sampleFraction, float planeDistanceThreshold,
                                 float clusterEps, size_t maxPlaneIterations, size_t clusterMinPts,
                                 float minPlaneInlierRatio, Eigen::Vector4f& planeModel,
                                 std::vector<ClusterInfo>& clusterInfos, std::atomic<int>& progress);

#endif // CLUSTER_H
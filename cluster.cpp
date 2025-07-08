#include "cluster.h"
#include <QFormLayout>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <iostream>
#include <iomanip>
#include <unordered_set>
#include <cmath>

// Custom hash function for std::tuple<int, int, int>
namespace std {
template <>
struct hash<std::tuple<int, int, int>> {
    size_t operator()(const std::tuple<int, int, int>& t) const {
        size_t h1 = std::hash<int>{}(std::get<0>(t));
        size_t h2 = std::hash<int>{}(std::get<1>(t));
        size_t h3 = std::hash<int>{}(std::get<2>(t));
        return h1 ^ (h2 << 1) ^ (h3 << 2); // Combine hashes
    }
};
}

ClusteringParametersDialog::ClusteringParametersDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Clustering Parameters"));

    QFormLayout* formLayout = new QFormLayout;

    voxelSizeSpinBox = new QDoubleSpinBox(this);
    voxelSizeSpinBox->setRange(0.01, 1.0);
    voxelSizeSpinBox->setSingleStep(0.01);
    voxelSizeSpinBox->setValue(0.1);
    formLayout->addRow(tr("Voxel Size:"), voxelSizeSpinBox);

    sampleFractionSpinBox = new QDoubleSpinBox(this);
    sampleFractionSpinBox->setRange(0.01, 1.0);
    sampleFractionSpinBox->setSingleStep(0.01);
    sampleFractionSpinBox->setValue(0.1);
    formLayout->addRow(tr("Sample Fraction:"), sampleFractionSpinBox);

    planeDistanceThresholdSpinBox = new QDoubleSpinBox(this);
    planeDistanceThresholdSpinBox->setRange(0.001, 1.0);
    planeDistanceThresholdSpinBox->setSingleStep(0.001);
    planeDistanceThresholdSpinBox->setValue(0.02);
    formLayout->addRow(tr("Plane Distance Threshold:"), planeDistanceThresholdSpinBox);

    clusterEpsSpinBox = new QDoubleSpinBox(this);
    clusterEpsSpinBox->setRange(0.01, 1.0);
    clusterEpsSpinBox->setSingleStep(0.01);
    clusterEpsSpinBox->setValue(0.1);
    formLayout->addRow(tr("Cluster Epsilon:"), clusterEpsSpinBox);

    maxPlaneIterationsSpinBox = new QSpinBox(this);
    maxPlaneIterationsSpinBox->setRange(100, 10000);
    maxPlaneIterationsSpinBox->setSingleStep(100);
    maxPlaneIterationsSpinBox->setValue(1000);
    formLayout->addRow(tr("Max Plane Iterations:"), maxPlaneIterationsSpinBox);

    clusterMinPtsSpinBox = new QSpinBox(this);
    clusterMinPtsSpinBox->setRange(3, 100);
    clusterMinPtsSpinBox->setValue(10);
    formLayout->addRow(tr("Cluster Min Points:"), clusterMinPtsSpinBox);

    minPlaneInlierRatioSpinBox = new QDoubleSpinBox(this);
    minPlaneInlierRatioSpinBox->setRange(0.05, 0.5);
    minPlaneInlierRatioSpinBox->setSingleStep(0.01);
    minPlaneInlierRatioSpinBox->setValue(0.1);
    formLayout->addRow(tr("Min Plane Inlier Ratio:"), minPlaneInlierRatioSpinBox);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(Qt::Horizontal, this);
    QPushButton* okButton = buttonBox->addButton(QDialogButtonBox::Ok);
    QPushButton* cancelButton = buttonBox->addButton(QDialogButtonBox::Cancel);
    formLayout->addRow(buttonBox);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addLayout(formLayout);

    connect(okButton, &QPushButton::clicked, this, &ClusteringParametersDialog::on_okButton_clicked);
    connect(cancelButton, &QPushButton::clicked, this, &ClusteringParametersDialog::on_cancelButton_clicked);
}

void ClusteringParametersDialog::on_okButton_clicked()
{
    emit clusteringParametersConfirmed(
        static_cast<float>(voxelSizeSpinBox->value()),
        static_cast<float>(sampleFractionSpinBox->value()),
        static_cast<float>(planeDistanceThresholdSpinBox->value()),
        static_cast<float>(clusterEpsSpinBox->value()),
        maxPlaneIterationsSpinBox->value(),
        clusterMinPtsSpinBox->value(),
        static_cast<float>(minPlaneInlierRatioSpinBox->value())
        );
    accept();
}

void ClusteringParametersDialog::on_cancelButton_clicked()
{
    reject();
}

ProgressDialog::ProgressDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Clustering Progress"));
    setWindowFlags(Qt::Window | Qt::WindowTitleHint | Qt::CustomizeWindowHint);
    setModal(true);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    QLabel* label = new QLabel(tr("Processing point cloud clustering..."), this);
    progressBar = new QProgressBar(this);
    progressBar->setRange(0, 100);
    progressBar->setValue(0);

    mainLayout->addWidget(label);
    mainLayout->addWidget(progressBar);
    mainLayout->addStretch();
}

void ProgressDialog::updateProgress(int value)
{
    progressBar->setValue(value);
    if (value >= 100) {
        accept();
    }
}

ClusterResultsDialog::ClusterResultsDialog(const std::vector<ClusterInfo>& clusters, QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Cluster Coordinates"));

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    QTextEdit* textEdit = new QTextEdit(this);
    textEdit->setReadOnly(true);

    QString text;
    text += "Cluster Information:\n\n";
    for (const auto& cluster : clusters) {
        text += QString("Cluster %1:\n").arg(cluster.label);
        text += QString("  Centroid: (%2, %3, %4)\n")
                    .arg(cluster.centroid.x(), 0, 'f', 3)
                    .arg(cluster.centroid.y(), 0, 'f', 3)
                    .arg(cluster.centroid.z(), 0, 'f', 3);
        text += QString("  Point Count: %1\n").arg(cluster.pointCount);
        text += QString("  Extent: (%2, %3, %4)\n")
                    .arg(cluster.extent.x(), 0, 'f', 3)
                    .arg(cluster.extent.y(), 0, 'f', 3)
                    .arg(cluster.extent.z(), 0, 'f', 3);
        text += QString("  Density: %1 points/unit\n\n").arg(cluster.density, 0, 'f', 3);
    }

    textEdit->setText(text);
    mainLayout->addWidget(textEdit);

    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok, this);
    mainLayout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
}

QVector<Vertex> downsamplePointCloud(const QVector<Vertex>& inputCloud, float voxelSize, float sampleFraction)
{
    std::cout << "Downsampling point cloud with voxel size " << voxelSize << " and sample fraction " << sampleFraction << "..." << std::endl;
    if (inputCloud.isEmpty()) {
        std::cerr << "Error: Empty input cloud for downsampling" << std::endl;
        return QVector<Vertex>();
    }

    try {
        // Voxel grid downsampling
        std::unordered_map<std::tuple<int, int, int>, std::vector<size_t>> voxelMap;
        float invVoxelSize = 1.0f / voxelSize;

        for (size_t i = 0; i < static_cast<size_t>(inputCloud.size()); ++i) {
            const auto& pos = inputCloud[i].position;
            auto voxel = std::make_tuple(
                static_cast<int>(std::floor(pos.x() * invVoxelSize)),
                static_cast<int>(std::floor(pos.y() * invVoxelSize)),
                static_cast<int>(std::floor(pos.z() * invVoxelSize))
                );
            voxelMap[voxel].push_back(i);
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        size_t targetSize = static_cast<size_t>(inputCloud.size() * sampleFraction);
        QVector<Vertex> downsampledCloud;
        downsampledCloud.reserve(targetSize);

        for (const auto& voxel : voxelMap) {
            const auto& indices = voxel.second;
            size_t numToSample = std::max<size_t>(1, static_cast<size_t>(indices.size() * sampleFraction));
            std::vector<size_t> sampledIndices(indices.begin(), indices.end());
            std::shuffle(sampledIndices.begin(), sampledIndices.end(), gen);
            for (size_t i = 0; i < numToSample && i < indices.size(); ++i) {
                downsampledCloud.push_back(inputCloud[sampledIndices[i]]);
            }
        }

        std::cout << "Downsampled from " << inputCloud.size() << " to " << downsampledCloud.size() << " points" << std::endl;
        return downsampledCloud;
    } catch (const std::exception& e) {
        std::cerr << "Exception in downsamplePointCloud: " << e.what() << std::endl;
        return QVector<Vertex>();
    }
}

std::pair<QVector<Vertex>, QVector<Vertex>> detectPlane(const QVector<Vertex>& cloud, float distanceThreshold,
                                                        size_t maxPlaneIterations, float minPlaneInlierRatio)
{
    std::cout << "Detecting plane with RANSAC, threshold=" << distanceThreshold << ", max iterations=" << maxPlaneIterations << "..." << std::endl;
    if (cloud.size() < 10) {
        std::cerr << "Error: Too few points (" << cloud.size() << ") to detect a plane" << std::endl;
        return {QVector<Vertex>(), cloud};
    }

    try {
        Eigen::Matrix<float, 3, Eigen::Dynamic> points(3, cloud.size());
        for (size_t i = 0; i < static_cast<size_t>(cloud.size()); ++i) {
            points.col(i) = Eigen::Vector3f(cloud[i].position.x(), cloud[i].position.y(), cloud[i].position.z());
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dist(0, cloud.size() - 1);
        std::vector<size_t> bestInliers;
        Eigen::Vector4f bestPlaneModel;
        size_t minInliers = static_cast<size_t>(cloud.size() * minPlaneInlierRatio);
        size_t maxIter = maxPlaneIterations;
        size_t iter = 0;

        // Adaptive iteration count based on inlier ratio
        double confidence = 0.99;
        double inlierRatio = 0.1;
        size_t adaptiveMaxIter = maxPlaneIterations;

        while (iter < adaptiveMaxIter && iter < maxPlaneIterations) {
            std::vector<size_t> sampleIndices;
            sampleIndices.reserve(3);
            while (sampleIndices.size() < 3) {
                size_t idx = dist(gen);
                if (std::find(sampleIndices.begin(), sampleIndices.end(), idx) == sampleIndices.end()) {
                    sampleIndices.push_back(idx);
                }
            }

            Eigen::Vector3f p1 = points.col(sampleIndices[0]);
            Eigen::Vector3f p2 = points.col(sampleIndices[1]);
            Eigen::Vector3f p3 = points.col(sampleIndices[2]);

            Eigen::Vector3f v1 = p2 - p1;
            Eigen::Vector3f v2 = p3 - p1;
            Eigen::Vector3f normal = v1.cross(v2).normalized();
            if (normal.norm() < 1e-6f) continue;
            float d = -normal.dot(p1);
            Eigen::Vector4f planeModel(normal.x(), normal.y(), normal.z(), d);

            std::vector<size_t> inliers;
            inliers.reserve(cloud.size());
            for (size_t i = 0; i < static_cast<size_t>(cloud.size()); ++i) {
                float distance = std::abs(points.col(i).dot(normal) + d);
                if (distance < distanceThreshold) {
                    inliers.push_back(i);
                }
            }

            if (inliers.size() > bestInliers.size()) {
                bestInliers = inliers;
                bestPlaneModel = planeModel;
                // Update adaptive iteration count
                if (inliers.size() > minInliers) {
                    inlierRatio = static_cast<double>(inliers.size()) / cloud.size();
                    double logTerm = std::log(1.0 - confidence) / std::log(1.0 - std::pow(inlierRatio, 3));
                    adaptiveMaxIter = std::min(maxPlaneIterations, static_cast<size_t>(std::ceil(logTerm)));
                }
            }

            iter++;
        }

        if (bestInliers.size() < minInliers) {
            std::cerr << "Warning: Insufficient inliers (" << bestInliers.size() << " < " << minInliers << "). Plane detection failed." << std::endl;
            return {QVector<Vertex>(), cloud};
        }

        std::cout << "Detected " << bestInliers.size() << " inliers after " << iter << " iterations." << std::endl;

        QVector<Vertex> planeCloud(bestInliers.size());
        QVector<Vertex> nonPlaneCloud(cloud.size() - bestInliers.size());
        std::vector<bool> isInlier(cloud.size(), false);

        size_t planeIdx = 0;
        for (size_t i : bestInliers) {
            planeCloud[planeIdx] = cloud[i];
            isInlier[i] = true;
            planeIdx++;
        }

        size_t nonPlaneIdx = 0;
        for (size_t i = 0; i < cloud.size(); ++i) {
            if (!isInlier[i]) {
                nonPlaneCloud[nonPlaneIdx] = cloud[i];
                nonPlaneIdx++;
            }
        }

        return {planeCloud, nonPlaneCloud};
    } catch (const std::exception& e) {
        std::cerr << "Exception in detectPlane: " << e.what() << std::endl;
        return {QVector<Vertex>(), cloud};
    }
}

std::vector<int> performDensityClustering(const QVector<Vertex>& points, float eps, size_t minPts, std::atomic<int>& progress)
{
    std::cout << "Performing optimized density-based clustering with eps=" << eps << ", minPts=" << minPts << "..." << std::endl;
    if (points.size() < minPts) {
        std::cerr << "Error: Too few points (" << points.size() << ") for clustering with minPts=" << minPts << std::endl;
        return std::vector<int>(points.size(), -1);
    }

    try {
        cilantro::VectorSet<float, 3> pointData(3, points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            pointData.col(i) = Eigen::Vector3f(points[i].position.x(), points[i].position.y(), points[i].position.z());
        }

        cilantro::KDTree<float, 3> kdtree(pointData);
        std::vector<int> labels(points.size(), -1);
        std::vector<bool> visited(points.size(), false);
        size_t clusterId = 0;
        progress = 0;

        // Parallel processing of core points
        std::vector<size_t> corePoints;
        corePoints.reserve(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            if (!visited[i]) {
                cilantro::NeighborSet<float> neighbors;
                kdtree.radiusSearch(pointData.col(i), eps, neighbors);
                if (neighbors.size() >= minPts) {
                    corePoints.push_back(i);
                }
            }
        }

        for (size_t idx = 0; idx < corePoints.size(); ++idx) {
            size_t i = corePoints[idx];
            if (visited[i]) continue;

            visited[i] = true;
            labels[i] = static_cast<int>(clusterId);
            std::queue<size_t> seedSet;

            cilantro::NeighborSet<float> neighbors;
            kdtree.radiusSearch(pointData.col(i), eps, neighbors);
            for (const auto& nn : neighbors) {
                if (nn.index != i && !visited[nn.index]) seedSet.push(nn.index);
            }

            while (!seedSet.empty()) {
                size_t currentIdx = seedSet.front();
                seedSet.pop();

                if (visited[currentIdx]) continue;
                visited[currentIdx] = true;

                cilantro::NeighborSet<float> currentNeighbors;
                kdtree.radiusSearch(pointData.col(currentIdx), eps, currentNeighbors);

                if (currentNeighbors.size() >= minPts) {
                    labels[currentIdx] = static_cast<int>(clusterId);
                    for (const auto& nn : currentNeighbors) {
                        if (!visited[nn.index]) seedSet.push(nn.index);
                    }
                } else {
                    labels[currentIdx] = static_cast<int>(clusterId);
                }
            }

            clusterId++;
            progress = static_cast<int>((idx + 1) * 100 / corePoints.size());
        }

        std::cout << "Density clustering completed with " << clusterId << " clusters" << std::endl;
        return labels;
    } catch (const std::exception& e) {
        std::cerr << "Exception in performDensityClustering: " << e.what() << std::endl;
        return std::vector<int>(points.size(), -1);
    }
}

void processPointCloudClustering(QVector<Vertex>& combinedCloud, std::vector<int>& combinedLabels,
                                 float voxelSize, float sampleFraction, float planeDistanceThreshold,
                                 float clusterEps, size_t maxPlaneIterations, size_t clusterMinPts,
                                 float minPlaneInlierRatio, Eigen::Vector4f& planeModel,
                                 std::vector<ClusterInfo>& clusterInfos, std::atomic<int>& progress)
{
    if (combinedCloud.isEmpty()) {
        std::cerr << "Error: Empty point cloud in processPointCloudClustering" << std::endl;
        combinedLabels.clear();
        return;
    }

    try {
        progress = 10;
        QVector<Vertex> downsampledCloud = downsamplePointCloud(combinedCloud, voxelSize, sampleFraction);
        if (downsampledCloud.isEmpty()) {
            std::cerr << "Error: Downsampling resulted in an empty cloud" << std::endl;
            combinedLabels.assign(combinedCloud.size(), -1);
            return;
        }
        progress = 30;

        auto [planeCloud, nonPlaneCloud] = detectPlane(downsampledCloud, planeDistanceThreshold, maxPlaneIterations, minPlaneInlierRatio);
        progress = 50;

        std::cout << "Plane cloud size: " << planeCloud.size() << ", Non-plane cloud size: " << nonPlaneCloud.size() << std::endl;

        if (planeCloud.size() > 0) {
            Eigen::Matrix<float, 3, Eigen::Dynamic> points(3, planeCloud.size());
            Eigen::Vector3f minBounds(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
            Eigen::Vector3f maxBounds(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

            for (size_t i = 0; i < planeCloud.size(); ++i) {
                points.col(i) = Eigen::Vector3f(planeCloud[i].position.x(), planeCloud[i].position.y(), planeCloud[i].position.z());
                minBounds = minBounds.cwiseMin(points.col(i));
                maxBounds = maxBounds.cwiseMax(points.col(i));
            }
            Eigen::Vector3f centroid = points.rowwise().mean();
            Eigen::Matrix3f covMat = (points.colwise() - centroid) * (points.colwise() - centroid).transpose();
            Eigen::JacobiSVD<Eigen::Matrix3f> svd(covMat, Eigen::ComputeFullU);
            Eigen::Vector3f normal = svd.matrixU().col(2).normalized();
            float d = -normal.dot(centroid);
            planeModel = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d);
            std::cout << "Plane model computed: [" << normal.x() << ", " << normal.y() << ", " << normal.z() << ", " << d << "]" << std::endl;

            ClusterInfo planeCluster;
            planeCluster.label = 0;
            planeCluster.centroid = QVector3D(centroid.x(), centroid.y(), centroid.z());
            planeCluster.pointCount = planeCloud.size();
            planeCluster.extent = maxBounds - minBounds;
            float volume = planeCluster.extent.x() * planeCluster.extent.y() * planeCluster.extent.z();
            planeCluster.density = volume > 0 ? planeCloud.size() / volume : 0;
            clusterInfos.push_back(planeCluster);
        } else {
            planeModel = Eigen::Vector4f(0, 0, 1, 0);
            std::cerr << "Plane detection failed, using default plane model" << std::endl;
        }
        progress = 70;

        std::vector<int> densityLabels;
        if (nonPlaneCloud.size() >= clusterMinPts) {
            densityLabels = performDensityClustering(nonPlaneCloud, clusterEps, clusterMinPts, progress);
        } else {
            densityLabels.assign(nonPlaneCloud.size(), -1);
            std::cerr << "Warning: Non-plane cloud too small (" << nonPlaneCloud.size() << ") for density clustering" << std::endl;
        }
        progress = 90;

        // Compute enhanced cluster statistics
        std::unordered_map<int, std::vector<QVector3D>> clusterPoints;
        std::unordered_map<int, Eigen::Vector3f> minBounds, maxBounds;
        for (size_t i = 0; i < nonPlaneCloud.size(); ++i) {
            int label = densityLabels[i];
            if (label >= 0) {
                clusterPoints[label + 1].push_back(nonPlaneCloud[i].position);
                Eigen::Vector3f pt(nonPlaneCloud[i].position.x(), nonPlaneCloud[i].position.y(), nonPlaneCloud[i].position.z());
                if (minBounds.find(label + 1) == minBounds.end()) {
                    minBounds[label + 1] = pt;
                    maxBounds[label + 1] = pt;
                } else {
                    minBounds[label + 1] = minBounds[label + 1].cwiseMin(pt);
                    maxBounds[label + 1] = maxBounds[label + 1].cwiseMax(pt);
                }
            }
        }

        for (const auto& pair : clusterPoints) {
            ClusterInfo cluster;
            cluster.label = pair.first;
            Eigen::Vector3f sum(0, 0, 0);
            size_t count = pair.second.size();
            for (const auto& point : pair.second) {
                sum += Eigen::Vector3f(point.x(), point.y(), point.z());
            }
            if (count > 0) {
                sum /= static_cast<float>(count);
                cluster.centroid = QVector3D(sum.x(), sum.y(), sum.z());
                cluster.pointCount = count;
                cluster.extent = maxBounds[pair.first] - minBounds[pair.first];
                float volume = cluster.extent.x() * cluster.extent.y() * cluster.extent.z();
                cluster.density = volume > 0 ? count / volume : 0;
                clusterInfos.push_back(cluster);
            }
        }

        std::mutex dataMutex;
        std::lock_guard<std::mutex> lock(dataMutex);
        combinedCloud.resize(planeCloud.size() + nonPlaneCloud.size());
        combinedLabels.resize(combinedCloud.size(), -1);

        for (size_t i = 0; i < planeCloud.size(); ++i) {
            combinedCloud[i] = planeCloud[i];
            combinedLabels[i] = 0;
        }

        for (size_t i = 0; i < nonPlaneCloud.size(); ++i) {
            combinedCloud[planeCloud.size() + i] = nonPlaneCloud[i];
            combinedLabels[planeCloud.size() + i] = (densityLabels[i] >= 0) ? densityLabels[i] + 1 : -1;
        }

        std::unordered_map<int, size_t> clusterSizes;
        for (int label : combinedLabels) {
            if (label >= 0) clusterSizes[label]++;
        }
        std::cout << "Cluster sizes after processing:" << std::endl;
        for (const auto& pair : clusterSizes) {
            std::cout << "Cluster " << pair.first << ": " << pair.second << " points" << std::endl;
        }
        progress = 100;
    } catch (const std::exception& e) {
        std::cerr << "Exception in processPointCloudClustering: " << e.what() << std::endl;
        combinedLabels.assign(combinedCloud.size(), -1);
    }
}
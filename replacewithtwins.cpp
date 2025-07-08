#include "replacewithtwins.h"
#include <QMessageBox>
#include <QDir>
#include <QDebug>
#include <limits>
#include <QFileInfo>
#include <random>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <cmath>

// Utility function to compute the bounding box of a PLY model
bool computeModelBoundingBox(const QString& filePath, QVector3D& minBounds, QVector3D& maxBounds) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filePath.toStdString(),
        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);
    
    if (!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) {
        qDebug() << "Failed to load model for bounding box computation:" << filePath;
        return false;
    }

    minBounds = QVector3D(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    maxBounds = QVector3D(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

    std::function<void(const aiNode*, const aiMatrix4x4&)> processNode = [&](const aiNode* node, const aiMatrix4x4& parentTransform) {
        aiMatrix4x4 transform = parentTransform * node->mTransformation;
        
        for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
            const aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
            for (unsigned int j = 0; j < mesh->mNumVertices; ++j) {
                aiVector3D vertex = mesh->mVertices[j];
                vertex = transform * vertex;
                minBounds.setX(std::min(minBounds.x(), vertex.x));
                minBounds.setY(std::min(minBounds.y(), vertex.y));
                minBounds.setZ(std::min(minBounds.z(), vertex.z));
                maxBounds.setX(std::max(maxBounds.x(), vertex.x));
                maxBounds.setY(std::max(maxBounds.y(), vertex.y));
                maxBounds.setZ(std::max(maxBounds.z(), vertex.z));
            }
        }
        
        for (unsigned int i = 0; i < node->mNumChildren; ++i) {
            processNode(node->mChildren[i], transform);
        }
    };

    processNode(scene->mRootNode, aiMatrix4x4());
    
    return (maxBounds - minBounds).length() > 0.0f;
}

// Utility function to compute PCA-based orientation and dimension-based uniform scaling
void computePCAScaling(const std::vector<QVector3D>& points, const QString& modelFilePath, 
                      QVector3D& centroid, float& uniformScale, QMatrix4x4& rotationMatrix,
                      QVector3D& clusterDimensions, QVector3D& twinDimensions, QVector3D& scaledTwinDimensions) {
    // Compute centroid
    centroid = QVector3D(0, 0, 0);
    for (const auto& pt : points) {
        centroid += pt;
    }
    if (!points.empty()) {
        centroid /= points.size();
    }

    // Compute cluster bounding box
    QVector3D minBounds(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    QVector3D maxBounds(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    for (const auto& pt : points) {
        minBounds.setX(std::min(minBounds.x(), pt.x()));
        minBounds.setY(std::min(minBounds.y(), pt.y()));
        minBounds.setZ(std::min(minBounds.z(), pt.z()));
        maxBounds.setX(std::max(maxBounds.x(), pt.x()));
        maxBounds.setY(std::max(maxBounds.y(), pt.y()));
        maxBounds.setZ(std::max(maxBounds.z(), pt.z()));
    }
    clusterDimensions = maxBounds - minBounds;

    // Compute model bounding box
    QVector3D modelMin, modelMax;
    if (!computeModelBoundingBox(modelFilePath, modelMin, modelMax)) {
        qDebug() << "Failed to compute model dimensions, using default scale";
        uniformScale = 1.0f;
        twinDimensions = QVector3D(1.0f, 1.0f, 1.0f);
        scaledTwinDimensions = twinDimensions;
    } else {
        twinDimensions = modelMax - modelMin;

        // Compute uniform scale using the largest dimension to fit within cluster
        float scaleX = clusterDimensions.x() / twinDimensions.x();
        float scaleY = clusterDimensions.y() / twinDimensions.y();
        float scaleZ = clusterDimensions.z() / twinDimensions.z();
        uniformScale = std::min({scaleX, scaleY, scaleZ}); // Use the smallest ratio to ensure fit

        // Apply a scaling multiplier to adjust size
        const float scaleMultiplier = 0.5f; // Adjust size to be slightly smaller
        uniformScale *= scaleMultiplier;

        uniformScale = std::max(uniformScale, 0.01f); // Ensure minimum scale
        scaledTwinDimensions = twinDimensions * uniformScale;
    }

    // Compute PCA for orientation
    Eigen::MatrixXd data(3, points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        QVector3D centered = points[i] - centroid;
        data(0, i) = centered.x();
        data(1, i) = centered.y();
        data(2, i) = centered.z();
    }

    Eigen::Matrix3d cov = (data * data.transpose()) / static_cast<double>(points.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    // Create rotation matrix from eigenvectors
    rotationMatrix.setToIdentity();
    for (int i = 0; i < 3; ++i) {
        rotationMatrix(i, 0) = eigenvectors(i, 2); // Principal axis
        rotationMatrix(i, 1) = eigenvectors(i, 1); // Secondary axis
        rotationMatrix(i, 2) = eigenvectors(i, 0); // Tertiary axis
    }

    // Adjust rotation to align with the ground plane (Z-axis up)
    QMatrix4x4 alignToGround;
    alignToGround.setToIdentity();
    alignToGround.rotate(90, QVector3D(1, 0, 0)); // Rotate to align model's up with Z-axis
    rotationMatrix = alignToGround * rotationMatrix;
}

std::map<std::string, std::vector<QVector3D>> computeObjectPositions(
    const std::map<std::string, int>& clusterCounts,
    const std::vector<int>& labels,
    const QVector<Vertex>& points,
    std::map<std::string, std::vector<int>>& outClassLabels,
    std::map<std::string, std::map<int, std::vector<QVector3D>>>& outClassClusterPoints
) {
    std::map<std::string, std::vector<QVector3D>> objectPositions;
    std::map<int, std::vector<QVector3D>> labelPoints;

    // Group points by label
    for (size_t i = 0; i < labels.size() && i < points.size(); ++i) {
        if (labels[i] >= 0) {
            labelPoints[labels[i]].push_back(points[i].position);
        }
    }

    // Log all detected clusters
    qDebug() << "Detected clusters (label, point count):";
    for (const auto& [label, pts] : labelPoints) {
        qDebug() << "Label:" << label << "Points:" << pts.size();
    }

    // Assign labels to classes based on clusterCounts
    std::map<std::string, int> remainingCounts = clusterCounts;
    std::map<int, std::string> labelToClass;

    // Sort labels by point count to prioritize larger clusters
    std::vector<std::pair<int, size_t>> labelSizes;
    for (const auto& [label, pts] : labelPoints) {
        labelSizes.emplace_back(label, pts.size());
    }
    std::sort(labelSizes.begin(), labelSizes.end(), 
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Assign clusters to classes
    for (const auto& [label, size] : labelSizes) {
        std::string bestClass;
        int maxCount = -1;
        for (const auto& [className, count] : remainingCounts) {
            if (count > 0 && count > maxCount) {
                maxCount = count;
                bestClass = className;
            }
        }
        if (!bestClass.empty()) {
            labelToClass[label] = bestClass;
            outClassLabels[bestClass].push_back(label);
            outClassClusterPoints[bestClass][label] = labelPoints[label];
            remainingCounts[bestClass]--;
            qDebug() << "Assigned label:" << label << "to class:" << QString::fromStdString(bestClass);
        } else {
            qDebug() << "No class available for label:" << label << ", skipping.";
        }
    }

    // Compute centroids for all assigned clusters
    for (const auto& [label, className] : labelToClass) {
        const auto& pts = labelPoints[label];
        if (pts.empty()) {
            qDebug() << "Empty point set for label:" << label << "class:" << QString::fromStdString(className) << ", skipping.";
            continue;
        }
        QVector3D centroid(0, 0, 0);
        for (const auto& pt : pts) {
            centroid += pt;
        }
        centroid /= pts.size();
        objectPositions[className].push_back(centroid);
        qDebug() << "Computed centroid for class:" << QString::fromStdString(className) 
                 << "label:" << label << "at:" << centroid;
    }

    // Handle additional instances required by clusterCounts
    for (const auto& [className, count] : clusterCounts) {
        int currentCount = objectPositions[className].size();
        if (currentCount < count) {
            qDebug() << "Need" << (count - currentCount) << "additional positions for class:" << QString::fromStdString(className);
            // Compute class bounding box for additional placements
            QVector3D classMinBounds(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
            QVector3D classMaxBounds(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
            for (const auto& [label, pts] : outClassClusterPoints[className]) {
                for (const auto& pt : pts) {
                    classMinBounds.setX(std::min(classMinBounds.x(), pt.x()));
                    classMinBounds.setY(std::min(classMinBounds.y(), pt.y()));
                    classMinBounds.setZ(std::min(classMinBounds.z(), pt.z()));
                    classMaxBounds.setX(std::max(classMaxBounds.x(), pt.x()));
                    classMaxBounds.setY(std::max(classMaxBounds.y(), pt.y()));
                    classMaxBounds.setZ(std::max(classMaxBounds.z(), pt.z()));
                }
            }
            QVector3D classDimensions = classMaxBounds - classMinBounds;
            QVector3D classCenter = (classMinBounds + classMaxBounds) / 2.0f;
            int additionalCount = count - currentCount;

            // Place additional instances in a grid pattern
            int gridSize = std::ceil(std::sqrt(static_cast<float>(additionalCount)));
            float stepX = classDimensions.x() / (gridSize + 1);
            float stepY = classDimensions.y() / (gridSize + 1);
            int placed = 0;
            for (int i = 0; i < gridSize && placed < additionalCount; ++i) {
                for (int j = 0; j < gridSize && placed < additionalCount; ++j) {
                    float xOffset = (i + 1) * stepX - (classDimensions.x() / 2.0f);
                    float yOffset = (j + 1) * stepY - (classDimensions.y() / 2.0f);
                    QVector3D newPosition = classCenter + QVector3D(xOffset, yOffset, 0.0f);
                    newPosition.setX(std::max(classMinBounds.x(), std::min(classMaxBounds.x(), newPosition.x())));
                    newPosition.setY(std::max(classMinBounds.y(), std::min(classMaxBounds.y(), newPosition.y())));
                    newPosition.setZ(classMinBounds.z()); // Place on the ground plane
                    objectPositions[className].push_back(newPosition);
                    qDebug() << "Added additional position for class:" << QString::fromStdString(className) 
                             << "at:" << newPosition;
                    placed++;
                }
            }
        }
    }

    // Final verification
    for (const auto& [className, positions] : objectPositions) {
        qDebug() << "Class:" << QString::fromStdString(className) 
                 << "has" << positions.size() << "objects at positions:";
        for (const auto& pos : positions) {
            qDebug() << "  Position:" << pos;
        }
    }

    return objectPositions;
}

ReplaceWithTwinsWorker::ReplaceWithTwinsWorker(const QString& twinsFolder, const std::map<std::string, int>& clusterCounts,
                                             const std::vector<int>& labels, const QVector<Vertex>& points, OpenGLWidget* glWidget,
                                             QObject* parent)
    : QObject(parent), m_twinsFolder(twinsFolder), m_clusterCounts(clusterCounts), m_labels(labels), m_points(points), m_glWidget(glWidget) {
}

ReplaceWithTwinsWorker::~ReplaceWithTwinsWorker() {
}

void ReplaceWithTwinsWorker::process() {
    try {
        // Validate twins folder
        QDir twinsDir(m_twinsFolder);
        if (!twinsDir.exists()) {
            emit errorOccurred(QString("Twins folder does not exist: %1").arg(m_twinsFolder));
            return;
        }

        emit progressUpdated(10); // Initial progress

        // Get list of PLY files in twins folder
        QStringList plyFiles = twinsDir.entryList(QStringList() << "*.ply" << "*.PLY", QDir::Files);
        if (plyFiles.isEmpty()) {
            emit errorOccurred("No PLY files found in the twins folder.");
            return;
        }

        // Map class names to PLY file paths (case-insensitive)
        std::map<std::string, QString> classToFile;
        for (const QString& file : plyFiles) {
            QString baseName = QFileInfo(file).baseName().toLower();
            classToFile[baseName.toStdString()] = twinsDir.absoluteFilePath(file);
            qDebug() << "Mapped class:" << baseName << "to file:" << classToFile[baseName.toStdString()];
        }

        emit progressUpdated(20); // After loading files

        // Compute object positions and clusters
        std::map<std::string, std::vector<int>> classLabels;
        std::map<std::string, std::map<int, std::vector<QVector3D>>> classClusterPoints;
        std::map<std::string, std::vector<QVector3D>> objectPositions = computeObjectPositions(
            m_clusterCounts, m_labels, m_points, classLabels, classClusterPoints
        );

        // Verify that we have positions for all expected objects
        int totalObjects = 0;
        for (const auto& [className, positions] : objectPositions) {
            totalObjects += positions.size();
        }
        int expectedTotal = 0;
        for (const auto& [className, count] : m_clusterCounts) {
            expectedTotal += count;
        }
        qDebug() << "Total objects found:" << totalObjects << "Expected:" << expectedTotal;

        // Copy original point cloud to the new viewer
        OpenGLWidget::PointCloud pc;
        pc.name = "TempPointCloud";
        pc.points.reserve(m_points.size());
        pc.colors.reserve(m_points.size());
        pc.labels = m_labels;
        pc.visible = true;
        pc.selected = false;
        pc.vboInitialized = false;
        pc.pointCount = m_points.size();
        for (const auto& vertex : m_points) {
            pc.points.push_back(vertex.position);
            pc.colors.push_back(QVector3D(1.0f, 1.0f, 1.0f)); // Default color
        }
        pc.updateBoundingBox();
        m_glWidget->m_pointClouds.push_back(pc);
        m_glWidget->updatePointCloudVBO(pc);

        // Store positions of placed twins
        std::map<std::string, std::vector<QVector3D>> twinPositions;

        // Count total clusters to process
        int totalClusters = 0;
        for (const auto& [className, labels] : classLabels) {
            totalClusters += labels.size();
        }
        for (const auto& [className, count] : m_clusterCounts) {
            totalClusters += std::max(0, count - static_cast<int>(classLabels[className].size()));
        }
        
        if (totalClusters == 0) {
            emit errorOccurred("No valid clusters or additional instances to process.");
            return;
        }

        qDebug() << "Total clusters to process (including additional instances):" << totalClusters;

        int processedClusters = 0;
        for (const auto& [className, labels] : classLabels) {
            QString classNameLower = QString::fromStdString(className).toLower();
            auto fileIt = classToFile.find(classNameLower.toStdString());
            if (fileIt == classToFile.end()) {
                qDebug() << "No PLY file for class:" << classNameLower << ", skipping.";
                continue;
            }

            QString twinFilePath = fileIt->second;
            int expectedCount = m_clusterCounts.at(className);
            
            qDebug() << "Processing class:" << classNameLower << "with expected count:" << expectedCount;

            // Compute the average uniform scale for this class
            float avgUniformScale = 0.0f;
            int validClustersForScaling = 0;
            std::vector<QVector3D> centroids;
            std::vector<QMatrix4x4> rotationMatrices;
            std::vector<QVector3D> clusterDimensionsList;
            std::vector<QVector3D> twinDimensionsList;
            std::vector<QVector3D> scaledTwinDimensionsList;
            
            for (size_t i = 0; i < labels.size(); ++i) {
                int clusterId = labels[i];
                auto clusterIt = classClusterPoints[className].find(clusterId);
                if (clusterIt == classClusterPoints[className].end() || clusterIt->second.empty()) {
                    qDebug() << "Empty point set for class:" << classNameLower << "cluster ID:" << clusterId << ", skipping scale computation.";
                    continue;
                }
                
                const auto& points = clusterIt->second;

                // Compute PCA-based orientation and dimension-based scaling
                QVector3D centroid, clusterDimensions, twinDimensions, scaledTwinDimensions;
                float uniformScale;
                QMatrix4x4 rotationMatrix;
                computePCAScaling(points, twinFilePath, centroid, uniformScale, rotationMatrix, 
                                clusterDimensions, twinDimensions, scaledTwinDimensions);

                avgUniformScale += uniformScale;
                validClustersForScaling++;
                centroids.push_back(centroid);
                rotationMatrices.push_back(rotationMatrix);
                clusterDimensionsList.push_back(clusterDimensions);
                twinDimensionsList.push_back(twinDimensions);
                scaledTwinDimensionsList.push_back(scaledTwinDimensions);
            }

            if (validClustersForScaling > 0) {
                avgUniformScale /= validClustersForScaling;
                qDebug() << "Computed average uniform scale for class" << classNameLower << ":" << avgUniformScale;
            } else {
                qDebug() << "No valid clusters for scaling in class:" << classNameLower << ", using default scale";
                avgUniformScale = 1.0f;
            }

            // Apply the average uniform scale to all twins of this class
            int processedCount = 0;
            for (size_t i = 0; i < labels.size() && i < objectPositions[className].size(); ++i) {
                if (processedCount >= expectedCount) {
                    break;
                }
                
                int clusterId = labels[i];
                auto clusterIt = classClusterPoints[className].find(clusterId);
                if (clusterIt == classClusterPoints[className].end() || clusterIt->second.empty()) {
                    qDebug() << "Empty point set for class:" << classNameLower << "cluster ID:" << clusterId << ", skipping.";
                    continue;
                }

                // Use the precomputed values
                QVector3D centroid = centroids[i];
                QMatrix4x4 rotationMatrix = rotationMatrices[i];
                QVector3D clusterDimensions = clusterDimensionsList[i];
                QVector3D twinDimensions = twinDimensionsList[i];
                QVector3D scaledTwinDimensions = twinDimensions * avgUniformScale;

                // Log size differences
                qDebug() << "Cluster dimensions (x, y, z):" << clusterDimensions.x() << "," 
                         << clusterDimensions.y() << "," << clusterDimensions.z();
                qDebug() << "Original twin dimensions (x, y, z):" << twinDimensions.x() << "," 
                         << twinDimensions.y() << "," << twinDimensions.z();
                qDebug() << "Scaled twin dimensions (x, y, z):" << scaledTwinDimensions.x() << "," 
                         << scaledTwinDimensions.y() << "," << scaledTwinDimensions.z();
                qDebug() << "Size differences after scaling (x, y, z):" 
                         << (clusterDimensions.x() - scaledTwinDimensions.x()) << "," 
                         << (clusterDimensions.y() - scaledTwinDimensions.y()) << "," 
                         << (clusterDimensions.z() - scaledTwinDimensions.z());

                // Extract rotation as Euler angles (in degrees)
                QVector3D rotation;
                QQuaternion quat = QQuaternion::fromRotationMatrix(rotationMatrix.normalMatrix());
                QMatrix4x4 rotMat;
                rotMat.rotate(quat);
                float sy = sqrt(rotMat(0,0) * rotMat(0,0) + rotMat(1,0) * rotMat(1,0));
                bool singular = sy < 1e-6;
                if (!singular) {
                    rotation.setX(atan2(rotMat(2,1), rotMat(2,2)) * 180.0f / M_PI);
                    rotation.setY(atan2(-rotMat(2,0), sy) * 180.0f / M_PI);
                    rotation.setZ(atan2(rotMat(1,0), rotMat(0,0)) * 180.0f / M_PI);
                } else {
                    rotation.setX(atan2(-rotMat(1,2), rotMat(1,1)) * 180.0f / M_PI);
                    rotation.setY(atan2(-rotMat(2,0), sy) * 180.0f / M_PI);
                    rotation.setZ(0.0f);
                }

                // Apply the average uniform scale to all dimensions
                QVector3D scale(avgUniformScale, avgUniformScale, avgUniformScale);

                // Add the twin model
                QString modelName = QString("Twin_%1_%2").arg(classNameLower).arg(clusterId);
                m_glWidget->addModel(twinFilePath, modelName);
                m_glWidget->applyTransformToModel(modelName, centroid, rotation, scale);
                m_glWidget->setModelVisibility(modelName, true);

                // Store the position
                twinPositions[className].push_back(centroid);

                qDebug() << "Added model:" << modelName << "for class:" << classNameLower
                         << "cluster ID:" << clusterId << "at position:" << centroid 
                         << "with uniform scale:" << avgUniformScale << "and rotation:" << rotation;

                processedCount++;
                processedClusters++;
                emit progressUpdated(20 + (processedClusters * 80 / totalClusters));
            }
            
            // Handle additional instances
            if (processedCount < expectedCount && !classClusterPoints[className].empty()) {
                qDebug() << "Creating" << (expectedCount - processedCount) << "additional instances for class:" << classNameLower;
                
                // Compute class bounding box and average centroid
                QVector3D classMinBounds(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
                QVector3D classMaxBounds(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
                for (const auto& [clusterId, points] : classClusterPoints[className]) {
                    if (points.empty()) continue;
                    for (const auto& pt : points) {
                        classMinBounds.setX(std::min(classMinBounds.x(), pt.x()));
                        classMinBounds.setY(std::min(classMinBounds.y(), pt.y()));
                        classMinBounds.setZ(std::min(classMinBounds.z(), pt.z()));
                        classMaxBounds.setX(std::max(classMaxBounds.x(), pt.x()));
                        classMaxBounds.setY(std::max(classMaxBounds.y(), pt.y()));
                        classMaxBounds.setZ(std::max(classMaxBounds.z(), pt.z()));
                    }
                }
                
                int additionalCount = expectedCount - processedCount;
                for (int i = 0; i < additionalCount; i++) {
                    // Use positions from objectPositions
                    QVector3D newPosition = objectPositions[className][processedCount + i];
                    
                    QVector3D finalScale(avgUniformScale, avgUniformScale, avgUniformScale);
                    
                    QString modelName = QString("Twin_%1_additional_%2").arg(classNameLower).arg(i);
                    m_glWidget->addModel(twinFilePath, modelName);
                    m_glWidget->applyTransformToModel(modelName, newPosition, QVector3D(0, 0, 0), finalScale);
                    m_glWidget->setModelVisibility(modelName, true);
                    
                    twinPositions[className].push_back(newPosition);

                    qDebug() << "Added additional model:" << modelName << "for class:" << classNameLower
                             << "at position:" << newPosition 
                             << "with uniform scale:" << avgUniformScale;
                    
                    processedClusters++;
                    emit progressUpdated(20 + (processedClusters * 80 / totalClusters));
                }
            }
        }

        // Clear the copied point cloud
        for (auto& pc : m_glWidget->m_pointClouds) {
            if (pc.name == "TempPointCloud") {
                pc.points.clear();
                pc.colors.clear();
                pc.labels.clear();
                pc.vboInitialized = false;
                m_glWidget->updatePointCloudVBO(pc);
            }
        }

        // Log all twin positions
        qDebug() << "Final twin positions:";
        for (const auto& [className, positions] : twinPositions) {
            qDebug() << "Class:" << QString::fromStdString(className) << "Positions:";
            for (const auto& pos : positions) {
                qDebug() << "  " << pos;
            }
        }

        emit progressUpdated(100);
        emit replacementFinished(twinPositions);
    } catch (const std::exception& e) {
        emit errorOccurred(QString("Error: %1").arg(e.what()));
    }
}

ReplaceWithTwinsDialog::ReplaceWithTwinsDialog(const std::map<std::string, int>& clusterCounts, const std::vector<int>& labels,
                                             const QVector<Vertex>& points, OpenGLWidget* glWidget, QWidget* parent)
    : QDialog(parent), m_clusterCounts(clusterCounts), m_labels(labels), m_points(points), m_glWidget(glWidget) {
    setWindowTitle(tr("Replace with Twins"));
    setMinimumWidth(400);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    QLabel* infoLabel = new QLabel(tr("Select a folder containing PLY models to replace detected objects."), this);
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);

    QHBoxLayout* folderLayout = new QHBoxLayout();
    m_folderEdit = new QLineEdit(this);
    m_folderEdit->setPlaceholderText(tr("Select twins folder"));
    QPushButton* browseButton = new QPushButton(tr("Browse"), this);
    folderLayout->addWidget(m_folderEdit);
    folderLayout->addWidget(browseButton);
    mainLayout->addLayout(folderLayout);

    QLabel* clustersLabel = new QLabel(tr("Detected objects:"), this);
    mainLayout->addWidget(clustersLabel);
    
    QString clusterInfo;
    for (const auto& [className, count] : m_clusterCounts) {
        clusterInfo += QString::fromStdString(className) + ": " + QString::number(count) + "\n";
    }
    
    QTextEdit* clustersTextEdit = new QTextEdit(this);
    clustersTextEdit->setPlainText(clusterInfo);
    clustersTextEdit->setReadOnly(true);
    clustersTextEdit->setMaximumHeight(100);
    mainLayout->addWidget(clustersTextEdit);

    m_progressBar = new QProgressBar(this);
    m_progressBar->setRange(0, 100);
    m_progressBar->setValue(0);
    mainLayout->addWidget(new QLabel(tr("Progress:"), this));
    mainLayout->addWidget(m_progressBar);

    m_statusLabel = new QLabel(this);
    mainLayout->addWidget(m_statusLabel);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    m_startButton = new QPushButton(tr("Start Replacement"), this);
    m_startButton->setEnabled(false);
    QPushButton* cancelButton = new QPushButton(tr("Cancel"), this);
    buttonLayout->addWidget(m_startButton);
    buttonLayout->addWidget(cancelButton);
    mainLayout->addLayout(buttonLayout);

    connect(browseButton, &QPushButton::clicked, this, &ReplaceWithTwinsDialog::selectTwinsFolder);
    connect(m_startButton, &QPushButton::clicked, this, &ReplaceWithTwinsDialog::startReplacement);
    connect(cancelButton, &QPushButton::clicked, this, &QDialog::reject);
    connect(m_folderEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
        m_startButton->setEnabled(!text.isEmpty());
    });

    m_workerThread = new QThread(this);
}

ReplaceWithTwinsDialog::~ReplaceWithTwinsDialog() {
    m_workerThread->quit();
    m_workerThread->wait();
}

void ReplaceWithTwinsDialog::selectTwinsFolder() {
    QString folder = QFileDialog::getExistingDirectory(this, tr("Select Twins Folder"));
    if (!folder.isEmpty()) {
        m_folderEdit->setText(folder);
        m_startButton->setEnabled(true);
        
        QDir dir(folder);
        QStringList plyFilters;
        plyFilters << "*.ply" << "*.PLY";
        QStringList plyFiles = dir.entryList(plyFilters, QDir::Files);
        
        if (plyFiles.isEmpty()) {
            m_statusLabel->setText(tr("Warning: No PLY files found in selected folder."));
            m_statusLabel->setStyleSheet("color: red");
        } else {
            m_statusLabel->setText(tr("Found %1 PLY files in selected folder.").arg(plyFiles.size()));
            m_statusLabel->setStyleSheet("color: green");
        }
    }
}

void ReplaceWithTwinsDialog::startReplacement() {
    if (m_folderEdit->text().isEmpty()) {
        QMessageBox::warning(this, tr("Error"), tr("Please select a twins folder."));
        return;
    }

    m_startButton->setEnabled(false);
    m_progressBar->setValue(0);
    m_statusLabel->setText(tr("Processing..."));
    m_statusLabel->setStyleSheet("");

    m_worker = new ReplaceWithTwinsWorker(m_folderEdit->text(), m_clusterCounts, m_labels, m_points, m_glWidget);
    m_worker->moveToThread(m_workerThread);

    connect(m_workerThread, &QThread::started, m_worker, &ReplaceWithTwinsWorker::process);
    connect(m_worker, &ReplaceWithTwinsWorker::progressUpdated, this, &ReplaceWithTwinsDialog::updateProgress);
    connect(m_worker, &ReplaceWithTwinsWorker::replacementFinished, this, &ReplaceWithTwinsDialog::handleReplacementFinished);
    connect(m_worker, &ReplaceWithTwinsWorker::errorOccurred, this, &ReplaceWithTwinsDialog::handleError);
    connect(m_worker, &ReplaceWithTwinsWorker::errorOccurred, m_workerThread, &QThread::quit);
    connect(m_worker, &ReplaceWithTwinsWorker::replacementFinished, m_workerThread, &QThread::quit);
    connect(m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);

    m_workerThread->start();
}

void ReplaceWithTwinsDialog::updateProgress(int value) {
    m_progressBar->setValue(value);
}

void ReplaceWithTwinsDialog::handleReplacementFinished(const std::map<std::string, std::vector<QVector3D>>& twinPositions) {
    QString message = tr("Digital twin replacement completed successfully.\n\nObject positions:\n");
    for (const auto& [className, positions] : twinPositions) {
        message += QString::fromStdString(className) + ":\n";
        for (const auto& pos : positions) {
            message += QString("  (%1, %2, %3)\n").arg(pos.x()).arg(pos.y()).arg(pos.z());
        }
    }
    QMessageBox::information(this, tr("Success"), message);
    emit replacementCompleted(twinPositions);
    accept();
}

void ReplaceWithTwinsDialog::handleError(const QString& error) {
    QMessageBox::critical(this, tr("Error"), error);
    m_startButton->setEnabled(true);
    m_progressBar->setValue(0);
    m_statusLabel->setText(tr("Error occurred. See details."));
    m_statusLabel->setStyleSheet("color: red");
}
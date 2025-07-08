#include "clusterlist.h"
#include "ui_clusterlist.h"
#include "identifysimilarclustersdialog.h"
#include <QDebug>
#include <QFileDialog>
#include <QTextStream>
#include <QStandardPaths>
#include <QDateTime>
#include <QHeaderView>
#include <QApplication>
#include <QStyle>
#include <limits>
#include <algorithm>
#include <cmath>

ClusterList::ClusterList(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ClusterAnalysisDashboard)
{
    ui->setupUi(this);
    
    // Configure table properties
    ui->clusterTable->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->clusterTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->clusterTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->clusterTable->horizontalHeader()->setStretchLastSection(true);
    ui->clusterTable->setSortingEnabled(true);
    
    // Set column widths for better display
    ui->clusterTable->horizontalHeader()->resizeSection(0, 120); // Cluster ID
    ui->clusterTable->horizontalHeader()->resizeSection(1, 100); // Point Count
    ui->clusterTable->horizontalHeader()->resizeSection(2, 180); // Dimensions
    ui->clusterTable->horizontalHeader()->resizeSection(3, 120); // Classification
    ui->clusterTable->horizontalHeader()->resizeSection(4, 100); // Volume
    
    // Connect signals
    connect(ui->clusterTable->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &ClusterList::onTableSelectionChanged);
    connect(ui->refreshButton, &QPushButton::clicked,
            this, &ClusterList::refreshAnalysis);
    connect(ui->exportButton, &QPushButton::clicked,
            this, &ClusterList::on_exportButton_clicked);
    connect(ui->identifySimilarButton, &QPushButton::clicked,
            this, &ClusterList::on_identifySimilarButton_clicked);
    
    // Set initial status
    setStatusMessage("Ready to display cluster analysis results");
    
    // Set window properties
    setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
    setMinimumSize(800, 500);
}

ClusterList::~ClusterList()
{
    delete ui;
}

void ClusterList::updateClusterList(const QString& pointCloudName,
                                   const std::vector<int>& labels,
                                   const std::vector<QVector3D>& points,
                                   const std::map<int, int>& clusterToGroup)
{
    // Store analysis data for similarity analysis
    m_analysisData.pointCloudName = pointCloudName;
    m_analysisData.labels = labels;
    m_analysisData.points = points;
    m_analysisData.clusterToGroup = clusterToGroup;
    m_analysisData.similarityGroups.clear(); // Reset similarity groups
    
    // Clear existing data
    ui->clusterTable->setRowCount(0);
    ui->clusterTable->setSortingEnabled(false);
    
    if (labels.empty() || points.empty()) {
        setStatusMessage("No cluster data available");
        return;
    }
    
    // Update status
    setStatusMessage(QString("Processing %1 data points from %2...")
                    .arg(points.size()).arg(pointCloudName));
    
    // Compute cluster statistics
    std::map<int, std::vector<size_t>> clusterIndices;
    std::map<int, int> groupPointCounts;
    
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            clusterIndices[labels[i]].push_back(i);
            if (clusterToGroup.find(labels[i]) != clusterToGroup.end()) {
                groupPointCounts[clusterToGroup.at(labels[i])]++;
            } else {
                groupPointCounts[labels[i]]++;
            }
        }
    }

    // Compute bounding boxes and volumes
    struct BoundingBox {
        QVector3D size;
        double volume;
    };
    
    std::map<int, BoundingBox> clusterBoxes;
    std::map<int, BoundingBox> groupBoxes;

    for (const auto& cluster : clusterIndices) {
        int label = cluster.first;
        BoundingBox bbox;
        
        QVector3D minPoint(std::numeric_limits<float>::max(), 
                          std::numeric_limits<float>::max(), 
                          std::numeric_limits<float>::max());
        QVector3D maxPoint(std::numeric_limits<float>::lowest(), 
                          std::numeric_limits<float>::lowest(), 
                          std::numeric_limits<float>::lowest());

        for (size_t idx : cluster.second) {
            const QVector3D& pos = points[idx];
            minPoint.setX(std::min(minPoint.x(), pos.x()));
            minPoint.setY(std::min(minPoint.y(), pos.y()));
            minPoint.setZ(std::min(minPoint.z(), pos.z()));
            maxPoint.setX(std::max(maxPoint.x(), pos.x()));
            maxPoint.setY(std::max(maxPoint.y(), pos.y()));
            maxPoint.setZ(std::max(maxPoint.z(), pos.z()));
        }

        bbox.size = maxPoint - minPoint;
        bbox.volume = calculateVolume(bbox.size);
        clusterBoxes[label] = bbox;

        if (clusterToGroup.find(label) != clusterToGroup.end()) {
            int groupId = clusterToGroup.at(label);
            if (groupBoxes.find(groupId) == groupBoxes.end()) {
                groupBoxes[groupId] = bbox;
            } else {
                groupBoxes[groupId].size.setX(std::max(groupBoxes[groupId].size.x(), bbox.size.x()));
                groupBoxes[groupId].size.setY(std::max(groupBoxes[groupId].size.y(), bbox.size.y()));
                groupBoxes[groupId].size.setZ(std::max(groupBoxes[groupId].size.z(), bbox.size.z()));
                groupBoxes[groupId].volume = calculateVolume(groupBoxes[groupId].size);
            }
        }
    }

    // Populate table with individual clusters
    for (const auto& cluster : clusterIndices) {
        int label = cluster.first;
        int row = ui->clusterTable->rowCount();
        ui->clusterTable->insertRow(row);

        ui->clusterTable->setItem(row, 0, new QTableWidgetItem(QString("C-%1").arg(label, 3, 10, QChar('0'))));
        
        QTableWidgetItem* countItem = new QTableWidgetItem(QString::number(cluster.second.size()));
        countItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->clusterTable->setItem(row, 1, countItem);
        
        ui->clusterTable->setItem(row, 2, new QTableWidgetItem(formatDimensions(clusterBoxes[label].size)));
        ui->clusterTable->setItem(row, 3, new QTableWidgetItem("Individual Cluster"));
        
        QTableWidgetItem* volumeItem = new QTableWidgetItem(QString::number(clusterBoxes[label].volume, 'f', 3));
        volumeItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->clusterTable->setItem(row, 4, volumeItem);
    }

    // Populate table with groups
    for (const auto& group : groupPointCounts) {
        int groupId = group.first;
        if (groupBoxes.find(groupId) == groupBoxes.end()) continue;

        int row = ui->clusterTable->rowCount();
        ui->clusterTable->insertRow(row);

        ui->clusterTable->setItem(row, 0, new QTableWidgetItem(QString("G-%1").arg(groupId, 3, 10, QChar('0'))));
        
        QTableWidgetItem* countItem = new QTableWidgetItem(QString::number(group.second));
        countItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->clusterTable->setItem(row, 1, countItem);
        
        ui->clusterTable->setItem(row, 2, new QTableWidgetItem(formatDimensions(groupBoxes[groupId].size)));
        ui->clusterTable->setItem(row, 3, new QTableWidgetItem("Cluster Group"));
        
        QTableWidgetItem* volumeItem = new QTableWidgetItem(QString::number(groupBoxes[groupId].volume, 'f', 3));
        volumeItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        ui->clusterTable->setItem(row, 4, volumeItem);
    }

    ui->clusterTable->setSortingEnabled(true);
    ui->clusterTable->resizeColumnsToContents();
    
    int totalClusters = clusterIndices.size();
    int totalGroups = groupBoxes.size();
    setStatusMessage(QString("Analysis complete: %1 clusters, %2 groups | %3 total points")
                    .arg(totalClusters).arg(totalGroups).arg(points.size()));
}

void ClusterList::on_identifySimilarButton_clicked()
{
    if (m_analysisData.labels.empty() || m_analysisData.points.empty()) {
        QMessageBox::information(this, "Identify Similar Clusters", 
                               "No cluster data available for similarity analysis.");
        return;
    }

    IdentifySimilarClustersDialog dialog(this);
    connect(&dialog, &IdentifySimilarClustersDialog::identifySimilarClustersConfirmed,
            this, &ClusterList::identifySimilarClusters);
    dialog.exec();
}

void ClusterList::identifySimilarClusters()
{
    setStatusMessage("Analyzing cluster similarities...");
    
    // Find similar clusters
    std::vector<int> similarityGroups = findSimilarClusters();
    
    // Store similarity results
    m_analysisData.similarityGroups = similarityGroups;
    
    // Update the table to show similarity information
    updateTableWithSimilarity(similarityGroups);
    
    int uniqueGroups = *std::max_element(similarityGroups.begin(), similarityGroups.end()) + 1;
    setStatusMessage(QString("Similarity analysis complete: Found %1 similarity groups")
                    .arg(uniqueGroups));
}

std::vector<int> ClusterList::findSimilarClusters(double dimensionTolerance, double volumeTolerance) const
{
    // Compute cluster statistics
    std::map<int, std::vector<size_t>> clusterIndices;
    for (size_t i = 0; i < m_analysisData.labels.size(); ++i) {
        if (m_analysisData.labels[i] >= 0) {
            clusterIndices[m_analysisData.labels[i]].push_back(i);
        }
    }
    
    // Compute bounding boxes and volumes for all clusters
    std::map<int, QVector3D> clusterSizes;
    std::map<int, double> clusterVolumes;
    
    for (const auto& cluster : clusterIndices) {
        int label = cluster.first;
        
        QVector3D minPoint(std::numeric_limits<float>::max(), 
                          std::numeric_limits<float>::max(), 
                          std::numeric_limits<float>::max());
        QVector3D maxPoint(std::numeric_limits<float>::lowest(), 
                          std::numeric_limits<float>::lowest(), 
                          std::numeric_limits<float>::lowest());

        for (size_t idx : cluster.second) {
            const QVector3D& pos = m_analysisData.points[idx];
            minPoint.setX(std::min(minPoint.x(), pos.x()));
            minPoint.setY(std::min(minPoint.y(), pos.y()));
            minPoint.setZ(std::min(minPoint.z(), pos.z()));
            maxPoint.setX(std::max(maxPoint.x(), pos.x()));
            maxPoint.setY(std::max(maxPoint.y(), pos.y()));
            maxPoint.setZ(std::max(maxPoint.z(), pos.z()));
        }

        QVector3D size = maxPoint - minPoint;
        clusterSizes[label] = size;
        clusterVolumes[label] = calculateVolume(size);
    }
    
    // Group similar clusters
    std::vector<int> similarityGroups(clusterIndices.size(), -1);
    std::map<int, int> labelToIndex;
    int index = 0;
    for (const auto& cluster : clusterIndices) {
        labelToIndex[cluster.first] = index++;
    }
    
    int currentGroup = 0;
    for (const auto& cluster1 : clusterIndices) {
        int label1 = cluster1.first;
        int idx1 = labelToIndex[label1];
        
        if (similarityGroups[idx1] != -1) continue; // Already grouped
        
        similarityGroups[idx1] = currentGroup;
        
        for (const auto& cluster2 : clusterIndices) {
            int label2 = cluster2.first;
            int idx2 = labelToIndex[label2];
            
            if (label1 == label2 || similarityGroups[idx2] != -1) continue;
            
            if (areClustersSimlar(clusterSizes[label1], clusterSizes[label2],
                                clusterVolumes[label1], clusterVolumes[label2],
                                dimensionTolerance, volumeTolerance)) {
                similarityGroups[idx2] = currentGroup;
            }
        }
        currentGroup++;
    }
    
    return similarityGroups;
}

bool ClusterList::areClustersSimlar(const QVector3D& size1, const QVector3D& size2, 
                                  double volume1, double volume2,
                                  double dimTolerance, double volTolerance) const
{
    // Check volume similarity
    double volumeDifference = std::abs(volume1 - volume2) / std::max(volume1, volume2);
    if (volumeDifference > volTolerance) return false;
    
    // Check dimension similarity
    double xDiff = std::abs(size1.x() - size2.x()) / std::max(size1.x(), size2.x());
    double yDiff = std::abs(size1.y() - size2.y()) / std::max(size1.y(), size2.y());
    double zDiff = std::abs(size1.z() - size2.z()) / std::max(size1.z(), size2.z());
    
    return (xDiff <= dimTolerance && yDiff <= dimTolerance && zDiff <= dimTolerance);
}

void ClusterList::updateTableWithSimilarity(const std::vector<int>& similarityGroups)
{
    // Get current cluster indices
    std::map<int, int> labelToIndex;
    int index = 0;
    
    // Build label to index mapping
    std::map<int, std::vector<size_t>> clusterIndices;
    for (size_t i = 0; i < m_analysisData.labels.size(); ++i) {
        if (m_analysisData.labels[i] >= 0) {
            clusterIndices[m_analysisData.labels[i]].push_back(i);
        }
    }
    
    for (const auto& cluster : clusterIndices) {
        labelToIndex[cluster.first] = index++;
    }
    
    // Update table rows with similarity information
    for (int row = 0; row < ui->clusterTable->rowCount(); ++row) {
        QTableWidgetItem* idItem = ui->clusterTable->item(row, 0);
        if (!idItem) continue;
        
        QString idText = idItem->text();
        if (idText.startsWith("C-")) {
            int clusterId = idText.mid(2).toInt();
            if (labelToIndex.find(clusterId) != labelToIndex.end()) {
                int idx = labelToIndex[clusterId];
                if (idx < static_cast<int>(similarityGroups.size())) {
                    int simGroup = similarityGroups[idx];
                    QTableWidgetItem* classItem = ui->clusterTable->item(row, 3);
                    if (classItem) {
                        classItem->setText(QString("Similar Group %1").arg(simGroup + 1));
                        
                        // Color code similar clusters
                        if (simGroup >= 0) {
                            QColor groupColor;
                            switch (simGroup % 6) {
                                case 0: groupColor = QColor(100, 149, 237); break; // CornflowerBlue
                                case 1: groupColor = QColor(144, 238, 144); break; // LightGreen
                                case 2: groupColor = QColor(255, 182, 193); break; // LightPink
                                case 3: groupColor = QColor(255, 218, 185); break; // PeachPuff
                                case 4: groupColor = QColor(221, 160, 221); break; // Plum
                                case 5: groupColor = QColor(176, 224, 230); break; // PowderBlue
                                default: groupColor = QColor(211, 211, 211); break; // LightGray
                            }
                            
                            for (int col = 0; col < ui->clusterTable->columnCount(); ++col) {
                                QTableWidgetItem* item = ui->clusterTable->item(row, col);
                                if (item) {
                                    item->setBackground(groupColor);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void ClusterList::setStatusMessage(const QString& message)
{
    ui->statusLabel->setText(message);
}

int ClusterList::getSelectedClusterID() const
{
    int currentRow = ui->clusterTable->currentRow();
    if (currentRow >= 0) {
        QTableWidgetItem* item = ui->clusterTable->item(currentRow, 0);
        if (item) {
            QString idText = item->text();
            if (idText.startsWith("C-") || idText.startsWith("G-")) {
                return idText.mid(2).toInt();
            }
        }
    }
    return -1;
}

void ClusterList::refreshAnalysis()
{
    setStatusMessage("Refreshing cluster analysis...");
    
    // Clear similarity groups and refresh the display
    if (!m_analysisData.labels.empty()) {
        updateClusterList(m_analysisData.pointCloudName, 
                         m_analysisData.labels, 
                         m_analysisData.points, 
                         m_analysisData.clusterToGroup);
    }
}

void ClusterList::on_exportButton_clicked()
{
    if (ui->clusterTable->rowCount() == 0) {
        QMessageBox::information(this, "Export Data", 
                               "No cluster data available to export.");
        return;
    }

    QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString defaultFileName = QString("%1/cluster_analysis_%2.csv").arg(defaultPath, timestamp);
    
    QString fileName = QFileDialog::getSaveFileName(
        this, 
        "Export Cluster Analysis Data", 
        defaultFileName,
        "CSV Files (*.csv);;All Files (*.*)"
    );
    
    if (fileName.isEmpty()) {
        return;
    }

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        showExportError(QString("Failed to create file: %1").arg(fileName));
        return;
    }

    QTextStream out(&file);
    
    out << "Cluster_ID,Point_Count,Dimension_X,Dimension_Y,Dimension_Z,Classification,Volume_Cubic_Units\n";

    for (int row = 0; row < ui->clusterTable->rowCount(); ++row) {
        QStringList rowData;
        
        QTableWidgetItem* idItem = ui->clusterTable->item(row, 0);
        rowData << (idItem ? idItem->text() : "");
        
        QTableWidgetItem* countItem = ui->clusterTable->item(row, 1);
        rowData << (countItem ? countItem->text() : "0");
        
        QTableWidgetItem* dimItem = ui->clusterTable->item(row, 2);
        if (dimItem) {
            QString dimText = dimItem->text();
            dimText.replace(" × ", ",");
            rowData << dimText.split(',');
        } else {
            rowData << "0" << "0" << "0";
        }
        
        QTableWidgetItem* classItem = ui->clusterTable->item(row, 3);
        rowData << (classItem ? classItem->text() : "Unknown");
        
        QTableWidgetItem* volumeItem = ui->clusterTable->item(row, 4);
        rowData << (volumeItem ? volumeItem->text() : "0.000");
        
        out << rowData.join(',') << "\n";
    }

    file.close();
    showExportSuccess(fileName);
}

void ClusterList::onTableSelectionChanged()
{
    int selectedID = getSelectedClusterID();
    if (selectedID >= 0) {
        setStatusMessage(QString("Selected cluster/group ID: %1").arg(selectedID));
    }
}

double ClusterList::calculateVolume(const QVector3D& size) const
{
    return static_cast<double>(size.x() * size.y() * size.z());
}

QString ClusterList::formatDimensions(const QVector3D& size) const
{
    return QString("%1 × %2 × %3")
        .arg(size.x(), 0, 'f', 2)
        .arg(size.y(), 0, 'f', 2)
        .arg(size.z(), 0, 'f', 2);
}

void ClusterList::showExportSuccess(const QString& fileName)
{
    QMessageBox msgBox(this);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setWindowTitle("Export Successful");
    msgBox.setText("Cluster analysis data exported successfully.");
    msgBox.setInformativeText(QString("File saved to:\n%1").arg(fileName));
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.exec();
}

void ClusterList::showExportError(const QString& error)
{
    QMessageBox::critical(this, "Export Failed", 
                         QString("Failed to export cluster data:\n%1").arg(error));
}
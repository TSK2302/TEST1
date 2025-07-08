#include "colorassigner.h"
#include "identifysimilarclustersdialog.h"
#include <QHeaderView>
#include <cmath>

ColorAssignerDialog::ColorAssignerDialog(const std::vector<ClusterInfo>& clusters, 
                                         const std::vector<QVector3D>& currentColors,
                                         const std::vector<int>& labels,
                                         QWidget* parent)
    : QDialog(parent),
      clusters(clusters),
      assignedColors(currentColors),
      labels(labels)
{
    setWindowTitle(tr("Assign Colors to Clusters"));
    resize(600, 400);

    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Add info label
    QLabel* infoLabel = new QLabel(tr("Select clusters and assign colors. Colors will be applied to all points in each cluster."), this);
    infoLabel->setWordWrap(true);
    mainLayout->addWidget(infoLabel);

    tableWidget = new QTableWidget(this);
    tableWidget->setRowCount(clusters.size());
    tableWidget->setColumnCount(4);
    tableWidget->setHorizontalHeaderLabels({tr("Cluster ID"), tr("Points"), tr("Current Color"), tr("Assign Color")});
    tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);

    // Build label to index mapping
    labelToIndexMap.clear();
    for (size_t i = 0; i < clusters.size(); ++i) {
        labelToIndexMap[clusters[i].label] = i;
        
        // Cluster ID
        QTableWidgetItem* labelItem = new QTableWidgetItem(QString("Cluster %1").arg(clusters[i].label));
        labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
        labelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, labelItem);

        // Point count
        QTableWidgetItem* countItem = new QTableWidgetItem(QString::number(clusters[i].pointCount));
        countItem->setFlags(countItem->flags() & ~Qt::ItemIsEditable);
        countItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, countItem);

        // Color display
        QTableWidgetItem* colorItem = new QTableWidgetItem();
        QColor displayColor(
            static_cast<int>(assignedColors[i].x() * 255),
            static_cast<int>(assignedColors[i].y() * 255),
            static_cast<int>(assignedColors[i].z() * 255));
        colorItem->setBackground(displayColor);
        colorItem->setFlags(colorItem->flags() & ~Qt::ItemIsEditable);
        colorItem->setText(QString("RGB(%1,%2,%3)")
                          .arg(displayColor.red())
                          .arg(displayColor.green())
                          .arg(displayColor.blue()));
        tableWidget->setItem(i, 2, colorItem);

        // Assign button
        QPushButton* assignButton = new QPushButton(tr("Change Color"), this);
        connect(assignButton, &QPushButton::clicked, this, [this, i]() { onAssignColorClicked(i); });
        tableWidget->setCellWidget(i, 3, assignButton);
    }

    mainLayout->addWidget(tableWidget);

    QHBoxLayout* buttonLayout = new QHBoxLayout;
    applyToSimilarButton = new QPushButton(tr("Apply to Similar Clusters"), this);
    okButton = new QPushButton(tr("Apply Colors"), this);
    cancelButton = new QPushButton(tr("Cancel"), this);

    buttonLayout->addWidget(applyToSimilarButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(okButton);
    buttonLayout->addWidget(cancelButton);

    mainLayout->addLayout(buttonLayout);

    connect(applyToSimilarButton, &QPushButton::clicked, this, &ColorAssignerDialog::onApplyToSimilarClustersClicked);
    connect(okButton, &QPushButton::clicked, this, &ColorAssignerDialog::onOkClicked);
    connect(cancelButton, &QPushButton::clicked, this, &ColorAssignerDialog::onCancelClicked);

    updateTableColors();
}

void ColorAssignerDialog::updateTableColors()
{
    for (size_t i = 0; i < clusters.size(); ++i) {
        QTableWidgetItem* colorItem = tableWidget->item(i, 2);
        if (colorItem) {
            QColor displayColor(
                static_cast<int>(assignedColors[i].x() * 255),
                static_cast<int>(assignedColors[i].y() * 255),
                static_cast<int>(assignedColors[i].z() * 255));
            colorItem->setBackground(displayColor);
            colorItem->setText(QString("RGB(%1,%2,%3)")
                              .arg(displayColor.red())
                              .arg(displayColor.green())
                              .arg(displayColor.blue()));
        }
    }
}

void ColorAssignerDialog::assignColorToCluster(int clusterIndex, const QColor& color)
{
    if (clusterIndex >= 0 && clusterIndex < static_cast<int>(assignedColors.size())) {
        assignedColors[clusterIndex] = QVector3D(
            color.redF(),
            color.greenF(),
            color.blueF()
        );
        updateTableColors();
    }
}

void ColorAssignerDialog::onAssignColorClicked(int row)
{
    if (row >= 0 && row < static_cast<int>(assignedColors.size())) {
        QColor currentColor(
            static_cast<int>(assignedColors[row].x() * 255),
            static_cast<int>(assignedColors[row].y() * 255),
            static_cast<int>(assignedColors[row].z() * 255));

        QColor color = QColorDialog::getColor(currentColor, this, 
                                             tr("Select Color for Cluster %1").arg(clusters[row].label));
        if (color.isValid()) {
            assignColorToCluster(row, color);
        }
    }
}

void ColorAssignerDialog::onApplyToSimilarClustersClicked()
{
    int selectedRow = tableWidget->currentRow();
    if (selectedRow < 0) {
        QMessageBox::information(this, tr("No Selection"), 
                               tr("Please select a cluster row first to apply its color to similar clusters."));
        return;
    }

    groupSimilarClusters();
    if (!similarClusterGroups.empty()) {
        applyColorToSimilarClusters(selectedRow);
    }
}

void ColorAssignerDialog::applyColorToSimilarClusters(int sourceClusterIndex)
{
    if (sourceClusterIndex < 0 || sourceClusterIndex >= static_cast<int>(clusters.size())) {
        return;
    }

    QVector3D sourceColor = assignedColors[sourceClusterIndex];
    int appliedCount = 0;

    // Find the group containing the source cluster
    for (const auto& group : similarClusterGroups) {
        bool containsSource = std::find(group.begin(), group.end(), sourceClusterIndex) != group.end();
        if (containsSource) {
            // Apply source color to all clusters in this group
            for (size_t clusterIdx : group) {
                if (clusterIdx != static_cast<size_t>(sourceClusterIndex)) {
                    assignedColors[clusterIdx] = sourceColor;
                    appliedCount++;
                }
            }
            break;
        }
    }

    updateTableColors();
    
    if (appliedCount > 0) {
        QMessageBox::information(this, tr("Colors Applied"), 
                               tr("Applied color to %1 similar clusters.").arg(appliedCount));
    } else {
        QMessageBox::information(this, tr("No Similar Clusters"), 
                               tr("No similar clusters found for the selected cluster."));
    }
}

void ColorAssignerDialog::groupSimilarClusters()
{
    similarClusterGroups.clear();
    std::vector<bool> processed(clusters.size(), false);

    for (size_t i = 0; i < clusters.size(); ++i) {
        if (processed[i]) continue;

        const ClusterInfo& referenceCluster = clusters[i];
        std::vector<size_t> similarGroup;
        similarGroup.push_back(i);
        processed[i] = true;

        for (size_t j = i + 1; j < clusters.size(); ++j) {
            if (processed[j]) continue;

            const ClusterInfo& compareCluster = clusters[j];

            // Compare clusters based on extent and point count
            float extentDiff = (referenceCluster.extent - compareCluster.extent).norm();
            float referenceExtentMagnitude = referenceCluster.extent.norm();
            float relativeExtentDiff = (referenceExtentMagnitude > 0) ? extentDiff / referenceExtentMagnitude : 0;

            float pointCountDiff = std::abs(static_cast<float>(referenceCluster.pointCount - compareCluster.pointCount)) / 
                                  std::max(static_cast<float>(referenceCluster.pointCount), static_cast<float>(compareCluster.pointCount));
            
            // Thresholds for similarity (adjust as needed)
            const float extentThreshold = 0.3f; // 30% difference in extent
            const float pointCountThreshold = 0.5f; // 50% difference in point count
            
            if (relativeExtentDiff < extentThreshold && pointCountDiff < pointCountThreshold) {
                similarGroup.push_back(j);
                processed[j] = true;
            }
        }

        if (similarGroup.size() > 1) { // Only add groups with more than one cluster
            similarClusterGroups.push_back(similarGroup);
        }
    }

    // Notify user of the results
    QString message = QString("Found %1 groups of similar clusters.").arg(similarClusterGroups.size());
    if (similarClusterGroups.size() > 0) {
        message += "\n\nGroups:";
        for (size_t i = 0; i < similarClusterGroups.size(); ++i) {
            message += QString("\nGroup %1: ").arg(i + 1);
            for (size_t j = 0; j < similarClusterGroups[i].size(); ++j) {
                if (j > 0) message += ", ";
                message += QString("Cluster %1").arg(clusters[similarClusterGroups[i][j]].label);
            }
        }
    }
    QMessageBox::information(this, tr("Similar Clusters"), message);
}

std::map<int, QVector3D> ColorAssignerDialog::getClusterColorMap() const
{
    std::map<int, QVector3D> colorMap;
    for (size_t i = 0; i < clusters.size(); ++i) {
        colorMap[clusters[i].label] = assignedColors[i];
    }
    return colorMap;
}

void ColorAssignerDialog::onOkClicked()
{
    std::map<int, QVector3D> clusterColorMap = getClusterColorMap();
    emit colorsAssigned(assignedColors, clusterColorMap);
    accept();
}

void ColorAssignerDialog::onCancelClicked()
{
    reject();
}
#ifndef COLORASSIGNER_H
#define COLORASSIGNER_H

#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <QColorDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QCheckBox>
#include <QMessageBox>
#include <vector>
#include <QVector3D>
#include <map>
#include "cluster.h"

class ColorAssignerDialog : public QDialog {
    Q_OBJECT
public:
    explicit ColorAssignerDialog(const std::vector<ClusterInfo>& clusters, 
                                const std::vector<QVector3D>& currentColors,
                                const std::vector<int>& labels,
                                QWidget* parent = nullptr);

    std::vector<QVector3D> getAssignedColors() const { return assignedColors; }
    std::map<int, QVector3D> getClusterColorMap() const;

signals:
    void colorsAssigned(const std::vector<QVector3D>& colors, const std::map<int, QVector3D>& clusterColorMap);
    void identifySimilarClustersRequested();

private slots:
    void onAssignColorClicked(int row);
    void onApplyToSimilarClustersClicked();
    void onOkClicked();
    void onCancelClicked();
    void groupSimilarClusters();
    void applyColorToSimilarClusters(int sourceClusterIndex);

private:
    void updateTableColors();
    void assignColorToCluster(int clusterIndex, const QColor& color);
    
    QTableWidget* tableWidget;
    QPushButton* applyToSimilarButton;
    QPushButton* okButton;
    QPushButton* cancelButton;
    std::vector<ClusterInfo> clusters;
    std::vector<QVector3D> assignedColors;
    std::vector<int> labels;
    std::vector<std::vector<size_t>> similarClusterGroups; // Groups of similar clusters
    std::map<int, int> labelToIndexMap; // Maps cluster label to index in assignedColors
};

#endif // COLORASSIGNER_H
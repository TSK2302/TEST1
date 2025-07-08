#ifndef CLUSTERLIST_H
#define CLUSTERLIST_H

#include <QDialog>
#include <QTableWidget>
#include <QVector3D>
#include <QMessageBox>
#include <QProgressDialog>
#include <vector>
#include <map>

namespace Ui {
class ClusterAnalysisDashboard;
}

// Structure to hold cluster analysis data
struct ClusterAnalysisData {
    QString pointCloudName;
    std::vector<int> labels;
    std::vector<QVector3D> points;
    std::map<int, int> clusterToGroup;
    std::vector<int> similarityGroups; // New field for similarity grouping
};

class ClusterList : public QDialog
{
    Q_OBJECT

public:
    explicit ClusterList(QWidget *parent = nullptr);
    ~ClusterList();

    void updateClusterList(const QString& pointCloudName,
                         const std::vector<int>& labels,
                         const std::vector<QVector3D>& points,
                         const std::map<int, int>& clusterToGroup);

    void setStatusMessage(const QString& message);
    int getSelectedClusterID() const;

public slots:
    void refreshAnalysis();
    void identifySimilarClusters(); // New slot

private slots:
    void on_exportButton_clicked();
    void onTableSelectionChanged();
    void on_identifySimilarButton_clicked(); // New slot

private:
    Ui::ClusterAnalysisDashboard *ui;
    ClusterAnalysisData m_analysisData; // Store analysis data
    
    double calculateVolume(const QVector3D& size) const;
    QString formatDimensions(const QVector3D& size) const;
    void showExportSuccess(const QString& fileName);
    void showExportError(const QString& error);
    
    // New methods for similarity analysis
    std::vector<int> findSimilarClusters(double dimensionTolerance = 0.1, 
                                       double volumeTolerance = 0.15) const;
    void updateTableWithSimilarity(const std::vector<int>& similarityGroups);
    bool areClustersSimlar(const QVector3D& size1, const QVector3D& size2, 
                          double volume1, double volume2,
                          double dimTolerance, double volTolerance) const;
};

#endif // CLUSTERLIST_H
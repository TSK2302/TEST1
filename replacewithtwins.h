#ifndef REPLACEWITHTWINS_H
#define REPLACEWITHTWINS_H

#include <QDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QThread>
#include <QString>
#include <QVector3D>
#include <vector>
#include <map>
#include "openglwidget.h"
#include "cluster.h"

// Utility function to compute object positions
std::map<std::string, std::vector<QVector3D>> computeObjectPositions(
    const std::map<std::string, int>& clusterCounts,
    const std::vector<int>& labels,
    const QVector<Vertex>& points,
    std::map<std::string, std::vector<int>>& outClassLabels,
    std::map<std::string, std::map<int, std::vector<QVector3D>>>& outClassClusterPoints
);

class ReplaceWithTwinsWorker : public QObject {
    Q_OBJECT
public:
    ReplaceWithTwinsWorker(const QString& twinsFolder, const std::map<std::string, int>& clusterCounts,
                          const std::vector<int>& labels, const QVector<Vertex>& points, OpenGLWidget* glWidget,
                          QObject* parent = nullptr);
    ~ReplaceWithTwinsWorker();

public slots:
    void process();

signals:
    void progressUpdated(int value);
    void replacementFinished(const std::map<std::string, std::vector<QVector3D>>& twinPositions);
    void errorOccurred(const QString& error);

private:
    QString m_twinsFolder;
    std::map<std::string, int> m_clusterCounts;
    std::vector<int> m_labels;
    QVector<Vertex> m_points;
    OpenGLWidget* m_glWidget;
};

class ReplaceWithTwinsDialog : public QDialog {
    Q_OBJECT
public:
    explicit ReplaceWithTwinsDialog(const std::map<std::string, int>& clusterCounts, const std::vector<int>& labels,
                                   const QVector<Vertex>& points, OpenGLWidget* glWidget, QWidget* parent = nullptr);
    ~ReplaceWithTwinsDialog();

signals:
    void replacementCompleted(const std::map<std::string, std::vector<QVector3D>>& twinPositions);

public slots:
    void updateProgress(int value);
    void handleReplacementFinished(const std::map<std::string, std::vector<QVector3D>>& twinPositions);
    void handleError(const QString& error);

private slots:
    void selectTwinsFolder();
    void startReplacement();

private:
    QLineEdit* m_folderEdit;
    QProgressBar* m_progressBar;
    QPushButton* m_startButton;
    QLabel* m_statusLabel;
    QThread* m_workerThread;
    ReplaceWithTwinsWorker* m_worker;
    std::map<std::string, int> m_clusterCounts;
    std::vector<int> m_labels;
    QVector<Vertex> m_points;
    OpenGLWidget* m_glWidget;
};

#endif // REPLACEWITHTWINS_H
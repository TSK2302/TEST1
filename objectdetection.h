#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

#include <QDialog>
#include <QProgressBar>
#include <QPushButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QThread>
#include <QProcess>
#include <QString>
#include <QVector3D>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <vector>
#include <map>
#include <QSpinBox>
#include <QTableWidget>
#include "cluster.h"

class ObjectDetectionWorker : public QObject {
    Q_OBJECT
public:
    ObjectDetectionWorker(const QString& trainingFolder, const QVector<Vertex>& points,
                         const std::vector<int>& labels, const QString& scriptPath,
                         const QString& pythonPath, const QString& modelPath, int numEpochs, 
                         float learningRate, QObject* parent = nullptr);
    ~ObjectDetectionWorker();

public slots:
    void process();
    void cleanup();

signals:
    void progressUpdated(int value, const QString& message);
    void detectionFinished(const std::vector<int>& updatedLabels, const QString& visPcdPath,
                          const std::map<std::string, int>& clusterCounts, float accuracy,
                          int numEpochs, float learningRate, const QString& trainingFolder);
    void errorOccurred(const QString& error);

private:
    QString m_trainingFolder;
    QVector<Vertex> m_points;
    std::vector<int> m_labels;
    QString m_scriptPath;
    QString m_pythonPath;
    QString m_modelPath;
    int m_numEpochs;
    float m_learningRate;
    QString m_tempPointsFile;
    QString m_tempLabelsFile;
    QString m_tempOutputFile;
    QString m_tempScriptFile;
};

class ObjectDetectionDialog : public QDialog {
    Q_OBJECT
public:
    explicit ObjectDetectionDialog(const QVector<Vertex>& points, const std::vector<int>& labels,
                                  const QString& scriptPath, const QString& pythonPath, 
                                  const QString& modelPath, QWidget* parent = nullptr);
    ~ObjectDetectionDialog();

signals:
    void detectionCompleted(const std::vector<int>& updatedLabels, const QString& visPcdPath,
                            const std::map<std::string, int>& clusterCounts, int numEpochs,
                            float learningRate, const QString& trainingFolder, float accuracy);
    void showResults(const std::map<std::string, int>& clusterCounts, float accuracy);

public slots:
    void updateProgress(int value, const QString& message);
    void handleDetectionFinished(const std::vector<int>& updatedLabels, const QString& visPcdPath,
                                const std::map<std::string, int>& clusterCounts, float accuracy,
                                int numEpochs, float learningRate, const QString& trainingFolder);
    void handleError(const QString& error);

private slots:
    void selectTrainingFolder();
    void startDetection();
    void showResultsDialog(const std::map<std::string, int>& clusterCounts, float accuracy);
    void exportToCSV();

private:
    QLineEdit* m_folderEdit;
    QProgressBar* m_progressBar;
    QSpinBox* m_epochSpinBox;
    QLineEdit* m_lrEdit;
    QPushButton* m_startButton;
    QLabel* m_statusLabel;
    QThread* m_workerThread;
    ObjectDetectionWorker* m_worker;
    QString m_scriptPath;
    QString m_pythonPath;
    QString m_modelPath;
    QVector<Vertex> m_points;
    std::vector<int> m_labels;
    std::map<std::string, int> m_clusterCounts;
    QTableWidget* m_resultsTable;
    QPushButton* m_exportButton;
    int m_numEpochs;
    float m_learningRate;
    QString m_trainingFolder;
    float m_accuracy;
};

#endif // OBJECTDETECTION_H
#ifndef SAMPLINGPROCESSOR_H
#define SAMPLINGPROCESSOR_H

#include <QDialog>
#include <QVector3D>
#include <vector>
#include <tuple>

QT_BEGIN_NAMESPACE
namespace Ui {
class SamplingProcessor;
}
QT_END_NAMESPACE

class SamplingProcessor : public QDialog
{
    Q_OBJECT

public:
    explicit SamplingProcessor(QWidget *parent = nullptr);
    ~SamplingProcessor();

    void downSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, float voxelSize);
    void upSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int multiplier);
    void subSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, float ratio);
    void sample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount);

    // Method for mesh surface sampling
    void sampleMeshSurface(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount);

    // Method to export to PLY
    bool exportToPly(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& filePath);

    // Method to set point cloud data and configure UI for a specific operation
    void setPointCloudData(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& operation);

    // Method to set mesh data for surface sampling
    void setMeshData(const std::vector<QVector3D>& vertices, const std::vector<unsigned int>& indices);

private slots:
    void on_applyButton_clicked();
    void on_cancelButton_clicked();
    void on_operationComboBox_currentIndexChanged(int index);

signals:
    void samplingApplied(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& operation);

private:
    Ui::SamplingProcessor *ui;

    // Store point cloud data
    std::vector<QVector3D> m_points;
    std::vector<QVector3D> m_colors;

    // Store mesh data
    std::vector<QVector3D> m_vertices;
    std::vector<unsigned int> m_indices;

    // Custom hash function for std::tuple<int,int,int>
    struct TupleHash {
        std::size_t operator()(const std::tuple<int, int, int>& t) const {
            auto [x, y, z] = t;
            std::size_t h1 = std::hash<int>{}(x);
            std::size_t h2 = std::hash<int>{}(y);
            std::size_t h3 = std::hash<int>{}(z);
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    // Method to configure UI visibility
    void configureUIVisibility(const QString& operation);
};

#endif // SAMPLINGPROCESSOR_H
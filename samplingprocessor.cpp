#include "samplingprocessor.h"
#include "ui_samplingprocessor.h"
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <random>
#include <unordered_map>
#include <fstream>

SamplingProcessor::SamplingProcessor(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SamplingProcessor)
{
    ui->setupUi(this);

    // Set default values for spin boxes
    ui->downSampleSpinBox->setRange(0.001, 100.0);
    ui->downSampleSpinBox->setValue(0.1);
    ui->downSampleSpinBox->setSingleStep(0.01);

    ui->upSampleSpinBox->setRange(1, 100);
    ui->upSampleSpinBox->setValue(2);

    ui->subSampleSpinBox->setRange(0.01, 1.0);
    ui->subSampleSpinBox->setValue(0.5);
    ui->subSampleSpinBox->setSingleStep(0.01);

    ui->sampleSpinBox->setRange(1, 1000000);
    ui->sampleSpinBox->setValue(1000);

    // Connect signals
    connect(ui->operationComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SamplingProcessor::on_operationComboBox_currentIndexChanged);

    // Initialize UI visibility
    configureUIVisibility("Down Sample");
}

SamplingProcessor::~SamplingProcessor()
{
    delete ui;
}

void SamplingProcessor::setPointCloudData(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& operation)
{
    m_points = points;
    m_colors = colors;

    // Set the operation in the combo box
    int index = ui->operationComboBox->findText(operation);
    if (index != -1) {
        ui->operationComboBox->setCurrentIndex(index);
    }

    configureUIVisibility(operation);
}

void SamplingProcessor::setMeshData(const std::vector<QVector3D>& vertices, const std::vector<unsigned int>& indices)
{
    m_vertices = vertices;
    m_indices = indices;
}

void SamplingProcessor::configureUIVisibility(const QString& operation)
{
    // Hide all parameter widgets by default
    ui->downSampleLabel->setVisible(false);
    ui->downSampleSpinBox->setVisible(false);
    ui->upSampleLabel->setVisible(false);
    ui->upSampleSpinBox->setVisible(false);
    ui->subSampleLabel->setVisible(false);
    ui->subSampleSpinBox->setVisible(false);
    ui->sampleLabel->setVisible(false);
    ui->sampleSpinBox->setVisible(false);
    ui->exportPlyLabel->setVisible(false);
    ui->exportPlyCheckBox->setVisible(false);

    // Show relevant widgets based on operation
    if (operation == "Down Sample") {
        ui->downSampleLabel->setVisible(true);
        ui->downSampleSpinBox->setVisible(true);
        ui->exportPlyLabel->setVisible(true);
        ui->exportPlyCheckBox->setVisible(true);
    } else if (operation == "Up Sample") {
        ui->upSampleLabel->setVisible(true);
        ui->upSampleSpinBox->setVisible(true);
        ui->exportPlyLabel->setVisible(true);
        ui->exportPlyCheckBox->setVisible(true);
    } else if (operation == "Sub Sample") {
        ui->subSampleLabel->setVisible(true);
        ui->subSampleSpinBox->setVisible(true);
        ui->exportPlyLabel->setVisible(true);
        ui->exportPlyCheckBox->setVisible(true);
    } else if (operation == "Sampling") {
        ui->sampleLabel->setVisible(true);
        ui->sampleSpinBox->setVisible(true);
        ui->exportPlyLabel->setVisible(true);
        ui->exportPlyCheckBox->setVisible(true);
    }
}

void SamplingProcessor::downSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, float voxelSize)
{
    if (points.empty()) return;

    std::unordered_map<std::tuple<int, int, int>, std::pair<QVector3D, QVector3D>, TupleHash> voxelMap;

    for (size_t i = 0; i < points.size(); ++i) {
        const QVector3D& point = points[i];
        const QVector3D& color = colors[i];

        // Compute voxel grid coordinates
        int x = static_cast<int>(point.x() / voxelSize);
        int y = static_cast<int>(point.y() / voxelSize);
        int z = static_cast<int>(point.z() / voxelSize);
        std::tuple<int, int, int> voxelKey = {x, y, z};

        // Store point and color (could average multiple points per voxel if needed)
        voxelMap[voxelKey] = {point, color};
    }

    // Collect results
    points.clear();
    colors.clear();
    points.reserve(voxelMap.size());
    colors.reserve(voxelMap.size());

    for (const auto& voxel : voxelMap) {
        points.push_back(voxel.second.first);
        colors.push_back(voxel.second.second);
    }
}

void SamplingProcessor::upSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int multiplier)
{
    if (points.empty() || multiplier <= 1) return;

    std::vector<QVector3D> newPoints;
    std::vector<QVector3D> newColors;
    newPoints.reserve(points.size() * multiplier);
    newColors.reserve(colors.size() * multiplier);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(-0.01f, 0.01f);

    for (size_t i = 0; i < points.size(); ++i) {
        const QVector3D& point = points[i];
        const QVector3D& color = colors[i];

        // Add original point
        newPoints.push_back(point);
        newColors.push_back(color);

        // Add interpolated points
        for (int j = 1; j < multiplier; ++j) {
            QVector3D offset(dist(gen), dist(gen), dist(gen));
            newPoints.push_back(point + offset);
            newColors.push_back(color);
        }
    }

    points = std::move(newPoints);
    colors = std::move(newColors);
}

void SamplingProcessor::subSample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, float ratio)
{
    if (points.empty() || ratio <= 0.0f || ratio >= 1.0f) return;

    std::vector<QVector3D> newPoints;
    std::vector<QVector3D> newColors;
    size_t targetSize = static_cast<size_t>(points.size() * ratio);
    newPoints.reserve(targetSize);
    newColors.reserve(targetSize);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (size_t i = 0; i < points.size(); ++i) {
        if (dist(gen) < ratio) {
            newPoints.push_back(points[i]);
            newColors.push_back(colors[i]);
        }
    }

    points = std::move(newPoints);
    colors = std::move(newColors);
}

void SamplingProcessor::sample(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount)
{
    if (points.empty() || sampleCount <= 0) return;

    std::vector<QVector3D> newPoints;
    std::vector<QVector3D> newColors;
    newPoints.reserve(sampleCount);
    newColors.reserve(sampleCount);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    for (int i = 0; i < sampleCount; ++i) {
        size_t index = dist(gen);
        newPoints.push_back(points[index]);
        newColors.push_back(colors[index]);
    }

    points = std::move(newPoints);
    colors = std::move(newColors);
}

void SamplingProcessor::sampleMeshSurface(std::vector<QVector3D>& points, std::vector<QVector3D>& colors, int sampleCount)
{
    if (m_vertices.empty() || m_indices.empty() || sampleCount <= 0) return;

    // Collect triangles and compute areas
    struct Triangle {
        QVector3D v0, v1, v2;
        float area;
    };
    std::vector<Triangle> triangles;
    std::vector<float> areas;
    float totalArea = 0.0f;

    for (size_t i = 0; i < m_indices.size(); i += 3) {
        if (i + 2 >= m_indices.size()) continue;

        unsigned int i0 = m_indices[i];
        unsigned int i1 = m_indices[i + 1];
        unsigned int i2 = m_indices[i + 2];

        if (i0 >= m_vertices.size() || i1 >= m_vertices.size() || i2 >= m_vertices.size()) continue;

        Triangle tri;
        tri.v0 = m_vertices[i0];
        tri.v1 = m_vertices[i1];
        tri.v2 = m_vertices[i2];

        // Compute triangle area using cross product
        QVector3D e1 = tri.v1 - tri.v0;
        QVector3D e2 = tri.v2 - tri.v0;
        tri.area = 0.5f * QVector3D::crossProduct(e1, e2).length();

        triangles.push_back(tri);
        areas.push_back(tri.area);
        totalArea += tri.area;
    }

    if (triangles.empty() || totalArea == 0.0f) return;

    // Normalize areas for cumulative distribution
    std::vector<float> cumulativeAreas(areas.size());
    cumulativeAreas[0] = areas[0] / totalArea;
    for (size_t i = 1; i < areas.size(); ++i) {
        cumulativeAreas[i] = cumulativeAreas[i - 1] + (areas[i] / totalArea);
    }

    // Random number generation
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    points.clear();
    colors.clear();
    points.reserve(sampleCount);
    colors.reserve(sampleCount);

    // Generate sample points
    for (int i = 0; i < sampleCount; ++i) {
        // Select triangle based on area
        float r = dist(gen);
        auto it = std::lower_bound(cumulativeAreas.begin(), cumulativeAreas.end(), r);
        size_t triIndex = std::distance(cumulativeAreas.begin(), it);
        if (triIndex >= triangles.size()) triIndex = triangles.size() - 1;

        const Triangle& tri = triangles[triIndex];

        // Generate random barycentric coordinates
        float r1 = dist(gen);
        float r2 = dist(gen);
        if (r1 + r2 > 1.0f) {
            r1 = 1.0f - r1;
            r2 = 1.0f - r2;
        }
        float r3 = 1.0f - r1 - r2;

        // Compute point position
        QVector3D point = r1 * tri.v0 + r2 * tri.v1 + r3 * tri.v2;
        points.push_back(point);

        // Assign default white color
        colors.push_back(QVector3D(1.0f, 1.0f, 1.0f));
    }
}

bool SamplingProcessor::exportToPly(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& filePath)
{
    std::ofstream file(filePath.toStdString(), std::ios::binary);
    if (!file.is_open()) {
        qDebug() << "Failed to open file for writing:" << filePath;
        return false;
    }

    // Write PLY header
    file << "ply\n";
    file << "format binary_little_endian 1.0\n";
    file << "element vertex " << points.size() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "property uchar red\n";
    file << "property uchar green\n";
    file << "property uchar blue\n";
    file << "end_header\n";

    // Write vertex data
    for (size_t i = 0; i < points.size(); ++i) {
        const QVector3D& point = points[i];
        const QVector3D& color = colors[i];

        // Store coordinates in temporary variables
        float x = point.x();
        float y = point.y();
        float z = point.z();

        file.write(reinterpret_cast<const char*>(&x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&y), sizeof(float));
        file.write(reinterpret_cast<const char*>(&z), sizeof(float));

        unsigned char r = static_cast<unsigned char>(color.x() * 255.0f);
        unsigned char g = static_cast<unsigned char>(color.y() * 255.0f);
        unsigned char b = static_cast<unsigned char>(color.z() * 255.0f);

        file.write(reinterpret_cast<const char*>(&r), sizeof(unsigned char));
        file.write(reinterpret_cast<const char*>(&g), sizeof(unsigned char));
        file.write(reinterpret_cast<const char*>(&b), sizeof(unsigned char));
    }

    file.close();
    qDebug() << "Exported PLY file:" << filePath;
    return true;
}

void SamplingProcessor::on_applyButton_clicked()
{
    QString operation = ui->operationComboBox->currentText();
    std::vector<QVector3D> processedPoints = m_points;
    std::vector<QVector3D> processedColors = m_colors;

    if (operation == "Down Sample") {
        float voxelSize = ui->downSampleSpinBox->value();
        downSample(processedPoints, processedColors, voxelSize);
    } else if (operation == "Up Sample") {
        int multiplier = ui->upSampleSpinBox->value();
        upSample(processedPoints, processedColors, multiplier);
    } else if (operation == "Sub Sample") {
        float ratio = ui->subSampleSpinBox->value();
        subSample(processedPoints, processedColors, ratio);
    } else if (operation == "Sampling") {
        int sampleCount = ui->sampleSpinBox->value();
        sampleMeshSurface(processedPoints, processedColors, sampleCount);
    }

    // Export to PLY if checked
    if (ui->exportPlyCheckBox->isChecked()) {
        QString filePath = QFileDialog::getSaveFileName(this, tr("Save PLY File"), "", tr("PLY Files (*.ply)"));
        if (!filePath.isEmpty()) {
            if (!exportToPly(processedPoints, processedColors, filePath)) {
                QMessageBox::critical(this, tr("Error"), tr("Failed to export PLY file."));
                return;
            }
        }
    }

    // Emit signal with processed data
    emit samplingApplied(processedPoints, processedColors, operation);
    accept();
}

void SamplingProcessor::on_cancelButton_clicked()
{
    reject();
}

void SamplingProcessor::on_operationComboBox_currentIndexChanged(int index)
{
    QString operation = ui->operationComboBox->itemText(index);
    configureUIVisibility(operation);
}
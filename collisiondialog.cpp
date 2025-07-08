#include "collisiondialog.h"
#include "ui_detection.h"
#include <pcl/io/ply_io.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <QDebug>
#include <QTimer>
#include <Eigen/Dense>
#include <vtkOutputWindow.h>
#include <vtkSmartPointer.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <set>
#include <cmath>

// Custom VTK Output Window to redirect messages to qDebug
class CustomVTKOutputWindow : public vtkOutputWindow {
public:
    static CustomVTKOutputWindow* New() {
        return new CustomVTKOutputWindow;
    }
    void DisplayText(const char* text) override {
        qDebug() << "VTK Output:" << text;
    }
protected:
    CustomVTKOutputWindow() {}
};

// SpatialGrid implementation (used only for visualization grid)
SpatialGrid::SpatialGrid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float cell_size_) : cell_size(cell_size_) {
    min_pt = Eigen::Vector3f(FLT_MAX, FLT_MAX, FLT_MAX);
    Eigen::Vector3f max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& pt : cloud->points) {
        min_pt(0) = std::min(min_pt(0), pt.x);
        min_pt(1) = std::min(min_pt(1), pt.y);
        min_pt(2) = std::min(min_pt(2), pt.z);
        max_pt(0) = std::max(max_pt(0), pt.x);
        max_pt(1) = std::max(max_pt(1), pt.y);
        max_pt(2) = std::max(max_pt(2), pt.z);
    }

    min_pt = min_pt.array() - cell_size;
    max_pt = max_pt.array() + cell_size;

    qDebug() << "Spatial Grid Bounds: ["
             << min_pt(0) << ", " << min_pt(1) << ", " << min_pt(2) << "] to ["
             << max_pt(0) << ", " << max_pt(1) << ", " << max_pt(2) << "]";

    Eigen::Vector3f dims = (max_pt - min_pt).array() / cell_size;
    grid_dimensions = dims.array().ceil();

    qDebug() << "Grid Dimensions: " << grid_dimensions(0) << " x "
             << grid_dimensions(1) << " x " << grid_dimensions(2);

    int total_cells = grid_dimensions.x() * grid_dimensions.y() * grid_dimensions.z();
    if (total_cells <= 0) {
        qDebug() << "WARNING: Grid has 0 or negative cells! Check min/max values.";
        total_cells = 1;
    }

    cells.resize(total_cells);
    qDebug() << "Allocated grid with " << total_cells << " cells";

    int points_assigned = 0;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        int idx = getCellIndex(pt.x, pt.y, pt.z);
        if (idx >= 0 && idx < total_cells) {
            cells[idx].point_indices.push_back(i);
            points_assigned++;
        }
    }
    qDebug() << "Assigned " << points_assigned << " points to grid cells";
}

int SpatialGrid::getCellIndex(float x, float y, float z) const {
    Eigen::Vector3f pos(x, y, z);
    Eigen::Vector3f rel_pos = pos - min_pt;
    Eigen::Vector3f idx = rel_pos.array() / cell_size;

    if (idx.x() < 0 || idx.y() < 0 || idx.z() < 0 ||
        idx.x() >= grid_dimensions.x() ||
        idx.y() >= grid_dimensions.y() ||
        idx.z() >= grid_dimensions.z()) {
        return -1;
    }

    return static_cast<int>(idx.x() +
                            idx.y() * grid_dimensions.x() +
                            idx.z() * grid_dimensions.x() * grid_dimensions.y());
}

std::vector<size_t> SpatialGrid::getNeighborIndices(float x, float y, float z, float radius) const {
    std::vector<size_t> result;
    int cells_to_check = static_cast<int>(std::ceil(radius / cell_size));
    cells_to_check = std::min(cells_to_check, 10);
    result.reserve(cells_to_check * cells_to_check * cells_to_check * 10);

    for (int dx = -cells_to_check; dx <= cells_to_check; ++dx) {
        for (int dy = -cells_to_check; dy <= cells_to_check; ++dy) {
            for (int dz = -cells_to_check; dz <= cells_to_check; ++dz) {
                float nx = x + dx * cell_size;
                float ny = y + dy * cell_size;
                float nz = z + dz * cell_size;

                int idx = getCellIndex(nx, ny, nz);
                if (idx >= 0 && idx < static_cast<int>(cells.size())) {
                    const auto& cell = cells[idx];
                    result.insert(result.end(), cell.point_indices.begin(), cell.point_indices.end());
                }
            }
        }
    }

    return result;
}

Eigen::Vector3i SpatialGrid::getGridCellForPoint(float x, float y, float z) const {
    Eigen::Vector3f pos(x, y, z);
    Eigen::Vector3f rel_pos = pos - min_pt;
    Eigen::Vector3f idx = rel_pos.array() / cell_size;

    return Eigen::Vector3i(
        static_cast<int>(std::floor(idx.x())),
        static_cast<int>(std::floor(idx.y())),
        static_cast<int>(std::floor(idx.z()))
        );
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloud(const std::string& filePath) {
    qDebug() << "Loading point cloud from:" << QString::fromStdString(filePath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filePath);

    if (!file.is_open()) {
        qDebug() << "Failed to open point cloud file:" << QString::fromStdString(filePath);
        return cloud;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        float x, y, z;
        if (iss >> x >> y >> z) {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->points.push_back(point);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    qDebug() << "Loaded" << cloud->points.size() << "points from" << QString::fromStdString(filePath);
    file.close();
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr loadCSVAsPointCloud(const std::string& filePath) {
    qDebug() << "Loading CSV from:" << QString::fromStdString(filePath);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filePath);

    if (!file.is_open()) {
        qDebug() << "Failed to open CSV file:" << QString::fromStdString(filePath);
        return cloud;
    }

    std::string line;
    std::getline(file, line);

    int lineNum = 1;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string x_str, y_str, z_str;
        std::getline(ss, x_str, ',');
        std::getline(ss, y_str, ',');
        std::getline(ss, z_str, ',');

        try {
            pcl::PointXYZ point;
            point.x = std::stof(x_str);
            point.y = std::stof(y_str);
            point.z = std::stof(z_str);
            cloud->points.push_back(point);
        } catch (const std::exception& e) {
            qDebug() << "Error parsing CSV line " << lineNum << ":" << QString::fromStdString(line) << "Error:" << e.what();
        }
        lineNum++;
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    qDebug() << "Loaded" << cloud->points.size() << "points from" << QString::fromStdString(filePath);
    file.close();
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr createColoredCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    uint8_t r, uint8_t g, uint8_t b) {
    qDebug() << "Creating colored cloud with" << cloud->points.size() << "points";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (!cloud || cloud->empty()) {
        qDebug() << "Input cloud is null or empty";
        return colored_cloud;
    }

    colored_cloud->points.reserve(cloud->points.size());
    for (const auto& pt : cloud->points) {
        pcl::PointXYZRGB rgbPt;
        rgbPt.x = pt.x;
        rgbPt.y = pt.y;
        rgbPt.z = pt.z;
        rgbPt.r = r;
        rgbPt.g = g;
        rgbPt.b = b;
        colored_cloud->points.push_back(rgbPt);
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = true;

    qDebug() << "Created colored cloud with" << colored_cloud->points.size() << "points";
    return colored_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampleCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    float leaf_size) {
    qDebug() << "Downsampling cloud with leaf size:" << leaf_size;

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*downsampled_cloud);

    qDebug() << "Downsampled from" << cloud->points.size() << "to" << downsampled_cloud->points.size();
    return downsampled_cloud;
}

CollisionDialog::CollisionDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CollisionDialog),
    animationTimer(new QTimer(this)),
    currentWaypointIndex(0),
    currentDistance(0.0f)
{
    ui->setupUi(this);

    vtkSmartPointer<CustomVTKOutputWindow> customOutput = vtkSmartPointer<CustomVTKOutputWindow>::New();
    vtkOutputWindow::SetInstance(customOutput);

    connect(ui->browsePointCloudButton, &QPushButton::clicked, this, &CollisionDialog::onBrowsePointCloud);
    connect(ui->browseMapButton, &QPushButton::clicked, this, &CollisionDialog::onBrowseMap);
    connect(ui->browseCsvButton, &QPushButton::clicked, this, &CollisionDialog::onBrowseCsv);
    connect(ui->pushButton, &QPushButton::clicked, this, &CollisionDialog::onApply);
    connect(animationTimer, &QTimer::timeout, this, &CollisionDialog::updateAnimation);

    collidedIndices.reserve(1000);

    qDebug() << "CollisionDialog constructed";
}

CollisionDialog::~CollisionDialog()
{
    if (viewer) {
        viewer->close();
    }
    delete ui;
    qDebug() << "CollisionDialog destroyed";
}

void CollisionDialog::onBrowsePointCloud()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    "Select Point Cloud File",
                                                    "",
                                                    "Point Files (*.pts *.txt *.xyz);;All Files (*)");
    if (!filePath.isEmpty()) {
        ui->pointCloudPathLineEdit->setText(filePath);
        qDebug() << "Selected point cloud file:" << filePath;
    }
}

void CollisionDialog::onBrowseMap()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    "Select Map File",
                                                    "",
                                                    "PLY Files (*.ply);;All Files (*)");
    if (!filePath.isEmpty()) {
        ui->mapPathLineEdit->setText(filePath);
        qDebug() << "Selected map file:" << filePath;
    }
}

void CollisionDialog::onBrowseCsv()
{
    QString filePath = QFileDialog::getOpenFileName(this,
                                                    "Select CSV File",
                                                    "",
                                                    "CSV Files (*.csv)");
    if (!filePath.isEmpty()) {
        ui->csvPathLineEdit->setText(filePath);
        qDebug() << "Selected CSV file:" << filePath;
    }
}

void CollisionDialog::setupVisualizer()
{
    qDebug() << "Setting up PCL visualizer";
    try {
        viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
        if (!viewer) {
            qDebug() << "Failed to create PCL visualizer";
            return;
        }
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        viewer->setCameraPosition(
            0, 0, 50,
            0, 0, 0,
            0, 1, 0
            );

        qDebug() << "PCL visualizer setup complete";
    } catch (const std::exception& e) {
        qDebug() << "Exception in setupVisualizer:" << e.what();
    }
}

void CollisionDialog::addGridToViewer(float cell_size)
{
    Eigen::Vector3f min_pt = pathGrid->getMinPt();
    Eigen::Vector3f grid_dims = pathGrid->getGridDimensions();
    float max_x = min_pt.x() + grid_dims.x() * cell_size;
    float max_y = min_pt.y() + grid_dims.y() * cell_size;
    float z = min_pt.z();

    int grid_step = 5;

    for (float x = min_pt.x(); x <= max_x; x += cell_size * grid_step) {
        std::string line_id = "grid_x_" + std::to_string(static_cast<int>((x - min_pt.x()) / cell_size));
        viewer->addLine(
            pcl::PointXYZ(x, min_pt.y(), z),
            pcl::PointXYZ(x, max_y, z),
            0.4, 0.4, 0.4,
            line_id
            );
    }

    for (float y = min_pt.y(); y <= max_y; y += cell_size * grid_step) {
        std::string line_id = "grid_y_" + std::to_string(static_cast<int>((y - min_pt.y()) / cell_size));
        viewer->addLine(
            pcl::PointXYZ(min_pt.x(), y, z),
            pcl::PointXYZ(max_x, y, z),
            0.4, 0.4, 0.4,
            line_id
            );
    }
}

void CollisionDialog::highlightGridCell(const Eigen::Vector3i& cell_idx, float cell_size, const std::string& id)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cubeCloud, centroid);

    Eigen::Vector3f min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
    Eigen::Vector3f max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& pt : cubeCloud->points) {
        min_pt(0) = std::min(min_pt(0), pt.x);
        min_pt(1) = std::min(min_pt(1), pt.y);
        min_pt(2) = std::min(min_pt(2), pt.z);
        max_pt(0) = std::max(max_pt(0), pt.x);
        max_pt(1) = std::max(max_pt(1), pt.y);
        max_pt(2) = std::max(max_pt(2), pt.z);
    }

    float cube_x_min = min_pt(0);
    float cube_x_max = max_pt(0);
    float cube_y_min = min_pt(1);
    float cube_y_max = max_pt(1);
    float cube_z_min = min_pt(2);
    float cube_z_max = max_pt(2);

    float padding = 0.05f;
    cube_x_min -= padding;
    cube_x_max += padding;
    cube_y_min -= padding;
    cube_y_max += padding;
    cube_z_min -= padding;
    cube_z_max += padding;

    viewer->removeShape(id);
    viewer->addCube(
        cube_x_min, cube_x_max,
        cube_y_min, cube_y_max,
        cube_z_min, cube_z_max,
        1.0, 1.0, 0.0,
        id
        );

    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        id
        );

    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
        2.0,
        id
        );
}

// Modify the checkCollisions function to focus on visualizing collision points
// Replace the checkCollisions function with this corrected version

void CollisionDialog::onApply()
{
    QString pointCloudPath = ui->pointCloudPathLineEdit->text();
    QString mapPath = ui->mapPathLineEdit->text();
    QString csvPath = ui->csvPathLineEdit->text();

    qDebug() << "Applying with files:"
             << "\nPoint Cloud:" << pointCloudPath
             << "\nMap:" << mapPath
             << "\nCSV:" << csvPath;

    if (pointCloudPath.isEmpty() || mapPath.isEmpty() || csvPath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select all required files.");
        qDebug() << "Missing file paths";
        return;
    }

    cubeCloud = loadPointCloud(pointCloudPath.toStdString());
    if (!cubeCloud || cubeCloud->empty()) {
        QMessageBox::critical(this, "Error", "Failed to load point cloud or it's empty.");
        qDebug() << "Cube cloud loading failed";
        return;
    }
    qDebug() << "Loaded cube with" << cubeCloud->size() << "points";

    pathCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    qDebug() << "Loading PLY file";
    if (pcl::io::loadPLYFile(mapPath.toStdString(), *pathCloud) == -1) {
        QMessageBox::critical(this, "Error", "Failed to load map file.");
        qDebug() << "PLY file loading failed";
        return;
    }
    qDebug() << "Loaded PLY with" << pathCloud->points.size() << "points";

    pathCloudRGB = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& pt : pathCloud->points) {
        pcl::PointXYZRGB rgbPt;
        rgbPt.x = pt.x;
        rgbPt.y = pt.y;
        rgbPt.z = pt.z;
        rgbPt.r = 255; // Initialize to white
        rgbPt.g = 255;
        rgbPt.b = 255;
        pathCloudRGB->points.push_back(rgbPt);
    }
    pathCloudRGB->width = pathCloudRGB->points.size();
    pathCloudRGB->height = 1;
    pathCloudRGB->is_dense = true;
    qDebug() << "Created path cloud with" << pathCloudRGB->points.size() << "points";

    const size_t MAX_PATH_POINTS = 100000;
    if (pathCloudRGB->points.size() > MAX_PATH_POINTS) {
        qDebug() << "Path cloud exceeds" << MAX_PATH_POINTS << "points, downsampling...";
        float leaf_size = 0.05f;
        pathCloudRGB = downsampleCloud(pathCloudRGB, leaf_size);
    }

    csvCloudRGB = createColoredCloud(loadCSVAsPointCloud(csvPath.toStdString()), 255, 255, 0);
    if (!csvCloudRGB || csvCloudRGB->empty()) {
        QMessageBox::critical(this, "Error", "Failed to load CSV as point cloud.");
        qDebug() << "CSV cloud loading failed";
        return;
    }
    qDebug() << "Loaded CSV cloud with" << csvCloudRGB->size() << "points";

    animatedCubeCloud = createColoredCloud(cubeCloud, 255, 255, 255);
    if (!animatedCubeCloud || animatedCubeCloud->empty()) {
        qDebug() << "Animated cube cloud is null or empty";
        return;
    }

    collidedIndices.clear();
    currentWaypointIndex = 0;
    currentDistance = 0.0f;

    float path_grid_cell_size = 1.0f;

    try {
        qDebug() << "Building spatial grid for visualization...";
        pathGrid = std::make_unique<SpatialGrid>(pathCloudRGB, path_grid_cell_size);
    }
    catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Failed to build spatial grid: %1").arg(e.what()));
        qDebug() << "Failed to build spatial grid:" << e.what();
        return;
    }

    setupVisualizer();
    if (!viewer) {
        QMessageBox::critical(this, "Error", "Failed to initialize visualizer.");
        qDebug() << "Visualizer initialization failed";
        return;
    }

    try {
        // Add path cloud with initial white color for PLY points
        viewer->addPointCloud<pcl::PointXYZRGB>(pathCloudRGB, "path_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "path_cloud");
        qDebug() << "Added path cloud (white)";

        for (size_t i = 0; i < csvCloudRGB->points.size(); ++i) {
            const auto& waypoint = csvCloudRGB->points[i];
            std::string sphere_id = "waypoint_sphere_" + std::to_string(i);
            viewer->addSphere(waypoint, 0.15, 1.0, 1.0, 0.0, sphere_id);
        }
        qDebug() << "Added CSV waypoints as yellow spheres";

        viewer->addPointCloud<pcl::PointXYZRGB>(animatedCubeCloud, "cube_cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cube_cloud");
        qDebug() << "Added initial cube cloud (white)";

        addGridToViewer(path_grid_cell_size);
        qDebug() << "Added grid visualization";

        if (!csvCloudRGB->empty()) {
            const auto& firstWaypoint = csvCloudRGB->points.front();
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*animatedCubeCloud, centroid);

            float dx = firstWaypoint.x - centroid[0];
            float dy = firstWaypoint.y - centroid[1];
            float dz = firstWaypoint.z - centroid[2];

            for (auto& pt : animatedCubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }
            for (auto& pt : cubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }

            viewer->updatePointCloud<pcl::PointXYZRGB>(animatedCubeCloud, "cube_cloud");
            qDebug() << "Positioned cube at first waypoint";
        }
    } catch (const std::exception& e) {
        qDebug() << "Exception in adding clouds:" << e.what();
        QMessageBox::warning(this, "Warning", QString("Error in visualization: %1").arg(e.what()));
        return;
    }

    animationTimer->start(33);
    qDebug() << "Started animation timer";
}

void CollisionDialog::checkCollisions()
{
    auto start_time = std::chrono::high_resolution_clock::now();

    // Compute the bounding box of the cube (animatedCubeCloud, loaded from .pts file)
    Eigen::Vector3f min_pt(FLT_MAX, FLT_MAX, FLT_MAX);
    Eigen::Vector3f max_pt(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& pt : animatedCubeCloud->points) {
        min_pt(0) = std::min(min_pt(0), pt.x);
        min_pt(1) = std::min(min_pt(1), pt.y);
        min_pt(2) = std::min(min_pt(2), pt.z);
        max_pt(0) = std::max(max_pt(0), pt.x);
        max_pt(1) = std::max(max_pt(1), pt.y);
        max_pt(2) = std::max(max_pt(2), pt.z);
    }

    // Define proximity range for collision detection
    const float PROXIMITY_RANGE = 0.5f;
    Eigen::Vector3f min_corner = min_pt - Eigen::Vector3f(PROXIMITY_RANGE, PROXIMITY_RANGE, PROXIMITY_RANGE);
    Eigen::Vector3f max_corner = max_pt + Eigen::Vector3f(PROXIMITY_RANGE, PROXIMITY_RANGE, PROXIMITY_RANGE);

    // Debug: Log the bounding box
    qDebug() << "Cube Bounding Box: Min [" << min_corner.x() << ", " << min_corner.y() << ", " << min_corner.z() << "]"
             << " Max [" << max_corner.x() << ", " << max_corner.y() << ", " << max_corner.z() << "]";

    // Reset non-collided PLY points to white
    for (size_t i = 0; i < pathCloudRGB->points.size(); ++i) {
        if (collidedIndices.find(i) == collidedIndices.end()) {
            pathCloudRGB->points[i].r = 255; // White for non-colliding PLY points
            pathCloudRGB->points[i].g = 255;
            pathCloudRGB->points[i].b = 255;
        }
    }

    // Check for collisions between cube (.pts file) and PLY file's point cloud
    size_t collidedPoints = 0;
    for (size_t i = 0; i < pathCloudRGB->points.size(); ++i) {
        const auto& pt = pathCloudRGB->points[i];
        Eigen::Vector3f point(pt.x, pt.y, pt.z);

        // Check if PLY point is within the cube's detection box
        if (point.x() >= min_corner.x() && point.x() <= max_corner.x() &&
            point.y() >= min_corner.y() && point.y() <= max_corner.y() &&
            point.z() >= min_corner.z() && point.z() <= max_corner.z()) {

            // Turn PLY point red to indicate collision
            pathCloudRGB->points[i].r = 255; // Bright red
            pathCloudRGB->points[i].g = 0;
            pathCloudRGB->points[i].b = 0;

            collidedPoints++;

            if (collidedIndices.find(i) == collidedIndices.end()) {
                collidedIndices.insert(i);
                qDebug() << "Collision detected at PLY point " << i << ": (" << pt.x << ", " << pt.y << ", " << pt.z << ")";
            }
        }
    }

    // Ensure cube points (from .pts file) remain white
    for (auto& pt : animatedCubeCloud->points) {
        pt.r = 255;
        pt.g = 255;
        pt.b = 255;
    }

    // Force re-render by removing and re-adding the point cloud to ensure color updates
    viewer->removePointCloud("path_cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(pathCloudRGB, "path_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "path_cloud");

    viewer->updatePointCloud<pcl::PointXYZRGB>(animatedCubeCloud, "cube_cloud");

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if (duration > 10 || collidedPoints > 0) {
        qDebug() << "Collision points in this frame:" << collidedPoints
                 << ", Total collision points:" << collidedIndices.size();
    }
}

// Modified updateAnimation to ensure viewer refreshes correctly
void CollisionDialog::updateAnimation()
{
    if (!viewer || viewer->wasStopped()) {
        animationTimer->stop();
        qDebug() << "Viewer stopped or null, stopping animation";
        return;
    }

    try {
        if (currentWaypointIndex >= csvCloudRGB->points.size() - 1) {
            qDebug() << "Animation complete - reached final waypoint";
            checkCollisions();

            QString resultMessage = QString("%1 collision points detected").arg(collidedIndices.size());
            qDebug() << resultMessage;

            animationTimer->stop();
            return;
        }

        const auto& start = csvCloudRGB->points[currentWaypointIndex];
        const auto& end = csvCloudRGB->points[currentWaypointIndex + 1];

        Eigen::Vector3f direction(end.x - start.x, end.y - start.y, end.z - start.z);
        float segmentDistance = direction.norm();

        if (segmentDistance < 1e-6) {
            qDebug() << "Skipping zero-distance segment at waypoint" << currentWaypointIndex;
            currentWaypointIndex++;
            currentDistance = 0.0f;
            return;
        }

        direction.normalize();

        const float BASE_STEP_SIZE = 0.1f;
        float adaptiveStepSize = std::min(BASE_STEP_SIZE, segmentDistance / 10.0f);
        adaptiveStepSize = std::max(adaptiveStepSize, 0.01f);

        currentDistance += adaptiveStepSize;

        if (currentDistance >= segmentDistance) {
            float remainingDistance = segmentDistance - (currentDistance - adaptiveStepSize);

            float dx = direction[0] * remainingDistance;
            float dy = direction[1] * remainingDistance;
            float dz = direction[2] * remainingDistance;

            for (auto& pt : animatedCubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }
            for (auto& pt : cubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }

            currentWaypointIndex++;
            currentDistance = 0.0f;
            qDebug() << "Reached waypoint" << currentWaypointIndex;

            std::string waypointMarker = "waypoint_marker_" + std::to_string(currentWaypointIndex);
            viewer->addSphere(end, 0.15, 1.0, 1.0, 0.0, waypointMarker);
        } else {
            float dx = direction[0] * adaptiveStepSize;
            float dy = direction[1] * adaptiveStepSize;
            float dz = direction[2] * adaptiveStepSize;

            for (auto& pt : animatedCubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }
            for (auto& pt : cubeCloud->points) {
                pt.x += dx;
                pt.y += dy;
                pt.z += dz;
            }
        }

        checkCollisions();

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*animatedCubeCloud, centroid);

        Eigen::Vector3i gridCell = pathGrid->getGridCellForPoint(centroid[0], centroid[1], centroid[2]);
        highlightGridCell(gridCell, 1.0f, "cube_highlight");

        if (viewer->contains("progress_text")) {
            viewer->removeShape("progress_text");
        }

        // Increase spin time to ensure updates are rendered
        viewer->spinOnce(20);
    }
    catch (const std::exception& e) {
        qDebug() << "Exception in updateAnimation:" << e.what();
        animationTimer->stop();
    }
}

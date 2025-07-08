#ifndef COLLISIONDIALOG_H
#define COLLISIONDIALOG_H

#include <QDialog>
#include <QTimer>
#include <QFileDialog>
#include <QMessageBox>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>
#include <unordered_set>

// Forward declarations
namespace Ui {
class CollisionDialog;
}

struct GridCell {
    std::vector<size_t> point_indices;
};

class SpatialGrid {
public:
    SpatialGrid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float cell_size_);
    int getCellIndex(float x, float y, float z) const;
    std::vector<size_t> getNeighborIndices(float x, float y, float z, float radius) const;
    Eigen::Vector3i getGridCellForPoint(float x, float y, float z) const;
    Eigen::Vector3f getMinPt() const { return min_pt; }
    Eigen::Vector3f getGridDimensions() const { return grid_dimensions; }

private:
    float cell_size;
    Eigen::Vector3f min_pt;
    Eigen::Vector3f grid_dimensions;
    std::vector<GridCell> cells;
};

class CollisionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CollisionDialog(QWidget *parent = nullptr);
    ~CollisionDialog();

private slots:
    void onBrowsePointCloud();
    void onBrowseMap();
    void onBrowseCsv();
    void onApply();
    void updateAnimation();

private:
    Ui::CollisionDialog *ui;
    QTimer* animationTimer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cubeCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pathCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pathCloudRGB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr csvCloudRGB;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr animatedCubeCloud;
    std::unique_ptr<SpatialGrid> pathGrid; // Still needed for visualization grid
    std::unordered_set<size_t> collidedIndices;
    size_t currentWaypointIndex;
    float currentDistance;

    void setupVisualizer();
    void addGridToViewer(float cell_size);
    void highlightGridCell(const Eigen::Vector3i& cell_idx, float cell_size, const std::string& id);
    void checkCollisions();
};

#endif // COLLISIONDIALOG_H

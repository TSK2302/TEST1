#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProgressDialog>
#include <QStandardItemModel>
#include <QThread>
#include <QProcess>
#include <QFileInfo>
#include "openglwidget.h"
#include "collisiondialog.h"
#include "generate2dgriddialog.h"
#include "generate3dgriddialog.h"
#include "convert2dto3dgriddialog.h"
#include "collision_detection.h"
#include "cloudcomparison.h"
#include "samplingprocessor.h"
#include "colorchanger.h"
#include "objectdetection.h"
#include "manageobjects.h"
#include "managetwins.h"
#include "replacewithtwins.h"
#include "clusterlist.h"
#include "colorassigner.h"
#include "gridpopulationdensitydialog.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVector3D>
#include <QSplitter>
#include <QGridLayout>
#include <QItemSelection>
#include <QDockWidget>
#include <QComboBox>
#include <QMap>
#include <QTableWidget>
#include <QStringList>
#include <QHeaderView>
#include <QBrush>
#include <QFont>
#include <QColor>
#include <QTabWidget>
#include <QRandomGenerator>
#include <QLabel>
#include <QImage>
#include <QPainter>
#include <QGraphicsView>
#include <QGraphicsScene>
#include "viewportcapturedialog.h"
#include <QPushButton>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>

class OpenGLWidget;
class TransformManager;

class PythonReportWorker : public QObject {
    Q_OBJECT
public:
    PythonReportWorker(const QString& pythonScript, const QString& dataPath, QObject* parent = nullptr)
        : QObject(parent), pythonScript(pythonScript), dataPath(dataPath) {}

public slots:
    void run() {
        QProcess pythonProcess;
        pythonProcess.setProgram("python");
        pythonProcess.setArguments({pythonScript, dataPath});
        pythonProcess.setWorkingDirectory(QFileInfo(pythonScript).path());

        pythonProcess.start();
        if (!pythonProcess.waitForFinished(30000)) {
            emit error("Python script execution timed out or failed!\nError: " + pythonProcess.errorString());
            pythonProcess.kill();
            emit finished(-1, "", pythonProcess.readAllStandardError());
            return;
        }

        int exitCode = pythonProcess.exitCode();
        QString output = pythonProcess.readAllStandardOutput();
        QString errors = pythonProcess.readAllStandardError();
        emit finished(exitCode, output, errors);
    }

signals:
    void finished(int exitCode, const QString& output, const QString& errors);
    void error(const QString& message);

private:
    QString pythonScript;
    QString dataPath;
};

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
    friend class TransformManager;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    OpenGLWidget* getActiveViewWidget() const {
        return m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    }

    OpenGLWidget* view;
    //preview
    struct PreviewModelData {
        QString name;
        std::vector<QVector3D> vertices;
        std::vector<QVector3D> normals;
        std::vector<unsigned int> indices;
        QVector3D min, max;
    };
    struct PreviewPointCloudData {
        QString name;
        std::vector<QVector3D> points;
        std::vector<QVector3D> colors;
        QVector3D min, max;
        int originalCount = 0;
    };

    OpenGLWidget* previewOpenGLWidget;
    QLabel* m_previewLabel = nullptr;
    QGraphicsView* m_previewGraphicsView = nullptr;
    QGraphicsScene* m_previewScene = nullptr;

    //preview

    struct ViewportData {
        QString name;
        QMatrix4x4 projectionMatrix;
        QVector3D cameraPosition;
        QVector3D cameraTarget;
        QVector3D cameraUp;
        QVector3D panOffset;
        float xRotation;
        float yRotation;
        float zoom;
    };



    //grid arrangement
    QPoint m_dragStartPosition;
    QFrame* m_draggedContainer = nullptr;
    QFrame* m_lastMaximizedContainer = nullptr;
    QVector<QFrame*> m_gridContainers;
    QVector<QByteArray> m_savedWindowGeometries;

    void maximizeContainer(QFrame* container, OpenGLWidget* view);
    void restoreGridLayout();
    void swapContainers(QFrame* source, QFrame* target);

    void fixViewportFlickering(OpenGLWidget* view);
    void resizeEvent(QResizeEvent* event) override;

    // For Console Logging
    void logToConsole(const QString& message, const QString& level = "INFO");
    //console

    //for FullScreen Action
    bool m_fullscreenMainWindow = false;   // tracks if the main window is fullscreen
    bool m_fullscreen3DView    = false;   // tracks if the 3D view is fullscreen
    QDialog *m_fullScreenDialog = nullptr; // container for the fullscreen 3D view
    QDialog* m_openGLFullScreenDialog = nullptr;
    QWidget* m_openGLPreviousParent = nullptr;
    QLayout* m_openGLPreviousLayout = nullptr;
    //fullscreen

public slots:
    void onShowGridToggled(bool checked);
    void onShowAxisToggled(bool checked);
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;
    //fullscreen slots
    void toggleFullScreenMainWindow();
    void toggleFullScreen3DView();


private slots:
    void on_actionGenerate_2D_grid_triggered();
    void on_actionOpen_triggered();
    void on_actionDelete_triggered();
    void on_actionPointPicker_triggered();
    void on_actionPick_Points_triggered();
    void on_hierarchyItemChanged(QStandardItem *item);
    void on_hierarchySelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
    void on_actionCDD_triggered();
    void createActions();
    void updateHierarchyView();
    void onNewViewWindowRequested();
    void arrangeViewsInGrid();
    void exitGridView();
    void setActiveViewWidget(OpenGLWidget* widget);
    void updateViewSelector();
    void applyFrontViewToActive();
    void applyTopViewToActive();
    void applySideViewToActive();
    void applyIsometricViewToActive();
    void updateTransformProperties();
    void setupTransformConnections();
    void onPositionChanged();
    void onRotationChanged();
    void onScaleChanged();
    void applyTransformToSelection();
    void on_actionICP_triggered();
    void on_actionUser_Capture_triggered();
    void applyViewportByName(const QString& viewportName);
    void setupViewportContextMenu();
    void onHierarchyContextMenuRequested(const QPoint& pos);
    void editViewport(const QString& viewportName);
    void on_actionSegmentation_triggered();
    void on_actionMulti_Grid_triggered();
    void on_actionGenerate_3D_grid_triggered();
    void on_action2D_to_3D_Grid_triggered();
    void on_actionGrid_Visibility_triggered();
    void on_actionDown_Sample_triggered();
    void on_actionUp_Sample_triggered();
    void on_actionSub_Sample_triggered();
    void on_actionSampling_triggered();
    void on_samplingApplied(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& operation);
    void on_actionRun_Collision_Map_triggered();
    void onAssetTabChanged(int index);
    void on_fileSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
    void on_browseButton_clicked();
    void on_refreshAssetsButton_clicked();
    void on_actionCluster_2_triggered();
    void on_actionIdentify_Similar_Clusters_triggered();
    void on_actionSlection_Capture_triggered();
    void updateViewportCapturesInHierarchy();
    void on_actionHeatmap_triggered();
    void on_deleteObjectButton_clicked();
    void on_propertyButton_clicked();
    void on_addObjectButton_clicked();
    void on_actionRecognize_Objects_triggered();
    void on_actionManage_Objects_triggered();
    void on_actionCluster_list_triggered();
    void on_actionManage_Twins_triggered();
    void on_actionObject_detection_Report_triggered();
    void on_actionCDD_analysis_triggered();
    void on_actionReplace_with_Twins_triggered();
    void applyOrthographicViewToActive();
    void applyBottomViewToActive();
    void on_actionColor_Assigner_triggered();
    void applyClusterColorsToPointCloud(const QString& pointCloudName, const std::map<int, QVector3D>& clusterColorMap);
    void on_actionGrid_Population_density_triggered();
    void on_actionExport_triggered();
    // Toggle button slots
    void on_togglePropertiesDock_clicked();
    void on_toggleHierarchyDock_clicked();
    // Update button positions
    void updateToggleButtonPositions();

private:
    Ui::MainWindow *ui;
    OpenGLWidget *openGLWidget;
    QStandardItemModel *hierarchyModel;
    QThread *pclVisualizerThread;
    QTabWidget* hierarchyTabWidget;
    CloudComparison *cloudComparison;
    SamplingProcessor *samplingProcessor;
    QFileSystemModel *fileSystemModel;
    QTableWidget* m_propertiesTable;
    QString currentDirectory;
    std::map<std::string, int> m_clusterCounts;
    std::vector<int> m_detectedLabels;
    QVector<Vertex> m_detectedPoints;
    int m_numEpochs;           // Store number of epochs
    float m_learningRate;      // Store learning rate
    QString m_trainingFolder;  // Store training folder path
    float m_accuracy;

    // Toggle button declarations
    QPushButton* m_propertiesDockToggleButton;
    QPushButton* m_hierarchyDockToggleButton;
    QPropertyAnimation* m_propertiesButtonAnimation;
    QPropertyAnimation* m_hierarchyButtonAnimation;
    QParallelAnimationGroup* m_animationGroup;

    void setupButtonHoverEffects();

    QPropertyAnimation* m_propertiesDockAnimation = nullptr;
    QPropertyAnimation* m_hierarchyDockAnimation = nullptr;
    bool m_propertiesDockAnimating = false;
    bool m_hierarchyDockAnimating = false;

    void optimizePropertyTableLayout(QTableWidget* table);
    bool eventFilter(QObject* obj, QEvent* event) override;
    void initializeTransformManager();
    void setupTransformForView(OpenGLWidget* viewWidget);
    void updatePropertiesView();
    void setupPropertyTab();
    void addPropertyGroupHeader(QTableWidget* table, int row, const QString& title);
    void addPropertyRow(QTableWidget* table, int row, const QString& property, const QString& value);
    void loadFiles(const QStringList& filePaths);
    TransformManager *transformManager = nullptr;

    struct ViewWindow {
        QMainWindow* window;
        OpenGLWidget* openGLWidget;
        TransformManager* transformManager;
    };
    QList<ViewWindow> m_viewWindows;
    OpenGLWidget* m_activeViewWidget = nullptr;
    bool m_inGridViewMode = false;
    QGridLayout* m_gridLayout = nullptr;
    QSplitter* m_mainSplitter = nullptr;
    QList<QDockWidget*> m_visibleDocks;
    QComboBox* m_3DViewSelector = nullptr;
    bool m_lockScaleProportions = true;
    bool exportPointCloud(const QString& pointCloudName, const QString& filePath, const QString& format);
    bool exportModel(const QString& modelName, const QString& filePath, const QString& format);

    bool isPlyPointCloud(const QString& filePath);
    bool loadPlyPointCloud(const QString& filePath, std::vector<QVector3D>& points, std::vector<QVector3D>& colors);
    void updatePreview(const QString& filePath);
    void updateFileView(const QString& path);
    QString m_currentProjectFile;
    void exportViewportCapture(OpenGLWidget* view, const QString& captureName);
    void importViewportCapture();
    void initializeViewportCaptureSystem();
    void updateViewportFromSelection(const QString& viewportName);
    void applyViewportParameters(const ViewportData& viewportData, OpenGLWidget* targetWidget);
    QList<ViewportData> m_capturedViewports;
    void setupFileContextMenu();
    void connectSignals();
    void initializeFileBrowsing();
    void setupPreviewWidget();
    void loadPointCloudPreview(const QString& filePath, const QString& extension);
    void loadModelPreview(const QString& filePath, const QString& extension);
    void connectAssetBrowserSignals();
    void updateFileProperties(const QString& filePath);
    void updateDirectoryProperties(const QString& dirPath);
    void addPropertyHeader(QTableWidget* table, int row, const QString& title);
    void addProperty(QTableWidget* table, int row, const QString& name, const QString& value);
    void addPropertyToGroup(QStandardItem* group, const QString& name, const QString& value);
    void populatePropertiesTreeView(const QFileInfo& fileInfo);
    void setupAssetBrowser();
    void generatePointCloudPreview(const QString& filePath);
    void setupPreviewArea();
    void onAssetDockVisibilityChanged(bool visible);
    void loadPLYPointCloud(const QString& filePath, const QString& fileName, OpenGLWidget* targetView);
    void buildModelHierarchyInTree(const OpenGLWidget::Model& model, OpenGLWidget* view);
    QStandardItem* createModelNodeItem(
        const OpenGLWidget::Model& model,
        int nodeIndex,
        bool isRoot,
        OpenGLWidget* view);
    void showFbxInformation(const QString& filePath);
    void addViewportToHierarchy(const ViewportData& viewport, OpenGLWidget* view);
};
void customizeUI();

class DraggableViewFrame : public QFrame
{
    Q_OBJECT

public:
    explicit DraggableViewFrame(QWidget *parent = nullptr) : QFrame(parent) {setAcceptDrops(true);}
    QPoint m_dragStartPosition;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dropEvent(QDropEvent *event) override;
};

#endif // MAINWINDOW_H

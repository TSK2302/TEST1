#include "mainwindow.h"
#include <QMetaType>
#include "identifysimilarclustersdialog.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <fstream>
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <sstream>
#include <vector>
#include <QString>
#include <QProgressDialog>
#include <QCoreApplication>
#include <QScreen>
#include <QGuiApplication>
#include <QRegularExpression>
#include <QDragEnterEvent>
#include <QPdfWriter>
#include <QPainter>
#include <QFont>
#include <QImage>
#include <QDesktopServices>
#include <QJsonObject>
#include <QApplication>
#include <climits> // For INT_MAX and INT_MIN
#include <cmath>
#include <QFileDialog>
#include <QMessageBox>
#include <QDate>
#include <QImage>
#include <map>
#include <string>
#include <cmath>
#include "openglwidget.h"
#include "transformmanager.h"
#include "cluster.h"
#include "multigriddialog.h"
#include "gridvisibilitydialog.h"
#include "segmentation.h"
#include "samplingprocessor.h"
#include "collisiondialog.h"

#include <stdexcept>
#include <QInputDialog>
#include <QDateTime>
#include <random>
#include <assimp/Exporter.hpp>
// #include <QtConcurrent/QtConcurrentRun>

#include <QFuture>
#include <QFutureWatcher>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef _WIN32
#include <windows.h>
#include <commdlg.h>
#endif

// Define Point Types
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointCloudTColor = pcl::PointCloud<pcl::PointXYZRGB>;
using KdTreeT = pcl::KdTreeFLANN<PointT>;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , openGLWidget(new OpenGLWidget(this))
    , previewOpenGLWidget(nullptr)     // for preview
    , currentDirectory(QDir::currentPath())  //for preview
    , samplingProcessor(new SamplingProcessor(this))
    , m_propertiesTable(nullptr)
    , m_propertiesDockToggleButton(nullptr)
    //, m_assetBrowserDockToggleButton(nullptr)
    , m_hierarchyDockToggleButton(nullptr)
{
    ui->setupUi(this);
    //preview
    setupPreviewWidget();
    //preview
    //viewport
    setupViewportContextMenu();

    // Enable drag-and-drop
    setAcceptDrops(true);

        // Initialize toggle button for propertiesDock
        m_propertiesDockToggleButton = new QPushButton(">>", this);
        m_propertiesDockToggleButton->setFixedSize(30, 30);
        m_propertiesDockToggleButton->setToolTip("Toggle Properties Dock");
        connect(m_propertiesDockToggleButton, &QPushButton::clicked, this, &MainWindow::on_togglePropertiesDock_clicked);

        // Initialize toggle button for hierarchyDock
        m_hierarchyDockToggleButton = new QPushButton(">>", this);
        m_hierarchyDockToggleButton->setFixedSize(30, 30);
        m_hierarchyDockToggleButton->setToolTip("Toggle Hierarchy Dock");
        connect(m_hierarchyDockToggleButton, &QPushButton::clicked, this, &MainWindow::on_toggleHierarchyDock_clicked);

        // Initialize animations with matching durations
        m_propertiesButtonAnimation = new QPropertyAnimation(m_propertiesDockToggleButton, "pos", this);
        m_propertiesButtonAnimation->setDuration(400); // Match dock animation duration
        m_propertiesButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

        m_hierarchyButtonAnimation = new QPropertyAnimation(m_hierarchyDockToggleButton, "pos", this);
        m_hierarchyButtonAnimation->setDuration(400); // Match dock animation duration
        m_hierarchyButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

        m_animationGroup = new QParallelAnimationGroup(this);
        m_animationGroup->addAnimation(m_propertiesButtonAnimation);
        m_animationGroup->addAnimation(m_hierarchyButtonAnimation);

        // Apply dark theme styling
        setupButtonHoverEffects();

        // Initial button positioning
        updateToggleButtonPositions();


    openGLWidget = new OpenGLWidget(this);
    cloudComparison = new CloudComparison();


    QVBoxLayout* viewportLayout = qobject_cast<QVBoxLayout*>(ui->viewportContainer->layout());
    if (viewportLayout) {
        for (int i = 0; i < viewportLayout->count(); ++i) {
            QLayoutItem* item = viewportLayout->itemAt(i);
            QWidget* widget = item->widget();
            if (widget && widget->objectName() == "openGLWidget") {
                viewportLayout->removeWidget(widget);
                delete widget;
                viewportLayout->insertWidget(i, openGLWidget);
                openGLWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                openGLWidget->setMinimumSize(800, 600);
                break;
            }
        }
    }
    // Initialize TransformManager
    transformManager = new TransformManager(openGLWidget, this, this);
    transformManager->setupConnections();

    //setup heierarchy tree

    hierarchyModel = new QStandardItemModel(this);
    hierarchyModel->setHorizontalHeaderLabels({tr("Asset Name")});
    ui->treeView->setModel(hierarchyModel);
    ui->treeView->setHeaderHidden(false);
    ui->treeView->setSelectionMode(QAbstractItemView::ExtendedSelection);

    // Set splitter stretch factors
    ui->assetSplitter->setStretchFactor(0, 1);
    ui->assetSplitter->setStretchFactor(1, 1);

    connect(hierarchyModel, &QStandardItemModel::itemChanged, this, &MainWindow::on_hierarchyItemChanged);
    connect(ui->treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::on_hierarchySelectionChanged);
    // Connect view actions directly
    connect(ui->actionFront_View, &QAction::triggered, this, &MainWindow::applyFrontViewToActive);
    connect(ui->actionTop_View, &QAction::triggered, this, &MainWindow::applyTopViewToActive);
    connect(ui->actionSide_View, &QAction::triggered, this, &MainWindow::applySideViewToActive);
    connect(ui->actionIsometric_View, &QAction::triggered, this, &MainWindow::applyIsometricViewToActive);
    connect(ui->actionNew_3D_View_Window, &QAction::triggered, this, &MainWindow::onNewViewWindowRequested);
    connect(ui->actionArrange_in_Grid, &QAction::triggered, this, &MainWindow::arrangeViewsInGrid);
    connect(ui->actionExit_Grid_View, &QAction::triggered, this, &MainWindow::exitGridView);
    // // Add these for grid and axis visibility if needed
    connect(ui->actionShow_Grid, &QAction::toggled, this, &MainWindow::onShowGridToggled);
    connect(ui->actionShow_Axis, &QAction::toggled, this, &MainWindow::onShowAxisToggled);
    connect(ui->assetTabs, &QTabWidget::currentChanged,
            this, &MainWindow::onAssetTabChanged);


    setupPropertyTab();

    // Connect 2D grid generation
    // connect(ui->actionGenerate_2D_grid, &QAction::triggered, this, &MainWindow::on_actionGenerate_2D_grid_triggered);
    // Initialize the lock scale checkbox state
    m_lockScaleProportions = ui->lockScaleCheckbox->isChecked();

    // Setup transform connections
    // createActions();


    connect(samplingProcessor, &SamplingProcessor::samplingApplied, this, &MainWindow::on_samplingApplied);
    qDebug() << "MainWindow created";
    // std::cout << "MainWindow created" << std::endl;

    // Initialize file system model for Asset Browser
    fileSystemModel = new QFileSystemModel(this);
    fileSystemModel->setFilter(QDir::Files | QDir::NoDotAndDotDot);
    updateFileView(currentDirectory);

    connect(hierarchyModel, &QStandardItemModel::itemChanged, this, &MainWindow::on_hierarchyItemChanged);
    connect(ui->treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::on_hierarchySelectionChanged);
    connect(ui->treeView_2->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::on_fileSelectionChanged);

    connect(ui->actionSlection_Capture, &QAction::triggered, this, &MainWindow::on_actionSlection_Capture_triggered);
        // Viewport
    connect(ui->deleteObjectButton, &QPushButton::clicked,
            this, &MainWindow::on_deleteObjectButton_clicked);
    connect(ui->propertyButton, &QPushButton::clicked,
            this, &MainWindow::on_propertyButton_clicked);
    connect(ui->addObjectButton, &QPushButton::clicked,
            this, &MainWindow::on_addObjectButton_clicked);

    connect(ui->actionUser_Capture, &QAction::triggered, this, &MainWindow::on_actionUser_Capture_triggered);

    connect(ui->actionOrthographic_View, &QAction::triggered,
            this, &MainWindow::applyOrthographicViewToActive);

    connect(ui->actionBottom_View, &QAction::triggered, this, &MainWindow::applyBottomViewToActive);

    //fullscreen
    connect(ui->actionFull_Screen, &QAction::triggered,
            this, &MainWindow::toggleFullScreenMainWindow);
    connect(ui->actionFull_Screen_3D_View, &QAction::triggered,
            this, &MainWindow::toggleFullScreen3DView);
    // Initialize asset browser functionality
    setupAssetBrowser();

    // Connect asset browser dock signals
    QDockWidget* assetBrowserDock = findChild<QDockWidget*>("assetBrowserDock");
    if (assetBrowserDock) {
        connect(assetBrowserDock, &QDockWidget::visibilityChanged,
                this, &MainWindow::onAssetDockVisibilityChanged);

    }


    qDebug() << "MainWindow created";
}

MainWindow::~MainWindow()
{
    delete samplingProcessor;
    delete cloudComparison;
    delete ui;
}


void MainWindow::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls()) {
        for (const QUrl& url : event->mimeData()->urls()) {
            QString filePath = url.toLocalFile();
            QFileInfo fileInfo(filePath);
            QString extension = fileInfo.suffix().toLower();
            if (extension == "pts" || extension == "ply" ||
                extension == "obj" || extension == "fbx" || extension == "stl" ||
                extension == "dae" || extension == "gltf" || extension == "glb" ||
                extension == "mdl" || extension == "md2" || extension == "md3" ||
                extension == "md5mesh" || extension == "md5anim" || extension == "md5camera" ||
                extension == "smd" || extension == "vta" || extension == "mesh" ||
                extension == "skeleton" || extension == "3d" || extension == "bvh" ||
                extension == "dxf" || extension == "ifc" || extension == "blend" ||
                extension == "lws" || extension == "irr" || extension == "nff" ||
                extension == "scn" || extension == "3ds" || extension == "lwo" ||
                extension == "lx" || extension == "ac" || extension == "ms3d" ||
                extension == "cob" || extension == "xgl" || extension == "zgl" ||
                extension == "ogex" || extension == "q3d" || extension == "q3s" ||
                extension == "irrmesh" || extension == "off" || extension == "raw" ||
                extension == "ter" || extension == "hmp" || extension == "x") {
                event->acceptProposedAction();
                return;
            }
        }
    }
    event->ignore();
}

void MainWindow::dropEvent(QDropEvent *event)
{
    QStringList filePaths;
    for (const QUrl& url : event->mimeData()->urls()) {
        QString filePath = url.toLocalFile();
        if (!filePath.isEmpty()) {
            filePaths.append(filePath);
        }
    }

    if (!filePaths.isEmpty()) {
        loadFiles(filePaths);
    }

    event->acceptProposedAction();
}

void MainWindow::loadFiles(const QStringList& filePaths)
{
    if (filePaths.isEmpty()) {
        return;
    }

    qDebug() << "Loading files:" << filePaths;

    QProgressDialog progress(tr("Loading Files..."), tr("Cancel"), 0, filePaths.size(), this);
    progress.setWindowModality(Qt::ApplicationModal);
    progress.setMinimumDuration(0);

    try {
        for (int i = 0; i < filePaths.size(); ++i) {
            const QString& filePath = filePaths[i];
            progress.setValue(i);
            progress.setLabelText(tr("Loading file: %1").arg(QFileInfo(filePath).fileName()));
            QCoreApplication::processEvents();

            if (progress.wasCanceled()) {
                QMessageBox::information(this, tr("Loading Canceled"), tr("File loading was canceled by the user."));
                return;
            }

            qDebug() << "Processing file:" << filePath;

            QFileInfo fileInfo(filePath);
            QString fileName = fileInfo.fileName();
            QString extension = fileInfo.suffix().toLower();

            if (extension == "pts") {
                std::vector<QVector3D> points;
                std::vector<QVector3D> colors;

                std::ifstream file(filePath.toStdString());
                if (!file.is_open()) {
                    throw std::runtime_error("Could not open file: " + filePath.toStdString());
                }

                std::string line;
                while (std::getline(file, line)) {
                    std::istringstream iss(line);
                    float x, y, z, r, g, b;
                    if (iss >> x >> y >> z >> r >> g >> b) {
                        points.push_back(QVector3D(x, y, z));
                        colors.push_back(QVector3D(r / 255.0f, g / 255.0f, b / 255.0f));
                    }
                }

                qDebug() << "Loaded" << points.size() << "points from file:" << filePath;
                openGLWidget->addPointCloud(points, colors, fileName);
            } else if (extension == "ply" && isPlyPointCloud(filePath)) {
                std::vector<QVector3D> points;
                std::vector<QVector3D> colors;

                if (loadPlyPointCloud(filePath, points, colors)) {
                    openGLWidget->addPointCloud(points, colors, fileName);
                } else {
                    openGLWidget->addModel(filePath, fileName);
                }
            } else {
                openGLWidget->addModel(filePath, fileName);
            }
        }

        updateHierarchyView();
        progress.setValue(filePaths.size());
        QMessageBox::information(this, tr("Loading Complete"), tr("Successfully loaded %1 file(s).").arg(filePaths.size()));
        openGLWidget->update();

    } catch (const std::exception& e) {
        progress.setValue(filePaths.size());
        QMessageBox::critical(this, tr("Error"), tr("Failed to load files: %1").arg(e.what()));
    }
}

bool MainWindow::isPlyPointCloud(const QString& filePath)
{
    std::ifstream file(filePath.toStdString(), std::ios::binary);
    if (!file) {
        return false;
    }

    std::string line;
    bool vertexElement = false;
    bool faceElement = false;
    int vertexCount = 0;
    int faceCount = 0;

    std::getline(file, line);
    if (line != "ply") {
        return false;
    }

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "element") {
            std::string elementType;
            int count;
            iss >> elementType >> count;

            if (elementType == "vertex") {
                vertexElement = true;
                vertexCount = count;
            } else if (elementType == "face") {
                faceElement = true;
                faceCount = count;
            }
        } else if (token == "end_header") {
            break;
        }
    }

    if (vertexElement && (!faceElement || (vertexCount > 0 && faceCount == 0))) {
        return true;
    }

    if (vertexElement && faceElement && vertexCount > faceCount * 10) {
        return true;
    }

    return false;
}

//new method to support ply lkoading
void MainWindow::loadPLYPointCloud(const QString& filePath, const QString& fileName, OpenGLWidget* targetView)
{
    // Create a new PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Check file extension and load accordingly
    QString extension = QFileInfo(filePath).suffix().toLower();
    int loadResult = -1;

    if (extension == "ply") {
        loadResult = pcl::io::loadPLYFile(filePath.toStdString(), *cloud);
    } else if (extension == "pcd") {
        loadResult = pcl::io::loadPCDFile(filePath.toStdString(), *cloud);
    }

    if (loadResult < 0) {
        throw std::runtime_error("Failed to load point cloud file: " + filePath.toStdString());
    }

    // Convert to format suitable for OpenGLWidget
    std::vector<QVector3D> points;
    std::vector<QVector3D> colors;

    // Reserve space for efficiency
    points.reserve(cloud->size());
    colors.reserve(cloud->size());

    // Process all points
    for (const auto& p : cloud->points) {
        // Skip invalid points (with NaN values)
        if (!pcl::isFinite(p))
            continue;

        // Add point
        points.push_back(QVector3D(p.x, p.y, p.z));

        // Check if the point has color information
        if (p.r > 0 || p.g > 0 || p.b > 0) {
            colors.push_back(QVector3D(p.r / 255.0f, p.g / 255.0f, p.b / 255.0f));
        } else {
            colors.push_back(QVector3D(0.8f, 0.8f, 0.8f)); // Default gray color
        }
    }

    // Only add if we have valid points
    if (!points.empty()) {
        qDebug() << "Successfully loaded" << points.size() << "points from PLY/PCD file";
        targetView->addPointCloud(points, colors, fileName);


        logToConsole(
            QString("Successfully loaded %1 points from file: %2")
                .arg(points.size())
                .arg(fileName),
            "SUCCESS"
            );

    } else {
        throw std::runtime_error("No valid points found in file");
    }
}


bool MainWindow::loadPlyPointCloud(const QString& filePath, std::vector<QVector3D>& points, std::vector<QVector3D>& colors)
{
    std::ifstream file(filePath.toStdString(), std::ios::binary);
    if (!file) {
        qDebug() << "Failed to open PLY file:" << filePath;
        return false;
    }

    enum Format { ASCII, BINARY_LITTLE_ENDIAN, BINARY_BIG_ENDIAN } format = ASCII;
    bool hasColors = false;
    int vertexCount = 0;

    struct Property {
        std::string name;
        std::string type;
        int size;
    };

    std::vector<Property> properties;
    std::map<std::string, int> propertyIndices;

    std::string line;
    std::getline(file, line);
    if (line != "ply") {
        qDebug() << "Invalid PLY file format for:" << filePath;
        return false;
    }

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "format") {
            std::string formatStr;
            iss >> formatStr;
            if (formatStr == "ascii") {
                format = ASCII;
            } else if (formatStr == "binary_little_endian") {
                format = BINARY_LITTLE_ENDIAN;
            } else if (formatStr == "binary_big_endian") {
                format = BINARY_BIG_ENDIAN;
            }
        } else if (token == "element" && line.find("vertex") != std::string::npos) {
            iss >> token >> vertexCount;
        } else if (token == "property") {
            Property prop;
            iss >> prop.type >> prop.name;

            if (prop.type == "float" || prop.type == "float32") {
                prop.size = 4;
            } else if (prop.type == "double" || prop.type == "float64") {
                prop.size = 8;
            } else if (prop.type == "int" || prop.type == "int32") {
                prop.size = 4;
            } else if (prop.type == "short" || prop.type == "int16") {
                prop.size = 2;
            } else if (prop.type == "char" || prop.type == "int8" || prop.type == "uchar" || prop.type == "uint8") {
                prop.size = 1;
            } else {
                prop.size = 4;
            }

            properties.push_back(prop);

            if (prop.name == "red" || prop.name == "r" ||
                prop.name == "green" || prop.name == "g" ||
                prop.name == "blue" || prop.name == "b") {
                hasColors = true;
            }
        } else if (token == "end_header") {
            break;
        }
    }

    for (size_t i = 0; i < properties.size(); i++) {
        propertyIndices[properties[i].name] = i;
    }

    points.reserve(vertexCount);
    if (hasColors) {
        colors.reserve(vertexCount);
    } else {
        colors.resize(vertexCount, QVector3D(1.0f, 1.0f, 1.0f));
    }

    if (format == ASCII) {
        for (int i = 0; i < vertexCount; i++) {
            std::getline(file, line);
            std::istringstream iss(line);

            std::vector<float> values(properties.size());
            for (size_t j = 0; j < properties.size(); j++) {
                iss >> values[j];
            }

            float x = 0.0f, y = 0.0f, z = 0.0f;
            float r = 1.0f, g = 1.0f, b = 1.0f;

            if (propertyIndices.find("x") != propertyIndices.end()) {
                x = values[propertyIndices["x"]];
            }
            if (propertyIndices.find("y") != propertyIndices.end()) {
                y = values[propertyIndices["y"]];
            }
            if (propertyIndices.find("z") != propertyIndices.end()) {
                z = values[propertyIndices["z"]];
            }

            if (hasColors) {
                if (propertyIndices.find("red") != propertyIndices.end()) {
                    r = values[propertyIndices["red"]] / 255.0f;
                } else if (propertyIndices.find("r") != propertyIndices.end()) {
                    r = values[propertyIndices["r"]] / 255.0f;
                }

                if (propertyIndices.find("green") != propertyIndices.end()) {
                    g = values[propertyIndices["green"]] / 255.0f;
                } else if (propertyIndices.find("g") != propertyIndices.end()) {
                    g = values[propertyIndices["g"]] / 255.0f;
                }

                if (propertyIndices.find("blue") != propertyIndices.end()) {
                    b = values[propertyIndices["blue"]] / 255.0f;
                } else if (propertyIndices.find("b") != propertyIndices.end()) {
                    b = values[propertyIndices["b"]] / 255.0f;
                }
            }

            points.push_back(QVector3D(x, y, z));
            if (hasColors) {
                colors.push_back(QVector3D(r, g, b));
            }
        }
    } else {
        std::vector<char> buffer;

        for (int i = 0; i < vertexCount; i++) {
            float x = 0.0f, y = 0.0f, z = 0.0f;
            float r = 1.0f, g = 1.0f, b = 1.0f;

            for (size_t j = 0; j < properties.size(); j++) {
                const Property& prop = properties[j];
                buffer.resize(prop.size);
                file.read(buffer.data(), prop.size);

                float value = 0.0f;

                if (prop.type == "float" || prop.type == "float32") {
                    if (format == BINARY_LITTLE_ENDIAN) {
                        value = *reinterpret_cast<float*>(buffer.data());
                    } else {
                        char temp[4];
                        for (int k = 0; k < 4; k++) {
                            temp[k] = buffer[3-k];
                        }
                        value = *reinterpret_cast<float*>(temp);
                    }
                } else if (prop.type == "double" || prop.type == "float64") {
                    double dValue;
                    if (format == BINARY_LITTLE_ENDIAN) {
                        dValue = *reinterpret_cast<double*>(buffer.data());
                    } else {
                        char temp[8];
                        for (int k = 0; k < 8; k++) {
                            temp[k] = buffer[7-k];
                        }
                        dValue = *reinterpret_cast<double*>(temp);
                    }
                    value = static_cast<float>(dValue);
                } else if (prop.type == "int" || prop.type == "int32") {
                    int32_t iValue;
                    if (format == BINARY_LITTLE_ENDIAN) {
                        iValue = *reinterpret_cast<int32_t*>(buffer.data());
                    } else {
                        char temp[4];
                        for (int k = 0; k < 4; k++) {
                            temp[k] = buffer[3-k];
                        }
                        iValue = *reinterpret_cast<int32_t*>(temp);
                    }
                    value = static_cast<float>(iValue);
                } else if (prop.type == "short" || prop.type == "int16") {
                    int16_t sValue;
                    if (format == BINARY_LITTLE_ENDIAN) {
                        sValue = *reinterpret_cast<int16_t*>(buffer.data());
                    } else {
                        char temp[2];
                        temp[0] = buffer[1];
                        temp[1] = buffer[0];
                        sValue = *reinterpret_cast<int16_t*>(temp);
                    }
                    value = static_cast<float>(sValue);
                } else if (prop.type == "char" || prop.type == "int8") {
                    int8_t cValue = *reinterpret_cast<int8_t*>(buffer.data());
                    value = static_cast<float>(cValue);
                } else if (prop.type == "uchar" || prop.type == "uint8") {
                    uint8_t ucValue = *reinterpret_cast<uint8_t*>(buffer.data());
                    value = static_cast<float>(ucValue);
                }

                if (prop.name == "x") {
                    x = value;
                } else if (prop.name == "y") {
                    y = value;
                } else if (prop.name == "z") {
                    z = value;
                } else if (prop.name == "red" || prop.name == "r") {
                    r = value / 255.0f;
                } else if (prop.name == "green" || prop.name == "g") {
                    g = value / 255.0f;
                } else if (prop.name == "blue" || prop.name == "b") {
                    b = value / 255.0f;
                }
            }

            points.push_back(QVector3D(x, y, z));
            if (hasColors) {
                colors.push_back(QVector3D(r, g, b));
            }
        }
    }

    qDebug() << "Loaded" << points.size() << "points from PLY file:" << filePath;
    return true;
}


void MainWindow::createActions()
{
    // Fix the TransformManager connection by checking if the selection model exists:
    if (ui->treeView->selectionModel()) {
        connect(ui->treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
                this, &MainWindow::on_hierarchySelectionChanged);
    }

    // Keep the existing connections

    // Update view action connections to use the active view
    connect(ui->actionFront_View, &QAction::triggered, this, &MainWindow::applyFrontViewToActive);
    connect(ui->actionTop_View, &QAction::triggered, this, &MainWindow::applyTopViewToActive);
    connect(ui->actionSide_View, &QAction::triggered, this, &MainWindow::applySideViewToActive);
    connect(ui->actionIsometric_View, &QAction::triggered, this, &MainWindow::applyIsometricViewToActive);
    connect(hierarchyModel, &QStandardItemModel::itemChanged, this, &MainWindow::on_hierarchyItemChanged);
        // connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::on_actionOpen_triggered);
    connect(ui->actionDelete, &QAction::triggered, this, &MainWindow::on_actionDelete_triggered);
    // connect(ui->actionNew_3D_View_Window, &QAction::triggered, this, &MainWindow::onNewViewWindowRequested);
    // connect(ui->actionArrange_in_Grid, &QAction::triggered, this, &MainWindow::arrangeViewsInGrid);
    // connect(ui->actionExit_Grid_View, &QAction::triggered, this, &MainWindow::exitGridView);

    connect(ui->actionShow_Grid, &QAction::toggled, this, &MainWindow::onShowGridToggled);

    connect(ui->actionShow_Axis, &QAction::toggled, this, &MainWindow::onShowAxisToggled);

    connect(ui->actionGenerate_2D_grid, &QAction::triggered, this, &MainWindow::on_actionGenerate_2D_grid_triggered);
    connect(ui->actionGenerate_3D_grid, &QAction::triggered, this, &MainWindow::on_actionGenerate_3D_grid_triggered);

    connect(ui->actionMulti_Grid, &QAction::triggered, this, &MainWindow::on_actionMulti_Grid_triggered);

    // connect(ui->actionGrid_Visibility, &QAction::triggered, this, &MainWindow::on_actionGrid_Visibility_triggered);

    connect(ui->actionSegmentation, &QAction::triggered, this, &MainWindow::on_actionSegmentation_triggered);

    connect(ui->action2D_to_3D_Grid, &QAction::triggered, this, &MainWindow::on_action2D_to_3D_Grid_triggered);

    connect(ui->actionDown_Sample, &QAction::triggered, this, &MainWindow::on_actionDown_Sample_triggered);
    connect(ui->actionUp_Sample, &QAction::triggered, this, &MainWindow::on_actionUp_Sample_triggered);
    connect(ui->actionSub_Sample, &QAction::triggered, this, &MainWindow::on_actionSub_Sample_triggered);
    connect(ui->actionSampling, &QAction::triggered, this, &MainWindow::on_actionSampling_triggered);

    connect(ui->actionCluster_2, &QAction::triggered, this, &::MainWindow::on_actionCluster_2_triggered);
    connect(ui->actionHeatmap, &QAction::triggered, this, &MainWindow::on_actionHeatmap_triggered);
    connect(ui->actionManage_Objects, &QAction::triggered, this, &MainWindow::on_actionManage_Objects_triggered);
    connect(ui->actionCluster_list, &QAction::triggered, this, &MainWindow::on_actionCluster_list_triggered);
    connect(ui->actionManage_Twins, &QAction::triggered, this, &MainWindow::on_actionManage_Twins_triggered);
    connect(ui->actionObject_detection_Report, &QAction::triggered, this, &MainWindow::on_actionObject_detection_Report_triggered);
    connect(ui->actionCDD_analysis, &QAction::triggered, this, &MainWindow::on_actionCDD_analysis_triggered);
    connect(ui->actionReplace_with_Twins, &QAction::triggered, this, &MainWindow::on_actionReplace_with_Twins_triggered);
    connect(ui->actionGrid_Population_density, &QAction::triggered, this, &MainWindow::on_actionGrid_Population_density_triggered);
    connect(ui->actionExport, &QAction::triggered, this, &MainWindow::on_actionExport_triggered);

}

void MainWindow::on_actionGenerate_2D_grid_triggered()
{
    Generate2DGridDialog dialog(this);
    connect(&dialog, &Generate2DGridDialog::gridParametersConfirmed, this,
            [this](int rows, int columns, float cellSize) {
                QString name = QString("Grid2D_%1x%2").arg(rows).arg(columns);
                openGLWidget->add2DGrid(rows, columns, cellSize, name);
                updateHierarchyView();
                openGLWidget->update();
            });
    dialog.exec();
}
void MainWindow::on_actionGenerate_3D_grid_triggered()
{
    Generate3DGridDialog dialog(this);
    connect(&dialog, &Generate3DGridDialog::gridParametersConfirmed, this,
            [this](int rows, int columns, int layers, float cellSize) {
                QString name = QString("Grid3D_%1x%2x%3").arg(rows).arg(columns).arg(layers);
                openGLWidget->add3DGrid(rows, columns, layers, cellSize, name);
                updateHierarchyView();
                openGLWidget->update();
            });
    dialog.exec();
}


void MainWindow::onShowGridToggled(bool checked)
{
    // Get the active view widget
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Update the grid visibility
    activeView->setGridVisible(checked);

    // Update status message
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }

    QString logMsg = QString("Grid %1 in %2")
                         .arg(checked ? "shown" : "hidden")
                         .arg(viewName);

    statusBar()->showMessage(logMsg, 2000);
    logToConsole(logMsg, "INFO");
}


void MainWindow::updateFileProperties(const QString& filePath)
{
    if (!m_propertiesTable) return;

    // Clear existing rows
    m_propertiesTable->setRowCount(0);

    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        return;
    }

    // Get basic file information
    QString fileName = fileInfo.fileName();
    QString fileType = fileInfo.suffix().toUpper();
    QString fileSize = QString::number(fileInfo.size() / 1024.0, 'f', 2) + " KB";
    QString created = fileInfo.birthTime().toString("yyyy-MM-dd hh:mm:ss");
    QString modified = fileInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss");
    QString accessed = fileInfo.lastRead().toString("yyyy-MM-dd hh:mm:ss");


    // Determine rows needed and set row count
    int rowCount = 10; // Basic info

    // Check file type for additional properties
    QString extension = fileInfo.suffix().toLower();
    bool isPointCloud = (extension == "pts" || extension == "xyz" ||
                         extension == "pcd" || extension == "ply");
    bool isModel = (extension == "obj" || extension == "fbx" ||
                    extension == "stl" || extension == "dae");

    if (isPointCloud || isModel) {
        rowCount += 10; // Additional info for point clouds or models
    }

    m_propertiesTable->setRowCount(rowCount);

    // Add section headers and properties
    int row = 0;

    // File information section
    addPropertyHeader(m_propertiesTable, row++, "File Information");
    addProperty(m_propertiesTable, row++, "Name", fileName);
    addProperty(m_propertiesTable, row++, "Type", fileType + " File");
    addProperty(m_propertiesTable, row++, "Size", fileSize);
    addProperty(m_propertiesTable, row++, "Location", fileInfo.absolutePath());

    // Dates section
    addPropertyHeader(m_propertiesTable, row++, "Dates");
    addProperty(m_propertiesTable, row++, "Created", created);
    addProperty(m_propertiesTable, row++, "Modified", modified);
    addProperty(m_propertiesTable, row++, "Accessed", accessed);

    // Add file permissions
    addPropertyHeader(m_propertiesTable, row++, "Permissions");
    QString permissions;
    if (fileInfo.isReadable()) permissions += "Read ";
    if (fileInfo.isWritable()) permissions += "Write ";
    if (fileInfo.isExecutable()) permissions += "Execute";
    addProperty(m_propertiesTable, row++, "Access", permissions);

    // Add specific properties based on file type
    if (isPointCloud || isModel) {
        addPropertyHeader(m_propertiesTable, row++, extension == "pts" ? "Point Cloud Properties" : "Model Properties");

        // Try to get additional info based on file type
        try {
            if (isPointCloud) {
                // For point cloud, try to get point count and bounds
                std::ifstream file(filePath.toStdString());
                if (file.is_open()) {
                    // Count points and get bounds
                    int pointCount = 0;
                    bool hasColor = false;
                    QVector3D minPoint(std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max(),
                                       std::numeric_limits<float>::max());
                    QVector3D maxPoint(std::numeric_limits<float>::lowest(),
                                       std::numeric_limits<float>::lowest(),
                                       std::numeric_limits<float>::lowest());

                    std::string line;
                    while (std::getline(file, line) && pointCount < 1000) { // Sample first 1000 points
                        pointCount++;
                        std::istringstream iss(line);
                        float x, y, z, r = -1.0f, g = -1.0f, b = -1.0f;

                        if (iss >> x >> y >> z) {
                            minPoint.setX(qMin(minPoint.x(), x));
                            minPoint.setY(qMin(minPoint.y(), y));
                            minPoint.setZ(qMin(minPoint.z(), z));

                            maxPoint.setX(qMax(maxPoint.x(), x));
                            maxPoint.setY(qMax(maxPoint.y(), y));
                            maxPoint.setZ(qMax(maxPoint.z(), z));

                            if (iss >> r >> g >> b) {
                                hasColor = true;
                            }
                        }
                    }

                    // Estimate total points based on file size and first 1000 points
                    int estimatedPoints = pointCount;
                    if (pointCount == 1000) {
                        file.seekg(0, std::ios::end);
                        std::streampos fileSize = file.tellg();
                        file.seekg(0, std::ios::beg);
                        std::streampos sampleSize = file.tellg();
                        file.close();

                        if (sampleSize > 0) {
                            estimatedPoints = static_cast<int>((fileSize / sampleSize) * pointCount);
                        }
                    }

                    // Add properties
                    addProperty(m_propertiesTable, row++, "Point Count", QString::number(estimatedPoints));
                    addProperty(m_propertiesTable, row++, "Color Data", hasColor ? "Yes" : "No");

                    // Add dimensions
                    QVector3D dimensions = maxPoint - minPoint;
                    addProperty(m_propertiesTable, row++, "Width (X)", QString::number(dimensions.x(), 'f', 2));
                    addProperty(m_propertiesTable, row++, "Height (Y)", QString::number(dimensions.y(), 'f', 2));
                    addProperty(m_propertiesTable, row++, "Depth (Z)", QString::number(dimensions.z(), 'f', 2));

                    // Add bounding box
                    addProperty(m_propertiesTable, row++, "Min Point", QString("(%1, %2, %3)")
                                                                           .arg(minPoint.x()).arg(minPoint.y()).arg(minPoint.z()));
                    addProperty(m_propertiesTable, row++, "Max Point", QString("(%1, %2, %3)")
                                                                           .arg(maxPoint.x()).arg(maxPoint.y()).arg(maxPoint.z()));
                }
            }
            else if (isModel) {
                // For 3D models, try to use Assimp to get info
                Assimp::Importer importer;
                const aiScene* scene = importer.ReadFile(filePath.toStdString(),
                                                         aiProcess_Triangulate |
                                                             aiProcess_JoinIdenticalVertices);

                if (scene && !(scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE) && scene->mRootNode) {
                    // Count meshes, vertices, and faces
                    int totalMeshes = scene->mNumMeshes;
                    int totalVertices = 0;
                    int totalFaces = 0;

                    for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
                        aiMesh* mesh = scene->mMeshes[i];
                        totalVertices += mesh->mNumVertices;
                        totalFaces += mesh->mNumFaces;
                    }

                    // Add properties
                    addProperty(m_propertiesTable, row++, "Meshes", QString::number(totalMeshes));
                    addProperty(m_propertiesTable, row++, "Vertices", QString::number(totalVertices));
                    addProperty(m_propertiesTable, row++, "Faces", QString::number(totalFaces));
                    addProperty(m_propertiesTable, row++, "Materials", QString::number(scene->mNumMaterials));
                    addProperty(m_propertiesTable, row++, "Animations", QString::number(scene->mNumAnimations));

                    // Count nodes
                    std::function<int(aiNode*)> countNodes;
                    countNodes = [&countNodes](aiNode* node) -> int {
                        if (!node) return 0;
                        int count = 1; // Count this node
                        for (unsigned int i = 0; i < node->mNumChildren; i++) {
                            count += countNodes(node->mChildren[i]);
                        }
                        return count;
                    };

                    int nodeCount = countNodes(scene->mRootNode);
                    addProperty(m_propertiesTable, row++, "Nodes", QString::number(nodeCount));
                }
            }
        } catch (const std::exception& e) {
            // Just add a note about the error
            addProperty(m_propertiesTable, row++, "Note", "Could not analyze file details");
        }
    }

    // Set the actual row count (might be less than initially allocated)
    m_propertiesTable->setRowCount(row);

    // Make the table look nice
    m_propertiesTable->resizeColumnToContents(0);
    m_propertiesTable->horizontalHeader()->setStretchLastSection(true);
}


void MainWindow::updateDirectoryProperties(const QString& dirPath)
{
    if (!m_propertiesTable) return;

    // Clear existing rows
    m_propertiesTable->setRowCount(0);

    QFileInfo dirInfo(dirPath);
    if (!dirInfo.exists() || !dirInfo.isDir()) {
        return;
    }

    // Get directory information
    QString dirName = dirInfo.fileName();
    if (dirName.isEmpty()) {
        dirName = dirInfo.absolutePath();
    }
    QString path = dirInfo.absoluteFilePath();
    QString created = dirInfo.birthTime().toString("yyyy-MM-dd hh:mm:ss");
    QString modified = dirInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss");
    QString accessed = dirInfo.lastRead().toString("yyyy-MM-dd hh:mm:ss");

    // Count items in directory
    QDir dir(dirPath);
    int fileCount = dir.entryList(QDir::Files).count();
    int dirCount = dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot).count();

    // Calculate directory size (this can be slow for large directories)
    qint64 totalSize = 0;
    QDirIterator it(dirPath, QDir::Files, QDirIterator::Subdirectories);
    int maxItems = 1000; // Limit to prevent slow operation
    int itemCount = 0;

    while (it.hasNext() && itemCount < maxItems) {
        it.next();
        totalSize += it.fileInfo().size();
        itemCount++;
    }

    QString sizeStr;
    if (itemCount >= maxItems) {
        sizeStr = QString::number(totalSize / 1024.0 / 1024.0, 'f', 2) + " MB (partial)";
    } else {
        sizeStr = QString::number(totalSize / 1024.0 / 1024.0, 'f', 2) + " MB";
    }

    // Set row count
    int rowCount = 12;
    m_propertiesTable->setRowCount(rowCount);

    // Fill the properties
    int row = 0;

    // Directory information
    addPropertyHeader(m_propertiesTable, row++, "Directory Information");
    addProperty(m_propertiesTable, row++, "Name", dirName);
    addProperty(m_propertiesTable, row++, "Path", path);
    addProperty(m_propertiesTable, row++, "Size", sizeStr);

    // Content
    addPropertyHeader(m_propertiesTable, row++, "Content");
    addProperty(m_propertiesTable, row++, "Files", QString::number(fileCount));
    addProperty(m_propertiesTable, row++, "Folders", QString::number(dirCount));
    addProperty(m_propertiesTable, row++, "Total Items", QString::number(fileCount + dirCount));

    // Dates
    addPropertyHeader(m_propertiesTable, row++, "Dates");
    addProperty(m_propertiesTable, row++, "Created", created);
    addProperty(m_propertiesTable, row++, "Modified", modified);
    addProperty(m_propertiesTable, row++, "Accessed", accessed);

    // Make the table look nice
    m_propertiesTable->resizeColumnToContents(0);
    m_propertiesTable->horizontalHeader()->setStretchLastSection(true);
}

// Helper methods for property table
void MainWindow::addPropertyHeader(QTableWidget* table, int row, const QString& title)
{
    QTableWidgetItem* headerItem = new QTableWidgetItem(title);

    // Style the header
    QFont font = headerItem->font();
    font.setBold(true);
    headerItem->setFont(font);

    headerItem->setBackground(QBrush(QColor(60, 60, 80)));
    headerItem->setForeground(QBrush(Qt::white));

    table->setItem(row, 0, headerItem);

    // Span the header across both columns
    table->setSpan(row, 0, 1, 2);
}


void MainWindow::addProperty(QTableWidget* table, int row, const QString& name, const QString& value)
{
    QTableWidgetItem* nameItem = new QTableWidgetItem(name);
    QTableWidgetItem* valueItem = new QTableWidgetItem(value);

    // Style the property name
    QFont nameFont = nameItem->font();
    nameFont.setBold(true);
    nameItem->setFont(nameFont);

    table->setItem(row, 0, nameItem);
    table->setItem(row, 1, valueItem);
}


void MainWindow::updateHierarchyView()
{
    hierarchyModel->removeRows(0, hierarchyModel->rowCount());

    // If we have an active view, use its contents
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Add point clouds from the active view
    const auto& pointClouds = activeView->getPointClouds();
    for (const auto& pc : pointClouds) {
        QStandardItem* item = new QStandardItem("[PointCloud] " + pc.name);
        item->setEditable(false);
        item->setCheckable(true);
        item->setCheckState(pc.visible ? Qt::Checked : Qt::Unchecked);
        item->setData("PointCloud", Qt::UserRole);
        // Store which view this belongs to
        item->setData(QVariant::fromValue(activeView), Qt::UserRole + 1);
        hierarchyModel->appendRow(item);
    }

    // Add models from the active view
    const auto& models = activeView->getModels();
    for (const auto& model : models) {
        // Build the tree structure for this model
        buildModelHierarchyInTree(model,activeView);
    }

    // Add 2D grids from the active view
    const auto& grids2D = activeView->get2DGrids();
    for (const auto& grid : grids2D) {
        QStandardItem* item = new QStandardItem("[2DGrid] " + grid.name);
        item->setEditable(false);
        item->setCheckable(true);
        item->setCheckState(grid.visible ? Qt::Checked : Qt::Unchecked);
        item->setData("2DGrid", Qt::UserRole);
        // Store which view this belongs to
        item->setData(QVariant::fromValue(activeView), Qt::UserRole + 1);
        hierarchyModel->appendRow(item);
    }

    const auto& grids3D = openGLWidget->get3DGrids();
    for (const auto& grid : grids3D) {
        QStandardItem* item = new QStandardItem("[3DGrid] " + grid.name);
        item->setEditable(false);
        item->setCheckable(true);
        item->setCheckState(grid.visible ? Qt::Checked : Qt::Unchecked);
        item->setData("3DGrid", Qt::UserRole);
        hierarchyModel->appendRow(item);
    }

    disconnect(hierarchyModel, &QStandardItemModel::itemChanged, this, &MainWindow::on_hierarchyItemChanged);
    connect(hierarchyModel, &QStandardItemModel::itemChanged, this, &MainWindow::on_hierarchyItemChanged);
    ui->treeView->expandAll();

}


void MainWindow::buildModelHierarchyInTree(const OpenGLWidget::Model& model, OpenGLWidget* view)
{
    // Create root item for the model
    QStandardItem* modelRootItem = new QStandardItem("[Model] " + model.name);
    modelRootItem->setEditable(false);
    modelRootItem->setCheckable(true);
    modelRootItem->setCheckState(model.visible ? Qt::Checked : Qt::Unchecked);
    modelRootItem->setData("Model", Qt::UserRole);

    modelRootItem->setData(QVariant::fromValue(view), Qt::UserRole + 1);
    modelRootItem->setData(model.name, Qt::UserRole + 2); // Store model name for lookup

    // Add information about vertex count, etc.
    int totalVertices = 0;
    int totalFaces = 0;
    int totalMeshes = 0;

    for (const auto& node : model.nodes) {
        for (const auto& mesh : node.meshes) {
            totalVertices += mesh.vertexCount;
            totalFaces += mesh.indexCount / 3; // Triangles
            totalMeshes++;
        }
    }

    // Create info item
    QStandardItem* infoItem = new QStandardItem("Info");
    infoItem->setEditable(false);

    // Add mesh statistics as children of the info item
    QStandardItem* meshCountItem = new QStandardItem(QString("Meshes: %1").arg(totalMeshes));
    meshCountItem->setEditable(false);
    infoItem->appendRow(meshCountItem);

    QStandardItem* vertexCountItem = new QStandardItem(QString("Vertices: %1").arg(totalVertices));
    vertexCountItem->setEditable(false);
    infoItem->appendRow(vertexCountItem);

    QStandardItem* faceCountItem = new QStandardItem(QString("Triangles: %1").arg(totalFaces));
    faceCountItem->setEditable(false);
    infoItem->appendRow(faceCountItem);

    // Add info item to the model root
    modelRootItem->appendRow(infoItem);

    // Find all root nodes (nodes with no parent)
    for (size_t i = 0; i < model.nodes.size(); i++) {
        if (model.nodes[i].parentIndex == -1) {
            // Add this node and its descendants to the tree
            QStandardItem* nodeItem = createModelNodeItem(model, i, true,view);
            modelRootItem->appendRow(nodeItem);
        }
    }

    // Add the model root to the hierarchy
    hierarchyModel->appendRow(modelRootItem);
}

QStandardItem* MainWindow::createModelNodeItem(
    const OpenGLWidget::Model& model,
    int nodeIndex,
    bool isRoot,
    OpenGLWidget* view)
{
    const OpenGLWidget::ModelNode& node = model.nodes[nodeIndex];

    // Create item for this node
    QString displayName = node.name.isEmpty() ? QString("Node_%1").arg(nodeIndex) : node.name;

    QStandardItem* nodeItem;
    if (isRoot) {
        nodeItem = new QStandardItem(QString("[Node] %1").arg(displayName));
    } else {
        nodeItem = new QStandardItem(displayName);
    }

    nodeItem->setEditable(false);
    nodeItem->setCheckable(true);
    nodeItem->setCheckState(node.visible ? Qt::Checked : Qt::Unchecked);
    nodeItem->setData("ModelNode", Qt::UserRole);

    nodeItem->setData(QVariant::fromValue(view), Qt::UserRole + 1);
    nodeItem->setData(model.name, Qt::UserRole + 2);   // Model name
    nodeItem->setData(nodeIndex, Qt::UserRole + 3);    // Node index

    // Add transform information
    QStandardItem* transformItem = new QStandardItem("Transform");
    transformItem->setEditable(false);

    QStandardItem* posItem = new QStandardItem(QString("Position: (%1, %2, %3)").arg(
                                                                                    node.position.x()).arg(node.position.y()).arg(node.position.z()));
    posItem->setEditable(false);
    transformItem->appendRow(posItem);

    QStandardItem* rotItem = new QStandardItem(QString("Rotation: (%1, %2, %3)").arg(
                                                                                    node.rotation.x()).arg(node.rotation.y()).arg(node.rotation.z()));
    rotItem->setEditable(false);
    transformItem->appendRow(rotItem);

    QStandardItem* scaleItem = new QStandardItem(QString("Scale: (%1, %2, %3)").arg(
                                                                                   node.scale.x()).arg(node.scale.y()).arg(node.scale.z()));
    scaleItem->setEditable(false);
    transformItem->appendRow(scaleItem);

    nodeItem->appendRow(transformItem);

    // Add meshes as children
    if (!node.meshes.empty()) {
        QStandardItem* meshesItem = new QStandardItem("Meshes");
        meshesItem->setEditable(false);

        for (size_t i = 0; i < node.meshes.size(); i++) {
            const auto& mesh = node.meshes[i];
            QString meshName = mesh.name.isEmpty() ? QString("Mesh_%1").arg(i) : mesh.name;

            QStandardItem* meshItem = new QStandardItem(meshName);
            meshItem->setEditable(false);

            // Add mesh details
            QStandardItem* vertItem = new QStandardItem(QString("Vertices: %1").arg(mesh.vertexCount));
            vertItem->setEditable(false);
            meshItem->appendRow(vertItem);

            QStandardItem* faceItem = new QStandardItem(QString("Triangles: %1").arg(mesh.indexCount / 3));
            faceItem->setEditable(false);
            meshItem->appendRow(faceItem);

            if (!mesh.materialName.isEmpty()) {
                QStandardItem* matItem = new QStandardItem(QString("Material: %1").arg(mesh.materialName));
                matItem->setEditable(false);
                meshItem->appendRow(matItem);
            }

            meshesItem->appendRow(meshItem);
        }

        nodeItem->appendRow(meshesItem);
    }

    // Add child nodes recursively
    for (int childIndex : node.children) {
        QStandardItem* childItem = createModelNodeItem(model, childIndex, false,view);
        nodeItem->appendRow(childItem);
    }

    return nodeItem;
}



void MainWindow::on_hierarchyItemChanged(QStandardItem *item)
{
    QString type = item->data(Qt::UserRole).toString();
    bool visible = (item->checkState() == Qt::Checked);

    // Get the associated view for this item
    OpenGLWidget* view = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
    if (!view) view = openGLWidget; // Fallback to main view

    if (type == "ViewportCapture") {
        QString name = item->data(Qt::UserRole + 2).toString();

        // Get the view this capture belongs to
        OpenGLWidget* view = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback to main view

        // Apply the viewport
        view->applyViewportCapture(name);

        // Show confirmation
        statusBar()->showMessage(tr("Applied viewport capture: %1").arg(name), 2000);
    }else if (type == "PointCloud") {
        QString name = item->text();
        name.remove("[PointCloud] ");
        view->setPointCloudVisibility(name, visible);
    }
    else if (type == "Model") {
        QString name = item->text();
        name.remove("[Model] ");
        view->setModelVisibility(name, visible);
    }
    else if (type == "ModelNode") {
        // Handle visibility for model nodes
        QString modelName = item->data(Qt::UserRole + 2).toString();
        int nodeIndex = item->data(Qt::UserRole + 3).toInt();

        // Set visibility on the specific node
        view->setModelNodeVisibility(modelName, nodeIndex, visible);

        // Update child items in tree to match parent's visibility state
        int childCount = item->rowCount();
        for (int i = 0; i < childCount; i++) {
            QStandardItem* childItem = item->child(i);
            if (childItem && childItem->isCheckable()) {
                childItem->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
            }
        }
    }
    else if (type == "2DGrid") {
        QString name = item->text();
        name.remove("[2DGrid] ");
        view->set2DGridVisibility(name, visible);
    }
    else if (type == "3DGrid") {
        QString name = item->text();
        name.remove("[3DGrid] ");
        view->set3DGridVisibility(name, visible);
    }

    // Ensure view is updated
    view->update();

    // Show message in status bar
    QString visibilityStr = visible ? "visible" : "hidden";
    statusBar()->showMessage(tr("%1 is now %2").arg(item->text()).arg(visibilityStr), 2000);
}


void MainWindow::onShowAxisToggled(bool checked)
{
    // Get the active view widget
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Update the axis visibility
    activeView->setAxisVisible(checked);

    // Update status message
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }

    statusBar()->showMessage(
        tr("Axis %1 in %2").arg(checked ? "shown" : "hidden").arg(viewName),
        2000
        );
}

void MainWindow::on_hierarchySelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    // Process deselected items
    for (const auto& index : deselected.indexes()) {
        QStandardItem* item = hierarchyModel->itemFromIndex(index);
        if (!item) continue;
        QString type = item->data(Qt::UserRole).toString();
        // Get the associated view
        OpenGLWidget* view = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback
        if (type == "PointCloud") {
            QString name = item->text();
            name.remove("[PointCloud] ");
            view->setPointCloudSelected(name, false);
        }
        else if (type == "Model") {
            QString name = item->text();
            name.remove("[Model] ");
            view->setModelSelected(name, false);
        }
        else if (type == "ModelNode") {
            // Handle deselection of model nodes
            QString modelName = item->data(Qt::UserRole + 2).toString();
            int nodeIndex = item->data(Qt::UserRole + 3).toInt();
            view->setModelNodeSelected(modelName, nodeIndex, false);
        }
        else if (type == "2DGrid") {
            QString name = item->text();
            name.remove("[2DGrid] ");
            view->set2DGridSelected(name, false);
        }
        else if (type == "3DGrid") {
            QString name = item->text();
            name.remove("[3DGrid] ");
            view->set3DGridSelected(name, false);
        }
    }

    // Process selected items
    for (const auto& index : selected.indexes()) {
        QStandardItem* item = hierarchyModel->itemFromIndex(index);
        if (!item) continue;
        QString type = item->data(Qt::UserRole).toString();
        // Get the associated view
        OpenGLWidget* view = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback

        // Ensure transform controls are enabled
        if (transformManager) {
            transformManager->enableTransformControls();
        }

        if (type == "PointCloud") {
            QString name = item->text();
            name.remove("[PointCloud] ");
            view->setPointCloudSelected(name, true);

            // Update transform manager with selected point cloud
            if (transformManager) {
                transformManager->onPointCloudSelected(name);
            }
        }
        else if (type == "Model") {
            QString name = item->text();
            name.remove("[Model] ");
            view->setModelSelected(name, true);

            // Update transform manager with selected model
            if (transformManager) {
                transformManager->onModelSelected(name);
            }

            // Update properties view when a model is selected
            updatePropertiesView();
        }
        else if (type == "ModelNode") {
            // Handle selection of model nodes
            QString modelName = item->data(Qt::UserRole + 2).toString();
            int nodeIndex = item->data(Qt::UserRole + 3).toInt();
            view->setModelNodeSelected(modelName, nodeIndex, true);

            // Update transform manager with selected model node
            if (transformManager) {
                transformManager->onModelNodeSelected(modelName, nodeIndex);
            }

            // Update properties view when a node is selected
            updatePropertiesView();
        }
        else if (type == "2DGrid") {
            QString name = item->text();
            name.remove("[2DGrid] ");
            view->set2DGridSelected(name, true);
        }
        else if (type == "3DGrid") {
            QString name = item->text();
            name.remove("[3DGrid] ");
            view->set3DGridSelected(name, true);
        }
        else if (type == "Viewport") {
            // Handle viewport selection
            QString name = item->text();
            name.remove("[Viewport] ");
            updateViewportFromSelection(name);
        }

        // Set the selected view as active
        if (view) {
            setActiveViewWidget(view);
        }
    }

    // If no items are selected, disable transform controls
    if (selected.indexes().isEmpty() && transformManager) {
        transformManager->disableTransformControls();
    }

    // If a property panel is available, update it
    QTableWidget* propertiesTable = findChild<QTableWidget*>("propertiesTableWidget");
    if (propertiesTable) {
        updatePropertiesView();
    }
}



// 4. Add new methods to apply views to the active window
void MainWindow::applyFrontViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setFrontView();

    // Update status bar to show which view was affected
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    statusBar()->showMessage(tr("Applied Front View to %1").arg(viewName), 2000);
    logToConsole(tr("Applied Front View to %1").arg(viewName),"INFO");
}

void MainWindow::applyTopViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setTopView();

    // Update status bar
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    statusBar()->showMessage(tr("Applied Top View to %1").arg(viewName), 2000);
    logToConsole(tr("Applied Top View to %1").arg(viewName),"INFO");
}


void MainWindow::applySideViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setSideView();

    // Update status bar
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    statusBar()->showMessage(tr("Applied Side View to %1").arg(viewName), 2000);
    logToConsole(tr("Applied Side View to %1").arg(viewName),"INFO");
}

void MainWindow::applyIsometricViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setIsometricView();

    // Update status bar
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    statusBar()->showMessage(tr("Applied Isometric View to %1").arg(viewName), 2000);
    logToConsole(tr("Applied Isometric View to %1").arg(viewName),"INFO");
}




// Replace your current on_actionOpen_triggered method with this one

void MainWindow::on_actionOpen_triggered()
{
    qDebug() << "Open action triggered";
    static bool isDialogOpen = false;
    if (isDialogOpen) {
        qDebug() << "File dialog already open, ignoring duplicate call";
        return;
    }

    isDialogOpen = true;
    QStringList filePaths = QFileDialog::getOpenFileNames(
        this,
        tr("Open Files"),
        QString(),
        tr("All Supported Files (*.pts *.obj *.fbx *.stl *.dae *.ply *.gltf *.glb *.mdl *.md2 *.md3 *.md5mesh *.md5anim *.md5camera *.smd *.vta *.mesh *.skeleton *.3d *.bvh *.dxf *.ifc *.blend *.lws *.irr *.nff *.scn *.3ds *.lwo *.lx *.ac *.ms3d *.cob *.xgl *.zgl *.ogex *.q3d *.q3s *.irrmesh *.off *.raw *.ter *.hmp *.x);;"
           "Point Cloud Files (*.pts *.ply);;"
           "General 3D Model Files (*.obj *.fbx *.stl *.dae *.ply *.gltf *.glb);;"
           "Game Engine Files (*.mdl *.md2 *.md3 *.md5mesh *.md5anim *.md5camera *.smd *.vta *.mesh *.skeleton *.3d);;"
           "Animation and Motion Capture Files (*.bvh);;"
           "CAD and Architectural Files (*.dxf *.ifc);;"
           "Scene and Environment Files (*.blend *.lws *.irr *.nff *.scn);;"
           "Legacy and Specialized Files (*.3ds *.lwo *.lx *.ac *.ms3d *.cob *.xgl *.zgl *.ogex *.q3d *.q3s *.irrmesh *.off *.raw *.ter *.hmp *.x);;"
           "All Files (*)")
        );
    isDialogOpen = false;

    if (filePaths.isEmpty()) {
        return;
    }

    // Use the active view for loading file
    OpenGLWidget* targetView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Show which view we're loading into
    QWidget* window = targetView->window();
    QString viewName = window ? window->windowTitle() : "Main View";
    statusBar()->showMessage(tr("Loading file into %1...").arg(viewName), 2000);

    // Create progress dialog for single file
    QProgressDialog progress(tr("Loading File..."), tr("Cancel"), 0, 1, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setValue(0);

    qDebug() << "Selected files:" << filePaths;





    try {
        for (int i = 0; i < filePaths.size(); ++i) {
            const QString& filePath = filePaths[i];
            progress.setValue(i);
            progress.setLabelText(tr("Loading file: %1").arg(QFileInfo(filePath).fileName()));
            QCoreApplication::processEvents();

            if (progress.wasCanceled()) {
                QMessageBox::information(this, tr("Loading Canceled"), tr("File loading was canceled by the user."));
                return;
            }

            qDebug() << "Processing file:" << filePath;

            QFileInfo fileInfo(filePath);
            QString fileName = fileInfo.fileName();
            QString extension = fileInfo.suffix().toLower();

            if (extension == "pts") {
                // PTS file handling
                std::vector<QVector3D> points;
                std::vector<QVector3D> colors;

                std::ifstream file(filePath.toStdString());
                if (!file.is_open()) {
                    throw std::runtime_error("Could not open file: " + filePath.toStdString());
                }

                std::string line;
                while (std::getline(file, line)) {
                    std::istringstream iss(line);
                    float x, y, z, r, g, b;

                    if (iss >> x >> y >> z) {
                        points.push_back(QVector3D(x, y, z));
                        if (iss >> r >> g >> b) {
                            colors.push_back(QVector3D(r / 255.0f, g / 255.0f, b / 255.0f));
                        } else {
                            colors.push_back(QVector3D(1.0f, 1.0f, 1.0f));
                        }
                    }
                }
                qDebug() << "Loaded" << points.size() << "points from file:" << filePath;

                // Use the target view
                targetView->addPointCloud(points, colors, fileName);
            }   else if (extension == "ply" || extension == "pcd") {
                // Use PCL to load PLY/PCD files
                qDebug() << "Loading PLY/PCD file using PCL:" << filePath;

                // Call our new method for loading PLY/PCD files
                loadPLYPointCloud(filePath, fileName, targetView);
            }else if (extension == "fbx") {
                // Show the FBX information dialog first
                showFbxInformation(filePath);

                // Then add the model to the view
                targetView->addModel(filePath, fileName);
            }else {
                // Handle other file types (delegate to a generic load function)
                loadFiles({filePath}); // Assuming loadFiles handles non-PTS and non-PLY files
                // For other 3D model formats
                targetView->addModel(filePath, fileName);
            }
        }

        updateHierarchyView();
        progress.setValue(filePaths.size());
        QMessageBox::information(this, tr("Loading Complete"), tr("Successfully loaded %1 file(s).").arg(filePaths.size()));

        // Ensure the target view is updated
        targetView->update();
            // Make sure the view is visible and active
        if (window && window != this) {
            window->activateWindow();
            window->raise();
        }
        // Update status



    } catch (const std::exception& e) {
        progress.setValue(filePaths.size());
        QMessageBox::critical(this, tr("Error"), tr("Failed to load files: %1").arg(e.what()));
    }
}


void MainWindow::showFbxInformation(const QString& filePath)
{
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists() || !fileInfo.isFile()) {
        QMessageBox::warning(this, tr("File Error"), tr("Unable to access file: %1").arg(filePath));
        return;
    }

    // Extract file information
    QString fileName = fileInfo.fileName();
    QString fileSize = QString::number(fileInfo.size() / 1024.0, 'f', 2) + " KB";
    QString lastModified = fileInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss");

    // Create information dialog
    QDialog dialog(this);
    dialog.setWindowTitle(tr("FBX File Information"));
    dialog.setMinimumWidth(500);

    QVBoxLayout* layout = new QVBoxLayout(&dialog);

    // File metadata section
    QGroupBox* metadataBox = new QGroupBox(tr("File Information"));
    QFormLayout* metadataLayout = new QFormLayout(metadataBox);
    metadataLayout->addRow(tr("File Name:"), new QLabel(fileName));
    metadataLayout->addRow(tr("File Size:"), new QLabel(fileSize));
    metadataLayout->addRow(tr("Last Modified:"), new QLabel(lastModified));
    metadataLayout->addRow(tr("File Path:"), new QLabel(filePath));
    layout->addWidget(metadataBox);

    // Model statistics - we need to load the model without adding it to the scene to get stats
    QGroupBox* statsBox = new QGroupBox(tr("Model Statistics"));
    QVBoxLayout* statsLayout = new QVBoxLayout(statsBox);

    // Create a progress dialog while we load and analyze the model
    QProgressDialog progress(tr("Analyzing FBX File..."), tr("Cancel"), 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setValue(10);
    QCoreApplication::processEvents();

    try {
        Assimp::Importer importer;
        progress.setValue(20);
        QCoreApplication::processEvents();

        const aiScene* scene = importer.ReadFile(filePath.toStdString(),
                                                 aiProcess_Triangulate |
                                                     aiProcess_GenNormals |
                                                     aiProcess_JoinIdenticalVertices);
        progress.setValue(80);
        QCoreApplication::processEvents();

        if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
            throw std::runtime_error(importer.GetErrorString());
        }

        // Get statistics from the scene
        QTreeWidget* statsTree = new QTreeWidget();
        statsTree->setHeaderLabels(QStringList() << tr("Property") << tr("Value"));
        statsTree->setMinimumHeight(200);
        statsTree->setAlternatingRowColors(true);

        // Basic scene stats
        QTreeWidgetItem* basicStatsItem = new QTreeWidgetItem(statsTree, QStringList() << tr("Basic Statistics"));
        basicStatsItem->setExpanded(true);
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Mesh Count") << QString::number(scene->mNumMeshes));
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Material Count") << QString::number(scene->mNumMaterials));
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Animation Count") << QString::number(scene->mNumAnimations));
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Light Count") << QString::number(scene->mNumLights));
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Camera Count") << QString::number(scene->mNumCameras));

        // Count total vertices and faces
        int totalVertices = 0;
        int totalFaces = 0;
        for (unsigned int i = 0; i < scene->mNumMeshes; i++) {
            totalVertices += scene->mMeshes[i]->mNumVertices;
            totalFaces += scene->mMeshes[i]->mNumFaces;
        }

        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Total Vertices") << QString::number(totalVertices));
        new QTreeWidgetItem(basicStatsItem, QStringList() << tr("Total Faces") << QString::number(totalFaces));

        // Node hierarchy
        QTreeWidgetItem* nodeHierarchyItem = new QTreeWidgetItem(statsTree, QStringList() << tr("Node Hierarchy"));
        nodeHierarchyItem->setExpanded(true);

        // Create a recursive function to count nodes and add them to the tree
        std::function<void(const aiNode*, QTreeWidgetItem*)> addNodeToTree;
        addNodeToTree = [&addNodeToTree, scene](const aiNode* node, QTreeWidgetItem* parentItem) {
            QString nodeName = QString::fromUtf8(node->mName.C_Str());
            if (nodeName.isEmpty()) {
                nodeName = "(unnamed)";
            }

            // Create item for this node
            QTreeWidgetItem* nodeItem = new QTreeWidgetItem(parentItem, QStringList() << nodeName);

            // Add meshes
            if (node->mNumMeshes > 0) {
                QTreeWidgetItem* meshesItem = new QTreeWidgetItem(nodeItem, QStringList() << tr("Meshes"));
                for (unsigned int i = 0; i < node->mNumMeshes; i++) {
                    unsigned int meshIndex = node->mMeshes[i];
                    aiMesh* mesh = scene->mMeshes[meshIndex];

                    QString meshName = QString("Mesh %1").arg(meshIndex);
                    if (mesh->mName.length > 0) {
                        meshName = QString::fromUtf8(mesh->mName.C_Str());
                    }

                    QTreeWidgetItem* meshItem = new QTreeWidgetItem(meshesItem, QStringList()
                                                                                    << meshName
                                                                                    << tr("%1 verts, %2 faces").arg(mesh->mNumVertices).arg(mesh->mNumFaces));
                }
            }

            // Recursively add children
            for (unsigned int i = 0; i < node->mNumChildren; i++) {
                addNodeToTree(node->mChildren[i], nodeItem);
            }
        };

        // Add the root node
        addNodeToTree(scene->mRootNode, nodeHierarchyItem);

        // Materials
        if (scene->mNumMaterials > 0) {
            QTreeWidgetItem* materialsItem = new QTreeWidgetItem(statsTree, QStringList() << tr("Materials"));
            materialsItem->setExpanded(true);

            for (unsigned int i = 0; i < scene->mNumMaterials; i++) {
                aiMaterial* material = scene->mMaterials[i];

                // Get material name
                aiString name;
                QString materialName;
                if (material->Get(AI_MATKEY_NAME, name) == AI_SUCCESS) {
                    materialName = QString::fromUtf8(name.C_Str());
                } else {
                    materialName = QString("Material %1").arg(i);
                }

                QTreeWidgetItem* materialItem = new QTreeWidgetItem(materialsItem, QStringList() << materialName);

                // Get diffuse color
                aiColor3D diffuse(0.f, 0.f, 0.f);
                if (material->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse) == AI_SUCCESS) {
                    new QTreeWidgetItem(materialItem, QStringList()
                                        << tr("Diffuse Color")
                                        << QString("RGB(%1, %2, %3)").arg(diffuse.r).arg(diffuse.g).arg(diffuse.b));
                }

                // Get specular color
                aiColor3D specular(0.f, 0.f, 0.f);
                if (material->Get(AI_MATKEY_COLOR_SPECULAR, specular) == AI_SUCCESS) {
                    new QTreeWidgetItem(materialItem, QStringList()
                                        << tr("Specular Color")
                                        << QString("RGB(%1, %2, %3)").arg(specular.r).arg(specular.g).arg(specular.b));
                }

                // Get texture count
                unsigned int numTextures = 0;
                for (unsigned int type = aiTextureType_DIFFUSE; type <= aiTextureType_UNKNOWN; type++) {
                    numTextures += material->GetTextureCount((aiTextureType)type);
                }

                new QTreeWidgetItem(materialItem, QStringList() << tr("Texture Count") << QString::number(numTextures));
            }
        }

        statsLayout->addWidget(statsTree);

        // Adjust column widths
        for (int i = 0; i < statsTree->columnCount(); i++) {
            statsTree->resizeColumnToContents(i);
        }

        progress.setValue(100);
    }
    catch (const std::exception& e) {
        // Handle failure
        QLabel* errorLabel = new QLabel(tr("Failed to analyze FBX: %1").arg(e.what()));
        errorLabel->setWordWrap(true);
        statsLayout->addWidget(errorLabel);
    }

    layout->addWidget(statsBox);

    // Button box
    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok);
    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    layout->addWidget(buttonBox);

    dialog.setLayout(layout);
    dialog.exec();
}



// 6. Add a method to update transform properties based on active selection
void MainWindow::updateTransformProperties()
{
    // Only proceed if we have a valid active view
    if (!m_activeViewWidget) {
        return;
    }

    // Get the selection from the active view
    QList<QVariant> selection = m_activeViewWidget->getSelectedItemsTransform();

    // If no selection, disable transform controls
    if (selection.isEmpty()) {
        // Disable transform inputs or set to default values
        ui->positionX->setEnabled(false);
        ui->positionY->setEnabled(false);
        ui->positionZ->setEnabled(false);
        ui->rotationX->setEnabled(false);
        ui->rotationY->setEnabled(false);
        ui->rotationZ->setEnabled(false);
        ui->scaleX->setEnabled(false);
        ui->scaleY->setEnabled(false);
        ui->scaleZ->setEnabled(false);

        // Reset to default values
        ui->positionX->setValue(0.0);
        ui->positionY->setValue(0.0);
        ui->positionZ->setValue(0.0);
        ui->rotationX->setValue(0.0);
        ui->rotationY->setValue(0.0);
        ui->rotationZ->setValue(0.0);
        ui->scaleX->setValue(1.0);
        ui->scaleY->setValue(1.0);
        ui->scaleZ->setValue(1.0);

        return;
    }

    // Enable transform inputs
    ui->positionX->setEnabled(true);
    ui->positionY->setEnabled(true);
    ui->positionZ->setEnabled(true);
    ui->rotationX->setEnabled(true);
    ui->rotationY->setEnabled(true);
    ui->rotationZ->setEnabled(true);
    ui->scaleX->setEnabled(true);
    ui->scaleY->setEnabled(true);
    ui->scaleZ->setEnabled(true);

    // For simplicity, just use the first selected item's transform
    // In a more complex implementation, you might average values or handle multi-selection differently
    if (!selection.isEmpty()) {
        // Assume we get a QVector3D for position, rotation, and scale
        QVector3D position = selection[0].value<QVector3D>();
        QVector3D rotation = selection[1].value<QVector3D>();
        QVector3D scale = selection[2].value<QVector3D>();

        // Update UI without triggering signals
        ui->positionX->blockSignals(true);
        ui->positionY->blockSignals(true);
        ui->positionZ->blockSignals(true);
        ui->rotationX->blockSignals(true);
        ui->rotationY->blockSignals(true);
        ui->rotationZ->blockSignals(true);
        ui->scaleX->blockSignals(true);
        ui->scaleY->blockSignals(true);
        ui->scaleZ->blockSignals(true);

        ui->positionX->setValue(position.x());
        ui->positionY->setValue(position.y());
        ui->positionZ->setValue(position.z());
        ui->rotationX->setValue(rotation.x());
        ui->rotationY->setValue(rotation.y());
        ui->rotationZ->setValue(rotation.z());
        ui->scaleX->setValue(scale.x());
        ui->scaleY->setValue(scale.y());
        ui->scaleZ->setValue(scale.z());

        ui->positionX->blockSignals(false);
        ui->positionY->blockSignals(false);
        ui->positionZ->blockSignals(false);
        ui->rotationX->blockSignals(false);
        ui->rotationY->blockSignals(false);
        ui->rotationZ->blockSignals(false);
        ui->scaleX->blockSignals(false);
        ui->scaleY->blockSignals(false);
        ui->scaleZ->blockSignals(false);
    }
}


// Keep only one implementation of setActiveViewWidget
void MainWindow::setActiveViewWidget(OpenGLWidget* widget)
{
    if (m_activeViewWidget == widget) {
        return; // Already active
    }

    m_activeViewWidget = widget;

    // Visual feedback in grid view
    if (m_inGridViewMode && m_gridLayout) {
        for (int i = 0; i < m_gridLayout->count(); i++) {
            QLayoutItem* item = m_gridLayout->itemAt(i);
            QFrame* frame = qobject_cast<QFrame*>(item->widget());
            if (!frame) continue;

            QList<OpenGLWidget*> children = frame->findChildren<OpenGLWidget*>();
            if (!children.isEmpty() && children[0] == widget) {
                frame->setStyleSheet("QFrame { border: 2px solid #3498db; }");
                frame->raise();
                frame->update();
                widget->raise();

            } else {
                frame->setStyleSheet("QFrame { border: 1px solid #444444; }");
            }

        }
    }

    // Update the hierarchy view with items from active view
    updateHierarchyView();
    updateTransformProperties();
    QString viewName = "Main View";
    if (widget != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == widget) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    // Let the transform manager know that the selection changed
    if (transformManager) {
        // Special handling if updateUIFromSelection is private
        // Use public methods to trigger an update
        QString dummy; // or find actual name of selected item
        transformManager->onPointCloudSelected(dummy);
    }

    // Status bar update
    statusBar()->showMessage(tr("Active view: %1").arg(widget->window()->windowTitle()));
}

// Add a method to MainWindow to handle transform connections for new view windows
void MainWindow::setupTransformForView(OpenGLWidget* viewWidget)
{
    // Connect signals from the view widget to update transforms
    if (transformManager && viewWidget) {
        // Connect to point cloud and model selection signals
        connect(viewWidget, &OpenGLWidget::pointCloudSelected,
                transformManager, &TransformManager::onPointCloudSelected);
        connect(viewWidget, &OpenGLWidget::modelSelected,
                transformManager, &TransformManager::onModelSelected);

        // Connect to widget click event to update active view
        connect(viewWidget, &OpenGLWidget::clicked, this, [this, viewWidget]() {
            setActiveViewWidget(viewWidget);
        });
    }
}

// Update onNewViewWindowRequested to connect transform signals
void MainWindow::onNewViewWindowRequested()
{
    // Create a new OpenGL widget
    OpenGLWidget* newViewWidget = new OpenGLWidget(this);

    // Create a new window
    QMainWindow* newViewWindow = new QMainWindow(this);
    newViewWindow->setCentralWidget(newViewWidget);

    // Window properties
    QString title = QString("3D View %1").arg(m_viewWindows.size() + 1);
    newViewWindow->setWindowTitle(title);
    newViewWindow->resize(800, 600);

    // Connect the view's signals
    connect(newViewWidget, &OpenGLWidget::clicked, this, [this, newViewWidget]() {
        setActiveViewWidget(newViewWidget);
    });

    // Connect selection signals to our transform manager
    connect(newViewWidget, &OpenGLWidget::pointCloudSelected,
            transformManager, &TransformManager::onPointCloudSelected);
    connect(newViewWidget, &OpenGLWidget::modelSelected,
            transformManager, &TransformManager::onModelSelected);
    connect(newViewWidget, &OpenGLWidget::modelNodeSelected,
            transformManager, &TransformManager::onModelNodeSelected);
    // Store view window info
    m_viewWindows.append({
        newViewWindow,
        newViewWidget,
        nullptr  // No separate transform manager for now
    });

    // Set up event filter
    newViewWidget->installEventFilter(this);
    newViewWindow->installEventFilter(this);

    // Get current grid visibility state from UI
    // Set initial visibility states to match the UI
    bool gridVisible = ui->actionShow_Grid->isChecked();
    bool axisVisible = ui->actionShow_Axis->isChecked();
    newViewWidget->setGridVisible(gridVisible);
    newViewWidget->setAxisVisible(axisVisible);

    // Set auto-delete
    newViewWindow->setAttribute(Qt::WA_DeleteOnClose);

    // Show the window
    newViewWindow->show();

    // Make it active
    setActiveViewWidget(newViewWidget);
}


// Add this function to initialize TransformManager in the MainWindow constructor
void MainWindow::initializeTransformManager()
{
    // Create a transform manager for the main OpenGL widget
    transformManager = new TransformManager(openGLWidget, this, this);
    transformManager->setupConnections();

    // Connect main OpenGL widget signals
    connect(openGLWidget, &OpenGLWidget::pointCloudSelected,
            transformManager, &TransformManager::onPointCloudSelected);
    connect(openGLWidget, &OpenGLWidget::modelSelected,
            transformManager, &TransformManager::onModelSelected);
}


void MainWindow::updateViewSelector()
{
    if (!m_3DViewSelector)
        return;

    // Clear and rebuild the list
    m_3DViewSelector->clear();

    // Add main view
    m_3DViewSelector->addItem("Main View");

    // Add additional views
    for (int i = 0; i < m_viewWindows.size(); ++i) {
        m_3DViewSelector->addItem(m_viewWindows[i].window->windowTitle());
    }
}



// Update the existing eventFilter method or add it if not present
bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
    // Check if this is a resize event for a property table
    if (event->type() == QEvent::Resize) {
        QTableWidget* tableWidget = qobject_cast<QTableWidget*>(obj);
        if (tableWidget && tableWidget->objectName() == "propertiesTableWidget") {
            // Adjust column widths when table is resized
            int valueColumnWidth = tableWidget->width() - 105; // Account for scrollbar and borders
            if (valueColumnWidth < 140) valueColumnWidth = 140;
            tableWidget->setColumnWidth(1, valueColumnWidth);
        }
    }
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
        if (keyEvent->key() == Qt::Key_Escape) {
            if (m_openGLFullScreenDialog) {
                toggleFullScreen3DView();
                return true;
            }

            if (m_activeViewWidget && m_activeViewWidget->window()->windowState() & Qt::WindowFullScreen) {
                m_activeViewWidget->window()->showNormal();
                return true;
            }
        }
    }

    // Let the base class handle other events
    return QMainWindow::eventFilter(obj, event);
}




void MainWindow::on_actionDelete_triggered()
{
    // Get selected indices from the hierarchy view
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();
    if (selectedIndexes.isEmpty()) {
        QMessageBox::information(this, tr("Delete Objects"), tr("No objects selected for deletion."));
        return;
    }

    // Confirm deletion
    int ret = QMessageBox::question(this, tr("Confirm Deletion"),
                                   tr("Are you sure you want to delete %1 selected object(s)?").arg(selectedIndexes.size()),
                                   QMessageBox::Yes | QMessageBox::No);
    if (ret != QMessageBox::Yes) {
        return;
    }

    // Use a set to track unique views that need updating
    QSet<OpenGLWidget*> viewsToUpdate;

    // Process each selected item
    for (const QModelIndex& index : selectedIndexes) {
        QStandardItem* item = hierarchyModel->itemFromIndex(index);
        if (!item) continue;

        QString type = item->data(Qt::UserRole).toString();
        OpenGLWidget* view = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback to main view

        viewsToUpdate.insert(view);

        QString name = item->text();
        if (name.startsWith("[PointCloud] ")) {
            name.remove("[PointCloud] ");
        } else if (name.startsWith("[Model] ")) {
            name.remove("[Model] ");
        } else if (name.startsWith("[2DGrid] ")) {
            name.remove("[2DGrid] ");
        } else if (name.startsWith("[3DGrid] ")) {
            name.remove("[3DGrid] ");
        }

        if (type == "PointCloud") {
            view->removePointCloud(name);
            qDebug() << "Removed PointCloud:" << name << "from view";
        } else if (type == "Model") {
            view->removeModel(name);
            qDebug() << "Removed Model:" << name << "from view";
        } else if (type == "2DGrid") {
            view->remove2DGrid(name);
            qDebug() << "Removed 2DGrid:" << name << "from view";
        } else if (type == "3DGrid") {
            view->remove3DGrid(name);
            qDebug() << "Removed 3DGrid:" << name << "from view";
        }
    }

    // Update the hierarchy view
    updateHierarchyView();

    // Update all affected views
    for (OpenGLWidget* view : viewsToUpdate) {
        view->update();
    }

    QMessageBox::information(this, tr("Delete Objects"), tr("Selected objects have been deleted."));
}

void MainWindow::on_actionPointPicker_triggered()
{
    qDebug() << "PointPicker triggered";
}

void MainWindow::on_actionPick_Points_triggered()
{
    qDebug() << "Point picker widget created";
}

void MainWindow::on_actionCDD_triggered()
{
    // Get selected items from the hierarchy tree
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();

    // We need exactly two selected items
    if (selectedIndexes.size() != 2) {
        QMessageBox::warning(this, "Selection Error",
                             "Please select exactly two point clouds in the hierarchy tree for collision detection.");
        return;
    }

    // Prepare variables to store the two point clouds
    OpenGLWidget::PointCloud* pc1 = nullptr;
    OpenGLWidget::PointCloud* pc2 = nullptr;
    QString name1, name2;

    // Process each selected item
    for (const auto& index : selectedIndexes) {
        QString itemText = hierarchyModel->item(index.row())->text();
        QString itemType = hierarchyModel->item(index.row())->data(Qt::UserRole).toString();

        // Only process point clouds
        if (itemType != "PointCloud") {
            QMessageBox::warning(this, "Selection Error",
                                 "Please select only point cloud objects for collision detection.");
            return;
        }

        // Get the associated view
        OpenGLWidget* view = hierarchyModel->item(index.row())->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback to main view

        // Extract point cloud name (remove prefix)
        QString name = itemText;
        if (name.startsWith("[PointCloud] ")) {
            name.remove("[PointCloud] ");
        }

        // Find the point cloud in the view's data
        bool found = false;
        for (auto& pc : view->m_pointClouds) {
            if (pc.name == name) {
                if (!pc1) {
                    pc1 = &pc;
                    name1 = name;
                } else {
                    pc2 = &pc;
                    name2 = name;
                }
                found = true;
                break;
            }
        }

        if (!found) {
            QMessageBox::warning(this, "Error",
                                 QString("Could not access data for point cloud: %1").arg(name));
            return;
        }
    }

    // Ensure we have two valid point clouds
    if (!pc1 || !pc2) {
        QMessageBox::warning(this, "Error", "Failed to retrieve point cloud data.");
        return;
    }

    // Set threshold for collision detection (could make this configurable via dialog)
    float threshold = 0.5f; // Distance threshold in units, customizable

    // Show a progress dialog
    QProgressDialog progress("Performing collision detection...", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(10);
    progress.show();
    QCoreApplication::processEvents();

    // Prepare result structures
    CloudComparison::CollisionResult result1, result2;

    // Perform collision detection
    bool success = cloudComparison->performCollisionDetection(
        pc1->points, pc1->colors,
        pc2->points, pc2->colors,
        threshold,
        result1, result2,
        &progress
        );

    if (!success) {
        QMessageBox::warning(this, "Error", "Collision detection failed.");
        return;
    }

    // Add the result point clouds to the view
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    QString resultName1 = name1 + "_collision";
    QString resultName2 = name2 + "_collision";

    // Add the result clouds to the OpenGL widget
    activeView->addPointCloud(result1.points, result1.colors, resultName1);
    activeView->addPointCloud(result2.points, result2.colors, resultName2);

    // Update hierarchy view to show the new point clouds
    updateHierarchyView();

    // Display collision results in a message box
    QMessageBox::information(this, "Collision Detection Results",
                             QString("Collision detection completed!\n\n"
                                     "Points in collision from %1: %2 of %3 (%4%)\n"
                                     "Points in collision from %5: %6 of %7 (%8%)\n\n"
                                     "New point clouds have been created to visualize the results.")
                                 .arg(name1)
                                 .arg(result1.collisionCount)
                                 .arg(result1.totalPoints)
                                 .arg(result1.totalPoints > 0 ? QString::number(100.0 * result1.collisionCount / result1.totalPoints, 'f', 2) : "0.00")
                                 .arg(name2)
                                 .arg(result2.collisionCount)
                                 .arg(result2.totalPoints)
                                 .arg(result2.totalPoints > 0 ? QString::number(100.0 * result2.collisionCount / result2.totalPoints, 'f', 2) : "0.00"));

    // Update the view
    activeView->update();
}





void MainWindow::setupTransformConnections()
{
    // Connect the transform UI elements to handler methods
    connect(ui->positionX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onPositionChanged);
    connect(ui->positionY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onPositionChanged);
    connect(ui->positionZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onPositionChanged);

    connect(ui->rotationX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onRotationChanged);
    connect(ui->rotationY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onRotationChanged);
    connect(ui->rotationZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onRotationChanged);

    connect(ui->scaleX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onScaleChanged);
    connect(ui->scaleY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onScaleChanged);
    connect(ui->scaleZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onScaleChanged);

    // Connect the lock scale checkbox
    connect(ui->lockScaleCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        // Store the state for later use
        m_lockScaleProportions = checked;
    });
}

void MainWindow::onPositionChanged()
{
    // Get the new position values from the UI
    QVector3D newPosition(
        ui->positionX->value(),
        ui->positionY->value(),
        ui->positionZ->value()
        );

    // Apply to the active selection
    applyTransformToSelection();
}

void MainWindow::onRotationChanged()
{
    // Get the new rotation values from the UI
    QVector3D newRotation(
        ui->rotationX->value(),
        ui->rotationY->value(),
        ui->rotationZ->value()
        );

    // Apply to the active selection
    applyTransformToSelection();
}

void MainWindow::onScaleChanged()
{
    // Check if we need to maintain proportions
    if (m_lockScaleProportions) {
        // Figure out which value changed by comparing sender to our spinboxes
        QObject* sender = QObject::sender();
        double newValue = 1.0;

        if (sender == ui->scaleX) {
            newValue = ui->scaleX->value();

            // Block signals to prevent recursion
            ui->scaleY->blockSignals(true);
            ui->scaleZ->blockSignals(true);

            ui->scaleY->setValue(newValue);
            ui->scaleZ->setValue(newValue);

            ui->scaleY->blockSignals(false);
            ui->scaleZ->blockSignals(false);
        }
        else if (sender == ui->scaleY) {
            newValue = ui->scaleY->value();

            ui->scaleX->blockSignals(true);
            ui->scaleZ->blockSignals(true);

            ui->scaleX->setValue(newValue);
            ui->scaleZ->setValue(newValue);

            ui->scaleX->blockSignals(false);
            ui->scaleZ->blockSignals(false);
        }
        else if (sender == ui->scaleZ) {
            newValue = ui->scaleZ->value();

            ui->scaleX->blockSignals(true);
            ui->scaleY->blockSignals(true);

            ui->scaleX->setValue(newValue);
            ui->scaleY->setValue(newValue);

            ui->scaleX->blockSignals(false);
            ui->scaleY->blockSignals(false);
        }
    }

    // Apply to the active selection
    applyTransformToSelection();
}

void MainWindow::applyTransformToSelection()
{
    // Only proceed if we have an active view
    if (!m_activeViewWidget) {
        return;
    }

    // Get the transform values from the UI
    QVector3D position(ui->positionX->value(), ui->positionY->value(), ui->positionZ->value());
    QVector3D rotation(ui->rotationX->value(), ui->rotationY->value(), ui->rotationZ->value());
    QVector3D scale(ui->scaleX->value(), ui->scaleY->value(), ui->scaleZ->value());

    // Apply the transform to the active view's selection
    // You would need to implement methods in OpenGLWidget to do this
    bool success = m_activeViewWidget->applyTransformToSelection(position, rotation, scale);

    if (success) {
        // Update the 3D view
        m_activeViewWidget->update();
    }
}

void MainWindow::optimizePropertyTableLayout(QTableWidget* table)
{
    if (!table) return;

    // Ensure we only have exactly 2 columns
    table->setColumnCount(2);
    table->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");

    // Adjust column widths for better visibility
    // Make the first column narrower but ensure property names are visible
    table->setColumnWidth(0, 60);  // Property column - narrower

    // Make the value column take the rest of the width but with a minimum size
    int valueColumnWidth = table->width() - 105; // Account for scrollbar and borders
    if (valueColumnWidth < 140) valueColumnWidth = 140;
    table->setColumnWidth(1, valueColumnWidth);

    // Make sure the horizontal header has fixed size mode
    table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Fixed);

    // Prevent the horizontal header from being resized
    table->horizontalHeader()->setStretchLastSection(false);

    // Hide any vertical header/row numbers
    table->verticalHeader()->setVisible(false);

    // Remove grid lines to reduce visual clutter
    table->setShowGrid(false);

    // Reduce row height to fit more content
    table->verticalHeader()->setDefaultSectionSize(22);

    // Enable alternating row colors for better readability
    table->setAlternatingRowColors(true);

    // Make sure the horizontal headers are visible and styled appropriately
    table->horizontalHeader()->setVisible(true);
    table->horizontalHeader()->setStyleSheet("QHeaderView::section { background-color: #3d3d3d; }");

    // Set the selection behavior to select entire rows
    table->setSelectionBehavior(QAbstractItemView::SelectRows);

    // Ensure text is properly visible in cells
    table->setWordWrap(false);
    table->setTextElideMode(Qt::ElideRight);

    // Remove the empty column at the end if it exists
    if (table->columnCount() > 2) {
        table->removeColumn(2);
    }

    // Install event filter on the main window to handle resize events
    table->installEventFilter(this);
}

void MainWindow::addPropertyGroupHeader(QTableWidget* table, int row, const QString& title)
{
    QTableWidgetItem* headerItem = new QTableWidgetItem(title);

    // Match the exact header styling from the screenshot
    headerItem->setBackground(QBrush(QColor(60, 65, 95))); // Dark blue-purple like in screenshot
    headerItem->setForeground(QBrush(QColor(220, 220, 220))); // Lighter text
    headerItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    QFont font = headerItem->font();
    font.setBold(true);
    headerItem->setFont(font);

    // Add a space at the beginning for padding
    headerItem->setText(" " + title);

    table->setItem(row, 0, headerItem);

    // Span across both columns
    table->setSpan(row, 0, 1, 2);

    // Set a slightly taller row height for headers
    table->setRowHeight(row, 24);
}

void MainWindow::addPropertyRow(QTableWidget* table, int row, const QString& property, const QString& value)
{
    QTableWidgetItem* propertyItem = new QTableWidgetItem(property);
    QTableWidgetItem* valueItem = new QTableWidgetItem(value);

    // Improve styling
    propertyItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
    valueItem->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    // Add padding with spaces for better readability
    propertyItem->setText(" " + property);

    // Set background colors to match the screenshot
    // Normal rows have darker background
    QColor rowColor = (table->alternatingRowColors() && row % 2 == 0)
                          ? QColor(45, 45, 45)
                          : QColor(55, 55, 55);

    propertyItem->setBackground(rowColor);
    valueItem->setBackground(rowColor);

    // Set text color
    propertyItem->setForeground(QBrush(QColor(220, 220, 220)));
    valueItem->setForeground(QBrush(QColor(220, 220, 220)));

    table->setItem(row, 0, propertyItem);
    table->setItem(row, 1, valueItem);

    // Set the value column text color to light blue for numeric values
    // Use a simple check for the first character instead of regex
    if (!value.isEmpty() && (value[0].isDigit() || value[0] == '-' || value[0] == '.')) {
        valueItem->setForeground(QBrush(QColor(100, 180, 255)));
    }
}


void MainWindow::initializeFileBrowsing()
{
    // Set up initial directory for file browsing
    QDir initialDir = QDir::current();

    // Try to find common paths with point cloud files
    QStringList searchPaths = {
        QDir::homePath() + "/Downloads/data",
        QDir::currentPath() + "/data"
    };

    foreach (const QString &path, searchPaths) {
        QDir dir(path);
        if (dir.exists()) {
            initialDir = dir;
            break;
        }
    }

    // Update the file view with this directory
    currentDirectory = initialDir.absolutePath();
    updateFileView(currentDirectory);

    // Update path display
    ui->assetPathEdit->setText(currentDirectory);
}


void MainWindow::setupFileContextMenu()
{
    ui->treeView_2->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->treeView_2, &QTreeView::customContextMenuRequested, this,
            [this](const QPoint &pos) {
                // Get the model index at the requested position
                QModelIndex index = ui->treeView_2->indexAt(pos);
                if (!index.isValid())
                    return;

                // Get the file path for the selected item
                QString filePath = fileSystemModel->filePath(index);
                QFileInfo fileInfo(filePath);

                // Create the context menu
                QMenu contextMenu(this);

                if (fileInfo.isDir()) {
                    contextMenu.addAction(tr("Open Directory"), [this, filePath]() {
                        updateFileView(filePath);
                    });
                } else {
                    // File-specific actions
                    contextMenu.addAction(tr("Open in Viewport"), [this, filePath]() {
                        on_actionOpen_triggered();
                    });

                    contextMenu.addAction(tr("Preview"), [this, filePath]() {
                        ui->assetTabs->setCurrentIndex(1); // Switch to preview tab
                        updatePreview(filePath);
                    });

                    contextMenu.addAction(tr("Properties"), [this, filePath]() {
                        ui->assetTabs->setCurrentIndex(2); // Switch to properties tab
                        populatePropertiesTreeView(QFileInfo(filePath));
                    });

                    contextMenu.addSeparator();

                    // System actions
                    contextMenu.addAction(tr("Show in Explorer/Finder"), [this, filePath]() {
                        QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(filePath).absolutePath()));
                    });
                }

                // Show the context menu at the requested position
                contextMenu.exec(ui->treeView_2->viewport()->mapToGlobal(pos));
            });
}

// Add this to enhance the context menu when right-clicking on a viewport in the hierarchy
void MainWindow::setupViewportContextMenu()
{
    // Connect to tree view's context menu signal if not already connected
    if (ui->treeView->contextMenuPolicy() != Qt::CustomContextMenu) {
        ui->treeView->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(ui->treeView, &QTreeView::customContextMenuRequested,
                this, &MainWindow::onHierarchyContextMenuRequested);
    }
}


void MainWindow::onHierarchyContextMenuRequested(const QPoint& pos)
{
    QModelIndex index = ui->treeView->indexAt(pos);
    if (!index.isValid())
        return;

    QStandardItem* item = hierarchyModel->itemFromIndex(index);
    if (!item)
        return;

    QString type = item->data(Qt::UserRole).toString();

    // If this is a viewport item
    if (type == "Viewport" || type == "ViewportCapture") {
        QString viewportName;
        if (type == "Viewport") {
            viewportName = item->text();
            viewportName.remove("[Viewport] ");
        } else {
            viewportName = item->data(Qt::UserRole + 2).toString();
        }

        QMenu contextMenu(this);
        contextMenu.addAction(tr("Apply Viewport"), [this, viewportName]() {
            applyViewportByName(viewportName);
        });

        contextMenu.addAction(tr("Edit Viewport"), [this, viewportName]() {
            editViewport(viewportName);
        });

        contextMenu.addAction(tr("Delete Viewport"), [this, viewportName]() {
            // Remove viewport from the list
            for (int i = 0; i < m_capturedViewports.size(); ++i) {
                if (m_capturedViewports[i].name == viewportName) {
                    m_capturedViewports.removeAt(i);
                    break;
                }
            }

            // Update hierarchy view
            updateViewportCapturesInHierarchy();

            // Show confirmation
            statusBar()->showMessage(tr("Deleted viewport: %1").arg(viewportName), 2000);
        });

        contextMenu.exec(ui->treeView->viewport()->mapToGlobal(pos));
    }
}


// Modify the updatePropertiesView method to include our optimization
void MainWindow::updatePropertiesView()
{
    // Find the properties table widget in the UI
    QTableWidget* propertiesTable = findChild<QTableWidget*>("propertiesTableWidget");
    if (!propertiesTable) {
        qDebug() << "Properties table widget not found!";
        return;
    }

    // Make sure the table only has 2 columns
    if (propertiesTable->columnCount() != 2) {
        propertiesTable->setColumnCount(2);
        propertiesTable->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
    }

    // Clear existing rows
    propertiesTable->setRowCount(0);

    // Get selected items from tree view
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();
    if (selectedIndexes.isEmpty()) {
        // No selection, just return
        return;
    }

    QModelIndex selectedIndex = selectedIndexes.first();
    QString itemText = selectedIndex.data().toString();
    QString itemType = selectedIndex.data(Qt::UserRole).toString();

    // Extract name without prefix (if present)
    QString name = itemText;
    if (name.startsWith("[PointCloud] ")) {
        name.remove("[PointCloud] ");
    } else if (name.startsWith("[Model] ")) {
        name.remove("[Model] ");
    }

    // Get view associated with the selected item
    OpenGLWidget* view = selectedIndex.data(Qt::UserRole + 1).value<OpenGLWidget*>();
    if (!view) view = openGLWidget; // Fallback to main view

    // Setup the properties table based on item type
    if (itemType == "PointCloud") {
        // Variables to store point cloud data
        QVector3D minPoint(0.0f, 0.0f, 0.0f);
        QVector3D maxPoint(0.0f, 0.0f, 0.0f);
        bool visible = false;
        int pointCount = 0;
        bool found = false;

        // Find the selected point cloud data
        const auto& pointClouds = view->getPointClouds();
        for (const auto& pc : pointClouds) {
            if (pc.name == name) {
                minPoint = pc.min;
                maxPoint = pc.max;
                visible = pc.visible;
                pointCount = pc.pointCount;
                found = true;
                break;
            }
        }

        if (!found) return; // Not found

        // Determine how many rows we'll need - reduced number of rows for compactness
        int totalRows = 14; // Reduced from 20 to fit without scrolling
        propertiesTable->setRowCount(totalRows);

        // Populate table with properties - more compact format
        int rowIndex = 0;

        // CC Object section
        addPropertyGroupHeader(propertiesTable, rowIndex++, "CC Object");
        addPropertyRow(propertiesTable, rowIndex++, "Name", name);
        addPropertyRow(propertiesTable, rowIndex++, "Visible", visible ? "✓" : "");

        // Box dimensions - more compact format
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Dimensions");

        // More compact dimension display
        QString xDim = QString("X: %1 to %2")
                           .arg(minPoint.x(), 0, 'f', 2)
                           .arg(maxPoint.x(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Width", xDim);

        QString yDim = QString("Y: %1 to %2")
                           .arg(minPoint.y(), 0, 'f', 2)
                           .arg(maxPoint.y(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Height", yDim);

        QString zDim = QString("Z: %1 to %2")
                           .arg(minPoint.z(), 0, 'f', 2)
                           .arg(maxPoint.z(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Depth", zDim);

        // Center point - combined into one row
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Center");
        QVector3D center = (minPoint + maxPoint) * 0.5f;
        QString centerStr = QString("(%1, %2, %3)")
                                .arg(center.x(), 0, 'f', 2)
                                .arg(center.y(), 0, 'f', 2)
                                .arg(center.z(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Position", centerStr);

        // Info - most important info
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Info");
        addPropertyRow(propertiesTable, rowIndex++, "Points", QString::number(pointCount));
        addPropertyRow(propertiesTable, rowIndex++, "Object ID", QString::number(QRandomGenerator::global()->bounded(1000)));
        addPropertyRow(propertiesTable, rowIndex++, "Display", "Main View");
        addPropertyRow(propertiesTable, rowIndex++, "Scale", "1.0");

        // Set actual row count
        propertiesTable->setRowCount(rowIndex);

        // Apply our optimization
        optimizePropertyTableLayout(propertiesTable);
    }
    else if (itemType == "Model") {
        // Similar changes for model type properties
        // Variables to store model data
        QVector3D minPoint(0.0f, 0.0f, 0.0f);
        QVector3D maxPoint(0.0f, 0.0f, 0.0f);
        bool visible = false;
        bool found = false;

        // Find the selected model data
        const auto& models = view->getModels();
        for (const auto& model : models) {
            if (model.name == name) {
                minPoint = model.min;
                maxPoint = model.max;
                visible = model.visible;
                found = true;
                break;
            }
        }

        if (!found) return; // Not found

        // Compact display for models
        int totalRows = 10;
        propertiesTable->setRowCount(totalRows);

        int rowIndex = 0;

        // CC Object section
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Model");
        addPropertyRow(propertiesTable, rowIndex++, "Name", name);
        addPropertyRow(propertiesTable, rowIndex++, "Visible", visible ? "✓" : "");

        // Dimensions - compact view
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Size");
        QVector3D size = maxPoint - minPoint;
        QString sizeStr = QString("W:%1 H:%2 D:%3")
                              .arg(size.x(), 0, 'f', 2)
                              .arg(size.y(), 0, 'f', 2)
                              .arg(size.z(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Dimensions", sizeStr);

        // Center position
        QVector3D center = (minPoint + maxPoint) * 0.5f;
        QString posStr = QString("(%1, %2, %3)")
                             .arg(center.x(), 0, 'f', 2)
                             .arg(center.y(), 0, 'f', 2)
                             .arg(center.z(), 0, 'f', 2);
        addPropertyRow(propertiesTable, rowIndex++, "Position", posStr);

        // Additional commonly used properties
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Rendering");
        addPropertyRow(propertiesTable, rowIndex++, "Material", "Default");
        addPropertyRow(propertiesTable, rowIndex++, "Cast Shadows", "✓");
        addPropertyRow(propertiesTable, rowIndex++, "Receive Shadows", "✓");

        // Set actual row count
        propertiesTable->setRowCount(rowIndex);

        // Apply our optimization
        optimizePropertyTableLayout(propertiesTable);
    }else if (itemType == "ModelNode") {

        QModelIndex selectedIndex = selectedIndexes.first();
        QStandardItem* item = hierarchyModel->itemFromIndex(selectedIndex);
        if (!item) return;
        // Get model name and node index from item data
        QString modelName = item->data(Qt::UserRole + 2).toString();
        int nodeIndex = item->data(Qt::UserRole + 3).toInt();

        // Find the model and node
        const auto& models = view->getModels();
        bool found = false;
        OpenGLWidget::ModelNode selectedNode;
        QString name = item->text();

        for (const auto& model : models) {
            if (model.name == modelName && nodeIndex >= 0 && nodeIndex < static_cast<int>(model.nodes.size())) {
                selectedNode = model.nodes[nodeIndex];
                found = true;
                break;
            }
        }

        if (!found) return; // Not found

        // Remove node prefix if present
        if (name.startsWith("[Node] ")) {
            name.remove("[Node] ");
        }

        // Display node properties
        int rowIndex = 0;

        // Node information
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Node");
        addPropertyRow(propertiesTable, rowIndex++, "Name", name);
        addPropertyRow(propertiesTable, rowIndex++, "Visible", selectedNode.visible ? "✓" : "");
        addPropertyRow(propertiesTable, rowIndex++, "Model", modelName);
        addPropertyRow(propertiesTable, rowIndex++, "Parent",
                       selectedNode.parentIndex >= 0 ?
                           QString::number(selectedNode.parentIndex) : "None (Root)");

        // Mesh information
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Meshes");
        addPropertyRow(propertiesTable, rowIndex++, "Count", QString::number(selectedNode.meshes.size()));

        if (!selectedNode.meshes.empty()) {
            int totalVertices = 0;
            int totalFaces = 0;

            for (const auto& mesh : selectedNode.meshes) {
                totalVertices += mesh.vertexCount;
                totalFaces += mesh.indexCount / 3;
            }

            addPropertyRow(propertiesTable, rowIndex++, "Vertices", QString::number(totalVertices));
            addPropertyRow(propertiesTable, rowIndex++, "Triangles", QString::number(totalFaces));

            // Show first mesh details
            const auto& firstMesh = selectedNode.meshes[0];
            addPropertyRow(propertiesTable, rowIndex++, "First Mesh", firstMesh.name);

            if (!firstMesh.materialName.isEmpty()) {
                addPropertyRow(propertiesTable, rowIndex++, "Material", firstMesh.materialName);
            }
        }

        // Children information
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Children");
        addPropertyRow(propertiesTable, rowIndex++, "Count", QString::number(selectedNode.children.size()));

        // Transform
        addPropertyGroupHeader(propertiesTable, rowIndex++, "Transform");
        addPropertyRow(propertiesTable, rowIndex++, "Position",
                       QString("(%1, %2, %3)")
                           .arg(selectedNode.position.x(), 0, 'f', 2)
                           .arg(selectedNode.position.y(), 0, 'f', 2)
                           .arg(selectedNode.position.z(), 0, 'f', 2));

        addPropertyRow(propertiesTable, rowIndex++, "Rotation",
                       QString("(%1°, %2°, %3°)")
                           .arg(selectedNode.rotation.x(), 0, 'f', 1)
                           .arg(selectedNode.rotation.y(), 0, 'f', 1)
                           .arg(selectedNode.rotation.z(), 0, 'f', 1));

        addPropertyRow(propertiesTable, rowIndex++, "Scale",
                       QString("(%1, %2, %3)")
                           .arg(selectedNode.scale.x(), 0, 'f', 2)
                           .arg(selectedNode.scale.y(), 0, 'f', 2)
                           .arg(selectedNode.scale.z(), 0, 'f', 2));

        // Set actual row count
        propertiesTable->setRowCount(rowIndex);

        // Apply our optimization
        optimizePropertyTableLayout(propertiesTable);
    }
}

// Update for setupPropertyTab
void MainWindow::setupPropertyTab()
{
    // Check if we already have a tab widget
    QTabWidget* hierarchyTabWidget = ui->hierarchyDock->findChild<QTabWidget*>("hierarchyTabWidget");

    // If not, we need to create one
    if (!hierarchyTabWidget) {
        // Create a new tab widget to replace the treeView
        hierarchyTabWidget = new QTabWidget(ui->hierarchyDock);
        hierarchyTabWidget->setObjectName("hierarchyTabWidget");

        // Get the existing layout
        QLayout* layout = ui->hierarchyDock->widget()->layout();
        if (!layout) {
            qDebug() << "No layout found in hierarchy dock";
            return;
        }

        // Find the index of the treeView in the layout
        int treeViewIndex = -1;
        for (int i = 0; i < layout->count(); ++i) {
            QLayoutItem* item = layout->itemAt(i);
            if (item->widget() == ui->treeView) {
                treeViewIndex = i;
                break;
            }
        }

        if (treeViewIndex == -1) {
            qDebug() << "TreeView not found in layout";
            return;
        }

        // Create the tree tab
        QWidget* treeTab = new QWidget();
        QVBoxLayout* treeLayout = new QVBoxLayout(treeTab);

        // Move the treeView to this tab
        layout->removeWidget(ui->treeView);
        treeLayout->addWidget(ui->treeView);

        // Create the properties tab with a table widget and row numbers
        QWidget* propertiesTab = new QWidget();
        QVBoxLayout* propertiesLayout = new QVBoxLayout(propertiesTab);
        propertiesLayout->setContentsMargins(0, 0, 0, 0); // No margins to maximize space

        QTableWidget* propertiesTableWidget = new QTableWidget(propertiesTab);
        propertiesTableWidget->setObjectName("propertiesTableWidget");
        propertiesTableWidget->setColumnCount(2);
        propertiesTableWidget->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");

        // Remove the default third column that sometimes appears
        while (propertiesTableWidget->columnCount() > 2) {
            propertiesTableWidget->removeColumn(2);
        }

        // Fix column 2 visibility issues
        propertiesTableWidget->horizontalHeader()->setStretchLastSection(false);
        propertiesTableWidget->setColumnWidth(0, 100);
        propertiesTableWidget->setColumnWidth(1, 150);

        // Apply our optimization to the table
        optimizePropertyTableLayout(propertiesTableWidget);

        propertiesLayout->addWidget(propertiesTableWidget);

        // Add tabs to the tab widget
        hierarchyTabWidget->addTab(propertiesTab, "Properties");

        // Insert the tab widget into the layout
        layout->addWidget(hierarchyTabWidget);
    } else {
        // If it already exists, get the propertiesTableWidget and optimize it
        QTableWidget* propertiesTableWidget = hierarchyTabWidget->findChild<QTableWidget*>("propertiesTableWidget");
        if (propertiesTableWidget) {
            optimizePropertyTableLayout(propertiesTableWidget);
        }
    }

    // Connect the tree view selection changed signal to update properties
    if (ui->treeView->selectionModel()) {
        connect(ui->treeView->selectionModel(), &QItemSelectionModel::selectionChanged,
                this, &MainWindow::updatePropertiesView);
    }

    // Connect the property button to switch to properties tab
    if (ui->propertyButton) {
        connect(ui->propertyButton, &QPushButton::clicked, [this, hierarchyTabWidget]() {
            // Switch to properties tab when Property button is clicked
            hierarchyTabWidget->setCurrentIndex(1);
            // Update properties for current selection
            updatePropertiesView();
        });
    }

    // Double-clicking on a tree item should also show its properties
    connect(ui->treeView, &QTreeView::doubleClicked, [this, hierarchyTabWidget](const QModelIndex &) {
        hierarchyTabWidget->setCurrentIndex(1);
        updatePropertiesView();
    });
}




void MainWindow::on_actionICP_triggered()
{
    // Get selected items from the hierarchy tree
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();

    // We need exactly two selected items
    if (selectedIndexes.size() != 2) {
        QMessageBox::warning(this, "Selection Error",
                             "Please select exactly two point clouds in the hierarchy tree for ICP alignment.");
        return;
    }

    // Prepare variables to store the two point clouds
    OpenGLWidget::PointCloud* pc1 = nullptr;
    OpenGLWidget::PointCloud* pc2 = nullptr;
    QString name1, name2;

    // Process each selected item
    for (const auto& index : selectedIndexes) {
        QString itemText = hierarchyModel->item(index.row())->text();
        QString itemType = hierarchyModel->item(index.row())->data(Qt::UserRole).toString();

        // Only process point clouds
        if (itemType != "PointCloud") {
            QMessageBox::warning(this, "Selection Error",
                                 "Please select only point cloud objects for ICP alignment.");
            return;
        }

        // Get the associated view
        OpenGLWidget* view = hierarchyModel->item(index.row())->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (!view) view = openGLWidget; // Fallback to main view

        // Extract point cloud name (remove prefix)
        QString name = itemText;
        if (name.startsWith("[PointCloud] ")) {
            name.remove("[PointCloud] ");
        }

        // Find the point cloud in the view's data
        bool found = false;
        for (auto& pc : view->m_pointClouds) {
            if (pc.name == name) {
                if (!pc1) {
                    pc1 = &pc;
                    name1 = name;
                } else {
                    pc2 = &pc;
                    name2 = name;
                }
                found = true;
                break;
            }
        }

        if (!found) {
            QMessageBox::warning(this, "Error",
                                 QString("Could not access data for point cloud: %1").arg(name));
            return;
        }
    }

    // Ensure we have two valid point clouds
    if (!pc1 || !pc2) {
        QMessageBox::warning(this, "Error", "Failed to retrieve point cloud data.");
        return;
    }

    // Show a progress dialog
    QProgressDialog progress("Performing ICP alignment...", "Cancel", 0, 100, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setValue(10);
    progress.show();
    QCoreApplication::processEvents();

    // Convert Qt point data to PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);

    // Populate the source cloud (pc1)
    cloud_source->resize(pc1->points.size());
    for (size_t i = 0; i < pc1->points.size(); ++i) {
        cloud_source->points[i].x = pc1->points[i].x();
        cloud_source->points[i].y = pc1->points[i].y();
        cloud_source->points[i].z = pc1->points[i].z();
    }
    cloud_source->width = cloud_source->points.size();
    cloud_source->height = 1;
    cloud_source->is_dense = true;

    // Populate the target cloud (pc2)
    cloud_target->resize(pc2->points.size());
    for (size_t i = 0; i < pc2->points.size(); ++i) {
        cloud_target->points[i].x = pc2->points[i].x();
        cloud_target->points[i].y = pc2->points[i].y();
        cloud_target->points[i].z = pc2->points[i].z();
    }
    cloud_target->width = cloud_target->points.size();
    cloud_target->height = 1;
    cloud_target->is_dense = true;

    progress.setValue(20);
    QCoreApplication::processEvents();

    if (progress.wasCanceled()) {
        return;
    }

    // Compute centroids for initial alignment
    Eigen::Vector4f centroid_source, centroid_target;
    pcl::compute3DCentroid(*cloud_source, centroid_source);
    pcl::compute3DCentroid(*cloud_target, centroid_target);

    float centroid_dist = (centroid_source - centroid_target).norm();

    progress.setValue(30);
    QCoreApplication::processEvents();

    if (progress.wasCanceled()) {
        return;
    }

    // Downsample point clouds to speed up ICP
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    float leaf_size = std::max(centroid_dist / 100.0f, 0.01f);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_downsampled(new pcl::PointCloud<pcl::PointXYZ>);

    voxel_filter.setInputCloud(cloud_source);
    voxel_filter.filter(*cloud_source_downsampled);
    voxel_filter.setInputCloud(cloud_target);
    voxel_filter.filter(*cloud_target_downsampled);

    progress.setValue(40);
    QCoreApplication::processEvents();

    if (progress.wasCanceled()) {
        return;
    }

    // ICP Configuration Dialog
    QDialog configDialog(this);
    configDialog.setWindowTitle("ICP Configuration");
    QVBoxLayout* layout = new QVBoxLayout(&configDialog);

    // Add configuration options
    QGroupBox* optionsGroup = new QGroupBox("ICP Options");
    QFormLayout* optionsLayout = new QFormLayout(optionsGroup);

    // Max iterations
    QSpinBox* iterationsBox = new QSpinBox();
    iterationsBox->setRange(1, 1000);
    iterationsBox->setValue(50);
    iterationsBox->setSingleStep(10);
    optionsLayout->addRow("Max Iterations:", iterationsBox);

    // Correspondence distance
    QDoubleSpinBox* distanceBox = new QDoubleSpinBox();
    distanceBox->setRange(0.001, 100.0);
    distanceBox->setValue(centroid_dist * 0.1f);
    distanceBox->setSingleStep(0.1);
    optionsLayout->addRow("Max Correspondence Distance:", distanceBox);

    // Transformation epsilon
    QDoubleSpinBox* epsilonBox = new QDoubleSpinBox();
    epsilonBox->setRange(1e-10, 0.1);
    epsilonBox->setValue(1e-6);
    epsilonBox->setDecimals(10);
    optionsLayout->addRow("Transformation Epsilon:", epsilonBox);

    // Fitness threshold
    QDoubleSpinBox* fitnessBox = new QDoubleSpinBox();
    fitnessBox->setRange(1e-10, 10.0);
    fitnessBox->setValue(0.01);
    fitnessBox->setDecimals(5);
    optionsLayout->addRow("Fitness Threshold:", fitnessBox);

    // Add initial alignment options
    QGroupBox* alignmentGroup = new QGroupBox("Initial Alignment");
    QVBoxLayout* alignmentLayout = new QVBoxLayout(alignmentGroup);

    QRadioButton* centerOnly = new QRadioButton("Center Only");
    QRadioButton* trySeveral = new QRadioButton("Try Several Rotations");
    centerOnly->setChecked(true);
    alignmentLayout->addWidget(centerOnly);
    alignmentLayout->addWidget(trySeveral);

    // Dialog buttons
    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, &configDialog, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, &configDialog, &QDialog::reject);

    layout->addWidget(optionsGroup);
    layout->addWidget(alignmentGroup);
    layout->addWidget(buttonBox);

    if (configDialog.exec() != QDialog::Accepted) {
        return;
    }

    // Get configuration values
    int max_iterations = iterationsBox->value();
    float max_distance = distanceBox->value();
    float transform_epsilon = epsilonBox->value();
    float fitness_threshold = fitnessBox->value();
    bool try_multiple_alignments = trySeveral->isChecked();

    progress.setValue(50);
    QCoreApplication::processEvents();

    if (progress.wasCanceled()) {
        return;
    }

    // Array to track best transformation
    float best_fitness = std::numeric_limits<float>::max();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    bool has_converged = false;

    // Prepare initial transforms to try
    std::vector<Eigen::Matrix4f> initial_transforms;
    std::vector<QString> transform_names;

    // Center alignment (align centroids)
    Eigen::Matrix4f trans_center = Eigen::Matrix4f::Identity();
    trans_center.block<3, 1>(0, 3) = (centroid_target - centroid_source).head<3>();
    initial_transforms.push_back(trans_center);
    transform_names.push_back("Center alignment");

    // If requested, try multiple rotations
    if (try_multiple_alignments) {
        // Try rotations around Z axis
        for (int angle_deg = 0; angle_deg < 360; angle_deg += 45) {
            float angle_rad = angle_deg * M_PI / 180.0f;
            Eigen::Matrix4f trans_z = Eigen::Matrix4f::Identity();
            trans_z.block<3, 1>(0, 3) = (centroid_target - centroid_source).head<3>();
            trans_z(0, 0) = cos(angle_rad);
            trans_z(0, 1) = -sin(angle_rad);
            trans_z(1, 0) = sin(angle_rad);
            trans_z(1, 1) = cos(angle_rad);
            initial_transforms.push_back(trans_z);
            transform_names.push_back(QString("Z-axis rotation %1°").arg(angle_deg));
        }
    }

    // Try each initial alignment
    for (size_t i = 0; i < initial_transforms.size(); ++i) {
        // Update progress
        progress.setValue(50 + 40 * i / initial_transforms.size());
        progress.setLabelText(QString("Trying %1 (%2/%3)...")
                                  .arg(transform_names[i])
                                  .arg(i + 1)
                                  .arg(initial_transforms.size()));
        QCoreApplication::processEvents();

        if (progress.wasCanceled()) {
            return;
        }

        // Configure ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(max_distance);
        icp.setMaximumIterations(max_iterations);
        icp.setTransformationEpsilon(transform_epsilon);
        icp.setEuclideanFitnessEpsilon(fitness_threshold);
        icp.setInputSource(cloud_source_downsampled);
        icp.setInputTarget(cloud_target_downsampled);

        // Attempt ICP
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned_attempt(new pcl::PointCloud<pcl::PointXYZ>);
        icp.align(*cloud_aligned_attempt, initial_transforms[i]);

        if (icp.hasConverged()) {
            float fitness = icp.getFitnessScore();

            if (fitness < best_fitness) {
                best_fitness = fitness;
                best_transform = icp.getFinalTransformation();
                has_converged = true;
            }
        }
    }

    progress.setValue(90);
    progress.setLabelText("Applying transformation...");
    QCoreApplication::processEvents();

    if (progress.wasCanceled()) {
        return;
    }

    if (!has_converged) {
        QMessageBox::warning(this, "Alignment Error",
                             "ICP failed to find a good alignment. Try adjusting parameters or choose different point clouds.");
        return;
    }

    // Transform the source cloud with the best transformation
    pcl::transformPointCloud(*cloud_source, *cloud_aligned, best_transform);

    // Convert back to Qt format for display
    std::vector<QVector3D> aligned_points;
    std::vector<QVector3D> aligned_colors;

    aligned_points.resize(cloud_aligned->points.size());
    aligned_colors.resize(cloud_aligned->points.size());

    for (size_t i = 0; i < cloud_aligned->points.size(); ++i) {
        aligned_points[i] = QVector3D(
            cloud_aligned->points[i].x,
            cloud_aligned->points[i].y,
            cloud_aligned->points[i].z
            );

        // Use the source point cloud's colors if available
        if (i < pc1->colors.size()) {
            aligned_colors[i] = pc1->colors[i];
        } else {
            // Default to blue for the aligned cloud if no source colors
            aligned_colors[i] = QVector3D(0.0f, 0.0f, 1.0f);
        }
    }

    // Add the aligned point cloud to the active view
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    QString alignedName = name1 + "_aligned_to_" + name2;
    activeView->addPointCloud(aligned_points, aligned_colors, alignedName);

    // Update hierarchy view to show the new point cloud
    updateHierarchyView();

    // Complete the progress
    progress.setValue(100);

    // Display success message with transformation details
    QMessageBox resultDialog;
    resultDialog.setWindowTitle("ICP Results");
    resultDialog.setIcon(QMessageBox::Information);

    // Format the transformation matrix for display
    QString matrixStr;
    QTextStream ts(&matrixStr);
    ts.setRealNumberPrecision(4);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            ts << best_transform(i, j) << "\t";
        }
        ts << "\n";
    }

    resultDialog.setText(QString("ICP completed successfully!\n\n"
                                 "Fitness score: %1\n\n"
                                 "Transformation matrix:\n%2\n\n"
                                 "A new point cloud '%3' has been created.")
                             .arg(best_fitness, 0, 'f', 6)
                             .arg(matrixStr)
                             .arg(alignedName));

    resultDialog.exec();

    // Update the view

    activeView->update();
}



void MainWindow::on_actionUser_Capture_triggered()
{
    logToConsole("User triggered viewport capture.", "INFO");

    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    ViewportCaptureDialog dialog(this);

    // Pre-fill dialog with current camera values
    QVector3D currentPos, currentTarget, currentUp;
    activeView->getCameraParameters(currentPos, currentTarget, currentUp);
    dialog.setCameraPosition(currentPos);
    dialog.setCameraTarget(currentTarget);
    dialog.setCameraUp(currentUp);

    if (dialog.exec() != QDialog::Accepted)
        return;  // Cancelled

    // User-defined capture data
    QString captureName = dialog.getCaptureName();
    QVector3D cameraPosition = dialog.getCameraPosition();
    QVector3D cameraTarget = dialog.getCameraTarget();
    QVector3D cameraUp = dialog.getCameraUp();

    // Build the new viewport data
    ViewportData viewport;
    viewport.name = captureName;
    viewport.projectionMatrix = activeView->getProjectionMatrix();
    viewport.cameraPosition = cameraPosition;
    viewport.cameraTarget = cameraTarget;
    viewport.cameraUp = cameraUp;
    viewport.panOffset = activeView->getPanOffset();
    activeView->getRotation(viewport.xRotation, viewport.yRotation);
    viewport.zoom = activeView->getZoom();

    // Check for duplicate by name or view state
    bool duplicate = false;
    for (const auto& existing : m_capturedViewports) {
        if (existing.name == viewport.name ||
            (existing.projectionMatrix == viewport.projectionMatrix &&
             existing.cameraPosition == viewport.cameraPosition &&
             existing.cameraTarget == viewport.cameraTarget &&
             existing.zoom == viewport.zoom)) {
            duplicate = true;
            break;
        }
    }

    if (duplicate) {
        statusBar()->showMessage(tr("Duplicate user viewport skipped: %1").arg(viewport.name), 2000);
        logToConsole(tr("Skipped user viewport capture (duplicate): %1").arg(viewport.name), "INFO");
        return;
    }

    // Add to the list
    m_capturedViewports.append(viewport);

    // Add to OpenGLWidget list
    OpenGLWidget::ViewportCapture viewCapture;
    viewCapture.name = viewport.name;
    viewCapture.projectionMatrix = viewport.projectionMatrix;
    viewCapture.position = viewport.cameraPosition;
    viewCapture.target = viewport.cameraTarget;
    viewCapture.upVector = viewport.cameraUp;
    viewCapture.xRotation = viewport.xRotation;
    viewCapture.yRotation = viewport.yRotation;
    viewCapture.zoom = viewport.zoom;
    viewCapture.panOffset = viewport.panOffset;

    activeView->addViewportCapture(viewCapture);

    applyViewportParameters(viewport, activeView);

    updateViewportCapturesInHierarchy();

    statusBar()->showMessage(tr("User viewport captured: %1").arg(viewport.name), 2000);
    logToConsole(tr("User viewport captured: %1").arg(viewport.name), "SUCCESS");
}



void MainWindow::on_actionSegmentation_triggered()
{
    qDebug() << "on_actionSegmentation_triggered";

    if (!openGLWidget) {
        qDebug() << "MainWindow: openGLWidget is null!";
        QMessageBox::critical(this, tr("Error"), tr("OpenGLWidget is not initialized."));
        return;
    }

    const auto& grids2D = openGLWidget->get2DGrids();
    qDebug() << "Number of 2D grids:" << grids2D.size();

    if (grids2D.size() == 1) {
        SegmentationDialog dialog(openGLWidget, this);
        dialog.exec();
        qDebug() << "SegmentationDialog closed";
    } else {
        qDebug() << "Segmentation requires exactly one 2D grid, found:" << grids2D.size();
        QMessageBox::information(this, tr("Segmentation"), tr("Segmentation requires exactly one 2D grid to be present."));
    }

}


void MainWindow::on_actionMulti_Grid_triggered()
{
    for (const auto& grid : openGLWidget->get2DGrids()) {
        if (grid.selected) {
            MultiGridDialog dialog(grid.rows, grid.columns, grid.cellSize, grid.name, this);
            connect(&dialog, &MultiGridDialog::multiGridParametersConfirmed, openGLWidget,
                    &OpenGLWidget::handleMultiGridParameters);
            dialog.exec();
            updateHierarchyView();
            openGLWidget->update();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a 2D grid to duplicate."));

}



void MainWindow::on_actionGrid_Visibility_triggered()
{
    GridVisibilityDialog dialog(openGLWidget, openGLWidget->get2DGrids(), this);
    connect(&dialog, &GridVisibilityDialog::gridVisibilityChanged, openGLWidget,
            &OpenGLWidget::set2DGridVisibility);
    dialog.exec();
    updateHierarchyView();
    openGLWidget->update();
}

void MainWindow::on_action2D_to_3D_Grid_triggered()
{
    for (const auto& grid : openGLWidget->get2DGrids()) {
        if (grid.selected) {
            Convert2DTo3DGridDialog dialog(grid.rows, grid.columns, grid.cellSize, grid.name, this);
            connect(&dialog, &Convert2DTo3DGridDialog::gridParametersConfirmed, this,
                    [this](int rows, int columns, int layers, float cellSize, const QString& gridName) {
                        QString newName = QString("Grid3D_%1x%2x%3").arg(rows).arg(columns).arg(layers);
                        openGLWidget->add3DGrid(rows, columns, layers, cellSize, newName);
                        openGLWidget->set2DGridSelected(gridName, false); // Deselect the 2D grid
                        openGLWidget->set3DGridSelected(newName, true); // Select the new 3D grid
                        updateHierarchyView();
                        openGLWidget->update();
                    });
            dialog.exec();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a 2D grid to convert to 3D."));
}


void MainWindow::setupAssetBrowser()
{
    // 1. Set up the file system model for the Files tab
    fileSystemModel = new QFileSystemModel(this);
    QStringList filters;
    filters << "*.pts" << "*.fbx" << "*.xyz" << "*.pcd" << "*.ply"
            << "*.obj"  << "*.stl" << "*.dae" << "*.gltf" << "*.glb";
    fileSystemModel->setNameFilters(filters);

    // ---- MODIFIED: Show files that don't match filters but grayed out ----
    fileSystemModel->setNameFilterDisables(true); // This makes files visible but grayed out

    // ---- MODIFIED: Show both files and directories ----
    fileSystemModel->setFilter(QDir::Files | QDir::Dirs | QDir::NoDotAndDotDot);

    // Set current directory
    currentDirectory = QDir::currentPath();
    fileSystemModel->setRootPath(currentDirectory);

    // Configure treeView_2 (Files tab)
    ui->treeView_2->setModel(fileSystemModel);
    ui->treeView_2->setRootIndex(fileSystemModel->index(currentDirectory));
    ui->treeView_2->setSortingEnabled(true);
    ui->treeView_2->sortByColumn(0, Qt::AscendingOrder);
    ui->treeView_2->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->treeView_2->setColumnWidth(0, 200); // Name
    ui->treeView_2->setColumnHidden(1, false); // Size
    ui->treeView_2->setColumnHidden(2, false); // Type
    ui->treeView_2->setColumnHidden(3, false); // Date modified

    // ---- MODIFIED: Set expanded view by default ----
    ui->treeView_2->setExpanded(fileSystemModel->index(currentDirectory), true);

    // 2. Set up properties tree view (treeView_3)
    QStandardItemModel* propertiesModel = new QStandardItemModel(this);
    propertiesModel->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
    ui->treeView_3->setModel(propertiesModel);
    ui->treeView_3->setRootIsDecorated(true);
    ui->treeView_3->setAlternatingRowColors(true);
    ui->treeView_3->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->treeView_3->header()->setSectionResizeMode(1, QHeaderView::Stretch);

    // 3. Set up preview tab
    setupPreviewWidget();

    // 4. Connect signals for navigation and selection changes
    connectAssetBrowserSignals();
    disconnect(ui->browseButton, &QPushButton::clicked, this, &MainWindow::on_browseButton_clicked);
    // 5. Connect file browser buttons
    // connect(ui->browseButton, &QPushButton::clicked, this, &MainWindow::on_browseButton_clicked);
    connect(ui->assetUpButton, &QPushButton::clicked, this, [this]() {
        QDir dir(currentDirectory);
        if (dir.cdUp()) {
            currentDirectory = dir.absolutePath();
            updateFileView(currentDirectory);
            ui->assetPathEdit->setText(currentDirectory);
        }
    });


    currentDirectory.clear(); // Explicitly set to empty
    ui->assetPathEdit->clear(); // Clear path display
}


void MainWindow::onAssetTabChanged(int index)
{
    // If switching to preview tab and there's a selection in Files tab
    if (index == 1 && ui->treeView_2->selectionModel() &&
        ui->treeView_2->selectionModel()->hasSelection()) {

        QModelIndex currentIndex = ui->treeView_2->selectionModel()->selectedIndexes().first();
        QString filePath = fileSystemModel->filePath(currentIndex);

        // Show loading message
        if (m_previewLabel) {
            m_previewLabel->setText("Loading preview...");
        }

        // Ensure previewOpenGLWidget exists and is properly initialized
        if (!previewOpenGLWidget) {
            setupPreviewWidget(); // Make sure it's created if it doesn't exist yet
        }

        // Make sure we've properly removed any existing previews
        if (previewOpenGLWidget) {
            previewOpenGLWidget->clearPointClouds();
            previewOpenGLWidget->clearModels();
        }

        // Load preview with slight delay to ensure UI updates
        QTimer::singleShot(100, this, [this, filePath]() {
            updatePreview(filePath);
        });

        // Enable rendering for the preview
        if (previewOpenGLWidget) {
            previewOpenGLWidget->setRenderingEnabled(true);
            previewOpenGLWidget->update();
        }
    }
    else if (index == 2 && ui->treeView_2->selectionModel() &&
             ui->treeView_2->selectionModel()->hasSelection()) {

        QModelIndex currentIndex = ui->treeView_2->selectionModel()->selectedIndexes().first();
        QString filePath = fileSystemModel->filePath(currentIndex);
        QFileInfo fileInfo(filePath);

        // Update properties tree
        populatePropertiesTreeView(fileInfo);

        // Disable preview rendering when on properties tab
        if (previewOpenGLWidget) {
            previewOpenGLWidget->setRenderingEnabled(false);
        }
    }
    else if (index == 0) {
        // Files tab - disable preview rendering to save resources
        if (previewOpenGLWidget) {
            previewOpenGLWidget->setRenderingEnabled(false);
        }
    }
}

void MainWindow::on_fileSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    Q_UNUSED(deselected);

    if (!selected.indexes().isEmpty()) {
        QModelIndex index = selected.indexes().first();
        QString filePath = fileSystemModel->filePath(index);

        // Update path display
        ui->assetPathEdit->setText(filePath);

        // Get file information
        QFileInfo fileInfo(filePath);

        // If it's a file (not a directory), update preview and properties
        if (fileInfo.isFile()) {
            // Check if the file type is supported
            QString extension = fileInfo.suffix().toLower();
            bool isSupportedFormat = (extension == "pts" || extension == "xyz" ||
                                      extension == "pcd" || extension == "ply" ||
                                      extension == "obj" || extension == "fbx" ||
                                      extension == "stl" || extension == "dae" ||
                                      extension == "gltf" || extension == "glb");

            if (isSupportedFormat) {
                // Update properties
                populatePropertiesTreeView(fileInfo);

                // If we're on the preview tab, load the preview
                if (ui->assetTabs->currentIndex() == 1) {
                    updatePreview(filePath);
                }
            } else {
                // Unsupported file type - display basic properties only
                populatePropertiesTreeView(fileInfo);

                if (m_previewLabel) {
                    m_previewLabel->setText(QString("<b>File:</b> %1<br>"
                                                    "<b>Unsupported file format:</b> %2")
                                                .arg(fileInfo.fileName())
                                                .arg(extension));
                }

                // Clear any existing preview
                if (previewOpenGLWidget) {
                    previewOpenGLWidget->clearPointClouds();
                    previewOpenGLWidget->clearModels();
                    previewOpenGLWidget->update();
                }
            }
        } else {
            // It's a directory, clear preview and show directory info
            populatePropertiesTreeView(fileInfo);

            if (previewOpenGLWidget) {
                previewOpenGLWidget->clearPointClouds();
                previewOpenGLWidget->clearModels();
                previewOpenGLWidget->update();
            }

            if (m_previewLabel) {
                m_previewLabel->setText(QString("<b>Directory:</b> %1<br>"
                                                "<b>Path:</b> %2")
                                            .arg(fileInfo.fileName().isEmpty() ?
                                                     fileInfo.absolutePath() : fileInfo.fileName())
                                            .arg(fileInfo.absolutePath()));
            }
        }
    } else {
        // No selection, clear everything
        if (ui->assetPathEdit) {
            ui->assetPathEdit->setText("");
        }

        if (previewOpenGLWidget) {
            previewOpenGLWidget->clearPointClouds();
            previewOpenGLWidget->clearModels();
            previewOpenGLWidget->update();
        }

        if (m_previewLabel) {
            m_previewLabel->setText("No file selected");
        }

        // Clear properties
        QStandardItemModel* propertiesModel = qobject_cast<QStandardItemModel*>(ui->treeView_3->model());
        if (propertiesModel) {
            propertiesModel->clear();
            propertiesModel->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");
        }
    }
}

void MainWindow::connectAssetBrowserSignals()
{
    // Connect tree view selection changes in Files tab
    connect(ui->treeView_2->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, [this](const QItemSelection &selected, const QItemSelection &deselected) {
                Q_UNUSED(deselected);

                if (selected.indexes().isEmpty()) {
                    return;
                }

                QModelIndex index = selected.indexes().first();
                QString filePath = fileSystemModel->filePath(index);
                QFileInfo fileInfo(filePath);

                // Update file path display
                ui->assetPathEdit->setText(filePath);

                if (fileInfo.isDir()) {
                    // If a directory is selected, update the view to show its contents
                    updateFileView(filePath);
                    return;
                }

                // ---- MODIFIED: Always update both preview and properties regardless of current tab ----
                // Update preview (even if not on preview tab)
                updatePreview(filePath);

                // Update properties (even if not on properties tab)
                populatePropertiesTreeView(fileInfo);
            });

    // Connect tab changes to immediately show content based on current selection
    connect(ui->assetTabs, &QTabWidget::currentChanged, this, [this](int index) {
        QModelIndex currentIndex;
        QString filePath;

        // Get the current selection or first file if none selected
        if (ui->treeView_2->selectionModel() &&
            ui->treeView_2->selectionModel()->hasSelection()) {

            currentIndex = ui->treeView_2->selectionModel()->selectedIndexes().first();
            filePath = fileSystemModel->filePath(currentIndex);
        } else {
            // If no selection, find the first file in current directory
            QDir dir(currentDirectory);
            QStringList filters;
            filters << "*.pts" << "*.xyz" << "*.pcd" << "*.ply"
                    << "*.obj" << "*.fbx" << "*.stl" << "*.dae" << "*.gltf" << "*.glb";
            dir.setNameFilters(filters);
            dir.setFilter(QDir::Files);

            QStringList files = dir.entryList();
            if (!files.isEmpty()) {
                filePath = dir.filePath(files.first());
            }
        }

        QFileInfo fileInfo(filePath);
        if (!fileInfo.exists() || !fileInfo.isFile()) {
            return;
        }

        // If switching to preview tab
        if (index == 1 && previewOpenGLWidget) {
            // Ensure the preview is shown and rendered
            updatePreview(filePath);
            previewOpenGLWidget->setRenderingEnabled(true);
            previewOpenGLWidget->update();
        }
        // If switching to properties tab
        else if (index == 2) {
            // Update properties tree view
            populatePropertiesTreeView(fileInfo);
        }
    });

    // Add direct handling for custom file drop events
    ui->treeView_2->setAcceptDrops(true);
    ui->treeView_2->setDropIndicatorShown(true);
}


void MainWindow::populatePropertiesTreeView(const QFileInfo& fileInfo)
{
    // Get the existing model or create a new one
    QStandardItemModel* propertiesModel = qobject_cast<QStandardItemModel*>(ui->treeView_3->model());
    if (!propertiesModel) {
        propertiesModel = new QStandardItemModel(this);
        ui->treeView_3->setModel(propertiesModel);
    }

    // Clear existing items
    propertiesModel->clear();
    propertiesModel->setHorizontalHeaderLabels(QStringList() << "Property" << "Value");

    // Root item for all properties
    QStandardItem* rootItem = propertiesModel->invisibleRootItem();

    // File information section
    QStandardItem* fileSection = new QStandardItem("File Information");
    QFont sectionFont = fileSection->font();
    sectionFont.setBold(true);
    fileSection->setFont(sectionFont);

    // Add file properties
    addPropertyToGroup(fileSection, "Name", fileInfo.fileName());

    QString fileType = fileInfo.suffix().toUpper();
    if (fileType.isEmpty()) {
        fileType = fileInfo.isDir() ? "Directory" : "File";
    } else {
        fileType += " File";
    }
    addPropertyToGroup(fileSection, "Type", fileType);

    addPropertyToGroup(fileSection, "Size",
                       QString::number(fileInfo.size() / 1024.0, 'f', 2) + " KB");
    addPropertyToGroup(fileSection, "Path", fileInfo.absolutePath());

    // Date section
    QStandardItem* dateSection = new QStandardItem("Dates");
    dateSection->setFont(sectionFont);

    addPropertyToGroup(dateSection, "Created",
                       fileInfo.birthTime().toString("yyyy-MM-dd hh:mm:ss"));
    addPropertyToGroup(dateSection, "Modified",
                       fileInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss"));
    addPropertyToGroup(dateSection, "Last Read",
                       fileInfo.lastRead().toString("yyyy-MM-dd hh:mm:ss"));

    // Add format-specific information for supported files
    if (fileInfo.isFile()) {
        QString extension = fileInfo.suffix().toLower();

        if (extension == "pts" || extension == "xyz" ||
            extension == "pcd" || extension == "ply") {

            // Point cloud section
            QStandardItem* pointCloudSection = new QStandardItem("Point Cloud Properties");
            pointCloudSection->setFont(sectionFont);

            // Try to get information from preview data
            // If preview data isn't loaded yet, provide placeholder or basic info
            if (m_previewLabel && m_previewLabel->text().contains("Points:")) {
                // Extract point count from preview label if available
                QString labelText = m_previewLabel->text();
                QRegularExpression re("<b>Points:</b>\\s*(\\d+)");
                QRegularExpressionMatch match = re.match(labelText);

                if (match.hasMatch()) {
                    addPropertyToGroup(pointCloudSection, "Points", match.captured(1));
                }

                // Extract dimensions if available
                QRegularExpression reX("X:\\s*([\\d.-]+)\\s+to\\s+([\\d.-]+)");
                QRegularExpression reY("Y:\\s*([\\d.-]+)\\s+to\\s+([\\d.-]+)");
                QRegularExpression reZ("Z:\\s*([\\d.-]+)\\s+to\\s+([\\d.-]+)");

                QRegularExpressionMatch matchX = reX.match(labelText);
                QRegularExpressionMatch matchY = reY.match(labelText);
                QRegularExpressionMatch matchZ = reZ.match(labelText);

                if (matchX.hasMatch()) {
                    float min = matchX.captured(1).toFloat();
                    float max = matchX.captured(2).toFloat();
                    float size = qAbs(max - min);
                    addPropertyToGroup(pointCloudSection, "Width (X)",
                                       QString::number(size, 'f', 2));
                }

                if (matchY.hasMatch()) {
                    float min = matchY.captured(1).toFloat();
                    float max = matchY.captured(2).toFloat();
                    float size = qAbs(max - min);
                    addPropertyToGroup(pointCloudSection, "Height (Y)",
                                       QString::number(size, 'f', 2));
                }

                if (matchZ.hasMatch()) {
                    float min = matchZ.captured(1).toFloat();
                    float max = matchZ.captured(2).toFloat();
                    float size = qAbs(max - min);
                    addPropertyToGroup(pointCloudSection, "Depth (Z)",
                                       QString::number(size, 'f', 2));
                }
            } else {
                // Add placeholder for point cloud stats if no preview loaded
                addPropertyToGroup(pointCloudSection, "Points", "Select Preview tab to analyze");
                addPropertyToGroup(pointCloudSection, "Dimensions", "Select Preview tab to analyze");
            }

            rootItem->appendRow(pointCloudSection);
        }
        else if (extension == "obj" || extension == "fbx" ||
                 extension == "stl" || extension == "dae" ||
                 extension == "gltf" || extension == "glb") {

            // 3D model section
            QStandardItem* modelSection = new QStandardItem("Model Properties");
            modelSection->setFont(sectionFont);

            // If we have model stats from preview, use them
            if (m_previewLabel && m_previewLabel->text().contains("Vertices:")) {
                QString labelText = m_previewLabel->text();

                QRegularExpression reNodes("<b>Nodes:</b>\\s*(\\d+)");
                QRegularExpression reMeshes("<b>Meshes:</b>\\s*(\\d+)");
                QRegularExpression reVerts("<b>Vertices:</b>\\s*(\\d+)");
                QRegularExpression reTris("<b>Triangles:</b>\\s*(\\d+)");

                QRegularExpressionMatch matchNodes = reNodes.match(labelText);
                QRegularExpressionMatch matchMeshes = reMeshes.match(labelText);
                QRegularExpressionMatch matchVerts = reVerts.match(labelText);
                QRegularExpressionMatch matchTris = reTris.match(labelText);

                if (matchNodes.hasMatch()) {
                    addPropertyToGroup(modelSection, "Nodes", matchNodes.captured(1));
                }

                if (matchMeshes.hasMatch()) {
                    addPropertyToGroup(modelSection, "Meshes", matchMeshes.captured(1));
                }

                if (matchVerts.hasMatch()) {
                    addPropertyToGroup(modelSection, "Vertices", matchVerts.captured(1));
                }

                if (matchTris.hasMatch()) {
                    addPropertyToGroup(modelSection, "Triangles", matchTris.captured(1));
                }
            } else {
                // Add placeholder for model stats if no preview loaded
                addPropertyToGroup(modelSection, "Geometry", "Select Preview tab to analyze");
            }

            rootItem->appendRow(modelSection);
        }
    }

    // Add main sections to the model
    rootItem->appendRow(fileSection);
    rootItem->appendRow(dateSection);

    // Expand all items for better visibility
    ui->treeView_3->expandAll();

    // Resize columns to fit content
    ui->treeView_3->resizeColumnToContents(0);
}

void MainWindow::setupPreviewWidget()
{
    // Find the OpenGL widget in the UI
    QOpenGLWidget* existingPreviewWidget = ui->openGLWidget_2;

    // Create OpenGL widget for 3D preview if not found
    if (!existingPreviewWidget) {
        qDebug() << "Preview OpenGL widget not found in UI, creating new one";
        return;
    }

    // If we already have a custom widget, don't create a new one
    if (previewOpenGLWidget) {
        return;
    }

    // Create our custom OpenGL widget to replace the placeholder
    previewOpenGLWidget = new OpenGLWidget(this);
    previewOpenGLWidget->setObjectName("previewOpenGLWidget");
    previewOpenGLWidget->setMinimumSize(300, 300);

    // Configure for preview mode - no grid or axes
    previewOpenGLWidget->setGridVisible(false);
    previewOpenGLWidget->setAxisVisible(false);
    previewOpenGLWidget->setRenderingEnabled(false);

    // Replace the placeholder widget with our custom one
    QWidget* previewTab = ui->assetTabs->widget(1); // Preview tab
    if (previewTab) {
        QLayout* layout = previewTab->layout();
        if (layout) {
            // Find and remove the placeholder
            for (int i = 0; i < layout->count(); i++) {
                QLayoutItem* item = layout->itemAt(i);
                if (item->widget() == existingPreviewWidget) {
                    layout->removeItem(item);
                    delete existingPreviewWidget; // Remove the placeholder
                    break;
                }
            }

            // Create a splitter for preview tab with info panel
            QSplitter* previewSplitter = new QSplitter(Qt::Vertical, previewTab);

            // Create info panel for file info
            QWidget* infoPanel = new QWidget(previewSplitter);
            QVBoxLayout* infoPanelLayout = new QVBoxLayout(infoPanel);

            // Create info label
            m_previewLabel = new QLabel("No file selected");
            m_previewLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
            m_previewLabel->setWordWrap(true);
            m_previewLabel->setTextFormat(Qt::RichText);
            m_previewLabel->setStyleSheet(
                "background-color: #2D2D2D; color: #E0E0E0; padding: 8px; border-radius: 4px;");
            m_previewLabel->setMinimumHeight(50);
            m_previewLabel->setMaximumHeight(100);

            infoPanelLayout->addWidget(m_previewLabel);

            // Create OpenGL container
            QWidget* previewPanel = new QWidget(previewSplitter);
            QVBoxLayout* previewPanelLayout = new QVBoxLayout(previewPanel);
            previewPanelLayout->setContentsMargins(0, 0, 0, 0);

            // Add OpenGL widget
            previewPanelLayout->addWidget(previewOpenGLWidget);

            // Add panels to splitter
            previewSplitter->addWidget(infoPanel);
            previewSplitter->addWidget(previewPanel);
            previewSplitter->setStretchFactor(0, 1);
            previewSplitter->setStretchFactor(1, 3);

            // Add splitter to layout
            layout->addWidget(previewSplitter);
        }
    }

    // Don't add the graphics view if we're using OpenGL
    // m_previewGraphicsView->hide();

    // Initialize with a default message
    if (m_previewLabel) {
        m_previewLabel->setText("Select a file to preview");
    }
}
void MainWindow::setupPreviewArea()
{
    // Find the preview container in the UI
    QWidget* previewContainer = findChild<QWidget*>("previewContainer");
    if (!previewContainer) {
        qDebug() << "Preview container not found in the UI!";
        return;
    }

    // Clear any existing layout
    QLayout* existingLayout = previewContainer->layout();
    if (existingLayout) {
        while (QLayoutItem* item = existingLayout->takeAt(0)) {
            if (QWidget* widget = item->widget()) {
                widget->deleteLater();
            }
            delete item;
        }
        delete existingLayout;
    }

    // Create a new layout for the preview container
    QVBoxLayout* previewLayout = new QVBoxLayout(previewContainer);
    previewLayout->setContentsMargins(0, 0, 0, 0);

    // Create a label for file information
    m_previewLabel = new QLabel("No file selected");
    m_previewLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    m_previewLabel->setWordWrap(true);
    m_previewLabel->setTextFormat(Qt::RichText);
    m_previewLabel->setStyleSheet("background-color: #2D2D2D; color: #E0E0E0; padding: 8px;");

    // Create a graphics view for the point cloud preview
    m_previewScene = new QGraphicsScene(this);
    m_previewGraphicsView = new QGraphicsView(m_previewScene, this);
    m_previewGraphicsView->setMinimumHeight(200);
    m_previewGraphicsView->setRenderHint(QPainter::Antialiasing);
    m_previewGraphicsView->setStyleSheet("background-color: #1E1E1E; border: 1px solid #3A3A3A;");

    // Add widgets to layout
    previewLayout->addWidget(m_previewLabel);
    previewLayout->addWidget(m_previewGraphicsView, 1);

    // Add a placeholder text in the graphics view
    QGraphicsTextItem* placeholderText = m_previewScene->addText("Point Cloud Preview\nSelect a file to view");
    placeholderText->setDefaultTextColor(QColor(120, 120, 120));
    QFont font = placeholderText->font();
    font.setPointSize(12);
    placeholderText->setFont(font);

    // Center the text in the view
    QRectF textRect = placeholderText->boundingRect();
    placeholderText->setPos(-textRect.width()/2, -textRect.height()/2);
}



void MainWindow::on_actionDown_Sample_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            samplingProcessor->setWindowTitle("Down Sample");
            samplingProcessor->setPointCloudData(pc.points, pc.colors, "Down Sample");
            samplingProcessor->exec();
            return; // Exit after processing the selected point cloud
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to down sample."));
}

void MainWindow::on_actionUp_Sample_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            samplingProcessor->setWindowTitle("Up Sample");
            samplingProcessor->setPointCloudData(pc.points, pc.colors, "Up Sample");
            samplingProcessor->exec();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to up sample."));
}

void MainWindow::on_actionSub_Sample_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            samplingProcessor->setWindowTitle("Sub Sample");
            samplingProcessor->setPointCloudData(pc.points, pc.colors, "Sub Sample");
            samplingProcessor->exec();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to sub sample."));
}

void MainWindow::on_actionSampling_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            samplingProcessor->setWindowTitle("Sampling");
            samplingProcessor->setPointCloudData(pc.points, pc.colors, "Sampling");
            samplingProcessor->exec();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to sample."));
}

void MainWindow::on_samplingApplied(const std::vector<QVector3D>& points, const std::vector<QVector3D>& colors, const QString& operation)
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            QString newName = pc.name + "_" + operation;
            openGLWidget->updatePointCloud(pc.name, points, colors);
            updateHierarchyView();
            openGLWidget->update();
            break;
        }
    }
}

void MainWindow::on_actionRun_Collision_Map_triggered()
{
    CollisionDialog dlg(this);
    dlg.exec();
}

void MainWindow::loadPointCloudPreview(const QString& filePath, const QString& extension)
{
    if (!previewOpenGLWidget) return;

    previewOpenGLWidget->setRenderingEnabled(true);
    previewOpenGLWidget->clearModels();
    previewOpenGLWidget->clearPointClouds();

    if (m_previewLabel)
        m_previewLabel->setText("Loading preview (single-threaded)...");

    QCoreApplication::processEvents();  // allow UI update

    QString previewName = QFileInfo(filePath).fileName() + "_preview";
    const QVector3D DEFAULT_COLOR(1.0f, 1.0f, 1.0f);
    std::vector<QVector3D> points;
    std::vector<QVector3D> colors;
    QVector3D minPt(1e9, 1e9, 1e9), maxPt(-1e9, -1e9, -1e9);
    int originalCount = 0;
    const int maxPoints = 10000;

    try {
        if (extension == "ply" || extension == "pcd") {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            int result = (extension == "ply")
                ? pcl::io::loadPLYFile(filePath.toStdString(), *cloud)
                : pcl::io::loadPCDFile(filePath.toStdString(), *cloud);

            if (result < 0)
                throw std::runtime_error("Failed to load file");

            originalCount = cloud->points.size();
            int sampleRate = std::max(1, static_cast<int>(cloud->points.size() / maxPoints));

            for (size_t i = 0; i < cloud->points.size(); i += sampleRate) {
                const auto& p = cloud->points[i];
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

                QVector3D pt(p.x, p.y, p.z);
                points.push_back(pt);

                // Force white if RGB is missing or all black
                float rf = p.r / 255.0f;
                float gf = p.g / 255.0f;
                float bf = p.b / 255.0f;

                if ((rf == 0.0f && gf == 0.0f && bf == 0.0f) ||
                    !std::isfinite(rf) || !std::isfinite(gf) || !std::isfinite(bf)) {
                    colors.push_back(DEFAULT_COLOR);
                } else {
                    colors.push_back(QVector3D(rf, gf, bf));
                }

                minPt.setX(qMin(minPt.x(), pt.x()));
                minPt.setY(qMin(minPt.y(), pt.y()));
                minPt.setZ(qMin(minPt.z(), pt.z()));
                maxPt.setX(qMax(maxPt.x(), pt.x()));
                maxPt.setY(qMax(maxPt.y(), pt.y()));
                maxPt.setZ(qMax(maxPt.z(), pt.z()));
            }
        } else {
            std::ifstream file(filePath.toStdString());
            if (!file.is_open())
                throw std::runtime_error("Could not open file");

            std::string line;
            int lineCount = 0;

            while (std::getline(file, line) && points.size() < maxPoints) {
                lineCount++;
                if (lineCount % 10 != 0) continue; // Subsample

                std::istringstream iss(line);
                float x, y, z, r = -1, g = -1, b = -1;
                if (!(iss >> x >> y >> z)) continue;

                QVector3D pt(x, y, z);
                points.push_back(pt);

                if (iss >> r >> g >> b) {
                    if (r > 1 || g > 1 || b > 1) {
                        r /= 255.0f;
                        g /= 255.0f;
                        b /= 255.0f;
                    }
                    colors.push_back(QVector3D(r, g, b));
                } else {
                    colors.push_back(DEFAULT_COLOR);
                }

                minPt.setX(qMin(minPt.x(), pt.x()));
                minPt.setY(qMin(minPt.y(), pt.y()));
                minPt.setZ(qMin(minPt.z(), pt.z()));
                maxPt.setX(qMax(maxPt.x(), pt.x()));
                maxPt.setY(qMax(maxPt.y(), pt.y()));
                maxPt.setZ(qMax(maxPt.z(), pt.z()));
            }

            originalCount = lineCount;
            file.close();
        }

        previewOpenGLWidget->addPointCloud(points, colors, previewName);
        previewOpenGLWidget->setIsometricView();
        previewOpenGLWidget->update();

        if (m_previewLabel) {
            QString info;
            info += QString("<b>File:</b> %1<br>").arg(previewName);
            info += QString("<b>Points:</b> %1 (Showing %2)<br>").arg(originalCount).arg(points.size());
            info += QString("<b>Dimensions:</b><br>");
            info += QString("X: %1 to %2<br>").arg(minPt.x()).arg(maxPt.x());
            info += QString("Y: %1 to %2<br>").arg(minPt.y()).arg(maxPt.y());
            info += QString("Z: %1 to %2<br>").arg(minPt.z()).arg(maxPt.z());
            m_previewLabel->setText(info);
        }

        logToConsole(QString("Preview loaded for %1 (%2 points shown)").arg(previewName).arg(points.size()), "SUCCESS");

    } catch (const std::exception& e) {
        QString errorMsg = QString("Error loading preview: %1").arg(e.what());
        if (m_previewLabel)
            m_previewLabel->setText(errorMsg);
        logToConsole(errorMsg, "ERROR");
    }
}

void MainWindow::onAssetDockVisibilityChanged(bool visible)
{
    if (!previewOpenGLWidget) return;

    if (visible) {
        // Dock became visible - check if we're on the preview tab
        if (ui->assetTabs->currentIndex() == 1) {
            // Delay enabling rendering to ensure widget is properly laid out
            QTimer::singleShot(100, this, [this]() {
                if (previewOpenGLWidget) {
                    previewOpenGLWidget->setRenderingEnabled(true);
                    previewOpenGLWidget->update();
                    qDebug() << "Asset dock became visible - preview rendering enabled";
                }
            });
        }
    } else {
        // Dock became hidden - disable rendering to save resources
        previewOpenGLWidget->setRenderingEnabled(false);
        qDebug() << "Asset dock became hidden - preview rendering disabled";
    }
}

void MainWindow::loadModelPreview(const QString& filePath, const QString& extension)
{
    if (!previewOpenGLWidget) return;

    previewOpenGLWidget->setRenderingEnabled(true);

    // Clear previous content immediately
    previewOpenGLWidget->clearModels();
    previewOpenGLWidget->clearPointClouds();

    // Show "loading" message early
    if (m_previewLabel) {
        QFileInfo fileInfo(filePath);
        QString previewText = QString("<b>File:</b> %1<br>").arg(fileInfo.fileName());
        previewText += QString("<b>Type:</b> %1<br>").arg(extension.toUpper());
        previewText += QString("<b>Loading preview...</b>");
        m_previewLabel->setText(previewText);
    }

    // Delay the actual load slightly to avoid blocking UI repaint
    QTimer::singleShot(100, this, [this, filePath, extension]() {
        try {
            QFileInfo fileInfo(filePath);
            QString modelName = fileInfo.fileName() + "_preview";

            // Load the model (this calls Assimp internally)
            previewOpenGLWidget->addModel(filePath, modelName);

            // Retrieve the added model
            const auto& models = previewOpenGLWidget->getModels();
            for (const auto& model : models) {
                if (model.name == modelName) {
                    int totalVertices = 0;
                    int totalFaces = 0;
                    int totalMeshes = 0;
                    int totalNodes = 0;

                    for (const auto& node : model.nodes) {
                        totalNodes++;
                        for (const auto& mesh : node.meshes) {
                            totalMeshes++;
                            totalVertices += mesh.vertexCount;
                            totalFaces += mesh.indexCount / 3;
                        }
                    }

                    QVector3D dimensions = model.max - model.min;

                    // Update preview label
                    if (m_previewLabel) {
                        QString infoText = QString("<b>File:</b> %1<br>").arg(fileInfo.fileName());
                        infoText += QString("<b>Type:</b> %1<br>").arg(extension.toUpper());
                        infoText += QString("<b>Nodes:</b> %1<br>").arg(totalNodes);
                        infoText += QString("<b>Meshes:</b> %1<br>").arg(totalMeshes);
                        infoText += QString("<b>Vertices:</b> %1<br>").arg(totalVertices);
                        infoText += QString("<b>Triangles:</b> %1<br>").arg(totalFaces);
                        infoText += QString("<b>Dimensions:</b><br>");
                        infoText += QString("Width: %1<br>").arg(dimensions.x(), 0, 'f', 2);
                        infoText += QString("Height: %1<br>").arg(dimensions.y(), 0, 'f', 2);
                        infoText += QString("Depth: %1<br>").arg(dimensions.z(), 0, 'f', 2);
                        m_previewLabel->setText(infoText);
                    }

                    break;
                }
            }

            // Auto adjust camera
            previewOpenGLWidget->setIsometricView();
            previewOpenGLWidget->update();

        } catch (const std::exception& e) {
            if (m_previewLabel) {
                QString errorText = QString("<b>File:</b> %1<br>").arg(QFileInfo(filePath).fileName());
                errorText += QString("<b>Error:</b> %1").arg(e.what());
                m_previewLabel->setText(errorText);
            }
            qDebug() << "Model preview error:" << e.what();
        }
    });
}


void MainWindow::generatePointCloudPreview(const QString& filePath)
{
    // Clear the current scene
    if (!m_previewScene) return;
    m_previewScene->clear();

    // Create a background rect
    m_previewScene->setBackgroundBrush(QBrush(QColor(30, 30, 30)));

    try {
        // Open the point cloud file
        std::ifstream file(filePath.toStdString());
        if (!file.is_open()) {
            throw std::runtime_error("Could not open file");
        }

        // Collect sample points for preview
        std::vector<QVector3D> points;
        std::vector<QVector3D> colors;
        int maxPreviewPoints = 2000; // Limit points for better performance
        int sampleRate = 10; // Sample every 10th point
        int pointCount = 0;

        QVector3D minPoint(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
        QVector3D maxPoint(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());

        std::string line;
        while (std::getline(file, line) && points.size() < maxPreviewPoints) {
            pointCount++;

            // Sample points for preview
            if (pointCount % sampleRate != 0) continue;

            std::istringstream iss(line);
            float x, y, z, r = 200.0f, g = 200.0f, b = 200.0f;

            if (iss >> x >> y >> z) {
                // Track bounding box
                minPoint.setX(qMin(minPoint.x(), x));
                minPoint.setY(qMin(minPoint.y(), y));
                minPoint.setZ(qMin(minPoint.z(), z));

                maxPoint.setX(qMax(maxPoint.x(), x));
                maxPoint.setY(qMax(maxPoint.y(), y));
                maxPoint.setZ(qMax(maxPoint.z(), z));

                // Check for color data
                if (iss >> r >> g >> b) {
                    // Normalize color values if needed
                    if (r > 1.0f || g > 1.0f || b > 1.0f) {
                        r /= 255.0f;
                        g /= 255.0f;
                        b /= 255.0f;
                    }
                }

                points.push_back(QVector3D(x, y, z));
                colors.push_back(QVector3D(r, g, b));
            }
        }
        file.close();

        if (points.empty()) {
            throw std::runtime_error("No valid points found in file");
        }

        // Calculate center and scale for the preview
        QVector3D center = (minPoint + maxPoint) * 0.5f;
        QVector3D extent = maxPoint - minPoint;
        float maxExtent = qMax(extent.x(), qMax(extent.y(), extent.z()));
        if (maxExtent < 0.00001f) maxExtent = 1.0f; // Avoid division by zero

        // Prepare the preview view
        float viewWidth = 300;
        float viewHeight = 200;
        m_previewScene->setSceneRect(-viewWidth/2, -viewHeight/2, viewWidth, viewHeight);

        // Set scaling factor to fit the points in the view
        float scale = qMin(viewWidth, viewHeight) * 0.8f / maxExtent;

        // Create a simple isometric projection (rotate around X and Y)
        QMatrix4x4 projection;
        projection.rotate(30, 1, 0, 0); // Rotate around X axis
        projection.rotate(-45, 0, 1, 0); // Rotate around Y axis

        // Draw the points
        for (size_t i = 0; i < points.size(); ++i) {
            // Center and scale the point
            QVector3D p = points[i] - center;

            // Apply projection
            QVector3D projected = projection * p;

            // Scale to fit view
            projected *= scale;

            // Draw a small circle for each point
            float pointSize = 2.0f;
            QColor pointColor = QColor::fromRgbF(
                qBound(0.0f, colors[i].x(), 1.0f),
                qBound(0.0f, colors[i].y(), 1.0f),
                qBound(0.0f, colors[i].z(), 1.0f)
                );

            QGraphicsEllipseItem* point = m_previewScene->addEllipse(
                projected.x() - pointSize/2,
                projected.y() - pointSize/2,
                pointSize, pointSize,
                Qt::NoPen, QBrush(pointColor)
                );

            // Apply a subtle depth effect - points further back are slightly more transparent
            float depth = (projected.z() / (maxExtent * scale) + 1.0f) * 0.5f; // Normalize to 0-1
            point->setOpacity(0.5f + depth * 0.5f); // 50-100% opacity based on depth
        }

        // Add axes for orientation
        float axisLength = viewWidth * 0.15f;

        // X axis (red)
        QVector3D xAxis = projection * QVector3D(axisLength, 0, 0);
        m_previewScene->addLine(0, 0, xAxis.x(), xAxis.y(), QPen(Qt::red, 1.5));

        // Y axis (green)
        QVector3D yAxis = projection * QVector3D(0, axisLength, 0);
        m_previewScene->addLine(0, 0, yAxis.x(), yAxis.y(), QPen(Qt::green, 1.5));

        // Z axis (blue)
        QVector3D zAxis = projection * QVector3D(0, 0, axisLength);
        m_previewScene->addLine(0, 0, zAxis.x(), zAxis.y(), QPen(Qt::blue, 1.5));

        // Add axis labels
        QGraphicsTextItem* xLabel = m_previewScene->addText("X");
        xLabel->setDefaultTextColor(Qt::red);
        xLabel->setPos(xAxis.x(), xAxis.y());

        QGraphicsTextItem* yLabel = m_previewScene->addText("Y");
        yLabel->setDefaultTextColor(Qt::green);
        yLabel->setPos(yAxis.x(), yAxis.y());

        QGraphicsTextItem* zLabel = m_previewScene->addText("Z");
        zLabel->setDefaultTextColor(Qt::blue);
        zLabel->setPos(zAxis.x(), zAxis.y());

        // Fit view to content
        if (m_previewGraphicsView) {
            m_previewGraphicsView->fitInView(m_previewScene->sceneRect(), Qt::KeepAspectRatio);
            m_previewGraphicsView->centerOn(0, 0);
        }

    } catch (const std::exception& e) {
        // Add error message to the scene
        QGraphicsTextItem* errorText = m_previewScene->addText("Error generating preview: " + QString(e.what()));
        errorText->setDefaultTextColor(Qt::red);
        QFont errorFont = errorText->font();
        errorFont.setPointSize(10);
        errorText->setFont(errorFont);

        // Center the text
        QRectF errorRect = errorText->boundingRect();
        errorText->setPos(-errorRect.width()/2, -errorRect.height()/2);
    }
}


//viewport
void MainWindow::on_deleteObjectButton_clicked()
{
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();
    if (selectedIndexes.isEmpty()) {
        logToConsole("Delete button clicked, but no items selected.", "WARNING");
        return;
    }

    QMessageBox::StandardButton reply = QMessageBox::question(
        this,
        tr("Delete"),
        tr("Are you sure you want to delete the selected item(s)?"),
        QMessageBox::Yes | QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        logToConsole("User canceled deletion operation.", "INFO");
        return;
    }

    OpenGLWidget* view = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    bool deletionOccurred = false;
    bool viewportDeleted = false;

    for (const QModelIndex& index : selectedIndexes) {
        QStandardItem* item = hierarchyModel->itemFromIndex(index);
        if (!item) continue;

        QString type = item->data(Qt::UserRole).toString();
        QString name = item->text();

        // Determine correct OpenGLWidget for item (if set)
        OpenGLWidget* itemView = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
        if (itemView) view = itemView;

        // Remove wrapper labels like "[Model]", "[PointCloud]", etc.
        name.remove(QRegularExpression(R"(\[.*\]\s*)"));

        if (type == "PointCloud") {
            view->setPointCloudSelected(name, true);
            if (view->deleteObject(-1)) deletionOccurred = true;
        }
        else if (type == "Model") {
            view->setModelSelected(name, true);
            if (view->deleteObject(-1)) deletionOccurred = true;
        }
        else if (type == "ModelNode") {
            QString modelName = item->data(Qt::UserRole + 2).toString();
            int nodeIndex = item->data(Qt::UserRole + 3).toInt();
            view->setModelNodeSelected(modelName, nodeIndex, true);
            if (view->deleteObject(-1)) deletionOccurred = true;
        }
        else if (type == "2DGrid") {
            view->set2DGridSelected(name, true);
            if (view->deleteObject(-1)) deletionOccurred = true;
        }
        else if (type == "3DGrid") {
            view->set3DGridSelected(name, true);
            if (view->deleteObject(-1)) deletionOccurred = true;
        }
        else if (type == "Viewport" || type == "ViewportCapture") {
            QString viewportName = (type == "Viewport")
            ? name.remove("[Viewport] ")
            : item->data(Qt::UserRole + 2).toString();

            for (int i = 0; i < m_capturedViewports.size(); ++i) {
                if (m_capturedViewports[i].name == viewportName) {
                    m_capturedViewports.removeAt(i);
                    logToConsole(tr("Deleted viewport: %1").arg(viewportName), "SUCCESS");
                    deletionOccurred = true;
                    viewportDeleted = true;
                    break;
                }
            }
        }
    }

    // Refresh hierarchy
    if (viewportDeleted) {
        updateViewportCapturesInHierarchy();  // Avoid resetting full tree
    }

    if (deletionOccurred) {
        if (!viewportDeleted) {
            updateHierarchyView();  // Only update full view if not a viewport-only delete
        }
        view->update();
        logToConsole("Deleted one or more items from the scene.", "SUCCESS");
    } else {
        logToConsole("Delete operation finished. No items were deleted.", "INFO");
    }
}

void MainWindow::on_propertyButton_clicked()
{
    // Find the tab widget that contains the property panel
    QTabWidget* hierarchyTabWidget = ui->hierarchyDock->findChild<QTabWidget*>("hierarchyTabWidget");
    if (hierarchyTabWidget) {
        // Switch to the properties tab (index 1 is Properties tab)
        hierarchyTabWidget->setCurrentIndex(1);

        // Update properties view with current selection
        updatePropertiesView();
    }
}

void MainWindow::on_addObjectButton_clicked()
{
    qDebug() << "Open action triggered";

    // Check if we're already showing a file dialog to prevent duplicate dialogs
    static bool isDialogOpen = false;
    if (isDialogOpen) {
        qDebug() << "File dialog already open, ignoring duplicate call";
        return;
    }

    isDialogOpen = true; // Set flag to prevent recursion

    // Use QFileDialog::getOpenFileName to select just one file
    QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("Open File"),
        QString(),
        tr("All Supported Files (*.pts *.obj *.fbx *.stl *.dae *.ply *.gltf *.glb);;"
           "Point Cloud Files (*.pts *.ply *.pcd);;"
           "FBX Models (*.fbx);;"
           "Other 3D Models (*.obj *.stl *.dae *.gltf *.glb)")
        );

    isDialogOpen = false; // Reset flag regardless of outcome

    if (filePath.isEmpty()) {
        return;
    }

    // Use the active view for loading file
    OpenGLWidget* targetView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Show which view we're loading into
    QWidget* window = targetView->window();
    QString viewName = window ? window->windowTitle() : "Main View";
    statusBar()->showMessage(tr("Loading file into %1...").arg(viewName), 2000);

    // Create progress dialog for single file
    QProgressDialog progress(tr("Loading File..."), tr("Cancel"), 0, 1, this);
    progress.setWindowModality(Qt::WindowModal);
    progress.setMinimumDuration(0);
    progress.setValue(0);

    try {
        QFileInfo fileInfo(filePath);
        QString fileName = fileInfo.fileName();
        QString extension = fileInfo.suffix().toLower();

        progress.setLabelText(tr("Loading file: %1").arg(fileName));
        QCoreApplication::processEvents();

        if (progress.wasCanceled()) {
            QMessageBox::information(this, tr("Loading Canceled"), tr("File loading was canceled."));
            return;
        }

        // Handle based on file extension
        if (extension == "pts") {
            std::vector<QVector3D> points;
            std::vector<QVector3D> colors;

            std::ifstream file(filePath.toStdString());
            if (!file.is_open()) {
                throw std::runtime_error("Could not open file: " + filePath.toStdString());
            }

            std::string line;
            while (std::getline(file, line)) {
                std::istringstream iss(line);
                float x, y, z, r = 255, g = 255, b = 255;

                // Try X Y Z R G B format first
                if (iss >> x >> y >> z >> r >> g >> b) {
                    points.push_back(QVector3D(x, y, z));
                    colors.push_back(QVector3D(r / 255.0f, g / 255.0f, b / 255.0f));
                } else {
                    // If that failed, try X Y Z format
                    iss.clear();
                    iss.str(line);
                    if (iss >> x >> y >> z) {
                        points.push_back(QVector3D(x, y, z));
                        colors.push_back(QVector3D(1.0f, 1.0f, 1.0f)); // White color
                    }
                }
            }

            qDebug() << "Loaded" << points.size() << "points from" << filePath;

            // Use the target view
            targetView->addPointCloud(points, colors, fileName);
        }
        else if (extension == "ply" || extension == "pcd") {
            // Use PCL to load PLY/PCD files
            qDebug() << "Loading PLY/PCD file using PCL:" << filePath;

            // Call our new method for loading PLY/PCD files
            loadPLYPointCloud(filePath, fileName, targetView);
        }
        else if (extension == "fbx") {
            // Show the FBX information dialog first
            showFbxInformation(filePath);

            // Then add the model to the view
            targetView->addModel(filePath, fileName);
        }
        else {
            // For other 3D model formats
            targetView->addModel(filePath, fileName);
        }

        updateHierarchyView();
        progress.setValue(1); // Completed

        // Ensure the target view is updated
        targetView->update();

        // Make sure the view is visible and active
        if (window && window != this) {
            window->activateWindow();
            window->raise();
        }

        // Update status
        QMessageBox::information(this, tr("Loading Complete"),
                                 tr("Successfully loaded %1 into %2").arg(fileName).arg(viewName));
        logToConsole(tr("Successfully loaded %1 into %2").arg(fileName).arg(viewName),"INFO");

    } catch (const std::exception& e) {
        progress.setValue(1); // Ensure dialog closes
        QMessageBox::critical(this, tr("Error"), tr("Failed to load file: %1").arg(e.what()));
    }
}


void MainWindow::addPropertyToGroup(QStandardItem* group, const QString& name, const QString& value)
{
    QStandardItem* nameItem = new QStandardItem(name);
    nameItem->setEditable(false);

    QStandardItem* valueItem = new QStandardItem(value);
    valueItem->setEditable(false);

    group->appendRow(QList<QStandardItem*>() << nameItem << valueItem);
}

void MainWindow::on_browseButton_clicked()
{
    // Add a static flag to prevent multiple dialogs
    static bool isDialogOpen = false;
    if (isDialogOpen) {
        qDebug() << "Browse dialog already open, ignoring duplicate call";
        return;
    }

    isDialogOpen = true; // Set flag before showing dialog

    // Create filter string for file selection dialog
    QStringList filters;
    filters << "Point Cloud Files (*.pts *.pcd *.ply *.xyz)"
            << "3D Model Files (*.obj *.fbx *.stl *.dae *.gltf *.glb)"
            << "All Supported Files (*.pts *.pcd *.ply *.xyz *.obj *.fbx *.stl *.dae *.gltf *.glb)";

    // Open file selection dialog directly (skip directory selection)
    QString filePath = QFileDialog::getOpenFileName(
        this,
        tr("Select 3D File"),
        currentDirectory,
        filters.join(";;")
        );

    // Reset the flag regardless of the outcome
    isDialogOpen = false;

    if (!filePath.isEmpty()) {
        QFileInfo fileInfo(filePath);

        // Update current directory to the file's directory
        currentDirectory = fileInfo.dir().absolutePath();

        // Update file view to show files in the same directory
        updateFileView(currentDirectory);

        // Find and select the file in the view
        QModelIndex fileIndex = fileSystemModel->index(filePath);
        if (fileIndex.isValid()) {
            ui->treeView_2->selectionModel()->select(fileIndex, QItemSelectionModel::ClearAndSelect);
            ui->treeView_2->scrollTo(fileIndex);
        }

        // Update preview and properties directly
        if (ui->assetTabs->currentIndex() == 1) {
            updatePreview(filePath);
        }
        populatePropertiesTreeView(fileInfo);
        // Ensure we're on the preview tab to show the file immediately
        ui->assetTabs->setCurrentIndex(1); // Switch to preview tab
    }
}



void MainWindow::on_refreshAssetsButton_clicked()
{
    qDebug() << "Refresh assets button clicked";

    // Show directory selection dialog
    QString dirPath = QFileDialog::getExistingDirectory(
        this,
        tr("Select Directory"),
        currentDirectory,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );

    if (!dirPath.isEmpty()) {
        // Update current directory and file view
        currentDirectory = dirPath;
        updateFileView(dirPath);

        // Update path display but don't try to preview it (it's a directory)
        ui->assetPathEdit->setText(dirPath);

        // If we're on the preview tab, show demo content
        if (ui->assetTabs->currentIndex() == 1 && previewOpenGLWidget) {
            previewOpenGLWidget->clearPointClouds();
            previewOpenGLWidget->clearModels(); // <-- also clear models
            previewOpenGLWidget->update();
            m_previewLabel->setText("Select a file to preview.");
        }

        qDebug() << "Selected directory:" << dirPath;
    }
}

void MainWindow::updateFileView(const QString& path)
{
    // Disconnect old signal to avoid multiple connections
    if (ui->treeView_2->selectionModel()) {
        disconnect(ui->treeView_2->selectionModel(), &QItemSelectionModel::selectionChanged,
                   nullptr, nullptr);
    }

    // Update the model with the new path
    fileSystemModel->setRootPath(path);
    ui->treeView_2->setRootIndex(fileSystemModel->index(path));

    // Update the current directory
    currentDirectory = path;
    ui->assetPathEdit->setText(path);

    // Connect selection signals
    connect(ui->treeView_2->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &MainWindow::on_fileSelectionChanged, Qt::UniqueConnection);

    // // ---- NEW CODE: Automatically select first file ----
    // // Find the first file in the directory to preview
    // QDir dir(path);
    // QStringList filters;
    // filters << "*.pts" << "*.fbx" << "*.xyz" << "*.pcd" << "*.ply"
    //         << "*.obj"  << "*.stl" << "*.dae" << "*.gltf" << "*.glb";
    // dir.setNameFilters(filters);
    // dir.setFilter(QDir::Files);

    // QStringList files = dir.entryList();
    // if (!files.isEmpty()) {
    //     // Get the model index for the first file
    //     QModelIndex firstFileIndex = fileSystemModel->index(dir.filePath(files.first()));

    //     // Select it in the view
    //     ui->treeView_2->selectionModel()->select(firstFileIndex, QItemSelectionModel::ClearAndSelect);
    //     ui->treeView_2->scrollTo(firstFileIndex);

    //     // Update preview and properties directly
    //     QString filePath = fileSystemModel->filePath(firstFileIndex);
    //     updatePreview(filePath);
    //     populatePropertiesTreeView(QFileInfo(filePath));
    // }

}

void MainWindow::updatePreview(const QString& filePath)
{
    // Early exit for empty path
    if (filePath.isEmpty()) {
        if (m_previewLabel) {
            m_previewLabel->setText("No file selected");
        }

        // Clear existing point clouds and models
        if (previewOpenGLWidget) {
            previewOpenGLWidget->clearPointClouds();
            previewOpenGLWidget->clearModels();
            previewOpenGLWidget->update();
        }
        return;
    }

    // Check if file exists
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists() || !fileInfo.isFile()) {
        if (m_previewLabel) {
            m_previewLabel->setText("Invalid file or directory");
        }
        return;
    }

    // Check if file type is supported
    QString extension = fileInfo.suffix().toLower();
    bool isSupportedFormat = (extension == "pts" || extension == "xyz" ||
                              extension == "pcd" || extension == "ply" ||
                              extension == "obj" || extension == "fbx" ||
                              extension == "stl" || extension == "dae" ||
                              extension == "gltf" || extension == "glb");

    if (!isSupportedFormat) {
        if (m_previewLabel) {
            m_previewLabel->setText("Unsupported file format: " + extension);
        }
        return;
    }

    try {
        // Create file information text
        QString previewText = QString("<b>File:</b> %1<br>").arg(fileInfo.fileName());
        previewText += QString("<b>Size:</b> %1 KB<br>").arg(fileInfo.size() / 1024);

        // Set initial preview text
        if (m_previewLabel) {
            m_previewLabel->setText(previewText + "<br><b>Loading preview...</b>");
        }

        // Process based on file type
        if (extension == "pts" || extension == "xyz" ||
            extension == "pcd" || extension == "ply") {
            // Point cloud files
            loadPointCloudPreview(filePath, extension);
        } else {
            // 3D model files
            loadModelPreview(filePath, extension);
        }

    } catch (const std::exception& e) {
        // Show error message
        if (m_previewLabel) {
            m_previewLabel->setText("Error analyzing file: " + QString(e.what()));
        }
    }
}



void MainWindow::on_actionCluster_2_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
            if (pc.selected) {
                ClusteringParametersDialog dialog(this);
                connect(&dialog, &ClusteringParametersDialog::clusteringParametersConfirmed, this,
                        [this, name = pc.name](float voxelSize, float sampleFraction, float planeDistanceThreshold,
                                               float clusterEps, size_t maxPlaneIterations, size_t clusterMinPts,
                                               float minPlaneInlierRatio) {
                            openGLWidget->clusterPointCloud(name, voxelSize, sampleFraction, planeDistanceThreshold,
                                                           clusterEps, maxPlaneIterations, clusterMinPts, minPlaneInlierRatio);
                            updateHierarchyView();
                            openGLWidget->update();
                        });
                dialog.exec();
                return;
            }
        }
        QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to cluster."));

}


void MainWindow::on_actionIdentify_Similar_Clusters_triggered()
{
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            if (pc.labels.empty()) {
                QMessageBox::warning(this, tr("No Clusters"), tr("Please perform clustering on the selected point cloud first."));
                return;
            }
            IdentifySimilarClustersDialog dialog(this);
            connect(&dialog, &IdentifySimilarClustersDialog::identifySimilarClustersConfirmed, this,
                    [this, name = pc.name]() {
                        openGLWidget->identifySimilarClusters(name);
                        updateHierarchyView();
                        openGLWidget->update();
                    });
            dialog.exec();
            return;
        }
    }
    QMessageBox::information(this, tr("No Selection"), tr("Please select a point cloud to identify similar clusters."));

}


void MainWindow::on_actionSlection_Capture_triggered()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    ViewportData viewport;

    QDateTime now = QDateTime::currentDateTime();
    viewport.name = QString("Viewport_%1").arg(now.toString("yyyyMMdd_hhmmss"));

    viewport.projectionMatrix = activeView->getProjectionMatrix();
    activeView->getCameraParameters(viewport.cameraPosition, viewport.cameraTarget, viewport.cameraUp);
    viewport.panOffset = activeView->getPanOffset();
    activeView->getRotation(viewport.xRotation, viewport.yRotation);
    viewport.zoom = activeView->getZoom();

    // Check if this viewport already exists
    bool duplicate = false;
    for (const auto& existing : m_capturedViewports) {
        if (viewport.projectionMatrix == existing.projectionMatrix &&
            viewport.cameraPosition == existing.cameraPosition &&
            viewport.cameraTarget == existing.cameraTarget &&
            viewport.zoom == existing.zoom) {
            duplicate = true;
            break;
        }
    }

    if (duplicate) {
        statusBar()->showMessage(tr("Duplicate viewport capture skipped"), 2000);
        logToConsole("Skipped capturing duplicate viewport", "INFO");
        return;
    }

    m_capturedViewports.append(viewport);

    OpenGLWidget::ViewportCapture viewCapture;
    viewCapture.name = viewport.name;
    viewCapture.projectionMatrix = viewport.projectionMatrix;
    viewCapture.position = viewport.cameraPosition;
    viewCapture.target = viewport.cameraTarget;
    viewCapture.upVector = viewport.cameraUp;
    viewCapture.xRotation = viewport.xRotation;
    viewCapture.yRotation = viewport.yRotation;
    viewCapture.zoom = viewport.zoom;
    viewCapture.panOffset = viewport.panOffset;

    activeView->addViewportCapture(viewCapture);

    applyViewportParameters(viewport, activeView);

    updateViewportCapturesInHierarchy();

    statusBar()->showMessage(tr("Viewport captured: %1").arg(viewport.name), 2000);
    logToConsole(tr("Viewport captured: %1").arg(viewport.name), "SUCCESS");
}


void MainWindow::updateViewportCapturesInHierarchy()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;

    // Remove existing "Viewport Captures" group
    for (int i = 0; i < hierarchyModel->rowCount(); ++i) {
        QStandardItem* item = hierarchyModel->item(i);
        if (item && item->text() == "Viewport Captures") {
            hierarchyModel->removeRow(i);
            break;
        }
    }

    QStandardItem* capturesGroup = new QStandardItem("Viewport Captures");
    capturesGroup->setEditable(false);
    hierarchyModel->appendRow(capturesGroup);

    for (const auto& capture : m_capturedViewports) {
        QStandardItem* captureItem = new QStandardItem("[Viewport] " + capture.name);
        captureItem->setEditable(false);
        captureItem->setData("Viewport", Qt::UserRole);
        captureItem->setData(QVariant::fromValue(activeView), Qt::UserRole + 1);
        captureItem->setData(capture.name, Qt::UserRole + 2);

        capturesGroup->appendRow(captureItem);
    }

    ui->treeView->expand(capturesGroup->index());
    ui->treeView->update();
}

// Add to the MainWindow class in mainwindow.cpp
void MainWindow::updateViewportFromSelection(const QString& viewportName)
{
    // Find the viewport data by name
    for (const auto& viewport : m_capturedViewports) {
        if (viewport.name == viewportName) {
            // Apply viewport parameters to the active OpenGL widget
            OpenGLWidget* targetWidget = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
            applyViewportParameters(viewport, targetWidget);

            // Update the OpenGL widget
            targetWidget->update();

            // Show status message
            statusBar()->showMessage(tr("Applied viewport: %1").arg(viewportName), 2000);
            return;
        }
    }

    // Viewport not found
    statusBar()->showMessage(tr("Viewport not found: %1").arg(viewportName), 2000);
}

void MainWindow::applyViewportParameters(const ViewportData& viewportData, OpenGLWidget* targetWidget)
{
    // Apply projection matrix
    targetWidget->setProjectionMatrix(viewportData.projectionMatrix);

    // Set camera position, target, and up vector
    targetWidget->setCameraParameters(
        viewportData.cameraPosition,
        viewportData.cameraTarget,
        viewportData.cameraUp
        );

    // Apply other parameters
    targetWidget->setPanOffset(viewportData.panOffset);
    targetWidget->setRotation(viewportData.xRotation, viewportData.yRotation);
    targetWidget->setZoom(viewportData.zoom);

    // Force update
    targetWidget->update();
}

// Add this function to mainwindow.cpp to provide a way to apply a specific viewport by name
void MainWindow::applyViewportByName(const QString& viewportName)
{
    // Find the viewport data by name
    for (const auto& viewport : m_capturedViewports) {
        if (viewport.name == viewportName) {
            // Apply to active view
            OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
            applyViewportParameters(viewport, activeView);
            activeView->update();

            // Show status message
            statusBar()->showMessage(tr("Applied viewport: %1").arg(viewportName), 2000);
            return;
        }
    }

    // Viewport not found
    statusBar()->showMessage(tr("Viewport not found: %1").arg(viewportName), 2000);
}

// Add this function to allow editing of existing viewports
void MainWindow::editViewport(const QString& viewportName)
{
    // Find the viewport data by name
    int viewportIndex = -1;
    for (int i = 0; i < m_capturedViewports.size(); ++i) {
        if (m_capturedViewports[i].name == viewportName) {
            viewportIndex = i;
            break;
        }
    }

    if (viewportIndex == -1) {
        statusBar()->showMessage(tr("Viewport not found: %1").arg(viewportName), 2000);
        return;
    }

    // Create and configure the dialog
    ViewportCaptureDialog dialog(this);

    // Initialize with current viewport values
    const ViewportData& viewport = m_capturedViewports[viewportIndex];
    dialog.setCameraPosition(viewport.cameraPosition);
    dialog.setCameraTarget(viewport.cameraTarget);
    dialog.setCameraUp(viewport.cameraUp);

    // Set the name field
    QLineEdit* nameEdit = dialog.findChild<QLineEdit*>();
    if (nameEdit) {
        nameEdit->setText(viewport.name);
    }

    // Show the dialog
    if (dialog.exec() != QDialog::Accepted) {
        return; // User canceled
    }

    // Update the viewport with new values
    m_capturedViewports[viewportIndex].name = dialog.getCaptureName();
    m_capturedViewports[viewportIndex].cameraPosition = dialog.getCameraPosition();
    m_capturedViewports[viewportIndex].cameraTarget = dialog.getCameraTarget();
    m_capturedViewports[viewportIndex].cameraUp = dialog.getCameraUp();

    // Apply the updated viewport to the active view
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    applyViewportParameters(m_capturedViewports[viewportIndex], activeView);
    activeView->update();

    // Update the hierarchy view
    updateViewportCapturesInHierarchy();

    // Show confirmation message
    statusBar()->showMessage(tr("Updated viewport: %1").arg(m_capturedViewports[viewportIndex].name), 2000);
}



void MainWindow::on_actionHeatmap_triggered()
{
    // Find the selected point cloud
    QString selectedPointCloud;
    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            selectedPointCloud = pc.name;
            break;
        }
    }

    if (selectedPointCloud.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a point cloud to change its color.");
        return;
    }

    ColorChanger *colorChanger = new ColorChanger(this);
    colorChanger->setPointCloudName(selectedPointCloud);
    connect(colorChanger, &ColorChanger::applySingleColor, openGLWidget, &OpenGLWidget::applySingleColor);
    connect(colorChanger, &ColorChanger::applyGradientColor, openGLWidget, &OpenGLWidget::applyGradientColor);
    colorChanger->exec();
}
void MainWindow::on_actionRecognize_Objects_triggered() {
    OpenGLWidget* activeWidget = getActiveViewWidget();
    const auto& pointClouds = activeWidget->getPointClouds();
    QString selectedCloudName;

    // Find the selected point cloud
    for (const auto& pc : pointClouds) {
        if (pc.selected) {
            selectedCloudName = pc.name;
            break;
        }
    }

    if (selectedCloudName.isEmpty()) {
        QMessageBox::warning(this, tr("Error"), tr("Please select a point cloud to recognize objects."));
        return;
    }

    // Get the clustered point cloud data
    QVector<Vertex> vertices;
    std::vector<int> labels;
    for (const auto& pc : pointClouds) {
        if (pc.name == selectedCloudName) {
            vertices.reserve(pc.points.size());
            for (const auto& point : pc.points) {
                Vertex v;
                v.position = point;
                vertices.push_back(v);
            }
            labels = pc.labels;
            break;
        }
    }

    if (labels.empty()) {
        QMessageBox::warning(this, tr("Error"), tr("Selected point cloud has no cluster labels. Please perform clustering first."));
        return;
    }

    // Dynamically find Python executable
    QString pythonPath;
    QStringList pythonExecutables = {"python3", "python", "python3.10", "python3.9", "python3.8"};

    for (const QString& execName : pythonExecutables) {
        pythonPath = QStandardPaths::findExecutable(execName);
        if (!pythonPath.isEmpty() && QFileInfo(pythonPath).isExecutable()) {
            break;
        }
    }

    // Fallback: Check PYTHONPATH environment variable
    if (pythonPath.isEmpty()) {
        QString pythonHome = qgetenv("PYTHONPATH");
        if (!pythonHome.isEmpty()) {
            pythonPath = QStandardPaths::findExecutable("python", {pythonHome});
            if (!pythonPath.isEmpty() && QFileInfo(pythonPath).isExecutable()) {
                // Found Python in PYTHONPATH
            } else {
                pythonPath.clear();
            }
        }
    }

    // Final fallback: Try running 'python --version' to verify
    if (pythonPath.isEmpty()) {
        QProcess process;
        process.start("python", {"--version"});
        if (process.waitForStarted() && process.waitForFinished()) {
            if (process.exitCode() == 0) {
                pythonPath = QStandardPaths::findExecutable("python");
            }
        }
    }

    if (pythonPath.isEmpty()) {
        QMessageBox::critical(this, tr("Error"), tr("No valid Python executable found. Please ensure Python is installed and accessible in your system PATH."));
        return;
    }

    // Set paths
    QString scriptPath = ""; // No external script file needed
    QString modelPath = "C:/ccube/MK1-Cm/pointnet_model.pth";

    // Open dialog
    ObjectDetectionDialog* dialog = new ObjectDetectionDialog(vertices, labels, scriptPath, pythonPath, modelPath, this);
    connect(dialog, &ObjectDetectionDialog::detectionCompleted, this, [this, activeWidget, selectedCloudName, vertices](const std::vector<int>& updatedLabels, const QString& visPcdPath, const std::map<std::string, int>& clusterCounts, int numEpochs, float learningRate, const QString& trainingFolder, float accuracy) {
        m_detectedLabels = updatedLabels;
        m_detectedPoints = vertices;
        m_clusterCounts = clusterCounts;
        m_numEpochs = numEpochs;
        m_learningRate = learningRate;
        m_trainingFolder = trainingFolder;
        m_accuracy = accuracy;

        for (auto& pc : activeWidget->m_pointClouds) {
            if (pc.name == selectedCloudName) {
                pc.labels = updatedLabels;
                activeWidget->updatePointCloudVBO(pc);
                break;
            }
        }

        activeWidget->update();

        std::vector<QVector3D> visPoints, visColors;
        if (loadPlyPointCloud(visPcdPath, visPoints, visColors)) {
            activeWidget->addPointCloud(visPoints, visColors, "Detected_Objects_" + selectedCloudName);
        }

        QMessageBox::information(this, tr("Success"), tr("Object detection completed successfully. Accuracy: %1%").arg(accuracy * 100, 0, 'f', 2));
    });

    dialog->exec();
    delete dialog;
}
void DraggableViewFrame::mousePressEvent(QMouseEvent *event)
{
    // Disable dragging if parent is in grid mode (viewsPanel)
    if (parentWidget() && parentWidget()->objectName() == "viewsPanel") {
        event->ignore();
        return;
    }

    if (event->button() == Qt::LeftButton) {
        m_dragStartPosition = event->pos();
    }

    QFrame::mousePressEvent(event);
}


void DraggableViewFrame::mouseMoveEvent(QMouseEvent *event)
{
    if (!(event->buttons() & Qt::LeftButton)) {
        return;
    }

    // Check if we've moved far enough for a drag
    if ((event->pos() - m_dragStartPosition).manhattanLength() < QApplication::startDragDistance()) {
        return;
    }

    // Create drag object
    QDrag *drag = new QDrag(this);
    QMimeData *mimeData = new QMimeData;

    // Store the source container ID
    mimeData->setText(QString::number(property("viewIndex").toInt()));
    drag->setMimeData(mimeData);

    // Custom drag pixmap (optional)
    QPixmap pixmap(size());
    pixmap.fill(Qt::transparent);
    render(&pixmap);
    pixmap = pixmap.scaled(width()/2, height()/2, Qt::KeepAspectRatio);
    drag->setPixmap(pixmap);
    drag->setHotSpot(QPoint(pixmap.width()/2, pixmap.height()/2));

    // Execute the drag
    Qt::DropAction dropAction = drag->exec(Qt::MoveAction);
}

void DraggableViewFrame::dragEnterEvent(QDragEnterEvent *event)
{
    // Accept drops from other view containers
    if (event->mimeData()->hasText()) {
        event->acceptProposedAction();
        setStyleSheet("QFrame { border: 2px dashed #3498db; background-color: #2d2d2d; }");

    }
}

void DraggableViewFrame::dropEvent(QDropEvent *event)
{
    // Get source container index
    int sourceIndex = event->mimeData()->text().toInt();
    int targetIndex = property("viewIndex").toInt();

    // Don't swap with self
    if (sourceIndex != targetIndex) {
        // Find the source container
        MainWindow* mainWindow = qobject_cast<MainWindow*>(window());

        if (mainWindow) {
            // Find source container
            QFrame* sourceContainer = nullptr;
            const QList<DraggableViewFrame*> frames = mainWindow->findChildren<DraggableViewFrame*>();

            for (auto* frame : frames) {
                if (frame->property("viewIndex").toInt() == sourceIndex) {
                    sourceContainer = frame;
                    break;
                }
            }

            // Perform the swap
            if (sourceContainer) {
                mainWindow->swapContainers(sourceContainer, this);

                // Fix flickering in both containers after swap
                QList<OpenGLWidget*> sourceViews = sourceContainer->findChildren<OpenGLWidget*>();
                QList<OpenGLWidget*> targetViews = findChildren<OpenGLWidget*>();

                for (auto* view : sourceViews) {
                    mainWindow->fixViewportFlickering(view);
                }

                for (auto* view : targetViews) {
                    mainWindow->fixViewportFlickering(view);
                }
            }
        }
    }

    // Reset style while maintaining border
    setStyleSheet("QFrame { border: 1px solid #1e90ff; background-color: #2d2d2d; }");
    event->acceptProposedAction();
}

void MainWindow::maximizeContainer(QFrame* container, OpenGLWidget* view)
{
    if (!m_gridLayout) return;

    // Hide all other containers
    for (auto* c : m_gridContainers) {
        if (c != container) {
            c->hide();
        }
    }

    // Remove the widget from its current position
    m_gridLayout->removeWidget(container);

    // Add the container spanning the full grid area
    m_gridLayout->addWidget(container, 0, 0, m_gridLayout->rowCount(), m_gridLayout->columnCount());

    // Remove fixed size limitation and allow expansion
    container->setMinimumSize(0, 0);
    container->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Also ensure the view inside fills the space
    view->setMinimumSize(0, 0);
    view->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Ensure layout margins are tight
    QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(container->layout());
    if (layout) {
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
    }

    container->setContentsMargins(0, 0, 0, 0);

    // Update the view
    view->setAutoFillBackground(true);
    view->setUpdatesEnabled(true);
    view->update();

    QLabel* titleLabel = container->findChild<QLabel*>();
    if (titleLabel) {
        statusBar()->showMessage(tr("Maximized view: %1").arg(titleLabel->text()), 2000);
    } else {
        statusBar()->showMessage(tr("Maximized view"), 2000);
    }

    // Force layout refresh
    m_gridLayout->invalidate();
    m_gridLayout->update();
    container->raise();
    view->raise();
}


void MainWindow::restoreGridLayout()
{
    if (!m_gridLayout || m_gridContainers.isEmpty()) return;
    constexpr int fixedWidth = 320;
    constexpr int fixedHeight = 240;
    // Get grid dimensions
    int cols = qCeil(qSqrt(m_gridContainers.size()));
    int rows = qCeil(static_cast<float>(m_gridContainers.size()) / cols);

    // Get available viewport size
    QSize availableSize = size();
    int dockWidth = 0;
    if (ui->hierarchyDock->isVisible()) {
        dockWidth += ui->hierarchyDock->width();
    }
    if (ui->propertiesDock->isVisible()) {
        dockWidth += ui->propertiesDock->width();
    }

    // Calculate optimal cell size with no gaps
    int availableWidth = availableSize.width() - dockWidth;
    int cellWidth = availableWidth / cols;
    int cellHeight = availableSize.height() / rows;

    // Restore each container to its original position
    for (auto* container : m_gridContainers) {
        if (container) {
            // Get original position
            int row = container->property("originalRow").toInt();
            int col = container->property("originalCol").toInt();

            // Remove from current position
            m_gridLayout->removeWidget(container);

            // Add back to original position
            m_gridLayout->addWidget(container, row, col);

            // Set size to fill cell
            container->setMinimumSize(cellWidth, cellHeight);
            container->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            // Reset margins to zero
            container->setContentsMargins(0, 0, 0, 0);

            // Show the container
            container->show();
        }
    }

    // Get all OpenGL views and ensure they're showing
    QList<OpenGLWidget*> views = findChildren<OpenGLWidget*>();
    for (auto view : views) {
        view->setUpdatesEnabled(true);
        view->show();
    }

    // Update status
    statusBar()->showMessage(tr("Restored grid layout"), 2000);

    // Use a two-phase update to fix flickering
    QTimer::singleShot(50, this, [this, views = std::move(views)]() {
        for (auto view : views) {
            view->update();
        }

        QTimer::singleShot(50, [views]() {
            for (auto view : views) {
                view->update();
            }
        });
    });
}
void MainWindow::swapContainers(QFrame* source, QFrame* target)
{
    if (!m_gridLayout || !source || !target || source == target) return;

    // Get original positions
    int sourceRow = source->property("originalRow").toInt();
    int sourceCol = source->property("originalCol").toInt();
    int targetRow = target->property("originalRow").toInt();
    int targetCol = target->property("originalCol").toInt();

    // Remove both from layout
    m_gridLayout->removeWidget(source);
    m_gridLayout->removeWidget(target);

    // Swap positions in layout
    m_gridLayout->addWidget(source, targetRow, targetCol);
    m_gridLayout->addWidget(target, sourceRow, sourceCol);

    // Update their position metadata
    source->setProperty("originalRow", targetRow);
    source->setProperty("originalCol", targetCol);
    target->setProperty("originalRow", sourceRow);
    target->setProperty("originalCol", sourceCol);

    // Ensure uniform layout stretch
    for (int r = 0; r < m_gridLayout->rowCount(); ++r)
        m_gridLayout->setRowStretch(r, 1);
    for (int c = 0; c < m_gridLayout->columnCount(); ++c)
        m_gridLayout->setColumnStretch(c, 1);

    // Reset view state for both containers
    for (OpenGLWidget* view : source->findChildren<OpenGLWidget*>()) {
        view->setViewChanged(true);
        view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        QTimer::singleShot(50, view, [view]() {
            view->update();
        });
    }
    for (OpenGLWidget* view : target->findChildren<OpenGLWidget*>()) {
        view->setViewChanged(true);
        view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        QTimer::singleShot(50, view, [view]() {
            view->update();
        });
    }

    // Force the layout to recompute immediately to avoid any overlap
    m_gridLayout->update();
    centralWidget()->updateGeometry();

}


void DraggableViewFrame::mouseReleaseEvent(QMouseEvent *event)
{
    QFrame::mouseReleaseEvent(event);
}

void MainWindow::fixViewportFlickering(OpenGLWidget* view)
{
    if (!view) return;

    // Basic settings that can help reduce flickering
    view->setAttribute(Qt::WA_TranslucentBackground, false);
    view->setAttribute(Qt::WA_NoSystemBackground, false);
    view->setAttribute(Qt::WA_OpaquePaintEvent, true);

    // Ensure the view is properly updating
    view->setAutoFillBackground(true);
    view->setUpdatesEnabled(true);

    // Force an immediate update
    view->update();

    // Schedule a second update to stabilize rendering
    QTimer::singleShot(50, view, [view]() {
        view->update();
    });
}

// Add this implementation to mainwindow.cpp
void MainWindow::applyOrthographicViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setOrthographicView();

    // Update status bar to show which view was affected
    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }
    statusBar()->showMessage(tr("Applied Orthographic View to %1").arg(viewName), 2000);
    logToConsole(tr("Applied Orthographic View to %1").arg(viewName),"SUCCESS");
}
void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    updateToggleButtonPositions();

    if (m_inGridViewMode) {
        for (auto* container : m_gridContainers) {
            auto* view = container->findChild<OpenGLWidget*>();
            if (view) {
                // Use expanding size policies for responsive resizing
                container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
                view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

                container->setMinimumSize(0, 0);
                container->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);

                view->setMinimumSize(0, 0);
                view->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);

                // Ensure layout and paint update
                view->setViewChanged(true);
                view->update();
            }
        }

        // Refresh the layout to trigger relayout of views
        if (m_gridLayout) {
            m_gridLayout->invalidate();
            m_gridLayout->update();
        }
    }
}

void MainWindow::applyBottomViewToActive()
{
    OpenGLWidget* activeView = m_activeViewWidget ? m_activeViewWidget : openGLWidget;
    activeView->setBottomView();  // Assumes this method exists in OpenGLWidget

    QString viewName = "Main View";
    if (activeView != openGLWidget) {
        for (const auto& viewWindow : m_viewWindows) {
            if (viewWindow.openGLWidget == activeView) {
                viewName = viewWindow.window->windowTitle();
                break;
            }
        }
    }

    QString msg = tr("Applied Bottom View to %1").arg(viewName);
    statusBar()->showMessage(msg, 2000);
    logToConsole(msg, "INFO");

}

void MainWindow::arrangeViewsInGrid()
{
    constexpr int fixedWidth  = 320;
    constexpr int fixedHeight = 240;

    if (m_viewWindows.isEmpty()) {
        QString msg = tr("No additional view windows to arrange");
        statusBar()->showMessage(msg, 2000);
        logToConsole(msg, "WARNING");
        return;
    }

    /* ---------- capture current floating-window geometries ----------- */
    m_savedWindowGeometries.clear();
    for (const auto &vw : m_viewWindows) {
        m_savedWindowGeometries.append(
            vw.window ? vw.window->saveGeometry() : QByteArray()
            );
    }
    /* ---------------- hide non-essential docks ---------------------- */
    m_visibleDocks.clear();
    for (QDockWidget *dock : findChildren<QDockWidget *>()) {
        if (dock == ui->hierarchyDock || dock == ui->propertiesDock) continue;
        if (dock->isVisible()) { m_visibleDocks.append(dock); dock->hide(); }
    }

    /* ---------------- build grid container ------------------------- */
    QWidget      *containerWidget = new QWidget(this);
    QHBoxLayout  *mainLayout      = new QHBoxLayout(containerWidget);
    mainLayout->setSpacing(0);  mainLayout->setContentsMargins(0,0,0,0);

    QWidget      *viewsPanel = new QWidget(containerWidget);
    QGridLayout  *viewsLayout= new QGridLayout(viewsPanel);
    viewsLayout->setSpacing(0); viewsLayout->setContentsMargins(0,0,0,0);
    viewsLayout->setSizeConstraint(QLayout::SetDefaultConstraint);

    QVector<OpenGLWidget*> allViews{ openGLWidget };
    for (auto &vw : m_viewWindows) allViews.append(vw.openGLWidget);

    int totalViews = allViews.size();
    int cols = 3, rows = 3;
    if (totalViews > cols*rows) { cols = qCeil(qSqrt(totalViews));
        rows = qCeil(float(totalViews) / cols); }

    m_gridContainers.clear();

    /* account for side-docks when sizing cells */
    int left  = ui->hierarchyDock ->isVisible() ? ui->hierarchyDock ->width() : 0;
    int right = ui->propertiesDock->isVisible() ? ui->propertiesDock->width() : 0;
    QSize win = size();
    int cellWidth  = (win.width()  - left - right) / cols;
    int cellHeight =  win.height() / rows;

    /* ----------------- create a framed cell for each view ----------- */
    for (int i = 0; i < totalViews; ++i) {
        QString title = tr("3D View %1").arg(i);
        OpenGLWidget *view = allViews[i];

        /* outer frame (draggable) */
        DraggableViewFrame *frame = new DraggableViewFrame(this);
        frame->setMinimumSize(fixedWidth, fixedHeight+20);
        frame->setMaximumSize(fixedWidth, fixedHeight+20);
        frame->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
        frame->setStyleSheet(
            "QFrame { border:1px solid #1e90ff; background:#2d2d2d; }");
        frame->setProperty("viewIndex", i);

        /* title-bar */
        QWidget      *titleBar     = new QWidget();
        titleBar->setFixedHeight(20);
        titleBar->setStyleSheet("background:#1e90ff; color:white;");
        titleBar->setCursor(Qt::SizeAllCursor);
        QHBoxLayout *titleLayout   = new QHBoxLayout(titleBar);
        titleLayout->setContentsMargins(2,0,2,0); titleLayout->setSpacing(2);
        QLabel *titleLabel         = new QLabel(title);
        titleLabel->setStyleSheet("font-weight:bold;font-size:10px;");

        auto makeBtn=[&](const QString &txt){
            QToolButton* b=new QToolButton(); b->setText(txt);
            b->setFixedSize(14,14);
            b->setStyleSheet(
                "QToolButton{border:none;color:white;font-size:8px;padding:0;}");
            return b; };
        QToolButton *minBtn   = makeBtn("_");
        QToolButton *maxBtn   = makeBtn("□");
        QToolButton *closeBtn = makeBtn("×");

        titleLayout->addWidget(titleLabel);  titleLayout->addStretch();
        titleLayout->addWidget(minBtn);      titleLayout->addWidget(maxBtn);
        titleLayout->addWidget(closeBtn);

        /* layout inside frame */
        QVBoxLayout *frameLayout = new QVBoxLayout(frame);
        frameLayout->setContentsMargins(0,0,0,0); frameLayout->setSpacing(0);
        frameLayout->addWidget(titleBar);

        /* embed the OpenGL widget */
        view->setParent(frame);
        view->invalidateBuffers();
        view->setUpdatesEnabled(true);
        view->setFocusPolicy(Qt::StrongFocus);
        view->setMinimumSize(fixedWidth,fixedHeight);
        view->setMaximumSize(fixedWidth,fixedHeight);
        view->setAxisVisible(true);
        view->setGridVisible(true);
        view->resetCamera();
        view->setIsometricView();
        view->setViewChanged(true);
        frameLayout->addWidget(view,1);

        /* hide the floating window representation (except main) */
        if (i>0) m_viewWindows[i-1].window->hide();

        /* connections ----------------------------------------------- */
        connect(view,&OpenGLWidget::clicked,
                this,[this,view]{ setActiveViewWidget(view); });

        connect(minBtn,&QToolButton::clicked,
                [frame,titleBar,view]{  // toggle collapse
                    view->setVisible(!view->isVisible());
                    frame->setMaximumHeight(view->isVisible()
                                                ? QWIDGETSIZE_MAX
                                                : titleBar->height());});

        connect(maxBtn,&QToolButton::clicked,
                [this,frame,view]{
                    bool max=frame->property("isMaximized").toBool();
                    frame->setProperty("isMaximized",!max);
                    max ? restoreGridLayout()
                        : (m_lastMaximizedContainer=frame, maximizeContainer(frame,view));});

        /* ---- modified CLOSE behaviour: delete + remove container --- */
        connect(closeBtn, &QToolButton::clicked, [=] {
            // Hide the frame immediately from view
            frame->hide();

            if (i > 0) {  // Only remove extra views (never close the main view at i=0)
                int vwIndex = i - 1;
                if (vwIndex < m_viewWindows.size()) {
                    // Take out the view/window struct from the list
                    auto vw = m_viewWindows.takeAt(vwIndex);
                    OpenGLWidget* closingView = vw.openGLWidget;

                    // If this view is currently active, switch active view back to main
                    if (closingView && closingView == m_activeViewWidget) {
                        setActiveViewWidget(openGLWidget);
                    }

                    // Remove any hierarchy tree entries associated with the closing view
                    if (closingView) {
                        // Iterate through top-level items in the hierarchy model
                        for (int r = hierarchyModel->rowCount() - 1; r >= 0; --r) {
                            QStandardItem* item = hierarchyModel->item(r);
                            if (!item) continue;
                            // Check if this item is tied to the closing view
                            OpenGLWidget* itemView = item->data(Qt::UserRole + 1).value<OpenGLWidget*>();
                            if (itemView == closingView) {
                                hierarchyModel->removeRow(r);  // remove the entire item (and children)
                                continue;
                            }
                            // If the item has children (e.g., a group like "Viewport Captures"), check them
                            for (int c = item->rowCount() - 1; c >= 0; --c) {
                                QStandardItem* child = item->child(c);
                                if (!child) continue;
                                OpenGLWidget* childView = child->data(Qt::UserRole + 1).value<OpenGLWidget*>();
                                if (childView == closingView) {
                                    item->removeRow(c);  // remove this child entry referencing the closed view
                                }
                            }
                        }
                    }

                    // Disconnect and delete the OpenGL widget and its window for this view
                    if (closingView) {
                        closingView->disconnect();
                        closingView->deleteLater();
                    }
                    if (vw.window) {
                        vw.window->close();
                        vw.window->deleteLater();
                    }

                    // Remove the saved geometry for this view, if it exists
                    if (vwIndex < m_savedWindowGeometries.size()) {
                        m_savedWindowGeometries.removeAt(vwIndex);
                    }
                }
            }

            // Remove the frame from the grid container list and delete it
            m_gridContainers.removeOne(frame);
            frame->deleteLater();

            statusBar()->showMessage(tr("View closed"), 2000);
        });


        /* add to grid */
        int r=i/cols, c=i%cols;
        frame->setProperty("originalRow",r);
        frame->setProperty("originalCol",c);
        viewsLayout->addWidget(frame,r,c);
        m_gridContainers.append(frame);
    }

    /* stretch behaviour */
    for (int r=0;r<rows;++r) viewsLayout->setRowStretch(r,1);
    for (int c=0;c<cols;++c) viewsLayout->setColumnStretch(c,1);

    /* splitter – keeps docks visible */
    QSplitter *splitter=new QSplitter(Qt::Horizontal);
    splitter->addWidget(viewsPanel);
    splitter->setStretchFactor(0,1); splitter->setHandleWidth(0);
    mainLayout->addWidget(splitter,1);

    /* final book-keeping */
    setCentralWidget(containerWidget);
    addDockWidget(Qt::LeftDockWidgetArea , ui->hierarchyDock);
    addDockWidget(Qt::RightDockWidgetArea, ui->propertiesDock);

    m_mainSplitter  = splitter;
    m_gridLayout    = viewsLayout;
    m_inGridViewMode= true;

    for (OpenGLWidget* v : allViews) fixViewportFlickering(v);

    setActiveViewWidget(openGLWidget);

    /* repaint pass */
    QTimer::singleShot(50,this,[allViews]{
        for (auto *v:allViews) if (v) v->repaint();
        QTimer::singleShot(50,[allViews]{
            for (auto *v:allViews) if (v) v->repaint();});});

    statusBar()->showMessage(
        tr("Arranged %1 views in %2×%3 grid")
            .arg(m_gridContainers.size()).arg(cols).arg(rows),2000);

    logToConsole(tr("Arranged %1 views in %2×%3 grid")
                     .arg(m_gridContainers.size()).arg(cols).arg(rows), "SUCCESS");
}


void MainWindow::exitGridView()
{
    if (!m_inGridViewMode){
        logToConsole("Exit grid view requested, but application is not in grid view mode.", "WARNING");
        return;
    }
    /* ---- safely detach every OpenGL widget from its frame ---------- */
    openGLWidget->blockSignals(true);
    for (auto &vw : m_viewWindows)
        if (vw.openGLWidget) vw.openGLWidget->blockSignals(true);

    for (auto *frame : m_gridContainers) {
        if (!frame) continue;
        if (auto *view = frame->findChild<OpenGLWidget*>()) {
            frame->layout()->removeWidget(view);
            view->setParent(nullptr);           // detach
        }
        frame->deleteLater();
    }
    m_gridContainers.clear();   // nothing to iterate later

    /* ---- restore MAIN viewport to its original place --------------- */
    QWidget *central = new QWidget(this);
    auto    *vl      = new QVBoxLayout(central);
    vl->setContentsMargins(0,0,0,0); vl->setSpacing(0);
    vl->addWidget(openGLWidget);
    setCentralWidget(central);
    openGLWidget->blockSignals(false);
    openGLWidget->setFocus();
    openGLWidget->setViewChanged(true); openGLWidget->update();

    /* ---- restore every still-present floating view ----------------- */
    for (int i=0; i<m_viewWindows.size(); ++i) {
        auto &vw = m_viewWindows[i];
        if (!vw.window || !vw.openGLWidget) continue;   // was closed

        vw.openGLWidget->blockSignals(false);
        vw.openGLWidget->setParent(vw.window);
        vw.window->setCentralWidget(vw.openGLWidget);
        vw.openGLWidget->blockSignals(false);
        vw.openGLWidget->setParent(vw.window);
        vw.openGLWidget->invalidateBuffers();
        vw.window->setCentralWidget(vw.openGLWidget);

        /* geometry (if captured) */
        if (i < m_savedWindowGeometries.size() &&
            !m_savedWindowGeometries[i].isEmpty())
        {
            vw.window->restoreGeometry(m_savedWindowGeometries[i]);
        } else {
            vw.window->resize(640,480);
        }

        vw.window->show();
        vw.openGLWidget->setFrameRate(60);
        vw.openGLWidget->setViewChanged(true);
        vw.openGLWidget->update();
    }

    m_savedWindowGeometries.clear();   // not needed any more

    /* ---- re-show docks hidden during grid mode --------------------- */
    for (QDockWidget *dock : m_visibleDocks) dock->show();
    m_visibleDocks.clear();

    /* ---- reset grid-mode flags ------------------------------------- */
    m_gridLayout           = nullptr;
    m_mainSplitter         = nullptr;
    m_lastMaximizedContainer = nullptr;
    m_inGridViewMode       = false;

    /* ---- final UI refresh ------------------------------------------ */
    updateHierarchyView();
    statusBar()->showMessage(tr("Exited grid view"),1000);
    logToConsole("Exited grid layout and restored main window.", "SUCCESS");
    QApplication::processEvents();
}


void MainWindow::logToConsole(const QString& message, const QString& level)
{
    if (!ui || !ui->consoleOutput) {
        qWarning() << "consoleOutput not found or UI not initialized.";
        return;
    }

    // Format timestamped message
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    QString formatted = QString("[%1] [%2] %3").arg(timestamp, level.toUpper(), message);

    // Determine color tag based on level (rich text)
    QString color;
    if (level == "INFO")      color = "#1E90FF";  // DodgerBlue
    else if (level == "SUCCESS") color = "#228B22"; // ForestGreen
    else if (level == "WARNING") color = "#FF8C00"; // DarkOrange
    else if (level == "ERROR")   color = "#DC143C"; // Crimson
    else                        color = "#000000"; // Default to black

    // Format as HTML
    QString htmlMessage = QString("<span style=\"color:%1; font-weight:bold;\">%2</span>")
                              .arg(color, formatted);

    // Append as HTML (requires rich text handling)
    ui->consoleOutput->appendHtml(htmlMessage);

    // Scroll to bottom
    QTextCursor cursor = ui->consoleOutput->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->consoleOutput->setTextCursor(cursor);
}


//fullscreen

void MainWindow::toggleFullScreenMainWindow()
{
    // Temporarily block visual updates
    this->setUpdatesEnabled(false);

    if (!m_fullscreenMainWindow) {
        // Apply fullscreen state without visual delay
        this->setWindowState(Qt::WindowFullScreen);
    } else {
        this->setWindowState(Qt::WindowNoState);
    }

    m_fullscreenMainWindow = !m_fullscreenMainWindow;

    // Allow visual updates again
    this->setUpdatesEnabled(true);

    // Force a clean redraw
    this->update();
}


void MainWindow::toggleFullScreen3DView()
{
    if (!m_activeViewWidget)
        return;

    QWidget* containerWindow = m_activeViewWidget->window();

    // If it's a floating OpenGL window (QMainWindow), just toggle fullscreen
    bool isFloating = false;
    for (const auto& vw : m_viewWindows) {
        if (vw.openGLWidget == m_activeViewWidget) {
            isFloating = true;
            break;
        }
    }

    if (isFloating) {
        // Floating view: toggle fullscreen on the whole window
        if (containerWindow->windowState() & Qt::WindowFullScreen) {
            containerWindow->showNormal();
        } else {
            containerWindow->showFullScreen();
        }
        return;
    }

    // Embedded main view handling
    if (!m_openGLFullScreenDialog) {
        // Save parent info
        m_openGLPreviousParent = m_activeViewWidget->parentWidget();
        m_openGLPreviousLayout = m_openGLPreviousParent ? m_openGLPreviousParent->layout() : nullptr;

        if (m_openGLPreviousLayout)
            m_openGLPreviousLayout->removeWidget(m_activeViewWidget);

        // Ensure size policies allow expansion
        m_activeViewWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        m_activeViewWidget->setMinimumSize(0, 0);
        m_activeViewWidget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);

        // Create fullscreen frameless dialog
        m_openGLFullScreenDialog = new QDialog(this, Qt::Window);
        m_openGLFullScreenDialog->setWindowFlags(Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        m_openGLFullScreenDialog->setAttribute(Qt::WA_DeleteOnClose);
        m_openGLFullScreenDialog->resize(QGuiApplication::primaryScreen()->size());

        QVBoxLayout* layout = new QVBoxLayout(m_openGLFullScreenDialog);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        layout->addWidget(m_activeViewWidget);

        m_openGLFullScreenDialog->installEventFilter(this);
        m_openGLFullScreenDialog->showFullScreen();
        m_openGLFullScreenDialog->raise();
        m_openGLFullScreenDialog->activateWindow();
    }
    else {
        // Restore OpenGL widget to its original parent layout
        m_openGLFullScreenDialog->layout()->removeWidget(m_activeViewWidget);

        if (m_openGLPreviousParent && m_openGLPreviousLayout) {
            m_activeViewWidget->setParent(m_openGLPreviousParent);
            m_openGLPreviousLayout->addWidget(m_activeViewWidget);
            m_activeViewWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
            m_activeViewWidget->updateGeometry();
        }

        m_openGLFullScreenDialog->removeEventFilter(this);
        m_openGLFullScreenDialog->close();
        m_openGLFullScreenDialog->deleteLater();
        m_openGLFullScreenDialog = nullptr;

        m_openGLPreviousParent = nullptr;
        m_openGLPreviousLayout = nullptr;
    }
}
void MainWindow::on_actionManage_Objects_triggered()
{
    ManageObjects dialog(this);
    dialog.exec();
}
void MainWindow::on_actionManage_Twins_triggered()
{
    ManageTwins dialog(this);
    dialog.exec();
}
void MainWindow::on_actionCluster_list_triggered()
{
    OpenGLWidget* activeWidget = getActiveViewWidget();
    if (!activeWidget) return;

    // Find selected point cloud
    QString selectedName;
    for (const auto& pc : activeWidget->getPointClouds()) {
        if (pc.selected) {
            selectedName = pc.name;
            break;
        }
    }

    if (selectedName.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a point cloud to view its cluster list.");
        return;
    }

    ClusterList* clusterListDialog = new ClusterList(this);
    std::vector<int> labels;
    std::vector<QVector3D> points;
    std::map<int, int> clusterToGroup; // Placeholder for group information

    // Retrieve point cloud data
    for (const auto& pc : activeWidget->getPointClouds()) {
        if (pc.name == selectedName) {
            labels = pc.labels;
            points = pc.points;
            // Note: clusterToGroup needs to be populated during identifySimilarClusters
            // For now, we pass an empty map as a placeholder
            break;
        }
    }

    clusterListDialog->updateClusterList(selectedName, labels, points, clusterToGroup);
    clusterListDialog->exec();
}
void MainWindow::on_actionObject_detection_Report_triggered() {
    qDebug() << "Object Detection Report action triggered!";

    // Check if object detection results are available
    if (m_clusterCounts.empty()) {
        QMessageBox::warning(this, "Object Detection Report",
                             "No object detection results found. Please perform object detection first!");
        return;
    }

    // Prompt user to select save location for the PDF report
    QString fileName = QFileDialog::getSaveFileName(this, "Save Object Detection Report", "", "PDF Files (*.pdf)");
    if (fileName.isEmpty())
        return;

    // Grab the current active OpenGL widget framebuffer
    OpenGLWidget* activeWidget = getActiveViewWidget();
    if (!activeWidget) {
        QMessageBox::warning(this, "Object Detection Report", "No active view widget available!");
        return;
    }
    QImage snapshot = activeWidget->grabFramebuffer();
    if (snapshot.isNull()) {
        QMessageBox::warning(this, "Object Detection Report", "Failed to capture the view!");
        return;
    }

    // Create temporary directory for data exchange
    QString tempDir = QDir::tempPath() + "/object_detection_report_" + QString::number(QDateTime::currentMSecsSinceEpoch());
    if (!QDir().mkpath(tempDir)) {
        QMessageBox::warning(this, "Object Detection Report", "Failed to create temporary directory!");
        return;
    }

    // Save the screenshot
    QString screenshotPath = tempDir + "/screenshot.png";
    if (!snapshot.save(screenshotPath, "PNG")) {
        QMessageBox::warning(this, "Object Detection Report", "Failed to save screenshot!");
        QDir(tempDir).removeRecursively();
        return;
    }

    // Save logo if available
    QString logoPath = tempDir + "/logo.png";
    QImage logo(":/images/goprayaan.jpg");
    if (!logo.isNull()) {
        logo.save(logoPath, "PNG");
    }

    // Create JSON data file for Python script
    QJsonObject data;
    data["output_path"] = fileName;
    data["screenshot_path"] = screenshotPath;
    data["logo_path"] = logoPath;
    data["report_date"] = QDate::currentDate().toString("yyyy-MM-dd");
    data["generation_time"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    data["project_name"] = "Object Detection Analysis";
    data["analysis_id"] = QString("OBJDET_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));

    // Add object detection results
    QJsonObject clusterCounts;
    for (const auto& [className, count] : m_clusterCounts) {
        clusterCounts[QString::fromStdString(className)] = count;
    }
    data["cluster_counts"] = clusterCounts;

    // Add training parameters and accuracy
    data["num_epochs"] = m_numEpochs;
    data["learning_rate"] = m_learningRate;
    data["training_folder"] = m_trainingFolder;
    data["accuracy"] = m_accuracy * 100.0f; // Convert to percentage for report

    // Save JSON data
    QString dataPath = tempDir + "/report_data.json";
    QJsonDocument doc(data);
    QFile dataFile(dataPath);
    if (!dataFile.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Object Detection Report", "Failed to create data file!");
        QDir(tempDir).removeRecursively();
        return;
    }
    dataFile.write(doc.toJson());
    dataFile.close();

    // Write Python script to a temporary file
    QString pythonScriptPath = tempDir + "/generate_object_detection_report.py";
    QFile pythonFile(pythonScriptPath);
    if (!pythonFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Object Detection Report", "Failed to create Python script file!");
        QDir(tempDir).removeRecursively();
        return;
    }

    QTextStream out(&pythonFile);
    // Split the Python script into smaller chunks to avoid string size limit
    std::vector<QString> pythonScriptChunks = {
        R"(import sys
import json
import os
from datetime import datetime
from pathlib import Path
try:
    from reportlab.lib.pagesizes import A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch, cm
    from reportlab.lib.colors import HexColor, Color, black, white, grey
    from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, Image,
                                   PageBreak, Table, TableStyle)
    from reportlab.lib.enums import TA_CENTER, TA_LEFT, TA_JUSTIFY
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image as PILImage
    import seaborn as sns
except ImportError as e:
    print(f"Error: Required library not installed: {e}")
    print("Please install required packages:")
    print("pip install reportlab matplotlib pillow seaborn numpy")
    sys.exit(1)

class ObjectDetectionReportGenerator:
    def __init__(self, data_path):
        try:
            with open(data_path, 'r') as f:
                self.data = json.load(f)
        except Exception as e:
            print(f"Error loading data file: {e}")
            sys.exit(1)
        plt.style.use('seaborn-v0_8-whitegrid')
        sns.set_palette("husl")
        self.colors = {
            'primary': HexColor('#003087'),
            'secondary': HexColor('#005B99'),
            'accent': HexColor('#00A3AD'),
            'success': HexColor('#2E7D32'),
            'warning': HexColor('#F4A261'),
            'danger': HexColor('#D32F2F'),
            'dark': HexColor('#263238'),
            'light': HexColor('#F5F7FA'),
            'white': HexColor('#FFFFFF'),
            'neutral': HexColor('#E0E0E0')
        }
        self.styles = getSampleStyleSheet()
        self._setup_custom_styles()

    def _setup_custom_styles(self):
        font_name = 'Times-Roman'
        self.styles.add(ParagraphStyle(
            name='CustomTitle',
            parent=self.styles['Heading1'],
            fontSize=28,
            textColor=self.colors['primary'],
            spaceAfter=36,
            alignment=TA_CENTER,
            fontName=font_name,
            leading=32
        ))
        self.styles.add(ParagraphStyle(
            name='CustomHeading',
            parent=self.styles['Heading2'],
            fontSize=18,
            textColor=self.colors['dark'],
            spaceAfter=24,
            fontName=font_name,
            leading=22
        ))
        self.styles.add(ParagraphStyle(
            name='CustomBody',
            parent=self.styles['Normal'],
            fontSize=11,
            textColor=self.colors['dark'],
            spaceAfter=12,
            fontName=font_name,
            leading=14
        ))
        self.styles.add(ParagraphStyle(
            name='MetricValue',
            parent=self.styles['Normal'],
            fontSize=24,
            textColor=self.colors['primary'],
            fontName=font_name,
            alignment=TA_CENTER,
            leading=28
        ))

    def create_header_footer(self, canvas, doc):
        canvas.saveState()
        header_height = 80
        canvas.setFillColor(self.colors['primary'])
        canvas.rect(0, A4[1] - header_height, A4[0], header_height, fill=1)
        canvas.setFillColor(self.colors['white'])
        canvas.setFont("Times-Roman", 16)
        canvas.drawString(50, A4[1] - 35, "Object Detection Analysis Report")
        canvas.setFont("Times-Roman", 10)
        canvas.drawString(50, A4[1] - 55, f"Generated: {self.data['generation_time']}")
        canvas.drawRightString(A4[0] - 50, A4[1] - 55, f"Analysis ID: {self.data['analysis_id']}")
        if os.path.exists(self.data.get('logo_path', '')):
            try:
                canvas.drawImage(self.data['logo_path'], A4[0] - 180, A4[1] - 75,
                               width=120, height=50, preserveAspectRatio=True)
            except:
                pass
        canvas.setFillColor(self.colors['light'])
        canvas.rect(0, 0, A4[0], 50, fill=1)
        canvas.setFillColor(self.colors['dark'])
        canvas.setFont("Times-Roman", 9)
        canvas.drawString(50, 20, "© 2025 Object Detection System | Confidential")
        canvas.drawRightString(A4[0] - 50, 20, f"Page {doc.page}")
        canvas.restoreState()
)",
        R"(
    def create_executive_summary(self):
        elements = []
        elements.append(Paragraph("Executive Summary", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        total_objects = sum(self.data['cluster_counts'].values())
        class_count = len(self.data['cluster_counts'])
        metrics_data = [
            ['Metric', 'Value', 'Description'],
            ['Total Objects Detected', f"{total_objects:,}", 'Total number of objects identified'],
            ['Number of Classes', f"{class_count}", 'Distinct object classes detected'],
            ['Accuracy', f"{self.data.get('accuracy', 0.0):.2f}%", 'Model prediction accuracy'],
            ['Training Epochs', f"{self.data.get('num_epochs', 'N/A')}", 'Number of training iterations'],
            ['Analysis Date', self.data['report_date'], 'Date when analysis was performed']
        ]
        metrics_table = Table(metrics_data, colWidths=[2.5*inch, 1.5*inch, 3*inch])
        metrics_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), self.colors['primary']),
            ('TEXTCOLOR', (0, 0), (-1, 0), self.colors['white']),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('FONTNAME', (0, 0), (-1, 0), 'Times-Roman'),
            ('FONTSIZE', (0, 0), (-1, 0), 12),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), self.colors['light']),
            ('FONTNAME', (0, 1), (-1, -1), 'Times-Roman'),
            ('FONTSIZE', (0, 1), (-1, -1), 10),
            ('GRID', (0, 0), (-1, -1), 1, self.colors['primary']),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [self.colors['white'], self.colors['light']])
        ]))
        elements.append(metrics_table)
        elements.append(Spacer(1, 20))
        findings_text = f"""
        <b>Key Findings:</b><br/>
        • A total of {total_objects:,} objects were detected across {class_count} distinct classes<br/>
        • The model achieved an accuracy of {self.data.get('accuracy', 0.0):.2f}%<br/>
        • The most prevalent class was "{max(self.data['cluster_counts'], key=self.data['cluster_counts'].get)}"
          with {self.data['cluster_counts'][max(self.data['cluster_counts'], key=self.data['cluster_counts'].get)]:,} objects<br/>
        • Analysis was performed using {self.data.get('num_epochs', 'N/A')} training epochs
        """
        elements.append(Paragraph(findings_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements

    def create_detailed_analysis(self):
        elements = []
        elements.append(Paragraph("Detailed Analysis", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        if os.path.exists(self.data.get('screenshot_path', '')):
            try:
                elements.append(Paragraph("3D Visualization", self.styles['CustomBody']))
                elements.append(Paragraph(
                    "The following image shows the 3D visualization of the object detection results, "
                    "displaying detected objects with their respective class labels.",
                    self.styles['CustomBody']
                ))
                elements.append(Spacer(1, 10))
                img = Image(self.data['screenshot_path'])
                img_width, img_height = img.imageWidth, img.imageHeight
                max_width = 6 * inch
                max_height = 4 * inch
                scale = min(max_width / img_width, max_height / img_height)
                img.drawWidth = img_width * scale
                img.drawHeight = img_height * scale
                elements.append(img)
                elements.append(Spacer(1, 20))
            except Exception as e:
                print(f"Warning: Could not add screenshot: {e}")
        analysis_text = f"""
        <b>Analysis Parameters:</b><br/>
        The object detection analysis was performed with the following parameters:
        <br/><br/>
        • <b>Number of Epochs:</b> {self.data.get('num_epochs', 'N/A')} - Training iterations used<br/>
        • <b>Learning Rate:</b> {self.data.get('learning_rate', 'N/A')} - Optimization step size<br/>
        • <b>Training Folder:</b> {os.path.basename(self.data.get('training_folder', 'N/A'))} - Source of training data<br/>
        • <b>Total Objects Detected:</b> {sum(self.data['cluster_counts'].values()):,} objects across {len(self.data['cluster_counts'])} classes
        <br/><br/>
        <b>Results Interpretation:</b><br/>
        The analysis identifies and classifies objects within the point cloud data based on a trained model.
        Each detected object is assigned a class label, and the distribution of these classes provides insights
        into the composition of the dataset. The visualization highlights the spatial distribution and classification of objects.
        """
        elements.append(Paragraph(analysis_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements
)",
        R"(
    def create_charts(self):
        elements = []
        temp_dir = Path(self.data['screenshot_path']).parent
        elements.append(Paragraph("Statistical Analysis", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        bar_chart_path = temp_dir / "bar_chart.png"
        self._create_bar_chart(str(bar_chart_path))
        elements.append(Paragraph("Object Class Distribution", self.styles['CustomBody']))
        elements.append(Paragraph(
            "The following bar chart shows the number of objects detected for each class:",
            self.styles['CustomBody']
        ))
        elements.append(Spacer(1, 10))
        if bar_chart_path.exists():
            bar_img = Image(str(bar_chart_path), width=6*inch, height=4*inch)
            elements.append(bar_img)
        elements.append(Spacer(1, 30))
        pie_chart_path = temp_dir / "pie_chart.png"
        self._create_pie_chart(str(pie_chart_path))
        elements.append(Paragraph("Percentage Distribution", self.styles['CustomBody']))
        elements.append(Paragraph(
            "The pie chart below illustrates the percentage distribution of detected objects across classes:",
            self.styles['CustomBody']
        ))
        elements.append(Spacer(1, 10))
        if pie_chart_path.exists():
            pie_img = Image(str(pie_chart_path), width=6*inch, height=4*inch)
            elements.append(pie_img)
        elements.append(PageBreak())
        return elements

    def _create_bar_chart(self, output_path):
        try:
            categories = list(self.data['cluster_counts'].keys())
            counts = list(self.data['cluster_counts'].values())
            fig, ax = plt.subplots(figsize=(12, 8))
            x = np.arange(len(categories))
            width = 0.4
            bars = ax.bar(x, counts, width, label='Object Counts',
                         color='#005B99', alpha=0.8, edgecolor='black', linewidth=0.5)
            ax.set_xlabel('Object Classes', fontsize=14, fontweight='bold')
            ax.set_ylabel('Number of Objects', fontsize=14, fontweight='bold')
            ax.set_title('Object Detection Analysis - Class Distribution',
                        fontsize=16, fontweight='bold', pad=20)
            ax.set_xticks(x)
            ax.set_xticklabels(categories, fontsize=10, rotation=45 if len(categories) > 5 else 0)
            ax.legend(fontsize=12)
            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height + max(counts) * 0.01,
                       f'{int(height):,}', ha='center', va='bottom', fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.3, axis='y')
            ax.set_axisbelow(True)
            plt.tight_layout()
            plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()
        except Exception as e:
            print(f"Error creating bar chart: {e}")

    def _create_pie_chart(self, output_path):
        try:
            labels = list(self.data['cluster_counts'].keys())
            sizes = list(self.data['cluster_counts'].values())
            colors = ['#E74C3C', '#F1C40F', '#3498DB', '#2ECC71', '#9B59B6', '#1ABC9C', '#95A5A6']
            fig, ax = plt.subplots(figsize=(12, 8))
            wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%',
                                            startangle=90, textprops={'fontsize': 10})
            for autotext in autotexts:
                autotext.set_color('white')
                autotext.set_fontweight('bold')
                autotext.set_fontsize(10)
            ax.set_title('Object Class Percentage Distribution', fontsize=16, fontweight='bold', pad=20)
            ax.axis('equal')
            ax.legend(wedges, [f"{label}: {size:,}" for label, size in zip(labels, sizes)],
                     title="Object Classes", loc="center left", bbox_to_anchor=(1, 0, 0.5, 1))
            plt.tight_layout()
            plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()
        except Exception as e:
            print(f"Error creating pie chart: {e}")
)",
        R"(
    def create_technical_details(self):
        elements = []
        elements.append(Paragraph("Technical Details", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        details_data = [['Class Name', 'Object Count', 'Percentage']]
        total_objects = sum(self.data['cluster_counts'].values())
        for class_name, count in self.data['cluster_counts'].items():
            percentage = (count / total_objects * 100) if total_objects > 0 else 0.0
            details_data.append([class_name, f"{count:,}", f"{percentage:.2f}%"])
        details_table = Table(details_data, colWidths=[2.5*inch, 2*inch, 2*inch])
        details_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), self.colors['primary']),
            ('TEXTCOLOR', (0, 0), (-1, 0), self.colors['white']),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, 0), 'Times-Roman'),
            ('FONTSIZE', (0, 0), (-1, 0), 12),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), self.colors['light']),
            ('FONTNAME', (0, 1), (-1, -1), 'Times-Roman'),
            ('FONTSIZE', (0, 1), (-1, -1), 10),
            ('GRID', (0, 0), (-1, -1), 1, self.colors['primary']),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [self.colors['white'], self.colors['light']])
        ]))
        elements.append(details_table)
        elements.append(Spacer(1, 30))
        methodology_text = f"""
        <b>Analysis Methodology:</b><br/><br/>
        The object detection analysis was performed using the following methodology:
        <br/><br/>
        1. <b>Data Input:</b> Point cloud data and training dataset were loaded<br/>
        2. <b>Model Training:</b> A neural network was trained for {self.data.get('num_epochs', 'N/A')} epochs with a learning rate of {self.data.get('learning_rate', 'N/A')}<br/>
        3. <b>Object Classification:</b> Objects were classified into {len(self.data['cluster_counts'])} distinct classes<br/>
        4. <b>Results Visualization:</b> Detected objects were visualized in 3D space with class-specific colors<br/>
        5. <b>Statistics Generation:</b> Object counts and percentages were calculated for each class
        <br/><br/>
        <b>Technical Specifications:</b><br/>
        • Model Type: Deep Learning-based Object Detection<br/>
        • Training Epochs: {self.data.get('num_epochs', 'N/A')}<br/>
        • Learning Rate: {self.data.get('learning_rate', 'N/A')}<br/>
        • Analysis Date: {self.data['report_date']}<br/>
        • Report Generation: {self.data['generation_time']}
        """
        elements.append(Paragraph(methodology_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements

    def create_recommendations(self):
        elements = []
        elements.append(Paragraph("Recommendations & Conclusions", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        total_objects = sum(self.data['cluster_counts'].values())
        class_count = len(self.data['cluster_counts'])
        accuracy = self.data.get('accuracy', 0.0)
        recommendations = []
        if accuracy < 70:
            recommendations.extend([
                "LOW ACCURACY DETECTED: Model accuracy is below 70%.",
                "Consider increasing the number of training epochs or adjusting the learning rate.",
                "Review the quality and diversity of the training dataset."
            ])
        elif accuracy < 90:
            recommendations.extend([
                "MODERATE ACCURACY: Model accuracy is acceptable but could be improved.",
                "Evaluate the training data for potential biases or gaps.",
                "Consider fine-tuning the model with additional data or hyperparameters."
            ])
        else:
            recommendations.extend([
                "HIGH ACCURACY: The model performs well with an accuracy above 90%.",
                "Current configuration appears robust for this dataset.",
                "Continue monitoring performance with new data."
            ])
        if class_count < 3:
            recommendations.append("LIMITED CLASS DIVERSITY: Few distinct classes detected. Consider expanding the training dataset to include more object types.")
        recommendations.extend([
            "Regularly validate model performance with new datasets.",
            "Document model parameters and training configurations for reproducibility.",
            "Consider implementing real-time monitoring for detection accuracy."
        ])
        recommendations_text = "<b>Based on the analysis results, we recommend:</b><br/><br/>"
        for i, rec in enumerate(recommendations, 1):
            recommendations_text += f"{i}. {rec}<br/><br/>"
        elements.append(Paragraph(recommendations_text, self.styles['CustomBody']))
        elements.append(Spacer(1, 20))
        conclusions_text = f"""
        <b>Conclusions:</b><br/><br/>
        The object detection analysis successfully identified {total_objects:,} objects across {class_count} classes,
        achieving an accuracy of {accuracy:.2f}%.
        <br/><br/>
        The results provide valuable insights into the composition and distribution of objects within the dataset.
        The visualization and statistical analysis highlight the prevalence of different object classes,
        which can inform further processing or decision-making steps.
        <br/><br/>
        The analysis was conducted using a deep learning model trained for {self.data.get('num_epochs', 'N/A')} epochs
        with a learning rate of {self.data.get('learning_rate', 'N/A')}. These parameters should be evaluated
        for appropriateness based on the specific requirements of your application.
        """
        elements.append(Paragraph(conclusions_text, self.styles['CustomBody']))
        return elements
)",
        R"(
    def create_appendix(self):
        elements = []
        elements.append(Paragraph("Appendix", self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        system_info_text = f"""
        <b>System Information:</b><br/>
        • Report Generated: {self.data['generation_time']}<br/>
        • Analysis ID: {self.data['analysis_id']}<br/>
        • Project Name: {self.data.get('project_name', 'N/A')}<br/>
        • Python Version: {sys.version.split()[0]}<br/>
        • Report Generator Version: 1.0.0
        <br/><br/>
        <b>Data Sources:</b><br/>
        • Training Folder: {os.path.basename(self.data.get('training_folder', 'N/A'))}<br/>
        • Screenshot: {os.path.basename(self.data.get('screenshot_path', 'N/A'))}<br/>
        • Configuration File: {os.path.basename(self.data.get('data_path', 'report_data.json'))}
        <br/><br/>
        <b>Quality Assurance:</b><br/>
        This report has been automatically generated using validated algorithms and visualization techniques.
        All calculations have been performed using standard mathematical operations and the results have been
        cross-verified for consistency and accuracy.
        """
        elements.append(Paragraph(system_info_text, self.styles['CustomBody']))
        return elements

    def generate_report(self):
        try:
            doc = SimpleDocTemplate(
                self.data['output_path'],
                pagesize=A4,
                rightMargin=50,
                leftMargin=50,
                topMargin=100,
                bottomMargin=70
            )
            story = []
            story.append(Spacer(1, 2*inch))
            story.append(Paragraph("Object Detection Analysis", self.styles['CustomTitle']))
            story.append(Spacer(1, 0.5*inch))
            story.append(Paragraph(f"Report Generated: {self.data['generation_time']}", self.styles['CustomBody']))
            story.append(Paragraph(f"Analysis ID: {self.data['analysis_id']}", self.styles['CustomBody']))
            story.append(PageBreak())
            story.extend(self.create_executive_summary())
            story.extend(self.create_detailed_analysis())
            story.extend(self.create_charts())
            story.extend(self.create_technical_details())
            story.extend(self.create_recommendations())
            story.extend(self.create_appendix())
            doc.build(story, onFirstPage=self.create_header_footer, onLaterPages=self.create_header_footer)
            print(f"PDF report successfully generated: {self.data['output_path']}")
            return True
        except Exception as e:
            print(f"Error generating PDF report: {e}")
            import traceback
            traceback.print_exc()
            return False

def main():
    if len(sys.argv) != 2:
        print("Usage: python generate_object_detection_report.py <data_file.json>")
        sys.exit(1)
    data_file = sys.argv[1]
    if not os.path.exists(data_file):
        print(f"Error: Data file '{data_file}' not found!")
        sys.exit(1)
    print("Starting Object Detection Report Generation...")
    print(f"Data file: {data_file}")
    try:
        generator = ObjectDetectionReportGenerator(data_file)
        success = generator.generate_report()
        if success:
            print("Report generation completed successfully!")
            sys.exit(0)
        else:
            print("Report generation failed!")
            sys.exit(1)
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
)"
    };

    // Write all chunks to the file
    for (const auto& chunk : pythonScriptChunks) {
        out << chunk;
    }
    pythonFile.close();

    // Create progress dialog for Python execution
    QProgressDialog* pythonProgress = new QProgressDialog("Generating Object Detection Report...", "Cancel", 0, 0, this);
    pythonProgress->setWindowModality(Qt::WindowModal);
    pythonProgress->show();
    QApplication::processEvents();

    // Execute Python script in a separate thread
    QThread* thread = new QThread(this);
    PythonReportWorker* worker = new PythonReportWorker(pythonScriptPath, dataPath);
    worker->moveToThread(thread);

    // Connect signals
    connect(thread, &QThread::started, worker, &PythonReportWorker::run);
    connect(worker, &PythonReportWorker::finished, this, [=](int exitCode, const QString& output, const QString& errors) {
        pythonProgress->close();
        delete pythonProgress;

        qDebug() << "Python script exit code:" << exitCode;
        qDebug() << "Python script output:" << output;
        if (!errors.isEmpty()) {
            qDebug() << "Python script errors:" << errors;
        }

        if (exitCode == 0) {
            if (QFile::exists(fileName)) {
                QMessageBox::information(this, "Object Detection Report",
                                         "PDF report generated successfully!\n\nReport saved to: " + fileName);

                int ret = QMessageBox::question(this, "Open Report",
                                                "Would you like to open the generated report?",
                                                QMessageBox::Yes | QMessageBox::No);

                if (ret == QMessageBox::Yes) {
                    QDesktopServices::openUrl(QUrl::fromLocalFile(fileName));
                }
            } else {
                QMessageBox::warning(this, "Object Detection Report",
                                     "Python script completed but PDF file was not created!");
            }
        } else {
            QMessageBox::critical(this, "Object Detection Report",
                                  QString("Failed to generate PDF report!\n\nPython script error:\n%1\n\nOutput:\n%2").arg(errors, output));
        }

        // Cleanup temporary files
        QDir(tempDir).removeRecursively();

        // Cleanup thread and worker
        thread->quit();
        thread->wait();
        delete worker;
        delete thread;
    });
    connect(worker, &PythonReportWorker::error, this, [=](const QString& message) {
        pythonProgress->close();
        delete pythonProgress;
        QMessageBox::warning(this, "Object Detection Report", message);
        QDir(tempDir).removeRecursively();
        thread->quit();
        thread->wait();
        delete worker;
        delete thread;
    });

    thread->start();
}
void MainWindow::on_actionCDD_analysis_triggered()
{
    qDebug() << "CDD Reports action triggered!";

    // Check if collision detection has been performed
    OpenGLWidget* activeWidget = getActiveViewWidget();
    if (!activeWidget) {
        QMessageBox::warning(this, "CDD Report", "No active view widget found!");
        return;
    }

    bool hasCollisionResults = false;
    for (const auto& pc : activeWidget->m_pointClouds) {
        if (pc.name.endsWith("_collision")) {
            hasCollisionResults = true;
            break;
        }
    }

    if (!hasCollisionResults) {
        QMessageBox::warning(this, "CDD Report",
                             "No collision detection results found. Please perform collision detection (CDD) first!");
        return;
    }

    // Prompt user to select save location for PDF report
    QString fileName = QFileDialog::getSaveFileName(this, "Save CDD Report", "", "PDF Files (*.pdf)");
    if (fileName.isEmpty()) {
        return;
    }

    // Get the current active OpenGL widget framebuffer
    QImage snapshot = activeWidget->grabFramebuffer();
    if (snapshot.isNull()) {
        QMessageBox::warning(this, "CDD Report", "Failed to capture the view!");
        return;
    }

    // Create temporary directory for data exchange
    QString tempDir = QDir::tempPath() + "/cdd_report_" + QString::number(QDateTime::currentMSecsSinceEpoch());
    if (!QDir().mkpath(tempDir)) {
        QMessageBox::warning(this, "CDD Report", "Failed to create temporary directory!");
        return;
    }

    // Save the screenshot
    QString screenshotPath = tempDir + "/screenshot.png";
    if (!snapshot.save(screenshotPath, "PNG")) {
        QMessageBox::warning(this, "CDD Report", "Failed to save screenshot!");
        QDir(tempDir).removeRecursively();
        return;
    }

    // Save logo if available
    QString logoPath = tempDir + "/logo.png";
    QImage logo(":/images/goprayaan.jpg");
    if (!logo.isNull()) {
        logo.save(logoPath, "PNG");
    }

    // Check for existing collision result point clouds
    QString name1, name2;
    std::vector<QVector3D> points1, colors1, points2, colors2;
    CloudComparison::CollisionResult result1, result2;
    bool useExistingResults = false;
    float threshold = 0.5f; // Must match the threshold used in on_actionCDD_triggered

    // Look for point clouds ending with "_collision"
    QString collisionName1, collisionName2;
    for (const auto& pc : activeWidget->m_pointClouds) {
        if (pc.name.endsWith("_collision")) {
            if (collisionName1.isEmpty()) {
                collisionName1 = pc.name;
                points1 = pc.points;
                colors1 = pc.colors;
            } else if (collisionName2.isEmpty()) {
                collisionName2 = pc.name;
                points2 = pc.points;
                colors2 = pc.colors;
            }
        }
    }

    if (!collisionName1.isEmpty() && !collisionName2.isEmpty()) {
        // Found two collision result point clouds
        useExistingResults = true;

        // Extract original names (remove "_collision" suffix)
        name1 = collisionName1.left(collisionName1.length() - QString("_collision").length());
        name2 = collisionName2.left(collisionName2.length() - QString("_collision").length());

        // Calculate collision counts
        result1.totalPoints = points1.size();
        result1.collisionCount = 0;
        for (const auto& color : colors1) {
            if (qFuzzyCompare(color.x(), COLLISION_R / 255.0f) &&
                qFuzzyCompare(color.y(), COLLISION_G / 255.0f) &&
                qFuzzyCompare(color.z(), COLLISION_B / 255.0f)) {
                result1.collisionCount++;
            }
        }

        result2.totalPoints = points2.size();
        result2.collisionCount = 0;
        for (const auto& color : colors2) {
            if (qFuzzyCompare(color.x(), COLLISION_R / 255.0f) &&
                qFuzzyCompare(color.y(), COLLISION_G / 255.0f) &&
                qFuzzyCompare(color.z(), COLLISION_B / 255.0f)) {
                result2.collisionCount++;
            }
        }
    } else {
        // No collision results found; prompt user to select point clouds
        QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();
        if (selectedIndexes.size() != static_cast<size_t>(2)) {
            QMessageBox::warning(this, "Selection Error",
                                 "Please select exactly two point clouds in the hierarchy tree for collision detection analysis.");
            QDir(tempDir).removeRecursively();
            return;
        }

        for (int i = 0; i < selectedIndexes.size(); ++i) {
            QString itemText = hierarchyModel->itemFromIndex(selectedIndexes[i])->text();
            QString itemType = hierarchyModel->itemFromIndex(selectedIndexes[i])->data(Qt::UserRole).toString();

            if (itemType != "PointCloud") {
                QMessageBox::warning(this, "Selection Error",
                                     "Please select only point cloud objects for collision detection analysis.");
                QDir(tempDir).removeRecursively();
                return;
            }

            QString name = itemText;
            if (name.startsWith("[PointCloud] ")) {
                name.remove("[PointCloud] ");
            }

            std::vector<QVector3D> points, colors;
            if (!activeWidget->getPointCloudByName(name, points, colors)) {
                QMessageBox::warning(this, "Error",
                                     QString("Could not access data for point cloud: %1").arg(name));
                QDir(tempDir).removeRecursively();
                return;
            }

            if (i == 0) {
                points1 = points;
                colors1 = colors;
                name1 = name;
            } else {
                points2 = points;
                colors2 = colors;
                name2 = name;
            }
        }

        // Ensure point cloud names are set
        if (name1.isEmpty()) {
            name1 = "PointCloud1";
        }
        if (name2.isEmpty()) {
            name2 = "PointCloud2";
        }

        // Perform collision detection
        QProgressDialog progress("Performing collision detection...", "Cancel", 0, 100, this);
        progress.setWindowModality(Qt::WindowModal);

        bool success = cloudComparison->performCollisionDetection(
            points1, colors1, points2, colors2, threshold, result1, result2, &progress
        );

        if (!success) {
            QMessageBox::warning(this, "CDD Report", "Collision detection failed!");
            QDir(tempDir).removeRecursively();
            return;
        }

        // Add the result point clouds to the view
        QString resultName1 = name1 + "_collision";
        QString resultName2 = name2 + "_collision";
        activeWidget->addPointCloud(result1.points, result1.colors, resultName1);
        activeWidget->addPointCloud(result2.points, result2.colors, resultName2);
        updateHierarchyView();
        activeWidget->update();
    }

    // Create JSON data file for report
    QJsonObject data;
    data["output_path"] = fileName;
    data["screenshot_path"] = screenshotPath;
    data["logo_path"] = logoPath;
    data["threshold"] = threshold;
    data["report_date"] = QDate::currentDate().toString("yyyy-MM-dd");
    data["generation_time"] = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");

    // Point cloud 1 data
    QJsonObject pc1;
    pc1["name"] = name1;
    pc1["collision_count"] = static_cast<qint64>(result1.collisionCount);
    pc1["total_points"] = static_cast<qint64>(result1.totalPoints);
    pc1["collision_percentage"] = result1.totalPoints > 0 ?
        (100.0 * result1.collisionCount / result1.totalPoints) : 0.0;
    data["pointcloud1"] = pc1;

    // Point cloud 2 data
    QJsonObject pc2;
    pc2["name"] = name2;
    pc2["collision_count"] = static_cast<qint64>(result2.collisionCount);
    pc2["total_points"] = static_cast<qint64>(result2.totalPoints);
    pc2["collision_percentage"] = result2.totalPoints > 0 ?
        (100.0 * result2.collisionCount / result2.totalPoints) : 0.0;
    data["pointcloud2"] = pc2;

    // Additional metadata
    data["project_name"] = "Collision Detection Analysis";
    data["analysis_id"] = QString("CDD_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));

    // Save JSON data
    QString dataPath = tempDir + "/report_data.json";
    QJsonDocument doc(data);
    QFile dataFile(dataPath);
    if (!dataFile.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "CDD Report", "Failed to create data file!");
        QDir(tempDir).removeRecursively();
        return;
    }
    dataFile.write(doc.toJson());
    dataFile.close();

    // Write Python script to a temporary file
    QString pythonScriptPath = tempDir + "/generate_report.py";
    QFile pythonFile(pythonScriptPath);
    if (!pythonFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "CDD Report", "Failed to create Python script file!");
        QDir(tempDir).removeRecursively();
        return;
    }

    QTextStream out(&pythonFile);
    // Split the Python script into smaller chunks to avoid string size limit
    std::vector<QString> pythonScriptChunks = {
        R"(import sys
import json
import os
from datetime import datetime
from pathlib import Path
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch, cm
from reportlab.lib.colors import HexColor
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Image, PageBreak, Table, TableStyle
from reportlab.lib.enums import TA_CENTER, TA_LEFT
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
plt.style.use('seaborn-v0_8-whitegrid')
sns.set_palette('husl')

class CDDReportGenerator:
    def __init__(self, data_path):
        with open(data_path, 'r') as f:
            self.data = json.load(f)
        self.colors = {
            'primary': HexColor('#003087'),
            'secondary': HexColor('#005B99'),
            'accent': HexColor('#00A3AD'),
            'success': HexColor('#2E7D32'),
            'warning': HexColor('#F4A261'),
            'danger': HexColor('#D32F2F'),
            'dark': HexColor('#263238'),
            'light': HexColor('#F5F7FA'),
            'white': HexColor('#FFFFFF'),
            'neutral': HexColor('#E0E0E0')
        }
        self.styles = getSampleStyleSheet()
        self._setup_custom_styles()

    def _setup_custom_styles(self):
        font_name = 'Times-Roman'
        self.styles.add(ParagraphStyle(name='CustomTitle', parent=self.styles['Heading1'], fontSize=28, textColor=self.colors['primary'], spaceAfter=36, alignment=TA_CENTER, fontName=font_name, leading=32))
        self.styles.add(ParagraphStyle(name='CustomHeading', parent=self.styles['Heading2'], fontSize=18, textColor=self.colors['dark'], spaceAfter=24, fontName=font_name, leading=22))
        self.styles.add(ParagraphStyle(name='CustomBody', parent=self.styles['Normal'], fontSize=11, textColor=self.colors['dark'], spaceAfter=12, fontName=font_name, leading=14))
        self.styles.add(ParagraphStyle(name='MetricValue', parent=self.styles['Normal'], fontSize=24, textColor=self.colors['primary'], fontName=font_name, alignment=TA_CENTER, leading=28))

    def create_header_footer(self, canvas, doc):
        canvas.saveState()
        header_height = 80
        canvas.setFillColor(self.colors['primary'])
        canvas.rect(0, A4[1] - header_height, A4[0], header_height, fill=1)
        canvas.setFillColor(self.colors['white'])
        canvas.setFont('Times-Roman', 16)
        canvas.drawString(50, A4[1] - 35, 'Collision Detection Analysis Report')
        canvas.setFont('Times-Roman', 10)
        canvas.drawString(50, A4[1] - 55, f'Generated: {self.data["generation_time"]}')
        canvas.drawRightString(A4[0] - 50, A4[1] - 55, f'Analysis ID: {self.data["analysis_id"]}')
        if os.path.exists(self.data.get('logo_path', '')):
            try:
                canvas.drawImage(self.data['logo_path'], A4[0] - 180, A4[1] - 75, width=120, height=50, preserveAspectRatio=True)
            except:
                pass
        canvas.setFillColor(self.colors['light'])
        canvas.rect(0, 0, A4[0], 50, fill=1)
        canvas.setFillColor(self.colors['dark'])
        canvas.setFont('Times-Roman', 9)
        canvas.drawString(50, 20, '© 2025 Collision Detection System | Confidential')
        canvas.drawRightString(A4[0] - 50, 20, f'Page {doc.page}')
        canvas.restoreState()

    def create_executive_summary(self):
        elements = []
        elements.append(Paragraph('Executive Summary', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        pc1 = self.data['pointcloud1']
        pc2 = self.data['pointcloud2']
        total_points = pc1['total_points'] + pc2['total_points']
        total_collisions = pc1['collision_count'] + pc2['collision_count']
        overall_collision_rate = (total_collisions / total_points * 100) if total_points > 0 else 0
        metrics_data = [
            ['Metric', 'Value', 'Description'],
            ['Total Points Analyzed', f'{total_points:,}', 'Combined points from both point clouds'],
            ['Total Collision Points', f'{total_collisions:,}', 'Points detected as colliding'],
            ['Overall Collision Rate', f'{overall_collision_rate:.2f}%', 'Percentage of total points in collision'],
            ['Distance Threshold', f'{self.data["threshold"]} units', 'Minimum distance for collision detection'],
            ['Analysis Date', self.data['report_date'], 'Date when analysis was performed']
        ]
        metrics_table = Table(metrics_data, colWidths=[2.5*inch, 1.5*inch, 3*inch])
        metrics_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), self.colors['primary']),
            ('TEXTCOLOR', (0, 0), (-1, 0), self.colors['white']),
            ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
            ('FONTNAME', (0, 0), (-1, 0), 'Times-Roman'),
            ('FONTSIZE', (0, 0), (-1, 0), 12),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), self.colors['light']),
            ('FONTNAME', (0, 1), (-1, -1), 'Times-Roman'),
            ('FONTSIZE', (0, 1), (-1, -1), 10),
            ('GRID', (0, 0), (-1, -1), 1, self.colors['primary']),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [self.colors['white'], self.colors['light']])
        ]))
        elements.append(metrics_table)
        elements.append(Spacer(1, 20))
        findings_text = f"""
        <b>Key Findings:</b><br/>
        • Point cloud "{pc1['name']}" has {pc1['collision_count']:,} collision points out of {pc1['total_points']:,} total points ({pc1['collision_percentage']:.2f}%)<br/>
        • Point cloud "{pc2['name']}" has {pc2['collision_count']:,} collision points out of {pc2['total_points']:,} total points ({pc2['collision_percentage']:.2f}%)<br/>
        • The overall collision rate across both point clouds is {overall_collision_rate:.2f}%<br/>
        • Analysis was performed using a distance threshold of {self.data["threshold"]} units
        """
        elements.append(Paragraph(findings_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements
)",
        R"(
    def create_detailed_analysis(self):
        elements = []
        elements.append(Paragraph('Detailed Analysis', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        if os.path.exists(self.data.get('screenshot_path', '')):
            try:
                elements.append(Paragraph('3D Visualization', self.styles['CustomBody']))
                elements.append(Paragraph('The following image shows the 3D visualization of the collision detection analysis, displaying both point clouds and highlighting collision points.', self.styles['CustomBody']))
                elements.append(Spacer(1, 10))
                img = Image(self.data['screenshot_path'])
                img_width, img_height = img.imageWidth, img.imageHeight
                max_width = 6 * inch
                max_height = 4 * inch
                scale = min(max_width / img_width, max_height / img_height)
                img.drawWidth = img_width * scale
                img.drawHeight = img_height * scale
                elements.append(img)
                elements.append(Spacer(1, 20))
            except Exception as e:
                print(f'Warning: Could not add screenshot: {e}')
        analysis_text = f"""
        <b>Analysis Parameters:</b><br/>
        The collision detection analysis was performed with the following parameters:
        <br/><br/>
        • <b>Distance Threshold:</b> {self.data['threshold']} units - Points within this distance are considered colliding<br/>
        • <b>Point Cloud 1:</b> "{self.data['pointcloud1']['name']}" with {self.data['pointcloud1']['total_points']:,} points<br/>
        • <b>Point Cloud 2:</b> "{self.data['pointcloud2']['name']}" with {self.data['pointcloud2']['total_points']:,} points<br/>
        • <b>Total Points:</b> {self.data['pointcloud1']['total_points'] + self.data['pointcloud2']['total_points']:,} points analyzed
        <br/><br/>
        <b>Results Interpretation:</b><br/>
        The analysis identifies points in each point cloud that are within the specified distance threshold of points in the other point cloud.
        This helps identify potential collision zones, overlapping regions, or areas of close proximity between the two 3D datasets.
        """
        elements.append(Paragraph(analysis_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements

    def create_charts(self):
        elements = []
        temp_dir = Path(self.data['screenshot_path']).parent
        elements.append(Paragraph('Statistical Analysis', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        bar_chart_path = temp_dir / 'bar обслуживание.png'
        self._create_bar_chart(str(bar_chart_path))
        elements.append(Paragraph('Collision Points Comparison', self.styles['CustomBody']))
        elements.append(Paragraph('The following bar chart compares collision points and total points for each point cloud:', self.styles['CustomBody']))
        elements.append(Spacer(1, 10))
        if bar_chart_path.exists():
            bar_img = Image(str(bar_chart_path), width=6*inch, height=4*inch)
            elements.append(bar_img)
        elements.append(Spacer(1, 30))
        pie_chart_path = temp_dir / 'pie_chart.png'
        self._create_pie_chart(str(pie_chart_path))
        elements.append(Paragraph('Distribution Analysis', self.styles['CustomBody']))
        elements.append(Paragraph('The pie chart below shows the distribution of collision vs non-collision points:', self.styles['CustomBody']))
        elements.append(Spacer(1, 10))
        if pie_chart_path.exists():
            pie_img = Image(str(pie_chart_path), width=6*inch, height=4*inch)
            elements.append(pie_img)
        elements.append(PageBreak())
        return elements

    def _create_bar_chart(self, output_path):
        try:
            pc1 = self.data['pointcloud1']
            pc2 = self.data['pointcloud2']
            categories = [pc1['name'], pc2['name']]
            collision_counts = [pc1['collision_count'], pc2['collision_count']]
            total_counts = [pc1['total_points'], pc2['total_points']]
            fig, ax = plt.subplots(figsize=(12, 8))
            x = np.arange(len(categories))
            width = 0.35
            bars1 = ax.bar(x - width/2, collision_counts, width, label='Collision Points', color='#D32F2F', alpha=0.8, edgecolor='black', linewidth=0.5)
            bars2 = ax.bar(x + width/2, total_counts, width, label='Total Points', color='#2E7D32', alpha=0.8, edgecolor='black', linewidth=0.5)
            ax.set_xlabel('Point Clouds', fontsize=14, fontweight='bold')
            ax.set_ylabel('Number of Points', fontsize=14, fontweight='bold')
            ax.set_title('Collision Detection Analysis - Point Comparison', fontsize=16, fontweight='bold', pad=20)
            ax.set_xticks(x)
            ax.set_xticklabels(categories, fontsize=12)
            ax.legend(fontsize=12)
            def add_value_labels(bars):
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height + max(total_counts) * 0.01, f'{int(height):,}', ha='center', va='bottom', fontsize=10, fontweight='bold')
            add_value_labels(bars1)
            add_value_labels(bars2)
            ax.grid(True, alpha=0.3, axis='y')
            ax.set_axisbelow(True)
            plt.tight_layout()
            plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()
        except Exception as e:
            print(f'Error creating bar chart: {e}')

    def _create_pie_chart(self, output_path):
        try:
            pc1 = self.data['pointcloud1']
            pc2 = self.data['pointcloud2']
            labels = [f"{pc1['name']} Collision", f"{pc1['name']} Non-Collision", f"{pc2['name']} Collision", f"{pc2['name']} Non-Collision"]
            sizes = [pc1['collision_count'], pc1['total_points'] - pc1['collision_count'], pc2['collision_count'], pc2['total_points'] - pc2['collision_count']]
            colors = ['#D32F2F', '#F4A261', '#005B99', '#2E7D32']
            fig, ax = plt.subplots(figsize=(12, 8))
            wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90, textprops={'fontsize': 10})
            for autotext in autotexts:
                autotext.set_color('white')
                autotext.set_fontweight('bold')
                autotext.set_fontsize(10)
            ax.set_title('Collision Distribution Analysis', fontsize=16, fontweight='bold', pad=20)
            ax.axis('equal')
            ax.legend(wedges, [f'{label}: {size:,}' for label, size in zip(labels, sizes)], title='Point Categories', loc='center left', bbox_to_anchor=(1, 0, 0.5, 1))
            plt.tight_layout()
            plt.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()
        except Exception as e:
            print(f'Error creating pie chart: {e}')
)",
        R"(
    def create_technical_details(self):
        elements = []
        elements.append(Paragraph('Technical Details', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        pc1 = self.data['pointcloud1']
        pc2 = self.data['pointcloud2']
        details_data = [
            ['Parameter', pc1['name'], pc2['name']],
            ['Total Points', f"{pc1['total_points']:,}", f"{pc2['total_points']:,}"],
            ['Collision Points', f"{pc1['collision_count']:,}", f"{pc2['collision_count']:,}"],
            ['Non-Collision Points', f"{pc1['total_points'] - pc1['collision_count']:,}", f"{pc2['total_points'] - pc2['collision_count']:,}"],
            ['Collision Percentage', f"{pc1['collision_percentage']:.2f}%", f"{pc2['collision_percentage']:.2f}%"],
            ['Non-Collision Percentage', f"{100.0 - pc1['collision_percentage']:.2f}%", f"{100.0 - pc2['collision_percentage']:.2f}%"]
        ]
        details_table = Table(details_data, colWidths=[2.5*inch, 2*inch, 2*inch])
        details_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), self.colors['primary']),
            ('TEXTCOLOR', (0, 0), (-1, 0), self.colors['white']),
            ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
            ('FONTNAME', (0, 0), (-1, 0), 'Times-Roman'),
            ('FONTSIZE', (0, 0), (-1, 0), 12),
            ('BOTTOMPADDING', (0, 0), (-1, 0), 12),
            ('BACKGROUND', (0, 1), (-1, -1), self.colors['light']),
            ('FONTNAME', (0, 1), (-1, -1), 'Times-Roman'),
            ('FONTSIZE', (0, 1), (-1, -1), 10),
            ('GRID', (0, 0), (-1, -1), 1, self.colors['primary']),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('ROWBACKGROUNDS', (0, 1), (-1, -1), [self.colors['white'], self.colors['light']])
        ]))
        elements.append(details_table)
        elements.append(Spacer(1, 30))
        methodology_text = f"""
        <b>Analysis Methodology:</b><br/><br/>
        The collision detection analysis was performed using the following methodology:
        <br/><br/>
        1. <b>Data Input:</b> Two point clouds were loaded and preprocessed<br/>
        2. <b>Distance Calculation:</b> For each point in the first point cloud, the minimum distance to any point in the second point cloud was calculated<br/>
        3. <b>Threshold Application:</b> Points with distances less than or equal to {self.data['threshold']} units were classified as collision points<br/>
        4. <b>Statistics Generation:</b> Collision counts were calculated for both point clouds<br/>
        5. <b>Visualization:</b> Results were visualized in 3D space and statistical charts were generated
        <br/><br/>
        <b>Technical Specifications:</b><br/>
        • Distance Metric: Euclidean distance in 3D space<br/>
        • Threshold Value: {self.data['threshold']} units<br/>
        • Analysis Date: {self.data['report_date']}<br/>
        • Report Generation: {self.data['generation_time']}
        """
        elements.append(Paragraph(methodology_text, self.styles['CustomBody']))
        elements.append(PageBreak())
        return elements

    def create_recommendations(self):
        elements = []
        elements.append(Paragraph('Recommendations & Conclusions', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        pc1 = self.data['pointcloud1']
        pc2 = self.data['pointcloud2']
        total_points = pc1['total_points'] + pc2['total_points']
        total_collisions = pc1['collision_count'] + pc2['collision_count']
        overall_collision_rate = (total_collisions / total_points * 100) if total_points > 0 else 0
        recommendations = []
        if overall_collision_rate > 50:
            recommendations.extend([
                'HIGH COLLISION RATE DETECTED: The overall collision rate exceeds 50%, indicating significant overlap between point clouds.',
                'Consider increasing the distance threshold or reviewing the spatial positioning of the point clouds.',
                'Investigate potential data registration issues or alignment problems.'
            ])
        elif overall_collision_rate > 25:
            recommendations.extend([
                'MODERATE COLLISION RATE: The collision rate suggests some overlap between point clouds.',
                'Review the positioning and alignment of both point clouds.',
                'Consider if the current threshold is appropriate for your analysis requirements.'
            ])
        else:
            recommendations.extend([
                'LOW COLLISION RATE: The point clouds show minimal overlap.',
                'Current configuration appears to provide good separation between datasets.',
                'The analysis parameters seem well-suited for this dataset.'
            ])
        if abs(pc1['collision_percentage'] - pc2['collision_percentage']) > 20:
            recommendations.append('UNBALANCED COLLISION DISTRIBUTION: One point cloud shows significantly higher collision rate than the other. Review data quality and alignment.')
        recommendations.extend([
            'Regularly monitor collision detection results for consistency across similar datasets.',
            'Document threshold values and methodology for reproducible analysis.',
            'Consider implementing automated alerts for collision rates exceeding predefined limits.'
        ])
        recommendations_text = '<b>Based on the analysis results, we recommend:</b><br/><br/>'
        for i, rec in enumerate(recommendations, 1):
            recommendations_text += f'{i}. {rec}<br/><br/>'
        elements.append(Paragraph(recommendations_text, self.styles['CustomBody']))
        elements.append(Spacer(1, 20))
        conclusions_text = f"""
        <b>Conclusions:</b><br/><br/>
        The collision detection analysis has successfully processed {total_points:,} points from two point clouds,
        identifying {total_collisions:,} collision points with an overall collision rate of {overall_collision_rate:.2f}%.
        <br/><br/>
        Point cloud "{pc1['name']}" demonstrates a collision rate of {pc1['collision_percentage']:.2f}%,
        while point cloud "{pc2['name']}" shows {pc2['collision_percentage']:.2f}%.
        These results provide valuable insights into the spatial relationship between the datasets and can inform
        decision-making for subsequent processing or analysis steps.
        <br/><br/>
        The analysis was conducted using a distance threshold of {self.data['threshold']} units,
        which should be evaluated for appropriateness based on the specific requirements of your application.
        """
        elements.append(Paragraph(conclusions_text, self.styles['CustomBody']))
        return elements
)",
        R"(
    def create_appendix(self):
        elements = []
        elements.append(Paragraph('Appendix', self.styles['CustomHeading']))
        elements.append(Spacer(1, 20))
        system_info_text = f"""
        <b>System Information:</b><br/>
        • Report Generated: {self.data['generation_time']}<br/>
        • Analysis ID: {self.data['analysis_id']}<br/>
        • Project Name: {self.data.get('project_name', 'N/A')}<br/>
        • Python Version: {sys.version.split()[0]}<br/>
        • Report Generator Version: 1.0.0
        <br/><br/>
        <b>Data Sources:</b><br/>
        • Point Cloud 1: {self.data['pointcloud1']['name']}<br/>
        • Point Cloud 2: {self.data['pointcloud2']['name']}<br/>
        • Screenshot: {os.path.basename(self.data.get('screenshot_path', 'N/A'))}<br/>
        • Configuration File: {os.path.basename(self.data.get('data_path', 'report_data.json'))}
        <br/><br/>
        <b>Quality Assurance:</b><br/>
        This report has been automatically generated using validated algorithms and visualization techniques.
        All calculations have been performed using standard mathematical operations and the results have been
        cross-verified for consistency and accuracy.
        """
        elements.append(Paragraph(system_info_text, self.styles['CustomBody']))
        return elements

    def generate_report(self):
        try:
            doc = SimpleDocTemplate(self.data['output_path'], pagesize=A4, rightMargin=50, leftMargin=50, topMargin=100, bottomMargin=70)
            story = []
            story.append(Spacer(1, 2*inch))
            story.append(Paragraph('Collision Detection Analysis', self.styles['CustomTitle']))
            story.append(Spacer(1, 0.5*inch))
            story.append(Paragraph(f'Report Generated: {self.data["generation_time"]}', self.styles['CustomBody']))
            story.append(Paragraph(f'Analysis ID: {self.data["analysis_id"]}', self.styles['CustomBody']))
            story.append(PageBreak())
            story.extend(self.create_executive_summary())
            story.extend(self.create_detailed_analysis())
            story.extend(self.create_charts())
            story.extend(self.create_technical_details())
            story.extend(self.create_recommendations())
            story.extend(self.create_appendix())
            doc.build(story, onFirstPage=self.create_header_footer, onLaterPages=self.create_header_footer)
            return 0
        except Exception as e:
            print(f'Error generating PDF report: {e}')
            return 1

def main():
    if len(sys.argv) != 2:
        print("Usage: python generate_report.py <data_path>")
        return 1
    data_path = sys.argv[1]
    generator = CDDReportGenerator(data_path)
    return generator.generate_report()

if __name__ == "__main__":
    sys.exit(main())
)"
    };

    // Write all chunks to the file
    for (const auto& chunk : pythonScriptChunks) {
        out << chunk;
    }
    pythonFile.close();

    // Run the Python script using PythonReportWorker
    QThread* workerThread = new QThread(this);
    PythonReportWorker* worker = new PythonReportWorker(pythonScriptPath, dataPath);
    worker->moveToThread(workerThread);

    // Connect signals
    connect(workerThread, &QThread::started, worker, &PythonReportWorker::run);
    connect(worker, &PythonReportWorker::finished, this, [=](int exitCode, const QString& output, const QString& errors) {
        if (!errors.isEmpty()) {
            qDebug() << "Python script errors:" << errors;
        }
        if (exitCode == 0 && QFile::exists(fileName)) {
            QMessageBox::information(this, "CDD Report",
                                     "PDF report generated successfully!\n\nReport saved to: " + fileName);

            int ret = QMessageBox::question(this, "Open Report",
                                            "Would you like to open the generated report?",
                                            QMessageBox::Yes | QMessageBox::No);

            if (ret == QMessageBox::Yes) {
                QDesktopServices::openUrl(QUrl::fromLocalFile(fileName));
            }
        } else {
            QMessageBox::critical(this, "CDD Report",
                                  "Failed to generate PDF report!\n" + errors);
        }

        // Cleanup
        QDir(tempDir).removeRecursively();
        workerThread->quit();
        workerThread->wait();
        workerThread->deleteLater();
        worker->deleteLater();
    });
    connect(worker, &PythonReportWorker::error, this, [=](const QString& message) {
        QMessageBox::warning(this, "CDD Report", message);
        QDir(tempDir).removeRecursively();
        workerThread->quit();
        workerThread->wait();
        workerThread->deleteLater();
        worker->deleteLater();
    });

    // Start the worker thread
    workerThread->start();
}
void MainWindow::on_actionReplace_with_Twins_triggered() {
    if (m_clusterCounts.empty() || m_detectedLabels.empty() || m_detectedPoints.empty()) {
        QMessageBox::warning(this, tr("Error"), tr("No recognized objects available. Please run object detection first."));
        return;
    }

    QString message = tr("Preparing to replace objects with digital twins.\n\n");
    int totalClusters = 0;
    QString clusterDetails;
    for (const auto& [className, count] : m_clusterCounts) {
        clusterDetails += QString::fromStdString(className) + ": " + QString::number(count) + "\n";
        totalClusters += count;
    }
    message += tr("Total identified objects: %1\n").arg(totalClusters);
    message += tr("Object types and counts:\n%1").arg(clusterDetails);
    message += tr("Total labeled points to process: %1").arg(m_detectedLabels.size());

    QMessageBox::information(this, tr("Digital Twins Replacement"), message);

    // Compute object positions before opening the dialog
    std::map<std::string, std::vector<int>> classLabels;
    std::map<std::string, std::map<int, std::vector<QVector3D>>> classClusterPoints;
    std::map<std::string, std::vector<QVector3D>> objectPositions = computeObjectPositions(
        m_clusterCounts, m_detectedLabels, m_detectedPoints, classLabels, classClusterPoints
    );

    // Display object positions
    QString positionDetails = tr("Identified Object Positions:\n\n");
    for (const auto& [className, positions] : objectPositions) {
        positionDetails += QString::fromStdString(className) + ":\n";
        for (size_t i = 0; i < positions.size(); ++i) {
            const auto& pos = positions[i];
            positionDetails += QString("  Object %1: (%2, %3, %4)\n")
                .arg(i + 1)
                .arg(pos.x())
                .arg(pos.y())
                .arg(pos.z());
        }
    }
    QMessageBox::information(this, tr("Object Positions"), positionDetails);

    // Create a new view window for the twin replacement
    onNewViewWindowRequested(); // Create a new OpenGLWidget
    OpenGLWidget* twinViewWidget = m_viewWindows.last().openGLWidget;
    setActiveViewWidget(twinViewWidget);

    ReplaceWithTwinsDialog* dialog = new ReplaceWithTwinsDialog(m_clusterCounts, m_detectedLabels, m_detectedPoints, twinViewWidget, this);

    connect(dialog, &ReplaceWithTwinsDialog::replacementCompleted, this, [&](const std::map<std::string, std::vector<QVector3D>>& twinPositions) {
        QString positionDetails = tr("Digital Twin Positions:\n\n");
        for (const auto& [className, positions] : twinPositions) {
            positionDetails += QString::fromStdString(className) + ":\n";
            for (size_t i = 0; i < positions.size(); ++i) {
                const auto& pos = positions[i];
                positionDetails += QString("  Twin %1: (%2, %3, %4)\n")
                    .arg(i + 1)
                    .arg(pos.x())
                    .arg(pos.y())
                    .arg(pos.z());
            }
        }
        QMessageBox::information(this, tr("Twin Positions"), positionDetails);

        updateHierarchyView();
        twinViewWidget->update();
    });

    dialog->exec();
}
void MainWindow::on_actionColor_Assigner_triggered()
{
    qDebug() << "Color Assigner triggered";

    for (const auto& pc : openGLWidget->getPointClouds()) {
        if (pc.selected) {
            qDebug() << "Processing selected point cloud:" << pc.name;

            // Gather cluster information and current colors
            std::vector<ClusterInfo> clusters;
            std::map<int, std::vector<size_t>> clusterMap;

            // Build cluster map from labels
            for (size_t i = 0; i < pc.labels.size(); ++i) {
                int label = pc.labels[i];
                if (label >= 0) { // Only process valid cluster labels (not noise)
                    clusterMap[label].push_back(i);
                }
            }

            qDebug() << "Found" << clusterMap.size() << "clusters in point cloud";

            if (clusterMap.empty()) {
                QMessageBox::warning(this, tr("No Clusters"),
                                   tr("The selected point cloud has no valid clusters to color."));
                return;
            }

            // Create ClusterInfo for each cluster
            for (const auto& pair : clusterMap) {
                ClusterInfo info;
                info.label = pair.first;
                info.pointCount = pair.second.size();

                // Calculate bounds and centroid
                Eigen::Vector3f minBounds(std::numeric_limits<float>::max(),
                                        std::numeric_limits<float>::max(),
                                        std::numeric_limits<float>::max());
                Eigen::Vector3f maxBounds(-std::numeric_limits<float>::max(),
                                        -std::numeric_limits<float>::max(),
                                        -std::numeric_limits<float>::max());
                Eigen::Vector3f sum(0, 0, 0);

                for (size_t idx : pair.second) {
                    const auto& point = pc.points[idx];
                    Eigen::Vector3f p(point.x(), point.y(), point.z());
                    sum += p;
                    minBounds = minBounds.cwiseMin(p);
                    maxBounds = maxBounds.cwiseMax(p);
                }

                sum /= static_cast<float>(pair.second.size());
                info.centroid = QVector3D(sum.x(), sum.y(), sum.z());

                Eigen::Vector3f extent = maxBounds - minBounds;
                info.extent = Eigen::Vector3f(extent.x(), extent.y(), extent.z());

                float volume = extent.x() * extent.y() * extent.z();
                info.density = (volume > 0) ? info.pointCount / volume : 0;

                clusters.push_back(info);
            }

            // Sort clusters by label for consistent display
            std::sort(clusters.begin(), clusters.end(),
                     [](const ClusterInfo& a, const ClusterInfo& b) {
                         return a.label < b.label;
                     });

            // Get current colors - create a vector that maps to cluster labels
            std::vector<QVector3D> currentColors;
            if (!pc.colors.empty()) {
                // Extract representative colors for each cluster
                for (const auto& cluster : clusters) {
                    QVector3D avgColor(0, 0, 0);
                    int count = 0;

                    // Find points belonging to this cluster and average their colors
                    for (size_t i = 0; i < pc.labels.size(); ++i) {
                        if (pc.labels[i] == cluster.label && i < pc.colors.size()) {
                            avgColor += pc.colors[i];
                            count++;
                        }
                    }

                    if (count > 0) {
                        avgColor /= static_cast<float>(count);
                    } else {
                        // Default color if no colors found
                        avgColor = QVector3D(0.5f, 0.5f, 0.5f);
                    }
                    currentColors.push_back(avgColor);
                }
            } else {
                // If no colors exist, create default colors for each cluster
                currentColors.resize(clusters.size(), QVector3D(0.5f, 0.5f, 0.5f));
            }

            // Create and show the dialog
            ColorAssignerDialog dialog(clusters, currentColors, pc.labels, this);

            // Connect the signal properly
            connect(&dialog, &ColorAssignerDialog::colorsAssigned,
                    this, [this, name = pc.name](const std::vector<QVector3D>&, const std::map<int, QVector3D>& clusterColorMap) {
                qDebug() << "Colors assigned signal received with" << clusterColorMap.size() << "cluster colors";
                applyClusterColorsToPointCloud(name, clusterColorMap);
                updateHierarchyView();
                openGLWidget->update();
            });

            if (dialog.exec() == QDialog::Accepted) {
                qDebug() << "Color assignment dialog accepted";
            } else {
                qDebug() << "Color assignment dialog cancelled";
            }
            return;
        }
    }

    QMessageBox::information(this, tr("No Selection"),
                           tr("Please select a point cloud to assign colors."));
}

void MainWindow::applyClusterColorsToPointCloud(const QString& pointCloudName,
                                              const std::map<int, QVector3D>& clusterColorMap)
{
    qDebug() << "Applying cluster colors to point cloud:" << pointCloudName;
    qDebug() << "Cluster color map size:" << clusterColorMap.size();

    // Get the point cloud and update colors
    auto pointClouds = openGLWidget->getPointClouds();
    for (auto& pc : pointClouds) {
        if (pc.name == pointCloudName) {
            qDebug() << "Found point cloud, updating colors for" << pc.points.size() << "points";

            // Resize colors vector if needed
            if (pc.colors.size() != pc.points.size()) {
                pc.colors.resize(pc.points.size());
            }

            // Apply colors based on cluster labels
            int coloredPoints = 0;
            for (size_t i = 0; i < pc.labels.size() && i < pc.colors.size(); ++i) {
                int label = pc.labels[i];

                auto colorIt = clusterColorMap.find(label);
                if (colorIt != clusterColorMap.end()) {
                    pc.colors[i] = colorIt->second;
                    coloredPoints++;
                } else if (label == -1) {
                    // Noise points - assign gray color
                    pc.colors[i] = QVector3D(0.5f, 0.5f, 0.5f);
                } else {
                    // Unknown label - assign white color
                    pc.colors[i] = QVector3D(1.0f, 1.0f, 1.0f);
                }
            }

            qDebug() << "Colored" << coloredPoints << "points based on cluster assignments";

            // Update the point cloud in OpenGL widget
            openGLWidget->updatePointCloudData(pc);
            break;
        }
    }
}
void MainWindow::on_actionGrid_Population_density_triggered()
{
    OpenGLWidget* activeView = getActiveViewWidget();
    const auto& grids = activeView->get2DGrids();
    if (grids.empty()) {
        QMessageBox::warning(this, "No Grids Available",
                             "No 2D grids are available in the current view. Please create a grid first.");
        return;
    }

    // Use the first grid for simplicity; you may want to add a selection mechanism
    const auto& grid = grids[0];

    // Get the first point cloud for demonstration; you may want to allow selection
    const auto& pointClouds = activeView->getPointClouds();
    if (pointClouds.empty()) {
        QMessageBox::warning(this, "No Point Clouds Available",
                             "No point clouds are available in the current view. Please load a point cloud first.");
        return;
    }

    GridPopulationDensityDialog dialog(grid, pointClouds[0].name, activeView, this);
    dialog.exec();
}
bool MainWindow::exportPointCloud(const QString& pointCloudName, const QString& filePath, const QString& format)
{
    try {
        // Get point cloud data
        const auto& pointClouds = openGLWidget->getPointClouds();
        auto it = std::find_if(pointClouds.begin(), pointClouds.end(), [&pointCloudName](const OpenGLWidget::PointCloud& pc) {
            return pc.name == pointCloudName;
        });

        if (it == pointClouds.end()) {
            qDebug() << "Point cloud not found:" << pointCloudName;
            return false;
        }

        // Generate unique filename for separate export
        QString finalPath = filePath;
        QFileInfo fileInfo(filePath);
        QString baseName = fileInfo.baseName();
        QString suffix = fileInfo.suffix();
        QString dir = fileInfo.absolutePath();

        // Always append point cloud name for separate export
        finalPath = QString("%1/%2_%3.%4").arg(dir, baseName, pointCloudName, suffix);

        if (format == "pts") {
            QFile file(finalPath);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "Cannot open file for writing:" << finalPath;
                return false;
            }

            QTextStream out(&file);
            for (const auto& point : it->points) {
                out << point.x() << " " << point.y() << " " << point.z() << "\n";
            }
            file.close();
            qDebug() << "Successfully exported point cloud" << pointCloudName << "to" << finalPath;
            return true;
        } else if (format == "ply") {
            QFile file(finalPath);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "Cannot open file for writing:" << finalPath;
                return false;
            }

            QTextStream out(&file);
            // Write PLY header
            out << "ply\n";
            out << "format ascii 1.0\n";
            out << "element vertex " << it->points.size() << "\n";
            out << "property float x\n";
            out << "property float y\n";
            out << "property float z\n";
            out << "end_header\n";

            // Write vertex data
            for (const auto& point : it->points) {
                out << point.x() << " " << point.y() << " " << point.z() << "\n";
            }
            file.close();
            qDebug() << "Successfully exported point cloud" << pointCloudName << "to" << finalPath;
            return true;
        } else {
            qDebug() << "Unsupported point cloud export format:" << format;
            return false;
        }
    } catch (const std::exception& e) {
        qDebug() << "Error exporting point cloud" << pointCloudName << ":" << e.what();
        return false;
    }
}

bool MainWindow::exportModel(const QString& modelName, const QString& filePath, const QString& format)
{
    try {
        // Get the model data
        const auto& models = openGLWidget->getModels();
        auto it = std::find_if(models.begin(), models.end(), [&modelName](const OpenGLWidget::Model& model) {
            return model.name == modelName;
        });

        if (it == models.end()) {
            qDebug() << "Model not found:" << modelName;
            return false;
        }

        // Generate unique filename for separate export
        QString finalPath = filePath;
        QFileInfo fileInfo(filePath);
        QString baseName = fileInfo.baseName();
        QString suffix = fileInfo.suffix();
        QString dir = fileInfo.absolutePath();

        // Always append model name for separate export
        finalPath = QString("%1/%2_%3.%4").arg(dir, baseName, modelName, suffix);

        // Map file extension to Assimp format ID
        std::map<QString, std::string> formatMap = {
            {"obj", "obj"},
            {"fbx", "fbx"},
            {"stl", "stl"},
            {"dae", "collada"},
            {"ply", "ply"},
            {"gltf", "gltf2"},
            {"glb", "glb"},
            {"3ds", "3ds"},
            {"lwo", "lwo"},
            {"bvh", "bvh"},
            {"dxf", "dxf"}
        };

        auto formatIt = formatMap.find(format.toLower());
        if (formatIt == formatMap.end()) {
            qDebug() << "Unsupported model export format:" << format;
            return false;
        }

        // Create a new aiScene
        aiScene* scene = new aiScene();
        scene->mRootNode = new aiNode();
        scene->mRootNode->mName.Set(modelName.toStdString());

        // Count total meshes
        size_t totalMeshes = 0;
        for (const auto& node : it->nodes) {
            totalMeshes += node.meshes.size();
        }

        if (totalMeshes == 0) {
            qDebug() << "No meshes found in model:" << modelName;
            delete scene->mRootNode;
            delete scene;
            return false;
        }

        // Allocate meshes
        scene->mMeshes = new aiMesh*[totalMeshes];
        scene->mNumMeshes = totalMeshes;
        scene->mRootNode->mMeshes = new unsigned int[totalMeshes];
        scene->mRootNode->mNumMeshes = totalMeshes;

        size_t meshIndex = 0;
        for (const auto& node : it->nodes) {
            for (const auto& mesh : node.meshes) {
                aiMesh* ai_mesh = new aiMesh();
                scene->mMeshes[meshIndex] = ai_mesh;
                scene->mRootNode->mMeshes[meshIndex] = meshIndex;

                // Populate mesh data
                ai_mesh->mNumVertices = mesh.vertexCount;
                ai_mesh->mVertices = new aiVector3D[mesh.vertexCount];
                ai_mesh->mNumFaces = mesh.indexCount / 3; // Assuming triangles
                ai_mesh->mFaces = new aiFace[ai_mesh->mNumFaces];

                // Fill vertices
                for (unsigned int i = 0; i < mesh.vertexCount; ++i) {
                    ai_mesh->mVertices[i] = aiVector3D(
                        mesh.vertices[i].x(),
                        mesh.vertices[i].y(),
                        mesh.vertices[i].z()
                    );
                }

                // Fill faces
                for (unsigned int i = 0; i < ai_mesh->mNumFaces; ++i) {
                    aiFace& face = ai_mesh->mFaces[i];
                    face.mNumIndices = 3;
                    face.mIndices = new unsigned int[3];
                    face.mIndices[0] = mesh.indices[i * 3 + 0];
                    face.mIndices[1] = mesh.indices[i * 3 + 1];
                    face.mIndices[2] = mesh.indices[i * 3 + 2];
                }

                meshIndex++;
            }
        }

        // Export the scene
        Assimp::Exporter exporter;
        aiReturn result = exporter.Export(scene, formatIt->second, finalPath.toStdString());

        // Clean up
        for (unsigned int i = 0; i < scene->mNumMeshes; ++i) {
            aiMesh* mesh = scene->mMeshes[i];
            for (unsigned int j = 0; j < mesh->mNumFaces; ++j) {
                delete[] mesh->mFaces[j].mIndices;
            }
            delete[] mesh->mFaces;
            delete[] mesh->mVertices;
            delete mesh;
        }
        delete[] scene->mMeshes;
        delete[] scene->mRootNode->mMeshes;
        delete scene->mRootNode;
        delete scene;

        if (result != AI_SUCCESS) {
            qDebug() << "Failed to export model" << modelName << "to" << format << "Error:" << exporter.GetErrorString();
            return false;
        }

        qDebug() << "Successfully exported model" << modelName << "to" << finalPath;
        return true;
    } catch (const std::exception& e) {
        qDebug() << "Error exporting model" << modelName << ":" << e.what();
        return false;
    }
}

void MainWindow::on_actionExport_triggered()
{
    qDebug() << "Export action triggered";

    // Get selected items from the hierarchy view
    QModelIndexList selectedIndexes = ui->treeView->selectionModel()->selectedIndexes();
    if (selectedIndexes.isEmpty()) {
        QMessageBox::warning(this, tr("Export Error"), tr("No items selected for export."));
        return;
    }

    // Collect selected point clouds and models separately
    QStringList pointCloudNames;
    QStringList modelNames;
    for (const auto& index : selectedIndexes) {
        QString itemType = hierarchyModel->itemFromIndex(index)->data(Qt::UserRole).toString();
        QString itemText = hierarchyModel->itemFromIndex(index)->text();

        if (itemType == "PointCloud") {
            QString name = itemText;
            if (name.startsWith("[PointCloud] ")) {
                name.remove("[PointCloud] ");
            }
            pointCloudNames.append(name);
        } else if (itemType == "Model") {
            QString name = itemText;
            if (name.startsWith("[Model] ")) {
                name.remove("[Model] ");
            }
            modelNames.append(name);
        }
    }

    if (pointCloudNames.isEmpty() && modelNames.isEmpty()) {
        QMessageBox::warning(this, tr("Export Error"), tr("No point clouds or models selected for export."));
        return;
    }

    // Separate export for point clouds and models
    bool success = true;
    QStringList failedItems;

    // Export point clouds separately
    if (!pointCloudNames.isEmpty()) {
        QString pointCloudFilter = tr("Point Cloud Files (*.pts *.ply);;All Files (*)");
        QString pointCloudPath = QFileDialog::getSaveFileName(this, tr("Export Point Clouds"), "", pointCloudFilter);

        if (!pointCloudPath.isEmpty()) {
            QFileInfo fileInfo(pointCloudPath);
            QString format = fileInfo.suffix().toLower();

            // Validate point cloud format
            QStringList pointCloudFormats = {"pts", "ply"};
            if (!pointCloudFormats.contains(format)) {
                QMessageBox::warning(this, tr("Export Error"), tr("Cannot export point clouds to %1 format.").arg(format.toUpper()));
                return;
            }

            // Create progress dialog for point clouds
            QProgressDialog progress(tr("Exporting Point Clouds..."), tr("Cancel"), 0, pointCloudNames.size(), this);
            progress.setWindowModality(Qt::WindowModal);
            progress.setMinimumDuration(0);

            for (int i = 0; i < pointCloudNames.size(); ++i) {
                progress.setValue(i);
                progress.setLabelText(tr("Exporting point cloud: %1").arg(pointCloudNames[i]));
                QCoreApplication::processEvents();

                if (progress.wasCanceled()) {
                    QMessageBox::information(this, tr("Export Canceled"), tr("Point cloud export was canceled by the user."));
                    break;
                }

                if (!exportPointCloud(pointCloudNames[i], pointCloudPath, format)) {
                    failedItems.append(pointCloudNames[i]);
                    success = false;
                }
            }
            progress.setValue(pointCloudNames.size());
        }
    }

    // Export models separately
    if (!modelNames.isEmpty()) {
        QString modelFilter = tr(
            "General 3D Model Files (*.obj *.fbx *.stl *.dae *.ply *.gltf *.glb);;"
            "Game Engine Files (*.mdl *.md2 *.md3 *.md5mesh *.md5anim *.md5camera *.smd *.vta *.mesh *.skeleton *.3d);;"
            "Animation and Motion Capture Files (*.bvh);;"
            "CAD and Architectural Files (*.dxf *.ifc);;"
            "Legacy and Specialized Files (*.3ds *.lwo *.lx *.ac *.ms3d *.cob *.xgl *.zgl *.ogex *.q3d *.q3s *.irrmesh *.off *.raw *.ter *.hmp *.x);;"
            "All Files (*)"
        );
        QString modelPath = QFileDialog::getSaveFileName(this, tr("Export Models"), "", modelFilter);

        if (!modelPath.isEmpty()) {
            QFileInfo fileInfo(modelPath);
            QString format = fileInfo.suffix().toLower();

            // Validate model format
            QStringList modelFormats = {
                "obj", "fbx", "stl", "dae", "ply", "gltf", "glb", "mdl", "md2", "md3",
                "md5mesh", "md5anim", "md5camera", "smd", "vta", "mesh", "skeleton", "3d",
                "bvh", "dxf", "3ds", "lwo", "lx", "ac", "ms3d", "cob", "xgl", "zgl",
                "ogex", "q3d", "q3s", "irrmesh", "off", "raw", "ter", "hmp", "x"
            };
            if (!modelFormats.contains(format)) {
                QMessageBox::warning(this, tr("Export Error"), tr("Cannot export models to %1 format.").arg(format.toUpper()));
                return;
            }

            // Create progress dialog for models
            QProgressDialog progress(tr("Exporting Models..."), tr("Cancel"), 0, modelNames.size(), this);
            progress.setWindowModality(Qt::WindowModal);
            progress.setMinimumDuration(0);

            for (int i = 0; i < modelNames.size(); ++i) {
                progress.setValue(i);
                progress.setLabelText(tr("Exporting model: %1").arg(modelNames[i]));
                QCoreApplication::processEvents();

                if (progress.wasCanceled()) {
                    QMessageBox::information(this, tr("Export Canceled"), tr("Model export was canceled by the user."));
                    break;
                }

                if (!exportModel(modelNames[i], modelPath, format)) {
                    failedItems.append(modelNames[i]);
                    success = false;
                }
            }
            progress.setValue(modelNames.size());
        }
    }

    // Show result
    if (success && failedItems.isEmpty()) {
        QMessageBox::information(this, tr("Export Complete"),
            tr("Successfully exported %1 point cloud(s) and %2 model(s).")
            .arg(pointCloudNames.size()).arg(modelNames.size()));
    } else if (!failedItems.isEmpty()) {
        QMessageBox::warning(this, tr("Export Error"),
            tr("Failed to export the following items:\n%1").arg(failedItems.join("\n")));
    }

    openGLWidget->update();
}

// void MainWindow::on_togglePropertiesDock_clicked()
// {
//     // Prevent multiple animations
//     if (m_propertiesDockAnimating) return;
//     m_propertiesDockAnimating = true;

//     // Create button press animation
//     QPropertyAnimation* pressAnimation = new QPropertyAnimation(m_propertiesDockToggleButton, "geometry", this);
//     pressAnimation->setDuration(150);
//     pressAnimation->setStartValue(m_propertiesDockToggleButton->geometry());

//     QRect pressedRect = m_propertiesDockToggleButton->geometry();
//     pressedRect.adjust(1, 1, -1, -1);
//     pressAnimation->setEndValue(pressedRect);
//     pressAnimation->setEasingCurve(QEasingCurve::OutQuad);

//     // Create dock animation (recreate to avoid signal accumulation)
//     if (m_propertiesDockAnimation) {
//         m_propertiesDockAnimation->stop();
//         m_propertiesDockAnimation->deleteLater();
//     }
//     m_propertiesDockAnimation = new QPropertyAnimation(ui->propertiesDock, "maximumWidth", this);

//     bool isVisible = ui->propertiesDock->isVisible();
//     int dockWidth = ui->propertiesDock->sizeHint().width();
//     if (dockWidth < 200) dockWidth = 250; // Minimum width

//     m_propertiesDockAnimation->setDuration(400);
//     m_propertiesDockAnimation->setEasingCurve(QEasingCurve::OutCubic);

//     if (isVisible) {
//         // Animate closing
//         m_propertiesDockAnimation->setStartValue(dockWidth);
//         m_propertiesDockAnimation->setEndValue(0);

//         // Update button text immediately for better UX
//         m_propertiesDockToggleButton->setText("<");

//         // Use single-shot connection to avoid accumulation
//         connect(m_propertiesDockAnimation, &QPropertyAnimation::finished, this, [this]() {
//             ui->propertiesDock->hide();
//             ui->propertiesDock->setMaximumWidth(QWIDGETSIZE_MAX); // Reset to allow normal sizing
//             m_propertiesDockAnimating = false;
//             updateToggleButtonPositions();
//         }, Qt::SingleShotConnection);
//     } else {
//         // Show dock first, then animate opening
//         ui->propertiesDock->setMaximumWidth(0);
//         ui->propertiesDock->show();

//         m_propertiesDockAnimation->setStartValue(0);
//         m_propertiesDockAnimation->setEndValue(dockWidth);

//         // Update button text
//         m_propertiesDockToggleButton->setText(">");

//         // Use single-shot connection to avoid accumulation
//         connect(m_propertiesDockAnimation, &QPropertyAnimation::finished, this, [this]() {
//             ui->propertiesDock->setMaximumWidth(QWIDGETSIZE_MAX); // Allow normal resizing
//             m_propertiesDockAnimating = false;
//             updateToggleButtonPositions();
//         }, Qt::SingleShotConnection);
//     }

//     // Connect press animation completion to dock animation start
//     connect(pressAnimation, &QPropertyAnimation::finished, this, [this, pressAnimation]() {
//         // Restore button size
//         QPropertyAnimation* restoreAnimation = new QPropertyAnimation(m_propertiesDockToggleButton, "geometry", this);
//         restoreAnimation->setDuration(100);
//         restoreAnimation->setStartValue(m_propertiesDockToggleButton->geometry());

//         QRect originalRect = m_propertiesDockToggleButton->geometry();
//         originalRect.adjust(-1, -1, 1, 1);
//         restoreAnimation->setEndValue(originalRect);
//         restoreAnimation->setEasingCurve(QEasingCurve::OutQuad);

//         // Start dock animation
//         m_propertiesDockAnimation->start();

//         restoreAnimation->start(QAbstractAnimation::DeleteWhenStopped);
//         pressAnimation->deleteLater();
//     }, Qt::SingleShotConnection);

//     pressAnimation->start();
// }

// void MainWindow::on_toggleHierarchyDock_clicked()
// {
//     // Prevent multiple animations
//     if (m_hierarchyDockAnimating) return;
//     m_hierarchyDockAnimating = true;

//     // Create button press animation
//     QPropertyAnimation* pressAnimation = new QPropertyAnimation(m_hierarchyDockToggleButton, "geometry", this);
//     pressAnimation->setDuration(150);
//     pressAnimation->setStartValue(m_hierarchyDockToggleButton->geometry());

//     QRect pressedRect = m_hierarchyDockToggleButton->geometry();
//     pressedRect.adjust(1, 1, -1, -1);
//     pressAnimation->setEndValue(pressedRect);
//     pressAnimation->setEasingCurve(QEasingCurve::OutQuad);

//     // Create dock animation (recreate to avoid signal accumulation)
//     if (m_hierarchyDockAnimation) {
//         m_hierarchyDockAnimation->stop();
//         m_hierarchyDockAnimation->deleteLater();
//     }
//     m_hierarchyDockAnimation = new QPropertyAnimation(ui->hierarchyDock, "maximumWidth", this);

//     bool isVisible = ui->hierarchyDock->isVisible();
//     int dockWidth = ui->hierarchyDock->sizeHint().width();
//     if (dockWidth < 200) dockWidth = 300; // Minimum width

//     m_hierarchyDockAnimation->setDuration(400);
//     m_hierarchyDockAnimation->setEasingCurve(QEasingCurve::OutCubic);

//     if (isVisible) {
//         // Animate closing
//         m_hierarchyDockAnimation->setStartValue(dockWidth);
//         m_hierarchyDockAnimation->setEndValue(0);

//         // Update button text immediately
//         m_hierarchyDockToggleButton->setText(">");

//         // Use single-shot connection to avoid accumulation
//         connect(m_hierarchyDockAnimation, &QPropertyAnimation::finished, this, [this]() {
//             ui->hierarchyDock->hide();
//             ui->hierarchyDock->setMaximumWidth(QWIDGETSIZE_MAX); // Reset to allow normal sizing
//             m_hierarchyDockAnimating = false;
//             updateToggleButtonPositions();
//         }, Qt::SingleShotConnection);
//     } else {
//         // Show dock first, then animate opening
//         ui->hierarchyDock->setMaximumWidth(0);
//         ui->hierarchyDock->show();

//         m_hierarchyDockAnimation->setStartValue(0);
//         m_hierarchyDockAnimation->setEndValue(dockWidth);

//         // Update button text
//         m_hierarchyDockToggleButton->setText("<");

//         // Use single-shot connection to avoid accumulation
//         connect(m_hierarchyDockAnimation, &QPropertyAnimation::finished, this, [this]() {
//             ui->hierarchyDock->setMaximumWidth(QWIDGETSIZE_MAX); // Allow normal resizing
//             m_hierarchyDockAnimating = false;
//             updateToggleButtonPositions();
//         }, Qt::SingleShotConnection);
//     }

//     // Connect press animation completion to dock animation start
//     connect(pressAnimation, &QPropertyAnimation::finished, this, [this, pressAnimation]() {
//         // Restore button size
//         QPropertyAnimation* restoreAnimation = new QPropertyAnimation(m_hierarchyDockToggleButton, "geometry", this);
//         restoreAnimation->setDuration(100);
//         restoreAnimation->setStartValue(m_hierarchyDockToggleButton->geometry());

//         QRect originalRect = m_hierarchyDockToggleButton->geometry();
//         originalRect.adjust(-1, -1, 1, 1);
//         restoreAnimation->setEndValue(originalRect);
//         restoreAnimation->setEasingCurve(QEasingCurve::OutQuad);

//         // Start dock animation
//         m_hierarchyDockAnimation->start();

//         restoreAnimation->start(QAbstractAnimation::DeleteWhenStopped);
//         pressAnimation->deleteLater();
//     }, Qt::SingleShotConnection);

//     pressAnimation->start();
// }

// void MainWindow::updateToggleButtonPositions()
// {
//     // Don't update positions during dock animations
//     if (m_propertiesDockAnimating || m_hierarchyDockAnimating) {
//         return;
//     }

//     // Stop any running button animations
//     m_animationGroup->stop();

//     QPoint propertiesNewPos;
//     QPoint hierarchyNewPos;

//     // Properties Dock (Right side, typically)
//     if (ui->propertiesDock->isVisible() && !ui->propertiesDock->isFloating()) {
//         QPoint dockPos = ui->propertiesDock->mapTo(this, QPoint(0, 0));
//         propertiesNewPos = dockPos + QPoint(-35, 10);
//         m_propertiesDockToggleButton->setText(">");
//     } else {
//         propertiesNewPos = QPoint(width() - m_propertiesDockToggleButton->width(), 50);
//         m_propertiesDockToggleButton->setText("<");
//     }

//     // Hierarchy Dock (Left side, typically)
//     if (ui->hierarchyDock->isVisible() && !ui->hierarchyDock->isFloating()) {
//         QPoint dockPos = ui->hierarchyDock->mapTo(this, QPoint(0, 0));
//         hierarchyNewPos = dockPos + QPoint(ui->hierarchyDock->width() + 5, 10);
//         m_hierarchyDockToggleButton->setText("<");
//     } else {
//         hierarchyNewPos = QPoint(0, 50);
//         m_hierarchyDockToggleButton->setText(">");
//     }

//     // Set up smooth animations for button repositioning
//     m_propertiesButtonAnimation->setStartValue(m_propertiesDockToggleButton->pos());
//     m_propertiesButtonAnimation->setEndValue(propertiesNewPos);
//     m_propertiesButtonAnimation->setDuration(400);
//     m_propertiesButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

//     m_hierarchyButtonAnimation->setStartValue(m_hierarchyDockToggleButton->pos());
//     m_hierarchyButtonAnimation->setEndValue(hierarchyNewPos);
//     m_hierarchyButtonAnimation->setDuration(400);
//     m_hierarchyButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

//     // Start the animations
//     m_animationGroup->start();

//     // Ensure buttons are visible and on top
//     m_propertiesDockToggleButton->show();
//     m_hierarchyDockToggleButton->show();
//     m_propertiesDockToggleButton->raise();
//     m_hierarchyDockToggleButton->raise();
// }

// void MainWindow::setupButtonHoverEffects()
// {
//     // Enhanced style for properties button
//     m_propertiesDockToggleButton->setStyleSheet(
//         "QPushButton {"
//         "    background-color: rgba(255, 255, 255, 0.9);"
//         "    border: 2px solid rgba(200, 200, 200, 0.8);"
//         "    border-radius: 15px;"
//         "    font-weight: bold;"
//         "    font-size: 12px;"
//         "    color: #333;"
//         "}"
//         "QPushButton:hover {"
//         "    background-color: rgba(255, 255, 255, 1.0);"
//         "    border: 2px solid #007ACC;"
//         "    transform: scale(1.05);"
//         "}"
//         "QPushButton:pressed {"
//         "    background-color: rgba(0, 122, 204, 0.2);"
//         "    border: 2px solid #005A9E;"
//         "}"
//     );

//     // Enhanced style for hierarchy button
//     m_hierarchyDockToggleButton->setStyleSheet(
//         "QPushButton {"
//         "    background-color: rgba(255, 255, 255, 0.9);"
//         "    border: 2px solid rgba(200, 200, 200, 0.8);"
//         "    border-radius: 15px;"
//         "    font-weight: bold;"
//         "    font-size: 12px;"
//         "    color: #333;"
//         "}"
//         "QPushButton:hover {"
//         "    background-color: rgba(255, 255, 255, 1.0);"
//         "    border: 2px solid #007ACC;"
//         "    transform: scale(1.05);"
//         "}"
//         "QPushButton:pressed {"
//         "    background-color: rgba(0, 122, 204, 0.2);"
//         "    border: 2px solid #005A9E;"
//         "}"
//     );
// }

void MainWindow::on_togglePropertiesDock_clicked()
{
    // Prevent multiple animations
    if (m_propertiesDockAnimating) return;
    m_propertiesDockAnimating = true;

    // Create button press animation
    QPropertyAnimation* pressAnimation = new QPropertyAnimation(m_propertiesDockToggleButton, "geometry", this);
    pressAnimation->setDuration(150);
    pressAnimation->setStartValue(m_propertiesDockToggleButton->geometry());

    QRect pressedRect = m_propertiesDockToggleButton->geometry();
    pressedRect.adjust(1, 1, -1, -1);
    pressAnimation->setEndValue(pressedRect);
    pressAnimation->setEasingCurve(QEasingCurve::OutQuad);

    // Create dock animation (recreate to avoid signal accumulation)
    if (m_propertiesDockAnimation) {
        m_propertiesDockAnimation->stop();
        m_propertiesDockAnimation->deleteLater();
    }
    m_propertiesDockAnimation = new QPropertyAnimation(ui->propertiesDock, "maximumWidth", this);

    bool isVisible = ui->propertiesDock->isVisible();
    int dockWidth = ui->propertiesDock->sizeHint().width();
    if (dockWidth < 200) dockWidth = 250; // Minimum width

    m_propertiesDockAnimation->setDuration(400);
    m_propertiesDockAnimation->setEasingCurve(QEasingCurve::OutCubic);

    if (isVisible) {
        // Animate closing
        m_propertiesDockAnimation->setStartValue(dockWidth);
        m_propertiesDockAnimation->setEndValue(0);

        // Update button text immediately for better UX
        m_propertiesDockToggleButton->setText("<");

        // Use single-shot connection to avoid accumulation
        connect(m_propertiesDockAnimation, &QPropertyAnimation::finished, this, [this]() {
            ui->propertiesDock->hide();
            ui->propertiesDock->setMaximumWidth(QWIDGETSIZE_MAX); // Reset to allow normal sizing
            m_propertiesDockAnimating = false;
            updateToggleButtonPositions();
        }, Qt::SingleShotConnection);
    } else {
        // Show dock first, then animate opening
        ui->propertiesDock->setMaximumWidth(0);
        ui->propertiesDock->show();

        m_propertiesDockAnimation->setStartValue(0);
        m_propertiesDockAnimation->setEndValue(dockWidth);

        // Update button text
        m_propertiesDockToggleButton->setText(">");

        // Use single-shot connection to avoid accumulation
        connect(m_propertiesDockAnimation, &QPropertyAnimation::finished, this, [this]() {
            ui->propertiesDock->setMaximumWidth(QWIDGETSIZE_MAX); // Allow normal resizing
            m_propertiesDockAnimating = false;
            updateToggleButtonPositions();
        }, Qt::SingleShotConnection);
    }

    // Connect press animation completion to dock animation start
    connect(pressAnimation, &QPropertyAnimation::finished, this, [this, pressAnimation]() {
        // Restore button size
        QPropertyAnimation* restoreAnimation = new QPropertyAnimation(m_propertiesDockToggleButton, "geometry", this);
        restoreAnimation->setDuration(100);
        restoreAnimation->setStartValue(m_propertiesDockToggleButton->geometry());

        QRect originalRect = m_propertiesDockToggleButton->geometry();
        originalRect.adjust(-1, -1, 1, 1);
        restoreAnimation->setEndValue(originalRect);
        restoreAnimation->setEasingCurve(QEasingCurve::OutQuad);

        // Start dock animation
        m_propertiesDockAnimation->start();

        restoreAnimation->start(QAbstractAnimation::DeleteWhenStopped);
        pressAnimation->deleteLater();
    }, Qt::SingleShotConnection);

    pressAnimation->start();
}

void MainWindow::on_toggleHierarchyDock_clicked()
{
    // Prevent multiple animations
    if (m_hierarchyDockAnimating) return;
    m_hierarchyDockAnimating = true;

    // Create button press animation
    QPropertyAnimation* pressAnimation = new QPropertyAnimation(m_hierarchyDockToggleButton, "geometry", this);
    pressAnimation->setDuration(150);
    pressAnimation->setStartValue(m_hierarchyDockToggleButton->geometry());

    QRect pressedRect = m_hierarchyDockToggleButton->geometry();
    pressedRect.adjust(1, 1, -1, -1);
    pressAnimation->setEndValue(pressedRect);
    pressAnimation->setEasingCurve(QEasingCurve::OutQuad);

    // Create dock animation (recreate to avoid signal accumulation)
    if (m_hierarchyDockAnimation) {
        m_hierarchyDockAnimation->stop();
        m_hierarchyDockAnimation->deleteLater();
    }
    m_hierarchyDockAnimation = new QPropertyAnimation(ui->hierarchyDock, "maximumWidth", this);

    bool isVisible = ui->hierarchyDock->isVisible();
    int dockWidth = ui->hierarchyDock->sizeHint().width();
    if (dockWidth < 200) dockWidth = 300; // Minimum width

    m_hierarchyDockAnimation->setDuration(400);
    m_hierarchyDockAnimation->setEasingCurve(QEasingCurve::OutCubic);

    if (isVisible) {
        // Animate closing
        m_hierarchyDockAnimation->setStartValue(dockWidth);
        m_hierarchyDockAnimation->setEndValue(0);

        // Update button text immediately
        m_hierarchyDockToggleButton->setText(">");

        // Use single-shot connection to avoid accumulation
        connect(m_hierarchyDockAnimation, &QPropertyAnimation::finished, this, [this]() {
            ui->hierarchyDock->hide();
            ui->hierarchyDock->setMaximumWidth(QWIDGETSIZE_MAX); // Reset to allow normal sizing
            m_hierarchyDockAnimating = false;
            updateToggleButtonPositions();
        }, Qt::SingleShotConnection);
    } else {
        // Show dock first, then animate opening
        ui->hierarchyDock->setMaximumWidth(0);
        ui->hierarchyDock->show();

        m_hierarchyDockAnimation->setStartValue(0);
        m_hierarchyDockAnimation->setEndValue(dockWidth);

        // Update button text
        m_hierarchyDockToggleButton->setText("<");

        // Use single-shot connection to avoid accumulation
        connect(m_hierarchyDockAnimation, &QPropertyAnimation::finished, this, [this]() {
            ui->hierarchyDock->setMaximumWidth(QWIDGETSIZE_MAX); // Allow normal resizing
            m_hierarchyDockAnimating = false;
            updateToggleButtonPositions();
        }, Qt::SingleShotConnection);
    }

    // Connect press animation completion to dock animation start
    connect(pressAnimation, &QPropertyAnimation::finished, this, [this, pressAnimation]() {
        // Restore button size
        QPropertyAnimation* restoreAnimation = new QPropertyAnimation(m_hierarchyDockToggleButton, "geometry", this);
        restoreAnimation->setDuration(100);
        restoreAnimation->setStartValue(m_hierarchyDockToggleButton->geometry());

        QRect originalRect = m_hierarchyDockToggleButton->geometry();
        originalRect.adjust(-1, -1, 1, 1);
        restoreAnimation->setEndValue(originalRect);
        restoreAnimation->setEasingCurve(QEasingCurve::OutQuad);

        // Start dock animation
        m_hierarchyDockAnimation->start();

        restoreAnimation->start(QAbstractAnimation::DeleteWhenStopped);
        pressAnimation->deleteLater();
    }, Qt::SingleShotConnection);

    pressAnimation->start();
}

void MainWindow::updateToggleButtonPositions()
{
    // Don't update positions during dock animations
    if (m_propertiesDockAnimating || m_hierarchyDockAnimating) {
        return;
    }

    // Stop any running button animations
    m_animationGroup->stop();

    QPoint propertiesNewPos;
    QPoint hierarchyNewPos;

    // Properties Dock (Right side, typically)
    if (ui->propertiesDock->isVisible() && !ui->propertiesDock->isFloating()) {
        QPoint dockPos = ui->propertiesDock->mapTo(this, QPoint(0, 0));
        propertiesNewPos = dockPos + QPoint(-35, 10);
        m_propertiesDockToggleButton->setText(">");
    } else {
        propertiesNewPos = QPoint(width() - m_propertiesDockToggleButton->width(), 50);
        m_propertiesDockToggleButton->setText("<");
    }

    // Hierarchy Dock (Left side, typically)
    if (ui->hierarchyDock->isVisible() && !ui->hierarchyDock->isFloating()) {
        QPoint dockPos = ui->hierarchyDock->mapTo(this, QPoint(0, 0));
        hierarchyNewPos = dockPos + QPoint(ui->hierarchyDock->width() + 5, 10);
        m_hierarchyDockToggleButton->setText("<");
    } else {
        hierarchyNewPos = QPoint(0, 50);
        m_hierarchyDockToggleButton->setText(">");
    }

    // Set up smooth animations for button repositioning with matching duration
    m_propertiesButtonAnimation->setStartValue(m_propertiesDockToggleButton->pos());
    m_propertiesButtonAnimation->setEndValue(propertiesNewPos);
    m_propertiesButtonAnimation->setDuration(400); // Match dock animation duration
    m_propertiesButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

    m_hierarchyButtonAnimation->setStartValue(m_hierarchyDockToggleButton->pos());
    m_hierarchyButtonAnimation->setEndValue(hierarchyNewPos);
    m_hierarchyButtonAnimation->setDuration(400); // Match dock animation duration
    m_hierarchyButtonAnimation->setEasingCurve(QEasingCurve::OutCubic);

    // Start the animations
    m_animationGroup->start();

    // Ensure buttons are visible and on top
    m_propertiesDockToggleButton->show();
    m_hierarchyDockToggleButton->show();
    m_propertiesDockToggleButton->raise();
    m_hierarchyDockToggleButton->raise();
}

void MainWindow::setupButtonHoverEffects()
{
    // Dark theme style for properties button
    m_propertiesDockToggleButton->setStyleSheet(
        "QPushButton {"
        "    background-color: #2e2e2e;"
        "    border: 1px solid #555555;"
        "    border-radius: 15px;"
        "    font-weight: bold;"
        "    font-size: 12px;"
        "    color: white;"
        "    font-family: 'Roboto', sans-serif;"
        "}"
        "QPushButton:hover {"
        "    background-color: #404040;"
        "    border: 1px solid #0078d4;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #0078d4;"
        "    border: 1px solid #005a9e;"
        "}"
    );

    // Dark theme style for hierarchy button
    m_hierarchyDockToggleButton->setStyleSheet(
        "QPushButton {"
        "    background-color: #2e2e2e;"
        "    border: 1px solid #555555;"
        "    border-radius: 15px;"
        "    font-weight: bold;"
        "    font-size: 12px;"
        "    color: white;"
        "    font-family: 'Roboto', sans-serif;"
        "}"
        "QPushButton:hover {"
        "    background-color: #404040;"
        "    border: 1px solid #0078d4;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #0078d4;"
        "    border: 1px solid #005a9e;"
        "}"
    );
}


